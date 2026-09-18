import time
import unittest
from unittest.mock import Mock, patch

from explore_skill.controller import ExploreController, TaskHandle


class _Grid:
    @staticmethod
    def world_to_cell(_x, _y):
        return (4, 7)


class ExploreControlFlowTest(unittest.TestCase):
    def setUp(self) -> None:
        """Isolate navigation control-flow tests from speed RPCs and live ROS."""
        self.controller = ExploreController(
            map_topic="/map",
            nav_navigate_endpoint="http://127.0.0.1:1/mcp/",
            nav_status_endpoint="http://127.0.0.1:1/mcp/",
            nav_cancel_endpoint="http://127.0.0.1:1/mcp/",
        )
        self.controller._latest_pose_xyyaw = (1.0, 2.0, 0.0)
        self.controller._latest_map = object()
        self.controller._apply_speed_limit = Mock()
        self.controller._restore_speed_limit = Mock()

    def _handle(self, legs: int = 0) -> TaskHandle:
        return TaskHandle(
            task_id="test", started_at=time.time(), timeout_s=60.0,
            max_speed_m_s=0.1, legs_completed=legs,
        )

    @patch("explore_skill.frontier.GridView.from_msg", return_value=_Grid())
    def test_sweep_requires_periodic_leg_and_missing_coverage(self, _grid) -> None:
        """Only periodic arrivals with missing viewing coverage cause a sweep."""
        self.controller._viewed_sectors[(4, 7)] = set()
        self.assertFalse(self.controller._should_sweep(self._handle(1)))
        self.assertFalse(self.controller._should_sweep(self._handle(2)))
        self.assertTrue(self.controller._should_sweep(self._handle(3)))

        self.controller._viewed_sectors[(4, 7)] = set(range(6))
        self.assertFalse(self.controller._should_sweep(self._handle(3)))

    def test_navigation_timeout_cancels_the_accepted_run(self) -> None:
        """Leg timeout cancels its accepted navigation run and confirms completion."""
        calls = []

        def rpc(tool, args):
            """Model delayed task completion after cancellation submission."""
            calls.append((tool, args))
            if tool == "navigate":
                return {"accepted": True, "run_id": "run-42"}
            if tool == "status":
                return {"state": "CANCELED" if any(t == "cancel" for t, _ in calls) else "RUNNING"}
            return {"accepted": True}

        self.controller._mcp_call_sync = rpc
        self.controller.NAV_POLL_PERIOD_S = 0.001
        ok, detail = self.controller._nav_navigate_blocking(
            1.0, 2.0, yaw=None, timeout_s=0.003,
            cancel_evt=self._handle(),
        )

        self.assertFalse(ok)
        self.assertEqual(detail, "leg timeout")
        self.assertIn(("cancel", {"run_id": "run-42"}), calls)

    def test_navigation_cancel_request_cancels_the_accepted_run(self) -> None:
        """A cancellation arriving after acceptance targets exactly that run."""
        calls = []

        def rpc(tool, args):
            """Cancel during goal acceptance and then confirm terminal status."""
            calls.append((tool, args))
            if tool == "navigate":
                handle.cancel_requested = True
                return {"accepted": True, "run_id": "run-7"}
            if tool == "cancel":
                return {"accepted": True}
            return {"state": "CANCELED"}

        handle = self._handle()
        self.controller._mcp_call_sync = rpc
        ok, detail = self.controller._nav_navigate_blocking(
            1.0, 2.0, yaw=None, timeout_s=1.0, cancel_evt=handle,
        )

        self.assertFalse(ok)
        self.assertEqual(detail, "canceled during nav")
        self.assertIn(("cancel", {"run_id": "run-7"}), calls)

    def test_cancel_from_async_handler_does_not_call_asyncio_run(self):
        """The MCP handler only signals; it never invokes a nested client event loop."""
        import asyncio
        handle = self._handle()
        self.controller._task = handle
        with patch.object(self.controller, "_nav_cancel_rpc") as cancel_rpc:
            async def cancel():
                return self.controller.cancel(handle.task_id)
            self.assertTrue(asyncio.run(cancel())[0])
            cancel_rpc.assert_not_called()
        self.assertTrue(handle.cancel_requested)
        self.assertTrue(self.controller._task_wake.is_set())

    def test_status_rpc_failure_cancels_exact_run(self):
        """A failed status request cannot leave its accepted navigation run unaddressed."""
        calls = []
        def rpc(tool, args):
            """Fail status while accepting start and cleanup cancellation."""
            calls.append((tool, args))
            if tool == "navigate":
                return {"accepted": True, "run_id": "owned"}
            if tool == "status":
                if any(t == 'cancel' for t, _ in calls):
                    return {'known': True, 'state': 'CANCELED'}
                raise RuntimeError("status unavailable")
            return {"accepted": True}
        self.controller._mcp_call_sync = rpc
        with self.assertRaisesRegex(RuntimeError, "status unavailable"):
            self.controller._nav_navigate_blocking(1, 2, yaw=None, timeout_s=1, cancel_evt=self._handle())
        self.assertIn(("cancel", {"run_id": "owned"}), calls)

    def test_cancel_failure_is_not_reported_as_clean_timeout(self):
        """Failure to acknowledge stop remains an observable error."""
        def rpc(tool, args):
            """Accept a run and reject cancellation after its deadline."""
            if tool == "navigate":
                return {"accepted": True, "run_id": "owned"}
            if tool == "cancel":
                return {"accepted": False, "detail": "unavailable"}
            return {"state": "RUNNING"}
        self.controller._mcp_call_sync = rpc
        self.controller.NAV_CANCEL_TIMEOUT_S = .003
        self.controller.NAV_POLL_PERIOD_S = .001
        with self.assertRaisesRegex(RuntimeError, "did not reach terminal state"):
            self.controller._nav_navigate_blocking(1, 2, yaw=None, timeout_s=.001, cancel_evt=self._handle())

    def test_rejected_cancel_still_clears_finished_run(self):
        """A normal completion racing cancellation must not leave the active-run block."""
        for state in ('SUCCEEDED', 'FAILED', 'CANCELED', 'TIMEOUT'):
            with self.subTest(state=state):
                self.controller._active_nav_run = 'owned'
                def rpc(tool, args):
                    self.assertEqual(args['run_id'], 'owned')
                    return {'accepted': False} if tool == 'cancel' else {'known': True, 'state': state}
                self.controller._mcp_call_sync = rpc
                self.controller._nav_cancel_rpc('owned')
                self.assertIsNone(self.controller._active_nav_run)

    def test_cancel_transport_error_still_checks_terminal_status(self):
        """Lost cancel responses can be resolved by a subsequent exact-run status query."""
        self.controller._active_nav_run = 'owned'
        def rpc(tool, args):
            if tool == 'cancel':
                raise RuntimeError('connection reset')
            self.assertEqual(args['run_id'], 'owned')
            return {'known': True, 'state': 'SUCCEEDED'}
        self.controller._mcp_call_sync = rpc
        self.controller._nav_cancel_rpc('owned')
        self.assertIsNone(self.controller._active_nav_run)

    def test_failed_leg_recomputes_fresh_map_then_completes_another_leg(self):
        """The worker excludes a failed goal across map resizing and completes another leg."""
        import numpy as np
        from types import SimpleNamespace
        def message(data, origin=0.0):
            return SimpleNamespace(data=data.ravel().tolist(), info=SimpleNamespace(
                height=data.shape[0], width=data.shape[1], resolution=.05,
                origin=SimpleNamespace(position=SimpleNamespace(x=origin, y=0))))
        data = np.full((60, 140), 100, dtype=np.int8)
        data[5:55, 5:135] = 0
        data[5:55, :5] = data[5:55, 135:] = -1
        self.controller._latest_map = message(data)
        self.controller._latest_pose_xyyaw = (3.5, 1.5, 0)
        self.controller.config['failed_goal_radius_m'] = 2.5
        self.controller.LOOP_QUIET_PERIOD_S = 0
        handle = self._handle()
        calls = []
        def navigate(x, y, **kwargs):
            """Fail the first target, expand the map, then confirm a distinct arrival."""
            calls.append((x, y))
            if len(calls) == 1:
                self.controller._on_map(message(np.pad(data, ((0, 0), (20, 0)), constant_values=100), -1))
                return False, 'planner rejected'
            self.controller._latest_pose_xyyaw = (x, y, 0)
            return True, 'succeeded'
        def stop_after_arrival(task):
            task.cancel_requested = True
            return False
        self.controller._nav_navigate_blocking = navigate
        self.controller._should_sweep = stop_after_arrival
        self.controller._run_task(handle)
        self.assertEqual(handle.legs_completed, 1)
        self.assertEqual(len(calls), 2)
        self.assertGreater(sum((a-b)**2 for a,b in zip(*calls))**.5, 2.5)
        self.assertEqual(handle.state, 'canceled')

    def test_cancel_ack_waits_for_terminal_before_task_canceled(self):
        """Accepted cancellation followed by RUNNING never publishes CANCELED early."""
        handle = self._handle()
        self.controller._task = handle
        polls = []
        def rpc(tool, args):
            """Delay terminal confirmation after cancellation has been submitted."""
            if tool == 'navigate':
                handle.cancel_requested = True
                return {'accepted': True, 'run_id': 'owned'}
            if tool == 'cancel':
                return {'accepted': True}
            self.assertEqual(args['run_id'], 'owned')
            self.assertEqual(handle.state, 'exploring')
            polls.append(args)
            return {'known': True, 'state': 'CANCELED' if len(polls) == 3 else 'RUNNING'}
        def loop(task):
            self.controller._nav_navigate_blocking(1, 2, yaw=None, timeout_s=1, cancel_evt=task)
            self.controller._terminate(task, 'canceled', 'confirmed')
        self.controller._mcp_call_sync = rpc
        self.controller._explore_loop = loop
        self.controller.NAV_POLL_PERIOD_S = .001
        self.controller._run_task(handle)
        self.assertEqual(len(polls), 3)
        self.assertEqual(handle.state, 'canceled')
        self.assertIsNone(self.controller._active_nav_run)

    def test_unconfirmed_cancel_blocks_next_task_and_speed_restore(self):
        """A navigation run stuck in RUNNING retains its cap and blocks another task."""
        handle = self._handle()
        def rpc(tool, args):
            if tool == 'navigate':
                return {'accepted': True, 'run_id': 'owned'}
            return {'accepted': True, 'known': True, 'state': 'RUNNING'}
        def loop(task):
            self.controller._nav_navigate_blocking(1, 2, yaw=None, timeout_s=.001, cancel_evt=task)
        self.controller._mcp_call_sync = rpc
        self.controller._explore_loop = loop
        self.controller.NAV_CANCEL_TIMEOUT_S = .003
        self.controller.NAV_POLL_PERIOD_S = .001
        self.controller._run_task(handle)
        self.assertEqual(handle.state, 'error')
        self.assertEqual(self.controller._active_nav_run, 'owned')
        self.controller._restore_speed_limit.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, 'stop is unconfirmed'):
            self.controller.start(area_hint='', timeout_s=10, max_speed_m_s=.18)


if __name__ == "__main__":
    unittest.main()
