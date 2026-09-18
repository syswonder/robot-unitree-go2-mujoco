# SPDX-License-Identifier: MulanPSL-2.0
"""Request speed enforcement and conditional restoration through existing navigation tools."""
import time
import unittest

from explore_skill.controller import ExploreController, TaskHandle


class SpeedLimitTest(unittest.TestCase):
    def setUp(self):
        """Model the navigation service's session speed and existing tool responses."""
        self.controller = ExploreController(map_topic='/map', nav_navigate_endpoint='nav',
            nav_status_endpoint='nav', nav_cancel_endpoint='nav')
        self.state = dict(available=True, max_linear_speed_mps=.25, min_percentage=20.,
            effective_percentage=80., scope='session', run_id='')
        self.calls = []
        self.controller._mcp_call_sync = self.rpc

    def rpc(self, tool, args):
        """Apply actual percentages and record the limit visible at navigation start."""
        self.calls.append((tool, dict(args)))
        if tool == 'get_speed_limit':
            return dict(self.state)
        if tool == 'set_speed_limit':
            self.state['effective_percentage'] = args['percentage']
            return dict(accepted=True, effective_percentage=args['percentage'])
        if tool == 'navigate':
            self.assertLessEqual(self.state['max_linear_speed_mps'] * self.state['effective_percentage']/100, .18)
            return dict(accepted=True, run_id='owned')
        return dict(known=True, state='SUCCEEDED')

    def test_requested_speed_is_applied_before_navigation(self):
        """A 0.18 m/s request uses 72 percent of the real 0.25 m/s maximum."""
        handle = TaskHandle('test', time.time(), 5, .18)
        ok, _ = self.controller._nav_navigate_blocking(1, 2, yaw=None, timeout_s=1, cancel_evt=handle)
        self.assertTrue(ok)
        self.assertAlmostEqual(self.state['effective_percentage'], 72., places=5)
        self.assertLess([x[0] for x in self.calls].index('set_speed_limit'), [x[0] for x in self.calls].index('navigate'))
        self.controller._restore_speed_limit()
        self.assertEqual(self.state['effective_percentage'], 80.)

    def test_lower_existing_limit_is_never_raised(self):
        self.state['effective_percentage'] = 40.
        self.controller._apply_speed_limit(.18)
        self.controller._restore_speed_limit()
        self.assertEqual(self.state['effective_percentage'], 40.)
        self.assertFalse(any(tool == 'set_speed_limit' for tool, _ in self.calls))

    def test_restoration_preserves_external_change(self):
        self.controller._apply_speed_limit(.18)
        self.state['effective_percentage'] = 50.
        self.controller._restore_speed_limit()
        self.assertEqual(self.state['effective_percentage'], 50.)

    def test_restore_waits_for_confirmed_stop(self):
        """The previous cap remains withheld while an accepted navigation run is active."""
        self.controller._apply_speed_limit(.18)
        self.controller._active_nav_run = 'owned'
        self.controller._restore_speed_limit()
        self.assertAlmostEqual(self.state['effective_percentage'], 72., places=5)
        self.controller._active_nav_run = None
        self.controller._restore_speed_limit()
        self.assertEqual(self.state['effective_percentage'], 80.)

    def test_real_maximum_and_minimum_are_honored(self):
        self.state['max_linear_speed_mps'] = .5
        self.controller._apply_speed_limit(.18)
        self.assertAlmostEqual(self.state['effective_percentage'], 36., places=5)
        with self.assertRaisesRegex(RuntimeError, 'minimum enforceable'):
            self.controller._apply_speed_limit(.01)


if __name__ == '__main__':
    unittest.main()
