"""Control-flow regressions for the fixed two-floor transition skill."""
from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import yaml

from floor_transition_skill.controller import FloorTransitionController, Task


ROOT = Path(__file__).resolve().parents[1]


def pose(x: float, y: float, yaw: float = 0.0):
    import math
    orientation = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.3), orientation=orientation)))


class FloorTransitionControllerTest(unittest.TestCase):
    def setUp(self):
        source = yaml.safe_load((ROOT / "config/scenesmith_multilevel_house.yaml").read_text())
        self.temp = tempfile.TemporaryDirectory()
        source["state_file"] = str(Path(self.temp.name) / "state.json")
        profile = Path(self.temp.name) / "profile.yaml"
        profile.write_text(yaml.safe_dump(source))
        self.controller = FloorTransitionController(
            profile_file=str(profile), startup_floor=1, bridge_url="http://127.0.0.1:1",
            endpoints={"nav_navigate": "nav", "nav_status": "nav", "nav_cancel": "nav", "map_load": "map"},
            topics={"cmd_vel": "/cmd_vel", "odom": "/odom", "map_pose": "/pose", "imu": "/imu"},
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_connection_zone_uses_direct_alignment_without_nav2(self):
        self.controller._map_pose = pose(-2.90, -1.70)
        self.controller._align_heading = Mock()
        self.controller._navigate = Mock()
        task = Task("run", 2, 30, time.time())
        self.controller._position_at_entry(task, {"x": -2.95, "y": -1.70, "yaw": 0.0}, 10)
        self.controller._navigate.assert_not_called()
        self.controller._align_heading.assert_called_once_with(task, 0.0, 10)

    def test_alignment_is_stricter_than_entry_verification(self):
        connection = self.controller.profile["connection"]
        self.assertLess(connection["alignment_yaw_tolerance_rad"],
                        connection["entry_yaw_tolerance_rad"])

    def test_navigation_timeout_cancels_exact_accepted_run(self):
        calls = []
        def rpc(endpoint, tool, arguments):
            calls.append((tool, dict(arguments)))
            if tool == "navigate":
                return {"accepted": True, "run_id": "nav-owned"}
            return {"state": "RUNNING"}
        self.controller._mcp = rpc
        self.controller._cancel_navigation = Mock(return_value=True)
        task = Task("run", 2, 30, time.time())
        with self.assertRaisesRegex(TimeoutError, "entry navigation timed out"):
            self.controller._navigate(task, {"x": 1, "y": 2, "yaw": 0}, 0.001)
        self.controller._cancel_navigation.assert_called_once_with("nav-owned")
        self.assertIsNone(self.controller._active_nav_run)

    def test_policy_switch_requires_acknowledged_target_state(self):
        responses = [
            SimpleNamespace(__enter__=lambda value: value, __exit__=lambda *args: None),
        ]
        health = {"ok": True, "environment": "scenesmith_multilevel_house",
                  "policy": {"id": "moe_rough", "loaded": False}}
        command = {"ok": True, "policy": {"id": "moe_rough", "loaded": True}}
        with patch("floor_transition_skill.controller.urlopen") as opened:
            opened.return_value.__enter__.side_effect = [
                SimpleNamespace(read=lambda: b""), SimpleNamespace(read=lambda: b"")]
            with patch("floor_transition_skill.controller.json.load", side_effect=[health, command]):
                self.controller._set_policy(True)
        request = opened.call_args_list[1].args[0]
        self.assertIn(b'"action": "load"', request.data)


if __name__ == "__main__":
    unittest.main()
