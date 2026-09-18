"""Offline acceptance-validator regressions; no ROS, simulator or stack required.

Run with a Python environment containing NumPy:
python -m unittest discover -s tests -p test_deployment.py
"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as Message
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("acceptance", ROOT / "sim/tests/ros_acceptance.py")
acceptance = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(acceptance)


def image(values, encoding, padding=0):
    """Build ROS-shaped image data with optional non-pixel row bytes."""
    height, width = values.shape[:2]
    rows = [row.tobytes() + b"\xff" * padding for row in values]
    return Message(encoding=encoding, width=width, height=height, step=len(rows[0]),
                   data=b"".join(rows), is_bigendian=values.dtype.byteorder == ">")


def exploration_run(state="RUNNING", canceled_state="CANCELED"):
    """Exercise bounded exploration against a virtual clock and a deterministic MCP run."""
    clock, canceled, calls = [0.0], [False], []

    def rpc(provider, contract, arguments, **kwargs):
        """Advance virtual time and return contract-shaped start/status/cancel results."""
        clock[0] += 0.5
        calls.append((contract, arguments))
        if contract == acceptance.EXPLORE:
            return {"accepted": True, "run_id": "test-run"}
        if contract.endswith("/cancel"):
            canceled[0] = True
            return {"ok": True, "message": "cancel requested"}
        return {"known": True, "state": canceled_state if canceled[0] else state}

    def pause(seconds):
        clock[0] += seconds

    node = Message(rpc=rpc, pause=pause, runs=[])
    args = Message(explore_duration=2.0, explore_timeout=180.0, explore_speed=0.18)
    with patch.object(acceptance.time, "monotonic", side_effect=lambda: clock[0]):
        report = acceptance.observe_exploration(node, args)
    return report, calls, node.runs


class AcceptanceValidationTests(unittest.TestCase):
    def test_multilevel_upper_landing_has_gallery_entrance(self):
        """Keep a Go2-width opening between the stair landing and upper gallery."""
        root = ET.parse(
            ROOT / "assets/environments/scenesmith_multilevel_house/scene.xml"
        ).getroot()
        geoms = {geom.get("name"): geom for geom in root.findall(".//geom")}
        guard = geoms["multilevel_gallery_guard"]
        landing = geoms["multilevel_stair_upper_landing"]
        guard_x = float(guard.get("pos").split()[0])
        guard_half_x = float(guard.get("size").split()[0])
        landing_x = float(landing.get("pos").split()[0])
        landing_half_x = float(landing.get("size").split()[0])
        opening = (landing_x - landing_half_x) - (guard_x + guard_half_x)
        self.assertGreaterEqual(opening, 0.45)

    def test_multilevel_gallery_open_edge_is_guarded(self):
        """Prevent a coverage or navigation turn from walking off the west edge."""
        root = ET.parse(
            ROOT / "assets/environments/scenesmith_multilevel_house/scene.xml"
        ).getroot()
        geoms = {geom.get("name"): geom for geom in root.findall(".//geom")}
        gallery = geoms["multilevel_upper_gallery"]
        guard = geoms["multilevel_gallery_west_guard"]
        gallery_pos = [float(value) for value in gallery.get("pos").split()]
        gallery_size = [float(value) for value in gallery.get("size").split()]
        guard_pos = [float(value) for value in guard.get("pos").split()]
        guard_size = [float(value) for value in guard.get("size").split()]
        self.assertAlmostEqual(guard_pos[0], gallery_pos[0] - gallery_size[0] - guard_size[0])
        self.assertAlmostEqual(guard_pos[1], gallery_pos[1])
        self.assertGreaterEqual(guard_size[1], gallery_size[1])
        self.assertGreaterEqual(guard_pos[2] - guard_size[2], 3.0)

    def test_multilevel_upper_bedroom_has_go2_passage(self):
        """Keep a traversable aisle between the upper bedroom bed and bench."""
        root = ET.parse(
            ROOT / "assets/environments/scenesmith_multilevel_house/scene.xml"
        ).getroot()
        bodies = {body.get("name"): body for body in root.findall(".//body")}
        bed = bodies["floor_2_bedroom_bed_0"]
        bench = bodies["floor_2_bedroom_bench_0"]
        bed_y = float(bed.get("pos").split()[1])
        bench_y = float(bench.get("pos").split()[1])
        bed_half_y = float(bed.find("./geom[@name='floor_2_bedroom_bed_0_static_collision']").get("size").split()[1])
        bench_half_y = float(bench.find("./geom[@name='floor_2_bedroom_bench_0_static_collision']").get("size").split()[1])
        self.assertGreaterEqual((bench_y - bench_half_y) - (bed_y + bed_half_y), 0.8)

    def test_explore_worker_requires_local_build_without_cache_fallback(self):
        """Activation and MCP calls require local explore stubs even if an old cache exists."""
        expected = ROOT / "skills/explore/rbnx-build/codegen/proto_gen/atlas_pb2.py"
        for operation in ("activate", "call"):
            with self.subTest(operation=operation), patch.object(Path, "is_file", autospec=True,
                                                                return_value=False) as exists:
                with self.assertRaisesRegex(AssertionError, "build skills/explore first"):
                    acceptance.rpc_worker({"provider": "explore", "operation": operation})
                self.assertEqual(exists.call_args_list[0].args[0], expected)
                self.assertEqual(exists.call_count, 1)

    def test_skill_activation_is_once_and_waits_for_atlas(self):
        """A lowercase active wire response still waits for Atlas ACTIVE after one driver call."""
        driver = Message(transport=Message(name="GRPC"), contract_id="robonix/skill/explore/driver")
        inactive = Message(kind=Message(name="SKILL"), state=Message(name="INACTIVE"), capabilities=[driver])
        active = Message(kind=Message(name="SKILL"), state=Message(name="ACTIVE"), capabilities=[driver])
        atlas = Message(query=Mock(side_effect=[[inactive], [inactive], [active]]))
        call = Mock(return_value={"ok": True, "state": "active", "error": ""})
        with patch.object(acceptance.time, "sleep"):
            result = acceptance.ensure_skill_active(atlas, "explore", call)
        self.assertTrue(result["activated"])
        call.assert_called_once_with(driver)
        self.assertEqual(atlas.query.call_count, 3)

    def test_active_skill_is_reused_and_bad_states_fail(self):
        """ACTIVE skips the driver; registered, failed and terminated skills cannot auto-activate."""
        for state in ("ACTIVE", "REGISTERED", "ERROR", "TERMINATED"):
            provider = Message(kind=Message(name="SKILL"), state=Message(name=state), capabilities=[])
            atlas, call = Message(query=Mock(return_value=[provider])), Mock()
            with self.subTest(state=state):
                if state == "ACTIVE":
                    self.assertFalse(acceptance.ensure_skill_active(atlas, "explore", call)["activated"])
                else:
                    with self.assertRaisesRegex(AssertionError, "requires INACTIVE"):
                        acceptance.ensure_skill_active(atlas, "explore", call)
                call.assert_not_called()

    def test_skill_activation_failure_is_not_retried(self):
        """A rejected activation fails immediately without issuing another lifecycle command."""
        driver = Message(transport=Message(name="GRPC"), contract_id="robonix/skill/explore/driver")
        provider = Message(kind=Message(name="SKILL"), state=Message(name="INACTIVE"), capabilities=[driver])
        atlas = Message(query=Mock(return_value=[provider]))
        call = Mock(return_value={"ok": False, "state": "ERROR", "error": "upstream unavailable"})
        with self.assertRaisesRegex(AssertionError, "CMD_ACTIVATE"):
            acceptance.ensure_skill_active(atlas, "explore", call)
        call.assert_called_once_with(driver)

    def test_exploration_intentional_cancel_threads_run_id(self):
        """An ongoing run is canceled after the observation interval and reaches CANCELED."""
        report, calls, pending = exploration_run()
        self.assertTrue(report["cancel_requested"])
        self.assertEqual(report["status"]["state"], "CANCELED")
        self.assertGreaterEqual(report["observed_s"], 2.0)
        self.assertEqual(report["observations"][0]["state"], "RUNNING")
        self.assertEqual(pending, [])
        self.assertEqual(sum(contract.endswith("/cancel") for contract, _ in calls), 1)
        self.assertTrue(all(args == {"run_id": "test-run"} for contract, args in calls
                            if contract != acceptance.EXPLORE))

    def test_exploration_early_success_needs_no_cancel(self):
        report, calls, _ = exploration_run("SUCCEEDED")
        self.assertFalse(report["cancel_requested"])
        self.assertFalse(any(contract.endswith("/cancel") for contract, _ in calls))

    def test_exploration_never_accepts_failure_or_timeout(self):
        """Neither budget expiry nor unsolicited cancellation counts as successful observation."""
        for state in ("FAILED", "TIMEOUT", "CANCELED"):
            with self.subTest(state=state), self.assertRaises(AssertionError):
                exploration_run(state)
        for state in ("FAILED", "TIMEOUT", "RUNNING"):
            with self.subTest(after_cancel=state), self.assertRaises(AssertionError):
                exploration_run(canceled_state=state)

    def test_retained_map_epoch_and_post_motion_refresh(self):
        """Old retained geometry passes; future/prior-epoch data and replay after motion fail."""
        grid = Message(header=Message(stamp=Message(sec=47, nanosec=0)),
                       info=Message(map_load_time=Message(sec=0, nanosec=0)))
        cloud = Message(header=Message(stamp=Message(sec=45, nanosec=0)))
        before = acceptance.map_timestamps(grid, cloud, 363, 0)
        self.assertEqual(before["cloud_stamp_s"], 45)
        for now, epoch in ((40, 0), (363, 50)):
            with self.assertRaisesRegex(AssertionError, "epoch"):
                acceptance.map_timestamps(grid, cloud, now, epoch)
        node = Message(messages={"map": grid, "map_cloud": cloud})
        self.assertFalse(acceptance.map_refreshed(node, before, 363))
        grid.header.stamp.sec = 364
        self.assertFalse(acceptance.map_refreshed(node, before, 363))
        cloud.header.stamp.sec = 364
        self.assertTrue(acceptance.map_refreshed(node, before, 363))

    def test_scene_observation_timestamp_contract(self):
        """Accept numeric Unix seconds and reject strings, ROS stamps, booleans and NaN."""
        self.assertEqual(acceptance.scene_last_seen({"last_seen_unix": 1788867000.5}), 1788867000.5)
        for value in ("1788867000.5", {"sec": 10}, True, float("nan"), 0, None):
            with self.subTest(value=value), self.assertRaises(AssertionError):
                acceptance.scene_last_seen({"last_seen_unix": value})

    def test_uniform_color_is_blank_even_with_padding(self):
        """Channel differences and row padding must not create false visual detail."""
        pixels = np.zeros((16, 16, 3), dtype=np.uint8)
        pixels[:, :, 0] = 255
        with self.assertRaisesRegex(AssertionError, "blank"):
            acceptance.image_stats(image(pixels, "rgb8", padding=32))

    def test_valid_rgb_and_big_endian_depth(self):
        """Accept genuine spatial variation and padded big-endian optical depth."""
        rgb = np.tile(np.arange(16, dtype=np.uint8)[None, :, None] * 10, (16, 1, 3))
        self.assertGreater(acceptance.image_stats(image(rgb, "rgb8", 5))["spatial_std"], 2)
        depth = np.full((16, 16), 2.5, dtype=">f4")
        stats = acceptance.image_stats(image(depth, "32FC1", 8))
        self.assertEqual(stats["min_m"], 2.5)
        self.assertEqual(stats["valid"], 256)

    def test_invalid_depth_and_truncated_image_fail(self):
        """Reject NaN/zero depth and malformed buffers rather than counting bytes."""
        for value in (0, float("nan"), float("inf"), -1):
            with self.subTest(value=value), self.assertRaises(AssertionError):
                acceptance.image_stats(image(np.full((16, 16), value, dtype="<f4"), "32FC1"))
        malformed = image(np.ones((16, 16), dtype="<f4"), "32FC1")
        malformed.data = malformed.data[:-1]
        with self.assertRaisesRegex(AssertionError, "length"):
            acceptance.image_stats(malformed)

    def test_cloud_checks_xyz_with_stride(self):
        """Reject clouds made entirely of invalid XYZ despite nonempty payloads."""
        fields = [Message(name=name, offset=index*4, datatype=7, count=1)
                  for index, name in enumerate(("x", "y", "z"))]
        xyz = np.column_stack((np.linspace(1, 5, 120), np.ones(120), np.zeros(120))).astype("<f4")
        cloud = Message(width=120, height=1, point_step=12, row_step=1448,
                        data=xyz.tobytes()+b"\x00"*8, fields=fields, is_bigendian=False)
        self.assertEqual(acceptance.cloud_stats(cloud)["valid"], 120)
        cloud.data = np.full_like(xyz, np.nan).tobytes()+b"\x00"*8
        with self.assertRaisesRegex(AssertionError, "spatial returns"):
            acceptance.cloud_stats(cloud)

    def test_quaternion_rejects_invalid_and_measures_fall(self):
        """A zero quaternion must fail, and a fallen body must report excessive tilt."""
        with self.assertRaises(AssertionError):
            acceptance.quaternion(Message(x=0, y=0, z=0, w=0))
        _, tilt = acceptance.quaternion(Message(x=1, y=0, z=0, w=0))
        self.assertGreater(tilt, 3)

    def test_failure_is_json_and_nonzero(self):
        """Invalid numeric bounds must fail before attempting ROS or Docker startup."""
        result = subprocess.run([sys.executable, str(ROOT / "sim/tests/ros_acceptance.py"),
                                 "--motion-seconds", "nan"], text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 1)
        report = json.loads(result.stdout)
        self.assertFalse(report["ok"])
        self.assertIn("invalid --motion-seconds", report["error"])


if __name__ == "__main__":
    unittest.main()
