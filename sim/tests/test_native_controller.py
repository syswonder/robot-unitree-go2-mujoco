"""Deterministic Go2 control, sensor, live policy and real-physics checks.

Run: python3 -m unittest sim.tests.test_native_controller -v
"""
from __future__ import annotations

import asyncio
import ctypes
import json
import math
import queue
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import mujoco
import numpy as np
import websockets

from sim.native.controller import (
    COMMAND_LIMITS, DEFAULT_JOINT_POS, Go2Controller, SCRIPTED_HOME,
    build_moe_rough_observation, compute_go2_targets,
)
from sim.native.runtime import (
    NativeRuntime,
    configure_viewer_camera,
    disable_mujoco_viewer_shortcuts,
)
from sim.native.scene_builder import NativeSceneBuilder
from sim.native.sensors import NativeSensorSuite, pinhole_directions

ROOT = Path(__file__).resolve().parents[2]


def flat_model() -> mujoco.MjModel:
    """Compile the actual Go2 asset with a physical floor and front depth wall."""
    spec = mujoco.MjSpec.from_file(str(ROOT / "assets/robots/go2/go2.xml"))
    spec.option.timestep = .002
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[100, 100, .1], group=3)
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[20, 0, 3],
                           size=[.1, 100, 100], group=3)
    return spec.compile()


def advance(controller: Go2Controller, seconds: float, command=None) -> tuple[float, float]:
    """Run genuine contact physics with repeated Twist, recording minimum height/up."""
    minimum_z, minimum_up = math.inf, math.inf
    for index in range(round(seconds/controller.model.opt.timestep)):
        if command is not None and index % 50 == 0:
            controller.command(dict(zip(("type", "linearX", "linearY", "angularZ"),
                                        ("cmd_vel", *command))))
        controller.step()
        mujoco.mj_step(controller.model, controller.data)
        minimum_z = min(minimum_z, controller.data.qpos[controller.base_qpos+2])
        minimum_up = min(minimum_up, controller.data.xmat[controller.base, 8])
    return float(minimum_z), float(minimum_up)


class NativeControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = flat_model()

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        self.controller = Go2Controller(self.model, self.data, clock=lambda: self.data.time)

    def test_observation_scales_order_and_gravity(self):
        """Check every observation slot against explicit released-contract values."""
        offsets = np.arange(12, dtype=np.float32)*.125
        observation = build_moe_rough_observation(
            [4, -8, 12], [1, 0, 0, 0], [.3, -.2, .4],
            DEFAULT_JOINT_POS+offsets, np.arange(12)*20, -np.arange(12))
        expected = np.r_[[1, -2, 3, 0, 0, -1, .6, -.4, .1],
                         offsets, np.arange(12), -np.arange(12)].astype(np.float32)
        np.testing.assert_allclose(observation, expected, atol=2e-7)
        self.assertEqual(observation.dtype, np.float32)
        rotated = build_moe_rough_observation(
            [0]*3, [math.sqrt(.5), math.sqrt(.5), 0, 0], [0]*3,
            DEFAULT_JOINT_POS, [0]*12, [0]*12)
        np.testing.assert_allclose(rotated[3:6], [0, -1, 0], atol=2e-7)
        with self.assertRaises(ValueError):
            build_moe_rough_observation([math.nan, 0, 0], [1, 0, 0, 0],
                                        [0]*3, DEFAULT_JOINT_POS, [0]*12, [0]*12)

    def test_scripted_gait_home_diagonal_turn_and_continuous_commands(self):
        """Verify gait symmetry, two-axis yaw steps and fractional Twist."""
        np.testing.assert_array_equal(compute_go2_targets(np.zeros(3), 1), SCRIPTED_HOME)
        full = compute_go2_targets(np.array([1., 0, 0]), .7).reshape(4, 3)
        np.testing.assert_array_equal(full[0], full[3])
        np.testing.assert_array_equal(full[1], full[2])
        half = compute_go2_targets(np.array([.5, 0, 0]), .7).reshape(4, 3)
        self.assertGreater(np.max(abs(full-half)), .01)
        turn = compute_go2_targets(np.array([0., 0, .4]), .7).reshape(4, 3)
        self.assertGreater(abs(turn[0, 0]), .01)
        self.assertGreater(abs(turn[0, 1]), .01)
        np.testing.assert_allclose(turn[0, 0], -turn[3, 0])
        np.testing.assert_allclose(turn[1, 0], -turn[2, 0])
        self.assertFalse(np.allclose(turn[:, 1], .9))
        self.assertTrue(self.controller.command({"type": "cmd_vel", "linearX": .123,
                                                 "linearY": -.087, "angularZ": .231}))
        np.testing.assert_array_equal(self.controller.twist, [.123, -.087, .231])

    def test_invalid_twist_watchdog_and_stop(self):
        """Reject invalid inputs atomically and return to standing after stale Twist."""
        controller = self.controller
        for value in (None, "bad", float("inf"), float("nan"), []):
            self.assertFalse(controller.command({"type": "cmd_vel", "linearX": value}))
        self.assertFalse(controller.command({"type": "arm_joint_command"}))
        controller.command({"type": "cmd_vel", "linearX": 9})
        self.assertEqual(controller.twist[0], COMMAND_LIMITS[0])
        advance(controller, .6)
        np.testing.assert_array_equal(controller.targets, SCRIPTED_HOME)
        controller.command({"type": "emergency_stop"})
        controller.step()
        self.assertTrue(controller.state()["estopped"])
        np.testing.assert_array_equal(controller.twist, 0)

    def test_state_frames_and_no_arm(self):
        """A rotated base still exports free-joint world linear/local angular values."""
        self.data.qpos[3:7] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        self.data.qvel[:6] = [1, 2, 3, 4, 5, 6]
        mujoco.mj_forward(self.model, self.data)
        state = self.controller.state()
        self.assertEqual(state["base"]["linearVelocity"], [1, 2, 3])
        self.assertEqual(state["base"]["angularVelocity"], [4, 5, 6])
        self.assertNotIn("arm", state)
        self.assertEqual(len(state["joints"]["names"]), 12)
        np.testing.assert_array_equal(state["joints"]["positions"], self.data.qpos[self.controller.qpos])
        self.assertEqual(state["policy"], {"id": "moe_rough", "loaded": False, "error": None})

    def test_history_action_mapping_and_50hz(self):
        """Check zero history, prior actions, decimation and permuted motor ordering."""
        controller = self.controller
        action = (np.arange(12, dtype=np.float32)-6)*.2
        session = Mock()
        session.run.return_value = [action[None, :]]
        controller.session = session
        controller.policy_status["loaded"] = True
        controller.step()
        self.assertEqual(session.run.call_count, 1)
        np.testing.assert_array_equal(controller.history[:4], 0)
        np.testing.assert_array_equal(controller.history[-1, 33:], 0)
        np.testing.assert_allclose(controller.targets, DEFAULT_JOINT_POS+.25*action)
        expected_torque = 20*(controller.targets-self.data.qpos[controller.qpos])
        limits = self.model.actuator_ctrlrange[controller.actuators]
        np.testing.assert_allclose(self.data.ctrl[controller.actuators],
                                   np.clip(expected_torque, limits[:, 0], limits[:, 1]))
        first = controller.history[-1].copy()
        for _ in range(9):
            mujoco.mj_step(self.model, self.data)
            controller.step()
        self.assertEqual(session.run.call_count, 1)
        mujoco.mj_step(self.model, self.data)
        controller.step()
        self.assertEqual(session.run.call_count, 2)
        np.testing.assert_array_equal(controller.history[-2], first)
        np.testing.assert_array_equal(controller.history[-1, 33:], action)

    def test_invalid_action_falls_back_visibly(self):
        """Inference failure disables policy, reports why, and retains bounded torques."""
        session = Mock()
        session.run.return_value = [np.full((1, 12), np.nan)]
        self.controller.session = session
        self.controller.step()
        self.assertIsNone(self.controller.session)
        self.assertFalse(self.controller.policy_status["loaded"])
        self.assertIn("finite", self.controller.policy_status["error"])
        self.assertTrue(np.isfinite(self.data.ctrl).all())

    def test_policy_load_validation_is_atomic(self):
        """Bad IDs, failed inference and bad signatures never replace a controller."""
        controller = self.controller
        qpos, qvel = self.data.qpos.copy(), self.data.qvel.copy()
        with patch.object(controller, "_load_session", side_effect=ValueError("invalid ONNX")):
            self.assertFalse(controller.command({"type": "policy", "action": "load"}))
        self.assertIsNone(controller.session)
        self.assertEqual(controller.policy_status["error"], "invalid ONNX")
        with patch("onnxruntime.InferenceSession") as factory:
            factory.return_value.get_inputs.return_value = []
            factory.return_value.get_outputs.return_value = []
            self.assertFalse(controller.command({"type": "policy", "action": "load"}))
        self.assertIn("signature", controller.policy_status["error"])
        self.assertFalse(controller.command({"type": "policy", "action": "load", "id": "pie"}))
        np.testing.assert_array_equal(self.data.qpos, qpos)
        np.testing.assert_array_equal(self.data.qvel, qvel)

    def test_real_onnx_deterministic_live_switch(self):
        """Load the supplied ONNX twice and switch without resetting physical state."""
        controller = self.controller
        advance(controller, 1)
        qpos, qvel = self.data.qpos.copy(), self.data.qvel.copy()
        self.assertTrue(controller.command({"type": "policy", "action": "load"}), controller.policy_status)
        np.testing.assert_array_equal(self.data.qpos, qpos)
        np.testing.assert_array_equal(self.data.qvel, qvel)
        observation = build_moe_rough_observation([0]*3, [1, 0, 0, 0], [.3, 0, 0],
                                                 DEFAULT_JOINT_POS, [0]*12, [0]*12)
        history = np.zeros((5, 45), dtype=np.float32)
        history[-1] = observation
        first = controller._infer(controller.session, history)
        golden = [-.0437369905, -.4096648693, .9274917245, .0577296652,
                  .0452234782, .1489708424, -.2136805356, -.1205801219,
                  .5562964678, .4621673822, -.1025342047, .8116059303]
        np.testing.assert_allclose(first, golden, atol=2e-5)
        np.testing.assert_array_equal(first, controller._infer(controller.session, history))
        height, up = advance(controller, 3, [.3, 0, 0])
        self.assertGreater(height, .20)
        self.assertGreater(up, .8)
        np.testing.assert_array_equal(controller.kp, 20)
        np.testing.assert_array_equal(controller.kd, .5)
        qpos, qvel = self.data.qpos.copy(), self.data.qvel.copy()
        self.assertTrue(controller.command({"type": "policy", "action": "unload"}))
        np.testing.assert_array_equal(self.data.qpos, qpos)
        np.testing.assert_array_equal(self.data.qvel, qvel)
        height, up = advance(controller, 2, [0, 0, 0])
        self.assertGreater(height, .19)
        self.assertGreater(up, .8)
        self.assertTrue(controller.command({"type": "policy", "action": "load"}), controller.policy_status)
        np.testing.assert_array_equal(first, controller._infer(controller.session, history))

    def test_real_policy_tracks_navigation_speed_in_both_directions(self):
        """Measure low-speed translation and turning through the released ONNX policy."""
        commands = ([.18, 0, 0], [-.18, 0, 0], [0, 0, .3], [0, 0, -.3])
        for command in commands:
            with self.subTest(command=command):
                controller = Go2Controller(self.model, mujoco.MjData(self.model))
                controller.clock = lambda: controller.data.time
                advance(controller, 1)
                self.assertTrue(controller.command(
                    {"type": "policy", "action": "load"}), controller.policy_status)
                samples = []
                for index in range(round(8 / self.model.opt.timestep)):
                    if index % 50 == 0:
                        controller.command(dict(zip(
                            ("type", "linearX", "linearY", "angularZ"),
                            ("cmd_vel", *command))))
                    controller.step()
                    mujoco.mj_step(self.model, controller.data)
                    if index >= round(6 / self.model.opt.timestep):
                        matrix = controller.data.xmat[controller.base].reshape(3, 3)
                        local = matrix.T @ controller.data.qvel[:3]
                        samples.append([local[0], local[1], controller.data.qvel[5]])
                mean = np.mean(samples, axis=0)
                axis = 0 if command[0] else 2
                self.assertAlmostEqual(mean[axis], command[axis], delta=.09)
                self.assertGreater(controller.data.qpos[2], .23)
                self.assertGreater(controller.data.xmat[controller.base, 8], .9)
                controller.command({"type": "cmd_vel"})
                controller.step()
                np.testing.assert_array_equal(controller.policy_velocity_integral, 0)

    def test_scripted_velocity_tracking_and_reversal(self):
        """Measure settled body Twist for forward, reverse, lateral and yaw commands."""
        commands = ([.3, 0, 0], [-.3, 0, 0], [.1, 0, 0], [-.1, 0, 0],
                    [0, .1, 0], [0, -.1, 0], [0, 0, .3], [0, 0, -.3],
                    [0, 0, .12], [0, 0, -.12])
        for command in commands:
            with self.subTest(command=command):
                self.controller.reset()
                advance(self.controller, 1)
                advance(self.controller, 5, command)
                samples = []
                for index in range(1000):
                    if index % 50 == 0:
                        self.controller.command(dict(zip(
                            ("type", "linearX", "linearY", "angularZ"), ("cmd_vel", *command))))
                    self.controller.step()
                    mujoco.mj_step(self.model, self.data)
                    self.assertGreater(self.data.qpos[2], .23)
                    self.assertGreater(self.data.xmat[self.controller.base, 8], .9)
                    matrix = self.data.xmat[self.controller.base].reshape(3, 3)
                    samples.append([*(matrix.T @ self.data.qvel[:3])[:2], self.data.qvel[5]])
                np.testing.assert_allclose(np.mean(samples, axis=0), command, atol=.035)
        for command in ([.3, 0, 0], [-.3, 0, 0], [0, 0, 0]):
            height, up = advance(self.controller, 4, command)
            self.assertGreater(height, .22)
            self.assertGreater(up, .85)
        self.assertLess(np.linalg.norm(self.data.qvel[:3]), .02)

    def test_keyboard_policy_and_stop(self):
        """Native press-only keys toggle ONNX and stop all latched velocity axes."""
        runtime = NativeRuntime.__new__(NativeRuntime)
        runtime.controller = self.controller
        runtime.developer_mode = True
        runtime.keys = queue.SimpleQueue()
        runtime.keyboard_active = False
        runtime.keyboard_twist = np.zeros(3)
        for key in (87, 81, 65):
            runtime.keys.put(key)
        runtime._keyboard()
        np.testing.assert_array_equal(self.controller.twist, COMMAND_LIMITS)
        pose = self.data.qpos.copy()
        for loaded in (True, False):
            runtime.keys.put(76)
            runtime._keyboard()
            self.assertEqual(self.controller.policy_status["loaded"], loaded)
            np.testing.assert_array_equal(self.data.qpos, pose)
        runtime.keys.put(32)
        runtime._keyboard()
        self.assertFalse(runtime.keyboard_active)
        np.testing.assert_array_equal(self.controller.twist, 0)
        self.assertTrue(self.controller.estopped)

    def test_native_keyboard_is_ignored_outside_developer_mode(self):
        """Production native runs leave motion exclusively under bridge control."""
        runtime = NativeRuntime.__new__(NativeRuntime)
        runtime.controller = self.controller
        runtime.developer_mode = False
        runtime.keys = queue.SimpleQueue()
        runtime.keyboard_active = False
        runtime.keyboard_twist = np.zeros(3)
        self.controller.command({"type": "cmd_vel", "linearX": -0.2})
        runtime.keys.put(87)
        runtime.keys.put(32)
        runtime._keyboard()
        self.assertEqual(self.controller.twist[0], -0.2)
        self.assertFalse(runtime.keyboard_active)
        self.assertFalse(self.controller.estopped)

    def test_scripted_physics_six_directions(self):
        """Exercise real foot contact in each direction, with no state/force shortcuts."""
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    controller = Go2Controller(self.model, mujoco.MjData(self.model))
                    controller.clock = lambda: controller.data.time
                    advance(controller, 1)
                    start = controller.data.qpos[:3].copy()
                    command = np.zeros(3)
                    command[axis] = sign*COMMAND_LIMITS[axis]
                    height, up = advance(controller, 5, command)
                    self.assertGreater(height, .20)
                    self.assertGreater(up, .8)
                    if axis < 2:
                        self.assertGreater(sign*(controller.data.qpos[axis]-start[axis]), .05)
                    else:
                        w, x, y, z = controller.data.qpos[3:7]
                        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                        self.assertGreater(sign*yaw, .2)
                    np.testing.assert_array_equal(controller.data.xfrc_applied, 0)
                    np.testing.assert_array_equal(controller.data.qfrc_applied, 0)
                    before = controller.data.qpos.copy()
                    velocity = controller.data.qvel.copy()
                    controller.step()
                    np.testing.assert_array_equal(controller.data.qpos, before)
                    np.testing.assert_array_equal(controller.data.qvel, velocity)

    def test_scripted_full_turn_stays_upright_and_near_its_start(self):
        """A sustained in-place turn must not accumulate unsafe translation."""
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                controller = Go2Controller(self.model, mujoco.MjData(self.model))
                controller.clock = lambda: controller.data.time
                advance(controller, 1)
                start = controller.data.qpos[:2].copy()
                minimum_z, minimum_up = advance(controller, 4, [0, 0, sign*.4])
                w, x, y, z = controller.data.qpos[3:7]
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                self.assertGreater(sign*yaw, 1.2)
                later_z, later_up = advance(controller, 12, [0, 0, sign*.4])
                minimum_z = min(minimum_z, later_z)
                minimum_up = min(minimum_up, later_up)
                displacement = np.linalg.norm(controller.data.qpos[:2]-start)
                self.assertGreater(minimum_z, .23)
                self.assertGreater(minimum_up, .9)
                self.assertLess(displacement, .30)
                advance(controller, 2, [0, 0, 0])
                self.assertLess(np.linalg.norm(controller.data.qvel[:3]), .03)

    def test_scripted_motion_is_suppressed_after_large_tilt(self):
        """The fallback gait prioritizes recovery when the body is not upright."""
        controller = self.controller
        controller.data.qpos[3:7] = [math.cos(math.pi/6), math.sin(math.pi/6), 0, 0]
        mujoco.mj_forward(controller.model, controller.data)
        controller.command({"type": "cmd_vel", "linearX": .4, "angularZ": .4})
        controller.step()
        np.testing.assert_array_equal(controller.targets, SCRIPTED_HOME)
        np.testing.assert_array_equal(controller.velocity_integral, 0)

    def test_sensor_frames_optical_depth_and_missing_ids(self):
        """A front-parallel wall has constant optical depth, including corner pixels."""
        config = json.loads((ROOT / "assets/robots/go2/robot.json").read_text())["sensors"]
        config["cameras"][0].update(maxDepth=25, depthWidth=8, depthHeight=6)
        sensors = NativeSensorSuite(self.model, self.data, config)
        camera_id = next(iter(sensors.cameras))
        camera = sensors.camera_ids[camera_id]
        np.testing.assert_allclose(self.model.cam_pos[camera], [.32, 0, .12])
        np.testing.assert_allclose(self.model.site_pos[sensors.lidar_site], [0, 0, .16])
        matrix = self.data.cam_xmat[camera].reshape(3, 3)
        np.testing.assert_allclose(matrix @ [0, 0, -1], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(matrix @ [1, 0, 0], [0, -1, 0], atol=1e-12)
        np.testing.assert_allclose(matrix @ [0, -1, 0], [0, 0, -1], atol=1e-12)
        renderer = Mock()
        renderer.render.return_value = np.zeros((240, 320, 3), dtype=np.uint8)
        sensors.renderers[camera_id] = renderer
        self.data.qpos[2] = 50
        mujoco.mj_forward(self.model, self.data)
        depth = sensors.camera(camera_id)["depth"]["data"]
        np.testing.assert_allclose(depth, 19.9-.32, atol=2e-5)
        self.assertLess(-pinhole_directions(8, 6, 60)[0, 2], 1)
        sensors.close()
        config["lidar"]["site"] = "missing"
        with self.assertRaisesRegex(ValueError, "missing"):
            NativeSensorSuite(self.model, self.data, config)

    def test_spawn_rotates_keyframe_position_and_attitude(self):
        """Spawn yaw transforms keyframes exactly like the root body pose."""
        builder = NativeSceneBuilder(ROOT)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.xml"
            tree = ET.ElementTree(ET.fromstring(
                '<mujoco><worldbody><body pos="1 2 .3" quat="1 0 0 0"/></worldbody>'
                '<keyframe><key qpos="1 2 .3 1 0 0 0 .9 -1.8"/></keyframe></mujoco>'))
            tree.write(path)
            builder._apply_spawn(path, {"position": [10, 20], "yaw": math.pi/2})
            result = ET.parse(path)
            body = result.find("worldbody/body")
            qpos = np.fromstring(result.find("keyframe/key").get("qpos"), sep=" ")
            np.testing.assert_allclose(qpos[:3], [8, 21, .3])
            np.testing.assert_allclose(qpos[3:7], [math.sqrt(.5), 0, 0, math.sqrt(.5)])
            np.testing.assert_allclose(qpos[:3], np.fromstring(body.get("pos"), sep=" "))
            np.testing.assert_array_equal(qpos[7:], [.9, -1.8])

    def test_native_viewer_camera_can_pan(self):
        """Keep the native free camera movable instead of locking it to the robot."""
        camera = mujoco.MjvCamera()
        configure_viewer_camera(self.model, self.data, camera)
        scene = mujoco.MjvScene(self.model, maxgeom=1000)
        mujoco.mjv_updateScene(self.model, self.data, mujoco.MjvOption(), mujoco.MjvPerturb(),
                             camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
        previous = camera.lookat.copy()
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, .15, 0, scene, camera)
        self.assertEqual(camera.type, mujoco.mjtCamera.mjCAMERA_FREE)
        self.assertGreater(np.linalg.norm(camera.lookat-previous), .01)

    def test_native_viewer_render_shortcuts_are_disabled(self):
        """Robot keys must not toggle MuJoCo visualization or rendering flags."""
        disable_mujoco_viewer_shortcuts()
        disable_mujoco_viewer_shortcuts()
        library = ctypes.CDLL(str(Path(mujoco.__file__).with_name(
            f"libmujoco.so.{mujoco.__version__}")))
        for symbol, rows in (("mjVISSTRING", len(mujoco.mjVISSTRING)),
                             ("mjRNDSTRING", len(mujoco.mjRNDSTRING))):
            table = ((ctypes.c_char_p * 3) * rows).in_dll(library, symbol)
            self.assertTrue(all(table[index][2] == b"" for index in range(rows)))


class NativeWebsocketTests(unittest.IsolatedAsyncioTestCase):
    async def test_runtime_wire_commands_and_state(self):
        """Exercise the native bridge socket with live physics and policy switching."""
        runtime = NativeRuntime(ROOT, "go2_rl_stairs", "go2", False)
        runtime.next_camera = math.inf
        completion = asyncio.get_running_loop().create_future()

        async def peer(websocket, *unused):
            """Act as the existing bridge and check acknowledgements and Go2 state."""
            try:
                hello = json.loads(await websocket.recv())
                self.assertEqual((hello["type"], hello["backend"], hello["robot"]),
                                 ("hello", "native", "go2"))
                await websocket.send("[]")
                await websocket.send("{invalid")
                for sequence, command in enumerate((
                    {"type": "cmd_vel", "linearX": .15},
                    {"type": "policy", "action": "load", "id": "moe_rough"},
                    {"type": "policy", "action": "unload", "id": "moe_rough"},
                    {"type": "emergency_stop"},
                )):
                    await websocket.send(json.dumps({**command, "sequence": sequence}))
                    while True:
                        message = json.loads(await websocket.recv())
                        if message["type"] == "state":
                            self.assertNotIn("arm", message["robot"])
                            self.assertEqual(len(message["robot"]["joints"]["names"]), 12)
                        if message["type"] == "command_ack" and message["sequence"] == sequence:
                            self.assertTrue(message["accepted"], message)
                            if command["type"] == "policy":
                                self.assertEqual(message["policy"]["loaded"], command["action"] == "load")
                            break
                completion.set_result(True)
            except Exception as error:
                completion.set_exception(error)

        async with websockets.serve(peer, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            task = asyncio.create_task(runtime.run(f"ws://127.0.0.1:{port}"))
            try:
                await asyncio.wait_for(completion, 10)
                await asyncio.wait_for(task, 5)
            finally:
                runtime.stop()
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
        self.assertFalse(runtime.scene_path.exists())


if __name__ == "__main__":
    unittest.main()
