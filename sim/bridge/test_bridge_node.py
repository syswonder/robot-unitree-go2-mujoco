"""Bridge regressions; run with unittest discovery inside the ROS Humble image."""
from __future__ import annotations

import asyncio
import base64
import functools
import http.client
import json
import math
import threading
import unittest
from contextlib import asynccontextmanager
from http.server import ThreadingHTTPServer
from unittest.mock import Mock, patch

import numpy as np
import rclpy
import websockets

from bridge_node import (
    BridgeState, CommandError, HealthHandler, MujocoRosBridge,
    handle_runtime, validate_command, world_to_body,
)


def setUpModule() -> None:
    rclpy.init()


def tearDownModule() -> None:
    rclpy.shutdown()


def state_frame() -> dict:
    """Return a yawed Go2 with world velocity, twelve leg joints, and policy state."""
    return {
        "type": "state", "simulationTime": 2.25, "environment": "test_room",
        "robot": {
            "base": {
                "position": [1.0, 2.0, 0.3],
                "quaternion": [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
                "linearVelocity": [0.0, 1.0, 0.2], "angularVelocity": [0.1, 0.2, 0.3],
            },
            "joints": {
                "names": [f"{leg}_{joint}_joint" for leg in ("FL", "FR", "RL", "RR")
                          for joint in ("hip", "thigh", "calf")],
                "positions": [0.1] * 12, "velocities": [0.2] * 12,
            },
            "policy": {"id": "moe_rough", "loaded": False, "error": None},
        },
        "imu": {"gyroscope": [0.1, 0.2, 0.3], "accelerometer": [0.0, 0.0, 9.81]},
    }


def encode(array: np.ndarray) -> str:
    return base64.b64encode(array.tobytes()).decode("ascii")


class RosBridgeTests(unittest.TestCase):
    """Exercise real ROS message construction and the resulting topic graph."""

    def setUp(self) -> None:
        self.state = BridgeState()
        self.node = MujocoRosBridge(self.state)
        self.addCleanup(self.node.destroy_node)

    def test_only_go2_topics_and_mounts(self) -> None:
        """The graph keeps sensors and navigation and has no manipulation endpoints."""
        topics = {publisher.topic_name for publisher in self.node.publishers}
        subscriptions = {subscription.topic_name for subscription in self.node.subscriptions}
        self.assertTrue({"/clock", "/odom", "/scan", "/mid360/points", "/mid360/imu",
                         "/joint_states", "/front/rgb", "/front/depth", "/front/camera_info",
                         "/front/extrinsics", "/rtabmap/cloud_map"} <= topics)
        self.assertTrue({"/cmd_vel", "/sim/reset", "/cloud_map"} <= subscriptions)
        self.assertFalse(any(word in topic for topic in topics | subscriptions
                             for word in ("arm", "wrist", "pick", "task_objects")))
        self.node.static_tf = Mock()
        self.node.front_extrinsics_pub = Mock(wraps=self.node.front_extrinsics_pub)
        self.node._publish_static_transforms()
        mounts = self.node.static_tf.sendTransform.call_args.args[0]
        self.assertEqual([m.child_frame_id for m in mounts],
                         ["mid360_link", "front_camera_optical_frame", "imu_link"])
        for mount, xyz, xyzw in zip(mounts,
                [(0, 0, 0.16), (0.32, 0, 0.12), (-0.02557, 0, 0.04232)],
                [(0, 0, 0, 1), (0.5, -0.5, 0.5, -0.5), (0, 0, 0, 1)]):
            translation, rotation = mount.transform.translation, mount.transform.rotation
            self.assertEqual(mount.header.frame_id, "base_link")
            np.testing.assert_allclose([translation.x, translation.y, translation.z], xyz)
            np.testing.assert_allclose([rotation.x, rotation.y, rotation.z, rotation.w], xyzw)
        self.node.front_extrinsics_pub.publish.assert_called_once_with(mounts[1])

    def test_state_odometry_imu_joints_and_clock(self) -> None:
        """A yawed base reports body translation and preserves already-local rotation."""
        for name in ("odom_pub", "imu_pub", "standard_joints_pub", "clock_pub"):
            setattr(self.node, name, Mock(wraps=getattr(self.node, name)))
        self.node.tf = Mock()
        self.node.receive(state_frame())
        odom = self.node.odom_pub.publish.call_args.args[0]
        linear, angular = odom.twist.twist.linear, odom.twist.twist.angular
        np.testing.assert_allclose([linear.x, linear.y, linear.z], [1, 0, 0.2], atol=1e-12)
        np.testing.assert_allclose([angular.x, angular.y, angular.z], [0.1, 0.2, 0.3])
        self.assertEqual((odom.header.frame_id, odom.child_frame_id), ("odom", "base_link"))
        joints = self.node.standard_joints_pub.publish.call_args.args[0]
        self.assertEqual(joints.name, state_frame()["robot"]["joints"]["names"])
        self.assertEqual(list(joints.position), [0.1] * 12)
        self.assertEqual(list(joints.velocity), [0.2] * 12)
        imu = self.node.imu_pub.publish.call_args.args[0]
        self.assertEqual(imu.header.frame_id, "imu_link")
        self.assertEqual(imu.orientation_covariance[0], -1)
        self.assertEqual(imu.linear_acceleration.z, 9.81)
        clock = self.node.clock_pub.publish.call_args.args[0].clock
        self.assertEqual((clock.sec, clock.nanosec), (2, 250_000_000))
        self.assertEqual(self.state.snapshot(True)["robot"], state_frame()["robot"])

    def test_top_level_joints_and_invalid_lengths(self) -> None:
        """Accept the alternate state.joints placement and reject mismatched arrays."""
        frame = state_frame()
        frame["joints"] = frame["robot"].pop("joints")
        self.node.standard_joints_pub = Mock(wraps=self.node.standard_joints_pub)
        self.node.receive(frame)
        self.assertEqual(len(self.node.standard_joints_pub.publish.call_args.args[0].name), 12)
        frame["joints"]["velocities"] = [0.0]
        with self.assertRaisesRegex(ValueError, "matching lengths"):
            self.node.receive(frame)

    def test_full_quaternion_rotation(self) -> None:
        """Roll and pitch must affect translation, not just yaw."""
        root = math.sqrt(0.5)
        np.testing.assert_allclose(world_to_body([0, 0, 1], [root, root, 0, 0]),
                                   [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(world_to_body([0, 0, -1], [root, 0, root, 0]),
                                   [1, 0, 0], atol=1e-12)
        with self.assertRaises(ValueError):
            world_to_body([1, 0, 0], [0, 0, 0, 0])

    def test_depth_noninteger_resize_and_downsample(self) -> None:
        """Every output pixel has optical-Z depth, including nonmultiple right edges."""
        publishers = self.node.camera_publishers["front_rgbd"]
        for name in publishers:
            publishers[name] = Mock(wraps=publishers[name])
        source = np.array([[1, 2, 3], [4, 5, 6]], dtype="<f4")
        for width, height, expected in (
            (5, 3, [[1, 1, 2, 3, 3], [4, 4, 5, 6, 6], [4, 4, 5, 6, 6]]),
            (2, 1, [[4, 6]]), (3, 2, source),
        ):
            frame = {"type": "camera", "id": "front_rgbd", "width": width, "height": height,
                     "rgbaU8": encode(np.full((height, width, 4), 127, dtype=np.uint8)),
                     "depth": {"width": 3, "height": 2, "dataF32": encode(source)}}
            self.node.receive(frame)
            depth = publishers["depth"].publish.call_args.args[0]
            self.assertEqual(len(depth.data), depth.height * depth.step)
            self.assertEqual(depth.encoding, "32FC1")
            self.assertEqual(depth.header.frame_id, "front_camera_optical_frame")
            np.testing.assert_array_equal(np.frombuffer(depth.data, dtype="<f4").reshape(height, width), expected)
            rgb = publishers["rgb"].publish.call_args.args[0]
            self.assertEqual(len(rgb.data), width * height * 3)
            info = publishers["info"].publish.call_args.args[0]
            self.assertEqual((info.width, info.height), (width, height))

    def test_lidar_and_map_relay(self) -> None:
        """Keep packed lidar values and relay the actual map message unchanged."""
        for name in ("scan_pub", "cloud_pub", "map_cloud_pub"):
            setattr(self.node, name, Mock(wraps=getattr(self.node, name)))
        self.node.receive({"type": "scan", "rangesF32": encode(np.array([1, 2], dtype="<f4"))})
        scan = self.node.scan_pub.publish.call_args.args[0]
        self.assertEqual(scan.header.frame_id, "mid360_link")
        self.assertEqual(list(scan.ranges), [1, 2])
        points = np.array([[1, 2, 3], [4, 5, 6]], dtype="<f4")
        self.node.receive({"type": "pointcloud", "xyzF32": encode(points)})
        cloud = self.node.cloud_pub.publish.call_args.args[0]
        self.assertEqual(cloud.header.frame_id, "mid360_link")
        self.assertEqual(len(cloud.data), cloud.row_step)
        np.testing.assert_array_equal(np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3), points)
        self.node._on_map_cloud(cloud)
        self.node.map_cloud_pub.publish.assert_called_once_with(cloud)


class ValidationTests(unittest.TestCase):
    """Reject malformed local commands before any runtime I/O."""

    def test_invalid_commands(self) -> None:
        """Nonfinite values, invalid policies, unexpected fields, and sequence types fail."""
        invalid = [None, [], {}, {"type": []}, {"type": "arm_joint_command"},
                   {"type": "policy", "action": "load", "id": "not_registered"},
                   {"type": "policy", "action": "load", "id": []},
                   {"type": "policy", "action": "toggle", "id": "moe_rough"},
                   {"type": "reset", "unexpected": True}]
        invalid += [{"type": "cmd_vel", "linearX": value}
                    for value in (float("nan"), float("inf"), -float("inf"), True, "0.1", None, 0.601)]
        invalid += [{"type": "reset", "sequence": value} for value in (-1, True, 1.0, "1", 2**53)]
        invalid += [{"type": "cmd_vel", "linearY": 0.351}, {"type": "cmd_vel", "angularZ": -0.601}]
        for command in invalid:
            with self.subTest(command=command), self.assertRaises(CommandError) as error:
                validate_command(command)
            self.assertEqual(error.exception.status, 400)

    def test_valid_commands(self) -> None:
        """All four authorized commands preserve their explicit fields."""
        for command in (
            {"type": "reset"}, {"type": "emergency_stop", "sequence": 42},
            {"type": "cmd_vel", "linearX": 0.6, "linearY": -0.35, "angularZ": 0.6},
            {"type": "policy", "action": "load", "id": "moe_rough"},
            {"type": "policy", "action": "unload", "id": "moe_rough"},
        ):
            self.assertEqual(validate_command(command), command)

    def test_state_freshness_boundary_ignores_sensor_traffic(self) -> None:
        """Only connected robot state younger than three monotonic seconds protects ownership."""
        state = BridgeState()
        state.connect()
        self.assertFalse(state.has_fresh_state())
        with patch("bridge_node.time.monotonic", return_value=100.0):
            state.update(state_frame())
        with patch("bridge_node.time.monotonic", return_value=102.999):
            self.assertTrue(state.has_fresh_state())
        state.record("scan")
        with patch("bridge_node.time.monotonic", return_value=103.0):
            self.assertFalse(state.has_fresh_state())
        state.connect()
        self.assertFalse(state.has_fresh_state())
        state.update(state_frame())
        state.connected = False
        self.assertFalse(state.has_fresh_state())


class HttpWebsocketTests(unittest.IsolatedAsyncioTestCase):
    """Use real loopback HTTP and WebSocket transports with runtime protocol fixtures."""

    async def asyncSetUp(self) -> None:
        """Start isolated HTTP and WebSocket listeners on ephemeral loopback ports."""
        self.state = BridgeState()
        self.node = MujocoRosBridge(self.state)
        self.node.command_timeout = 0.2
        self.node.loop = asyncio.get_running_loop()
        handler = type("TestHealthHandler", (HealthHandler,), {"state": self.state, "node": self.node})
        self.http = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()
        self.ws_server = await websockets.serve(functools.partial(handle_runtime, self.node), "127.0.0.1", 0)
        self.ws_url = f"ws://127.0.0.1:{self.ws_server.sockets[0].getsockname()[1]}"

    async def asyncTearDown(self) -> None:
        """Close all peers and HTTP threads before destroying ROS resources."""
        self.ws_server.close()
        await self.ws_server.wait_closed()
        await asyncio.to_thread(self.http.shutdown)
        self.http.server_close()
        self.http_thread.join(timeout=2)
        self.node.destroy_node()

    def request(self, method: str, path: str, body=None, origin="http://127.0.0.1:5181") -> tuple:
        """Perform one JSON HTTP exchange from a worker thread."""
        connection = http.client.HTTPConnection("127.0.0.1", self.http.server_port, timeout=3)
        headers = {"Content-Type": "application/json"}
        if origin is not None:
            headers["Origin"] = origin
        try:
            connection.request(method, path, None if body is None else json.dumps(body), headers)
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), json.loads(response.read())
        finally:
            connection.close()

    async def post(self, command: dict) -> tuple:
        return await asyncio.to_thread(self.request, "POST", "/command", command)

    async def ack(self, runtime, command: dict, accepted=True, **extra) -> None:
        await runtime.send(json.dumps({"type": "command_ack", "sequence": command["sequence"],
                                       "accepted": accepted, **extra}))

    @asynccontextmanager
    async def runtime(self, environment="test_room", backend="web"):
        """Connect a runtime and wait until its hello has assigned command ownership."""
        previous = self.node.websocket
        async with websockets.connect(self.ws_url) as runtime:
            await runtime.send(json.dumps({"type": "hello", "backend": backend, "environment": environment}))
            for _ in range(100):
                if self.node.websocket is not None and self.node.websocket is not previous:
                    break
                await asyncio.sleep(0.01)
            self.assertIsNotNone(self.node.websocket)
            self.assertIsNot(self.node.websocket, previous)
            yield runtime

    async def publish_state(self, runtime) -> None:
        """Wait for the real socket receiver to process a new valid state frame."""
        previous = self.state.snapshot()["frames"].get("state", 0)
        await runtime.send(json.dumps(state_frame()))
        for _ in range(100):
            if self.state.snapshot()["frames"].get("state", 0) > previous:
                return
            await asyncio.sleep(0.01)
        self.fail("runtime state was not processed")

    async def test_state_health_and_cors(self) -> None:
        """Expose runtime state and policy, and deny foreign hosts, ports, and null origins."""
        async with websockets.connect(self.ws_url) as runtime:
            await runtime.send(json.dumps({"type": "hello", "backend": "native", "environment": "test_room"}))
            await runtime.send(json.dumps(state_frame()))
            for _ in range(50):
                status, headers, state = await asyncio.to_thread(self.request, "GET", "/state")
                if "robot" in state:
                    break
                await asyncio.sleep(0.01)
            self.assertEqual(status, 200)
            self.assertEqual(headers["Access-Control-Allow-Origin"], "http://127.0.0.1:5181")
            self.assertEqual(state["robot"], state_frame()["robot"])
            self.assertEqual(state["backend"], "native")
            self.assertEqual(state["environment"], "test_room")
            self.assertEqual(state["simulationTime"], 2.25)
            self.assertTrue(state["runtimeConnected"])
            health = (await asyncio.to_thread(self.request, "GET", "/health"))[2]
            self.assertEqual(health["policy"], state_frame()["robot"]["policy"])
            self.assertNotIn("robot", health)
            status, headers, _ = await asyncio.to_thread(self.request, "OPTIONS", "/command")
            self.assertEqual(status, 200)
            self.assertIn("POST", headers["Access-Control-Allow-Methods"])
            for origin in ("http://evil.test:5181", "null", "http://127.0.0.1:9999",
                           "http://localhost:5181", "http://127.0.0.1.evil.test:5181",
                           "http://127.0.0.1:5181/path", "http://user@127.0.0.1:5181"):
                for method, path in (("GET", "/state"), ("OPTIONS", "/command"), ("POST", "/command")):
                    status, headers, _ = await asyncio.to_thread(self.request, method, path, {"type": "reset"}, origin)
                    self.assertEqual(status, 403, origin)
                    self.assertNotIn("Access-Control-Allow-Origin", headers)
            self.assertEqual((await asyncio.to_thread(self.request, "GET", "/missing"))[0], 404)

    async def test_disconnected_and_validation_errors(self) -> None:
        """UI errors are JSON and remain visible in health diagnostics."""
        status, headers, body = await self.post({"type": "reset"})
        self.assertEqual(status, 503)
        self.assertFalse(body["ok"])
        self.assertIn("disconnected", body["error"])
        self.assertIn("Access-Control-Allow-Origin", headers)
        health = (await asyncio.to_thread(self.request, "GET", "/health"))[2]
        self.assertIn("disconnected", health["lastCommandError"])
        self.assertIsNone(health["policy"])
        for command in ({"type": "cmd_vel", "linearX": float("nan")},
                        {"type": "policy", "id": "unregistered", "action": "load"}, []):
            self.assertEqual((await self.post(command))[0], 400)

    async def test_waits_for_real_ack_and_forwards_sequence(self) -> None:
        """Wrong or malformed acknowledgements cannot produce HTTP success."""
        async with self.runtime() as runtime:
            request = asyncio.create_task(self.post({"type": "cmd_vel", "linearX": 0.2, "sequence": 42}))
            command = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual(command["sequence"], 42)
            self.assertEqual(command["linearX"], 0.2)
            await self.ack(runtime, {"sequence": 43})
            await self.ack(runtime, {"sequence": "42"})
            await self.ack(runtime, command, accepted="true")
            await asyncio.sleep(0.02)
            self.assertFalse(request.done())
            await self.ack(runtime, command)
            status, _, body = await request
            self.assertEqual(status, 200)
            self.assertEqual(body, {"type": "command_ack", "sequence": 42, "accepted": True, "ok": True})
            self.assertEqual((await self.post({"type": "reset", "sequence": 42}))[0], 409)

    async def test_concurrent_ack_order_and_duplicates(self) -> None:
        """Independent commands correlate out-of-order acks; duplicate sequences fail."""
        async with self.runtime() as runtime:
            first = asyncio.create_task(self.post({"type": "reset", "sequence": 10}))
            command1 = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual((await self.post({"type": "reset", "sequence": 10}))[0], 409)
            second = asyncio.create_task(self.post({"type": "emergency_stop"}))
            command2 = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertGreater(command2["sequence"], command1["sequence"])
            await self.ack(runtime, command2)
            self.assertEqual((await second)[2]["sequence"], command2["sequence"])
            self.assertFalse(first.done())
            await self.ack(runtime, command1, accepted=False, error="reset failed")
            status, _, body = await first
            self.assertEqual(status, 422)
            self.assertEqual(body["error"], "reset failed")

    async def test_timeout_is_error_and_late_ack_cannot_complete_next_command(self) -> None:
        """Missing acks time out, and a late ack never belongs to a later request."""
        async with self.runtime() as runtime:
            first = asyncio.create_task(self.post({"type": "reset"}))
            command1 = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual((await first)[0], 504)
            second = asyncio.create_task(self.post({"type": "reset"}))
            command2 = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            await self.ack(runtime, command1)
            await asyncio.sleep(0.02)
            self.assertFalse(second.done())
            await self.ack(runtime, command2)
            self.assertEqual((await second)[0], 200)

    async def test_policy_failures_concurrency_and_late_ack(self) -> None:
        """Policy errors reach the UI; an unknown load outcome blocks overlapping policy work."""
        async with self.runtime() as runtime:
            load = {"type": "policy", "action": "load", "id": "moe_rough"}
            first = asyncio.create_task(self.post(load))
            command = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual((await self.post(load))[0], 409)
            policy = {"id": "moe_rough", "loaded": False, "error": "ONNX load failed"}
            await self.ack(runtime, command, accepted=False, policy=policy)
            status, _, body = await first
            self.assertEqual(status, 422)
            self.assertEqual(body["error"], "ONNX load failed")
            health = (await asyncio.to_thread(self.request, "GET", "/health"))[2]
            self.assertEqual(health["policy"], policy)
            timed_out = asyncio.create_task(self.post(load))
            pending = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual((await timed_out)[0], 504)
            self.assertEqual((await self.post(load))[0], 409)
            stop = asyncio.create_task(self.post({"type": "emergency_stop"}))
            await self.ack(runtime, json.loads(await asyncio.wait_for(runtime.recv(), 1)))
            self.assertEqual((await stop)[0], 200)
            await self.ack(runtime, pending)
            unload = asyncio.create_task(self.post({**load, "action": "unload"}))
            command = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual(command["action"], "unload")
            await self.ack(runtime, command)
            self.assertEqual((await unload)[0], 200)

    async def test_replacement_fails_old_command_and_clears_state(self) -> None:
        """A stale replacement fails old commands without reusing sequences or old sensor state."""
        async with self.runtime() as old:
            await self.publish_state(old)
            request = asyncio.create_task(self.post({"type": "reset"}))
            command = json.loads(await asyncio.wait_for(old.recv(), 1))
            old_owner = self.node.websocket
            with self.state.lock:
                self.state.last_state_monotonic -= 4.0
            self.state.record("scan")
            async with self.runtime(environment="new_room") as new:
                self.assertEqual((await request)[0], 503)
                await asyncio.sleep(0.02)
                state = (await asyncio.to_thread(self.request, "GET", "/state"))[2]
                self.assertEqual(state["environment"], "new_room")
                self.assertNotIn("robot", state)
                self.assertIsNone(state["policy"])
                current = asyncio.create_task(self.post({"type": "emergency_stop"}))
                next_command = json.loads(await asyncio.wait_for(new.recv(), 1))
                self.assertGreater(next_command["sequence"], command["sequence"])
                self.node._acknowledge({"type": "command_ack", "sequence": next_command["sequence"],
                                        "accepted": True}, old_owner)
                self.node._disconnect_commands(old_owner)
                await self.ack(new, command)
                await asyncio.sleep(0.02)
                self.assertFalse(current.done())
                await self.ack(new, next_command)
                self.assertEqual((await current)[0], 200)
                self.assertTrue(self.state.snapshot()["runtimeConnected"])

    async def test_disconnect_and_malformed_runtime_frame(self) -> None:
        """Malformed JSON frames are reported without dropping a valid runtime session."""
        async with self.runtime() as runtime:
            for raw in ("[1,2]", "not json", '{"type":"state","simulationTime":NaN}'):
                await runtime.send(raw)
            await runtime.send(json.dumps(state_frame()))
            request = asyncio.create_task(self.post({"type": "reset"}))
            await asyncio.wait_for(runtime.recv(), 1)
            await runtime.close()
            self.assertEqual((await request)[0], 503)
            health = (await asyncio.to_thread(self.request, "GET", "/health"))[2]
            self.assertFalse(health["runtimeConnected"])
            self.assertFalse(health["ok"])

    async def test_ros_commands_receive_ack_and_report_errors(self) -> None:
        """ROS callbacks use the same sequence allocator and expose failures in health."""
        from geometry_msgs.msg import Twist
        async with self.runtime() as runtime:
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.angular.z = 0.2, -0.1, 0.3
            self.node._on_twist(twist)
            command = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual({key: command[key] for key in ("linearX", "linearY", "angularZ")},
                             {"linearX": 0.2, "linearY": -0.1, "angularZ": 0.3})
            await self.ack(runtime, command, accepted=False, error="motion rejected")
            for _ in range(50):
                if self.state.snapshot()["lastCommandError"]:
                    break
                await asyncio.sleep(0.01)
            self.assertEqual(self.state.snapshot()["lastCommandError"], "motion rejected")
            self.node._on_reset(None)
            reset = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual(reset["type"], "reset")
            self.assertGreater(reset["sequence"], command["sequence"])
            await self.ack(runtime, reset)

    async def test_healthy_runtime_rejects_second_socket_without_disturbing_commands(self) -> None:
        """A rejected peer cannot reset state, clock, policy ownership, or caller sequences."""
        self.node.command_timeout = 1.0
        self.node.clock_pub = Mock(wraps=self.node.clock_pub)
        async with self.runtime(backend="native") as old:
            await self.publish_state(old)
            owner = self.node.websocket
            before = self.state.snapshot(True)
            request = asyncio.create_task(self.post({
                "type": "policy", "id": "moe_rough", "action": "load", "sequence": 42,
            }))
            command = json.loads(await asyncio.wait_for(old.recv(), 1))
            async with websockets.connect(self.ws_url) as second:
                await asyncio.wait_for(second.wait_closed(), 1)
                self.assertEqual(second.close_code, 4001)
            self.assertIs(self.node.websocket, owner)
            self.assertEqual(self.node.sequence, 42)
            self.assertEqual(self.node.policy_sequence, 42)
            self.assertIs(self.node.pending[42][0], owner)
            self.assertFalse(request.done())
            after = self.state.snapshot(True)
            for key in ("robot", "backend", "environment", "policy", "frames", "runtimeConnected"):
                self.assertEqual(after[key], before[key])
            self.assertEqual(self.node.last_sim_time, 2.25)
            self.node.clock_pub.publish.assert_called_once()
            await self.ack(old, command)
            self.assertEqual((await request)[0], 200)
            next_request = asyncio.create_task(self.post({"type": "reset"}))
            next_command = json.loads(await asyncio.wait_for(old.recv(), 1))
            self.assertEqual(next_command["sequence"], 43)
            await self.ack(old, next_command)
            self.assertEqual((await next_request)[0], 200)

    async def test_hello_required_before_first_connection_and_commands(self) -> None:
        """A socket alone never marks health connected or receives a forwarded command."""
        async with websockets.connect(self.ws_url) as runtime:
            self.assertIsNone(self.node.websocket)
            self.assertFalse(self.state.snapshot()["runtimeConnected"])
            self.assertEqual((await self.post({"type": "reset"}))[0], 503)
            self.assertEqual(self.node.sequence, -1)
            await runtime.send(json.dumps({"type": "hello", "backend": "native", "environment": "first"}))
            for _ in range(100):
                if self.node.websocket is not None:
                    break
                await asyncio.sleep(0.01)
            self.assertIsNotNone(self.node.websocket)
            self.assertTrue(self.state.snapshot()["runtimeConnected"])
            self.assertEqual(self.state.snapshot()["environment"], "first")
            self.assertFalse(self.state.has_fresh_state())
            request = asyncio.create_task(self.post({"type": "reset"}))
            command = json.loads(await asyncio.wait_for(runtime.recv(), 1))
            self.assertEqual(command["sequence"], 0)
            await self.ack(runtime, command)
            self.assertEqual((await request)[0], 200)

    async def test_invalid_first_frames_cannot_take_over_stale_owner(self) -> None:
        """State, acknowledgements, and malformed JSON cannot bypass the hello requirement."""
        async with self.runtime(backend="native") as old:
            owner = self.node.websocket
            for raw in ("not json", "[]", json.dumps(state_frame()),
                        '{"type":"command_ack","sequence":0,"accepted":true}'):
                async with websockets.connect(self.ws_url) as candidate:
                    await candidate.send(raw)
                    await asyncio.wait_for(candidate.wait_closed(), 1)
                    self.assertEqual(candidate.close_code, 1008)
                self.assertIs(self.node.websocket, owner)
                self.assertEqual(self.state.snapshot()["backend"], "native")
                self.assertEqual(self.node.sequence, -1)
                self.assertEqual(self.state.snapshot()["frames"], {})
            request = asyncio.create_task(self.post({"type": "reset"}))
            command = json.loads(await asyncio.wait_for(old.recv(), 1))
            await self.ack(old, command)
            self.assertEqual((await request)[0], 200)

    async def test_hello_timeout_preserves_existing_owner(self) -> None:
        """An idle candidate times out without closing or claiming the existing stale socket."""
        async with self.runtime() as old:
            owner = self.node.websocket
            with patch("bridge_node.RUNTIME_HELLO_TIMEOUT", 0.05):
                async with websockets.connect(self.ws_url) as candidate:
                    await asyncio.wait_for(candidate.wait_closed(), 1)
                    self.assertEqual(candidate.close_code, 1008)
            self.assertIs(self.node.websocket, owner)
            self.assertTrue(self.state.snapshot()["runtimeConnected"])
            await self.publish_state(old)
            self.assertTrue(self.state.has_fresh_state())

    async def test_owner_recovers_while_candidate_waits_for_hello(self) -> None:
        """Recheck freshness after hello so a recovered owner cannot be evicted."""
        async with self.runtime(backend="native") as old:
            owner = self.node.websocket
            async with websockets.connect(self.ws_url) as candidate:
                self.assertIs(self.node.websocket, owner)
                await self.publish_state(old)
                await candidate.send(json.dumps({"type": "hello", "backend": "web"}))
                await asyncio.wait_for(candidate.wait_closed(), 1)
                self.assertEqual(candidate.close_code, 4001)
            self.assertIs(self.node.websocket, owner)
            self.assertEqual(self.state.snapshot()["backend"], "native")
            self.assertTrue(self.state.has_fresh_state())


class DelayedSocket:
    """Controllable socket fixture for frames arriving while old close is suspended."""

    def __init__(self) -> None:
        """Keep socket reads and close completion under explicit test control."""
        self.incoming = asyncio.Queue()
        self.close_started = asyncio.Event()
        self.finish_close = asyncio.Event()
        self.iterating = asyncio.Event()

    async def close(self, _code: int, _reason: str) -> None:
        self.close_started.set()
        await self.finish_close.wait()

    async def recv(self):
        return await self.__anext__()

    def __aiter__(self):
        self.iterating.set()
        return self

    async def __anext__(self):
        value = await self.incoming.get()
        if value is None:
            raise StopAsyncIteration
        return json.dumps(value)


class OwnershipTests(unittest.IsolatedAsyncioTestCase):
    """Control the specific replacement interleavings that real close timing can conceal."""

    async def test_stale_frames_and_three_overlapping_connections(self) -> None:
        """Old frames and finalizers cannot overwrite a third connection while closes wait."""
        state = BridgeState()
        node = MujocoRosBridge(state)
        self.addCleanup(node.destroy_node)
        first, second, third = DelayedSocket(), DelayedSocket(), DelayedSocket()
        tasks = []
        try:
            await first.incoming.put({"type": "hello"})
            tasks.append(asyncio.create_task(handle_runtime(node, first)))
            await asyncio.wait_for(first.iterating.wait(), 1)
            await second.incoming.put({"type": "hello"})
            tasks.append(asyncio.create_task(handle_runtime(node, second)))
            await asyncio.wait_for(first.close_started.wait(), 1)
            self.assertIs(node.websocket, second)
            await third.incoming.put({"type": "hello"})
            tasks.append(asyncio.create_task(handle_runtime(node, third)))
            await asyncio.wait_for(second.close_started.wait(), 1)
            self.assertIs(node.websocket, third)
            await first.incoming.put({"type": "hello", "environment": "stale-first"})
            await asyncio.wait_for(tasks[0], 1)
            first.finish_close.set()
            await asyncio.wait_for(second.iterating.wait(), 1)
            await second.incoming.put({"type": "state", "environment": "stale-second"})
            await asyncio.wait_for(tasks[1], 1)
            self.assertIs(node.websocket, third)
            self.assertTrue(state.snapshot()["runtimeConnected"])
            self.assertEqual(state.snapshot()["environment"], "")
            self.assertEqual(state.snapshot()["frames"], {})
            second.finish_close.set()
            await asyncio.wait_for(third.iterating.wait(), 1)
            await third.incoming.put({"type": "hello", "backend": "native", "environment": "current"})
            await third.incoming.put(None)
            await asyncio.wait_for(tasks[2], 1)
            self.assertEqual(state.snapshot()["environment"], "current")
            self.assertFalse(state.snapshot()["runtimeConnected"])
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)


if __name__ == "__main__":
    unittest.main()
