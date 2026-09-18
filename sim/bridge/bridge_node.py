#!/usr/bin/env python3
"""Bridge a selectable MuJoCo runtime to the ROS 2 graph used by Robonix."""
from __future__ import annotations

import asyncio
import base64
import concurrent.futures
import copy
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlsplit

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import websockets


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
VELOCITY_LIMITS = {"linearX": 0.6, "linearY": 0.35, "angularZ": 0.6}
POLICY_IDS = frozenset({"moe_rough"})
LOOPBACK_HOSTS = frozenset({"localhost", "127.0.0.1", "::1"})
RUNTIME_HELLO_TIMEOUT = 3.0
RUNTIME_STATE_TIMEOUT = 3.0


def reject_json_constant(value: str) -> None:
    """Reject nonstandard nonfinite JSON numbers before caching runtime state."""
    raise ValueError(f"nonfinite JSON number: {value}")


class CommandError(Exception):
    """Expose a command failure as an HTTP status and JSON error."""

    def __init__(self, status: int, message: str, sequence: int | None = None) -> None:
        super().__init__(message)
        self.status = status
        self.sequence = sequence


def validate_command(payload: Any) -> dict[str, Any]:
    """Allow only local simulator commands with finite, bounded numeric values."""
    if not isinstance(payload, dict):
        raise CommandError(400, "command must be a JSON object")
    kind = payload.get("type")
    fields = {"type", "sequence"}
    command = {"type": kind}
    if kind == "cmd_vel":
        fields.update(VELOCITY_LIMITS)
        for name, limit in VELOCITY_LIMITS.items():
            value = payload.get(name, 0.0)
            if type(value) not in (int, float) or not -limit <= value <= limit:
                raise CommandError(400, f"{name} must be finite and between {-limit} and {limit}")
            command[name] = float(value)
    elif kind == "policy":
        fields.update(("id", "action"))
        if payload.get("id") not in tuple(POLICY_IDS):
            raise CommandError(400, "unregistered policy id; expected moe_rough")
        if payload.get("action") not in ("load", "unload"):
            raise CommandError(400, "policy action must be load or unload")
        command.update(id=payload["id"], action=payload["action"])
    elif kind not in ("emergency_stop", "reset"):
        raise CommandError(400, "unsupported command type")
    if payload.keys() - fields:
        raise CommandError(400, "unexpected command fields")
    if "sequence" in payload:
        sequence = payload["sequence"]
        if type(sequence) is not int or not 0 <= sequence <= 2**53 - 1:
            raise CommandError(400, "sequence must be a nonnegative safe integer")
        command["sequence"] = sequence
    return command


def world_to_body(linear: list[float], quaternion: list[float]) -> np.ndarray:
    """Rotate world freejoint translation velocity by the inverse base orientation."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(norm) or norm == 0:
        raise ValueError("base quaternion must be finite and nonzero")
    w, x, y, z = q / norm
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return rotation.T @ np.asarray(linear, dtype=float)


def decode_array(encoded: str | None, dtype: Any) -> np.ndarray:
    """Decode a base64 payload without making an unnecessary second copy."""
    if not encoded:
        return np.empty(0, dtype=dtype)
    return np.frombuffer(base64.b64decode(encoded), dtype=dtype)


def stamp(seconds: float) -> RosTime:
    """Convert a floating-point simulation timestamp to a ROS timestamp."""
    value = max(0.0, float(seconds))
    sec = int(value)
    return RosTime(sec=sec, nanosec=int((value - sec) * 1_000_000_000))


def set_quaternion(target: Any, quaternion_wxyz: list[float]) -> None:
    """Copy a MuJoCo wxyz quaternion into a ROS xyzw message field."""
    if len(quaternion_wxyz) != 4:
        quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]
    target.w, target.x, target.y, target.z = map(float, quaternion_wxyz)


def camera_info(width: int, height: int, fovy_deg: float, frame_id: str, at: RosTime) -> CameraInfo:
    """Build a pinhole CameraInfo matching the browser projection matrix."""
    fy = height / (2.0 * math.tan(math.radians(fovy_deg) * 0.5))
    fx = fy
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    msg = CameraInfo()
    msg.header.stamp = at
    msg.header.frame_id = frame_id
    msg.width, msg.height = width, height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


class BridgeState:
    """Thread-safe latest runtime state and diagnostics for the local HTTP UI."""

    def __init__(self) -> None:
        """Start disconnected with no state or policy observation."""
        self.lock = threading.Lock()
        self.connected = False
        self.backend = ""
        self.environment = ""
        self.last_frame_wall_time = 0.0
        self.last_state_monotonic: float | None = None
        self.frames: dict[str, int] = {}
        self.latest_state: dict[str, Any] = {}
        self.policy: dict[str, Any] | None = None
        self.last_command_error: str | None = None

    def connect(self) -> None:
        """Clear all stale diagnostics when a new runtime takes ownership."""
        with self.lock:
            self.connected = True
            self.backend = ""
            self.environment = ""
            self.last_frame_wall_time = 0.0
            self.last_state_monotonic = None
            self.frames.clear()
            self.latest_state = {}
            self.policy = None
            self.last_command_error = None

    def hello(self, payload: dict[str, Any]) -> None:
        """Record only runtime metadata, never process environment variables."""
        with self.lock:
            self.environment = str(payload.get("environment", ""))
            self.backend = str(payload.get("backend") or "web")
            if isinstance(payload.get("policy"), dict):
                self.policy = copy.deepcopy(payload["policy"])

    def update(self, payload: dict[str, Any]) -> None:
        """Keep the runtime state envelope and policy exactly as reported."""
        with self.lock:
            self.latest_state = copy.deepcopy(payload)
            self.last_state_monotonic = time.monotonic()
            self.environment = str(payload.get("environment", self.environment))
            robot = payload.get("robot") or {}
            policy = payload.get("policy", robot.get("policy"))
            if isinstance(policy, dict):
                self.policy = copy.deepcopy(policy)

    def has_fresh_state(self) -> bool:
        """Protect a connected runtime only while its latest robot state is under three seconds old."""
        with self.lock:
            return (
                self.connected and self.last_state_monotonic is not None
                and time.monotonic() - self.last_state_monotonic < RUNTIME_STATE_TIMEOUT
            )

    def command_error(self, message: str | None) -> None:
        """Make command failures visible on subsequent health and state requests."""
        with self.lock:
            self.last_command_error = message

    def record(self, kind: str) -> None:
        """Record one received simulator frame."""
        with self.lock:
            self.last_frame_wall_time = time.time()
            self.frames[kind] = self.frames.get(kind, 0) + 1

    def snapshot(self, include_state: bool = False) -> dict[str, Any]:
        """Return health, optionally merged over the latest runtime state envelope."""
        with self.lock:
            age = time.time() - self.last_frame_wall_time if self.last_frame_wall_time else None
            return {
                **(copy.deepcopy(self.latest_state) if include_state else {}),
                "ok": self.connected and age is not None and age < 3.0,
                "runtimeConnected": self.connected,
                "browserConnected": self.connected,
                "backend": self.backend,
                "environment": self.environment,
                "lastFrameAgeSec": age,
                "frames": dict(self.frames),
                "policy": copy.deepcopy(self.policy),
                "lastCommandError": self.last_command_error,
            }


class HealthHandler(BaseHTTPRequestHandler):
    """Serve local UI state and acknowledged commands with restricted CORS."""

    state: BridgeState
    node: MujocoRosBridge

    def _origin_allowed(self) -> bool:
        """Allow browser origins only on known local ports and the request hostname."""
        origin = self.headers.get("Origin")
        if origin is None:
            return True
        try:
            source = urlsplit(origin)
            host = urlsplit("//" + self.headers.get("Host", ""))
            return (
                source.scheme in ("http", "https")
                and source.hostname in LOOPBACK_HOSTS
                and source.hostname == host.hostname
                and source.port in (5181, 8766, self.server.server_port)
                and source.username is None and source.password is None
                and not source.path and not source.query and not source.fragment
            )
        except ValueError:
            return False

    def _respond(self, status: int, body: dict[str, Any]) -> None:
        """Write JSON errors and successes with CORS only for an allowed origin."""
        payload = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Vary", "Origin")
        if self.headers.get("Origin") and self._origin_allowed():
            self.send_header("Access-Control-Allow-Origin", self.headers["Origin"])
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802
        """Expose the latest robot state and runtime health to the local UI."""
        if not self._origin_allowed():
            self._respond(403, {"ok": False, "error": "origin is not allowed"})
        elif self.path in ("/", "/health", "/state"):
            self._respond(200, self.state.snapshot(include_state=self.path == "/state"))
        else:
            self._respond(404, {"ok": False, "error": "unknown endpoint"})

    def do_OPTIONS(self) -> None:  # noqa: N802
        """Answer local UI preflight requests without forwarding a runtime command."""
        if not self._origin_allowed():
            self._respond(403, {"ok": False, "error": "origin is not allowed"})
        elif self.path not in ("/", "/health", "/state", "/command"):
            self._respond(404, {"ok": False, "error": "unknown endpoint"})
        else:
            self._respond(200, {"ok": True})

    def do_POST(self) -> None:  # noqa: N802
        """Validate bounded commands and wait for the matching runtime acknowledgement."""
        try:
            if not self._origin_allowed():
                raise CommandError(403, "origin is not allowed")
            if self.path != "/command":
                raise CommandError(404, "unknown endpoint")
            if self.headers.get_content_type() != "application/json":
                raise CommandError(415, "Content-Type must be application/json")
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096 or self.headers.get("Transfer-Encoding"):
                raise CommandError(400, "command body must be 1 to 4096 bytes")
            self.connection.settimeout(3.0)
            command = validate_command(json.loads(self.rfile.read(length), parse_constant=reject_json_constant))
            result = self.node.submit_command(command)
            try:
                ack = result.result(timeout=self.node.command_timeout + 1.0)
            except concurrent.futures.TimeoutError as error:
                result.cancel()
                raise CommandError(504, "runtime command acknowledgement timed out") from error
            self._respond(200, {**ack, "ok": True})
        except (ValueError, TypeError, UnicodeError) as error:
            self.state.command_error(str(error))
            self._respond(400, {"ok": False, "accepted": False, "error": str(error)})
        except CommandError as error:
            self.state.command_error(str(error))
            self._respond(error.status, {
                "ok": False, "accepted": False, "error": str(error), "sequence": error.sequence,
            })
        except TimeoutError:
            self._respond(408, {"ok": False, "accepted": False, "error": "request body timed out"})

    def log_message(self, _format: str, *_args: Any) -> None:
        return


class MujocoRosBridge(Node):
    """Publish simulator state as standard ROS messages and forward commands."""

    def __init__(self, shared_state: BridgeState) -> None:
        """Create the Go2 ROS graph and runtime command acknowledgement tracking."""
        super().__init__("mujoco_robonix_bridge")
        self.shared_state = shared_state
        self.loop: asyncio.AbstractEventLoop | None = None
        self.websocket: Any = None
        self.sequence = -1
        self.last_sim_time = 0.0
        self.pending: dict[int, tuple[Any, asyncio.Future]] = {}
        self.policy_sequence: int | None = None
        self.command_timeout = float(os.environ.get("BRIDGE_COMMAND_TIMEOUT", "10"))
        if not math.isfinite(self.command_timeout) or not 0 < self.command_timeout <= 60:
            raise ValueError("BRIDGE_COMMAND_TIMEOUT must be finite and in (0, 60] seconds")

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", SENSOR_QOS)
        self.cloud_pub = self.create_publisher(PointCloud2, "/mid360/points", SENSOR_QOS)
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, "/rtabmap/cloud_map", LATCHED_QOS)
        self.imu_pub = self.create_publisher(Imu, "/mid360/imu", SENSOR_QOS)
        self.standard_joints_pub = self.create_publisher(JointState, "/joint_states", 20)
        self.camera_publishers: dict[str, dict[str, Any]] = {
            "front_rgbd": {
                "rgb": self.create_publisher(Image, "/front/rgb", SENSOR_QOS),
                "depth": self.create_publisher(Image, "/front/depth", SENSOR_QOS),
                "info": self.create_publisher(CameraInfo, "/front/camera_info", LATCHED_QOS),
            },
        }
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.front_extrinsics_pub = self.create_publisher(
            TransformStamped, "/front/extrinsics", LATCHED_QOS)
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 20)
        self.create_subscription(Empty, "/sim/reset", self._on_reset, 10)
        self.create_subscription(
            PointCloud2, "/cloud_map", self._on_map_cloud, LATCHED_QOS)
        self._publish_static_transforms()

    def _publish_static_transforms(self) -> None:
        """Publish measured simulator mounts in the base_link frame."""
        transforms = [
            ("base_link", "mid360_link", (0.0, 0.0, 0.16), (0.0, 0.0, 0.0, 1.0)),
            ("base_link", "front_camera_optical_frame", (0.32, 0.0, 0.12), (0.5, -0.5, 0.5, -0.5)),
            ("base_link", "imu_link", (-0.02557, 0.0, 0.04232), (0.0, 0.0, 0.0, 1.0)),
        ]
        messages = []
        for parent, child, xyz, xyzw in transforms:
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = parent, child
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
            message.transform.rotation.x, message.transform.rotation.y = xyzw[0], xyzw[1]
            message.transform.rotation.z, message.transform.rotation.w = xyzw[2], xyzw[3]
            messages.append(message)
        self.static_tf.sendTransform(messages)
        self.front_extrinsics_pub.publish(messages[1])

    def send(self, payload: dict[str, Any]) -> None:
        """Forward ROS commands asynchronously and report rejected or missing acknowledgements."""
        try:
            result = self.submit_command(validate_command(payload))
            result.add_done_callback(self._command_done)
        except CommandError as error:
            self.shared_state.command_error(str(error))
            self.get_logger().warning(str(error))

    def _command_done(self, result: concurrent.futures.Future) -> None:
        """Consume ROS command failures so transport errors are never silently discarded."""
        try:
            result.result()
        except (CommandError, concurrent.futures.CancelledError) as error:
            self.shared_state.command_error(str(error))
            self.get_logger().warning(f"simulator command failed: {error}")

    def submit_command(self, payload: dict[str, Any]) -> concurrent.futures.Future:
        """Schedule a command for the runtime that owns the connection at submission."""
        loop, owner = self.loop, self.websocket
        if loop is None or not loop.is_running() or owner is None:
            raise CommandError(503, "simulator runtime is disconnected")
        coroutine = self._send_command(payload, owner)
        try:
            return asyncio.run_coroutine_threadsafe(coroutine, loop)
        except RuntimeError as error:
            coroutine.close()
            raise CommandError(503, "simulator event loop is unavailable") from error

    async def _send_command(self, payload: dict[str, Any], owner: Any) -> dict[str, Any]:
        """Allocate sequences on the WS loop and await only the owning runtime's ack."""
        if self.websocket is not owner:
            raise CommandError(503, "simulator runtime was disconnected or replaced")
        command = dict(payload)
        sequence = command.get("sequence", self.sequence + 1)
        if sequence <= self.sequence or sequence > 2**53 - 1:
            raise CommandError(409, f"sequence must be greater than {self.sequence}", sequence)
        if command["type"] == "policy" and self.policy_sequence is not None:
            raise CommandError(409, "a policy command is still awaiting runtime acknowledgement", sequence)
        self.sequence = sequence
        command["sequence"] = sequence
        future = asyncio.get_running_loop().create_future()
        self.pending[sequence] = (owner, future)
        if command["type"] == "policy":
            self.policy_sequence = sequence
        try:
            deadline = asyncio.get_running_loop().time() + self.command_timeout
            await asyncio.wait_for(owner.send(json.dumps(command)), timeout=self.command_timeout)
            remaining = max(0.0, deadline - asyncio.get_running_loop().time())
            ack = await asyncio.wait_for(asyncio.shield(future), timeout=remaining)
            if self.websocket is not owner:
                raise CommandError(503, "simulator runtime was replaced before command completion", sequence)
            if ack.get("accepted") is not True:
                policy = ack.get("policy")
                policy_error = policy.get("error") if isinstance(policy, dict) else None
                detail = ack.get("error") or ack.get("detail") or policy_error or "runtime rejected command"
                raise CommandError(422, str(detail), sequence)
            self.shared_state.command_error(None)
            return ack
        except asyncio.TimeoutError as error:
            raise CommandError(504, "runtime command acknowledgement timed out; outcome is unknown", sequence) from error
        except (websockets.exceptions.ConnectionClosed, OSError) as error:
            raise CommandError(503, "simulator runtime disconnected before acknowledgement", sequence) from error
        finally:
            # A timed-out policy operation may still run; keep its fence until ack or disconnect.
            if self.policy_sequence != sequence or future.done():
                self.pending.pop(sequence, None)
                if self.policy_sequence == sequence:
                    self.policy_sequence = None
            if not future.done():
                future.cancel()
            elif not future.cancelled():
                future.exception()

    def _acknowledge(self, payload: dict[str, Any], owner: Any) -> None:
        """Resolve a command only for an exact integer sequence and runtime owner."""
        sequence = payload.get("sequence")
        if type(sequence) is not int:
            return
        pending = self.pending.get(sequence)
        if pending is None or pending[0] is not owner:
            return
        if type(payload.get("accepted")) is not bool:
            return
        if not pending[1].done():
            pending[1].set_result(payload)
        self.pending.pop(sequence, None)
        if self.policy_sequence == sequence:
            self.policy_sequence = None
        if isinstance(payload.get("policy"), dict):
            with self.shared_state.lock:
                self.shared_state.policy = copy.deepcopy(payload["policy"])

    def _disconnect_commands(self, owner: Any) -> None:
        """Fail commands from a replaced or disconnected runtime without affecting its successor."""
        for sequence, (command_owner, future) in list(self.pending.items()):
            if command_owner is owner:
                if not future.done():
                    future.set_exception(CommandError(503, "simulator runtime disconnected or replaced", sequence))
                self.pending.pop(sequence, None)
                if self.policy_sequence == sequence:
                    self.policy_sequence = None

    def _on_twist(self, msg: Twist) -> None:
        self.send({
            "type": "cmd_vel", "linearX": msg.linear.x, "linearY": msg.linear.y,
            "angularZ": msg.angular.z,
        })

    def _on_reset(self, _msg: Empty) -> None:
        self.send({"type": "reset"})

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        """Bridge RTAB-Map's actual output to its advertised Robonix topic."""
        self.map_cloud_pub.publish(msg)

    def receive(self, payload: dict[str, Any]) -> None:
        """Route one simulator protocol frame to its ROS publisher."""
        kind = str(payload.get("type", ""))
        if kind == "state":
            self._publish_state(payload)
            self.shared_state.update(payload)
        elif kind == "scan":
            self._publish_scan(payload)
        elif kind == "pointcloud":
            self._publish_cloud(payload)
        elif kind == "camera":
            self._publish_camera(payload)
        else:
            return
        self.shared_state.record(kind)

    def _publish_state(self, payload: dict[str, Any]) -> None:
        """Publish Go2 pose, body-frame twist, leg joints, clock, TF, and IMU."""
        sim_time = float(payload.get("simulationTime", 0.0))
        self.last_sim_time = sim_time
        at = stamp(sim_time)
        self.clock_pub.publish(Clock(clock=at))
        robot = payload.get("robot") or {}
        base = robot.get("base") or {}
        position = base.get("position") or [0.0, 0.0, 0.0]
        quaternion = base.get("quaternion") or [1.0, 0.0, 0.0, 0.0]
        linear = world_to_body(base.get("linearVelocity") or [0.0] * 3, quaternion)
        angular = base.get("angularVelocity") or [0.0] * 3

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = at, "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
        set_quaternion(odom.pose.pose.orientation, quaternion)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, linear)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, angular)
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = at, "odom", "base_link"
        transform.transform.translation.x, transform.transform.translation.y = float(position[0]), float(position[1])
        transform.transform.translation.z = float(position[2])
        set_quaternion(transform.transform.rotation, quaternion)
        self.tf.sendTransform(transform)

        joint_data = robot.get("joints") or payload.get("joints") or {}
        joints = JointState()
        joints.header.stamp, joints.header.frame_id = at, "base_link"
        joints.name = list(joint_data.get("names") or [])
        joints.position = [float(value) for value in joint_data.get("positions") or []]
        joints.velocity = [float(value) for value in joint_data.get("velocities") or []]
        if any(len(values) not in (0, len(joints.name)) for values in (joints.position, joints.velocity)):
            raise ValueError("joint names, positions, and velocities must have matching lengths")
        self.standard_joints_pub.publish(joints)

        imu_data = payload.get("imu")
        if imu_data:
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = at, "imu_link"
            gyro = imu_data.get("gyroscope") or [0.0] * 3
            accel = imu_data.get("accelerometer") or [0.0] * 3
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, gyro)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, accel)
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)

    def _publish_scan(self, payload: dict[str, Any]) -> None:
        """Publish the planar scan in the lidar mount frame."""
        ranges = decode_array(payload.get("rangesF32"), np.dtype("<f4"))
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp(payload.get("timestamp", self.last_sim_time)), "mid360_link"
        msg.angle_min = float(payload.get("angleMin", -math.pi))
        msg.angle_max = float(payload.get("angleMax", math.pi))
        msg.angle_increment = float(payload.get("angleIncrement", 2 * math.pi / max(1, ranges.size)))
        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / max(1, ranges.size)
        msg.range_min, msg.range_max = float(payload.get("rangeMin", 0.1)), float(payload.get("rangeMax", 30.0))
        msg.ranges = ranges.astype(np.float32, copy=False).tolist()
        self.scan_pub.publish(msg)

    def _publish_cloud(self, payload: dict[str, Any]) -> None:
        """Publish packed XYZ lidar returns in the lidar mount frame."""
        points = decode_array(payload.get("xyzF32"), np.dtype("<f4")).reshape((-1, 3))
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp(payload.get("timestamp", self.last_sim_time)), "mid360_link"
        msg.height, msg.width = 1, int(points.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian, msg.point_step = False, 12
        msg.row_step, msg.is_dense = msg.width * msg.point_step, False
        msg.data = points.astype(np.dtype("<f4"), copy=False).tobytes()
        self.cloud_pub.publish(msg)

    def _publish_camera(self, payload: dict[str, Any]) -> None:
        """Publish front RGB and optical-Z depth, resizing all rows and columns."""
        camera_id = str(payload.get("id", ""))
        publishers = self.camera_publishers.get(camera_id)
        if publishers is None:
            return
        width, height = int(payload["width"]), int(payload["height"])
        if width <= 0 or height <= 0:
            raise ValueError("camera dimensions must be positive")
        frame_id = "front_camera_optical_frame"
        at = stamp(payload.get("timestamp", self.last_sim_time))
        rgba = decode_array(payload.get("rgbaU8"), np.uint8).reshape((height, width, 4))
        rgb = Image()
        rgb.header.stamp, rgb.header.frame_id = at, frame_id
        rgb.height, rgb.width, rgb.encoding, rgb.is_bigendian, rgb.step = height, width, "rgb8", 0, width * 3
        rgb.data = np.ascontiguousarray(rgba[:, :, :3]).tobytes()
        publishers["rgb"].publish(rgb)
        publishers["info"].publish(camera_info(width, height, float(payload.get("fovyDeg", 60.0)), frame_id, at))

        depth_payload = payload.get("depth")
        if depth_payload and "depth" in publishers:
            depth_width, depth_height = int(depth_payload["width"]), int(depth_payload["height"])
            if depth_width <= 0 or depth_height <= 0:
                raise ValueError("depth dimensions must be positive")
            depth_values = decode_array(depth_payload.get("dataF32"), np.dtype("<f4")).reshape((depth_height, depth_width))
            if (depth_width, depth_height) != (width, height):
                rows = np.minimum(((np.arange(height) + 0.5) * depth_height / height).astype(int), depth_height - 1)
                columns = np.minimum(((np.arange(width) + 0.5) * depth_width / width).astype(int), depth_width - 1)
                depth_values = depth_values[np.ix_(rows, columns)]
            depth = Image()
            depth.header.stamp, depth.header.frame_id = at, frame_id
            depth.height, depth.width, depth.encoding, depth.is_bigendian, depth.step = height, width, "32FC1", 0, width * 4
            depth.data = depth_values.astype(np.dtype("<f4"), copy=False).tobytes()
            publishers["depth"].publish(depth)


async def handle_runtime(node: MujocoRosBridge, websocket: Any) -> None:
    """Require hello and protect fresh owners before atomically replacing a stale runtime."""
    try:
        if node.websocket is not None and node.shared_state.has_fresh_state():
            await websocket.close(4001, "a healthy simulator runtime already owns the bridge")
            return
        raw = await asyncio.wait_for(websocket.recv(), timeout=RUNTIME_HELLO_TIMEOUT)
        hello = json.loads(raw, parse_constant=reject_json_constant)
        if not isinstance(hello, dict) or hello.get("type") != "hello":
            raise ValueError("first runtime frame must be hello")
        # The previous owner can recover while this peer is sending its hello.
        if node.websocket is not None and node.shared_state.has_fresh_state():
            await websocket.close(4001, "a healthy simulator runtime already owns the bridge")
            return
    except (ValueError, TypeError, asyncio.TimeoutError) as error:
        node.get_logger().warning(f"runtime handshake rejected: {str(error) or 'hello timed out'}")
        await websocket.close(1008, "a valid hello is required within three seconds")
        return
    except websockets.exceptions.ConnectionClosed:
        return

    previous = node.websocket
    node.websocket = websocket
    node.last_sim_time = 0.0
    node.shared_state.connect()
    node.shared_state.hello(hello)
    node.get_logger().info(
        f"runtime connected: backend={node.shared_state.backend} "
        f"environment={node.shared_state.environment}"
    )
    if previous is not None:
        node._disconnect_commands(previous)
    try:
        if previous is not None:
            await previous.close(4001, "replaced by a newer simulator runtime")
        async for raw in websocket:
            if node.websocket is not websocket:
                break
            try:
                payload = json.loads(raw, parse_constant=reject_json_constant)
                if not isinstance(payload, dict):
                    raise ValueError("runtime frame must be a JSON object")
                if payload.get("type") == "hello":
                    node.shared_state.hello(payload)
                    node.get_logger().info(
                        f"runtime connected: backend={node.shared_state.backend} "
                        f"environment={node.shared_state.environment}"
                    )
                elif payload.get("type") == "command_ack":
                    node._acknowledge(payload, websocket)
                else:
                    node.receive(payload)
            except (ValueError, TypeError, KeyError, IndexError, OverflowError) as error:
                node.get_logger().warning(f"invalid simulator frame: {error}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        node._disconnect_commands(websocket)
        if node.websocket is websocket:
            node.websocket = None
            with node.shared_state.lock:
                node.shared_state.connected = False
            node.get_logger().warning("simulator runtime disconnected; motion watchdog will stop the robot")


async def websocket_main(node: MujocoRosBridge, host: str, port: int) -> None:
    """Accept one active simulator runtime and replace stale sessions deterministically."""
    node.loop = asyncio.get_running_loop()

    async def handle(websocket: Any) -> None:
        await handle_runtime(node, websocket)

    async with websockets.serve(handle, host, port, max_size=32 * 1024 * 1024, ping_interval=10, ping_timeout=10):
        await asyncio.Future()


def main() -> None:
    """Start ROS, health HTTP, and WebSocket runtimes with clean shutdown."""
    rclpy.init()
    state = BridgeState()
    node = MujocoRosBridge(state)

    def spin_ros() -> None:
        """Run ROS callbacks until the context shuts down."""
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    HealthHandler.state = state
    HealthHandler.node = node
    bind = os.environ.get("BRIDGE_BIND", "127.0.0.1")
    health = ThreadingHTTPServer((bind, int(os.environ.get("BRIDGE_HEALTH_PORT", "8766"))), HealthHandler)
    threading.Thread(target=health.serve_forever, daemon=True).start()
    try:
        asyncio.run(websocket_main(node, bind, int(os.environ.get("BRIDGE_WS_PORT", "8765"))))
    except KeyboardInterrupt:
        pass
    finally:
        health.shutdown()
        health.server_close()
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2)


if __name__ == "__main__":
    main()
