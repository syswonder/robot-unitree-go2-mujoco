#!/usr/bin/env python3
"""Go2 live acceptance; run --help for bounded, opt-in motion and stack checks.

The parent boots the deployment. This test never resets or teleports the body.
stdout contains one JSON report; unrequested checks are explicitly skipped.
Robonix discovery and MCP JSON-RPC run in bounded subprocesses so ROS keeps spinning.
"""
from __future__ import annotations

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


NAV = "robonix/service/navigation/navigate"
CHASSIS = "robonix/primitive/chassis/move"
EXPLORE = "robonix/skill/explore/explore"
SCENE = "robonix/system/scene/"
TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"}
PROVIDERS = ("go2_chassis", "mid360_lidar", "mid360_imu", "front_camera",
             "mapping", "nav2", "scene", "soma", "explore")


def require(condition, detail):
    if not condition:
        raise AssertionError(detail)


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def quaternion(q):
    """Validate a unit quaternion and return yaw and tilt from vertical."""
    values = (q.x, q.y, q.z, q.w)
    require(all(math.isfinite(v) for v in values), "nonfinite quaternion")
    require(abs(sum(v * v for v in values) - 1) < 0.02, "invalid quaternion norm")
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    tilt = math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x*q.x + q.y*q.y))))
    return yaw, tilt


def image_stats(message):
    """Validate active pixels with row padding/endian handling; depth is optical Z in metres."""
    import numpy as np
    formats = {"rgb8": ("u1", 3), "bgr8": ("u1", 3), "rgba8": ("u1", 4),
               "bgra8": ("u1", 4), "32FC1": ("f4", 1), "16UC1": ("u2", 1)}
    require(message.encoding in formats, f"unsupported image encoding {message.encoding}")
    dtype, channels = formats[message.encoding]
    dtype = np.dtype((">" if message.is_bigendian else "<") + dtype)
    row_bytes = message.width * channels * dtype.itemsize
    require(message.width > 0 and message.height > 0 and message.step >= row_bytes,
            "invalid image dimensions/stride")
    require(len(message.data) == message.height * message.step, "image buffer length mismatch")
    pixels = np.ndarray((message.height, message.width, channels), dtype=dtype,
                        buffer=bytes(message.data), strides=(message.step, channels*dtype.itemsize, dtype.itemsize))
    if channels == 1:
        values = pixels.astype(float).ravel() * (0.001 if message.encoding == "16UC1" else 1.0)
        valid = values[np.isfinite(values) & (values > 0) & (values < 100)]
        require(valid.size >= max(100, values.size * 0.01), "depth has insufficient valid optical returns")
        return {"valid": int(valid.size), "min_m": float(valid.min()), "max_m": float(valid.max())}
    pixels = pixels[:, :, :3].astype(float)
    spatial_std = float(pixels.std(axis=(0, 1)).max())
    require(pixels.size > 100 and spatial_std > 2, "RGB is spatially blank")
    return {"spatial_std": spatial_std, "width": message.width, "height": message.height}


def cloud_stats(message):
    """Decode XYZ fields, checking finite geometric returns rather than buffer size alone."""
    import numpy as np
    require(message.width > 0 and message.height > 0, "empty point cloud")
    require(message.row_step >= message.width * message.point_step, "invalid cloud row stride")
    require(len(message.data) == message.row_step * message.height, "cloud buffer length mismatch")
    fields = {field.name: field for field in message.fields}
    coordinates = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        require(field is not None and field.datatype in (7, 8) and field.count == 1,
                f"missing/invalid cloud field {name}")
        dtype = np.dtype((">" if message.is_bigendian else "<") + ("f4" if field.datatype == 7 else "f8"))
        require(0 <= field.offset <= message.point_step - dtype.itemsize, "cloud field outside point")
        coordinates.append(np.ndarray((message.height, message.width), dtype=dtype,
                           buffer=bytes(message.data), offset=field.offset,
                           strides=(message.row_step, message.point_step)).ravel())
    xyz = np.column_stack(coordinates)
    valid = xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.01)]
    require(len(valid) >= 100 and float(np.ptp(valid, axis=0).max()) > 0.1,
            "point cloud lacks finite spatial returns")
    return {"points": int(len(xyz)), "valid": int(len(valid))}


def ensure_skill_active(atlas, provider_id, call_driver):
    """Mirror Executor's lifecycle guard; activate once from INACTIVE and confirm Atlas ACTIVE."""
    providers = atlas.query(id=provider_id)
    require(len(providers) == 1, f"skill {provider_id} not uniquely registered")
    provider = providers[0]
    require(provider.kind.name == "SKILL", f"{provider_id} is not a skill")
    state = provider.state.name
    if state == "ACTIVE":
        return {"provider_id": provider_id, "state": state, "activated": False}
    require(state == "INACTIVE", f"skill {provider_id} is {state}; activation requires INACTIVE")
    drivers = [cap for cap in provider.capabilities
               if cap.transport.name == "GRPC" and cap.contract_id.endswith("/driver")]
    require(len(drivers) == 1, f"skill {provider_id} must have exactly one gRPC driver")
    response = call_driver(drivers[0])
    require(response.get("ok") is True and str(response.get("state", "")).upper() == "ACTIVE",
            f"Driver(CMD_ACTIVATE) failed: {response}")
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        providers = atlas.query(id=provider_id)
        require(len(providers) == 1, f"skill {provider_id} disappeared during activation")
        state = providers[0].state.name
        if state == "ACTIVE":
            return {"provider_id": provider_id, "state": state, "activated": True,
                    "driver_contract": drivers[0].contract_id}
        require(state == "INACTIVE", f"skill {provider_id} entered {state} during activation")
        time.sleep(0.1)
    raise AssertionError(f"skill {provider_id} did not become ACTIVE after CMD_ACTIVATE")


def rpc_worker(request):
    """Discover exact provider/contracts with robonix_api; use MCP's negotiated JSON-RPC client."""
    import asyncio
    package = ("skills/explore" if request.get("provider") == "explore"
               else "primitives/chassis")
    codegen = Path(__file__).resolve().parents[2] / package / "rbnx-build/codegen"
    require((codegen / "proto_gen/atlas_pb2.py").is_file()
            and (codegen / "proto_gen/atlas_pb2_grpc.py").is_file(),
            f"missing Atlas stubs in {codegen / 'proto_gen'}; build {package} first")
    sys.path[:0] = [str(codegen / "proto_gen"), str(codegen / "robonix_mcp_types")]
    for root in ("/robonix/pylib/robonix-api", os.environ.get("ROBONIX_SOURCE_PATH", "/home/cyt/robonix") + "/pylib/robonix-api"):
        if Path(root).is_dir():
            sys.path.insert(0, root)
    from robonix_api import ATLAS
    if request["operation"] == "providers":
        return {p.id: p.state.name for p in ATLAS.query()}
    providers = ATLAS.query(id=request["provider"])
    require(len(providers) == 1, f"{request['provider']} not uniquely registered")
    if request["operation"] == "chassis":
        import grpc
        import chassis_pb2
        import robonix_contracts_pb2_grpc
        cap = ATLAS.find_unique_capability(
            provider_id=request["provider"], contract_id=request["contract"], transport="grpc")
        command = chassis_pb2.MoveCommand(**request["arguments"])
        with ATLAS.connect_capability(
                consumer_id="go2_acceptance", provider_id=cap.provider_id,
                contract_id=cap.contract_id, transport="grpc") as channel:
            with grpc.insecure_channel(channel.endpoint, options=[("grpc.enable_http_proxy", 0)]) as wire:
                response = robonix_contracts_pb2_grpc.RobonixPrimitiveChassisMoveStub(
                    wire).ExecuteMoveCommand(
                        chassis_pb2.ExecuteMoveCommand_Request(command=command), timeout=360)
        return json.loads(response.status.data)
    if providers[0].kind.name == "SKILL" or request["operation"] == "activate":
        def call_driver(cap):
            """Use package-generated Driver serialization and stub routing with a 60-second deadline."""
            import grpc
            import lifecycle_pb2
            import robonix_contracts_pb2_grpc
            from robonix_api.lifecycle import CMD_ACTIVATE, contract_id_to_pascal
            fields = lifecycle_pb2.Driver_Request.DESCRIPTOR.fields_by_name
            require("command" in fields and "config_json" in fields, "generated Driver request shape mismatch")
            stub_name = contract_id_to_pascal(cap.contract_id) + "Stub"
            stub_type = getattr(robonix_contracts_pb2_grpc, stub_name, None)
            require(stub_type is not None, f"missing package-generated driver stub {stub_name}")
            with ATLAS.connect_capability(consumer_id="go2_acceptance", provider_id=cap.provider_id,
                                          contract_id=cap.contract_id, transport="grpc") as channel:
                with grpc.insecure_channel(channel.endpoint, options=[("grpc.enable_http_proxy", 0)]) as wire:
                    response = stub_type(wire).Driver(
                        lifecycle_pb2.Driver_Request(command=CMD_ACTIVATE, config_json="{}"), timeout=60)
            return {"ok": response.ok, "state": response.state, "error": response.error}
        activation = ensure_skill_active(ATLAS, request["provider"], call_driver)
        if request["operation"] == "activate":
            return activation
    cap = ATLAS.find_unique_capability(provider_id=request["provider"],
                                      contract_id=request["contract"], transport="mcp")

    async def call(endpoint):
        """Discover the actual input schema and reject MCP errors or non-object results."""
        from fastmcp import Client
        async with Client(endpoint, timeout=12) as client:
            name = request["contract"].rsplit("/", 1)[-1]
            matches = [tool for tool in await client.list_tools() if tool.name == name]
            require(len(matches) == 1, f"MCP tools/list missing {name}")
            properties = matches[0].inputSchema.get("properties", {})
            arguments = request["arguments"]
            if len(properties) == 1 and next(iter(properties)) in ("req", "_req", "request"):
                arguments = {next(iter(properties)): arguments}
            result = await client.call_tool(name, arguments, raise_on_error=True)
            data = result.structured_content
            if data is None:
                texts = [c.text for c in result.content if getattr(c, "type", "") == "text"]
                require(len(texts) == 1, "MCP response has no unambiguous JSON result")
                data = json.loads(texts[0])
            # FastMCP wraps handlers annotated Any in a structured result object.
            if isinstance(data, dict) and set(data) == {"result"}:
                data = data["result"]
            require(isinstance(data, dict), "MCP result is not an object")
            return data
    with ATLAS.connect_capability(consumer_id="go2_acceptance", provider_id=cap.provider_id,
                                  contract_id=cap.contract_id, transport="mcp") as channel:
        return asyncio.run(asyncio.wait_for(call(channel.endpoint), timeout=14))


def parser():
    """Expose independent checks, parent-supplied endpoints and explicit measurement bounds."""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--motion", action="store_true", help="drive forward without resetting the body")
    result.add_argument("--relative", type=float, metavar="METRES",
                        help="verify measured forward/backward and +/-30 degree chassis moves")
    result.add_argument("--require-map", action="store_true", help="require occupancy and fused cloud")
    result.add_argument("--navigate", nargs=2, type=float, metavar=("X", "Y"), help="map-frame goal; also test cancellation on the return leg")
    result.add_argument("--yaw", type=float, default=0.0)
    result.add_argument("--explore", action="store_true", help="observe exploration movement/map growth, then cancel and verify terminal status")
    result.add_argument("--explore-speed", type=float, default=0.18, help="requested exploration speed in m/s")
    result.add_argument("--explore-duration", type=float, default=45.0,
                        help="wall seconds to observe exploration before intentional cancellation (default: 45)")
    result.add_argument("--semantic", action="store_true", help="observe a live Scene object, goal_near, then Robonix navigation")
    result.add_argument("--object-id", default="", help="optional observed Scene object ID; otherwise select a reachable live object")
    result.add_argument("--require-stack", action="store_true", help="check all nine registered primitives/services/skills")
    result.add_argument("--atlas", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    result.add_argument("--map-epoch-start", type=float, default=0.0,
                        help="earliest map timestamp in this simulation run; supply when the clock continues across map resets")
    result.add_argument("--container", default="", help=argparse.SUPPRESS)
    for name, default in (("sensor-timeout", 30), ("navigation-timeout", 120), ("explore-timeout", 180),
                          ("semantic-timeout", 60), ("max-age", 2), ("max-skew", 0.3),
                          ("position-tolerance", 0.2), ("yaw-tolerance", 0.2), ("min-motion", 0.06),
                          ("max-tilt", 0.65), ("min-height", 0.15), ("motion-seconds", 3)):
        result.add_argument("--" + name, type=float, default=default)
    return result


def runtime(args, report):
    """Subscribe to the live ROS body and guarantee bounded cancellation/stop on every exit."""
    import numpy as np
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2, CameraInfo
    from tf2_ros import Buffer, TransformListener, TransformException

    class AcceptanceNode(Node):
        def __init__(self):
            """Collect bounded histories and motion evidence from ROS, including /clock and TF."""
            super().__init__("go2_acceptance")
            self.messages, self.received, self.history = {}, {}, {}
            self.samples = []
            self.monitoring = False
            self.violation = None
            self.clock_reset = False
            self.handles, self.runs, self.pending_goals = [], [], []
            self.pool = ThreadPoolExecutor(max_workers=1)
            sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            topics = (("scan", LaserScan, "/scan"), ("cloud", PointCloud2, "/mid360/points"),
                      ("imu", Imu, "/mid360/imu"), ("rgb", Image, "/front/rgb"),
                      ("depth", Image, "/front/depth"), ("info", CameraInfo, "/front/camera_info"),
                      ("odom", Odometry, "/odom"), ("joints", JointState, "/joint_states"),
                      ("map", OccupancyGrid, "/map"), ("map_cloud", PointCloud2, "/rtabmap/cloud_map"),
                      ("clock", Clock, "/clock"))
            self.subscriptions_owned = []
            for key, kind, topic in topics:
                self.history[key] = deque(maxlen=8)
                # RTAB-Map retains geometry between keyframes, including long stationary periods.
                qos = latched if key in ("map", "map_cloud") else sensor
                self.subscriptions_owned.append(self.create_subscription(
                    kind, topic, lambda msg, key=key: self.receive(key, msg), qos))
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        def receive(self, key, msg):
            """Track fresh stamps and latch any fall or discontinuity throughout commanded motion."""
            now = time.monotonic()
            previous = self.messages.get(key)
            if key == "clock" and previous is not None:
                before = (previous.clock.sec, previous.clock.nanosec)
                self.clock_reset |= (msg.clock.sec, msg.clock.nanosec) < before
            if self.monitoring and previous is not None and key != "clock" and stamp(msg) < stamp(previous):
                self.violation = f"{key} timestamp moved backwards during motion"
            self.messages[key], self.received[key] = msg, now
            self.history[key].append(msg)
            if key == "odom" and self.monitoring:
                try:
                    pose = msg.pose.pose
                    yaw, tilt = quaternion(pose.orientation)
                    point = np.array([pose.position.x, pose.position.y, pose.position.z])
                    require(np.isfinite(point).all() and point[2] >= args.min_height, "body height invalid/not upright")
                    require(tilt <= args.max_tilt, f"body tilted {tilt:.3f}rad")
                    if self.samples:
                        at, old, _, _ = self.samples[-1]
                        dt = stamp(msg) - at
                        require(dt >= 0 and np.linalg.norm(point-old) <= 2 * dt + 0.05,
                                "body pose jumped; reset/teleport or invalid odometry")
                    self.samples.append((stamp(msg), point, tilt, yaw))
                except AssertionError as exc:
                    self.violation = str(exc)

        def wait(self, predicate, seconds, detail, safety=True):
            """Spin against wall time; moving tasks fail promptly on stale odometry or a fall."""
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if safety:
                    require(not self.clock_reset, "simulation clock reset; map belongs to an unverified epoch")
                if safety and self.monitoring:
                    require(self.violation is None, self.violation)
                    require(time.monotonic() - self.received.get("odom", 0) < args.max_age, "odometry stalled during motion")
                if predicate():
                    return
            raise AssertionError(f"timeout waiting for {detail}")

        def pause(self, seconds, safety=True):
            end = time.monotonic() + seconds
            self.wait(lambda: time.monotonic() >= end, seconds + 1, "observation interval", safety)

        def rpc(self, provider="", contract="", arguments=None, operation="call", safety=True):
            """Bound discovery and MCP execution in a child while continuing ROS monitoring."""
            request = dict(provider=provider, contract=contract, arguments=arguments or {}, operation=operation)
            timeout = 360 if operation == "chassis" else (
                90 if operation == "activate" or provider == "explore" else 18)
            future = self.pool.submit(subprocess.run, [sys.executable, __file__, "--rpc-worker"],
                                      input=json.dumps(request), text=True, capture_output=True, timeout=timeout)
            self.wait(future.done, timeout + 2, f"Robonix {contract or operation}", safety)
            completed = future.result()
            require(completed.returncode == 0, f"MCP {contract}: {completed.stderr[-1500:]}")
            return json.loads(completed.stdout)

        def pose(self, frame="map", at=None):
            """Resolve current or sensor-time body pose in the requested frame, rejecting stale TF."""
            found = []
            def lookup():
                """Wait for the requested transform with a current dynamic timestamp."""
                try:
                    transform = self.tf.lookup_transform(frame, "base_link", at or Time())
                except TransformException:
                    return False
                if at is None and abs(stamp(transform) - stamp(self.messages["odom"])) > args.max_skew:
                    return False
                found.append(transform)
                return True
            self.wait(lookup, 5, f"fresh {frame} -> base_link TF")
            transform = found[0].transform
            yaw, _ = quaternion(transform.rotation)
            return np.array([transform.translation.x, transform.translation.y, yaw])

        def begin_motion(self, frame="map"):
            """Start a fresh body trace; wait for an upright sample before issuing commands."""
            self.samples, self.violation = [], None
            self.monitoring = True
            self.wait(lambda: len(self.samples) >= 2, 5, "fresh upright odometry")
            return self.pose(frame)

        def evidence(self, allow_rotation=False):
            """Require measured translation, or measured rotation when cancellation starts by turning."""
            require(self.violation is None, self.violation)
            require(len(self.samples) >= 3, "insufficient live body samples")
            points = np.array([sample[1][:2] for sample in self.samples])
            displacement = float(np.linalg.norm(points - points[0], axis=1).max())
            headings = np.unwrap(np.array([sample[3] for sample in self.samples]))
            heading_change = float(np.abs(headings - headings[0]).max())
            require(displacement >= args.min_motion or (
                allow_rotation and heading_change >= math.radians(8)),
                f"body moved only {displacement:.4f}m / {math.degrees(heading_change):.2f}deg")
            return {"max_displacement_m": displacement, "samples": len(points),
                    "max_heading_change_deg": math.degrees(heading_change),
                    "trace_frame": "odom", "start_position_xy": points[0].tolist(),
                    "end_position_xy": points[-1].tolist(),
                    "bounds_xy": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                    "max_tilt_rad": max(sample[2] for sample in self.samples)}

        def stop(self):
            """Send repeated zero commands and observe the body settling without reset."""
            for _ in range(6):
                self.cmd.publish(Twist())
                self.pause(0.05, safety=False)

        def settled(self):
            """Require actual stopped body displacement over a fresh observation interval."""
            self.pause(1)
            start = len(self.samples)
            self.wait(
                lambda: len(self.samples) >= start + 3 and
                self.samples[-1][0] - self.samples[start][0] >= 0.4,
                12, "three fresh stopped odometry samples")
            samples = self.samples[start:]
            points = np.array([sample[1][:2] for sample in samples])
            require(float(np.linalg.norm(points - points[0], axis=1).max()) < 0.05,
                    "body continues moving after stop/cancel")

        def arrival(self, target):
            """Check map-frame position/yaw after terminal success and independently prove movement."""
            self.settled()
            actual = self.pose()
            error = float(np.linalg.norm(actual[:2] - target[:2]))
            yaw_error = abs(math.atan2(math.sin(actual[2]-target[2]), math.cos(actual[2]-target[2])))
            require(error <= args.position_tolerance, f"goal position error {error:.3f}m")
            require(yaw_error <= args.yaw_tolerance, f"goal yaw error {yaw_error:.3f}rad")
            return dict(target=list(target), actual=actual.tolist(), position_error_m=error,
                        yaw_error_rad=yaw_error, **self.evidence())

        def send_goal(self, target):
            """Send one Nav2 action goal and retain its handle for cancellation on all exits."""
            require(self.nav.wait_for_server(timeout_sec=5), "Nav2 action server unavailable")
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.messages["odom"].header.stamp
            goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, target[:2])
            goal.pose.pose.orientation.z = math.sin(target[2]/2)
            goal.pose.pose.orientation.w = math.cos(target[2]/2)
            future = self.nav.send_goal_async(goal)
            self.pending_goals.append(future)
            self.wait(future.done, 8, "Nav2 acceptance", safety=False)
            handle = future.result()
            require(handle is not None and handle.accepted, "Nav2 rejected goal")
            self.handles.append(handle)
            return handle, handle.get_result_async()

        def task(self, provider, contract, arguments, seconds):
            """Start and poll an existing async capability with the exact returned run_id."""
            started = self.rpc(provider, contract, arguments, safety=False)
            require(started.get("accepted") is True and started.get("run_id"), f"task rejected: {started}")
            run_id = started["run_id"]
            self.runs.append((provider, contract, run_id))
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                status = self.rpc(provider, contract + "/status", {"run_id": run_id})
                require(status.get("known") is True, f"unknown run {run_id}")
                require(status.get("state") in TERMINAL | {"PENDING", "RUNNING", "PAUSED"}, f"invalid task state: {status}")
                if status["state"] in TERMINAL:
                    require(status["state"] == "SUCCEEDED", f"task did not succeed: {status}")
                    self.runs.remove((provider, contract, run_id))
                    return status
                self.pause(0.3)
            raise AssertionError(f"timeout waiting for {contract} run {run_id}")

        def cleanup(self):
            """Cancel retained goals/tasks and always stop; cleanup failures fail the JSON report."""
            errors = []
            self.monitoring = False
            try:
                for pending in self.pending_goals:
                    try:
                        self.wait(pending.done, 3, "pending goal acceptance", safety=False)
                        handle = pending.result()
                        if handle and handle.accepted and handle not in self.handles:
                            self.handles.append(handle)
                    except Exception as exc:
                        errors.append(str(exc))
                for handle in self.handles:
                    try:
                        future = handle.cancel_goal_async()
                        self.wait(future.done, 5, "Nav2 cleanup cancellation", safety=False)
                        future.result()
                        result = handle.get_result_async()
                        self.wait(result.done, 5, "Nav2 cleanup terminal result", safety=False)
                        require(result.result().status in (4, 5, 6), "Nav2 cleanup has no terminal result")
                    except Exception as exc:
                        errors.append(str(exc))
                for provider, contract, run_id in self.runs:
                    try:
                        self.rpc(provider, contract + "/cancel", {"run_id": run_id}, safety=False)
                        status = self.rpc(provider, contract + "/status", {"run_id": run_id}, safety=False)
                        require(status.get("state") in TERMINAL, f"task still active after cleanup: {status}")
                    except Exception as exc:
                        errors.append(str(exc))
            finally:
                self.stop()
                self.pool.shutdown(wait=True)
            return errors

    # Keep the ROS context alive through finally so interrupt cleanup can still publish.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AcceptanceNode()
    try:
        check_sensors(node, args, report, Time)
        if args.require_stack or args.semantic or args.explore:
            states = node.rpc(operation="providers")
            for provider in PROVIDERS:
                allowed = {"ACTIVE", "INACTIVE"} if provider == "explore" else {"ACTIVE"}
                require(states.get(provider) in allowed, f"{provider} not ready: {states.get(provider)}")
            report["checks"]["stack"] = states
        if args.require_map or args.navigate or args.semantic or args.explore:
            report["checks"]["map"] = check_map(node, args)
        if args.motion:
            before_map = check_map(node, args) if "map" in report["checks"] else None
            node.begin_motion("odom")
            try:
                command = Twist()
                command.linear.x = 0.18
                deadline = time.monotonic() + args.motion_seconds
                while time.monotonic() < deadline:
                    node.cmd.publish(command)
                    node.pause(0.08)
            finally:
                node.stop()
            node.settled()
            report["checks"]["motion"] = node.evidence()
            if before_map is not None:
                report["checks"]["motion"]["map_after"] = check_map(
                    node, args, before_map, node.samples[0][0])
        if args.relative:
            start = node.begin_motion("odom")
            forward = node.rpc("go2_chassis", CHASSIS, {"forward_m": args.relative},
                               operation="chassis")
            node.settled()
            reached = node.pose("odom")
            delta = reached[:2] - start[:2]
            progress = float(delta @ np.array([math.cos(start[2]), math.sin(start[2])]))
            require(abs(progress - args.relative) <= 0.12,
                    f"relative forward error {progress-args.relative:.3f}m")
            backward = node.rpc("go2_chassis", CHASSIS, {"forward_m": -args.relative},
                                operation="chassis")
            node.settled()
            returned = node.pose("odom")
            return_error = float(np.linalg.norm(returned[:2] - start[:2]))
            require(return_error <= 0.15, f"relative return error {return_error:.3f}m")
            left = node.rpc("go2_chassis", CHASSIS, {"rotate_deg": 30}, operation="chassis")
            node.settled()
            left_pose = node.pose("odom")
            left_turn = math.atan2(math.sin(left_pose[2]-returned[2]),
                                   math.cos(left_pose[2]-returned[2]))
            require(abs(left_turn-math.radians(30)) <= math.radians(7),
                    f"relative left-turn error {math.degrees(left_turn)-30:.1f}deg")
            right = node.rpc("go2_chassis", CHASSIS, {"rotate_deg": -30}, operation="chassis")
            node.settled()
            final = node.pose("odom")
            heading_error = abs(math.atan2(math.sin(final[2]-returned[2]),
                                           math.cos(final[2]-returned[2])))
            require(heading_error <= math.radians(8),
                    f"relative heading return error {math.degrees(heading_error):.1f}deg")
            report["checks"]["relative"] = {
                "forward": forward, "backward": backward, "left": left, "right": right,
                "measured_forward_m": progress, "return_error_m": return_error,
                "measured_left_deg": math.degrees(left_turn),
                "heading_return_error_deg": math.degrees(heading_error), **node.evidence()}
        if args.navigate:
            before_map = check_map(node, args)
            start = node.begin_motion()
            target = [*args.navigate, args.yaw]
            require(np.linalg.norm(start[:2] - target[:2]) > args.position_tolerance + args.min_motion,
                    "navigation target too close to prove motion")
            _, result = node.send_goal(target)
            node.wait(result.done, args.navigation_timeout, "Nav2 goal completion")
            require(result.result().status == GoalStatus.STATUS_SUCCEEDED, "Nav2 did not succeed")
            report["checks"]["navigation"] = node.arrival(target)
            report["checks"]["navigation"]["map_after"] = check_map(
                node, args, before_map, node.samples[0][0])
            node.begin_motion()
            handle, result = node.send_goal(start)
            node.wait(lambda: len(node.samples) >= 3 and (
                np.linalg.norm(node.samples[-1][1][:2] - node.samples[0][1][:2]) >= args.min_motion or
                abs(math.atan2(math.sin(node.samples[-1][3] - node.samples[0][3]),
                               math.cos(node.samples[-1][3] - node.samples[0][3]))) >= math.radians(8)),
                      min(30, args.navigation_timeout), "movement before cancellation")
            require(not result.done(), "return goal completed before cancellation could be tested")
            canceled = handle.cancel_goal_async()
            node.wait(canceled.done, 5, "Nav2 cancel response")
            require(any(bytes(g.goal_id.uuid) == bytes(handle.goal_id.uuid) for g in canceled.result().goals_canceling),
                    "Nav2 did not acknowledge cancellation of this goal")
            node.wait(result.done, 10, "Nav2 canceled terminal status")
            require(result.result().status == GoalStatus.STATUS_CANCELED, "Nav2 did not terminate CANCELED")
            node.settled()
            report["checks"]["cancellation"] = dict(
                state="CANCELED", **node.evidence(allow_rotation=True))
        if args.explore:
            activation = node.rpc("explore", operation="activate")
            before = check_map(node, args)
            node.begin_motion()
            exploration = observe_exploration(node, args)
            node.settled()
            after = check_map(node, args, before, node.samples[0][0])
            require(after["known_area_m2"] > before["known_area_m2"], "exploration produced no observed map growth")
            report["checks"]["exploration"] = dict(exploration, activation=activation,
                                                    before=before, after=after, **node.evidence())
        if args.semantic:
            report["checks"]["semantic"] = check_semantic(node, args)
    finally:
        try:
            errors = node.cleanup()
            report["cleanup_errors"] = errors
            require(not errors, f"cleanup failed: {errors}")
        finally:
            node.destroy_node()
            rclpy.shutdown()


def observe_exploration(node, args):
    """Observe one run for a bounded duration, then verify intentional cancellation or early success.

The caller independently verifies body movement, upright posture, stopping and map growth.
Retain unfinished runs for the outer finally cleanup; never accept FAILED or TIMEOUT.
"""
    started = node.rpc("explore", EXPLORE, {"area_hint": "", "timeout_s": args.explore_timeout,
                       "max_speed_m_s": args.explore_speed}, safety=False)
    require(started.get("accepted") is True and started.get("run_id"), f"exploration rejected: {started}")
    run_id = started["run_id"]
    run = ("explore", EXPLORE, run_id)
    node.runs.append(run)
    began = time.monotonic()
    deadline = began + args.explore_duration
    observations = []

    def poll():
        """Validate status for this run and retain timestamped progress in the JSON report."""
        status = node.rpc("explore", EXPLORE + "/status", {"run_id": run_id})
        require(status.get("known") is True, f"unknown exploration run {run_id}")
        require(status.get("state") in TERMINAL | {"PENDING", "RUNNING", "PAUSED"},
                f"invalid exploration state: {status}")
        observations.append({"observed_s": round(time.monotonic() - began, 3), **status})
        return status

    status = poll()
    while status["state"] not in TERMINAL and time.monotonic() < deadline:
        node.pause(min(0.3, max(0.001, deadline - time.monotonic())))
        status = poll()
    cancel_requested = status["state"] not in TERMINAL
    if cancel_requested:
        canceled = node.rpc("explore", EXPLORE + "/cancel", {"run_id": run_id}, safety=False)
        require(canceled.get("ok") is True, f"exploration cancellation rejected: {canceled}")
        cancel_deadline = time.monotonic() + 20
        status = poll()
        while status["state"] not in TERMINAL and time.monotonic() < cancel_deadline:
            node.pause(0.3)
            status = poll()
        require(status["state"] in TERMINAL, "exploration did not terminate after cancellation")
    node.runs.remove(run)
    allowed = {"CANCELED", "SUCCEEDED"} if cancel_requested else {"SUCCEEDED"}
    require(status["state"] in allowed, f"exploration did not succeed or intentionally cancel: {status}")
    return {"run_id": run_id, "duration_s": args.explore_duration,
            "observed_s": round(time.monotonic() - began, 3), "cancel_requested": cancel_requested,
            "status": status, "observations": observations}


def check_sensors(node, args, report, ros_time):
    """Require live synchronized sensor streams, calibrated optical frames, 12 joints and matching TF."""
    import numpy as np
    required = {"scan", "cloud", "imu", "rgb", "depth", "info", "odom", "joints", "clock"}
    node.wait(lambda: required <= node.messages.keys() and all(len(node.history[k]) >= 3 for k in required),
              args.sensor_timeout, "live sensor suite")
    frames = {"scan": "mid360_link", "cloud": "mid360_link", "imu": "imu_link",
              "rgb": "front_camera_optical_frame", "depth": "front_camera_optical_frame",
              "info": "front_camera_optical_frame", "odom": "odom", "joints": "base_link"}
    times = []
    for key, frame in frames.items():
        message = node.messages[key]
        history = [stamp(m) for m in node.history[key]]
        require(history[-1] > history[0] > 0 and all(b >= a for a, b in zip(history, history[1:])), f"{key} stale/regressing timestamps")
        require(time.monotonic() - node.received[key] <= args.max_age, f"{key} receipt stale")
        require(message.header.frame_id == frame, f"{key} frame {message.header.frame_id!r} != {frame}")
        times.append(stamp(message))
    clock = node.messages["clock"].clock
    now = clock.sec + clock.nanosec * 1e-9
    require(max(times) - min(times) <= args.max_skew, "sensor timestamps disagree")
    require(all(-args.max_skew <= now - value <= args.max_age for value in times), "sensor timestamps disagree with /clock")
    require(time.monotonic() - node.received["clock"] <= args.max_age, "/clock stale")
    rgb, depth, info = (node.messages[k] for k in ("rgb", "depth", "info"))
    common = set(stamp(m) for m in node.history["rgb"]) & set(stamp(m) for m in node.history["depth"]) & set(stamp(m) for m in node.history["info"])
    require(bool(common), "RGB/depth/calibration have no matching acquisition timestamp")
    require((rgb.width, rgb.height) == (depth.width, depth.height) == (info.width, info.height), "RGBD/calibration dimensions disagree")
    require(np.isfinite(info.k).all() and info.k[0] > 0 and info.k[4] > 0 and info.k[8] == 1,
            "invalid camera intrinsics")
    require(0 <= info.k[2] < info.width and 0 <= info.k[5] < info.height, "principal point outside image")
    scan = node.messages["scan"]
    ranges = np.asarray(scan.ranges)
    require(len(ranges) >= 360 and np.count_nonzero(np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)) >= 10, "lidar scan has no valid returns")
    joints = node.messages["joints"]
    require(len(joints.name) == len(set(joints.name)) == len(joints.position) == 12,
            "expected exactly 12 uniquely named leg joints")
    require(np.isfinite(joints.position).all() and len(joints.velocity) in (0, 12), "invalid joint state")
    require(np.isfinite(joints.velocity).all(), "nonfinite joint velocity")
    imu = node.messages["imu"]
    accel = np.array([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])
    gyro = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
    require(np.isfinite(gyro).all() and np.isfinite(accel).all() and 1 < np.linalg.norm(accel) < 40, "IMU has invalid acceleration/gyro")
    odom = node.messages["odom"]
    require(odom.child_frame_id == "base_link", "odometry child frame mismatch")
    at = ros_time.from_msg(odom.header.stamp)
    transformed = node.pose("odom", at)
    pose = odom.pose.pose
    yaw, tilt = quaternion(pose.orientation)
    require(tilt <= args.max_tilt and pose.position.z >= args.min_height, "body is not upright")
    require(np.linalg.norm(transformed[:2] - [pose.position.x, pose.position.y]) < 0.02,
            "odometry position disagrees with TF at its timestamp")
    require(abs(math.atan2(math.sin(transformed[2]-yaw), math.cos(transformed[2]-yaw))) < 0.02,
            "odometry orientation disagrees with TF")
    for key in ("cloud", "imu", "rgb"):
        message = node.messages[key]
        node.wait(lambda: node.tf.can_transform("base_link", message.header.frame_id, ros_time.from_msg(message.header.stamp)),
                  5, f"{key} sensor-time TF")
    optical = node.tf.lookup_transform("base_link", rgb.header.frame_id, ros_time.from_msg(rgb.header.stamp)).transform.rotation
    quaternion(optical)
    # Optical +Z must point forward, +X right and +Y down in the body frame.
    x, y, z, w = optical.x, optical.y, optical.z, optical.w
    require(2*(x*z + w*y) > 0.8 and 2*(x*y + w*z) < -0.8 and 2*(y*z + w*x) < -0.8,
            "camera TF is not forward-facing optical axes")
    report["checks"]["sensors"] = {"frames": frames, "stamp_skew_s": max(times)-min(times),
        "front_rgb": image_stats(rgb), "front_depth": image_stats(depth),
        "cloud": cloud_stats(node.messages["cloud"]), "leg_joints": list(joints.name)}


def map_timestamps(grid, cloud, sim_now, epoch_start):
    """Accept retained snapshots within the current simulation/map epoch, regardless of age."""
    loaded = grid.info.map_load_time.sec + grid.info.map_load_time.nanosec * 1e-9
    require(0 <= loaded <= sim_now, "invalid map load timestamp")
    lower = max(epoch_start, loaded)
    times = {"map_stamp_s": stamp(grid), "cloud_stamp_s": stamp(cloud)}
    require(all(lower <= value <= sim_now for value in times.values()),
            f"map/cloud timestamps outside current map epoch [{lower}, {sim_now}]: {times}")
    return dict(times, epoch_start_s=lower)


def map_refreshed(node, before, moved_since):
    """Require newer grid and fused-cloud acquisitions after movement, not a retained replay."""
    grid, cloud = node.messages["map"], node.messages["map_cloud"]
    return (stamp(grid) > max(before["map_stamp_s"], moved_since)
            and stamp(cloud) > max(before["cloud_stamp_s"], moved_since))


def check_map(node, args, before=None, moved_since=None):
    """Validate retained geometry; require post-motion refresh only when a baseline is supplied."""
    import numpy as np
    node.wait(lambda: {"map", "map_cloud"} <= node.messages.keys(), args.sensor_timeout, "occupancy and fused cloud")
    if before is not None:
        node.wait(lambda: map_refreshed(node, before, moved_since), args.sensor_timeout,
                  "occupancy and fused-cloud refresh after movement")
    grid, cloud = node.messages["map"], node.messages["map_cloud"]
    require(grid.header.frame_id == cloud.header.frame_id == "map", "map/cloud frame disagreement")
    clock = node.messages["clock"].clock
    timestamps = map_timestamps(grid, cloud, clock.sec + clock.nanosec*1e-9, args.map_epoch_start)
    origin = grid.info.origin
    require(all(math.isfinite(v) for v in (origin.position.x, origin.position.y, origin.position.z)),
            "invalid map origin")
    quaternion(origin.orientation)
    require(grid.info.resolution > 0 and math.isfinite(grid.info.resolution), "invalid map resolution")
    values = np.asarray(grid.data)
    require(grid.info.width > 0 and grid.info.height > 0 and values.size == grid.info.width*grid.info.height,
            "occupancy dimensions mismatch")
    require(np.all((values >= -1) & (values <= 100)), "invalid occupancy values")
    free, occupied = int(np.count_nonzero(values == 0)), int(np.count_nonzero(values >= 50))
    require(free > 0 and occupied > 0, "map lacks observed free/occupied cells")
    node.pose()
    return {"free": free, "occupied": occupied, **timestamps,
            "known_area_m2": float(np.count_nonzero(values >= 0)*grid.info.resolution**2),
            "cloud": cloud_stats(cloud)}


def scene_last_seen(obj):
    """Read semantic_map Object/SceneGraphNode float64 Unix seconds without coercing malformed data."""
    value = obj.get("last_seen_unix")
    require(type(value) in (int, float) and math.isfinite(value) and value > 0,
            "Scene last_seen_unix must be positive finite numeric Unix seconds")
    return float(value)


def check_semantic(node, args):
    """Select a freshly observed physical object, request goal_near, and navigate via Robonix."""
    import numpy as np
    deadline = time.monotonic() + args.semantic_timeout
    observed = {}
    selected = None
    while time.monotonic() < deadline and selected is None:
        objects = node.rpc("scene", SCENE + "list_objects").get("objects", [])
        for obj in objects:
            object_id = obj.get("id", "")
            if (not object_id.startswith("scene.object.") or obj.get("label") == "robot"
                    or (args.object_id and object_id != args.object_id)):
                continue
            seen = scene_last_seen(obj)
            prior = observed.setdefault(object_id, seen)
            if seen <= prior or not 0 <= time.time() - seen <= args.semantic_timeout:
                continue
            context = node.rpc("scene", SCENE + "get_object_context", {"object_id": object_id})
            observation = context.get("object", {})
            require(observation.get("object_id") == object_id, "Scene context returned a different object")
            count = observation.get("observation_count")
            require(type(count) is int and count > 0, "Scene object lacks observation evidence")
            require(scene_last_seen(observation) >= seen, "Scene context predates the observed object")
            goal = node.rpc("scene", SCENE + "goal_near", {"object_id": object_id})
            if goal.get("reachable") is not True:
                continue
            target = [goal[key] for key in ("x", "y", "yaw")]
            require(all(math.isfinite(v) for v in target), "Scene goal has nonfinite coordinates")
            if np.linalg.norm(node.pose()[:2] - target[:2]) <= args.position_tolerance + args.min_motion:
                continue
            selected = (object_id, target, context)
            break
        if selected is None:
            node.pause(0.5)
    require(selected is not None, "no freshly observed reachable Scene object with a nontrivial goal")
    object_id, target, context = selected
    before_map = check_map(node, args)
    node.begin_motion()
    goal = {"header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
            "pose": {"position": {"x": target[0], "y": target[1], "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(target[2]/2), "w": math.cos(target[2]/2)}}}
    status = node.task("nav2", NAV, {"goal": goal}, args.navigation_timeout)
    arrival = node.arrival(target)
    after_map = check_map(node, args, before_map, node.samples[0][0])
    return dict(object_id=object_id, observation=context["object"], status=status,
                map_after=after_map, **arrival)


def main():
    """Emit exactly one JSON result on success, missing dependencies, timeout or interruption."""
    if sys.argv[1:] == ["--rpc-worker"]:
        print(json.dumps(rpc_worker(json.load(sys.stdin))))
        return 0
    args = parser().parse_args()
    report = {"ok": False, "checks": {}, "skipped": []}
    start = time.monotonic()
    try:
        for key, value in vars(args).items():
            if isinstance(value, float):
                valid = key == "yaw" or (value >= 0 if key == "map_epoch_start" else value > 0)
                require(math.isfinite(value) and valid, f"invalid --{key.replace('_', '-')}")
        require(not args.navigate or all(math.isfinite(v) for v in args.navigate), "nonfinite navigation target")
        require(args.relative is None or args.relative > 0, "--relative must be positive")
        require(not args.explore or args.explore_duration < args.explore_timeout,
                "--explore-duration must be less than --explore-timeout")
        os.environ["ROBONIX_ATLAS"] = args.atlas
        if args.container:
            forwarded = sys.argv[1:]
            index = forwarded.index("--container")
            forwarded = forwarded[:index] + forwarded[index+2:]
            budget = int(600 + args.explore_timeout + 2*args.navigation_timeout + args.semantic_timeout)
            command = ["docker", "exec", "-e", f"ROBONIX_ATLAS={args.atlas}", args.container,
                       "bash", "-lc", "source /opt/ros/humble/setup.bash && exec timeout --signal=INT --kill-after=60s \"$@\"",
                       "_", str(budget), "python3", "/workspace/sim/tests/ros_acceptance.py", *forwarded]
            try:
                completed = subprocess.run(command, text=True, capture_output=True, timeout=budget + 90)
            except (KeyboardInterrupt, subprocess.TimeoutExpired):
                stopped = subprocess.run(["docker", "exec", args.container, "pkill", "-INT", "-f",
                                          "^python3 /workspace/sim/tests/ros_acceptance.py"],
                                         text=True, capture_output=True, timeout=10)
                report["container_stop_signal_exit"] = stopped.returncode
                raise
            require(bool(completed.stdout.strip()), f"container runner failed: {completed.stderr[-1500:]}")
            report = json.loads(completed.stdout)
            require(completed.returncode == 0 and report.get("ok") is True, report.get("error", "container acceptance failed"))
        else:
            for name, enabled in (("motion", args.motion), ("relative", args.relative),
                                  ("navigation", args.navigate),
                                  ("cancellation", args.navigate), ("exploration", args.explore),
                                  ("semantic", args.semantic), ("stack", args.require_stack or args.semantic or args.explore),
                                  ("map", args.require_map or args.navigate or args.semantic or args.explore)):
                if not enabled:
                    report["skipped"].append(name)
            runtime(args, report)
            report["ok"] = True
    except (Exception, KeyboardInterrupt) as exc:
        report["ok"] = False
        report["error"] = f"{type(exc).__name__}: {exc}"
    report["elapsed_s"] = round(time.monotonic() - start, 3)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    raise SystemExit(main())
