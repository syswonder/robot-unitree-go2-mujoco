"""Go2 chassis primitive backed by simulated ROS topics."""
from __future__ import annotations

import json
import math
import os
import threading
import time

from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, topic_value

provider = Primitive(
    id=provider_id("go2_chassis"), namespace="robonix/primitive/chassis",
    pkg_root=package_root())
cmd_vel_pub = None
odom_subscription = None
motion_lock = threading.Lock()
stopped = threading.Event()
odom_condition = threading.Condition()
odom_state = None

CONTROL_PERIOD_S = 0.1
ODOM_MAX_AGE_S = 1.0

import chassis_mcp  # noqa: E402
import chassis_pb2  # noqa: E402
import std_msgs_mcp  # noqa: E402
import std_msgs_pb2  # noqa: E402


def _yaw(orientation):
    """Return planar yaw from a ROS quaternion without depending on tf helpers."""
    return math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))


def _wrap(angle):
    """Wrap a signed angle to the shortest interval around zero."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _on_odom(message):
    """Store one finite pose/velocity sample and wake a waiting relative move."""
    global odom_state
    pose = message.pose.pose
    twist = message.twist.twist
    values = (pose.position.x, pose.position.y, _yaw(pose.orientation),
              twist.linear.x, twist.angular.z)
    if not all(math.isfinite(float(value)) for value in values):
        return
    with odom_condition:
        odom_state = (*map(float, values), time.monotonic())
        odom_condition.notify_all()


def _read_odom(timeout_s=2.0):
    """Wait for a recent odometry sample, rejecting a disconnected simulator."""
    deadline = time.monotonic() + timeout_s
    with odom_condition:
        while True:
            if stopped.is_set():
                raise RuntimeError("chassis stopped during move")
            if odom_state is not None and time.monotonic() - odom_state[5] <= ODOM_MAX_AGE_S:
                return odom_state
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("fresh odometry is unavailable")
            odom_condition.wait(min(CONTROL_PERIOD_S, remaining))


def _publish_and_wait(velocity):
    """Refresh the velocity watchdog for one control period or abort on shutdown."""
    cmd_vel_pub.publish(velocity)
    if stopped.wait(CONTROL_PERIOD_S):
        raise RuntimeError("chassis stopped during move")


def _distance_move(target_m, speed_m_s, velocity_type):
    """Track signed displacement along the initial body heading using odometry."""
    start_x, start_y, start_yaw, *_ = _read_odom()
    configured_tolerance = float(os.environ.get("SIM_DISTANCE_TOLERANCE_M", "0.03"))
    if not math.isfinite(configured_tolerance) or not 0.01 <= configured_tolerance <= 0.15:
        raise RuntimeError("SIM_DISTANCE_TOLERANCE_M must be in [0.01, 0.15] m")
    tolerance = min(configured_tolerance, max(0.015, abs(target_m) * 0.15))
    hard_timeout = float(os.environ.get(
        "SIM_RELATIVE_TIMEOUT_S", str(max(60.0, abs(target_m) / speed_m_s * 30 + 20))))
    stall_timeout = float(os.environ.get("SIM_RELATIVE_STALL_TIMEOUT_S", "20"))
    if not (math.isfinite(hard_timeout) and hard_timeout > 0 and
            math.isfinite(stall_timeout) and 5 <= stall_timeout <= hard_timeout):
        raise RuntimeError("relative motion timeouts are invalid")
    deadline = time.monotonic() + hard_timeout
    stall_deadline = time.monotonic() + stall_timeout
    best_error = abs(target_m)
    settled = 0
    progress = 0.0
    while time.monotonic() < deadline:
        x, y, yaw, linear_x, _, _ = _read_odom()
        progress = (x - start_x) * math.cos(start_yaw) + (y - start_y) * math.sin(start_yaw)
        remaining = target_m - progress
        if abs(remaining) < best_error - 0.01:
            best_error = abs(remaining)
            stall_deadline = time.monotonic() + stall_timeout
        if time.monotonic() >= stall_deadline:
            raise RuntimeError(
                f"relative distance stalled: requested={target_m:.3f}m actual={progress:.3f}m")
        if abs(remaining) <= tolerance and abs(linear_x) <= 0.08:
            settled += 1
            if settled >= 2:
                return {"requested_m": target_m, "actual_m": progress,
                        "error_m": remaining}
        else:
            settled = 0
        velocity = velocity_type()
        if abs(remaining) > tolerance:
            velocity.linear.x = math.copysign(
                min(speed_m_s, max(0.10, abs(remaining) * 0.8)), remaining)
            velocity.angular.z = max(-0.30, min(0.30, 1.5 * _wrap(start_yaw - yaw)))
        _publish_and_wait(velocity)
    raise RuntimeError(
        f"relative distance timed out: requested={target_m:.3f}m actual={progress:.3f}m")


def _rotation_move(target_rad, speed_rad_s, velocity_type):
    """Track signed accumulated yaw, including rotations that cross +/-pi."""
    _, _, previous_yaw, _, _, _ = _read_odom()
    tolerance = float(os.environ.get("SIM_ANGLE_TOLERANCE_DEG", "4"))
    if not math.isfinite(tolerance) or not 1 <= tolerance <= 12:
        raise RuntimeError("SIM_ANGLE_TOLERANCE_DEG must be in [1, 12] degrees")
    tolerance = math.radians(tolerance)
    hard_timeout = float(os.environ.get(
        "SIM_RELATIVE_TIMEOUT_S", str(max(60.0, abs(target_rad) / speed_rad_s * 30 + 20))))
    stall_timeout = float(os.environ.get("SIM_RELATIVE_STALL_TIMEOUT_S", "20"))
    if not (math.isfinite(hard_timeout) and hard_timeout > 0 and
            math.isfinite(stall_timeout) and 5 <= stall_timeout <= hard_timeout):
        raise RuntimeError("relative motion timeouts are invalid")
    deadline = time.monotonic() + hard_timeout
    stall_deadline = time.monotonic() + stall_timeout
    best_error = abs(target_rad)
    accumulated = 0.0
    settled = 0
    while time.monotonic() < deadline:
        _, _, yaw, _, angular_z, _ = _read_odom()
        accumulated += _wrap(yaw - previous_yaw)
        previous_yaw = yaw
        remaining = target_rad - accumulated
        if abs(remaining) < best_error - math.radians(1):
            best_error = abs(remaining)
            stall_deadline = time.monotonic() + stall_timeout
        if time.monotonic() >= stall_deadline:
            raise RuntimeError(
                "relative rotation stalled: "
                f"requested={math.degrees(target_rad):.1f}deg "
                f"actual={math.degrees(accumulated):.1f}deg")
        if abs(remaining) <= tolerance and abs(angular_z) <= 0.12:
            settled += 1
            if settled >= 2:
                return {"requested_deg": math.degrees(target_rad),
                        "actual_deg": math.degrees(accumulated),
                        "error_deg": math.degrees(remaining)}
        else:
            settled = 0
        velocity = velocity_type()
        if abs(remaining) > tolerance:
            velocity.angular.z = math.copysign(
                min(speed_rad_s, max(0.18, abs(remaining) * 1.2)), remaining)
        _publish_and_wait(velocity)
    raise RuntimeError(
        "relative rotation timed out: "
        f"requested={math.degrees(target_rad):.1f}deg actual={math.degrees(accumulated):.1f}deg")


def _timed_move(velocity, duration):
    """Publish an explicit velocity command for its bounded requested duration."""
    for _ in range(max(1, math.ceil(duration / CONTROL_PERIOD_S))):
        _publish_and_wait(velocity)
    return {"duration_sec": duration}


def _execute_move(command):
    """Execute measured relative motion or a bounded velocity command, then stop."""
    if cmd_vel_pub is None or stopped.is_set():
        raise RuntimeError("chassis is not active")
    from geometry_msgs.msg import Twist
    linear_speed = float(os.environ.get("SIM_LINEAR_SPEED", "0.18"))
    angular_speed = float(os.environ.get("SIM_ANGULAR_SPEED", "0.55"))
    if not (math.isfinite(linear_speed) and 0 < linear_speed <= 0.25):
        raise RuntimeError("SIM_LINEAR_SPEED must be in (0, 0.25] m/s")
    if not (math.isfinite(angular_speed) and 0 < angular_speed <= 0.55):
        raise RuntimeError("SIM_ANGULAR_SPEED must be in (0, 0.55] rad/s")
    forward_m = float(getattr(command, "forward_m", 0.0))
    rotate_deg = float(getattr(command, "rotate_deg", 0.0))
    if not all(math.isfinite(float(value)) for value in (
            forward_m, rotate_deg, command.linear_x, command.linear_y,
            command.angular_z, command.duration_sec)):
        raise RuntimeError("motion values must be finite")
    if not motion_lock.acquire(blocking=False):
        raise RuntimeError("another direct move is running")
    started = time.monotonic()
    try:
        try:
            if forward_m:
                details = _distance_move(forward_m, linear_speed, Twist)
            elif rotate_deg:
                details = _rotation_move(math.radians(rotate_deg), angular_speed, Twist)
            else:
                velocity = Twist()
                velocity.linear.x = float(command.linear_x)
                velocity.linear.y = float(command.linear_y)
                velocity.angular.z = float(command.angular_z)
                magnitude = math.hypot(velocity.linear.x, velocity.linear.y)
                if magnitude > 0.25:
                    velocity.linear.x *= 0.25 / magnitude
                    velocity.linear.y *= 0.25 / magnitude
                velocity.angular.z = max(-0.55, min(0.55, velocity.angular.z))
                duration = max(0.05, float(command.duration_sec or 1.0))
                if not math.isfinite(duration) or duration <= 0:
                    raise RuntimeError("motion duration must be finite and positive")
                details = _timed_move(velocity, duration)
        finally:
            cmd_vel_pub.publish(Twist())
    finally:
        motion_lock.release()
    details.update(status="done", elapsed_sec=time.monotonic() - started)
    return details


@provider.grpc(
    "robonix/primitive/chassis/move",
    description=("Execute direct base-relative forward/backward distance or yaw rotation; "
                 "use navigation only for absolute map-frame goals."))
def move(request):
    """Expose the measured direct-move implementation over gRPC."""
    details = _execute_move(request.command)
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps(details)))


@provider.mcp(
    "robonix/primitive/chassis/move",
    description=("Use for direct base-relative motion requested as forward/backward metres or "
                 "left/right yaw degrees. Do not convert relative motion into a map goal. Use "
                 "navigation only for an absolute map-frame pose or collision-aware destination."))
def move_mcp(request: chassis_mcp.ExecuteMoveCommand_Request) -> chassis_mcp.ExecuteMoveCommand_Response:
    """Expose the same measured direct-move implementation as a Pilot tool."""
    details = _execute_move(request.command)
    return chassis_mcp.ExecuteMoveCommand_Response(
        status=std_msgs_mcp.String(data=json.dumps(details)))


@provider.on_init
def initialize(config):
    """Bind capability metadata and subscribe to measured bridge odometry."""
    global cmd_vel_pub, odom_subscription
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    try:
        odom_topic = topic_value(config, "odom_topic", "/odom")
        command_topic = topic_value(config, "command_topic", "/cmd_vel")
    except ValueError as error:
        return Err(str(error))
    cmd_vel_pub = provider.create_publisher(
        "robonix/primitive/chassis/twist_in", topic=command_topic,
        msg_type=Twist, qos="reliable", declare=False)
    odom_subscription = provider.create_subscription(
        "robonix/primitive/chassis/odom", topic=odom_topic,
        msg_type=Odometry, callback=_on_odom, qos="best_effort", declare=False)
    provider.declare_ros2_topic("robonix/primitive/chassis/twist_in", command_topic, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/chassis/odom", odom_topic, qos="best_effort")
    return Ok()


@provider.on_activate
def activate():
    """Enable direct moves after initialization or reactivation."""
    stopped.clear()
    return Ok()


@provider.on_deactivate
def deactivate():
    """Interrupt direct motion and serialize the final stop after its last command."""
    stopped.set()
    with odom_condition:
        odom_condition.notify_all()
    with motion_lock:
        if cmd_vel_pub is not None:
            from geometry_msgs.msg import Twist
            cmd_vel_pub.publish(Twist())
    return Ok()


@provider.on_shutdown
def shutdown():
    """Stop the mobile base before unregistering its provider."""
    return deactivate()


if __name__ == "__main__":
    provider.run()
