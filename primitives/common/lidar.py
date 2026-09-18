"""MID-360 planar and 3-D lidar capability provider."""
from __future__ import annotations

import threading

from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, timeout_value, topic_value

provider = Primitive(
    id=provider_id("mid360_lidar"), namespace="robonix/primitive/lidar",
    pkg_root=package_root())
lock = threading.Lock()
latest_scan = None

import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import LaserScan  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def receive_scan(message):
    """Cache the newest planar scan for the snapshot capability."""
    global latest_scan
    with lock:
        latest_scan = message


@provider.mcp("robonix/primitive/lidar/snapshot")
def snapshot(_request: Empty) -> LaserScan:
    """Return the latest ordered planar scan for inspection and diagnostics."""
    with lock:
        message = latest_scan
    if message is None:
        raise RuntimeError("no lidar scan received")
    header = std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
        frame_id=message.header.frame_id)
    return LaserScan(
        header=header, angle_min=message.angle_min, angle_max=message.angle_max,
        angle_increment=message.angle_increment, time_increment=message.time_increment,
        scan_time=message.scan_time, range_min=message.range_min, range_max=message.range_max,
        ranges=list(message.ranges), intensities=list(message.intensities))


@provider.on_init
def initialize(config):
    """Declare both lidar streams and wait for a simulator-produced scan."""
    try:
        scan_topic = topic_value(config, "scan_topic", "/scan")
        cloud_topic = topic_value(config, "cloud_topic", "/mid360/points")
        sentinel_timeout_s = timeout_value(config)
    except ValueError as error:
        return Err(str(error))
    provider.create_subscription(
        "robonix/primitive/lidar/lidar", topic=scan_topic, msg_type="LaserScan",
        callback=receive_scan, qos="best_effort", declare=False)
    if not provider.wait_for_topic(scan_topic, "LaserScan", sentinel_timeout_s):
        return Err(f"no LaserScan received on {scan_topic}; start the simulator runtime first")
    provider.declare_ros2_topic("robonix/primitive/lidar/lidar", scan_topic, qos="best_effort")
    provider.declare_ros2_topic("robonix/primitive/lidar/lidar3d", cloud_topic, qos="best_effort")
    return Ok()


@provider.on_shutdown
def shutdown():
    """Complete lifecycle shutdown; bridge owns the actual sensor publishers."""
    return Ok()


if __name__ == "__main__":
    provider.run()
