"""RGB or RGB-D camera capability provider with MCP snapshots."""
from __future__ import annotations

import math
import threading
import time
from io import BytesIO

import numpy as np
from PIL import Image as PillowImage
from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, timeout_value, topic_value

provider = Primitive(
    id=provider_id("front_camera"), namespace="robonix/primitive/camera",
    pkg_root=package_root())
lock = threading.Lock()
latest_rgb = None
latest_depth = None
camera_config = {}

import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def image_to_jpeg(message, depth=False):
    """Convert bridge RGB/float-depth images to compact MCP JPEG payloads."""
    if depth:
        raw = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width)
        valid = np.isfinite(raw)
        if valid.any():
            near, far = np.percentile(raw[valid], [2, 98])
            array = np.where(valid, np.clip((raw - near) / max(far - near, 1e-6), 0, 1) * 255, 0).astype(np.uint8)
        else:
            array = np.zeros((message.height, message.width), dtype=np.uint8)
    else:
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
    buffer = BytesIO()
    PillowImage.fromarray(array).save(buffer, format="JPEG", quality=88)
    return buffer.getvalue(), message.width, message.height, message.header


def receive_rgb(message):
    """Cache a compressed RGB frame for snapshot calls."""
    global latest_rgb
    with lock:
        latest_rgb = image_to_jpeg(message)


def receive_depth(message):
    """Cache a normalized depth frame for depth snapshot calls."""
    global latest_depth
    with lock:
        latest_depth = image_to_jpeg(message, depth=True)


def as_mcp(cached) -> Image:
    """Wrap cached JPEG bytes as the camera contract's Image message."""
    if cached is None:
        raise RuntimeError("camera has not produced a frame")
    data, width, height, source_header = cached
    header = std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(
            sec=source_header.stamp.sec, nanosec=source_header.stamp.nanosec),
        frame_id=source_header.frame_id)
    return Image(header=header, height=height, width=width, encoding="jpeg", is_bigendian=0, step=len(data), data=data)


@provider.mcp("robonix/primitive/camera/snapshot")
def snapshot(_request: Empty) -> Image:
    """Return the most recent RGB image as JPEG."""
    with lock:
        return as_mcp(latest_rgb)


@provider.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(_request: Empty) -> Image:
    """Return the most recent normalized depth image as JPEG."""
    with lock:
        return as_mcp(latest_depth)


@provider.on_init
def initialize(config):
    """Validate all four required RGB-D inputs without opening ROS resources."""
    global camera_config
    config = config or {}
    try:
        rgb_topic = topic_value(config, "rgb_topic", "/front/rgb")
        depth_topic = topic_value(config, "depth_topic", "/front/depth")
        info_topic = topic_value(config, "info_topic", "/front/camera_info")
        extrinsics_topic = topic_value(config, "extrinsics_topic", "/front/extrinsics")
        sentinel_timeout_s = timeout_value(config)
    except ValueError as error:
        return Err(str(error))
    camera_config = dict(rgb=rgb_topic, depth=depth_topic, info=info_topic,
                         extrinsics=extrinsics_topic, timeout=sentinel_timeout_s)
    return Ok()


@provider.on_activate
def activate():
    """Require coherent RGB-D, calibration and a latched mount before declaring streams."""
    global latest_rgb, latest_depth
    with lock:
        latest_rgb = latest_depth = None
    ready = {name: threading.Event() for name in ("rgb", "depth", "info", "extrinsics")}
    calibration_size = []
    frame = "front_camera_optical_frame"

    def rgb_received(message):
        """Cache only front optical RGB images with the bridge's RGB8 encoding."""
        if message.header.frame_id == frame and message.encoding == "rgb8":
            receive_rgb(message)
            ready["rgb"].set()

    def depth_received(message):
        """Require a metric depth image with finite positive returns."""
        if message.header.frame_id != frame or message.encoding != "32FC1":
            return
        values = np.frombuffer(message.data, dtype=np.float32)
        if not np.any(np.isfinite(values) & (values > 0)):
            return
        receive_depth(message)
        ready["depth"].set()

    def info_received(message):
        """Require calibrated front optical intrinsics for Scene backprojection."""
        if (message.header.frame_id == frame and message.width > 0 and message.height > 0
                and all(math.isfinite(value) for value in message.k)
                and message.k[0] > 0 and message.k[4] > 0):
            calibration_size[:] = [message.width, message.height]
            ready["info"].set()

    def extrinsics_received(message):
        """Require the base-to-camera transform, including a valid unit quaternion."""
        rotation = message.transform.rotation
        translation = message.transform.translation
        values = (rotation.x, rotation.y, rotation.z, rotation.w)
        if (message.header.frame_id == "base_link" and message.child_frame_id == frame
                and all(math.isfinite(value) for value in (*values, translation.x,
                                                           translation.y, translation.z))
                and abs(sum(value * value for value in values) - 1.0) < 1e-3):
            ready["extrinsics"].set()

    provider.create_subscription(
        "robonix/primitive/camera/rgb", topic=camera_config["rgb"], msg_type="Image",
        callback=rgb_received, qos="best_effort", declare=False)
    provider.create_subscription(
        "robonix/primitive/camera/depth", topic=camera_config["depth"], msg_type="Image",
        callback=depth_received, qos="best_effort", declare=False)
    provider.create_subscription(
        "robonix/primitive/camera/intrinsics", topic=camera_config["info"], msg_type="CameraInfo",
        callback=info_received, qos="latched", declare=False)
    provider.create_subscription(
        "robonix/primitive/camera/extrinsics", topic=camera_config["extrinsics"],
        msg_type="TransformStamped", callback=extrinsics_received, qos="latched", declare=False)
    deadline = time.monotonic() + camera_config["timeout"]
    for name, event in ready.items():
        if not event.wait(max(0, deadline - time.monotonic())):
            return Err(f"no valid {name} on {camera_config[name]}; Scene requires complete front RGB-D")
    with lock:
        if latest_rgb[1:3] != latest_depth[1:3]:
            return Err("front RGB and depth dimensions must match")
        if tuple(calibration_size) != latest_rgb[1:3]:
            return Err("front camera intrinsics dimensions must match RGB-D")
    for contract, name, qos in (
            ("rgb", "rgb", "best_effort"), ("depth", "depth", "best_effort"),
            ("intrinsics", "info", "latched"), ("extrinsics", "extrinsics", "latched")):
        provider.declare_ros2_topic(f"robonix/primitive/camera/{contract}", camera_config[name], qos=qos)
    return Ok()


@provider.on_shutdown
def shutdown():
    """Complete lifecycle shutdown."""
    return Ok()


if __name__ == "__main__":
    provider.run()
