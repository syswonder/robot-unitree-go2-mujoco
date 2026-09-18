#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import base64
import ctypes
import json
import os
import queue
import signal
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import websockets

from sim.native.controller import Go2Controller, COMMAND_LIMITS
from sim.native.scene_builder import NativeSceneBuilder
from sim.native.sensors import NativeSensorSuite


_MUJOCO_SHORTCUT_STORAGE = ctypes.create_string_buffer(b"")


def disable_mujoco_viewer_shortcuts() -> None:
    """Remove MuJoCo rendering hotkeys before Simulate builds its native UI."""
    library_path = Path(mujoco.__file__).with_name(
        f"libmujoco.so.{mujoco.__version__}")
    library = ctypes.CDLL(str(library_path))
    empty = ctypes.cast(_MUJOCO_SHORTCUT_STORAGE, ctypes.c_char_p)
    for symbol, rows in (("mjVISSTRING", len(mujoco.mjVISSTRING)),
                         ("mjRNDSTRING", len(mujoco.mjRNDSTRING))):
        table = ((ctypes.c_char_p * 3) * rows).in_dll(library, symbol)
        for index in range(rows):
            table[index][2] = empty


def encoded(array: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")


def configure_viewer_camera(model: mujoco.MjModel, data: mujoco.MjData, camera) -> None:
    """Initialize a robot-centered free camera that retains user pan controls."""
    base_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_body < 0:
        raise ValueError("Go2 model is missing base_link")
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.trackbodyid = -1
    camera.lookat[:] = data.xpos[base_body]
    camera.distance = 1.4
    camera.azimuth = 135.0
    camera.elevation = -45.0


class NativeRuntime:
    def __init__(self, root: Path, environment: str, robot: str, viewer_enabled: bool,
                 developer_mode: bool = False) -> None:
        """Build the Go2 scene and initialize physics, sensors and viewer input."""
        self.builder = NativeSceneBuilder(root)
        self.scene_path, self.environment, self.robot = self.builder.build(environment, robot)
        try:
            self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
            self.data = mujoco.MjData(self.model)
            self.controller = Go2Controller(
                self.model, self.data, root / "assets/robots/go2/policy/moe_rough")
            self.sensors = NativeSensorSuite(self.model, self.data, self.robot["sensors"])
        except Exception:
            self.builder.cleanup(self.scene_path)
            raise
        self.viewer_enabled = viewer_enabled
        self.developer_mode = developer_mode
        self.viewer = None
        self.running = True
        self.keys = queue.SimpleQueue()
        self.keyboard_twist = np.zeros(3)
        self.keyboard_active = False
        self.camera_index = 0
        self.next_state = 0.0
        self.next_lidar = 0.0
        self.next_camera = 0.0

    def stop(self) -> None:
        self.running = False

    async def run(self, websocket_url: str) -> None:
        """Connect the existing bridge and close all native resources on exit."""
        try:
            if self.viewer_enabled:
                import mujoco.viewer
                disable_mujoco_viewer_shortcuts()
                viewer_options = {"show_left_ui": False, "show_right_ui": False}
                if self.developer_mode:
                    viewer_options["key_callback"] = self.keys.put
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data, **viewer_options)
                with self.viewer.lock():
                    self.viewer.opt.geomgroup[3] = 0
                    self.viewer.opt.geomgroup[4] = 0
                    configure_viewer_camera(self.model, self.data, self.viewer.cam)
                if self.developer_mode:
                    print("[native:dev] W/S forward, A/D turn, Q/E lateral (latched); "
                          "Space stop, X reset, L load/unload moe_rough", flush=True)
                else:
                    print("[native] local keyboard motion disabled; Robonix owns motion", flush=True)
            async with websockets.connect(websocket_url, max_size=32 * 1024 * 1024) as websocket:
                await websocket.send(json.dumps({
                    "type": "hello", "protocolVersion": 1, "backend": "native",
                    "environment": self.environment["id"], "robot": self.robot["id"],
                    "visualMode": self.environment["visualMode"],
                }))
                receiver = asyncio.create_task(self._receive(websocket))
                try:
                    try:
                        await self._simulate(websocket)
                    except websockets.ConnectionClosed:
                        self.running = False
                    if receiver.done():
                        await receiver
                finally:
                    receiver.cancel()
                    await asyncio.gather(receiver, return_exceptions=True)
        finally:
            self.sensors.close()
            if self.viewer is not None:
                self.viewer.close()
            self.builder.cleanup(self.scene_path)

    async def _receive(self, websocket) -> None:
        """Acknowledge commands and stop simulation when the bridge disconnects."""
        try:
            await self._receive_messages(websocket)
        finally:
            self.running = False
            self.controller.command({"type": "emergency_stop"})

    async def _receive_messages(self, websocket) -> None:
        """Decode bridge commands; malformed requests cannot terminate reception."""
        async for raw in websocket:
            try:
                message = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(message, dict):
                continue
            if message.get("type") == "ping":
                await websocket.send(json.dumps({
                    "type": "pong", "sequence": message.get("sequence"),
                    "simulationTime": float(self.data.time),
                }))
                continue
            accepted = self.controller.command(message)
            if accepted and message.get("type") in ("cmd_vel", "emergency_stop", "reset"):
                self.keyboard_active = False
                self.keyboard_twist[:] = 0
            await websocket.send(json.dumps({
                "type": "command_ack", "sequence": message.get("sequence"),
                "accepted": bool(accepted),
                "policy": dict(self.controller.policy_status),
            }))

    def _keyboard(self) -> None:
        """Process press-only viewer callbacks; Space clears latched velocity axes."""
        if not self.developer_mode:
            return
        bindings = {87: (0, 1), 83: (0, -1), 81: (1, 1),
                    69: (1, -1), 65: (2, 1), 68: (2, -1)}
        while not self.keys.empty():
            key = self.keys.get_nowait()
            if key in bindings:
                axis, sign = bindings[key]
                self.keyboard_twist[axis] = sign * COMMAND_LIMITS[axis]
                self.keyboard_active = True
            elif key in (32, 88):
                self.keyboard_active = False
                self.keyboard_twist[:] = 0
                self.controller.command({"type": "reset" if key == 88 else "emergency_stop"})
            elif key == 76:
                accepted = self.controller.command({
                    "type": "policy", "id": "moe_rough",
                    "action": "unload" if self.controller.policy_status["loaded"] else "load",
                })
                if not accepted:
                    print(f"[native] policy: {self.controller.policy_status['error']}", file=sys.stderr)
        if self.keyboard_active:
            self.controller.command(dict(zip(
                ("type", "linearX", "linearY", "angularZ"),
                ("cmd_vel", *self.keyboard_twist))))

    async def _simulate(self, websocket) -> None:
        """Advance real physics in bounded batches so control input stays responsive."""
        start_wall = time.monotonic()
        start_sim = float(self.data.time)
        next_viewer_sync = start_wall
        while self.running and (self.viewer is None or self.viewer.is_running()):
            if self.developer_mode:
                self._keyboard()
            now = time.monotonic()
            target_sim = start_sim + now - start_wall
            steps = 0
            while self.data.time < target_sim and self.running and steps < 25:
                self.controller.step()
                mujoco.mj_step(self.model, self.data)
                steps += 1
            await self._publish_due(websocket)
            if self.viewer is not None and now >= next_viewer_sync:
                self.viewer.sync()
                next_viewer_sync = now + 1.0 / 60.0
            await asyncio.sleep(0.001)

    async def _publish_due(self, websocket) -> None:
        """Publish base/policy state and timestamped native sensor frames."""
        sim_time = float(self.data.time)
        if sim_time >= self.next_state:
            await websocket.send(json.dumps({
                "type": "state", "environment": self.environment["id"],
                "simulationTime": sim_time, "robot": self.controller.state(),
                "imu": self.sensors.imu(),
            }, separators=(",", ":")))
            self.next_state = sim_time + 0.02
        if sim_time >= self.next_lidar:
            scan, cloud = self.sensors.lidar()
            await websocket.send(json.dumps({
                "type": "scan", "timestamp": scan["timestamp"], "frame": "mid360_link",
                "angleMin": scan["angleMin"], "angleMax": scan["angleMax"],
                "angleIncrement": scan["angleIncrement"], "rangeMin": scan["rangeMin"],
                "rangeMax": scan["rangeMax"], "rangesF32": encoded(scan["ranges"]),
            }, separators=(",", ":")))
            await websocket.send(json.dumps({
                "type": "pointcloud", "timestamp": cloud["timestamp"], "frame": "mid360_link",
                "pointCount": int(cloud["points"].shape[0]), "xyzF32": encoded(cloud["points"]),
            }, separators=(",", ":")))
            self.next_lidar = sim_time + 1.0 / float(self.robot["sensors"]["lidar"]["updateHz"])
        if sim_time >= self.next_camera:
            camera_ids = list(self.sensors.cameras)
            camera = self.sensors.camera(camera_ids[self.camera_index % len(camera_ids)])
            self.camera_index += 1
            await websocket.send(json.dumps({
                "type": "camera", "timestamp": camera["timestamp"], "id": camera["id"],
                "frame": "front_camera_optical_frame",
                "width": camera["width"], "height": camera["height"],
                "rgbaU8": encoded(camera["rgb"]), "fovyDeg": camera["fovyDeg"],
                "depth": {"width": camera["depth"]["width"], "height": camera["depth"]["height"],
                          "dataF32": encoded(camera["depth"]["data"])},
            }, separators=(",", ":")))
            self.next_camera = sim_time + 1.0 / float(
                self.sensors.cameras[camera["id"]].get("updateHz", 10))


def parse_args() -> argparse.Namespace:
    """Read native startup options from CLI flags or deployment environment."""
    parser = argparse.ArgumentParser(description="Native MuJoCo runtime for Robonix")
    parser.add_argument("--environment", default=os.environ.get("SIM_ENVIRONMENT", "scenesmith_house_185"))
    parser.add_argument("--robot", default=os.environ.get("SIM_ROBOT", "go2"), choices=["go2"])
    parser.add_argument("--websocket", default=os.environ.get("BRIDGE_WS_URL", "ws://127.0.0.1:8765"))
    parser.add_argument("--headless", action="store_true", default=os.environ.get("SIM_HEADLESS") == "1")
    parser.add_argument("--dev", action="store_true", default=os.environ.get("SIM_DEV") == "1")
    return parser.parse_args()


async def async_main() -> int:
    """Run until signalled, reporting startup and runtime failures to stderr."""
    args = parse_args()
    try:
        runtime = NativeRuntime(
            Path(__file__).resolve().parents[2], args.environment, args.robot,
            not args.headless, args.dev)
    except Exception as error:
        print(f"[native] startup failed: {error}", file=sys.stderr)
        return 2
    loop = asyncio.get_running_loop()
    for name in ("SIGINT", "SIGTERM"):
        signal_name = getattr(signal, name, None)
        if signal_name is not None:
            loop.add_signal_handler(signal_name, runtime.stop)
    try:
        await runtime.run(args.websocket)
    except Exception as error:
        print(f"[native] runtime failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(async_main()))
