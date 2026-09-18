# SPDX-License-Identifier: Apache-2.0
"""Native simulation, local HTTP controls and web preview; no hardware transport."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import time
import numpy as np
import mujoco

from .controller import Go2Controller
from .scene import load_scene
from .sensors import Sensors
from .actions import ACTION_DURATIONS, ActionController, available_actions

ROOT = Path(__file__).resolve().parents[1]


class CommandState:
    """Single active velocity writer with an expiring lease; reset clears it."""
    def __init__(self):
        self.lock = threading.RLock()
        self.owner = None
        self.deadline = 0.
        self.velocity = [0., 0., 0.]
        self.reset_requested = False
        self.shutdown_requested = False
        self.epoch = 0
        self.latest = None
        self.latest_time = 0.
        self.camera = None
        self.action_request = None
        self.action_running = False
        self.view_mode = 'follow'

    def command(self, message, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if 'view' in message:
                if message['view'] not in ('overview','follow'):
                    raise ValueError('view must be overview or follow')
                self.view_mode = message['view']
                return {'accepted':True}
            if any(message.get(k) is True for k in ("stop", "reset", "shutdown")):
                self.velocity, self.owner, self.deadline = [0., 0., 0.], None, 0.
                self.reset_requested |= message.get("reset") is True
                self.shutdown_requested |= message.get("shutdown") is True
                self.action_request = None
                self.action_running = False
                return {"accepted": True}
            owner = message.get("owner")
            if not isinstance(owner, str) or not owner or len(owner) > 64:
                raise ValueError("A short owner string is required")
            if self.owner not in (None, owner) and now < self.deadline:
                raise PermissionError("Another active writer owns the velocity lease")
            if "action" in message:
                name, token = message["action"], message.get("action_id")
                if name not in ACTION_DURATIONS or not isinstance(token,str) or not 1<=len(token)<=64:
                    raise ValueError("Supported action name and short action_id required")
                if self.action_running and self.action_request != (name,token):
                    raise PermissionError("Another action is executing")
                self.action_request = (name,token)
                self.owner, self.deadline = owner, now+.4
                self.velocity = [0.,0.,0.]
                return {"accepted":True,"action_id":token}
            if self.action_running:
                raise PermissionError("Action owns motors until completion or stop")
            v = np.asarray(message.get("velocity"), dtype=float)
            if v.shape != (3,) or not np.isfinite(v).all():
                raise ValueError("velocity must be three finite values [x,y,yaw]")
            self.action_request = None
            self.owner = owner
            self.velocity = np.clip(v, [-.5, -.3, -.9], [.5, .3, .9]).tolist()
            self.deadline = now + .4
            return {"accepted": True, "velocity": self.velocity}

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            return self.velocity.copy() if now < self.deadline else [0., 0., 0.]


def handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, data, kind="application/json"):
            payload = json.dumps(data, allow_nan=False).encode() if kind == "application/json" else data
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Reader closed its preview; physics is unaffected.

        def do_GET(self):
            with state.lock:
                if self.path in ("/state", "/health"):
                    age = time.monotonic() - state.latest_time
                    value = dict(state.latest or {})
                    value.update({"ready": state.latest is not None and age < 1., "age_s": age,
                                  "backend": "native", "robot": "go2", "pid": os.getpid(),
                                  "epoch": state.epoch, "command": state.sample()})
                    status, value, kind = 200 if value["ready"] else 503, value, "application/json"
                elif self.path == "/camera":
                    status, value, kind = 200 if state.camera else 503, state.camera or {}, "application/json"
                elif self.path in ("/", "/web"):
                    status, value, kind = 200, (ROOT / "web/index.html").read_bytes(), "text/html; charset=utf-8"
                else:
                    status, value, kind = 404, {"error": "not found"}, "application/json"
            # Never block physics while a slow HTTP reader drains its socket.
            self.reply(status, value, kind)

        def do_POST(self):
            if self.path != "/command":
                return self.reply(404, {"error": "not found"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected bounded application/json body")
                message = json.loads(self.rfile.read(length))
                if not isinstance(message, dict):
                    raise ValueError("Expected JSON object")
                self.reply(200, state.command(message))
            except PermissionError as error:
                self.reply(409, {"error": str(error)})
            except (ValueError, TypeError) as error:
                self.reply(400, {"error": str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18765)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--follow-camera", action="store_true",
                        help="Presentation-only smooth observer camera; does not affect control")
    parser.add_argument("--duration", type=float, default=0.)
    parser.add_argument("--scene", choices=("room","courtyard"), default="room")
    args = parser.parse_args()
    state = CommandState()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler(state))
    model = load_scene(ROOT / ".runtime/assets", scene=args.scene)
    data = mujoco.MjData(model)
    control = ActionController(model, data, ROOT / ".runtime/assets")
    sensors = Sensors(model, data, camera=not args.no_camera)
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    viewer = None
    if args.viewer:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data)
        if args.follow_camera:
            with viewer.lock():
                viewer.cam.lookat[:] = data.xpos[control.bid]
                viewer.cam.distance = 2.6
                viewer.cam.azimuth = 135.
                viewer.cam.elevation = -25.
    start = time.monotonic()
    next_step = start
    tick = 0
    try:
        while not stopping.is_set() and not state.shutdown_requested and (not args.duration or time.monotonic()-start < args.duration):
            if viewer is not None and not viewer.is_running():
                break
            with state.lock:
                if state.reset_requested:
                    control.reset()
                    state.reset_requested = False
                    state.latest, state.camera = None, None
                    state.epoch += 1
                pending = state.action_request
                live = time.monotonic() < state.deadline
                if pending and live and control.action['id'] != pending[1]:
                    try:
                        control.start_action(*pending)
                        state.action_running = True
                    except Exception as exc:
                        control.active = False
                        control.action = {'name': pending[0], 'id': pending[1],
                                          'status': 'failed', 'error': str(exc)}
                        state.action_running = False
                elif control.active and (not pending or not live):
                    control.cancel_action()
            control.step(state.sample())
            with state.lock:
                state.action_running = control.active
            if tick % 10 == 0:
                latest = control.state()
                latest["action"] = dict(control.action)
                latest["scene"] = args.scene
                latest["available_actions"] = available_actions()
                latest["scan"] = sensors.scan()
                latest["imu_gyro"] = data.sensor("imu_gyro").data.tolist()
                latest["imu_acc"] = data.sensor("imu_acc").data.tolist()
                with state.lock:
                    state.latest, state.latest_time = latest, time.monotonic()
            if tick % 40 == 0 and sensors.renderer:
                camera = sensors.camera()
                with state.lock:
                    state.camera = camera
            if viewer and tick % 4 == 0:
                if args.follow_camera:
                    with viewer.lock():
                        overview=state.view_mode=='overview'
                        target = np.array([0.,2.5,.0]) if overview else data.xpos[control.bid].copy()
                        target[2] = .30
                        viewer.cam.lookat[:] += .12 * (target-viewer.cam.lookat)
                        distance=20.0 if overview else (3.2 if args.scene=='courtyard' else 2.6)
                        viewer.cam.distance += .08*(distance-viewer.cam.distance)
                        viewer.cam.elevation += .08*((-55. if overview else -25.)-viewer.cam.elevation)
                if args.scene=='courtyard':
                    from .courtyard import ZONES
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        for label, pos in ZONES:
                            geom=viewer.user_scn.geoms[viewer.user_scn.ngeom]
                            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LABEL,
                                np.zeros(3),np.asarray(pos),np.eye(3).ravel(),
                                np.array([.12,.20,.30,1.]))
                            geom.label=label
                            viewer.user_scn.ngeom+=1
                    action=control.action
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        'ROBONIX x GO2 | COURTYARD\nTORQUE-DRIVEN SIMULATION\n'
                        +f"Action: {action['name'] or 'locomotion'} / {action['status']}",
                        ''))
                viewer.sync()
            if tick == 0:
                print(f"READY native Go2 simulation http://127.0.0.1:{args.port}/web", flush=True)
            tick += 1
            next_step += model.opt.timestep
            delay = next_step-time.monotonic()
            if delay > 0:
                stopping.wait(delay)
            elif delay < -.2:
                next_step = time.monotonic()
    finally:
        state.command({"stop": True})
        control.cancel_action()
        for _ in range(400):
            control.step([0, 0, 0])
        if viewer:
            viewer.close()
        sensors.close()
        server.shutdown()
        server.server_close()
        print("STOPPED simulation only", flush=True)


if __name__ == "__main__":
    main()
