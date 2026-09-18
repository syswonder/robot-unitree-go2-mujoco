# SPDX-License-Identifier: Apache-2.0
"""Robonix lifecycle and capability adapters for the Native Go2 simulator."""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import uuid

from robonix_api import Primitive, Ok, Err
from .bridge import request
from .motion import distance_velocity

KIND = sys.argv[1]
provider = Primitive(id=f"go2_sim_{KIND}", namespace=f"robonix/primitive/{KIND}",
                     pkg_root=Path(os.environ["RBNX_PACKAGE_ROOT"]))
cancelled = threading.Event()
move_lock = threading.Lock()

STREAMS = {
    "chassis": [("odom", "/go2_sim/odom", "Odometry", "reliable"),
                ("twist_in", "/go2_sim/cmd_vel", "Twist", "reliable")],
    "lidar": [("lidar", "/go2_sim/scan", "LaserScan", "best_effort")],
    "camera": [("rgb", "/go2_sim/camera/color/image_raw", "Image", "best_effort"),
               ("depth", "/go2_sim/camera/depth/image_raw", "Image", "best_effort"),
               ("intrinsics", "/go2_sim/camera/camera_info", "CameraInfo", "latched"),
               ("extrinsics", "/go2_sim/camera/extrinsics", "TransformStamped", "latched")],
    "imu": [("imu", "/go2_sim/imu", "Imu", "best_effort")],
}


@provider.on_init
def initialize(config):
    try:
        if not request("/health")["ready"]:
            return Err("Native simulator has no fresh state")
        for capability, topic, msg_type, qos in STREAMS[KIND]:
            if capability != "twist_in" and not provider.wait_for_topic(topic, msg_type, 8.):
                return Err(f"No simulation sample on {topic}")
            provider.declare_ros2_topic(f"robonix/primitive/{KIND}/{capability}", topic, qos=qos)
        cancelled.clear()
        return Ok()
    except Exception as error:
        return Err(str(error))


@provider.on_activate
def activate():
    cancelled.clear()
    return Ok()


@provider.on_deactivate
@provider.on_shutdown
def stop():
    cancelled.set()
    if KIND == "chassis":
        try:
            request("/command", {"stop": True})
        except Exception:
            pass  # Native lease independently expires in 0.4 seconds.
    return Ok()


if KIND == "chassis":
    import chassis_pb2
    import std_msgs_pb2
    import go2_sim_action_pb2

    @provider.grpc("robonix/primitive/chassis/action")
    def perform_action(req):
        result = {"status":"busy"}
        if not move_lock.acquire(blocking=False):
            return go2_sim_action_pb2.PerformAction_Response(success=False,status="busy",detail="Chassis is executing another call")
        acquired=False
        try:
            initial=request('/state')
            if req.name not in initial.get('available_actions',[]):
                raise ValueError('Action not supported by this simulator: '+req.name)
            token=uuid.uuid4().hex
            epoch=initial['epoch']
            start=time.monotonic()
            while time.monotonic()-start<50.:
                if cancelled.is_set():
                    result={'status':'cancelled'}
                    break
                state=request('/state')
                if not state['ready'] or state['epoch']!=epoch:
                    raise RuntimeError('Simulator reset or unavailable')
                action=state.get('action',{})
                if action.get('id')==token and action.get('status')!='running':
                    result=action
                    break
                request('/command',{'owner':'primitive','action':req.name,'action_id':token})
                acquired=True
                cancelled.wait(.06)
            else:
                result={'status':'timeout'}
        except Exception as error:
            result={'status':'error','error':str(error)}
        finally:
            if acquired:
                try: request('/command',{'stop':True})
                except Exception: pass
            move_lock.release()
        return go2_sim_action_pb2.PerformAction_Response(success=result['status']=='done',
            status=result['status'], detail=json.dumps(result,allow_nan=False))

    def response(value):
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps(value, allow_nan=False)))

    @provider.grpc("robonix/primitive/chassis/move")
    def move(req):
        """Timed velocity or measured relative displacement, with one active caller."""
        if not move_lock.acquire(blocking=False):
            return response({"status": "busy"})
        acquired = False
        try:
            cmd = req.command
            fields = (cmd.forward_m, cmd.rotate_deg, cmd.duration_sec, cmd.linear_x,
                      cmd.linear_y, cmd.linear_z, cmd.angular_x, cmd.angular_y, cmd.angular_z)
            if not all(math.isfinite(v) for v in fields):
                raise ValueError("Command fields must be finite")
            if any(v != 0 for v in (cmd.linear_z, cmd.angular_x, cmd.angular_y)):
                raise ValueError("Go2 planar interface supports x/y/yaw only")
            duration = cmd.duration_sec or 1.
            if not 0 < duration <= 30. or abs(cmd.forward_m) > 5. or abs(cmd.rotate_deg) > 360.:
                raise ValueError("Use 0..30 s bursts, relative distance <=5 m, angle <=360 deg")
            initial = request("/state")
            epoch, heading = initial["epoch"], initial["yaw"]
            previous_yaw, turned = heading, 0.
            mode = "distance" if cmd.forward_m else "angle" if cmd.rotate_deg else "velocity"
            timeout = 30. if mode != "velocity" else duration
            start = time.monotonic()
            result = "done"
            while time.monotonic()-start < timeout:
                if cancelled.is_set():
                    result = "cancelled"
                    break
                state = request("/state")
                if not state["ready"] or state["epoch"] != epoch:
                    raise RuntimeError("Simulation stopped or reset during command")
                delta_yaw = state["yaw"]-previous_yaw
                turned += math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
                previous_yaw = state["yaw"]
                velocity = [cmd.linear_x, cmd.linear_y, cmd.angular_z]
                if mode == "distance":
                    dx, dy = (state["position"][i]-initial["position"][i] for i in (0, 1))
                    error = cmd.forward_m-dx*math.cos(heading)-dy*math.sin(heading)
                    if abs(error) < .04:
                        break
                    # Avoid the policy's near-zero velocity standing deadband.
                    velocity = [distance_velocity(error, cmd.linear_x), 0., 0.]
                elif mode == "angle":
                    error = math.radians(cmd.rotate_deg)-turned
                    if abs(error) < .04:
                        break
                    velocity = [0., 0., math.copysign(max(.18,min(.8,1.6*abs(error))),error)]
                request("/command", {"owner": "primitive", "velocity": velocity})
                acquired = True
                cancelled.wait(.08)
            else:
                if mode != "velocity":
                    result = "timeout"
            final = request("/state")
            return response({"status": result, "mode": mode,
                             "elapsed_s": time.monotonic()-start,
                             "position": final["position"], "yaw_rad": final["yaw"]})
        except Exception as error:
            return response({"status": "error", "error": str(error)})
        finally:
            if acquired:
                try:
                    request("/command", {"owner": "primitive", "velocity": [0., 0., 0.]})
                except Exception:
                    pass
            move_lock.release()


if __name__ == "__main__":
    provider.run()
