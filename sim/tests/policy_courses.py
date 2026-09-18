#!/usr/bin/env python3
"""Bounded, rendering-free validation of the released native Go2 ONNX policy.

Run with --environment stairs for the first staircase, or --environment track
for a short starting-area displacement only. Neither test validates a full race.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.native.controller import Go2Controller
from sim.native.scene_builder import NativeSceneBuilder


def switch_policy(controller: Go2Controller, action: str) -> None:
    """Use the runtime command and verify that switching preserves physical state."""
    position, velocity = controller.data.qpos.copy(), controller.data.qvel.copy()
    accepted = controller.command({"type": "policy", "action": action, "id": "moe_rough"})
    assert accepted, controller.policy_status
    assert controller.policy_status["loaded"] == (action == "load")
    np.testing.assert_array_equal(controller.data.qpos, position)
    np.testing.assert_array_equal(controller.data.qvel, velocity)


def foot_support(controller: Go2Controller) -> tuple[int, float | None]:
    """Count feet carrying positive normal load on terrain and read contact height."""
    model, data = controller.model, controller.data
    feet, heights = set(), []
    contact_force = np.empty(6)
    for index, contact in enumerate(data.contact):
        first, second = (int(value) for value in contact.geom)
        foot = first if first in controller.foot_geoms else second
        terrain = second if foot == first else first
        if foot not in controller.foot_geoms or model.geom_group[terrain] != 3:
            continue
        mujoco.mj_contactForce(model, data, index, contact_force)
        if contact_force[0] > 1.0 and abs(contact.frame[2]) > .5:
            feet.add(foot)
            heights.append(float(contact.pos[2]))
    return len(feet), max(heights) if heights else None


def step_physics(controller: Go2Controller, command: tuple, metrics: dict,
                 loaded: bool) -> None:
    """Advance unchanged runtime physics while checking state, support and policy health."""
    model, data = controller.model, controller.data
    assert controller.command(dict(zip(
        ("type", "linearX", "linearY", "angularZ"), ("cmd_vel", *command))))
    position, velocity = data.qpos.copy(), data.qvel.copy()
    controller.step()
    np.testing.assert_array_equal(data.qpos, position)
    np.testing.assert_array_equal(data.qvel, velocity)
    assert not np.any(data.xfrc_applied) and not np.any(data.qfrc_applied)
    mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert np.isfinite(data.ctrl).all() and np.isfinite(data.qacc).all()
    assert not np.any(data.warning.number), "MuJoCo reported a physics warning"
    assert controller.policy_status["loaded"] == loaded, controller.policy_status
    assert controller.policy_status["error"] is None, controller.policy_status
    q = controller.base_qpos
    x, y, z = (float(value) for value in data.qpos[q:q+3])
    quaternion = data.qpos[q+3:q+7]
    up = 1 - 2*(quaternion[1]**2 + quaternion[2]**2)
    count, height = foot_support(controller)
    metrics["samples"] += 1
    metrics["supported_samples"] += int(count > 0)
    metrics["min_up"] = min(metrics["min_up"], float(up))
    metrics["min_height"] = min(metrics["min_height"], z)
    metrics["min_support_feet"] = min(metrics["min_support_feet"], count)
    metrics["max_support_feet"] = max(metrics["max_support_feet"], count)
    metrics["unsupported_time"] = metrics["unsupported_time"] + model.opt.timestep if not count else 0
    metrics["max_unsupported_s"] = max(metrics["max_unsupported_s"], metrics["unsupported_time"])
    metrics["max_abs_lane_error"] = max(metrics["max_abs_lane_error"], abs(y-metrics["lane_y"]))
    if z > metrics["peak_height"]:
        metrics.update(peak_height=z, peak_height_x=x)
    if height is not None:
        metrics["max_support_height"] = max(metrics["max_support_height"], height)
        if 3.0 <= x <= 3.9 and height >= .30:
            metrics["top_supported_samples"] += 1
        if x >= 5.9 and height < .12:
            metrics["landing_supported_samples"] += 1
    metrics["final_position"] = [x, y, z]
    metrics["simulation_time"] = float(data.time)
    assert up > .7, f"Go2 lost upright attitude: up={up:.3f}, x={x:.3f}"
    assert z > .20, f"Go2 base fell below standing clearance: z={z:.3f}"
    assert abs(y-metrics["lane_y"]) < .45, f"Go2 left the test lane at y={y:.3f}"
    assert metrics["max_unsupported_s"] < .4, "Go2 lost foot support for 0.4 s"


def hold(controller: Go2Controller, duration: float, metrics: dict, loaded: bool) -> None:
    """Hold zero Twist through real physics for a fixed simulation duration."""
    for _ in range(round(duration/controller.model.opt.timestep)):
        step_physics(controller, (0.0, 0.0, 0.0), metrics, loaded)


def follow_lane(controller: Go2Controller, speed: float, lane_y: float) -> tuple:
    """Issue only high-level Twist, correcting heading and lateral lane drift."""
    q = controller.base_qpos
    w, x, y, z = controller.data.qpos[q+3:q+7]
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    lane_error = lane_y - controller.data.qpos[q+1]
    return (speed, float(np.clip(.5*lane_error, -.15, .15)),
            float(np.clip(-1.5*yaw, -.5, .5)))


def run_course(environment: str) -> dict:
    """Validate one bounded course and always release its temporary scene directory."""
    builder = NativeSceneBuilder(ROOT)
    environment_id = f"go2_rl_{environment}"
    scene, _, _ = builder.build(environment_id, "go2")
    metrics = {"environment": environment_id, "ok": False}
    try:
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
        controller = Go2Controller(model, data, clock=lambda: data.time)
        assert not controller.policy_status["loaded"]
        np.testing.assert_allclose(data.qpos[:2], [-.5, 0], atol=1e-8)
        policy_path = ROOT / "assets/robots/go2/policy/moe_rough/policy.onnx"
        metrics.update(
            mujoco_version=mujoco.__version__, onnxruntime_version=onnxruntime.__version__,
            policy_sha256=hashlib.sha256(policy_path.read_bytes()).hexdigest(),
            timestep=float(model.opt.timestep), speed=.5 if environment == "stairs" else .2,
            lane_y=0.0, samples=0, supported_samples=0, min_up=1.0, min_height=1.0,
            peak_height=0.0, min_support_feet=4, max_support_feet=0,
            unsupported_time=0.0, max_unsupported_s=0.0, max_abs_lane_error=0.0,
            max_support_height=0.0, top_supported_samples=0, landing_supported_samples=0,
        )
        hold(controller, 1, metrics, False)
        switch_policy(controller, "load")
        hold(controller, 1, metrics, True)
        metrics["start_position"] = data.qpos[:3].tolist()
        metrics["policy_start_height"] = float(data.qpos[2])
        target_x, budget = (6.25, 20.0) if environment == "stairs" else (0.0, 5.0)
        start_time = float(data.time)
        for _ in range(round(budget/model.opt.timestep)):
            if data.qpos[0] >= target_x:
                break
            command = follow_lane(controller, metrics["speed"], metrics["lane_y"])
            step_physics(controller, command, metrics, True)
        metrics["travel_simulation_s"] = float(data.time)-start_time
        hold(controller, 1, metrics, True)
        metrics["course_final_position"] = data.qpos[:3].tolist()
        metrics["policy_rise"] = metrics["peak_height"]-metrics["policy_start_height"]
        metrics["policy_descent"] = metrics["peak_height"]-float(data.qpos[2])
        if environment == "stairs":
            assert data.qpos[0] > 5.9, "Did not finish the first staircase within 20 simulation seconds"
            assert metrics["policy_rise"] > .25, "No substantial ascent was observed"
            assert metrics["policy_descent"] > .22, "No substantial descent was observed"
            assert data.qpos[2] < metrics["policy_start_height"]+.15, "Go2 did not return to the lower landing"
            assert metrics["top_supported_samples"] > 10, "No loaded foot contacts on the top landing"
            assert metrics["landing_supported_samples"] > 10, "No loaded foot contacts after descent"
        else:
            displacement = float(data.qpos[0])-metrics["start_position"][0]
            assert .35 < displacement < .8, "Track check requires a short, bounded displacement"
            assert data.qpos[0] < .3, "Track check left the clear starting area"
        switch_policy(controller, "unload")
        hold(controller, 2, metrics, False)
        assert np.linalg.norm(data.qvel[:3]) < .05, "Go2 did not settle after unload"
        assert metrics["supported_samples"] / metrics["samples"] > .8, "Insufficient loaded foot support"
        metrics.update(ok=True, policy_load_unload=True)
    except Exception as error:
        metrics["error"] = f"{type(error).__name__}: {error}"
    finally:
        builder.cleanup(scene)
    if metrics.get("samples"):
        metrics["supported_fraction"] = metrics["supported_samples"] / metrics["samples"]
    metrics.pop("unsupported_time", None)
    return metrics


def main() -> int:
    """Print a reproducible JSON report and fail the process when assertions fail."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", choices=("stairs", "track"), default="stairs")
    args = parser.parse_args()
    result = run_course(args.environment)
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
