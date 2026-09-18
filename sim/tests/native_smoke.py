#!/usr/bin/env python3
"""Go2 model, sensor, rendering and physics checks for current native environments."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.native.controller import Go2Controller
from sim.native.runtime import configure_viewer_camera
from sim.native.scene_builder import NativeSceneBuilder
from sim.native.sensors import NativeSensorSuite


def check_environment(builder: NativeSceneBuilder, environment_id: str, policy: bool) -> dict:
    """Compile shared assets and verify Go2 standing, sensors, camera and live switches."""
    scene_path, environment, robot = builder.build(environment_id, "go2")
    sensors = None
    try:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        controller = Go2Controller(model, data, clock=lambda: data.time)
        assert not controller.state()["policy"]["loaded"]
        sensors = NativeSensorSuite(model, data, robot["sensors"])
        for _ in range(round(2/model.opt.timestep)):
            controller.step()
            mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all() and data.xmat[controller.base, 8] > .9
        assert np.linalg.norm(data.qvel[:3]) < .05, "Go2 did not settle while standing"
        assert "arm" not in controller.state()
        if policy:
            for action in ("load", "unload"):
                pose, velocity = data.qpos.copy(), data.qvel.copy()
                assert controller.command({"type": "policy", "action": action, "id": "moe_rough"}), controller.policy_status
                np.testing.assert_array_equal(data.qpos, pose)
                np.testing.assert_array_equal(data.qvel, velocity)
                for _ in range(round(2/model.opt.timestep)):
                    controller.step()
                    mujoco.mj_step(model, data)
                    assert data.xmat[controller.base, 8] > .8, "Go2 tilted during policy switch"

        viewer_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(viewer_camera)
        configure_viewer_camera(model, data, viewer_camera)
        option = mujoco.MjvOption()
        perturb = mujoco.MjvPerturb()
        scene = mujoco.MjvScene(model, maxgeom=1000)
        mujoco.mjv_updateScene(
            model, data, option, perturb, viewer_camera,
            mujoco.mjtCatBit.mjCAT_ALL, scene)
        lookat_before = viewer_camera.lookat.copy()
        mujoco.mjv_moveCamera(
            model, mujoco.mjtMouse.mjMOUSE_MOVE_H, 0.15, 0.0,
            scene, viewer_camera)
        pan_delta = float(np.linalg.norm(viewer_camera.lookat - lookat_before))
        assert viewer_camera.type == mujoco.mjtCamera.mjCAMERA_FREE
        assert pan_delta > 1e-6, "native viewer camera did not accept pan input"

        scan, cloud = sensors.lidar()
        imu = sensors.imu()
        cameras = {}
        for camera_id in sensors.cameras:
            frame = sensors.camera(camera_id)
            rgb = frame["rgb"][:, :, :3]
            depth = frame["depth"]["data"]
            valid_depth = depth[np.isfinite(depth) & (depth > 0)]
            assert rgb.size > 100 and float(rgb.std()) > 2.0, f"{camera_id} RGB is blank"
            assert valid_depth.size > 100, f"{camera_id} depth has no valid returns"
            cameras[camera_id] = {
                "rgb_std": float(rgb.std()),
                "valid_depth": int(valid_depth.size),
            }

        finite_scan = int(np.count_nonzero(np.isfinite(scan["ranges"])))
        assert scan["ranges"].size == robot["sensors"]["lidar"]["planarRays"]
        assert np.all(np.isfinite(cloud["points"]))
        assert finite_scan > 0 and cloud["points"].shape[0] > 0
        assert len(imu["gyroscope"]) == 3 and len(imu["accelerometer"]) == 3
        np.testing.assert_array_equal(data.xfrc_applied, 0)
        np.testing.assert_array_equal(data.qfrc_applied, 0)
        return {
            "ok": True,
            "environment": environment["id"],
            "model": {"nq": model.nq, "nv": model.nv, "nu": model.nu},
            "scan_returns": finite_scan,
            "cloud_points": int(cloud["points"].shape[0]),
            "cameras": cameras,
            "viewer_pan_delta": pan_delta,
            "policy_switches": policy,
            "base_height": float(data.qpos[2]),
            "policy": controller.state()["policy"],
        }
    finally:
        if sensors is not None:
            sensors.close()
        builder.cleanup(scene_path)


def main() -> None:
    """Check one named environment or every manifest environment with --environment all."""
    builder = NativeSceneBuilder(ROOT)
    environments = [item["id"] for item in builder.environment_manifest["environments"]]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", default="all", choices=["all", *environments])
    parser.add_argument("--policy", action="store_true", help="also exercise live ONNX load and unload")
    args = parser.parse_args()
    selected = environments if args.environment == "all" else [args.environment]
    for environment in selected:
        print(json.dumps(check_environment(builder, environment, args.policy), indent=2), flush=True)


if __name__ == "__main__":
    main()
