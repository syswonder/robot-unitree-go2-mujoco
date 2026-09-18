from __future__ import annotations

import math

import mujoco
import numpy as np


GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))
ELEVATION_SEQUENCE = 0.7548776662466927


def mid360_directions(count: int, vertical_fov: list[float]) -> np.ndarray:
    """Generate deterministic spherical lidar directions inside the vertical FOV."""
    minimum = math.sin(math.radians(vertical_fov[0]))
    maximum = math.sin(math.radians(vertical_fov[1]))
    result = np.empty((count, 3), dtype=np.float64)
    for index in range(count):
        azimuth = index * GOLDEN_ANGLE
        unit = (0.5 + index * ELEVATION_SEQUENCE) % 1.0
        elevation = math.asin(minimum + (maximum - minimum) * unit)
        horizontal = math.cos(elevation)
        result[index] = [horizontal * math.cos(azimuth), horizontal * math.sin(azimuth), math.sin(elevation)]
    return result


def planar_directions(count: int, elevation_deg: float = 0.0) -> np.ndarray:
    """Return one evenly spaced horizontal scan, optionally tilted in elevation."""
    elevation = math.radians(elevation_deg)
    angles = -math.pi + np.arange(count) * (2.0 * math.pi / count)
    horizontal = math.cos(elevation)
    return np.column_stack((horizontal * np.cos(angles), horizontal * np.sin(angles),
                            np.full(count, math.sin(elevation))))


def pinhole_directions(width: int, height: int, fovy_deg: float) -> np.ndarray:
    """Generate unit pixel rays in MuJoCo camera coordinates (forward is -z)."""
    tan_y = math.tan(math.radians(fovy_deg) * 0.5)
    tan_x = tan_y * width / height
    columns = (2.0 * (np.arange(width) + 0.5) / width - 1.0) * tan_x
    rows = (1.0 - 2.0 * (np.arange(height) + 0.5) / height) * tan_y
    x, y = np.meshgrid(columns, rows)
    vectors = np.stack((x, y, -np.ones_like(x)), axis=-1)
    vectors /= np.linalg.norm(vectors, axis=-1, keepdims=True)
    return vectors.reshape((-1, 3))


class NativeSensorSuite:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: dict) -> None:
        """Resolve the agreed Go2 sensors and reject missing IDs before indexing."""
        self.model = model
        self.data = data
        self.config = config
        self.geom_group = np.array([1, 0, 0, 1, 0, 1], dtype=np.uint8)
        self.lidar_config = config["lidar"]
        self.lidar_site = self._id(mujoco.mjtObj.mjOBJ_SITE, self.lidar_config["site"])
        self.lidar_body = int(model.site_bodyid[self.lidar_site])
        self.lidar_directions = mid360_directions(
            self.lidar_config["raysPerScan"], self.lidar_config["verticalFovDeg"])
        self.planar_directions = planar_directions(
            self.lidar_config.get("planarRays", 720), self.lidar_config.get("planarElevationDeg", 0.0))
        self.scan_index = 0
        self.imu_config = config["imu"]
        self.gyroscope = self._id(mujoco.mjtObj.mjOBJ_SENSOR, self.imu_config["gyroscope"])
        self.accelerometer = self._id(mujoco.mjtObj.mjOBJ_SENSOR, self.imu_config["accelerometer"])
        if model.sensor_dim[self.gyroscope] != 3 or model.sensor_dim[self.accelerometer] != 3:
            raise ValueError("Go2 IMU must have three-axis gyro and accelerometer sensors")
        self.cameras = {item["id"]: item for item in config.get("cameras", [])
                        if item["camera"] == "front_rgbd_camera"}
        if len(self.cameras) != 1:
            raise ValueError("Go2 sensors require one front_rgbd_camera configuration")
        self.camera_ids = {
            key: self._id(mujoco.mjtObj.mjOBJ_CAMERA, item["camera"])
            for key, item in self.cameras.items()
        }
        self.renderers: dict[str, mujoco.Renderer] = {}
        self.render_option = mujoco.MjvOption()
        self.render_option.geomgroup[3] = 0
        self.render_option.geomgroup[4] = 0

    def _id(self, kind, name: str) -> int:
        result = mujoco.mj_name2id(self.model, kind, name)
        if result < 0:
            raise ValueError(f"Go2 model is missing sensor configuration target {name}")
        return result

    def imu(self) -> dict:
        """Return native gyro and specific acceleration in the XML IMU site frame."""
        def vector(sensor_id: int) -> list[float]:
            address = int(self.model.sensor_adr[sensor_id])
            return self.data.sensordata[address:address + 3].astype(float).tolist()
        return {
            "timestamp": float(self.data.time),
            "gyroscope": vector(self.gyroscope),
            "accelerometer": vector(self.accelerometer),
        }

    def _raycast(self, local: np.ndarray, phase: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """Cast lidar rays through static and moving collision geometry."""
        matrix = self.data.site_xmat[self.lidar_site].reshape(3, 3)
        if phase:
            cosine, sine = math.cos(phase), math.sin(phase)
            rotated = local.copy()
            rotated[:, 0] = cosine * local[:, 0] - sine * local[:, 1]
            rotated[:, 1] = sine * local[:, 0] + cosine * local[:, 1]
            local = rotated
        world = local @ matrix.T
        distances = np.empty(local.shape[0], dtype=np.float64)
        geom_ids = np.empty(local.shape[0], dtype=np.int32)
        mujoco.mj_multiRay(
            self.model, self.data, self.data.site_xpos[self.lidar_site], world.reshape(-1),
            self.geom_group, 1, self.lidar_body, geom_ids, distances,
            local.shape[0], float(self.lidar_config["maxRange"]),
        )
        return distances, local

    def lidar(self) -> tuple[dict, dict]:
        """Return a planar scan and rotating 3-D cloud in the lidar site frame."""
        config = self.lidar_config
        distances, local = self._raycast(self.lidar_directions, self.scan_index * GOLDEN_ANGLE)
        valid = (distances >= config["minRange"]) & (distances <= config["maxRange"])
        points = (local[valid] * distances[valid, None]).astype(np.dtype("<f4"), copy=False)
        self.scan_index += 1

        planar_distances, _ = self._raycast(self.planar_directions)
        planar_valid = ((planar_distances >= config["minRange"]) &
                        (planar_distances <= config["maxRange"]))
        planar = np.where(planar_valid, planar_distances, np.inf).astype(np.dtype("<f4"))
        count = planar.size
        scan = {
            "timestamp": float(self.data.time), "ranges": planar,
            "angleMin": -math.pi, "angleMax": math.pi - 2.0 * math.pi / count,
            "angleIncrement": 2.0 * math.pi / count,
            "rangeMin": float(config["minRange"]), "rangeMax": float(config["maxRange"]),
        }
        cloud = {"timestamp": float(self.data.time), "points": points}
        return scan, cloud

    def camera(self, camera_id: str) -> dict:
        """Render front RGB and optical-z depth, with infinity for missing pixels."""
        config = self.cameras[camera_id]
        camera = self.camera_ids[camera_id]
        renderer = self.renderers.get(camera_id)
        if renderer is None:
            renderer = mujoco.Renderer(
                self.model, height=int(config["height"]), width=int(config["width"]))
            self.renderers[camera_id] = renderer
        renderer.update_scene(self.data, camera=config["camera"], scene_option=self.render_option)
        rendered = np.ascontiguousarray(renderer.render(), dtype=np.uint8)
        alpha = np.full((*rendered.shape[:2], 1), 255, dtype=np.uint8)
        rgba = np.concatenate((rendered, alpha), axis=2)

        depth_width = int(config.get("depthWidth", config["width"]))
        depth_height = int(config.get("depthHeight", config["height"]))
        directions = pinhole_directions(depth_width, depth_height, float(self.model.cam_fovy[camera]))
        matrix = self.data.cam_xmat[camera].reshape(3, 3)
        world = directions @ matrix.T
        distances = np.empty(directions.shape[0], dtype=np.float64)
        geom_ids = np.empty(directions.shape[0], dtype=np.int32)
        body = int(self.model.cam_bodyid[camera])
        mujoco.mj_multiRay(
            self.model, self.data, self.data.cam_xpos[camera], world.reshape(-1),
            self.geom_group, 1, body, geom_ids, distances, directions.shape[0],
            float(config["maxDepth"]) / float(np.min(-directions[:, 2])),
        )
        distances *= -directions[:, 2]
        valid = ((distances >= config["minDepth"]) & (distances <= config["maxDepth"]))
        depth = np.where(valid, distances, np.inf).astype(np.dtype("<f4"))
        return {
            "timestamp": float(self.data.time), "id": camera_id,
            "width": int(config["width"]), "height": int(config["height"]), "rgb": rgba,
            "fovyDeg": float(self.model.cam_fovy[camera]),
            "depth": {"width": depth_width, "height": depth_height, "data": depth},
        }

    def close(self) -> None:
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()
