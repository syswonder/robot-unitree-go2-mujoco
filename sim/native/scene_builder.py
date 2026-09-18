from __future__ import annotations

import json
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def _numbers(raw: str | None, fallback: list[float]) -> list[float]:
    if not raw:
        return list(fallback)
    return [float(value) for value in raw.split()]


def _multiply_quaternion(left: list[float], right: list[float]) -> list[float]:
    """Compose wxyz rotations, applying the right rotation first."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def _format(values: list[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


class NativeSceneBuilder:
    """Build the modular scene layout on a native filesystem."""

    def __init__(self, project_root: Path) -> None:
        """Read the shared environment and robot catalogues without changing assets."""
        self.root = project_root.resolve()
        self.environment_manifest = self._read_json(
            self.root / "assets/environments/manifest.json")
        self.robot_index = self._read_json(self.root / "assets/robots/index.json")

    @staticmethod
    def _read_json(path: Path):
        return json.loads(path.read_text(encoding="utf-8"))

    def environment(self, environment_id: str) -> dict:
        """Resolve an environment by its manifest ID or report an unknown ID."""
        for item in self.environment_manifest["environments"]:
            if item["id"] == environment_id:
                return item
        raise ValueError(f"unknown environment: {environment_id}")

    def robot(self, robot_id: str) -> dict:
        """Resolve the robot package metadata from the shared robot index."""
        robots = self.robot_index.get("robots", self.robot_index)
        entries = robots.values() if isinstance(robots, dict) else robots
        for item in entries:
            candidate = item.get("id") if isinstance(item, dict) else str(item)
            if candidate == robot_id:
                package_path = self.root / "assets/robots" / robot_id
                return self._read_json(package_path / "robot.json")
        raise ValueError(f"unknown robot: {robot_id}")

    def build(self, environment_id: str, robot_id: str) -> tuple[Path, dict, dict]:
        """Assemble temporary mesh scenes, preserving the parent-owned robot XML."""
        environment = self.environment(environment_id)
        if environment.get("visualMode") != "mesh":
            mode = environment.get("visualMode", "unknown")
            raise ValueError(
                f"native backend does not support {mode.upper()} environment "
                f"'{environment_id}'; use --backend web")

        robot = self.robot(robot_id)
        scene_dir = Path(tempfile.mkdtemp(prefix=f"mujoco-{environment_id}-"))
        environment_dir = (self.root / environment["xmlPath"]).resolve().parent
        shutil.copytree(environment_dir, scene_dir, dirs_exist_ok=True)

        robot_dir = self.root / "assets/robots" / robot_id
        for relative in self._read_json(robot_dir / robot["files"]):
            source = robot_dir / relative
            target = scene_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)

        robot_xml = scene_dir / robot["model"]
        robot_tree = ET.parse(robot_xml)
        self._resolve_asset_paths(robot_tree.getroot(), scene_dir)
        robot_tree.write(robot_xml, encoding="utf-8", xml_declaration=True)
        spawn = environment.get("spawn")
        if environment.get("spawnPath"):
            spawn = self._read_json(self.root / environment["spawnPath"])
        if spawn:
            self._apply_spawn(robot_xml, spawn)

        environment_xml = self.root / environment["xmlPath"]
        tree = ET.parse(environment_xml)
        root = tree.getroot()
        self._resolve_asset_paths(root, scene_dir)
        root.set("model", f"{environment_id}_{robot_id}")
        root.insert(0, ET.Element("include", {"file": robot["model"]}))
        objects_source = environment.get("objectsPath")
        if objects_source:
            objects_path = self.root / objects_source
            shutil.copy2(objects_path, scene_dir / "objects.xml")
            root.insert(1, ET.Element("include", {"file": "objects.xml"}))
        scene_path = scene_dir / "scene.xml"
        tree.write(scene_path, encoding="utf-8", xml_declaration=True)
        return scene_path, environment, robot

    @staticmethod
    def _resolve_asset_paths(root: ET.Element, directory: Path) -> None:
        """Make asset paths absolute before includes merge global compiler settings."""
        compiler = root.find("compiler")
        settings = {} if compiler is None else dict(compiler.attrib)
        for asset in root.findall("asset/*[@file]"):
            subdirectory = settings.get("texturedir" if asset.tag == "texture" else "meshdir",
                                        settings.get("assetdir", ""))
            asset.set("file", str((directory / subdirectory / asset.get("file")).resolve()))
        if compiler is not None:
            for name in ("meshdir", "texturedir", "assetdir"):
                compiler.attrib.pop(name, None)

    def _apply_spawn(self, robot_xml: Path, spawn: dict) -> None:
        """Transform root poses and free-joint keyframes by the same world yaw."""
        position = spawn.get("position")
        if not isinstance(position, list) or len(position) != 2:
            raise ValueError("environment spawn position must contain x and y")
        yaw = float(spawn.get("yaw", 0.0))
        yaw_quaternion = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
        tree = ET.parse(robot_xml)
        root = tree.getroot()
        worldbody = root.find("worldbody")
        bodies = [] if worldbody is None else worldbody.findall("body")
        if not bodies:
            raise ValueError("robot XML has no top-level body")
        for body in bodies:
            xyz = _numbers(body.get("pos"), [0.0, 0.0, 0.0])
            while len(xyz) < 3:
                xyz.append(0.0)
            local_x, local_y = xyz[:2]
            xyz[0] = float(position[0]) + math.cos(yaw) * local_x - math.sin(yaw) * local_y
            xyz[1] = float(position[1]) + math.sin(yaw) * local_x + math.cos(yaw) * local_y
            body.set("pos", _format(xyz))
            quaternion = _numbers(body.get("quat"), [1.0, 0.0, 0.0, 0.0])
            body.set("quat", _format(_multiply_quaternion(yaw_quaternion, quaternion)))
        for key in root.findall(".//keyframe/key[@qpos]"):
            qpos = _numbers(key.get("qpos"), [])
            if len(qpos) >= 7:
                local_x, local_y = qpos[:2]
                qpos[0] = float(position[0]) + math.cos(yaw)*local_x - math.sin(yaw)*local_y
                qpos[1] = float(position[1]) + math.sin(yaw)*local_x + math.cos(yaw)*local_y
                qpos[3:7] = _multiply_quaternion(yaw_quaternion, qpos[3:7])
                key.set("qpos", _format(qpos))
        tree.write(robot_xml, encoding="utf-8", xml_declaration=True)

    @staticmethod
    def cleanup(scene_path: Path) -> None:
        shutil.rmtree(scene_path.parent, ignore_errors=True)
