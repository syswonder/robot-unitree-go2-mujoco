#!/usr/bin/env python3
"""Convert a SceneSmith MuJoCo export into a browser-friendly static package."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def file_digest(path: Path) -> str:
    """Hash a file without loading it entirely into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def optimize_obj(source: Path, destination: Path) -> dict[str, int]:
    """Remove duplicate OBJ attributes while preserving every face and UV exactly."""
    lines = source.read_text(encoding="utf-8", errors="ignore").splitlines(keepends=True)
    kinds = ("v", "vt", "vn")
    mappings: dict[str, list[int]] = {kind: [0] for kind in kinds}
    representatives: dict[str, set[int]] = {kind: set() for kind in kinds}
    values: dict[str, dict[str, int]] = {kind: {} for kind in kinds}

    for line in lines:
        parts = line.split()
        if not parts or parts[0] not in mappings:
            continue
        kind = parts[0]
        value = " ".join(parts[1:])
        mapped = values[kind].get(value)
        if mapped is None:
            mapped = len(values[kind]) + 1
            values[kind][value] = mapped
            representatives[kind].add(len(mappings[kind]))
        mappings[kind].append(mapped)

    original_indices = {kind: 0 for kind in kinds}
    output_lines = []
    for line in lines:
        parts = line.split()
        if parts and parts[0] in mappings:
            kind = parts[0]
            original_indices[kind] += 1
            if original_indices[kind] in representatives[kind]:
                output_lines.append(line)
            continue
        if parts and parts[0] == "f":
            remapped = []
            for vertex in parts[1:]:
                indices = vertex.split("/")
                for offset, kind in enumerate(kinds):
                    if offset >= len(indices) or not indices[offset]:
                        continue
                    index = int(indices[offset])
                    if index <= 0:
                        raise ValueError(f"Negative OBJ index is not supported: {source}")
                    indices[offset] = str(mappings[kind][index])
                remapped.append("/".join(indices))
            output_lines.append("f " + " ".join(remapped) + "\n")
        else:
            output_lines.append(line)

    destination.write_text("".join(output_lines), encoding="utf-8")
    return {
        "verticesBefore": len(mappings["v"]) - 1,
        "verticesAfter": len(values["v"]),
        "texcoordsBefore": len(mappings["vt"]) - 1,
        "texcoordsAfter": len(values["vt"]),
    }


def deduplicate_assets(root: ET.Element, source_meshes: Path) -> dict[str, int]:
    """Merge byte-identical assets and equivalent MJCF definitions."""
    asset = root.find("asset")
    if asset is None:
        return {"meshes": 0, "textures": 0, "materials": 0}

    removed = {"meshes": 0, "textures": 0, "materials": 0}

    def merge(tag: str, reference_attribute: str, report_key: str, hash_file: bool) -> None:
        """Replace duplicate definitions and redirect their consumers."""
        canonical_by_signature: dict[tuple, ET.Element] = {}
        for element in list(asset.findall(tag)):
            name = element.get("name")
            file_name = element.get("file")
            attributes = dict(element.attrib)
            attributes.pop("name", None)
            if hash_file and file_name:
                attributes["file"] = file_digest(source_meshes / file_name)
            signature = tuple(sorted(attributes.items()))
            canonical = canonical_by_signature.get(signature)
            if canonical is None:
                canonical_by_signature[signature] = element
                continue
            canonical_name = canonical.get("name")
            for consumer in root.iter():
                if consumer.get(reference_attribute) == name:
                    consumer.set(reference_attribute, canonical_name)
            asset.remove(element)
            removed[report_key] += 1

    merge("mesh", "mesh", "meshes", True)
    merge("texture", "texture", "textures", True)
    merge("material", "material", "materials", False)
    return removed


def vector(text: str | None, length: int, default: tuple[float, ...]) -> list[float]:
    values = [float(value) for value in text.split()] if text else list(default)
    if len(values) != length:
        raise ValueError(f"Expected {length} values, got {values}")
    return values


def quaternion_multiply(left: list[float], right: list[float]) -> list[float]:
    """Compose rotations expressed as scalar-first quaternions."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def quaternion_rotate(quaternion: list[float], point: list[float]) -> list[float]:
    """Rotate a point using a unit scalar-first quaternion."""
    w, x, y, z = quaternion
    px, py, pz = point
    tx = 2 * (y * pz - z * py)
    ty = 2 * (z * px - x * pz)
    tz = 2 * (x * py - y * px)
    return [
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    ]


def compose(
    parent_position: list[float],
    parent_quaternion: list[float],
    local_position: list[float],
    local_quaternion: list[float],
) -> tuple[list[float], list[float]]:
    offset = quaternion_rotate(parent_quaternion, local_position)
    return (
        [parent_position[index] + offset[index] for index in range(3)],
        quaternion_multiply(parent_quaternion, local_quaternion),
    )


def obj_bounds(path: Path, scale: list[float]) -> tuple[list[float], list[float]]:
    """Read scaled vertex bounds for an OBJ mesh."""
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            values = [float(value) for value in line.split()[1:4]]
            values = [values[index] * scale[index] for index in range(3)]
            for index, value in enumerate(values):
                minimum[index] = min(minimum[index], value)
                maximum[index] = max(maximum[index], value)
    if not all(math.isfinite(value) for value in minimum + maximum):
        raise ValueError(f"OBJ has no vertices: {path}")
    return minimum, maximum


def bounds_corners(minimum: list[float], maximum: list[float]) -> list[list[float]]:
    return [
        [x, y, z]
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
        for z in (minimum[2], maximum[2])
    ]


def fmt(values: list[float]) -> str:
    return " ".join(f"{value:.7g}" for value in values)


def relative_visual_bounds(
    top_body: ET.Element,
    mesh_definitions: dict[str, ET.Element],
    mesh_directory: Path,
    bounds_cache: dict[tuple[str, tuple[float, ...]], tuple[list[float], list[float]]],
) -> tuple[list[float], list[float]]:
    """Bound the visual meshes of an object in its top body's frame."""
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]

    def visit(body: ET.Element, body_position: list[float], body_quaternion: list[float]) -> None:
        """Accumulate mesh bounds through nested body transforms."""
        for geom in body.findall("geom"):
            mesh_name = geom.get("mesh")
            if not mesh_name or "visual" not in (geom.get("name") or ""):
                continue
            mesh = mesh_definitions[mesh_name]
            file_name = mesh.get("file")
            scale = vector(mesh.get("scale"), 3, (1, 1, 1))
            key = (file_name, tuple(scale))
            if key not in bounds_cache:
                bounds_cache[key] = obj_bounds(mesh_directory / file_name, scale)
            mesh_minimum, mesh_maximum = bounds_cache[key]
            geom_position = vector(geom.get("pos"), 3, (0, 0, 0))
            geom_quaternion = vector(geom.get("quat"), 4, (1, 0, 0, 0))
            position, quaternion = compose(body_position, body_quaternion, geom_position, geom_quaternion)
            for corner in bounds_corners(mesh_minimum, mesh_maximum):
                rotated = quaternion_rotate(quaternion, corner)
                point = [position[index] + rotated[index] for index in range(3)]
                for index, value in enumerate(point):
                    minimum[index] = min(minimum[index], value)
                    maximum[index] = max(maximum[index], value)
        for child in body.findall("body"):
            position, quaternion = compose(
                body_position,
                body_quaternion,
                vector(child.get("pos"), 3, (0, 0, 0)),
                vector(child.get("quat"), 4, (1, 0, 0, 0)),
            )
            visit(child, position, quaternion)

    visit(top_body, [0, 0, 0], [1, 0, 0, 0])
    if not all(math.isfinite(value) for value in minimum + maximum):
        raise ValueError(f"No visual mesh found for body {top_body.get('name')}")
    return minimum, maximum


def remove_descendants(root: ET.Element, tags: set[str]) -> int:
    """Remove matching descendants and return the number removed."""
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag in tags:
                parent.remove(child)
                removed += 1
    return removed


def world_aabb(
    local_minimum: list[float],
    local_maximum: list[float],
    position: list[float],
    quaternion: list[float],
) -> tuple[list[float], list[float]]:
    """Transform a local box into conservative world-aligned bounds."""
    points = []
    for corner in bounds_corners(local_minimum, local_maximum):
        rotated = quaternion_rotate(quaternion, corner)
        points.append([position[index] + rotated[index] for index in range(3)])
    return (
        [min(point[index] for point in points) for index in range(3)],
        [max(point[index] for point in points) for index in range(3)],
    )


def find_spawn(
    floors: list[tuple[list[float], list[float]]],
    obstacles: list[tuple[list[float], list[float]]],
    robot_radius: float,
) -> tuple[list[float], float]:
    """Find floor-supported XY space with clearance for a circular footprint."""
    if not floors:
        raise RuntimeError("Could not locate any SceneSmith floor collision")
    step = 0.10
    margin = robot_radius + 0.08
    best_position = None
    best_clearance = -math.inf
    overall_minimum = [min(floor[0][axis] for floor in floors) for axis in range(2)]
    overall_maximum = [max(floor[1][axis] for floor in floors) for axis in range(2)]
    x = overall_minimum[0] + margin
    while x <= overall_maximum[0] - margin:
        y = overall_minimum[1] + margin
        while y <= overall_maximum[1] - margin:
            containing_floors = [
                floor for floor in floors
                if floor[0][0] + margin <= x <= floor[1][0] - margin
                and floor[0][1] + margin <= y <= floor[1][1] - margin
            ]
            if not containing_floors:
                y += step
                continue
            wall_clearance = max(
                min(
                    x - minimum[0], maximum[0] - x,
                    y - minimum[1], maximum[1] - y,
                )
                for minimum, maximum in containing_floors
            )
            clearance = wall_clearance
            for minimum, maximum in obstacles:
                dx = max(minimum[0] - x, 0, x - maximum[0])
                dy = max(minimum[1] - y, 0, y - maximum[1])
                if dx == 0 and dy == 0:
                    clearance = -1
                    break
                clearance = min(clearance, math.hypot(dx, dy))
            if clearance > best_clearance:
                best_clearance = clearance
                best_position = [round(x, 3), round(y, 3)]
            y += step
        x += step
    if best_position is None or best_clearance < robot_radius:
        raise RuntimeError(f"No collision-free spawn found (best clearance={best_clearance:.3f})")
    return best_position, best_clearance


def subtract_intervals(
    interval: tuple[float, float], cuts: list[tuple[float, float]], tolerance: float = 1e-5
) -> list[tuple[float, float]]:
    """Subtract covered portions from a boundary interval."""
    remaining = [interval]
    for cut_start, cut_end in cuts:
        updated = []
        for start, end in remaining:
            if cut_end <= start + tolerance or cut_start >= end - tolerance:
                updated.append((start, end))
                continue
            if cut_start > start + tolerance:
                updated.append((start, min(cut_start, end)))
            if cut_end < end - tolerance:
                updated.append((max(cut_end, start), end))
        remaining = updated
    return remaining


def add_floor_perimeter_guards(
    worldbody: ET.Element,
    floors: list[tuple[list[float], list[float]]],
    wall_height: float = 3.0,
) -> int:
    """Seal only the exposed boundary of an axis-aligned floor union."""
    tolerance = 1e-4
    segments: list[tuple[str, float, float, float]] = []
    for index, (minimum, maximum) in enumerate(floors):
        edges = (
            ("vertical", minimum[0], minimum[1], maximum[1], "left"),
            ("vertical", maximum[0], minimum[1], maximum[1], "right"),
            ("horizontal", minimum[1], minimum[0], maximum[0], "bottom"),
            ("horizontal", maximum[1], minimum[0], maximum[0], "top"),
        )
        for orientation, coordinate, start, end, side in edges:
            cuts = []
            for other_index, (other_minimum, other_maximum) in enumerate(floors):
                if other_index == index:
                    continue
                if orientation == "vertical":
                    adjacent = (
                        abs(other_maximum[0] - coordinate) < tolerance if side == "left"
                        else abs(other_minimum[0] - coordinate) < tolerance
                    )
                    if adjacent:
                        cuts.append((other_minimum[1], other_maximum[1]))
                else:
                    adjacent = (
                        abs(other_maximum[1] - coordinate) < tolerance if side == "bottom"
                        else abs(other_minimum[1] - coordinate) < tolerance
                    )
                    if adjacent:
                        cuts.append((other_minimum[0], other_maximum[0]))
            for exposed_start, exposed_end in subtract_intervals((start, end), cuts):
                if exposed_end - exposed_start > tolerance:
                    segments.append((orientation, coordinate, exposed_start, exposed_end))

    for index, (orientation, coordinate, start, end) in enumerate(segments):
        midpoint = (start + end) * 0.5
        half_length = (end - start) * 0.5
        if orientation == "vertical":
            position = [coordinate, midpoint, wall_height * 0.5]
            size = [0.04, half_length, wall_height * 0.5]
        else:
            position = [midpoint, coordinate, wall_height * 0.5]
            size = [half_length, 0.04, wall_height * 0.5]
        worldbody.append(ET.Element("geom", {
            "name": f"generated_perimeter_guard_{index}",
            "type": "box",
            "pos": fmt(position),
            "size": fmt(size),
            "group": "3",
            "contype": "1",
            "conaffinity": "1",
            "rgba": "0.2 0.8 0.3 0.001",
            "friction": "0.8 0.02 0.002",
        }))
    return len(segments)


def build_package(
    source: Path,
    output: Path,
    robot_radius: float,
    scene_id: str,
    source_archive: str,
    source_subset: str,
    interactive_config: Path | None,
    spawn_yaw: float = 0,
) -> dict:
    """Export textured static geometry, a file index, provenance and safe spawn."""
    if not math.isfinite(robot_radius) or robot_radius <= 0:
        raise ValueError("Robot radius must be finite and positive")
    if not math.isfinite(spawn_yaw):
        raise ValueError("Spawn yaw must be finite")
    source_xml = source / "scene.xml"
    source_meshes = source / "meshes"
    root = ET.parse(source_xml).getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", "meshes")
    compiler.set("texturedir", "meshes/scenesmith")

    asset = root.find("asset")
    worldbody = root.find("worldbody")
    if asset is None or worldbody is None:
        raise ValueError("SceneSmith export is missing asset or worldbody")
    mesh_definitions = {mesh.get("name"): mesh for mesh in asset.findall("mesh")}
    bounds_cache = {}
    static_boxes = []
    original_collision_geoms = 0
    interactive_spec = {"objects": []}
    if interactive_config is not None:
        interactive_spec = json.loads(interactive_config.read_text(encoding="utf-8"))
    interactive_by_source = {
        item["sourceBody"]: item for item in interactive_spec.get("objects", [])
    }
    objects_root = ET.Element("mujoco", {"model": f"{scene_id}_objects"})
    objects_worldbody = ET.SubElement(objects_root, "worldbody")
    dynamic_objects = []

    ground = worldbody.find("geom[@name='ground_plane']")
    if ground is not None:
        worldbody.remove(ground)

    floors = []
    room_obstacles = []
    room_bodies = [
        body for body in worldbody.findall("body")
        if body.get("name", "").startswith("room_geometry_")
    ]
    if not room_bodies:
        raise RuntimeError("Could not locate any SceneSmith room geometry")
    room_types = []
    for room_body in room_bodies:
        room_types.append(
            room_body.get("name", "room_geometry_unknown")
            .removeprefix("room_geometry_")
            .split("_room_geometry")[0]
        )
        room_position = vector(room_body.get("pos"), 3, (0, 0, 0))
        room_quaternion = vector(room_body.get("quat"), 4, (1, 0, 0, 0))

        def visit_room(body: ET.Element, position: list[float], quaternion: list[float]) -> None:
            """Retain room colliders and collect floor and wall bounds for spawn."""
            for geom in body.findall("geom"):
                name = geom.get("name", "")
                if "collision" in name:
                    geom.set("group", "3")
                    geom.set("contype", "1")
                    geom.set("conaffinity", "1")
                    geom.set("rgba", "0.2 0.8 0.3 0.001")
                if "collision" in name and geom.get("type") == "box":
                    center, orientation = compose(
                        position,
                        quaternion,
                        vector(geom.get("pos"), 3, (0, 0, 0)),
                        vector(geom.get("quat"), 4, (1, 0, 0, 0)),
                    )
                    size = vector(geom.get("size"), 3, (0, 0, 0))
                    bounds = world_aabb(
                        [-value for value in size], size, center, orientation
                    )
                    if "floor_collision" in name:
                        floors.append(bounds)
                    elif bounds[0][2] < 0.8 and bounds[1][2] > 0.02:
                        room_obstacles.append(bounds)
            for child in body.findall("body"):
                child_position, child_quaternion = compose(
                    position,
                    quaternion,
                    vector(child.get("pos"), 3, (0, 0, 0)),
                    vector(child.get("quat"), 4, (1, 0, 0, 0)),
                )
                visit_room(child, child_position, child_quaternion)

        visit_room(room_body, room_position, room_quaternion)

    for top_body in list(worldbody.findall("body")):
        if top_body in room_bodies:
            continue
        name = top_body.get("name", "object")
        local_minimum, local_maximum = relative_visual_bounds(
            top_body, mesh_definitions, source_meshes, bounds_cache
        )
        original_collision_geoms += sum(
            1 for geom in top_body.iter("geom") if "collision" in geom.get("name", "")
        )
        remove_descendants(top_body, {"joint", "freejoint", "inertial"})
        for parent in top_body.iter():
            for geom in list(parent.findall("geom")):
                if "collision" in geom.get("name", ""):
                    parent.remove(geom)
        center = [(local_minimum[index] + local_maximum[index]) * 0.5 for index in range(3)]
        size = [max((local_maximum[index] - local_minimum[index]) * 0.5, 0.005) for index in range(3)]
        interactive = interactive_by_source.get(name)
        if interactive is not None:
            task_name = interactive["name"]
            top_body.set("name", task_name)
            if "position" in interactive:
                top_body.set("pos", fmt(interactive["position"]))
            if "quaternion" in interactive:
                top_body.set("quat", fmt(interactive["quaternion"]))
            top_body.insert(0, ET.Element("freejoint", {"name": f"{task_name}_freejoint"}))
            top_body.append(ET.Element("geom", {
                "name": f"{task_name}_collision",
                "type": "box",
                "pos": fmt(center),
                "size": fmt(size),
                "group": "3",
                "contype": "1",
                "conaffinity": "1",
                "rgba": "0.2 0.8 0.3 0.001",
                "friction": "0.9 0.03 0.003",
                "density": str(interactive.get("density", 350)),
            }))
            worldbody.remove(top_body)
            objects_worldbody.append(top_body)
            dynamic_objects.append({
                "name": task_name,
                "sourceBody": name,
                "position": vector(top_body.get("pos"), 3, (0, 0, 0)),
                "size": [round(value * 2, 5) for value in size],
            })
            continue
        top_body.append(ET.Element("geom", {
            "name": f"{name}_static_collision",
            "type": "box",
            "pos": fmt(center),
            "size": fmt(size),
            "group": "3",
            "contype": "1",
            "conaffinity": "1",
            "rgba": "0.2 0.8 0.3 0.001",
            "friction": "0.8 0.02 0.002",
        }))
        position = vector(top_body.get("pos"), 3, (0, 0, 0))
        quaternion = vector(top_body.get("quat"), 4, (1, 0, 0, 0))
        static_boxes.append(world_aabb(local_minimum, local_maximum, position, quaternion))

    for geom in worldbody.iter("geom"):
        if "visual" in geom.get("name", ""):
            geom.set("group", "1")
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
    for geom in objects_worldbody.iter("geom"):
        if "visual" in geom.get("name", ""):
            geom.set("group", "1")
            geom.set("contype", "0")
            geom.set("conaffinity", "0")

    original_mesh_count = len(asset.findall("mesh"))
    original_texture_count = len([item for item in asset.findall("texture") if item.get("file")])
    deduplicated = deduplicate_assets(root, source_meshes)

    referenced_mesh_names = {
        geom.get("mesh")
        for container in (worldbody, objects_worldbody)
        for geom in container.iter("geom")
        if geom.get("mesh")
    }
    for mesh in list(asset.findall("mesh")):
        if mesh.get("name") not in referenced_mesh_names:
            asset.remove(mesh)
        else:
            mesh.set("file", f"scenesmith/{mesh.get('file')}")

    perimeter_guard_count = add_floor_perimeter_guards(worldbody, floors)
    spawn_position, spawn_clearance = find_spawn(
        floors, static_boxes + room_obstacles, robot_radius
    )

    used_files = {Path(mesh.get("file")).name for mesh in asset.findall("mesh")}
    used_files.update(Path(texture.get("file")).name for texture in asset.findall("texture") if texture.get("file"))
    output_meshes = output / "meshes" / "scenesmith"
    if output_meshes.exists():
        shutil.rmtree(output_meshes)
    output_meshes.mkdir(parents=True, exist_ok=True)
    obj_optimization = {
        "verticesBefore": 0,
        "verticesAfter": 0,
        "texcoordsBefore": 0,
        "texcoordsAfter": 0,
    }
    for file_name in sorted(used_files):
        source_file = source_meshes / file_name
        destination_file = output_meshes / file_name
        if source_file.suffix.lower() == ".obj":
            result = optimize_obj(source_file, destination_file)
            for key, value in result.items():
                obj_optimization[key] += value
        else:
            shutil.copy2(source_file, destination_file)

    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output / "scene.xml", encoding="unicode")
    if dynamic_objects:
        ET.indent(objects_root, space="  ")
        ET.ElementTree(objects_root).write(output / "objects.xml", encoding="unicode")
    package_files = [f"meshes/scenesmith/{name}" for name in sorted(used_files)]
    (output / "index.json").write_text(json.dumps(package_files, indent=2) + "\n", encoding="utf-8")
    spawn = {
        "schemaVersion": 1,
        "position": spawn_position,
        "yaw": spawn_yaw,
        "clearanceMeters": round(spawn_clearance, 4),
        "robotRadiusMeters": robot_radius,
        "method": "static-aabb-distance-field",
        "sceneId": scene_id,
    }
    (output / "spawn.json").write_text(json.dumps(spawn, indent=2) + "\n", encoding="utf-8")
    notice = f"""# SceneSmith {source_archive.removesuffix('.tar')}

This runtime package is derived from `{source_subset}/{source_archive}` in
`nepfaff/scenesmith-example-scenes`.

- Source: https://huggingface.co/datasets/nepfaff/scenesmith-example-scenes
- SceneSmith project: https://github.com/nepfaff/scenesmith
- License: Apache-2.0 (as declared by the source dataset)

`scripts/prepare-scenesmith.py` removes furniture free joints, replaces the
generated convex decomposition with one static collision box per object, keeps
the textured visual meshes, and computes a collision-free robot spawn.
"""
    (output / "NOTICE.md").write_text(notice, encoding="utf-8")
    report = {
        "source": f"nepfaff/scenesmith-example-scenes {source_subset}/{source_archive}",
        "sourceLicense": "Apache-2.0",
        "roomTypes": room_types,
        "roomCount": len(room_bodies),
        "furnitureMode": "static",
        "visualMeshCount": len(asset.findall("mesh")),
        "textureCount": len([item for item in asset.findall("texture") if item.get("file")]),
        "assetDeduplication": {
            "meshDefinitionsBefore": original_mesh_count,
            "textureDefinitionsBefore": original_texture_count,
            "removed": deduplicated,
        },
        "objOptimization": obj_optimization,
        "originalFurnitureCollisionGeomCount": original_collision_geoms,
        "staticFurnitureCollisionBoxCount": len(static_boxes),
        "floorCollisionCount": len(floors),
        "perimeterGuardCount": perimeter_guard_count,
        "runtimeAssetCount": len(used_files),
        "dynamicObjects": dynamic_objects,
        "taskApproach": interactive_spec.get("approach"),
        "spawn": spawn,
    }
    (output / "generation_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    """Parse import options and write the converted environment package."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True, help="Extracted SceneSmith mujoco directory")
    parser.add_argument("--output", type=Path, required=True, help="Runtime environment package directory")
    parser.add_argument("--scene-id", required=True, help="Environment ID written to spawn metadata")
    parser.add_argument("--source-archive", required=True, help="Dataset archive name, e.g. scene_036.tar")
    parser.add_argument("--source-subset", choices=("Room", "House"), default="Room")
    parser.add_argument(
        "--interactive-config",
        type=Path,
        help="JSON mapping SceneSmith bodies to environment-level task objects",
    )
    parser.add_argument("--robot-radius", type=float, default=0.50)
    parser.add_argument("--spawn-yaw", type=float, default=0, help="Authored spawn heading in radians")
    args = parser.parse_args()
    report = build_package(
        args.source.resolve(),
        args.output.resolve(),
        args.robot_radius,
        args.scene_id,
        args.source_archive,
        args.source_subset,
        args.interactive_config.resolve() if args.interactive_config else None,
        spawn_yaw=args.spawn_yaw,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
