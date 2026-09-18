#!/usr/bin/env python3
"""Validate imported XML, indexed assets, world collision groups and Go2 spawn."""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {"scenesmith_house_185", "scenesmith_house_186", "go2_rl_stairs", "go2_rl_track"}


def validate_environment(entry: dict, cache: Path | None) -> dict:
    """Compile one static scene and probe its floor and reserved robot volume."""
    package = (ROOT / entry["xmlPath"]).parent
    spawn = json.loads((ROOT / entry["spawnPath"]).read_text())
    assert spawn["sceneId"] == entry["id"]
    yaw = spawn["yaw"]
    assert math.isfinite(yaw)
    if entry["id"] == "scenesmith_house_186":
        assert math.isclose(yaw, math.pi), "House 186 must initially face the interior"
    assert spawn["robotRadiusMeters"] >= 0.5
    assert spawn["clearanceMeters"] >= spawn["robotRadiusMeters"]
    files = json.loads((ROOT / entry["filesPath"]).read_text())
    assert files and len(set(files)) == len(files)
    for name in files:
        assert not Path(name).is_absolute() and ".." not in Path(name).parts
        assert (package / name).is_file(), name
    model = mujoco.MjModel.from_xml_path(str(package / "scene.xml"))
    assert model.nq == model.nv == 0, "Environment must remain static"
    colliders = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
    assert np.all(model.geom_group[colliders] == 3), "World colliders must use group 3"
    assert np.all(model.geom_group[~colliders] == 1), "Visual geometry must use group 1"
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    groups = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
    geom_id = np.array([-1], dtype=np.int32)
    x, y = spawn["position"][:2]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    # This 0.84 x 0.44 m rectangle fits within the conservative 0.50 m disk.
    for dx in np.linspace(-0.42, 0.42, 7):
        for dy in np.linspace(-0.22, 0.22, 5):
            position = np.array([x + cosine * dx - sine * dy, y + sine * dx + cosine * dy, 0.8])
            distance = mujoco.mj_ray(model, data, position, np.array([0., 0., -1.]), groups, True, -1, geom_id)
            assert abs(distance - 0.8) < 0.015, f"Unsupported/obstructed footprint at {dx}, {dy}: {distance}"
    root = ET.parse(package / "scene.xml").getroot()
    compiler = root.find("compiler")
    if compiler is not None:
        for key in ("meshdir", "texturedir"):
            compiler.set(key, str(package / compiler.get(key, "")))
    worldbody = root.find("worldbody")
    body = ET.SubElement(worldbody, "body", {
        "name": "validation_go2_footprint", "pos": f"{x} {y} 0.41",
        "quat": f"{math.cos(yaw / 2)} 0 0 {math.sin(yaw / 2)}",
    })
    ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", {"name": "validation_go2_volume", "type": "box", "size": "0.42 0.22 0.39", "group": "4"})
    probe = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    probe_data = mujoco.MjData(probe)
    mujoco.mj_forward(probe, probe_data)
    assert probe_data.ncon == 0, f"Spawn intersects {probe_data.ncon} world colliders"
    provenance = json.loads((package / "provenance.json").read_text())
    if entry["id"].startswith("scenesmith"):
        assert provenance["roomCount"] >= 2 and provenance["path"] != "House/scene_187.tar"
        assert model.nmesh > 0 and model.ntex > 2
    elif cache is not None:
        source = cache / f"{provenance['revision']}-{Path(provenance['path']).name}"
        upstream = ET.parse(source).getroot()
        original_geoms = upstream.findall("worldbody/geom")
        visuals = [g for g in worldbody.findall("geom") if g.get("group") == "1"]
        collisions = [g for g in worldbody.findall("geom") if g.get("group") == "3"]
        assert len(visuals) == len(collisions) == len(original_geoms)
        for original, visual, collision in zip(original_geoms, visuals, collisions):
            for key, value in original.attrib.items():
                if key != "name":
                    assert visual.get(key) == value, (entry["id"], key)
                if key not in ("name", "material", "rgba"):
                    assert collision.get(key) == value, (entry["id"], key)
        assert [(item.tag, item.attrib) for item in upstream.find("asset")] == [
            (item.tag, item.attrib) for item in root.find("asset")
        ], "Upstream asset definitions changed"
    return {"id": entry["id"], "geoms": model.ngeom, "meshes": model.nmesh,
            "textures": model.ntex, "worldColliders": int(colliders.sum()),
            "spawnContacts": probe_data.ncon, "supportedFootprintPoints": 35,
            "clearanceMeters": spawn["clearanceMeters"]}


def main() -> None:
    """Validate exactly four selectable environments and print a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, help="Also compare course geometry against cached pinned upstream XML")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "assets/environments/manifest.json").read_text())
    assert manifest["defaultEnvironment"] == "scenesmith_house_185"
    assert len(manifest["environments"]) == 4
    assert {entry["id"] for entry in manifest["environments"]} == EXPECTED
    assert all(entry["visualMode"] == "mesh" for entry in manifest["environments"])
    results = [validate_environment(entry, args.cache) for entry in manifest["environments"]]
    print(json.dumps({"mujocoVersion": mujoco.__version__, "environments": results}, indent=2))


if __name__ == "__main__":
    main()
