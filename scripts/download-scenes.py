#!/usr/bin/env python3
"""Download pinned scene sources and rebuild the four environment packages."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import shutil
import subprocess
import tarfile
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENTS = ROOT / "assets/environments"
DATASET_REVISION = "5425b6ac0ee7d5043af75f9ec78494cf5c94f3ca"
COURSE_REVISION = "30e74dc507bec7a642a8c98be26081f2c6f0822d"
HOUSE_HASHES = {
    "185": "053f63909816d8ea985eba4133620092baaf1825d4fbe3f69775552d948e3054",
    "186": "ffa2a7ceff7d7cb04325967624a8acfa99295aa9d924862cca8f9ba189266501",
}
COURSE_HASHES = {
    "stairs": "d3e43ab7e11bf7a7320c45a5cbd42fd5991bca813452cb026da28cc8cf5ce922",
    "race_track": "b5f5d9995429f71e38b7b37e98588c7ac8eb035a87ea1a62931fa91d0e5cc79c",
    "LICENSE": "7e4c757f79c318aadd9ce07209811779779f635ffe3a286181748cc63e1bac36",
}
COURSE_BASE = f"https://raw.githubusercontent.com/wty-yy/go2_rl_gym/{COURSE_REVISION}"
spec = importlib.util.spec_from_file_location("prepare_scenesmith", Path(__file__).with_name("prepare-scenesmith.py"))
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def download(url: str, destination: Path, expected_hash: str | None = None) -> str:
    """Download atomically, verify pinned archive hashes, and reuse valid cache."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        digest = converter.file_digest(destination)
        if expected_hash is None or digest == expected_hash:
            return digest
        raise RuntimeError(f"Cached source checksum mismatch: {destination}")
    temporary = destination.with_suffix(destination.suffix + ".part")
    subprocess.run([
        "curl", "-fsSL", "--retry", "3", "--connect-timeout", "30",
        "--max-time", "1800", "-o", str(temporary), url,
    ], check=True)
    digest = converter.file_digest(temporary)
    if expected_hash and digest != expected_hash:
        raise RuntimeError(f"Downloaded source checksum mismatch: {url}")
    temporary.replace(destination)
    return digest


def import_house(number: str, cache: Path, endpoint: str) -> None:
    """Verify and extract only the genuine dataset MuJoCo export, then convert."""
    archive = cache / f"go2-house-{number}.tar"
    source_path = f"House/scene_{number}.tar"
    url = f"{endpoint.rstrip('/')}/datasets/nepfaff/scenesmith-example-scenes/resolve/{DATASET_REVISION}/{source_path}"
    digest = download(url, archive, HOUSE_HASHES[number])
    scene_id = f"scenesmith_house_{number}"
    output = ENVIRONMENTS / scene_id
    with tempfile.TemporaryDirectory(prefix=f"go2-house-{number}-") as temporary:
        extracted = Path(temporary)
        with tarfile.open(archive) as source:
            for member in source:
                path = PurePosixPath(member.name)
                if path.is_absolute() or ".." in path.parts:
                    raise ValueError(f"Unsafe archive path: {member.name}")
                if not path.parts or path.parts[0] != "mujoco" or not member.isfile():
                    continue
                destination = extracted / path
                destination.parent.mkdir(parents=True, exist_ok=True)
                with source.extractfile(member) as incoming, destination.open("wb") as outgoing:
                    shutil.copyfileobj(incoming, outgoing)
        report = converter.build_package(
            extracted / "mujoco", output, 0.50, scene_id,
            f"scene_{number}.tar", "House", None,
            spawn_yaw=math.pi if number == "186" else 0,
        )
    if report["roomCount"] < 2:
        raise RuntimeError(f"{scene_id}: expected a genuine multiroom house")
    write_json(output / "provenance.json", {
        "repository": "nepfaff/scenesmith-example-scenes",
        "revision": DATASET_REVISION, "path": source_path, "sha256": digest,
        "url": url.replace(endpoint.rstrip('/'), "https://huggingface.co", 1),
        "license": "Apache-2.0", "converter": "scripts/prepare-scenesmith.py",
        "roomCount": report["roomCount"], "roomTypes": report["roomTypes"],
        "spawnYawRadians": report["spawn"]["yaw"],
    })
    print(f"{scene_id}: {report['roomCount']} rooms, {report['textureCount']} textures, spawn {report['spawn']['position']}", flush=True)


def import_course(name: str, cache: Path) -> None:
    """Keep upstream course shapes/materials and separate visual/collision groups."""
    scene_id = "go2_rl_stairs" if name == "stairs" else "go2_rl_track"
    output = ENVIRONMENTS / scene_id
    output.mkdir(parents=True, exist_ok=True)
    source_path = f"resources/robots/go2/{name}.xml"
    source = cache / f"{COURSE_REVISION}-{name}.xml"
    digest = download(f"{COURSE_BASE}/{source_path}", source, COURSE_HASHES[name])
    root = ET.parse(source).getroot()
    for include in list(root.findall("include")):
        if include.get("file") != "go2.xml":
            raise ValueError(f"Unexpected upstream include: {include.attrib}")
        root.remove(include)
    root.set("model", scene_id)
    worldbody = root.find("worldbody")
    obstacles = []
    original = list(worldbody.findall("geom"))
    for index, geom in enumerate(original):
        name_prefix = f"{scene_id}_{index}"
        visual = copy.deepcopy(geom)
        visual.set("name", f"{name_prefix}_visual")
        visual.set("group", "1")
        visual.set("contype", "0")
        visual.set("conaffinity", "0")
        worldbody.append(visual)
        geom.set("name", f"{name_prefix}_collision")
        geom.set("group", "3")
        geom.set("contype", "1")
        geom.set("conaffinity", "1")
        geom.attrib.pop("material", None)
        geom.set("rgba", "0.2 0.8 0.3 0.001")
        if geom.get("type") == "box":
            size = converter.vector(geom.get("size"), 3, (0, 0, 0))
            obstacles.append(converter.world_aabb(
                [-value for value in size], size,
                converter.vector(geom.get("pos"), 3, (0, 0, 0)),
                converter.vector(geom.get("quat"), 4, (1, 0, 0, 0)),
            ))
    # The upstream starting plane at x=-0.5 faces the first obstacle at +X.
    x, y = -0.5, 0.0
    clearance = min(math.hypot(max(a[0] - x, 0, x - b[0]), max(a[1] - y, 0, y - b[1])) for a, b in obstacles)
    if clearance < 0.58:
        raise RuntimeError(f"{scene_id}: insufficient Go2 spawn clearance")
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output / "scene.xml", encoding="unicode")
    write_json(output / "index.json", ["scene.xml"])
    write_json(output / "spawn.json", {
        "schemaVersion": 1, "sceneId": scene_id, "position": [x, y], "yaw": 0,
        "robotRadiusMeters": 0.5, "clearanceMeters": round(clearance, 4),
        "method": "upstream-start-plane-static-aabb-clearance",
    })
    license_path = cache / f"{COURSE_REVISION}-LICENSE"
    download(f"{COURSE_BASE}/LICENSE", license_path, COURSE_HASHES["LICENSE"])
    shutil.copy2(license_path, output / "LICENSE.upstream")
    write_json(output / "provenance.json", {
        "repository": "wty-yy/go2_rl_gym", "revision": COURSE_REVISION,
        "path": source_path, "sha256": digest, "url": f"{COURSE_BASE}/{source_path}",
        "license": "MIT; upstream notice also retains Unitree BSD-3-Clause",
        "upstreamGeomCount": len(original), "converter": "scripts/download-scenes.py",
        "modifications": ["Remove robot include", "Split visuals group 1 and static colliders group 3; preserve geometry and contact parameters"],
    })
    (output / "NOTICE.md").write_text(
        f"# {scene_id}\n\nDerived from [{source_path}]({COURSE_BASE}/{source_path}).\n\n"
        "Copyright (c) 2025 Wu Tianyang. See LICENSE.upstream for MIT terms and\n"
        "the retained Unitree BSD-3-Clause notice. Import removes the robot include\n"
        "and separates visuals from identical static colliders; upstream positions,\n"
        "sizes, rotations, textures, materials and contact parameters are retained.\n",
        encoding="utf-8",
    )
    print(f"{scene_id}: {len(original)} upstream geoms, spawn clearance {clearance:.3f} m", flush=True)


def main() -> None:
    """Rebuild selected pinned environments without accessing robot assets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=Path(tempfile.gettempdir()) / "go2-scene-sources")
    parser.add_argument("--endpoint", default="https://huggingface.co", help="Dataset host; hf-mirror.com is supported with the same SHA-256 checks")
    parser.add_argument("--only", choices=("all", "houses", "courses"), default="all")
    args = parser.parse_args()
    if args.only in ("all", "houses"):
        download(
            "https://www.apache.org/licenses/LICENSE-2.0.txt", ENVIRONMENTS / "LICENSE.scenesmith",
            "cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30",
        )
        for number in HOUSE_HASHES:
            import_house(number, args.cache, args.endpoint)
    if args.only in ("all", "courses"):
        for name in ("stairs", "race_track"):
            import_course(name, args.cache)


if __name__ == "__main__":
    main()
