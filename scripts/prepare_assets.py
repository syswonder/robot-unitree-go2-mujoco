#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Download immutable source/model files; never run an installer or SDK."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def prepare(source=None, lock_name="assets.lock.json"):
    """SHA256-check model files and Git-blob-check the pinned policy assets."""
    lock = json.loads((ROOT / lock_name).read_text())
    target = ROOT / ".runtime/assets"
    for entry in lock["files"]:
        dest = target / entry["destination"]
        if dest.exists():
            data = dest.read_bytes()
        elif source and entry["repository"] == "unitreerobotics/unitree_mujoco":
            data = (Path(source) / entry["path"]).read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/{entry['repository']}/{entry['commit']}/{entry['path']}"
            with urllib.request.urlopen(url, timeout=60) as response:
                data = response.read()
        if "sha256" in entry:
            actual = hashlib.sha256(data).hexdigest()
            expected = entry["sha256"]
        else:
            actual = hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()
            expected = entry["git_blob"]
        if actual != expected:
            raise ValueError(f"Asset digest mismatch: {dest}")
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            with dest.open("xb") as stream:
                stream.write(data)
    return target


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-source", type=Path)
    parser.add_argument("--stunts", action="store_true", help="Prepare optional pinned ONNX stunt policies only")
    args = parser.parse_args()
    print(prepare(args.model_source, "stunt-assets.lock.json" if args.stunts else "assets.lock.json"))
