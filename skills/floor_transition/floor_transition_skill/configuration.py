"""Validation for deployment-owned floor transition settings."""
from __future__ import annotations

from pathlib import Path
from urllib.parse import urlsplit


def parse_config(config, package_root: Path) -> dict:
    """Resolve one package-local profile and reject unsafe remote bridge URLs."""
    raw = dict(config or {})
    allowed = {"profile_file", "startup_floor", "bridge_url"}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown floor_transition config fields: {sorted(unknown)}")
    configured = Path(str(raw.get("profile_file") or "/scene_profile/multifloor.yaml"))
    if configured.is_absolute():
        if configured != Path("/scene_profile/multifloor.yaml"):
            raise ValueError("absolute profile_file must be /scene_profile/multifloor.yaml")
        profile = configured
    else:
        if ".." in configured.parts:
            raise ValueError("relative profile_file must stay inside the skill package")
        profile = (package_root / configured).resolve()
        if package_root.resolve() not in profile.parents:
            raise ValueError("relative profile_file must stay inside the skill package")
    if not profile.is_file():
        raise ValueError(f"floor profile not found: {configured}")
    startup_floor = int(raw.get("startup_floor", 1))
    if startup_floor not in (1, 2):
        raise ValueError("startup_floor must be 1 or 2")
    bridge_url = str(raw.get("bridge_url") or "http://127.0.0.1:8766").rstrip("/")
    parsed = urlsplit(bridge_url)
    if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"}:
        raise ValueError("bridge_url must be a local HTTP endpoint")
    return {"profile_file": str(profile), "startup_floor": startup_floor,
            "bridge_url": bridge_url}
