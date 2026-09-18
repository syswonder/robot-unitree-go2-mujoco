"""Common helpers for primitive processes running in the ROS bridge container."""
from __future__ import annotations

import os
import math
from pathlib import Path


def config_value(config: dict, key: str, default):
    """Read a deployment config value while preserving explicit false values."""
    value = (config or {}).get(key)
    return default if value is None else value


def provider_id(default: str) -> str:
    """Return the deployment instance id selected by ``rbnx boot``."""
    return os.environ.get("RBNX_INSTANCE_NAME") or os.environ.get("SIM_PROVIDER_ID", default)


def package_root() -> Path:
    """Return the package root mapped into the simulation container."""
    return Path(os.environ.get("RBNX_PACKAGE_ROOT", os.getcwd())).resolve()


def topic_value(config: dict, key: str, default: str, *, allow_empty: bool = False) -> str:
    """Read and validate an absolute ROS topic from lifecycle config."""
    value = config_value(config, key, default)
    if not isinstance(value, str):
        raise ValueError(f"{key} must be a string")
    value = value.strip()
    if not value and allow_empty:
        return value
    if not value.startswith("/") or value == "/":
        raise ValueError(f"{key} must be a non-root absolute ROS topic, got {value!r}")
    return value


def timeout_value(config: dict, key: str = "sentinel_timeout_s", default: float = 30.0) -> float:
    """Read and validate a positive timeout in seconds."""
    try:
        value = float(config_value(config, key, default))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{key} must be a number") from error
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{key} must be finite and greater than zero")
    return value
