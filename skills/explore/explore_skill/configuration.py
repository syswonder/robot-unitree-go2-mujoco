# SPDX-License-Identifier: MulanPSL-2.0
"""Deployment configuration for footprint clearance and failed-goal suppression."""
import math

DEFAULTS = {
    "robot_radius_m": math.hypot(0.35, 0.20),
    "approach_distance_m": 0.8,
    "failed_goal_radius_m": 0.6,
    "failed_goal_ttl_s": 60.0,
}


def parse_config(config):
    """Validate finite positive package settings while keeping request limits separate."""
    if config is None:
        config = {}
    if not isinstance(config, dict):
        raise ValueError("Explore config must be a mapping")
    unknown = set(config) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"unknown Explore config fields: {sorted(unknown)}")
    result = dict(DEFAULTS)
    for name, value in config.items():
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a number")
        result[name] = float(value)
    for name, value in result.items():
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if result["robot_radius_m"] < DEFAULTS["robot_radius_m"]:
        raise ValueError("robot_radius_m must enclose the Go2 0.70 x 0.40 m footprint")
    if result["approach_distance_m"] <= result["robot_radius_m"]:
        raise ValueError("approach_distance_m must exceed robot_radius_m")
    return result
