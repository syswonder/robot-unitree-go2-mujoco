"""Atlas registration and typed MCP tools for the floor transition skill."""
from __future__ import annotations

import logging
import time
from pathlib import Path

from robonix_api import ATLAS, Err, Ok, Skill

from .configuration import parse_config
from .controller import FloorTransitionController

logging.basicConfig(level=logging.INFO, format="[floor-transition] %(levelname)s %(message)s")
log = logging.getLogger("floor_transition")

skill = Skill(id="floor_transition", namespace="robonix/skill/floor_transition")
controller: FloorTransitionController | None = None
selection = None

REQUIRED = {
    "odom": ("robonix/primitive/chassis/odom", "ros2"),
    "cmd_vel": ("robonix/primitive/chassis/twist_in", "ros2"),
    "imu": ("robonix/primitive/imu/imu", "ros2"),
    "map_pose": ("robonix/service/map/pose", "ros2"),
    "map_load": ("robonix/service/map/load_map", "mcp"),
    "nav_navigate": ("robonix/service/navigation/navigate", "mcp"),
    "nav_status": ("robonix/service/navigation/navigate/status", "mcp"),
    "nav_cancel": ("robonix/service/navigation/navigate/cancel", "mcp"),
}


def resolve_inputs(deadline_s: float = 60.0) -> dict[str, str]:
    """Resolve every cross-package dependency through Atlas without topic fallbacks."""
    resolved = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, (contract_id, transport) in REQUIRED.items():
            if key in resolved:
                continue
            try:
                view = ATLAS.find_unique_capability(contract_id=contract_id, transport=transport)
                channel = skill.connect_capability(view, contract_id, transport)
                endpoint = channel.endpoint
                channel.close()
                if endpoint:
                    resolved[key] = endpoint
            except Exception:  # noqa: BLE001
                continue
        if len(resolved) == len(REQUIRED):
            return resolved
        time.sleep(1.0)
    missing = [REQUIRED[key][0] for key in REQUIRED if key not in resolved]
    raise RuntimeError(f"floor_transition dependencies unavailable: {missing}")


from floor_transition_mcp import (  # noqa: E402
    CancelFloorTransition_Request, CancelFloorTransition_Response,
    ExecuteFloorTransition_Request, ExecuteFloorTransition_Response,
    GetFloorTransitionStatus_Request, GetFloorTransitionStatus_Response,
)


@skill.mcp("robonix/skill/floor_transition/execute")
def execute(req: ExecuteFloorTransition_Request) -> ExecuteFloorTransition_Response:
    """Start UP, DOWN, or GO_TO_FLOOR and return an asynchronous run id."""
    if controller is None:
        raise RuntimeError("floor transition controller is not active")
    try:
        task = controller.start(req.command, int(req.target_floor), float(req.timeout_s))
        return ExecuteFloorTransition_Response(accepted=True, run_id=task.run_id, message=task.detail)
    except RuntimeError as error:
        return ExecuteFloorTransition_Response(accepted=False, run_id="", message=str(error))


@skill.mcp("robonix/skill/floor_transition/status")
def status(req: GetFloorTransitionStatus_Request) -> GetFloorTransitionStatus_Response:
    """Return progress, physical floor, and the active Mapping identity."""
    if controller is None:
        raise RuntimeError("floor transition controller is not active")
    value = controller.status(req.run_id or None)
    if value is None:
        return GetFloorTransitionStatus_Response(
            known=False, state="PENDING", current_floor=0, target_floor=0,
            phase="IDLE", elapsed_s=0.0, active_map_id="", detail="unknown run id")
    return GetFloorTransitionStatus_Response(known=True, **value)


@skill.mcp("robonix/skill/floor_transition/cancel")
def cancel(req: CancelFloorTransition_Request) -> CancelFloorTransition_Response:
    """Request bounded cancellation of the exact transition run."""
    if controller is None:
        raise RuntimeError("floor transition controller is not active")
    ok, message = controller.cancel(req.run_id or None)
    return CancelFloorTransition_Response(ok=ok, message=message)


@skill.on_init
def init(config):
    """Validate only package-local configuration during light initialization."""
    global selection
    try:
        selection = parse_config(config, Path(__file__).resolve().parents[1])
        return Ok()
    except (TypeError, ValueError) as error:
        return Err(str(error))


@skill.on_activate
def activate():
    """Resolve dependencies, load the stair policy, and activate the startup map."""
    global controller
    if controller is not None:
        return Ok()
    try:
        inputs = resolve_inputs()
        controller = FloorTransitionController(
            profile_file=selection["profile_file"], startup_floor=selection["startup_floor"],
            bridge_url=selection["bridge_url"],
            endpoints={key: inputs[key] for key in ("map_load", "nav_navigate", "nav_status", "nav_cancel")},
            topics={key: inputs[key] for key in ("odom", "cmd_vel", "imu", "map_pose")},
        )
        controller.start_runtime()
        return Ok()
    except Exception as error:  # noqa: BLE001
        log.exception("activation failed")
        controller = None
        return Err(str(error))


@skill.on_deactivate
def deactivate():
    """Stop any active transition before the provider is evicted."""
    global controller
    if controller is not None:
        controller.stop_runtime()
        controller = None
    return Ok()


@skill.on_shutdown
def shutdown():
    return deactivate()


if __name__ == "__main__":
    skill.run()
