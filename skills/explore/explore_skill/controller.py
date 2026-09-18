# SPDX-License-Identifier: MulanPSL-2.0
"""Exploration task lifecycle + frontier-loop driver.

State machine per task:

    explore() called ──► EXPLORING ──► (success) ──► DONE
                              │
                              ├──► (deadline)   ──► TIMEOUT
                              ├──► (cancel)     ──► CANCELED
                              └──► (nav errors) ──► ERROR

The controller owns one in-flight task at a time. Concurrent explore()
is rejected with accepted=false. Caller can cancel + retry.

Threading:
  - rclpy thread (ExploreController._spin_thread): hosts the /map
    subscriber and tf2 buffer.
  - task thread (per explore() invocation): runs the frontier loop.
  - heartbeat thread (in atlas_bridge): unrelated.

Nav integration:
  The skill calls service/navigation/navigate and its status/cancel controls as
  gRPC RPCs against the endpoints atlas resolved at startup. We do
  NOT subscribe to /goal_pose or /cmd_vel — that would skip the nav
  service's contract surface and break the abstraction.
"""
from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

from .configuration import parse_config

log = logging.getLogger("explore_rbnx.controller")


@dataclass
class TaskHandle:
    task_id: str
    started_at: float
    timeout_s: float                  # 0 = no timeout
    max_speed_m_s: float
    state: str = "exploring"          # exploring | done | timeout | canceled | error
    detail: str = "started"
    last_target_xy: Optional[Tuple[float, float]] = None
    initial_area_m2: float = 0.0
    last_area_m2: float = 0.0
    last_progress_t: float = field(default_factory=time.time)
    cancel_requested: bool = False
    legs_completed: int = 0           # successful nav legs in this task
    thread: Optional[threading.Thread] = None


class ExploreController:
    """Frontier-loop controller. One instance per service process;
    one active task at a time."""

    # Tunables
    FRONTIER_MIN_SIZE_CELLS = 3           # below this = noise, ignore
    DONE_QUIET_SECONDS = 30.0             # no progress for this long → done
    PROGRESS_AREA_DELTA_M2 = 1.0          # area gain considered "progress"
    NAV_POLL_PERIOD_S = 1.0
    NAV_GOAL_TIMEOUT_S = 60.0             # per-leg cap; not the global timeout
    NAV_CANCEL_TIMEOUT_S = 30.0
    LOOP_QUIET_PERIOD_S = 2.0             # delay between consecutive goals
    # "Local exploration" radius: candidates farther than this from
    # current robot pose are skipped. Forces the skill to clear the
    # current room before jumping to a far-away frontier.
    MAX_FRONTIER_DISTANCE_M = 6.0
    # Mark cells within this radius of the robot as "visited" each
    # time we update the pose. Used to deprioritise re-revisiting
    # already-cleared areas when multiple frontiers tie in score.
    VISITED_RADIUS_M = 0.4
    # Coverage tracking: each cell records which yaw sectors the
    # camera has pointed at. 8 sectors of 45° each. A cell with all
    # 8 sectors filled has been observed from every direction.
    YAW_SECTORS = 8
    # A 360° sweep is expensive for wheel odometry, so only perform it
    # periodically and when the current cell still lacks yaw coverage.
    # Previously the OR condition swept at every newly reached cell,
    # accumulating rotation drift and starving the global timeout.
    SWEEP_MIN_SECTORS = 6
    SWEEP_EVERY_N_LEGS = 3

    def __init__(self, *, map_topic: str,
                 nav_navigate_endpoint: str,
                 nav_status_endpoint: str,
                 nav_cancel_endpoint: str, config=None,
                 nav_get_speed_endpoint=None, nav_set_speed_endpoint=None):
        """Configure selection and navigation endpoints without starting ROS or threads."""
        self.config = parse_config(config)
        self.map_topic = map_topic
        # All three nav endpoints typically point at the same FastMCP
        # server (http://host:port/mcp/) — atlas hands us the URL each
        # tool registered under. Keep separate fields so a future
        # multi-endpoint nav setup (e.g. nav2-wrapper for some modes,
        # simple_nav for others) still works.
        self._nav_endpoints = {
            "navigate": nav_navigate_endpoint,
            "status":   nav_status_endpoint,
            "cancel":   nav_cancel_endpoint,
            "get_speed_limit": nav_get_speed_endpoint or nav_navigate_endpoint,
            "set_speed_limit": nav_set_speed_endpoint or nav_navigate_endpoint,
        }
        self._lock = threading.Lock()
        self._latest_map: Any = None     # latest OccupancyGrid msg
        self._latest_pose_xyyaw: Optional[Tuple[float, float, float]] = None
        # Visited cells (set of (cx, cy) cell-space ints). Reset per
        # task so a second explore() doesn't inherit stale state.
        self._visited_cells: set = set()
        self._failed_goals = []
        self._task_wake = threading.Event()
        self._active_nav_run = None
        self._speed_restore = None
        # Per-cell viewing-angle coverage: dict[(cx,cy), set[int]]
        # where the int is a sector index in [0, YAW_SECTORS).
        # Used to decide whether to insert a 360° sweep at this cell
        # and to evaluate "fully observed" coverage at done-time.
        self._viewed_sectors: dict = {}
        self._task: Optional[TaskHandle] = None

        self._ros: Optional[dict] = None
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        self._tf_buffer = None
        self._tf_listener = None

        # MCP client for nav RPCs. fastmcp.Client provides a typed
        # JSON-RPC over the streamable-http MCP transport. Lazy-init
        # to avoid pulling fastmcp imports during module load.
        self._mcp_client: Optional[Any] = None

    # ── ROS runtime ─────────────────────────────────────────────────
    def start_runtime(self) -> None:
        """Spin up rclpy node + /map subscriber + tf2 listener."""
        if self._ros is not None:
            return
        self._ros = _import_ros()
        rclpy = self._ros["rclpy"]
        rclpy.init(args=None)

        node = self._ros["Node"]("explore_skill")
        self._node = node

        QoS = self._ros["QoSProfile"]
        Rel = self._ros["ReliabilityPolicy"]
        Dur = self._ros["DurabilityPolicy"]
        Hist = self._ros["HistoryPolicy"]
        # /map is published TRANSIENT_LOCAL by mapping; we must match.
        map_qos = QoS(reliability=Rel.RELIABLE,
                      durability=Dur.TRANSIENT_LOCAL,
                      history=Hist.KEEP_LAST, depth=1)
        node.create_subscription(self._ros["OccupancyGrid"],
                                  self.map_topic, self._on_map, map_qos)

        # tf2 for map→base_link lookup (so we know where the robot is)
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._stop_evt.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop,
                                              daemon=True)
        self._spin_thread.start()
        log.info("rclpy spinning; subscribed to map_topic=%s", self.map_topic)

    def stop_runtime(self) -> None:
        """Cancel and join the task before releasing its ROS inputs."""
        if self._task and self._task.state == "exploring":
            self.cancel(None)
        if self._task and self._task.thread:
            self._task.thread.join(timeout=90.0)
            if self._task.thread.is_alive():
                raise RuntimeError("Explore task did not stop; navigation cancellation is pending")
        if self._active_nav_run is not None:
            raise RuntimeError(f"navigation stop is unconfirmed for {self._active_nav_run}")
        self._stop_evt.set()
        if self._spin_thread:
            self._spin_thread.join(timeout=3.0)
        if self._ros:
            try:
                self._ros["rclpy"].shutdown()
            except Exception:
                pass

    def _spin_loop(self) -> None:
        """Consume map/TF updates until shutdown, reporting ROS failures."""
        rclpy = self._ros["rclpy"]
        while not self._stop_evt.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.2)
                self._refresh_robot_pose()
            except Exception as e:  # noqa: BLE001
                log.exception("spin error: %s", e)
                time.sleep(0.1)

    def _on_map(self, msg) -> None:
        """Replace the map and reproject visited cells when its origin or size changes."""
        with self._lock:
            if self._latest_map is not None:
                from .frontier import GridView
                old = GridView.from_msg(self._latest_map)
                new = GridView.from_msg(msg)
                if (old.origin_x, old.origin_y, old.resolution) != (new.origin_x, new.origin_y, new.resolution):
                    self._visited_cells = {new.world_to_cell(*old.cell_to_world(x, y))
                                           for x, y in self._visited_cells}
                    self._viewed_sectors.clear()
            self._latest_map = msg

    def _refresh_robot_pose(self) -> None:
        """Look up map→base_link; record (x,y,yaw); update visited
        cells + per-cell viewing-angle coverage."""
        if self._tf_buffer is None:
            return
        try:
            from rclpy.time import Time
            tr = self._tf_buffer.lookup_transform(
                "map", "base_link", Time(),
                timeout=self._ros["Duration"](seconds=0.5))
        except Exception:
            return
        x = float(tr.transform.translation.x)
        y = float(tr.transform.translation.y)
        # Quaternion → yaw (z rotation only — Force3DoF world).
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._latest_pose_xyyaw = (x, y, yaw)
            self._mark_visited(x, y, yaw)

    def _mark_visited(self, x: float, y: float, yaw: float) -> None:
        """Stamp visited cells (radius-disk) and add the current yaw
        sector to the per-cell coverage map. Called with self._lock."""
        if self._latest_map is None:
            return
        from .frontier import GridView
        gv = GridView.from_msg(self._latest_map)
        cx, cy = gv.world_to_cell(x, y)
        r = max(1, int(round(self.VISITED_RADIUS_M / gv.resolution)))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self._visited_cells.add((cx + dx, cy + dy))
        # Yaw normalised to [0, 2π) → sector index in [0, YAW_SECTORS).
        sector = int(((yaw % (2 * math.pi)) / (2 * math.pi))
                     * self.YAW_SECTORS) % self.YAW_SECTORS
        self._viewed_sectors.setdefault((cx, cy), set()).add(sector)

    # ── Public API (called from MCP tool handlers) ─────────────────
    def start(self, *, area_hint: str, timeout_s: float,
              max_speed_m_s: float) -> TaskHandle:
        """Start one bounded exploration task with independent visited and failure history."""
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise RuntimeError("timeout_s must be finite and nonnegative")
        if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0:
            raise RuntimeError("max_speed_m_s must be finite and positive")
        with self._lock:
            if self._active_nav_run is not None:
                raise RuntimeError(f"navigation stop is unconfirmed for {self._active_nav_run}")
            if self._task is not None and (self._task.state == "exploring" or
                    (self._task.thread is not None and self._task.thread.is_alive())):
                raise RuntimeError(
                    f"another exploration task is already running "
                    f"(task_id={self._task.task_id}). Call cancel() "
                    f"before starting a new one.")
            # Reset visited history for the new task so we don't carry
            # over stale cells from a prior incomplete run.
            self._visited_cells = set()
            self._failed_goals = []
            self._task_wake.clear()
            handle = TaskHandle(
                task_id="exp-" + uuid.uuid4().hex[:8],
                started_at=time.time(),
                timeout_s=float(timeout_s),
                max_speed_m_s=float(max_speed_m_s),
                detail=f"started (area_hint={area_hint!r})",
            )
            self._task = handle
        handle.thread = threading.Thread(
            target=self._run_task, args=(handle,),
            name=f"explore-{handle.task_id}", daemon=True)
        handle.thread.start()
        return handle

    def cancel(self, task_id: Optional[str]) -> Tuple[bool, str]:
        """Signal cancellation; only the worker cancels its exact navigation run."""
        with self._lock:
            t = self._task
            if t is None:
                return True, "no active task (no-op)"
            if task_id and t.task_id != task_id:
                return False, f"task_id mismatch (active={t.task_id})"
            if t.state != "exploring":
                return True, f"task already in terminal state {t.state} (no-op)"
            t.cancel_requested = True
            t.detail = "cancel requested"
        self._task_wake.set()
        return True, f"cancel requested for {t.task_id}"

    def status(self, task_id: Optional[str]) -> Optional[dict]:
        """Return task progress, including completed legs within the existing detail field."""
        with self._lock:
            t = self._task
            if t is None:
                return None
            if task_id and t.task_id != task_id:
                return None
            elapsed = time.time() - t.started_at
            return {
                "state": t.state,
                "area_m2": float(t.last_area_m2),
                "frontiers_left": _frontiers_left_count(self._latest_map),
                "elapsed_s": float(elapsed),
                "eta_s": -1.0,    # we don't currently estimate
                "detail": f"{t.detail}; completed_legs={t.legs_completed}",
                "task_id": t.task_id,
            }

    # ── Frontier loop (runs in its own thread per task) ─────────────
    def _run_task(self, handle: TaskHandle) -> None:
        """Apply request speed, run exploration, and restore only after navigation stops."""
        try:
            if not handle.cancel_requested:
                self._apply_speed_limit(handle.max_speed_m_s)
            self._explore_loop(handle)
        except Exception as error:
            log.exception("[%s] exploration failed", handle.task_id)
            self._terminate(handle, "error", str(error))
        finally:
            if self._active_nav_run is None:
                try:
                    self._restore_speed_limit()
                except Exception as error:
                    log.exception("[%s] speed restoration failed", handle.task_id)
                    self._terminate(handle, "error", f"speed restoration failed: {error}")

    def _explore_loop(self, handle: TaskHandle) -> None:
        """Recompute reachable approaches from each latest map and suppress failed goals."""
        from .frontier import (GridView, mapped_free_area_m2, pick_target,
                                total_frontier_count)
        log.info("[%s] exploration task starting", handle.task_id)
        # Capture initial state once map is available.
        deadline = handle.started_at + handle.timeout_s if handle.timeout_s > 0 else None
        gm = self._wait_for_map(timeout_s=self._bounded_timeout(15.0, deadline), handle=handle)
        if gm is None:
            state = "canceled" if handle.cancel_requested else (
                "timeout" if deadline is not None and time.time() >= deadline else "error")
            self._terminate(handle, state,
                             "no /map received within 15s — is mapping running?")
            return
        gv = GridView.from_msg(gm)
        handle.initial_area_m2 = mapped_free_area_m2(gv)
        handle.last_area_m2 = handle.initial_area_m2

        deadline = handle.started_at + handle.timeout_s if handle.timeout_s > 0 else None

        while True:
            # Cancel / timeout checks.
            if handle.cancel_requested:
                self._terminate(handle, "canceled", "user-requested cancel")
                return
            if deadline is not None and time.time() > deadline:
                self._terminate(handle, "timeout",
                                 f"hit {handle.timeout_s}s ceiling")
                return

            # Quiet-area check: if frontier count has been low + area
            # hasn't grown for DONE_QUIET_SECONDS, declare done.
            with self._lock:
                latest = self._latest_map
                pose_xyyaw = self._latest_pose_xyyaw
            if latest is None:
                time.sleep(0.5)
                continue
            pose = (pose_xyyaw[0], pose_xyyaw[1]) if pose_xyyaw else None
            gv = GridView.from_msg(latest)
            cur_area = mapped_free_area_m2(gv)
            if cur_area - handle.last_area_m2 > self.PROGRESS_AREA_DELTA_M2:
                handle.last_progress_t = time.time()
            handle.last_area_m2 = cur_area
            n_frontiers = total_frontier_count(
                gv, min_size=self.FRONTIER_MIN_SIZE_CELLS)
            if n_frontiers == 0 and \
                    time.time() - handle.last_progress_t > self.DONE_QUIET_SECONDS:
                self._terminate(handle, "done",
                                f"no frontiers + no progress for "
                                f"{self.DONE_QUIET_SECONDS:.0f}s "
                                f"(area={cur_area:.1f}m²)")
                return

            # Pick next target.
            if pose is None:
                log.info("[%s] no map→base_link tf yet, waiting...",
                          handle.task_id)
                time.sleep(0.5)
                continue
            with self._lock:
                visited_snapshot = set(self._visited_cells)
            now = time.monotonic()
            self._failed_goals = [goal for goal in self._failed_goals if goal[2] > now]
            target = pick_target(gv, pose,
                                  min_size=self.FRONTIER_MIN_SIZE_CELLS,
                                  max_distance_m=self.MAX_FRONTIER_DISTANCE_M,
                                  visited_cells=visited_snapshot,
                                  robot_radius_m=self.config["robot_radius_m"],
                                  approach_distance_m=self.config["approach_distance_m"],
                                  failed_goals=self._failed_goals,
                                  failed_goal_radius_m=self.config["failed_goal_radius_m"],
                                  now=now)
            if target is None:
                handle.detail = (f"no reachable known-free frontier approach; "
                                 f"waiting for map updates ({n_frontiers} clusters)")
                self._task_wake.wait(self.LOOP_QUIET_PERIOD_S)
                continue

            tx, ty = target.centroid_xy
            handle.last_target_xy = (tx, ty)
            handle.detail = (f"driving to frontier ({tx:.2f},{ty:.2f}) "
                             f"size={target.size}, {n_frontiers} clusters left")
            log.info("[%s] %s", handle.task_id, handle.detail)

            leg_timeout = self._bounded_timeout(
                self.NAV_GOAL_TIMEOUT_S, deadline)
            if leg_timeout <= 0.0:
                self._terminate(handle, "timeout",
                                f"hit {handle.timeout_s}s ceiling")
                return
            ok, msg = self._nav_navigate_blocking(
                tx, ty, yaw=None, timeout_s=leg_timeout,
                cancel_evt=handle)
            if handle.cancel_requested:
                self._terminate(handle, "canceled", "cancel during nav")
                return
            if not ok:
                self._failed_goals.append((tx, ty, time.monotonic() + self.config["failed_goal_ttl_s"]))
                log.warning("[%s] nav leg failed: %s", handle.task_id, msg)
                handle.detail = f"nav leg failed ({msg}); trying next frontier"
                self._task_wake.wait(self.LOOP_QUIET_PERIOD_S)
                continue

            handle.legs_completed += 1
            handle.detail = f"frontier reached ({tx:.2f},{ty:.2f}); completed_legs={handle.legs_completed}"
            log.info("[%s] %s", handle.task_id, handle.detail)
            with self._lock:
                self._mark_visited(tx, ty, self._latest_pose_xyyaw[2])

            # 360° sweep to fill camera viewing-angle coverage at the
            # leg endpoint. Two triggers:
            #   - the cell's covered yaw-sectors < SWEEP_MIN_SECTORS,
            #     i.e. we haven't looked around enough here yet
            #   - or every Nth leg as a periodic safety, in case the
            #     coverage tracker missed something
            if self._should_sweep(handle):
                self._sweep_360(handle, deadline=deadline)

            # Brief settle before re-evaluating frontiers.
            self._task_wake.wait(self.LOOP_QUIET_PERIOD_S)

    def _terminate(self, handle: TaskHandle, state: str, detail: str) -> None:
        """Publish a task terminal state and its diagnostic detail."""
        with self._lock:
            handle.state = state
            handle.detail = detail
        log.info("[%s] task %s: %s", handle.task_id, state, detail)

    def _wait_for_map(self, timeout_s: float, handle=None) -> Any:
        """Wait within the task deadline and wake promptly when canceled."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if handle is not None and handle.cancel_requested:
                return None
            with self._lock:
                if self._latest_map is not None:
                    return self._latest_map
            self._task_wake.wait(0.2)
        return None

    # ── Sweep / coverage helpers ───────────────────────────────────
    def _should_sweep(self, handle: TaskHandle) -> bool:
        """Decide whether to do a 360° sweep at the current pose."""
        if handle.legs_completed <= 0 or \
                handle.legs_completed % self.SWEEP_EVERY_N_LEGS != 0:
            return False
        with self._lock:
            pose = self._latest_pose_xyyaw
            latest = self._latest_map
        if pose is None or latest is None:
            return False
        from .frontier import GridView
        gv = GridView.from_msg(latest)
        cx, cy = gv.world_to_cell(pose[0], pose[1])
        sectors = self._viewed_sectors.get((cx, cy), set())
        return len(sectors) < self.SWEEP_MIN_SECTORS

    @staticmethod
    def _bounded_timeout(requested_s: float,
                         deadline: Optional[float]) -> float:
        if deadline is None:
            return requested_s
        return max(0.0, min(requested_s, deadline - time.time()))

    def _sweep_360(self, handle: TaskHandle, *,
                   deadline: Optional[float]) -> None:
        """Send 4 nav goals at the current xy with yaw advanced 90°
        each time. Robot rotates in place 4 times → 360° sweep. The
        camera covers all 8 sectors over those 4 stops. Per-sector
        update is automatic via _refresh_robot_pose() ticks during
        the rotation."""
        with self._lock:
            pose = self._latest_pose_xyyaw
        if pose is None:
            return
        x, y, yaw = pose
        for step in (0.5 * math.pi, math.pi, 1.5 * math.pi, 2.0 * math.pi):
            if handle.cancel_requested:
                return
            segment_timeout = self._bounded_timeout(15.0, deadline)
            if segment_timeout <= 0.0:
                return
            target_yaw = yaw + step
            handle.detail = (f"sweep at ({x:.2f},{y:.2f}) "
                             f"yaw={math.degrees(target_yaw)%360:.0f}°")
            log.info("[%s] %s", handle.task_id, handle.detail)
            ok, _ = self._nav_navigate_blocking(
                x, y, yaw=target_yaw, timeout_s=segment_timeout,
                cancel_evt=handle)
            if not ok:
                return
            time.sleep(0.5)

    # ── Nav RPC over MCP HTTP ──────────────────────────────────────
    # simple_nav exposes navigate/status/cancel as MCP tools, so we
    # call them via FastMCP's client. atlas_bridge resolved the MCP
    # endpoint URL for each contract; in practice all three point at
    # the same FastMCP server, but we keep them separate so a future
    # multi-nav setup still works.
    async def _mcp_call(self, tool: str, args: dict) -> dict:
        """Use a fresh MCP client on this worker's event loop and surface tool errors."""
        import json
        from fastmcp import Client
        async with Client(self._nav_endpoints[tool]) as client:
            result = await client.call_tool(tool, args)
            if result.is_error:
                raise RuntimeError(f"navigation {tool} failed: {result.content}")
            structured = getattr(result, "structured_content", None)
            if isinstance(structured, dict):
                response = structured
            else:
                if not result.content:
                    raise RuntimeError(f"navigation {tool} returned no result")
                response = json.loads(result.content[0].text)
            # FastMCP wraps Any-annotated handlers in a structured result field.
            if isinstance(response, dict) and set(response) == {"result"}:
                response = response["result"]
            if not isinstance(response, dict):
                raise RuntimeError(f"navigation {tool} returned invalid data")
            return response

    def _mcp_call_sync(self, tool: str, args: dict) -> dict:
        """Bound worker RPC time and propagate errors instead of swallowing cancellation."""
        import asyncio
        async def call():
            return await asyncio.wait_for(self._mcp_call(tool, args), timeout=10.0)
        return asyncio.run(call())

    def _nav_navigate_blocking(self, x: float, y: float, *,
                                 yaw: Optional[float],
                                 timeout_s: float,
                                 cancel_evt: TaskHandle) -> Tuple[bool, str]:
        """Send a goal via the nav MCP `navigate` tool, then poll the
        sibling `status` tool until terminal or timeout."""
        if cancel_evt.cancel_requested:
            return False, "canceled before nav"
        if timeout_s <= 0:
            return False, "leg timeout"
        if self._active_nav_run is not None:
            raise RuntimeError(f"navigation stop is unconfirmed for {self._active_nav_run}")
        deadline = time.time() + timeout_s
        self._apply_speed_limit(cancel_evt.max_speed_m_s)
        if cancel_evt.cancel_requested:
            return False, "canceled before nav"
        if time.time() >= deadline:
            return False, "leg timeout"
        qz, qw = (math.sin(yaw / 2), math.cos(yaw / 2)) if yaw is not None else (0.0, 1.0)
        goal_pose = {
            "header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
        }
        resp = self._mcp_call_sync("navigate", {"goal": goal_pose})
        if not resp.get("accepted", False):
            return False, f"goal rejected: {resp.get('detail', '')}"
        run_id = resp.get("run_id", "")
        if not run_id:
            raise RuntimeError("navigation accepted a goal without a run_id")
        self._active_nav_run = run_id
        terminal = False
        try:
            while time.time() < deadline:
                if cancel_evt.cancel_requested:
                    return False, "canceled during nav"
                result = self._mcp_call_sync("status", {"run_id": run_id})
                state = str(result.get("state", "")).upper()
                if state in ("SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                    terminal = True
                    self._active_nav_run = None
                    return state == "SUCCEEDED", f"nav terminal: {state}"
                self._task_wake.wait(min(self.NAV_POLL_PERIOD_S, max(0.0, deadline - time.time())))
            return False, "leg timeout"
        finally:
            if not terminal:
                self._nav_cancel_rpc(run_id)

    def _nav_cancel_rpc(self, run_id: str) -> None:
        """Confirm the exact run's terminal state even if cancellation races its result."""
        deadline = time.monotonic() + self.NAV_CANCEL_TIMEOUT_S
        reason = "terminal state not observed"
        try:
            response = self._mcp_call_sync("cancel", {"run_id": run_id})
            if not response.get("accepted", False):
                reason = f"cancel rejected: {response.get('detail', '')}"
        except Exception as error:
            reason = f"cancel RPC failed: {error}"
            log.warning("%s; checking exact navigation run %s", reason, run_id)
        while time.monotonic() < deadline:
            try:
                status = self._mcp_call_sync("status", {"run_id": run_id})
                if status.get("known", True) and str(status.get("state", "")).upper() in (
                        "SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"):
                    if self._active_nav_run == run_id:
                        self._active_nav_run = None
                    return
            except Exception as error:
                reason = f"status RPC failed: {error}"
                log.warning("%s for navigation run %s", reason, run_id)
            time.sleep(min(self.NAV_POLL_PERIOD_S, max(0.0, deadline - time.monotonic())))
        raise RuntimeError(f"navigation cancellation did not reach terminal state: {run_id}; {reason}")

    def _read_speed_limit(self):
        """Validate Navigation's actual maximum and effective session speed."""
        state = self._mcp_call_sync("get_speed_limit", {})
        if not state.get("available"):
            raise RuntimeError("navigation speed limit is unavailable")
        for key in ("max_linear_speed_mps", "min_percentage", "effective_percentage"):
            value = float(state[key])
            if not math.isfinite(value) or value <= 0:
                raise RuntimeError(f"navigation speed limit has invalid {key}")
            state[key] = value
        if not state["min_percentage"] <= state["effective_percentage"] <= 100:
            raise RuntimeError("navigation speed percentages are inconsistent")
        return state

    def _apply_speed_limit(self, requested_mps):
        """Lower the session cap before motion, retaining any already lower limit."""
        current = self._read_speed_limit()
        if current.get("scope") != "session" or current.get("run_id"):
            raise RuntimeError("navigation speed is owned by another goal")
        maximum = current["max_linear_speed_mps"]
        requested = (100.0 if requested_mps >= maximum else
                     math.floor(requested_mps / maximum * 100_000_000) / 1_000_000)
        if requested < current["min_percentage"]:
            raise RuntimeError("requested speed is below Navigation's minimum enforceable limit")
        target = min(requested, current["effective_percentage"])
        if target >= current["effective_percentage"]:
            return
        result = self._mcp_call_sync("set_speed_limit", {
            "percentage": target, "run_id": "", "persist": True})
        if not result.get("accepted"):
            raise RuntimeError(f"navigation speed limit rejected: {result.get('detail', '')}")
        self._speed_restore = (current, target)
        applied = float(result["effective_percentage"])
        if not math.isfinite(applied) or abs(applied-target) > 1e-6:
            raise RuntimeError("navigation did not apply the requested speed percentage")

    def _restore_speed_limit(self):
        """Restore a changed session cap only after stop and if its value is still ours."""
        if self._speed_restore is None or self._active_nav_run is not None:
            return
        previous, applied = self._speed_restore
        current = self._read_speed_limit()
        self._speed_restore = None
        if (current.get("scope") != "session" or current.get("run_id") or
                current["max_linear_speed_mps"] != previous["max_linear_speed_mps"] or
                abs(current["effective_percentage"] - applied) > 1e-6):
            log.info("navigation speed changed externally; preserving current limit")
            return
        result = self._mcp_call_sync("set_speed_limit", {
            "percentage": previous["effective_percentage"], "run_id": "", "persist": True})
        if not result.get("accepted"):
            raise RuntimeError(f"navigation speed restore rejected: {result.get('detail', '')}")
        restored = float(result["effective_percentage"])
        if not math.isfinite(restored) or abs(restored - previous["effective_percentage"]) > 1e-6:
            raise RuntimeError("navigation did not restore the previous speed percentage")


# ── Helpers ───────────────────────────────────────────────────────────
def _frontiers_left_count(latest_map: Any) -> int:
    """Count raw frontiers for status, returning -1 if no usable map is available."""
    if latest_map is None:
        return -1
    try:
        from .frontier import GridView, total_frontier_count
        gv = GridView.from_msg(latest_map)
        return int(total_frontier_count(gv))
    except Exception:
        return -1


def _import_ros() -> dict:
    """Lazy ROS import — keeps unit tests importable without rclpy."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                            DurabilityPolicy, HistoryPolicy)
    from rclpy.duration import Duration
    from nav_msgs.msg import OccupancyGrid
    return {
        "rclpy": rclpy,
        "Node": Node,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
        "Duration": Duration,
        "OccupancyGrid": OccupancyGrid,
    }
