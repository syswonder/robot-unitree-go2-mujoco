"""Two-floor task lifecycle, stair control, and mapping context switching."""
from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.request import Request, urlopen

import yaml

log = logging.getLogger("floor_transition.controller")


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


@dataclass
class Task:
    run_id: str
    target_floor: int
    timeout_s: float
    started_at: float
    state: str = "RUNNING"
    phase: str = "VALIDATE"
    detail: str = "accepted"
    cancel_requested: bool = False
    thread: threading.Thread | None = None


class FloorTransitionController:
    """Own one transition at a time and fail closed outside the marked stair lane."""

    def __init__(self, *, profile_file: str, startup_floor: int, bridge_url: str,
                 endpoints: dict[str, str], topics: dict[str, str]):
        self.package_root = Path(__file__).resolve().parents[1]
        self.profile = self._load_profile(Path(profile_file))
        self.bridge_url = bridge_url
        self.endpoints = endpoints
        self.topics = topics
        state_path = Path(str(self.profile.get("state_file", "rbnx-build/data/current_floor.json")))
        self.state_path = self.package_root / state_path
        self.current_floor = self._load_floor(startup_floor)
        self.active_map_id = str(self.profile["floors"][self.current_floor]["map_id"])
        self._lock = threading.Lock()
        self._task: Task | None = None
        self._active_nav_run: str | None = None
        self._wake = threading.Event()
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._stop = threading.Event()
        self._cmd_pub = None
        self._odom = None
        self._map_pose = None
        self._imu = None

    @staticmethod
    def _load_profile(path: Path) -> dict:
        """Load a strict two-floor profile whose transitions are reciprocal."""
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("environment_id") != "scenesmith_multilevel_house":
            raise ValueError("profile environment_id must be scenesmith_multilevel_house")
        floors = data.get("floors") or {}
        floors = {int(key): value for key, value in floors.items()}
        if set(floors) != {1, 2}:
            raise ValueError("profile must define exactly floors 1 and 2")
        data["floors"] = floors
        connection = data.get("connection") or {}
        if connection.get("up", {}).get("from_floor") != 1 or connection.get("up", {}).get("to_floor") != 2:
            raise ValueError("up transition must connect floor 1 to floor 2")
        if connection.get("down", {}).get("from_floor") != 2 or connection.get("down", {}).get("to_floor") != 1:
            raise ValueError("down transition must connect floor 2 to floor 1")
        return data

    def _load_floor(self, startup_floor: int) -> int:
        """Restore only a valid completed floor; otherwise use the declared startup floor."""
        try:
            saved = json.loads(self.state_path.read_text(encoding="utf-8"))
            if saved.get("environment_id") == self.profile["environment_id"] and int(saved["floor"]) in (1, 2):
                return int(saved["floor"])
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return startup_floor

    def _store_floor(self, floor: int) -> None:
        """Persist only after traversal and target-map activation both succeed."""
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"environment_id": self.profile["environment_id"],
                                         "floor": floor}) + "\n", encoding="utf-8")
        temporary.replace(self.state_path)

    def start_runtime(self) -> None:
        """Start ROS subscriptions and bootstrap the configured floor map."""
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from rclpy.executors import SingleThreadedExecutor
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("floor_transition_skill")
        self._cmd_pub = self._node.create_publisher(Twist, self.topics["cmd_vel"], 10)
        self._node.create_subscription(Odometry, self.topics["odom"], lambda msg: setattr(self, "_odom", msg), 20)
        self._node.create_subscription(PoseWithCovarianceStamped, self.topics["map_pose"],
                                       lambda msg: setattr(self, "_map_pose", msg), 10)
        self._node.create_subscription(Imu, self.topics["imu"], lambda msg: setattr(self, "_imu", msg), 20)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._ensure_environment()
        self._set_policy(False)
        deadline = time.time() + 5.0
        while self._odom is None and time.time() < deadline:
            time.sleep(0.05)
        if self._odom is None:
            raise RuntimeError("odometry unavailable while seeding startup floor map")
        result = self._load_map(self.current_floor, self._odom_pose())
        if not result.get("ok"):
            raise RuntimeError(f"startup floor map unavailable: {result.get('detail', result)}")

    def stop_runtime(self) -> None:
        """Cancel active work, publish zero velocity, and stop ROS resources."""
        self.cancel(None)
        self._publish_stop()
        self._stop.set()
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=3)
        if self._node is not None:
            self._node.destroy_node()

    def start(self, command: str, target_floor: int, timeout_s: float) -> Task:
        """Validate one user command and launch a serial transition worker."""
        command = command.strip().upper()
        if command == "UP":
            target = self.current_floor + 1
        elif command == "DOWN":
            target = self.current_floor - 1
        elif command == "GO_TO_FLOOR":
            target = int(target_floor)
        else:
            raise RuntimeError("command must be UP, DOWN, or GO_TO_FLOOR")
        if target not in (1, 2):
            raise RuntimeError(f"floor {target} is outside this two-floor demo")
        with self._lock:
            if self._task is not None and self._task.state == "RUNNING":
                raise RuntimeError("another floor transition is running")
            limit = float(timeout_s or 180.0)
            if not math.isfinite(limit) or limit <= 0:
                raise RuntimeError("timeout_s must be finite and positive")
            task = Task(str(uuid.uuid4()), target, limit, time.time())
            self._task = task
            task.thread = threading.Thread(target=self._run, args=(task,), daemon=True)
            task.thread.start()
            return task

    def status(self, run_id: str | None) -> dict | None:
        """Return a stable snapshot for the requested or latest task."""
        with self._lock:
            task = self._task
            if task is None or (run_id and task.run_id != run_id):
                return None
            return {"state": task.state, "target_floor": task.target_floor,
                    "phase": task.phase, "detail": task.detail,
                    "elapsed_s": time.time() - task.started_at,
                    "current_floor": self.current_floor, "active_map_id": self.active_map_id}

    def cancel(self, run_id: str | None) -> tuple[bool, str]:
        """Request cancellation and address the exact active navigation run."""
        with self._lock:
            task = self._task
            if task is None or (run_id and task.run_id != run_id):
                return True, "no matching active task"
            if task.state != "RUNNING":
                return True, f"task already {task.state}"
            task.cancel_requested = True
            active_nav = self._active_nav_run
        self._wake.set()
        if active_nav:
            try:
                self._cancel_navigation(active_nav)
                with self._lock:
                    if self._active_nav_run == active_nav:
                        self._active_nav_run = None
            except Exception as error:  # noqa: BLE001
                return False, f"cancel requested but navigation cancellation failed: {error}"
        self._publish_stop()
        return True, "cancel requested"

    def _run(self, task: Task) -> None:
        """Move through each adjacent floor and commit state only after map load."""
        deadline = task.started_at + task.timeout_s
        try:
            if task.target_floor == self.current_floor:
                task.phase, task.detail, task.state = "COMPLETE", "already on requested floor", "SUCCEEDED"
                return
            while self.current_floor != task.target_floor:
                direction = "up" if task.target_floor > self.current_floor else "down"
                self._run_leg(task, direction, deadline)
            task.phase, task.detail, task.state = "COMPLETE", f"arrived at floor {self.current_floor}", "SUCCEEDED"
        except TimeoutError as error:
            task.state, task.detail = "TIMEOUT", str(error)
        except InterruptedError as error:
            task.state, task.detail = "CANCELED", str(error)
        except Exception as error:  # noqa: BLE001
            log.exception("floor transition failed")
            task.state, task.detail = "FAILED", str(error)
        finally:
            self._publish_stop()

    def _run_leg(self, task: Task, direction: str, deadline: float) -> None:
        """Navigate to a marked entry, traverse the fixed flight, and activate the target map."""
        spec = self.profile["connection"][direction]
        if int(spec["from_floor"]) != self.current_floor:
            raise RuntimeError(f"profile has no {direction} transition from floor {self.current_floor}")
        self._check_task(task, deadline)
        self._set_policy(False)
        task.phase, task.detail = "POSITION_AT_ENTRY", f"moving to {direction} stair entry"
        self._position_at_entry(task, spec["approach"], min(90.0, deadline - time.time()))
        task.phase, task.detail = "VERIFY_ENTRY", "checking marked stair entry pose"
        self._verify_entry(spec["approach"])
        task.phase, task.detail = "TRAVERSE_STAIRS", f"executing fixed {direction} profile"
        self._set_policy(True)
        self._traverse(task, direction, spec, min(float(spec["timeout_s"]), deadline - time.time()))
        self._set_policy(False)
        destination = int(spec["to_floor"])
        task.phase, task.detail = "ALIGN_DESTINATION", f"aligning on floor {destination} landing"
        self._align_heading(task, float(self.profile["floors"][destination]["initial_pose"]["yaw"]),
                            min(float(self.profile["connection"]["alignment_timeout_s"]),
                                deadline - time.time()))
        task.phase, task.detail = "LOAD_TARGET_MAP", f"loading floor {destination} map"
        result = self._load_map(destination, self._odom_pose())
        if not result.get("ok"):
            raise RuntimeError(f"physical traversal completed but map switch failed: {result.get('detail', result)}")
        self.current_floor = destination
        self.active_map_id = str(self.profile["floors"][destination]["map_id"])
        self._store_floor(destination)

    def _odom_pose(self) -> dict[str, float]:
        """Return the native simulator's world-aligned odometry as a map seed."""
        if self._odom is None:
            raise RuntimeError("odometry unavailable for map pose seed")
        pose = self._odom.pose.pose
        return {"x": float(pose.position.x), "y": float(pose.position.y),
                "yaw": _yaw(pose.orientation)}

    def _position_at_entry(self, task: Task, pose: dict, timeout_s: float) -> None:
        """Use Nav2 outside the connection zone and direct flat-gait alignment inside it."""
        msg = self._map_pose
        if msg is None:
            raise RuntimeError("no map-frame pose available before stair entry")
        actual = msg.pose.pose
        distance = math.hypot(actual.position.x - float(pose["x"]),
                              actual.position.y - float(pose["y"]))
        direct_radius = float(self.profile["connection"]["direct_alignment_radius_m"])
        if distance > direct_radius:
            self._navigate(task, pose, timeout_s)
        self._align_heading(task, float(pose["yaw"]), min(
            float(self.profile["connection"]["alignment_timeout_s"]), timeout_s))

    def _align_heading(self, task: Task, target_yaw: float, timeout_s: float) -> None:
        """Turn on the flat landing with the deterministic gait before loading stair policy."""
        if timeout_s <= 0:
            raise TimeoutError("task deadline expired before entry alignment")
        end = time.time() + timeout_s
        settled = 0
        tolerance = float(self.profile["connection"]["alignment_yaw_tolerance_rad"])
        max_speed = float(self.profile["connection"]["alignment_speed_rad_s"])
        while time.time() < end:
            self._check_task(task, end)
            odom = self._odom
            if odom is None:
                self._wake.wait(0.05)
                continue
            pose = odom.pose.pose
            error = _wrap(target_yaw - _yaw(pose.orientation))
            if abs(error) <= tolerance:
                self._publish_stop()
                settled += 1
                if settled >= 8:
                    return
            else:
                settled = 0
                magnitude = max(0.18, min(max_speed, abs(error)))
                self._publish_twist(0.0, math.copysign(magnitude, error))
            self._wake.wait(0.05)
        raise TimeoutError("stair entry heading alignment timed out")

    def _navigate(self, task: Task, pose: dict, timeout_s: float) -> None:
        """Run one exact navigation goal and confirm its terminal state."""
        if timeout_s <= 0:
            raise TimeoutError("task deadline expired before entry navigation")
        goal = {"header": {"frame_id": "map", "stamp": {"sec": 0, "nanosec": 0}},
                "pose": {"position": {"x": float(pose["x"]), "y": float(pose["y"]), "z": 0.0},
                         "orientation": {"x": 0.0, "y": 0.0,
                                         "z": math.sin(float(pose["yaw"]) / 2.0),
                                         "w": math.cos(float(pose["yaw"]) / 2.0)}}}
        response = self._mcp("nav_navigate", "navigate", {"goal": goal})
        if not response.get("accepted") or not response.get("run_id"):
            raise RuntimeError(f"entry navigation rejected: {response.get('detail', response)}")
        run_id = str(response["run_id"])
        self._active_nav_run = run_id
        end = time.time() + timeout_s
        terminal = False
        try:
            while time.time() < end:
                if task.cancel_requested:
                    raise InterruptedError("canceled during entry navigation")
                status = self._mcp("nav_status", "status", {"run_id": run_id})
                state = str(status.get("state", "")).upper()
                if state == "SUCCEEDED":
                    terminal = True
                    return
                if state in {"FAILED", "CANCELED", "TIMEOUT"}:
                    terminal = True
                    raise RuntimeError(f"entry navigation ended {state}: {status.get('detail', '')}")
                self._wake.wait(0.5)
            raise TimeoutError("entry navigation timed out")
        finally:
            if not terminal:
                terminal = self._cancel_navigation(run_id)
            if terminal and self._active_nav_run == run_id:
                self._active_nav_run = None

    def _cancel_navigation(self, run_id: str) -> bool:
        """Cancel an accepted Nav2 run and confirm a terminal state before releasing it."""
        response = self._mcp("nav_cancel", "cancel", {"run_id": run_id})
        accepted = bool(response.get("accepted", response.get("ok", False)))
        if not accepted:
            raise RuntimeError(f"navigation cancellation rejected: {response.get('detail', response)}")
        end = time.time() + 8.0
        while time.time() < end:
            status = self._mcp("nav_status", "status", {"run_id": run_id})
            if str(status.get("state", "")).upper() in {"SUCCEEDED", "FAILED", "CANCELED", "TIMEOUT"}:
                return True
            self._wake.wait(0.2)
        raise RuntimeError("navigation cancellation was not confirmed")

    def _verify_entry(self, expected: dict) -> None:
        """Require a fresh map pose inside the manually calibrated entry tolerance."""
        msg = self._map_pose
        if msg is None:
            raise RuntimeError("no map-frame pose available at stair entry")
        actual = msg.pose.pose
        distance = math.hypot(actual.position.x - float(expected["x"]),
                              actual.position.y - float(expected["y"]))
        yaw_error = abs(_wrap(_yaw(actual.orientation) - float(expected["yaw"])))
        connection = self.profile["connection"]
        if distance > float(connection["entry_xy_tolerance_m"]) or yaw_error > float(connection["entry_yaw_tolerance_rad"]):
            raise RuntimeError(f"stair entry mismatch: distance={distance:.3f} yaw_error={yaw_error:.3f}")

    def _traverse(self, task: Task, direction: str, spec: dict, timeout_s: float) -> None:
        """Follow the fixed stair centerline using odom feedback and bounded Twist."""
        if timeout_s <= 0:
            raise TimeoutError("task deadline expired before stair traversal")
        lane = float(self.profile["connection"]["lane_y"])
        sign = 1.0 if direction == "up" else -1.0
        heading = 0.0 if direction == "up" else math.pi
        end = time.time() + timeout_s
        settled = 0
        while time.time() < end:
            self._check_task(task, end)
            odom = self._odom
            if odom is None:
                self._wake.wait(0.05)
                continue
            pose = odom.pose.pose
            lane_error = pose.position.y - lane
            if abs(lane_error) > float(self.profile["connection"]["max_lane_error_m"]):
                raise RuntimeError(f"left calibrated stair lane by {lane_error:.3f} m")
            upright = 1.0 - 2.0 * (pose.orientation.x ** 2 + pose.orientation.y ** 2)
            if upright < float(self.profile["connection"]["minimum_upright_cosine"]):
                raise RuntimeError(f"lost upright attitude on stairs: {upright:.3f}")
            reached = pose.position.x >= float(spec["target_x"]) if sign > 0 else pose.position.x <= float(spec["target_x"])
            if reached:
                self._publish_stop()
                height = float(pose.position.z)
                bounds = spec["expected_height"]
                if not float(bounds["min"]) <= height <= float(bounds["max"]):
                    raise RuntimeError(f"landing height {height:.3f} outside expected range")
                settled += 1
                if settled >= 10:
                    return
            else:
                settled = 0
                angular = _wrap(heading - _yaw(pose.orientation)) - 1.2 * sign * lane_error
                self._publish_twist(float(spec["speed_m_s"]), max(-0.5, min(0.5, angular)))
            self._wake.wait(0.05)
        raise TimeoutError(f"{direction} stair traversal timed out")

    def _load_map(self, floor: int, initial_pose: dict | None = None) -> dict:
        """Switch Mapping to the floor's immutable artifact with a calibrated pose seed."""
        pose = initial_pose or self.profile["floors"][floor]["initial_pose"]
        response = self._mcp("map_load", "load_map", {
            "map_id": str(self.profile["floors"][floor]["map_id"]), "mode": "localization",
            "has_initial_pose": True, "x": float(pose["x"]), "y": float(pose["y"]),
            "theta": float(pose["yaw"]),
        })
        if response.get("ok"):
            self.active_map_id = str(self.profile["floors"][floor]["map_id"])
        return response

    def _ensure_environment(self) -> None:
        """Reject activation unless the bridge is serving the annotated two-floor scene."""
        with urlopen(self.bridge_url + "/health", timeout=3) as response:
            health = json.load(response)
        if not health.get("ok") or health.get("environment") != self.profile["environment_id"]:
            raise RuntimeError(f"simulator is not running {self.profile['environment_id']}")

    def _set_policy(self, loaded: bool) -> None:
        """Use flat gait on landings and the released rough-terrain policy only on stairs."""
        with urlopen(self.bridge_url + "/health", timeout=3) as response:
            health = json.load(response)
        policy = health.get("policy") or {}
        if bool(policy.get("loaded")) == loaded:
            return
        action = "load" if loaded else "unload"
        payload = json.dumps({"type": "policy", "action": action, "id": "moe_rough"}).encode()
        request = Request(self.bridge_url + "/command", data=payload,
                          headers={"Content-Type": "application/json"}, method="POST")
        with urlopen(request, timeout=15) as response:
            result = json.load(response)
        actual = bool((result.get("policy") or {}).get("loaded"))
        if not result.get("ok") or actual != loaded:
            raise RuntimeError(f"failed to {action} stair policy: {result}")

    def _publish_twist(self, linear: float, angular: float) -> None:
        """Publish one bounded command on the Atlas-resolved chassis input."""
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        if self._cmd_pub is not None:
            self._publish_twist(0.0, 0.0)

    @staticmethod
    def _check_task(task: Task, deadline: float) -> None:
        if task.cancel_requested:
            raise InterruptedError("floor transition canceled")
        if time.time() >= deadline:
            raise TimeoutError("floor transition deadline expired")

    async def _mcp_async(self, endpoint_key: str, tool: str, arguments: dict) -> dict:
        """Call one Atlas-resolved MCP endpoint and normalize FastMCP responses."""
        from fastmcp import Client
        async with Client(self.endpoints[endpoint_key]) as client:
            result = await client.call_tool(tool, arguments)
            if result.is_error:
                raise RuntimeError(f"{tool} failed: {result.content}")
            value = getattr(result, "structured_content", None)
            if not isinstance(value, dict):
                if not result.content:
                    raise RuntimeError(f"{tool} returned no result")
                value = json.loads(result.content[0].text)
            if set(value) == {"result"}:
                value = value["result"]
            if not isinstance(value, dict):
                raise RuntimeError(f"{tool} returned invalid data")
            return value

    def _mcp(self, endpoint_key: str, tool: str, arguments: dict) -> dict:
        return asyncio.run(asyncio.wait_for(self._mcp_async(endpoint_key, tool, arguments), timeout=250.0))
