"""Physically actuated Go2 gait and optional released MoE Rough inference."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np


LEGS = ("FL", "FR", "RL", "RR")
ACTUATORS = tuple(f"{leg}_{joint}" for leg in LEGS for joint in ("hip", "thigh", "calf"))
JOINT_NAMES = tuple(f"{name}_joint" for name in ACTUATORS)
DEFAULT_JOINT_POS = np.array([
    0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
    0.1, 1.0, -1.5, -0.1, 1.0, -1.5,
], dtype=np.float32)
SCRIPTED_HOME = np.tile([0.0, 0.9, -1.8], 4)
SCRIPTED_KP = np.tile([45.0, 55.0, 60.0], 4)
SCRIPTED_KD = np.tile([1.5, 2.0, 2.0], 4)
COMMAND_LIMITS = np.array([0.6, 0.35, 0.6])
POLICY_COMMAND_LIMITS = np.array([0.6, 0.35, 1.0])
POLICY_FEEDFORWARD = np.array([2.0, 2.0, 2.2])
POLICY_INTEGRAL_GAINS = np.array([2.5, 2.5, 5.0])
SCRIPTED_COMMAND_LIMITS = np.array([0.4, 0.15, 0.5])
SCRIPTED_UPRIGHT_RECOVERY_COS = 0.72
SCRIPTED_UPRIGHT_FULL_SPEED_COS = 0.90
COMMAND_WATCHDOG_S = 0.35
CONTROL_DT = 0.02
LINK_LENGTH = 0.213
HOME_FOOT_Z = -2 * LINK_LENGTH * math.cos(0.9)


def compute_go2_targets(command: np.ndarray, phase: float) -> np.ndarray:
    """Build diagonal-trot joint targets for a body-frame velocity command."""
    magnitude = min(1.0, float(np.linalg.norm(command)))
    if magnitude < 1e-6:
        return SCRIPTED_HOME.copy()
    targets = np.empty(12)
    for index, (front, side, offset) in enumerate((
            (1, 1, 0), (1, -1, .5), (-1, 1, .5), (-1, -1, 0))):
        cycle = (phase / math.tau + offset) % 1.0
        stance = cycle < .56
        progress = cycle / .56 if stance else (cycle - .56) / .44
        travel = .5 - progress if stance else -.5 + progress
        lift = 0.0 if stance else math.sin(math.pi * progress)
        # A yaw step is tangential to each hip: left/right legs need opposite
        # fore-aft travel and front/rear legs need opposite lateral travel.
        # The previous gait implemented only the first component, so an
        # in-place turn translated the body and could walk it into nearby
        # geometry.  The coefficient ratio follows the Go2 hip offsets.
        x = .13 * np.clip(command[0] - side * command[2] * .65, -1, 1) * travel
        y = (.075 * command[1] + .115 * front * command[2]) * travel
        z = HOME_FOOT_Z + .045 * lift * magnitude
        sagittal_z = -math.hypot(y, z)
        cosine = np.clip((x*x + sagittal_z*sagittal_z - 2*LINK_LENGTH**2)
                         / (2*LINK_LENGTH**2), -.999, .999)
        knee = -math.acos(cosine)
        targets[index*3:index*3+3] = [
            np.clip(math.atan2(y, -z), -.75, .75),
            np.clip(math.atan2(-x, -sagittal_z) - knee*.5, -.45, 2.6),
            np.clip(knee, -2.65, -.9),
        ]
    return targets


def build_moe_rough_observation(angular_velocity, quaternion, command,
                                joint_position, joint_velocity, last_action) -> np.ndarray:
    """Build the released 45-float frame in FL/FR/RL/RR order, wxyz attitude."""
    w, x, y, z = np.asarray(quaternion, dtype=np.float32)
    gravity = [2*(w*y - x*z), -2*(w*x + y*z), 2*(x*x + y*y) - 1]
    observation = np.concatenate((
        np.asarray(angular_velocity, dtype=np.float32) * .25, gravity,
        np.asarray(command, dtype=np.float32) * [2, 2, .25],
        np.asarray(joint_position, dtype=np.float32) - DEFAULT_JOINT_POS,
        np.asarray(joint_velocity, dtype=np.float32) * .05,
        np.asarray(last_action, dtype=np.float32),
    )).astype(np.float32)
    if observation.shape != (45,) or not np.isfinite(observation).all():
        raise ValueError("MoE Rough observation must contain 45 finite values")
    return observation


class Go2Controller:
    """Accept continuous Twist and switch joint controllers without moving qpos."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 policy_dir: Path | None = None, clock=time.monotonic) -> None:
        """Resolve named transmissions and initialize the robot at its home key."""
        self.model, self.data, self.clock = model, data, clock
        self.policy_dir = policy_dir or (
            Path(__file__).resolve().parents[2] / "assets/robots/go2/policy/moe_rough")
        self.joints = np.array([self._id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES])
        self.actuators = np.array([self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATORS])
        self.qpos = model.jnt_qposadr[self.joints]
        self.dof = model.jnt_dofadr[self.joints]
        self.base = self._id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.foot_geoms = [self._id(mujoco.mjtObj.mjOBJ_GEOM, leg) for leg in LEGS]
        self.foot_bodies = [self._id(mujoco.mjtObj.mjOBJ_BODY, f"{leg}_foot") for leg in LEGS]
        root = self._id(mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
        if model.jnt_type[root] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("Go2 requires a free base joint")
        self.base_qpos, self.base_dof = int(model.jnt_qposadr[root]), int(model.jnt_dofadr[root])
        if not np.array_equal(model.actuator_trnid[self.actuators, 0], self.joints):
            raise ValueError("Go2 motor transmissions do not match the leg joints")
        if not np.allclose(model.actuator_gear[self.actuators], [1, 0, 0, 0, 0, 0]):
            raise ValueError("Go2 requires unit-gear joint torque motors")
        self.home_key = self._id(mujoco.mjtObj.mjOBJ_KEY, "home")
        self.session = None
        self.policy_status = {"id": "moe_rough", "loaded": False, "error": None}
        self.history = np.zeros((5, 45), dtype=np.float32)
        self.last_action = np.zeros(12, dtype=np.float32)
        self.twist = np.zeros(3)
        self.targets = SCRIPTED_HOME.copy()
        self.reset()

    def _id(self, kind, name: str) -> int:
        result = mujoco.mj_name2id(self.model, kind, name)
        if result < 0:
            raise ValueError(f"Go2 model is missing {name}")
        return result

    def reset(self) -> None:
        """Explicit reset is the only operation that writes physical state."""
        simulation_time = float(self.data.time)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_key)
        self.data.time = simulation_time
        self.data.ctrl[:] = 0
        self.twist[:] = 0
        self.last_twist_time = -math.inf
        self.estopped = False
        self.phase = 0.0
        self.gait_blend = 0.0
        self.velocity_integral = np.zeros(3)
        self.policy_velocity_integral = np.zeros(3)
        self.history[:] = 0
        self.last_action[:] = 0
        self.targets[:] = SCRIPTED_HOME
        self.next_policy_time = float(self.data.time)
        self.transition_time = -math.inf
        self.transition_targets = self.targets.copy()
        self.transition_kp = SCRIPTED_KP.copy()
        self.transition_kd = SCRIPTED_KD.copy()
        self.kp, self.kd = SCRIPTED_KP.copy(), SCRIPTED_KD.copy()
        self.feedforward = np.zeros(12)
        self.transition_feedforward = self.feedforward.copy()
        mujoco.mj_forward(self.model, self.data)

    def command(self, message: dict) -> bool:
        """Validate Twist and policy requests, returning an acknowledgement flag."""
        if not isinstance(message, dict):
            return False
        kind = message.get("type")
        if kind == "cmd_vel":
            try:
                values = np.array([float(message.get(key, 0))
                                   for key in ("linearX", "linearY", "angularZ")])
            except (TypeError, ValueError, OverflowError):
                return False
            if not np.isfinite(values).all():
                return False
            self.twist[:] = np.clip(values, -COMMAND_LIMITS, COMMAND_LIMITS)
            self.last_twist_time = self.clock()
            self.estopped = False
            return True
        if kind == "policy":
            return self._policy_command(message)
        if kind == "emergency_stop":
            self.twist[:] = 0
            self.last_twist_time = -math.inf
            self.estopped = True
            return True
        if kind == "reset":
            self.reset()
            return True
        return False

    def _observation(self, command: np.ndarray) -> np.ndarray:
        """Read local angular velocity and joint state in policy order."""
        q, v = self.base_qpos, self.base_dof
        return build_moe_rough_observation(
            self.data.qvel[v+3:v+6], self.data.qpos[q+3:q+7], command,
            self.data.qpos[self.qpos], self.data.qvel[self.dof], self.last_action)

    def _load_session(self):
        """Validate the released config and ONNX signature before a trial run."""
        import onnxruntime as ort

        config = json.loads((self.policy_dir / "policy.json").read_text())
        expected = {"control_dt": .02, "stiffness": 20.0, "damping": .5,
                    "action_scale": .25, "control_type": "joint_position"}
        if any(config.get(key) != value for key, value in expected.items()):
            raise ValueError("MoE Rough gains, action scale or control period do not match upstream")
        if config.get("policy_joint_names") != list(JOINT_NAMES) or not np.array_equal(
                np.asarray(config.get("default_joint_pos"), dtype=np.float32), DEFAULT_JOINT_POS):
            raise ValueError("MoE Rough joint order or default pose does not match upstream")
        if config.get("onnx") != {"path": "policy.onnx", "input_name": "history", "output_name": "actions"}:
            raise ValueError("MoE Rough ONNX configuration does not match upstream")
        if not math.isclose(self.model.opt.timestep, .002, abs_tol=1e-9):
            raise ValueError("MoE Rough requires a 0.002 s physics timestep")
        if self.data.xmat[self.base, 8] < .7:
            raise ValueError("MoE Rough requires an upright Go2 before loading")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(str(self.policy_dir / "policy.onnx"),
                                       sess_options=options, providers=["CPUExecutionProvider"])
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if (len(inputs) != 1 or inputs[0].name != "history" or
                inputs[0].shape != [1, 225] or inputs[0].type != "tensor(float)" or
                [item.name for item in outputs] != ["actions", "weights", "latent"] or
                outputs[0].shape != [1, 12] or outputs[0].type != "tensor(float)"):
            raise ValueError("MoE Rough ONNX tensor signature does not match upstream")
        history = np.zeros((5, 45), dtype=np.float32)
        history[-1] = build_moe_rough_observation(
            self.data.qvel[self.base_dof+3:self.base_dof+6],
            self.data.qpos[self.base_qpos+3:self.base_qpos+7], np.zeros(3),
            self.data.qpos[self.qpos], self.data.qvel[self.dof], np.zeros(12))
        self._infer(session, history)
        return session

    @staticmethod
    def _infer(session, history: np.ndarray) -> np.ndarray:
        """Reject non-finite or incorrectly shaped actions before they reach motors."""
        output = np.asarray(session.run(["actions"], {"history": history.reshape(1, 225)})[0])
        if output.shape != (1, 12) or output.dtype != np.float32 or not np.isfinite(output).all():
            raise ValueError("MoE Rough ONNX actions must have shape [1,12] and be finite float32")
        return output[0].astype(np.float32)

    def _begin_transition(self) -> None:
        """Blend targets and gains over 0.25 seconds, preserving all physical state."""
        self.transition_time = float(self.data.time)
        self.transition_targets = self.targets.copy()
        self.transition_kp, self.transition_kd = self.kp.copy(), self.kd.copy()
        self.transition_feedforward = self.feedforward.copy()
        self.history[:] = 0
        self.last_action[:] = 0
        self.next_policy_time = float(self.data.time)
        self.velocity_integral[:] = 0
        self.policy_velocity_integral[:] = 0

    def _scripted_command(self, command: np.ndarray) -> np.ndarray:
        """Correct measured gait drift with bounded, anti-windup velocity integration."""
        if np.linalg.norm(command) < 1e-6:
            self.velocity_integral[:] = 0
            return np.zeros(3)
        desired = np.clip(command, -SCRIPTED_COMMAND_LIMITS, SCRIPTED_COMMAND_LIMITS)
        matrix = self.data.xmat[self.base].reshape(3, 3)
        local_velocity = matrix.T @ self.data.qvel[self.base_dof:self.base_dof+3]
        measured = np.r_[local_velocity[:2], self.data.qvel[self.base_dof+5]]
        error = desired - measured
        normalized = desired / SCRIPTED_COMMAND_LIMITS
        raw = normalized + self.velocity_integral
        integrate = (np.abs(raw) < 1) | (raw * error < 0)
        self.velocity_integral += np.array([4, 4, 2])*error*self.model.opt.timestep*integrate
        return np.clip(normalized + self.velocity_integral, -1, 1)*self.gait_blend

    def _tracked_policy_command(self, command: np.ndarray) -> np.ndarray:
        """Compensate policy response so low-speed Twist tracks measured body velocity."""
        desired = np.clip(command, -COMMAND_LIMITS, COMMAND_LIMITS)
        active = np.abs(desired) > 1e-6
        self.policy_velocity_integral[~active] = 0
        if not active.any():
            return np.zeros(3)
        matrix = self.data.xmat[self.base].reshape(3, 3)
        local_velocity = matrix.T @ self.data.qvel[self.base_dof:self.base_dof+3]
        measured = np.r_[local_velocity[:2], self.data.qvel[self.base_dof+5]]
        error = desired - measured
        raw = POLICY_FEEDFORWARD*desired + self.policy_velocity_integral
        integrate = active & ((np.abs(raw) < POLICY_COMMAND_LIMITS) | (raw*error < 0))
        self.policy_velocity_integral += (
            POLICY_INTEGRAL_GAINS*error*CONTROL_DT*integrate)
        self.policy_velocity_integral[:] = np.clip(
            self.policy_velocity_integral, -POLICY_COMMAND_LIMITS, POLICY_COMMAND_LIMITS)
        return np.where(active, np.clip(
            POLICY_FEEDFORWARD*desired + self.policy_velocity_integral,
            -POLICY_COMMAND_LIMITS, POLICY_COMMAND_LIMITS), 0)

    def _policy_command(self, message: dict) -> bool:
        """Load atomically; a failed load retains the currently active controller."""
        try:
            if message.get("id", "moe_rough") != "moe_rough":
                raise ValueError("Only the released moe_rough policy is supported")
            action = message.get("action")
            if action == "load":
                if self.session is None:
                    candidate = self._load_session()
                    self._begin_transition()
                    self.session = candidate
            elif action == "unload":
                if self.session is not None:
                    self._begin_transition()
                    self.session = None
            else:
                raise ValueError("Policy action must be load or unload")
        except Exception as error:
            self.policy_status["error"] = str(error)
            return False
        self.policy_status.update(loaded=self.session is not None, error=None)
        return True

    def _support_torque(self) -> np.ndarray:
        """Support weight through contacting feet using joint Jacobian torques only."""
        contacts = set()
        for contact in self.data.contact:
            if abs(contact.frame[2]) > .5 and contact.dist < .003:
                contacts.update(contact.geom)
        support = [i for i, geom in enumerate(self.foot_geoms) if geom in contacts]
        torque = np.zeros(12)
        if support:
            force = -self.model.opt.gravity * self.model.body_subtreemass[self.base] / len(support)
            jacobian = np.zeros((3, self.model.nv))
            for index in support:
                mujoco.mj_jacBody(self.model, self.data, jacobian, None, self.foot_bodies[index])
                torque -= jacobian[:, self.dof].T @ force
        return torque

    def step(self) -> None:
        """Update gait/inference and apply only bounded motor torques each physics step."""
        command = self.twist.copy() if (
            not self.estopped and self.clock() - self.last_twist_time <= COMMAND_WATCHDOG_S
        ) else np.zeros(3)
        now = float(self.data.time)
        if self.session is not None and now + 1e-9 >= self.next_policy_time:
            try:
                policy_command = self._tracked_policy_command(command)
                self.history[:-1] = self.history[1:]
                self.history[-1] = self._observation(policy_command)
                self.last_action[:] = self._infer(self.session, self.history)
                self.next_policy_time = now + CONTROL_DT
            except Exception as error:
                self._begin_transition()
                self.session = None
                self.twist[:] = 0
                command[:] = 0
                self.policy_status.update(loaded=False, error=str(error))
        if self.session is not None:
            desired = DEFAULT_JOINT_POS + .25 * self.last_action
            kp, kd = np.full(12, 20.0), np.full(12, .5)
            feedforward = np.zeros(12)
        else:
            # Reduce commanded motion before a large roll/pitch error becomes
            # unrecoverable. This is deliberately inactive for the rough-
            # terrain policy, which owns its own attitude recovery on stairs.
            upright = float(self.data.xmat[self.base, 8])
            upright_scale = np.clip(
                (upright-SCRIPTED_UPRIGHT_RECOVERY_COS)
                / (SCRIPTED_UPRIGHT_FULL_SPEED_COS-SCRIPTED_UPRIGHT_RECOVERY_COS),
                0, 1)
            command *= upright_scale
            active = float(np.linalg.norm(command) > 1e-6)
            rate = 5.0 if active else 7.0
            self.gait_blend += (active - self.gait_blend) * min(1.0, rate*self.model.opt.timestep)
            if self.gait_blend > 1e-3:
                self.phase += math.tau * 2.0 * self.model.opt.timestep
            desired = compute_go2_targets(self._scripted_command(command), self.phase)
            kp, kd = SCRIPTED_KP, SCRIPTED_KD
            feedforward = self._support_torque()
        blend = np.clip((now - self.transition_time) / .25, 0, 1)
        self.targets = self.transition_targets*(1-blend) + desired*blend
        self.kp = self.transition_kp*(1-blend) + kp*blend
        self.kd = self.transition_kd*(1-blend) + kd*blend
        self.feedforward = self.transition_feedforward*(1-blend) + feedforward*blend
        torque = (self.kp*(self.targets - self.data.qpos[self.qpos])
                  - self.kd*self.data.qvel[self.dof] + self.feedforward)
        limits = self.model.actuator_ctrlrange[self.actuators]
        self.data.ctrl[self.actuators] = np.clip(torque, limits[:, 0], limits[:, 1])

    def state(self) -> dict:
        """Publish the bridge base format: world linear and body angular velocity."""
        q, v = self.base_qpos, self.base_dof
        return {
            "timestamp": float(self.data.time), "externalControl": True, "estopped": self.estopped,
            "base": {
                "position": self.data.qpos[q:q+3].tolist(),
                "quaternion": self.data.qpos[q+3:q+7].tolist(),
                "linearVelocity": self.data.qvel[v:v+3].tolist(),
                "angularVelocity": self.data.qvel[v+3:v+6].tolist(),
            },
            "policy": dict(self.policy_status),
            "joints": {
                "names": list(JOINT_NAMES),
                "positions": self.data.qpos[self.qpos].tolist(),
                "velocities": self.data.qvel[self.dof].tolist(),
            },
        }
