"""RL policy inference node for walking gait generation.

Modes:
  - viz: RViz-only visualization with mock sensors (no physics)
  - sim: Pure inference — receives /policy_obs, publishes /policy_actions
  - control: Real hardware (future, same interface as sim)

In sim mode, the genesis_sim_node constructs the observation vector and
publishes it on /policy_obs. This node only performs inference.

Reference:
  - rl_ws/biped_walking/envs/droid_env_unitree.py (observation vector)
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from biped_rl_policy.actor_mlp import ActorMLP

# Deferred import — torch is heavy
torch = None  # type: ignore[assignment]


def _ensure_torch():
    global torch
    if torch is None:
        import torch as _torch

        torch = _torch


NUM_OBS = 50
NUM_ACTIONS = 10


class BipedRLPolicyNode(Node):
    """RL policy inference node for walking gait generation."""

    def __init__(self) -> None:
        super().__init__("biped_rl_policy_node")
        _ensure_torch()

        # ----- Parameters -----
        self.declare_parameter("mode", "viz")
        self.declare_parameter("model_path", "")
        self.declare_parameter("policy_rate", 50.0)
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("gait_frequency", 1.5)
        self.declare_parameter("obs_scales.lin_vel", 2.0)
        self.declare_parameter("obs_scales.ang_vel", 0.25)
        self.declare_parameter("obs_scales.dof_pos", 1.0)
        self.declare_parameter("obs_scales.dof_vel", 0.05)
        self.declare_parameter("commands_scale", [2.0, 2.0, 0.25])
        self.declare_parameter(
            "default_dof_pos",
            [0.0, 0.0, 1.047, -1.745, 0.785, 0.0, 0.0, 1.047, -1.745, 0.785],
        )
        self.declare_parameter(
            "joint_names",
            [
                "left_hip_yaw_joint",
                "left_hip_roll_joint",
                "left_hip_pitch_joint",
                "left_knee_pitch_joint",
                "left_ankle_pitch_joint",
                "right_hip_yaw_joint",
                "right_hip_roll_joint",
                "right_hip_pitch_joint",
                "right_knee_pitch_joint",
                "right_ankle_pitch_joint",
            ],
        )

        self._mode: str = self.get_parameter("mode").value
        self._model_path: str = self.get_parameter("model_path").value
        self._policy_rate: float = self.get_parameter("policy_rate").value
        self._dt = 1.0 / self._policy_rate
        self._emergency_stop = False

        # ----- Load model -----
        self._actor: ActorMLP | None = None
        if self._model_path:
            self._load_model(self._model_path)
        else:
            self.get_logger().warn("No model_path specified. Policy will output zeros.")

        # ----- Mode-specific setup -----
        if self._mode == "sim":
            self._setup_sim_mode()
        elif self._mode == "viz":
            self._setup_viz_mode()
        else:
            self.get_logger().error(f"Unknown mode: {self._mode}")

        # ----- Common subscribers -----
        self._estop_sub = self.create_subscription(Bool, "/emergency_stop", self._estop_cb, 10)

        self.get_logger().info(
            f"RL policy node started (mode={self._mode}, rate={self._policy_rate}Hz)"
        )

    def _setup_sim_mode(self) -> None:
        """Sim mode: pure inference. Obs from env node, actions to env node."""
        self._action_pub = self.create_publisher(Float64MultiArray, "/policy_actions", 1)
        self._obs_sub = self.create_subscription(
            Float64MultiArray, "/policy_obs", self._policy_obs_cb, 1
        )

    def _setup_viz_mode(self) -> None:
        """Viz mode: self-contained obs construction + inference + /joint_states."""
        self._action_scale: float = self.get_parameter("action_scale").value
        self._gait_frequency: float = self.get_parameter("gait_frequency").value
        self._obs_scale_lin_vel: float = self.get_parameter("obs_scales.lin_vel").value
        self._obs_scale_ang_vel: float = self.get_parameter("obs_scales.ang_vel").value
        self._obs_scale_dof_pos: float = self.get_parameter("obs_scales.dof_pos").value
        self._obs_scale_dof_vel: float = self.get_parameter("obs_scales.dof_vel").value
        self._commands_scale = np.array(self.get_parameter("commands_scale").value, dtype=np.float64)
        self._default_dof_pos = np.array(self.get_parameter("default_dof_pos").value, dtype=np.float64)
        self._joint_names: list[str] = list(self.get_parameter("joint_names").value)

        self._gait_phase = 0.0
        self._prev_actions = np.zeros(NUM_ACTIONS, dtype=np.float64)
        self._commands = np.zeros(3, dtype=np.float64)
        self._dof_pos = self._default_dof_pos.copy()
        self._dof_vel = np.zeros(NUM_ACTIONS, dtype=np.float64)
        self._base_lin_vel = np.zeros(3, dtype=np.float64)
        self._base_ang_vel = np.zeros(3, dtype=np.float64)
        self._projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        self._feet_pos_z = np.zeros(2, dtype=np.float64)
        self._contact_state = np.ones(2, dtype=np.float64)

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self._timer = self.create_timer(self._dt, self._viz_step)

    def _load_model(self, path: str) -> None:
        try:
            self._actor = ActorMLP.from_checkpoint(path, num_obs=NUM_OBS, num_actions=NUM_ACTIONS)
            self.get_logger().info(f"Loaded model: {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self._actor = None

    # ---- Common callbacks ----

    def _estop_cb(self, msg: Bool) -> None:
        if msg.data and not self._emergency_stop:
            self.get_logger().warn("Emergency stop received")
        self._emergency_stop = msg.data

    # ---- Sim mode: event-driven inference ----

    def _policy_obs_cb(self, msg: Float64MultiArray) -> None:
        """Receive observation from env node, run inference, publish actions."""
        if len(msg.data) != NUM_OBS:
            return

        if self._emergency_stop:
            action_msg = Float64MultiArray()
            action_msg.data = [0.0] * NUM_ACTIONS
            self._action_pub.publish(action_msg)
            return

        if self._actor is not None:
            obs_tensor = torch.tensor(msg.data, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                actions = self._actor(obs_tensor).squeeze(0).numpy()
        else:
            actions = np.zeros(NUM_ACTIONS, dtype=np.float32)

        action_msg = Float64MultiArray()
        action_msg.data = actions.tolist()
        self._action_pub.publish(action_msg)

    # ---- Viz mode: self-contained timer-driven ----

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._commands[0] = msg.linear.x
        self._commands[1] = msg.linear.y
        self._commands[2] = msg.angular.z

    def _viz_step(self) -> None:
        """Viz mode: construct obs, run inference, publish /joint_states."""
        if self._emergency_stop:
            self._publish_joint_state(self._default_dof_pos)
            return

        self._gait_phase = (self._gait_phase + self._dt * self._gait_frequency) % 1.0
        leg_phase_l = self._gait_phase
        leg_phase_r = (self._gait_phase + 0.5) % 1.0

        obs = np.concatenate([
            self._base_lin_vel * self._obs_scale_lin_vel,
            self._base_ang_vel * self._obs_scale_ang_vel,
            self._projected_gravity,
            self._commands * self._commands_scale,
            (self._dof_pos - self._default_dof_pos) * self._obs_scale_dof_pos,
            self._dof_vel * self._obs_scale_dof_vel,
            self._prev_actions,
            [math.sin(self._gait_phase * 2.0 * math.pi)],
            [math.cos(self._gait_phase * 2.0 * math.pi)],
            [leg_phase_l, leg_phase_r],
            self._feet_pos_z,
            self._contact_state,
        ])

        if self._actor is not None:
            obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                actions = self._actor(obs_tensor).squeeze(0).numpy()
        else:
            actions = np.zeros(NUM_ACTIONS, dtype=np.float64)

        actions = np.clip(actions, -10.0, 10.0)
        self._prev_actions = actions.copy()

        target_positions = actions * self._action_scale + self._default_dof_pos
        self._dof_pos = target_positions.copy()
        self._publish_joint_state(target_positions)

    def _publish_joint_state(self, positions: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = positions.tolist()
        self._joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BipedRLPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
