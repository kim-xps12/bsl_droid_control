"""Genesis environment node for BSL-Droid.

Action-driven ROS 2 wrapper for the Genesis physics engine.
Receives raw actions from the RL policy node, steps physics,
constructs the 50-dim observation vector, and publishes it back.
This matches the training loop in droid_env_unitree.py exactly.

Published topics:
  - /policy_obs (Float64MultiArray) — 50-dim observation vector
  - /joint_states (JointState) — joint positions and velocities (for TF)
  - /clock (Clock) — simulation time

Subscribed topics:
  - /policy_actions (Float64MultiArray) — 10-dim raw actions from policy
  - /cmd_vel (Twist) — velocity commands from teleop

Reference:
  - rl_ws/biped_walking/envs/droid_env_unitree.py (step function, obs construction)
  - rl_ws/biped_walking/biped_eval.py (debug arrow visualization)
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Deferred imports — genesis and torch are heavy
gs = None  # type: ignore[assignment]
torch = None  # type: ignore[assignment]


def _ensure_genesis() -> None:
    global gs, torch
    if gs is None:
        import genesis as _gs
        import torch as _torch

        gs = _gs
        torch = _torch


NUM_ACTIONS = 10
NUM_OBS = 50


class GenesisSimNode(Node):
    """Genesis environment node for BSL-Droid.

    Acts as the environment (like droid_env_unitree.py): receives actions,
    steps physics with action latency, constructs observations, and publishes.
    The rl_policy_node acts as a pure inference wrapper.
    """

    def __init__(self) -> None:
        super().__init__("genesis_sim_node")
        _ensure_genesis()

        # ----- Physics parameters -----
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("dt", 0.02)
        self.declare_parameter("substeps", 2)
        self.declare_parameter("kp", 35.0)
        self.declare_parameter("kd", 2.0)
        self.declare_parameter("base_init_pos", [0.0, 0.0, 0.35])
        self.declare_parameter("base_init_quat", [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter("show_viewer", True)
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
        self.declare_parameter("kp_override_names", ["left_knee_pitch_joint", "right_knee_pitch_joint"])
        self.declare_parameter("kp_override_values", [50.0, 50.0])
        self.declare_parameter("kd_override_names", ["left_ankle_pitch_joint", "right_ankle_pitch_joint"])
        self.declare_parameter("kd_override_values", [8.0, 8.0])
        self.declare_parameter("feet_names", ["left_foot_link", "right_foot_link"])
        self.declare_parameter("contact_threshold", 0.05)
        self.declare_parameter("hip_roll_inward_limit", 0.0)

        # ----- Observation parameters (from training config) -----
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("gait_frequency", 1.5)
        self.declare_parameter("obs_scales.lin_vel", 2.0)
        self.declare_parameter("obs_scales.ang_vel", 0.25)
        self.declare_parameter("obs_scales.dof_pos", 1.0)
        self.declare_parameter("obs_scales.dof_vel", 0.05)
        self.declare_parameter("commands_scale", [2.0, 2.0, 0.25])

        # Read all parameters
        self._urdf_path: str = self.get_parameter("urdf_path").value
        self._dt: float = self.get_parameter("dt").value
        self._substeps: int = self.get_parameter("substeps").value
        self._kp: float = self.get_parameter("kp").value
        self._kd: float = self.get_parameter("kd").value
        self._base_init_pos: list[float] = list(self.get_parameter("base_init_pos").value)
        self._base_init_quat: list[float] = list(self.get_parameter("base_init_quat").value)
        self._show_viewer: bool = self.get_parameter("show_viewer").value
        self._default_dof_pos = np.array(self.get_parameter("default_dof_pos").value, dtype=np.float64)
        self._joint_names: list[str] = list(self.get_parameter("joint_names").value)

        kp_override_names: list[str] = list(self.get_parameter("kp_override_names").value)
        kp_override_values: list[float] = list(self.get_parameter("kp_override_values").value)
        self._kp_overrides: dict[str, float] = dict(zip(kp_override_names, kp_override_values))

        kd_override_names: list[str] = list(self.get_parameter("kd_override_names").value)
        kd_override_values: list[float] = list(self.get_parameter("kd_override_values").value)
        self._kd_overrides: dict[str, float] = dict(zip(kd_override_names, kd_override_values))
        self._feet_names: list[str] = list(self.get_parameter("feet_names").value)
        self._contact_threshold: float = self.get_parameter("contact_threshold").value
        self._hip_roll_inward_limit: float = self.get_parameter("hip_roll_inward_limit").value

        # Hip roll joint indices in joint_names order (droid_env_unitree.py:324,329)
        self._LEFT_HIP_ROLL_IDX = 1
        self._RIGHT_HIP_ROLL_IDX = 6

        self._action_scale: float = self.get_parameter("action_scale").value
        self._gait_frequency: float = self.get_parameter("gait_frequency").value
        self._obs_scale_lin_vel: float = self.get_parameter("obs_scales.lin_vel").value
        self._obs_scale_ang_vel: float = self.get_parameter("obs_scales.ang_vel").value
        self._obs_scale_dof_pos: float = self.get_parameter("obs_scales.dof_pos").value
        self._obs_scale_dof_vel: float = self.get_parameter("obs_scales.dof_vel").value
        self._commands_scale = np.array(self.get_parameter("commands_scale").value, dtype=np.float64)

        if not self._urdf_path:
            self.get_logger().fatal("urdf_path parameter is required")
            raise RuntimeError("urdf_path parameter is required")

        # ----- Environment state (matches droid_env_unitree.py) -----
        self._last_actions = np.zeros(NUM_ACTIONS, dtype=np.float64)
        self._gait_phase = 0.0
        self._commands = np.zeros(3, dtype=np.float64)  # [vx, vy, vyaw]
        self._sim_time = 0.0
        self._loop_started = False

        # Debug arrow state (biped_eval.py:451-454)
        self._cmd_arrow_node = None
        self._vel_arrow_node = None
        self._vel_ema = np.zeros(3, dtype=np.float64)
        self._VEL_EMA_ALPHA = 0.05

        # ----- Initialize Genesis -----
        self._init_genesis()

        # ----- Publishers -----
        self._obs_pub = self.create_publisher(Float64MultiArray, "/policy_obs", 1)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)

        # ----- Subscribers -----
        self._action_sub = self.create_subscription(
            Float64MultiArray, "/policy_actions", self._action_cb, 1
        )
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, 10
        )

        # Periodically publish initial obs until the control loop starts
        self._init_timer = self.create_timer(0.1, self._publish_initial_obs_tick)

        if self._hip_roll_inward_limit < 0.0:
            self.get_logger().info(
                f"Hip roll inward limit enabled: {self._hip_roll_inward_limit} rad "
                f"({self._hip_roll_inward_limit * 180.0 / math.pi:.1f} deg)"
            )

        self.get_logger().info(
            f"Genesis env node started (dt={self._dt}s, urdf={self._urdf_path})"
        )

    def _init_genesis(self) -> None:
        """Initialize Genesis scene, robot, and PD controllers."""
        try:
            gs.init(backend=gs.gpu, logging_level="warning")
        except Exception:
            gs.init(backend=gs.cpu, logging_level="warning")

        self._scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=self._dt,
                substeps=self._substeps,
            ),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=False,
                tolerance=1e-5,
                max_collision_pairs=20,
            ),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(2.0, 0.0, 1.0),
                camera_lookat=(0.0, 0.0, 0.3),
                camera_fov=40,
                max_FPS=int(1.0 / self._dt),
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=[0]),
            show_viewer=self._show_viewer,
        )

        # Ground plane
        self._scene.add_entity(
            gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True)
        )

        # Robot
        self._robot = self._scene.add_entity(
            gs.morphs.URDF(
                file=self._urdf_path,
                pos=self._base_init_pos,
                quat=self._base_init_quat,
            ),
        )

        # Contact sensors (droid_env_unitree.py:208-222, must be added before scene.build)
        self._contact_sensors: list = []
        for foot_name in self._feet_names:
            try:
                link = self._robot.get_link(foot_name)
                sensor = self._scene.add_sensor(
                    gs.sensors.Contact(
                        entity_idx=self._robot.idx,
                        link_idx_local=link.idx_local,
                    )
                )
                self._contact_sensors.append(sensor)
            except Exception:
                self.get_logger().warn(f"Failed to add contact sensor for '{foot_name}'")

        # Build scene (single environment)
        self._scene.build(n_envs=1)

        # Motor DOF indices (droid_env_unitree.py:228-233)
        self._motors_dof_idx = torch.tensor(
            [self._robot.get_joint(name).dofs_idx_local[0] for name in self._joint_names],
            dtype=gs.tc_int,
            device=gs.device,
        )
        self._actions_dof_idx = torch.argsort(self._motors_dof_idx)

        # PD gains with per-joint override (droid_env_unitree.py:236-256)
        kp_values = [self._kp] * NUM_ACTIONS
        for joint_name, kp_val in self._kp_overrides.items():
            if joint_name in self._joint_names:
                idx = self._joint_names.index(joint_name)
                kp_values[idx] = kp_val
        self._robot.set_dofs_kp(kp_values, self._motors_dof_idx)

        kd_values = [self._kd] * NUM_ACTIONS
        for joint_name, kd_val in self._kd_overrides.items():
            if joint_name in self._joint_names:
                idx = self._joint_names.index(joint_name)
                kd_values[idx] = kd_val
        self._robot.set_dofs_kv(kd_values, self._motors_dof_idx)

        # Initial joint positions
        default_dof_tensor = torch.tensor(
            self._default_dof_pos, dtype=gs.tc_float, device=gs.device
        ).unsqueeze(0)
        self._robot.set_dofs_position(default_dof_tensor[:, self._actions_dof_idx], self._motors_dof_idx)

        # PD targets to default pose (prevents collapse if physics steps before first _action_cb)
        self._robot.control_dofs_position(
            default_dof_tensor[:, self._actions_dof_idx],
            slice(6, 6 + NUM_ACTIONS),
        )

        # Feet link indices (droid_env_unitree.py:274-284)
        self._feet_indices: list[int] = []
        for name in self._feet_names:
            try:
                link = self._robot.get_link(name)
                self._feet_indices.append(link.idx_local)
            except Exception:
                self.get_logger().warn(f"Foot link '{name}' not found in URDF")

        self.get_logger().info("Genesis scene initialized successfully")

    # ---- Callbacks ----

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._commands[0] = msg.linear.x
        self._commands[1] = msg.linear.y
        self._commands[2] = msg.angular.z

    def _action_cb(self, msg: Float64MultiArray) -> None:
        """Receive actions from policy, step physics, publish observation.

        This is the core environment step, matching droid_env_unitree.py:477-624.
        """
        if len(msg.data) != NUM_ACTIONS:
            return

        if not self._loop_started:
            self._loop_started = True
            self.get_logger().info("Control loop started (first action received)")

        actions = np.clip(np.array(msg.data, dtype=np.float64), -10.0, 10.0)

        # Action latency simulation (droid_env_unitree.py:486)
        # Apply PREVIOUS step's actions, not current
        exec_actions = self._last_actions
        target_dof_pos = exec_actions * self._action_scale + self._default_dof_pos

        # Hip roll PDターゲットクランプ (droid_env_unitree.py:534-538, 全位相モード)
        if self._hip_roll_inward_limit < 0.0:
            target_dof_pos[self._LEFT_HIP_ROLL_IDX] = max(
                target_dof_pos[self._LEFT_HIP_ROLL_IDX], self._hip_roll_inward_limit
            )
            target_dof_pos[self._RIGHT_HIP_ROLL_IDX] = min(
                target_dof_pos[self._RIGHT_HIP_ROLL_IDX], -self._hip_roll_inward_limit
            )

        # Apply PD targets and step physics (droid_env_unitree.py:543-544)
        target_tensor = torch.tensor(
            target_dof_pos, dtype=gs.tc_float, device=gs.device
        ).unsqueeze(0)
        self._robot.control_dofs_position(
            target_tensor[:, self._actions_dof_idx],
            slice(6, 6 + NUM_ACTIONS),
        )
        self._scene.step()
        self._sim_time += self._dt

        # Extract state (droid_env_unitree.py:548-559)
        base_pos = self._robot.get_pos()  # (1, 3)
        base_quat_tensor = self._robot.get_quat()  # (1, 4) [w, x, y, z]
        base_vel_world = self._robot.get_vel()  # (1, 3)
        base_ang_world = self._robot.get_ang()  # (1, 3)
        dof_pos = self._robot.get_dofs_position(self._motors_dof_idx)  # (1, 10)
        dof_vel = self._robot.get_dofs_velocity(self._motors_dof_idx)  # (1, 10)

        # To numpy (single env)
        pos_np = base_pos[0].cpu().numpy().astype(np.float64)
        quat_wxyz = base_quat_tensor[0].cpu().numpy().astype(np.float64)
        vel_world = base_vel_world[0].cpu().numpy().astype(np.float64)
        ang_world = base_ang_world[0].cpu().numpy().astype(np.float64)
        positions = dof_pos[0].cpu().numpy().astype(np.float64)
        velocities = dof_vel[0].cpu().numpy().astype(np.float64)

        # Body-frame transforms (droid_env_unitree.py:553-556)
        inv_q = _inv_quat(quat_wxyz)
        base_lin_vel = _rotate_by_quat(vel_world, inv_q)
        base_ang_vel = _rotate_by_quat(ang_world, inv_q)
        # Projected gravity: unit vector [0,0,-1] rotated to body frame
        projected_gravity = _rotate_by_quat(np.array([0.0, 0.0, -1.0]), inv_q)

        # Feet state (droid_env_unitree.py:467-475, 726-727)
        feet_pos_z = np.zeros(2, dtype=np.float64)
        if self._feet_indices:
            link_pos = self._robot.get_links_pos()  # (1, n_links, 3)
            for i, idx in enumerate(self._feet_indices):
                feet_pos_z[i] = float(link_pos[0, idx, 2].cpu().numpy())

        # Contact state via Contact Sensor (droid_env_unitree.py:446-458)
        contact_state = np.ones(2, dtype=np.float64)
        if self._contact_sensors:
            for i, sensor in enumerate(self._contact_sensors):
                contact = sensor.read()  # (1, 1) for single env
                contact_state[i] = 1.0 if float(contact.flatten()[0]) > 0.5 else 0.0
        elif self._feet_indices:
            # Fallback: Z-threshold (pre-V20 compat)
            for i, idx in enumerate(self._feet_indices):
                contact_state[i] = 1.0 if feet_pos_z[i] < self._contact_threshold else 0.0

        # Gait phase update (droid_env_unitree.py:562-566)
        self._gait_phase = (self._gait_phase + self._dt * self._gait_frequency) % 1.0
        leg_phase_l = self._gait_phase
        leg_phase_r = (self._gait_phase + 0.5) % 1.0

        # Build 50-dim observation (droid_env_unitree.py:714-730)
        obs = np.concatenate([
            base_lin_vel * self._obs_scale_lin_vel,                          # [0:3]
            base_ang_vel * self._obs_scale_ang_vel,                          # [3:6]
            projected_gravity,                                                # [6:9]
            self._commands * self._commands_scale,                           # [9:12]
            (positions - self._default_dof_pos) * self._obs_scale_dof_pos,   # [12:22]
            velocities * self._obs_scale_dof_vel,                            # [22:32]
            actions,                                                          # [32:42]
            [math.sin(self._gait_phase * 2.0 * math.pi)],                   # [42]
            [math.cos(self._gait_phase * 2.0 * math.pi)],                   # [43]
            [leg_phase_l, leg_phase_r],                                       # [44:46]
            feet_pos_z,                                                       # [46:48]
            contact_state,                                                    # [48:50]
        ])

        # Update action buffer (droid_env_unitree.py:619)
        self._last_actions = actions.copy()

        # Publish observation
        obs_msg = Float64MultiArray()
        obs_msg.data = obs.tolist()
        self._obs_pub.publish(obs_msg)

        # Publish JointState for robot_state_publisher
        sim_time_msg = _make_time_msg(self._sim_time)
        js_msg = JointState()
        js_msg.header.stamp = sim_time_msg
        js_msg.name = self._joint_names
        js_msg.position = positions.tolist()
        js_msg.velocity = velocities.tolist()
        self._joint_state_pub.publish(js_msg)

        # Publish Clock
        clock_msg = Clock()
        clock_msg.clock = sim_time_msg
        self._clock_pub.publish(clock_msg)

        # Debug arrows (biped_eval.py:520-561)
        if self._show_viewer:
            self._draw_debug_arrows(pos_np, quat_wxyz, base_lin_vel)

    def _publish_initial_obs_tick(self) -> None:
        """Timer callback: re-publish initial obs until the loop starts."""
        if self._loop_started:
            self.destroy_timer(self._init_timer)
            return
        self._publish_initial_obs()

    def _publish_initial_obs(self) -> None:
        """Publish initial observation to kick off the policy-env loop."""
        # Extract initial state (before any physics step)
        base_quat_tensor = self._robot.get_quat()
        base_vel_world = self._robot.get_vel()
        base_ang_world = self._robot.get_ang()
        dof_pos = self._robot.get_dofs_position(self._motors_dof_idx)
        dof_vel = self._robot.get_dofs_velocity(self._motors_dof_idx)

        quat_wxyz = base_quat_tensor[0].cpu().numpy().astype(np.float64)
        vel_world = base_vel_world[0].cpu().numpy().astype(np.float64)
        ang_world = base_ang_world[0].cpu().numpy().astype(np.float64)
        positions = dof_pos[0].cpu().numpy().astype(np.float64)
        velocities = dof_vel[0].cpu().numpy().astype(np.float64)

        inv_q = _inv_quat(quat_wxyz)
        base_lin_vel = _rotate_by_quat(vel_world, inv_q)
        base_ang_vel = _rotate_by_quat(ang_world, inv_q)
        projected_gravity = _rotate_by_quat(np.array([0.0, 0.0, -1.0]), inv_q)

        # Match training env reset state (droid_env_unitree.py:650-654):
        # feet_pos, contact_state, leg_phase are all zeroed during reset
        feet_pos_z = np.zeros(2, dtype=np.float64)
        contact_state = np.zeros(2, dtype=np.float64)

        obs = np.concatenate([
            base_lin_vel * self._obs_scale_lin_vel,
            base_ang_vel * self._obs_scale_ang_vel,
            projected_gravity,
            self._commands * self._commands_scale,
            (positions - self._default_dof_pos) * self._obs_scale_dof_pos,
            velocities * self._obs_scale_dof_vel,
            np.zeros(NUM_ACTIONS, dtype=np.float64),  # initial actions = zeros
            [math.sin(self._gait_phase * 2.0 * math.pi)],
            [math.cos(self._gait_phase * 2.0 * math.pi)],
            [0.0, 0.0],  # leg_phase zeroed to match training env reset
            feet_pos_z,
            contact_state,
        ])

        obs_msg = Float64MultiArray()
        obs_msg.data = obs.tolist()
        self._obs_pub.publish(obs_msg)

        # Publish initial clock
        clock_msg = Clock()
        clock_msg.clock = _make_time_msg(0.0)
        self._clock_pub.publish(clock_msg)

    def _draw_debug_arrows(
        self, pos: np.ndarray, quat_wxyz: np.ndarray, base_lin_vel: np.ndarray
    ) -> None:
        """Draw velocity arrows above the robot (biped_eval.py:520-561)."""
        # Clear previous arrows
        if self._cmd_arrow_node is not None:
            self._scene.clear_debug_object(self._cmd_arrow_node)
            self._cmd_arrow_node = None
        if self._vel_arrow_node is not None:
            self._scene.clear_debug_object(self._vel_arrow_node)
            self._vel_arrow_node = None

        # Arrow origin: above robot head
        arrow_pos = pos.copy()
        arrow_pos[2] += 0.4

        # Green arrow: target velocity command (body → world)
        cmd_body = np.array([self._commands[0], self._commands[1], 0.0])
        cmd_world = _rotate_by_quat(cmd_body, quat_wxyz)
        cmd_world[2] = 0.0  # keep horizontal
        if np.linalg.norm(cmd_world) > 0.01:
            self._cmd_arrow_node = self._scene.draw_debug_arrow(
                pos=tuple(arrow_pos),
                vec=tuple(cmd_world),
                radius=0.008,
                color=(0.0, 1.0, 0.0, 0.8),
            )

        # Blue arrow: actual velocity with EMA smoothing (body → world)
        vel_body = np.array([base_lin_vel[0], base_lin_vel[1], 0.0])
        vel_world_raw = _rotate_by_quat(vel_body, quat_wxyz)
        vel_world_raw[2] = 0.0
        self._vel_ema = self._VEL_EMA_ALPHA * vel_world_raw + (1.0 - self._VEL_EMA_ALPHA) * self._vel_ema
        if np.linalg.norm(self._vel_ema) > 0.01:
            self._vel_arrow_node = self._scene.draw_debug_arrow(
                pos=tuple(arrow_pos),
                vec=tuple(self._vel_ema),
                radius=0.006,
                color=(0.2, 0.5, 1.0, 0.8),
            )


def _inv_quat(q: np.ndarray) -> np.ndarray:
    """Invert a unit quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _rotate_by_quat(v: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q [w, x, y, z]."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    vx, vy, vz = v[0], v[1], v[2]
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return np.array([rx, ry, rz], dtype=np.float64)


def _make_time_msg(t: float) -> Time:
    """Convert float seconds to builtin_interfaces/Time."""
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    msg = Time()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GenesisSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
