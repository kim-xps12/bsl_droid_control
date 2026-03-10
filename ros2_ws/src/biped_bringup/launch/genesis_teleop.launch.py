"""Genesis teleop launch: Genesis simulation with gamepad control.

Launches:
  - robot_state_publisher (URDF -> TF)
  - genesis_sim_node (Genesis environment node: physics + obs construction)
  - joy_node (F710 gamepad)
  - teleop_twist_joy_node (joystick -> /cmd_vel)
  - biped_joy_safety_node (emergency stop / disconnect)
  - biped_rl_policy_node (sim mode: /policy_obs -> inference -> /policy_actions)

The genesis_sim_node and rl_policy_node form a synchronous event-driven loop:
  genesis publishes /policy_obs -> policy infers -> publishes /policy_actions
  -> genesis receives, steps physics, publishes new /policy_obs -> ...
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare("biped_description")
    pkg_bringup = FindPackageShare("biped_bringup")

    urdf_file = PathJoinSubstitution(
        [pkg_description, "urdf", "bsl_droid_simplified_v2.urdf.xacro"]
    )
    joy_config = PathJoinSubstitution([pkg_bringup, "config", "joy_f710.yaml"])
    genesis_config = PathJoinSubstitution([pkg_bringup, "config", "genesis_sim.yaml"])

    # Default URDF path for Genesis (plain URDF, not xacro)
    repo_root = Path(__file__).resolve().parents[4]
    default_genesis_urdf = str(repo_root / "rl_ws" / "assets" / "bsl_droid_simplified_v2.urdf")

    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        "model_path", default_value="", description="Path to RL policy checkpoint (.pt)"
    )
    show_viewer_arg = DeclareLaunchArgument(
        "show_viewer", default_value="true", description="Show Genesis viewer"
    )
    genesis_urdf_arg = DeclareLaunchArgument(
        "genesis_urdf",
        default_value=default_genesis_urdf,
        description="Path to plain URDF for Genesis (not xacro)",
    )

    model_path = LaunchConfiguration("model_path")
    show_viewer = LaunchConfiguration("show_viewer")
    genesis_urdf = LaunchConfiguration("genesis_urdf")

    # robot_state_publisher (for TF visualization, uses xacro URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_file]), value_type=str
                ),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # genesis_sim_node (environment node: physics + obs construction)
    genesis_sim = Node(
        package="biped_genesis_sim",
        executable="genesis_sim_node",
        parameters=[
            genesis_config,
            {
                "urdf_path": genesis_urdf,
                "show_viewer": show_viewer,
                "use_sim_time": False,  # This node is the clock source
            },
        ],
        output="screen",
    )

    # joy_node (gamepad driver)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_config, {"use_sim_time": True}],
        output="screen",
    )

    # teleop_twist_joy (joystick -> /cmd_vel)
    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_config, {"use_sim_time": True}],
        output="screen",
    )

    # biped_joy_safety_node (emergency stop)
    joy_safety = Node(
        package="biped_teleop",
        executable="biped_joy_safety_node",
        parameters=[joy_config, {"use_sim_time": True}],
        output="screen",
    )

    # biped_rl_policy_node (sim mode: pure inference via /policy_obs -> /policy_actions)
    rl_policy = Node(
        package="biped_rl_policy",
        executable="biped_rl_policy_node",
        parameters=[
            {"mode": "sim", "model_path": model_path, "use_sim_time": True},
        ],
        output="screen",
    )

    # Staged startup:
    # t=0: robot_state_publisher, genesis_sim_node
    # t=3: rl_policy + joy + teleop (after Genesis initializes)

    # Shutdown entire launch when genesis_sim exits (prevents orphan processes)
    shutdown_on_sim_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=genesis_sim,
            on_exit=[Shutdown(reason='genesis_sim_node exited')],
        )
    )

    return LaunchDescription(
        [
            model_path_arg,
            show_viewer_arg,
            genesis_urdf_arg,
            robot_state_publisher,
            genesis_sim,
            TimerAction(
                period=3.0,
                actions=[rl_policy, joy_node, teleop_twist_joy, joy_safety],
            ),
            shutdown_on_sim_exit,
        ]
    )
