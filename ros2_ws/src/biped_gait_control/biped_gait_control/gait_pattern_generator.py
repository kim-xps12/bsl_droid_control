#!/usr/bin/env python3
"""
ROS 2 Gait Pattern Generator Node.

This node generates walking gait patterns for a digitigrade biped robot
and publishes joint commands at a configurable rate (default 50Hz).

Published Topics:
    /joint_states (sensor_msgs/JointState):
        Joint angle targets for all leg joints.
        Standard topic for robot_state_publisher integration.

    /joint_trajectory (trajectory_msgs/JointTrajectory):
        Alternative output for trajectory controllers.

    /foot_markers (visualization_msgs/MarkerArray):
        Visualization markers for foot trajectories (for RViz).

Parameters:
    publish_rate (double): Publishing rate in Hz (default: 50.0)
    step_height (double): Foot lift height in meters (default: 0.04)
    step_length (double): Stride length in meters (default: 0.08)
    step_frequency (double): Walking frequency in Hz (default: 0.5)
    leg_extension_ratio (double): Standing leg extension ratio (default: 0.90)
    thigh_length (double): Thigh link length in meters (default: 0.18)
    shank_length (double): Shank link length in meters (default: 0.20)
    enabled (bool): Enable/disable walking (default: true)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

import numpy as np
from typing import List

from .trajectory import GaitParameters, WalkingPatternGenerator
from .kinematics import BipedKinematics, LinkLengths


class GaitPatternGeneratorNode(Node):
    """ROS 2 node for generating biped walking gait patterns."""

    # Joint names matching URDF (biped_digitigrade.urdf.xacro)
    LEFT_JOINT_NAMES = [
        'left_hip_yaw_joint',
        'left_hip_roll_joint',
        'left_hip_pitch_joint',
        'left_knee_pitch_joint',
        'left_ankle_pitch_joint',
    ]
    RIGHT_JOINT_NAMES = [
        'right_hip_yaw_joint',
        'right_hip_roll_joint',
        'right_hip_pitch_joint',
        'right_knee_pitch_joint',
        'right_ankle_pitch_joint',
    ]

    def __init__(self):
        super().__init__('gait_pattern_generator')

        # Declare parameters
        self._declare_parameters()

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        step_height = self.get_parameter('step_height').value
        step_length = self.get_parameter('step_length').value
        step_frequency = self.get_parameter('step_frequency').value
        leg_extension_ratio = self.get_parameter('leg_extension_ratio').value
        thigh_length = self.get_parameter('thigh_length').value
        shank_length = self.get_parameter('shank_length').value
        self.enabled = self.get_parameter('enabled').value

        # Create gait parameters
        gait_params = GaitParameters(
            step_height=step_height,
            step_length=step_length,
            step_frequency=step_frequency,
            leg_extension_ratio=leg_extension_ratio
        )

        # Initialize walking pattern generator
        self.walking_generator = WalkingPatternGenerator(
            gait_params=gait_params,
            thigh_length=thigh_length,
            shank_length=shank_length
        )

        # Initialize kinematics for visualization
        link_lengths = LinkLengths(thigh=thigh_length, shank=shank_length)
        self.kinematics = BipedKinematics(link_lengths, leg_extension_ratio)

        # Create publishers
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile
        )

        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            qos_profile
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/foot_markers',
            qos_profile
        )

        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # Record start time
        self.start_time = self.get_clock().now()
        self._log_count = 0

        # Log startup info
        self.get_logger().info(
            f'Gait Pattern Generator started:\n'
            f'  Publish Rate: {self.publish_rate} Hz\n'
            f'  Step Height: {step_height} m\n'
            f'  Step Length: {step_length} m\n'
            f'  Step Frequency: {step_frequency} Hz\n'
            f'  Leg Extension Ratio: {leg_extension_ratio}\n'
            f'  Body Height: {self.kinematics.body_height:.3f} m\n'
            f'  Enabled: {self.enabled}'
        )

    def _declare_parameters(self):
        """Declare ROS parameters with defaults."""
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('step_height', 0.04)
        self.declare_parameter('step_length', 0.08)
        self.declare_parameter('step_frequency', 0.5)
        self.declare_parameter('leg_extension_ratio', 0.90)
        self.declare_parameter('thigh_length', 0.18)
        self.declare_parameter('shank_length', 0.20)
        self.declare_parameter('enabled', True)

    def _timer_callback(self):
        """Timer callback for publishing joint commands."""
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9

        if not self.enabled:
            # Publish zero pose when disabled
            left_angles = [0.0] * 5
            right_angles = [0.0] * 5
        else:
            # Generate walking pattern
            left_angles, right_angles = self.walking_generator.generate(elapsed_sec)

        # Publish joint states
        self._publish_joint_state(left_angles, right_angles, current_time)

        # Publish joint trajectory
        self._publish_joint_trajectory(left_angles, right_angles, current_time)

        # Publish visualization markers
        self._publish_foot_markers(left_angles, right_angles, current_time)

        # Periodic logging
        self._log_count += 1
        if self._log_count % int(self.publish_rate * 2) == 0:  # Every 2 seconds
            self.get_logger().info(
                f't={elapsed_sec:.2f}s | '
                f'L_hip_pitch={left_angles[2]:+6.1f}deg | '
                f'R_hip_pitch={right_angles[2]:+6.1f}deg'
            )

    def _publish_joint_state(
        self,
        left_angles: List[float],
        right_angles: List[float],
        stamp
    ):
        """Publish joint states message."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_link'

        # Combine joint names and angles
        msg.name = self.LEFT_JOINT_NAMES + self.RIGHT_JOINT_NAMES

        # Convert degrees to radians for ROS
        left_rad = [np.radians(a) for a in left_angles]
        right_rad = [np.radians(a) for a in right_angles]
        msg.position = left_rad + right_rad

        # Zero velocities and efforts
        msg.velocity = [0.0] * 10
        msg.effort = [0.0] * 10

        self.joint_state_pub.publish(msg)

    def _publish_joint_trajectory(
        self,
        left_angles: List[float],
        right_angles: List[float],
        stamp
    ):
        """Publish joint trajectory message."""
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_link'

        msg.joint_names = self.LEFT_JOINT_NAMES + self.RIGHT_JOINT_NAMES

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Convert degrees to radians
        left_rad = [np.radians(a) for a in left_angles]
        right_rad = [np.radians(a) for a in right_angles]
        point.positions = left_rad + right_rad

        # Zero velocities and accelerations
        point.velocities = [0.0] * 10
        point.accelerations = [0.0] * 10

        # Time from start
        point.time_from_start = Duration(sec=0, nanosec=int(1e9 / self.publish_rate))

        msg.points = [point]

        self.joint_trajectory_pub.publish(msg)

    def _publish_foot_markers(
        self,
        left_angles: List[float],
        right_angles: List[float],
        stamp
    ):
        """Publish visualization markers for foot positions."""
        marker_array = MarkerArray()

        # Get foot positions using forward kinematics
        left_positions = self.kinematics.forward_kinematics_leg(left_angles, is_left=True)
        right_positions = self.kinematics.forward_kinematics_leg(right_angles, is_left=False)

        # Left foot marker
        left_foot_marker = Marker()
        left_foot_marker.header.stamp = stamp.to_msg()
        left_foot_marker.header.frame_id = 'base_link'
        left_foot_marker.ns = 'left_foot'
        left_foot_marker.id = 0
        left_foot_marker.type = Marker.SPHERE
        left_foot_marker.action = Marker.ADD
        left_foot_marker.pose.position.x = float(left_positions[-1][0])
        left_foot_marker.pose.position.y = float(left_positions[-1][1])
        left_foot_marker.pose.position.z = float(left_positions[-1][2])
        left_foot_marker.pose.orientation.w = 1.0
        left_foot_marker.scale.x = 0.03
        left_foot_marker.scale.y = 0.03
        left_foot_marker.scale.z = 0.03
        left_foot_marker.color.r = 1.0
        left_foot_marker.color.g = 0.0
        left_foot_marker.color.b = 0.0
        left_foot_marker.color.a = 1.0
        marker_array.markers.append(left_foot_marker)

        # Right foot marker
        right_foot_marker = Marker()
        right_foot_marker.header.stamp = stamp.to_msg()
        right_foot_marker.header.frame_id = 'base_link'
        right_foot_marker.ns = 'right_foot'
        right_foot_marker.id = 1
        right_foot_marker.type = Marker.SPHERE
        right_foot_marker.action = Marker.ADD
        right_foot_marker.pose.position.x = float(right_positions[-1][0])
        right_foot_marker.pose.position.y = float(right_positions[-1][1])
        right_foot_marker.pose.position.z = float(right_positions[-1][2])
        right_foot_marker.pose.orientation.w = 1.0
        right_foot_marker.scale.x = 0.03
        right_foot_marker.scale.y = 0.03
        right_foot_marker.scale.z = 0.03
        right_foot_marker.color.r = 0.0
        right_foot_marker.color.g = 0.0
        right_foot_marker.color.b = 1.0
        right_foot_marker.color.a = 1.0
        marker_array.markers.append(right_foot_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = GaitPatternGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
