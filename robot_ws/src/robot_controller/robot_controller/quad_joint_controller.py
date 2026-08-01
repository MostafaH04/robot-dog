"""Translate Cartesian leg targets into simulated joint commands."""

import numpy as np
import rclpy
from rclpy.node import Node
from robot_controller.leg_kin import LegKin
from sensor_msgs.msg import JointState


SIM_JOINT_NAMES_BY_LEG = (
    ('Revolute_1', 'Revolute_25', 'Revolute_40'),
    ('Revolute_3', 'Revolute_20', 'Revolute_23'),
    ('Revolute_4', 'Revolute_35', 'Revolute_45'),
    ('Revolute_5', 'Revolute_31', 'Revolute_48'),
)


class JointController(Node):
    """Adapt physical leg commands to the simplified simulation joints."""

    def __init__(self):
        super().__init__('quadruped_joint_controller')

        self.leg_kinematics = LegKin()
        self.command_publisher = self.create_publisher(JointState, '/cmd_jnts', 10)
        self.control_subscription = self.create_subscription(
            JointState,
            '/control_inputs',
            self.command_callback,
            10,
        )
        self.declare_parameter('publish_initial_configuration', True)
        self.initialization_timer = None
        if self.get_parameter('publish_initial_configuration').value:
            self.initialization_timer = self.create_timer(
                1.0,
                self._publish_initial_configuration,
            )

    def _publish_initial_configuration(self):
        if self.initialization_timer is not None:
            self.initialization_timer.cancel()
        initial_x, initial_y, initial_z = (0.0, 0.038, 0.14)
        initial_configuration = [
            [initial_x, -initial_y, -initial_z],
            [initial_x, -initial_y, -initial_z],
            [initial_x, initial_y, -initial_z],
            [initial_x, initial_y, -initial_z],
        ]
        self.command_configuration(initial_configuration)

    def command_configuration(self, configuration):
        """Publish a four-leg Cartesian target when all IK results are valid."""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for leg_index, leg_target in enumerate(configuration):
            right_leg = leg_index <= 1
            angles = self.leg_kinematics.leg_ik(*leg_target, right=right_leg)
            joints = self.leg_kinematics.leg_control_conversion(*angles)
            if not np.all(np.isfinite(joints)):
                self.get_logger().warning(
                    f'Ignoring unreachable leg target {leg_target}'
                )
                return

            for joint_name, joint_position in zip(
                SIM_JOINT_NAMES_BY_LEG[leg_index],
                joints,
            ):
                message.name.append(joint_name)
                message.position.append(joint_position)

        self.command_publisher.publish(message)

    def cmd_config(self, configuration):
        """Preserve the original command method name for downstream callers."""
        self.command_configuration(configuration)

    def command_callback(self, input_message):
        """Validate and convert indexed physical joint commands."""
        if len(input_message.name) != len(input_message.position):
            self.get_logger().warning('Ignoring command with mismatched names and positions')
            return

        angles = [None] * 12
        for name, position in zip(input_message.name, input_message.position):
            try:
                joint_index = int(name)
            except ValueError:
                self.get_logger().warning(
                    f'Ignoring non-numeric physical joint name {name}'
                )
                return
            if not 0 <= joint_index < len(angles):
                self.get_logger().warning(f'Ignoring out-of-range joint index {name}')
                return
            if angles[joint_index] is not None:
                self.get_logger().warning(
                    f'Ignoring command with duplicate joint index {name}'
                )
                return
            angles[joint_index] = position

        if any(angle is None for angle in angles):
            self.get_logger().warning('Ignoring incomplete physical joint command')
            return

        output_message = JointState()
        output_message.header.stamp = self.get_clock().now().to_msg()
        for leg_index in range(4):
            theta_0, theta_1, theta_2 = angles[leg_index * 3:leg_index * 3 + 3]
            outside_limits = (
                abs(theta_0) > np.pi / 2
                or not -np.pi / 6 <= theta_1 <= np.pi / 2
                or not -7 * np.pi / 18 <= theta_2 <= 7 * np.pi / 18
            )
            if outside_limits:
                self.get_logger().warning('Ignoring command outside configured joint limits')
                return

            joints = self.leg_kinematics.leg_control_conversion(theta_0, theta_1, theta_2)
            if not np.all(np.isfinite(joints)):
                self.get_logger().warning('Ignoring command with invalid kinematic result')
                return

            for joint_name, joint_position in zip(
                SIM_JOINT_NAMES_BY_LEG[leg_index],
                joints,
            ):
                output_message.name.append(joint_name)
                output_message.position.append(joint_position)

        self.command_publisher.publish(output_message)


def main(args=None):
    """Start the ROS 2 joint controller node."""
    rclpy.init(args=args)
    controller = JointController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
