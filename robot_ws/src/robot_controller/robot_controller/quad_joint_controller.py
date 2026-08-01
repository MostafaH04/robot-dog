"""Translate Cartesian leg targets into simulated joint commands."""

import numpy as np
import rclpy
from rclpy.node import Node
from robot_controller.leg_kin import LegKin
from sensor_msgs.msg import JointState


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
        self.initialization_timer = self.create_timer(1.0, self._publish_initial_configuration)

    def _publish_initial_configuration(self):
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
        for leg_index, leg_target in enumerate(configuration):
            right_leg = leg_index <= 1
            angles = self.leg_kinematics.leg_ik(*leg_target, right=right_leg)
            joints = self.leg_kinematics.leg_control_conversion(*angles)
            if not np.all(np.isfinite(joints)):
                self.get_logger().warning('Ignoring unreachable leg target %s', leg_target)
                return

            for joint_offset, joint_position in enumerate(joints):
                message.name.append(str(leg_index * 3 + joint_offset))
                message.position.append(joint_position)

        self.command_publisher.publish(message)

    def cmd_config(self, configuration):
        """Preserve the original command method name for downstream callers."""
        self.command_configuration(configuration)

    def command_callback(self, input_message):
        """Validate and convert indexed physical joint commands."""
        angles = [0.0] * 12
        for name, position in zip(input_message.name, input_message.position):
            joint_index = int(name)
            if not 0 <= joint_index < len(angles):
                self.get_logger().warning('Ignoring out-of-range joint index %s', name)
                return
            angles[joint_index] = position

        output_message = JointState()
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

            for joint_offset, joint_position in enumerate(joints):
                output_message.name.append(str(leg_index * 3 + joint_offset))
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
