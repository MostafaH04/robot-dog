"""Publish a deterministic, gentle stance command for teaching experiments."""

from math import pi, sin

import numpy as np
import rclpy
from rclpy.node import Node
from robot_controller.leg_kin import LegKin
from sensor_msgs.msg import JointState


LEG_RIGHT = (True, True, False, False)
SHOULDER_OFFSET_METERS = 0.038
NOMINAL_FOOT_HEIGHT_METERS = 0.14
HEIGHT_AMPLITUDE_METERS = 0.004


class StancePattern:
    """Generate a fixed-step, symmetric stance-height exercise."""

    def __init__(self, steps_per_cycle=80):
        if steps_per_cycle < 1:
            raise ValueError('steps_per_cycle must be positive')
        self.steps_per_cycle = steps_per_cycle
        self.kinematics = LegKin()

    def command(self, step):
        """Return 12 physical-leg angles for a deterministic cycle step."""
        phase = 2 * pi * (step % self.steps_per_cycle) / self.steps_per_cycle
        foot_z = -NOMINAL_FOOT_HEIGHT_METERS + HEIGHT_AMPLITUDE_METERS * sin(phase)
        angles = []
        for right in LEG_RIGHT:
            foot_y = -SHOULDER_OFFSET_METERS if right else SHOULDER_OFFSET_METERS
            leg_angles = self.kinematics.leg_ik(0.0, foot_y, foot_z, right=right)
            if not np.all(np.isfinite(leg_angles)):
                raise RuntimeError('stance pattern produced an invalid IK result')
            angles.extend(float(value) for value in leg_angles)
        return angles


class ExampleStanceController(Node):
    """Publish the stance pattern through the existing controller adapter."""

    def __init__(self):
        super().__init__('example_stance_controller')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cycle_period_seconds', 4.0)

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        cycle_period = float(self.get_parameter('cycle_period_seconds').value)
        if publish_rate <= 0.0 or cycle_period <= 0.0:
            raise ValueError('publish rate and cycle period must be positive')

        steps_per_cycle = max(1, round(publish_rate * cycle_period))
        self.pattern = StancePattern(steps_per_cycle)
        self.step = 0
        self.publisher = self.create_publisher(JointState, '/control_inputs', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

    def publish_command(self):
        """Publish one sample of the repeatable stance-height command."""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [str(index) for index in range(12)]
        message.position = self.pattern.command(self.step)
        self.publisher.publish(message)
        self.step += 1


def main(args=None):
    """Start the deterministic example stance controller."""
    rclpy.init(args=args)
    controller = ExampleStanceController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
