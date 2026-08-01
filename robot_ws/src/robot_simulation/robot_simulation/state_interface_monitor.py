"""Example subscriber that checks the PyBullet teaching interfaces together."""

from functools import partial

from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64


FEET = ('front_left', 'front_right', 'rear_left', 'rear_right')
MAX_HEADER_LAG_NANOSECONDS = 100_000_000


class StateInterfaceMonitor(Node):
    """Publish readiness after observing commands and every teaching topic."""

    def __init__(self):
        super().__init__('state_interface_monitor')
        self.received = set()
        self.contact_states = {foot: False for foot in FEET}
        self.normal_forces = {foot: 0.0 for foot in FEET}
        self.command_joint_names = set()
        self.sensor_joint_names = set()
        self.clock_nanoseconds = 0
        self.clock_advanced = False
        self.latest_header_stamps = {}
        self.base_height = 0.0
        self.logged_ready = False
        self.input_subscriptions = []
        self.header_keys = {
            'control_input',
            'joint_command',
            'imu',
            'joint_states',
            'ground_truth',
        }
        self.header_keys.update(f'wrench:{foot}' for foot in FEET)

        self._subscribe(Clock, '/clock', 'clock')
        self._subscribe(JointState, '/control_inputs', 'control_input')
        self._subscribe(JointState, '/cmd_jnts', 'joint_command')
        self._subscribe(Imu, '/sim/sensors/imu', 'imu')
        self._subscribe(JointState, '/sim/sensors/joint_states', 'joint_states')
        self._subscribe(Odometry, '/sim/ground_truth/odom', 'ground_truth')
        for foot in FEET:
            prefix = f'/sim/sensors/foot_contacts/{foot}'
            self._subscribe(Bool, prefix, f'contact:{foot}')
            self._subscribe(Float64, f'{prefix}/normal_force', f'normal_force:{foot}')
            self._subscribe(WrenchStamped, f'{prefix}/wrench', f'wrench:{foot}')

        self.expected = {
            'clock',
            'control_input',
            'joint_command',
            'imu',
            'joint_states',
            'ground_truth',
        }
        for foot in FEET:
            self.expected.update({
                f'contact:{foot}',
                f'normal_force:{foot}',
                f'wrench:{foot}',
            })

        self.ready_publisher = self.create_publisher(
            Bool,
            '/sim/experiment/ready',
            10,
        )
        self.timer = self.create_timer(0.25, self.publish_status)

    def _subscribe(self, message_type, topic, key):
        subscription = self.create_subscription(
            message_type,
            topic,
            partial(self._mark_received, key),
            10,
        )
        self.input_subscriptions.append(subscription)

    def _mark_received(self, key, message):
        self.received.add(key)
        if key == 'clock':
            clock_nanoseconds = self._stamp_nanoseconds(message.clock)
            self.clock_advanced = clock_nanoseconds > self.clock_nanoseconds
            self.clock_nanoseconds = clock_nanoseconds
        elif key in self.header_keys:
            self.latest_header_stamps[key] = self._stamp_nanoseconds(
                message.header.stamp
            )

        if key == 'ground_truth':
            self.base_height = message.pose.pose.position.z
        elif key == 'joint_command':
            self.command_joint_names = set(message.name)
        elif key == 'joint_states':
            self.sensor_joint_names = set(message.name)
        elif key.startswith('contact:'):
            self.contact_states[key.split(':', maxsplit=1)[1]] = message.data
        elif key.startswith('normal_force:'):
            self.normal_forces[key.split(':', maxsplit=1)[1]] = message.data

    @staticmethod
    def _stamp_nanoseconds(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def publish_status(self):
        """Publish whether a coherent sample has arrived from every interface."""
        joint_names_match = (
            len(self.command_joint_names) == 12
            and self.command_joint_names == self.sensor_joint_names
        )
        has_ground_contact = any(force > 0.0 for force in self.normal_forces.values())
        timestamps_follow_clock = (
            self.header_keys.issubset(self.latest_header_stamps)
            and all(
                0 < stamp <= self.clock_nanoseconds
                and self.clock_nanoseconds - stamp <= MAX_HEADER_LAG_NANOSECONDS
                for stamp in self.latest_header_stamps.values()
            )
        )
        ready = (
            self.expected.issubset(self.received)
            and joint_names_match
            and has_ground_contact
            and self.clock_advanced
            and timestamps_follow_clock
        )
        self.ready_publisher.publish(Bool(data=ready))
        if ready and not self.logged_ready:
            contact_count = sum(self.contact_states.values())
            total_normal_force = sum(self.normal_forces.values())
            self.get_logger().info(
                f'Teaching interfaces ready: base height {self.base_height:.3f} m, '
                f'{contact_count} feet in contact, '
                f'{total_normal_force:.2f} N total normal force'
            )
            self.logged_ready = True


def main(args=None):
    """Start the example state-interface monitor."""
    rclpy.init(args=args)
    monitor = StateInterfaceMonitor()
    try:
        rclpy.spin(monitor)
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
