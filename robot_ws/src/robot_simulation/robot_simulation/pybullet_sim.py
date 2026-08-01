"""Run the headless PyBullet quadruped and publish teaching interfaces."""

from math import pi
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, WrenchStamped
from nav_msgs.msg import Odometry
import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64
from tf2_ros import TransformBroadcaster


GRAVITY_WORLD = np.asarray([0.0, 0.0, -9.81])
INERTIAL_OFFSET = np.asarray([
    -0.008382264142625067,
    -1.4798434925308436e-05,
    0.030113658418836745,
])
REVERSED_JOINT_INDICES = {1, 3, 4, 8, 9}
FOOT_LINK_NAMES = {
    'front_left': 'Component1_Mirror___5__1',
    'front_right': 'Foot_v3_2',
    'rear_left': 'Component1_Mirror___5__2',
    'rear_right': 'Foot_v3_1',
}


def euler_to_quaternion(euler_angles):
    """Convert XYZ Euler angles into an ``xyzw`` quaternion."""
    return Rotation.from_euler('xyz', euler_angles).as_quat()


def rotate_vector(quaternion, vector):
    """Rotate a vector from base coordinates into world coordinates."""
    return Rotation.from_quat(quaternion).apply(np.asarray(vector))


def inverse_rotate_vector(quaternion, vector):
    """Rotate a vector from world coordinates into base coordinates."""
    return Rotation.from_quat(quaternion).inv().apply(np.asarray(vector))


class QuadSim(Node):
    """Expose PyBullet commands, idealized sensors, and separate ground truth."""

    def __init__(self):
        super().__init__('quadruped_sim')

        self.time_step = 0.01
        self.client = self._initialize_simulation()
        self.joint_name_to_index, self.link_name_to_index = self._index_model()
        self.foot_link_indices = {
            foot: self.link_name_to_index[link_name]
            for foot, link_name in FOOT_LINK_NAMES.items()
        }
        self.angles = [0.0] * p.getNumJoints(self.quad, physicsClientId=self.client)

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.sim_joint_state_publisher = self.create_publisher(
            JointState,
            '/sim/sensors/joint_states',
            10,
        )
        self.imu_publisher = self.create_publisher(Imu, '/sim/sensors/imu', 10)
        self.ground_truth_publisher = self.create_publisher(
            Odometry,
            '/sim/ground_truth/odom',
            10,
        )
        self.contact_flag_publishers = {}
        self.contact_normal_force_publishers = {}
        self.contact_wrench_publishers = {}
        for foot in FOOT_LINK_NAMES:
            sensor_prefix = f'/sim/sensors/foot_contacts/{foot}'
            self.contact_flag_publishers[foot] = self.create_publisher(
                Bool,
                sensor_prefix,
                10,
            )
            self.contact_normal_force_publishers[foot] = self.create_publisher(
                Float64,
                f'{sensor_prefix}/normal_force',
                10,
            )
            self.contact_wrench_publishers[foot] = self.create_publisher(
                WrenchStamped,
                f'{sensor_prefix}/wrench',
                10,
            )

        self.command_subscription = self.create_subscription(
            JointState,
            '/cmd_jnts',
            self.joint_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        _position, _orientation, linear_velocity, _angular_velocity = self._base_state()
        self.previous_base_velocity_world = linear_velocity
        self.timer = self.create_timer(self.time_step, self.run_simulation)

    def _initialize_simulation(self):
        client = p.connect(p.DIRECT)
        p.setGravity(*GRAVITY_WORLD, physicsClientId=client)
        p.setTimeStep(self.time_step, physicsClientId=client)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        self.plane = p.loadURDF('plane.urdf', physicsClientId=client)

        package_share = Path(get_package_share_directory('robot_simulation'))
        quad_urdf = package_share / 'urdf' / 'robot_core.xacro'
        start_orientation = euler_to_quaternion([0, 0, pi])
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
        self.quad = p.loadURDF(
            str(quad_urdf),
            basePosition=[0, 0, 1],
            baseOrientation=start_orientation,
            flags=flags,
            physicsClientId=client,
        )

        return client

    def _index_model(self):
        joint_names = {}
        link_names = {}
        for joint_index in range(p.getNumJoints(self.quad, physicsClientId=self.client)):
            joint_info = p.getJointInfo(
                self.quad,
                joint_index,
                physicsClientId=self.client,
            )
            joint_names[joint_info[1].decode('utf-8')] = joint_index
            link_names[joint_info[12].decode('utf-8')] = joint_index

        missing_links = set(FOOT_LINK_NAMES.values()) - set(link_names)
        if missing_links:
            raise RuntimeError(f'foot links missing from PyBullet model: {sorted(missing_links)}')
        return joint_names, link_names

    def _base_state(self):
        position, orientation = p.getBasePositionAndOrientation(
            self.quad,
            physicsClientId=self.client,
        )
        linear_com_velocity, angular_velocity = p.getBaseVelocity(
            self.quad,
            physicsClientId=self.client,
        )
        orientation = np.asarray(orientation)
        offset_world = rotate_vector(orientation, INERTIAL_OFFSET)
        base_position = np.asarray(position) - offset_world
        base_velocity = np.asarray(linear_com_velocity) - np.cross(
            angular_velocity,
            offset_world,
        )
        return (
            base_position,
            orientation,
            base_velocity,
            np.asarray(angular_velocity),
        )

    def run_simulation(self):
        """Advance PyBullet by one fixed step and publish the resulting state."""
        for joint_index, target_angle in enumerate(self.angles):
            p.setJointMotorControl2(
                self.quad,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=target_angle,
                physicsClientId=self.client,
            )
        p.stepSimulation(physicsClientId=self.client)

        stamp = self.get_clock().now().to_msg()
        base_position, orientation, linear_velocity, angular_velocity = self._base_state()
        linear_acceleration = (
            linear_velocity - self.previous_base_velocity_world
        ) / self.time_step
        self.previous_base_velocity_world = linear_velocity

        self._publish_transform(stamp, base_position, orientation)
        self._publish_joint_states(stamp)
        self._publish_imu(
            stamp,
            orientation,
            angular_velocity,
            linear_acceleration,
        )
        self._publish_ground_truth(
            stamp,
            base_position,
            orientation,
            linear_velocity,
            angular_velocity,
        )
        self._publish_contacts(stamp)

    def _publish_transform(self, stamp, position, orientation):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(transform)

    def _publish_joint_states(self, stamp):
        message = JointState()
        message.header.stamp = stamp
        for joint_index in range(len(self.angles)):
            joint_info = p.getJointInfo(
                self.quad,
                joint_index,
                physicsClientId=self.client,
            )
            angle, velocity, _reactions, effort = p.getJointState(
                self.quad,
                joint_index,
                physicsClientId=self.client,
            )
            message.name.append(joint_info[1].decode('utf-8'))
            message.position.append(angle)
            message.velocity.append(velocity)
            message.effort.append(effort)

        self.joint_state_publisher.publish(message)
        self.sim_joint_state_publisher.publish(message)

    def _publish_imu(self, stamp, orientation, angular_velocity, linear_acceleration):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        message.orientation.x = orientation[0]
        message.orientation.y = orientation[1]
        message.orientation.z = orientation[2]
        message.orientation.w = orientation[3]

        angular_velocity_body = inverse_rotate_vector(orientation, angular_velocity)
        message.angular_velocity.x = angular_velocity_body[0]
        message.angular_velocity.y = angular_velocity_body[1]
        message.angular_velocity.z = angular_velocity_body[2]

        specific_force_world = linear_acceleration - GRAVITY_WORLD
        specific_force_body = inverse_rotate_vector(orientation, specific_force_world)
        message.linear_acceleration.x = specific_force_body[0]
        message.linear_acceleration.y = specific_force_body[1]
        message.linear_acceleration.z = specific_force_body[2]
        self.imu_publisher.publish(message)

    def _publish_ground_truth(
        self,
        stamp,
        position,
        orientation,
        linear_velocity,
        angular_velocity,
    ):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'world'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = position[0]
        message.pose.pose.position.y = position[1]
        message.pose.pose.position.z = position[2]
        message.pose.pose.orientation.x = orientation[0]
        message.pose.pose.orientation.y = orientation[1]
        message.pose.pose.orientation.z = orientation[2]
        message.pose.pose.orientation.w = orientation[3]

        linear_velocity_body = inverse_rotate_vector(orientation, linear_velocity)
        angular_velocity_body = inverse_rotate_vector(orientation, angular_velocity)
        message.twist.twist.linear.x = linear_velocity_body[0]
        message.twist.twist.linear.y = linear_velocity_body[1]
        message.twist.twist.linear.z = linear_velocity_body[2]
        message.twist.twist.angular.x = angular_velocity_body[0]
        message.twist.twist.angular.y = angular_velocity_body[1]
        message.twist.twist.angular.z = angular_velocity_body[2]
        self.ground_truth_publisher.publish(message)

    def _publish_contacts(self, stamp):
        for foot, link_index in self.foot_link_indices.items():
            contact_points = p.getContactPoints(
                bodyA=self.quad,
                bodyB=self.plane,
                linkIndexA=link_index,
                physicsClientId=self.client,
            )
            normal_force = 0.0
            force_world = np.zeros(3)
            torque_world = np.zeros(3)
            link_state = p.getLinkState(
                self.quad,
                link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            link_origin_world = np.asarray(link_state[4])
            link_orientation_world = np.asarray(link_state[5])
            for contact in contact_points:
                contact_force = (
                    np.asarray(contact[7]) * contact[9]
                    + np.asarray(contact[11]) * contact[10]
                    + np.asarray(contact[13]) * contact[12]
                )
                normal_force += contact[9]
                force_world += contact_force
                lever_arm = np.asarray(contact[5]) - link_origin_world
                torque_world += np.cross(lever_arm, contact_force)

            force_link = inverse_rotate_vector(link_orientation_world, force_world)
            torque_link = inverse_rotate_vector(link_orientation_world, torque_world)

            self.contact_flag_publishers[foot].publish(
                Bool(data=normal_force > 0.0)
            )
            self.contact_normal_force_publishers[foot].publish(
                Float64(data=float(normal_force))
            )
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = FOOT_LINK_NAMES[foot]
            wrench.wrench.force.x = force_link[0]
            wrench.wrench.force.y = force_link[1]
            wrench.wrench.force.z = force_link[2]
            wrench.wrench.torque.x = torque_link[0]
            wrench.wrench.torque.y = torque_link[1]
            wrench.wrench.torque.z = torque_link[2]
            self.contact_wrench_publishers[foot].publish(wrench)

    def joint_callback(self, message):
        """Apply semantic joint-name commands, with numeric legacy fallback."""
        for name, position in zip(message.name, message.position):
            joint_index = self.joint_name_to_index.get(name)
            if joint_index is None:
                try:
                    joint_index = int(name)
                except ValueError:
                    self.get_logger().warning(f'Ignoring unknown joint name {name}')
                    continue

            if not 0 <= joint_index < len(self.angles):
                self.get_logger().warning(f'Ignoring out-of-range joint index {name}')
                continue
            self.angles[joint_index] = (
                -position if joint_index in REVERSED_JOINT_INDICES else position
            )

    def destroy_node(self):
        """Disconnect the dedicated PyBullet client before destroying the node."""
        if p.isConnected(self.client):
            p.disconnect(physicsClientId=self.client)
        return super().destroy_node()


def main(args=None):
    """Start the ROS 2 PyBullet simulation node."""
    rclpy.init(args=args)
    simulator = QuadSim()
    try:
        rclpy.spin(simulator)
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
