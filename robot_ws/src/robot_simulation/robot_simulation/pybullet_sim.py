"""Run the existing headless PyBullet quadruped simulation."""

from math import pi
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(euler_angles):
    """Convert XYZ Euler angles into an ``xyzw`` quaternion."""
    return Rotation.from_euler('xyz', euler_angles).as_quat()


def rotate_vector(quaternion, vector):
    """Rotate a vector by an ``xyzw`` quaternion."""
    return Rotation.from_quat(quaternion).apply(np.asarray(vector))


class QuadSim(Node):
    """Expose the PyBullet robot state and joint commands through ROS 2."""

    def __init__(self):
        super().__init__('quadruped_sim')

        self.time_step = 0.01
        self.client = self._initialize_simulation()
        self.angles = [0.0] * p.getNumJoints(self.quad, physicsClientId=self.client)

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.command_subscription = self.create_subscription(
            JointState,
            '/cmd_jnts',
            self.joint_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.time_step, self.run_simulation)

    def _initialize_simulation(self):
        client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.setTimeStep(self.time_step, physicsClientId=client)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        p.loadURDF('plane.urdf', physicsClientId=client)

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

    def run_simulation(self):
        """Advance PyBullet once and publish the resulting ROS state."""
        position, orientation = p.getBasePositionAndOrientation(
            self.quad,
            physicsClientId=self.client,
        )

        inertial_offset = [-0.008382264142625067, -1.4798434925308436e-05,
                           0.030113658418836745]
        base_position = np.asarray(position) - rotate_vector(orientation, inertial_offset)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = base_position[0]
        transform.transform.translation.y = base_position[1]
        transform.transform.translation.z = base_position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(transform)

        message = JointState()
        message.header.stamp = transform.header.stamp
        for joint_index, target_angle in enumerate(self.angles):
            p.setJointMotorControl2(
                self.quad,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=target_angle,
                physicsClientId=self.client,
            )
            joint_name = p.getJointInfo(
                self.quad,
                joint_index,
                physicsClientId=self.client,
            )[1]
            angle, velocity, _reactions, effort = p.getJointState(
                self.quad,
                joint_index,
                physicsClientId=self.client,
            )
            message.name.append(joint_name.decode('utf-8'))
            message.position.append(angle)
            message.velocity.append(velocity)
            message.effort.append(effort)

        self.joint_state_publisher.publish(message)
        p.stepSimulation(physicsClientId=self.client)

    def joint_callback(self, message):
        """Apply indexed joint commands from the controller adapter."""
        reversed_joints = {1, 3, 4, 8, 9}
        for name, position in zip(message.name, message.position):
            joint_index = int(name)
            if not 0 <= joint_index < len(self.angles):
                self.get_logger().warning('Ignoring out-of-range joint index %s', name)
                continue
            self.angles[joint_index] = -position if joint_index in reversed_joints else position

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
