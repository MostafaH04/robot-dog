import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

import pybullet as p
import pybullet_data
import numpy as np

from math import pi
from scipy.spatial.transform import Rotation

def quat_2_mat(q):
  rot = Rotation.from_quat(q)
  rot_mat = rot.as_matrix()
  
  return rot_mat

def euler_2_quat(eular):
  rot = Rotation.from_euler("xyz", eular)
  quat = rot.as_quat()

  return quat

def euler_2_mat(eular):
  rot = Rotation.from_euler("xyz", eular)
  rot_mat = rot.as_matrix()

  return rot_mat

def quat_rot(q, vec):
  rot = Rotation.from_quat(q)
  return rot.apply(np.array(vec))

class IMU(Node):
  
  def __init__(self, quad_sim_instance):
    super().__init__('quadruped_imu')
    self.quad = quad_sim_instance.quad
    self.time_step = quad_sim_instance.time_step
    
    self.b_acc = np.array([0, 0, 0])  # Accelerometer bias
    self.b_gyro = np.array([0, 0, 0])  # Gyroscope bias
    
    self.acc_noise_std = 0.02  
    self.gyro_noise_std = 0.01  
    
    self.gravity = np.array([0, 0, -9.81])
    
    
    
  def get_prev_base_velocity(self):
    # Update velocities from pybullet
    self.linear_velocity, self.angular_velocity = p.getBaseVelocity(self.quad)
    
    if not hasattr(self, 'prev_linear_velocity') or not hasattr(self, 'prev_angular_velocity'):
        self.prev_linear_velocity = np.array([0, 0, 0])
        self.prev_angular_velocity = np.array([0, 0, 0])
    
    prev_linear_velocity = self.prev_linear_velocity
    prev_angular_velocity = self.prev_angular_velocity
    
    self.prev_linear_velocity = self.linear_velocity
    self.prev_angular_velocity = self.angular_velocity
    
    return prev_linear_velocity, prev_angular_velocity

  def get_base_linear_acceleration(self):
    prev_linear_velocity, _ = self.get_prev_base_velocity()
    
    if np.array_equal(prev_linear_velocity, np.array([0, 0, 0])):
        return np.array([0, 0, 0])
    
    base_linear_acceleration = (np.array(self.linear_velocity) - np.array(prev_linear_velocity)) / self.time_step
    
    return base_linear_acceleration

  def get_base_angular_acceleration(self):
    _, prev_angular_velocity = self.get_prev_base_velocity()
    
    if np.array_equal(prev_angular_velocity, np.array([0, 0, 0])):
        return np.array([0, 0, 0])
    
    base_angular_acceleration = (np.array(self.angular_velocity) - np.array(prev_angular_velocity)) / self.time_step
    
    return base_angular_acceleration

  def get_noisy_imu_acceleration(self):
    base_acceleration = self.get_base_linear_acceleration()
    
    R_W_IMU = np.eye(3)
    imu_acceleration = R_W_IMU @ (base_acceleration + self.gravity)
    
    w_acc = np.random.normal(0, self.acc_noise_std, 3)  # Accelerometer noise
    noisy_imu_acceleration = imu_acceleration + self.b_acc + w_acc
    
    return noisy_imu_acceleration

  def get_noisy_imu_angular_velocity(self):

    base_angular_velocity = np.array(self.angular_velocity)
    
   
    R_W_IMU = np.eye(3)  
    imu_angular_velocity = R_W_IMU @ base_angular_velocity
    
    
    w_gyro = np.random.normal(0, self.gyro_noise_std, 3)
    noisy_imu_angular_velocity = imu_angular_velocity + self.b_gyro + w_gyro
    
    return noisy_imu_angular_velocity

  #found in the depths of robotics forums, need to incorporate this
  def update_bias(self):
    
    b_acc_noise = np.random.normal(0, 0.001, 3)
    b_gyro_noise = np.random.normal(0, 0.001, 3)
    
    self.b_acc += b_acc_noise
    self.b_gyro += b_gyro_noise
  
    
class Quad_Sim(Node):
  def __init__(self):
    super().__init__('quadruped_sim')

  
    # Create a publisher
    self.pub = self.create_publisher(JointState, "/joint_states", 10)
    self.imu_pub = self.create_publisher(Imu,"/imu_state",10) #DESIGN: do we want to implement ros-based imu utilization within quad_sim or should the imu class be a solution?
    self.time_step = 0.01
    self.sim_init()
    self.imu = IMU(self)
    self.timer = self.create_timer(self.time_step, self.run_sim)
    self.tf_broadcaster = TransformBroadcaster(self)
    
    self.angles = [0,0,0,0,0,0,0,0,0,0,0,0]

    self.cmd_sub = self.create_subscription(
       JointState,
       "/cmd_jnts",
       self.joint_callback,
       10
    )
    self.cmd_sub # prevents unused variable warning
      

  def sim_init(self):
    # Start sim (headless)
    self.client = p.connect(p.GUI)

    # Set Gravity
    p.setGravity(0,0,-9.81, physicsClientId=self.client)

    # Set timestep
    p.setTimeStep(self.time_step, self.client)

    # Add a plane
    # TODO: Change this with environment in the future (make modular)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")

    # Load robot URDF
    # TODO: use full leg (implement closed chain kinematics)
    quad_urdf = "./install/robot_simulation/share/robot_simulation/urdf/robot_core.xacro"
    startPos = [0,0,1]
    startRot_euler = [0,0,pi]
    startRot_quat = euler_2_quat(startRot_euler)
    self.startRot_mat = euler_2_mat(startRot_euler)
    quad_flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE # enable self collisions and add calculated inertias from urdf
    self.quad = p.loadURDF(quad_urdf, basePosition = startPos, baseOrientation = startRot_quat, flags = quad_flags)

    # TODO: close kinematic chains in legs

    # TODO: Debug hidden meshes

    # TODO: create a topic for joint control commands
    # Sliders for temporary control
    self.num_joints = p.getNumJoints(self.quad)

  def run_sim(self):
    # Get updated position and orientation of the quad
    position, orientation = p.getBasePositionAndOrientation(self.quad)
    
    inertial_offset = [-0.008382264142625067,-1.4798434925308436e-05,0.030113658418836745]
    pos_offset = quat_rot(np.array(orientation), inertial_offset)
    pos = np.array(position) - pos_offset

    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"
    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]
    t.transform.rotation.x = orientation[0]
    t.transform.rotation.y = orientation[1]
    t.transform.rotation.z = orientation[2]
    t.transform.rotation.w = orientation[3]

    self.tf_broadcaster.sendTransform(t)

    # Initialize joint state message to publish
    msg = JointState()

    for i in range(self.num_joints):
        # Update teh commanded joint angles from the sliders for temporary control
        angle = self.angles[i]
        p.setJointMotorControl2(self.quad, i, p.POSITION_CONTROL, targetPosition = angle)

        # Update current joint's state to be sent as part 
        # of the joint state msg being publishd
        name = p.getJointInfo(self.quad, i, self.client)[1]
        msg.name.append(name.decode("utf-8")) # decode name from bytes to utf-8
        angle_pos,velocity,reactions,effort = p.getJointState(self.quad, i, self.client)
        msg.position.append(angle_pos)
        msg.velocity.append(velocity)
        msg.effort.append(effort)

    msg.header.stamp = self.get_clock().now().to_msg()
    self.pub.publish(msg)

    # Get Quad Rotation Matrix
    rotMat = quat_2_mat(orientation) @ self.startRot_mat
    position = list(position)

    # Set up axes for the camera to be the quad reference
    # frame's z-axis
    up_axes = np.matrix([[0],[0],[1]])
    up_axes = rotMat @ up_axes

    # Set the camera's position and its target's position
    # TODO: add global constants for cam displacment
    # TODO: base camera target to be based on camera rotation
    cam_displacement = np.array([0.2,0,0.05])
    target_displacement = np.array([0.1,0,0])
    cam_position = position.copy()
    cam_position += rotMat @ cam_displacement
    target_position = cam_position.copy()
    target_position += rotMat @ target_displacement 


    # Compute Camera view and projection matricies
    cam_view_mat = p.computeViewMatrix (cam_position,target_position,up_axes)
    # TODO: add global constants for fov and other info for cam proj mat
    cam_proj_mat = p.computeProjectionMatrixFOV(53.50,1280/720,0.001,1)
    # TODO: look into speeding up generating camera images
    #image = p.getCameraImage(10,10,cam_view_mat,cam_proj_mat,renderer = p.ER_BULLET_HARDWARE_OPENGL,physicsClientId = self.client)
    # TODO: publish camera images over an images topic
    # image = np.reshape(image, (720,1280))

    # Step forward in the bullet physics simulation
    p.stepSimulation()

  def joint_callback(self, msg):
    for i in range(len(msg.name)):
      jointNum = int(msg.name[i])
      if jointNum == 1 or jointNum == 3 or jointNum == 4 or jointNum == 8 or jointNum == 9:
        self.angles[jointNum] = -msg.position[jointNum]
      else:
        self.angles[jointNum] = msg.position[jointNum]
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    quad_sim = Quad_Sim()

    # Spin the node so the callback function is called.
    rclpy.spin(quad_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quad_sim.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()