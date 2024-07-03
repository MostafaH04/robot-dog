import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState 

import pybullet as p
import pybullet_data
import numpy as np

from math import pi
from scipy.spatial.transform import Rotation

def quad_2_mat(q):
    rot = Rotation.from_quat(q)
    rot_mat = rot.as_matrix()
    
    return rot_mat

def eular_2_quat(eular):
    rot = Rotation.from_euler("xyz", eular)
    quat = rot.as_quat()

    return quat

def eular_2_mat(eular):
    rot = Rotation.from_euler("xyz", eular)
    rot_mat = rot.as_matrix()

    return rot_mat
 
class Quad_Sim(Node):
  def __init__(self):
    super().__init__('quadruped_sim')
      
    # Create a publisher
    self.pub = self.create_publisher(JointState, "/joint_states", 10)
    self.time_step = 0.01
    self.sim_init()
    self.timer = self.create_timer(self.time_step, self.run_sim)
      

  def sim_init(self):
    # Start sim GUI
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
    startRot_eular = [0,0,pi]
    startRot_quat = eular_2_quat(startRot_eular)
    self.startRot_mat = eular_2_mat(startRot_eular)
    quad_flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE # enable self collisions and add calculated inertias from urdf
    self.quad = p.loadURDF(quad_urdf, basePosition = startPos, baseOrientation = startRot_quat, flags = quad_flags)

    # TODO: close kinematic chains in legs

    # TODO: Debug hidden meshes

    # TODO: create a topic for joint control commands
    # Sliders for temporary control
    self.num_joints = p.getNumJoints(self.quad)
    self.angles = []
    for i in range(self.num_joints):
        self.angles.append(p.addUserDebugParameter(f"Angle {i}", -1.57,1.57, self.client))

  def run_sim(self):
    # Get updated position and orientation of the quad
    position,orinetation = p.getBasePositionAndOrientation(self.quad)

    # Initialize joint state message to publish
    msg = JointState()

    for i in range(self.num_joints):
        # Update teh commanded joint angles from the sliders for temporary control
        angle = p.readUserDebugParameter(self.angles[i])
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
    rotMat = quad_2_mat(orinetation) @ self.startRot_mat
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