import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState
<<<<<<< Updated upstream
=======
from robot_controller.leg_kin import LEG_KIN
import numpy as np
>>>>>>> Stashed changes

class Joint_Controller(Node):
  def __init__(self):
    super().__init__('quadruped_joint_controller')

    # Create publisher
    self.pub = self.create_publisher(JointState, "/cmd_jnts", 10)\
    
    self.cmd_sub = self.create_subscription(
       JointState,
       "/control_inputs",
       self.cmd_callback,
       10
    )
    
    self.leg_kin = LEG_KIN()

  def cmd_callback(self, msg):
    angles = [0,0,0,0,0,0,0,0,0,0,0,0]
    for i in range(len(msg.name)):
      jointNum = int(msg.name[i])
      angles[jointNum] = msg.position[jointNum] * np.pi/180

    msg = JointState()
    for leg in range(4):
      t0, t1, t2 = angles[leg * 3: leg*3 + 3]

      if abs(t0) > np.pi/2 or t1 > np.pi/2 or t1 < -np.pi/6 or t2 > 7 * np.pi / 18 or t2 < -7 * np.pi / 18:
        return

      try:
        t0,t1,t2 = self.leg_kin.leg_control_conv(t0, t1, t2)
      except:
        return
      
      joints = [t0,t1,t2]

      for i in range(3):
        msg.name.append(str(leg*3 + i))
        msg.position.append(joints[i])

    self.pub.publish(msg)

<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
def main(args = None):
  rclpy.init(args=args)

  joint_control = Joint_Controller()

  rclpy.spin(joint_control)

  joint_control.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

  