import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robot_controller.leg_kin import LEG_KIN
import numpy as np
import time

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

    time.sleep(5)
    
    self.leg_kin = LEG_KIN()

    init_msg = JointState()
    init_msg.name = ['0','1','2','3','4','5','6','7','8','9','10','11']
    init_msg.position = [0.,np.pi/9,-np.pi/9,0.,np.pi/9,-np.pi/9,0.,np.pi/9,-np.pi/9,0.,np.pi/9,-np.pi/9]
    self.cmd_callback(init_msg)

  def cmd_config(self, config: list):
    bad = False
    msg = JointState()
    for i in range(len(config)):
      right = True
      if i > 1:
        right = False
      
      try:
        t0, t1, t2 = self.leg_kin.leg_ik(config[i][0],config[i][1],config[i][2], right)
        t0,t1,t2 = self.leg_kin.leg_control_conv(t0, t1, t2)
        joints = [t0,t1,t2]
      except:
        bad = True

      for j in range(3):
        if np.isnan(joints[j]):
          bad = True
        msg.name.append(str(i*3 + j))
        msg.position.append(joints[j])

    if not bad:
      self.pub.publish(msg)

  def cmd_callback(self, msg: JointState):
    angles = [0,0,0,0,0,0,0,0,0,0,0,0]
    for i in range(len(msg.name)):
      jointNum = int(msg.name[i])
      angles[jointNum] = msg.position[jointNum]

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

def main(args = None):
  rclpy.init(args=args)

  joint_control = Joint_Controller()

  rclpy.spin(joint_control)

  joint_control.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

  