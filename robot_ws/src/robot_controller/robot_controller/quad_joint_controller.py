import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Joint_Controller(Node):
  def __init__(self):
    super().__init__('quadruped_joint_controller')

    # Create publisher
    self.pub = self.create_publisher(JointState, "/cmd_jnts", 10)
    
    # TODO: implement IK and add subscriber for each leg's config
    # meanwhile using a temp setup to publish 20 degrees to each joint
    self.timer = self.create_timer(1, self.test_callback)

  def test_callback(self):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    
    for i in range(12):
      msg.name.append(str(i))
      msg.position.append(0)
    
    self.pub.publish(msg)

def main(args = None):
  rclpy.init(args=args)

  joint_control = Joint_Controller()

  rclpy.spin(joint_control)

  joint_control.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

  