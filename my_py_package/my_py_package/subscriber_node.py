import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

"""
  @author: Mehmet Kahraman
  @date: 03.10.2023
  @about: Subscriber node
"""

class MyROSClass(Node):

    def __init__(self):

        super().__init__('subscriber_node')

        # publisher
        self.position = 0

        # subscriber
        self.js_subscription = self.create_subscription(JointState, "/joint_states", self.subscriber_callback, 10)
        self.js_subscription  # prevent unused variable warning

        self.get_clock().sleep_for(Duration(seconds=1)) 
        self.get_logger().info("Subscriber node is ready.")


    def subscriber_callback(self, msg):
        self.position = msg.position[0]
        self.get_logger().info(' JS subscriber position received: "%s" ' %self.position)


def main(args=None):
    rclpy.init(args=args)
    node = MyROSClass()
    rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
