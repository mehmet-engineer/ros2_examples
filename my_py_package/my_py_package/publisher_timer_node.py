#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
  @author: Mehmet Kahraman
  @date: 21.02.2024
  @about: Publisher timer node
"""

class MyROSClass(Node):

    def __init__(self):

        super().__init__('publisher_timer_node')

        # publisher
        self.hz = 50
        self.timer_period = 1/self.hz
        self.publisher = self.create_publisher(String, "/python_nodes/publisher_topic", 10)
        self.normal_msg = String()
        self.normal_msg.data = "Hello ROS 2 developer!"

        # timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        time.sleep(1)
        self.get_logger().info("Publisher node is ready.")


    def timer_callback(self):
        self.publisher.publish(self.normal_msg)
        self.get_logger().info("Publisher running...")



def main(args=None):
    rclpy.init(args=args)
    node = MyROSClass()
    rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
