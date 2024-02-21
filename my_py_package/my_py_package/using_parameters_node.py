#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters, GetParameters

"""
  @author: Mehmet Kahraman
  @date: 21.02.2024
  @about: Using parameters node
"""

class MyROSClass(Node):

    def __init__(self):
        super().__init__('using_parameters_node')

        # Declaring new parameters in this node
        self.declare_parameter('my_frame', 'world')
        self.declare_parameter('my_count', 8)
        
        # Reset parameters in this node
        new_parameters = []
        new_value = 12
        new_param = rclpy.parameter.Parameter('using_parameters_node/my_count', rclpy.parameter.Parameter.Type.INTEGER, new_value)
        new_parameters.append(new_param)
        self.set_parameters(new_parameters)

        # get or set parameters from another node
        # service_topic = "using_parameters_node/my_frame"
        # self.set_param_client = self.create_client(SetParameters, service_topic)
        # self.get_param_client = self.create_client(GetParameters, service_topic)

        # Publisher
        self.timer_publisher = self.create_publisher(String, '/using_parameters_topic', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('using_parameters_node is ready.')
        time.sleep(1)

    def timer_callback(self):
        my_frame = self.get_parameter('using_parameters_node/my_frame').get_parameter_value().string_value
        my_count = self.get_parameter('using_parameters_node/my_count').get_parameter_value().integer_value

        timer_msg = String()
        timer_msg.data = 'Received parameter: ' + my_frame
        self.timer_publisher.publish(timer_msg)
        self.get_logger().info('Received my_frame: %s' % my_frame)
        self.get_logger().info('Received my_count: %d' % my_count)


def main(args=None):
    rclpy.init(args=args)
    node = MyROSClass()
    rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
