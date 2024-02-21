#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

"""
  @author: Mehmet Kahraman
  @date: 21.02.2024
  @about: Service server node
"""

class MyROSClass(Node):

    def __init__(self):

        super().__init__('service_server_node')

        self.service_topic = "/my_bool_service"
        self.service = self.create_service(SetBool, self.service_topic, self.service_callback)

        time.sleep(1)
        self.get_logger().info("Service server is ready.")


    def service_callback(self, request, response):

        req_data = request.data
        message = ""

        if req_data == True:
            message = "Service called with TRUE"
            self.get_logger().info(message)
        else:
            message = "Service called with FALSE"
            self.get_logger().info(message)
        
        response.message = message
        response.success = True

        return response


def main():
    rclpy.init()
    node = MyROSClass()
    rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
