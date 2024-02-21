#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

"""
  @author: Mehmet Kahraman
  @date: 21.02.2024
  @about: Service client node
"""

class MyROSClass(Node):

    def __init__(self):

        super().__init__('service_client_node')

        self.service_topic = "/my_bool_service"
        self.service_wait_timeout = 0.2

        self.srv_client = self.create_client(SetBool, self.service_topic)

        while not self.srv_client.wait_for_service(timeout_sec=self.service_wait_timeout):
            self.get_logger().info('waiting for %s service...' %self.service_topic)
    
        time.sleep(1)
        self.get_logger().info("Service client is ready.")

        self.call_service_once()


    def call_service_once(self):
        data = True
        seconds = 3
        self.get_logger().info("Service client will call server with TRUE data after %d seconds..." %seconds)
        time.sleep(seconds)
        
        self.call_service(data)
        self.get_logger().info("Client process finished.")

    def call_service(self, request_data):
        self.get_logger().info("Sending data to server by client...")
        
        request = SetBool.Request()
        request.data = request_data

        future_result = self.srv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future_result)
        
        response = future_result.result()
        success = response.success
        message = response.message

        if success == True:
            self.get_logger().info("Service call successfull.")
            self.get_logger().info("Service server message: %s" %message)
        else:
            self.get_logger().info("Service success false, failed!")



def main():
    rclpy.init()
    node = MyROSClass()
    rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
