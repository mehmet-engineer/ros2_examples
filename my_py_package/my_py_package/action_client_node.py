#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_msgs.action import MoveRobot

"""
  @author: Mehmet Kahraman
  @date: 21.02.2024
  @about: Action client node
"""

class MyROSClass(Node):

    def __init__(self):

        super().__init__('action_client_node')

        self.action_topic = "/my_action_topic"
        self.action_client = ActionClient(self, MoveRobot, self.action_topic)
        
        self.get_logger().info('waiting for %s action server...' %self.action_topic)
        self.action_client.wait_for_server()
        self.get_logger().info("Action client is now connected to action server.")
    
        time.sleep(1)
        self.get_logger().info("Action client is ready.")

        self.execute_once()


    def execute_once(self):
        seconds = 3
        self.get_logger().info(f'Action goal will be sent in {seconds} seconds...')
        time.sleep(seconds)

        target_pos_vector = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_goal(target_pos_vector)
        self.get_logger().info("Client process finished.")

    def send_goal(self, request_data):
        self.get_logger().info("Sending action goal...")
        goal_msg = MoveRobot.Goal()
        goal_msg.target_position = request_data
        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future_result):
        goal_handle = future_result.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by action server!')
            return
        else:
            self.get_logger().info('Goal accepted by action server, waiting for result.')
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        progress = feedback.feedback.progress
        self.get_logger().info(f'Feedback received, Progress: {progress}')

    def result_callback(self, future_result):
        result = future_result.result()
        if result.result.finish_success == True:
            self.get_logger().info('Action goal successfully done.')
        else:
            self.get_logger().error('Action goal resulted with FALSE!')


def main():
    rclpy.init()
    node = MyROSClass()
    #rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
