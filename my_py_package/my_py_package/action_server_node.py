#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from my_msgs.action import MoveRobot

"""
  @author: Mehmet Kahraman
  @date: 21.02.2024
  @about: Action server node
"""

class MyROSClass(Node):

    def __init__(self):

        super().__init__('action_server_node')

        self.service_topic = "/my_action_topic"
    
        self.action_server = ActionServer(self, 
            MoveRobot, 
            self.service_topic, 
            execute_callback=self.execute_action,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel
        )
        
        time.sleep(1)
        self.get_logger().info("Action server is ready.")
    

    def handle_goal(self, goal_request):
        self.get_logger().info('Received goal request.')
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().warn('Received request to cancel goal')
        return CancelResponse.ACCEPT

    def execute_action(self, goal_handle):
        self.get_logger().info('Action started. Executing goal...')

        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()

        start_time = self.get_clock().now()
        current_time = self.get_clock().now()
        elapsed_time = current_time - start_time
        seconds = 0.0

        progress = 0
        loop_hz = 1
        loop_rate = 1/loop_hz

        while rclpy.ok():
            
            if goal_handle.is_cancel_requested == True:
                result.finish_success = False
                goal_handle.canceled(result)
                self.get_logger().warn('Goal canceled.')
                return  # break
            
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Action progress: {progress} percent.')

            current_time = self.get_clock().now()
            elapsed_time = current_time - start_time
            seconds = elapsed_time.nanoseconds / 1e9
            self.get_logger().info(f'Elapsed time: {seconds} seconds.')

            if progress == 100:
                self.get_logger().info('Progress done.')
                break

            progress = progress + 10
            time.sleep(loop_rate)

        if (rclpy.ok() == True) and (progress == 100):
            result.finish_success = True
            goal_handle.succeed()
            self.get_logger().info('Action goal successfully done.')

        return result


def main():
    rclpy.init()
    node = MyROSClass()
    rclpy.spin(node)

    Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
