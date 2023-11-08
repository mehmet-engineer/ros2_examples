#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

"""
  @author: Mehmet Kahraman
  @date: 07.11.2023
  @about: executors and callback groups example, publisher and subscriber are working synchronously and subscriber callbacks as well.
"""

class PublisherClass(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.cb_group = ReentrantCallbackGroup()
        self.timer_publisher = self.create_publisher(String, '/publisher_executor', 10)
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.cb_group)
        time.sleep(1)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello, this message comes from publisher_executor_node"
        self.timer_publisher.publish(msg)
        self.get_logger().info("publisher_executor_node is publishing...")

class SubscriberClass(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        self.cb_group = ReentrantCallbackGroup()
        self.value_1 = ""
        self.value_2 = ""
        self.string_subscription = self.create_subscription(String, "/publisher_executor", 
                                                            self.subscriber_callback_1, 10, callback_group=self.cb_group)
        self.string_subscription
        self.other_subscription = self.create_subscription(String, "/publisher_executor", 
                                                           self.subscriber_callback_2, 10, callback_group=self.cb_group)
        self.other_subscription

        time.sleep(1)

    def subscriber_callback_1(self, msg):
        self.value_1 = msg.data
        self.get_logger().info("subscriber_node is receiving...")
    
    def subscriber_callback_2(self, msg):
        self.get_logger().info("subscriber_callback_2 started to own process...")
        # heavy process !!
        time.sleep(5)
        # heavy process !!
        self.value_2 = msg.data
        self.get_logger().info("subscriber_callback_2 is running...")


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    pub_node = PublisherClass()
    sub_node = SubscriberClass()

    executor.add_node(pub_node)
    executor.add_node(sub_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
