#!/usr/bin/env python3

import time
import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.timer_period = 4
        self.timer = self.create_timer(self.timer_period, self.start_control)

        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        
        self.home_pos = self.deg_to_rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.down_pos = self.deg_to_rad([0.0, 0.0, 90.0, 0.0, -90.0, 0.0])
        self.my_pos = [self.home_pos, self.down_pos]
        self.pos_idx = 0

        self.get_logger().info("Initializing...")
        time.sleep(1.0)


    def deg_to_rad(self, deg_list):
        rad_list = [deg * (math.pi / 180) for deg in deg_list]
        return rad_list

    def next_pos_idx(self):
        if self.pos_idx == 1:
            self.pos_idx = 0
            return
        if self.pos_idx == 0:
            self.pos_idx = 1
            return

    def start_control(self):
        msg = JointTrajectory()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.my_pos[self.pos_idx]
        point.time_from_start.sec = self.timer_period - 1
        self.next_pos_idx()

        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info("Moving...")


def main():
    rclpy.init()
    node = RobotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()