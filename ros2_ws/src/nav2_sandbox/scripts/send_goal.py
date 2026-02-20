#!/usr/bin/env python3
"""
Send Navigation Goal Script

Send a navigation goal to the Nav2 stack programmatically.

Usage:
    ros2 run nav2_sandbox send_goal.py --x 2.0 --y 1.0 --yaw 0.0
    ros2 run nav2_sandbox send_goal.py --x 2.0 --y 1.0
"""

import argparse
import sys
import math

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    parser = argparse.ArgumentParser(description="Send a navigation goal to Nav2")
    parser.add_argument("--x", type=float, required=True, help="Goal X position (meters)")
    parser.add_argument("--y", type=float, required=True, help="Goal Y position (meters)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw (radians)")
    parser.add_argument("--frame", type=str, default="map", help="Reference frame")

    args = parser.parse_args()

    rclpy.init()

    navigator = BasicNavigator()

    print("Waiting for Nav2 to become active...")
    # localizer=False: skip waiting for AMCL — we use localization_bridge_node instead
    navigator.waitUntilNav2Active(localizer=False)
    print("Nav2 is active!")

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = args.frame
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = args.x
    goal_pose.pose.position.y = args.y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(args.yaw / 2.0)

    print(f"Sending goal: x={args.x}, y={args.y}, yaw={args.yaw}")
    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"Estimated time remaining: {eta:.1f}s")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal reached successfully!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print(f"Goal finished with status: {result}")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

    return 0 if result == TaskResult.SUCCEEDED else 1


if __name__ == "__main__":
    sys.exit(main())
