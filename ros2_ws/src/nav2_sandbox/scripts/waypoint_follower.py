#!/usr/bin/env python3
"""
Waypoint Follower Demo Script

Navigate through a series of waypoints in the Isaac Sim warehouse.

Usage:
    ros2 run nav2_sandbox waypoint_follower.py
    ros2 run nav2_sandbox waypoint_follower.py --waypoints-file custom.yaml
    ros2 run nav2_sandbox waypoint_follower.py --loop
"""

import argparse
import sys
import os
import math
import time
import yaml

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Default waypoints in the Isaac Sim full_warehouse
DEFAULT_WAYPOINTS = [
    {"x": 2.0, "y": 2.0, "yaw": 3.14, "name": "Aisle entrance"},
    {"x": 0.0, "y": 0.0, "yaw": -0.78, "name": "Center"},
    {"x": -2.0, "y": 3.0, "yaw": 1.57, "name": "Cabinet area"},
    {"x": 2.0, "y": -1.0, "yaw": 0.0, "name": "Loading zone"},
    {"x": 4.0, "y": 2.0, "yaw": 1.57, "name": "Home (spawn)"},
]


def load_waypoints(waypoints_file=None):
    if waypoints_file and os.path.exists(waypoints_file):
        with open(waypoints_file, "r") as f:
            data = yaml.safe_load(f)
            return data.get("waypoints", DEFAULT_WAYPOINTS)
    return DEFAULT_WAYPOINTS


def create_pose(navigator, x, y, yaw, frame="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


def main():
    parser = argparse.ArgumentParser(description="Navigate through waypoints")
    parser.add_argument("--waypoints-file", type=str, default=None)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--pause", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()

    waypoints = load_waypoints(args.waypoints_file)
    print(f"Loaded {len(waypoints)} waypoints")

    print("Waiting for Nav2 to become active...")
    # localizer=False: skip waiting for AMCL — we use localization_bridge_node instead
    navigator.waitUntilNav2Active(localizer=False)
    print("Nav2 is active!")

    try:
        loop_count = 0
        while True:
            loop_count += 1
            print(f"\n=== Waypoint sequence (loop {loop_count}) ===")

            for i, wp in enumerate(waypoints):
                name = wp.get("name", f"Waypoint {i+1}")
                print(f"\nNavigating to {name} (x={wp['x']}, y={wp['y']})")

                pose = create_pose(navigator, wp["x"], wp["y"], wp.get("yaw", 0.0))
                navigator.goToPose(pose)

                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    if feedback:
                        eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                        if eta > 0:
                            print(f"  ETA: {eta:.1f}s", end="\r")
                    rclpy.spin_once(navigator, timeout_sec=0.5)

                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f"  Reached {name}!")
                    if args.pause > 0:
                        print(f"  Pausing for {args.pause}s...")
                        time.sleep(args.pause)
                elif result == TaskResult.CANCELED:
                    print(f"  Canceled navigation to {name}")
                    break
                else:
                    print(f"  Failed to reach {name}, continuing...")

            if not args.loop:
                break
            print("\nRestarting waypoint sequence...")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        navigator.cancelTask()

    print("\nWaypoint following complete!")
    navigator.lifecycleShutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
