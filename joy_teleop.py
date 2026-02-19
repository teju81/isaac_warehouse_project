#!/usr/bin/env python3
"""
Simple joystick teleop — reads /joy and publishes Twist to a configurable topic.

Usage:
    python3 joy_teleop.py                          # defaults to /carter/cmd_vel
    python3 joy_teleop.py --topic /explorer/cmd_vel
    python3 joy_teleop.py --topic /drone/cmd_vel --linear-axis 3 --angular-axis 2
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyTeleop(Node):
    def __init__(self, topic, linear_axis, angular_axis, vertical_axis,
                 linear_scale, angular_scale, vertical_scale, deadzone):
        super().__init__("joy_teleop")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(Joy, "/joy", self.joy_cb, 10)
        self.linear_axis = linear_axis
        self.angular_axis = angular_axis
        self.vertical_axis = vertical_axis
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        self.vertical_scale = vertical_scale
        self.deadzone = deadzone
        self.get_logger().info(
            f"Publishing to {topic} | "
            f"linear=axis{linear_axis}*{linear_scale} "
            f"angular=axis{angular_axis}*{angular_scale} "
            f"vertical=axis{vertical_axis}*{vertical_scale} "
            f"deadzone={deadzone}")

    def _apply_deadzone(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def joy_cb(self, msg):
        t = Twist()
        axes = msg.axes
        if len(axes) > self.linear_axis:
            t.linear.x = self._apply_deadzone(axes[self.linear_axis]) * self.linear_scale
        if len(axes) > self.angular_axis:
            t.angular.z = self._apply_deadzone(axes[self.angular_axis]) * self.angular_scale
        if len(axes) > self.vertical_axis:
            t.linear.z = self._apply_deadzone(axes[self.vertical_axis]) * self.vertical_scale
        self.pub.publish(t)


def main():
    parser = argparse.ArgumentParser(description="Joystick teleop for Isaac Sim robots")
    parser.add_argument("--topic", default="/carter/cmd_vel", help="Twist topic to publish")
    parser.add_argument("--linear-axis", type=int, default=1, help="Axis for forward/back")
    parser.add_argument("--angular-axis", type=int, default=0, help="Axis for yaw rotation")
    parser.add_argument("--vertical-axis", type=int, default=3, help="Axis for up/down (flying)")
    parser.add_argument("--linear-scale", type=float, default=2.0, help="Linear speed scale")
    parser.add_argument("--angular-scale", type=float, default=1.5, help="Angular speed scale")
    parser.add_argument("--vertical-scale", type=float, default=1.0, help="Vertical speed scale")
    parser.add_argument("--deadzone", type=float, default=0.15, help="Axis deadzone (ignore values below this)")
    args = parser.parse_args()

    rclpy.init()
    node = JoyTeleop(
        topic=args.topic,
        linear_axis=args.linear_axis,
        angular_axis=args.angular_axis,
        vertical_axis=args.vertical_axis,
        linear_scale=args.linear_scale,
        angular_scale=args.angular_scale,
        vertical_scale=args.vertical_scale,
        deadzone=args.deadzone,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
