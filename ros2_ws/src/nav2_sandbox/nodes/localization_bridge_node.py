#!/usr/bin/env python3
"""
Localization Bridge Node
========================
Bridges Isaac Sim ground truth to Nav2's expected localization interface.

Subscribes to ground truth odometry from Isaac Sim and publishes:
  1. map -> odom TF  (offset by initial_pose — accounts for odom starting at 0,0,0)
  2. odom -> base TF  (from the odom message pose)
  3. Republished odometry on /<ns>/odom

IsaacComputeOdometry reports position relative to the robot's initial pose
(i.e., odom starts at 0,0,0). The initial_pose parameter provides the robot's
spawn position in world (map) coordinates so map -> odom corrects for this offset.

This node is the PLUGGABLE component: swap it out for AMCL, visual odometry,
or any other localizer. The interface to Nav2 stays the same (map -> odom TF).

All robot-specific values (topics, frames) are passed as ROS2 parameters
from the launch file. No hardcoded robot names.

Published TF:
    map -> odom       (the localization correction, set from initial_pose)
    odom -> base_link (from ground truth odometry pose)

Subscribed topics:
    /<ns>/ground_truth/odom  (nav_msgs/Odometry from Isaac Sim)

Published topics:
    /<ns>/odom  (nav_msgs/Odometry, republished for Nav2)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class LocalizationBridgeNode(Node):
    def __init__(self):
        super().__init__("localization_bridge")

        # Declare parameters (defaults are generic — actual values from launch file)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gt_odom_topic", "/ground_truth/odom")
        self.declare_parameter("odom_topic", "/odom")
        # Initial world pose [x, y, yaw_radians] — the robot's spawn position
        # in map (world) coordinates. IsaacComputeOdometry reports position
        # relative to start, so this offset is applied as map -> odom.
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0])

        self._global_frame = self.get_parameter("global_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        gt_odom_topic = self.get_parameter("gt_odom_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        initial_pose = self.get_parameter("initial_pose").value

        # Parse initial pose [x, y, yaw]
        init_x = float(initial_pose[0]) if len(initial_pose) > 0 else 0.0
        init_y = float(initial_pose[1]) if len(initial_pose) > 1 else 0.0
        init_yaw = float(initial_pose[2]) if len(initial_pose) > 2 else 0.0

        # TF broadcaster for map->odom and odom->base
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to ground truth odometry from Isaac Sim
        self._gt_odom_sub = self.create_subscription(
            Odometry, gt_odom_topic, self._gt_odom_callback, 10
        )

        # Republish odometry for Nav2
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # map -> odom: offset by initial world pose
        # IsaacComputeOdometry starts at (0,0,0), so map -> odom translates
        # the odom origin to the robot's actual spawn position in the world.
        self._map_to_odom = TransformStamped()
        self._map_to_odom.header.frame_id = self._global_frame
        self._map_to_odom.child_frame_id = self._odom_frame
        self._map_to_odom.transform.translation.x = init_x
        self._map_to_odom.transform.translation.y = init_y
        self._map_to_odom.transform.translation.z = 0.0
        # Convert yaw to quaternion (rotation around Z axis)
        self._map_to_odom.transform.rotation.x = 0.0
        self._map_to_odom.transform.rotation.y = 0.0
        self._map_to_odom.transform.rotation.z = math.sin(init_yaw / 2.0)
        self._map_to_odom.transform.rotation.w = math.cos(init_yaw / 2.0)

        # odom -> base_link (populated from each odom message)
        self._odom_to_base = TransformStamped()
        self._odom_to_base.header.frame_id = self._odom_frame
        self._odom_to_base.child_frame_id = self._base_frame

        self._received_first_odom = False
        self.get_logger().info(
            f"Localization bridge started: "
            f"{gt_odom_topic} -> {self._global_frame}->{self._odom_frame}->{self._base_frame} TF + {odom_topic}"
        )
        if init_x != 0.0 or init_y != 0.0 or init_yaw != 0.0:
            self.get_logger().info(
                f"Initial pose offset (map->odom): x={init_x:.2f}, y={init_y:.2f}, yaw={init_yaw:.3f} rad"
            )

    def _gt_odom_callback(self, msg: Odometry):
        """Process ground truth odometry from Isaac Sim."""
        if not self._received_first_odom:
            self._received_first_odom = True
            self.get_logger().info(
                f"Received first ground truth odometry message "
                f"(pos: {msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})"
            )

        stamp = msg.header.stamp

        # Republish on the topic Nav2 expects, with correct frame IDs
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist
        self._odom_pub.publish(odom_msg)

        # Publish map -> odom TF (initial pose offset)
        self._map_to_odom.header.stamp = stamp
        self._tf_broadcaster.sendTransform(self._map_to_odom)

        # Publish odom -> base_link TF (from the odom pose)
        self._odom_to_base.header.stamp = stamp
        self._odom_to_base.transform.translation.x = msg.pose.pose.position.x
        self._odom_to_base.transform.translation.y = msg.pose.pose.position.y
        self._odom_to_base.transform.translation.z = msg.pose.pose.position.z
        self._odom_to_base.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(self._odom_to_base)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
