"""
Sensor Bridge Launch File
=========================
Launches the localization bridge node, static sensor frame TFs,
and optionally depthimage_to_laserscan.

All robot-specific values are loaded from config/robots/<robot>.yaml via
config_loader. No hardcoded robot names, topics, or frames.

Nodes launched:
  - localization_bridge_node: ground truth odom -> map->odom + odom->base TF
  - static_transform_publisher: sensor frames (lidar, camera) relative to base
  - depthimage_to_laserscan:  (only if scan_source == "depth_camera")
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_sandbox.config_loader import load_project_config, load_robot_config


def generate_launch_description():
    # Load config
    project_cfg = load_project_config()
    robot_cfg = load_robot_config(project_cfg)

    topics = robot_cfg.get('topics', {})
    base_frame = robot_cfg.get('base_frame', 'base_link')
    scan_source = robot_cfg.get('scan_source', 'depth_camera')
    initial_pose = robot_cfg.get('initial_pose', [0.0, 0.0, 0.0])

    # Declare launch arguments (with config-driven defaults)
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    # Localization bridge: ground truth -> map->odom + odom->base TF
    localization_bridge = Node(
        package="nav2_sandbox",
        executable="localization_bridge_node.py",
        name="localization_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "global_frame": "map",
            "odom_frame": "odom",
            "base_frame": base_frame,
            "gt_odom_topic": topics.get('ground_truth_odom', '/ground_truth/odom'),
            "odom_topic": topics.get('odom', '/odom'),
            "initial_pose": [float(v) for v in initial_pose],
        }],
    )

    actions = [use_sim_time, localization_bridge]

    # Static transforms for sensor frames (lidar, camera relative to base_frame)
    # These replace the Isaac Sim ROS2PublishTransformTree which has timestamp issues.
    sensor_frames = robot_cfg.get('sensor_frames', [])
    for i, sf in enumerate(sensor_frames):
        frame_id = sf['frame_id']
        xyz = sf.get('xyz', [0.0, 0.0, 0.0])
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{frame_id}",
            arguments=[
                "--x", str(xyz[0]),
                "--y", str(xyz[1]),
                "--z", str(xyz[2]),
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", base_frame,
                "--child-frame-id", frame_id,
            ],
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        )
        actions.append(static_tf)

    # Conditionally add depthimage_to_laserscan (only for robots without native 2D lidar)
    if scan_source == "depth_camera":
        depth_to_scan = Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "scan_height": 1,
                "scan_time": 0.033,
                "range_min": 0.2,
                "range_max": 10.0,
                "output_frame_id": base_frame,
            }],
            remappings=[
                ("depth", topics.get('depth_image', '/front_cam/depth')),
                ("depth_camera_info", topics.get('camera_info', '/front_cam/camera_info')),
                ("scan", topics.get('scan', '/scan')),
            ],
        )
        actions.append(depth_to_scan)

    return LaunchDescription(actions)
