"""
Full Bringup Launch File
========================
All-in-one launcher: sensor bridge + Nav2 navigation + RViz.
Assumes Isaac Sim is already running on the host with nav2_sim_launcher.py.

Robot selection is driven by project_config.yaml — change 'robot: <name>'
to switch between carter, g1, anymal, etc. All sub-launch files resolve
their own config automatically.

Startup sequence (timed):
  0s  - Sensor bridge (localization + optional depth-to-scan)
  3s  - Nav2 stack (map_server, planner, controller, bt_navigator, etc.)
  5s  - RViz (optional, enabled by default)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_sandbox.config_loader import (
    load_project_config, get_map_path, get_nav2_params_path,
)


def generate_launch_description():
    pkg_share = get_package_share_directory("nav2_sandbox")

    # Load config for defaults
    project_cfg = load_project_config()
    default_map = get_map_path(project_cfg)
    default_params = get_nav2_params_path(project_cfg)
    default_rviz = os.path.join(pkg_share, "config", "rviz_nav.rviz")

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_yaml = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Full path to the map YAML file (default: from project_config.yaml)"
    )
    params_file = DeclareLaunchArgument(
        "params_file", default_value=default_params
    )
    rviz = DeclareLaunchArgument("rviz", default_value="true")
    rviz_config = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz
    )

    # --- Sensor bridge (starts immediately) ---
    sensor_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sensor_bridge.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # --- Nav2 stack (delayed 3s to wait for TF) ---
    navigation = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "navigation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": "true",
                }.items(),
            )
        ],
    )

    # --- RViz (delayed 5s, conditional) ---
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        ],
    )

    return LaunchDescription([
        use_sim_time,
        map_yaml,
        params_file,
        rviz,
        rviz_config,
        sensor_bridge,
        navigation,
        rviz_node,
    ])
