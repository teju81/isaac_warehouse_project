"""
Navigation Launch File
======================
Launches the Nav2 stack directly (without AMCL — localization is handled
by the localization_bridge_node in sensor_bridge.launch.py).

All robot-specific values (cmd_vel topic, nav2 params file) are resolved
from config/robots/<robot>.yaml and config/nav2_<robot>_params.yaml via
config_loader. No hardcoded robot names, topics, or frames.

Nodes launched:
  - map_server (serves the static 2D map)
  - planner_server (global path planning)
  - controller_server (local path following)
  - bt_navigator (behavior tree orchestration)
  - behavior_server (recovery behaviors)
  - smoother_server (path smoothing)
  - velocity_smoother (velocity smoothing)
  - waypoint_follower (multi-waypoint navigation)
  - lifecycle_manager (auto-starts all above nodes)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_sandbox.config_loader import (
    load_project_config, load_robot_config, get_map_path, get_nav2_params_path,
)


def generate_launch_description():
    # Load config
    project_cfg = load_project_config()
    robot_cfg = load_robot_config(project_cfg)

    topics = robot_cfg.get('topics', {})
    cmd_vel_topic = topics.get('cmd_vel', '/cmd_vel')

    default_map = get_map_path(project_cfg)
    default_params = get_nav2_params_path(project_cfg)

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_yaml = DeclareLaunchArgument(
        "map", default_value=default_map,
        description="Full path to the map YAML file (default: from project_config.yaml)"
    )
    params_file = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Full path to the Nav2 params YAML file"
    )
    autostart = DeclareLaunchArgument("autostart", default_value="true")

    # Shared parameters for all nodes
    configured_params = [
        LaunchConfiguration("params_file"),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    # Map server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=configured_params + [{
            "yaml_filename": LaunchConfiguration("map"),
        }],
    )

    # Planner server (global path planning)
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=configured_params,
    )

    # Controller server (local path following)
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=configured_params,
        remappings=[("cmd_vel", cmd_vel_topic)],
    )

    # BT navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=configured_params,
    )

    # Behavior server (recovery behaviors)
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=configured_params,
        remappings=[("cmd_vel", cmd_vel_topic)],
    )

    # Smoother server
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=configured_params,
    )

    # Velocity smoother
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=configured_params,
        remappings=[
            ("cmd_vel", cmd_vel_topic),
            ("cmd_vel_smoothed", cmd_vel_topic),
        ],
    )

    # Waypoint follower
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=configured_params,
    )

    # Lifecycle manager — auto-starts all Nav2 nodes
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("autostart"),
            "node_names": [
                "map_server",
                "planner_server",
                "controller_server",
                "bt_navigator",
                "behavior_server",
                "smoother_server",
                "velocity_smoother",
                "waypoint_follower",
            ],
        }],
    )

    return LaunchDescription([
        use_sim_time,
        map_yaml,
        params_file,
        autostart,
        map_server,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        smoother_server,
        velocity_smoother,
        waypoint_follower,
        lifecycle_manager,
    ])
