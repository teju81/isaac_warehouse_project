#!/usr/bin/env python3
"""
Warehouse Multi-Robot Simulation with ROS 2 Bridge
===================================================
Spawns a warehouse environment with ANYmal-D, Unitree G1, and Nova Carter robots.
Each robot is equipped with RGB cameras, depth cameras, LiDAR, and IMU sensors.
All sensor data is published over ROS 2 topics for downstream algorithm containers.

Usage:
    cd ~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /path/to/spawn_warehouse.py [--headless] [--warehouse TYPE]

Author: Generated for raviteja's Isaac Sim 5.1 setup
"""

import argparse
import sys
import numpy as np

# =============================================================================
# 1. Parse arguments BEFORE launching SimulationApp
# =============================================================================
parser = argparse.ArgumentParser(description="Warehouse Multi-Robot Simulation")
parser.add_argument("--headless", action="store_true", help="Run headless (no GUI)")
parser.add_argument(
    "--warehouse",
    type=str,
    default="full_warehouse",
    choices=["warehouse", "full_warehouse", "warehouse_with_forklifts", "warehouse_multiple_shelves"],
    help="Which warehouse environment to load",
)
args = parser.parse_args()

# =============================================================================
# 2. Launch Isaac Sim application
# =============================================================================
from isaacsim import SimulationApp

CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": args.headless,
    "width": 1920,
    "height": 1080,
}

simulation_app = SimulationApp(CONFIG)
print("[INFO] Isaac Sim application started.")

# =============================================================================
# 3. Omniverse imports (MUST come after SimulationApp is created)
# =============================================================================
import carb
import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import create_prim, define_prim
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf
import usdrt.Sdf
import omni.replicator.core as rep

# Enable required extensions
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.physics")
simulation_app.update()
print("[INFO] ROS 2 Bridge and Sensors extensions enabled.")

# =============================================================================
# 3a. Print internal ROS 2 / DDS version details
# =============================================================================
import os
import platform

def print_ros2_bridge_info():
    """Print details about the internal ROS 2 bridge bundled with Isaac Sim."""
    print("\n" + "=" * 60)
    print("  Internal ROS 2 Bridge Info")
    print("=" * 60)

    # OS info
    print(f"  Platform:           {platform.system()} {platform.release()}")
    try:
        import distro
        print(f"  OS:                 {distro.name()} {distro.version()}")
    except ImportError:
        pass

    # ROS distro setting from the extension config
    settings = carb.settings.get_settings()
    ros_distro_setting = settings.get("/exts/isaacsim.ros2.bridge/ros_distro")
    print(f"  ros_distro setting: {ros_distro_setting}")

    # Resolved ROS_DISTRO env var (set by setup_ros_env.sh / the bridge)
    ros_distro_env = os.environ.get("ROS_DISTRO", "not set")
    print(f"  ROS_DISTRO env:     {ros_distro_env}")

    # RMW implementation
    rmw_impl = os.environ.get("RMW_IMPLEMENTATION", "not set")
    print(f"  RMW_IMPLEMENTATION: {rmw_impl}")

    # ROS domain ID
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0 (default)")
    print(f"  ROS_DOMAIN_ID:      {domain_id}")

    # Bridge extension version from carb settings
    bridge_version = settings.get("/exts/isaacsim.ros2.bridge/version")
    if bridge_version:
        print(f"  Bridge ext version: {bridge_version}")

    # Try to get rclpy version from the internal bundle
    try:
        import rclpy
        rclpy_path = getattr(rclpy, "__file__", "unknown")
        print(f"  rclpy location:     {rclpy_path}")
    except Exception as e:
        print(f"  rclpy:              not queryable ({e})")

    # LD_LIBRARY_PATH entries related to ros2 bridge
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    ros2_libs = [p for p in ld_path.split(":") if "ros2" in p.lower() or "bridge" in p.lower()]
    if ros2_libs:
        print(f"  ROS 2 lib paths:    {ros2_libs[0]}")
        for p in ros2_libs[1:]:
            print(f"                      {p}")

    # FastDDS version (from shared lib if available)
    for lib_dir in ros2_libs:
        fastdds_libs = [f for f in os.listdir(lib_dir) if "fastrtps" in f or "fastdds" in f] if os.path.isdir(lib_dir) else []
        if fastdds_libs:
            print(f"  FastDDS libs:       {', '.join(sorted(set(fastdds_libs)))}")
            break

    print("=" * 60 + "\n")

print_ros2_bridge_info()

# =============================================================================
# 4. Configuration
# =============================================================================

# Asset root path (configured via .kit file or command-line flag)
ASSETS_ROOT = get_assets_root_path()
if ASSETS_ROOT is None:
    # Fallback to local path if Nucleus/CDN is not configured
    ASSETS_ROOT = "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1"
    print(f"[WARN] get_assets_root_path() returned None. Using local fallback: {ASSETS_ROOT}")
else:
    print(f"[INFO] Assets root: {ASSETS_ROOT}")

# Warehouse environment USD paths
WAREHOUSE_PATHS = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "full_warehouse": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "warehouse_with_forklifts": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "warehouse_multiple_shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
}

# Robot configurations — spawn info, sensors, and control all in one place
ROBOTS = {
    "anymal_d": {
        "usd_path": "/Isaac/Robots/ANYbotics/anymal_d/anymal_d.usd",
        "prim_path": "/World/Robots/ANYmalD",
        "position": (2.0, 0.0, 0.5),
        "orientation": (0.0, 0.0, 0.0),  # euler degrees
        "namespace": "anymal",
        "type": "legged",
        "base_link": "base",
        "camera": {
            "parent_link": "base",
            "local_position": (0.35, 0.0, 0.05),
            "focal_length": 24.0,
        },
        "lidar_3d": {
            "parent_prim": "/World/Robots/ANYmalD/base",
            "local_position": (0.0, 0.0, 0.15),
            "config": "Example_Rotary",
            "frame_id": "base_lidar",
        },
        "imu": {
            "parent_prim": "/World/Robots/ANYmalD/base",
            "frame_id": "base",
        },
    },
    "unitree_g1": {
        "usd_path": "/Isaac/Robots/Unitree/G1/g1.usd",
        "prim_path": "/World/Robots/UnitreeG1",
        "position": (4.0, 2.0, 1.0),
        "orientation": (0.0, 0.0, 90.0),
        "namespace": "g1",
        "type": "legged",
        "base_link": "pelvis",
        "camera": {
            "parent_link": "torso_link",
            "local_position": (0.15, 0.0, 0.0),
            "focal_length": 24.0,
        },
        "lidar_3d": {
            "parent_prim": "/World/Robots/UnitreeG1/torso_link",
            "local_position": (0.0, 0.0, 0.1),
            "config": "Example_Rotary",
            "frame_id": "torso_lidar",
        },
        "imu": {
            "parent_prim": "/World/Robots/UnitreeG1/pelvis",
            "frame_id": "pelvis",
        },
    },
    "nova_carter": {
        "usd_path": "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        "prim_path": "/World/Robots/NovaCarter",
        "position": (0.0, -2.0, 0.0),
        "orientation": (0.0, 0.0, 0.0),
        "namespace": "carter",
        "type": "wheeled",
        "camera": {
            "parent_link": "chassis_link",
            "local_position": (0.3, 0.0, 0.15),
            "focal_length": 24.0,
        },
        "lidar_3d": {
            "parent_prim": "/World/Robots/NovaCarter/chassis_link",
            "local_position": (-0.364, 0.0, 0.228),
            "config": "Example_Rotary",
            "frame_id": "rear_lidar",
        },
        "lidar_2d": {
            "parent_prim": "/World/Robots/NovaCarter/chassis_link",
            "local_position": (0.0, 0.0, 0.245),
            "config": "Example_Rotary_2D",
            "frame_id": "front_2d_lidar",
        },
        "imu": {
            "parent_prim": "/World/Robots/NovaCarter/chassis_link",
            "frame_id": "chassis_link",
        },
        "drive": {
            "wheel_radius": 0.04295,
            "wheel_base": 0.4132,
            "wheel_joint_names": ["joint_wheel_left", "joint_wheel_right"],
        },
    },
    "quadcopter": {
        "usd_path": "/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd",
        "prim_path": "/World/Robots/Quadcopter",
        "position": (0.0, 4.0, 2.0),
        "orientation": (0.0, 0.0, 0.0),
        "namespace": "drone",
        "type": "multirotor",
        "camera": {
            "parent_link": "chassis",
            "local_position": (0.1, 0.0, -0.02),
            "focal_length": 18.0,
        },
        "imu": {
            "parent_prim": "/World/Robots/Quadcopter/chassis",
            "frame_id": "chassis",
        },
    },
    "explorer_camera": {
        "prim_path": "/World/Robots/ExplorerCam",
        "position": (0.0, 0.0, 3.0),
        "orientation": (0.0, 0.0, 0.0),
        "namespace": "explorer",
        "type": "floating_camera",
        "base_link": "body",
        "camera": {
            "parent_link": "body",
            "local_position": (0.0, 0.0, 0.0),
            "focal_length": 18.0,
        },
    },
}

# Surveillance / monitoring cameras at fixed positions in the warehouse
FIXED_CAMERAS = [
    {
        "name": "overhead_cam_1",
        "prim_path": "/World/Cameras/overhead_cam_1",
        "position": (0.0, 0.0, 8.0),
        "target": (0.0, 0.0, 0.0),  # look-at target
        "focal_length": 18.0,
        "resolution": (1280, 720),
    },
    {
        "name": "corner_cam_1",
        "prim_path": "/World/Cameras/corner_cam_1",
        "position": (10.0, 8.0, 5.0),
        "target": (0.0, 0.0, 0.0),
        "focal_length": 24.0,
        "resolution": (1280, 720),
    },
    {
        "name": "entrance_cam",
        "prim_path": "/World/Cameras/entrance_cam",
        "position": (-5.0, 0.0, 4.0),
        "target": (2.0, 0.0, 0.0),
        "focal_length": 35.0,
        "resolution": (1920, 1080),
    },
]


# =============================================================================
# 5. Helper functions
# =============================================================================

def load_warehouse(warehouse_type: str):
    """Load the warehouse environment USD onto the stage."""
    usd_path = ASSETS_ROOT + WAREHOUSE_PATHS[warehouse_type]
    print(f"[INFO] Loading warehouse: {usd_path}")
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/Warehouse")
    print(f"[INFO] Warehouse '{warehouse_type}' loaded.")


def find_articulation_root(robot_prim_path: str) -> str:
    """Find the prim with ArticulationRootAPI in the robot's hierarchy."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(robot_prim_path)

    # Check the root prim itself
    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return robot_prim_path

    # Search children
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim.GetPath().pathString

    # Fallback to root path
    print(f"[WARN] No ArticulationRootAPI found under {robot_prim_path}, using root path")
    return robot_prim_path


def spawn_robot(name: str, config: dict):
    """Spawn a robot from USD or create a simple rigid body for virtual entities."""
    prim_path = config["prim_path"]
    pos = config["position"]
    rot = config["orientation"]
    stage = omni.usd.get_context().get_stage()

    if config.get("type") == "floating_camera":
        # Create an invisible rigid body with no collision — a free-flying camera mount
        print(f"[INFO] Creating floating camera '{name}' at position {pos}...")
        from pxr import PhysxSchema

        define_prim(prim_path, "Xform")
        xformable = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], pos[2]))
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(rot[0], rot[1], rot[2]))

        # Create the body child prim with rigid body physics (no collision, no mass shape)
        base_link = config.get("base_link", "body")
        body_path = f"{prim_path}/{base_link}"
        define_prim(body_path, "Xform")
        body_prim = stage.GetPrimAtPath(body_path)
        UsdPhysics.RigidBodyAPI.Apply(body_prim)
        mass_api = UsdPhysics.MassAPI.Apply(body_prim)
        mass_api.GetMassAttr().Set(1.0)

        # Disable gravity so it floats
        PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
        PhysxSchema.PhysxRigidBodyAPI(body_prim).GetDisableGravityAttr().Set(True)

        config["articulation_root"] = prim_path
        print(f"[INFO] Floating camera '{name}' created at {prim_path}")
        return

    usd_path = ASSETS_ROOT + config["usd_path"]
    print(f"[INFO] Spawning robot '{name}' at position {pos}...")
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    # Set the transform
    robot_prim = stage.GetPrimAtPath(prim_path)
    if robot_prim.IsValid():
        xformable = UsdGeom.Xformable(robot_prim)

        # Clear existing transforms
        xformable.ClearXformOpOrder()

        # Set translation
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(pos[0], pos[1], pos[2]))

        # Set rotation (euler XYZ in degrees)
        rotate_op = xformable.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(rot[0], rot[1], rot[2]))

        # Find the existing ArticulationRootAPI prim within this robot's hierarchy
        art_root = find_articulation_root(prim_path)
        config["articulation_root"] = art_root
        print(f"[INFO] Robot '{name}' spawned at {prim_path} (articulation root: {art_root})")

        # For multirotors: disable gravity on all rigid body prims
        if config.get("type") == "multirotor":
            _disable_gravity_on_robot(stage, prim_path)
    else:
        print(f"[ERROR] Failed to spawn robot '{name}' — prim not valid at {prim_path}")


def _disable_gravity_on_robot(stage, robot_prim_path: str):
    """Disable gravity on all rigid body prims under a robot."""
    from pxr import PhysxSchema
    root = stage.GetPrimAtPath(robot_prim_path)
    count = 0
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Apply PhysxRigidBodyAPI if not already present
            if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            physx_rb = PhysxSchema.PhysxRigidBodyAPI(prim)
            physx_rb.GetDisableGravityAttr().Set(True)
            count += 1
    print(f"[INFO] Disabled gravity on {count} rigid bodies under {robot_prim_path}")


def create_camera_prim(prim_path: str, position: tuple, target: tuple, focal_length: float = 24.0):
    """Create a USD camera at the given position, aimed at the target."""
    stage = omni.usd.get_context().get_stage()

    # Create the camera prim
    camera_prim = UsdGeom.Camera.Define(stage, prim_path)
    camera_prim.GetFocalLengthAttr().Set(focal_length)
    camera_prim.GetHorizontalApertureAttr().Set(20.955)
    camera_prim.GetVerticalApertureAttr().Set(15.2908)
    camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))

    # Set position via xform ops
    xformable = UsdGeom.Xformable(camera_prim.GetPrim())
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))

    print(f"[INFO] Camera created at {prim_path}")
    return camera_prim


def create_fixed_cameras():
    """Create surveillance cameras at fixed positions in the warehouse."""
    # Ensure parent prim exists
    define_prim("/World/Cameras", "Xform")

    for cam_cfg in FIXED_CAMERAS:
        create_camera_prim(
            prim_path=cam_cfg["prim_path"],
            position=cam_cfg["position"],
            target=cam_cfg["target"],
            focal_length=cam_cfg["focal_length"],
        )


def create_robot_camera(parent_prim_path: str, camera_name: str,
                        local_position: tuple, focal_length: float = 24.0):
    """Create a camera prim as a child of a robot link."""
    camera_prim_path = f"{parent_prim_path}/{camera_name}"
    stage = omni.usd.get_context().get_stage()

    camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
    camera_prim.GetFocalLengthAttr().Set(focal_length)
    camera_prim.GetHorizontalApertureAttr().Set(20.955)
    camera_prim.GetVerticalApertureAttr().Set(15.2908)
    camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

    xformable = UsdGeom.Xformable(camera_prim.GetPrim())
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*local_position))

    print(f"[INFO] Robot camera created at {camera_prim_path}")
    return camera_prim_path


def create_imu_sensor(parent_prim_path: str, sensor_name: str = "imu_sensor"):
    """Create a physics-based IMU sensor at the specified parent prim."""
    result, _ = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=f"/{sensor_name}",
        parent=parent_prim_path,
        sensor_period=-1.0,  # match physics rate
        translation=Gf.Vec3d(0, 0, 0),
        orientation=Gf.Quatd(1, 0, 0, 0),
    )
    if result:
        full_path = f"{parent_prim_path}/{sensor_name}"
        print(f"[INFO] IMU sensor created at {full_path}")
        return full_path
    else:
        print(f"[WARN] Failed to create IMU sensor at {parent_prim_path}")
        return None


def create_rtx_lidar_sensor(parent_prim_path: str, sensor_name: str,
                            config: str, translation: tuple):
    """Create an RTX LiDAR sensor at the specified position."""
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=f"/{sensor_name}",
        parent=parent_prim_path,
        config=config,
        translation=Gf.Vec3d(*translation),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if sensor:
        sensor_path = sensor.GetPath().pathString
        print(f"[INFO] RTX LiDAR sensor created at {sensor_path}")
        return sensor_path
    else:
        print(f"[WARN] Failed to create RTX LiDAR at {parent_prim_path}")
        return None


# =============================================================================
# 6. ROS 2 OmniGraph setup for sensor publishing
# =============================================================================

def create_ros2_camera_graph(camera_prim_path: str, topic_prefix: str, graph_path: str):
    """
    Create an OmniGraph that publishes RGB, depth, and camera_info for a camera.
    """
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("CameraHelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                    ("CameraHelperRGB.inputs:topicName", f"{topic_prefix}/rgb"),
                    ("CameraHelperRGB.inputs:type", "rgb"),
                    ("CameraHelperRGB.inputs:frameId", camera_prim_path.split("/")[-1]),
                    ("CameraHelperDepth.inputs:topicName", f"{topic_prefix}/depth"),
                    ("CameraHelperDepth.inputs:type", "depth"),
                    ("CameraHelperDepth.inputs:frameId", camera_prim_path.split("/")[-1]),
                    ("CameraInfoHelper.inputs:topicName", f"{topic_prefix}/camera_info"),
                    ("CameraInfoHelper.inputs:frameId", camera_prim_path.split("/")[-1]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "CameraHelperRGB.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "CameraHelperDepth.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "CameraInfoHelper.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperRGB.inputs:renderProductPath"),
                    ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperDepth.inputs:renderProductPath"),
                    ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoHelper.inputs:renderProductPath"),
                    ("ROS2Context.outputs:context", "CameraHelperRGB.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraHelperDepth.inputs:context"),
                    ("ROS2Context.outputs:context", "CameraInfoHelper.inputs:context"),
                ],
            },
        )
        print(f"[INFO] ROS 2 camera graph created: {graph_path} → {topic_prefix}/rgb, {topic_prefix}/depth")
    except Exception as e:
        print(f"[WARN] Failed to create camera graph {graph_path}: {e}")


def create_ros2_lidar_graph(lidar_prim_path: str, topic_prefix: str, graph_path: str):
    """
    Create an OmniGraph that publishes LaserScan and PointCloud2 for a LiDAR.
    """
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("CreateRenderProduct.inputs:cameraPrim", lidar_prim_path),
                    ("LidarHelper.inputs:topicName", f"{topic_prefix}/point_cloud"),
                    ("LidarHelper.inputs:frameId", lidar_prim_path.split("/")[-1]),
                    ("LidarHelper.inputs:type", "point_cloud"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                    ("ROS2Context.outputs:context", "LidarHelper.inputs:context"),
                ],
            },
        )
        print(f"[INFO] ROS 2 LiDAR graph created: {graph_path} → {topic_prefix}/point_cloud")
    except Exception as e:
        print(f"[WARN] Failed to create LiDAR graph {graph_path}: {e}")


def create_ros2_clock_graph():
    """Publish /clock so ROS 2 nodes can use sim time."""
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ROS2/ClockGraph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SimTime", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SimTime.inputs:execIn"),
                    ("ROS2Context.outputs:context", "SimTime.inputs:context"),
                ],
            },
        )
        print("[INFO] ROS 2 clock publisher created → /clock")
    except Exception as e:
        print(f"[WARN] Failed to create clock graph: {e}")


def create_ros2_tf_graph(robot_prim_path: str, namespace: str, graph_path: str):
    """Publish TF tree for a robot."""
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("PublishTF.inputs:topicName", "/tf"),
                    ("PublishTF.inputs:targetPrims", [robot_prim_path]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishTF.inputs:context"),
                ],
            },
        )
        print(f"[INFO] ROS 2 TF graph created: {graph_path} for {robot_prim_path}")
    except Exception as e:
        print(f"[WARN] Failed to create TF graph {graph_path}: {e}")


def create_ros2_joint_state_graph(robot_prim_path: str, namespace: str, graph_path: str):
    """Publish joint states for a robot."""
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("PublishJointState.inputs:topicName", f"/{namespace}/joint_states"),
                    ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim_path)]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ],
            },
        )
        print(f"[INFO] Joint state graph: {graph_path} -> /{namespace}/joint_states")
    except Exception as e:
        print(f"[WARN] Failed to create joint state graph {graph_path}: {e}")


def create_ros2_imu_graph(imu_prim_path: str, namespace: str,
                          frame_id: str, graph_path: str):
    """Read IMU sensor and publish as ROS 2 Imu message."""
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                    ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_prim_path)]),
                    ("ReadIMU.inputs:readGravity", True),
                    ("PublishIMU.inputs:topicName", f"/{namespace}/imu"),
                    ("PublishIMU.inputs:frameId", frame_id),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                    ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                    ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                    ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                    ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                    ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                    ("ROS2Context.outputs:context", "PublishIMU.inputs:context"),
                ],
            },
        )
        print(f"[INFO] IMU graph: {graph_path} -> /{namespace}/imu")
    except Exception as e:
        print(f"[WARN] Failed to create IMU graph {graph_path}: {e}")


def create_ros2_rtx_lidar_pipeline(sensor_prim_path: str, namespace: str,
                                   topic_suffix: str, frame_id: str,
                                   publish_laser_scan: bool = False):
    """Set up RTX LiDAR publishing via replicator writers."""
    try:
        hydra_texture = rep.create.render_product(
            sensor_prim_path, [1, 1], name=f"{namespace}_{topic_suffix}"
        )

        pc_writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
        pc_writer.initialize(
            topicName=f"/{namespace}/{topic_suffix}/point_cloud",
            frameId=frame_id,
        )
        pc_writer.attach([hydra_texture])

        if publish_laser_scan:
            scan_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            scan_writer.initialize(
                topicName=f"/{namespace}/{topic_suffix}/scan",
                frameId=frame_id,
            )
            scan_writer.attach([hydra_texture])

        print(f"[INFO] RTX LiDAR pipeline: {sensor_prim_path} -> "
              f"/{namespace}/{topic_suffix}/point_cloud")
    except Exception as e:
        print(f"[WARN] Failed to create LiDAR pipeline for {sensor_prim_path}: {e}")


def create_ros2_differential_drive_graph(
    robot_prim_path: str, namespace: str,
    wheel_radius: float, wheel_base: float,
    wheel_joint_names: list, graph_path: str
):
    """Create differential drive control + odometry graph for a wheeled robot."""
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                    ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                    ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                    ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("SubscribeTwist.inputs:topicName", f"/{namespace}/cmd_vel"),
                    ("DiffController.inputs:wheelRadius", wheel_radius),
                    ("DiffController.inputs:wheelDistance", wheel_base),
                    ("ArtController.inputs:jointNames", wheel_joint_names),
                    ("ArtController.inputs:robotPath", robot_prim_path),
                    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_prim_path)]),
                    ("PublishOdom.inputs:topicName", f"/{namespace}/odom"),
                    ("PublishOdom.inputs:chassisFrameId", "base_link"),
                    ("PublishOdom.inputs:odomFrameId", "odom"),
                ],
                keys.CONNECT: [
                    # Twist subscriber
                    ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                    ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                    # Break twist vectors into scalars
                    ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                    ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                    # Differential controller
                    ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                    ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
                    ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
                    # Articulation controller
                    ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),
                    ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
                    # Odometry
                    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                    ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                    ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                    ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                    ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                    ("ROS2Context.outputs:context", "PublishOdom.inputs:context"),
                ],
            },
        )
        print(f"[INFO] Differential drive graph: {graph_path}")
        print(f"       cmd_vel <- /{namespace}/cmd_vel | odom -> /{namespace}/odom")
    except Exception as e:
        print(f"[WARN] Failed to create differential drive graph {graph_path}: {e}")


def create_ros2_joint_command_graph(robot_prim_path: str, namespace: str, graph_path: str):
    """Subscribe to joint commands and apply to a legged robot's articulation."""
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", 47),
                    ("SubscribeJointState.inputs:topicName", f"/{namespace}/joint_commands"),
                    ("ArtController.inputs:robotPath", robot_prim_path),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),
                    ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("SubscribeJointState.outputs:jointNames", "ArtController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand", "ArtController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand", "ArtController.inputs:effortCommand"),
                ],
            },
        )
        print(f"[INFO] Joint command graph: {graph_path} <- /{namespace}/joint_commands")
    except Exception as e:
        print(f"[WARN] Failed to create joint command graph {graph_path}: {e}")


# =============================================================================
# 6b. Body velocity controller (cmd_vel → USD physics velocities)
# =============================================================================

class BodyVelocityController:
    """
    Velocity-based body controller using USD physics API.

    Subscribes to /{namespace}/cmd_vel (geometry_msgs/Twist) and applies
    velocities to the robot's base rigid body each simulation step.

    Twist mapping (robot local frame):
        linear.x  → forward/backward
        linear.y  → left/right (strafe)
        linear.z  → up/down (flying robots only)
        angular.z → yaw rotation

    For ground robots (flying=False), vertical velocity and roll/pitch angular
    velocity are left to the physics engine (gravity + ground contact).
    """

    def __init__(self, body_prim_path: str, namespace: str, flying: bool = False):
        self._body_path = body_prim_path
        self._namespace = namespace
        self._flying = flying
        self._cmd_lin = [0.0, 0.0, 0.0]
        self._cmd_ang_z = 0.0
        self._max_speed = 3.0        # m/s
        self._max_yaw_rate = 1.5     # rad/s

        # USD stage and prim references
        self._stage = omni.usd.get_context().get_stage()
        self._body_prim = self._stage.GetPrimAtPath(self._body_path)

        # Create the ROS 2 twist subscriber OmniGraph
        self._create_twist_subscriber_graph()

        mode = "flying" if flying else "ground"
        print(f"[INFO] BodyVelocityController ({mode}) created for {body_prim_path} "
              f"(cmd_vel: /{namespace}/cmd_vel)", flush=True)

    def _create_twist_subscriber_graph(self):
        """Create OmniGraph that subscribes to cmd_vel and stores the twist."""
        graph_path = f"/World/ROS2/{self._namespace}_twist_graph"
        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:domain_id", 47),
                        ("SubscribeTwist.inputs:topicName", f"/{self._namespace}/cmd_vel"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                        ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                    ],
                },
            )
            self._graph = graph
            # Store reference to the SubscribeTwist node for reading outputs
            for node in nodes:
                if "SubscribeTwist" in node.get_prim_path():
                    self._twist_node = node
                    break
            print(f"[INFO] Twist subscriber: {graph_path} <- "
                  f"/{self._namespace}/cmd_vel", flush=True)
        except Exception as e:
            print(f"[WARN] Failed to create twist graph: {e}", flush=True)
            self._graph = None
            self._twist_node = None

    def update(self):
        """Called each simulation step to apply velocities via USD physics."""
        import math

        if not self._body_prim.IsValid():
            return

        # Read the latest twist command from the OmniGraph node
        if self._twist_node is not None:
            try:
                lin_attr = self._twist_node.get_attribute("outputs:linearVelocity")
                ang_attr = self._twist_node.get_attribute("outputs:angularVelocity")
                if lin_attr is not None:
                    lin = lin_attr.get()
                    if lin is not None and len(lin) >= 3:
                        self._cmd_lin = [
                            max(-self._max_speed, min(self._max_speed, float(lin[0]))),
                            max(-self._max_speed, min(self._max_speed, float(lin[1]))),
                            max(-self._max_speed, min(self._max_speed, float(lin[2]))),
                        ]
                if ang_attr is not None:
                    ang = ang_attr.get()
                    if ang is not None and len(ang) >= 3:
                        self._cmd_ang_z = max(
                            -self._max_yaw_rate,
                            min(self._max_yaw_rate, float(ang[2])),
                        )
            except Exception:
                pass  # Use last known command

        # Get current orientation to transform local → world frame
        xformable = UsdGeom.Xformable(self._body_prim)
        world_xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        rot = world_xform.ExtractRotation()
        quat = rot.GetQuaternion()
        q_real = quat.GetReal()
        q_imag = quat.GetImaginary()
        w, x, y, z = q_real, q_imag[0], q_imag[1], q_imag[2]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Transform local x/y velocity to world frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = self._cmd_lin[0] * cos_yaw - self._cmd_lin[1] * sin_yaw
        vy_world = self._cmd_lin[0] * sin_yaw + self._cmd_lin[1] * cos_yaw

        vel_attr = self._body_prim.GetAttribute("physics:velocity")
        ang_vel_attr = self._body_prim.GetAttribute("physics:angularVelocity")

        if self._flying:
            # Flying: control all 3 axes, zero roll/pitch angular velocity
            vz_world = self._cmd_lin[2]
            if vel_attr.IsValid():
                vel_attr.Set(Gf.Vec3f(vx_world, vy_world, vz_world))
            if ang_vel_attr.IsValid():
                ang_vel_attr.Set(Gf.Vec3f(0.0, 0.0, math.degrees(self._cmd_ang_z)))
        else:
            # Ground: control x/y, preserve physics-computed vertical velocity
            # and roll/pitch angular velocity (gravity + ground contact)
            if vel_attr.IsValid():
                cur_vel = vel_attr.Get()
                vz_phys = float(cur_vel[2]) if cur_vel else 0.0
                vel_attr.Set(Gf.Vec3f(vx_world, vy_world, vz_phys))
            if ang_vel_attr.IsValid():
                cur_ang = ang_vel_attr.Get()
                wx_phys = float(cur_ang[0]) if cur_ang else 0.0
                wy_phys = float(cur_ang[1]) if cur_ang else 0.0
                ang_vel_attr.Set(Gf.Vec3f(wx_phys, wy_phys, math.degrees(self._cmd_ang_z)))


# =============================================================================
# 7. Per-robot sensor & control orchestrator
# =============================================================================

def setup_robot_sensors_and_control(robot_name: str, robot_config: dict):
    """Create all sensors and ROS 2 graphs for a single robot.
    Returns a BodyVelocityController for robots that support cmd_vel, else None."""
    ns = robot_config["namespace"]
    prim = robot_config["prim_path"]
    art_root = robot_config.get("articulation_root", prim)

    print(f"\n[INFO] Setting up sensors for '{robot_name}' (namespace: {ns}, articulation: {art_root})")

    # --- Joint States Publisher ---
    create_ros2_joint_state_graph(
        robot_prim_path=art_root,
        namespace=ns,
        graph_path=f"/World/ROS2/{ns}_joint_state_graph",
    )

    # --- IMU Sensor + Publisher ---
    imu_cfg = robot_config.get("imu")
    if imu_cfg:
        imu_prim_path = create_imu_sensor(
            parent_prim_path=imu_cfg["parent_prim"],
            sensor_name=f"{ns}_imu_sensor",
        )
        if imu_prim_path:
            create_ros2_imu_graph(
                imu_prim_path=imu_prim_path,
                namespace=ns,
                frame_id=imu_cfg["frame_id"],
                graph_path=f"/World/ROS2/{ns}_imu_graph",
            )

    # --- Camera (RGB + Depth) ---
    cam_cfg = robot_config.get("camera")
    if cam_cfg:
        cam_prim_path = create_robot_camera(
            parent_prim_path=f"{prim}/{cam_cfg['parent_link']}",
            camera_name=f"{ns}_front_cam",
            local_position=cam_cfg["local_position"],
            focal_length=cam_cfg.get("focal_length", 24.0),
        )
        create_ros2_camera_graph(
            camera_prim_path=cam_prim_path,
            topic_prefix=f"/{ns}/front_cam",
            graph_path=f"/World/ROS2/{ns}_front_cam_graph",
        )

    # --- 3D LiDAR ---
    lidar_3d_cfg = robot_config.get("lidar_3d")
    if lidar_3d_cfg:
        lidar_prim_path = create_rtx_lidar_sensor(
            parent_prim_path=lidar_3d_cfg["parent_prim"],
            sensor_name=f"{ns}_lidar_3d",
            config=lidar_3d_cfg["config"],
            translation=lidar_3d_cfg.get("local_position", (0, 0, 0)),
        )
        if lidar_prim_path:
            create_ros2_rtx_lidar_pipeline(
                sensor_prim_path=lidar_prim_path,
                namespace=ns,
                topic_suffix="lidar",
                frame_id=lidar_3d_cfg["frame_id"],
            )

    # --- 2D LiDAR (Nova Carter only) ---
    lidar_2d_cfg = robot_config.get("lidar_2d")
    if lidar_2d_cfg:
        lidar_2d_path = create_rtx_lidar_sensor(
            parent_prim_path=lidar_2d_cfg["parent_prim"],
            sensor_name=f"{ns}_lidar_2d",
            config=lidar_2d_cfg["config"],
            translation=lidar_2d_cfg.get("local_position", (0, 0, 0)),
        )
        if lidar_2d_path:
            create_ros2_rtx_lidar_pipeline(
                sensor_prim_path=lidar_2d_path,
                namespace=ns,
                topic_suffix="lidar_2d",
                frame_id=lidar_2d_cfg["frame_id"],
                publish_laser_scan=True,
            )

    # --- Control (type-dependent) ---
    robot_type = robot_config.get("type", "legged")
    if robot_type == "wheeled":
        drive_cfg = robot_config["drive"]
        create_ros2_differential_drive_graph(
            robot_prim_path=art_root,
            namespace=ns,
            wheel_radius=drive_cfg["wheel_radius"],
            wheel_base=drive_cfg["wheel_base"],
            wheel_joint_names=drive_cfg["wheel_joint_names"],
            graph_path=f"/World/ROS2/{ns}_drive_graph",
        )
    elif robot_type == "multirotor":
        body_prim = f"{prim}/chassis"
        return BodyVelocityController(body_prim_path=body_prim, namespace=ns, flying=True)
    elif robot_type == "floating_camera":
        base_link = robot_config.get("base_link", "body")
        body_prim = f"{prim}/{base_link}"
        return BodyVelocityController(body_prim_path=body_prim, namespace=ns, flying=True)
    else:  # legged
        # Joint-level control (for precise joint commands)
        create_ros2_joint_command_graph(
            robot_prim_path=art_root,
            namespace=ns,
            graph_path=f"/World/ROS2/{ns}_joint_cmd_graph",
        )
        # Body velocity control via cmd_vel (for joystick teleop)
        base_link = robot_config.get("base_link")
        if base_link:
            body_prim = f"{prim}/{base_link}"
            return BodyVelocityController(body_prim_path=body_prim, namespace=ns, flying=False)
    return None


# =============================================================================
# 8. Main simulation setup
# =============================================================================

def main():
    print("=" * 60)
    print("  Warehouse Multi-Robot Simulation")
    print("=" * 60)

    # Create the World
    world = World(stage_units_in_meters=1.0)

    # --- Load environment ---
    load_warehouse(args.warehouse)
    simulation_app.update()

    # --- Spawn robots ---
    define_prim("/World/Robots", "Xform")
    for robot_name, robot_config in ROBOTS.items():
        spawn_robot(robot_name, robot_config)
    simulation_app.update()

    # --- Create fixed surveillance cameras ---
    create_fixed_cameras()
    simulation_app.update()

    # --- Set up ROS 2 OmniGraphs ---
    define_prim("/World/ROS2", "Xform")

    # Ensure publishing even without subscribers (useful when containers start later)
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/publish_without_verification", True
    )

    # Clock publisher (essential for sim_time)
    create_ros2_clock_graph()

    # For each robot, publish TF
    for robot_name, robot_config in ROBOTS.items():
        ns = robot_config["namespace"]
        prim = robot_config["prim_path"]

        # TF tree
        create_ros2_tf_graph(
            robot_prim_path=prim,
            namespace=ns,
            graph_path=f"/World/ROS2/{ns}_tf_graph",
        )

    # --- Robot sensor feeds and control ---
    velocity_controllers = []
    for robot_name, robot_config in ROBOTS.items():
        ctrl = setup_robot_sensors_and_control(robot_name, robot_config)
        if ctrl is not None:
            velocity_controllers.append(ctrl)
        simulation_app.update()

    # Publish fixed camera feeds over ROS 2
    for cam_cfg in FIXED_CAMERAS:
        cam_name = cam_cfg["name"]
        create_ros2_camera_graph(
            camera_prim_path=cam_cfg["prim_path"],
            topic_prefix=f"/{cam_name}",
            graph_path=f"/World/ROS2/{cam_name}_graph",
        )

    simulation_app.update()

    # --- Print expected topic list ---
    print("\n" + "=" * 60)
    print("  Expected ROS 2 Topics")
    print("=" * 60)
    print("  /clock")
    print("  /tf")
    for robot_name, robot_config in ROBOTS.items():
        ns = robot_config["namespace"]
        rtype = robot_config.get("type", "legged")
        print(f"  /{ns}/imu")
        print(f"  /{ns}/front_cam/{{rgb,depth,camera_info}}")
        if rtype == "wheeled":
            print(f"  /{ns}/joint_states")
            print(f"  /{ns}/lidar/point_cloud")
            print(f"  /{ns}/cmd_vel  (subscribe)")
            print(f"  /{ns}/odom")
            if "lidar_2d" in robot_config:
                print(f"  /{ns}/lidar_2d/{{point_cloud,scan}}")
        elif rtype == "multirotor":
            print(f"  /{ns}/cmd_vel  (subscribe - fly the drone)")
        elif rtype == "floating_camera":
            print(f"  /{ns}/front_cam/{{rgb,depth,camera_info}}")
            print(f"  /{ns}/cmd_vel  (subscribe - fly the camera)")
        else:
            print(f"  /{ns}/joint_states")
            print(f"  /{ns}/lidar/point_cloud")
            print(f"  /{ns}/joint_commands  (subscribe)")
            print(f"  /{ns}/cmd_vel  (subscribe - body velocity teleop)")
    for cam_cfg in FIXED_CAMERAS:
        print(f"  /{cam_cfg['name']}/{{rgb,depth,camera_info}}")
    print("=" * 60)
    print("[INFO] To verify: ros2 topic list")

    # --- Start simulation via timeline (drives physics + OmniGraph nodes) ---
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()

    # Initialize physics by resetting the world, then play the timeline
    world.reset()
    simulation_app.update()

    # Play the timeline — this is what drives OnPlaybackTick OmniGraph nodes
    timeline.play()

    # Warm-up frames to let physics and rendering stabilize
    for _ in range(20):
        simulation_app.update()

    print("[INFO] Simulation is running. Press Ctrl+C to stop.", flush=True)
    print(f"[INFO] Timeline playing: {timeline.is_playing()}", flush=True)

    # --- Simulation loop ---
    # Use simulation_app.update() instead of world.step() so the timeline
    # clock advances and OnPlaybackTick OmniGraph nodes fire correctly.
    try:
        while simulation_app.is_running():
            simulation_app.update()
            # Update velocity controllers (apply cmd_vel to robot bodies)
            for dc in velocity_controllers:
                dc.update()
    except KeyboardInterrupt:
        print("\n[INFO] Simulation stopped by user.", flush=True)
    finally:
        timeline.stop()
        simulation_app.close()
        print("[INFO] Isaac Sim closed.")


if __name__ == "__main__":
    main()