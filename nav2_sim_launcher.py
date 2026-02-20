#!/usr/bin/env python3
"""
Nav2 Simulation Launcher
=========================
Spawns the warehouse environment with a single robot (configured via nav2.robot
in config.yaml) and publishes ground truth odometry over ROS 2 for the Nav2
stack running in the Docker container.

Differences from spawn_warehouse.py:
  - Only spawns ONE robot (the nav2 target)
  - No fixed surveillance cameras
  - Adds ground truth odometry graph (IsaacComputeOdometry → ROS2PublishOdometry)
  - All OmniGraph domain_id values read from config

Usage:
    cd ~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /path/to/nav2_sim_launcher.py --config /path/to/config.yaml [--headless]
"""

import argparse
import sys
import yaml
import numpy as np

# =============================================================================
# 1. Parse arguments BEFORE launching SimulationApp
# =============================================================================
parser = argparse.ArgumentParser(description="Nav2 Simulation Launcher")
parser.add_argument("--headless", action="store_true", help="Run headless (no GUI)")
parser.add_argument(
    "--config",
    type=str,
    required=True,
    help="Path to config.yaml",
)
args = parser.parse_args()

# Load unified config
with open(args.config, "r") as f:
    CFG = yaml.safe_load(f)

# Nav2-specific config
NAV2_CFG = CFG.get("nav2", {})
NAV2_ROBOT_KEY = NAV2_CFG.get("robot", "nova_carter")
DOMAIN_ID = NAV2_CFG.get("domain_id", 47)

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
# 4. Configuration
# =============================================================================

# Asset root path
ASSETS_ROOT = get_assets_root_path()
if ASSETS_ROOT is None:
    ASSETS_ROOT = "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1"
    print(f"[WARN] get_assets_root_path() returned None. Using local fallback: {ASSETS_ROOT}")
else:
    print(f"[INFO] Assets root: {ASSETS_ROOT}")

# Warehouse environment USD paths
WAREHOUSE_PATHS = CFG["scene"].get("warehouse_paths", {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "full_warehouse": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "warehouse_with_forklifts": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "warehouse_multiple_shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
})

# Robot config — only the Nav2 target robot
if NAV2_ROBOT_KEY not in CFG["robots"]:
    print(f"[ERROR] Robot '{NAV2_ROBOT_KEY}' not found in config.yaml robots section")
    print(f"        Available robots: {list(CFG['robots'].keys())}")
    simulation_app.close()
    sys.exit(1)

ROBOT_CONFIG = CFG["robots"][NAV2_ROBOT_KEY]


# =============================================================================
# 5. Helper functions
# =============================================================================

def load_warehouse(warehouse_type: str):
    """Load the warehouse environment USD onto the stage."""
    usd_path = ASSETS_ROOT + WAREHOUSE_PATHS[warehouse_type]
    print(f"[INFO] Loading warehouse: {usd_path}")
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/Warehouse")
    print(f"[INFO] Warehouse '{warehouse_type}' loaded.")


def spawn_objects():
    """Spawn scene objects (pallets, bins, etc.) from config.yaml scene.objects."""
    objects = CFG.get("scene", {}).get("objects", [])
    if not objects:
        print("[INFO] No scene objects to spawn.")
        return

    define_prim("/World/Objects", "Xform")
    stage = omni.usd.get_context().get_stage()

    for obj in objects:
        prim_name = obj["prim_name"]
        prim_path = f"/World/Objects/{prim_name}"
        usd_path = ASSETS_ROOT + obj["usd_path"]
        pos = obj.get("position", [0, 0, 0])
        rot = obj.get("orientation", [0, 0, 0])

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        obj_prim = stage.GetPrimAtPath(prim_path)
        if obj_prim.IsValid():
            xformable = UsdGeom.Xformable(obj_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], pos[2]))
            xformable.AddRotateXYZOp().Set(Gf.Vec3f(rot[0], rot[1], rot[2]))

    print(f"[INFO] Spawned {len(objects)} scene objects.")


def find_articulation_root(robot_prim_path: str) -> str:
    """Find the prim with ArticulationRootAPI in the robot's hierarchy."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(robot_prim_path)

    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return robot_prim_path

    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim.GetPath().pathString

    print(f"[WARN] No ArticulationRootAPI found under {robot_prim_path}, using root path")
    return robot_prim_path


def spawn_robot(name: str, config: dict):
    """Spawn a robot from USD."""
    prim_path = config["prim_path"]
    pos = config["position"]
    rot = config["orientation"]
    stage = omni.usd.get_context().get_stage()

    usd_path = ASSETS_ROOT + config["usd_path"]
    print(f"[INFO] Spawning robot '{name}' at position {pos}...")
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    robot_prim = stage.GetPrimAtPath(prim_path)
    if robot_prim.IsValid():
        xformable = UsdGeom.Xformable(robot_prim)
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(pos[0], pos[1], pos[2]))
        rotate_op = xformable.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(rot[0], rot[1], rot[2]))

        art_root = find_articulation_root(prim_path)
        config["articulation_root"] = art_root
        print(f"[INFO] Robot '{name}' spawned at {prim_path} (articulation root: {art_root})")
    else:
        print(f"[ERROR] Failed to spawn robot '{name}' — prim not valid at {prim_path}")


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
        sensor_period=-1.0,
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
# 6. ROS 2 OmniGraph setup
# =============================================================================

def create_ros2_clock_graph(domain_id: int):
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
                    ("ROS2Context.inputs:domain_id", domain_id),
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


def create_ros2_tf_graph(robot_prim_path: str, namespace: str,
                         graph_path: str, domain_id: int):
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
                    ("ROS2Context.inputs:domain_id", domain_id),
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


def create_ros2_camera_graph(camera_prim_path: str, topic_prefix: str,
                             graph_path: str, domain_id: int):
    """Create an OmniGraph that publishes RGB, depth, and camera_info."""
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
                    ("ROS2Context.inputs:domain_id", domain_id),
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


def create_ros2_imu_graph(imu_prim_path: str, namespace: str,
                          frame_id: str, graph_path: str, domain_id: int):
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
                    ("ROS2Context.inputs:domain_id", domain_id),
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


def create_ros2_joint_state_graph(robot_prim_path: str, namespace: str,
                                  graph_path: str, domain_id: int):
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
                    ("ROS2Context.inputs:domain_id", domain_id),
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


def create_ros2_joint_command_graph(robot_prim_path: str, namespace: str,
                                    graph_path: str, domain_id: int):
    """Subscribe to joint commands and apply to a robot's articulation."""
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
                    ("ROS2Context.inputs:domain_id", domain_id),
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


def create_ground_truth_odom_graph(robot_prim_path: str, namespace: str,
                                   base_frame: str, odom_topic: str,
                                   graph_path: str, domain_id: int):
    """
    Create an OmniGraph that computes ground truth odometry from the robot's
    USD prim transform and publishes it as nav_msgs/Odometry + odom→base TF.

    Uses IsaacComputeOdometry → ROS2PublishOdometry.
    """
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ],
                keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_prim_path)]),
                    ("PublishOdom.inputs:topicName", odom_topic),
                    ("PublishOdom.inputs:chassisFrameId", base_frame),
                    ("PublishOdom.inputs:odomFrameId", "odom"),
                ],
                keys.CONNECT: [
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
        print(f"[INFO] Ground truth odometry graph: {graph_path}")
        print(f"       odom → {odom_topic} | TF: odom → {base_frame}")
    except Exception as e:
        print(f"[WARN] Failed to create ground truth odom graph {graph_path}: {e}")


# =============================================================================
# 6b. Differential drive controller (for wheeled robots like Nova Carter)
# =============================================================================

def create_ros2_differential_drive_graph(
    robot_prim_path: str, namespace: str, base_frame: str,
    wheel_radius: float, wheel_base: float,
    wheel_joint_names: list, graph_path: str, domain_id: int
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
                    ("ROS2Context.inputs:domain_id", domain_id),
                    ("SubscribeTwist.inputs:topicName", f"/{namespace}/cmd_vel"),
                    ("DiffController.inputs:wheelRadius", wheel_radius),
                    ("DiffController.inputs:wheelDistance", wheel_base),
                    ("ArtController.inputs:jointNames", wheel_joint_names),
                    ("ArtController.inputs:robotPath", robot_prim_path),
                    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_prim_path)]),
                    ("PublishOdom.inputs:topicName", f"/{namespace}/odom"),
                    ("PublishOdom.inputs:chassisFrameId", base_frame),
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


# =============================================================================
# 6c. Body velocity controller (cmd_vel → USD physics velocities, for legged)
# =============================================================================

class BodyVelocityController:
    """
    Velocity-based body controller using USD physics API.

    Subscribes to /{namespace}/cmd_vel (geometry_msgs/Twist) and applies
    velocities to the robot's base rigid body each simulation step.
    """

    def __init__(self, body_prim_path: str, namespace: str,
                 flying: bool = False, domain_id: int = 47):
        self._body_path = body_prim_path
        self._namespace = namespace
        self._flying = flying
        self._domain_id = domain_id
        self._cmd_lin = [0.0, 0.0, 0.0]
        self._cmd_ang_z = 0.0
        self._max_speed = 3.0
        self._max_yaw_rate = 1.5

        self._stage = omni.usd.get_context().get_stage()
        self._body_prim = self._stage.GetPrimAtPath(self._body_path)

        self._create_twist_subscriber_graph()

        mode = "flying" if flying else "ground"
        print(f"[INFO] BodyVelocityController ({mode}) created for {body_prim_path} "
              f"(cmd_vel: /{namespace}/cmd_vel, domain_id: {domain_id})", flush=True)

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
                        ("ROS2Context.inputs:domain_id", self._domain_id),
                        ("SubscribeTwist.inputs:topicName", f"/{self._namespace}/cmd_vel"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                        ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                    ],
                },
            )
            self._graph = graph
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
                pass

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

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = self._cmd_lin[0] * cos_yaw - self._cmd_lin[1] * sin_yaw
        vy_world = self._cmd_lin[0] * sin_yaw + self._cmd_lin[1] * cos_yaw

        vel_attr = self._body_prim.GetAttribute("physics:velocity")
        ang_vel_attr = self._body_prim.GetAttribute("physics:angularVelocity")

        if self._flying:
            vz_world = self._cmd_lin[2]
            if vel_attr.IsValid():
                vel_attr.Set(Gf.Vec3f(vx_world, vy_world, vz_world))
            if ang_vel_attr.IsValid():
                ang_vel_attr.Set(Gf.Vec3f(0.0, 0.0, math.degrees(self._cmd_ang_z)))
        else:
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
# 7. Nav2 robot setup orchestrator
# =============================================================================

def setup_nav2_robot(robot_name: str, robot_config: dict, domain_id: int):
    """Spawn the Nav2 target robot with all sensors + ground truth odometry.

    Automatically handles wheeled (differential drive) and legged (body
    velocity controller) robot types based on robot_config['type'].

    Returns a BodyVelocityController for legged robots, or None for wheeled.
    """
    ns = robot_config["namespace"]
    prim = robot_config["prim_path"]
    robot_type = robot_config.get("type", "legged")

    # Spawn the robot
    spawn_robot(robot_name, robot_config)
    simulation_app.update()

    art_root = robot_config.get("articulation_root", prim)
    base_link = robot_config.get("base_link", "base_link")
    gt_odom_topic = f"/{ns}/ground_truth/odom"

    print(f"\n[INFO] Setting up Nav2 sensors for '{robot_name}' "
          f"(namespace: {ns}, base: {base_link}, type: {robot_type})")

    # --- TF tree ---
    # NOTE: Disabled — ROS2PublishTransformTree publishes with timestamp 0 and
    # creates conflicting frames (world -> NovaCarter -> chassis_link) that
    # clash with the odom -> chassis_link chain from the localization bridge.
    # Sensor frame static transforms are handled by the ROS2 launch file instead.
    # create_ros2_tf_graph(
    #     robot_prim_path=prim,
    #     namespace=ns,
    #     graph_path=f"/World/ROS2/{ns}_tf_graph",
    #     domain_id=domain_id,
    # )

    # --- Joint states ---
    create_ros2_joint_state_graph(
        robot_prim_path=art_root,
        namespace=ns,
        graph_path=f"/World/ROS2/{ns}_joint_state_graph",
        domain_id=domain_id,
    )

    # --- IMU ---
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
                domain_id=domain_id,
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
            domain_id=domain_id,
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

    # --- 2D LiDAR (wheeled robots like Nova Carter) ---
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

    # --- Ground truth odometry (the key Nav2 addition) ---
    # Must use art_root (not the top-level prim) — IsaacComputeOdometry
    # requires a valid rigid body or articulation root.
    create_ground_truth_odom_graph(
        robot_prim_path=art_root,
        namespace=ns,
        base_frame=base_link,
        odom_topic=gt_odom_topic,
        graph_path=f"/World/ROS2/{ns}_gt_odom_graph",
        domain_id=domain_id,
    )

    # --- Control (type-dependent) ---
    if robot_type == "wheeled":
        # Differential drive: subscribes to cmd_vel, drives wheel joints,
        # publishes wheel odometry on /{ns}/odom
        drive_cfg = robot_config.get("drive")
        if drive_cfg:
            create_ros2_differential_drive_graph(
                robot_prim_path=art_root,
                namespace=ns,
                base_frame=base_link,
                wheel_radius=drive_cfg["wheel_radius"],
                wheel_base=drive_cfg["wheel_base"],
                wheel_joint_names=drive_cfg["wheel_joint_names"],
                graph_path=f"/World/ROS2/{ns}_drive_graph",
                domain_id=domain_id,
            )
        else:
            print(f"[WARN] Wheeled robot '{robot_name}' has no 'drive' config — "
                  "no cmd_vel control will be available")
        return None  # no controller.update() needed

    else:  # legged
        # Joint-level control
        create_ros2_joint_command_graph(
            robot_prim_path=art_root,
            namespace=ns,
            graph_path=f"/World/ROS2/{ns}_joint_cmd_graph",
            domain_id=domain_id,
        )
        # Body velocity control via cmd_vel
        body_prim = f"{prim}/{base_link}"
        controller = BodyVelocityController(
            body_prim_path=body_prim,
            namespace=ns,
            flying=False,
            domain_id=domain_id,
        )
        return controller


# =============================================================================
# 8. Main simulation setup
# =============================================================================

def main():
    ns = ROBOT_CONFIG["namespace"]
    base_link = ROBOT_CONFIG.get("base_link", "base_link")
    robot_type = ROBOT_CONFIG.get("type", "legged")

    print("=" * 60)
    print("  Nav2 Simulation Launcher")
    print(f"  Robot: {NAV2_ROBOT_KEY} ({robot_type})")
    print(f"  Namespace: {ns}")
    print(f"  Domain ID: {DOMAIN_ID}")
    print("=" * 60)

    # Create the World
    world = World(stage_units_in_meters=1.0)

    # --- Load environment ---
    load_warehouse(CFG["scene"]["warehouse"])
    simulation_app.update()

    # --- Spawn scene objects (pallets, bins, etc.) ---
    spawn_objects()
    simulation_app.update()

    # --- Spawn the Nav2 robot ---
    define_prim("/World/Robots", "Xform")
    define_prim("/World/ROS2", "Xform")

    # Ensure publishing even without subscribers
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/publish_without_verification", True
    )

    # Clock publisher
    create_ros2_clock_graph(domain_id=DOMAIN_ID)

    # Setup the single Nav2 robot with all sensors + ground truth
    controller = setup_nav2_robot(
        robot_name=NAV2_ROBOT_KEY,
        robot_config=ROBOT_CONFIG,
        domain_id=DOMAIN_ID,
    )

    simulation_app.update()

    # --- Print expected topic list ---
    print("\n" + "=" * 60)
    print("  Expected ROS 2 Topics")
    print("=" * 60)
    print("  /clock")
    print("  /tf")
    print(f"  /{ns}/joint_states")
    if ROBOT_CONFIG.get("imu"):
        print(f"  /{ns}/imu")
    if ROBOT_CONFIG.get("camera"):
        print(f"  /{ns}/front_cam/{{rgb,depth,camera_info}}")
    if ROBOT_CONFIG.get("lidar_3d"):
        print(f"  /{ns}/lidar/point_cloud")
    if ROBOT_CONFIG.get("lidar_2d"):
        print(f"  /{ns}/lidar_2d/{{point_cloud,scan}}")
    print(f"  /{ns}/ground_truth/odom            (ground truth odometry)")
    if robot_type == "wheeled":
        print(f"  /{ns}/odom                          (wheel odometry)")
        print(f"  /{ns}/cmd_vel                       (subscribe — differential drive)")
    else:
        print(f"  /{ns}/cmd_vel                       (subscribe — body velocity)")
        print(f"  /{ns}/joint_commands                (subscribe — joint control)")
    print("=" * 60)
    print(f"[INFO] TF: odom -> {base_link} (from ground truth odom graph)")
    print(f"[INFO] The localization_bridge_node in the container will compute map -> odom")
    print("[INFO] To verify: ros2 topic list")

    # --- Start simulation ---
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()

    world.reset()
    simulation_app.update()
    timeline.play()

    # Warm-up frames
    for _ in range(20):
        simulation_app.update()

    print("[INFO] Simulation is running. Press Ctrl+C to stop.", flush=True)
    print(f"[INFO] Timeline playing: {timeline.is_playing()}", flush=True)

    # --- Simulation loop ---
    try:
        while simulation_app.is_running():
            simulation_app.update()
            if controller is not None:
                controller.update()
    except KeyboardInterrupt:
        print("\n[INFO] Simulation stopped by user.", flush=True)
    finally:
        timeline.stop()
        simulation_app.close()
        print("[INFO] Isaac Sim closed.")


if __name__ == "__main__":
    main()
