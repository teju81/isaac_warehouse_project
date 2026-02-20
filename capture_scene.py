#!/usr/bin/env python3
"""
Capture Scene — RGBD Recording with Teleoperable Camera
========================================================
Spawns a warehouse scene with objects and a free-flying explorer camera.
The camera is teleoperated via ROS 2 cmd_vel (e.g. joystick from a container).
RGBD frames + camera-to-world poses are recorded to disk in a format compatible
with the Perception Super App.

Output structure:
    <output_dir>/
        color/          frame_0.png, frame_1.png, ...   (uint8 BGR)
        depth/          frame_0.npy, frame_0.png, ...   (float32 meters / uint16 mm)
        poses.json      {"poses": {"0": [[4x4 c2w]], ...}, "intrinsics": {...}}

Usage:
    cd ~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /path/to/capture_scene.py --config /path/to/config.yaml
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import yaml

# =============================================================================
# 1. Parse arguments BEFORE launching SimulationApp
# =============================================================================
parser = argparse.ArgumentParser(description="Capture Scene — RGBD + Pose Recorder")
parser.add_argument("--config", type=str, required=True, help="Path to YAML config file")
parser.add_argument("--headless", action="store_true", help="Run headless (no GUI)")
args = parser.parse_args()

with open(args.config, "r") as f:
    cfg = yaml.safe_load(f)

# =============================================================================
# 2. Launch Isaac Sim
# =============================================================================
from isaacsim import SimulationApp

SIM_CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": args.headless,
    "width": cfg["camera"]["width"],
    "height": cfg["camera"]["height"],
}

simulation_app = SimulationApp(SIM_CONFIG)
print("[INFO] Isaac Sim application started.")

# =============================================================================
# 3. Omniverse imports (MUST come after SimulationApp)
# =============================================================================
import carb
import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import define_prim
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
print("[INFO] ROS 2 Bridge extension enabled.")

# =============================================================================
# 4. Asset root
# =============================================================================
ASSETS_ROOT = get_assets_root_path()
if ASSETS_ROOT is None:
    ASSETS_ROOT = "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1"
    print(f"[WARN] Using local asset fallback: {ASSETS_ROOT}")
else:
    print(f"[INFO] Assets root: {ASSETS_ROOT}")

WAREHOUSE_PATHS = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "full_warehouse": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "warehouse_with_forklifts": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "warehouse_multiple_shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
}


# =============================================================================
# 5. Scene creation
# =============================================================================

def load_warehouse(warehouse_type: str):
    rel_path = WAREHOUSE_PATHS.get(warehouse_type)
    if rel_path is None:
        print(f"[ERROR] Unknown warehouse type: {warehouse_type}")
        sys.exit(1)
    usd_path = ASSETS_ROOT + rel_path
    print(f"[INFO] Loading warehouse: {usd_path}")
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/Warehouse")
    print(f"[INFO] Warehouse '{warehouse_type}' loaded.")


def spawn_objects(objects_cfg: list):
    """Spawn all scene objects from the config."""
    define_prim("/World/Objects", "Xform")
    stage = omni.usd.get_context().get_stage()

    for i, obj in enumerate(objects_cfg):
        usd_path = ASSETS_ROOT + obj["usd_path"]
        prim_name = obj.get("prim_name", f"object_{i:03d}")
        prim_path = f"/World/Objects/{prim_name}"
        pos = obj.get("position", [0, 0, 0])
        rot = obj.get("orientation", [0, 0, 0])

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            xformable = UsdGeom.Xformable(prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], pos[2]))
            xformable.AddRotateXYZOp().Set(Gf.Vec3f(rot[0], rot[1], rot[2]))
            print(f"[INFO] Spawned '{prim_name}' at {pos}")
        else:
            print(f"[WARN] Failed to spawn '{prim_name}' from {obj['usd_path']}")

    print(f"[INFO] Spawned {len(objects_cfg)} objects.")


# =============================================================================
# 6. Explorer camera (invisible rigid body + camera prim)
# =============================================================================

def create_explorer_camera(cam_cfg: dict, teleop_cfg: dict):
    """Create a free-flying invisible rigid body with a camera attached."""
    from pxr import PhysxSchema

    stage = omni.usd.get_context().get_stage()
    prim_path = "/World/ExplorerCam"
    body_path = f"{prim_path}/body"
    camera_path = f"{body_path}/camera"

    # Root xform
    define_prim(prim_path, "Xform")
    ip = cam_cfg.get("initial_pose", {})
    xformable = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(
        ip.get("tx", 0), ip.get("ty", 0), ip.get("tz", 3)))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(
        ip.get("roll", 0), ip.get("pitch", 0), ip.get("yaw", 0)))

    # Rigid body (no collision, no visual)
    define_prim(body_path, "Xform")
    body_prim = stage.GetPrimAtPath(body_path)
    UsdPhysics.RigidBodyAPI.Apply(body_prim)
    mass_api = UsdPhysics.MassAPI.Apply(body_prim)
    mass_api.GetMassAttr().Set(1.0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
    PhysxSchema.PhysxRigidBodyAPI(body_prim).GetDisableGravityAttr().Set(True)

    # Camera prim
    camera_prim = UsdGeom.Camera.Define(stage, camera_path)
    camera_prim.GetFocalLengthAttr().Set(cam_cfg.get("focal_length", 18.0))
    camera_prim.GetHorizontalApertureAttr().Set(cam_cfg.get("horizontal_aperture", 20.955))
    camera_prim.GetVerticalApertureAttr().Set(cam_cfg.get("vertical_aperture", 15.2908))
    camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(
        cam_cfg.get("near_plane", 0.01), cam_cfg.get("far_plane", 100.0)))

    print(f"[INFO] Explorer camera created at {camera_path}")
    return prim_path, body_path, camera_path


# =============================================================================
# 7. Body velocity controller (cmd_vel teleop)
# =============================================================================

class BodyVelocityController:
    """Subscribe to cmd_vel and apply velocities to a rigid body via USD physics."""

    def __init__(self, body_prim_path: str, namespace: str, domain_id: int,
                 max_speed: float = 3.0, max_yaw_rate: float = 1.5):
        self._body_path = body_prim_path
        self._namespace = namespace
        self._cmd_lin = [0.0, 0.0, 0.0]
        self._cmd_ang_z = 0.0
        self._max_speed = max_speed
        self._max_yaw_rate = max_yaw_rate

        self._stage = omni.usd.get_context().get_stage()
        self._body_prim = self._stage.GetPrimAtPath(self._body_path)

        # OmniGraph twist subscriber
        graph_path = f"/World/ROS2/{namespace}_twist_graph"
        try:
            keys = og.Controller.Keys
            (_, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:domain_id", domain_id),
                        ("SubscribeTwist.inputs:topicName", f"/{namespace}/cmd_vel"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                        ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                    ],
                },
            )
            self._twist_node = None
            for node in nodes:
                if "SubscribeTwist" in node.get_prim_path():
                    self._twist_node = node
                    break
            print(f"[INFO] Twist subscriber: {graph_path} <- /{namespace}/cmd_vel "
                  f"(domain_id={domain_id})")
        except Exception as e:
            print(f"[WARN] Failed to create twist graph: {e}")
            self._twist_node = None

    def update(self):
        if not self._body_prim.IsValid():
            return

        # Read twist from OmniGraph
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
                            min(self._max_yaw_rate, float(ang[2])))
            except Exception:
                pass

        # Get yaw from current orientation
        xformable = UsdGeom.Xformable(self._body_prim)
        world_xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        rot = world_xform.ExtractRotation()
        quat = rot.GetQuaternion()
        q_real = quat.GetReal()
        q_imag = quat.GetImaginary()
        w, x, y, z = q_real, q_imag[0], q_imag[1], q_imag[2]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = self._cmd_lin[0] * cos_yaw - self._cmd_lin[1] * sin_yaw
        vy_world = self._cmd_lin[0] * sin_yaw + self._cmd_lin[1] * cos_yaw
        vz_world = self._cmd_lin[2]

        vel_attr = self._body_prim.GetAttribute("physics:velocity")
        ang_vel_attr = self._body_prim.GetAttribute("physics:angularVelocity")

        if vel_attr.IsValid():
            vel_attr.Set(Gf.Vec3f(vx_world, vy_world, vz_world))
        if ang_vel_attr.IsValid():
            ang_vel_attr.Set(Gf.Vec3f(0.0, 0.0, math.degrees(self._cmd_ang_z)))


# =============================================================================
# 8. Posed image recorder
# =============================================================================

class PosedImageRecorder:
    """
    Captures RGBD frames from a camera and saves them with camera-to-world poses.

    Output format matches the Perception Super App:
        color/frame_0.png           uint8 BGR
        depth/frame_0.npy           float32 meters
        depth/frame_0.png           uint16 millimeters (optional)
        poses.json                  {"poses": {"0": [[4x4]], ...}, "intrinsics": {...}}
    """

    def __init__(self, camera_prim_path: str, cam_cfg: dict, rec_cfg: dict):
        self._camera_path = camera_prim_path
        self._cam_cfg = cam_cfg
        self._rec_cfg = rec_cfg
        self._frame_idx = 0
        self._poses = {}
        self._last_capture_time = 0.0
        self._capture_interval = 1.0 / rec_cfg.get("capture_rate", 5.0)

        # Output directories
        output_dir = rec_cfg["output_dir"]
        self._color_dir = os.path.join(output_dir, rec_cfg.get("subdir_color", "color"))
        self._depth_dir = os.path.join(output_dir, rec_cfg.get("subdir_depth", "depth"))
        os.makedirs(self._color_dir, exist_ok=True)
        os.makedirs(self._depth_dir, exist_ok=True)

        self._depth_format = rec_cfg.get("depth_format", "both")
        self._depth_scale = rec_cfg.get("depth_scale", 1000)
        self._poses_file = os.path.join(output_dir, "poses.json")

        # Compute intrinsics
        self._intrinsics = self._compute_intrinsics()

        # Set up Replicator render product + annotators
        import omni.replicator.core as rep

        width = cam_cfg["width"]
        height = cam_cfg["height"]
        self._rp = rep.create.render_product(camera_prim_path, (width, height))

        self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self._rgb_annot.attach([self._rp])

        self._depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self._depth_annot.attach([self._rp])

        self._stage = omni.usd.get_context().get_stage()
        self._camera_prim = self._stage.GetPrimAtPath(camera_prim_path)

        print(f"[INFO] Recorder initialized: {output_dir}")
        print(f"       Resolution: {width}x{height}, capture rate: {rec_cfg.get('capture_rate', 5.0)} Hz")
        print(f"       Intrinsics: fx={self._intrinsics['fx']:.1f}, fy={self._intrinsics['fy']:.1f}, "
              f"cx={self._intrinsics['cx']:.1f}, cy={self._intrinsics['cy']:.1f}")

    def _compute_intrinsics(self):
        """Compute camera intrinsics from config (fx/fy or focal_length + aperture)."""
        w = self._cam_cfg["width"]
        h = self._cam_cfg["height"]
        fx = self._cam_cfg.get("fx")
        fy = self._cam_cfg.get("fy")
        cx = self._cam_cfg.get("cx")
        cy = self._cam_cfg.get("cy")

        if not fx or not fy:
            focal_mm = self._cam_cfg.get("focal_length", 18.0)
            h_aperture = self._cam_cfg.get("horizontal_aperture", 20.955)
            v_aperture = self._cam_cfg.get("vertical_aperture", 15.2908)
            fx = focal_mm * w / h_aperture
            fy = focal_mm * h / v_aperture
        if not cx:
            cx = w / 2.0
        if not cy:
            cy = h / 2.0

        return {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "width": w, "height": h}

    def _get_camera_c2w(self):
        """Read camera-to-world 4x4 matrix from USD."""
        xformable = UsdGeom.Xformable(self._camera_prim)
        mat = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # Convert GfMatrix4d to nested list (row-major)
        return [[mat[r][c] for c in range(4)] for r in range(4)]

    def try_capture(self):
        """Capture a frame if enough time has elapsed since the last one."""
        now = time.time()
        if now - self._last_capture_time < self._capture_interval:
            return False

        # Read RGB
        rgb_data = self._rgb_annot.get_data()
        if rgb_data is None or rgb_data.size == 0:
            return False

        # Read depth (meters)
        depth_data = self._depth_annot.get_data()
        if depth_data is None or depth_data.size == 0:
            return False

        # Read pose
        c2w = self._get_camera_c2w()

        # Save color (RGBA → BGR for cv2 compatibility)
        import cv2
        if rgb_data.ndim == 3 and rgb_data.shape[2] == 4:
            bgr = cv2.cvtColor(rgb_data, cv2.COLOR_RGBA2BGR)
        elif rgb_data.ndim == 3 and rgb_data.shape[2] == 3:
            bgr = cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR)
        else:
            bgr = rgb_data
        color_path = os.path.join(self._color_dir, f"frame_{self._frame_idx}.png")
        cv2.imwrite(color_path, bgr)

        # Save depth
        depth_2d = depth_data.squeeze() if depth_data.ndim > 2 else depth_data
        depth_2d = depth_2d.astype(np.float32)

        if self._depth_format in ("npy", "both"):
            npy_path = os.path.join(self._depth_dir, f"frame_{self._frame_idx}.npy")
            np.save(npy_path, depth_2d)
        if self._depth_format in ("png", "both"):
            depth_mm = np.clip(depth_2d * self._depth_scale, 0, 65535).astype(np.uint16)
            png_path = os.path.join(self._depth_dir, f"frame_{self._frame_idx}.png")
            cv2.imwrite(png_path, depth_mm)

        # Store pose
        self._poses[str(self._frame_idx)] = c2w

        self._last_capture_time = now
        self._frame_idx += 1

        if self._frame_idx % 10 == 0:
            print(f"[INFO] Captured frame {self._frame_idx}", flush=True)

        return True

    def save(self):
        """Write poses.json to disk."""
        data = {
            "intrinsics": {
                "fx": self._intrinsics["fx"],
                "fy": self._intrinsics["fy"],
                "cx": self._intrinsics["cx"],
                "cy": self._intrinsics["cy"],
                "width": self._intrinsics["width"],
                "height": self._intrinsics["height"],
            },
            "poses": self._poses,
        }
        with open(self._poses_file, "w") as f:
            json.dump(data, f, indent=4)
        print(f"[INFO] Saved {len(self._poses)} poses to {self._poses_file}")


# =============================================================================
# 9. Main
# =============================================================================

def main():
    print("=" * 60)
    print("  Capture Scene — RGBD + Pose Recorder")
    print("=" * 60)

    scene_cfg = cfg["scene"]
    cam_cfg = cfg["camera"]
    rec_cfg = cfg["recording"]
    teleop_cfg = cfg["teleop"]

    # --- World ---
    world = World(stage_units_in_meters=1.0)

    # --- Warehouse ---
    load_warehouse(scene_cfg.get("warehouse", "full_warehouse"))
    simulation_app.update()

    # --- Objects ---
    objects_list = scene_cfg.get("objects", [])
    if objects_list:
        spawn_objects(objects_list)
    simulation_app.update()

    # --- Explorer camera ---
    _, body_path, camera_path = create_explorer_camera(cam_cfg, teleop_cfg)
    simulation_app.update()

    # --- ROS 2 teleop ---
    define_prim("/World/ROS2", "Xform")
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/publish_without_verification", True)

    ns = teleop_cfg.get("namespace", "explorer")
    domain_id = teleop_cfg.get("domain_id", 47)
    controller = BodyVelocityController(
        body_prim_path=body_path,
        namespace=ns,
        domain_id=domain_id,
        max_speed=teleop_cfg.get("max_speed", 3.0),
        max_yaw_rate=teleop_cfg.get("max_yaw_rate", 1.5),
    )
    simulation_app.update()

    # --- Recorder ---
    recorder = PosedImageRecorder(
        camera_prim_path=camera_path,
        cam_cfg=cam_cfg,
        rec_cfg=rec_cfg,
    )

    # --- Start simulation ---
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    world.reset()
    simulation_app.update()
    timeline.play()

    for _ in range(30):
        simulation_app.update()

    print()
    print("=" * 60)
    print(f"  Recording to: {rec_cfg['output_dir']}")
    print(f"  Teleop:  ROS_DOMAIN_ID={domain_id}  topic=/{ns}/cmd_vel")
    print(f"  Press Ctrl+C to stop and save poses.json")
    print("=" * 60)
    print("[INFO] Simulation running. Recording started.", flush=True)

    # --- Main loop ---
    try:
        while simulation_app.is_running():
            simulation_app.update()
            controller.update()
            recorder.try_capture()
    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped. Captured {recorder._frame_idx} frames.", flush=True)
    finally:
        recorder.save()
        timeline.stop()
        simulation_app.close()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
