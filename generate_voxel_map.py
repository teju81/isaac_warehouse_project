#!/usr/bin/env python3
"""
Generate Voxel Map — 3D Occupancy Grid from USD Scene
======================================================
Extracts all mesh geometry from the USD stage, voxelizes it into a 3D
occupancy grid, and optionally projects a 2D Nav2-compatible map.

Outputs (all paths from config):
    voxel_map.npz      3D occupancy grid  (grid, origin, resolution)
    scene_mesh.ply     Triangle mesh for Blender verification
    voxel_grid.ply     Voxel point cloud for Blender verification
    map.pgm + map.yaml Nav2 2D occupancy grid (optional)

Usage:
    cd ~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /path/to/generate_voxel_map.py --config /path/to/capture_config.yaml --headless
"""

import argparse
import os
import struct
import sys

import numpy as np
import yaml

# =============================================================================
# 1. Parse arguments BEFORE launching SimulationApp
# =============================================================================
parser = argparse.ArgumentParser(description="Generate Voxel Map from USD Scene")
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
    "width": 640,
    "height": 480,
}

simulation_app = SimulationApp(SIM_CONFIG)
print("[INFO] Isaac Sim application started.")

# =============================================================================
# 3. Omniverse imports (MUST come after SimulationApp)
# =============================================================================
import omni
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import define_prim
from pxr import Gf, Usd, UsdGeom

# =============================================================================
# 4. Asset root + warehouse paths (reused from capture_scene.py)
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
# 5. Scene creation (reused from capture_scene.py)
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
# 6. Mesh extraction from USD stage
# =============================================================================

DEFAULT_SKIP_PREFIXES = ["/World/Cameras", "/World/ROS2", "/World/Robots"]


def extract_meshes_from_stage(stage, skip_prefixes=None):
    """Extract all UsdGeom.Mesh triangles from the stage in world space.

    Args:
        stage: The USD stage.
        skip_prefixes: List of prim path prefixes to exclude (robots, cameras, etc.).

    Returns:
        List of (T, 3, 3) numpy arrays — one per mesh prim. Each row is a
        triangle with three 3D vertices in world coordinates.
    """
    if skip_prefixes is None:
        skip_prefixes = DEFAULT_SKIP_PREFIXES

    root = stage.GetPseudoRoot()
    all_mesh_triangles = []
    mesh_count = 0

    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        path_str = str(prim.GetPath())

        # Skip excluded subtrees
        if any(path_str.startswith(prefix) for prefix in skip_prefixes):
            continue

        if not prim.IsA(UsdGeom.Mesh):
            continue

        # Skip invisible prims
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue

        mesh = UsdGeom.Mesh(prim)

        points = mesh.GetPointsAttr().Get()
        fv_counts = mesh.GetFaceVertexCountsAttr().Get()
        fv_indices = mesh.GetFaceVertexIndicesAttr().Get()

        if not points or not fv_counts or not fv_indices:
            continue

        pts = np.array(points, dtype=np.float64)
        counts = np.array(fv_counts, dtype=np.int32)
        indices = np.array(fv_indices, dtype=np.int32)

        # Transform points to world space (USD uses row-vector convention)
        xformable = UsdGeom.Xformable(prim)
        mat = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        mat_np = np.array(mat, dtype=np.float64)  # 4x4 row-major
        pts_world = pts @ mat_np[:3, :3] + mat_np[3, :3]

        # Triangulate
        triangles = triangulate_faces(pts_world, counts, indices)
        if triangles is not None and len(triangles) > 0:
            all_mesh_triangles.append(triangles)
            mesh_count += 1

    print(f"[INFO] Extracted {mesh_count} meshes from stage.")
    return all_mesh_triangles


def triangulate_faces(pts_world, fv_counts, fv_indices):
    """Fan-triangulate polygon faces into (T, 3, 3) triangle array.

    For each face with N vertices, produces N-2 triangles by fanning from
    vertex 0 of that face.

    Args:
        pts_world: (V, 3) world-space vertex positions.
        fv_counts: (F,) number of vertices per face.
        fv_indices: (sum(fv_counts),) vertex indices.

    Returns:
        (T, 3, 3) float64 array of triangle vertices, or None if empty.
    """
    total_tris = int(np.sum(fv_counts - 2))
    if total_tris <= 0:
        return None

    triangles = np.empty((total_tris, 3, 3), dtype=np.float64)
    tri_idx = 0
    face_start = 0

    for n_verts in fv_counts:
        face_indices = fv_indices[face_start : face_start + n_verts]
        v0 = pts_world[face_indices[0]]
        for j in range(1, n_verts - 1):
            triangles[tri_idx, 0] = v0
            triangles[tri_idx, 1] = pts_world[face_indices[j]]
            triangles[tri_idx, 2] = pts_world[face_indices[j + 1]]
            tri_idx += 1
        face_start += n_verts

    return triangles


# =============================================================================
# 7. Surface voxelization
# =============================================================================

def voxelize_triangles(all_triangles, origin, resolution, grid_shape):
    """Voxelize triangle surfaces by barycentric sampling.

    For each triangle, samples points on its surface and marks the
    corresponding voxels as occupied.

    Args:
        all_triangles: List of (T, 3, 3) triangle arrays.
        origin: (3,) world position of grid[0,0,0].
        resolution: Voxel edge length in meters.
        grid_shape: (Nx, Ny, Nz) grid dimensions.

    Returns:
        (Nx, Ny, Nz) uint8 occupancy grid (0=free, 1=occupied).
    """
    grid = np.zeros(grid_shape, dtype=np.uint8)
    total_tris = sum(t.shape[0] for t in all_triangles)
    processed = 0

    for tri_batch in all_triangles:
        for t in range(tri_batch.shape[0]):
            v0 = tri_batch[t, 0]
            v1 = tri_batch[t, 1]
            v2 = tri_batch[t, 2]

            e1 = v1 - v0
            e2 = v2 - v0

            len1 = np.linalg.norm(e1)
            len2 = np.linalg.norm(e2)

            # Number of samples along each edge
            n1 = max(1, int(np.ceil(len1 / resolution)))
            n2 = max(1, int(np.ceil(len2 / resolution)))

            # Barycentric grid: u + v <= 1
            u_vals = np.linspace(0, 1, n1 + 1)
            v_vals = np.linspace(0, 1, n2 + 1)
            uu, vv = np.meshgrid(u_vals, v_vals)
            mask = (uu + vv) <= 1.0
            uu = uu[mask]
            vv = vv[mask]

            # Sample points on the triangle surface
            points = v0 + np.outer(uu, e1) + np.outer(vv, e2)

            # Convert to voxel indices
            ijk = np.floor((points - origin) / resolution).astype(np.int64)

            # Clip to grid bounds
            valid = (
                (ijk[:, 0] >= 0) & (ijk[:, 0] < grid_shape[0]) &
                (ijk[:, 1] >= 0) & (ijk[:, 1] < grid_shape[1]) &
                (ijk[:, 2] >= 0) & (ijk[:, 2] < grid_shape[2])
            )
            ijk = ijk[valid]

            grid[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = 1

            processed += 1
            if processed % 50000 == 0:
                print(f"[INFO] Voxelized {processed}/{total_tris} triangles...", flush=True)

    print(f"[INFO] Voxelized {processed} triangles total.")
    return grid


# =============================================================================
# 8. Output writers
# =============================================================================

def save_voxel_map(grid, origin, resolution, path):
    """Save 3D voxel occupancy grid as compressed .npz."""
    np.savez_compressed(
        path,
        grid=grid,
        origin=origin.astype(np.float64),
        resolution=np.float64(resolution),
    )
    occupied = int(grid.sum())
    total = grid.size
    print(f"[INFO] Saved voxel map: {path}")
    print(f"       Shape: {grid.shape}, Occupied: {occupied}/{total} "
          f"({100.0 * occupied / total:.2f}%)")


def _height_to_rgb(z_vals, z_min, z_max):
    """Map height values to a blue→green→red colour gradient (uint8 RGB)."""
    t = np.clip((z_vals - z_min) / max(z_max - z_min, 1e-6), 0, 1)
    r = np.clip(2.0 * t - 1.0, 0, 1)
    g = 1.0 - 2.0 * np.abs(t - 0.5)
    b = np.clip(1.0 - 2.0 * t, 0, 1)
    rgb = np.stack([r, g, b], axis=-1)
    return (rgb * 255).astype(np.uint8)


def save_mesh_ply(all_triangles, path):
    """Write extracted scene mesh as a binary PLY with per-vertex height colours.

    Args:
        all_triangles: List of (T, 3, 3) triangle arrays.
        path: Output .ply file path.
    """
    # Concatenate all triangles
    triangles = np.concatenate(all_triangles, axis=0)  # (T, 3, 3)
    n_tris = triangles.shape[0]
    n_verts = n_tris * 3

    # Flatten vertices: (T*3, 3)
    vertices = triangles.reshape(-1, 3)

    # Colour by height
    z_min, z_max = float(vertices[:, 2].min()), float(vertices[:, 2].max())
    colours = _height_to_rgb(vertices[:, 2], z_min, z_max)

    # Face indices: each triangle references 3 consecutive vertices
    face_indices = np.arange(n_verts, dtype=np.int32).reshape(n_tris, 3)

    # Write binary PLY
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n_verts}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        f"element face {n_tris}\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    )

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))

        # Write vertices
        for i in range(n_verts):
            f.write(struct.pack("<fff", *vertices[i]))
            f.write(struct.pack("<BBB", *colours[i]))

        # Write faces
        for i in range(n_tris):
            f.write(struct.pack("<B", 3))
            f.write(struct.pack("<iii", *face_indices[i]))

    size_mb = os.path.getsize(path) / (1024 * 1024)
    print(f"[INFO] Saved mesh PLY: {path} ({n_tris} triangles, {size_mb:.1f} MB)")


def save_voxel_ply(grid, origin, resolution, path):
    """Write occupied voxel centres as a coloured point cloud PLY.

    Args:
        grid: (Nx, Ny, Nz) uint8 occupancy grid.
        origin: (3,) world position of grid[0,0,0].
        resolution: Voxel edge length in meters.
        path: Output .ply file path.
    """
    occupied = np.argwhere(grid)  # (N, 3)
    if len(occupied) == 0:
        print("[WARN] No occupied voxels — skipping voxel PLY.")
        return

    # Voxel centres in world space
    centres = origin + (occupied + 0.5) * resolution
    n_pts = len(centres)

    # Colour by height (Z)
    z_min, z_max = float(centres[:, 2].min()), float(centres[:, 2].max())
    colours = _height_to_rgb(centres[:, 2], z_min, z_max)

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n_pts}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        for i in range(n_pts):
            f.write(struct.pack("<fff", *centres[i]))
            f.write(struct.pack("<BBB", *colours[i]))

    size_mb = os.path.getsize(path) / (1024 * 1024)
    print(f"[INFO] Saved voxel PLY: {path} ({n_pts} points, {size_mb:.1f} MB)")


def _write_nav2_pgm(occupancy_2d, origin, resolution, output_base, label=""):
    """Write a 2D occupancy array as Nav2 PGM + YAML.

    Args:
        occupancy_2d: (Nx, Ny) bool — True = occupied.
        origin: (3,) world position of grid[0,0,0].
        resolution: Voxel edge length in meters.
        output_base: Base path (without extension).
        label: Description printed in the log line.
    """
    pgm = np.where(occupancy_2d, 0, 254).astype(np.uint8)
    pgm = np.flipud(pgm.T)  # transpose (X,Y->row,col) then flip for image Y-up

    height, width = pgm.shape
    pgm_path = f"{output_base}.pgm"
    yaml_path = f"{output_base}.yaml"

    with open(pgm_path, "wb") as f:
        header = f"P5\n{width} {height}\n255\n"
        f.write(header.encode("ascii"))
        f.write(pgm.tobytes())

    map_yaml = {
        "image": os.path.basename(pgm_path),
        "resolution": float(resolution),
        "origin": [float(origin[0]), float(origin[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(yaml_path, "w") as f:
        yaml.dump(map_yaml, f, default_flow_style=False)

    occ_pct = 100.0 * occupancy_2d.sum() / occupancy_2d.size
    tag = f" ({label})" if label else ""
    print(f"[INFO] Saved Nav2 map{tag}: {pgm_path} ({width}x{height}, "
          f"{occ_pct:.1f}% occupied)")


def save_nav2_maps(grid, origin, resolution, z_min, z_max, output_base):
    """Generate three Nav2 2D map variants for comparison.

    Variants:
        {output_base}_raw      — direct projection of [z_min, z_max] (includes floor)
        {output_base}_raised   — skip the bottom voxel layer (floor surface)
        {output_base}_auto     — auto-detect the floor layer and exclude it

    Args:
        grid: (Nx, Ny, Nz) uint8 occupancy grid.
        origin: (3,) world position of grid[0,0,0].
        resolution: Voxel edge length in meters.
        z_min: Lower Z bound for 2D projection (meters).
        z_max: Upper Z bound for 2D projection (meters).
        output_base: Base path (without extension).
    """
    iz_min = max(0, int(np.floor((z_min - origin[2]) / resolution)))
    iz_max = min(grid.shape[2], int(np.ceil((z_max - origin[2]) / resolution)))

    if iz_min >= iz_max:
        print(f"[WARN] nav2_height_range [{z_min}, {z_max}] produces empty slice — skipping.")
        return

    slab = grid[:, :, iz_min:iz_max]  # (Nx, Ny, Nz_slice)
    print(f"[INFO] Nav2 Z-slice: [{z_min}, {z_max}] m -> grid layers [{iz_min}, {iz_max})")

    # --- Variant 1: raw (full Z-slice, includes floor) ---
    occ_raw = np.any(slab, axis=2)
    _write_nav2_pgm(occ_raw, origin, resolution, f"{output_base}_raw", "raw")

    # --- Variant 2: raised (skip bottom layer of the slice) ---
    if slab.shape[2] > 1:
        occ_raised = np.any(slab[:, :, 1:], axis=2)
    else:
        occ_raised = occ_raw
    _write_nav2_pgm(occ_raised, origin, resolution, f"{output_base}_raised",
                     f"raised — skip layer {iz_min}")

    # --- Variant 3: auto (detect floor layer by max occupancy, exclude it) ---
    per_layer_occ = slab.sum(axis=(0, 1))  # (Nz_slice,) occupancy count per layer
    floor_layer = int(np.argmax(per_layer_occ))
    floor_z = origin[2] + (iz_min + floor_layer + 0.5) * resolution
    floor_occ_pct = 100.0 * per_layer_occ[floor_layer] / (slab.shape[0] * slab.shape[1])

    print(f"[INFO] Auto-detected floor: layer {iz_min + floor_layer} "
          f"(z={floor_z:.2f} m, {floor_occ_pct:.1f}% occupied)")

    slab_no_floor = slab.copy()
    slab_no_floor[:, :, floor_layer] = 0
    occ_auto = np.any(slab_no_floor, axis=2)
    _write_nav2_pgm(occ_auto, origin, resolution, f"{output_base}_auto",
                     f"auto — excluded layer z={floor_z:.2f}m")


# =============================================================================
# 9. Main
# =============================================================================

def main():
    print("=" * 60)
    print("  Generate Voxel Map from USD Scene")
    print("=" * 60)

    scene_cfg = cfg["scene"]
    voxel_cfg = cfg["voxel_map"]

    # --- Output directory ---
    output_dir = voxel_cfg["output_dir"]
    os.makedirs(output_dir, exist_ok=True)

    # Resolve output paths (relative to output_dir unless absolute)
    def resolve_path(name, default):
        val = voxel_cfg.get(name, default)
        if val is None:
            return None
        if os.path.isabs(val):
            return val
        return os.path.join(output_dir, val)

    voxel_npz_path = resolve_path("voxel_npz", "voxel_map.npz")
    mesh_ply_path = resolve_path("mesh_ply", "scene_mesh.ply")
    voxel_ply_path = resolve_path("voxel_ply", "voxel_grid.ply")
    nav2_base = resolve_path("nav2_map", None)

    resolution = float(voxel_cfg.get("resolution", 0.1))
    height_range = voxel_cfg.get("height_range", [-0.5, 8.5])
    padding = float(voxel_cfg.get("padding", 0.5))

    # --- Load scene ---
    load_warehouse(scene_cfg.get("warehouse", "full_warehouse"))
    simulation_app.update()

    objects_list = scene_cfg.get("objects", [])
    if objects_list:
        spawn_objects(objects_list)
    simulation_app.update()

    # Let USD composition fully resolve
    for _ in range(5):
        simulation_app.update()

    # --- Extract meshes ---
    stage = omni.usd.get_context().get_stage()
    all_triangles = extract_meshes_from_stage(stage)

    if not all_triangles:
        print("[ERROR] No mesh geometry found in stage.")
        simulation_app.close()
        sys.exit(1)

    # --- Compute bounding box ---
    all_verts = np.concatenate(
        [t.reshape(-1, 3) for t in all_triangles], axis=0
    )
    scene_min = all_verts.min(axis=0)
    scene_max = all_verts.max(axis=0)
    print(f"[INFO] Scene bounds: min={scene_min}, max={scene_max}")

    # Apply height range and padding
    origin = np.array([
        scene_min[0] - padding,
        scene_min[1] - padding,
        max(scene_min[2], height_range[0]),
    ])
    extent = np.array([
        scene_max[0] + padding,
        scene_max[1] + padding,
        min(scene_max[2], height_range[1]),
    ]) - origin

    grid_shape = tuple(np.ceil(extent / resolution).astype(int))
    print(f"[INFO] Voxel grid: shape={grid_shape}, "
          f"origin={origin}, resolution={resolution} m")

    # --- Voxelize ---
    grid = voxelize_triangles(all_triangles, origin, resolution, grid_shape)

    # --- Save outputs ---
    save_voxel_map(grid, origin, resolution, voxel_npz_path)

    if mesh_ply_path:
        save_mesh_ply(all_triangles, mesh_ply_path)

    if voxel_ply_path:
        save_voxel_ply(grid, origin, resolution, voxel_ply_path)

    if nav2_base:
        nav2_hr = voxel_cfg.get("nav2_height_range", [0.0, 1.5])
        save_nav2_maps(grid, origin, resolution, nav2_hr[0], nav2_hr[1], nav2_base)

    # --- Done ---
    simulation_app.close()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
