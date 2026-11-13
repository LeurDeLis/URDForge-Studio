"""URDForge Studio FastAPI Server & API Backend.

Provides REST and WebSocket endpoints for STEP assembly parsing,
B-Rep feature snapping, interactive forward kinematics (FK),
mesh streaming, and ROS / ROS 2 package generation.
"""

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import Body, FastAPI, HTTPException, Query, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from urdforge.core.features import FeatureDetector, GeometricFeature
from urdforge.core.kinematics import ForwardKinematicsEngine
from urdforge.core.math_types import Transform3D
from urdforge.core.mesher import TessellationConfig
from urdforge.core.physics import (
    compute_composite_physical_properties,
    compute_shape_physical_properties,
)
from urdforge.core.robot_model import (
    JointLimits,
    JointModel,
    JointType,
    LinkModel,
    RobotModel,
    VisualGeometry,
)
from urdforge.core.step_parser import AssemblyNode, StepAssemblyParser, get_cad_up_axis_transform
from urdforge.exporter.ros_package import ROSPackageConfig, ROSPackageExporter
from urdforge.utils.naming import sanitize_urdf_name


app = FastAPI(
    title="URDForge Studio API",
    description="Next-generation CAD to URDF & ROS 2 Robot Modeling Studio",
    version="0.2.0",
)

# CORS middleware for local frontend dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Server state
if getattr(sys, "frozen", False):
    base_path = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    candidates = [
        base_path / "urdforge" / "server" / "static",
        base_path / "src" / "urdforge" / "server" / "static",
        Path(__file__).resolve().parent / "static",
    ]
    STATIC_DIR = next((p for p in candidates if p.exists()), candidates[0])
else:
    STATIC_DIR = Path(__file__).resolve().parent / "static"
CACHE_DIR = Path("output").resolve()
CACHE_DIR.mkdir(parents=True, exist_ok=True)
MESH_DIR = CACHE_DIR / "meshes"
MESH_DIR.mkdir(parents=True, exist_ok=True)

class SessionState:
    def __init__(self):
        self.step_parser: Optional[StepAssemblyParser] = None
        self.root_node: Optional[AssemblyNode] = None
        self.features: List[GeometricFeature] = []
        self.robot: RobotModel = RobotModel(name="urdforge_robot")
        self.exported_meshes: Dict[str, Path] = {}
        self.unit_scale: float = 0.001
        self.default_density: float = 7850.0
        self.cad_up_axis: str = "Z"
        self.export_dir: Optional[Path] = None

state = SessionState()


# --- Pydantic Request Models ---

class LoadStepRequest(BaseModel):
    filepath: str
    unit_scale: float = 0.001
    default_density: float = 7850.0
    cad_up_axis: str = "Z"

class SetCadUpAxisRequest(BaseModel):
    cad_up_axis: str = "Y"

class CreateLinkRequest(BaseModel):
    name: str
    parent_link: Optional[str] = "base_link"
    joint_name: Optional[str] = None
    joint_type: str = "revolute"
    part_names: List[str] = Field(default_factory=list)
    cad_part_ids: Optional[List[str]] = None
    create_joint: bool = True
    origin_xyz: Optional[List[float]] = None
    origin_rpy: Optional[List[float]] = None

class AssignPartsRequest(BaseModel):
    link_name: str
    part_names: List[str] = Field(default_factory=list)
    cad_part_ids: Optional[List[str]] = None

class RemovePartRequest(BaseModel):
    link_name: str
    part_name: str

class ClearPartsRequest(BaseModel):
    link_name: str

class AddJointRequest(BaseModel):
    name: str
    parent_link: str
    child_link: str
    joint_type: str = "revolute"
    origin_xyz: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    axis: List[float] = Field(default_factory=lambda: [0.0, 0.0, 1.0])
    lower_limit: Optional[float] = None
    upper_limit: Optional[float] = None
    max_effort: float = 100.0
    max_velocity: float = 3.14159

class SnapJointRequest(BaseModel):
    feature_id: str
    joint_name: str
    parent_link: str
    child_link: str
    joint_type: str = "revolute"
    flip_axis: bool = False
    snap_point: str = "midpoint"  # "midpoint", "top", "bottom"

class FKRequest(BaseModel):
    joint_positions: Dict[str, float]

class OverridePhysicsRequest(BaseModel):
    link_name: str
    mass: float = 0.0
    com: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0
    reset_auto: bool = False

class RenameLinkRequest(BaseModel):
    old_name: str
    new_name: str

class ExportROSRequest(BaseModel):
    target_directory: str = "./output/ros_ws"
    package_name: Optional[str] = None
    robot_name: Optional[str] = None
    use_package_uri: bool = True

class UpdateURDFRequest(BaseModel):
    urdf_xml: str

class RecalculateAllPhysicsRequest(BaseModel):
    force_reset: bool = True



def recalculate_link_composite(link_name: str) -> None:
    """Recalculate composite physical properties and visual offsets for a multi-part link."""
    if state.robot is None or link_name not in state.robot.links:
        return
    link = state.robot.links[link_name]

    # Evaluate zero-configuration link world pose
    fk_engine = ForwardKinematicsEngine(state.robot)
    fk_res = fk_engine.evaluate(joint_values={})
    link_world = fk_res.link_poses_world.get(link_name, Transform3D.identity())

    # Build part map
    all_parts: Dict[str, AssemblyNode] = {}
    if state.root_node is not None:
        for p in state.root_node.get_all_parts():
            all_parts[p.name] = p
            all_parts[p.id] = p

    part_pairs = []
    link.visuals = []
    link.collisions = []

    for pid in link.cad_part_ids:
        part = all_parts.get(pid)
        if part is None:
            continue
        p_mat = part.global_transform.matrix.copy()
        p_mat[:3, 3] = part.global_transform.translation * state.unit_scale
        part_world_xform = Transform3D(matrix=p_mat)

        # Relative visual origin: T_rel = T_link_world^-1 @ T_part_world
        t_rel = link_world.inverse() @ part_world_xform
        vis = VisualGeometry(
            mesh_filepath=f"meshes/visual/{part.name}.stl",
            origin=t_rel,
            color_rgba=part.color_rgba,
        )
        col = VisualGeometry(
            mesh_filepath=f"meshes/collision/{part.name}.stl",
            origin=t_rel,
        )
        link.visuals.append(vis)
        link.collisions.append(col)

        # Pass t_rel so composite Center of Mass and parallel-axis inertia tensor
        # are calculated directly in the link's local coordinate frame, guaranteeing
        # 100% mathematical coincidence with the visual and collision mesh geometries.
        part_pairs.append((part.physical_props, t_rel))

    # Compute composite mass, CoM, and 3x3 inertia tensor directly in link local frame (unless manually overridden)
    if not (getattr(link, "has_manual_inertial", False) and link.inertial is not None):
        if part_pairs:
            composite_props = compute_composite_physical_properties(
                parts_data=part_pairs,
                link_frame_world=None,
            )
            link.inertial = composite_props
        elif link.visuals:
            from urdforge.core.physics import compute_link_physics_from_meshes
            composite_props = compute_link_physics_from_meshes(
                visuals=link.visuals,
                specified_mass=link.inertial.mass if link.inertial else None,
                default_density=state.default_density,
                base_dir=str(state.export_dir) if state.export_dir else None,
            )
            link.inertial = composite_props



def recalculate_link_and_descendants(link_name: str) -> None:
    """Recursively update composite properties for link and all its kinematic descendants."""
    recalculate_link_composite(link_name)
    if state.robot is not None:
        for j in state.robot.get_child_joints(link_name):
            recalculate_link_and_descendants(j.child_link)


# --- API Routes ---

@app.post("/api/system/choose_file")
def choose_file_dialog() -> Dict[str, Any]:
    """Open native OS file chooser to pick any STEP file on disk."""
    import tkinter as tk
    from tkinter import filedialog
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        filepath = filedialog.askopenfilename(
            title="选择 STEP/STP 机器人装配体文件",
            filetypes=[
                ("STEP CAD Files (*.step;*.stp)", "*.step;*.stp;*.STEP;*.STP"),
                ("All Files (*.*)", "*.*"),
            ],
        )
        root.destroy()
        if filepath:
            return {"status": "selected", "filepath": str(Path(filepath).resolve())}
        return {"status": "cancelled", "filepath": ""}
    except Exception as e:
        return {"status": "error", "error": str(e), "filepath": ""}


@app.post("/api/system/choose_dir")
def choose_directory_dialog() -> Dict[str, Any]:
    """Open native OS directory chooser for export target."""
    import tkinter as tk
    from tkinter import filedialog
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        folder = filedialog.askdirectory(title="选择 ROS 2 功能包保存目录")
        root.destroy()
        if folder:
            return {"status": "selected", "directory": str(Path(folder).resolve())}
        return {"status": "cancelled", "directory": ""}
    except Exception as e:
        return {"status": "error", "error": str(e), "directory": ""}


def _detect_all_features() -> List[GeometricFeature]:
    """Run B-Rep geometric feature detection across all leaf parts in current global frame."""
    if state.root_node is None:
        return []
    detector = FeatureDetector(unit_scale=state.unit_scale)
    feats_all: List[GeometricFeature] = []
    for part in state.root_node.get_all_parts():
        if part.shape and not part.shape.IsNull():
            feats = detector.detect_features(
                shape=part.shape,
                part_name=part.name,
                global_transform=part.global_transform,
            )
            feats_all.extend(feats)
    return feats_all


@app.post("/api/step/upload")
async def upload_step_file(
    file: UploadFile = File(...),
    cad_up_axis: str = Query("Z"),
) -> Dict[str, Any]:
    """Upload and parse a custom STEP/STP file directly."""
    fn = file.filename or "uploaded_model.step"
    if not fn.lower().endswith((".step", ".stp")):
        raise HTTPException(status_code=400, detail="Only .step and .stp files are supported.")

    upload_dir = CACHE_DIR / "uploads"
    upload_dir.mkdir(parents=True, exist_ok=True)
    target_path = upload_dir / fn

    with open(target_path, "wb") as f:
        content = await file.read()
        f.write(content)

    return load_step_file(LoadStepRequest(filepath=str(target_path), cad_up_axis=cad_up_axis))


@app.post("/api/step/load")
def load_step_file(req: LoadStepRequest) -> Dict[str, Any]:
    """Parse a STEP CAD assembly and prepare visual meshes & features."""
    path = Path(req.filepath).resolve()
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"File not found: {path}")

    try:
        state.unit_scale = req.unit_scale
        state.default_density = req.default_density
        state.step_parser = StepAssemblyParser(
            filepath=path,
            unit_scale=req.unit_scale,
            default_density=req.default_density,
            cad_up_axis=req.cad_up_axis,
        )
        state.cad_up_axis = state.step_parser.cad_up_axis
        state.root_node = state.step_parser.load()

        # Export meshes for all parts to cache dir
        tess_cfg = TessellationConfig(
            linear_deflection=0.001,
            angular_deflection=0.3,
            scale_to_meter=True,
            unit_scale=req.unit_scale,
        )
        mesh_results = state.step_parser.export_all_parts(
            output_dir=MESH_DIR,
            config=tess_cfg,
            frame="local",
        )
        state.exported_meshes = {k: v.file_path for k, v in mesh_results.items()}

        # Detect B-Rep geometric features on all leaf parts
        state.features = _detect_all_features()

        # Initialize robot model with ONLY base_link (do not auto-create 56 links and joints)
        robot_name = path.stem.replace(" ", "_")
        state.robot = RobotModel(name=robot_name, root_link="base_link")
        state.robot.add_link(LinkModel(name="base_link", cad_part_ids=[]))

        return {
            "status": "success",
            "message": f"Successfully parsed {path.name}",
            "robot_name": state.robot.name,
            "parts_count": len(state.root_node.get_all_parts()),
            "features_count": len(state.features),
            "unit_scale": state.unit_scale,
            "cad_up_axis": state.cad_up_axis,
            "assembly_tree": state.root_node.to_dict(),
            "robot_model": state.robot.to_dict(),
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error parsing STEP: {str(e)}")


@app.post("/api/step/set_cad_up_axis")
def set_cad_up_axis(req: SetCadUpAxisRequest) -> Dict[str, Any]:
    """Reorient CAD assembly root ('Z', 'Y' for SolidWorks +Y->+Z, or '-Y') across CAD tree, features, joints, visuals, inertials, and URDF."""
    if state.step_parser is None or state.root_node is None:
        raise HTTPException(status_code=400, detail="请先加载 STEP 模型。")

    new_axis = (req.cad_up_axis or "Z").strip().upper()
    if new_axis not in ("Z", "Y", "-Y"):
        raise HTTPException(status_code=400, detail=f"Unsupported cad_up_axis: {req.cad_up_axis}")

    old_trsf = get_cad_up_axis_transform(state.cad_up_axis)
    new_trsf = state.step_parser.apply_cad_up_axis(new_axis)
    delta_trsf = new_trsf @ old_trsf.inverse()
    state.cad_up_axis = new_axis

    # Re-detect features in the updated global frame
    state.features = _detect_all_features()

    # Coherently transform robot kinematics and link composite properties
    if state.robot is not None:
        for joint in state.robot.joints.values():
            if joint.parent_link == state.robot.root_link:
                joint.origin = delta_trsf @ joint.origin
        for l_name in state.robot.links:
            recalculate_link_composite(l_name)

    fk_res = compute_forward_kinematics(FKRequest(joint_positions={}))
    return {
        "status": "success",
        "cad_up_axis": state.cad_up_axis,
        "features_count": len(state.features),
        "assembly_tree": state.root_node.to_dict(),
        "robot_model": state.robot.to_dict(),
        "fk": fk_res,
    }


@app.get("/api/step/hierarchy")
def get_hierarchy() -> Dict[str, Any]:
    """Get the parsed assembly hierarchy and part metadata."""
    if state.root_node is None:
        raise HTTPException(status_code=400, detail="No STEP assembly loaded.")
    return {
        "assembly_tree": state.root_node.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.get("/api/step/features")
def get_features(part_name: Optional[str] = None) -> List[Dict[str, Any]]:
    """Get detected candidate geometric features for joint snapping."""
    if part_name:
        return [f.to_dict() for f in state.features if f.part_name == part_name]
    return [f.to_dict() for f in state.features]


@app.get("/api/mesh/{part_name}")
def get_part_mesh(part_name: str):
    """Serve the binary STL mesh for Three.js 3D viewport rendering."""
    stl_path = MESH_DIR / f"{part_name}.stl"
    if not stl_path.exists():
        raise HTTPException(status_code=404, detail=f"Mesh not found for part: {part_name}")
    return FileResponse(
        path=stl_path,
        media_type="application/octet-stream",
        filename=f"{part_name}.stl",
    )


# --- Link & Multi-part Management Routes ---

@app.post("/api/robot/link/create")
@app.post("/api/robot/add_link")
def create_robot_link(req: CreateLinkRequest) -> Dict[str, Any]:
    """Create a new Link starting from base_link in the robot kinematic hierarchy."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    link_name = req.name.strip()
    if not link_name:
        raise HTTPException(status_code=400, detail="Link name cannot be empty.")

    if link_name in state.robot.links:
        target_link = state.robot.links[link_name]
    else:
        target_link = LinkModel(name=link_name, cad_part_ids=[])
        state.robot.add_link(target_link)

    # Assign parts if specified (support both part_names and cad_part_ids)
    assigned_parts = req.part_names or req.cad_part_ids or []
    if assigned_parts:
        for other_name, other_link in list(state.robot.links.items()):
            if other_name == link_name:
                continue
            orig_len = len(other_link.cad_part_ids)
            other_link.cad_part_ids = [p for p in other_link.cad_part_ids if p not in assigned_parts]
            if len(other_link.cad_part_ids) != orig_len:
                recalculate_link_composite(other_name)
        for p in assigned_parts:
            if p not in target_link.cad_part_ids:
                target_link.cad_part_ids.append(p)

    # If user chose to create joint
    if req.create_joint and req.parent_link and req.parent_link in state.robot.links and req.parent_link != link_name:
        j_name = req.joint_name or f"joint_{link_name}"
        try:
            j_type = JointType(req.joint_type.lower()) if req.joint_type else JointType.REVOLUTE
        except (ValueError, AttributeError):
            j_type = JointType.REVOLUTE

        if req.origin_xyz is not None and req.origin_rpy is not None:
            orig = Transform3D.from_rpy_xyz(req.origin_rpy, req.origin_xyz)
        else:
            orig = Transform3D.identity()

        joint = JointModel(
            name=j_name,
            parent_link=req.parent_link,
            child_link=link_name,
            joint_type=j_type,
            origin=orig,
            axis=(0.0, 0.0, 1.0),
        )
        state.robot.add_joint(joint)

    recalculate_link_and_descendants(link_name)

    return {
        "status": "success",
        "message": f"Link '{link_name}' created.",
        "link": state.robot.links[link_name].to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/link/assign_parts")
@app.post("/api/robot/assign_parts")
def assign_parts_to_link(req: AssignPartsRequest) -> Dict[str, Any]:
    """Assign multiple CAD parts to a single Link, computing composite physics."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    link_name = req.link_name
    if link_name not in state.robot.links:
        # Auto-create link if it does not exist
        state.robot.add_link(LinkModel(name=link_name, cad_part_ids=[]))

    target_link = state.robot.links[link_name]

    parts_to_assign = req.part_names or req.cad_part_ids or []

    # Remove assigned parts from any other links so each part belongs to exactly one rigid link
    for other_name, other_link in list(state.robot.links.items()):
        if other_name == link_name:
            continue
        orig_len = len(other_link.cad_part_ids)
        other_link.cad_part_ids = [p for p in other_link.cad_part_ids if p not in parts_to_assign]
        if len(other_link.cad_part_ids) != orig_len:
            recalculate_link_composite(other_name)

    # Append new parts to target link
    for p in parts_to_assign:
        if p not in target_link.cad_part_ids:
            target_link.cad_part_ids.append(p)

    recalculate_link_and_descendants(link_name)

    return {
        "status": "success",
        "message": f"Assigned {len(parts_to_assign)} parts to link '{link_name}'.",
        "link": target_link.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/link/remove_part")
@app.post("/api/robot/remove_part")
def remove_part_from_link(req: RemovePartRequest) -> Dict[str, Any]:
    """Remove a specific CAD part from a Link."""
    if state.robot is None or req.link_name not in state.robot.links:
        raise HTTPException(status_code=404, detail=f"Link '{req.link_name}' not found.")

    target_link = state.robot.links[req.link_name]
    target_link.cad_part_ids = [p for p in target_link.cad_part_ids if p != req.part_name]
    recalculate_link_and_descendants(req.link_name)

    return {
        "status": "success",
        "message": f"Removed '{req.part_name}' from link '{req.link_name}'.",
        "link": target_link.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/link/clear_parts")
@app.post("/api/robot/clear_parts")
def clear_parts_from_link(req: ClearPartsRequest) -> Dict[str, Any]:
    """Clear all CAD parts from a Link."""
    if state.robot is None or req.link_name not in state.robot.links:
        raise HTTPException(status_code=404, detail=f"Link '{req.link_name}' not found.")

    target_link = state.robot.links[req.link_name]
    target_link.cad_part_ids = []
    recalculate_link_and_descendants(req.link_name)

    return {
        "status": "success",
        "message": f"Cleared all parts from link '{req.link_name}'.",
        "link": target_link.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.delete("/api/robot/link/{link_name}")
def delete_robot_link(link_name: str) -> Dict[str, Any]:
    """Delete a Link from the robot model (cannot delete base_link)."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")
    if link_name == "base_link":
        raise HTTPException(status_code=400, detail="Cannot delete root 'base_link'.")
    if link_name not in state.robot.links:
        raise HTTPException(status_code=404, detail=f"Link '{link_name}' not found.")

    del state.robot.links[link_name]

    # Delete connected joints
    joints_to_del = [
        j_name for j_name, j in state.robot.joints.items()
        if j.parent_link == link_name or j.child_link == link_name
    ]
    for j_name in joints_to_del:
        del state.robot.joints[j_name]

    return {
        "status": "success",
        "message": f"Link '{link_name}' and connected joints deleted.",
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/link/rename")
def rename_robot_link(req: RenameLinkRequest) -> Dict[str, Any]:
    """Rename a Link and update all referencing joints and kinematics."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    old_name = req.old_name.strip()
    new_name = sanitize_urdf_name(req.new_name.strip())

    if not old_name or not new_name:
        raise HTTPException(status_code=400, detail="连杆名称不能为空。")

    if old_name not in state.robot.links:
        raise HTTPException(status_code=404, detail=f"未找到连杆 '{old_name}'。")

    if old_name == new_name:
        return {
            "status": "success",
            "message": "连杆名称未改变。",
            "old_name": old_name,
            "new_name": new_name,
            "robot_model": state.robot.to_dict(),
            "urdf_xml": state.robot.to_urdf_xml(),
        }

    if new_name in state.robot.links:
        raise HTTPException(status_code=400, detail=f"已存在同名连杆 '{new_name}'，请输入不同名称。")

    # Update links dictionary
    link = state.robot.links.pop(old_name)
    link.name = new_name
    state.robot.links[new_name] = link

    # If root_link was renamed
    if state.robot.root_link == old_name:
        state.robot.root_link = new_name

    # Update all joint parent/child references
    for j in state.robot.joints.values():
        if j.parent_link == old_name:
            j.parent_link = new_name
        if j.child_link == old_name:
            j.child_link = new_name

    # Recalculate kinematics for this link and descendants
    recalculate_link_and_descendants(new_name)

    return {
        "status": "success",
        "message": f"连杆 '{old_name}' 已成功重命名为 '{new_name}'。",
        "old_name": old_name,
        "new_name": new_name,
        "robot_model": state.robot.to_dict(),
        "urdf_xml": state.robot.to_urdf_xml(),
    }


@app.post("/api/robot/add_joint")
def add_or_update_joint(req: AddJointRequest) -> Dict[str, Any]:
    """Add or update a joint in the robot kinematic model."""
    try:
        j_type = JointType(req.joint_type.lower())
    except ValueError:
        raise HTTPException(status_code=400, detail=f"Invalid joint type: {req.joint_type}")

    origin = Transform3D.from_rpy_xyz(req.origin_rpy, req.origin_xyz)
    default_lower = -3.14159 if j_type in (JointType.REVOLUTE, JointType.CONTINUOUS) else 0.0
    default_upper = 3.14159 if j_type in (JointType.REVOLUTE, JointType.CONTINUOUS) else 0.1

    lower_val = req.lower_limit if req.lower_limit is not None else default_lower
    upper_val = req.upper_limit if req.upper_limit is not None else default_upper

    limits = JointLimits(
        lower=lower_val,
        upper=upper_val,
        effort=req.max_effort,
        velocity=req.max_velocity,
    )

    joint = JointModel(
        name=req.name,
        parent_link=req.parent_link,
        child_link=req.child_link,
        joint_type=j_type,
        origin=origin,
        axis=tuple(req.axis),
        limits=limits,
    )
    if req.child_link not in state.robot.links:
        state.robot.add_link(LinkModel(name=req.child_link, cad_part_ids=[req.child_link]))
    state.robot.add_joint(joint)
    recalculate_link_and_descendants(req.child_link)

    return {
        "status": "success",
        "joint": joint.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/snap_joint")
def snap_joint_from_feature(req: SnapJointRequest) -> Dict[str, Any]:
    """Create or snap a joint directly to a detected CAD cylinder/hole feature."""
    target_feat = next((f for f in state.features if f.id == req.feature_id), None)
    if target_feat is None:
        raise HTTPException(status_code=404, detail=f"Feature not found: {req.feature_id}")

    try:
        j_type = JointType(req.joint_type.lower())
    except ValueError:
        j_type = JointType.REVOLUTE

    joint = target_feat.snap_joint(
        joint_name=req.joint_name,
        parent_link=req.parent_link,
        child_link=req.child_link,
        joint_type=j_type,
        flip_axis=req.flip_axis,
        snap_point_type=req.snap_point,
    )
    if req.child_link not in state.robot.links:
        state.robot.add_link(LinkModel(name=req.child_link, cad_part_ids=[req.child_link]))
    state.robot.add_joint(joint)
    recalculate_link_and_descendants(req.child_link)

    return {
        "status": "success",
        "snapped_to": target_feat.to_dict(),
        "joint": joint.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.delete("/api/robot/joint/{joint_name}")
def delete_joint(joint_name: str) -> Dict[str, Any]:
    """Delete a joint from the robot model."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")
    if joint_name in state.robot.joints:
        j = state.robot.joints[joint_name]
        c_link = j.child_link
        del state.robot.joints[joint_name]
        recalculate_link_and_descendants(c_link)
        return {
            "status": "success",
            "message": f"Joint '{joint_name}' deleted.",
            "robot_model": state.robot.to_dict(),
        }
    raise HTTPException(status_code=404, detail=f"Joint '{joint_name}' not found.")


@app.post("/api/robot/joint/clear_all")
@app.delete("/api/robot/joints")
def clear_all_joints() -> Dict[str, Any]:
    """Clear all joints from the robot model for easy rebuilding."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    state.robot.joints.clear()
    for link_name in state.robot.links:
        recalculate_link_composite(link_name)

    return {
        "status": "success",
        "message": "All joints cleared.",
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/fk")
def compute_forward_kinematics(req: FKRequest) -> Dict[str, Any]:
    """Compute real-time forward kinematics for all links given active joint positions."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    fk_engine = ForwardKinematicsEngine(state.robot)
    result = fk_engine.evaluate(joint_values=req.joint_positions)
    res_dict = result.to_dict()

    # Compute world poses for all individual CAD parts so 3D meshes transform accurately
    part_poses: Dict[str, Any] = {}
    if state.root_node is not None:
        for part in state.root_node.get_all_parts():
            p_mat = part.global_transform.matrix.copy()
            p_mat[:3, 3] = part.global_transform.translation * state.unit_scale
            part_poses[part.name] = Transform3D(matrix=p_mat).to_dict()

    for link_name, link in state.robot.links.items():
        if link_name in result.link_poses_world:
            link_world = result.link_poses_world[link_name]
            for vis in link.visuals:
                if vis.mesh_filepath:
                    p_name = Path(vis.mesh_filepath).stem
                    t_part_world = link_world @ vis.origin
                    part_poses[p_name] = t_part_world.to_dict()

    res_dict["part_poses"] = part_poses
    return res_dict


@app.get("/api/robot/urdf")
def get_urdf_xml():
    """Return the generated URDF XML text with proper application/xml media type."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")
    return Response(content=state.robot.to_urdf_xml(), media_type="application/xml")


@app.post("/api/robot/urdf/update")
def update_urdf_xml(req: UpdateURDFRequest) -> Dict[str, Any]:
    """Parse modified URDF XML, update state.robot links & joints, and return updated model."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    xml_str = req.urdf_xml.strip()
    try:
        root = ET.fromstring(xml_str)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"URDF XML 解析失败: {str(e)}")

    if root.tag != "robot":
        raise HTTPException(status_code=400, detail="根节点必须为 <robot>")

    if "name" in root.attrib and root.attrib["name"].strip():
        state.robot.name = root.attrib["name"].strip()

    # Parse all joints from XML
    for joint_elem in root.findall("joint"):
        j_name = joint_elem.attrib.get("name")
        if not j_name:
            continue
        j_type_str = joint_elem.attrib.get("type", "revolute").lower()
        try:
            j_type = JointType(j_type_str)
        except ValueError:
            j_type = JointType.REVOLUTE

        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        if parent_elem is None or child_elem is None:
            continue
        p_link = parent_elem.attrib.get("link", "").strip()
        c_link = child_elem.attrib.get("link", "").strip()

        # Origin
        origin_elem = joint_elem.find("origin")
        origin_xyz = [0.0, 0.0, 0.0]
        origin_rpy = [0.0, 0.0, 0.0]
        if origin_elem is not None:
            if "xyz" in origin_elem.attrib:
                try:
                    origin_xyz = [float(v) for v in origin_elem.attrib["xyz"].split()]
                except Exception:
                    pass
            if "rpy" in origin_elem.attrib:
                try:
                    origin_rpy = [float(v) for v in origin_elem.attrib["rpy"].split()]
                except Exception:
                    pass
        origin_t = Transform3D.from_rpy_xyz(origin_rpy, origin_xyz)

        # Axis
        axis_elem = joint_elem.find("axis")
        axis = [0.0, 0.0, 1.0]
        if axis_elem is not None and "xyz" in axis_elem.attrib:
            try:
                axis = [float(v) for v in axis_elem.attrib["xyz"].split()]
            except Exception:
                pass

        # Limits
        limit_elem = joint_elem.find("limit")
        lower = -3.14159
        upper = 3.14159
        effort = 100.0
        velocity = 3.14159
        if limit_elem is not None:
            try:
                if "lower" in limit_elem.attrib:
                    lower = float(limit_elem.attrib["lower"])
                if "upper" in limit_elem.attrib:
                    upper = float(limit_elem.attrib["upper"])
                if "effort" in limit_elem.attrib:
                    effort = float(limit_elem.attrib["effort"])
                if "velocity" in limit_elem.attrib:
                    velocity = float(limit_elem.attrib["velocity"])
            except Exception:
                pass
        limits = JointLimits(lower=lower, upper=upper, effort=effort, velocity=velocity)

        # Ensure links exist
        if p_link and p_link not in state.robot.links:
            state.robot.add_link(LinkModel(name=p_link, cad_part_ids=[p_link]))
        if c_link and c_link not in state.robot.links:
            state.robot.add_link(LinkModel(name=c_link, cad_part_ids=[c_link]))

        if j_name in state.robot.joints:
            j_model = state.robot.joints[j_name]
            j_model.parent_link = p_link
            j_model.child_link = c_link
            j_model.joint_type = j_type
            j_model.origin = origin_t
            j_model.axis = tuple(axis)
            j_model.limits = limits
        else:
            j_model = JointModel(
                name=j_name,
                parent_link=p_link,
                child_link=c_link,
                joint_type=j_type,
                origin=origin_t,
                axis=tuple(axis),
                limits=limits,
            )
            state.robot.add_joint(j_model)

    # Parse link inertials from XML if present
    for link_elem in root.findall("link"):
        l_name = link_elem.attrib.get("name")
        if not l_name:
            continue
        if l_name not in state.robot.links:
            state.robot.add_link(LinkModel(name=l_name, cad_part_ids=[l_name]))
        link = state.robot.links[l_name]

        inertial_elem = link_elem.find("inertial")
        if inertial_elem is not None:
            mass_val = 0.0
            mass_elem = inertial_elem.find("mass")
            if mass_elem is not None and "value" in mass_elem.attrib:
                try:
                    mass_val = float(mass_elem.attrib["value"])
                except Exception:
                    pass

            com_xyz = [0.0, 0.0, 0.0]
            origin_elem = inertial_elem.find("origin")
            if origin_elem is not None and "xyz" in origin_elem.attrib:
                try:
                    com_xyz = [float(v) for v in origin_elem.attrib["xyz"].split()]
                except Exception:
                    pass

            ixx = iyy = izz = ixy = ixz = iyz = 0.0
            inertia_elem = inertial_elem.find("inertia")
            if inertia_elem is not None:
                try:
                    ixx = float(inertia_elem.attrib.get("ixx", 0.0))
                    iyy = float(inertia_elem.attrib.get("iyy", 0.0))
                    izz = float(inertia_elem.attrib.get("izz", 0.0))
                    ixy = float(inertia_elem.attrib.get("ixy", 0.0))
                    ixz = float(inertia_elem.attrib.get("ixz", 0.0))
                    iyz = float(inertia_elem.attrib.get("iyz", 0.0))
                except Exception:
                    pass

            from urdforge.core.physics import InertiaTensor, PhysicalProperties
            tensor = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
            val = tensor.validate(mass=mass_val)
            props = PhysicalProperties(
                volume_m3=0.0,
                volume_mm3=0.0,
                density=state.default_density,
                mass=mass_val,
                com_local=tuple(com_xyz),
                com_global=tuple(com_xyz),
                inertia_tensor=tensor,
                validation=val,
            )
            link.inertial = props
            link.has_manual_inertial = True

    # Recalculate kinematics and descendants
    for l_name in state.robot.links:
        recalculate_link_composite(l_name)

    return {
        "status": "success",
        "message": "URDF 机器人模型已更新并重新计算运动学与物性。",
        "robot_model": state.robot.to_dict(),
        "urdf_xml": state.robot.to_urdf_xml(),
    }


@app.post("/api/robot/link/override_physics")
def override_link_physics(req: OverridePhysicsRequest) -> Dict[str, Any]:
    """Manually override composite mass, CoM, and 3x3 inertia tensor for a link."""
    if state.robot is None or req.link_name not in state.robot.links:
        raise HTTPException(status_code=404, detail=f"Link '{req.link_name}' not found.")

    link = state.robot.links[req.link_name]
    if req.reset_auto:
        link.has_manual_inertial = False
        recalculate_link_composite(req.link_name)
    else:
        from urdforge.core.physics import InertiaTensor, PhysicalProperties
        tensor = InertiaTensor(
            ixx=req.ixx,
            iyy=req.iyy,
            izz=req.izz,
            ixy=req.ixy,
            ixz=req.ixz,
            iyz=req.iyz,
        )
        val = tensor.validate(mass=req.mass)
        props = PhysicalProperties(
            volume_m3=0.0,
            volume_mm3=0.0,
            density=state.default_density,
            mass=req.mass,
            com_local=tuple(req.com[:3]),
            com_global=tuple(req.com[:3]),
            inertia_tensor=tensor,
            validation=val,
        )
        link.inertial = props
        link.has_manual_inertial = True

    return {
        "status": "success",
        "message": f"Link '{req.link_name}' 物性已更新。",
        "link": link.to_dict(),
        "robot_model": state.robot.to_dict(),
    }


@app.post("/api/robot/recalculate_all_physics")
def recalculate_all_physics(req: Optional[RecalculateAllPhysicsRequest] = None) -> Dict[str, Any]:
    """Force exact recomputation of composite physical properties (Mass, CoM, 3x3 Inertia Tensor) for all links."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="未初始化机器人模型。")

    force_reset = req.force_reset if req is not None else True
    if force_reset:
        for link in state.robot.links.values():
            link.has_manual_inertial = False

    for l_name in state.robot.links:
        recalculate_link_composite(l_name)

    return {
        "status": "success",
        "message": "已成功基于连杆局部几何与CAD部件精确重新计算全部连杆的质心与3×3惯量张量。",
        "robot_model": state.robot.to_dict(),
        "urdf_xml": state.robot.to_urdf_xml(),
    }



@app.post("/api/robot/export_ros")
def export_ros_package(req: ExportROSRequest) -> Dict[str, Any]:
    """Export the complete ROS 2 description package to disk."""
    if state.robot is None:
        raise HTTPException(status_code=400, detail="No robot model defined.")

    pkg_name = req.package_name or f"{state.robot.name}_description"
    cfg = ROSPackageConfig(
        package_name=pkg_name,
        robot_name=state.robot.name,
        use_package_uri=req.use_package_uri,
    )
    exporter = ROSPackageExporter(robot=state.robot, config=cfg)
    target_dir = Path(req.target_directory).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)

    out_pkg_path = exporter.export(
        target_directory=target_dir,
        exported_mesh_paths=state.exported_meshes,
    )

    return {
        "status": "success",
        "package_name": pkg_name,
        "package_directory": str(out_pkg_path),
        "files_generated": [
            str(p.relative_to(out_pkg_path)) for p in out_pkg_path.rglob("*") if p.is_file()
        ],
    }


# Static web app route
@app.get("/", response_class=HTMLResponse)
def serve_index():
    index_file = STATIC_DIR / "index.html"
    if not index_file.exists():
        return HTMLResponse("<h2>URDForge Studio static UI is loading...</h2>")
    with open(index_file, "r", encoding="utf-8") as f:
        return HTMLResponse(f.read())
