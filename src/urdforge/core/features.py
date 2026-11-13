"""URDForge CAD B-Rep Feature Snapping & Joint Alignment Engine.

Extracts analytical cylindrical axes, circular hole edges, and planar normals directly
from OpenCASCADE B-Rep topology. Automatically computes SE(3) Joint origins and
motion axis orientations (aligning Joint Z-axis to the feature axis).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Circle, GeomAbs_Cylinder, GeomAbs_Plane
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape

from urdforge.core.math_types import Transform3D
from urdforge.core.robot_model import JointLimits, JointModel, JointType


class FeatureType(Enum):
    """Geometric feature classification."""

    CYLINDER_AXIS = "cylinder_axis"
    CIRCLE_EDGE = "circle_edge"
    PLANAR_FACE = "planar_face"


def align_z_to_vector(target_vec: Sequence[float]) -> Transform3D:
    """Compute rotation matrix R that rotates standard Z-axis [0, 0, 1] into target_vec.

    Uses Rodrigues' rotation formula.
    """
    n = np.asarray(target_vec, dtype=np.float64)
    norm = np.linalg.norm(n)
    if norm < 1e-9:
        return Transform3D.identity()
    n = n / norm

    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    dot = float(np.dot(z, n))

    if np.isclose(dot, 1.0, atol=1e-7):
        # Already aligned with Z
        return Transform3D.identity()
    elif np.isclose(dot, -1.0, atol=1e-7):
        # 180 degrees rotation about X axis
        rot = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot
        return Transform3D(matrix=mat)
    else:
        v = np.cross(z, n)
        v = v / np.linalg.norm(v)
        theta = math.acos(max(-1.0, min(1.0, dot)))

        # Skew symmetric matrix
        vx = np.array(
            [
                [0.0, -v[2], v[1]],
                [v[2], 0.0, -v[0]],
                [-v[1], v[0], 0.0],
            ],
            dtype=np.float64,
        )

        rot = np.eye(3, dtype=np.float64) + math.sin(theta) * vx + (1.0 - math.cos(theta)) * np.matmul(vx, vx)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot
        return Transform3D(matrix=mat)


@dataclass
class GeometricFeature:
    """Analytical geometric feature on a CAD part for joint alignment."""

    id: str
    feature_type: FeatureType
    part_name: str
    center: Tuple[float, float, float]  # In meters (default midpoint for cylinder)
    direction: Tuple[float, float, float]  # Normalized axis / normal vector
    radius: float = 0.0  # In meters (for circle/cylinder)
    length: float = 0.0  # In meters (for cylinder)
    center_mid: Optional[Tuple[float, float, float]] = None  # Midpoint along cylinder axis
    center_top: Optional[Tuple[float, float, float]] = None  # Top circle center
    center_bot: Optional[Tuple[float, float, float]] = None  # Bottom circle center

    def __post_init__(self) -> None:
        if self.center_mid is None:
            self.center_mid = self.center
        if self.center_top is None:
            self.center_top = self.center
        if self.center_bot is None:
            self.center_bot = self.center

    @property
    def align_transform(self) -> Transform3D:
        """Transformation with translation at center and rotation aligning Z to direction."""
        rot_t = align_z_to_vector(self.direction)
        mat = rot_t.matrix.copy()
        mat[:3, 3] = self.center_mid or self.center
        return Transform3D(matrix=mat)

    def to_dict(self) -> Dict[str, Any]:
        t = self.align_transform
        r, p, y = t.to_rpy()
        qx, qy, qz, qw = t.to_quaternion()
        c_mid = list(self.center_mid or self.center)
        c_top = list(self.center_top or self.center)
        c_bot = list(self.center_bot or self.center)
        return {
            "id": self.id,
            "type": self.feature_type.value,
            "part_name": self.part_name,
            "center": list(self.center),
            "center_mid": c_mid,
            "center_top": c_top,
            "center_bot": c_bot,
            "direction": list(self.direction),
            "radius_m": self.radius,
            "radius_mm": self.radius * 1000.0,
            "length_m": self.length,
            "length_mm": self.length * 1000.0,
            "origin_xyz": c_mid,
            "origin_rpy": [r, p, y],
            "origin_quaternion": [qx, qy, qz, qw],
        }

    def snap_joint(
        self,
        joint_name: str,
        parent_link: str,
        child_link: str,
        joint_type: JointType = JointType.REVOLUTE,
        flip_axis: bool = False,
        snap_point_type: str = "midpoint",
    ) -> JointModel:
        """Create a joint model snapped to this geometric feature.

        Args:
            joint_name: Name of the created joint.
            parent_link: Parent link name.
            child_link: Child link name.
            joint_type: Kinematic joint type.
            flip_axis: Invert rotation Z axis direction (180 deg).
            snap_point_type: 'midpoint' (geometric cylinder center), 'top' (top end face),
                             or 'bottom' (bottom end face).
        """
        direction = np.array(self.direction, dtype=np.float64)
        if flip_axis:
            direction = -direction

        rot_t = align_z_to_vector(direction)
        origin_mat = rot_t.matrix.copy()

        if snap_point_type == "top" and self.center_top is not None:
            chosen_center = self.center_top
        elif snap_point_type == "bottom" and self.center_bot is not None:
            chosen_center = self.center_bot
        else:
            chosen_center = self.center_mid or self.center

        origin_mat[:3, 3] = chosen_center

        origin = Transform3D(matrix=origin_mat)
        return JointModel(
            name=joint_name,
            parent_link=parent_link,
            child_link=child_link,
            joint_type=joint_type,
            origin=origin,
            axis=(0.0, 0.0, 1.0),  # Standardized Z-axis
            limits=JointLimits(),
        )


class FeatureDetector:
    """Scans OpenCASCADE shapes to extract all candidate joint alignment features."""

    def __init__(self, unit_scale: float = 0.001) -> None:
        self.unit_scale = unit_scale

    def detect_features(
        self,
        shape: TopoDS_Shape,
        part_name: str,
        global_transform: Optional[Transform3D] = None,
    ) -> List[GeometricFeature]:
        """Detect all cylindrical axes and circular hole edges on a shape.

        Args:
            shape: TopoDS_Shape to inspect.
            part_name: Name of the part.
            global_transform: Transform to convert from local CAD part coordinates to global coordinates.
        """
        if shape.IsNull():
            return []

        features: List[GeometricFeature] = []
        feature_idx = 0

        # 1. Detect Cylindrical Faces
        face_exp = TopExp_Explorer(shape, TopAbs_FACE)
        while face_exp.More():
            face = TopoDS.Face_s(face_exp.Current())
            try:
                surf_adaptor = BRepAdaptor_Surface(face)
                if surf_adaptor.GetType() == GeomAbs_Cylinder:
                    feature_idx += 1
                    cyl = surf_adaptor.Cylinder()
                    loc = cyl.Location()
                    axis = cyl.Axis().Direction()
                    radius = float(cyl.Radius()) * self.unit_scale

                    d_local = np.array([axis.X(), axis.Y(), axis.Z()], dtype=np.float64)
                    d_local = d_local / np.linalg.norm(d_local)

                    # Compute geometric midpoint and end centers along cylinder axis (in CAD coordinates)
                    vmin = surf_adaptor.FirstVParameter()
                    vmax = surf_adaptor.LastVParameter()

                    p_loc_cad = np.array([loc.X(), loc.Y(), loc.Z()], dtype=np.float64)
                    if math.isfinite(vmin) and math.isfinite(vmax) and abs(vmax - vmin) < 1e6:
                        vmid = (vmin + vmax) / 2.0
                        cyl_length = abs(vmax - vmin) * self.unit_scale
                        p_mid_loc_cad = p_loc_cad + d_local * vmid
                        p_bot_loc_cad = p_loc_cad + d_local * vmin
                        p_top_loc_cad = p_loc_cad + d_local * vmax
                    else:
                        vmid = 0.0
                        cyl_length = 0.0
                        p_mid_loc_cad = p_loc_cad
                        p_bot_loc_cad = p_loc_cad
                        p_top_loc_cad = p_loc_cad

                    if global_transform is not None:
                        p_mid_world = global_transform.transform_point(p_mid_loc_cad) * self.unit_scale
                        p_bot_world = global_transform.transform_point(p_bot_loc_cad) * self.unit_scale
                        p_top_world = global_transform.transform_point(p_top_loc_cad) * self.unit_scale
                        d_world = np.matmul(global_transform.rotation_matrix, d_local)
                    else:
                        p_mid_world = p_mid_loc_cad * self.unit_scale
                        p_bot_world = p_bot_loc_cad * self.unit_scale
                        p_top_world = p_top_loc_cad * self.unit_scale
                        d_world = d_local

                    d_world = d_world / np.linalg.norm(d_world)
                    feat = GeometricFeature(
                        id=f"{part_name}_cyl_{feature_idx}",
                        feature_type=FeatureType.CYLINDER_AXIS,
                        part_name=part_name,
                        center=(float(p_mid_world[0]), float(p_mid_world[1]), float(p_mid_world[2])),
                        direction=(float(d_world[0]), float(d_world[1]), float(d_world[2])),
                        radius=radius,
                        length=cyl_length,
                        center_mid=(float(p_mid_world[0]), float(p_mid_world[1]), float(p_mid_world[2])),
                        center_bot=(float(p_bot_world[0]), float(p_bot_world[1]), float(p_bot_world[2])),
                        center_top=(float(p_top_world[0]), float(p_top_world[1]), float(p_top_world[2])),
                    )
                    features.append(feat)
            except Exception:
                pass
            face_exp.Next()

        # 2. Detect Circular Edges (Holes / Rim Edges)
        edge_exp = TopExp_Explorer(shape, TopAbs_EDGE)
        seen_centers: List[np.ndarray] = []

        while edge_exp.More():
            edge = TopoDS.Edge_s(edge_exp.Current())
            try:
                curve_adaptor = BRepAdaptor_Curve(edge)
                if curve_adaptor.GetType() == GeomAbs_Circle:
                    circ = curve_adaptor.Circle()
                    c_loc = circ.Location()
                    c_axis = circ.Axis().Direction()
                    radius = float(circ.Radius()) * self.unit_scale

                    p_loc_cad = np.array([c_loc.X(), c_loc.Y(), c_loc.Z()], dtype=np.float64)
                    d_local = np.array([c_axis.X(), c_axis.Y(), c_axis.Z()], dtype=np.float64)
                    d_local = d_local / np.linalg.norm(d_local)

                    if global_transform is not None:
                        p_world = global_transform.transform_point(p_loc_cad) * self.unit_scale
                        d_world = np.matmul(global_transform.rotation_matrix, d_local)
                    else:
                        p_world = p_loc_cad * self.unit_scale
                        d_world = d_local

                    # Deduplicate overlapping concentric edge circles
                    is_dup = False
                    for seen in seen_centers:
                        if np.linalg.norm(p_world - seen) < 1e-4:
                            is_dup = True
                            break
                    if not is_dup:
                        feature_idx += 1
                        seen_centers.append(p_world)
                        d_world = d_world / np.linalg.norm(d_world)
                        feat = GeometricFeature(
                            id=f"{part_name}_hole_{feature_idx}",
                            feature_type=FeatureType.CIRCLE_EDGE,
                            part_name=part_name,
                            center=(float(p_world[0]), float(p_world[1]), float(p_world[2])),
                            direction=(float(d_world[0]), float(d_world[1]), float(d_world[2])),
                            radius=radius,
                        )
                        features.append(feat)
            except Exception:
                pass
            edge_exp.Next()

        return features

    def find_nearest_feature(
        self,
        features: Sequence[GeometricFeature],
        query_point: Sequence[float],
        max_distance: float = 0.1,  # 10 cm search radius
    ) -> Optional[GeometricFeature]:
        """Find the closest feature to a 3D query point (e.g. from mouse click raycast)."""
        qp = np.asarray(query_point, dtype=np.float64)
        best_feat = None
        best_dist = float("inf")

        for f in features:
            dist = float(np.linalg.norm(np.asarray(f.center) - qp))
            if dist < best_dist and dist <= max_distance:
                best_dist = dist
                best_feat = f

        return best_feat
