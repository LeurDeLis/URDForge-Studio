"""URDForge Robot Kinematic Data Model.

Defines Links, Joints, Inertial, Visual, Collision, and Robot aggregate models
conforming to ROS / ROS 2 and simulation engine specifications.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from urdforge.core.math_types import Transform3D
from urdforge.core.physics import InertiaTensor, PhysicalProperties
from urdforge.utils.naming import sanitize_urdf_name


class JointType(Enum):
    """URDF joint kinematic classification."""

    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    FLOATING = "floating"
    PLANAR = "planar"


@dataclass
class JointLimits:
    """Joint mechanical limits."""

    lower: float = -3.14159265
    upper: float = 3.14159265
    effort: float = 100.0  # N*m (revolute) or N (prismatic)
    velocity: float = 3.14159265  # rad/s or m/s

    def to_dict(self) -> Dict[str, float]:
        return {
            "lower": self.lower,
            "upper": self.upper,
            "effort": self.effort,
            "velocity": self.velocity,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> JointLimits:
        low = d.get("lower")
        up = d.get("upper")
        eff = d.get("effort")
        vel = d.get("velocity")
        return cls(
            lower=float(low) if low is not None else -3.14159265,
            upper=float(up) if up is not None else 3.14159265,
            effort=float(eff) if eff is not None else 100.0,
            velocity=float(vel) if vel is not None else 3.14159265,
        )


@dataclass
class JointDynamics:
    """Joint dynamics parameters (friction, damping)."""

    damping: float = 0.1
    friction: float = 0.0

    def to_dict(self) -> Dict[str, float]:
        return {"damping": self.damping, "friction": self.friction}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> JointDynamics:
        return cls(
            damping=float(d.get("damping", 0.1)),
            friction=float(d.get("friction", 0.0)),
        )


@dataclass
class JointModel:
    """Robotic joint connecting two links with 1-DOF or fixed constraint."""

    name: str
    parent_link: str
    child_link: str
    joint_type: JointType = JointType.REVOLUTE
    origin: Transform3D = field(default_factory=Transform3D.identity)
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    limits: JointLimits = field(default_factory=JointLimits)
    dynamics: JointDynamics = field(default_factory=JointDynamics)
    current_value: float = 0.0  # Current angle/displacement for FK debugging

    def __post_init__(self) -> None:
        self.name = sanitize_urdf_name(self.name, fallback="joint")
        self.parent_link = sanitize_urdf_name(self.parent_link, fallback="parent")
        self.child_link = sanitize_urdf_name(self.child_link, fallback="child")
        # Normalize axis vector
        ax = np.asarray(self.axis, dtype=np.float64)
        norm = np.linalg.norm(ax)
        if norm > 1e-9:
            ax = ax / norm
            self.axis = (float(ax[0]), float(ax[1]), float(ax[2]))
        else:
            self.axis = (0.0, 0.0, 1.0)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "type": self.joint_type.value,
            "parent_link": self.parent_link,
            "child_link": self.child_link,
            "origin": self.origin.to_dict(),
            "axis": list(self.axis),
            "limits": self.limits.to_dict(),
            "dynamics": self.dynamics.to_dict(),
            "current_value": self.current_value,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> JointModel:
        j_type = JointType(d.get("type", "revolute"))
        orig_data = d.get("origin", {})
        if "matrix" in orig_data:
            orig = Transform3D(matrix=np.array(orig_data["matrix"]))
        elif "xyz" in orig_data and "rpy_rad" in orig_data:
            orig = Transform3D.from_rpy_xyz(orig_data["rpy_rad"], orig_data["xyz"])
        else:
            orig = Transform3D.identity()

        return cls(
            name=d["name"],
            parent_link=d["parent_link"],
            child_link=d["child_link"],
            joint_type=j_type,
            origin=orig,
            axis=tuple(d.get("axis", (0.0, 0.0, 1.0))),
            limits=JointLimits.from_dict(d.get("limits", {})),
            dynamics=JointDynamics.from_dict(d.get("dynamics", {})),
            current_value=float(d.get("current_value", 0.0)),
        )

    def to_urdf_xml(self, indent: int = 2) -> str:
        """Generate XML string for this joint."""
        sp = " " * indent
        sp_in = " " * (indent + 2)
        lines = [
            f'{sp}<joint name="{self.name}" type="{self.joint_type.value}">',
            f'{sp_in}<parent link="{self.parent_link}"/>',
            f'{sp_in}<child link="{self.child_link}"/>',
            self.origin.to_urdf_origin_xml(indent=indent + 2),
        ]

        if self.joint_type != JointType.FIXED:
            ax = self.axis
            lines.append(f'{sp_in}<axis xyz="{ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f}"/>')
            if self.joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
                lim = self.limits
                if self.joint_type == JointType.PRISMATIC:
                    # Support up to 7 decimal places (0.0000001 m / 0.1 µm) for high-precision prismatic linear rails
                    lines.append(
                        f'{sp_in}<limit lower="{lim.lower:.7f}" upper="{lim.upper:.7f}" '
                        f'effort="{lim.effort:.2f}" velocity="{lim.velocity:.4f}"/>'
                    )
                else:
                    lines.append(
                        f'{sp_in}<limit lower="{lim.lower:.6f}" upper="{lim.upper:.6f}" '
                        f'effort="{lim.effort:.2f}" velocity="{lim.velocity:.4f}"/>'
                    )
            dyn = self.dynamics
            if dyn.damping > 0.0 or dyn.friction > 0.0:
                lines.append(
                    f'{sp_in}<dynamics damping="{dyn.damping:.4f}" friction="{dyn.friction:.4f}"/>'
                )

        lines.append(f"{sp}</joint>")
        return "\n".join(lines)


@dataclass
class VisualGeometry:
    """Visual geometry specification (mesh file or primitive)."""

    mesh_filepath: Optional[str] = None
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    origin: Transform3D = field(default_factory=Transform3D.identity)
    color_rgba: Optional[Tuple[float, float, float, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mesh_filepath": self.mesh_filepath,
            "scale": list(self.scale),
            "origin": self.origin.to_dict(),
            "color_rgba": list(self.color_rgba) if self.color_rgba else None,
        }


@dataclass
class LinkModel:
    """Rigid body link in the kinematic tree."""

    name: str
    cad_part_ids: List[str] = field(default_factory=list)
    inertial: Optional[PhysicalProperties] = None
    visuals: List[VisualGeometry] = field(default_factory=list)
    collisions: List[VisualGeometry] = field(default_factory=list)
    has_manual_inertial: bool = False

    def __post_init__(self) -> None:
        self.name = sanitize_urdf_name(self.name, fallback="link")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "cad_part_ids": self.cad_part_ids,
            "inertial": self.inertial.to_dict() if self.inertial else None,
            "visuals": [v.to_dict() for v in self.visuals],
            "collisions": [c.to_dict() for c in self.collisions],
            "has_manual_inertial": self.has_manual_inertial,
        }

    def to_urdf_xml(self, indent: int = 2, package_name: Optional[str] = None) -> str:
        """Generate XML string for this link."""
        sp = " " * indent
        sp_in = " " * (indent + 2)
        sp_geom = " " * (indent + 4)

        lines = [f'{sp}<link name="{self.name}">']

        # Inertial
        if self.inertial is not None:
            lines.append(self.inertial.to_urdf_inertial_xml(indent=indent + 2))

        # Visuals
        for vis in self.visuals:
            lines.append(f"{sp_in}<visual>")
            lines.append(vis.origin.to_urdf_origin_xml(indent=indent + 4))
            lines.append(f"{sp_geom}<geometry>")
            if vis.mesh_filepath:
                sx, sy, sz = vis.scale
                fn = vis.mesh_filepath.replace("\\", "/")
                if fn.startswith("package://"):
                    if package_name:
                        parts = fn.split("/", 3)
                        if len(parts) >= 4:
                            fn = f"package://{package_name}/{parts[3]}"
                elif not fn.startswith("file://"):
                    file_name = Path(fn).name
                    if "meshes/visual" not in fn:
                        fn = f"meshes/visual/{file_name}"
                    if package_name:
                        fn = f"package://{package_name}/{fn}"
                lines.append(f'{sp_geom}  <mesh filename="{fn}" scale="{sx:.4f} {sy:.4f} {sz:.4f}"/>')
            lines.append(f"{sp_geom}</geometry>")
            if vis.color_rgba is not None:
                r, g, b, a = vis.color_rgba
                lines.append(f'{sp_geom}<material name="mat_{self.name}">')
                lines.append(f'{sp_geom}  <color rgba="{r:.4f} {g:.4f} {b:.4f} {a:.4f}"/>')
                lines.append(f"{sp_geom}</material>")
            lines.append(f"{sp_in}</visual>")

        # Collisions
        for col in self.collisions:
            lines.append(f"{sp_in}<collision>")
            lines.append(col.origin.to_urdf_origin_xml(indent=indent + 4))
            lines.append(f"{sp_geom}<geometry>")
            if col.mesh_filepath:
                sx, sy, sz = col.scale
                fn = col.mesh_filepath.replace("\\", "/")
                if fn.startswith("package://"):
                    if package_name:
                        parts = fn.split("/", 3)
                        if len(parts) >= 4:
                            fn = f"package://{package_name}/{parts[3]}"
                elif not fn.startswith("file://"):
                    file_name = Path(fn).name
                    if "meshes/collision" not in fn:
                        fn = f"meshes/collision/{file_name}"
                    if package_name:
                        fn = f"package://{package_name}/{fn}"
                lines.append(f'{sp_geom}  <mesh filename="{fn}" scale="{sx:.4f} {sy:.4f} {sz:.4f}"/>')
            lines.append(f"{sp_geom}</geometry>")
            lines.append(f"{sp_in}</collision>")

        lines.append(f"{sp}</link>")
        return "\n".join(lines)


@dataclass
class RobotModel:
    """Complete robot model containing the kinematic tree of links and joints."""

    name: str = "urdforge_robot"
    root_link: str = "base_link"
    links: Dict[str, LinkModel] = field(default_factory=dict)
    joints: Dict[str, JointModel] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.name = sanitize_urdf_name(self.name, fallback="robot")
        if self.root_link not in self.links:
            # Ensure base_link exists
            self.links[self.root_link] = LinkModel(name=self.root_link)

    def add_link(self, link: LinkModel) -> None:
        """Add or update a link."""
        self.links[link.name] = link

    def add_joint(self, joint: JointModel) -> None:
        """Add or update a joint."""
        self.joints[joint.name] = joint

    def get_child_joints(self, link_name: str) -> List[JointModel]:
        """Find all joints that have link_name as parent."""
        return [j for j in self.joints.values() if j.parent_link == link_name]

    def get_parent_joint(self, link_name: str) -> Optional[JointModel]:
        """Find the single joint that has link_name as child."""
        for j in self.joints.values():
            if j.child_link == link_name:
                return j
        return None

    def get_reachable_links(self) -> List[str]:
        """Find all link names reachable from root_link via joints in topological order."""
        reachable: List[str] = []
        if self.root_link not in self.links:
            return reachable
        visited: set = {self.root_link}
        queue = [self.root_link]
        while queue:
            curr = queue.pop(0)
            reachable.append(curr)
            for j in self.get_child_joints(curr):
                if j.child_link in self.links and j.child_link not in visited:
                    visited.add(j.child_link)
                    queue.append(j.child_link)
        return reachable

    def get_orphan_links(self) -> List[str]:
        """Return names of all links not reachable from root_link (orphaned/disconnected)."""
        reachable_set = set(self.get_reachable_links())
        return [name for name in self.links if name not in reachable_set]

    def simulate_orphans_if_joint_removed(self, joint_name: str) -> List[str]:
        """Return which currently reachable links would become disconnected from root_link if joint_name is removed."""
        if joint_name not in self.joints:
            return []
        curr_reachable = self.get_reachable_links()
        
        # Build hypothetical graph without joint_name
        visited: set = set()
        if self.root_link in self.links:
            visited.add(self.root_link)
            queue = [self.root_link]
            while queue:
                curr = queue.pop(0)
                for j in self.joints.values():
                    if j.name != joint_name and j.parent_link == curr:
                        if j.child_link in self.links and j.child_link not in visited:
                            visited.add(j.child_link)
                            queue.append(j.child_link)
        return [l for l in curr_reachable if l not in visited]

    def simulate_orphans_if_link_removed(self, link_name: str) -> List[str]:
        """Return which downstream links would become disconnected from root_link if link_name is removed."""
        if link_name not in self.links or link_name == self.root_link:
            return []
        curr_reachable = self.get_reachable_links()
        
        visited: set = set()
        if self.root_link in self.links:
            visited.add(self.root_link)
            queue = [self.root_link]
            while queue:
                curr = queue.pop(0)
                for j in self.joints.values():
                    if j.parent_link == curr and j.parent_link != link_name and j.child_link != link_name:
                        if j.child_link in self.links and j.child_link not in visited:
                            visited.add(j.child_link)
                            queue.append(j.child_link)
        return [l for l in curr_reachable if l != link_name and l not in visited]

    def to_urdf_xml(self, package_name: Optional[str] = None) -> str:
        """Serialize complete robot to URDF XML."""
        lines = [
            '<?xml version="1.0" encoding="utf-8"?>',
            '<!-- =================================================================================== -->',
            '<!-- |    Generated by URDForge Studio                                                 | -->',
            '<!-- =================================================================================== -->',
            f'<robot name="{self.name}">',
        ]

        # Order links starting from root
        visited_links: set = set()
        ordered_links: List[str] = []

        def visit(link_name: str):
            if link_name in visited_links or link_name not in self.links:
                return
            visited_links.add(link_name)
            ordered_links.append(link_name)
            for j in self.get_child_joints(link_name):
                visit(j.child_link)

        visit(self.root_link)
        # Any orphan links
        for name in self.links:
            if name not in visited_links:
                ordered_links.append(name)

        # Write Links
        for l_name in ordered_links:
            lines.append(f"\n  <!-- Link: {l_name} -->")
            lines.append(self.links[l_name].to_urdf_xml(indent=2, package_name=package_name))

        # Write Joints
        for j in self.joints.values():
            lines.append(f"\n  <!-- Joint: {j.name} -->")
            lines.append(j.to_urdf_xml(indent=2))

        lines.append("\n</robot>\n")
        return "\n".join(lines)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "root_link": self.root_link,
            "links": {k: v.to_dict() for k, v in self.links.items()},
            "joints": {k: v.to_dict() for k, v in self.joints.items()},
            "orphan_links": self.get_orphan_links(),
        }
