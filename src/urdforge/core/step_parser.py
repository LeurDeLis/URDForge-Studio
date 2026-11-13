"""URDForge OpenCASCADE STEP Assembly Parser.

Parses industrial standard STEP files (AP214 / AP242) via OpenCASCADE XCAF framework.
Extracts product structure, part hierarchies, names, local/global SE(3) transformation matrices,
computes mass properties, and manages visual mesh tessellation.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_ColorRGBA
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence, TDF_Tool
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_COMPOUND, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import (
    XCAFDoc_ColorGen,
    XCAFDoc_ColorSurf,
    XCAFDoc_ColorTool,
    XCAFDoc_ColorType,
    XCAFDoc_DocumentTool,
    XCAFDoc_ShapeTool,
)

from urdforge.core.math_types import Transform3D
from urdforge.core.mesher import MeshExportResult, TessellationConfig, export_shape_to_stl
from urdforge.core.physics import PhysicalProperties, compute_shape_physical_properties
from urdforge.utils.naming import UniqueNameGenerator, sanitize_urdf_name


class AssemblyNodeType(Enum):
    """Classification of nodes within the CAD assembly graph."""

    ROOT = "root"
    ASSEMBLY = "assembly"
    PART = "part"
    COMPOUND = "compound"


@dataclass
class AssemblyNode:
    """Node in the hierarchical assembly tree representing an assembly or part."""

    id: str
    name: str
    original_name: str
    node_type: AssemblyNodeType
    local_transform: Transform3D = field(default_factory=Transform3D.identity)
    global_transform: Transform3D = field(default_factory=Transform3D.identity)
    raw_global_transform: Transform3D = field(default_factory=Transform3D.identity)
    shape: Optional[TopoDS_Shape] = None
    instance_name: str = ""
    definition_name: str = ""
    label_entry: str = ""
    color_rgba: Optional[Tuple[float, float, float, float]] = None
    bounding_box_local: Optional[Tuple[float, float, float, float, float, float]] = None
    bounding_box_global: Optional[Tuple[float, float, float, float, float, float]] = None
    children: List[AssemblyNode] = field(default_factory=list)
    physical_props: Optional[PhysicalProperties] = None

    @property
    def is_leaf(self) -> bool:
        """True if the node has no children (is a leaf part)."""
        return len(self.children) == 0 or self.node_type == AssemblyNodeType.PART

    def get_all_parts(self) -> List[AssemblyNode]:
        """Recursively collect all leaf parts."""
        parts: List[AssemblyNode] = []
        if self.is_leaf and self.shape is not None and not self.shape.IsNull():
            parts.append(self)
        for child in self.children:
            parts.extend(child.get_all_parts())
        return parts

    def find_node_by_name(self, target_name: str) -> Optional[AssemblyNode]:
        """Search node by sanitized name or original name."""
        if self.name == target_name or self.original_name == target_name:
            return self
        for child in self.children:
            found = child.find_node_by_name(target_name)
            if found is not None:
                return found
        return None

    def find_node_by_id(self, target_id: str) -> Optional[AssemblyNode]:
        """Search node by unique ID."""
        if self.id == target_id:
            return self
        for child in self.children:
            found = child.find_node_by_id(target_id)
            if found is not None:
                return found
        return None

    def to_dict(self) -> Dict[str, Any]:
        """Convert assembly node and sub-tree to a JSON-serializable dictionary."""
        return {
            "id": self.id,
            "name": self.name,
            "original_name": self.original_name,
            "instance_name": self.instance_name,
            "definition_name": self.definition_name,
            "node_type": self.node_type.value,
            "is_leaf": self.is_leaf,
            "label_entry": self.label_entry,
            "local_transform": self.local_transform.to_dict(),
            "global_transform": self.global_transform.to_dict(),
            "color_rgba": list(self.color_rgba) if self.color_rgba else None,
            "bounding_box_local": list(self.bounding_box_local) if self.bounding_box_local else None,
            "bounding_box_global": list(self.bounding_box_global) if self.bounding_box_global else None,
            "physical_props": self.physical_props.to_dict() if self.physical_props else None,
            "children": [c.to_dict() for c in self.children],
        }


def get_cad_up_axis_transform(cad_up_axis: str) -> Transform3D:
    """Return rigid rotation transforming CAD assembly space into ROS Z-up space.

    - 'Z': Identity (CAD already uses Z-up / XY top/ground plane).
    - 'Y': Rotate +90 deg (+pi/2) around +X axis so SolidWorks +Y Up (ZX top plane) -> ROS +Z Up (XY ground plane).
    - '-Y': Rotate -90 deg (-pi/2) around +X axis so CAD -Y Up -> ROS +Z Up.
    """
    axis = (cad_up_axis or "Z").strip().upper()
    if axis == "Y":
        return Transform3D.from_rpy_xyz((math.pi / 2.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    if axis == "-Y":
        return Transform3D.from_rpy_xyz((-math.pi / 2.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    return Transform3D.identity()


class StepAssemblyParser:
    """Industrial STEP (AP214/AP242) assembly parser and URDF geometry pipeline."""

    def __init__(
        self,
        filepath: Path | str,
        unit_scale: float = 0.001,  # CAD mm to SI meters
        default_density: float = 7850.0,  # kg / m^3 (structural steel default)
        cad_up_axis: str = "Z",  # 'Z' (native Z-up), 'Y' (SolidWorks +Y-up -> Z-up), '-Y' (-Y-up -> Z-up)
    ) -> None:
        self.filepath = Path(filepath).resolve()
        if not self.filepath.exists():
            raise FileNotFoundError(f"STEP file not found: {self.filepath}")

        self.unit_scale = unit_scale
        self.default_density = default_density
        self.cad_up_axis = (cad_up_axis or "Z").strip().upper()
        if self.cad_up_axis not in ("Z", "Y", "-Y"):
            self.cad_up_axis = "Z"

        self._app: Optional[XCAFApp_Application] = None
        self._doc: Optional[TDocStd_Document] = None
        self._shape_tool: Optional[XCAFDoc_ShapeTool] = None
        self._color_tool: Optional[XCAFDoc_ColorTool] = None
        self._name_gen = UniqueNameGenerator()
        self.root_node: Optional[AssemblyNode] = None

    def load(self) -> AssemblyNode:
        """Parse the STEP file and build the hierarchical AssemblyNode tree."""
        # 1. Initialize OpenCASCADE Application Framework (XCAF)
        self._app = XCAFApp_Application.GetApplication_s()
        self._doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
        self._app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), self._doc)

        # 2. Configure STEPCAF reader
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        reader.SetColorMode(True)
        reader.SetLayerMode(True)
        reader.SetPropsMode(True)

        status = reader.ReadFile(str(self.filepath))
        if status != IFSelect_RetDone:
            raise RuntimeError(f"OpenCASCADE failed to read STEP file: {self.filepath} (status={status})")

        transfer_status = reader.Transfer(self._doc)
        if not transfer_status:
            raise RuntimeError(f"OpenCASCADE failed to transfer STEP file data to XCAF document: {self.filepath}")

        # 3. Retrieve shape & color tools
        main_label = self._doc.Main()
        self._shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(main_label)
        self._color_tool = XCAFDoc_DocumentTool.ColorTool_s(main_label)

        # 4. Extract free shapes (roots of the assembly)
        free_shapes = TDF_LabelSequence()
        self._shape_tool.GetFreeShapes(free_shapes)

        if free_shapes.Length() == 0:
            raise ValueError(f"No shapes or assemblies detected in {self.filepath.name}")

        root_name = self.filepath.stem
        self.root_node = AssemblyNode(
            id="root",
            name=self._name_gen.get_unique_name(root_name, fallback_prefix="robot_root"),
            original_name=root_name,
            node_type=AssemblyNodeType.ROOT,
            local_transform=Transform3D.identity(),
            global_transform=Transform3D.identity(),
        )

        # 5. Traverse free shapes
        for i in range(1, free_shapes.Length() + 1):
            lbl = free_shapes.Value(i)
            child_node = self._parse_label_node(
                lbl=lbl,
                parent_id="root",
                parent_global_trsf=gp_Trsf(),
                depth=0,
            )
            if child_node is not None:
                self.root_node.children.append(child_node)

        # If root has exactly one assembly child, promote it or keep clean root wrapper
        if len(self.root_node.children) == 1 and self.root_node.children[0].node_type == AssemblyNodeType.ASSEMBLY:
            # Assembly root is the single child
            pass

        if self.cad_up_axis != "Z":
            self.apply_cad_up_axis(self.cad_up_axis)

        return self.root_node

    def apply_cad_up_axis(self, cad_up_axis: str) -> Transform3D:
        """Apply or switch CAD source up-axis conversion ('Z', 'Y', or '-Y') across the entire assembly tree."""
        axis = (cad_up_axis or "Z").strip().upper()
        if axis not in ("Z", "Y", "-Y"):
            axis = "Z"
        self.cad_up_axis = axis
        root_trsf = get_cad_up_axis_transform(axis)

        if self.root_node is None:
            return root_trsf

        def _update_node(node: AssemblyNode) -> None:
            node.global_transform = root_trsf @ node.raw_global_transform
            if node.is_leaf and node.shape is not None and not node.shape.IsNull():
                node.bounding_box_global = self._compute_bounding_box(node.shape, transform=node.global_transform)
                if node.physical_props is not None:
                    c_global = node.global_transform.transform_point(
                        np.asarray(node.physical_props.com_local, dtype=np.float64) / self.unit_scale
                    ) * self.unit_scale
                    node.physical_props.com_global = (float(c_global[0]), float(c_global[1]), float(c_global[2]))
            for child in node.children:
                _update_node(child)

        _update_node(self.root_node)
        return root_trsf

    def _get_label_name(self, lbl: TDF_Label) -> str:
        """Extract name attribute from a TDF_Label."""
        attr = TDataStd_Name()
        if lbl.FindAttribute(TDataStd_Name.GetID_s(), attr):
            return str(attr.Get().ToExtString()).strip()
        return ""

    def _get_label_entry(self, lbl: TDF_Label) -> str:
        """Extract entry string representation of a label e.g. '0:1:1:1'."""
        entry = TCollection_AsciiString()
        TDF_Tool.Entry_s(lbl, entry)
        return str(entry.ToCString())

    def _extract_color(self, shape: TopoDS_Shape, lbl: TDF_Label) -> Optional[Tuple[float, float, float, float]]:
        """Try to extract RGBA color for a shape/label."""
        if self._color_tool is None:
            return None

        col = Quantity_ColorRGBA()
        # Try shape color
        if self._color_tool.GetColor(shape, XCAFDoc_ColorGen, col) or self._color_tool.GetColor(shape, XCAFDoc_ColorSurf, col):
            rgb = col.GetRGB()
            return (float(rgb.Red()), float(rgb.Green()), float(rgb.Blue()), float(col.Alpha()))

        # Try instance color
        if self._color_tool.GetInstanceColor(shape, XCAFDoc_ColorGen, col):
            rgb = col.GetRGB()
            return (float(rgb.Red()), float(rgb.Green()), float(rgb.Blue()), float(col.Alpha()))

        return None

    def _compute_bounding_box(
        self,
        shape: TopoDS_Shape,
        transform: Optional[Transform3D] = None,
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Compute axis-aligned bounding box (AABB) in meters."""
        if shape.IsNull():
            return None

        target = shape
        if transform is not None and not np.allclose(transform.matrix, np.eye(4)):
            ocp_trsf = gp_Trsf()
            m = transform.matrix
            ocp_trsf.SetValues(
                m[0, 0], m[0, 1], m[0, 2], m[0, 3],
                m[1, 0], m[1, 1], m[1, 2], m[1, 3],
                m[2, 0], m[2, 1], m[2, 2], m[2, 3],
            )
            target = BRepBuilderAPI_Transform(target, ocp_trsf).Shape()

        bbox = Bnd_Box()
        BRepBndLib.Add_s(target, bbox)
        if bbox.IsVoid():
            return None

        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        return (
            float(xmin) * self.unit_scale,
            float(ymin) * self.unit_scale,
            float(zmin) * self.unit_scale,
            float(xmax) * self.unit_scale,
            float(ymax) * self.unit_scale,
            float(zmax) * self.unit_scale,
        )

    def _parse_label_node(
        self,
        lbl: TDF_Label,
        parent_id: str,
        parent_global_trsf: gp_Trsf,
        depth: int,
        visited_labels: Optional[set] = None,
    ) -> Optional[AssemblyNode]:
        """Recursively parse a TDF_Label into an AssemblyNode."""
        if visited_labels is None:
            visited_labels = set()

        entry = self._get_label_entry(lbl)
        if entry in visited_labels and depth > 10:
            # Cycle safeguard
            return None
        visited_labels.add(entry)

        is_assembly = XCAFDoc_ShapeTool.IsAssembly_s(lbl)
        is_component = XCAFDoc_ShapeTool.IsComponent_s(lbl)
        is_simple = XCAFDoc_ShapeTool.IsSimpleShape_s(lbl)

        # Extract names
        instance_name = self._get_label_name(lbl)
        ref_lbl = TDF_Label()
        has_ref = XCAFDoc_ShapeTool.GetReferredShape_s(lbl, ref_lbl)
        definition_name = self._get_label_name(ref_lbl) if has_ref else ""

        # Filter out OpenCASCADE internal reference strings like "=>[0:1:1:1]"
        def _is_meaningful(n: str) -> bool:
            return bool(n and n.strip() and not n.startswith("=>") and not n.startswith("->"))

        if _is_meaningful(instance_name):
            display_name = instance_name
        elif _is_meaningful(definition_name):
            display_name = definition_name
        else:
            display_name = f"part_{entry.replace(':', '_')}"

        sanitized_name = self._name_gen.get_unique_name(display_name)

        # Extract local transform
        local_loc = XCAFDoc_ShapeTool.GetLocation_s(lbl)
        local_ocp_trsf = local_loc.Transformation()
        local_transform = Transform3D.from_ocp_trsf(local_ocp_trsf)

        # Compute chained global transform: parent * local
        global_ocp_trsf = parent_global_trsf.Multiplied(local_ocp_trsf)
        global_transform = Transform3D.from_ocp_trsf(global_ocp_trsf)

        node_id = f"{parent_id}/{sanitized_name}"

        # Check if the referred shape itself is an assembly
        target_lbl = ref_lbl if has_ref else lbl
        target_is_assembly = XCAFDoc_ShapeTool.IsAssembly_s(target_lbl)

        if target_is_assembly or is_assembly:
            # --- ASSEMBLY NODE ---
            node = AssemblyNode(
                id=node_id,
                name=sanitized_name,
                original_name=display_name,
                instance_name=instance_name,
                definition_name=definition_name,
                node_type=AssemblyNodeType.ASSEMBLY,
                local_transform=local_transform,
                global_transform=global_transform,
                raw_global_transform=global_transform,
                label_entry=entry,
            )

            # Recurse on children components
            components = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(target_lbl, components)
            for c_idx in range(1, components.Length() + 1):
                c_lbl = components.Value(c_idx)
                c_node = self._parse_label_node(
                    lbl=c_lbl,
                    parent_id=node_id,
                    parent_global_trsf=global_ocp_trsf,
                    depth=depth + 1,
                    visited_labels=visited_labels.copy(),
                )
                if c_node is not None:
                    node.children.append(c_node)

            return node

        else:
            # --- LEAF PART NODE ---
            # Retrieve shape: prefer definition shape for local frame coordinates
            part_shape = XCAFDoc_ShapeTool.GetShape_s(target_lbl)
            if part_shape.IsNull():
                part_shape = XCAFDoc_ShapeTool.GetShape_s(lbl)

            if part_shape.IsNull():
                return None

            # Check if this shape is a compound of multiple solids
            solid_count = 0
            exp = TopExp_Explorer(part_shape, TopAbs_SOLID)
            while exp.More():
                solid_count += 1
                exp.Next()

            node_type = AssemblyNodeType.COMPOUND if solid_count > 1 else AssemblyNodeType.PART

            # Compute physical properties in local frame
            phys_props = compute_shape_physical_properties(
                shape=part_shape,
                density=self.default_density,
                unit_scale=self.unit_scale,
                global_com_offset=global_transform.translation * self.unit_scale,
            )

            color = self._extract_color(part_shape, target_lbl)
            bbox_local = self._compute_bounding_box(part_shape)
            bbox_global = self._compute_bounding_box(part_shape, transform=global_transform)

            node = AssemblyNode(
                id=node_id,
                name=sanitized_name,
                original_name=display_name,
                instance_name=instance_name,
                definition_name=definition_name,
                node_type=node_type,
                local_transform=local_transform,
                global_transform=global_transform,
                raw_global_transform=global_transform,
                shape=part_shape,
                label_entry=entry,
                color_rgba=color,
                bounding_box_local=bbox_local,
                bounding_box_global=bbox_global,
                physical_props=phys_props,
            )
            return node

    def print_hierarchy(
        self,
        node: Optional[AssemblyNode] = None,
        prefix: str = "",
        is_last: bool = True,
        show_physics: bool = True,
        show_transforms: bool = True,
    ) -> None:
        """Print rich, readable ASCII assembly tree."""
        if node is None:
            if self.root_node is None:
                print("No assembly loaded.")
                return
            print(f"\n📦 Assembly Hierarchy: {self.filepath.name}")
            print("=" * 78)
            for i, child in enumerate(self.root_node.children):
                self.print_hierarchy(
                    node=child,
                    prefix="",
                    is_last=(i == len(self.root_node.children) - 1),
                    show_physics=show_physics,
                    show_transforms=show_transforms,
                )
            print("=" * 78 + "\n")
            return

        branch = "└── " if is_last else "├── "
        type_badge = {
            AssemblyNodeType.ROOT: "[ROOT]",
            AssemblyNodeType.ASSEMBLY: "[ASM ]",
            AssemblyNodeType.PART: "[PART]",
            AssemblyNodeType.COMPOUND: "[COMP]",
        }.get(node.node_type, "[NODE]")

        line = f"{prefix}{branch}{type_badge} {node.name}"
        if node.definition_name and node.definition_name != node.name:
            line += f" ({node.definition_name})"

        print(line)

        # Print details for node
        child_prefix = prefix + ("    " if is_last else "│   ")
        if show_transforms:
            tx, ty, tz = node.local_transform.translation * self.unit_scale
            r, p, y = node.local_transform.to_rpy()
            r_deg, p_deg, y_deg = np.degrees([r, p, y])
            t_line = f"{child_prefix}  📍 Local Pos: [{tx:.4f}, {ty:.4f}, {tz:.4f}] m | RPY: [{r_deg:.1f}°, {p_deg:.1f}°, {y_deg:.1f}°]"
            print(t_line)

        if show_physics and node.physical_props is not None:
            p = node.physical_props
            cx, cy, cz = p.com_local
            phys_line = (
                f"{child_prefix}  ⚖️  Mass: {p.mass:.4f} kg | Vol: {p.volume_mm3:.1f} mm³ | "
                f"CoM: [{cx:.4f}, {cy:.4f}, {cz:.4f}] m"
            )
            print(phys_line)
            # Physical validation status
            v = p.validation
            status_symbol = "✅" if v.is_valid else "⚠️"
            v_line = f"{child_prefix}  {status_symbol} Inertia: Ixx={p.inertia_tensor.ixx:.2e}, Iyy={p.inertia_tensor.iyy:.2e}, Izz={p.inertia_tensor.izz:.2e} kg·m²"
            if not v.is_valid:
                v_line += f" (Violations: {', '.join(v.errors)})"
            print(v_line)

        # Recurse on children
        for i, child in enumerate(node.children):
            self.print_hierarchy(
                node=child,
                prefix=child_prefix,
                is_last=(i == len(node.children) - 1),
                show_physics=show_physics,
                show_transforms=show_transforms,
            )

    def compute_node_properties(
        self,
        node: AssemblyNode,
        density: Optional[float] = None,
        mass: Optional[float] = None,
    ) -> PhysicalProperties:
        """Compute mass properties for a specific node with custom density or mass override."""
        if node.shape is None or node.shape.IsNull():
            raise ValueError(f"Node '{node.name}' does not contain valid geometry.")

        eff_density = density if density is not None else self.default_density
        phys = compute_shape_physical_properties(
            shape=node.shape,
            density=eff_density,
            specified_mass=mass,
            unit_scale=self.unit_scale,
            global_com_offset=node.global_transform.translation * self.unit_scale,
        )
        node.physical_props = phys
        return phys

    def export_node_stl(
        self,
        node: AssemblyNode,
        output_path: Path | str,
        config: Optional[TessellationConfig] = None,
        frame: str = "local",  # "local" or "global"
    ) -> MeshExportResult:
        """Export a leaf part's mesh to STL."""
        if node.shape is None or node.shape.IsNull():
            raise ValueError(f"Cannot export mesh for node '{node.name}' without shape geometry.")

        cfg = config or TessellationConfig(unit_scale=self.unit_scale)
        additional_trsf = None
        if frame.lower() == "global":
            additional_trsf = node.global_transform

        return export_shape_to_stl(
            shape=node.shape,
            output_path=output_path,
            config=cfg,
            transform=additional_trsf,
        )

    def export_all_parts(
        self,
        output_dir: Path | str,
        config: Optional[TessellationConfig] = None,
        frame: str = "local",
    ) -> Dict[str, MeshExportResult]:
        """Export STL visual meshes for all leaf parts in the assembly."""
        out_dir = Path(output_dir).resolve()
        out_dir.mkdir(parents=True, exist_ok=True)

        if self.root_node is None:
            self.load()

        parts = self.root_node.get_all_parts()
        results: Dict[str, MeshExportResult] = {}

        for part in parts:
            stl_name = f"{part.name}.stl"
            stl_path = out_dir / stl_name
            res = self.export_node_stl(part, stl_path, config=config, frame=frame)
            results[part.name] = res

        return results

    def generate_urdf_skeleton(
        self,
        robot_name: str = "urdforge_robot",
        mesh_dir: str = "meshes/visual",
    ) -> str:
        """Generate a simulation-ready URDF XML document preserving the CAD kinematics structure."""
        if self.root_node is None:
            self.load()

        parts = self.root_node.get_all_parts()
        xml_lines: List[str] = [
            f'<?xml version="1.0" encoding="utf-8"?>',
            f'<!-- =================================================================================== -->',
            f'<!-- |    Generated automatically by URDForge CAD/URDF Studio                         | -->',
            f'<!-- |    Source CAD File: {self.filepath.name:<58} | -->',
            f'<!-- =================================================================================== -->',
            f'<robot name="{robot_name}">',
        ]

        # Base footprint / world link
        xml_lines.append('  <!-- Root Base Link -->')
        xml_lines.append('  <link name="base_link"/>')

        for part in parts:
            p_name = part.name
            xml_lines.append(f"\n  <!-- Link: {p_name} -->")
            xml_lines.append(f'  <link name="{p_name}">')

            # Inertial tag
            if part.physical_props is not None:
                xml_lines.append(part.physical_props.to_urdf_inertial_xml(indent=4))

            # Visual & Collision mesh paths
            clean_mesh_dir = mesh_dir.replace("\\", "/")
            if "visual" in clean_mesh_dir:
                vis_mesh_path = f"{clean_mesh_dir}/{p_name}.stl"
                col_mesh_path = f"{clean_mesh_dir.replace('visual', 'collision')}/{p_name}.stl"
            else:
                vis_mesh_path = f"{clean_mesh_dir}/visual/{p_name}.stl"
                col_mesh_path = f"{clean_mesh_dir}/collision/{p_name}.stl"

            # Visual tag
            xml_lines.append("    <visual>")
            xml_lines.append('      <origin xyz="0.000000 0.000000 0.000000" rpy="0.000000 0.000000 0.000000"/>')
            xml_lines.append("      <geometry>")
            xml_lines.append(f'        <mesh filename="{vis_mesh_path}" scale="1.0 1.0 1.0"/>')
            xml_lines.append("      </geometry>")

            # Color material if detected
            if part.color_rgba is not None:
                r, g, b, a = part.color_rgba
                xml_lines.append("      <material name=\"material_{}\">".format(p_name))
                xml_lines.append(f'        <color rgba="{r:.4f} {g:.4f} {b:.4f} {a:.4f}"/>')
                xml_lines.append("      </material>")
            xml_lines.append("    </visual>")

            # Collision tag
            xml_lines.append("    <collision>")
            xml_lines.append('      <origin xyz="0.000000 0.000000 0.000000" rpy="0.000000 0.000000 0.000000"/>')
            xml_lines.append("      <geometry>")
            xml_lines.append(f'        <mesh filename="{col_mesh_path}" scale="1.0 1.0 1.0"/>')
            xml_lines.append("      </geometry>")
            xml_lines.append("    </collision>")
            xml_lines.append("  </link>")

            # Joint connecting link to base (using global transform as preliminary joint origin)
            joint_name = f"joint_{p_name}"
            xml_lines.append(f"\n  <!-- Joint: {joint_name} -->")
            xml_lines.append(f'  <joint name="{joint_name}" type="fixed">')
            xml_lines.append('    <parent link="base_link"/>')
            xml_lines.append(f'    <child link="{p_name}"/>')

            # Joint origin in meters
            tx, ty, tz = part.global_transform.translation * self.unit_scale
            r, p, y = part.global_transform.to_rpy()
            xml_lines.append(f'    <origin xyz="{tx:.6f} {ty:.6f} {tz:.6f}" rpy="{r:.6f} {p:.6f} {y:.6f}"/>')
            xml_lines.append("  </joint>")

        xml_lines.append("</robot>")
        return "\n".join(xml_lines)
