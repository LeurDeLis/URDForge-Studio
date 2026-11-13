"""URDForge Mesh Tessellation & STL Exporter.

Discretizes OpenCASCADE B-Rep geometric shapes into high-quality triangular meshes
and exports simulation-ready visual/collision meshes (binary STL and Wavefront OBJ).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.gp import gp_Pnt, gp_Trsf
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Shape

from urdforge.core.math_types import Transform3D


@dataclass
class TessellationConfig:
    """Configuration for surface meshing and tessellation."""

    linear_deflection: float = 0.001  # Chord error tolerance (in meters if scaled, or mm if CAD units)
    angular_deflection: float = 0.3  # Angular deflection in radians (~17.2 degrees)
    is_relative: bool = False  # If True, linear_deflection is relative to bounding box
    scale_to_meter: bool = True  # If True, scales CAD model (mm) to meters (m) for URDF
    unit_scale: float = 0.001  # Scale factor from CAD units to meters
    binary_stl: bool = True  # Binary STL (compact & fast) vs ASCII


@dataclass
class MeshExportResult:
    """Result of mesh export operation."""

    file_path: Path
    file_size_bytes: int
    triangle_count: int
    vertex_count: int
    face_count: int
    scaled: bool


def count_mesh_statistics(shape: TopoDS_Shape) -> Tuple[int, int, int]:
    """Count faces, vertices (nodes), and triangles in an already meshed shape.

    Returns:
        (face_count, vertex_count, triangle_count)
    """
    face_count = 0
    vertex_count = 0
    triangle_count = 0

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            face_count += 1
            vertex_count += tri.NbNodes()
            triangle_count += tri.NbTriangles()
        exp.Next()

    return face_count, vertex_count, triangle_count


def export_shape_to_stl(
    shape: TopoDS_Shape,
    output_path: Path | str,
    config: Optional[TessellationConfig] = None,
    transform: Optional[Transform3D] = None,
) -> MeshExportResult:
    """Tessellate an OpenCASCADE shape and export it to an STL file.

    Args:
        shape: TopoDS_Shape to tessellate and export.
        output_path: Target .stl file path.
        config: Tessellation and scaling options.
        transform: Optional additional transform to apply (e.g. for global assembly frame).

    Returns:
        MeshExportResult with file info and mesh statistics.
    """
    if shape.IsNull():
        raise ValueError("Cannot export a null shape to STL.")

    cfg = config or TessellationConfig()
    out_file = Path(output_path).resolve()
    out_file.parent.mkdir(parents=True, exist_ok=True)

    target_shape = shape

    # 1. Apply additional transform if provided
    if transform is not None and not np_is_identity(transform.matrix):
        ocp_trsf = gp_Trsf()
        mat = transform.matrix
        # In OpenCASCADE gp_Trsf.SetValues(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34)
        ocp_trsf.SetValues(
            mat[0, 0], mat[0, 1], mat[0, 2], mat[0, 3],
            mat[1, 0], mat[1, 1], mat[1, 2], mat[1, 3],
            mat[2, 0], mat[2, 1], mat[2, 2], mat[2, 3],
        )
        target_shape = BRepBuilderAPI_Transform(target_shape, ocp_trsf).Shape()

    # 2. Scale to meters if requested
    scaled = False
    if cfg.scale_to_meter and abs(cfg.unit_scale - 1.0) > 1e-9:
        scale_trsf = gp_Trsf()
        scale_trsf.SetScale(gp_Pnt(0.0, 0.0, 0.0), cfg.unit_scale)
        target_shape = BRepBuilderAPI_Transform(target_shape, scale_trsf).Shape()
        scaled = True

    # 3. Meshing with BRepMesh_IncrementalMesh
    # Adjust deflection based on whether model was scaled
    deflection = cfg.linear_deflection
    mesher = BRepMesh_IncrementalMesh(
        target_shape,
        deflection,
        cfg.is_relative,
        cfg.angular_deflection,
        True,  # Parallel meshing
    )
    mesher.Perform()

    if not mesher.IsDone():
        raise RuntimeError(f"BRepMesh failed to tessellate shape for {out_file.name}")

    # 4. Write STL file
    writer = StlAPI_Writer()
    writer.ASCIIMode = not cfg.binary_stl
    success = writer.Write(target_shape, str(out_file))

    if not success or not out_file.exists():
        raise IOError(f"Failed to write STL file: {out_file}")

    # 5. Extract statistics
    face_count, vertex_count, triangle_count = count_mesh_statistics(target_shape)
    file_size = out_file.stat().st_size

    return MeshExportResult(
        file_path=out_file,
        file_size_bytes=file_size,
        triangle_count=triangle_count,
        vertex_count=vertex_count,
        face_count=face_count,
        scaled=scaled,
    )


def np_is_identity(mat) -> bool:
    """Helper to check if a 4x4 matrix is close to identity."""
    import numpy as np
    return bool(np.allclose(mat, np.eye(4, dtype=np.float64), atol=1e-8))
