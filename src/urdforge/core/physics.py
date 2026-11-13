"""URDForge Physical & Mass Properties Engine.

Calculates volumetric properties, mass, Center of Mass (CoM), and 3x3 inertia tensors
via OpenCASCADE BRepGProp. Validates inertia tensors against physical feasibility criteria
(positive definiteness and triangle inequalities) required by simulation engines
(Isaac Sim, MuJoCo, Gazebo, Bullet).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopoDS import TopoDS_Shape


@dataclass
class InertiaValidationResult:
    """Diagnostic report for physical feasibility of inertia tensor."""

    is_valid: bool = True
    is_positive_definite: bool = True
    satisfies_triangle_inequality: bool = True
    principal_moments: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    determinant: float = 0.0
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "is_valid": self.is_valid,
            "is_positive_definite": self.is_positive_definite,
            "satisfies_triangle_inequality": self.satisfies_triangle_inequality,
            "principal_moments": list(self.principal_moments),
            "determinant": self.determinant,
            "warnings": self.warnings,
            "errors": self.errors,
        }


@dataclass
class InertiaTensor:
    """3x3 Rotational Inertia Tensor represented about the Center of Mass (CoM).

    Convention:
        I = [
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz]
        ]
    Units: kg * m^2
    """

    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0

    @classmethod
    def from_matrix(cls, mat: np.ndarray) -> InertiaTensor:
        """Create from a 3x3 symmetric matrix."""
        mat = np.asarray(mat, dtype=np.float64)
        if mat.shape != (3, 3):
            raise ValueError(f"Expected 3x3 matrix, got shape {mat.shape}")
        # Average off-diagonal elements for strict symmetry
        ixy = 0.5 * (mat[0, 1] + mat[1, 0])
        ixz = 0.5 * (mat[0, 2] + mat[2, 0])
        iyz = 0.5 * (mat[1, 2] + mat[2, 1])
        return cls(
            ixx=float(mat[0, 0]),
            ixy=float(ixy),
            ixz=float(ixz),
            iyy=float(mat[1, 1]),
            iyz=float(iyz),
            izz=float(mat[2, 2]),
        )

    def to_matrix(self) -> np.ndarray:
        """Return 3x3 numpy array."""
        return np.array(
            [
                [self.ixx, self.ixy, self.ixz],
                [self.ixy, self.iyy, self.iyz],
                [self.ixz, self.iyz, self.izz],
            ],
            dtype=np.float64,
        )

    def rotate(self, rotation_matrix: np.ndarray) -> InertiaTensor:
        """Transform inertia tensor by rotation R: I' = R * I * R^T."""
        r = np.asarray(rotation_matrix, dtype=np.float64)
        i_mat = self.to_matrix()
        rotated_mat = np.matmul(r, np.matmul(i_mat, r.T))
        return InertiaTensor.from_matrix(rotated_mat)

    def translate_parallel_axis(
        self,
        displacement: Sequence[float],
        mass: float,
        from_com_to_ref: bool = True,
    ) -> InertiaTensor:
        """Apply parallel axis theorem (Steiner's theorem).

        I_ref = I_com + m * ( (d^T * d) * E - d * d^T )
        Args:
            displacement: offset vector d = p_ref - p_com
            mass: total mass in kg
            from_com_to_ref: True if shifting from CoM to reference frame;
                             False if shifting from reference frame back to CoM.
        """
        d = np.asarray(displacement, dtype=np.float64)
        d_sq = float(np.dot(d, d))
        outer = np.outer(d, d)
        delta_i = mass * (d_sq * np.eye(3, dtype=np.float64) - outer)

        i_mat = self.to_matrix()
        if from_com_to_ref:
            res_mat = i_mat + delta_i
        else:
            res_mat = i_mat - delta_i
        return InertiaTensor.from_matrix(res_mat)

    def principal_moments(self) -> Tuple[np.ndarray, np.ndarray]:
        """Compute principal moments of inertia and principal axes.

        Returns:
            (eigenvalues, eigenvectors) where eigenvalues are sorted in ascending order.
        """
        eigvals, eigvecs = np.linalg.eigh(self.to_matrix())
        idx = np.argsort(eigvals)
        return eigvals[idx], eigvecs[:, idx]

    def validate(self, mass: float, tol: float = 1e-9) -> InertiaValidationResult:
        """Validate physical feasibility according to rigid body dynamics laws.

        Criteria:
        1. Mass must be strictly positive.
        2. Diagonal terms must be strictly positive.
        3. Inertia tensor must be positive definite (all eigenvalues > 0).
        4. Triangle inequalities must hold:
           lambda_1 + lambda_2 >= lambda_3
           lambda_2 + lambda_3 >= lambda_1
           lambda_1 + lambda_3 >= lambda_2
        """
        warnings: List[str] = []
        errors: List[str] = []
        is_valid = True

        if mass <= 0.0:
            errors.append(f"Mass must be positive, got {mass:.6e} kg")
            is_valid = False

        if self.ixx <= 0.0 or self.iyy <= 0.0 or self.izz <= 0.0:
            errors.append(
                f"Diagonal moments must be strictly positive: "
                f"ixx={self.ixx:.4e}, iyy={self.iyy:.4e}, izz={self.izz:.4e}"
            )
            is_valid = False

        mat = self.to_matrix()
        det = float(np.linalg.det(mat))
        eigvals, _ = self.principal_moments()
        i1, i2, i3 = float(eigvals[0]), float(eigvals[1]), float(eigvals[2])

        is_pos_def = (i1 > -tol) and (i2 > -tol) and (i3 > -tol) and (det > -tol)
        if not is_pos_def:
            errors.append(
                f"Inertia matrix is not positive-definite! Eigenvalues: "
                f"[{i1:.4e}, {i2:.4e}, {i3:.4e}], Det: {det:.4e}"
            )
            is_valid = False

        # Triangle inequality check on principal moments
        # For any physical 3D mass distribution:
        # I1 + I2 >= I3, I2 + I3 >= I1, I1 + I3 >= I2
        violates_triangle = False
        if (i1 + i2) < (i3 - tol):
            errors.append(f"Triangle inequality violated: I1 + I2 ({i1+i2:.4e}) < I3 ({i3:.4e})")
            violates_triangle = True
        if (i2 + i3) < (i1 - tol):
            errors.append(f"Triangle inequality violated: I2 + I3 ({i2+i3:.4e}) < I1 ({i1:.4e})")
            violates_triangle = True
        if (i1 + i3) < (i2 - tol):
            errors.append(f"Triangle inequality violated: I1 + I3 ({i1+i3:.4e}) < I2 ({i2:.4e})")
            violates_triangle = True

        if violates_triangle:
            is_valid = False

        # Check for near-planar / thin-sheet condition
        if min(i1, i2, i3) < 1e-12 * max(i1, i2, i3):
            warnings.append("Body appears nearly 2D or degenerate along one axis.")

        return InertiaValidationResult(
            is_valid=is_valid,
            is_positive_definite=is_pos_def,
            satisfies_triangle_inequality=(not violates_triangle),
            principal_moments=(i1, i2, i3),
            determinant=det,
            warnings=warnings,
            errors=errors,
        )

    def to_urdf_xml(self, indent: int = 4) -> str:
        """Format URDF `<inertia ... />` tag."""
        sp = " " * indent
        return (
            f'{sp}<inertia ixx="{self.ixx:.8e}" ixy="{self.ixy:.8e}" ixz="{self.ixz:.8e}" '
            f'iyy="{self.iyy:.8e}" iyz="{self.iyz:.8e}" izz="{self.izz:.8e}"/>'
        )

    def to_dict(self) -> Dict[str, float]:
        return {
            "ixx": self.ixx,
            "ixy": self.ixy,
            "ixz": self.ixz,
            "iyy": self.iyy,
            "iyz": self.iyz,
            "izz": self.izz,
        }


@dataclass
class PhysicalProperties:
    """Complete physical specification of a component or link."""

    volume_m3: float
    volume_mm3: float
    density: float  # kg / m^3
    mass: float  # kg
    com_local: Tuple[float, float, float]  # (x, y, z) in meters in local frame
    com_global: Tuple[float, float, float]  # (x, y, z) in meters in global assembly frame
    inertia_tensor: InertiaTensor  # Inertia tensor about CoM in kg * m^2
    validation: InertiaValidationResult = field(default_factory=InertiaValidationResult)

    def to_urdf_inertial_xml(self, indent: int = 4) -> str:
        """Generate simulation-ready URDF `<inertial>` block."""
        sp = " " * indent
        sp_inner = " " * (indent + 2)
        cx, cy, cz = self.com_local
        xml_lines = [
            f"{sp}<inertial>",
            f'{sp_inner}<origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0.000000 0.000000 0.000000"/>',
            f'{sp_inner}<mass value="{self.mass:.6f}"/>',
            self.inertia_tensor.to_urdf_xml(indent=indent + 2),
            f"{sp}</inertial>",
        ]
        return "\n".join(xml_lines)

    def to_dict(self) -> Dict[str, Any]:
        i_mat = self.inertia_tensor.to_matrix().tolist()
        return {
            "volume_m3": self.volume_m3,
            "volume_mm3": self.volume_mm3,
            "density_kg_m3": self.density,
            "mass_kg": self.mass,
            "mass": self.mass,
            "com_local_m": list(self.com_local),
            "com_global_m": list(self.com_global),
            "com_xyz": list(self.com_local),
            "inertia_tensor": self.inertia_tensor.to_dict(),
            "inertia_matrix": i_mat,
            "validation": self.validation.to_dict(),
        }


def compute_shape_physical_properties(
    shape: TopoDS_Shape,
    density: Optional[float] = 7850.0,
    specified_mass: Optional[float] = None,
    unit_scale: float = 0.001,
    global_com_offset: Optional[Sequence[float]] = None,
) -> PhysicalProperties:
    """Compute physical properties of an OpenCASCADE shape using BRepGProp.

    Args:
        shape: TopoDS_Shape to analyze.
        density: Material density in kg/m^3 (used if specified_mass is None).
        specified_mass: Fixed total mass in kg (overrides density if provided).
        unit_scale: Length scale from CAD units to meters (default 0.001 for mm -> m).
        global_com_offset: Optional global translation to add for global CoM output.

    Returns:
        PhysicalProperties object with mass, CoM, inertia tensor, and physics validation.
    """
    if shape.IsNull():
        raise ValueError("Cannot calculate physical properties for a null shape.")

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)

    # CAD raw volume (e.g. in mm^3)
    raw_vol = float(props.Mass())
    if raw_vol <= 1e-15:
        # Fallback or degenerate shape
        raw_vol = 1e-9

    # Scale volume to m^3: V_m3 = V_raw * (unit_scale)^3
    vol_scale_factor = unit_scale**3
    volume_m3 = raw_vol * vol_scale_factor
    volume_mm3 = volume_m3 * 1e9

    # Determine mass & density
    if specified_mass is not None and specified_mass > 0.0:
        mass = float(specified_mass)
        eff_density = mass / volume_m3 if volume_m3 > 0.0 else 7850.0
    else:
        eff_density = float(density if density is not None and density > 0.0 else 7850.0)
        mass = volume_m3 * eff_density

    # Center of mass in CAD units converted to meters
    raw_com = props.CentreOfMass()
    com_local = (
        float(raw_com.X()) * unit_scale,
        float(raw_com.Y()) * unit_scale,
        float(raw_com.Z()) * unit_scale,
    )

    if global_com_offset is not None:
        com_global = (
            com_local[0] + float(global_com_offset[0]),
            com_local[1] + float(global_com_offset[1]),
            com_local[2] + float(global_com_offset[2]),
        )
    else:
        com_global = com_local

    # OpenCASCADE MatrixOfInertia is computed at the Center of Mass!
    # Raw inertia has units of [Length^5].
    # To convert to kg * m^2:
    # I_kg_m2 = I_raw * (unit_scale)^5 * eff_density
    # Note: if mass is specified, eff_density = mass / volume_m3, which is exact.
    occ_mat = props.MatrixOfInertia()
    inertia_scale_factor = (unit_scale**5) * eff_density

    i_mat = np.zeros((3, 3), dtype=np.float64)
    for r in range(1, 4):
        for c in range(1, 4):
            i_mat[r - 1, c - 1] = occ_mat.Value(r, c) * inertia_scale_factor

    inertia_tensor = InertiaTensor.from_matrix(i_mat)
    validation = inertia_tensor.validate(mass=mass)

    return PhysicalProperties(
        volume_m3=volume_m3,
        volume_mm3=volume_mm3,
        density=eff_density,
        mass=mass,
        com_local=com_local,
        com_global=com_global,
        inertia_tensor=inertia_tensor,
        validation=validation,
    )


def compute_composite_physical_properties(
    parts_data: Sequence[Tuple[PhysicalProperties, Optional[Any]]],
    link_frame_world: Optional[Any] = None,
) -> PhysicalProperties:
    """Compute exact composite rigid body mass, CoM, and 3x3 inertia tensor for a multi-part link.

    Uses Steiner's Parallel Axis Theorem to aggregate multiple rigid bodies:
        I_total = sum_i [ R_i * I_i * R_i^T + m_i * ( (d_i^T * d_i) * E - d_i * d_i^T ) ]
    where d_i = c_i_world - C_world, and finally transforms the composite tensor into the
    local coordinate frame of the parent link.

    Args:
        parts_data: List of (PhysicalProperties, part_global_transform).
        link_frame_world: Global world transform of the target Link coordinate frame.

    Returns:
        Aggregated PhysicalProperties in link local frame and world frame.
    """
    if not parts_data:
        # Fallback for empty virtual links (e.g. dummy base_link)
        empty_i = InertiaTensor(ixx=1e-5, iyy=1e-5, izz=1e-5)
        return PhysicalProperties(
            volume_m3=1e-6,
            volume_mm3=1000.0,
            density=1000.0,
            mass=0.001,
            com_local=(0.0, 0.0, 0.0),
            com_global=(0.0, 0.0, 0.0),
            inertia_tensor=empty_i,
            validation=empty_i.validate(mass=0.001),
        )

    total_mass = 0.0
    total_volume_m3 = 0.0
    total_volume_mm3 = 0.0

    # First pass: compute total mass and composite Center of Mass in world frame
    c_weighted_sum = np.zeros(3, dtype=np.float64)
    extracted_items = []

    for prop, xform in parts_data:
        m = max(1e-9, float(prop.mass))
        total_mass += m
        total_volume_m3 += float(prop.volume_m3)
        total_volume_mm3 += float(prop.volume_mm3)

        # Determine world CoM of this part
        if xform is not None and hasattr(xform, "transform_point"):
            c_world = np.asarray(xform.transform_point(prop.com_local), dtype=np.float64)
            r_mat = np.asarray(xform.rotation_matrix, dtype=np.float64)
        else:
            c_world = np.asarray(prop.com_global, dtype=np.float64)
            r_mat = np.eye(3, dtype=np.float64)

        # Part inertia rotated to world orientation
        i_world = prop.inertia_tensor.rotate(r_mat).to_matrix()
        c_weighted_sum += m * c_world
        extracted_items.append((m, c_world, i_world))

    composite_com_world = c_weighted_sum / total_mass

    # Second pass: apply parallel axis theorem for each part to the composite CoM
    composite_i_world = np.zeros((3, 3), dtype=np.float64)
    for m, c_world, i_world in extracted_items:
        d = c_world - composite_com_world
        d_sq = float(np.dot(d, d))
        parallel_shift = m * (d_sq * np.eye(3, dtype=np.float64) - np.outer(d, d))
        composite_i_world += i_world + parallel_shift

    # Third pass: express in Link coordinate frame
    if link_frame_world is not None and hasattr(link_frame_world, "rotation_matrix"):
        r_link = np.asarray(link_frame_world.rotation_matrix, dtype=np.float64)
        p_link = np.asarray(link_frame_world.translation, dtype=np.float64)
        # c_link = R_link^T * (c_world - p_link)
        composite_com_link = np.matmul(r_link.T, composite_com_world - p_link)
        # I_link = R_link^T * I_world * R_link
        composite_i_link = np.matmul(r_link.T, np.matmul(composite_i_world, r_link))
    else:
        composite_com_link = composite_com_world
        composite_i_link = composite_i_world

    eff_density = total_mass / total_volume_m3 if total_volume_m3 > 0 else 7850.0
    final_inertia = InertiaTensor.from_matrix(composite_i_link)
    validation = final_inertia.validate(mass=total_mass)

    return PhysicalProperties(
        volume_m3=total_volume_m3,
        volume_mm3=total_volume_mm3,
        density=eff_density,
        mass=total_mass,
        com_local=(float(composite_com_link[0]), float(composite_com_link[1]), float(composite_com_link[2])),
        com_global=(float(composite_com_world[0]), float(composite_com_world[1]), float(composite_com_world[2])),
        inertia_tensor=final_inertia,
        validation=validation,
    )


def load_stl_triangles(stl_path: str) -> Optional[np.ndarray]:
    """Load triangle vertex coordinates (M, 3, 3) from a binary or ASCII STL file."""
    import os
    import struct
    if not os.path.exists(stl_path):
        return None
    try:
        with open(stl_path, "rb") as f:
            header = f.read(80)
            n_bytes = os.path.getsize(stl_path)
            if n_bytes >= 84:
                n_tris = struct.unpack("<I", f.read(4))[0]
                if n_bytes == 84 + n_tris * 50:
                    dtype = np.dtype([
                        ("normal", "<f4", (3,)),
                        ("vertices", "<f4", (3, 3)),
                        ("attr", "<u2")
                    ])
                    data = np.fromfile(f, dtype=dtype, count=n_tris)
                    return data["vertices"].astype(np.float64)
    except Exception:
        pass

    # Fallback to OpenCASCADE RWStl for ASCII or non-standard STL files
    try:
        from OCP.RWStl import RWStl
        from OCP.OSD import OSD_Path
        from OCP.TCollection import TCollection_AsciiString
        poly = RWStl.ReadFile_s(OSD_Path(TCollection_AsciiString(stl_path)))
        if poly is not None and not poly.IsNull():
            n_tri = poly.NbTriangles()
            tris = np.zeros((n_tri, 3, 3), dtype=np.float64)
            for i in range(1, n_tri + 1):
                n1, n2, n3 = poly.Triangle(i).Get()
                p1, p2, p3 = poly.Node(n1), poly.Node(n2), poly.Node(n3)
                tris[i - 1, 0] = [p1.X(), p1.Y(), p1.Z()]
                tris[i - 1, 1] = [p2.X(), p2.Y(), p2.Z()]
                tris[i - 1, 2] = [p3.X(), p3.Y(), p3.Z()]
            return tris
    except Exception:
        pass
    return None


def compute_mesh_triangles_mass_properties(
    tris: np.ndarray,
    density: float = 1000.0,
) -> Tuple[float, np.ndarray, np.ndarray]:
    """Compute exact volume, CoM, and 3x3 rotational inertia tensor at CoM for a closed triangle mesh.

    Uses David Eberly / Brian Mirtich tetrahedral decomposition formula.
    Args:
        tris: (M, 3, 3) array of triangle vertex coordinates.
        density: Material density in kg/m^3.

    Returns:
        (volume_m3, com_local (3,), inertia_tensor_at_com (3, 3)).
    """
    if tris is None or len(tris) == 0:
        return 0.0, np.zeros(3, dtype=np.float64), np.zeros((3, 3), dtype=np.float64)

    a = tris[:, 0, :]
    b = tris[:, 1, :]
    c = tris[:, 2, :]
    det = np.sum(a * np.cross(b, c), axis=1)  # 6 * V_tet
    vol = float(np.sum(det) / 6.0)
    if abs(vol) < 1e-15:
        return 0.0, np.zeros(3, dtype=np.float64), np.zeros((3, 3), dtype=np.float64)

    # First moments: int x dV = (1/24) * det * (a + b + c)
    first_moments = np.sum(det[:, None] * (a + b + c), axis=0) / 24.0
    com = first_moments / vol

    # Second moments: int x_j x_k dV
    sum_abc = a + b + c
    cov_origin = np.zeros((3, 3), dtype=np.float64)
    for j in range(3):
        for k in range(j, 3):
            term = (a[:, j] * a[:, k] + b[:, j] * b[:, k] + c[:, j] * c[:, k]) + sum_abc[:, j] * sum_abc[:, k]
            val = float(np.sum(det * term) / 120.0)
            cov_origin[j, k] = val
            cov_origin[k, j] = val

    # Parallel axis shift to CoM: int (x - c_x)(y - c_y) dV = int x y dV - vol * c_x * c_y
    cov_com = cov_origin - vol * np.outer(com, com)
    inertia_at_com = (np.trace(cov_com) * np.eye(3, dtype=np.float64) - cov_com) * density
    return vol, com, inertia_at_com


def compute_link_physics_from_meshes(
    visuals: Sequence[Any],
    specified_mass: Optional[float] = None,
    default_density: float = 7850.0,
    base_dir: Optional[str] = None,
) -> PhysicalProperties:
    """Compute rigid body physical properties directly from visual/collision STL meshes and their origins.

    Guarantees 100% mathematical coincidence between mesh geometry and the inertial frame.
    """
    import os
    parts_data = []
    total_vol = 0.0

    for vis in visuals:
        mesh_rel_path = getattr(vis, "mesh_filepath", None)
        origin_xform = getattr(vis, "origin", None)
        if not mesh_rel_path:
            continue

        cand_paths = [mesh_rel_path]
        if base_dir:
            cand_paths.append(os.path.join(base_dir, mesh_rel_path))
            cand_paths.append(os.path.join(base_dir, "meshes", os.path.basename(mesh_rel_path)))
            cand_paths.append(os.path.join(base_dir, "meshes", "visual", os.path.basename(mesh_rel_path)))

        found_path = next((p for p in cand_paths if os.path.exists(p)), None)
        if not found_path:
            continue

        tris = load_stl_triangles(found_path)
        if tris is None or len(tris) == 0:
            continue

        vol, com_mesh, i_mat = compute_mesh_triangles_mass_properties(tris, density=1.0)
        if abs(vol) < 1e-15:
            continue

        total_vol += vol
        t_mesh = InertiaTensor.from_matrix(i_mat)
        p_prop = PhysicalProperties(
            volume_m3=vol,
            volume_mm3=vol * 1e9,
            density=1.0,
            mass=vol,  # unit density
            com_local=(float(com_mesh[0]), float(com_mesh[1]), float(com_mesh[2])),
            com_global=(float(com_mesh[0]), float(com_mesh[1]), float(com_mesh[2])),
            inertia_tensor=t_mesh,
            validation=t_mesh.validate(mass=max(1e-9, vol)),
        )
        parts_data.append((p_prop, origin_xform))

    if not parts_data:
        empty_i = InertiaTensor(ixx=1e-5, iyy=1e-5, izz=1e-5)
        m = specified_mass if specified_mass and specified_mass > 0 else 0.001
        return PhysicalProperties(
            volume_m3=1e-6,
            volume_mm3=1000.0,
            density=default_density,
            mass=m,
            com_local=(0.0, 0.0, 0.0),
            com_global=(0.0, 0.0, 0.0),
            inertia_tensor=empty_i,
            validation=empty_i.validate(mass=m),
        )

    # Determine mass and density
    target_mass = float(specified_mass) if (specified_mass and specified_mass > 0) else (total_vol * default_density)
    eff_density = target_mass / total_vol if total_vol > 0 else default_density

    # Scale parts by eff_density
    scaled_parts = []
    for prop, xform in parts_data:
        m_scaled = prop.volume_m3 * eff_density
        i_scaled = prop.inertia_tensor.scale(eff_density)
        p_scaled = PhysicalProperties(
            volume_m3=prop.volume_m3,
            volume_mm3=prop.volume_mm3,
            density=eff_density,
            mass=m_scaled,
            com_local=prop.com_local,
            com_global=prop.com_global,
            inertia_tensor=i_scaled,
            validation=i_scaled.validate(mass=m_scaled),
        )
        scaled_parts.append((p_scaled, xform))

    return compute_composite_physical_properties(
        parts_data=scaled_parts,
        link_frame_world=None,  # Evaluated directly in link local frame
    )

