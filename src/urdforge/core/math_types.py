"""URDForge Mathematics & Kinematics Types.

Provides 3D transformations, Euler angles (RPY), quaternions, and matrix conversions
tailored for robotics and URDF kinematics specifications.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence, Tuple, Union

import numpy as np


@dataclass
class Transform3D:
    """Rigid body 3D transformation (SE(3)) representing rotation and translation.

    Conforms to standard robotics and ROS URDF conventions (Z-Y-X Euler angles).
    """

    matrix: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))

    def __post_init__(self) -> None:
        mat = np.asarray(self.matrix, dtype=np.float64)
        if mat.shape != (4, 4):
            raise ValueError(f"Expected 4x4 matrix, got shape {mat.shape}")
        self.matrix = mat

    @classmethod
    def identity(cls) -> Transform3D:
        """Create an identity transformation."""
        return cls(matrix=np.eye(4, dtype=np.float64))

    @classmethod
    def from_translation(cls, x: float, y: float, z: float) -> Transform3D:
        """Create a pure translation transformation."""
        mat = np.eye(4, dtype=np.float64)
        mat[:3, 3] = [x, y, z]
        return cls(matrix=mat)

    @classmethod
    def from_rpy_xyz(
        cls,
        rpy: Sequence[float],
        xyz: Sequence[float],
    ) -> Transform3D:
        """Construct transformation from Roll-Pitch-Yaw (radians) and XYZ translation."""
        roll, pitch, yaw = rpy
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        # Standard R = Rz(yaw) * Ry(pitch) * Rx(roll)
        rot = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )

        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot
        mat[:3, 3] = xyz
        return cls(matrix=mat)

    @classmethod
    def from_ocp_trsf(cls, trsf) -> Transform3D:
        """Construct Transform3D from OpenCASCADE gp_Trsf."""
        mat = np.eye(4, dtype=np.float64)
        for r in range(1, 4):
            for c in range(1, 4):
                mat[r - 1, c - 1] = trsf.Value(r, c)
            mat[r - 1, 3] = trsf.Value(r, 4)
        return cls(matrix=mat)

    @property
    def translation(self) -> np.ndarray:
        """3D translation vector [x, y, z]."""
        return self.matrix[:3, 3].copy()

    @property
    def rotation_matrix(self) -> np.ndarray:
        """3x3 rotation matrix."""
        return self.matrix[:3, :3].copy()

    def to_rpy(self) -> Tuple[float, float, float]:
        """Convert rotation matrix to Roll-Pitch-Yaw angles (radians) using ROS URDF convention.

        Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Returns:
            (roll, pitch, yaw) in radians.
        """
        r = self.matrix[:3, :3]
        # Pitch: -asin(R[2, 0])
        sin_p = -r[2, 0]
        # Clamp to avoid numerical domain error with asin
        sin_p = max(-1.0, min(1.0, sin_p))
        pitch = math.asin(sin_p)

        cos_p = math.cos(pitch)
        if abs(cos_p) > 1e-6:
            roll = math.atan2(r[2, 1], r[2, 2])
            yaw = math.atan2(r[1, 0], r[0, 0])
        else:
            # Gimbal lock at pitch = +- pi/2
            roll = 0.0
            yaw = math.atan2(-r[0, 1], r[1, 1])

        return roll, pitch, yaw

    def to_quaternion(self) -> Tuple[float, float, float, float]:
        """Convert rotation matrix to unit quaternion (x, y, z, w)."""
        r = self.matrix[:3, :3]
        trace = np.trace(r)

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm > 1e-12:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return x, y, z, w

    def multiplied(self, other: Transform3D) -> Transform3D:
        """Multiply with another transformation: result = self * other."""
        return Transform3D(matrix=np.matmul(self.matrix, other.matrix))

    def __matmul__(self, other: Transform3D) -> Transform3D:
        return self.multiplied(other)

    def inverse(self) -> Transform3D:
        """Compute the inverse transformation SE(3)^-1 = [R^T, -R^T * p]."""
        rot = self.matrix[:3, :3]
        pos = self.matrix[:3, 3]
        inv_rot = rot.T
        inv_pos = -np.matmul(inv_rot, pos)

        inv_mat = np.eye(4, dtype=np.float64)
        inv_mat[:3, :3] = inv_rot
        inv_mat[:3, 3] = inv_pos
        return Transform3D(matrix=inv_mat)

    def transform_point(self, pt: Sequence[float]) -> np.ndarray:
        """Transform a 3D point p' = R * p + t."""
        p_homo = np.array([pt[0], pt[1], pt[2], 1.0], dtype=np.float64)
        res = np.matmul(self.matrix, p_homo)
        return res[:3]

    def to_urdf_origin_xml(self, indent: int = 4) -> str:
        """Format origin XML tag for URDF `<origin xyz="..." rpy="..."/>`."""
        tx, ty, tz = self.translation
        r, p, y = self.to_rpy()
        sp = " " * indent
        return f'{sp}<origin xyz="{tx:.6f} {ty:.6f} {tz:.6f}" rpy="{r:.6f} {p:.6f} {y:.6f}"/>'

    def to_dict(self) -> dict:
        """Serialize transformation to a dictionary."""
        r, p, y = self.to_rpy()
        qx, qy, qz, qw = self.to_quaternion()
        return {
            "matrix": self.matrix.tolist(),
            "xyz": self.translation.tolist(),
            "rpy_rad": [r, p, y],
            "rpy_deg": [math.degrees(r), math.degrees(p), math.degrees(y)],
            "quaternion_xyzw": [qx, qy, qz, qw],
        }
