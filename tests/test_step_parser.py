"""Unit & Integration Test Suite for URDForge CAD/URDF Engine (Phase 1)."""

import math
from pathlib import Path
import pytest
import numpy as np

from urdforge.core.math_types import Transform3D
from urdforge.core.physics import (
    InertiaTensor,
    compute_shape_physical_properties,
)
from urdforge.core.mesher import (
    TessellationConfig,
    export_shape_to_stl,
    count_mesh_statistics,
)
from urdforge.core.step_parser import StepAssemblyParser, AssemblyNodeType
from urdforge.utils.naming import sanitize_urdf_name, UniqueNameGenerator

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


class TestMathTypes:
    """Tests for 3D transformations, Euler angles, and kinematics math."""

    def test_identity_and_translation(self):
        t = Transform3D.from_translation(1.0, 2.0, 3.0)
        assert np.allclose(t.translation, [1.0, 2.0, 3.0])
        r, p, y = t.to_rpy()
        assert np.allclose([r, p, y], [0.0, 0.0, 0.0])

    def test_rpy_roundtrip(self):
        roll, pitch, yaw = 0.2, -0.4, 0.8
        t = Transform3D.from_rpy_xyz([roll, pitch, yaw], [10.0, 20.0, 30.0])
        r_rec, p_rec, y_rec = t.to_rpy()
        assert math.isclose(roll, r_rec, abs_tol=1e-6)
        assert math.isclose(pitch, p_rec, abs_tol=1e-6)
        assert math.isclose(yaw, y_rec, abs_tol=1e-6)

    def test_quaternion_and_inverse(self):
        t1 = Transform3D.from_rpy_xyz([0.1, 0.2, 0.3], [1.0, 2.0, 3.0])
        t_inv = t1.inverse()
        ident = t1 @ t_inv
        assert np.allclose(ident.matrix, np.eye(4), atol=1e-7)

        # Quaternion norm must be 1.0
        qx, qy, qz, qw = t1.to_quaternion()
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        assert math.isclose(norm, 1.0, abs_tol=1e-7)


class TestPhysicsEngine:
    """Tests for mass, center of mass, inertia tensor, and simulation checks."""

    def test_box_analytic_physics(self):
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

        # Box dimensions: 100 x 200 x 300 mm = 0.1 x 0.2 x 0.3 m
        box = BRepPrimAPI_MakeBox(100.0, 200.0, 300.0).Shape()
        density = 7850.0  # kg/m^3 (steel)

        phys = compute_shape_physical_properties(
            shape=box,
            density=density,
            unit_scale=0.001,
        )

        expected_vol = 0.1 * 0.2 * 0.3  # 0.006 m^3
        expected_mass = expected_vol * density  # 47.1 kg
        expected_com = (0.05, 0.10, 0.15)  # meters
        # Ixx = 1/12 * M * (b^2 + c^2) = 1/12 * 47.1 * (0.04 + 0.09) = 0.51025
        expected_ixx = (1.0 / 12.0) * expected_mass * (0.2**2 + 0.3**2)
        expected_iyy = (1.0 / 12.0) * expected_mass * (0.1**2 + 0.3**2)
        expected_izz = (1.0 / 12.0) * expected_mass * (0.1**2 + 0.2**2)

        assert math.isclose(phys.volume_m3, expected_vol, rel_tol=1e-5)
        assert math.isclose(phys.mass, expected_mass, rel_tol=1e-5)
        assert np.allclose(phys.com_local, expected_com, atol=1e-5)
        assert math.isclose(phys.inertia_tensor.ixx, expected_ixx, rel_tol=1e-5)
        assert math.isclose(phys.inertia_tensor.iyy, expected_iyy, rel_tol=1e-5)
        assert math.isclose(phys.inertia_tensor.izz, expected_izz, rel_tol=1e-5)

        # Off-diagonals must be zero for aligned box
        assert math.isclose(phys.inertia_tensor.ixy, 0.0, abs_tol=1e-9)
        assert math.isclose(phys.inertia_tensor.ixz, 0.0, abs_tol=1e-9)
        assert math.isclose(phys.inertia_tensor.iyz, 0.0, abs_tol=1e-9)

        # Validation check
        assert phys.validation.is_valid is True
        assert phys.validation.is_positive_definite is True
        assert phys.validation.satisfies_triangle_inequality is True

    def test_invalid_inertia_detection(self):
        # Create non-physical tensor violating triangle inequality
        # e.g. Izz = 100 while Ixx = 1, Iyy = 1 -> Ixx + Iyy < Izz
        bad_tensor = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=100.0)
        res = bad_tensor.validate(mass=1.0)
        assert res.is_valid is False
        assert res.satisfies_triangle_inequality is False


class TestStepAssemblyParser:
    """Integration tests on STEP assembly file parsing."""

    def test_parse_single_box_step(self):
        step_file = FIXTURES_DIR / "sample_box.step"
        parser = StepAssemblyParser(filepath=step_file, unit_scale=0.001)
        root = parser.load()

        assert root is not None
        parts = root.get_all_parts()
        assert len(parts) >= 1
        part = parts[0]
        assert part.physical_props is not None
        assert math.isclose(part.physical_props.volume_m3, 0.006, rel_tol=1e-4)

    def test_parse_nested_robot_step(self):
        step_file = FIXTURES_DIR / "sample_nested_robot.step"
        parser = StepAssemblyParser(filepath=step_file, unit_scale=0.001)
        root = parser.load()

        assert root is not None
        parts = root.get_all_parts()
        # Should have 3 parts: base_pedestal, turret_cylinder, end_effector_sphere
        assert len(parts) == 3

        part_names = [p.name for p in parts]
        assert any("base" in n for n in part_names)
        assert any("turret" in n for n in part_names)
        assert any("sphere" in n or "effector" in n or "tool" in n for n in part_names)

        # Check global transform chaining for the sphere:
        # arm at (60, 60, 40) mm, sphere at (0, 0, 100) mm relative to arm
        # so sphere is at (60, 60, 140) mm
        sphere_part = next(p for p in parts if "sphere" in p.name or "effector" in p.name or "tool" in p.name)
        sphere_pos = sphere_part.global_transform.translation
        assert math.isclose(sphere_pos[0], 60.0, abs_tol=1e-3)
        assert math.isclose(sphere_pos[1], 60.0, abs_tol=1e-3)
        assert math.isclose(sphere_pos[2], 140.0, abs_tol=1e-3)

    def test_export_stl_and_urdf(self, tmp_path):
        step_file = FIXTURES_DIR / "sample_nested_robot.step"
        parser = StepAssemblyParser(filepath=step_file, unit_scale=0.001)
        parser.load()

        # Export meshes
        out_mesh_dir = tmp_path / "meshes"
        results = parser.export_all_parts(output_dir=out_mesh_dir)
        assert len(results) == 3

        for name, res in results.items():
            assert res.file_path.exists()
            assert res.file_size_bytes > 0
            assert res.triangle_count > 0
            assert res.scaled is True

        # Generate URDF
        urdf_text = parser.generate_urdf_skeleton(
            robot_name="test_robot",
            mesh_dir="meshes",
        )
        assert "<robot name=\"test_robot\">" in urdf_text
        assert "<inertial>" in urdf_text
        assert "<mass value=" in urdf_text
        assert "<inertia ixx=" in urdf_text
        assert "<mesh filename=" in urdf_text
        assert "<joint name=" in urdf_text


class TestNamingUtils:
    """Tests for URDF naming compliance."""

    def test_sanitize_name(self):
        assert sanitize_urdf_name("Base Plate 1.step") == "Base_Plate_1"
        assert sanitize_urdf_name("arm/elbow-joint:v2") == "arm_elbow_joint_v2"
        assert sanitize_urdf_name("123_invalid_start") == "n_123_invalid_start"
        assert sanitize_urdf_name("") == "link"

    def test_unique_generator(self):
        gen = UniqueNameGenerator()
        n1 = gen.get_unique_name("bolt")
        n2 = gen.get_unique_name("bolt")
        n3 = gen.get_unique_name("bolt")
        assert n1 == "bolt"
        assert n2 == "bolt_2"
        assert n3 == "bolt_3"
