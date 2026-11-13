"""Comprehensive Integration & Unit Tests for URDForge Studio Phases 2-4.

Tests robot modeling, feature snapping, forward kinematics (FK),
ROS 2 packaging, and FastAPI backend endpoints.
"""

import math
from pathlib import Path
import pytest
import numpy as np

from urdforge.core.math_types import Transform3D
from urdforge.core.robot_model import (
    JointLimits,
    JointModel,
    JointType,
    LinkModel,
    RobotModel,
    VisualGeometry,
)
from urdforge.core.features import (
    FeatureDetector,
    FeatureType,
    align_z_to_vector,
)
from urdforge.core.kinematics import ForwardKinematicsEngine
from urdforge.exporter.ros_package import ROSPackageConfig, ROSPackageExporter
from urdforge.core.step_parser import StepAssemblyParser

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


class TestRobotModel:
    """Tests for robot kinematics and URDF aggregation model."""

    def test_robot_hierarchy_and_urdf(self):
        robot = RobotModel(name="test_arm", root_link="base_link")
        link1 = LinkModel(name="link_1")
        robot.add_link(link1)

        joint1 = JointModel(
            name="joint_1",
            parent_link="base_link",
            child_link="link_1",
            joint_type=JointType.REVOLUTE,
            origin=Transform3D.from_translation(0.0, 0.0, 0.1),
            axis=(0.0, 0.0, 1.0),
            limits=JointLimits(lower=-1.57, upper=1.57),
        )
        robot.add_joint(joint1)

        assert len(robot.links) == 2
        assert len(robot.joints) == 1

        xml = robot.to_urdf_xml()
        assert '<robot name="test_arm">' in xml
        assert '<link name="base_link">' in xml
        assert '<link name="link_1">' in xml
        assert '<joint name="joint_1" type="revolute">' in xml
        assert '<limit lower="-1.570000" upper="1.570000"' in xml


class TestFeatureSnapping:
    """Tests for B-Rep cylinder and circular edge detection and alignment."""

    def test_align_z_vector(self):
        # Align Z to X-axis [1, 0, 0]
        t = align_z_to_vector([1.0, 0.0, 0.0])
        z_orig = np.array([0.0, 0.0, 1.0])
        z_new = np.matmul(t.rotation_matrix, z_orig)
        assert np.allclose(z_new, [1.0, 0.0, 0.0], atol=1e-6)

    def test_cylinder_midpoint_calculation(self):
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
        shape = BRepPrimAPI_MakeCylinder(15.0, 40.0).Shape()  # R=15mm, H=40mm
        detector = FeatureDetector(unit_scale=0.001)
        feats = detector.detect_features(shape, "test_cyl")
        cyl_feats = [f for f in feats if f.feature_type == FeatureType.CYLINDER_AXIS]
        assert len(cyl_feats) >= 1
        cyl = cyl_feats[0]
        # Midpoint should be at z = 20mm = 0.02m
        assert np.isclose(cyl.center_mid[2], 0.02, atol=1e-4)
        assert np.isclose(cyl.center_bot[2], 0.0, atol=1e-4)
        assert np.isclose(cyl.center_top[2], 0.04, atol=1e-4)
        assert np.isclose(cyl.length, 0.04, atol=1e-4)

        # Align Z to -Z-axis [0, 0, -1]
        z_orig = np.array([0.0, 0.0, 1.0])
        t_neg = align_z_to_vector([0.0, 0.0, -1.0])
        z_neg = np.matmul(t_neg.rotation_matrix, z_orig)
        assert np.allclose(z_neg, [0.0, 0.0, -1.0], atol=1e-6)
        assert np.allclose(z_neg, [0.0, 0.0, -1.0], atol=1e-6)

    def test_feature_detection_on_robot_step(self):
        step_file = FIXTURES_DIR / "sample_nested_robot.step"
        parser = StepAssemblyParser(filepath=step_file, unit_scale=0.001)
        root = parser.load()

        detector = FeatureDetector(unit_scale=0.001)
        all_features = []
        for part in root.get_all_parts():
            feats = detector.detect_features(
                shape=part.shape,
                part_name=part.name,
                global_transform=part.global_transform,
            )
            all_features.extend(feats)

        # Turret cylinder and circle edges must be detected
        assert len(all_features) >= 2
        cyl_features = [f for f in all_features if f.feature_type == FeatureType.CYLINDER_AXIS]
        assert len(cyl_features) >= 1
        cyl = cyl_features[0]
        assert math.isclose(cyl.radius, 0.040, abs_tol=1e-3)  # Radius 40mm = 0.04m

        # Test snapping joint to this cylinder feature
        joint = cyl.snap_joint(
            joint_name="joint_turret",
            parent_link="base_pedestal",
            child_link="turret_cylinder",
            joint_type=JointType.REVOLUTE,
        )
        assert joint.name == "joint_turret"
        assert joint.parent_link == "base_pedestal"
        assert joint.child_link == "turret_cylinder"
        assert joint.joint_type == JointType.REVOLUTE


class TestForwardKinematics:
    """Tests for FK motion simulation."""

    def test_revolute_joint_rotation(self):
        robot = RobotModel(name="two_link", root_link="base_link")
        link1 = LinkModel(name="link_1")
        robot.add_link(link1)

        # Joint at (1, 0, 0), rotation about Z
        joint1 = JointModel(
            name="j1",
            parent_link="base_link",
            child_link="link_1",
            joint_type=JointType.REVOLUTE,
            origin=Transform3D.from_translation(1.0, 0.0, 0.0),
            axis=(0.0, 0.0, 1.0),
            limits=JointLimits(lower=-math.pi, upper=math.pi),
        )
        robot.add_joint(joint1)

        fk_engine = ForwardKinematicsEngine(robot)

        # At angle 0, link_1 is at (1, 0, 0)
        res0 = fk_engine.evaluate({"j1": 0.0})
        p0 = res0.link_poses_world["link_1"].translation
        assert np.allclose(p0, [1.0, 0.0, 0.0], atol=1e-6)

        # Rotate +90 degrees (pi/2)
        res90 = fk_engine.evaluate({"j1": math.pi / 2.0})
        p90 = res90.link_poses_world["link_1"].translation
        assert np.allclose(p90, [1.0, 0.0, 0.0], atol=1e-6)  # Joint origin itself does not move
        # Check rotation matrix
        r90 = res90.link_poses_world["link_1"].rotation_matrix
        # Point [0.5, 0, 0] in link_1 local frame should now be rotated by 90 deg -> [0, 0.5, 0]
        p_pt = np.matmul(r90, np.array([0.5, 0.0, 0.0]))
        assert np.allclose(p_pt, [0.0, 0.5, 0.0], atol=1e-6)


class TestROSPackageExporter:
    """Tests for complete ROS 2 package generation."""

    def test_export_package(self, tmp_path):
        robot = RobotModel(name="my_arm", root_link="base_link")
        robot.add_link(LinkModel(name="base_link"))
        robot.add_link(
            LinkModel(
                name="arm_link",
                visuals=[VisualGeometry(mesh_filepath="meshes/arm_part.stl")],
                collisions=[VisualGeometry(mesh_filepath="meshes/arm_part.stl")],
            )
        )
        robot.add_joint(
            JointModel(
                name="joint_1",
                parent_link="base_link",
                child_link="arm_link",
                joint_type=JointType.REVOLUTE,
            )
        )

        cfg = ROSPackageConfig(
            package_name="my_arm_description",
            robot_name="my_arm",
        )
        exporter = ROSPackageExporter(robot, cfg)
        out_dir = exporter.export(target_directory=tmp_path)

        assert out_dir.exists()
        assert (out_dir / "package.xml").exists()
        assert (out_dir / "CMakeLists.txt").exists()
        assert (out_dir / "urdf" / "my_arm.urdf").exists()
        assert (out_dir / "launch" / "display.launch.py").exists()
        assert (out_dir / "launch" / "display.launch").exists()
        assert (out_dir / "rviz" / "urdf.rviz").exists()

        # Check content
        pkg_xml = (out_dir / "package.xml").read_text(encoding="utf-8")
        assert "<name>my_arm_description</name>" in pkg_xml
        assert "ament_cmake" in pkg_xml

        urdf_text = (out_dir / "urdf" / "my_arm.urdf").read_text(encoding="utf-8")
        assert "package://my_arm_description/meshes/visual/arm_part.stl" in urdf_text
        assert "package://my_arm_description/meshes/collision/arm_part.stl" in urdf_text


class TestAPIEndpoints:
    """Tests for FastAPI backend endpoints."""

    def test_api_full_flow(self, tmp_path):
        from starlette.testclient import TestClient
        from urdforge.server.app import app

        client = TestClient(app)

        # 1. Root index
        r0 = client.get("/")
        assert r0.status_code == 200

        # 2. Load step
        step_file = FIXTURES_DIR / "sample_nested_robot.step"
        r1 = client.post("/api/step/load", json={"filepath": str(step_file)})
        assert r1.status_code == 200
        data = r1.json()
        assert data["status"] == "success"
        assert data["parts_count"] == 3

        # 3. Features
        r2 = client.get("/api/step/features")
        assert r2.status_code == 200
        feats = r2.json()
        assert len(feats) >= 2

        # 4. Snap Joint
        first_feat = feats[0]
        r3 = client.post(
            "/api/robot/snap_joint",
            json={
                "feature_id": first_feat["id"],
                "joint_name": "snapped_joint",
                "parent_link": "base_link",
                "child_link": first_feat["part_name"],
                "joint_type": "revolute",
                "flip_axis": False,
            },
        )
        assert r3.status_code == 200
        assert r3.json()["status"] == "success"

        # 5. FK calculation
        r4 = client.post("/api/robot/fk", json={"joint_positions": {"snapped_joint": 0.5}})
        assert r4.status_code == 200
        fk_data = r4.json()
        assert "link_poses" in fk_data

        # 6. URDF export
        r5 = client.get("/api/robot/urdf")
        assert r5.status_code == 200
        assert "<robot" in r5.text
        assert "meshes/visual/" in r5.text
        assert "meshes/collision/" in r5.text

        # 7. ROS package export
        out_pkg_dir = tmp_path / "ros_out"
        r6 = client.post(
            "/api/robot/export_ros",
            json={"target_directory": str(out_pkg_dir)},
        )
        assert r6.status_code == 200
        res6 = r6.json()
        assert res6["status"] == "success"
        pkg_name = res6["package_name"]
        exported_pkg = out_pkg_dir / pkg_name
        exported_urdf_file = exported_pkg / "urdf" / f"{data['robot_name']}.urdf"
        assert exported_urdf_file.exists()
        urdf_text = exported_urdf_file.read_text(encoding="utf-8")
        assert f"package://{pkg_name}/meshes/visual/" in urdf_text
        assert f"package://{pkg_name}/meshes/collision/" in urdf_text
        assert (exported_pkg / "meshes" / "visual").exists()
        assert (exported_pkg / "meshes" / "collision").exists()

        # 8. Create new custom link starting from base_link
        r7 = client.post(
            "/api/robot/link/create",
            json={
                "name": "arm_assembly_link",
                "parent_link": "base_link",
                "joint_name": "base_to_arm_joint",
                "joint_type": "revolute",
            },
        )
        assert r7.status_code == 200
        assert r7.json()["status"] == "success"

        # 9. Assign multiple parts to the new link
        r8 = client.post(
            "/api/robot/link/assign_parts",
            json={
                "link_name": "arm_assembly_link",
                "part_names": ["turret_cylinder", "end_effector_sphere"],
            },
        )
        assert r8.status_code == 200
        assert r8.json()["status"] == "success"
        link_data = r8.json()["link"]
        assert len(link_data["cad_part_ids"]) == 2
        assert link_data["inertial"] is not None
        assert link_data["inertial"]["mass_kg"] > 0.1
        assert link_data["inertial"]["mass"] == link_data["inertial"]["mass_kg"]
        assert link_data["inertial"]["com_xyz"] == link_data["inertial"]["com_local_m"]
        assert len(link_data["inertial"]["inertia_matrix"]) == 3

        # 10. Snap joint to cylinder midpoint
        cyl_feat = next(f for f in feats if "cyl" in f["type"])
        r9 = client.post(
            "/api/robot/snap_joint",
            json={
                "feature_id": cyl_feat["id"],
                "joint_name": "base_to_arm_joint",
                "parent_link": "base_link",
                "child_link": "arm_assembly_link",
                "joint_type": "revolute",
                "flip_axis": False,
                "snap_point": "midpoint",
            },
        )
        assert r9.status_code == 200
        snap_res = r9.json()
        assert snap_res["status"] == "success"
        # Origin should match cylinder midpoint
        assert np.allclose(
            snap_res["joint"]["origin"]["xyz"],
            cyl_feat["center_mid"],
            atol=1e-5,
        )

        # 11. FK with part poses
        r10 = client.post(
            "/api/robot/fk",
            json={"joint_positions": {"base_to_arm_joint": 0.785}},
        )
        assert r10.status_code == 200
        fk_resp = r10.json()
        assert "part_poses" in fk_resp
        assert "turret_cylinder" in fk_resp["part_poses"]
        assert "end_effector_sphere" in fk_resp["part_poses"]

        # 12. Clear parts from link
        r11 = client.post(
            "/api/robot/link/clear_parts",
            json={"link_name": "arm_assembly_link"},
        )
        assert r11.status_code == 200
        assert r11.json()["status"] == "success"
        assert len(r11.json()["link"]["cad_part_ids"]) == 0

        # 13. Test raw URDF XML export endpoint (no JSON quotes)
        r12 = client.get("/api/robot/urdf")
        assert r12.status_code == 200
        assert "application/xml" in r12.headers.get("content-type", "")
        raw_xml = r12.text
        assert raw_xml.startswith('<?xml version="1.0"')
        assert '<robot name="sample_nested_robot">' in raw_xml

        # 14. Test updating robot via URDF XML edit endpoint
        # Replace axis in raw_xml with new axis xyz="0 1 0"
        import re
        modified_xml = re.sub(
            r'<axis xyz="[^"]+"/>',
            '<axis xyz="0 1 0"/>',
            raw_xml
        )
        r13 = client.post(
            "/api/robot/urdf/update",
            json={"urdf_xml": modified_xml}
        )
        assert r13.status_code == 200
        res13 = r13.json()
        assert res13["status"] == "success"
        assert "urdf_xml" in res13
        assert '<axis xyz="0.000000 1.000000 0.000000"/>' in res13["urdf_xml"]

        # 15. Test manual physics override API
        r14 = client.post(
            "/api/robot/link/override_physics",
            json={
                "link_name": "arm_assembly_link",
                "mass": 2.5,
                "com": [0.01, 0.02, 0.03],
                "ixx": 0.01,
                "iyy": 0.01,
                "izz": 0.01,
                "ixy": 0.0,
                "ixz": 0.0,
                "iyz": 0.0,
                "reset_auto": False,
            },
        )
        assert r14.status_code == 200
        res14 = r14.json()
        assert res14["status"] == "success"
        assert res14["link"]["has_manual_inertial"] is True
        assert res14["link"]["inertial"]["mass"] == 2.5

        # 16. Test renaming link
        r15 = client.post(
            "/api/robot/link/rename",
            json={
                "old_name": "arm_assembly_link",
                "new_name": "forearm_link",
            },
        )
        assert r15.status_code == 200
        res15 = r15.json()
        assert res15["status"] == "success"
        assert "forearm_link" in res15["robot_model"]["links"]
        assert "arm_assembly_link" not in res15["robot_model"]["links"]
        # Joint referencing arm_assembly_link as child must be updated
        j_updated = res15["robot_model"]["joints"]["base_to_arm_joint"]
        assert j_updated["child_link"] == "forearm_link"

        # 17. Test renaming link to existing name (should fail 400)
        r16 = client.post(
            "/api/robot/link/rename",
            json={
                "old_name": "forearm_link",
                "new_name": "base_link",
            },
        )
        assert r16.status_code == 400

        # 18. Test renaming root link (base_link -> robot_base)
        r17 = client.post(
            "/api/robot/link/rename",
            json={
                "old_name": "base_link",
                "new_name": "robot_base",
            },
        )
        assert r17.status_code == 200
        res17 = r17.json()
        assert res17["robot_model"]["root_link"] == "robot_base"
        assert res17["robot_model"]["joints"]["base_to_arm_joint"]["parent_link"] == "robot_base"

        # 19. Test renaming non-existent link (should fail 404)
        r18 = client.post(
            "/api/robot/link/rename",
            json={
                "old_name": "non_existent_link",
                "new_name": "new_name",
            },
        )
        assert r18.status_code == 404

        # 20. Test recalculate_all_physics endpoint
        r19 = client.post("/api/robot/recalculate_all_physics", json={"force_reset": True})
        assert r19.status_code == 200
        res19 = r19.json()
        assert res19["status"] == "success"
        assert "robot_model" in res19


class TestMeshMassProperties:
    """Tests for direct mesh polyhedral mass and inertia property computation."""

    def test_unit_cube_analytical_properties(self):
        from urdforge.core.physics import compute_mesh_triangles_mass_properties

        cube_nodes = np.array([
            [0,0,0], [1,0,0], [1,1,0], [0,1,0],
            [0,0,1], [1,0,1], [1,1,1], [0,1,1]
        ], dtype=float)
        cube_tris_idx = np.array([
            [0,2,1], [0,3,2],
            [4,5,6], [4,6,7],
            [0,1,5], [0,5,4],
            [2,3,7], [2,7,6],
            [0,4,7], [0,7,3],
            [1,2,6], [1,6,5]
        ])
        tris = cube_nodes[cube_tris_idx]
        vol, com, inertia_mat = compute_mesh_triangles_mass_properties(tris, density=1.0)

        assert pytest.approx(vol, rel=1e-5) == 1.0
        assert np.allclose(com, [0.5, 0.5, 0.5], atol=1e-5)
        # Analytical inertia for unit cube of mass 1: I_xx = (1/6) * m * L^2 = 1/6
        assert pytest.approx(inertia_mat[0, 0], rel=1e-5) == 1.0 / 6.0
        assert pytest.approx(inertia_mat[1, 1], rel=1e-5) == 1.0 / 6.0
        assert pytest.approx(inertia_mat[2, 2], rel=1e-5) == 1.0 / 6.0
        assert abs(inertia_mat[0, 1]) < 1e-6
        assert abs(inertia_mat[0, 2]) < 1e-6
        assert abs(inertia_mat[1, 2]) < 1e-6


class TestCadUpAxisConversion:
    """Tests for SolidWorks Y-up to ROS Z-up coordinate system conversion."""

    def test_solidworks_y_to_z_mapping(self):
        from urdforge.core.step_parser import get_cad_up_axis_transform

        t_y = get_cad_up_axis_transform("Y")
        # SolidWorks +Y up vector (0, 1, 0) must rotate +90 deg around X -> ROS +Z up (0, 0, 1)
        p_up = t_y.transform_point([0.0, 0.05, 0.0])
        assert np.allclose(p_up, [0.0, 0.0, 0.05], atol=1e-7)

        # -Y up mapping should rotate -90 deg around X -> (0, -0.05, 0) -> (0, 0, 0.05)
        t_neg_y = get_cad_up_axis_transform("-Y")
        p_up_neg = t_neg_y.transform_point([0.0, -0.05, 0.0])
        assert np.allclose(p_up_neg, [0.0, 0.0, 0.05], atol=1e-7)

    def test_api_set_cad_up_axis_roundtrip(self):
        from fastapi.testclient import TestClient
        from urdforge.server.app import app

        client = TestClient(app)
        step_file = FIXTURES_DIR / "sample_nested_robot.step"

        # Load with raw Z-up
        r_load = client.post("/api/step/load", json={"filepath": str(step_file), "cad_up_axis": "Z"})
        assert r_load.status_code == 200
        parts_z = {}
        def _collect(node):
            if node.get("is_leaf"):
                parts_z[node["name"]] = np.array(node["global_transform"]["xyz"])
            for c in node.get("children", []):
                _collect(c)
        _collect(r_load.json()["assembly_tree"])
        orig_turret_t = parts_z["turret_cylinder"]

        # Assign parts and create a joint before switching orientation
        client.post("/api/robot/link/create", json={
            "name": "turret_link",
            "parent_link": "base_link",
            "joint_name": "turret_joint",
            "joint_type": "revolute",
            "part_names": ["turret_cylinder"],
        })
        client.post("/api/robot/add_joint", json={
            "name": "turret_joint",
            "parent_link": "base_link",
            "child_link": "turret_link",
            "joint_type": "revolute",
            "origin_xyz": [0.06, 0.06, 0.04],
            "origin_rpy": [0.0, 0.0, 0.0],
        })

        # Switch CAD up-axis to SolidWorks 'Y' (+90 deg around X: (x, y, z) -> (x, -z, y))
        r_sw = client.post("/api/step/set_cad_up_axis", json={"cad_up_axis": "Y"})
        assert r_sw.status_code == 200
        res_sw = r_sw.json()
        assert res_sw["cad_up_axis"] == "Y"
        assert np.allclose(res_sw["robot_model"]["joints"]["turret_joint"]["origin"]["xyz"], [0.06, -0.04, 0.06], atol=1e-6)

        parts_sw = {}
        def _collect_sw(node):
            if node.get("is_leaf"):
                parts_sw[node["name"]] = np.array(node["global_transform"]["xyz"])
            for c in node.get("children", []):
                _collect_sw(c)
        _collect_sw(res_sw["assembly_tree"])
        expected_turret_t = np.array([orig_turret_t[0], -orig_turret_t[2], orig_turret_t[1]])
        assert np.allclose(parts_sw["turret_cylinder"], expected_turret_t, atol=1e-5)

        # Switch back to 'Z' -> roundtrip exact restoration
        r_back = client.post("/api/step/set_cad_up_axis", json={"cad_up_axis": "Z"})
        assert r_back.status_code == 200
        res_back = r_back.json()
        j_origin = res_back["robot_model"]["joints"]["turret_joint"]["origin"]["xyz"]
        assert np.allclose(j_origin, [0.06, 0.06, 0.04], atol=1e-6)


class TestKinematicIntegrity:
    """Test suite for kinematic tree graph reachability, orphan detection, and disconnection simulation."""

    def test_orphan_detection_and_disconnection_simulation(self):
        robot = RobotModel(name="integrity_robot", root_link="base_link")
        for i in range(1, 4):
            robot.add_link(LinkModel(name=f"link_{i}"))

        # Chain: base_link -> link_1 -> link_2 -> link_3
        robot.add_joint(JointModel(name="joint_1", parent_link="base_link", child_link="link_1"))
        robot.add_joint(JointModel(name="joint_2", parent_link="link_1", child_link="link_2"))
        robot.add_joint(JointModel(name="joint_3", parent_link="link_2", child_link="link_3"))

        assert set(robot.get_reachable_links()) == {"base_link", "link_1", "link_2", "link_3"}
        assert robot.get_orphan_links() == []

        # Removing joint_2 should orphan link_2 and link_3
        orphans_on_j2 = robot.simulate_orphans_if_joint_removed("joint_2")
        assert set(orphans_on_j2) == {"link_2", "link_3"}

        # Removing link_1 should orphan link_2 and link_3 (excluding link_1 which is being deleted)
        orphans_on_l1 = robot.simulate_orphans_if_link_removed("link_1")
        assert set(orphans_on_l1) == {"link_2", "link_3"}

        # Removing terminal joint_3 should only orphan link_3
        orphans_on_j3 = robot.simulate_orphans_if_joint_removed("joint_3")
        assert orphans_on_j3 == ["link_3"]


class TestJointLimitsAndPrecision:
    """Test suite for 0 limit support and high-precision prismatic rail limits."""

    def test_revolute_zero_limits(self):
        limits = JointLimits(lower=0.0, upper=0.0)
        assert limits.lower == 0.0
        assert limits.upper == 0.0

        d = limits.to_dict()
        limits2 = JointLimits.from_dict(d)
        assert limits2.lower == 0.0
        assert limits2.upper == 0.0

        joint = JointModel(
            name="zero_limit_joint",
            parent_link="base_link",
            child_link="link_1",
            joint_type=JointType.REVOLUTE,
            limits=limits,
        )
        xml = joint.to_urdf_xml()
        assert '<limit lower="0.000000" upper="0.000000"' in xml

    def test_prismatic_micro_precision_limits_and_fk(self):
        p_limits = JointLimits(lower=0.0000001, upper=0.0005)
        p_joint = JointModel(
            name="rail_joint",
            parent_link="base_link",
            child_link="slider_link",
            joint_type=JointType.PRISMATIC,
            axis=(0.0, 0.0, 1.0),
            limits=p_limits,
        )
        xml = p_joint.to_urdf_xml()
        # Must format with 7 decimal places for prismatic
        assert '<limit lower="0.0000001" upper="0.0005000"' in xml

        # FK evaluation with 0.1 µm displacement
        robot = RobotModel(name="rail_robot", root_link="base_link")
        robot.add_link(LinkModel(name="slider_link"))
        robot.add_joint(p_joint)

        fk = ForwardKinematicsEngine(robot)
        res = fk.evaluate({"rail_joint": 0.0000001})
        t_child = res.link_poses_world["slider_link"].translation
        assert np.isclose(t_child[2], 0.0000001, atol=1e-9)
        assert len(res.limit_violations) == 0

        # In-range motion with 0.0002 m (< 0.001 m)
        res2 = fk.evaluate({"rail_joint": 0.0002})
        assert np.isclose(res2.link_poses_world["slider_link"].translation[2], 0.0002, atol=1e-9)
        assert len(res2.limit_violations) == 0

    def test_api_add_joint_zero_limit_and_prismatic(self):
        from fastapi.testclient import TestClient
        from urdforge.server.app import app

        client = TestClient(app)
        client.post("/api/robot/link/create", json={"name": "test_link_rev"})
        client.post("/api/robot/link/create", json={"name": "test_link_pris"})

        # 1. Revolute joint with lower=0.0 and upper=0.0
        r_rev = client.post("/api/robot/add_joint", json={
            "name": "rev_zero_joint",
            "parent_link": "base_link",
            "child_link": "test_link_rev",
            "joint_type": "revolute",
            "lower_limit": 0.0,
            "upper_limit": 0.0,
        })
        assert r_rev.status_code == 200
        j_rev_limits = r_rev.json()["joint"]["limits"]
        assert j_rev_limits["lower"] == 0.0
        assert j_rev_limits["upper"] == 0.0

        # 2. Prismatic joint with 1e-7 and 0.0005
        r_pris = client.post("/api/robot/add_joint", json={
            "name": "pris_micro_joint",
            "parent_link": "base_link",
            "child_link": "test_link_pris",
            "joint_type": "prismatic",
            "lower_limit": 0.0000001,
            "upper_limit": 0.0005,
        })
        assert r_pris.status_code == 200
        j_pris_limits = r_pris.json()["joint"]["limits"]
        assert np.isclose(j_pris_limits["lower"], 0.0000001, atol=1e-9)
        assert np.isclose(j_pris_limits["upper"], 0.0005, atol=1e-9)



