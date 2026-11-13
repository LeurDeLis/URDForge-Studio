#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDForge STEP to URDF Parser & Physical Properties Engine (Phase 1 MVP).

Author: URDForge Architectural Team
Environment: Python 3.11+ / OpenCASCADE (OCP) 7.9+ / NumPy / SciPy

Usage Examples:
    # 1. Parse STEP file and print assembly hierarchy with mass properties:
    python step_urdf_parser.py -i robot_arm.step

    # 2. Inspect and calculate inertia for a specific part with custom density (e.g. Aluminum 2700 kg/m^3):
    python step_urdf_parser.py -i robot_arm.step --part link_1 --density 2700

    # 3. Export all parts to simulation-ready local STL meshes:
    python step_urdf_parser.py -i robot_arm.step --export-all -o ./meshes/visual

    # 4. Generate JSON summary and URDF draft skeleton:
    python step_urdf_parser.py -i robot_arm.step --export-all -o ./output --json ./output/model.json --urdf ./output/robot.urdf
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Optional

# Ensure UTF-8 output encoding on Windows consoles (GBK / CP936 safe)
if sys.platform == "win32":
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass

# Ensure local package import from src/
CURRENT_DIR = Path(__file__).resolve().parent
SRC_DIR = CURRENT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

try:
    from urdforge.core.math_types import Transform3D
    from urdforge.core.mesher import MeshExportResult, TessellationConfig, export_shape_to_stl
    from urdforge.core.physics import (
        InertiaTensor,
        InertiaValidationResult,
        PhysicalProperties,
        compute_shape_physical_properties,
    )
    from urdforge.core.step_parser import AssemblyNode, AssemblyNodeType, StepAssemblyParser
except ImportError as err:
    sys.stderr.write(
        f"[ERROR] Failed to import URDForge core modules: {err}\n"
        f"Please ensure OCP (cadquery-ocp) and dependencies are installed.\n"
    )
    sys.exit(1)


def build_arg_parser() -> argparse.ArgumentParser:
    """Build command-line interface arguments."""
    parser = argparse.ArgumentParser(
        prog="step_urdf_parser",
        description="URDForge STEP CAD Assembly Parser, Physical Properties Calculator & STL Mesher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python step_urdf_parser.py -i assembly.step
  python step_urdf_parser.py -i assembly.step --part base_link --density 7850 -o ./meshes
  python step_urdf_parser.py -i assembly.step --export-all -o ./export --urdf ./export/robot.urdf
        """,
    )

    # Input file
    parser.add_argument(
        "-i",
        "--input",
        type=str,
        required=True,
        help="Path to the input STEP/STP assembly file (.step, .stp)",
    )

    # Unit scaling
    parser.add_argument(
        "--unit-scale",
        type=float,
        default=0.001,
        help="Scale factor from CAD length units to SI meters (default: 0.001 for mm -> m)",
    )

    # Physical properties calculation
    parser.add_argument(
        "--density",
        type=float,
        default=7850.0,
        help="Material density in kg/m^3 (default: 7850.0 for structural steel; 2700 for aluminum; 1050 for ABS)",
    )
    parser.add_argument(
        "--mass",
        type=float,
        default=None,
        help="Override total mass (kg) for the selected part (overrides density calculation)",
    )

    # Target part selection
    parser.add_argument(
        "--part",
        type=str,
        default=None,
        help="Filter and inspect a specific part by name or node ID",
    )

    # Mesh export options
    parser.add_argument(
        "-o",
        "--output-dir",
        type=str,
        default=None,
        help="Output directory to save exported mesh files (STL)",
    )
    parser.add_argument(
        "--export-all",
        action="store_true",
        help="Export all leaf parts in the assembly to STL files in the output directory",
    )
    parser.add_argument(
        "--frame",
        type=str,
        choices=["local", "global"],
        default="local",
        help="Coordinate frame for exported meshes: 'local' (part origin, for URDF) or 'global' (assembly origin)",
    )
    parser.add_argument(
        "--deflection",
        type=float,
        default=0.001,
        help="Linear deflection tolerance for mesh tessellation (meters if scaled, or CAD units; default: 0.001)",
    )
    parser.add_argument(
        "--angular-deflection",
        type=float,
        default=0.3,
        help="Angular deflection tolerance for mesh tessellation in radians (default: 0.3 rad ~= 17 deg)",
    )
    parser.add_argument(
        "--ascii-stl",
        action="store_true",
        help="Export STL in ASCII format instead of default compact binary format",
    )

    # Export formats & URDF preview
    parser.add_argument(
        "--json",
        type=str,
        default=None,
        help="Path to save full assembly tree & physical properties report as JSON",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="Path to generate a simulation-ready URDF XML skeleton preserving CAD kinematics",
    )
    parser.add_argument(
        "--robot-name",
        type=str,
        default="urdforge_robot",
        help="Robot model name for URDF generation (default: urdforge_robot)",
    )

    return parser


def print_part_details(node: AssemblyNode, unit_scale: float) -> None:
    """Print comprehensive diagnostic and physical report for a part."""
    print("\n" + "=" * 78)
    print(f"🔍 Part Inspection: {node.name}")
    print("=" * 78)
    print(f"• Node ID        : {node.id}")
    print(f"• Original Name  : {node.original_name}")
    if node.instance_name:
        print(f"• Instance Name  : {node.instance_name}")
    if node.definition_name:
        print(f"• Definition Name: {node.definition_name}")
    print(f"• Node Type      : {node.node_type.value.upper()}")
    print(f"• XCAF Entry     : {node.label_entry}")

    # Transforms
    t_local = node.local_transform.translation * unit_scale
    r_local, p_local, y_local = node.local_transform.to_rpy()
    print(
        f"• Local Origin   : [{t_local[0]:.6f}, {t_local[1]:.6f}, {t_local[2]:.6f}] m | "
        f"RPY: [{r_local:.6f}, {p_local:.6f}, {y_local:.6f}] rad"
    )

    t_glob = node.global_transform.translation * unit_scale
    r_glob, p_glob, y_glob = node.global_transform.to_rpy()
    print(
        f"• Global Origin  : [{t_glob[0]:.6f}, {t_glob[1]:.6f}, {t_glob[2]:.6f}] m | "
        f"RPY: [{r_glob:.6f}, {p_glob:.6f}, {y_glob:.6f}] rad"
    )

    # Bounding Box
    if node.bounding_box_local:
        bx = node.bounding_box_local
        dx, dy, dz = bx[3] - bx[0], bx[4] - bx[1], bx[5] - bx[2]
        print(f"• Local BBox     : [{bx[0]:.4f}, {bx[1]:.4f}, {bx[2]:.4f}] to [{bx[3]:.4f}, {bx[4]:.4f}, {bx[5]:.4f}] m")
        print(f"• Dimensions     : {dx*1000:.1f} x {dy*1000:.1f} x {dz*1000:.1f} mm")

    # Physical Properties
    if node.physical_props is not None:
        p = node.physical_props
        print("-" * 78)
        print("⚖️  MASS & INERTIA PROPERTIES (Simulation-Ready)")
        print("-" * 78)
        print(f"• Volume         : {p.volume_mm3:,.2f} mm³ ({p.volume_m3:.6e} m³)")
        print(f"• Density        : {p.density:,.1f} kg/m³")
        print(f"• Total Mass     : {p.mass:.6f} kg")
        cx, cy, cz = p.com_local
        print(f"• Center of Mass : [{cx:.6f}, {cy:.6f}, {cz:.6f}] m (Local Link Frame)")
        cgx, cgy, cgz = p.com_global
        print(f"• Center of Mass : [{cgx:.6f}, {cgy:.6f}, {cgz:.6f}] m (Global Assembly Frame)")

        # 3x3 Inertia Matrix
        i = p.inertia_tensor
        print("\n• 3x3 Inertia Tensor about CoM (kg·m²):")
        print(f"  ┌ {i.ixx:+13.6e}  {i.ixy:+13.6e}  {i.ixz:+13.6e} ┐")
        print(f"  │ {i.ixy:+13.6e}  {i.iyy:+13.6e}  {i.iyz:+13.6e} │")
        print(f"  └ {i.ixz:+13.6e}  {i.iyz:+13.6e}  {i.izz:+13.6e} ┘")

        # Principal Moments
        v = p.validation
        p1, p2, p3 = v.principal_moments
        print(f"\n• Principal Moments (I1, I2, I3): [{p1:.6e}, {p2:.6e}, {p3:.6e}] kg·m²")
        print(f"• Determinant    : {v.determinant:.6e}")

        # Validation status
        print("\n• Physics Feasibility Check (Isaac Sim / MuJoCo / Gazebo Solver Standards):")
        if v.is_valid:
            print("  ✅ PASS: Positive-definite and satisfies triangle inequalities.")
        else:
            print("  ❌ FAIL: Inertia properties violate physical simulation constraints!")
            for err in v.errors:
                print(f"     [ERROR] {err}")
        for w in v.warnings:
            print(f"     [WARN]  {w}")

        # URDF snippet
        print("\n• Ready URDF <inertial> Block:")
        print(p.to_urdf_inertial_xml(indent=2))

    print("=" * 78 + "\n")


def main() -> int:
    """Main execution pipeline."""
    parser = build_arg_parser()
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        sys.stderr.write(f"[ERROR] Input file does not exist: {input_path}\n")
        return 1

    print(f"\n🚀 [URDForge] Parsing STEP Assembly: {input_path.name}")
    print(f"   • CAD to Meter Scale : {args.unit_scale}")
    print(f"   • Material Density   : {args.density} kg/m³")

    # 1. Parse STEP file
    try:
        step_parser = StepAssemblyParser(
            filepath=input_path,
            unit_scale=args.unit_scale,
            default_density=args.density,
        )
        root_node = step_parser.load()
    except Exception as e:
        sys.stderr.write(f"\n[ERROR] Failed to parse STEP assembly: {e}\n")
        return 1

    # 2. Print assembly hierarchy
    step_parser.print_hierarchy()

    all_parts = root_node.get_all_parts()
    print(f"📊 Assembly Summary: {len(all_parts)} leaf part(s) detected.")

    # 3. Handle specific part inspection / property override
    selected_node = None
    if args.part:
        selected_node = root_node.find_node_by_name(args.part) or root_node.find_node_by_id(args.part)
        if selected_node is None:
            # Fuzzy match
            candidates = [p for p in all_parts if args.part.lower() in p.name.lower()]
            if candidates:
                selected_node = candidates[0]
                print(f"ℹ️  Matched '{args.part}' to part '{selected_node.name}'")

        if selected_node is None:
            sys.stderr.write(f"[WARN] Part '{args.part}' not found in assembly.\n")
        else:
            if args.mass is not None:
                print(f"ℹ️  Overriding mass for '{selected_node.name}' with {args.mass:.4f} kg")
                step_parser.compute_node_properties(selected_node, mass=args.mass)
            elif args.density != 7850.0:
                step_parser.compute_node_properties(selected_node, density=args.density)

            print_part_details(selected_node, args.unit_scale)

    # 4. Handle mesh export
    out_dir = Path(args.output_dir) if args.output_dir else None
    tess_config = TessellationConfig(
        linear_deflection=args.deflection,
        angular_deflection=args.angular_deflection,
        scale_to_meter=True,
        unit_scale=args.unit_scale,
        binary_stl=not args.ascii_stl,
    )

    if args.export_all and out_dir:
        print(f"\n📦 Exporting visual STL meshes for all {len(all_parts)} parts to: {out_dir}")
        mesh_results = step_parser.export_all_parts(
            output_dir=out_dir,
            config=tess_config,
            frame=args.frame,
        )
        print("=" * 78)
        print(f"{'Part Name':<30} {'File Size':<12} {'Triangles':<12} {'Vertices':<12}")
        print("-" * 78)
        total_tris = 0
        total_bytes = 0
        for name, res in mesh_results.items():
            total_tris += res.triangle_count
            total_bytes += res.file_size_bytes
            size_kb = res.file_size_bytes / 1024.0
            print(f"{name:<30} {size_kb:>8.1f} KB  {res.triangle_count:>10,d}  {res.vertex_count:>10,d}")
        print("-" * 78)
        print(f"Total: {len(mesh_results)} files | {total_bytes / 1024 / 1024:.2f} MB | {total_tris:,d} triangles\n")

    elif selected_node and out_dir:
        stl_name = f"{selected_node.name}.stl"
        target_path = out_dir / stl_name
        print(f"\n📦 Exporting STL mesh for '{selected_node.name}' to: {target_path}")
        res = step_parser.export_node_stl(
            node=selected_node,
            output_path=target_path,
            config=tess_config,
            frame=args.frame,
        )
        print(
            f"✅ Exported: {res.file_path.name} ({res.file_size_bytes / 1024:.1f} KB, "
            f"{res.triangle_count:,d} triangles, frame={args.frame})"
        )

    # 5. Handle JSON serialization
    if args.json:
        json_path = Path(args.json).resolve()
        json_path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "source_file": str(input_path.resolve()),
            "unit_scale": args.unit_scale,
            "default_density": args.density,
            "assembly_tree": root_node.to_dict(),
        }
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"💾 Assembly structure and mass properties saved to JSON: {json_path}")

    # 6. Handle URDF draft generation
    if args.urdf:
        urdf_path = Path(args.urdf).resolve()
        urdf_path.parent.mkdir(parents=True, exist_ok=True)
        rel_mesh_dir = "meshes/visual"
        if out_dir:
            try:
                rel_mesh_dir = str(out_dir.relative_to(urdf_path.parent)).replace("\\", "/")
            except ValueError:
                rel_mesh_dir = str(out_dir).replace("\\", "/")

        urdf_content = step_parser.generate_urdf_skeleton(
            robot_name=args.robot_name,
            mesh_dir=rel_mesh_dir,
        )
        with open(urdf_path, "w", encoding="utf-8") as f:
            f.write(urdf_content)
        print(f"🤖 URDF robot model generated: {urdf_path}")

    print("\n✨ Done! Phase 1 CAD & URDF parsing completed successfully.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
