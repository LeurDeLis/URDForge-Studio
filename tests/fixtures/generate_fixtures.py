"""Generate test fixtures for URDForge CAD test suite."""

from pathlib import Path
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf, gp_Vec

FIXTURES_DIR = Path(__file__).resolve().parent
FIXTURES_DIR.mkdir(parents=True, exist_ok=True)


def create_simple_box_step():
    """Create a single box: 100 x 200 x 300 mm."""
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    box = BRepPrimAPI_MakeBox(100.0, 200.0, 300.0).Shape()
    lbl = st.AddShape(box, True)
    TDataStd_Name.Set_s(lbl, TCollection_ExtendedString("calibration_box"))

    st.UpdateAssemblies()

    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.Transfer(doc)
    out_path = FIXTURES_DIR / "sample_box.step"
    writer.Write(str(out_path))
    print(f"Generated: {out_path}")


def create_nested_robot_step():
    """Create a multi-level robot assembly: Robot -> Base + UpperArm (Turret + Tool)."""
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    # 1. Base link shape (Box 120 x 120 x 40 mm)
    base_shape = BRepPrimAPI_MakeBox(120.0, 120.0, 40.0).Shape()
    l_base_def = st.AddShape(base_shape, False)
    TDataStd_Name.Set_s(l_base_def, TCollection_ExtendedString("base_pedestal"))

    # 2. Turret shape (Cylinder radius 40, height 100 mm)
    turret_shape = BRepPrimAPI_MakeCylinder(40.0, 100.0).Shape()
    l_turret_def = st.AddShape(turret_shape, False)
    TDataStd_Name.Set_s(l_turret_def, TCollection_ExtendedString("turret_cylinder"))

    # 3. Tool shape (Sphere radius 25 mm)
    tool_shape = BRepPrimAPI_MakeSphere(25.0).Shape()
    l_tool_def = st.AddShape(tool_shape, False)
    TDataStd_Name.Set_s(l_tool_def, TCollection_ExtendedString("end_effector_sphere"))

    # Subassembly: UpperArm = Turret + Tool
    l_arm_asm = st.NewShape()
    TDataStd_Name.Set_s(l_arm_asm, TCollection_ExtendedString("upper_arm_assembly"))

    t_turret = gp_Trsf()
    st.AddComponent(l_arm_asm, l_turret_def, TopLoc_Location(t_turret))

    t_tool = gp_Trsf()
    t_tool.SetTranslation(gp_Vec(0.0, 0.0, 100.0))
    st.AddComponent(l_arm_asm, l_tool_def, TopLoc_Location(t_tool))

    # Top assembly: Robot
    l_robot_asm = st.NewShape()
    TDataStd_Name.Set_s(l_robot_asm, TCollection_ExtendedString("articulated_robot"))

    t_base = gp_Trsf()
    st.AddComponent(l_robot_asm, l_base_def, TopLoc_Location(t_base))

    t_arm = gp_Trsf()
    t_arm.SetTranslation(gp_Vec(60.0, 60.0, 40.0))
    st.AddComponent(l_robot_asm, l_arm_asm, TopLoc_Location(t_arm))

    st.UpdateAssemblies()

    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.Transfer(doc)
    out_path = FIXTURES_DIR / "sample_nested_robot.step"
    writer.Write(str(out_path))
    print(f"Generated: {out_path}")


if __name__ == "__main__":
    create_simple_box_step()
    create_nested_robot_step()
