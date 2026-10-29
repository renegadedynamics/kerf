// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCylinder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Builds cylinders or portions of cylinders about the Z axis of the global coordinate system or of a local one,
/// limited by V in 0..H and, for a portion, U in 0..Angle (radians). The result has a lateral cylindrical face, two
/// planar faces at z = 0 and z = H and, for a portion, two planar faces at u = 0 and u = Angle.
/// </summary>
public class BRepPrimAPI_MakeCylinder : BRepPrimAPI_MakeOneAxis
{
    private readonly BRepPrim_Cylinder myCylinder;

    /// <summary>A cylinder of radius R and height H.</summary>
    public BRepPrimAPI_MakeCylinder(double R, double H)
    {
        myCylinder = new BRepPrim_Cylinder(gp.XOY(), R, H);
    }

    /// <summary>A part cylinder of radius R, height H and angle Angle.</summary>
    public BRepPrimAPI_MakeCylinder(double R, double H, double Angle)
    {
        myCylinder = new BRepPrim_Cylinder(R, H);
        myCylinder.Angle(Angle);
    }

    /// <summary>A cylinder of radius R and length H on the axes.</summary>
    public BRepPrimAPI_MakeCylinder(in gp_Ax2 Axes, double R, double H)
    {
        myCylinder = new BRepPrim_Cylinder(Axes, R, H);
    }

    /// <summary>A part cylinder of radius R, length H and angle Angle on the axes.</summary>
    public BRepPrimAPI_MakeCylinder(in gp_Ax2 Axes, double R, double H, double Angle)
    {
        myCylinder = new BRepPrim_Cylinder(Axes, R, H);
        myCylinder.Angle(Angle);
    }

    internal override BRepPrim_OneAxis OneAxis()
    {
        return myCylinder;
    }

    /// <summary>The algorithm.</summary>
    internal BRepPrim_Cylinder Cylinder()
    {
        return myCylinder;
    }
}
