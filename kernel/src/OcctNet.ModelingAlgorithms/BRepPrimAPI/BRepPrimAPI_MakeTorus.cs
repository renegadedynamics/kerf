// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeTorus.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeTorus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds tori or portions of tori: R1 is the distance from the center of the pipe to the center of the torus, R2 the
/// radius of the pipe; angle1, angle2 cut the pipe's circle (a ring segment) and angle the revolution (a pipe segment).
/// </summary>
public class BRepPrimAPI_MakeTorus : BRepPrimAPI_MakeOneAxis
{
    private readonly BRepPrim_Torus myTorus;

    /// <summary>A torus.</summary>
    public BRepPrimAPI_MakeTorus(double R1, double R2)
    {
        myTorus = new BRepPrim_Torus(R1, R2);
    }

    /// <summary>A torus pipe segment.</summary>
    public BRepPrimAPI_MakeTorus(double R1, double R2, double angle)
    {
        myTorus = new BRepPrim_Torus(R1, R2);
        myTorus.Angle(angle);
    }

    /// <summary>A torus ring segment.</summary>
    public BRepPrimAPI_MakeTorus(double R1, double R2, double angle1, double angle2)
    {
        myTorus = new BRepPrim_Torus(R1, R2);
        myTorus.VMin(angle1);
        myTorus.VMax(angle2);
    }

    /// <summary>A torus ring and pipe segment.</summary>
    public BRepPrimAPI_MakeTorus(double R1, double R2, double angle1, double angle2, double angle)
    {
        myTorus = new BRepPrim_Torus(R1, R2);
        myTorus.VMin(angle1);
        myTorus.VMax(angle2);
        myTorus.Angle(angle);
    }

    /// <summary>A torus on the axes.</summary>
    public BRepPrimAPI_MakeTorus(in Frame3D Axes, double R1, double R2)
    {
        myTorus = new BRepPrim_Torus(Axes, R1, R2);
    }

    /// <summary>A torus pipe segment on the axes.</summary>
    public BRepPrimAPI_MakeTorus(in Frame3D Axes, double R1, double R2, double angle)
    {
        myTorus = new BRepPrim_Torus(Axes, R1, R2);
        myTorus.Angle(angle);
    }

    /// <summary>A torus ring segment on the axes.</summary>
    public BRepPrimAPI_MakeTorus(in Frame3D Axes, double R1, double R2, double angle1, double angle2)
    {
        myTorus = new BRepPrim_Torus(Axes, R1, R2);
        myTorus.VMin(angle1);
        myTorus.VMax(angle2);
    }

    /// <summary>A torus ring and pipe segment on the axes.</summary>
    public BRepPrimAPI_MakeTorus(in Frame3D Axes, double R1, double R2, double angle1, double angle2, double angle)
    {
        myTorus = new BRepPrim_Torus(Axes, R1, R2);
        myTorus.VMin(angle1);
        myTorus.VMax(angle2);
        myTorus.Angle(angle);
    }

    internal override BRepPrim_OneAxis OneAxis()
    {
        return myTorus;
    }

    /// <summary>The algorithm.</summary>
    internal BRepPrim_Torus Torus()
    {
        return myTorus;
    }
}
