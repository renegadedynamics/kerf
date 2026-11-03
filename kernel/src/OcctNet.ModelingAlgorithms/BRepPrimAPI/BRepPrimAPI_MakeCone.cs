// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCone.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds cones or portions of cones of height H, radius R1 at z = 0 and R2 at z = H (either may be 0), about the Z
/// axis of the global coordinate system or of a local one; a portion is limited by U in 0..angle.
/// </summary>
/// <remarks>
/// Throws Standard_DomainError if H is below Precision::Confusion(), if R1 and R2 are equal, or if a radius is
/// negative or not null and below Precision::Confusion().
/// </remarks>
public class BRepPrimAPI_MakeCone : BRepPrimAPI_MakeOneAxis
{
    private readonly BRepPrim_Cone myCone;

    /// <summary>A cone of radius R1 at z = 0, R2 at z = H.</summary>
    public BRepPrimAPI_MakeCone(double R1, double R2, double H)
    {
        myCone = new BRepPrim_Cone(Constants.XOY(), R1, R2, H);
    }

    /// <summary>A part cone of radius R1 at z = 0, R2 at z = H and angle.</summary>
    public BRepPrimAPI_MakeCone(double R1, double R2, double H, double angle)
    {
        myCone = new BRepPrim_Cone(R1, R2, H);
        myCone.Angle(angle);
    }

    /// <summary>A cone on the axes.</summary>
    public BRepPrimAPI_MakeCone(in Frame3D Axes, double R1, double R2, double H)
    {
        myCone = new BRepPrim_Cone(Axes, R1, R2, H);
    }

    /// <summary>A part cone on the axes.</summary>
    public BRepPrimAPI_MakeCone(in Frame3D Axes, double R1, double R2, double H, double angle)
    {
        myCone = new BRepPrim_Cone(Axes, R1, R2, H);
        myCone.Angle(angle);
    }

    internal override BRepPrim_OneAxis OneAxis()
    {
        return myCone;
    }

    /// <summary>The algorithm.</summary>
    internal BRepPrim_Cone Cone()
    {
        return myCone;
    }
}
