// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeRotation.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeRotation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A rotation about an axis, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class GC_MakeRotation
{
    private readonly Geom_Transformation TheRotation;

    /// <summary>The rotation of angle Angle about the line Line.</summary>
    public GC_MakeRotation(in Line3D Line, double Angle)
    {
        TheRotation = new Geom_Transformation();
        TheRotation.SetRotation(new Axis3D(Line.Location(), Line.Direction()), Angle);
    }

    /// <summary>The rotation of angle Angle about the axis Axis.</summary>
    public GC_MakeRotation(in Axis3D Axis, double Angle)
    {
        TheRotation = new Geom_Transformation();
        TheRotation.SetRotation(Axis, Angle);
    }

    /// <summary>The rotation of angle Angle about the axis through Point with direction Direc.</summary>
    public GC_MakeRotation(in Point3D Point, in Direction3D Direc, double Angle)
    {
        TheRotation = new Geom_Transformation();
        TheRotation.SetRotation(new Axis3D(Point, Direc), Angle);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Geom_Transformation Value()
    {
        TheRotation.Freeze();
        return TheRotation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Transformation(GC_MakeRotation theMaker) => theMaker.Value();
}
