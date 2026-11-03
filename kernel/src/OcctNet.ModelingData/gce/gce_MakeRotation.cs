// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A rotation in 3D space, as a gp_Trsf.</summary>
public sealed class gce_MakeRotation
{
    private Transform3D TheRotation = new();

    /// <summary>A rotation by Angle (radians) around the axis of a line.</summary>
    public gce_MakeRotation(in Line3D Line, double Angle) => TheRotation.SetRotation(Line.Position(), Angle);

    /// <summary>A rotation by Angle (radians) around an axis.</summary>
    public gce_MakeRotation(in Axis3D Axis, double Angle) => TheRotation.SetRotation(Axis, Angle);

    /// <summary>A rotation by Angle (radians) around the axis through a point with a direction.</summary>
    public gce_MakeRotation(in Point3D Point, in Direction3D Direc, double Angle) => TheRotation.SetRotation(new Axis3D(Point, Direc), Angle);

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => TheRotation;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(gce_MakeRotation theMaker) => theMaker.Operator();
}
