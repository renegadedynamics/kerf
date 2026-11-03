// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Values;

/// <summary>A rotation in 3D space, as a gp_Trsf.</summary>
public sealed class MakeRotation
{
    private Transform3D _theRotation = new();

    /// <summary>A rotation by Angle (radians) around the axis of a line.</summary>
    public MakeRotation(in Line3D line, double angle) => _theRotation = _theRotation.WithRotation(line.Position(), angle);

    /// <summary>A rotation by Angle (radians) around an axis.</summary>
    public MakeRotation(in Axis3D axis, double angle) => _theRotation = _theRotation.WithRotation(axis, angle);

    /// <summary>A rotation by Angle (radians) around the axis through a point with a direction.</summary>
    public MakeRotation(in Point3D point, in Direction3D direc, double angle) => _theRotation = _theRotation.WithRotation(new Axis3D(point, direc), angle);

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => _theRotation;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(MakeRotation maker) => maker.Operator();
}
