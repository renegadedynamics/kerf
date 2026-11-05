// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Values;

/// <summary>A rotation in 2D space, as a gp_Trsf2d.</summary>
internal sealed class MakeRotation2d
{
    private Transform2D _theRotation2d = new();

    /// <summary>A rotation by Angle (radians) around a point.</summary>
    public MakeRotation2d(in Point2D point, double angle) => _theRotation2d = _theRotation2d.WithRotation(point, angle);

    /// <summary>The constructed transformation.</summary>
    public Transform2D Value() => _theRotation2d;

    /// <summary>Alias for Value().</summary>
    public Transform2D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform2D(MakeRotation2d maker) => maker.Operator();
}
