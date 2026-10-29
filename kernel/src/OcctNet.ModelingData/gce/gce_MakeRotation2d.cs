// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeRotation2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A rotation in 2D space, as a gp_Trsf2d.</summary>
public sealed class gce_MakeRotation2d
{
    private gp_Trsf2d TheRotation2d = new();

    /// <summary>A rotation by Angle (radians) around a point.</summary>
    public gce_MakeRotation2d(in gp_Pnt2d Point, double Angle) => TheRotation2d.SetRotation(Point, Angle);

    /// <summary>The constructed transformation.</summary>
    public gp_Trsf2d Value() => TheRotation2d;

    /// <summary>Alias for Value().</summary>
    public gp_Trsf2d Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Trsf2d(gce_MakeRotation2d theMaker) => theMaker.Operator();
}
