// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A translation in 2D space, as a gp_Trsf2d.</summary>
public sealed class gce_MakeTranslation2d
{
    private gp_Trsf2d TheTranslation2d = new();

    /// <summary>A translation by a vector.</summary>
    public gce_MakeTranslation2d(in gp_Vec2d Vec) => TheTranslation2d.SetTranslation(Vec);

    /// <summary>A translation from Point1 to Point2.</summary>
    public gce_MakeTranslation2d(in gp_Pnt2d Point1, in gp_Pnt2d Point2) => TheTranslation2d.SetTranslation(new gp_Vec2d(Point1, Point2));

    /// <summary>The constructed transformation.</summary>
    public gp_Trsf2d Value() => TheTranslation2d;

    /// <summary>Alias for Value().</summary>
    public gp_Trsf2d Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Trsf2d(gce_MakeTranslation2d theMaker) => theMaker.Operator();
}
