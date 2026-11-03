// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A translation in 2D space, as a gp_Trsf2d.</summary>
public sealed class gce_MakeTranslation2d
{
    private Transform2D TheTranslation2d = new();

    /// <summary>A translation by a vector.</summary>
    public gce_MakeTranslation2d(in Vector2D Vec) => TheTranslation2d.SetTranslation(Vec);

    /// <summary>A translation from Point1 to Point2.</summary>
    public gce_MakeTranslation2d(in Point2D Point1, in Point2D Point2) => TheTranslation2d.SetTranslation(new Vector2D(Point1, Point2));

    /// <summary>The constructed transformation.</summary>
    public Transform2D Value() => TheTranslation2d;

    /// <summary>Alias for Value().</summary>
    public Transform2D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform2D(gce_MakeTranslation2d theMaker) => theMaker.Operator();
}
