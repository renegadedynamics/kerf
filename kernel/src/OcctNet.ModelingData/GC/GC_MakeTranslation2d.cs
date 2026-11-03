// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeTranslation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A translation in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class GC_MakeTranslation2d
{
    private readonly Geom2d_Transformation TheTranslation;

    /// <summary>The translation of vector Vec.</summary>
    public GC_MakeTranslation2d(in Vector2D Vec)
    {
        TheTranslation = new Geom2d_Transformation();
        TheTranslation.SetTranslation(Vec);
    }

    /// <summary>The translation from Point1 to Point2.</summary>
    public GC_MakeTranslation2d(in Point2D Point1, in Point2D Point2)
    {
        TheTranslation = new Geom2d_Transformation();
        TheTranslation.SetTranslation(Point1, Point2);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Geom2d_Transformation Value()
    {
        TheTranslation.Freeze();
        return TheTranslation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Transformation(GC_MakeTranslation2d theMaker) => theMaker.Value();
}
