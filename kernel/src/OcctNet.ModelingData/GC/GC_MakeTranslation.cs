// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A translation, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class GC_MakeTranslation
{
    private readonly Geom_Transformation TheTranslation;

    /// <summary>The translation of vector Vec.</summary>
    public GC_MakeTranslation(in gp_Vec Vec)
    {
        TheTranslation = new Geom_Transformation();
        TheTranslation.SetTranslation(Vec);
    }

    /// <summary>The translation from Point1 to Point2.</summary>
    public GC_MakeTranslation(in gp_Pnt Point1, in gp_Pnt Point2)
    {
        TheTranslation = new Geom_Transformation();
        TheTranslation.SetTranslation(Point1, Point2);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Geom_Transformation Value()
    {
        TheTranslation.Freeze();
        return TheTranslation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Transformation(GC_MakeTranslation theMaker) => theMaker.Value();
}
