// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeTranslation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Curves;

/// <summary>A translation in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
internal sealed class MakeTranslation2d
{
    private readonly Transformation2 _theTranslation;

    /// <summary>The translation of vector Vec.</summary>
    public MakeTranslation2d(in Vector2D vec)
    {
        _theTranslation = new Transformation2();
        _theTranslation.SetTranslation(vec);
    }

    /// <summary>The translation from Point1 to Point2.</summary>
    public MakeTranslation2d(in Point2D point1, in Point2D point2)
    {
        _theTranslation = new Transformation2();
        _theTranslation.SetTranslation(point1, point2);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Transformation2 Value()
    {
        _theTranslation.Freeze();
        return _theTranslation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Transformation2(MakeTranslation2d maker) => maker.Value();
}
