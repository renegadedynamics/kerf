// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeTranslation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Curves;

/// <summary>A translation, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
internal sealed class MakeTranslation
{
    private readonly GeometryTransformation _theTranslation;

    /// <summary>The translation of vector Vec.</summary>
    public MakeTranslation(in Vector3D vec)
    {
        _theTranslation = new GeometryTransformation();
        _theTranslation.SetTranslation(vec);
    }

    /// <summary>The translation from Point1 to Point2.</summary>
    public MakeTranslation(in Point3D point1, in Point3D point2)
    {
        _theTranslation = new GeometryTransformation();
        _theTranslation.SetTranslation(point1, point2);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public GeometryTransformation Value()
    {
        _theTranslation.Freeze();
        return _theTranslation;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator GeometryTransformation(MakeTranslation maker) => maker.Value();
}
