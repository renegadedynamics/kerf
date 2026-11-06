// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Values;

/// <summary>A translation in 2D space, as a gp_Trsf2d.</summary>
internal sealed class MakeTranslation2d
{
    private Transform2D _theTranslation2d = new();

    /// <summary>A translation by a vector.</summary>
    public MakeTranslation2d(in Vector2D vec) => _theTranslation2d = _theTranslation2d.WithTranslation(vec);

    /// <summary>A translation from Point1 to Point2.</summary>
    public MakeTranslation2d(in Point2D point1, in Point2D point2) => _theTranslation2d = _theTranslation2d.WithTranslation(new Vector2D(point1, point2));

    /// <summary>The constructed transformation.</summary>
    public Transform2D Value() => _theTranslation2d;

    /// <summary>Alias for Value().</summary>
    public Transform2D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform2D(MakeTranslation2d maker) => maker.Operator();
}
