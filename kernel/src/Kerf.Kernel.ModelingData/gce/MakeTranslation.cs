// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Values;

/// <summary>A translation in 3D space, as a gp_Trsf.</summary>
internal sealed class MakeTranslation
{
    private Transform3D _theTranslation = new();

    /// <summary>A translation by a vector.</summary>
    public MakeTranslation(in Vector3D vec) => _theTranslation = _theTranslation.WithTranslation(vec);

    /// <summary>A translation from Point1 to Point2.</summary>
    public MakeTranslation(in Point3D point1, in Point3D point2) => _theTranslation = _theTranslation.WithTranslation(new Vector3D(point1, point2));

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => _theTranslation;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(MakeTranslation maker) => maker.Operator();
}
