// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeTranslation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A translation in 3D space, as a gp_Trsf.</summary>
public sealed class gce_MakeTranslation
{
    private Transform3D TheTranslation = new();

    /// <summary>A translation by a vector.</summary>
    public gce_MakeTranslation(in Vector3D Vec) => TheTranslation.SetTranslation(Vec);

    /// <summary>A translation from Point1 to Point2.</summary>
    public gce_MakeTranslation(in Point3D Point1, in Point3D Point2) => TheTranslation.SetTranslation(new Vector3D(Point1, Point2));

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => TheTranslation;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(gce_MakeTranslation theMaker) => theMaker.Operator();
}
