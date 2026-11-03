// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeScale.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeScale.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A scaling in 3D space, as a gp_Trsf.</summary>
public sealed class gce_MakeScale
{
    private Transform3D TheScale = new();

    /// <summary>A scaling by Scale about a point.</summary>
    public gce_MakeScale(in Point3D Point, double Scale) => TheScale.SetScale(Point, Scale);

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => TheScale;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(gce_MakeScale theMaker) => theMaker.Operator();
}
