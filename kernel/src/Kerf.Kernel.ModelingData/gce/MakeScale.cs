// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeScale.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeScale.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Values;

/// <summary>A scaling in 3D space, as a gp_Trsf.</summary>
internal sealed class MakeScale
{
    private Transform3D _theScale = new();

    /// <summary>A scaling by Scale about a point.</summary>
    public MakeScale(in Point3D point, double scale) => _theScale = _theScale.WithScale(point, scale);

    /// <summary>The constructed transformation.</summary>
    public Transform3D Value() => _theScale;

    /// <summary>Alias for Value().</summary>
    public Transform3D Operator() => Value();

    /// <summary>The constructed transformation (OCCT's implicit conversion operator).</summary>
    public static explicit operator Transform3D(MakeScale maker) => maker.Operator();
}
