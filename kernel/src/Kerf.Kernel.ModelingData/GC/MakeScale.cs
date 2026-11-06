// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeScale.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeScale.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Curves;

/// <summary>A scaling about a point, as a Geom_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
internal sealed class MakeScale
{
    private readonly GeometryTransformation _theScale;

    /// <summary>The scaling of factor Scale about Point.</summary>
    public MakeScale(in Point3D point, double scale)
    {
        _theScale = new GeometryTransformation();
        _theScale.SetScale(point, scale);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public GeometryTransformation Value()
    {
        _theScale.Freeze();
        return _theScale;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator GeometryTransformation(MakeScale maker) => maker.Value();
}
