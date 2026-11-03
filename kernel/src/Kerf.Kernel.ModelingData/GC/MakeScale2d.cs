// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeScale2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeScale2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeScale.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Curves;

/// <summary>A scaling about a point in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class MakeScale2d
{
    private readonly Transformation2 _theScale;

    /// <summary>The scaling of factor Scale about Point.</summary>
    public MakeScale2d(in Point2D point, double scale)
    {
        _theScale = new Transformation2();
        _theScale.SetScale(point, scale);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Transformation2 Value()
    {
        _theScale.Freeze();
        return _theScale;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Transformation2(MakeScale2d maker) => maker.Value();
}
