// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeScale2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeScale2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A scaling about a point in 2D, as a Geom2d_Transformation.</summary>
/// <remarks>Not a GC_Root: the construction can't fail, as in OCCT.</remarks>
public sealed class GC_MakeScale2d
{
    private readonly Geom2d_Transformation TheScale;

    /// <summary>The scaling of factor Scale about Point.</summary>
    public GC_MakeScale2d(in gp_Pnt2d Point, double Scale)
    {
        TheScale = new Geom2d_Transformation();
        TheScale.SetScale(Point, Scale);
    }

    /// <summary>The constructed transformation (frozen).</summary>
    public Geom2d_Transformation Value()
    {
        TheScale.Freeze();
        return TheScale;
    }

    /// <summary>The constructed transformation (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Transformation(GC_MakeScale2d theMaker) => theMaker.Value();
}
