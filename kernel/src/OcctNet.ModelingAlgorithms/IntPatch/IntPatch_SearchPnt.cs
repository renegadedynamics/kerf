// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SearchPnt.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The interference of 2D polygons, under the name IntPatch_RstInt uses (OCCT's typedef of Intf_InterferencePolygon2d).</summary>
/// <remarks>A C++ typedef of a class: a sealed subclass with the constructors forwarded, so that the name is kept.</remarks>
public sealed class IntPatch_SearchPnt : Intf_InterferencePolygon2d
{
    public IntPatch_SearchPnt()
    {
    }

    /// <summary>The intersections of two polygons.</summary>
    public IntPatch_SearchPnt(Intf_Polygon2d Obje1, Intf_Polygon2d Obje2)
        : base(Obje1, Obje2)
    {
    }

    /// <summary>The self-intersections of a polygon.</summary>
    public IntPatch_SearchPnt(Intf_Polygon2d Obje)
        : base(Obje)
    {
    }
}
