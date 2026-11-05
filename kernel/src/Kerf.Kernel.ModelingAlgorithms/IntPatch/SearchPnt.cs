// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_SearchPnt.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Polygons;

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>The interference of 2D polygons, under the name IntPatch_RstInt uses (OCCT's typedef of Intf_InterferencePolygon2d).</summary>
/// <remarks>A C++ typedef of a class: a sealed subclass with the constructors forwarded, so that the name is kept.</remarks>
internal sealed class SearchPnt : InterferencePolygon2d
{
    public SearchPnt()
    {
    }

    /// <summary>The intersections of two polygons.</summary>
    public SearchPnt(Polygon2d obje1, Polygon2d obje2)
        : base(obje1, obje2)
    {
    }

    /// <summary>The self-intersections of a polygon.</summary>
    public SearchPnt(Polygon2d obje)
        : base(obje)
    {
    }
}
