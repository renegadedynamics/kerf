// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheInterferenceOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheInterferenceOfHInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The interference between the polygon of a curve and the polyhedron of a surface, as IntCurveSurface_HInter
/// computes it (OCCT's instantiation of Intf_InterferencePolygonPolyhedron.gxx).
/// </summary>
internal sealed class TheInterferenceOfHInter
    : InterferencePolygonPolyhedron<ThePolygonOfHInter, ThePolygonToolOfHInter,
                                         ThePolyhedronOfHInter,
                                         ThePolyhedronToolOfHInter>,
      ITheInterference<TheInterferenceOfHInter, ThePolygonOfHInter,
                                       ThePolyhedronOfHInter>
{
    public TheInterferenceOfHInter()
    {
    }

    public TheInterferenceOfHInter(ThePolygonOfHInter polyg,
                                                   ThePolyhedronOfHInter polyh)
        : base(polyg, polyh)
    {
    }

    public TheInterferenceOfHInter(in Line3D lin, ThePolyhedronOfHInter polyh)
        : base(lin, polyh)
    {
    }

    public TheInterferenceOfHInter(NCollection_Array1<Line3D> lins,
                                                   ThePolyhedronOfHInter polyh)
        : base(lins, polyh)
    {
    }

    public TheInterferenceOfHInter(ThePolygonOfHInter polyg,
                                                   ThePolyhedronOfHInter polyh,
                                                   BoundSortBox boundSB)
        : base(polyg, polyh, boundSB)
    {
    }

    public TheInterferenceOfHInter(in Line3D lin, ThePolyhedronOfHInter polyh,
                                                   BoundSortBox boundSB)
        : base(lin, polyh, boundSB)
    {
    }

    public TheInterferenceOfHInter(NCollection_Array1<Line3D> lins,
                                                   ThePolyhedronOfHInter polyh,
                                                   BoundSortBox boundSB)
        : base(lins, polyh, boundSB)
    {
    }

    static TheInterferenceOfHInter ITheInterference<TheInterferenceOfHInter, ThePolygonOfHInter, ThePolyhedronOfHInter>.Create(
        ThePolygonOfHInter polyg, ThePolyhedronOfHInter polyh) =>
        new(polyg, polyh);

    static TheInterferenceOfHInter ITheInterference<TheInterferenceOfHInter, ThePolygonOfHInter, ThePolyhedronOfHInter>.Create(
        ThePolygonOfHInter polyg, ThePolyhedronOfHInter polyh,
        BoundSortBox boundSB) => new(polyg, polyh, boundSB);
}
