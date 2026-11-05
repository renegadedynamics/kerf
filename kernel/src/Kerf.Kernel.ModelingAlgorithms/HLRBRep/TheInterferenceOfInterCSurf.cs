// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheInterferenceOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheInterferenceOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The interference between the polygon of a curve and the polyhedron of a surface, as HLRBRep_InterCSurf
/// computes it (OCCT's instantiation of Intf_InterferencePolygonPolyhedron.gxx).
/// </summary>
internal sealed class TheInterferenceOfInterCSurf
    : InterferencePolygonPolyhedron<ThePolygonOfInterCSurf, ThePolygonToolOfInterCSurf,
                                         ThePolyhedronOfInterCSurf,
                                         ThePolyhedronToolOfInterCSurf>,
      ITheInterference<TheInterferenceOfInterCSurf, ThePolygonOfInterCSurf,
                                       ThePolyhedronOfInterCSurf>
{
    public TheInterferenceOfInterCSurf()
    {
    }

    public TheInterferenceOfInterCSurf(ThePolygonOfInterCSurf polyg,
                                                   ThePolyhedronOfInterCSurf polyh)
        : base(polyg, polyh)
    {
    }

    public TheInterferenceOfInterCSurf(in Line3D lin, ThePolyhedronOfInterCSurf polyh)
        : base(lin, polyh)
    {
    }

    public TheInterferenceOfInterCSurf(NCollection_Array1<Line3D> lins,
                                                   ThePolyhedronOfInterCSurf polyh)
        : base(lins, polyh)
    {
    }

    public TheInterferenceOfInterCSurf(ThePolygonOfInterCSurf polyg,
                                                   ThePolyhedronOfInterCSurf polyh,
                                                   BoundSortBox boundSB)
        : base(polyg, polyh, boundSB)
    {
    }

    public TheInterferenceOfInterCSurf(in Line3D lin, ThePolyhedronOfInterCSurf polyh,
                                                   BoundSortBox boundSB)
        : base(lin, polyh, boundSB)
    {
    }

    public TheInterferenceOfInterCSurf(NCollection_Array1<Line3D> lins,
                                                   ThePolyhedronOfInterCSurf polyh,
                                                   BoundSortBox boundSB)
        : base(lins, polyh, boundSB)
    {
    }

    static TheInterferenceOfInterCSurf ITheInterference<TheInterferenceOfInterCSurf, ThePolygonOfInterCSurf, ThePolyhedronOfInterCSurf>.Create(
        ThePolygonOfInterCSurf polyg, ThePolyhedronOfInterCSurf polyh) =>
        new(polyg, polyh);

    static TheInterferenceOfInterCSurf ITheInterference<TheInterferenceOfInterCSurf, ThePolygonOfInterCSurf, ThePolyhedronOfInterCSurf>.Create(
        ThePolygonOfInterCSurf polyg, ThePolyhedronOfInterCSurf polyh,
        BoundSortBox boundSB) => new(polyg, polyh, boundSB);
}
