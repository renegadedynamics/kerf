// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheInterferenceOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheInterferenceOfHInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The interference between the polygon of a curve and the polyhedron of a surface, as IntCurveSurface_HInter
/// computes it (OCCT's instantiation of Intf_InterferencePolygonPolyhedron.gxx).
/// </summary>
public sealed class IntCurveSurface_TheInterferenceOfHInter
    : Intf_InterferencePolygonPolyhedron<IntCurveSurface_ThePolygonOfHInter, IntCurveSurface_ThePolygonToolOfHInter,
                                         IntCurveSurface_ThePolyhedronOfHInter,
                                         IntCurveSurface_ThePolyhedronToolOfHInter>,
      IIntCurveSurface_TheInterference<IntCurveSurface_TheInterferenceOfHInter, IntCurveSurface_ThePolygonOfHInter,
                                       IntCurveSurface_ThePolyhedronOfHInter>
{
    public IntCurveSurface_TheInterferenceOfHInter()
    {
    }

    public IntCurveSurface_TheInterferenceOfHInter(IntCurveSurface_ThePolygonOfHInter thePolyg,
                                                   IntCurveSurface_ThePolyhedronOfHInter thePolyh)
        : base(thePolyg, thePolyh)
    {
    }

    public IntCurveSurface_TheInterferenceOfHInter(in Line3D theLin, IntCurveSurface_ThePolyhedronOfHInter thePolyh)
        : base(theLin, thePolyh)
    {
    }

    public IntCurveSurface_TheInterferenceOfHInter(NCollection_Array1<Line3D> theLins,
                                                   IntCurveSurface_ThePolyhedronOfHInter thePolyh)
        : base(theLins, thePolyh)
    {
    }

    public IntCurveSurface_TheInterferenceOfHInter(IntCurveSurface_ThePolygonOfHInter thePolyg,
                                                   IntCurveSurface_ThePolyhedronOfHInter thePolyh,
                                                   BoundSortBox theBoundSB)
        : base(thePolyg, thePolyh, theBoundSB)
    {
    }

    public IntCurveSurface_TheInterferenceOfHInter(in Line3D theLin, IntCurveSurface_ThePolyhedronOfHInter thePolyh,
                                                   BoundSortBox theBoundSB)
        : base(theLin, thePolyh, theBoundSB)
    {
    }

    public IntCurveSurface_TheInterferenceOfHInter(NCollection_Array1<Line3D> theLins,
                                                   IntCurveSurface_ThePolyhedronOfHInter thePolyh,
                                                   BoundSortBox theBoundSB)
        : base(theLins, thePolyh, theBoundSB)
    {
    }

    static IntCurveSurface_TheInterferenceOfHInter IIntCurveSurface_TheInterference<IntCurveSurface_TheInterferenceOfHInter, IntCurveSurface_ThePolygonOfHInter, IntCurveSurface_ThePolyhedronOfHInter>.Create(
        IntCurveSurface_ThePolygonOfHInter thePolyg, IntCurveSurface_ThePolyhedronOfHInter thePolyh) =>
        new(thePolyg, thePolyh);

    static IntCurveSurface_TheInterferenceOfHInter IIntCurveSurface_TheInterference<IntCurveSurface_TheInterferenceOfHInter, IntCurveSurface_ThePolygonOfHInter, IntCurveSurface_ThePolyhedronOfHInter>.Create(
        IntCurveSurface_ThePolygonOfHInter thePolyg, IntCurveSurface_ThePolyhedronOfHInter thePolyh,
        BoundSortBox theBoundSB) => new(thePolyg, thePolyh, theBoundSB);
}
