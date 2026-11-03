// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheInterferenceOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheInterferenceOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The interference between the polygon of a curve and the polyhedron of a surface, as HLRBRep_InterCSurf
/// computes it (OCCT's instantiation of Intf_InterferencePolygonPolyhedron.gxx).
/// </summary>
public sealed class HLRBRep_TheInterferenceOfInterCSurf
    : Intf_InterferencePolygonPolyhedron<HLRBRep_ThePolygonOfInterCSurf, HLRBRep_ThePolygonToolOfInterCSurf,
                                         HLRBRep_ThePolyhedronOfInterCSurf,
                                         HLRBRep_ThePolyhedronToolOfInterCSurf>,
      IIntCurveSurface_TheInterference<HLRBRep_TheInterferenceOfInterCSurf, HLRBRep_ThePolygonOfInterCSurf,
                                       HLRBRep_ThePolyhedronOfInterCSurf>
{
    public HLRBRep_TheInterferenceOfInterCSurf()
    {
    }

    public HLRBRep_TheInterferenceOfInterCSurf(HLRBRep_ThePolygonOfInterCSurf thePolyg,
                                                   HLRBRep_ThePolyhedronOfInterCSurf thePolyh)
        : base(thePolyg, thePolyh)
    {
    }

    public HLRBRep_TheInterferenceOfInterCSurf(in Line3D theLin, HLRBRep_ThePolyhedronOfInterCSurf thePolyh)
        : base(theLin, thePolyh)
    {
    }

    public HLRBRep_TheInterferenceOfInterCSurf(NCollection_Array1<Line3D> theLins,
                                                   HLRBRep_ThePolyhedronOfInterCSurf thePolyh)
        : base(theLins, thePolyh)
    {
    }

    public HLRBRep_TheInterferenceOfInterCSurf(HLRBRep_ThePolygonOfInterCSurf thePolyg,
                                                   HLRBRep_ThePolyhedronOfInterCSurf thePolyh,
                                                   BoundSortBox theBoundSB)
        : base(thePolyg, thePolyh, theBoundSB)
    {
    }

    public HLRBRep_TheInterferenceOfInterCSurf(in Line3D theLin, HLRBRep_ThePolyhedronOfInterCSurf thePolyh,
                                                   BoundSortBox theBoundSB)
        : base(theLin, thePolyh, theBoundSB)
    {
    }

    public HLRBRep_TheInterferenceOfInterCSurf(NCollection_Array1<Line3D> theLins,
                                                   HLRBRep_ThePolyhedronOfInterCSurf thePolyh,
                                                   BoundSortBox theBoundSB)
        : base(theLins, thePolyh, theBoundSB)
    {
    }

    static HLRBRep_TheInterferenceOfInterCSurf IIntCurveSurface_TheInterference<HLRBRep_TheInterferenceOfInterCSurf, HLRBRep_ThePolygonOfInterCSurf, HLRBRep_ThePolyhedronOfInterCSurf>.Create(
        HLRBRep_ThePolygonOfInterCSurf thePolyg, HLRBRep_ThePolyhedronOfInterCSurf thePolyh) =>
        new(thePolyg, thePolyh);

    static HLRBRep_TheInterferenceOfInterCSurf IIntCurveSurface_TheInterference<HLRBRep_TheInterferenceOfInterCSurf, HLRBRep_ThePolygonOfInterCSurf, HLRBRep_ThePolyhedronOfInterCSurf>.Create(
        HLRBRep_ThePolygonOfInterCSurf thePolyg, HLRBRep_ThePolyhedronOfInterCSurf thePolyh,
        BoundSortBox theBoundSB) => new(thePolyg, thePolyh, theBoundSB);
}
