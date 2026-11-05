// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_ThePolygon2dOfTheIntPCurvePCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_ThePolygon2dOfTheIntPCurvePCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>A polygon sampling a 2D curve on a domain, for the 2D curve intersections.</summary>
/// <remarks>OCCT's instantiation of IntCurve_Polygon2dGen.gxx over Adaptor2d_Curve2d (decision 324).</remarks>
internal sealed class ThePolygon2dOfTheIntPCurvePCurveOfGInter
    : Polygon2dGen<Curve2DAdaptorBase, Geom2dCurveTool>
{
    /// <summary>Samples NbPnt points (at least 3) of the curve on the bounded domain.</summary>
    public ThePolygon2dOfTheIntPCurvePCurveOfGInter(Curve2DAdaptorBase curve, int nbPnt,
                                                              in TwoDDomain domain, double tol)
        : base(curve, nbPnt, domain, tol)
    {
    }
}
