// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_ExactIntersectionPointOfTheIntPCurvePCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_ExactIntersectionPointOfTheIntPCurvePCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The exact intersection point of two 2D curves near the crossing of two segments of their polygons.</summary>
/// <remarks>OCCT's instantiation of IntCurve_ExactIntersectionPoint.gxx over Adaptor2d_Curve2d (decision 324).</remarks>
internal sealed class ExactIntersectionPointOfTheIntPCurvePCurveOfGInter
    : ExactIntersectionPoint<Curve2DAdaptorBase, Geom2dCurveTool>
{
    public ExactIntersectionPointOfTheIntPCurvePCurveOfGInter(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2,
                                                                        double tol)
        : base(c1, c2, tol)
    {
    }
}
