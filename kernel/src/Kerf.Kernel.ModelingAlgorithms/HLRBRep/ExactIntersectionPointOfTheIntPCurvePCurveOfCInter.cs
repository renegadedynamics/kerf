// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ExactIntersectionPointOfTheIntPCurvePCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ExactIntersectionPointOfTheIntPCurvePCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Curves;

using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>The exact intersection point of two 2D curves near the crossing of two segments of their polygons.</summary>
/// <remarks>OCCT's instantiation of IntCurve_ExactIntersectionPoint.gxx over HLRBRep_Curve (decision 324).</remarks>
internal sealed class ExactIntersectionPointOfTheIntPCurvePCurveOfCInter
    : ExactIntersectionPoint<HiddenLineCurve, HiddenLineCurveTool>
{
    public ExactIntersectionPointOfTheIntPCurvePCurveOfCInter(HiddenLineCurve c1, HiddenLineCurve c2,
                                                                        double tol)
        : base(c1, c2, tol)
    {
    }
}
