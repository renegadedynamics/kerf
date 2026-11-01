// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ExactIntersectionPointOfTheIntPCurvePCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ExactIntersectionPointOfTheIntPCurvePCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The exact intersection point of two 2D curves near the crossing of two segments of their polygons.</summary>
/// <remarks>OCCT's instantiation of IntCurve_ExactIntersectionPoint.gxx over HLRBRep_Curve (decision 324).</remarks>
public sealed class HLRBRep_ExactIntersectionPointOfTheIntPCurvePCurveOfCInter
    : IntCurve_ExactIntersectionPoint<HLRBRep_Curve, HLRBRep_CurveTool>
{
    public HLRBRep_ExactIntersectionPointOfTheIntPCurvePCurveOfCInter(HLRBRep_Curve C1, HLRBRep_Curve C2,
                                                                        double Tol)
        : base(C1, C2, Tol)
    {
    }
}
