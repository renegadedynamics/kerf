// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Curves;

using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>The vector C1(u) - C2(v) between the points of two 2D curves, with its Jacobian.</summary>
/// <remarks>OCCT's instantiation of IntCurve_DistBetweenPCurvesGen.gxx over HLRBRep_Curve (decision 324).</remarks>
internal sealed class TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter
    : DistBetweenPCurvesGen<HiddenLineCurve, HiddenLineCurveTool>
{
    public TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter(HiddenLineCurve curve1,
                                                                       HiddenLineCurve curve2)
        : base(curve1, curve2)
    {
    }
}
