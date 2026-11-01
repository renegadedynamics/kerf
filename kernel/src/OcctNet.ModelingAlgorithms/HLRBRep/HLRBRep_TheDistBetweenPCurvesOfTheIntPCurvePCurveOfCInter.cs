// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The vector C1(u) - C2(v) between the points of two 2D curves, with its Jacobian.</summary>
/// <remarks>OCCT's instantiation of IntCurve_DistBetweenPCurvesGen.gxx over HLRBRep_Curve (decision 324).</remarks>
public sealed class HLRBRep_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter
    : IntCurve_DistBetweenPCurvesGen<HLRBRep_Curve, HLRBRep_CurveTool>
{
    public HLRBRep_TheDistBetweenPCurvesOfTheIntPCurvePCurveOfCInter(HLRBRep_Curve curve1,
                                                                       HLRBRep_Curve curve2)
        : base(curve1, curve2)
    {
    }
}
