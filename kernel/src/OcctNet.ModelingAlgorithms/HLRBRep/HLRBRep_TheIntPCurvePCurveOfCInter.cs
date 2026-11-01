// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntPCurvePCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntPCurvePCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The intersection of two bounded 2D curves, or the self-intersection of one, through their polygons.
/// </summary>
/// <remarks>OCCT's instantiation of IntCurve_IntPolyPolyGen.gxx over HLRBRep_Curve (decision 324).</remarks>
public sealed class HLRBRep_TheIntPCurvePCurveOfCInter
    : IntCurve_IntPolyPolyGen<HLRBRep_Curve, HLRBRep_CurveTool, HLRBRep_TheProjPCurOfCInter>
{
    public HLRBRep_TheIntPCurvePCurveOfCInter()
    {
    }
}
