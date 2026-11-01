// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygon2dOfTheIntPCurvePCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygon2dOfTheIntPCurvePCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A polygon sampling a 2D curve on a domain, for the 2D curve intersections.</summary>
/// <remarks>OCCT's instantiation of IntCurve_Polygon2dGen.gxx over HLRBRep_Curve (decision 324).</remarks>
public sealed class HLRBRep_ThePolygon2dOfTheIntPCurvePCurveOfCInter
    : IntCurve_Polygon2dGen<HLRBRep_Curve, HLRBRep_CurveTool>
{
    /// <summary>Samples NbPnt points (at least 3) of the curve on the bounded domain.</summary>
    public HLRBRep_ThePolygon2dOfTheIntPCurvePCurveOfCInter(HLRBRep_Curve Curve, int NbPnt,
                                                              in IntRes2d_Domain Domain, double Tol)
        : base(Curve, NbPnt, Domain, Tol)
    {
    }
}
