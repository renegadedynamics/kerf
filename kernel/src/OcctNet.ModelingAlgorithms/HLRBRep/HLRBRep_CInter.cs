// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The intersection between two 2D curves (HLRBRep_Curve), or the self-intersection of one.</summary>
/// <remarks>
/// OCCT's instantiation of IntCurve_IntCurveCurveGen.gxx over HLRBRep_Curve, the projected edges (OCCT's
/// HLRBRep_CurvePtr, a void pointer to the curve; decisions 321, 10601). The curves are kept by reference during the
/// computation only; the results are values. Generated from Geom2dInt_GInter by substituting the curve and its tool.
/// </remarks>
public sealed class HLRBRep_CInter
    : IntCurve_IntCurveCurveGen<HLRBRep_Curve, HLRBRep_CurveTool, HLRBRep_TheProjPCurOfCInter,
                                HLRBRep_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public HLRBRep_CInter()
    {
    }

    /// <summary>Self-intersection of a curve.</summary>
    public HLRBRep_CInter(HLRBRep_Curve C, double TolConf, double Tol)
        : base(C, TolConf, Tol)
    {
    }

    /// <summary>Self-intersection of a curve on a domain.</summary>
    public HLRBRep_CInter(HLRBRep_Curve C, in IntRes2d_Domain D, double TolConf, double Tol)
        : base(C, D, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves.</summary>
    public HLRBRep_CInter(HLRBRep_Curve C1, HLRBRep_Curve C2, double TolConf, double Tol)
        : base(C1, C2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public HLRBRep_CInter(HLRBRep_Curve C1, in IntRes2d_Domain D1, HLRBRep_Curve C2, double TolConf,
                            double Tol)
        : base(C1, D1, C2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public HLRBRep_CInter(HLRBRep_Curve C1, HLRBRep_Curve C2, in IntRes2d_Domain D2, double TolConf,
                            double Tol)
        : base(C1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves on their domains.</summary>
    public HLRBRep_CInter(HLRBRep_Curve C1, in IntRes2d_Domain D1, HLRBRep_Curve C2,
                            in IntRes2d_Domain D2, double TolConf, double Tol)
        : base(C1, D1, C2, D2, TolConf, Tol)
    {
    }
}
