// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Curves;

using Kerf.Engine.Intersection.TwoD;

using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>The intersection between two 2D curves (HLRBRep_Curve), or the self-intersection of one.</summary>
/// <remarks>
/// OCCT's instantiation of IntCurve_IntCurveCurveGen.gxx over HLRBRep_Curve, the projected edges (OCCT's
/// HLRBRep_CurvePtr, a void pointer to the curve; decisions 321, 1091). The curves are kept by reference during the
/// computation only; the results are values. Generated from Geom2dInt_GInter by substituting the curve and its tool.
/// </remarks>
public sealed class CInter
    : IntCurveCurveGen<HiddenLineCurve, HiddenLineCurveTool, TheProjPCurOfCInter,
                                MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public CInter()
    {
    }

    /// <summary>Self-intersection of a curve.</summary>
    public CInter(HiddenLineCurve c, double tolConf, double tol)
        : base(c, tolConf, tol)
    {
    }

    /// <summary>Self-intersection of a curve on a domain.</summary>
    public CInter(HiddenLineCurve c, in TwoDDomain d, double tolConf, double tol)
        : base(c, d, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves.</summary>
    public CInter(HiddenLineCurve c1, HiddenLineCurve c2, double tolConf, double tol)
        : base(c1, c2, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public CInter(HiddenLineCurve c1, in TwoDDomain d1, HiddenLineCurve c2, double tolConf,
                            double tol)
        : base(c1, d1, c2, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public CInter(HiddenLineCurve c1, HiddenLineCurve c2, in TwoDDomain d2, double tolConf,
                            double tol)
        : base(c1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves on their domains.</summary>
    public CInter(HiddenLineCurve c1, in TwoDDomain d1, HiddenLineCurve c2,
                            in TwoDDomain d2, double tolConf, double tol)
        : base(c1, d1, c2, d2, tolConf, tol)
    {
    }
}
