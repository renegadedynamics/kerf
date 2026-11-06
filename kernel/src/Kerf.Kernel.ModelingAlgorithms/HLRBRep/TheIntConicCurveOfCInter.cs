// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntConicCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntConicCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.HiddenLines;

/// <summary>The intersection between a conic (seen as an implicit curve) and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntCurve_IntConicCurveGen.gxx over HLRBRep_Curve (decision 321).</remarks>
internal sealed class TheIntConicCurveOfCInter
    : IntConicCurveGen<HiddenLineCurve, HiddenLineCurveTool, TheProjPCurOfCInter,
                               MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public TheIntConicCurveOfCInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public TheIntConicCurveOfCInter(in Line2D l, in TwoDDomain d1, HiddenLineCurve pCurve, in TwoDDomain d2,
                                              double tolConf, double tol)
        : base(l, d1, pCurve, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public TheIntConicCurveOfCInter(in Circle2D c, in TwoDDomain d1, HiddenLineCurve pCurve, in TwoDDomain d2,
                                              double tolConf, double tol)
        : base(c, d1, pCurve, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public TheIntConicCurveOfCInter(in Ellipse2D e, in TwoDDomain d1, HiddenLineCurve pCurve, in TwoDDomain d2,
                                              double tolConf, double tol)
        : base(e, d1, pCurve, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public TheIntConicCurveOfCInter(in Parabola2D prb, in TwoDDomain d1, HiddenLineCurve pCurve, in TwoDDomain d2,
                                              double tolConf, double tol)
        : base(prb, d1, pCurve, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public TheIntConicCurveOfCInter(in Hyperbola2D h, in TwoDDomain d1, HiddenLineCurve pCurve, in TwoDDomain d2,
                                              double tolConf, double tol)
        : base(h, d1, pCurve, d2, tolConf, tol)
    {
    }
}
