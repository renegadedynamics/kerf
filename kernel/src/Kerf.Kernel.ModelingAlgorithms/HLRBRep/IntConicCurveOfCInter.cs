// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_IntConicCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_IntConicCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>The intersection between a conic and a 2D curve that may be a conic or composite.</summary>
/// <remarks>OCCT's instantiation of IntCurve_UserIntConicCurveGen.gxx over HLRBRep_Curve (decision 321).</remarks>
public class IntConicCurveOfCInter
    : UserIntConicCurveGen<HiddenLineCurve, HiddenLineCurveTool, TheProjPCurOfCInter,
                               MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public IntConicCurveOfCInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public IntConicCurveOfCInter(in Line2D lin1, in TwoDDomain d1, HiddenLineCurve c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(lin1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public IntConicCurveOfCInter(in Circle2D circ1, in TwoDDomain d1, HiddenLineCurve c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(circ1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public IntConicCurveOfCInter(in Ellipse2D elips1, in TwoDDomain d1, HiddenLineCurve c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(elips1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public IntConicCurveOfCInter(in Parabola2D parab1, in TwoDDomain d1, HiddenLineCurve c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(parab1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public IntConicCurveOfCInter(in Hyperbola2D hyper1, in TwoDDomain d1, HiddenLineCurve c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(hyper1, d1, c2, d2, tolConf, tol)
    {
    }
}
