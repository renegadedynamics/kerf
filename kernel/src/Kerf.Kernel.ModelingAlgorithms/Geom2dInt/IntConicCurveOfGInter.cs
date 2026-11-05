// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_IntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_IntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The intersection between a conic and a 2D curve that may be a conic or composite.</summary>
/// <remarks>OCCT's instantiation of IntCurve_UserIntConicCurveGen.gxx over Adaptor2d_Curve2d (decision 321).</remarks>
internal class IntConicCurveOfGInter
    : UserIntConicCurveGen<Curve2DAdaptorBase, Geom2dCurveTool, TheProjPCurOfGInter,
                               MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public IntConicCurveOfGInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public IntConicCurveOfGInter(in Line2D lin1, in TwoDDomain d1, Curve2DAdaptorBase c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(lin1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public IntConicCurveOfGInter(in Circle2D circ1, in TwoDDomain d1, Curve2DAdaptorBase c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(circ1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public IntConicCurveOfGInter(in Ellipse2D elips1, in TwoDDomain d1, Curve2DAdaptorBase c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(elips1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public IntConicCurveOfGInter(in Parabola2D parab1, in TwoDDomain d1, Curve2DAdaptorBase c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(parab1, d1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public IntConicCurveOfGInter(in Hyperbola2D hyper1, in TwoDDomain d1, Curve2DAdaptorBase c2, in TwoDDomain d2,
                                           double tolConf, double tol)
        : base(hyper1, d1, c2, d2, tolConf, tol)
    {
    }
}
