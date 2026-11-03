// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_GInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_GInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The intersection between two 2D curves (Adaptor2d_Curve2d), or the self-intersection of one.</summary>
/// <remarks>
/// OCCT's instantiation of IntCurve_IntCurveCurveGen.gxx over Adaptor2d_Curve2d (decision 321). The curves are
/// kept by reference during the computation only; the results are values. Not sealed: Geom2dHatch_Intersector derives
/// from it, as in OCCT (decision 665).
/// </remarks>
public class GInter
    : IntCurveCurveGen<Curve2DAdaptorBase, Geom2dCurveTool, TheProjPCurOfGInter,
                                MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public GInter()
    {
    }

    /// <summary>Self-intersection of a curve.</summary>
    public GInter(Curve2DAdaptorBase c, double tolConf, double tol)
        : base(c, tolConf, tol)
    {
    }

    /// <summary>Self-intersection of a curve on a domain.</summary>
    public GInter(Curve2DAdaptorBase c, in TwoDDomain d, double tolConf, double tol)
        : base(c, d, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves.</summary>
    public GInter(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, double tolConf, double tol)
        : base(c1, c2, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public GInter(Curve2DAdaptorBase c1, in TwoDDomain d1, Curve2DAdaptorBase c2, double tolConf,
                            double tol)
        : base(c1, d1, c2, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public GInter(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, in TwoDDomain d2, double tolConf,
                            double tol)
        : base(c1, c2, d2, tolConf, tol)
    {
    }

    /// <summary>Intersection between two curves on their domains.</summary>
    public GInter(Curve2DAdaptorBase c1, in TwoDDomain d1, Curve2DAdaptorBase c2,
                            in TwoDDomain d2, double tolConf, double tol)
        : base(c1, d1, c2, d2, tolConf, tol)
    {
    }
}
