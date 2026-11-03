// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_TheIntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The intersection between a conic (seen as an implicit curve) and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntCurve_IntConicCurveGen.gxx over Adaptor2d_Curve2d (decision 321).</remarks>
public sealed class Geom2dInt_TheIntConicCurveOfGInter
    : IntCurve_IntConicCurveGen<Adaptor2d_Curve2d, Geom2dInt_Geom2dCurveTool, Geom2dInt_TheProjPCurOfGInter,
                               Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public Geom2dInt_TheIntConicCurveOfGInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public Geom2dInt_TheIntConicCurveOfGInter(in Line2D L, in IntRes2d_Domain D1, Adaptor2d_Curve2d PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(L, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public Geom2dInt_TheIntConicCurveOfGInter(in Circle2D C, in IntRes2d_Domain D1, Adaptor2d_Curve2d PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(C, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public Geom2dInt_TheIntConicCurveOfGInter(in Ellipse2D E, in IntRes2d_Domain D1, Adaptor2d_Curve2d PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(E, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public Geom2dInt_TheIntConicCurveOfGInter(in Parabola2D Prb, in IntRes2d_Domain D1, Adaptor2d_Curve2d PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(Prb, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public Geom2dInt_TheIntConicCurveOfGInter(in Hyperbola2D H, in IntRes2d_Domain D1, Adaptor2d_Curve2d PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(H, D1, PCurve, D2, TolConf, Tol)
    {
    }
}
