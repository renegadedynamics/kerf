// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_IntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_IntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>The intersection between a conic and a 2D curve that may be a conic or composite.</summary>
/// <remarks>OCCT's instantiation of IntCurve_UserIntConicCurveGen.gxx over Adaptor2d_Curve2d (decision 321).</remarks>
public class Geom2dInt_IntConicCurveOfGInter
    : IntCurve_UserIntConicCurveGen<Curve2DAdaptorBase, Geom2dInt_Geom2dCurveTool, Geom2dInt_TheProjPCurOfGInter,
                               Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public Geom2dInt_IntConicCurveOfGInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public Geom2dInt_IntConicCurveOfGInter(in Line2D Lin1, in IntRes2d_Domain D1, Curve2DAdaptorBase C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Lin1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public Geom2dInt_IntConicCurveOfGInter(in Circle2D Circ1, in IntRes2d_Domain D1, Curve2DAdaptorBase C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Circ1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public Geom2dInt_IntConicCurveOfGInter(in Ellipse2D Elips1, in IntRes2d_Domain D1, Curve2DAdaptorBase C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Elips1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public Geom2dInt_IntConicCurveOfGInter(in Parabola2D Parab1, in IntRes2d_Domain D1, Curve2DAdaptorBase C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Parab1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public Geom2dInt_IntConicCurveOfGInter(in Hyperbola2D Hyper1, in IntRes2d_Domain D1, Curve2DAdaptorBase C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Hyper1, D1, C2, D2, TolConf, Tol)
    {
    }
}
