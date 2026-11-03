// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_IntConicCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_IntConicCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The intersection between a conic and a 2D curve that may be a conic or composite.</summary>
/// <remarks>OCCT's instantiation of IntCurve_UserIntConicCurveGen.gxx over HLRBRep_Curve (decision 321).</remarks>
public class HLRBRep_IntConicCurveOfCInter
    : IntCurve_UserIntConicCurveGen<HLRBRep_Curve, HLRBRep_CurveTool, HLRBRep_TheProjPCurOfCInter,
                               HLRBRep_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public HLRBRep_IntConicCurveOfCInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public HLRBRep_IntConicCurveOfCInter(in Line2D Lin1, in IntRes2d_Domain D1, HLRBRep_Curve C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Lin1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public HLRBRep_IntConicCurveOfCInter(in Circle2D Circ1, in IntRes2d_Domain D1, HLRBRep_Curve C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Circ1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public HLRBRep_IntConicCurveOfCInter(in Ellipse2D Elips1, in IntRes2d_Domain D1, HLRBRep_Curve C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Elips1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public HLRBRep_IntConicCurveOfCInter(in Parabola2D Parab1, in IntRes2d_Domain D1, HLRBRep_Curve C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Parab1, D1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public HLRBRep_IntConicCurveOfCInter(in Hyperbola2D Hyper1, in IntRes2d_Domain D1, HLRBRep_Curve C2, in IntRes2d_Domain D2,
                                           double TolConf, double Tol)
        : base(Hyper1, D1, C2, D2, TolConf, Tol)
    {
    }
}
