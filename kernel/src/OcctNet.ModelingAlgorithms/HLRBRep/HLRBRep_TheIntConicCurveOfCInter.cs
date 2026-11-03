// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntConicCurveOfCInter.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheIntConicCurveOfCInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The intersection between a conic (seen as an implicit curve) and a bounded 2D curve.</summary>
/// <remarks>OCCT's instantiation of IntCurve_IntConicCurveGen.gxx over HLRBRep_Curve (decision 321).</remarks>
public sealed class HLRBRep_TheIntConicCurveOfCInter
    : IntCurve_IntConicCurveGen<HLRBRep_Curve, HLRBRep_CurveTool, HLRBRep_TheProjPCurOfCInter,
                               HLRBRep_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfCInter>
{
    /// <summary>Empty constructor.</summary>
    public HLRBRep_TheIntConicCurveOfCInter()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public HLRBRep_TheIntConicCurveOfCInter(in Line2D L, in IntRes2d_Domain D1, HLRBRep_Curve PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(L, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public HLRBRep_TheIntConicCurveOfCInter(in Circle2D C, in IntRes2d_Domain D1, HLRBRep_Curve PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(C, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public HLRBRep_TheIntConicCurveOfCInter(in Ellipse2D E, in IntRes2d_Domain D1, HLRBRep_Curve PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(E, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public HLRBRep_TheIntConicCurveOfCInter(in Parabola2D Prb, in IntRes2d_Domain D1, HLRBRep_Curve PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(Prb, D1, PCurve, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public HLRBRep_TheIntConicCurveOfCInter(in Hyperbola2D H, in IntRes2d_Domain D1, HLRBRep_Curve PCurve, in IntRes2d_Domain D2,
                                              double TolConf, double Tol)
        : base(H, D1, PCurve, D2, TolConf, Tol)
    {
    }
}
