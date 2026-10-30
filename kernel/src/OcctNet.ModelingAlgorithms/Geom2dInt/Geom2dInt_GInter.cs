// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_GInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_GInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The intersection between two 2D curves (Adaptor2d_Curve2d), or the self-intersection of one.</summary>
/// <remarks>
/// OCCT's instantiation of IntCurve_IntCurveCurveGen.gxx over Adaptor2d_Curve2d (decision 321). The curves are
/// kept by reference during the computation only; the results are values. Not sealed: Geom2dHatch_Intersector derives
/// from it, as in OCCT (decision 5803).
/// </remarks>
public class Geom2dInt_GInter
    : IntCurve_IntCurveCurveGen<Adaptor2d_Curve2d, Geom2dInt_Geom2dCurveTool, Geom2dInt_TheProjPCurOfGInter,
                                Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter>
{
    /// <summary>Empty constructor.</summary>
    public Geom2dInt_GInter()
    {
    }

    /// <summary>Self-intersection of a curve.</summary>
    public Geom2dInt_GInter(Adaptor2d_Curve2d C, double TolConf, double Tol)
        : base(C, TolConf, Tol)
    {
    }

    /// <summary>Self-intersection of a curve on a domain.</summary>
    public Geom2dInt_GInter(Adaptor2d_Curve2d C, in IntRes2d_Domain D, double TolConf, double Tol)
        : base(C, D, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves.</summary>
    public Geom2dInt_GInter(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, double TolConf, double Tol)
        : base(C1, C2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public Geom2dInt_GInter(Adaptor2d_Curve2d C1, in IntRes2d_Domain D1, Adaptor2d_Curve2d C2, double TolConf,
                            double Tol)
        : base(C1, D1, C2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public Geom2dInt_GInter(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, in IntRes2d_Domain D2, double TolConf,
                            double Tol)
        : base(C1, C2, D2, TolConf, Tol)
    {
    }

    /// <summary>Intersection between two curves on their domains.</summary>
    public Geom2dInt_GInter(Adaptor2d_Curve2d C1, in IntRes2d_Domain D1, Adaptor2d_Curve2d C2,
                            in IntRes2d_Domain D2, double TolConf, double Tol)
        : base(C1, D1, C2, D2, TolConf, Tol)
    {
    }
}
