// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_Geom2dCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_Geom2dCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_Geom2dCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>The tool over Adaptor2d_Curve2d that the 2D curve intersections use.</summary>
/// <remarks>A struct without state, so it can be a type argument of IntCurve's generic classes (decision 3502).</remarks>
public readonly struct Geom2dInt_Geom2dCurveTool : IIntCurve_CurveTool<Adaptor2d_Curve2d>
{
    // OCCT's IS_C2_COMPOSITE is 0: the curves are split at their C1 discontinuities.

    public static GeomAbs_CurveType GetType(in Adaptor2d_Curve2d C) => (C.GetType());

    public static gp_Lin2d Line(in Adaptor2d_Curve2d C) => (C.Line());

    public static gp_Circ2d Circle(in Adaptor2d_Curve2d C) => (C.Circle());

    public static gp_Elips2d Ellipse(in Adaptor2d_Curve2d C) => (C.Ellipse());

    public static gp_Parab2d Parabola(in Adaptor2d_Curve2d C) => (C.Parabola());

    public static gp_Hypr2d Hyperbola(in Adaptor2d_Curve2d C) => (C.Hyperbola());

    public static gp_Pnt2d Value(in Adaptor2d_Curve2d C, double U) => (C.Value(U));

    public static void D0(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P)
    {
        C.D0(U, out P);
    }

    public static void D1(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d T)
    {
        C.D1(U, out P, out T);
    }

    public static void D2(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d T, out gp_Vec2d N)
    {
        C.D2(U, out P, out T, out N);
    }

    public static void D3(in Adaptor2d_Curve2d C, double U, out gp_Pnt2d P, out gp_Vec2d T, out gp_Vec2d N,
                          out gp_Vec2d V)
    {
        C.D3(U, out P, out T, out N, out V);
    }

    public static gp_Vec2d DN(in Adaptor2d_Curve2d C, double U, int N) => C.DN(U, N);

    public static double FirstParameter(in Adaptor2d_Curve2d C) => (C.FirstParameter());

    public static double LastParameter(in Adaptor2d_Curve2d C) => (C.LastParameter());

    /// <summary>The tolerance used to determine the intersections (1e-10).</summary>
    public static double EpsX(in Adaptor2d_Curve2d C) => (1.0e-10);

    /// <summary>The parametric resolution of the 2D tolerance <paramref name="Eps_XYZ"/>.</summary>
    public static double EpsX(in Adaptor2d_Curve2d C, double Eps_XYZ) => (C.Resolution(Eps_XYZ));

    public static void Intervals(in Adaptor2d_Curve2d C, NCollection_Array1<double> Tab)
    {
        C.Intervals(Tab, GeomAbs_C1);
    }

    public static void GetInterval(in Adaptor2d_Curve2d C, int i, NCollection_Array1<double> Tab, out double a,
                                   out double b)
    {
        a = Tab.Value(i);
        b = Tab.Value(i + 1);
    }

    public static int NbIntervals(in Adaptor2d_Curve2d C)
    {
        int N = 1;
        N = C.NbIntervals(GeomAbs_C1);
        return (N);
    }

    public static int Degree(in Adaptor2d_Curve2d C) => C.Degree();

    /// <summary>The number of samples of the curve between <paramref name="U0"/> and <paramref name="U1"/>.</summary>
    public static int NbSamples(in Adaptor2d_Curve2d C, double U0, double U1)
    {
        GeomAbs_CurveType typC = C.GetType();
        int nbs = C.NbSamples();

        if (typC == GeomAbs_BSplineCurve)
        {
            double t = C.LastParameter() - C.FirstParameter();
            if (t > Precision.PConfusion())
            {
                double t1 = U1 - U0;
                if (t1 < 0.0)
                {
                    t1 = -t1;
                }
                nbs = C.NbKnots();
                nbs *= C.Degree();
                double anb = t1 / t * nbs;
                nbs = (int)anb;

                int aMinPntNb = Max(C.Degree() + 1, 4);
                if (nbs < aMinPntNb)
                {
                    nbs = aMinPntNb;
                }
            }
        }
        else if (typC == GeomAbs_Circle)
        {
            // Try to reach deflection = eps*R, eps = 0.01
            const double minR = 1.0; // eps = 0.01
            double R = C.Circle().Radius();
            if (R > minR)
            {
                double angl = 0.283079; // 2.*std::acos(1. - eps);
                int n = RealToInt(Math.Abs(U1 - U0) / angl);
                nbs = Max(n, nbs);
            }
        }

        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    /// <summary>The number of samples of the curve on its whole range.</summary>
    public static int NbSamples(in Adaptor2d_Curve2d C)
    {
        int nbs = C.NbSamples();
        GeomAbs_CurveType typC = C.GetType();
        if (typC == GeomAbs_Circle)
        {
            // Try to reach deflection = eps*R, eps = 0.01
            const double minR = 1.0; // eps = 0.01
            double R = C.Circle().Radius();
            if (R > minR)
            {
                double angl = 0.283079; // 2.*std::acos(1. - eps);
                int n = RealToInt((C.LastParameter() - C.FirstParameter()) / angl);
                nbs = Max(n, nbs);
            }
        }

        return nbs;
    }
}
