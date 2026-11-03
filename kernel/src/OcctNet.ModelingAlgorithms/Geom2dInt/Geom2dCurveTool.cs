// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_Geom2dCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_Geom2dCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_Geom2dCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>The tool over Adaptor2d_Curve2d that the 2D curve intersections use.</summary>
/// <remarks>A struct without state, so it can be a type argument of IntCurve's generic classes (decision 322).</remarks>
public readonly struct Geom2dCurveTool : ICurveCurveTool<Curve2DAdaptorBase>
{
    // OCCT's IS_C2_COMPOSITE is 0: the curves are split at their C1 discontinuities.

    public static CurveKind GetType(in Curve2DAdaptorBase c) => (c.GetType());

    public static Line2D Line(in Curve2DAdaptorBase c) => (c.Line());

    public static Circle2D Circle(in Curve2DAdaptorBase c) => (c.Circle());

    public static Ellipse2D Ellipse(in Curve2DAdaptorBase c) => (c.Ellipse());

    public static Parabola2D Parabola(in Curve2DAdaptorBase c) => (c.Parabola());

    public static Hyperbola2D Hyperbola(in Curve2DAdaptorBase c) => (c.Hyperbola());

    public static Point2D Value(in Curve2DAdaptorBase c, double u) => (c.Value(u));

    public static void D0(in Curve2DAdaptorBase c, double u, out Point2D p)
    {
        c.D0(u, out p);
    }

    public static void D1(in Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D t)
    {
        c.D1(u, out p, out t);
    }

    public static void D2(in Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D t, out Vector2D n)
    {
        c.D2(u, out p, out t, out n);
    }

    public static void D3(in Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D t, out Vector2D n,
                          out Vector2D v)
    {
        c.D3(u, out p, out t, out n, out v);
    }

    public static Vector2D DN(in Curve2DAdaptorBase c, double u, int n) => c.DN(u, n);

    public static double FirstParameter(in Curve2DAdaptorBase c) => (c.FirstParameter());

    public static double LastParameter(in Curve2DAdaptorBase c) => (c.LastParameter());

    /// <summary>The tolerance used to determine the intersections (1e-10).</summary>
    public static double EpsX(in Curve2DAdaptorBase c) => (1.0e-10);

    /// <summary>The parametric resolution of the 2D tolerance <paramref name="Eps_XYZ"/>.</summary>
    public static double EpsX(in Curve2DAdaptorBase c, double Eps_XYZ) => (c.Resolution(Eps_XYZ));

    public static void Intervals(in Curve2DAdaptorBase c, NCollection_Array1<double> tab)
    {
        c.Intervals(tab, C1);
    }

    public static void GetInterval(in Curve2DAdaptorBase c, int i, NCollection_Array1<double> tab, out double a,
                                   out double b)
    {
        a = tab.Value(i);
        b = tab.Value(i + 1);
    }

    public static int NbIntervals(in Curve2DAdaptorBase c)
    {
        int n = 1;
        n = c.NbIntervals(C1);
        return (n);
    }

    public static int Degree(in Curve2DAdaptorBase c) => c.Degree();

    /// <summary>The number of samples of the curve between <paramref name="u0"/> and <paramref name="u1"/>.</summary>
    public static int NbSamples(in Curve2DAdaptorBase c, double u0, double u1)
    {
        CurveKind typC = c.GetType();
        int nbs = c.SampleCount;

        if (typC == BSplineCurve)
        {
            double t = c.LastParameter() - c.FirstParameter();
            if (t > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                double t1 = u1 - u0;
                if (t1 < 0.0)
                {
                    t1 = -t1;
                }
                nbs = c.KnotCount;
                nbs *= c.Degree();
                double anb = t1 / t * nbs;
                nbs = (int)anb;

                int minPntNb = Max(c.Degree() + 1, 4);
                if (nbs < minPntNb)
                {
                    nbs = minPntNb;
                }
            }
        }
        else if (typC == global::Kerf.Geometry.CurveKind.Circle)
        {
            // Try to reach deflection = eps*R, eps = 0.01
            const double minR = 1.0; // eps = 0.01
            double r = c.Circle().Radius();
            if (r > minR)
            {
                double angl = 0.283079; // 2.*std::acos(1. - eps);
                int n = RealToInt(Math.Abs(u1 - u0) / angl);
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
    public static int NbSamples(in Curve2DAdaptorBase c)
    {
        int nbs = c.SampleCount;
        CurveKind typC = c.GetType();
        if (typC == global::Kerf.Geometry.CurveKind.Circle)
        {
            // Try to reach deflection = eps*R, eps = 0.01
            const double minR = 1.0; // eps = 0.01
            double r = c.Circle().Radius();
            if (r > minR)
            {
                double angl = 0.283079; // 2.*std::acos(1. - eps);
                int n = RealToInt((c.LastParameter() - c.FirstParameter()) / angl);
                nbs = Max(n, nbs);
            }
        }

        return nbs;
    }
}
