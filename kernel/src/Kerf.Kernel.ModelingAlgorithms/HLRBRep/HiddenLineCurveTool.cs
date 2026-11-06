// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CurveTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CurveTool.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_CurveTool.cxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TypeDef.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.HiddenLines;

/// <summary>The tool over the projected edges (HLRBRep_Curve) that HLRBRep's 2D curve intersections use.</summary>
/// <remarks>
/// A struct without state, so it can be a type argument of IntCurve's generic classes (decision 322). OCCT's curve
/// type is <c>HLRBRep_CurvePtr</c>, a void pointer cast to <c>HLRBRep_Curve*</c>; the port's type argument is the
/// class <see cref="HiddenLineCurve"/> itself (decision 1091). OCCT's <c>Bezier</c> and <c>BSpline</c> return null
/// handles ("not implemented") and are left out with nothing calling them. OCCT's function-scope static
/// <c>nbsOther</c>, never written, is a constant.
/// </remarks>
internal readonly struct HiddenLineCurveTool : ICurveCurveTool<HiddenLineCurve>
{
    // OCCT's IS_C2_COMPOSITE is 0: the curves are split at their C1 discontinuities.

    public static double FirstParameter(in HiddenLineCurve c) => (c.FirstParameter());

    public static double LastParameter(in HiddenLineCurve c) => (c.LastParameter());

    public static Continuity Continuity(in HiddenLineCurve c) => (c.Continuity());

    /// <summary>The number of intervals of C1 continuity.</summary>
    public static int NbIntervals(in HiddenLineCurve c) => (c.NbIntervals(C1));

    /// <summary>The bounds of the intervals of C1 continuity.</summary>
    public static void Intervals(in HiddenLineCurve c, NCollection_Array1<double> tab)
    {
        c.Intervals(tab, C1);
    }

    public static void GetInterval(in HiddenLineCurve c, int i, NCollection_Array1<double> tab, out double a,
                                   out double b)
    {
        a = tab.Value(i);
        b = tab.Value(i + 1);
    }

    public static bool IsClosed(in HiddenLineCurve c) => (c.IsClosed);

    public static bool IsPeriodic(in HiddenLineCurve c) => (c.IsPeriodic);

    public static double Period(in HiddenLineCurve c) => (c.Period());

    public static Point2D Value(in HiddenLineCurve c, double u) => (c.Value(u));

    public static void D0(in HiddenLineCurve c, double u, out Point2D p)
    {
        c.D0(u, out p);
    }

    public static void D1(in HiddenLineCurve c, double u, out Point2D p, out Vector2D t)
    {
        c.D1(u, out p, out t);
    }

    public static void D2(in HiddenLineCurve c, double u, out Point2D p, out Vector2D t, out Vector2D n)
    {
        c.D2(u, out p, out t, out n);
    }

    /// <summary>OCCT's HLRBRep_Curve::D3, which writes nothing.</summary>
    public static void D3(in HiddenLineCurve c, double u, ref Point2D p, ref Vector2D v1, ref Vector2D v2,
                          ref Vector2D v3)
    {
        c.D3(u, ref p, ref v1, ref v2, ref v3);
    }

    public static Vector2D DN(in HiddenLineCurve c, double u, int n) => (c.DN(u, n));

    public static double Resolution(in HiddenLineCurve c, double r3d) => (c.Resolution(r3d));

    public static CurveKind GetType(in HiddenLineCurve c) => (c.GetType());

    public static CurveKind TheType(in HiddenLineCurve c) => (c.GetType());

    public static Line2D Line(in HiddenLineCurve c) => (c.Line());

    public static Circle2D Circle(in HiddenLineCurve c) => (c.Circle());

    public static Ellipse2D Ellipse(in HiddenLineCurve c) => (c.Ellipse());

    public static Parabola2D Parabola(in HiddenLineCurve c) => (c.Parabola());

    public static Hyperbola2D Hyperbola(in HiddenLineCurve c) => (c.Hyperbola());

    public static double EpsX(in HiddenLineCurve c) => (1e-10);

    public static int Degree(in HiddenLineCurve c) => (c.Degree());

    /// <summary>The number of samples of the curve on its whole range, between 2 and 50.</summary>
    public static int NbSamples(in HiddenLineCurve c)
    {
        CurveKind typC = c.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + c.PoleCount;
        }
        else if (typC == BSplineCurve)
        {
            nbs = c.KnotCount;
            nbs *= c.Degree();
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        if (nbs > 50)
        {
            nbs = 50;
        }
        return ((int)nbs);
    }

    /// <summary>The number of samples of the curve on [u1, u2], between 2 and 50.</summary>
    public static int NbSamples(in HiddenLineCurve c, double u1, double u2)
    {
        CurveKind typC = c.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + c.PoleCount;
        }
        else if (typC == BSplineCurve)
        {
            Curve3D? curve = c.Curve().Curve().Curve();
            CurveAdaptor gAcurve = new(curve!, u1, u2);
            nbs = gAcurve.NbIntervals(CN) + 1;
            nbs *= c.Degree();
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        if (nbs > 50)
        {
            nbs = 50;
        }
        return ((int)nbs);
    }
}
