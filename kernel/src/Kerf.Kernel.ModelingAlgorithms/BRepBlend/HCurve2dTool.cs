// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>Static access to a 2D curve adaptor (an arc of restriction), as the blend walkers call it.</summary>
internal static class HCurve2dTool
{
    public static double FirstParameter(Curve2DAdaptorBase c) => c.FirstParameter();

    public static double LastParameter(Curve2DAdaptorBase c) => c.LastParameter();

    public static Continuity Continuity(Curve2DAdaptorBase c) => c.Continuity();

    public static int NbIntervals(Curve2DAdaptorBase c, Continuity s) => c.NbIntervals(s);

    public static void Intervals(Curve2DAdaptorBase c, NCollection_Array1<double> t, Continuity s) =>
        c.Intervals(t, s);

    public static bool IsClosed(Curve2DAdaptorBase c) => c.IsClosed;

    public static bool IsPeriodic(Curve2DAdaptorBase c) => c.IsPeriodic;

    public static double Period(Curve2DAdaptorBase c) => c.Period();

    public static Point2D Value(Curve2DAdaptorBase c, double u) => c.Value(u);

    public static void D0(Curve2DAdaptorBase c, double u, out Point2D p) => c.D0(u, out p);

    public static void D1(Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v) => c.D1(u, out p, out v);

    public static void D2(Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v1, out Vector2D v2) =>
        c.D2(u, out p, out v1, out v2);

    public static void D3(Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v1, out Vector2D v2,
                          out Vector2D v3) => c.D3(u, out p, out v1, out v2, out v3);

    public static Vector2D DN(Curve2DAdaptorBase c, double u, int n) => c.DN(u, n);

    public static double Resolution(Curve2DAdaptorBase c, double r3d) => c.Resolution(r3d);

    public static CurveKind GetType(Curve2DAdaptorBase c) => c.GetType();

    public static Line2D Line(Curve2DAdaptorBase c) => c.Line();

    public static Circle2D Circle(Curve2DAdaptorBase c) => c.Circle();

    public static Ellipse2D Ellipse(Curve2DAdaptorBase c) => c.Ellipse();

    public static Hyperbola2D Hyperbola(Curve2DAdaptorBase c) => c.Hyperbola();

    public static Parabola2D Parabola(Curve2DAdaptorBase c) => c.Parabola();

    public static BezierCurve2D Bezier(Curve2DAdaptorBase c) => c.Bezier();

    public static BSplineCurve2D BSpline(Curve2DAdaptorBase c) => c.BSpline();

    /// <summary>The number of samples on the curve between <paramref name="u0"/> and <paramref name="u1"/> (2 to 50).</summary>
    public static int NbSamples(Curve2DAdaptorBase c, double u0, double u1)
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
            nbs = 3 + c.Bezier().PoleCount;
        }
        else if (typC == BSplineCurve)
        {
            BSplineCurve2D bsc = c.BSpline();
            nbs = bsc.KnotCount;
            nbs *= bsc.Degree;
            nbs *= bsc.LastParameter - bsc.FirstParameter;
            nbs /= u1 - u0;
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
