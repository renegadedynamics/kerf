// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Static access to a 2D curve adaptor (an arc of restriction), as the blend walkers call it.</summary>
public static class BRepBlend_HCurve2dTool
{
    public static double FirstParameter(Curve2DAdaptorBase C) => C.FirstParameter();

    public static double LastParameter(Curve2DAdaptorBase C) => C.LastParameter();

    public static Continuity Continuity(Curve2DAdaptorBase C) => C.Continuity();

    public static int NbIntervals(Curve2DAdaptorBase C, Continuity S) => C.NbIntervals(S);

    public static void Intervals(Curve2DAdaptorBase C, NCollection_Array1<double> T, Continuity S) =>
        C.Intervals(T, S);

    public static bool IsClosed(Curve2DAdaptorBase C) => C.IsClosed;

    public static bool IsPeriodic(Curve2DAdaptorBase C) => C.IsPeriodic;

    public static double Period(Curve2DAdaptorBase C) => C.Period();

    public static Point2D Value(Curve2DAdaptorBase C, double U) => C.Value(U);

    public static void D0(Curve2DAdaptorBase C, double U, out Point2D P) => C.D0(U, out P);

    public static void D1(Curve2DAdaptorBase C, double U, out Point2D P, out Vector2D V) => C.D1(U, out P, out V);

    public static void D2(Curve2DAdaptorBase C, double U, out Point2D P, out Vector2D V1, out Vector2D V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(Curve2DAdaptorBase C, double U, out Point2D P, out Vector2D V1, out Vector2D V2,
                          out Vector2D V3) => C.D3(U, out P, out V1, out V2, out V3);

    public static Vector2D DN(Curve2DAdaptorBase C, double U, int N) => C.DN(U, N);

    public static double Resolution(Curve2DAdaptorBase C, double R3d) => C.Resolution(R3d);

    public static CurveKind GetType(Curve2DAdaptorBase C) => C.GetType();

    public static Line2D Line(Curve2DAdaptorBase C) => C.Line();

    public static Circle2D Circle(Curve2DAdaptorBase C) => C.Circle();

    public static Ellipse2D Ellipse(Curve2DAdaptorBase C) => C.Ellipse();

    public static Hyperbola2D Hyperbola(Curve2DAdaptorBase C) => C.Hyperbola();

    public static Parabola2D Parabola(Curve2DAdaptorBase C) => C.Parabola();

    public static BezierCurve2D Bezier(Curve2DAdaptorBase C) => C.Bezier();

    public static BSplineCurve2D BSpline(Curve2DAdaptorBase C) => C.BSpline();

    /// <summary>The number of samples on the curve between <paramref name="U0"/> and <paramref name="U1"/> (2 to 50).</summary>
    public static int NbSamples(Curve2DAdaptorBase C, double U0, double U1)
    {
        CurveKind typC = C.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + C.Bezier().PoleCount;
        }
        else if (typC == BSplineCurve)
        {
            BSplineCurve2D BSC = C.BSpline();
            nbs = BSC.KnotCount;
            nbs *= BSC.Degree();
            nbs *= BSC.LastParameter() - BSC.FirstParameter();
            nbs /= U1 - U0;
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
