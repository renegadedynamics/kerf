// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_HCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>Static access to a curve adaptor, as the blend walkers call it.</summary>
internal static class HCurveTool
{
    public static double FirstParameter(CurveAdaptorBase c) => c.FirstParameter();

    public static double LastParameter(CurveAdaptorBase c) => c.LastParameter();

    public static Continuity Continuity(CurveAdaptorBase c) => c.Continuity();

    public static int NbIntervals(CurveAdaptorBase c, Continuity s) => c.NbIntervals(s);

    public static void Intervals(CurveAdaptorBase c, NCollection_Array1<double> t, Continuity s) =>
        c.Intervals(t, s);

    public static bool IsClosed(CurveAdaptorBase c) => c.IsClosed;

    public static bool IsPeriodic(CurveAdaptorBase c) => c.IsPeriodic;

    public static double Period(CurveAdaptorBase c) => c.Period();

    public static Point3D Value(CurveAdaptorBase c, double u) => c.Value(u);

    public static void D0(CurveAdaptorBase c, double u, out Point3D p) => c.D0(u, out p);

    public static void D1(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v) => c.D1(u, out p, out v);

    public static void D2(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2) =>
        c.D2(u, out p, out v1, out v2);

    public static void D3(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2,
                          out Vector3D v3) => c.D3(u, out p, out v1, out v2, out v3);

    public static Vector3D DN(CurveAdaptorBase c, double u, int n) => c.DN(u, n);

    public static double Resolution(CurveAdaptorBase c, double r3d) => c.Resolution(r3d);

    public static CurveKind GetType(CurveAdaptorBase c) => c.GetType();

    public static Line3D Line(CurveAdaptorBase c) => c.Line();

    public static Circle3D Circle(CurveAdaptorBase c) => c.Circle();

    public static Ellipse3D Ellipse(CurveAdaptorBase c) => c.Ellipse();

    public static Hyperbola3D Hyperbola(CurveAdaptorBase c) => c.Hyperbola();

    public static Parabola3D Parabola(CurveAdaptorBase c) => c.Parabola();

    public static BezierCurve3D Bezier(CurveAdaptorBase c) => c.Bezier();

    public static BSplineCurve3D BSpline(CurveAdaptorBase c) => c.BSpline();

    /// <summary>The number of samples on the curve between <paramref name="u0"/> and <paramref name="u1"/> (2 to 50).</summary>
    public static int NbSamples(CurveAdaptorBase c, double u0, double u1)
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
            BSplineCurve3D bsc = c.BSpline();
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
