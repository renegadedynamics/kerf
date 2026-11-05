// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_Curve2dTool.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_Curve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The curve tool of the 2D curve extrema (Extrema_ExtPC2d, Extrema_LocateExtPC2d, Extrema_ECC2d, ...): static
/// functions forwarding to <see cref="Curve2DAdaptorBase"/>, plus <see cref="DeflCurvIntervals"/>.
/// </summary>
/// <remarks>Unlike <see cref="CurveTool"/>, <see cref="IsPeriodic"/> is the curve's own periodicity, as in
/// OCCT.</remarks>
internal static class Curve2dTool
{
    public static double FirstParameter(Curve2DAdaptorBase c) => c.FirstParameter();

    public static double LastParameter(Curve2DAdaptorBase c) => c.LastParameter();

    public static Continuity Continuity(Curve2DAdaptorBase c) => c.Continuity();

    /// <summary>The number of intervals of continuity theS (the array for <see cref="Intervals"/> needs one more).</summary>
    public static int NbIntervals(Curve2DAdaptorBase c, Continuity s) => c.NbIntervals(s);

    /// <summary>Stores the parameters of the intervals of continuity theS in theT.</summary>
    public static void Intervals(Curve2DAdaptorBase c, NCollection_Array1<double> t, Continuity s) =>
        c.Intervals(t, s);

    /// <summary>
    /// Parameters splitting the curve into parts that are each nearly linear in the deflection sense (from
    /// GCPnts_TangentialDeflection), or just the two bounds when the curve is short, nearly constant or very long.
    /// </summary>
    public static NCollection_HArray1<double> DeflCurvIntervals(Curve2DAdaptorBase c)
    {
        const double epsd = 1.0e-3;
        const double maxdefl = 1.0e3;
        const double mindefl = 1.0e-3;
        NCollection_HArray1<double> intervals;
        int nbpnts = 23, i;
        double l = 0.0;
        double tf = c.FirstParameter(), tl = c.LastParameter();
        Point2D p = c.Value(tf);
        for (i = 2; i <= nbpnts; ++i)
        {
            double t = (tf * (nbpnts - i) + (i - 1) * tl) / (nbpnts - 1);
            Point2D p1 = c.Value(t);
            // OCCT never moves aP along, so L sums the distances from the first point, not the polygon's length
            l += p.DistanceTo(p1);
        }
        //
        double dLdt = l / (tl - tf);
        if (l <= global::Kerf.Engine.Foundation.Precision.Confusion() || dLdt < epsd || (tl - tf) > 10000.0)
        {
            nbpnts = 2;
            intervals = new NCollection_HArray1<double>(1, nbpnts);
            intervals.SetValue(1, tf);
            intervals.SetValue(nbpnts, tl);
            return intervals;
        }
        //
        double defl = Max(0.01 * l / (2.0 * M_PI), mindefl);
        if (defl > maxdefl)
        {
            nbpnts = 2;
            intervals = new NCollection_HArray1<double>(1, nbpnts);
            intervals.SetValue(1, tf);
            intervals.SetValue(nbpnts, tl);
            return intervals;
        }
        double minLen = Max(.00001 * l, global::Kerf.Engine.Foundation.Precision.Confusion());
        double tol = Max(0.00001 * (tl - tf), global::Kerf.Engine.Foundation.Precision.PConfusion());
        TangentialDeflection pntGen = new(c, M_PI / 6, defl, 2, tol, minLen);
        nbpnts = pntGen.PointCount;
        intervals = new NCollection_HArray1<double>(1, nbpnts);
        for (i = 1; i <= nbpnts; ++i)
        {
            intervals.SetValue(i, pntGen.Parameter(i));
        }
        return intervals;
    }

    public static bool IsClosed(Curve2DAdaptorBase c) => c.IsClosed;

    public static bool IsPeriodic(Curve2DAdaptorBase c) => c.IsPeriodic;

    public static double Period(Curve2DAdaptorBase c) => c.Period();

    public static Point2D Value(Curve2DAdaptorBase c, double u) => c.Value(u);

    public static void D0(Curve2DAdaptorBase c, double u, out Point2D p) => c.D0(u, out p);

    public static void D1(Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v) =>
        c.D1(u, out p, out v);

    public static void D2(Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v1,
        out Vector2D v2) => c.D2(u, out p, out v1, out v2);

    public static void D3(Curve2DAdaptorBase c, double u, out Point2D p, out Vector2D v1,
        out Vector2D v2, out Vector2D v3) => c.D3(u, out p, out v1, out v2, out v3);

    public static Vector2D DN(Curve2DAdaptorBase c, double u, int n) => c.DN(u, n);

    public static double Resolution(Curve2DAdaptorBase c, double r3d) => c.Resolution(r3d);

    public static CurveKind GetType(Curve2DAdaptorBase c) => c.GetType();

    public static Line2D Line(Curve2DAdaptorBase c) => c.Line();

    public static Circle2D Circle(Curve2DAdaptorBase c) => c.Circle();

    public static Ellipse2D Ellipse(Curve2DAdaptorBase c) => c.Ellipse();

    public static Hyperbola2D Hyperbola(Curve2DAdaptorBase c) => c.Hyperbola();

    public static Parabola2D Parabola(Curve2DAdaptorBase c) => c.Parabola();

    public static int Degree(Curve2DAdaptorBase c) => c.Degree();

    public static bool IsRational(Curve2DAdaptorBase c) => c.IsRational;

    public static int NbPoles(Curve2DAdaptorBase c) => c.PoleCount;

    public static int NbKnots(Curve2DAdaptorBase c) => c.KnotCount;

    public static BezierCurve2D Bezier(Curve2DAdaptorBase c) => c.Bezier();

    public static BSplineCurve2D BSpline(Curve2DAdaptorBase c) => c.BSpline();
}
