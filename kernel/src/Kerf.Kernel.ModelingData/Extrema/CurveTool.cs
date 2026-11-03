// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_CurveTool.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_CurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The curve tool of the 3D curve extrema (Extrema_ExtPC, Extrema_LocateExtPC, Extrema_ECC, ...): static functions
/// forwarding to <see cref="CurveAdaptorBase"/>, plus <see cref="DeflCurvIntervals"/>.
/// </summary>
public static class CurveTool
{
    public static double FirstParameter(CurveAdaptorBase c) => c.FirstParameter();

    public static double LastParameter(CurveAdaptorBase c) => c.LastParameter();

    public static Continuity Continuity(CurveAdaptorBase c) => c.Continuity();

    /// <summary>The number of intervals of continuity theS (the array for <see cref="Intervals"/> needs one more).</summary>
    public static int NbIntervals(CurveAdaptorBase c, Continuity s) => c.NbIntervals(s);

    /// <summary>Stores the parameters of the intervals of continuity theS in theT.</summary>
    public static void Intervals(CurveAdaptorBase c, NCollection_Array1<double> t, Continuity s) =>
        c.Intervals(t, s);

    /// <summary>
    /// Parameters splitting the curve into parts that are each nearly linear in the deflection sense (from
    /// GCPnts_TangentialDeflection), or just the two bounds when the curve is short, nearly constant or very long.
    /// </summary>
    public static NCollection_HArray1<double> DeflCurvIntervals(CurveAdaptorBase c)
    {
        const double epsd = 1.0e-3;
        const double maxdefl = 1.0e3;
        const double mindefl = 1.0e-3;
        NCollection_HArray1<double> intervals;
        int nbpnts = 23, i;
        double l = 0.0;
        double tf = c.FirstParameter(), tl = c.LastParameter();
        Point3D p = c.Value(tf);
        for (i = 2; i <= nbpnts; ++i)
        {
            double t = (tf * (nbpnts - i) + (i - 1) * tl) / (nbpnts - 1);
            Point3D p1 = c.Value(t);
            // OCCT never moves aP along, so L sums the distances from the first point, not the polygon's length
            l += p.DistanceTo(p1);
        }
        //
        double dLdt = l / (tl - tf);
        if (l <= global::Kerf.Engine.Foundation.Precision.Confusion() || dLdt < epsd
            || (tl - tf) > 10000.0) // To avoid problemwith GCPnts
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
        //
        double minLen = Max(.00001 * l, global::Kerf.Engine.Foundation.Precision.Confusion());
        double tol = Max(0.00001 * (tl - tf), global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        TangentialDeflection pntGen = new(c, M_PI / 6, defl, 2, tol, minLen);
        nbpnts = pntGen.PointCount;
        intervals = new NCollection_HArray1<double>(1, nbpnts);
        for (i = 1; i <= nbpnts; ++i)
        {
            intervals.SetValue(i, pntGen.Parameter(i));
        }
        return intervals;
    }

    /// <summary>True for circles and ellipses (whatever their range), otherwise the curve's own periodicity.</summary>
    public static bool IsPeriodic(CurveAdaptorBase c)
    {
        CurveKind type = GetType(c);
        if (type == global::Kerf.Geometry.CurveKind.Circle || type == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            return true;
        }
        else
        {
            return c.IsPeriodic;
        }
    }

    public static double Period(CurveAdaptorBase c) => c.Period();

    public static double Resolution(CurveAdaptorBase c, double r3d) => c.Resolution(r3d);

    public static CurveKind GetType(CurveAdaptorBase c) => c.GetType();

    public static Point3D Value(CurveAdaptorBase c, double u) => c.Value(u);

    public static void D0(CurveAdaptorBase c, double u, out Point3D p) => c.D0(u, out p);

    public static void D1(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v) =>
        c.D1(u, out p, out v);

    public static void D2(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2) =>
        c.D2(u, out p, out v1, out v2);

    public static void D3(CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2,
        out Vector3D v3) => c.D3(u, out p, out v1, out v2, out v3);

    public static Vector3D DN(CurveAdaptorBase c, double u, int n) => c.DN(u, n);

    public static Line3D Line(CurveAdaptorBase c) => c.Line();

    public static Circle3D Circle(CurveAdaptorBase c) => c.Circle();

    public static Ellipse3D Ellipse(CurveAdaptorBase c) => c.Ellipse();

    public static Hyperbola3D Hyperbola(CurveAdaptorBase c) => c.Hyperbola();

    public static Parabola3D Parabola(CurveAdaptorBase c) => c.Parabola();

    public static int Degree(CurveAdaptorBase c) => c.Degree();

    public static bool IsRational(CurveAdaptorBase c) => c.IsRational;

    public static int NbPoles(CurveAdaptorBase c) => c.PoleCount;

    public static int NbKnots(CurveAdaptorBase c) => c.KnotCount;

    public static BezierCurve3D Bezier(CurveAdaptorBase c) => c.Bezier();

    public static BSplineCurve3D BSpline(CurveAdaptorBase c) => c.BSpline();
}
