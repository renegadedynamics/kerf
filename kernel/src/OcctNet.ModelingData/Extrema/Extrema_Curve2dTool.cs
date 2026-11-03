// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_Curve2dTool.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_Curve2dTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The curve tool of the 2D curve extrema (Extrema_ExtPC2d, Extrema_LocateExtPC2d, Extrema_ECC2d, ...): static
/// functions forwarding to <see cref="Adaptor2d_Curve2d"/>, plus <see cref="DeflCurvIntervals"/>.
/// </summary>
/// <remarks>Unlike <see cref="Extrema_CurveTool"/>, <see cref="IsPeriodic"/> is the curve's own periodicity, as in
/// OCCT.</remarks>
public static class Extrema_Curve2dTool
{
    public static double FirstParameter(Adaptor2d_Curve2d theC) => theC.FirstParameter();

    public static double LastParameter(Adaptor2d_Curve2d theC) => theC.LastParameter();

    public static Continuity Continuity(Adaptor2d_Curve2d theC) => theC.Continuity();

    /// <summary>The number of intervals of continuity theS (the array for <see cref="Intervals"/> needs one more).</summary>
    public static int NbIntervals(Adaptor2d_Curve2d theC, Continuity theS) => theC.NbIntervals(theS);

    /// <summary>Stores the parameters of the intervals of continuity theS in theT.</summary>
    public static void Intervals(Adaptor2d_Curve2d theC, NCollection_Array1<double> theT, Continuity theS) =>
        theC.Intervals(theT, theS);

    /// <summary>
    /// Parameters splitting the curve into parts that are each nearly linear in the deflection sense (from
    /// GCPnts_TangentialDeflection), or just the two bounds when the curve is short, nearly constant or very long.
    /// </summary>
    public static NCollection_HArray1<double> DeflCurvIntervals(Adaptor2d_Curve2d C)
    {
        const double epsd = 1.0e-3;
        const double maxdefl = 1.0e3;
        const double mindefl = 1.0e-3;
        NCollection_HArray1<double> Intervals;
        int nbpnts = 23, i;
        double L = 0.0;
        double tf = C.FirstParameter(), tl = C.LastParameter();
        Point2D aP = C.Value(tf);
        for (i = 2; i <= nbpnts; ++i)
        {
            double t = (tf * (nbpnts - i) + (i - 1) * tl) / (nbpnts - 1);
            Point2D aP1 = C.Value(t);
            // OCCT never moves aP along, so L sums the distances from the first point, not the polygon's length
            L += aP.Distance(aP1);
        }
        //
        double dLdt = L / (tl - tf);
        if (L <= global::Kerf.Engine.Foundation.Precision.Confusion() || dLdt < epsd || (tl - tf) > 10000.0)
        {
            nbpnts = 2;
            Intervals = new NCollection_HArray1<double>(1, nbpnts);
            Intervals.SetValue(1, tf);
            Intervals.SetValue(nbpnts, tl);
            return Intervals;
        }
        //
        double aDefl = Max(0.01 * L / (2.0 * M_PI), mindefl);
        if (aDefl > maxdefl)
        {
            nbpnts = 2;
            Intervals = new NCollection_HArray1<double>(1, nbpnts);
            Intervals.SetValue(1, tf);
            Intervals.SetValue(nbpnts, tl);
            return Intervals;
        }
        double aMinLen = Max(.00001 * L, global::Kerf.Engine.Foundation.Precision.Confusion());
        double aTol = Max(0.00001 * (tl - tf), global::Kerf.Engine.Foundation.Precision.PConfusion());
        GCPnts_TangentialDeflection aPntGen = new(C, M_PI / 6, aDefl, 2, aTol, aMinLen);
        nbpnts = aPntGen.NbPoints();
        Intervals = new NCollection_HArray1<double>(1, nbpnts);
        for (i = 1; i <= nbpnts; ++i)
        {
            Intervals.SetValue(i, aPntGen.Parameter(i));
        }
        return Intervals;
    }

    public static bool IsClosed(Adaptor2d_Curve2d theC) => theC.IsClosed();

    public static bool IsPeriodic(Adaptor2d_Curve2d theC) => theC.IsPeriodic();

    public static double Period(Adaptor2d_Curve2d theC) => theC.Period();

    public static Point2D Value(Adaptor2d_Curve2d theC, double theU) => theC.Value(theU);

    public static void D0(Adaptor2d_Curve2d theC, double theU, out Point2D theP) => theC.D0(theU, out theP);

    public static void D1(Adaptor2d_Curve2d theC, double theU, out Point2D theP, out Vector2D theV) =>
        theC.D1(theU, out theP, out theV);

    public static void D2(Adaptor2d_Curve2d theC, double theU, out Point2D theP, out Vector2D theV1,
        out Vector2D theV2) => theC.D2(theU, out theP, out theV1, out theV2);

    public static void D3(Adaptor2d_Curve2d theC, double theU, out Point2D theP, out Vector2D theV1,
        out Vector2D theV2, out Vector2D theV3) => theC.D3(theU, out theP, out theV1, out theV2, out theV3);

    public static Vector2D DN(Adaptor2d_Curve2d theC, double theU, int theN) => theC.DN(theU, theN);

    public static double Resolution(Adaptor2d_Curve2d theC, double theR3d) => theC.Resolution(theR3d);

    public static CurveKind GetType(Adaptor2d_Curve2d theC) => theC.GetType();

    public static Line2D Line(Adaptor2d_Curve2d theC) => theC.Line();

    public static Circle2D Circle(Adaptor2d_Curve2d theC) => theC.Circle();

    public static Ellipse2D Ellipse(Adaptor2d_Curve2d theC) => theC.Ellipse();

    public static Hyperbola2D Hyperbola(Adaptor2d_Curve2d theC) => theC.Hyperbola();

    public static Parabola2D Parabola(Adaptor2d_Curve2d theC) => theC.Parabola();

    public static int Degree(Adaptor2d_Curve2d theC) => theC.Degree();

    public static bool IsRational(Adaptor2d_Curve2d theC) => theC.IsRational();

    public static int NbPoles(Adaptor2d_Curve2d theC) => theC.NbPoles();

    public static int NbKnots(Adaptor2d_Curve2d theC) => theC.NbKnots();

    public static Geom2d_BezierCurve Bezier(Adaptor2d_Curve2d theC) => theC.Bezier();

    public static Geom2d_BSplineCurve BSpline(Adaptor2d_Curve2d theC) => theC.BSpline();
}
