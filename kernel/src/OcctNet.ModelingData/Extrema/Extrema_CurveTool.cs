// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_CurveTool.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_CurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The curve tool of the 3D curve extrema (Extrema_ExtPC, Extrema_LocateExtPC, Extrema_ECC, ...): static functions
/// forwarding to <see cref="Adaptor3d_Curve"/>, plus <see cref="DeflCurvIntervals"/>.
/// </summary>
public static class Extrema_CurveTool
{
    public static double FirstParameter(Adaptor3d_Curve theC) => theC.FirstParameter();

    public static double LastParameter(Adaptor3d_Curve theC) => theC.LastParameter();

    public static Continuity Continuity(Adaptor3d_Curve theC) => theC.Continuity();

    /// <summary>The number of intervals of continuity theS (the array for <see cref="Intervals"/> needs one more).</summary>
    public static int NbIntervals(Adaptor3d_Curve theC, Continuity theS) => theC.NbIntervals(theS);

    /// <summary>Stores the parameters of the intervals of continuity theS in theT.</summary>
    public static void Intervals(Adaptor3d_Curve theC, NCollection_Array1<double> theT, Continuity theS) =>
        theC.Intervals(theT, theS);

    /// <summary>
    /// Parameters splitting the curve into parts that are each nearly linear in the deflection sense (from
    /// GCPnts_TangentialDeflection), or just the two bounds when the curve is short, nearly constant or very long.
    /// </summary>
    public static NCollection_HArray1<double> DeflCurvIntervals(Adaptor3d_Curve C)
    {
        const double epsd = 1.0e-3;
        const double maxdefl = 1.0e3;
        const double mindefl = 1.0e-3;
        NCollection_HArray1<double> Intervals;
        int nbpnts = 23, i;
        double L = 0.0;
        double tf = C.FirstParameter(), tl = C.LastParameter();
        Point3D aP = C.Value(tf);
        for (i = 2; i <= nbpnts; ++i)
        {
            double t = (tf * (nbpnts - i) + (i - 1) * tl) / (nbpnts - 1);
            Point3D aP1 = C.Value(t);
            // OCCT never moves aP along, so L sums the distances from the first point, not the polygon's length
            L += aP.Distance(aP1);
        }
        //
        double dLdt = L / (tl - tf);
        if (L <= global::Kerf.Engine.Foundation.Precision.Confusion() || dLdt < epsd
            || (tl - tf) > 10000.0) // To avoid problemwith GCPnts
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
        //
        double aMinLen = Max(.00001 * L, global::Kerf.Engine.Foundation.Precision.Confusion());
        double aTol = Max(0.00001 * (tl - tf), global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        GCPnts_TangentialDeflection aPntGen = new(C, M_PI / 6, aDefl, 2, aTol, aMinLen);
        nbpnts = aPntGen.NbPoints();
        Intervals = new NCollection_HArray1<double>(1, nbpnts);
        for (i = 1; i <= nbpnts; ++i)
        {
            Intervals.SetValue(i, aPntGen.Parameter(i));
        }
        return Intervals;
    }

    /// <summary>True for circles and ellipses (whatever their range), otherwise the curve's own periodicity.</summary>
    public static bool IsPeriodic(Adaptor3d_Curve C)
    {
        CurveKind aType = GetType(C);
        if (aType == global::Kerf.Geometry.CurveKind.Circle || aType == global::Kerf.Geometry.CurveKind.Ellipse)
        {
            return true;
        }
        else
        {
            return C.IsPeriodic();
        }
    }

    public static double Period(Adaptor3d_Curve theC) => theC.Period();

    public static double Resolution(Adaptor3d_Curve theC, double theR3d) => theC.Resolution(theR3d);

    public static CurveKind GetType(Adaptor3d_Curve theC) => theC.GetType();

    public static Point3D Value(Adaptor3d_Curve theC, double theU) => theC.Value(theU);

    public static void D0(Adaptor3d_Curve theC, double theU, out Point3D theP) => theC.D0(theU, out theP);

    public static void D1(Adaptor3d_Curve theC, double theU, out Point3D theP, out Vector3D theV) =>
        theC.D1(theU, out theP, out theV);

    public static void D2(Adaptor3d_Curve theC, double theU, out Point3D theP, out Vector3D theV1, out Vector3D theV2) =>
        theC.D2(theU, out theP, out theV1, out theV2);

    public static void D3(Adaptor3d_Curve theC, double theU, out Point3D theP, out Vector3D theV1, out Vector3D theV2,
        out Vector3D theV3) => theC.D3(theU, out theP, out theV1, out theV2, out theV3);

    public static Vector3D DN(Adaptor3d_Curve theC, double theU, int theN) => theC.DN(theU, theN);

    public static Line3D Line(Adaptor3d_Curve theC) => theC.Line();

    public static Circle3D Circle(Adaptor3d_Curve theC) => theC.Circle();

    public static Ellipse3D Ellipse(Adaptor3d_Curve theC) => theC.Ellipse();

    public static Hyperbola3D Hyperbola(Adaptor3d_Curve theC) => theC.Hyperbola();

    public static Parabola3D Parabola(Adaptor3d_Curve theC) => theC.Parabola();

    public static int Degree(Adaptor3d_Curve theC) => theC.Degree();

    public static bool IsRational(Adaptor3d_Curve theC) => theC.IsRational();

    public static int NbPoles(Adaptor3d_Curve theC) => theC.NbPoles();

    public static int NbKnots(Adaptor3d_Curve theC) => theC.NbKnots();

    public static Geom_BezierCurve Bezier(Adaptor3d_Curve theC) => theC.Bezier();

    public static Geom_BSplineCurve BSpline(Adaptor3d_Curve theC) => theC.BSpline();
}
