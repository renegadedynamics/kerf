#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using Xunit.Abstractions;
using static Occt.GeomAbs_Shape;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The curve extrema (unit 3B) against native OCCT on random lines, conics, Bezier and B-spline curves built from the
/// same raw numbers (TestAdaptorOracleTests' generators): Extrema_ExtPC directly (build123d.NET binds it), and
/// Extrema_ExtPC, Extrema_ExtPC2d and Extrema_ExtCC through the native callers GeomAPI_ProjectPointOnCurve,
/// Geom2dAPI_ProjectPointOnCurve and BRepExtrema_DistShapeShape, whose call paths the port's side mirrors. Native OCCT
/// works on its GeomAdaptor_Curve, the port on its own GeomAdaptor_Curve or Geom2dAdaptor_Curve.
/// </summary>
public class ExtremaCurveOracleTests(ITestOutputHelper output)
{
    private const int Cases = 400;

    // Runs the native and the port computations; both must throw, or neither.
    private static bool Both(Oracle theO, Action theNative, Action thePort, string theWhat)
    {
        Exception? aNativeError = null, aPortError = null;
        try
        {
            theNative();
        }
        catch (N.OcctException anE)
        {
            aNativeError = anE;
        }
        try
        {
            thePort();
        }
        catch (Standard_Failure anE)
        {
            aPortError = anE;
        }
        theO.Same(aNativeError is not null, aPortError is not null,
            $"{theWhat} throws (native: {aNativeError?.Message}; port: {aPortError?.Message})");
        return aNativeError is null;
    }

    // A point near the curve (on it, or at a random distance from a point of it) or anywhere.
    private static gp_Pnt RandomPoint(Oracle theO, Adaptor3d_Curve theC)
    {
        int aKind = theO.Int(0, 3);
        if (aKind == 3)
        {
            return theO.Pnt();
        }
        gp_Pnt aP = theC.Value(theO.Real(theC.FirstParameter(), theC.LastParameter()));
        if (aKind == 0)
        {
            return aP;
        }
        double aScale = aKind == 1 ? theO.Real(1e-3, 1) : theO.Real(1, 30);
        return new gp_Pnt(aP.X() + aScale * theO.Real(-1, 1), aP.Y() + aScale * theO.Real(-1, 1),
            aP.Z() + aScale * theO.Real(-1, 1));
    }

    private static gp_Pnt2d RandomPoint2d(Oracle theO, Adaptor2d_Curve2d theC, double theFirst, double theLast)
    {
        int aKind = theO.Int(0, 3);
        if (aKind == 3)
        {
            return theO.Pnt2d();
        }
        gp_Pnt2d aP = theC.Value(theO.Real(theFirst, theLast));
        if (aKind == 0)
        {
            return aP;
        }
        double aScale = aKind == 1 ? theO.Real(1e-3, 1) : theO.Real(1, 30);
        return new gp_Pnt2d(aP.X() + aScale * theO.Real(-1, 1), aP.Y() + aScale * theO.Real(-1, 1));
    }

    private static void SameExtPC(Oracle theO, N.Extrema_ExtPC theN, Extrema_ExtPC theP, string theWhat,
        double theTol = Oracle.Tolerance)
    {
        theO.Same(theN.IsDone(), theP.IsDone(), $"{theWhat} IsDone");
        if (!theP.IsDone())
        {
            return;
        }
        theO.Same(theN.NbExt(), theP.NbExt(), $"{theWhat} NbExt");
        for (int j = 1; j <= theP.NbExt(); j++)
        {
            using N.Extrema_POnCurv aNPC = theN.Point(j);
            using N.gp_Pnt aNV = aNPC.Value();
            Extrema_POnCurv aPC = theP.Point(j);
            theO.Same(aNPC.Parameter(), aPC.Parameter(), $"{theWhat} Parameter({j})", theTol);
            theO.Same(aNV, aPC.Value(), $"{theWhat} Point({j})", theTol);
            theO.Same(theN.SquareDistance(j), theP.SquareDistance(j), $"{theWhat} SquareDistance({j})", theTol);
            theO.Same(theN.IsMin(j), theP.IsMin(j), $"{theWhat} IsMin({j})");
        }
        double aND1 = 0.0, aND2 = 0.0;
        using N.gp_Pnt aNP1 = new(), aNP2 = new();
        theN.TrimmedSquareDistances(ref aND1, ref aND2, aNP1, aNP2);
        theP.TrimmedSquareDistances(out double aD1, out double aD2, out gp_Pnt aP1, out gp_Pnt aP2);
        theO.Same(aND1, aD1, $"{theWhat} trimmed distance 1");
        theO.Same(aND2, aD2, $"{theWhat} trimmed distance 2");
        theO.Same(aNP1, aP1, $"{theWhat} trimmed point 1");
        theO.Same(aNP2, aP2, $"{theWhat} trimmed point 2");
    }

    /// <summary>Extrema_ExtPC itself, on random curves (and offsets of the smooth ones) and ranges: the curve's whole
    /// range, a sub-range given to Initialize (with a random function tolerance), and a reused object performing for
    /// several points.</summary>
    [Fact]
    public void ExtPC()
    {
        var anOracle = new Oracle(output);
        int aExtrema = 0;
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom_Curve aNC, Geom_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve(anOracle);
            if ((aKind.Contains("bezier", StringComparison.Ordinal) || aKind == "circle" || aKind == "ellipse")
                && anOracle.Int(0, 2) == 0)
            {
                // An offset of a smooth curve: the general path (C2 intervals or deflection intervals)
                (N.gp_Dir aND, gp_Dir aD) = GeomOracleInputs.Dirs(anOracle);
                double anOffset = anOracle.Real(-3, 3);
                using (aND)
                using (N.Geom_Curve aNBasis = aNC)
                {
                    aNC = new N.Geom_OffsetCurve(aNBasis, anOffset, aND);
                }
                aPC = new Geom_OffsetCurve(aPC, anOffset, aD);
                aKind = "offset " + aKind;
            }
            double aTol = Oracle.Tolerance;
            using (aNC)
            {
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                using var aNA = new N.GeomAdaptor_Curve(aNC, aFirst, aLast);
                var aPA = new GeomAdaptor_Curve(aPC, aFirst, aLast);
                string aW = $"{aKind} #{c} [{aFirst:R}, {aLast:R}]";

                for (int k = 0; k < 3; k++)
                {
                    gp_Pnt aP = RandomPoint(anOracle, aPA);
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    N.Extrema_ExtPC? aN = null;
                    Extrema_ExtPC? aPort = null;
                    if (Both(anOracle, () => aN = new N.Extrema_ExtPC(aNP, aNA), () => aPort = new Extrema_ExtPC(aP, aPA),
                            $"{aW} (P, C)"))
                    {
                        using (aN)
                        {
                            SameExtPC(anOracle, aN!, aPort!, $"{aW} (P, C) {aP}", aTol);
                            aExtrema += aPort!.IsDone() ? aPort.NbExt() : 0;
                        }
                    }
                }

                // A sub-range and a function tolerance, through Initialize; the object reused for several points
                double aU1 = anOracle.Real(aFirst, aLast), aU2 = anOracle.Real(aFirst, aLast);
                if (aU1 > aU2)
                {
                    (aU1, aU2) = (aU2, aU1);
                }
                double aTolF = anOracle.Bool() ? 1.0e-10 : anOracle.Real(1e-12, 1e-6);
                using var aNR = new N.Extrema_ExtPC();
                aNR.Initialize(aNA, aU1, aU2, aTolF);
                var aPR = new Extrema_ExtPC();
                aPR.Initialize(aPA, aU1, aU2, aTolF);
                for (int k = 0; k < 3; k++)
                {
                    gp_Pnt aP = RandomPoint(anOracle, aPA);
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    if (Both(anOracle, () => aNR.Perform(aNP), () => aPR.Perform(aP), $"{aW} range [{aU1:R}, {aU2:R}]"))
                    {
                        SameExtPC(anOracle, aNR, aPR, $"{aW} range [{aU1:R}, {aU2:R}] {aP}", aTol);
                        aExtrema += aPR.IsDone() ? aPR.NbExt() : 0;
                    }
                }
            }
        }
        output.WriteLine($"{aExtrema} extrema");
        anOracle.Report("Extrema_ExtPC");
    }

    /// <summary>
    /// GeomAPI_ProjectPointOnCurve, which loads its GeomAdaptor_Curve with the curve (and the range, if given) and runs
    /// Extrema_ExtPC over the adaptor's range; the port's Extrema_ExtPC is driven the same way.
    /// </summary>
    [Fact]
    public void GeomAPI_ProjectPointOnCurve()
    {
        var anOracle = new Oracle(output, 20261029);
        int aPoints = 0;
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom_Curve aNC, Geom_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve(anOracle);
            using (aNC)
            {
                bool aWithRange = anOracle.Bool() || Precision.IsInfinite(aPC.FirstParameter())
                                  || Precision.IsInfinite(aPC.LastParameter());
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                // The port's side of GeomAPI_ProjectPointOnCurve::Init: myC.Load(Curve[, Umin, Usup]);
                // myExtPC.Initialize(myC, myC.FirstParameter(), myC.LastParameter()); myExtPC.Perform(P)
                var aMyC = aWithRange ? new GeomAdaptor_Curve(aPC, aFirst, aLast) : new GeomAdaptor_Curve(aPC);
                string aW = $"{aKind} #{c}" + (aWithRange ? $" [{aFirst:R}, {aLast:R}]" : "");
                for (int k = 0; k < 3; k++)
                {
                    gp_Pnt aP = RandomPoint(anOracle, aMyC);
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    N.GeomAPI_ProjectPointOnCurve? aN = null;
                    var aPort = new Extrema_ExtPC();
                    if (!Both(anOracle,
                            () => aN = aWithRange ? new N.GeomAPI_ProjectPointOnCurve(aNP, aNC, aFirst, aLast)
                                                  : new N.GeomAPI_ProjectPointOnCurve(aNP, aNC),
                            () =>
                            {
                                aPort.Initialize(aMyC, aMyC.FirstParameter(), aMyC.LastParameter());
                                aPort.Perform(aP);
                            }, aW))
                    {
                        continue;
                    }
                    using (aN)
                    {
                        bool isDone = aPort.IsDone() && aPort.NbExt() > 0;
                        int aNb = isDone ? aPort.NbExt() : 0;
                        anOracle.Same(aN!.NbPoints(), aNb, $"{aW} NbPoints");
                        int anIndex = 0;
                        double aMin = 0.0;
                        for (int j = 1; j <= aNb; j++)
                        {
                            using N.gp_Pnt aNPt = aN.Point(j);
                            anOracle.Same(aN.Parameter(j), aPort.Point(j).Parameter(), $"{aW} Parameter({j})");
                            anOracle.Same(aNPt, aPort.Point(j).Value(), $"{aW} Point({j})");
                            anOracle.Same(aN.Distance(j), Math.Sqrt(aPort.SquareDistance(j)), $"{aW} Distance({j})");
                            if (j == 1 || aPort.SquareDistance(j) < aMin)
                            {
                                aMin = aPort.SquareDistance(j);
                                anIndex = j;
                            }
                            aPoints++;
                        }
                        if (aNb > 0)
                        {
                            anOracle.Same(aN.LowerDistanceParameter(), aPort.Point(anIndex).Parameter(),
                                $"{aW} LowerDistanceParameter");
                            anOracle.Same(aN.LowerDistance(), Math.Sqrt(aMin), $"{aW} LowerDistance");
                        }
                    }
                }
            }
        }
        output.WriteLine($"{aPoints} projections");
        anOracle.Report("Extrema_ExtPC through GeomAPI_ProjectPointOnCurve");
    }

    /// <summary>
    /// Geom2dAPI_ProjectPointOnCurve, which loads its Geom2dAdaptor_Curve with the curve and the range (the curve's own
    /// by default) and runs Extrema_ExtPC2d(P, adaptor); the port runs its Extrema_ExtPC2d on the real
    /// Geom2dAdaptor_Curve loaded the same way.
    /// </summary>
    [Fact]
    public void Geom2dAPI_ProjectPointOnCurve()
    {
        var anOracle = new Oracle(output, 20261030);
        int aPoints = 0;
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom2d_Curve aNC, Geom2d_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve2d(anOracle);
            using (aNC)
            {
                bool aWithRange = anOracle.Bool();
                (double aFirst, double aLast) = aWithRange
                    ? TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(), aPC.LastParameter(), aPC.IsPeriodic())
                    : (aPC.FirstParameter(), aPC.LastParameter());
                var aMyC = new Geom2dAdaptor_Curve();
                aMyC.Load(aPC, aFirst, aLast);
                (double aSF, double aSL) = Precision.IsInfinite(aFirst) || Precision.IsInfinite(aLast) ? (-50.0, 50.0) : (aFirst, aLast);
                string aW = $"{aKind} #{c} [{aFirst:R}, {aLast:R}]";
                for (int k = 0; k < 3; k++)
                {
                    gp_Pnt2d aP = RandomPoint2d(anOracle, aMyC, aSF, aSL);
                    using N.gp_Pnt2d aNP = Oracle.ToNative(aP);
                    N.Geom2dAPI_ProjectPointOnCurve? aN = null;
                    Extrema_ExtPC2d? aPort = null;
                    if (!Both(anOracle,
                            () => aN = aWithRange ? new N.Geom2dAPI_ProjectPointOnCurve(aNP, aNC, aFirst, aLast)
                                                  : new N.Geom2dAPI_ProjectPointOnCurve(aNP, aNC),
                            () => aPort = new Extrema_ExtPC2d(aP, aMyC), aW))
                    {
                        continue;
                    }
                    using (aN)
                    {
                        bool isDone = aPort!.IsDone() && aPort.NbExt() > 0;
                        int aNb = isDone ? aPort.NbExt() : 0;
                        anOracle.Same(aN!.NbPoints(), aNb, $"{aW} NbPoints");
                        int anIndex = 0;
                        double aMin = 0.0;
                        for (int j = 1; j <= aNb; j++)
                        {
                            using N.gp_Pnt2d aNPt = aN.Point(j);
                            anOracle.Same(aN.Parameter(j), aPort.Point(j).Parameter(), $"{aW} Parameter({j})");
                            anOracle.Same(aNPt, aPort.Point(j).Value(), $"{aW} Point({j})");
                            anOracle.Same(aN.Distance(j), Math.Sqrt(aPort.SquareDistance(j)), $"{aW} Distance({j})");
                            if (j == 1 || aPort.SquareDistance(j) < aMin)
                            {
                                aMin = aPort.SquareDistance(j);
                                anIndex = j;
                            }
                            aPoints++;
                        }
                        if (aNb > 0)
                        {
                            anOracle.Same(aN.LowerDistanceParameter(), aPort.Point(anIndex).Parameter(),
                                $"{aW} LowerDistanceParameter");
                            anOracle.Same(aN.LowerDistance(), Math.Sqrt(aMin), $"{aW} LowerDistance");
                        }
                    }
                }
            }
        }
        output.WriteLine($"{aPoints} projections");
        anOracle.Report("Extrema_ExtPC2d through Geom2dAPI_ProjectPointOnCurve");
    }

    /// <summary>
    /// Extrema_ExtCC through BRepExtrema_DistShapeShape between two edges. For edges, BRepExtrema_ExtCC loads
    /// Extrema_ExtCC with the edges' curves on their ranges (BRepAdaptor_Curve, a GeomAdaptor_Curve here) and the
    /// tolerances max(Resolution(min(edge tolerance, Confusion)), PConfusion), curve 2 first; the native solutions in the
    /// interior of both edges are Extrema_ExtCC's extrema, copied unchanged. Each one must be one of the port's
    /// extrema: parameters, points and distance. Solutions OCCT finds otherwise (at vertices, and by PERFORM_C0's
    /// projections when a curve is C0) are not compared.
    /// </summary>
    [Fact]
    public void ExtCC_through_BRepExtrema_DistShapeShape()
    {
        var anOracle = new Oracle(output, 20261031);
        int aCompared = 0, aSkipped = 0, aCases = 0;
        for (int c = 0; c < Cases / 2; c++)
        {
            (N.Geom_Curve aNC1, Geom_Curve aPC1, string aKind1) = TestAdaptorOracleTests.RandomCurve(anOracle);
            (N.Geom_Curve aNC2, Geom_Curve aPC2, string aKind2) = TestAdaptorOracleTests.RandomCurve(anOracle);
            using (aNC1)
            using (aNC2)
            {
                (double aF1, double aL1) = SubRange(anOracle, aPC1);
                (double aF2, double aL2) = SubRange(anOracle, aPC2);
                if (anOracle.Int(0, 2) != 0)
                {
                    // Move curve 2 near curve 1 (both sides translated by the same vector), so that the edges come
                    // close inside their ranges or cross
                    gp_Pnt aOn1 = aPC1.Value(anOracle.Real(aF1, aL1)), aOn2 = aPC2.Value(anOracle.Real(aF2, aL2));
                    double aGap = anOracle.Bool() ? 0.0 : anOracle.Real(0.01, 2.0);
                    gp_Vec aV = new(aOn2, aOn1);
                    aV += new gp_Vec(aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1));
                    using N.gp_Vec aNV = Oracle.ToNative(aV);
                    aNC2.Translate(aNV);
                    aPC2.Translate(aV);
                }
                string aW = $"{aKind1} [{aF1:R}, {aL1:R}] / {aKind2} [{aF2:R}, {aL2:R}] #{c}";

                using var aNME1 = new N.BRepBuilderAPI_MakeEdge(aNC1, aF1, aL1);
                using var aNME2 = new N.BRepBuilderAPI_MakeEdge(aNC2, aF2, aL2);
                if (!aNME1.IsDone() || !aNME2.IsDone())
                {
                    continue;
                }
                using N.TopoDS_Edge aNE1 = aNME1.Edge();
                using N.TopoDS_Edge aNE2 = aNME2.Edge();
                using var aN = new N.BRepExtrema_DistShapeShape(aNE1, aNE2);

                // The port's side of BRepExtrema_ExtCC(E1, E2): Initialize(E2), then Perform(E1)
                var aC1 = new GeomAdaptor_Curve(aPC1, aF1, aL1);
                var aC2 = new GeomAdaptor_Curve(aPC2, aF2, aL2);
                var aPort = new Extrema_ExtCC();
                double aTol2 = Math.Max(aC2.Resolution(Math.Min(Precision.Confusion(), Precision.Confusion())), Precision.PConfusion());
                aPort.SetCurve(2, aC2, aF2, aL2);
                aPort.SetTolerance(2, aTol2);
                double aTol1 = Math.Max(aC1.Resolution(Math.Min(Precision.Confusion(), Precision.Confusion())), Precision.PConfusion());
                aPort.SetCurve(1, aC1, aF1, aL1);
                aPort.SetTolerance(1, aTol1);
                aPort.Perform();
                aCases++;

                if (!aN.IsDone())
                {
                    continue;
                }
                int aNbPort = aPort.IsDone() ? (aPort.IsParallel() ? 0 : aPort.NbExt()) : 0;
                for (int i = 1; i <= aN.NbSolution(); i++)
                {
                    if (aN.SupportTypeShape1(i) != N.BRepExtrema_SupportType.BRepExtrema_IsOnEdge
                        || aN.SupportTypeShape2(i) != N.BRepExtrema_SupportType.BRepExtrema_IsOnEdge)
                    {
                        continue;
                    }
                    double aT1 = 0.0, aT2 = 0.0;
                    aN.ParOnEdgeS1(i, ref aT1);
                    aN.ParOnEdgeS2(i, ref aT2);
                    // DistanceSS drops Extrema_ExtCC's solutions at the vertices (isOnBoundary): one there comes
                    // from PERFORM_C0, at the bound of a C1 interval
                    double aEps = Precision.PConfusion();
                    if (Math.Abs(aT1 - aF1) < aEps || Math.Abs(aT1 - aL1) < aEps || Math.Abs(aT2 - aF2) < aEps
                        || Math.Abs(aT2 - aL2) < aEps)
                    {
                        aSkipped++;
                        continue;
                    }
                    // The port's extremum with these parameters
                    int aMatch = 0;
                    double aBest = double.MaxValue;
                    for (int j = 1; j <= aNbPort; j++)
                    {
                        aPort.Points(j, out Extrema_POnCurv aP1, out Extrema_POnCurv aP2);
                        double aD = Math.Abs(aP1.Parameter() - aT1) + Math.Abs(aP2.Parameter() - aT2);
                        if (aD < aBest)
                        {
                            aBest = aD;
                            aMatch = j;
                        }
                    }
                    if (aMatch == 0 || aBest > 1e-6)
                    {
                        // Not an Extrema_ExtCC solution: PERFORM_C0's projections of the C1 interval bounds (the ends
                        // included) of an edge whose whole curve is C0
                        Assert.True(aPC1.Continuity() == GeomAbs_C0 || aPC2.Continuity() == GeomAbs_C0,
                            $"{aW}: native solution ({aT1:R}, {aT2:R}) is not an extremum of the port's Extrema_ExtCC");
                        aSkipped++;
                        continue;
                    }
                    aPort.Points(aMatch, out Extrema_POnCurv aQ1, out Extrema_POnCurv aQ2);
                    using N.gp_Pnt aNP1 = aN.PointOnShape1(i);
                    using N.gp_Pnt aNP2 = aN.PointOnShape2(i);
                    anOracle.Same(aT1, aQ1.Parameter(), $"{aW} parameter on 1");
                    anOracle.Same(aT2, aQ2.Parameter(), $"{aW} parameter on 2");
                    anOracle.Same(aNP1, aQ1.Value(), $"{aW} point on 1");
                    anOracle.Same(aNP2, aQ2.Value(), $"{aW} point on 2");
                    // DistShapeShape's Value() is the least distance of all its solutions; the solution's own is the
                    // distance between its points
                    anOracle.Same(Oracle.FromNative(aNP1).SquareDistance(Oracle.FromNative(aNP2)),
                        aPort.SquareDistance(aMatch), $"{aW} square distance");
                    Assert.True(Math.Sqrt(aPort.SquareDistance(aMatch)) - aN.Value() < 1e-7, $"{aW} distance");
                    aCompared++;
                }
            }
        }
        output.WriteLine($"{aCases} edge pairs, {aCompared} interior solutions compared, {aSkipped} from vertices or PERFORM_C0 skipped");
        Assert.True(aCompared > aCases / 3, "too few solutions compared");
        anOracle.Report("Extrema_ExtCC through BRepExtrema_DistShapeShape");
    }

    // A range inside the curve's own (whole when finite and not periodic, half the time), so BRepLib_MakeEdge keeps it.
    private static (double First, double Last) SubRange(Oracle theO, Geom_Curve theC)
    {
        double aFirst = theC.FirstParameter(), aLast = theC.LastParameter();
        if (Precision.IsInfinite(aFirst) || Precision.IsInfinite(aLast))
        {
            double a = theO.Real(-5, 0), b = theO.Real(0.5, 5);
            return (a, a + b);
        }
        if (!theC.IsPeriodic() && theO.Bool())
        {
            return (aFirst, aLast);
        }
        double u = theO.Real(aFirst, aLast), v = theO.Real(aFirst, aLast);
        if (Math.Abs(u - v) < 1e-3 * (aLast - aFirst))
        {
            return (aFirst, aFirst + 0.5 * (aLast - aFirst));
        }
        return u < v ? (u, v) : (v, u);
    }
}
#endif
