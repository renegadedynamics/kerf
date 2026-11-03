// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Xunit.Abstractions;
using static Occt.Approx_ParametrizationType;
using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The GeomAPI classes of unit 3F against native OCCT, on inputs built from the same raw numbers on both sides:
/// GeomAPI_ProjectPointOnCurve and GeomAPI_ProjectPointOnSurf through their load-then-perform path (the
/// constructors and point Init are compared in ExtremaCurveOracleTests and ExtremaSurfaceOracleTests),
/// GeomAPI_Interpolate (periodic or not, chord-length or given parameters, no tangents, end tangents or tangents at
/// some points, scaled or not) and GeomAPI_PointsToBSpline (its three methods).
/// </summary>
public class GeomAPIOracleTests(ITestOutputHelper output)
{
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
        catch (global::Kerf.Engine.Foundation.FailureException anE)
        {
            aPortError = anE;
        }
        theO.Same(aNativeError is not null, aPortError is not null,
            $"{theWhat} throws (native: {aNativeError?.Message}; port: {aPortError?.Message})");
        return aNativeError is null && aPortError is null;
    }

    private static void SameCurve(Oracle theO, N.Geom_BSplineCurve theNative, Geom_BSplineCurve thePort, string theWhat,
                                  double theTolerance = Oracle.Tolerance)
    {
        theO.Same(theNative.Degree(), thePort.Degree(), $"{theWhat} degree");
        theO.Same(theNative.IsPeriodic(), thePort.IsPeriodic(), $"{theWhat} periodic");
        theO.Same(theNative.NbPoles(), thePort.NbPoles(), $"{theWhat} NbPoles");
        theO.Same(theNative.NbKnots(), thePort.NbKnots(), $"{theWhat} NbKnots");
        for (int i = 1; i <= thePort.NbPoles(); i++)
        {
            using N.gp_Pnt aNP = theNative.Pole(i);
            theO.Same(aNP, thePort.Pole(i), $"{theWhat} pole {i}", theTolerance);
        }
        for (int i = 1; i <= thePort.NbKnots(); i++)
        {
            theO.Same(theNative.Knot(i), thePort.Knot(i), $"{theWhat} knot {i}", theTolerance);
            theO.Same(theNative.Multiplicity(i), thePort.Multiplicity(i), $"{theWhat} mult {i}");
        }
    }

    /// <summary>
    /// GeomAPI_ProjectPointOnCurve: Init(curve, range) once, then Perform for several points, every extremum and the
    /// nearest one compared.
    /// </summary>
    [Fact]
    public void ProjectPointOnCurve_InitThenPerform()
    {
        var anOracle = new Oracle(output, 20261201);
        int aPoints = 0;
        for (int c = 0; c < 400; c++)
        {
            (N.Geom_Curve aNC, Geom_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                string aW = $"{aKind} #{c} [{aFirst:R}, {aLast:R}]";
                using var aN = new N.GeomAPI_ProjectPointOnCurve();
                var aPort = new GeomAPI_ProjectPointOnCurve();
                if (!Both(anOracle, () => aN.Init(aNC, aFirst, aLast), () => aPort.Init(aPC, aFirst, aLast), aW))
                {
                    continue;
                }
                anOracle.Same(aN.NbPoints(), aPort.NbPoints(), $"{aW} NbPoints after Init");
                var aRange = new GeomAdaptor_Curve(aPC, aFirst, aLast);
                for (int k = 0; k < 3; k++)
                {
                    Point3D aP = k == 0 ? anOracle.Pnt()
                                       : aRange.Value(anOracle.Real(aFirst, aLast)).Translated(anOracle.Real(0, 0.2) * anOracle.Vec());
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    string aWP = $"{aW} point {k}";
                    if (!Both(anOracle, () => aN.Perform(aNP), () => aPort.Perform(aP), aWP))
                    {
                        break;
                    }
                    int aNb = aPort.NbPoints();
                    anOracle.Same(aN.NbPoints(), aNb, $"{aWP} NbPoints");
                    anOracle.Same(aN.NbPoints(), (int)aPort, $"{aWP} operator int");
                    for (int j = 1; j <= aNb; j++)
                    {
                        using N.gp_Pnt aNPt = aN.Point(j);
                        anOracle.Same(aNPt, aPort.Point(j), $"{aWP} Point({j})");
                        double aNU = 0.0;
                        aN.Parameter(j, ref aNU);
                        aPort.Parameter(j, out double aU);
                        anOracle.Same(aNU, aU, $"{aWP} Parameter({j})");
                        anOracle.Same(aN.Distance(j), aPort.Distance(j), $"{aWP} Distance({j})");
                        aPoints++;
                    }
                    if (aNb > 0)
                    {
                        using N.gp_Pnt aNNearest = aN.NearestPoint();
                        anOracle.Same(aNNearest, aPort.NearestPoint(), $"{aWP} NearestPoint");
                        anOracle.Same(aN.LowerDistanceParameter(), aPort.LowerDistanceParameter(),
                            $"{aWP} LowerDistanceParameter");
                        anOracle.Same(aN.LowerDistance(), (double)aPort, $"{aWP} LowerDistance");
                    }
                }
            }
        }
        output.WriteLine($"{aPoints} projections");
        anOracle.Report("GeomAPI_ProjectPointOnCurve Init + Perform");
    }

    /// <summary>
    /// GeomAPI_ProjectPointOnSurf: Init(surface, range[, tolerance], algorithm) once (no point), then Perform for
    /// several points, with a flag set before the first Init and kept; every extremum and the nearest one compared.
    /// </summary>
    [Fact]
    public void ProjectPointOnSurf_InitThenPerform()
    {
        var anOracle = new Oracle(output, 20261202);
        int anExtrema = 0;
        int[] aByKind = new int[AdaptorOracleInputs.SurfaceKinds.Length];
        for (int c = 0; c < 550; c++)
        {
            int aKind = c % AdaptorOracleInputs.SurfaceKinds.Length;
            (N.Geom_Surface aNS, Geom_Surface aPS) = AdaptorOracleInputs.RandomSurface(anOracle, aKind);
            using (aNS)
            {
                aPS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                (double aWU1, double aWU2) = Window(anOracle, aU1, aU2);
                (double aWV1, double aWV2) = Window(anOracle, aV1, aV2);
                bool aWithTol = anOracle.Bool();
                double aTol = Math.Pow(10, anOracle.Real(-9, -4));
                Extrema_ExtAlgo anAlgo = anOracle.Int(0, 3) == 0 ? Extrema_ExtAlgo_Tree : Extrema_ExtAlgo_Grad;
                Extrema_ExtFlag aFlag = anOracle.Int(0, 3) switch
                {
                    0 => Extrema_ExtFlag_MIN,
                    1 => Extrema_ExtFlag_MAX,
                    _ => Extrema_ExtFlag_MINMAX,
                };
                string aW = $"{AdaptorOracleInputs.SurfaceKinds[aKind]} #{c} ({anAlgo}, {aFlag}, {(aWithTol ? aTol.ToString("R") : "default tol")})";

                using var aN = new N.GeomAPI_ProjectPointOnSurf();
                var aPort = new GeomAPI_ProjectPointOnSurf();
                aN.SetExtremaFlag((N.Extrema_ExtFlag)(int)aFlag);
                aPort.SetExtremaFlag(aFlag);
                var aNAlgo = (N.Extrema_ExtAlgo)(int)anAlgo;
                if (!Both(anOracle,
                        () =>
                        {
                            if (aWithTol)
                            {
                                aN.Init(aNS, aWU1, aWU2, aWV1, aWV2, aTol, aNAlgo);
                            }
                            else
                            {
                                aN.Init(aNS, aWU1, aWU2, aWV1, aWV2, aNAlgo);
                            }
                        },
                        () =>
                        {
                            if (aWithTol)
                            {
                                aPort.Init(aPS, aWU1, aWU2, aWV1, aWV2, aTol, anAlgo);
                            }
                            else
                            {
                                aPort.Init(aPS, aWU1, aWU2, aWV1, aWV2, anAlgo);
                            }
                        }, aW))
                {
                    continue;
                }
                anOracle.Same(aN.IsDone(), aPort.IsDone(), $"{aW} IsDone after Init");
                for (int k = 0; k < 3; k++)
                {
                    Point3D aP = k == 0 ? new Point3D(anOracle.Real(-30, 30), anOracle.Real(-30, 30), anOracle.Real(-30, 30))
                                       : aPS.Value(anOracle.Real(aWU1, aWU2), anOracle.Real(aWV1, aWV2))
                                            .Translated(anOracle.Real(0, 0.05) * anOracle.Vec());
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    string aWP = $"{aW} point {k} {aP}";
                    if (!Both(anOracle, () => aN.Perform(aNP), () => aPort.Perform(aP), aWP))
                    {
                        break;
                    }
                    anOracle.Same(aN.IsDone(), aPort.IsDone(), $"{aWP} IsDone");
                    int aNb = aN.NbPoints();
                    anOracle.Same(aNb, aPort.NbPoints(), $"{aWP} NbPoints");
                    for (int i = 1; i <= aNb; i++)
                    {
                        double aNU = 0.0, aNV = 0.0;
                        aN.Parameters(i, ref aNU, ref aNV);
                        aPort.Parameters(i, out double aU, out double aV);
                        anOracle.Same(aNU, aU, $"{aWP} extremum {i} U");
                        anOracle.Same(aNV, aV, $"{aWP} extremum {i} V");
                        using N.gp_Pnt aNPi = aN.Point(i);
                        anOracle.Same(aNPi, aPort.Point(i), $"{aWP} extremum {i} point");
                        anOracle.Same(aN.Distance(i), aPort.Distance(i), $"{aWP} extremum {i} distance");
                        anExtrema++;
                        aByKind[aKind]++;
                    }
                    if (aNb > 0)
                    {
                        double aNU = 0.0, aNV = 0.0;
                        aN.LowerDistanceParameters(ref aNU, ref aNV);
                        aPort.LowerDistanceParameters(out double aU, out double aV);
                        anOracle.Same(aNU, aU, $"{aWP} LowerDistanceParameters U");
                        anOracle.Same(aNV, aV, $"{aWP} LowerDistanceParameters V");
                        using N.gp_Pnt aNNearest = aN.NearestPoint();
                        anOracle.Same(aNNearest, (Point3D)aPort, $"{aWP} NearestPoint");
                        anOracle.Same(aN.LowerDistance(), aPort.LowerDistance(), $"{aWP} LowerDistance");
                    }
                }
            }
        }
        output.WriteLine($"{anExtrema} extrema compared");
        for (int k = 0; k < aByKind.Length; k++)
        {
            output.WriteLine($"  {AdaptorOracleInputs.SurfaceKinds[k]}: {aByKind[k]} extrema");
            Assert.True(aByKind[k] > 0, $"no extremum compared on {AdaptorOracleInputs.SurfaceKinds[k]}");
        }
        anOracle.Report("GeomAPI_ProjectPointOnSurf Init + Perform");
    }

    // A finite sub-range of [theFirst, theLast] (a random window where a bound is infinite).
    private static (double, double) Window(Oracle theO, double theFirst, double theLast)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double a = theO.Real(-10, 5);
            return (a, a + theO.Real(1, 12));
        }
        if (theO.Bool())
        {
            return (theFirst, theLast);
        }
        double aD = theLast - theFirst;
        double u = theO.Real(theFirst, theFirst + 0.45 * aD);
        return (u, theO.Real(theFirst + 0.55 * aD, theLast));
    }

    /// <summary>
    /// GeomAPI_Interpolate on random point sets: periodic or not, chord-length or random increasing parameters, no
    /// tangents, the end tangents, or tangents at random points (scaled or not). The poles, knots and multiplicities
    /// of the curves are compared, and failures (close points, bad parameters) must match.
    /// </summary>
    [Fact]
    public void Interpolate()
    {
        var anOracle = new Oracle(output, 20261203);
        int aDone = 0;
        for (int c = 0; c < 600; c++)
        {
            NCollection_Array1<Point3D> aPoints = AppDefOracleTests.Points(anOracle);
            int aNb = aPoints.Length();
            bool aPeriodic = anOracle.Int(0, 2) == 0;
            bool aWithParams = anOracle.Bool();
            int aTangents = anOracle.Int(0, 2); // none, the end tangents, some points
            bool aScale = anOracle.Bool();
            double aTol = anOracle.Bool() ? global::Kerf.Engine.Foundation.Precision.Confusion() : 1.0e-3;
            string aW = $"#{c} ({aNb} points, {(aPeriodic ? "periodic" : "open")}, {(aWithParams ? "parameters" : "chord")}, tangents {aTangents}, scale {aScale})";

            NCollection_HArray1<Point3D> aPnts = new(1, aNb);
            using var aNPnts = new N.TColgp_HArray1OfPnt(1, aNb);
            for (int i = 1; i <= aNb; i++)
            {
                aPnts.SetValue(i, aPoints[i]);
                using N.gp_Pnt aNP = Oracle.ToNative(aPoints[i]);
                aNPnts.SetValue(i, aNP);
            }
            int aNbParams = aPeriodic ? aNb + 1 : aNb;
            NCollection_HArray1<double> aParams = new(1, aNbParams);
            using var aNParams = new N.TColStd_HArray1OfReal(1, aNbParams);
            double u = anOracle.Real(-5, 5);
            for (int i = 1; i <= aNbParams; i++)
            {
                aParams.SetValue(i, u);
                aNParams.SetValue(i, u);
                u += anOracle.Real(0.05, 2.0);
            }

            N.GeomAPI_Interpolate? aN = null;
            GeomAPI_Interpolate? aPort = null;
            if (!Both(anOracle,
                    () => aN = aWithParams ? new N.GeomAPI_Interpolate(aNPnts, aNParams, aPeriodic, aTol)
                                           : new N.GeomAPI_Interpolate(aNPnts, aPeriodic, aTol),
                    () => aPort = aWithParams ? new GeomAPI_Interpolate(aPnts, aParams, aPeriodic, aTol)
                                              : new GeomAPI_Interpolate(aPnts, aPeriodic, aTol),
                    aW))
            {
                aN?.Dispose();
                continue;
            }
            using (aN)
            {
                if (aTangents == 1)
                {
                    Vector3D aT1 = anOracle.Vec(), aT2 = anOracle.Vec();
                    using N.gp_Vec aNT1 = Oracle.ToNative(aT1);
                    using N.gp_Vec aNT2 = Oracle.ToNative(aT2);
                    if (!Both(anOracle, () => aN!.Load(aNT1, aNT2, aScale), () => aPort!.Load(aT1, aT2, aScale),
                            $"{aW} Load ends"))
                    {
                        continue;
                    }
                }
                else if (aTangents == 2)
                {
                    NCollection_Array1<Vector3D> aTs = new(1, aNb);
                    NCollection_HArray1<bool> aFlags = new(1, aNb);
                    using var aNTs = new N.TColgp_Array1OfVec(1, aNb);
                    using var aNFlags = new N.TColStd_HArray1OfBoolean(1, aNb);
                    for (int i = 1; i <= aNb; i++)
                    {
                        aTs[i] = anOracle.Vec();
                        aFlags[i] = anOracle.Int(0, 2) == 0;
                        using N.gp_Vec aNT = Oracle.ToNative(aTs[i]);
                        aNTs.SetValue(i, aNT);
                        aNFlags.SetValue(i, aFlags[i]);
                    }
                    if (!Both(anOracle, () => aN!.Load(aNTs, aNFlags, aScale), () => aPort!.Load(aTs, aFlags, aScale),
                            $"{aW} Load tangents"))
                    {
                        continue;
                    }
                }
                if (!Both(anOracle, () => aN!.Perform(), () => aPort!.Perform(), $"{aW} Perform"))
                {
                    continue;
                }
                anOracle.Same(aN!.IsDone(), aPort!.IsDone(), $"{aW} IsDone");
                if (!aPort.IsDone())
                {
                    continue;
                }
                using N.Geom_BSplineCurve? aNCurve = aN.Curve();
                Assert.NotNull(aNCurve);
                SameCurve(anOracle, aNCurve, aPort.Curve(), aW);
                aDone++;
            }
        }
        output.WriteLine($"{aDone} curves");
        anOracle.Report("GeomAPI_Interpolate");
    }

    /// <summary>
    /// GeomAPI_PointsToBSpline's three methods (parametrisation type, given parameters, variational), on AppDef's
    /// oracle inputs. AppDefOracleTests mirrors the same paths around AppDef; here the port's own class runs them.
    /// </summary>
    [Fact]
    public void PointsToBSpline()
    {
        var anOracle = new Oracle(output, 20261204);
        int aDone = 0;
        for (int c = 0; c < 450; c++)
        {
            NCollection_Array1<Point3D> aPoints = AppDefOracleTests.Points(anOracle);
            int aMethod = c % 3;
            using N.TColgp_Array1OfPnt aNPoints = Oracle.ToNative(aPoints);
            N.GeomAPI_PointsToBSpline? aN = null;
            GeomAPI_PointsToBSpline? aPort = null;
            string aW;
            double aTolerance;
            if (aMethod == 0)
            {
                Approx_ParametrizationType aParType = anOracle.Int(0, 2) switch
                {
                    0 => Approx_ChordLength,
                    1 => Approx_Centripetal,
                    _ => Approx_IsoParametric,
                };
                int aDegMin = anOracle.Int(1, 4);
                int aDegMax = anOracle.Int(aDegMin, 8);
                Continuity aCont = AppDefOracleTests.Continuity(anOracle);
                double aTol3D = AppDefOracleTests.Tolerance(anOracle);
                aW = $"#{c} {aParType} {aDegMin}-{aDegMax} {aCont} {aTol3D}";
                // As in AppDefOracleTests: squares is a direct solve, the gradient iterates and amplifies rounding
                aTolerance = aTol3D <= 1.0e-3 ? 1.0e-9 : 1.0e-6;
                if (!Both(anOracle,
                        () => aN = new N.GeomAPI_PointsToBSpline(aNPoints, (N.Approx_ParametrizationType)(int)aParType,
                                                                 aDegMin, aDegMax, (N.GeomAbs_Shape)(int)aCont, aTol3D),
                        () => aPort = new GeomAPI_PointsToBSpline(aPoints, aParType, aDegMin, aDegMax, aCont, aTol3D), aW))
                {
                    aN?.Dispose();
                    continue;
                }
            }
            else if (aMethod == 1)
            {
                int aNbp = aPoints.Length();
                NCollection_Array1<double> aParams = new(1, aNbp);
                double u = anOracle.Real(-5, 5);
                for (int i = 1; i <= aNbp; i++)
                {
                    aParams[i] = u;
                    u += anOracle.Real(0.01, 2.0);
                }
                int aDegMin = anOracle.Int(1, 4);
                int aDegMax = anOracle.Int(aDegMin, 8);
                Continuity aCont = AppDefOracleTests.Continuity(anOracle);
                double aTol3D = AppDefOracleTests.Tolerance(anOracle);
                aW = $"#{c} parameters {aDegMin}-{aDegMax} {aCont} {aTol3D}";
                aTolerance = 1.0e-9;
                using N.TColStd_Array1OfReal aNParams = Oracle.ToNative(aParams);
                if (!Both(anOracle,
                        () => aN = new N.GeomAPI_PointsToBSpline(aNPoints, aNParams, aDegMin, aDegMax,
                                                                 (N.GeomAbs_Shape)(int)aCont, aTol3D),
                        () => aPort = new GeomAPI_PointsToBSpline(aPoints, aParams, aDegMin, aDegMax, aCont, aTol3D), aW))
                {
                    aN?.Dispose();
                    continue;
                }
            }
            else
            {
                if (aPoints.Length() < 3)
                {
                    continue; // OCCT reads past its points for two points (occt-issues.md)
                }
                double W1 = anOracle.Real(0, 1), W2 = anOracle.Real(0, 1), W3 = anOracle.Real(0, 1);
                if (W1 + W2 + W3 < 0.01)
                {
                    W1 = 1.0;
                }
                int aDegMax = anOracle.Int(3, 8);
                // Not C0: native results change from run to run there (occt-issues.md, AppDefOracleTests)
                Continuity aCont = anOracle.Bool() ? Continuity.C1 : Continuity.C2;
                double aTol3D = AppDefOracleTests.Tolerance(anOracle);
                aW = $"#{c} variational {W1:R} {W2:R} {W3:R} {aDegMax} {aCont} {aTol3D}";
                aTolerance = 1.0e-6;
                if (!Both(anOracle,
                        () =>
                        {
                            lock (Oracle.NativeVariational)
                            {
                                aN = new N.GeomAPI_PointsToBSpline(aNPoints, W1, W2, W3, aDegMax, (N.GeomAbs_Shape)(int)aCont, aTol3D);
                            }
                        },
                        () => aPort = new GeomAPI_PointsToBSpline(aPoints, W1, W2, W3, aDegMax, aCont, aTol3D), aW))
                {
                    aN?.Dispose();
                    continue;
                }
            }
            using (aN)
            {
                anOracle.Same(aN!.IsDone(), aPort!.IsDone(), $"{aW} IsDone");
                if (!aPort.IsDone())
                {
                    continue;
                }
                using N.Geom_BSplineCurve? aNCurve = aN.Curve();
                Assert.NotNull(aNCurve);
                SameCurve(anOracle, aNCurve, aPort.Curve(), aW, aTolerance);
                aDone++;
            }
        }
        output.WriteLine($"{aDone} curves");
        anOracle.Report("GeomAPI_PointsToBSpline");
    }
}
