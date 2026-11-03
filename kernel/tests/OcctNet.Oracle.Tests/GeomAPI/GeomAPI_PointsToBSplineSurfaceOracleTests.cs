// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Occt.Approx_ParametrizationType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomAPI_PointsToBSplineSurface (unit 5H) against native OCCT on random grids built from the same raw numbers:
/// every method (least squares with each parametrisation, interpolation, periodic interpolation and approximation,
/// the variational method, Z grids approximated and interpolated), every continuity, degrees and tolerances, through
/// the constructors and <c>Init</c>/<c>Interpolate</c> on a reused object. The surface's degrees, periodicity, poles,
/// knots and multiplicities are compared.
/// </summary>
public class GeomAPI_PointsToBSplineSurfaceOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(700, 125);

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
        catch (FailureException anE)
        {
            aPortError = anE;
        }
        theO.Same(aNativeError is not null, aPortError is not null,
            $"{theWhat} throws (native: {aNativeError?.Message}; port: {aPortError?.Message})");
        return aNativeError is null && aPortError is null;
    }

    private static void SameSurface(Oracle theO, N.Geom_BSplineSurface theNative, Geom_BSplineSurface thePort,
                                    string theWhat, double theTolerance)
    {
        theO.Same(theNative.UDegree(), thePort.UDegree(), $"{theWhat} UDegree");
        theO.Same(theNative.VDegree(), thePort.VDegree(), $"{theWhat} VDegree");
        theO.Same(theNative.IsUPeriodic(), thePort.IsUPeriodic(), $"{theWhat} UPeriodic");
        theO.Same(theNative.IsVPeriodic(), thePort.IsVPeriodic(), $"{theWhat} VPeriodic");
        theO.Same(theNative.NbUPoles(), thePort.NbUPoles(), $"{theWhat} NbUPoles");
        theO.Same(theNative.NbVPoles(), thePort.NbVPoles(), $"{theWhat} NbVPoles");
        theO.Same(theNative.NbUKnots(), thePort.NbUKnots(), $"{theWhat} NbUKnots");
        theO.Same(theNative.NbVKnots(), thePort.NbVKnots(), $"{theWhat} NbVKnots");
        for (int i = 1; i <= thePort.NbUPoles(); i++)
        {
            for (int j = 1; j <= thePort.NbVPoles(); j++)
            {
                using N.gp_Pnt aNP = theNative.Pole(i, j);
                theO.Same(aNP, thePort.Pole(i, j), $"{theWhat} pole {i},{j}", theTolerance);
            }
        }
        for (int i = 1; i <= thePort.NbUKnots(); i++)
        {
            theO.Same(theNative.UKnot(i), thePort.UKnot(i), $"{theWhat} uknot {i}", theTolerance);
            theO.Same(theNative.UMultiplicity(i), thePort.UMultiplicity(i), $"{theWhat} umult {i}");
        }
        for (int j = 1; j <= thePort.NbVKnots(); j++)
        {
            theO.Same(theNative.VKnot(j), thePort.VKnot(j), $"{theWhat} vknot {j}", theTolerance);
            theO.Same(theNative.VMultiplicity(j), thePort.VMultiplicity(j), $"{theWhat} vmult {j}");
        }
    }

    // A grid of NbU x NbV points, indexed from 1 (GeomAPI_PointsToBSplineSurface requires it): random points, or
    // samples of a random smooth surface (polynomial, wave, cylinder-like), on uneven parameters, with or without
    // noise.
    internal static NCollection_Array2<Point3D> Grid(Oracle theO, int theNbU, int theNbV, bool theClosedInU = false)
    {
        int aKind = theO.Int(0, 3);
        double aNoise = theO.Bool() ? 0.0 : theO.Real(0.0, 0.3);
        double[] aC = new double[12];
        for (int k = 0; k < aC.Length; k++)
        {
            aC[k] = theO.Real(-3, 3);
        }
        double[] aU = Steps(theO, theNbU), aV = Steps(theO, theNbV);
        NCollection_Array2<Point3D> aPoints = new(1, theNbU, 1, theNbV);
        for (int i = 1; i <= theNbU; i++)
        {
            for (int j = 1; j <= theNbV; j++)
            {
                double u = aU[i - 1], v = aV[j - 1];
                Point3D aP;
                if (theClosedInU)
                {
                    // rings around an axis: the grid closes up in U (the first index)
                    double a = 2 * Math.PI * (i - 1) / theNbU, r = 5 + aC[0] * v + aC[1] * Math.Sin(a);
                    aP = new Point3D(r * Math.Cos(a), r * Math.Sin(a), 10 * v + aC[2] * Math.Cos(2 * a));
                }
                else
                {
                    aP = aKind switch
                    {
                        0 => new Point3D(theO.Real(-10, 10), theO.Real(-10, 10), theO.Real(-10, 10)),
                        1 => new Point3D(10 * u, 10 * v,
                                        aC[0] + aC[1] * u + aC[2] * v + aC[3] * u * v + aC[4] * u * u * v + aC[5] * v * v),
                        2 => new Point3D(10 * u + aC[6] * v, 10 * v, aC[7] * Math.Sin(3 * u + aC[8]) * Math.Cos(2 * v)),
                        _ => new Point3D(5 * Math.Cos(aC[9] + 2 * u), 5 * Math.Sin(aC[9] + 2 * u), 10 * v + aC[10] * u),
                    };
                }
                if (aNoise > 0.0)
                {
                    aP = new Point3D(aP.X() + theO.Real(-aNoise, aNoise), aP.Y() + theO.Real(-aNoise, aNoise),
                                    aP.Z() + theO.Real(-aNoise, aNoise));
                }
                aPoints[i, j] = aP;
            }
        }
        return aPoints;
    }

    // Increasing parameters in [0, 1], unevenly spaced.
    private static double[] Steps(Oracle theO, int theNb)
    {
        double[] aT = new double[theNb];
        double t = 0.0;
        for (int i = 0; i < theNb; i++)
        {
            aT[i] = t;
            t += theO.Real(0.2, 1.0) / theNb;
        }
        return aT;
    }

    private static Approx_ParametrizationType ParType(Oracle theO) => theO.Int(0, 2) switch
    {
        0 => Approx_ChordLength,
        1 => Approx_Centripetal,
        _ => Approx_IsoParametric,
    };

    [Fact]
    public void PointsToBSplineSurface()
    {
        var anOracle = new Oracle(output, 20261205);
        int aDone = 0, aNotDone = 0, aThrown = 0;
        int[] aByMethod = new int[7];
        GeomAPI_PointsToBSplineSurface aPortReused = new();
        using N.GeomAPI_PointsToBSplineSurface aNativeReused = new();
        for (int c = 0; c < Cases; c++)
        {
            int aMethod = c % 7;
            bool aReuse = anOracle.Bool();
            int aNbU = anOracle.Int(2, 12), aNbV = anOracle.Int(2, 12);
            Continuity aCont = AppDefOracleTests.Continuity(anOracle);
            double aTol3D = AppDefOracleTests.Tolerance(anOracle);
            int aDegMin = anOracle.Int(1, 4);
            int aDegMax = anOracle.Int(Math.Max(aDegMin, 2), 8);
            Approx_ParametrizationType aParType = ParType(anOracle);
            N.GeomAbs_Shape aNCont = (N.GeomAbs_Shape)(int)aCont;
            N.Approx_ParametrizationType aNParType = (N.Approx_ParametrizationType)(int)aParType;

            N.GeomAPI_PointsToBSplineSurface? aNFresh = null;
            N.GeomAPI_PointsToBSplineSurface aN = aNativeReused;
            GeomAPI_PointsToBSplineSurface aPort = aPortReused;
            string aW;
            double aTolerance = aTol3D <= 1.0e-3 ? 1.0e-9 : 1.0e-6;
            bool aOk;
            NCollection_Array2<Point3D>? aPoints = null;
            NCollection_Array2<double>? aZ = null;
            if (aMethod <= 4)
            {
                aPoints = Grid(anOracle, aNbU, aNbV, aMethod == 3 || aMethod == 4);
            }
            else
            {
                aZ = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
                double a = anOracle.Real(-2, 2), b = anOracle.Real(-2, 2);
                bool aRandom = anOracle.Bool();
                for (int i = 1; i <= aNbU; i++)
                {
                    for (int j = 1; j <= aNbV; j++)
                    {
                        aZ[i, j] = aRandom ? anOracle.Real(-5, 5) : a * Math.Sin(0.7 * i) + b * Math.Cos(0.5 * j) + 0.1 * i * j;
                    }
                }
            }
            using N.TColgp_Array2OfPnt? aNPoints = aPoints is null ? null : Oracle.ToNative(aPoints);
            using N.TColStd_Array2OfReal? aNZ = aZ is null ? null : Oracle.ToNative(aZ);
            double X0 = anOracle.Real(-10, 10), dX = anOracle.Real(0.1, 2), Y0 = anOracle.Real(-10, 10),
                   dY = anOracle.Real(0.1, 2);

            switch (aMethod)
            {
                case 0: // least squares, parametrisation type (constructor or Init)
                    aW = $"#{c} {aNbU}x{aNbV} {aParType} {aDegMin}-{aDegMax} {aCont} {aTol3D}";
                    aOk = aReuse
                        ? Both(anOracle,
                               () => aN.Init(aNPoints!, aNParType, aDegMin, aDegMax, aNCont, aTol3D),
                               () => aPort.Init(aPoints!, aParType, aDegMin, aDegMax, aCont, aTol3D), aW)
                        : Both(anOracle,
                               () => aNFresh = new N.GeomAPI_PointsToBSplineSurface(aNPoints!, aNParType, aDegMin, aDegMax,
                                                                                    aNCont, aTol3D),
                               () => aPort = new GeomAPI_PointsToBSplineSurface(aPoints!, aParType, aDegMin, aDegMax,
                                                                                aCont, aTol3D), aW);
                    break;
                case 1: // chord length, default arguments past the degrees
                    aW = $"#{c} {aNbU}x{aNbV} chord {aDegMin}-{aDegMax} {aCont}";
                    aTolerance = 1.0e-9;
                    aOk = aReuse
                        ? Both(anOracle,
                               () => aN.Init(aNPoints!, aDegMin, aDegMax, aNCont),
                               () => aPort.Init(aPoints!, aDegMin, aDegMax, aCont), aW)
                        : Both(anOracle,
                               () => aNFresh = new N.GeomAPI_PointsToBSplineSurface(aNPoints!, aDegMin, aDegMax, aNCont),
                               () => aPort = new GeomAPI_PointsToBSplineSurface(aPoints!, aDegMin, aDegMax, aCont), aW);
                    break;
                case 2: // variational
                {
                    if (aNbU < 3 || aNbV < 3)
                    {
                        continue; // OCCT reads past its points for two points (occt-issues.md, unit 2B)
                    }
                    double W1 = anOracle.Real(0, 1), W2 = anOracle.Real(0, 1), W3 = anOracle.Real(0, 1);
                    if (W1 + W2 + W3 < 0.01)
                    {
                        W1 = 1.0;
                    }
                    // Mostly degrees AppDef_Variational accepts (below 5 for C1 and 7 for C2 it throws
                    // "WorkDegree too small for given ConstraintOrder", on both sides)
                    int aDeg = anOracle.Int(0, 3) == 0 ? anOracle.Int(3, 6) : anOracle.Int(7, 8);
                    // Not C0: native results change from run to run there (occt-issues.md, FEmTool_Curve::D2)
                    Continuity aVCont = anOracle.Bool() ? Continuity.C1 : Continuity.C2;
                    N.GeomAbs_Shape aNVCont = (N.GeomAbs_Shape)(int)aVCont;
                    aW = $"#{c} {aNbU}x{aNbV} variational {W1:R} {W2:R} {W3:R} {aDeg} {aVCont} {aTol3D}";
                    aTolerance = 1.0e-6;
                    aOk = aReuse
                        ? Both(anOracle,
                               () =>
                               {
                                   lock (Oracle.NativeVariational)
                                   {
                                       aN.Init(aNPoints!, W1, W2, W3, aDeg, aNVCont, aTol3D);
                                   }
                               },
                               () => aPort.Init(aPoints!, W1, W2, W3, aDeg, aVCont, aTol3D), aW)
                        : Both(anOracle,
                               () =>
                               {
                                   lock (Oracle.NativeVariational)
                                   {
                                       aNFresh = new N.GeomAPI_PointsToBSplineSurface(aNPoints!, W1, W2, W3, aDeg, aNVCont, aTol3D);
                                   }
                               },
                               () => aPort = new GeomAPI_PointsToBSplineSurface(aPoints!, W1, W2, W3, aDeg, aVCont,
                                                                                aTol3D), aW);
                    break;
                }
                case 3: // interpolation, periodic in U or not
                {
                    bool aPeriodic = anOracle.Bool();
                    bool aChord = anOracle.Bool();
                    aW = $"#{c} {aNbU}x{aNbV} interpolate {(aChord ? "chord" : aParType)} periodic {aPeriodic}";
                    aTolerance = 1.0e-9;
                    aOk = aChord
                        ? Both(anOracle,
                               () => aN.Interpolate(aNPoints!, aPeriodic),
                               () => aPort.Interpolate(aPoints!, aPeriodic), aW)
                        : Both(anOracle,
                               () => aN.Interpolate(aNPoints!, aNParType, aPeriodic),
                               () => aPort.Interpolate(aPoints!, aParType, aPeriodic), aW);
                    break;
                }
                case 4: // periodic approximation (BuildPeriodicTangent)
                    aW = $"#{c} {aNbU}x{aNbV} periodic {aParType} {aDegMin}-{aDegMax} {aCont} {aTol3D}";
                    aOk = Both(anOracle,
                               () => aN.Init(aNPoints!, aNParType, aDegMin, aDegMax, aNCont, aTol3D, true),
                               () => aPort.Init(aPoints!, aParType, aDegMin, aDegMax, aCont, aTol3D, true), aW);
                    break;
                case 5: // Z grid, approximated (constructor or Init)
                    aW = $"#{c} {aNbU}x{aNbV} Z {X0:R} {dX:R} {Y0:R} {dY:R} {aDegMin}-{aDegMax} {aCont} {aTol3D}";
                    aTolerance = 1.0e-9;
                    aOk = aReuse
                        ? Both(anOracle,
                               () => aN.Init(aNZ!, X0, dX, Y0, dY, aDegMin, aDegMax, aNCont, aTol3D),
                               () => aPort.Init(aZ!, X0, dX, Y0, dY, aDegMin, aDegMax, aCont, aTol3D), aW)
                        : Both(anOracle,
                               () => aNFresh = new N.GeomAPI_PointsToBSplineSurface(aNZ!, X0, dX, Y0, dY, aDegMin, aDegMax,
                                                                                    aNCont, aTol3D),
                               () => aPort = new GeomAPI_PointsToBSplineSurface(aZ!, X0, dX, Y0, dY, aDegMin, aDegMax,
                                                                                aCont, aTol3D), aW);
                    break;
                default: // Z grid, interpolated
                    aW = $"#{c} {aNbU}x{aNbV} Z interpolate {X0:R} {dX:R} {Y0:R} {dY:R}";
                    aTolerance = 1.0e-9;
                    aOk = Both(anOracle,
                               () => aN.Interpolate(aNZ!, X0, dX, Y0, dY),
                               () => aPort.Interpolate(aZ!, X0, dX, Y0, dY), aW);
                    break;
            }

            using (aNFresh)
            {
                if (!aOk)
                {
                    aThrown++;
                    continue;
                }
                N.GeomAPI_PointsToBSplineSurface aNUsed = aNFresh ?? aN;
                anOracle.Same(aNUsed.IsDone(), aPort.IsDone(), $"{aW} IsDone");
                if (!aPort.IsDone())
                {
                    aNotDone++;
                    continue;
                }
                using N.Geom_BSplineSurface? aNSurface = aNUsed.Surface();
                Assert.NotNull(aNSurface);
                SameSurface(anOracle, aNSurface, aPort.Surface(), aW, aTolerance);
                aDone++;
                aByMethod[aMethod]++;
            }
        }
        output.WriteLine($"{aDone} surfaces ({string.Join(", ", aByMethod)} by method), {aNotDone} not done, {aThrown} thrown on both sides");
        anOracle.Report("GeomAPI_PointsToBSplineSurface");
    }
}
