#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests.Convert;

/// <summary>
/// The Convert package against native OCCT, through GeomConvert::CurveToBSplineCurve,
/// Geom2dConvert::CurveToBSplineCurve and GeomConvert::SurfaceToBSplineSurface, which build the B-spline from the
/// converters' output. The tests repeat what those functions do around the converters (GeomConvert.cxx,
/// GeomConvert_1.cxx, Geom2dConvert.cxx): lifting the 2D poles into the conic's plane, the Geom_BSplineCurve
/// SetPeriodic of a whole circle or ellipse, the mirror of a left-handed 2D conic. Arcs of 6 or more radians with
/// Convert_RationalC1 are left out: OCCT converts two halves and concatenates them with
/// GeomConvert_CompCurveToBSplineCurve, which isn't ported.
/// </summary>
public class ConvertOracleTests(ITestOutputHelper output)
{
    private const int Cases = 400;

    // Axes built on both sides from the same raw components, so that the native and the port normalize the same
    // numbers (rather than the port's normalized directions being normalized again by native OCCT).

    private static (double, double, double) RawDir(Oracle theOracle)
    {
        while (true)
        {
            double x = theOracle.Real(-1, 1), y = theOracle.Real(-1, 1), z = theOracle.Real(-1, 1);
            if (x * x + y * y + z * z > 0.01)
            {
                return (x, y, z);
            }
        }
    }

    private static void Ax2(Oracle theOracle, out gp_Ax2 thePort, out N.gp_Ax2 theNative)
    {
        gp_Pnt aP = theOracle.Pnt();
        var (nx, ny, nz) = RawDir(theOracle);
        var (vx, vy, vz) = RawDir(theOracle);
        thePort = new gp_Ax2(aP, new gp_Dir(nx, ny, nz), new gp_Dir(vx, vy, vz));
        using N.gp_Pnt aNP = Oracle.ToNative(aP);
        using var aNN = new N.gp_Dir(nx, ny, nz);
        using var aNV = new N.gp_Dir(vx, vy, vz);
        theNative = new N.gp_Ax2(aNP, aNN, aNV);
    }

    private static void Ax3(Oracle theOracle, out gp_Ax3 thePort, out N.gp_Ax3 theNative)
    {
        gp_Pnt aP = theOracle.Pnt();
        var (nx, ny, nz) = RawDir(theOracle);
        var (vx, vy, vz) = RawDir(theOracle);
        thePort = new gp_Ax3(aP, new gp_Dir(nx, ny, nz), new gp_Dir(vx, vy, vz));
        using N.gp_Pnt aNP = Oracle.ToNative(aP);
        using var aNN = new N.gp_Dir(nx, ny, nz);
        using var aNV = new N.gp_Dir(vx, vy, vz);
        theNative = new N.gp_Ax3(aNP, aNN, aNV);
        if (theOracle.Int(0, 1) == 1)
        {
            thePort = thePort.YReversed();
            theNative.YReverse();
        }
    }

    private static void Ax22d(Oracle theOracle, out gp_Ax22d thePort, out N.gp_Ax22d theNative)
    {
        var aP = new gp_Pnt2d(theOracle.Real(), theOracle.Real());
        double x, y;
        do
        {
            x = theOracle.Real(-1, 1);
            y = theOracle.Real(-1, 1);
        }
        while (x * x + y * y < 0.01);
        bool isSense = theOracle.Int(0, 1) == 1;
        thePort = new gp_Ax22d(aP, new gp_Dir2d(x, y), isSense);
        using N.gp_Pnt2d aNP = Oracle.ToNative(aP);
        using var aND = new N.gp_Dir2d(x, y);
        theNative = new N.gp_Ax22d(aNP, aND, isSense);
    }

    /// <summary>A Convert result as GeomConvert's BSplineCurveBuilder makes it, before any SetPeriodic.</summary>
    private sealed class CurveData
    {
        public int Degree;
        public bool Periodic;
        public NCollection_Array1<double> Weights = null!;
        public NCollection_Array1<double> Knots = null!;
        public NCollection_Array1<int> Mults = null!;

        public static CurveData From(Convert_ConicToBSplineCurve theConvert)
            => new()
            {
                Degree = theConvert.Degree(),
                Periodic = theConvert.IsPeriodic(),
                Weights = theConvert.Weights(),
                Knots = theConvert.Knots(),
                Mults = theConvert.Multiplicities(),
            };

        /// <summary>Geom_BSplineCurve::SetPeriodic: the knots from the first to the last used one, the end multiplicities
        /// evened out, the poles cut to the periodic count. Returns that count.</summary>
        public int SetPeriodic()
        {
            int aFirst = Periodic ? 1 : Occt.BSplCLib.FirstUKnotIndex(Degree, Mults);
            int aLast = Periodic ? Knots.Length() : Occt.BSplCLib.LastUKnotIndex(Degree, Mults);
            var aKnots = new NCollection_Array1<double>(1, aLast - aFirst + 1);
            var aMults = new NCollection_Array1<int>(1, aLast - aFirst + 1);
            for (int k = aFirst; k <= aLast; k++)
            {
                aKnots[k - aFirst + 1] = Knots[k];
                aMults[k - aFirst + 1] = Mults[k];
            }

            int anEnd = Math.Min(Degree, Math.Max(aMults[1], aMults[aMults.Upper()]));
            aMults[1] = anEnd;
            aMults[aMults.Upper()] = anEnd;
            Knots = aKnots;
            Mults = aMults;
            Periodic = true;
            return Occt.BSplCLib.NbPoles(Degree, true, Mults);
        }
    }

    private static void SameCurve(Oracle theOracle, N.Geom_BSplineCurve theNative, CurveData theData,
                                  NCollection_Array1<gp_Pnt> thePoles, int theNbPoles, string theWhat)
    {
        theOracle.Same(theNative.Degree(), theData.Degree, theWhat + " degree");
        theOracle.Same(theNative.IsPeriodic(), theData.Periodic, theWhat + " periodic");
        theOracle.Same(theNative.NbPoles(), theNbPoles, theWhat + " NbPoles");
        for (int i = 1; i <= theNbPoles; i++)
        {
            using N.gp_Pnt aNP = theNative.Pole(i);
            theOracle.Same(aNP, thePoles[i], theWhat + " pole");
            // A curve whose weights are all equal is not rational for Geom, which then answers 1
            theOracle.Same(theNative.Weight(i), theNative.IsRational() ? theData.Weights[i] : 1.0, theWhat + " weight");
        }

        theOracle.Same(theNative.NbKnots(), theData.Knots.Length(), theWhat + " NbKnots");
        for (int k = 1; k <= theData.Knots.Length(); k++)
        {
            theOracle.Same(theNative.Knot(k), theData.Knots[k], theWhat + " knot");
            theOracle.Same(theNative.Multiplicity(k), theData.Mults[k], theWhat + " mult");
        }
    }

    private static void SameCurve2d(Oracle theOracle, N.Geom2d_BSplineCurve theNative, CurveData theData,
                                    NCollection_Array1<gp_Pnt2d> thePoles, int theNbPoles, string theWhat)
    {
        theOracle.Same(theNative.Degree(), theData.Degree, theWhat + " degree");
        theOracle.Same(theNative.IsPeriodic(), theData.Periodic, theWhat + " periodic");
        theOracle.Same(theNative.NbPoles(), theNbPoles, theWhat + " NbPoles");
        for (int i = 1; i <= theNbPoles; i++)
        {
            using N.gp_Pnt2d aNP = theNative.Pole(i);
            theOracle.Same(aNP, thePoles[i], theWhat + " pole");
            theOracle.Same(theNative.Weight(i), theNative.IsRational() ? theData.Weights[i] : 1.0, theWhat + " weight");
        }

        theOracle.Same(theNative.NbKnots(), theData.Knots.Length(), theWhat + " NbKnots");
        for (int k = 1; k <= theData.Knots.Length(); k++)
        {
            theOracle.Same(theNative.Knot(k), theData.Knots[k], theWhat + " knot");
            theOracle.Same(theNative.Multiplicity(k), theData.Mults[k], theWhat + " mult");
        }
    }

    private enum ConicKind
    {
        Circle,
        Ellipse,
        Hyperbola,
        Parabola,
    }

    [Fact]
    public void Conics_to_BSpline_curves()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0, aNbSplit = 0;
        for (int i = 0; i < Cases; i++)
        {
            var aKind = (ConicKind)anOracle.Int(0, 3);
            var aParam = (Convert_ParameterisationType)anOracle.Int(0, 7);
            bool isWhole = aKind <= ConicKind.Ellipse && anOracle.Int(0, 3) == 0;
            Ax2(anOracle, out gp_Ax2 anAx2, out N.gp_Ax2 aNAx2);
            using N.gp_Ax2 aNAx2Owned = aNAx2;
            double aR1 = anOracle.Real(0.5, 20), aR2 = anOracle.Real(0.1, aR1);
            using N.Geom_Curve aNConic = aKind switch
            {
                ConicKind.Circle => new N.Geom_Circle(aNAx2, aR1),
                ConicKind.Ellipse => new N.Geom_Ellipse(aNAx2, aR1, aR2),
                ConicKind.Hyperbola => new N.Geom_Hyperbola(aNAx2, aR1, aR2),
                _ => new N.Geom_Parabola(aNAx2, aR1),
            };

            double aU1 = 0.0, aU2 = 0.0;
            N.Geom_Curve aNCurve = aNConic;
            N.Geom_TrimmedCurve? aNTrimmed = null;
            if (!isWhole)
            {
                double aFirst = aKind <= ConicKind.Ellipse ? anOracle.Real(-7, 7) : aKind == ConicKind.Hyperbola ? anOracle.Real(-3, 2) : anOracle.Real(-10, 8);
                double aLength = aKind <= ConicKind.Ellipse ? anOracle.Real(0.05, 2 * Math.PI) : anOracle.Real(0.05, aKind == ConicKind.Hyperbola ? 3 : 10);
                aNTrimmed = new N.Geom_TrimmedCurve(aNConic, aFirst, aFirst + aLength);
                aNCurve = aNTrimmed;
                aU1 = aNTrimmed.FirstParameter();
                aU2 = aNTrimmed.LastParameter();
                if (aParam == Convert_ParameterisationType.Convert_RationalC1 && aKind <= ConicKind.Ellipse && aU2 - aU1 >= 6.0)
                {
                    aNbSplit++;
                    aNTrimmed.Dispose();
                    continue;
                }
            }

            N.Geom_BSplineCurve? aNative = null;
            bool isNativeThrown = false;
            try
            {
                aNative = N.GeomConvert.CurveToBSplineCurve(aNCurve, (N.Convert_ParameterisationType)(int)aParam);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            Convert_ConicToBSplineCurve? aConvert = null;
            bool isPortThrown = false;
            try
            {
                aConvert = aKind switch
                {
                    ConicKind.Circle when isWhole => new Convert_CircleToBSplineCurve(new gp_Circ2d(Occt.gp.OX2d(), aR1), aParam),
                    ConicKind.Circle => new Convert_CircleToBSplineCurve(new gp_Circ2d(Occt.gp.OX2d(), aR1), aU1, aU2, aParam),
                    ConicKind.Ellipse when isWhole => new Convert_EllipseToBSplineCurve(new gp_Elips2d(Occt.gp.OX2d(), aR1, aR2), aParam),
                    ConicKind.Ellipse => new Convert_EllipseToBSplineCurve(new gp_Elips2d(Occt.gp.OX2d(), aR1, aR2), aU1, aU2, aParam),
                    ConicKind.Hyperbola => new Convert_HyperbolaToBSplineCurve(new gp_Hypr2d(Occt.gp.OX2d(), aR1, aR2), aU1, aU2),
                    _ => new Convert_ParabolaToBSplineCurve(new gp_Parab2d(Occt.gp.OX2d(), aR1), aU1, aU2),
                };
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            aNTrimmed?.Dispose();
            anOracle.Same(isNativeThrown, isPortThrown, $"{aKind} {aParam} throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineCurve aNResult = aNative!;
            CurveData aData = CurveData.From(aConvert!);

            // BSplineCurveBuilder: the 2D poles in the plane z = 0, moved from the global system to the conic's
            NCollection_Array1<gp_Pnt2d> aPoles2d = aConvert!.Poles();
            gp_Trsf aT = gp_Trsf.Transformation(new gp_Ax3(anAx2), new gp_Ax3(Occt.gp.XOY()));
            var aPoles = new NCollection_Array1<gp_Pnt>(1, aPoles2d.Length());
            for (int k = aPoles2d.Lower(); k <= aPoles2d.Upper(); k++)
            {
                aPoles[k] = new gp_Pnt(aPoles2d[k].X(), aPoles2d[k].Y(), 0.0).Transformed(aT);
            }

            int aNbPoles = isWhole ? aData.SetPeriodic() : aPoles.Length();
            SameCurve(anOracle, aNResult, aData, aPoles, aNbPoles, $"{aKind} {aParam}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} conics; {aNbSplit} long RationalC1 arcs left out");
        anOracle.Report("Convert conics (GeomConvert::CurveToBSplineCurve)");
    }

    [Fact]
    public void Conics2d_to_BSpline_curves()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0, aNbSplit = 0;
        for (int i = 0; i < Cases; i++)
        {
            var aKind = (ConicKind)anOracle.Int(0, 1);
            var aParam = (Convert_ParameterisationType)anOracle.Int(0, 7);
            bool isWhole = anOracle.Int(0, 3) == 0;
            Ax22d(anOracle, out gp_Ax22d anAx, out N.gp_Ax22d aNAx);
            using N.gp_Ax22d aNAxOwned = aNAx;
            double aR1 = anOracle.Real(0.5, 20), aR2 = anOracle.Real(0.1, aR1);
            using N.Geom2d_Curve aNConic = aKind == ConicKind.Circle ? new N.Geom2d_Circle(aNAx, aR1)
                                                                     : new N.Geom2d_Ellipse(aNAx, aR1, aR2);
            double aU1 = 0.0, aU2 = 0.0;
            N.Geom2d_Curve aNCurve = aNConic;
            N.Geom2d_TrimmedCurve? aNTrimmed = null;
            if (!isWhole)
            {
                double aFirst = anOracle.Real(-7, 7);
                aNTrimmed = new N.Geom2d_TrimmedCurve(aNConic, aFirst, aFirst + anOracle.Real(0.05, 2 * Math.PI));
                aNCurve = aNTrimmed;
                aU1 = aNTrimmed.FirstParameter();
                aU2 = aNTrimmed.LastParameter();
                if (aParam == Convert_ParameterisationType.Convert_RationalC1 && aU2 - aU1 >= 6.0)
                {
                    aNbSplit++;
                    aNTrimmed.Dispose();
                    continue;
                }
            }

            N.Geom2d_BSplineCurve? aNative = null;
            bool isNativeThrown = false;
            try
            {
                aNative = N.Geom2dConvert.CurveToBSplineCurve(aNCurve, (N.Convert_ParameterisationType)(int)aParam);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            Convert_ConicToBSplineCurve? aConvert = null;
            bool isPortThrown = false;
            try
            {
                aConvert = (aKind, isWhole) switch
                {
                    (ConicKind.Circle, true) => new Convert_CircleToBSplineCurve(new gp_Circ2d(Occt.gp.OX2d(), aR1), aParam),
                    (ConicKind.Circle, false) => new Convert_CircleToBSplineCurve(new gp_Circ2d(Occt.gp.OX2d(), aR1), aU1, aU2, aParam),
                    (_, true) => new Convert_EllipseToBSplineCurve(new gp_Elips2d(Occt.gp.OX2d(), aR1, aR2), aParam),
                    _ => new Convert_EllipseToBSplineCurve(new gp_Elips2d(Occt.gp.OX2d(), aR1, aR2), aU1, aU2, aParam),
                };
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            aNTrimmed?.Dispose();
            anOracle.Same(isNativeThrown, isPortThrown, $"2d {aKind} {aParam} throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                continue;
            }

            using N.Geom2d_BSplineCurve aNResult = aNative!;
            CurveData aData = CurveData.From(aConvert!);

            // Geom2dConvert's BSplineCurveBuilder: a mirror for a left-handed conic, then from the global system to the
            // conic's X axis
            NCollection_Array1<gp_Pnt2d> aPoles = aConvert!.Poles();
            if ((anAx.XDirection() ^ anAx.YDirection()) < 0.0)
            {
                gp_Trsf2d aSym = gp_Trsf2d.Mirror(Occt.gp.OX2d());
                for (int k = aPoles.Lower(); k <= aPoles.Upper(); k++)
                {
                    aPoles[k] = aPoles[k].Transformed(aSym);
                }
            }

            gp_Trsf2d aT = gp_Trsf2d.Transformation(anAx.XAxis(), Occt.gp.OX2d());
            for (int k = aPoles.Lower(); k <= aPoles.Upper(); k++)
            {
                aPoles[k] = aPoles[k].Transformed(aT);
            }

            int aNbPoles = isWhole ? aData.SetPeriodic() : aPoles.Length();
            SameCurve2d(anOracle, aNResult, aData, aPoles, aNbPoles, $"2d {aKind} {aParam}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} 2D conics; {aNbSplit} long RationalC1 arcs left out");
        anOracle.Report("Convert 2D conics (Geom2dConvert::CurveToBSplineCurve)");
    }

    private enum QuadricKind
    {
        Cylinder,
        Cone,
        Sphere,
        Torus,
    }

    [Fact]
    public void Quadrics_to_BSpline_surfaces()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        for (int i = 0; i < Cases; i++)
        {
            var aKind = (QuadricKind)anOracle.Int(0, 3);
            Ax3(anOracle, out gp_Ax3 anAx3, out N.gp_Ax3 aNAx3);
            using N.gp_Ax3 aNAx3Owned = aNAx3;
            double aR1 = anOracle.Real(1, 20), aR2 = anOracle.Real(0.1, aR1 * 0.9), anAngle = anOracle.Real(0.05, 1.4);
            using N.Geom_Surface aNSurface = aKind switch
            {
                QuadricKind.Cylinder => new N.Geom_CylindricalSurface(aNAx3, aR1),
                QuadricKind.Cone => new N.Geom_ConicalSurface(aNAx3, anAngle, aR1),
                QuadricKind.Sphere => new N.Geom_SphericalSurface(aNAx3, aR1),
                _ => new N.Geom_ToroidalSurface(aNAx3, aR1, aR2),
            };

            // 0: trimmed in U and V; 1: closed in U (0 to 2 Pi); 2 (torus): closed in V; 3 (sphere, torus): untrimmed
            int aMode = aKind switch
            {
                QuadricKind.Cylinder or QuadricKind.Cone => anOracle.Int(0, 1),
                QuadricKind.Sphere => anOracle.Int(0, 3) switch { 2 => 3, var m => m },
                _ => anOracle.Int(0, 3),
            };
            double aU1 = anOracle.Real(0, 5), aU2 = aU1 + anOracle.Real(0.1, 2 * Math.PI - aU1 - 0.05);
            double aV1, aV2;
            if (aKind == QuadricKind.Sphere)
            {
                aV1 = anOracle.Real(-Math.PI / 2, 1.4);
                aV2 = aV1 + anOracle.Real(0.05, Math.PI / 2 - aV1);
            }
            else if (aKind == QuadricKind.Torus)
            {
                aV1 = anOracle.Real(0, 5);
                aV2 = aV1 + anOracle.Real(0.1, 2 * Math.PI - aV1 - 0.05);
            }
            else
            {
                aV1 = anOracle.Real(-20, 10);
                aV2 = aV1 + anOracle.Real(0.1, 20);
            }

            if (aMode == 1)
            {
                aU1 = 0.0;
                aU2 = 2 * Math.PI;
            }
            else if (aMode == 2)
            {
                aV1 = 0.0;
                aV2 = 2 * Math.PI;
            }

            N.Geom_RectangularTrimmedSurface? aNTrimmed = aMode == 3 ? null : new N.Geom_RectangularTrimmedSurface(aNSurface, aU1, aU2, aV1, aV2);
            if (aNTrimmed != null)
            {
                aNTrimmed.Bounds(ref aU1, ref aU2, ref aV1, ref aV2);
            }

            N.Geom_BSplineSurface? aNative = null;
            bool isNativeThrown = false;
            try
            {
                aNative = N.GeomConvert.SurfaceToBSplineSurface(aNTrimmed ?? aNSurface);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            // GeomConvert::SurfaceToBSplineSurface's choice of converter
            bool isUClosed = Math.Abs((aU2 - aU1) - 2.0 * Math.PI) <= Precision.PConfusion();
            Convert_ElementarySurfaceToBSplineSurface? aConvert = null;
            bool isPortThrown = false;
            try
            {
                switch (aKind)
                {
                    case QuadricKind.Cylinder:
                        var aCyl = new gp_Cylinder(anAx3, aR1);
                        aConvert = isUClosed ? new Convert_CylinderToBSplineSurface(aCyl, aV1, aV2)
                                             : new Convert_CylinderToBSplineSurface(aCyl, aU1, aU2, aV1, aV2);
                        break;
                    case QuadricKind.Cone:
                        var aCone = new gp_Cone(anAx3, anAngle, aR1);
                        aConvert = isUClosed ? new Convert_ConeToBSplineSurface(aCone, aV1, aV2)
                                             : new Convert_ConeToBSplineSurface(aCone, aU1, aU2, aV1, aV2);
                        break;
                    case QuadricKind.Sphere:
                        var aSph = new gp_Sphere(anAx3, aR1);
                        aConvert = aMode == 3 ? new Convert_SphereToBSplineSurface(aSph)
                                   : isUClosed ? new Convert_SphereToBSplineSurface(aSph, aV1, aV2, false)
                                               : new Convert_SphereToBSplineSurface(aSph, aU1, aU2, aV1, aV2);
                        break;
                    default:
                        var aTor = new gp_Torus(anAx3, aR1, aR2);
                        bool isVClosed = aMode == 2;
                        aConvert = aMode == 3 ? new Convert_TorusToBSplineSurface(aTor)
                                   : isUClosed ? new Convert_TorusToBSplineSurface(aTor, aV1, aV2, false)
                                   : isVClosed ? new Convert_TorusToBSplineSurface(aTor, aU1, aU2)
                                               : new Convert_TorusToBSplineSurface(aTor, aU1, aU2, aV1, aV2);
                        break;
                }
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            aNTrimmed?.Dispose();
            anOracle.Same(isNativeThrown, isPortThrown, $"{aKind} throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineSurface aNResult = aNative!;
            string aWhat = $"{aKind} mode {aMode}";
            Convert_ElementarySurfaceToBSplineSurface C = aConvert!;
            anOracle.Same(aNResult.UDegree(), C.UDegree(), aWhat + " UDegree");
            anOracle.Same(aNResult.VDegree(), C.VDegree(), aWhat + " VDegree");
            anOracle.Same(aNResult.IsUPeriodic(), C.IsUPeriodic(), aWhat + " UPeriodic");
            anOracle.Same(aNResult.IsVPeriodic(), C.IsVPeriodic(), aWhat + " VPeriodic");
            NCollection_Array2<gp_Pnt> aPoles = C.Poles();
            NCollection_Array2<double> aWeights = C.Weights();
            anOracle.Same(aNResult.NbUPoles(), aPoles.ColLength(), aWhat + " NbUPoles");
            anOracle.Same(aNResult.NbVPoles(), aPoles.RowLength(), aWhat + " NbVPoles");
            bool isRational = aNResult.IsURational() || aNResult.IsVRational();
            for (int r = 1; r <= aPoles.ColLength(); r++)
            {
                for (int c = 1; c <= aPoles.RowLength(); c++)
                {
                    using N.gp_Pnt aNP = aNResult.Pole(r, c);
                    anOracle.Same(aNP, aPoles[r, c], aWhat + " pole");
                    anOracle.Same(aNResult.Weight(r, c), isRational ? aWeights[r, c] : 1.0, aWhat + " weight");
                }
            }

            NCollection_Array1<double> aUKnots = C.UKnots(), aVKnots = C.VKnots();
            NCollection_Array1<int> aUMults = C.UMultiplicities(), aVMults = C.VMultiplicities();
            anOracle.Same(aNResult.NbUKnots(), aUKnots.Length(), aWhat + " NbUKnots");
            anOracle.Same(aNResult.NbVKnots(), aVKnots.Length(), aWhat + " NbVKnots");
            for (int k = 1; k <= aUKnots.Length(); k++)
            {
                anOracle.Same(aNResult.UKnot(k), aUKnots[k], aWhat + " UKnot");
                anOracle.Same(aNResult.UMultiplicity(k), aUMults[k], aWhat + " UMult");
            }

            for (int k = 1; k <= aVKnots.Length(); k++)
            {
                anOracle.Same(aNResult.VKnot(k), aVKnots[k], aWhat + " VKnot");
                anOracle.Same(aNResult.VMultiplicity(k), aVMults[k], aWhat + " VMult");
            }
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} surfaces");
        anOracle.Report("Convert quadrics (GeomConvert::SurfaceToBSplineSurface)");
    }
}
#endif
