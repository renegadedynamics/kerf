// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The Convert package against native OCCT, through GeomConvert::CurveToBSplineCurve,
/// Geom2dConvert::CurveToBSplineCurve and GeomConvert::SurfaceToBSplineSurface, which build their B-splines from the
/// converters' output, compared with the ported GeomConvert and Geom2dConvert on the same inputs. Long RationalC1
/// arcs (6 radians or more), which OCCT converts in two halves joined by GeomConvert_CompCurveToBSplineCurve, are
/// included (first version: a re-implementation of GeomConvert's post-processing in the test, which had to leave them
/// out).
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

            Geom_Curve aConic = aKind switch
            {
                ConicKind.Circle => new Geom_Circle(anAx2, aR1),
                ConicKind.Ellipse => new Geom_Ellipse(anAx2, aR1, aR2),
                ConicKind.Hyperbola => new Geom_Hyperbola(anAx2, aR1, aR2),
                _ => new Geom_Parabola(anAx2, aR1),
            };

            N.Geom_Curve aNCurve = aNConic;
            Geom_Curve aCurve = aConic;
            N.Geom_TrimmedCurve? aNTrimmed = null;
            if (!isWhole)
            {
                double aFirst = aKind <= ConicKind.Ellipse ? anOracle.Real(-7, 7) : aKind == ConicKind.Hyperbola ? anOracle.Real(-3, 2) : anOracle.Real(-10, 8);
                double aLength = aKind <= ConicKind.Ellipse ? anOracle.Real(0.05, 2 * Math.PI) : anOracle.Real(0.05, aKind == ConicKind.Hyperbola ? 3 : 10);
                aNTrimmed = new N.Geom_TrimmedCurve(aNConic, aFirst, aFirst + aLength);
                aNCurve = aNTrimmed;
                aCurve = new Geom_TrimmedCurve(aConic, aFirst, aFirst + aLength);
                if (aParam == Convert_ParameterisationType.Convert_RationalC1 && aKind <= ConicKind.Ellipse
                    && aNTrimmed.LastParameter() - aNTrimmed.FirstParameter() >= 6.0)
                {
                    aNbSplit++;
                }
            }

            N.Geom_BSplineCurve? aNative = null;
            Geom_BSplineCurve? aPort = null;
            Exception? aNErr = AdaptorOracleInputs.Catch(
                () => aNative = N.GeomConvert.CurveToBSplineCurve(aNCurve, (N.Convert_ParameterisationType)(int)aParam));
            Exception? aPErr = AdaptorOracleInputs.Catch(() => aPort = GeomConvert.CurveToBSplineCurve(aCurve, aParam));
            aNTrimmed?.Dispose();
            if (!AdaptorOracleInputs.SameOutcome(aNErr, aPErr, $"{aKind} {aParam}"))
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineCurve aNResult = aNative!;
            GeomConvertOracleTests.SameBSplineCurve(anOracle, aNResult, aPort!, $"{aKind} {aParam}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} conics; {aNbSplit} long RationalC1 arcs");
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
            Geom2d_Curve aConic = aKind == ConicKind.Circle ? new Geom2d_Circle(anAx, aR1) : new Geom2d_Ellipse(anAx, aR1, aR2);
            N.Geom2d_Curve aNCurve = aNConic;
            Geom2d_Curve aCurve = aConic;
            N.Geom2d_TrimmedCurve? aNTrimmed = null;
            if (!isWhole)
            {
                double aFirst = anOracle.Real(-7, 7);
                double aLast = aFirst + anOracle.Real(0.05, 2 * Math.PI);
                aNTrimmed = new N.Geom2d_TrimmedCurve(aNConic, aFirst, aLast);
                aNCurve = aNTrimmed;
                aCurve = new Geom2d_TrimmedCurve(aConic, aFirst, aLast);
                if (aParam == Convert_ParameterisationType.Convert_RationalC1
                    && aNTrimmed.LastParameter() - aNTrimmed.FirstParameter() >= 6.0)
                {
                    aNbSplit++;
                }
            }

            N.Geom2d_BSplineCurve? aNative = null;
            Geom2d_BSplineCurve? aPort = null;
            Exception? aNErr = AdaptorOracleInputs.Catch(
                () => aNative = N.Geom2dConvert.CurveToBSplineCurve(aNCurve, (N.Convert_ParameterisationType)(int)aParam));
            Exception? aPErr = AdaptorOracleInputs.Catch(() => aPort = Geom2dConvert.CurveToBSplineCurve(aCurve, aParam));
            aNTrimmed?.Dispose();
            if (!AdaptorOracleInputs.SameOutcome(aNErr, aPErr, $"2d {aKind} {aParam}"))
            {
                aNbThrows++;
                continue;
            }

            using N.Geom2d_BSplineCurve aNResult = aNative!;
            Geom2dOracle.SameStructure(anOracle, aNResult, aPort!, $"2d {aKind} {aParam}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} 2D conics; {aNbSplit} long RationalC1 arcs");
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

            Geom_Surface aSurface = aKind switch
            {
                QuadricKind.Cylinder => new Geom_CylindricalSurface(anAx3, aR1),
                QuadricKind.Cone => new Geom_ConicalSurface(anAx3, anAngle, aR1),
                QuadricKind.Sphere => new Geom_SphericalSurface(anAx3, aR1),
                _ => new Geom_ToroidalSurface(anAx3, aR1, aR2),
            };
            N.Geom_RectangularTrimmedSurface? aNTrimmed = aMode == 3 ? null : new N.Geom_RectangularTrimmedSurface(aNSurface, aU1, aU2, aV1, aV2);
            Geom_Surface aPortInput = aMode == 3 ? aSurface : new Geom_RectangularTrimmedSurface(aSurface, aU1, aU2, aV1, aV2);

            N.Geom_BSplineSurface? aNative = null;
            Geom_BSplineSurface? aPort = null;
            Exception? aNErr = AdaptorOracleInputs.Catch(() => aNative = N.GeomConvert.SurfaceToBSplineSurface(aNTrimmed ?? aNSurface));
            Exception? aPErr = AdaptorOracleInputs.Catch(() => aPort = GeomConvert.SurfaceToBSplineSurface(aPortInput));
            aNTrimmed?.Dispose();
            string aWhat = $"{aKind} mode {aMode}";
            if (!AdaptorOracleInputs.SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineSurface aNResult = aNative!;
            GeomConvertOracleTests.SameBSplineSurface(anOracle, aNResult, aPort!, aWhat);
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} surfaces");
        anOracle.Report("Convert quadrics (GeomConvert::SurfaceToBSplineSurface)");
    }
}
