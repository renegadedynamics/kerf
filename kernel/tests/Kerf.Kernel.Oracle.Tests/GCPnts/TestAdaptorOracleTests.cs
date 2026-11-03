// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Xunit.Abstractions;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The test stand-ins for GeomAdaptor_Curve and Geom2dAdaptor_Curve (tests/Kerf.Kernel.ModelingData.Tests/TestAdaptors.cs)
/// against native OCCT's adaptors, on random lines, conics, Bezier and B-spline curves built from the same raw numbers:
/// type, continuity, intervals, resolution, and D0 to D3 and DN at the ends, the interval bounds and inside.
/// </summary>
public class TestAdaptorOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(300, 91);

    private static readonly Continuity[] Shapes = [C0, C1, C2, C3, CN];

    // ---------------------------------------------------------------------------------------------
    // Random curves
    // ---------------------------------------------------------------------------------------------

    /// <summary>Random knots and multiplicities (some inner knots of high multiplicity, so the curve has C1/C2 breaks).</summary>
    private static (NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, int NbPoles) RandomKnots(
        Oracle theO, int theDegree, bool thePeriodic)
    {
        while (true)
        {
            int aNbKnots = theO.Int(2, 7);
            var aKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aMults = new NCollection_Array1<int>(1, aNbKnots);
            aKnots[1] = theO.Real(-10, 10);
            for (int i = 2; i <= aNbKnots; i++)
            {
                aKnots[i] = aKnots[i - 1] + theO.Real(0.1, 3);
            }
            for (int i = 2; i < aNbKnots; i++)
            {
                aMults[i] = theO.Int(1, theDegree);
            }
            if (thePeriodic)
            {
                aMults[1] = aMults[aNbKnots] = theO.Int(1, theDegree);
            }
            else
            {
                aMults[1] = aMults[aNbKnots] = theDegree + 1;
            }
            int aNbPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(theDegree, thePeriodic, aMults);
            if (aNbPoles < 2 || (thePeriodic && aNbPoles <= theDegree))
            {
                continue;
            }
            return (aKnots, aMults, aNbPoles);
        }
    }

    private static NCollection_Array1<double>? RandomWeights(Oracle theO, int theNb)
    {
        if (theO.Bool())
        {
            return null;
        }
        var aWeights = new NCollection_Array1<double>(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aWeights[i] = theO.Real(0.5, 2);
        }
        return aWeights;
    }

    internal static (N.Geom_Curve Native, Curve3D Port, string Kind) RandomCurve(Oracle theO)
    {
        switch (theO.Int(0, 6))
        {
            case 0:
            {
                (N.gp_Ax1 aNA, Axis3D aA) = GeomOracleInputs.Ax1s(theO);
                using (aNA)
                {
                    return (new N.Geom_Line(aNA), new LineCurve3D(aA), "line");
                }
            }
            case 1:
            {
                (N.gp_Ax2 aNA, Frame3D aA) = GeomOracleInputs.Ax2s(theO);
                double aR = theO.Real(0.1, 50);
                using (aNA)
                {
                    return (new N.Geom_Circle(aNA, aR), new CircleCurve3D(aA, aR), "circle");
                }
            }
            case 2:
            {
                (N.gp_Ax2 aNA, Frame3D aA) = GeomOracleInputs.Ax2s(theO);
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.05, aMaj);
                using (aNA)
                {
                    return (new N.Geom_Ellipse(aNA, aMaj, aMin), new EllipseCurve3D(aA, aMaj, aMin), "ellipse");
                }
            }
            case 3:
            {
                (N.gp_Ax2 aNA, Frame3D aA) = GeomOracleInputs.Ax2s(theO);
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.1, 50);
                using (aNA)
                {
                    return theO.Bool()
                        ? (new N.Geom_Hyperbola(aNA, aMaj, aMin), new HyperbolaCurve3D(aA, aMaj, aMin), "hyperbola")
                        : (new N.Geom_Parabola(aNA, aMaj), new ParabolaCurve3D(aA, aMaj), "parabola");
                }
            }
            case 4:
            {
                int aNbPoles = theO.Int(2, 11);
                var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    aPoles[i] = theO.Pnt();
                }
                NCollection_Array1<double>? aWeights = RandomWeights(theO, aNbPoles);
                using var aNPoles = Oracle.ToNative(aPoles);
                if (aWeights is null)
                {
                    return (new N.Geom_BezierCurve(aNPoles), new BezierCurve3D(aPoles), "bezier");
                }
                using var aNWeights = Oracle.ToNative(aWeights);
                return (new N.Geom_BezierCurve(aNPoles, aNWeights), new BezierCurve3D(aPoles, aWeights), "rational bezier");
            }
            default:
            {
                int aDeg = theO.Int(1, 6);
                bool aPer = theO.Int(0, 2) == 0;
                (NCollection_Array1<double> aKnots, NCollection_Array1<int> aMults, int aNbPoles) = RandomKnots(theO, aDeg, aPer);
                var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    aPoles[i] = theO.Pnt();
                }
                NCollection_Array1<double>? aWeights = RandomWeights(theO, aNbPoles);
                BSplineCurve3D aPort = aWeights is null
                    ? new BSplineCurve3D(aPoles, aKnots, aMults, aDeg, aPer)
                    : new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg, aPer);
                return (Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, aPer), aPort,
                        $"bspline deg {aDeg}{(aPer ? " periodic" : "")}{(aWeights is null ? "" : " rational")}");
            }
        }
    }

    private static (N.gp_Ax2d Native, Axis2D Port) Ax2ds(Oracle theO)
    {
        Point2D aP = theO.Pnt2d();
        double aX, aY;
        do
        {
            aX = theO.Real(-1, 1);
            aY = theO.Real(-1, 1);
        } while (aX * aX + aY * aY < 0.01);
        using var aNP = Oracle.ToNative(aP);
        using var aND = new N.gp_Dir2d(aX, aY);
        return (new N.gp_Ax2d(aNP, aND), new Axis2D(aP, new Direction2D(aX, aY)));
    }

    internal static (N.Geom2d_Curve Native, Curve2D Port, string Kind) RandomCurve2d(Oracle theO)
    {
        switch (theO.Int(0, 4))
        {
            case 0:
            {
                (N.gp_Ax2d aNA, Axis2D aA) = Ax2ds(theO);
                using (aNA)
                {
                    return (new N.Geom2d_Line(aNA), new LineCurve2D(aA), "line2d");
                }
            }
            case 1:
            {
                (N.gp_Ax2d aNA, Axis2D aA) = Ax2ds(theO);
                double aR = theO.Real(0.1, 50);
                bool aSense = theO.Bool();
                using (aNA)
                {
                    return (new N.Geom2d_Circle(aNA, aR, aSense), new CircleCurve2D(aA, aR, aSense), "circle2d");
                }
            }
            case 2:
            {
                (N.gp_Ax2d aNA, Axis2D aA) = Ax2ds(theO);
                double aMaj = theO.Real(0.1, 50), aMin = theO.Real(0.05, aMaj);
                using (aNA)
                {
                    return (new N.Geom2d_Ellipse(aNA, aMaj, aMin), new EllipseCurve2D(aA, aMaj, aMin), "ellipse2d");
                }
            }
            default:
            {
                int aDeg = theO.Int(1, 6);
                bool aPer = theO.Int(0, 2) == 0;
                (NCollection_Array1<double> aKnots, NCollection_Array1<int> aMults, int aNbPoles) = RandomKnots(theO, aDeg, aPer);
                var aPoles = new NCollection_Array1<Point2D>(1, aNbPoles);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    aPoles[i] = theO.Pnt2d();
                }
                NCollection_Array1<double>? aWeights = RandomWeights(theO, aNbPoles);
                BSplineCurve2D aPort = aWeights is null
                    ? new BSplineCurve2D(aPoles, aKnots, aMults, aDeg, aPer)
                    : new BSplineCurve2D(aPoles, aWeights, aKnots, aMults, aDeg, aPer);
                return (Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, aPer), aPort,
                        $"bspline2d deg {aDeg}{(aPer ? " periodic" : "")}{(aWeights is null ? "" : " rational")}");
            }
        }
    }

    /// <summary>Random bounds: the curve's own when finite (or a sub-range of them), otherwise a range around 0.</summary>
    internal static (double First, double Last) Bounds(Oracle theO, double theFirst, double theLast, bool thePeriodic)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double a = theO.Real(-5, 0), b = theO.Real(0.5, 5);
            return (a, a + b);
        }
        switch (theO.Int(0, 2))
        {
            case 0:
                return (theFirst, theLast);
            case 1:
            {
                double a = theO.Real(theFirst, theLast), b = theO.Real(theFirst, theLast);
                return a < b ? (a, b) : (b, a);
            }
            default:
                // Periodic curves may be evaluated beyond their period.
                return thePeriodic ? (theFirst + 0.3, theLast + 0.7) : (theFirst, theLast);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Comparisons
    // ---------------------------------------------------------------------------------------------

    private static List<double> Params(Oracle theO, double theFirst, double theLast, List<double> theBreaks)
    {
        var aParams = new List<double> { theFirst, theLast };
        aParams.AddRange(theBreaks);
        for (int i = 0; i < 6; i++)
        {
            aParams.Add(theO.Real(theFirst, theLast));
        }
        return aParams;
    }

    private static List<double> SameIntervals3d(Oracle theO, N.GeomAdaptor_Curve theN, TestGeomAdaptor_Curve theP, string theWhat)
    {
        var aBreaks = new List<double>();
        foreach (Continuity aS in Shapes)
        {
            int aNb = theN.NbIntervals((N.GeomAbs_Shape)(int)aS);
            theO.Same(aNb, theP.NbIntervals(aS), $"{theWhat} NbIntervals({aS})");
            using var aNT = new N.TColStd_Array1OfReal(1, aNb + 1);
            var aT = new NCollection_Array1<double>(1, aNb + 1);
            theN.Intervals(aNT, (N.GeomAbs_Shape)(int)aS);
            theP.Intervals(aT, aS);
            for (int i = 1; i <= aNb + 1; i++)
            {
                theO.Same(aNT.Value(i), aT[i], $"{theWhat} Intervals({aS})[{i}]");
                aBreaks.Add(aT[i]);
            }
        }
        return aBreaks;
    }

    private static List<double> SameIntervals2d(Oracle theO, N.Geom2dAdaptor_Curve theN, TestGeom2dAdaptor_Curve theP, string theWhat)
    {
        var aBreaks = new List<double>();
        foreach (Continuity aS in Shapes)
        {
            int aNb = theN.NbIntervals((N.GeomAbs_Shape)(int)aS);
            theO.Same(aNb, theP.NbIntervals(aS), $"{theWhat} NbIntervals({aS})");
            using var aNT = new N.TColStd_Array1OfReal(1, aNb + 1);
            var aT = new NCollection_Array1<double>(1, aNb + 1);
            theN.Intervals(aNT, (N.GeomAbs_Shape)(int)aS);
            theP.Intervals(aT, aS);
            for (int i = 1; i <= aNb + 1; i++)
            {
                theO.Same(aNT.Value(i), aT[i], $"{theWhat} Intervals({aS})[{i}]");
                aBreaks.Add(aT[i]);
            }
        }
        return aBreaks;
    }

    private static void SameEval3d(Oracle theO, N.GeomAdaptor_Curve theN, TestGeomAdaptor_Curve theP, double U, string theWhat)
    {
        using var aNP = new N.gp_Pnt();
        using var aNV1 = new N.gp_Vec();
        using var aNV2 = new N.gp_Vec();
        using var aNV3 = new N.gp_Vec();
        string aW = $"{theWhat} at {U:R}";

        theN.D0(U, aNP);
        theP.D0(U, out Point3D aP);
        theO.Same(aNP, aP, $"{aW} D0");

        theN.D1(U, aNP, aNV1);
        theP.D1(U, out aP, out Vector3D aV1);
        theO.Same(aNP, aP, $"{aW} D1 P");
        theO.Same(aNV1, aV1, $"{aW} D1 V1");

        theN.D2(U, aNP, aNV1, aNV2);
        theP.D2(U, out aP, out aV1, out Vector3D aV2);
        theO.Same(aNP, aP, $"{aW} D2 P");
        theO.Same(aNV1, aV1, $"{aW} D2 V1");
        theO.Same(aNV2, aV2, $"{aW} D2 V2");

        theN.D3(U, aNP, aNV1, aNV2, aNV3);
        theP.D3(U, out aP, out aV1, out aV2, out Vector3D aV3);
        theO.Same(aNP, aP, $"{aW} D3 P");
        theO.Same(aNV1, aV1, $"{aW} D3 V1");
        theO.Same(aNV2, aV2, $"{aW} D3 V2");
        theO.Same(aNV3, aV3, $"{aW} D3 V3");

        for (int n = 1; n <= 3; n++)
        {
            using N.gp_Vec aNDN = theN.DN(U, n);
            theO.Same(aNDN, theP.DN(U, n), $"{aW} DN({n})");
        }
    }

    private static void SameEval2d(Oracle theO, N.Geom2dAdaptor_Curve theN, TestGeom2dAdaptor_Curve theP, double U, string theWhat)
    {
        using var aNP = new N.gp_Pnt2d();
        using var aNV1 = new N.gp_Vec2d();
        using var aNV2 = new N.gp_Vec2d();
        using var aNV3 = new N.gp_Vec2d();
        string aW = $"{theWhat} at {U:R}";

        theN.D0(U, aNP);
        theP.D0(U, out Point2D aP);
        theO.Same(aNP, aP, $"{aW} D0");

        theN.D1(U, aNP, aNV1);
        theP.D1(U, out aP, out Vector2D aV1);
        theO.Same(aNP, aP, $"{aW} D1 P");
        theO.Same(aNV1, aV1, $"{aW} D1 V1");

        theN.D2(U, aNP, aNV1, aNV2);
        theP.D2(U, out aP, out aV1, out Vector2D aV2);
        theO.Same(aNP, aP, $"{aW} D2 P");
        theO.Same(aNV1, aV1, $"{aW} D2 V1");
        theO.Same(aNV2, aV2, $"{aW} D2 V2");

        theN.D3(U, aNP, aNV1, aNV2, aNV3);
        theP.D3(U, out aP, out aV1, out aV2, out Vector2D aV3);
        theO.Same(aNP, aP, $"{aW} D3 P");
        theO.Same(aNV1, aV1, $"{aW} D3 V1");
        theO.Same(aNV2, aV2, $"{aW} D3 V2");
        theO.Same(aNV3, aV3, $"{aW} D3 V3");

        for (int n = 1; n <= 3; n++)
        {
            using N.gp_Vec2d aNDN = theN.DN(U, n);
            theO.Same(aNDN, theP.DN(U, n), $"{aW} DN({n})");
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Tests
    // ---------------------------------------------------------------------------------------------

    [Fact]
    public void GeomAdaptor_Curve_stand_in()
    {
        var anOracle = new Oracle(output);
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom_Curve aNC, Curve3D aPC, string aKind) = RandomCurve(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = Bounds(anOracle, aPC.FirstParameter, aPC.LastParameter, aPC.IsPeriodic);
                N.GeomAdaptor_Curve aN;
                TestGeomAdaptor_Curve aP;
                N.Geom_TrimmedCurve? aNTrim = null;
                if (anOracle.Int(0, 3) == 0)
                {
                    // Through a trimmed curve, which the adaptors unwrap.
                    aNTrim = new N.Geom_TrimmedCurve(aNC, aFirst, aLast);
                    var aPTrim = new TrimmedCurve3D(aPC, aFirst, aLast);
                    aN = new N.GeomAdaptor_Curve(aNTrim);
                    aP = new TestGeomAdaptor_Curve(aPTrim);
                }
                else
                {
                    aN = new N.GeomAdaptor_Curve(aNC, aFirst, aLast);
                    aP = new TestGeomAdaptor_Curve(aPC, aFirst, aLast);
                }
                using (aNTrim)
                using (aN)
                {
                    string aW = $"{aKind} #{c}";
                    anOracle.Same((int)aN.GetType_(), (int)aP.GetType(), $"{aW} GetType");
                    anOracle.Same(aN.FirstParameter(), aP.FirstParameter(), $"{aW} First");
                    anOracle.Same(aN.LastParameter(), aP.LastParameter(), $"{aW} Last");
                    anOracle.Same((int)aN.Continuity(), (int)aP.Continuity(), $"{aW} Continuity");
                    anOracle.Same(aN.IsPeriodic(), aP.IsPeriodic, $"{aW} IsPeriodic");
                    anOracle.Same(aN.IsClosed(), aP.IsClosed, $"{aW} IsClosed");
                    if (aP.IsPeriodic)
                    {
                        anOracle.Same(aN.Period(), aP.Period(), $"{aW} Period");
                    }
                    anOracle.Same(aN.IsRational(), aP.IsRational, $"{aW} IsRational");
                    foreach (double aTol in new[] { global::Kerf.Engine.Foundation.Precision.Confusion(), 1e-3, 0.1 })
                    {
                        anOracle.SameRelative(aN.Resolution(aTol), aP.Resolution(aTol), $"{aW} Resolution({aTol})");
                    }
                    List<double> aBreaks = SameIntervals3d(anOracle, aN, aP, aW);
                    foreach (double u in Params(anOracle, aFirst, aLast, aBreaks))
                    {
                        SameEval3d(anOracle, aN, aP, u, aW);
                    }
                }
            }
        }
        anOracle.Report("TestGeomAdaptor_Curve against GeomAdaptor_Curve");
    }

    [Fact]
    public void Geom2dAdaptor_Curve_stand_in()
    {
        var anOracle = new Oracle(output);
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom2d_Curve aNC, Curve2D aPC, string aKind) = RandomCurve2d(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = Bounds(anOracle, aPC.FirstParameter, aPC.LastParameter, aPC.IsPeriodic);
                N.Geom2dAdaptor_Curve aN;
                TestGeom2dAdaptor_Curve aP;
                N.Geom2d_TrimmedCurve? aNTrim = null;
                if (anOracle.Int(0, 3) == 0)
                {
                    aNTrim = new N.Geom2d_TrimmedCurve(aNC, aFirst, aLast);
                    var aPTrim = new TrimmedCurve2D(aPC, aFirst, aLast);
                    aN = new N.Geom2dAdaptor_Curve(aNTrim);
                    aP = new TestGeom2dAdaptor_Curve(aPTrim);
                }
                else
                {
                    aN = new N.Geom2dAdaptor_Curve(aNC, aFirst, aLast);
                    aP = new TestGeom2dAdaptor_Curve(aPC, aFirst, aLast);
                }
                using (aNTrim)
                using (aN)
                {
                    string aW = $"{aKind} #{c}";
                    anOracle.Same((int)aN.GetType_(), (int)aP.GetType(), $"{aW} GetType");
                    anOracle.Same(aN.FirstParameter(), aP.FirstParameter(), $"{aW} First");
                    anOracle.Same(aN.LastParameter(), aP.LastParameter(), $"{aW} Last");
                    anOracle.Same((int)aN.Continuity(), (int)aP.Continuity(), $"{aW} Continuity");
                    anOracle.Same(aN.IsPeriodic(), aP.IsPeriodic, $"{aW} IsPeriodic");
                    anOracle.Same(aN.IsClosed(), aP.IsClosed, $"{aW} IsClosed");
                    anOracle.Same(aN.NbSamples(), aP.SampleCount, $"{aW} NbSamples");
                    if (aP.IsPeriodic)
                    {
                        anOracle.Same(aN.Period(), aP.Period(), $"{aW} Period");
                    }
                    anOracle.Same(aN.IsRational(), aP.IsRational, $"{aW} IsRational");
                    foreach (double aTol in new[] { global::Kerf.Engine.Foundation.Precision.Confusion(), 1e-3, 0.1 })
                    {
                        anOracle.SameRelative(aN.Resolution(aTol), aP.Resolution(aTol), $"{aW} Resolution({aTol})");
                    }
                    List<double> aBreaks = SameIntervals2d(anOracle, aN, aP, aW);
                    foreach (double u in Params(anOracle, aFirst, aLast, aBreaks))
                    {
                        SameEval2d(anOracle, aN, aP, u, aW);
                    }
                }
            }
        }
        anOracle.Report("TestGeom2dAdaptor_Curve against Geom2dAdaptor_Curve");
    }
}
