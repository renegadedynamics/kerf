#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Geom2dAdaptor_Curve against native OCCT on random curves: evaluation (cache, boundary spans and elementary
/// formulas), continuity and intervals, resolution and the type queries. Native OCCT binds the line, circle, ellipse,
/// B-spline and trimmed curves; the other types are covered by the unit tests.
/// </summary>
public class Geom2dAdaptorOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    private static readonly GeomAbs_Shape[] Shapes =
        { GeomAbs_Shape.GeomAbs_C0, GeomAbs_Shape.GeomAbs_C1, GeomAbs_Shape.GeomAbs_C2, GeomAbs_Shape.GeomAbs_C3, GeomAbs_Shape.GeomAbs_CN };

    /// <summary>D0 to D3, Value, DN and the Eval forms at U.</summary>
    private static void SameEval(Oracle theOracle, N.Geom2dAdaptor_Curve theNative, Geom2dAdaptor_Curve thePort, double U,
                                 string theWhat, int theMaxN)
    {
        using var aNP = new N.gp_Pnt2d();
        using var aNV1 = new N.gp_Vec2d();
        using var aNV2 = new N.gp_Vec2d();
        using var aNV3 = new N.gp_Vec2d();

        theNative.D0(U, aNP);
        thePort.D0(U, out gp_Pnt2d aP);
        theOracle.Same(aNP, aP, $"{theWhat} D0({U})");

        using (var aNVal = theNative.Value(U))
        {
            theOracle.Same(aNVal, thePort.Value(U), $"{theWhat} Value");
        }
        using (var aNE0 = theNative.EvalD0(U))
        {
            theOracle.Same(aNE0, thePort.EvalD0(U), $"{theWhat} EvalD0");
        }

        theNative.D1(U, aNP, aNV1);
        thePort.D1(U, out aP, out gp_Vec2d aV1);
        theOracle.Same(aNP, aP, $"{theWhat} D1 P");
        theOracle.Same(aNV1, aV1, $"{theWhat} D1 V1");

        theNative.D2(U, aNP, aNV1, aNV2);
        thePort.D2(U, out aP, out aV1, out gp_Vec2d aV2);
        theOracle.Same(aNP, aP, $"{theWhat} D2 P");
        theOracle.Same(aNV1, aV1, $"{theWhat} D2 V1");
        theOracle.Same(aNV2, aV2, $"{theWhat} D2 V2");

        theNative.D3(U, aNP, aNV1, aNV2, aNV3);
        thePort.D3(U, out aP, out aV1, out aV2, out gp_Vec2d aV3);
        theOracle.Same(aNP, aP, $"{theWhat} D3 P");
        theOracle.Same(aNV1, aV1, $"{theWhat} D3 V1");
        theOracle.Same(aNV2, aV2, $"{theWhat} D3 V2");
        theOracle.Same(aNV3, aV3, $"{theWhat} D3 V3");

        for (int n = 1; n <= theMaxN; n++)
        {
            using var aNDN = theNative.DN(U, n);
            theOracle.Same(aNDN, thePort.DN(U, n), $"{theWhat} DN({n})");
            using var aNEDN = theNative.EvalDN(U, n);
            theOracle.Same(aNEDN, thePort.EvalDN(U, n), $"{theWhat} EvalDN({n})");
        }
    }

    /// <summary>Range, closure, periodicity, continuity, intervals of every continuity, resolution, samples, type.</summary>
    private static void SameQueries(Oracle theOracle, N.Geom2dAdaptor_Curve theNative, Geom2dAdaptor_Curve thePort,
                                    string theWhat)
    {
        theOracle.Same((int)theNative.GetType_(), (int)thePort.GetType(), $"{theWhat} GetType");
        theOracle.Same(theNative.FirstParameter(), thePort.FirstParameter(), $"{theWhat} FirstParameter");
        theOracle.Same(theNative.LastParameter(), thePort.LastParameter(), $"{theWhat} LastParameter");
        theOracle.Same(theNative.IsClosed(), thePort.IsClosed(), $"{theWhat} IsClosed");
        theOracle.Same(theNative.IsPeriodic(), thePort.IsPeriodic(), $"{theWhat} IsPeriodic");
        if (thePort.IsPeriodic())
        {
            theOracle.Same(theNative.Period(), thePort.Period(), $"{theWhat} Period");
        }
        theOracle.Same((int)theNative.Continuity(), (int)thePort.Continuity(), $"{theWhat} Continuity");
        theOracle.Same(theNative.NbSamples(), thePort.NbSamples(), $"{theWhat} NbSamples");
        foreach (double aR in new[] { 1e-7, 1e-3, 0.1, theOracle.Real(0.0, 2.0) })
        {
            theOracle.SameRelative(theNative.Resolution(aR), thePort.Resolution(aR), $"{theWhat} Resolution({aR})");
        }

        foreach (GeomAbs_Shape aS in Shapes)
        {
            int aNb = theNative.NbIntervals((N.GeomAbs_Shape)(int)aS);
            theOracle.Same(aNb, thePort.NbIntervals(aS), $"{theWhat} NbIntervals({aS})");
            using var aNT = new N.TColStd_Array1OfReal(1, aNb + 1);
            NCollection_Array1<double> aT = new(1, aNb + 1);
            theNative.Intervals(aNT, (N.GeomAbs_Shape)(int)aS);
            thePort.Intervals(aT, aS);
            theOracle.Same(aNT.Length(), aT.Length(), $"{theWhat} Intervals({aS}) length");
            for (int i = 1; i <= aNb + 1; i++)
            {
                theOracle.Same(aNT.Value(i), aT[i], $"{theWhat} Intervals({aS})[{i}]");
            }
        }
    }

    private static void SameBSplineQueries(Oracle theOracle, N.Geom2dAdaptor_Curve theNative, Geom2dAdaptor_Curve thePort,
                                           string theWhat)
    {
        theOracle.Same(theNative.Degree(), thePort.Degree(), $"{theWhat} Degree");
        theOracle.Same(theNative.IsRational(), thePort.IsRational(), $"{theWhat} IsRational");
        theOracle.Same(theNative.NbPoles(), thePort.NbPoles(), $"{theWhat} NbPoles");
        theOracle.Same(theNative.NbKnots(), thePort.NbKnots(), $"{theWhat} NbKnots");
    }

    /// <summary>Evaluates at the ends (B-spline boundary spans), at the knots and at random parameters.</summary>
    private static void EvalAll(Oracle theOracle, N.Geom2dAdaptor_Curve theNative, Geom2dAdaptor_Curve thePort,
                                string theWhat, int theMaxN, IReadOnlyList<double>? theKnots = null)
    {
        double aF = thePort.FirstParameter(), aL = thePort.LastParameter();
        SameEval(theOracle, theNative, thePort, aF, $"{theWhat} first", theMaxN);
        SameEval(theOracle, theNative, thePort, aL, $"{theWhat} last", theMaxN);
        if (theKnots is not null)
        {
            foreach (double aK in theKnots)
            {
                if (aK >= aF && aK <= aL)
                {
                    SameEval(theOracle, theNative, thePort, aK, $"{theWhat} knot", theMaxN);
                }
            }
        }
        for (int k = 0; k < 6; k++)
        {
            SameEval(theOracle, theNative, thePort, theOracle.Real(aF, aL), theWhat, theMaxN);
        }
        // Evaluate the trimmed and the shallow copies too
        double aU1 = theOracle.Real(aF, aL), aU2 = theOracle.Real(aF, aL);
        if (aU1 > aU2)
        {
            (aU1, aU2) = (aU2, aU1);
        }
        using var aNTrim = N.Geom2dAdaptor_Curve.DownCast(theNative.Trim(aU1, aU2, 1e-7));
        Geom2dAdaptor_Curve aTrim = (Geom2dAdaptor_Curve)thePort.Trim(aU1, aU2, 1e-7);
        SameEval(theOracle, aNTrim!, aTrim, aU1, $"{theWhat} trimmed first", theMaxN);
        SameEval(theOracle, aNTrim!, aTrim, theOracle.Real(aU1, aU2), $"{theWhat} trimmed", theMaxN);
        SameQueries(theOracle, aNTrim!, aTrim, $"{theWhat} trimmed");
        using var aNCopy = N.Geom2dAdaptor_Curve.DownCast(theNative.ShallowCopy());
        Geom2dAdaptor_Curve aCopy = (Geom2dAdaptor_Curve)thePort.ShallowCopy();
        SameEval(theOracle, aNCopy!, aCopy, theOracle.Real(aF, aL), $"{theWhat} copy", theMaxN);
    }

    [Fact]
    public void BSpline()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle);
            using var aNCurve = Geom2dOracle.ToNative(aSpec);
            Geom2d_BSplineCurve aCurve = Geom2dOracle.ToPort(aSpec);
            double aF = aCurve.FirstParameter(), aL = aCurve.LastParameter();

            // The whole curve, a random part of it, and (periodic curves) a range shifted beyond the first period
            for (int r = 0; r < 3; r++)
            {
                double aU1 = aF, aU2 = aL;
                if (r == 1)
                {
                    aU1 = anOracle.Real(aF, aL);
                    aU2 = anOracle.Real(aF, aL);
                    if (aU1 > aU2)
                    {
                        (aU1, aU2) = (aU2, aU1);
                    }
                }
                else if (r == 2)
                {
                    if (!aCurve.IsPeriodic())
                    {
                        continue;
                    }
                    double aP = aL - aF;
                    aU1 = anOracle.Real(aF - aP, aL);
                    aU2 = aU1 + anOracle.Real(0.1, 1.5) * aP;
                }
                using var aNative = new N.Geom2dAdaptor_Curve(aNCurve, aU1, aU2);
                var aPort = new Geom2dAdaptor_Curve(aCurve, aU1, aU2);
                string aWhat = $"case {i}.{r}";
                SameQueries(anOracle, aNative, aPort, aWhat);
                SameBSplineQueries(anOracle, aNative, aPort, aWhat);
                EvalAll(anOracle, aNative, aPort, aWhat, aSpec.Degree + 1, aSpec.Knots);
            }

            // Through a trimmed curve: the adaptor loads the basis B-spline
            double aT1 = anOracle.Real(aF, aL), aT2 = anOracle.Real(aF, aL);
            if (Math.Abs(aT1 - aT2) > 1e-3)
            {
                if (aT1 > aT2 && !aCurve.IsPeriodic())
                {
                    (aT1, aT2) = (aT2, aT1);
                }
                using var aNTrimmed = new N.Geom2d_TrimmedCurve(aNCurve, aT1, aT2);
                var aTrimmed = new Geom2d_TrimmedCurve(aCurve, aT1, aT2);
                using var aNative = new N.Geom2dAdaptor_Curve(aNTrimmed);
                var aPort = new Geom2dAdaptor_Curve(aTrimmed);
                SameQueries(anOracle, aNative, aPort, $"case {i} trimmed curve");
                EvalAll(anOracle, aNative, aPort, $"case {i} trimmed curve", aSpec.Degree + 1, aSpec.Knots);
            }

            // Reloading the same curve with another range drops the cache
            {
                using var aNative = new N.Geom2dAdaptor_Curve(aNCurve);
                var aPort = new Geom2dAdaptor_Curve(aCurve);
                SameEval(anOracle, aNative, aPort, anOracle.Real(aF, aL), $"case {i} before reload", 2);
                double aU1 = anOracle.Real(aF, aL);
                double aU2 = anOracle.Real(aU1, aL);
                aNative.Load(aNCurve, aU1, aU2);
                aPort.Load(aCurve, aU1, aU2);
                SameQueries(anOracle, aNative, aPort, $"case {i} reloaded");
                SameEval(anOracle, aNative, aPort, aU1, $"case {i} reloaded", 2);
                SameEval(anOracle, aNative, aPort, anOracle.Real(aU1, aU2), $"case {i} reloaded", 2);
            }
        }
        anOracle.Report("Geom2dAdaptor_Curve on B-splines");
    }

    private static (gp_Ax2d Port, N.gp_Ax2d Native) Ax2d(Oracle theOracle)
    {
        double aX = theOracle.Real(), aY = theOracle.Real();
        double aDX, aDY;
        do
        {
            aDX = theOracle.Real(-1, 1);
            aDY = theOracle.Real(-1, 1);
        }
        while (aDX * aDX + aDY * aDY < 0.01);
        using var aNP = new N.gp_Pnt2d(aX, aY);
        using var aND = new N.gp_Dir2d(aDX, aDY);
        return (new gp_Ax2d(new gp_Pnt2d(aX, aY), new gp_Dir2d(aDX, aDY)), new N.gp_Ax2d(aNP, aND));
    }

    [Fact]
    public void Elementary()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var (anAx, aNAx) = Ax2d(anOracle);
            using (aNAx)
            {
                int aKind = i % 3;
                N.Geom2d_Curve aNCurve;
                Geom2d_Curve aCurve;
                switch (aKind)
                {
                    case 0:
                        aNCurve = new N.Geom2d_Line(aNAx);
                        aCurve = new Geom2d_Line(anAx);
                        break;
                    case 1:
                    {
                        double aR = anOracle.Real(0.01, 50);
                        aNCurve = new N.Geom2d_Circle(aNAx, aR);
                        aCurve = new Geom2d_Circle(anAx, aR);
                        break;
                    }
                    default:
                    {
                        double aMin = anOracle.Real(0.01, 20);
                        double aMaj = aMin + anOracle.Real(0, 20);
                        aNCurve = new N.Geom2d_Ellipse(aNAx, aMaj, aMin);
                        aCurve = new Geom2d_Ellipse(anAx, aMaj, aMin);
                        break;
                    }
                }
                using (aNCurve)
                {
                    double aU1 = aKind == 0 ? anOracle.Real(-50, 50) : anOracle.Real(-7, 7);
                    double aU2 = aU1 + anOracle.Real(0, aKind == 0 ? 50 : 7);
                    using var aNative = new N.Geom2dAdaptor_Curve(aNCurve, aU1, aU2);
                    var aPort = new Geom2dAdaptor_Curve(aCurve, aU1, aU2);
                    string aWhat = $"case {i} ({aPort.GetType()})";
                    SameQueries(anOracle, aNative, aPort, aWhat);
                    EvalAll(anOracle, aNative, aPort, aWhat, 5);

                    switch (aKind)
                    {
                        case 0:
                        {
                            using var aNL = aNative.Line();
                            using var aNLoc = aNL.Location();
                            using var aNDir = aNL.Direction();
                            gp_Lin2d aL = aPort.Line();
                            anOracle.Same(aNLoc, aL.Location(), $"{aWhat} Line location");
                            anOracle.Same(aNDir.X(), aL.Direction().X(), $"{aWhat} Line direction X");
                            anOracle.Same(aNDir.Y(), aL.Direction().Y(), $"{aWhat} Line direction Y");
                            break;
                        }
                        case 1:
                        {
                            using var aNC = aNative.Circle();
                            using var aNLoc = aNC.Location();
                            gp_Circ2d aC = aPort.Circle();
                            anOracle.Same(aNLoc, aC.Location(), $"{aWhat} Circle location");
                            anOracle.Same(aNC.Radius(), aC.Radius(), $"{aWhat} Circle radius");
                            break;
                        }
                        default:
                        {
                            using var aNE = aNative.Ellipse();
                            using var aNLoc = aNE.Location();
                            gp_Elips2d anE = aPort.Ellipse();
                            anOracle.Same(aNLoc, anE.Location(), $"{aWhat} Ellipse location");
                            anOracle.Same(aNE.MajorRadius(), anE.MajorRadius(), $"{aWhat} Ellipse major radius");
                            anOracle.Same(aNE.MinorRadius(), anE.MinorRadius(), $"{aWhat} Ellipse minor radius");
                            break;
                        }
                    }

                    // Through a trimmed curve
                    using var aNTrimmed = new N.Geom2d_TrimmedCurve(aNCurve, aU1, aU2 + 0.5);
                    var aTrimmed = new Geom2d_TrimmedCurve(aCurve, aU1, aU2 + 0.5);
                    using var aNT = new N.Geom2dAdaptor_Curve(aNTrimmed);
                    var aT = new Geom2dAdaptor_Curve(aTrimmed);
                    SameQueries(anOracle, aNT, aT, $"{aWhat} trimmed curve");
                    EvalAll(anOracle, aNT, aT, $"{aWhat} trimmed curve", 5);
                }
            }
        }
        anOracle.Report("Geom2dAdaptor_Curve on lines, circles and ellipses");
    }
}
#endif
