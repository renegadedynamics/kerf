// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests (no native binding exists for Geom2dGridEval): every grid evaluator against point-by-point
/// evaluation of the same curve, on random curves of every kind. Point evaluation (Geom2d curves and
/// Geom2dAdaptor_Curve) is bit-identical to native OCCT, so this is the oracle substitute.
/// </summary>
/// <remarks>
/// Where OCCT's grid algorithm is the point evaluation's (lines; the Bezier and B-spline caches against
/// Geom2dAdaptor_Curve; the ellipse and hyperbola derivatives; the adaptor fallback), values are compared bit for
/// bit. Elsewhere OCCT's grid formulas associate differently (the circle's R * (cos X + sin Y) against ElCLib's
/// R cos X + R sin Y, the conic points adding the centre first, B-spline points evaluated directly on flat knots
/// against the adaptor's cache): there the values agree to 1e-12 relative. Three OCCT errors are pinned
/// separately (occt-issues.md): B-spline DN, rational DN above the degree, and the parabola of an indirect frame.
/// </remarks>
public class Geom2dGridEval_ConsistencyTest(ITestOutputHelper theOutput)
{
    private const double THE_REL_TOL = 1e-12;

    private sealed class Stats
    {
        public int Count;
        public int ExactCurve;
        public int ExactAdaptor;
        public int ExactEither;
        public double MaxRel; // of the closer reference
    }

    // The basis that decides the known DN errors: B-spline or Bezier degree and rationality.
    private readonly record struct DnInfo(bool IsBSpline, bool IsRational, int Degree, int FirstDelegatedN);

    private readonly Dictionary<string, Stats> myStats = new();

    private Stats StatsOf(string theKey)
    {
        if (!myStats.TryGetValue(theKey, out Stats? aStats))
        {
            aStats = new Stats();
            myStats[theKey] = aStats;
        }
        return aStats;
    }

    private void Compare(string theKey, Xy theGrid, Xy? theCurve, Xy theAdaptor)
    {
        Stats aStats = StatsOf(theKey);
        for (int aCoord = 1; aCoord <= 2; ++aCoord)
        {
            double aG = theGrid.Coord(aCoord);
            double aA = theAdaptor.Coord(aCoord);
            double aScale = Math.Max(1.0, Math.Max(theGrid.Modulus(), theAdaptor.Modulus()));
            bool isExactA = aG == aA;
            double aRel = Math.Abs(aG - aA) / aScale;
            bool isExactC = false;
            if (theCurve is Xy aCurve)
            {
                double aC = aCurve.Coord(aCoord);
                isExactC = aG == aC;
                aRel = Math.Min(aRel, Math.Abs(aG - aC) / Math.Max(aScale, aCurve.Modulus()));
            }
            aStats.Count++;
            aStats.ExactCurve += isExactC ? 1 : 0;
            aStats.ExactAdaptor += isExactA ? 1 : 0;
            aStats.ExactEither += (isExactA || isExactC) ? 1 : 0;
            aStats.MaxRel = Math.Max(aStats.MaxRel, aRel);
        }
    }

    private static NCollection_Array1<double> SortedParams(Random theRnd, double theFirst, double theLast, int theNb,
                                                           NCollection_Array1<double>? theKnots)
    {
        List<double> aList = [theFirst, theLast];
        for (int i = 0; i < theNb; ++i)
        {
            aList.Add(theFirst + (theLast - theFirst) * theRnd.NextDouble());
        }
        if (theKnots is not null)
        {
            for (int i = theKnots.Lower(); i <= theKnots.Upper(); ++i)
            {
                double aK = theKnots.Value(i);
                if (aK >= theFirst && aK <= theLast)
                {
                    aList.Add(aK);
                }
            }
        }
        aList.Sort();
        NCollection_Array1<double> aParams = new(1, aList.Count);
        for (int i = 0; i < aList.Count; ++i)
        {
            aParams.SetValue(i + 1, aList[i]);
        }
        return aParams;
    }

    private static Frame2D RandomAx22d(Random theRnd, bool theIsDirect)
    {
        double anAngle = theRnd.NextDouble() * 2 * M_PI;
        return new Frame2D(new Point2D(theRnd.NextDouble() * 20 - 10, theRnd.NextDouble() * 20 - 10),
                            new Direction2D(Math.Cos(anAngle), Math.Sin(anAngle)),
                            theIsDirect);
    }

    private static Frame2D RandomAx22d(Random theRnd) => RandomAx22d(theRnd, theRnd.Next(2) == 0);

    private static NCollection_Array1<Point2D> RandomPoles(Random theRnd, int theNb)
    {
        NCollection_Array1<Point2D> aPoles = new(1, theNb);
        for (int i = 1; i <= theNb; ++i)
        {
            aPoles.SetValue(i, new Point2D(i + theRnd.NextDouble() * 4 - 2, theRnd.NextDouble() * 10 - 5));
        }
        return aPoles;
    }

    private static NCollection_Array1<double> RandomWeights(Random theRnd, int theNb)
    {
        NCollection_Array1<double> aWeights = new(1, theNb);
        for (int i = 1; i <= theNb; ++i)
        {
            aWeights.SetValue(i, 0.3 + 2.0 * theRnd.NextDouble());
        }
        return aWeights;
    }

    // theSmooth: degree 2 at least and interior multiplicities below the degree (C1, for offsets)
    private static BSplineCurve2D RandomBSpline(Random theRnd, bool theRational, bool thePeriodic,
                                                     bool theSmooth = false)
    {
        int aDeg = theRnd.Next(theSmooth ? 2 : 1, 6);
        int aNbKnots = theRnd.Next(2, 8);
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        double aK = theRnd.NextDouble() * 4 - 2;
        int aNbPoles = -aDeg - 1;
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots.SetValue(i, aK);
            aK += 0.1 + theRnd.NextDouble() * 2;
            int aMult = (i == 1 || i == aNbKnots) ? aDeg + 1 : theRnd.Next(1, theSmooth ? aDeg : aDeg + 1);
            aMults.SetValue(i, aMult);
            aNbPoles += aMult;
        }
        NCollection_Array1<Point2D> aPoles = RandomPoles(theRnd, aNbPoles);
        BSplineCurve2D aCurve = theRational
            ? new BSplineCurve2D(aPoles, RandomWeights(theRnd, aNbPoles), aKnots, aMults, aDeg)
            : new BSplineCurve2D(aPoles, aKnots, aMults, aDeg);
        return thePeriodic ? aCurve.WithPeriodic() : aCurve;
    }

    private static BezierCurve2D RandomBezier(Random theRnd, bool theRational)
    {
        int aNbPoles = theRnd.Next(2, 10);
        NCollection_Array1<Point2D> aPoles = RandomPoles(theRnd, aNbPoles);
        return theRational ? new BezierCurve2D(aPoles, RandomWeights(theRnd, aNbPoles))
                           : new BezierCurve2D(aPoles);
    }

    private static EllipseCurve2D RandomEllipse(Random theRnd)
    {
        double aMin = 0.1 + 5 * theRnd.NextDouble();
        return new EllipseCurve2D(RandomAx22d(theRnd), aMin + 5 * theRnd.NextDouble(), aMin);
    }

    private static Curve2D RandomCurve(Random theRnd, string theKind)
    {
        return theKind switch
        {
            "Line" => new LineCurve2D(new Axis2D(RandomAx22d(theRnd).Location(), RandomAx22d(theRnd).XDirection())),
            "Circle" => new CircleCurve2D(RandomAx22d(theRnd), 0.1 + 10 * theRnd.NextDouble()),
            "Ellipse" => RandomEllipse(theRnd),
            "Hyperbola" => new HyperbolaCurve2D(RandomAx22d(theRnd), 3 + 5 * theRnd.NextDouble(),
                                                0.1 + 3 * theRnd.NextDouble()),
            // Direct frames only: an indirect one is OCCT's error, pinned below
            "Parabola" => new ParabolaCurve2D(RandomAx22d(theRnd, true), 0.1 + 5 * theRnd.NextDouble()),
            "Bezier" => RandomBezier(theRnd, false),
            "RationalBezier" => RandomBezier(theRnd, true),
            "BSpline" => RandomBSpline(theRnd, false, false),
            "RationalBSpline" => RandomBSpline(theRnd, true, false),
            "PeriodicBSpline" => RandomBSpline(theRnd, theRnd.Next(2) == 0, true),
            "OffsetCircle" => new OffsetCurve2D(RandomCurve(theRnd, "Circle"), theRnd.NextDouble() * 2 - 1),
            "OffsetEllipse" => new OffsetCurve2D(RandomCurve(theRnd, "Ellipse"), theRnd.NextDouble() * 2 - 1),
            "OffsetBezier" => new OffsetCurve2D(RandomBezier(theRnd, theRnd.Next(2) == 0), theRnd.NextDouble() - 0.5),
            "OffsetBSpline" => new OffsetCurve2D(RandomBSpline(theRnd, theRnd.Next(2) == 0, false, true),
                                                      theRnd.NextDouble() - 0.5),
            _ => throw new ArgumentException(theKind),
        };
    }

    private static (double First, double Last) Range(Curve2D theCurve)
    {
        double aFirst = theCurve.FirstParameter();
        double aLast = theCurve.LastParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast))
        {
            return Basis(theCurve) is HyperbolaCurve2D ? (-3.0, 3.0) : (-10.0, 10.0);
        }
        return (aFirst, aLast);
    }

    private static Curve2D Basis(Curve2D theCurve)
    {
        Curve2D aBasis = theCurve;
        while (true)
        {
            if (aBasis is TrimmedCurve2D aTrimmed)
            {
                aBasis = aTrimmed.BasisCurve();
            }
            else if (aBasis is OffsetCurve2D anOffset)
            {
                aBasis = anOffset.BasisCurve();
            }
            else
            {
                return aBasis;
            }
        }
    }

    private static DnInfo DnInfoOf(Curve2D theCurve)
    {
        Curve2D aCurve = theCurve;
        while (aCurve is TrimmedCurve2D aTrimmed)
        {
            aCurve = aTrimmed.BasisCurve();
        }
        // An offset curve's DN above 3 is its basis's DN, in the grid as in Geom2d_OffsetCurve
        int aFirstDelegatedN = aCurve is OffsetCurve2D ? 4 : 1;
        Curve2D aBasis = Basis(theCurve);
        return aBasis switch
        {
            BSplineCurve2D aBS => new DnInfo(true, aBS.IsRational, aBS.Degree(), aFirstDelegatedN),
            BezierCurve2D aBz => new DnInfo(false, aBz.IsRational, aBz.Degree(), aFirstDelegatedN),
            _ => new DnInfo(false, false, int.MaxValue, aFirstDelegatedN),
        };
    }

    private void CheckCase(string theKind, Geom2dGridEvalGrid2D.ICurveEvaluator theEval, Curve2DAdaptorBase theAdaptor,
                           Curve2D? theCurve, NCollection_Array1<double> theParams, DnInfo theDn)
    {
        int aNb = theParams.Length();
        NCollection_Array1<Point2D> aD0 = theEval.EvaluateGrid(theParams);
        NCollection_Array1<Curve2D.ResD1> aD1 = theEval.EvaluateGridD1(theParams);
        NCollection_Array1<Curve2D.ResD2> aD2 = theEval.EvaluateGridD2(theParams);
        NCollection_Array1<Curve2D.ResD3> aD3 = theEval.EvaluateGridD3(theParams);
        Assert.Equal(aNb, aD0.Length());
        Assert.Equal(aNb, aD1.Length());
        Assert.Equal(aNb, aD2.Length());
        Assert.Equal(aNb, aD3.Length());

        for (int i = 1; i <= aNb; ++i)
        {
            double aU = theParams.Value(i);
            Point2D aA0 = theAdaptor.EvalD0(aU);
            Curve2D.ResD1 aA1 = theAdaptor.EvalD1(aU);
            Curve2D.ResD2 aA2 = theAdaptor.EvalD2(aU);
            Curve2D.ResD3 aA3 = theAdaptor.EvalD3(aU);
            Point2D? aC0 = theCurve?.EvalD0(aU);
            Curve2D.ResD1? aC1 = theCurve?.EvalD1(aU);
            Curve2D.ResD2? aC2 = theCurve?.EvalD2(aU);
            Curve2D.ResD3? aC3 = theCurve?.EvalD3(aU);

            Compare(theKind + " D0", aD0.Value(i).XY(), aC0?.XY(), aA0.XY());
            Compare(theKind + " D1.P", aD1.Value(i).Point.XY(), aC1?.Point.XY(), aA1.Point.XY());
            Compare(theKind + " D1.D1", aD1.Value(i).D1.XY(), aC1?.D1.XY(), aA1.D1.XY());
            Compare(theKind + " D2.P", aD2.Value(i).Point.XY(), aC2?.Point.XY(), aA2.Point.XY());
            Compare(theKind + " D2.D1", aD2.Value(i).D1.XY(), aC2?.D1.XY(), aA2.D1.XY());
            Compare(theKind + " D2.D2", aD2.Value(i).D2.XY(), aC2?.D2.XY(), aA2.D2.XY());
            Compare(theKind + " D3.P", aD3.Value(i).Point.XY(), aC3?.Point.XY(), aA3.Point.XY());
            Compare(theKind + " D3.D1", aD3.Value(i).D1.XY(), aC3?.D1.XY(), aA3.D1.XY());
            Compare(theKind + " D3.D2", aD3.Value(i).D2.XY(), aC3?.D2.XY(), aA3.D2.XY());
            Compare(theKind + " D3.D3", aD3.Value(i).D3.XY(), aC3?.D3.XY(), aA3.D3.XY());
        }

        for (int aN = 1; aN <= 6; ++aN)
        {
            bool isDelegated = aN >= theDn.FirstDelegatedN;
            if (isDelegated && theDn.IsBSpline && aN <= theDn.Degree)
            {
                // OCCT: wrong span or out of range; pinned by BSplineDN_UsesFlatSpanIndexWithKnots
                continue;
            }

            NCollection_Array1<Vector2D> aDN;
            try
            {
                aDN = theEval.EvaluateGridDN(theParams, aN);
            }
            catch (global::Kerf.Engine.Foundation.MethodNotImplementedException)
            {
                // The adaptor fallback calls the adaptor's DN above 3, which Adaptor2d_OffsetCurve doesn't have
                Assert.Null(theCurve);
                Assert.Throws<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => theAdaptor.EvalDN(theParams.Value(1), aN));
                continue;
            }
            Assert.Equal(aNb, aDN.Length());

            if (isDelegated && theDn.IsRational && aN > theDn.Degree)
            {
                // OCCT: zero above the degree, although a rational curve's derivatives aren't
                for (int i = 1; i <= aNb; ++i)
                {
                    Assert.Equal(0.0, aDN.Value(i).X());
                    Assert.Equal(0.0, aDN.Value(i).Y());
                }
                continue;
            }

            for (int i = 1; i <= aNb; ++i)
            {
                double aU = theParams.Value(i);
                Xy? aCurveDN = theCurve?.EvalDN(aU, aN).XY();
                Xy anAdaptorDN;
                if (theCurve is null)
                {
                    // The adaptor fallback evaluates DN1-DN3 through the adaptor's D1-D3
                    anAdaptorDN = aN switch
                    {
                        1 => theAdaptor.EvalD1(aU).D1.XY(),
                        2 => theAdaptor.EvalD2(aU).D2.XY(),
                        _ => theAdaptor.EvalD3(aU).D3.XY(),
                    };
                }
                else
                {
                    anAdaptorDN = theAdaptor.EvalDN(aU, aN).XY();
                }
                Compare(theKind + " DN" + aN, aDN.Value(i).XY(), aCurveDN, anAdaptorDN);
            }
        }
    }

    private static readonly string[] THE_KINDS =
    [
        "Line", "Circle", "Ellipse", "Hyperbola", "Parabola", "Bezier", "RationalBezier", "BSpline",
        "RationalBSpline", "PeriodicBSpline", "OffsetCircle", "OffsetEllipse", "OffsetBezier", "OffsetBSpline",
    ];

    [Fact]
    public void GridEqualsPointEvaluation_RandomCurves()
    {
        Random aRnd = new(20261029);
        foreach (string aKind in THE_KINDS)
        {
            for (int aTry = 0; aTry < 20; ++aTry)
            {
                Curve2D aCurve = RandomCurve(aRnd, aKind);
                DnInfo aDn = DnInfoOf(aCurve);
                (double aFirst, double aLast) = Range(aCurve);
                NCollection_Array1<double>? aKnots = (Basis(aCurve) as BSplineCurve2D)?.InternalKnots();
                NCollection_Array1<double> aParams = SortedParams(aRnd, aFirst, aLast, 30, aKnots);

                CheckCase(aKind, new CurveGrid2D(aCurve), new CurveAdaptor2D(aCurve), aCurve, aParams,
                          aDn);

                // Through the adaptor, and through a trimmed curve inside the range
                CheckCase(aKind, new CurveGrid2D(new CurveAdaptor2D(aCurve)),
                          new CurveAdaptor2D(aCurve), aCurve, aParams, aDn);
                double aT1 = aFirst + 0.25 * (aLast - aFirst);
                double aT2 = aFirst + 0.75 * (aLast - aFirst);
                TrimmedCurve2D aTrimmed = new(aCurve, aT1, aT2);
                NCollection_Array1<double> aTrimParams = SortedParams(aRnd, aT1, aT2, 20, aKnots);
                CheckCase("Trimmed" + aKind, new CurveGrid2D(aTrimmed), new CurveAdaptor2D(aTrimmed),
                          aTrimmed, aTrimParams, aDn);

                // Any adaptor other than Geom2dAdaptor_Curve: Geom2dGridEval_OtherCurve
                if (aKind is "Circle" or "BSpline" or "Bezier")
                {
                    OffsetCurveAdaptor2D anOther = new(new CurveAdaptor2D(aCurve), 0.3);
                    CurveGrid2D anOtherEval = new(anOther);
                    Assert.Equal(anOther.GetType(), anOtherEval.GetType());
                    CheckCase("Other" + aKind, anOtherEval, anOther, null, aParams,
                              new DnInfo(false, false, int.MaxValue, 1));
                }
            }
        }

        foreach (KeyValuePair<string, Stats> anItem in myStats.OrderBy(theKv => theKv.Key, StringComparer.Ordinal))
        {
            Stats aS = anItem.Value;
            theOutput.WriteLine($"{anItem.Key,-28} n={aS.Count,6} exactCurve={aS.ExactCurve,6} "
                                + $"exactAdaptor={aS.ExactAdaptor,6} exactEither={aS.ExactEither,6} maxRel={aS.MaxRel:E2}");
        }

        foreach (KeyValuePair<string, Stats> anItem in myStats)
        {
            string aKey = anItem.Key;
            Stats aS = anItem.Value;
            Assert.True(aS.Count > 0, aKey);
            Assert.True(aS.MaxRel <= THE_REL_TOL, $"{aKey}: {aS.MaxRel:E3}");

            string aKind = aKey.Substring(0, aKey.IndexOf(' '));
            string aWhat = aKey.Substring(aKey.IndexOf(' ') + 1);
            bool isPoint = aWhat is "D0" or "D1.P" or "D2.P" or "D3.P";
            bool isDN = aWhat.StartsWith("DN", StringComparison.Ordinal);
            string aBase = aKind.Replace("Trimmed", "", StringComparison.Ordinal);

            // Bit for bit where OCCT's grid algorithm is the point evaluation's
            if (aBase == "Line")
            {
                Assert.True(aS.ExactCurve == aS.Count && aS.ExactAdaptor == aS.Count, aKey);
            }
            else if ((aBase is "Ellipse" or "Hyperbola" or "OffsetEllipse") && !isPoint && aWhat != "DN3")
            {
                Assert.True(aS.ExactCurve == aS.Count, aKey);
            }
            else if (aBase is "Bezier" or "RationalBezier" or "OffsetBezier" && !isDN)
            {
                // One cache over the single span, as Geom2dAdaptor_Curve's
                Assert.True(aS.ExactAdaptor == aS.Count, aKey);
            }
            else if (aBase is "Bezier" or "RationalBezier" && isDN)
            {
                // BSplCLib::DN on flat knots: Geom2d_BezierCurve's DN gives the same values
                Assert.True(aS.ExactCurve == aS.Count, aKey);
            }
            else if (aBase.StartsWith("Other", StringComparison.Ordinal))
            {
                Assert.True(aS.ExactAdaptor == aS.Count, aKey);
            }
        }
    }

    [Fact]
    public void BSplineGrid_CacheMatchesAdaptorBitForBit()
    {
        // A run of at least 4 parameters in one span uses BSplCLib_Cache, as Geom2dAdaptor_Curve does: parameters
        // strictly inside the spans of a non-periodic curve give the adaptor's values exactly.
        Random aRnd = new(7);
        for (int aTry = 0; aTry < 50; ++aTry)
        {
            BSplineCurve2D aCurve = RandomBSpline(aRnd, aRnd.Next(2) == 0, false);
            NCollection_Array1<double> aKnots = aCurve.InternalKnots();
            List<double> aList = [];
            for (int k = aKnots.Lower(); k < aKnots.Upper(); ++k)
            {
                double aA = aKnots.Value(k);
                double aB = aKnots.Value(k + 1);
                for (int j = 1; j <= 5; ++j)
                {
                    aList.Add(aA + (aB - aA) * j / 6.0);
                }
            }
            NCollection_Array1<double> aParams = NCollection_Array1<double>.From(aList.ToArray());
            NCollection_Array1<Curve2D.ResD3> aGrid = new BSplineCurveGrid2D(aCurve).EvaluateGridD3(aParams);
            CurveAdaptor2D anAdaptor = new(aCurve);
            for (int i = 1; i <= aParams.Length(); ++i)
            {
                Curve2D.ResD3 aRef = anAdaptor.EvalD3(aParams.Value(i));
                Assert.Equal(aRef.Point, aGrid.Value(i).Point);
                Assert.Equal(aRef.D1, aGrid.Value(i).D1);
                Assert.Equal(aRef.D2, aGrid.Value(i).D2);
                Assert.Equal(aRef.D3, aGrid.Value(i).D3);
            }
        }
    }

    // OCCT's Geom2dGridEval_BSplineCurve::EvaluateGridDN locates the span in the flat knots, then passes that
    // flat index with the knots and multiplicities to BSplCLib::DN, which takes it as a knot index: if it is a
    // valid knot index it evaluates that span's polynomial at U, otherwise it reads outside the knots.
    [Fact]
    public void BSplineDN_UsesFlatSpanIndexWithKnots()
    {
        // Cubic, knots 0..5 (flat index of the first span is 4, a valid knot index)
        NCollection_Array1<Point2D> aPoles = RandomPoles(new Random(3), 8);
        NCollection_Array1<double> aKnots = new(1, 6);
        NCollection_Array1<int> aMults = new(1, 6);
        for (int i = 1; i <= 6; ++i)
        {
            aKnots.SetValue(i, i - 1);
            aMults.SetValue(i, (i == 1 || i == 6) ? 4 : 1);
        }
        BSplineCurve2D aCurve = new(aPoles, aKnots, aMults, 3);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.5);
        Vector2D aGrid = new BSplineCurveGrid2D(aCurve).EvaluateGridDN(aParams, 1).Value(1);
        Vector2D aPoint = aCurve.EvalDN(0.5, 1);
        Assert.True((aGrid - aPoint).Magnitude() > 1e-3); // the span [3, 4] extrapolated to 0.5

        // What OCCT computes: BSplCLib::DN with knot index 4
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(0.5, 1, 4, 3, false, aCurve.InternalPoles(), null, aKnots, aMults, out Vector2D aSpan4);
        Assert.Equal(aSpan4, aGrid);

        // In the third span the flat index (6) is the last knot's: building the span's knots reads past the
        // array (OCCT reads outside it, the port throws). The last two spans locate correctly (index past the knots).
        aParams.SetValue(1, 2.5);
        Assert.Throws<IndexOutOfRangeException>(() => new BSplineCurveGrid2D(aCurve).EvaluateGridDN(aParams, 1));
    }

    // OCCT's Geom2dGridEval_BezierCurve and Geom2dGridEval_BSplineCurve::EvaluateGridDN return zero above the
    // degree, also for rational curves, whose derivatives of every order are generally not zero.
    [Fact]
    public void RationalDN_AboveDegreeIsZero()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        NCollection_Array1<double> aWeights = new(1, 3);
        aPoles.SetValue(1, new Point2D(1, 0));
        aPoles.SetValue(2, new Point2D(1, 1));
        aPoles.SetValue(3, new Point2D(0, 1));
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0 / Math.Sqrt(2.0));
        aWeights.SetValue(3, 1.0);
        BezierCurve2D aBezier = new(aPoles, aWeights);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.5);
        Vector2D aGrid = new BezierCurveGrid2D(aBezier).EvaluateGridDN(aParams, 3).Value(1);
        Assert.Equal(0.0, aGrid.Magnitude());
        Assert.True(aBezier.EvalDN(0.5, 3).Magnitude() > 0.1);
    }

    // OCCT's Geom2dGridEval_Parabola builds YDir as the mirror axis turned by +90 degrees: on a parabola whose
    // frame is indirect, the grid is the parabola of the direct frame, mirrored across the axis.
    [Fact]
    public void IndirectParabola_GridFollowsTheFrame()
    {
        // occt-issues.md, Geom2dGridEval_Parabola (Kerf decision 1495): the grid took the Y direction as X turned by
        // +90 degrees, so an indirect parabola came out as the direct one. Now the grid evaluates the curve, whose
        // points are at the focal distance from the focus and from the directrix.
        Random aRnd = new(11);
        for (int aTry = 0; aTry < 20; ++aTry)
        {
            Frame2D anAxes = RandomAx22d(aRnd, false);
            double aFocal = 0.1 + 5 * aRnd.NextDouble();
            ParabolaCurve2D anIndirect = new(anAxes, aFocal);
            NCollection_Array1<double> aParams = SortedParams(aRnd, -10, 10, 20, null);

            NCollection_Array1<Curve2D.ResD3> aGrid = new ParabolaGrid2D(anIndirect).EvaluateGridD3(aParams);
            Point2D aFocus = anIndirect.Parab2d().Focus();
            var aDirectrix = new Line2D(anIndirect.Parab2d().Directrix());
            for (int i = 1; i <= aParams.Length(); ++i)
            {
                double aU = aParams.Value(i);
                Curve2D.ResD3 aRef = anIndirect.EvalD3(aU);
                double aScale = Math.Max(1.0, aRef.Point.XY().Modulus());
                Assert.True(aGrid.Value(i).Point.Distance(aRef.Point) <= 1e-12 * aScale);
                Assert.True((aGrid.Value(i).D1 - aRef.D1).Magnitude() <= 1e-12 * Math.Max(1.0, aRef.D1.Magnitude()));
                Assert.True((aGrid.Value(i).D2 - aRef.D2).Magnitude() <= 1e-12);
                Point2D aP = aGrid.Value(i).Point;
                Assert.Equal(aDirectrix.Distance(aP), aP.Distance(aFocus), 1e-9 * aScale);
                // the side of the mirror axis the frame's Y direction says
                Assert.Equal(Math.Sign(aU), Math.Sign((aP.XY() - anAxes.Location().XY()).Dot(anAxes.YDirection().XY())));
            }
        }
    }

    [Fact]
    public void NullAndEmpty()
    {
        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0);
        Geom2dGridEvalGrid2D.ICurveEvaluator[] aNullEvals =
        [
            new LineGrid2D(null), new CircleGrid2D(null), new EllipseGrid2D(null),
            new HyperbolaGrid2D(null), new ParabolaGrid2D(null), new BezierCurveGrid2D(null),
            new BSplineCurveGrid2D(null), new OffsetCurveGrid2D(null),
            new CurveGrid2D((Curve2D?)null),
        ];
        foreach (Geom2dGridEvalGrid2D.ICurveEvaluator anEval in aNullEvals)
        {
            Assert.True(anEval.EvaluateGrid(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD1(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD2(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD3(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridDN(aParams, 1).IsEmpty());
        }

        // DN of order below 1 is empty
        CurveGrid2D aCircle = new(new CircleCurve2D(new Frame2D(), 1.0));
        Assert.True(aCircle.EvaluateGridDN(aParams, 0).IsEmpty());

        // Results are 1-based whatever the parameters' bounds
        NCollection_Array1<double> aShifted = new(-3, -2);
        aShifted.SetValue(-3, 0.0);
        aShifted.SetValue(-2, 1.0);
        NCollection_Array1<Point2D> aGrid = aCircle.EvaluateGrid(aShifted);
        Assert.Equal(1, aGrid.Lower());
        Assert.Equal(2, aGrid.Upper());
    }

    /// <summary>
    /// OCCT's Geom2dGridEval_Parabola divides by the focal length, which Geom2d_Parabola allows to be zero
    /// (ElCLib then evaluates the line Location + U * XDir): the grid is not finite (occt-issues.md).
    /// </summary>
    [Fact]
    public void ZeroFocalParabola_GridIsNotFinite()
    {
        ParabolaCurve2D aParab = new(new Frame2D(new Point2D(1, 2), new Direction2D(1, 0), new Direction2D(0, 1)), 0.0);
        NCollection_Array1<double> aParams = NCollection_Array1<double>.From([0.0, 1.5]);

        Assert.Equal(new Point2D(2.5, 2), aParab.EvalD0(1.5));
        NCollection_Array1<Point2D> aGrid = new ParabolaGrid2D(aParab).EvaluateGrid(aParams);
        Assert.True(double.IsNaN(aGrid.Value(1).X())); // 0 * inf
        Assert.True(double.IsInfinity(aGrid.Value(2).X()));
    }
}
