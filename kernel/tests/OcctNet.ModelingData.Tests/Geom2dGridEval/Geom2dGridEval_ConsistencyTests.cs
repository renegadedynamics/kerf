// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.ModelingData.Tests;

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
public class Geom2dGridEval_ConsistencyTests(ITestOutputHelper theOutput)
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

    private void Compare(string theKey, gp_XY theGrid, gp_XY? theCurve, gp_XY theAdaptor)
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
            if (theCurve is gp_XY aCurve)
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

    private static gp_Ax22d RandomAx22d(Random theRnd, bool theIsDirect)
    {
        double anAngle = theRnd.NextDouble() * 2 * M_PI;
        return new gp_Ax22d(new gp_Pnt2d(theRnd.NextDouble() * 20 - 10, theRnd.NextDouble() * 20 - 10),
                            new gp_Dir2d(Math.Cos(anAngle), Math.Sin(anAngle)),
                            theIsDirect);
    }

    private static gp_Ax22d RandomAx22d(Random theRnd) => RandomAx22d(theRnd, theRnd.Next(2) == 0);

    private static NCollection_Array1<gp_Pnt2d> RandomPoles(Random theRnd, int theNb)
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, theNb);
        for (int i = 1; i <= theNb; ++i)
        {
            aPoles.SetValue(i, new gp_Pnt2d(i + theRnd.NextDouble() * 4 - 2, theRnd.NextDouble() * 10 - 5));
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
    private static Geom2d_BSplineCurve RandomBSpline(Random theRnd, bool theRational, bool thePeriodic,
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
        NCollection_Array1<gp_Pnt2d> aPoles = RandomPoles(theRnd, aNbPoles);
        Geom2d_BSplineCurve aCurve = theRational
            ? new Geom2d_BSplineCurve(aPoles, RandomWeights(theRnd, aNbPoles), aKnots, aMults, aDeg)
            : new Geom2d_BSplineCurve(aPoles, aKnots, aMults, aDeg);
        return thePeriodic ? aCurve.WithPeriodic() : aCurve;
    }

    private static Geom2d_BezierCurve RandomBezier(Random theRnd, bool theRational)
    {
        int aNbPoles = theRnd.Next(2, 10);
        NCollection_Array1<gp_Pnt2d> aPoles = RandomPoles(theRnd, aNbPoles);
        return theRational ? new Geom2d_BezierCurve(aPoles, RandomWeights(theRnd, aNbPoles))
                           : new Geom2d_BezierCurve(aPoles);
    }

    private static Geom2d_Ellipse RandomEllipse(Random theRnd)
    {
        double aMin = 0.1 + 5 * theRnd.NextDouble();
        return new Geom2d_Ellipse(RandomAx22d(theRnd), aMin + 5 * theRnd.NextDouble(), aMin);
    }

    private static Geom2d_Curve RandomCurve(Random theRnd, string theKind)
    {
        return theKind switch
        {
            "Line" => new Geom2d_Line(new gp_Ax2d(RandomAx22d(theRnd).Location(), RandomAx22d(theRnd).XDirection())),
            "Circle" => new Geom2d_Circle(RandomAx22d(theRnd), 0.1 + 10 * theRnd.NextDouble()),
            "Ellipse" => RandomEllipse(theRnd),
            "Hyperbola" => new Geom2d_Hyperbola(RandomAx22d(theRnd), 3 + 5 * theRnd.NextDouble(),
                                                0.1 + 3 * theRnd.NextDouble()),
            // Direct frames only: an indirect one is OCCT's error, pinned below
            "Parabola" => new Geom2d_Parabola(RandomAx22d(theRnd, true), 0.1 + 5 * theRnd.NextDouble()),
            "Bezier" => RandomBezier(theRnd, false),
            "RationalBezier" => RandomBezier(theRnd, true),
            "BSpline" => RandomBSpline(theRnd, false, false),
            "RationalBSpline" => RandomBSpline(theRnd, true, false),
            "PeriodicBSpline" => RandomBSpline(theRnd, theRnd.Next(2) == 0, true),
            "OffsetCircle" => new Geom2d_OffsetCurve(RandomCurve(theRnd, "Circle"), theRnd.NextDouble() * 2 - 1),
            "OffsetEllipse" => new Geom2d_OffsetCurve(RandomCurve(theRnd, "Ellipse"), theRnd.NextDouble() * 2 - 1),
            "OffsetBezier" => new Geom2d_OffsetCurve(RandomBezier(theRnd, theRnd.Next(2) == 0), theRnd.NextDouble() - 0.5),
            "OffsetBSpline" => new Geom2d_OffsetCurve(RandomBSpline(theRnd, theRnd.Next(2) == 0, false, true),
                                                      theRnd.NextDouble() - 0.5),
            _ => throw new ArgumentException(theKind),
        };
    }

    private static (double First, double Last) Range(Geom2d_Curve theCurve)
    {
        double aFirst = theCurve.FirstParameter();
        double aLast = theCurve.LastParameter();
        if (Precision.IsInfinite(aFirst) || Precision.IsInfinite(aLast))
        {
            return Basis(theCurve) is Geom2d_Hyperbola ? (-3.0, 3.0) : (-10.0, 10.0);
        }
        return (aFirst, aLast);
    }

    private static Geom2d_Curve Basis(Geom2d_Curve theCurve)
    {
        Geom2d_Curve aBasis = theCurve;
        while (true)
        {
            if (aBasis is Geom2d_TrimmedCurve aTrimmed)
            {
                aBasis = aTrimmed.BasisCurve();
            }
            else if (aBasis is Geom2d_OffsetCurve anOffset)
            {
                aBasis = anOffset.BasisCurve();
            }
            else
            {
                return aBasis;
            }
        }
    }

    private static DnInfo DnInfoOf(Geom2d_Curve theCurve)
    {
        Geom2d_Curve aCurve = theCurve;
        while (aCurve is Geom2d_TrimmedCurve aTrimmed)
        {
            aCurve = aTrimmed.BasisCurve();
        }
        // An offset curve's DN above 3 is its basis's DN, in the grid as in Geom2d_OffsetCurve
        int aFirstDelegatedN = aCurve is Geom2d_OffsetCurve ? 4 : 1;
        Geom2d_Curve aBasis = Basis(theCurve);
        return aBasis switch
        {
            Geom2d_BSplineCurve aBS => new DnInfo(true, aBS.IsRational(), aBS.Degree(), aFirstDelegatedN),
            Geom2d_BezierCurve aBz => new DnInfo(false, aBz.IsRational(), aBz.Degree(), aFirstDelegatedN),
            _ => new DnInfo(false, false, int.MaxValue, aFirstDelegatedN),
        };
    }

    private void CheckCase(string theKind, Geom2dGridEval.ICurveEvaluator theEval, Adaptor2d_Curve2d theAdaptor,
                           Geom2d_Curve? theCurve, NCollection_Array1<double> theParams, DnInfo theDn)
    {
        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt2d> aD0 = theEval.EvaluateGrid(theParams);
        NCollection_Array1<Geom2d_Curve.ResD1> aD1 = theEval.EvaluateGridD1(theParams);
        NCollection_Array1<Geom2d_Curve.ResD2> aD2 = theEval.EvaluateGridD2(theParams);
        NCollection_Array1<Geom2d_Curve.ResD3> aD3 = theEval.EvaluateGridD3(theParams);
        Assert.Equal(aNb, aD0.Length());
        Assert.Equal(aNb, aD1.Length());
        Assert.Equal(aNb, aD2.Length());
        Assert.Equal(aNb, aD3.Length());

        for (int i = 1; i <= aNb; ++i)
        {
            double aU = theParams.Value(i);
            gp_Pnt2d aA0 = theAdaptor.EvalD0(aU);
            Geom2d_Curve.ResD1 aA1 = theAdaptor.EvalD1(aU);
            Geom2d_Curve.ResD2 aA2 = theAdaptor.EvalD2(aU);
            Geom2d_Curve.ResD3 aA3 = theAdaptor.EvalD3(aU);
            gp_Pnt2d? aC0 = theCurve?.EvalD0(aU);
            Geom2d_Curve.ResD1? aC1 = theCurve?.EvalD1(aU);
            Geom2d_Curve.ResD2? aC2 = theCurve?.EvalD2(aU);
            Geom2d_Curve.ResD3? aC3 = theCurve?.EvalD3(aU);

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

            NCollection_Array1<gp_Vec2d> aDN;
            try
            {
                aDN = theEval.EvaluateGridDN(theParams, aN);
            }
            catch (Standard_NotImplemented)
            {
                // The adaptor fallback calls the adaptor's DN above 3, which Adaptor2d_OffsetCurve doesn't have
                Assert.Null(theCurve);
                Assert.Throws<Standard_NotImplemented>(() => theAdaptor.EvalDN(theParams.Value(1), aN));
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
                gp_XY? aCurveDN = theCurve?.EvalDN(aU, aN).XY();
                gp_XY anAdaptorDN;
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
                    try
                    {
                        anAdaptorDN = theAdaptor.EvalDN(aU, aN).XY();
                    }
                    catch (Standard_Failure)
                    {
                        // Geom2dAdaptor_Curve's offset DN3 can throw (Geom2d_OffsetCurveUtils::EvaluateDN)
                        anAdaptorDN = aCurveDN!.Value;
                    }
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
                Geom2d_Curve aCurve = RandomCurve(aRnd, aKind);
                DnInfo aDn = DnInfoOf(aCurve);
                (double aFirst, double aLast) = Range(aCurve);
                NCollection_Array1<double>? aKnots = (Basis(aCurve) as Geom2d_BSplineCurve)?.InternalKnots();
                NCollection_Array1<double> aParams = SortedParams(aRnd, aFirst, aLast, 30, aKnots);

                CheckCase(aKind, new Geom2dGridEval_Curve(aCurve), new Geom2dAdaptor_Curve(aCurve), aCurve, aParams,
                          aDn);

                // Through the adaptor, and through a trimmed curve inside the range
                CheckCase(aKind, new Geom2dGridEval_Curve(new Geom2dAdaptor_Curve(aCurve)),
                          new Geom2dAdaptor_Curve(aCurve), aCurve, aParams, aDn);
                double aT1 = aFirst + 0.25 * (aLast - aFirst);
                double aT2 = aFirst + 0.75 * (aLast - aFirst);
                Geom2d_TrimmedCurve aTrimmed = new(aCurve, aT1, aT2);
                NCollection_Array1<double> aTrimParams = SortedParams(aRnd, aT1, aT2, 20, aKnots);
                CheckCase("Trimmed" + aKind, new Geom2dGridEval_Curve(aTrimmed), new Geom2dAdaptor_Curve(aTrimmed),
                          aTrimmed, aTrimParams, aDn);

                // Any adaptor other than Geom2dAdaptor_Curve: Geom2dGridEval_OtherCurve
                if (aKind is "Circle" or "BSpline" or "Bezier")
                {
                    Adaptor2d_OffsetCurve anOther = new(new Geom2dAdaptor_Curve(aCurve), 0.3);
                    Geom2dGridEval_Curve anOtherEval = new(anOther);
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
            Geom2d_BSplineCurve aCurve = RandomBSpline(aRnd, aRnd.Next(2) == 0, false);
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
            NCollection_Array1<Geom2d_Curve.ResD3> aGrid = new Geom2dGridEval_BSplineCurve(aCurve).EvaluateGridD3(aParams);
            Geom2dAdaptor_Curve anAdaptor = new(aCurve);
            for (int i = 1; i <= aParams.Length(); ++i)
            {
                Geom2d_Curve.ResD3 aRef = anAdaptor.EvalD3(aParams.Value(i));
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
        NCollection_Array1<gp_Pnt2d> aPoles = RandomPoles(new Random(3), 8);
        NCollection_Array1<double> aKnots = new(1, 6);
        NCollection_Array1<int> aMults = new(1, 6);
        for (int i = 1; i <= 6; ++i)
        {
            aKnots.SetValue(i, i - 1);
            aMults.SetValue(i, (i == 1 || i == 6) ? 4 : 1);
        }
        Geom2d_BSplineCurve aCurve = new(aPoles, aKnots, aMults, 3);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.5);
        gp_Vec2d aGrid = new Geom2dGridEval_BSplineCurve(aCurve).EvaluateGridDN(aParams, 1).Value(1);
        gp_Vec2d aPoint = aCurve.EvalDN(0.5, 1);
        Assert.True((aGrid - aPoint).Magnitude() > 1e-3); // the span [3, 4] extrapolated to 0.5

        // What OCCT computes: BSplCLib::DN with knot index 4
        BSplCLib.DN(0.5, 1, 4, 3, false, aCurve.InternalPoles(), null, aKnots, aMults, out gp_Vec2d aSpan4);
        Assert.Equal(aSpan4, aGrid);

        // In the third span the flat index (6) is the last knot's: building the span's knots reads past the
        // array (OCCT reads outside it, the port throws). The last two spans locate correctly (index past the knots).
        aParams.SetValue(1, 2.5);
        Assert.Throws<IndexOutOfRangeException>(() => new Geom2dGridEval_BSplineCurve(aCurve).EvaluateGridDN(aParams, 1));
    }

    // OCCT's Geom2dGridEval_BezierCurve and Geom2dGridEval_BSplineCurve::EvaluateGridDN return zero above the
    // degree, also for rational curves, whose derivatives of every order are generally not zero.
    [Fact]
    public void RationalDN_AboveDegreeIsZero()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        NCollection_Array1<double> aWeights = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(1, 0));
        aPoles.SetValue(2, new gp_Pnt2d(1, 1));
        aPoles.SetValue(3, new gp_Pnt2d(0, 1));
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0 / Math.Sqrt(2.0));
        aWeights.SetValue(3, 1.0);
        Geom2d_BezierCurve aBezier = new(aPoles, aWeights);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.5);
        gp_Vec2d aGrid = new Geom2dGridEval_BezierCurve(aBezier).EvaluateGridDN(aParams, 3).Value(1);
        Assert.Equal(0.0, aGrid.Magnitude());
        Assert.True(aBezier.EvalDN(0.5, 3).Magnitude() > 0.1);
    }

    // OCCT's Geom2dGridEval_Parabola builds YDir as the mirror axis turned by +90 degrees: on a parabola whose
    // frame is indirect, the grid is the parabola of the direct frame, mirrored across the axis.
    [Fact]
    public void IndirectParabola_GridIsMirrored()
    {
        Random aRnd = new(11);
        for (int aTry = 0; aTry < 20; ++aTry)
        {
            gp_Ax22d anAxes = RandomAx22d(aRnd, false);
            double aFocal = 0.1 + 5 * aRnd.NextDouble();
            Geom2d_Parabola anIndirect = new(anAxes, aFocal);
            Geom2d_Parabola aDirect = new(new gp_Ax22d(anAxes.Location(), anAxes.XDirection(), true), aFocal);
            NCollection_Array1<double> aParams = SortedParams(aRnd, -10, 10, 20, null);

            NCollection_Array1<Geom2d_Curve.ResD3> aGrid = new Geom2dGridEval_Parabola(anIndirect).EvaluateGridD3(aParams);
            for (int i = 1; i <= aParams.Length(); ++i)
            {
                double aU = aParams.Value(i);
                Geom2d_Curve.ResD3 aRef = aDirect.EvalD3(aU);
                double aScale = Math.Max(1.0, aRef.Point.XY().Modulus());
                Assert.True(aGrid.Value(i).Point.Distance(aRef.Point) <= 1e-12 * aScale);
                Assert.True((aGrid.Value(i).D1 - aRef.D1).Magnitude() <= 1e-12 * Math.Max(1.0, aRef.D1.Magnitude()));
                Assert.True((aGrid.Value(i).D2 - aRef.D2).Magnitude() <= 1e-12);
                if (Math.Abs(aU) > 1e-3)
                {
                    Assert.True(aGrid.Value(i).Point.Distance(anIndirect.EvalD0(aU)) > 1e-6);
                }
            }
        }
    }

    [Fact]
    public void NullAndEmpty()
    {
        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0);
        Geom2dGridEval.ICurveEvaluator[] aNullEvals =
        [
            new Geom2dGridEval_Line(null), new Geom2dGridEval_Circle(null), new Geom2dGridEval_Ellipse(null),
            new Geom2dGridEval_Hyperbola(null), new Geom2dGridEval_Parabola(null), new Geom2dGridEval_BezierCurve(null),
            new Geom2dGridEval_BSplineCurve(null), new Geom2dGridEval_OffsetCurve(null),
            new Geom2dGridEval_Curve((Geom2d_Curve?)null),
        ];
        foreach (Geom2dGridEval.ICurveEvaluator anEval in aNullEvals)
        {
            Assert.True(anEval.EvaluateGrid(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD1(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD2(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD3(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridDN(aParams, 1).IsEmpty());
        }

        // DN of order below 1 is empty
        Geom2dGridEval_Curve aCircle = new(new Geom2d_Circle(new gp_Ax22d(), 1.0));
        Assert.True(aCircle.EvaluateGridDN(aParams, 0).IsEmpty());

        // Results are 1-based whatever the parameters' bounds
        NCollection_Array1<double> aShifted = new(-3, -2);
        aShifted.SetValue(-3, 0.0);
        aShifted.SetValue(-2, 1.0);
        NCollection_Array1<gp_Pnt2d> aGrid = aCircle.EvaluateGrid(aShifted);
        Assert.Equal(1, aGrid.Lower());
        Assert.Equal(2, aGrid.Upper());
    }
}
