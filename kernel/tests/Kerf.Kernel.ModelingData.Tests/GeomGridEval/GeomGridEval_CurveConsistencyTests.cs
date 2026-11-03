// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests (no native binding exists for GeomGridEval): every curve grid evaluator against
/// point-by-point evaluation of the same curve, on random curves of every kind. Point evaluation (Geom curves and
/// GeomAdaptor_Curve) is bit-identical to native OCCT, so this is the oracle substitute.
/// </summary>
/// <remarks>
/// Where OCCT's grid algorithm is the point evaluation's, values are compared bit for bit: lines; the ellipse and
/// hyperbola derivatives (ElCLib's own products); the parabola's derivatives; the Bezier cache and the B-spline
/// cache (a run of two or more parameters in a span) against GeomAdaptor_Curve, which uses the same caches;
/// Bezier DN (BSplCLib::DN on flat knots, as Geom_BezierCurve); the adaptor fallback. Elsewhere OCCT's grid
/// formulas associate differently from ElCLib (the circle's R * (cos X + sin Y) against R cos X + R sin Y, the
/// conic points adding the centre first) or evaluate a lone parameter of a span directly on flat knots, and the
/// values agree to 1e-12 relative. OCCT's errors are pinned separately (occt-issues.md): B-spline DN on the wrong
/// span, rational DN above the degree, and the offset DN read out of range after a failed offset.
/// </remarks>
public class GeomGridEval_CurveConsistencyTest(ITestOutputHelper theOutput)
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

    private void Compare(string theKey, Xyz theGrid, Xyz? theCurve, Xyz theAdaptor)
    {
        Stats aStats = StatsOf(theKey);
        for (int aCoord = 1; aCoord <= 3; ++aCoord)
        {
            double aG = theGrid.Coord(aCoord);
            double aA = theAdaptor.Coord(aCoord);
            double aScale = Math.Max(1.0, Math.Max(theGrid.Modulus(), theAdaptor.Modulus()));
            bool isExactA = aG == aA;
            double aRel = Math.Abs(aG - aA) / aScale;
            bool isExactC = false;
            if (theCurve is Xyz aCurve)
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

    private static Direction3D RandomDir(Random theRnd)
    {
        while (true)
        {
            double x = theRnd.NextDouble() * 2 - 1, y = theRnd.NextDouble() * 2 - 1, z = theRnd.NextDouble() * 2 - 1;
            double n = Math.Sqrt(x * x + y * y + z * z);
            if (n > 0.1 && n <= 1.0)
            {
                return new Direction3D(x, y, z);
            }
        }
    }

    private static Point3D RandomPnt(Random theRnd, double theScale) =>
        new(theRnd.NextDouble() * 2 * theScale - theScale, theRnd.NextDouble() * 2 * theScale - theScale,
            theRnd.NextDouble() * 2 * theScale - theScale);

    private static Frame3D RandomAx2(Random theRnd)
    {
        Direction3D aN = RandomDir(theRnd);
        Direction3D aX;
        do
        {
            aX = RandomDir(theRnd);
        } while (Math.Abs(aN.Dot(aX)) > 0.9);
        return new Frame3D(RandomPnt(theRnd, 10.0), aN, aX);
    }

    private static NCollection_Array1<Point3D> RandomPoles(Random theRnd, int theNb)
    {
        NCollection_Array1<Point3D> aPoles = new(1, theNb);
        for (int i = 1; i <= theNb; ++i)
        {
            aPoles.SetValue(i, new Point3D(i + theRnd.NextDouble() * 4 - 2, theRnd.NextDouble() * 10 - 5,
                                          theRnd.NextDouble() * 10 - 5));
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
    private static BSplineCurve3D RandomBSpline(Random theRnd, bool theRational, bool thePeriodic,
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
        NCollection_Array1<Point3D> aPoles = RandomPoles(theRnd, aNbPoles);
        BSplineCurve3D aCurve = theRational
            ? new BSplineCurve3D(aPoles, RandomWeights(theRnd, aNbPoles), aKnots, aMults, aDeg)
            : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
        return thePeriodic ? aCurve.WithPeriodic() : aCurve;
    }

    private static BezierCurve3D RandomBezier(Random theRnd, bool theRational)
    {
        int aNbPoles = theRnd.Next(2, 10);
        NCollection_Array1<Point3D> aPoles = RandomPoles(theRnd, aNbPoles);
        return theRational ? new BezierCurve3D(aPoles, RandomWeights(theRnd, aNbPoles))
                           : new BezierCurve3D(aPoles);
    }

    private static EllipseCurve3D RandomEllipse(Random theRnd)
    {
        double aMin = 0.1 + 5 * theRnd.NextDouble();
        return new EllipseCurve3D(RandomAx2(theRnd), aMin + 5 * theRnd.NextDouble(), aMin);
    }

    // An offset direction away from the basis curve's plane (or anywhere for a space curve)
    private static OffsetCurve3D RandomOffset(Random theRnd, Curve3D theBasis, double theScale)
    {
        Direction3D aDir = theBasis is ConicCurve3D aConic ? aConic.Axis.Direction : RandomDir(theRnd);
        return new OffsetCurve3D(theBasis, (theRnd.NextDouble() * 2 - 1) * theScale, aDir);
    }

    private static Curve3D RandomCurve(Random theRnd, string theKind)
    {
        return theKind switch
        {
            "Line" => new LineCurve3D(RandomPnt(theRnd, 10.0), RandomDir(theRnd)),
            "Circle" => new CircleCurve3D(RandomAx2(theRnd), 0.1 + 10 * theRnd.NextDouble()),
            "Ellipse" => RandomEllipse(theRnd),
            "Hyperbola" => new HyperbolaCurve3D(RandomAx2(theRnd), 3 + 5 * theRnd.NextDouble(),
                                              0.1 + 3 * theRnd.NextDouble()),
            "Parabola" => new ParabolaCurve3D(RandomAx2(theRnd), 0.1 + 5 * theRnd.NextDouble()),
            "Bezier" => RandomBezier(theRnd, false),
            "RationalBezier" => RandomBezier(theRnd, true),
            "BSpline" => RandomBSpline(theRnd, false, false),
            "RationalBSpline" => RandomBSpline(theRnd, true, false),
            "PeriodicBSpline" => RandomBSpline(theRnd, theRnd.Next(2) == 0, true),
            "OffsetCircle" => RandomOffset(theRnd, RandomCurve(theRnd, "Circle"), 1.0),
            "OffsetEllipse" => RandomOffset(theRnd, RandomCurve(theRnd, "Ellipse"), 1.0),
            "OffsetBezier" => RandomOffset(theRnd, RandomBezier(theRnd, theRnd.Next(2) == 0), 0.5),
            "OffsetBSpline" => RandomOffset(theRnd, RandomBSpline(theRnd, theRnd.Next(2) == 0, false, true), 0.5),
            _ => throw new ArgumentException(theKind),
        };
    }

    private static (double First, double Last) Range(Curve3D theCurve)
    {
        double aFirst = theCurve.FirstParameter;
        double aLast = theCurve.LastParameter;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast))
        {
            return Basis(theCurve) is HyperbolaCurve3D ? (-3.0, 3.0) : (-10.0, 10.0);
        }
        return (aFirst, aLast);
    }

    private static Curve3D Basis(Curve3D theCurve)
    {
        Curve3D aBasis = theCurve;
        while (true)
        {
            if (aBasis is TrimmedCurve3D aTrimmed)
            {
                aBasis = aTrimmed.BasisCurve;
            }
            else if (aBasis is OffsetCurve3D anOffset)
            {
                aBasis = anOffset.BasisCurve;
            }
            else
            {
                return aBasis;
            }
        }
    }

    private static DnInfo DnInfoOf(Curve3D theCurve)
    {
        Curve3D aCurve = theCurve;
        while (aCurve is TrimmedCurve3D aTrimmed)
        {
            aCurve = aTrimmed.BasisCurve;
        }
        // An offset curve's DN above 3 is its basis's DN, in the grid as in Geom_OffsetCurve
        int aFirstDelegatedN = aCurve is OffsetCurve3D ? 4 : 1;
        Curve3D aBasis = Basis(theCurve);
        return aBasis switch
        {
            BSplineCurve3D aBS => new DnInfo(true, aBS.IsRational, aBS.Degree, aFirstDelegatedN),
            BezierCurve3D aBz => new DnInfo(false, aBz.IsRational, aBz.Degree, aFirstDelegatedN),
            _ => new DnInfo(false, false, int.MaxValue, aFirstDelegatedN),
        };
    }

    private void CheckCase(string theKind, GeomGridEvalGrid.ICurveEvaluator theEval, CurveAdaptorBase theAdaptor,
                           Curve3D? theCurve, NCollection_Array1<double> theParams, DnInfo theDn)
    {
        int aNb = theParams.Length();
        NCollection_Array1<Point3D> aD0 = theEval.EvaluateGrid(theParams);
        NCollection_Array1<Curve3D.ResD1> aD1 = theEval.EvaluateGridD1(theParams);
        NCollection_Array1<Curve3D.ResD2> aD2 = theEval.EvaluateGridD2(theParams);
        NCollection_Array1<Curve3D.ResD3> aD3 = theEval.EvaluateGridD3(theParams);
        Assert.Equal(aNb, aD0.Length());
        Assert.Equal(aNb, aD1.Length());
        Assert.Equal(aNb, aD2.Length());
        Assert.Equal(aNb, aD3.Length());

        for (int i = 1; i <= aNb; ++i)
        {
            double aU = theParams.Value(i);
            Point3D aA0 = theAdaptor.EvalD0(aU);
            Curve3D.ResD1 aA1 = theAdaptor.EvalD1(aU);
            Curve3D.ResD2 aA2 = theAdaptor.EvalD2(aU);
            Curve3D.ResD3 aA3 = theAdaptor.EvalD3(aU);
            Point3D? aC0 = theCurve?.EvalD0(aU);
            Curve3D.ResD1? aC1 = theCurve?.EvalD1(aU);
            Curve3D.ResD2? aC2 = theCurve?.EvalD2(aU);
            Curve3D.ResD3? aC3 = theCurve?.EvalD3(aU);

            Compare(theKind + " D0", aD0.Value(i).XYZ(), aC0?.XYZ(), aA0.XYZ());
            Compare(theKind + " D1.P", aD1.Value(i).Point.XYZ(), aC1?.Point.XYZ(), aA1.Point.XYZ());
            Compare(theKind + " D1.D1", aD1.Value(i).D1.XYZ(), aC1?.D1.XYZ(), aA1.D1.XYZ());
            Compare(theKind + " D2.P", aD2.Value(i).Point.XYZ(), aC2?.Point.XYZ(), aA2.Point.XYZ());
            Compare(theKind + " D2.D1", aD2.Value(i).D1.XYZ(), aC2?.D1.XYZ(), aA2.D1.XYZ());
            Compare(theKind + " D2.D2", aD2.Value(i).D2.XYZ(), aC2?.D2.XYZ(), aA2.D2.XYZ());
            Compare(theKind + " D3.P", aD3.Value(i).Point.XYZ(), aC3?.Point.XYZ(), aA3.Point.XYZ());
            Compare(theKind + " D3.D1", aD3.Value(i).D1.XYZ(), aC3?.D1.XYZ(), aA3.D1.XYZ());
            Compare(theKind + " D3.D2", aD3.Value(i).D2.XYZ(), aC3?.D2.XYZ(), aA3.D2.XYZ());
            Compare(theKind + " D3.D3", aD3.Value(i).D3.XYZ(), aC3?.D3.XYZ(), aA3.D3.XYZ());
        }

        for (int aN = 1; aN <= 6; ++aN)
        {
            bool isDelegated = aN >= theDn.FirstDelegatedN;
            if (isDelegated && theDn.IsBSpline && aN <= theDn.Degree)
            {
                // OCCT: wrong span or out of range; pinned by BSplineDN_UsesFlatSpanIndexWithKnots
                continue;
            }

            NCollection_Array1<Vector3D> aDN = theEval.EvaluateGridDN(theParams, aN);
            Assert.Equal(aNb, aDN.Length());

            if (isDelegated && theDn.IsRational && aN > theDn.Degree)
            {
                // OCCT: zero above the degree, although a rational curve's derivatives aren't
                for (int i = 1; i <= aNb; ++i)
                {
                    Assert.Equal(0.0, aDN.Value(i).Magnitude);
                }
                continue;
            }

            for (int i = 1; i <= aNb; ++i)
            {
                double aU = theParams.Value(i);
                Xyz? aCurveDN = theCurve?.EvalDN(aU, aN).XYZ();
                Xyz anAdaptorDN;
                if (theCurve is null && aN <= 3)
                {
                    // The adaptor fallback evaluates DN1-DN3 through the adaptor's D1-D3
                    anAdaptorDN = aN switch
                    {
                        1 => theAdaptor.EvalD1(aU).D1.XYZ(),
                        2 => theAdaptor.EvalD2(aU).D2.XYZ(),
                        _ => theAdaptor.EvalD3(aU).D3.XYZ(),
                    };
                }
                else
                {
                    try
                    {
                        anAdaptorDN = theAdaptor.EvalDN(aU, aN).XYZ();
                    }
                    catch (global::Kerf.Engine.Foundation.FailureException) when (aCurveDN is not null)
                    {
                        // GeomAdaptor_Curve's offset DN3 can throw (Geom_OffsetCurveUtils::EvaluateDN passes one
                        // variable as both D1 and D2 to EvaluateD3)
                        anAdaptorDN = aCurveDN.Value;
                    }
                }
                Compare(theKind + " DN" + aN, aDN.Value(i).XYZ(), aCurveDN, anAdaptorDN);
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
                Curve3D aCurve = RandomCurve(aRnd, aKind);
                DnInfo aDn = DnInfoOf(aCurve);
                (double aFirst, double aLast) = Range(aCurve);
                NCollection_Array1<double>? aKnots = (Basis(aCurve) as BSplineCurve3D)?.InternalKnots();
                NCollection_Array1<double> aParams = SortedParams(aRnd, aFirst, aLast, 30, aKnots);

                CheckCase(aKind, new CurveGrid(aCurve), new CurveAdaptor(aCurve), aCurve, aParams, aDn);

                // Through the adaptor, and through a trimmed curve inside the range
                CheckCase(aKind, new CurveGrid(new CurveAdaptor(aCurve)), new CurveAdaptor(aCurve),
                          aCurve, aParams, aDn);
                double aT1 = aFirst + 0.25 * (aLast - aFirst);
                double aT2 = aFirst + 0.75 * (aLast - aFirst);
                TrimmedCurve3D aTrimmed = new(aCurve, aT1, aT2);
                NCollection_Array1<double> aTrimParams = SortedParams(aRnd, aT1, aT2, 20, aKnots);
                CheckCase("Trimmed" + aKind, new CurveGrid(aTrimmed), new CurveAdaptor(aTrimmed),
                          aTrimmed, aTrimParams, aDn);

                // Any adaptor other than GeomAdaptor_Curve: GeomGridEval_OtherCurve, point by point
                if (aKind is "Circle" or "BSpline" or "Bezier" or "OffsetEllipse")
                {
                    Transform3D aTrsf = new Transform3D().WithRotation(new Axis3D(RandomPnt(aRnd, 5.0), RandomDir(aRnd)), aRnd.NextDouble() * 6);
                    TransformedCurve anOther = new(aCurve, aTrsf);
                    CurveGrid anOtherEval = new(anOther);
                    Assert.Equal(anOther.GetType(), anOtherEval.GetType());
                    CheckCase("Other" + aKind, anOtherEval, anOther, null, aParams,
                              new DnInfo(false, false, int.MaxValue, 1));
                }
            }
        }

        // An iso-curve of a sphere: another adaptor for the fallback
        for (int aTry = 0; aTry < 20; ++aTry)
        {
            SphericalSurface aSphere = new(new CoordinateSystem(RandomAx2(aRnd)), 0.5 + 5 * aRnd.NextDouble());
            IsoCurve anIso = new(new SurfaceAdaptor(aSphere), IsoType.IsoV,
                                           aRnd.NextDouble() * 2 - 1);
            NCollection_Array1<double> aParams = SortedParams(aRnd, 0.0, 2 * M_PI, 20, null);
            CheckCase("OtherIso", new CurveGrid(anIso), anIso, null, aParams,
                      new DnInfo(false, false, int.MaxValue, 1));
        }

        foreach (KeyValuePair<string, Stats> anItem in myStats.OrderBy(theKv => theKv.Key, StringComparer.Ordinal))
        {
            Stats aS = anItem.Value;
            theOutput.WriteLine($"{anItem.Key,-28} n={aS.Count,6} exactCurve={aS.ExactCurve,6} "
                                + $"exactAdaptor={aS.ExactAdaptor,6} exactEither={aS.ExactEither,6} maxRel={aS.MaxRel:E2}");
        }

        List<string> aFailures = [];
        foreach (KeyValuePair<string, Stats> anItem in myStats)
        {
            string aKey = anItem.Key;
            Stats aS = anItem.Value;
            if (aS.Count == 0 || aS.MaxRel > THE_REL_TOL)
            {
                aFailures.Add($"{aKey}: maxRel {aS.MaxRel:E3}");
                continue;
            }
            Expectation anExpected = ExpectationOf(aKey);
            bool isOk = anExpected switch
            {
                Expectation.ExactCurveAndAdaptor => aS.ExactCurve == aS.Count && aS.ExactAdaptor == aS.Count,
                Expectation.ExactCurve => aS.ExactCurve == aS.Count,
                Expectation.ExactAdaptor => aS.ExactAdaptor == aS.Count,
                _ => true,
            };
            if (!isOk)
            {
                aFailures.Add($"{aKey}: expected {anExpected}");
            }
        }
        Assert.Empty(aFailures);
    }

    private enum Expectation
    {
        Tolerance,
        ExactCurve,
        ExactAdaptor,
        ExactCurveAndAdaptor,
    }

    // Bit for bit where OCCT's grid algorithm is the point evaluation's
    private static Expectation ExpectationOf(string theKey)
    {
        string aKind = theKey[..theKey.IndexOf(' ')];
        string aWhat = theKey[(theKey.IndexOf(' ') + 1)..];
        bool isPoint = aWhat is "D0" or "D1.P" or "D2.P" or "D3.P";
        bool isDN = aWhat.StartsWith("DN", StringComparison.Ordinal);
        string aBase = aKind.Replace("Trimmed", "", StringComparison.Ordinal);
        if (aBase == "Line")
        {
            return Expectation.ExactCurveAndAdaptor;
        }
        if (aBase.StartsWith("Other", StringComparison.Ordinal))
        {
            // Point by point through the adaptor
            return Expectation.ExactAdaptor;
        }
        if ((aBase is "Ellipse" or "Hyperbola" or "OffsetEllipse") && !isPoint && aWhat != "DN3")
        {
            // ElCLib's own products; the offset of an ellipse inherits them (DN3 is checked against the curve
            // only where GeomAdaptor_Curve's offset DN3 is wrong)
            return Expectation.ExactCurve;
        }
        if (aBase == "Parabola" && !isPoint && aWhat is not ("D1.D1" or "D2.D1" or "D3.D1" or "DN1"))
        {
            // D2 = (1/2F) X and D3 = 0 as ElCLib; ElCLib's D1 divides U by 2F where the grid multiplies by 1/(2F)
            return Expectation.ExactCurve;
        }
        if (aBase is "Bezier" or "RationalBezier")
        {
            // One cache over the single span, as GeomAdaptor_Curve's; DN by BSplCLib::DN on flat knots, as
            // Geom_BezierCurve's
            return isDN ? Expectation.ExactCurve : Expectation.ExactAdaptor;
        }
        return Expectation.Tolerance;
    }

    [Fact]
    public void BSplineGrid_CacheMatchesAdaptorBitForBit()
    {
        // A run of at least 2 parameters in one span uses BSplCLib_Cache, as GeomAdaptor_Curve does: parameters
        // strictly inside the spans of a non-periodic curve give the adaptor's values exactly.
        Random aRnd = new(7);
        for (int aTry = 0; aTry < 50; ++aTry)
        {
            BSplineCurve3D aCurve = RandomBSpline(aRnd, aRnd.Next(2) == 0, false);
            NCollection_Array1<double> aKnots = aCurve.InternalKnots();
            List<double> aList = [];
            for (int k = aKnots.Lower(); k < aKnots.Upper(); ++k)
            {
                double aA = aKnots.Value(k);
                double aB = aKnots.Value(k + 1);
                for (int j = 1; j <= 2; ++j)
                {
                    aList.Add(aA + (aB - aA) * j / 3.0);
                }
            }
            NCollection_Array1<double> aParams = NCollection_Array1<double>.From(aList.ToArray());
            NCollection_Array1<Curve3D.ResD3> aGrid = new BSplineCurveGrid(aCurve).EvaluateGridD3(aParams);
            NCollection_Array1<Point3D> aGrid0 = new BSplineCurveGrid(aCurve).EvaluateGrid(aParams);
            CurveAdaptor anAdaptor = new(aCurve);
            for (int i = 1; i <= aParams.Length(); ++i)
            {
                Curve3D.ResD3 aRef = anAdaptor.EvalD3(aParams.Value(i));
                Assert.Equal(anAdaptor.EvalD0(aParams.Value(i)), aGrid0.Value(i));
                Assert.Equal(aRef.Point, aGrid.Value(i).Point);
                Assert.Equal(aRef.D1, aGrid.Value(i).D1);
                Assert.Equal(aRef.D2, aGrid.Value(i).D2);
                Assert.Equal(aRef.D3, aGrid.Value(i).D3);
            }
        }
    }

    // OCCT's GeomGridEval_BSplineCurve::EvaluateGridDN locates the span in the flat knots, then passes that flat
    // index with the knots and multiplicities to BSplCLib::DN, which takes it as a knot index: if it is a valid
    // knot index it evaluates that span's polynomial at U, otherwise it reads outside the knots.
    [Fact]
    public void BSplineDN_UsesFlatSpanIndexWithKnots()
    {
        // Cubic, knots 0..5 (flat index of the first span is 4, a valid knot index)
        NCollection_Array1<Point3D> aPoles = RandomPoles(new Random(3), 8);
        NCollection_Array1<double> aKnots = new(1, 6);
        NCollection_Array1<int> aMults = new(1, 6);
        for (int i = 1; i <= 6; ++i)
        {
            aKnots.SetValue(i, i - 1);
            aMults.SetValue(i, (i == 1 || i == 6) ? 4 : 1);
        }
        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 3);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.5);
        Vector3D aGrid = new BSplineCurveGrid(aCurve).EvaluateGridDN(aParams, 1).Value(1);
        Vector3D aPoint = aCurve.EvalDN(0.5, 1);
        Assert.True((aGrid - aPoint).Magnitude > 1e-3); // the span [3, 4] extrapolated to 0.5

        // What OCCT computes: BSplCLib::DN with knot index 4
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(0.5, 1, 4, 3, false, aCurve.InternalPoles(), null, aKnots, aMults, out Vector3D aSpan4);
        Assert.Equal(aSpan4, aGrid);

        // Through the dispatcher too
        Assert.Equal(aGrid, new CurveGrid(aCurve).EvaluateGridDN(aParams, 1).Value(1));

        // In the third span the flat index (6) is the last knot's: building the span's knots reads past the array
        // (OCCT reads outside it, the port throws).
        aParams.SetValue(1, 2.5);
        Assert.Throws<IndexOutOfRangeException>(() => new BSplineCurveGrid(aCurve).EvaluateGridDN(aParams, 1));
    }

    // OCCT's GeomGridEval_BezierCurve and GeomGridEval_BSplineCurve::EvaluateGridDN return zero above the degree,
    // also for rational curves, whose derivatives of every order are generally not zero.
    [Fact]
    public void RationalDN_AboveDegreeIsZero()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        NCollection_Array1<double> aWeights = new(1, 3);
        aPoles.SetValue(1, new Point3D(1, 0, 0));
        aPoles.SetValue(2, new Point3D(1, 1, 0));
        aPoles.SetValue(3, new Point3D(0, 1, 0));
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0 / Math.Sqrt(2.0));
        aWeights.SetValue(3, 1.0);
        BezierCurve3D aBezier = new(aPoles, aWeights);

        NCollection_Array1<double> aParams = new(1, 1);
        aParams.SetValue(1, 0.5);
        Vector3D aGrid = new BezierCurveGrid(aBezier).EvaluateGridDN(aParams, 3).Value(1);
        Assert.Equal(0.0, aGrid.Magnitude);
        Assert.True(aBezier.EvalDN(0.5, 3).Magnitude > 0.1);

        // The same quarter circle as a rational B-spline
        NCollection_Array1<double> aKnots = new(1, 2);
        NCollection_Array1<int> aMults = new(1, 2);
        aKnots.SetValue(1, 0.0);
        aKnots.SetValue(2, 1.0);
        aMults.SetValue(1, 3);
        aMults.SetValue(2, 3);
        BSplineCurve3D aBSpline = new(aPoles, aWeights, aKnots, aMults, 2);
        Vector3D aGridBS = new BSplineCurveGrid(aBSpline).EvaluateGridDN(aParams, 3).Value(1);
        Assert.Equal(0.0, aGridBS.Magnitude);
        Assert.True(aBSpline.EvalDN(0.5, 3).Magnitude > 0.1);
    }

    // OCCT's GeomGridEval_OffsetCurve::EvaluateGridDN reads the D1, D2 or D3 grid without checking that it is not
    // empty; it is empty when the offset can't be computed at one parameter (a basis tangent parallel to the
    // offset direction). Undefined in OCCT; the port's range check throws Standard_OutOfRange.
    [Fact]
    public void OffsetDN_ReadsEmptyGridAfterFailedOffset()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        OffsetCurve3D anOffset = new(aLine, 1.0, Constants.DZ());
        OffsetCurveGrid anEval = new(anOffset);

        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0);
        Assert.True(anEval.EvaluateGrid(aParams).IsEmpty());
        Assert.True(anEval.EvaluateGridD1(aParams).IsEmpty());
        Assert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anEval.EvaluateGridDN(aParams, 1));
        Assert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anEval.EvaluateGridDN(aParams, 3));
    }

    // GeomGridEval_Parabola takes YDir from the gp_Ax2, which always gives the frame's own Y direction: unlike
    // Geom2dGridEval_Parabola (occt-issues.md), the 3D grid matches the parabola for any frame.
    [Fact]
    public void Parabola_AnyFrameMatchesPointEvaluation()
    {
        Random aRnd = new(11);
        for (int aTry = 0; aTry < 20; ++aTry)
        {
            ParabolaCurve3D aParab = new(RandomAx2(aRnd), 0.1 + 5 * aRnd.NextDouble());
            NCollection_Array1<double> aParams = SortedParams(aRnd, -10, 10, 20, null);
            NCollection_Array1<Curve3D.ResD3> aGrid = new ParabolaGrid(aParab).EvaluateGridD3(aParams);
            for (int i = 1; i <= aParams.Length(); ++i)
            {
                Curve3D.ResD3 aRef = aParab.EvalD3(aParams.Value(i));
                Assert.True(aGrid.Value(i).Point.DistanceTo(aRef.Point) <= 1e-12 * Math.Max(1.0, aRef.Point.XYZ().Modulus()));
                Assert.True((aGrid.Value(i).D1 - aRef.D1).Magnitude <= 1e-12 * Math.Max(1.0, aRef.D1.Magnitude));
                Assert.Equal(aRef.D2, aGrid.Value(i).D2);
                Assert.Equal(aRef.D3, aGrid.Value(i).D3);
            }
        }
    }

    // OCCT's GeomGridEval_Parabola divides by the focal length; a parabola of focal length 0 (allowed by
    // Geom_Parabola, a line that ElCLib evaluates as Location + U * XDir) gives infinite and NaN coordinates.
    [Fact]
    public void ZeroFocalParabola_GridIsNotFinite()
    {
        ParabolaCurve3D aParab = new(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 0.0);
        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.5);

        Assert.Equal(new Point3D(2.5, 2, 3), aParab.EvalD0(1.5));
        NCollection_Array1<Point3D> aGrid = new ParabolaGrid(aParab).EvaluateGrid(aParams);
        Assert.True(double.IsNaN(aGrid.Value(1).X)); // 0 * inf
        Assert.True(double.IsInfinity(aGrid.Value(2).X));
        Assert.True(double.IsInfinity(new ParabolaGrid(aParab).EvaluateGridDN(aParams, 2).Value(1).X));
    }

    [Fact]
    public void NullAndEmpty()
    {
        NCollection_Array1<double> aParams = new(1, 2);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0);
        GeomGridEvalGrid.ICurveEvaluator[] aNullEvals =
        [
            new LineGrid(null), new CircleGrid(null), new EllipseGrid(null),
            new HyperbolaGrid(null), new ParabolaGrid(null), new BezierCurveGrid(null),
            new BSplineCurveGrid(null), new OffsetCurveGrid(null),
            new CurveGrid((Curve3D?)null),
        ];
        foreach (GeomGridEvalGrid.ICurveEvaluator anEval in aNullEvals)
        {
            Assert.True(anEval.EvaluateGrid(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD1(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD2(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridD3(aParams).IsEmpty());
            Assert.True(anEval.EvaluateGridDN(aParams, 1).IsEmpty());
        }
        Assert.Equal(CurveKind.OtherCurve, new CurveGrid((Curve3D?)null).GetType());

        // DN of order below 1 is empty
        CurveGrid aCircle = new(new CircleCurve3D(new Frame3D(), 1.0));
        Assert.True(aCircle.EvaluateGridDN(aParams, 0).IsEmpty());

        // Results are 1-based whatever the parameters' bounds
        NCollection_Array1<double> aShifted = new(-3, -2);
        aShifted.SetValue(-3, 0.0);
        aShifted.SetValue(-2, 1.0);
        NCollection_Array1<Point3D> aGrid = aCircle.EvaluateGrid(aShifted);
        Assert.Equal(1, aGrid.Lower());
        Assert.Equal(2, aGrid.Upper());
    }
}
