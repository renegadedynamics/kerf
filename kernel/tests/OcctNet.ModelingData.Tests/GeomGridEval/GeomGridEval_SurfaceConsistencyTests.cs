// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The grid evaluators of B-spline, offset and other surfaces, and the GeomGridEval_Surface dispatcher (trimmed
/// surfaces, GeomAdaptor_TransformedSurface, the swept-surface adaptors), against point evaluation
/// (Geom_Surface.EvalD* / EvalDN and the adaptors) on random geometry: bit for bit where OCCT's grid algorithm is
/// the point evaluation's, to a relative tolerance where it differs (see <see cref="Expectation"/>). The analytic,
/// Bezier and swept evaluators are covered by <see cref="GeomGridEval_AnalyticSurfaceConsistencyTest"/>.
/// </summary>
public class GeomGridEval_SurfaceConsistencyTest
{
    private readonly ITestOutputHelper myOut;

    public GeomGridEval_SurfaceConsistencyTest(ITestOutputHelper theOut) => myOut = theOut;

    private sealed class Stats
    {
        public double MaxRel;
        public long Count;
        public long Identical;
    }

    private readonly SortedDictionary<string, Stats> myStats = new(StringComparer.Ordinal);
    private bool myRationalAboveDegreeNonZero;

    /// <summary>
    /// What each comparison must give: null for bit-identical, else the relative tolerance.
    /// <list type="bullet">
    /// <item>B-spline D0-D2: a (U span, V span) patch holding two grid points or more ("[cache]") is evaluated on a
    /// BSplSLib_Cache built for it, through D0Local/D1Local/D2Local on (U - SpanMid) / SpanHalfLen. Geom evaluates
    /// through BSplSLib, and GeomAdaptor_Surface through the cache's D0/D1/D2, which compute the local parameters
    /// and scale the derivatives differently (inverse span lengths): last-bit differences against both, in OCCT as
    /// in the port. A lone point in its patch ("[direct]") is evaluated by BSplSLib on the flat knots, bit-identical
    /// to Geom_BSplineSurface (BSplSLib on knots and multiplicities: the same span poles and knots).</item>
    /// <item>B-spline D3 and DN: BSplSLib::D3/DN, as Geom_BSplineSurface and GeomAdaptor_Surface evaluate them:
    /// bit-identical to both. Rational DN above a degree is OCCT's zero (checked separately).</item>
    /// <item>Offset surfaces (general path): D0, D1 and DN apply Geom_OffsetSurfaceUtils to the basis grid D1/D2,
    /// whose B-spline cache values differ in the last bits (above); D2 uses the basis D3 grid (BSplSLib::D3, or the
    /// Bezier cache) and D3 evaluates point by point through the basis adaptor: bit-identical to both. An offset of
    /// a Bezier surface is bit-identical to GeomAdaptor_Surface throughout (both evaluate the basis on the same
    /// single-span cache). Offsets with an equivalent elementary surface evaluate that surface's grid (OCCT's
    /// analytic formulas: last bits).</item>
    /// <item>Other surfaces: point evaluation through the object given: bit-identical.</item>
    /// <item>GeomAdaptor_TransformedSurface: the untransformed grid, each value transformed as the adaptor
    /// transforms its point evaluations: bit-identical where the untransformed grid is (B-spline D3 and DN).</item>
    /// <item>The swept-surface adaptors: rebuilt Geom swept surfaces, evaluated on their basis curve's grid.</item>
    /// </list>
    /// </summary>
    private static double? Expectation(string theKey)
    {
        bool isGeom = theKey.Contains(" vs Geom ", StringComparison.Ordinal);
        string aKind = theKey[..theKey.IndexOf(' ')];
        string aQty = theKey[(theKey.IndexOf(isGeom ? " vs Geom " : " vs Adaptor ", StringComparison.Ordinal)
                              + (isGeom ? 9 : 12))..];
        bool isDirect = aQty.Contains("[direct]", StringComparison.Ordinal);
        bool isD3OrDN = aQty.StartsWith("D3", StringComparison.Ordinal) || aQty.StartsWith("DN", StringComparison.Ordinal);
        switch (aKind)
        {
            case "BSpline":
            case "BSplineRational":
            case "BSplinePeriodic":
            case "Trimmed":
                return isD3OrDN || (isDirect && isGeom) ? null : 1e-12;
            case "OffsetBSpline":
                return aQty.StartsWith("D2", StringComparison.Ordinal) || aQty.StartsWith("D3", StringComparison.Ordinal)
                           ? null
                           : 1e-12;
            case "OffsetBezier":
                return !isGeom || aQty.StartsWith("D2", StringComparison.Ordinal)
                       || aQty.StartsWith("D3", StringComparison.Ordinal)
                           ? null
                           : 1e-12;
            case "OtherGeom":
            case "OtherAdaptor":
                return null;
            case "TransformedBSpline":
                return isD3OrDN ? null : 1e-12;
            default: // OffsetCylinder, OffsetSphere, TransformedCylinder, RevolutionAdaptor, ExtrusionAdaptor
                return 1e-12;
        }
    }

    private void Compare(string theKey, Xyz theGrid, Xyz theRef)
    {
        if (!myStats.TryGetValue(theKey, out Stats? aStats))
        {
            aStats = new Stats();
            myStats[theKey] = aStats;
        }
        aStats.Count++;
        if (theGrid.X().Equals(theRef.X()) && theGrid.Y().Equals(theRef.Y()) && theGrid.Z().Equals(theRef.Z()))
        {
            aStats.Identical++;
            return;
        }
        double aRel = (theGrid - theRef).Modulus() / Math.Max(1.0, theRef.Modulus());
        aStats.MaxRel = Math.Max(aStats.MaxRel, aRel);
    }

    private static double Uniform(Random theRnd, double theA, double theB) => theA + (theB - theA) * theRnd.NextDouble();

    private static Point3D RandomPnt(Random theRnd, double theScale) =>
        new(Uniform(theRnd, -theScale, theScale), Uniform(theRnd, -theScale, theScale), Uniform(theRnd, -theScale, theScale));

    private static Direction3D RandomDir(Random theRnd)
    {
        while (true)
        {
            double x = Uniform(theRnd, -1, 1), y = Uniform(theRnd, -1, 1), z = Uniform(theRnd, -1, 1);
            double n = Math.Sqrt(x * x + y * y + z * z);
            if (n > 0.1 && n <= 1.0)
            {
                return new Direction3D(x, y, z);
            }
        }
    }

    private static CoordinateSystem RandomAx3(Random theRnd)
    {
        Direction3D aN = RandomDir(theRnd);
        Direction3D aX = RandomDir(theRnd);
        while (aN.IsParallel(aX, 0.1))
        {
            aX = RandomDir(theRnd);
        }
        return new CoordinateSystem(RandomPnt(theRnd, 5.0), aN, aX);
    }

    // Knots and multiplicities of one direction: clamped (ends of multiplicity Degree + 1), or periodic (equal
    // end multiplicities of at most Degree). Returns the number of poles.
    private static int RandomKnots(Random theRnd,
                                   int theDeg,
                                   bool thePeriodic,
                                   out NCollection_Array1<double> theKnots,
                                   out NCollection_Array1<int> theMults)
    {
        int aNbKnots = thePeriodic ? theRnd.Next(theDeg + 2, theDeg + 5) : theRnd.Next(2, 5);
        theKnots = new NCollection_Array1<double>(1, aNbKnots);
        theMults = new NCollection_Array1<int>(1, aNbKnots);
        double aK = Uniform(theRnd, -2, 2);
        for (int i = 1; i <= aNbKnots; ++i)
        {
            theKnots[i] = aK;
            aK += Uniform(theRnd, 0.3, 2.0);
            theMults[i] = theRnd.Next(1, theDeg + 1);
        }
        if (thePeriodic)
        {
            theMults[aNbKnots] = theMults[1];
            int aNb = 0;
            for (int i = 1; i < aNbKnots; ++i)
            {
                aNb += theMults[i];
            }
            return aNb;
        }
        theMults[1] = theDeg + 1;
        theMults[aNbKnots] = theDeg + 1;
        int aSum = 0;
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aSum += theMults[i];
        }
        return aSum - theDeg - 1;
    }

    private static Geom_BSplineSurface RandomBSplineSurface(Random theRnd, bool theRational, bool thePeriodic)
    {
        int aUDeg = theRnd.Next(1, 5);
        int aVDeg = theRnd.Next(1, 5);
        bool aUPer = thePeriodic && theRnd.Next(3) != 0;
        bool aVPer = thePeriodic && (!aUPer || theRnd.Next(2) == 0);
        int aNbU = RandomKnots(theRnd, aUDeg, aUPer, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
        int aNbV = RandomKnots(theRnd, aVDeg, aVPer, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
        NCollection_Array2<Point3D> aPoles = new(1, aNbU, 1, aNbV);
        NCollection_Array2<double> aWeights = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                aPoles[i, j] = new Point3D(i + Uniform(theRnd, -0.4, 0.4), j + Uniform(theRnd, -0.4, 0.4),
                                          Uniform(theRnd, -2.0, 2.0));
                aWeights[i, j] = Uniform(theRnd, 0.5, 2.0);
            }
        }
        return theRational
                   ? new Geom_BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer)
                   : new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer);
    }

    private static Geom_BezierSurface RandomBezierSurface(Random theRnd)
    {
        int aNbU = theRnd.Next(2, 6), aNbV = theRnd.Next(2, 6);
        NCollection_Array2<Point3D> aPoles = new(1, aNbU, 1, aNbV);
        NCollection_Array2<double> aWeights = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                aPoles[i, j] = new Point3D(i + Uniform(theRnd, -0.3, 0.3), j + Uniform(theRnd, -0.3, 0.3),
                                          Uniform(theRnd, -1.0, 1.0));
                aWeights[i, j] = Uniform(theRnd, 0.5, 2.0);
            }
        }
        return theRnd.Next(2) == 0 ? new Geom_BezierSurface(aPoles) : new Geom_BezierSurface(aPoles, aWeights);
    }

    private static Geom_BSplineCurve RandomBSplineCurve(Random theRnd)
    {
        int aDeg = theRnd.Next(1, 4);
        int aNb = RandomKnots(theRnd, aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
        NCollection_Array1<Point3D> aPoles = new(1, aNb);
        for (int i = 1; i <= aNb; ++i)
        {
            aPoles[i] = new Point3D(Uniform(theRnd, 1.0, 4.0), Uniform(theRnd, -0.5, 0.5), i + Uniform(theRnd, -0.3, 0.3));
        }
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg);
    }

    private static Transform3D RandomTrsf(Random theRnd)
    {
        Transform3D aRot = new();
        aRot.SetRotation(new Axis3D(RandomPnt(theRnd, 3.0), RandomDir(theRnd)), Uniform(theRnd, -3.0, 3.0));
        Transform3D aTr = new();
        aTr.SetTranslation(new Vector3D(Uniform(theRnd, -5, 5), Uniform(theRnd, -5, 5), Uniform(theRnd, -5, 5)));
        Transform3D aScale = new();
        aScale.SetScale(RandomPnt(theRnd, 2.0), Uniform(theRnd, 0.5, 2.0));
        return aTr.Multiplied(aRot).Multiplied(aScale);
    }

    // Sorted parameters in [A, B]: the bounds, some knots (span boundaries), random values.
    private static NCollection_Array1<double> GridParams(Random theRnd,
                                                         double theA,
                                                         double theB,
                                                         NCollection_Array1<double>? theKnots)
    {
        List<double> aList = [theA, theB];
        int aNb = theRnd.Next(1, 9);
        for (int i = 0; i < aNb; ++i)
        {
            aList.Add(Uniform(theRnd, theA, theB));
        }
        if (theKnots is not null && theRnd.Next(2) == 0)
        {
            for (int i = theKnots.Lower(); i <= theKnots.Upper(); ++i)
            {
                if (theKnots[i] > theA && theKnots[i] < theB)
                {
                    aList.Add(theKnots[i]);
                }
            }
        }
        aList.Sort();
        return NCollection_Array1<double>.From(aList.ToArray());
    }

    // Whether the grid point (i, j) of a B-spline surface is in a patch holding two grid points or more (OCCT's
    // cache path), from the grid's own span location: OCCT's countSpanSize on each direction.
    private static bool[,] CachePattern(Geom_BSplineSurface theSurf, NCollection_Array1<double> theU, NCollection_Array1<double> theV)
    {
        int[] aUSpan = Spans(theSurf.InternalUKnotSequence(), theSurf.UDegree(), theSurf.IsUPeriodic(), theU);
        int[] aVSpan = Spans(theSurf.InternalVKnotSequence(), theSurf.VDegree(), theSurf.IsVPeriodic(), theV);
        bool[,] aRes = new bool[theU.Length() + 1, theV.Length() + 1];
        for (int i = 1; i <= theU.Length(); ++i)
        {
            for (int j = 1; j <= theV.Length(); ++j)
            {
                aRes[i, j] = RunLength(aUSpan, i) * RunLength(aVSpan, j) >= 2;
            }
        }
        return aRes;
    }

    // The span of each parameter as the grid locates it, and the length of the run (from its first parameter)
    // that the grid's countSpanSize gives for each run.
    private static int[] Spans(NCollection_Array1<double> theFlat, int theDeg, bool thePer, NCollection_Array1<double> theParams)
    {
        int[] aSpans = new int[theParams.Length() + 1];
        for (int i = 1; i <= theParams.Length(); ++i)
        {
            int aSpan = 0;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(theDeg, theFlat, null, theParams.Value(i), thePer, ref aSpan, out double _);
            aSpans[i] = aSpan;
        }
        return aSpans;
    }

    private static int RunLength(int[] theSpans, int theIndex)
    {
        int aStart = theIndex;
        while (aStart > 1 && theSpans[aStart - 1] == theSpans[theIndex])
        {
            --aStart;
        }
        int anEnd = aStart;
        while (anEnd + 1 < theSpans.Length && theSpans[anEnd + 1] == theSpans[theIndex])
        {
            ++anEnd;
        }
        return anEnd - aStart + 1;
    }

    private void CheckGrid(string theName,
                           GeomGridEval_Surface theEval,
                           Func<double, double, Geom_Surface.ResD3>? theGeomD3,
                           Func<double, double, Point3D>? theGeomD0,
                           Func<double, double, Geom_Surface.ResD1>? theGeomD1,
                           Func<double, double, Geom_Surface.ResD2>? theGeomD2,
                           Func<double, double, int, int, Vector3D>? theGeomDN,
                           Adaptor3d_Surface? theAdaptor,
                           NCollection_Array1<double> theU,
                           NCollection_Array1<double> theV,
                           bool[,]? theCache,
                           Geom_BSplineSurface? theRationalBSpline = null,
                           bool theWithDN = true)
    {
        NCollection_Array2<Point3D> aD0 = theEval.EvaluateGrid(theU, theV);
        NCollection_Array2<Geom_Surface.ResD1> aD1 = theEval.EvaluateGridD1(theU, theV);
        NCollection_Array2<Geom_Surface.ResD2> aD2 = theEval.EvaluateGridD2(theU, theV);
        NCollection_Array2<Geom_Surface.ResD3> aD3 = theEval.EvaluateGridD3(theU, theV);
        (int, int)[] anOrders = [(1, 0), (0, 1), (1, 1), (2, 0), (0, 2), (2, 1), (1, 2), (3, 0), (0, 3)];
        // The swept adaptors' DN is the basis curve's grid DN, one of OCCT's errors for a B-spline curve (wrong
        // span; pinned in GeomGridEval_CurveConsistencyTest): not compared (theWithDN false).
        NCollection_Array2<Vector3D>?[] aDN = new NCollection_Array2<Vector3D>?[anOrders.Length];
        for (int k = 0; k < anOrders.Length && theWithDN; ++k)
        {
            aDN[k] = theEval.EvaluateGridDN(theU, theV, anOrders[k].Item1, anOrders[k].Item2);
        }
        Assert.Equal(theU.Length(), aD0.NbRows());
        Assert.Equal(theV.Length(), aD0.NbColumns());

        for (int i = 1; i <= theU.Length(); ++i)
        {
            double u = theU.Value(i);
            for (int j = 1; j <= theV.Length(); ++j)
            {
                double v = theV.Value(j);
                string aTag = theCache is null ? "" : (theCache[i, j] ? " [cache]" : " [direct]");
                for (int r = 0; r < 2; ++r)
                {
                    Func<double, double, Point3D>? aRefD0 = r == 0 ? theGeomD0 : theAdaptor is null ? null : theAdaptor.EvalD0;
                    if (aRefD0 is null)
                    {
                        continue;
                    }
                    Func<double, double, Geom_Surface.ResD1> aRefD1 = r == 0 ? theGeomD1! : theAdaptor!.EvalD1;
                    Func<double, double, Geom_Surface.ResD2> aRefD2 = r == 0 ? theGeomD2! : theAdaptor!.EvalD2;
                    Func<double, double, Geom_Surface.ResD3> aRefD3 = r == 0 ? theGeomD3! : theAdaptor!.EvalD3;
                    Func<double, double, int, int, Vector3D>? aRefDN = r == 0 ? theGeomDN : theAdaptor!.EvalDN;
                    string aP = theName + (r == 0 ? " vs Geom " : " vs Adaptor ");

                    Compare(aP + "D0" + aTag, aD0.Value(i, j).XYZ(), aRefD0(u, v).XYZ());
                    Geom_Surface.ResD1 r1 = aRefD1(u, v), g1 = aD1.Value(i, j);
                    Compare(aP + "D1" + aTag, g1.Point.XYZ(), r1.Point.XYZ());
                    Compare(aP + "D1" + aTag, g1.D1U.XYZ(), r1.D1U.XYZ());
                    Compare(aP + "D1" + aTag, g1.D1V.XYZ(), r1.D1V.XYZ());
                    Geom_Surface.ResD2 r2 = aRefD2(u, v), g2 = aD2.Value(i, j);
                    Compare(aP + "D2" + aTag, g2.Point.XYZ(), r2.Point.XYZ());
                    Compare(aP + "D2" + aTag, g2.D1U.XYZ(), r2.D1U.XYZ());
                    Compare(aP + "D2" + aTag, g2.D1V.XYZ(), r2.D1V.XYZ());
                    Compare(aP + "D2" + aTag, g2.D2U.XYZ(), r2.D2U.XYZ());
                    Compare(aP + "D2" + aTag, g2.D2V.XYZ(), r2.D2V.XYZ());
                    Compare(aP + "D2" + aTag, g2.D2UV.XYZ(), r2.D2UV.XYZ());
                    Geom_Surface.ResD3 r3 = aRefD3(u, v), g3 = aD3.Value(i, j);
                    Compare(aP + "D3", g3.Point.XYZ(), r3.Point.XYZ());
                    Compare(aP + "D3", g3.D1U.XYZ(), r3.D1U.XYZ());
                    Compare(aP + "D3", g3.D1V.XYZ(), r3.D1V.XYZ());
                    Compare(aP + "D3", g3.D2U.XYZ(), r3.D2U.XYZ());
                    Compare(aP + "D3", g3.D2V.XYZ(), r3.D2V.XYZ());
                    Compare(aP + "D3", g3.D2UV.XYZ(), r3.D2UV.XYZ());
                    Compare(aP + "D3", g3.D3U.XYZ(), r3.D3U.XYZ());
                    Compare(aP + "D3", g3.D3V.XYZ(), r3.D3V.XYZ());
                    Compare(aP + "D3", g3.D3UUV.XYZ(), r3.D3UUV.XYZ());
                    Compare(aP + "D3", g3.D3UVV.XYZ(), r3.D3UVV.XYZ());
                    for (int k = 0; k < anOrders.Length; ++k)
                    {
                        if (aDN[k] is not null && theRationalBSpline is not null
                            && (anOrders[k].Item1 > theRationalBSpline.UDegree()
                                || anOrders[k].Item2 > theRationalBSpline.VDegree()))
                        {
                            // OCCT returns zero above a degree, also for a rational surface (occt-issues.md)
                            Assert.Equal(0.0, aDN[k]!.Value(i, j).Magnitude());
                            myRationalAboveDegreeNonZero |=
                                aRefDN!(u, v, anOrders[k].Item1, anOrders[k].Item2).Magnitude() > 1e-3;
                        }
                        else if (aDN[k] is not null)
                        {
                            Compare(aP + "DN", aDN[k]!.Value(i, j).XYZ(),
                                    aRefDN!(u, v, anOrders[k].Item1, anOrders[k].Item2).XYZ());
                        }
                    }
                }
            }
        }
    }

    private void CheckGeomSurface(string theName, Random theRnd, Geom_Surface theSurf, NCollection_Array1<double>? theUKnots,
                                  NCollection_Array1<double>? theVKnots, double theU1, double theU2, double theV1, double theV2)
    {
        NCollection_Array1<double> aU = GridParams(theRnd, theU1, theU2, theUKnots);
        NCollection_Array1<double> aV = GridParams(theRnd, theV1, theV2, theVKnots);
        Geom_BSplineSurface? aBSpline = theSurf as Geom_BSplineSurface
                                        ?? (theSurf as Geom_RectangularTrimmedSurface)?.BasisSurface() as Geom_BSplineSurface;
        bool[,]? aCache = aBSpline is null ? null : CachePattern(aBSpline, aU, aV);
        Geom_BSplineSurface? aRational = aBSpline is not null && (aBSpline.IsURational() || aBSpline.IsVRational()) ? aBSpline : null;
        GeomAdaptor_Surface anAdaptor = new(theSurf);
        CheckGrid(theName, new GeomGridEval_Surface(theSurf), theSurf.EvalD3, theSurf.EvalD0, theSurf.EvalD1,
                  theSurf.EvalD2, theSurf.EvalDN, anAdaptor, aU, aV, aCache, aRational);
        // Through the adaptor: the same evaluator
        CheckGrid(theName, new GeomGridEval_Surface(anAdaptor), theSurf.EvalD3, theSurf.EvalD0, theSurf.EvalD1,
                  theSurf.EvalD2, theSurf.EvalDN, anAdaptor, aU, aV, aCache, aRational);
    }

    [Fact]
    public void RandomSurfaces_MatchPointEvaluation()
    {
        Random aRnd = new(3001);
        for (int anIter = 0; anIter < 40; ++anIter)
        {
            // B-spline surfaces, non-rational, rational, periodic
            for (int aKind = 0; aKind < 3; ++aKind)
            {
                Geom_BSplineSurface aBS = RandomBSplineSurface(aRnd, aKind == 1 || (aKind == 2 && aRnd.Next(2) == 0), aKind == 2);
                aBS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                string aName = aKind == 0 ? "BSpline" : aKind == 1 ? "BSplineRational" : "BSplinePeriodic";
                CheckGeomSurface(aName, aRnd, aBS, aBS.InternalUKnots(), aBS.InternalVKnots(), aU1, aU2, aV1, aV2);
            }

            // A trimmed B-spline surface: evaluated through its basis
            {
                Geom_BSplineSurface aBS = RandomBSplineSurface(aRnd, aRnd.Next(2) == 0, false);
                aBS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                double aTU1 = aU1 + 0.2 * (aU2 - aU1), aTU2 = aU2 - 0.1 * (aU2 - aU1);
                Geom_RectangularTrimmedSurface aTrimmed = new(aBS, aTU1, aTU2, aV1, aV2);
                CheckGeomSurface("Trimmed", aRnd, aTrimmed, aBS.InternalUKnots(), aBS.InternalVKnots(), aTU1, aTU2, aV1, aV2);
            }

            // Offsets of B-spline and Bezier surfaces (general path) and of a cylinder and a sphere (equivalent surface)
            {
                Geom_BSplineSurface aBS = RandomBSplineSurface(aRnd, aRnd.Next(2) == 0, false);
                aBS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                Geom_OffsetSurface anOffset = new(aBS, Uniform(aRnd, -0.3, 0.3), true); // C0 bases too
                CheckGeomSurface("OffsetBSpline", aRnd, anOffset, aBS.InternalUKnots(), aBS.InternalVKnots(), aU1, aU2, aV1, aV2);

                Geom_OffsetSurface anOffsetBz = new(RandomBezierSurface(aRnd), Uniform(aRnd, -0.3, 0.3));
                CheckGeomSurface("OffsetBezier", aRnd, anOffsetBz, null, null, 0.0, 1.0, 0.0, 1.0);

                Geom_OffsetSurface anOffsetCyl = new(new Geom_CylindricalSurface(RandomAx3(aRnd), Uniform(aRnd, 1.0, 3.0)),
                                                     Uniform(aRnd, -0.5, 0.5));
                CheckGeomSurface("OffsetCylinder", aRnd, anOffsetCyl, null, null, 0.0, 2 * M_PI, -3.0, 3.0);

                Geom_OffsetSurface anOffsetSph = new(new Geom_SphericalSurface(RandomAx3(aRnd), Uniform(aRnd, 1.0, 3.0)),
                                                     Uniform(aRnd, -0.5, 0.5));
                CheckGeomSurface("OffsetSphere", aRnd, anOffsetSph, null, null, 0.0, 2 * M_PI, -1.4, 1.4);
            }

            // Other surfaces: a surface no evaluator knows (GeomEval_EllipsoidSurface), and an adaptor that isn't a
            // GeomAdaptor_Surface
            {
                GeomEval_EllipsoidSurface anEllipsoid = new(RandomAx3(aRnd), Uniform(aRnd, 1, 3), Uniform(aRnd, 1, 3),
                                                            Uniform(aRnd, 1, 3));
                NCollection_Array1<double> aU = GridParams(aRnd, 0.0, 2 * M_PI, null);
                NCollection_Array1<double> aV = GridParams(aRnd, -1.4, 1.4, null);
                GeomGridEval_Surface anEval = new(anEllipsoid);
                Assert.Equal(SurfaceKind.OtherSurface, anEval.GetType());
                CheckGrid("OtherGeom", anEval, anEllipsoid.EvalD3, anEllipsoid.EvalD0, anEllipsoid.EvalD1, anEllipsoid.EvalD2,
                          anEllipsoid.EvalDN, new GeomAdaptor_Surface(anEllipsoid), aU, aV, null);

                Geom_BSplineSurface aBS = RandomBSplineSurface(aRnd, true, false);
                aBS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                TestSurfaceAdaptor aTestAdaptor = new(aBS);
                GeomGridEval_Surface anEvalA = new(aTestAdaptor);
                Assert.Equal(SurfaceKind.BSplineSurface, anEvalA.GetType());
                CheckGrid("OtherAdaptor", anEvalA, aBS.EvalD3, aBS.EvalD0, aBS.EvalD1, aBS.EvalD2, aBS.EvalDN, aTestAdaptor,
                          GridParams(aRnd, aU1, aU2, null), GridParams(aRnd, aV1, aV2, null), null);
            }

            // GeomAdaptor_TransformedSurface: the grid of the original surface, transformed
            {
                Transform3D aTrsf = RandomTrsf(aRnd);
                Geom_BSplineSurface aBS = RandomBSplineSurface(aRnd, aRnd.Next(2) == 0, false);
                aBS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                GeomAdaptor_TransformedSurface aTransformed = new(aBS, aTrsf);
                GeomGridEval_Surface anEval = new(aTransformed);
                Assert.True(anEval.HasTransformation());
                NCollection_Array1<double> aU = GridParams(aRnd, aU1, aU2, aBS.InternalUKnots());
                NCollection_Array1<double> aV = GridParams(aRnd, aV1, aV2, aBS.InternalVKnots());
                CheckGrid("TransformedBSpline", anEval, null, null, null, null, null, aTransformed, aU, aV,
                          CachePattern(aBS, aU, aV),
                          aBS.IsURational() || aBS.IsVRational() ? aBS : null);

                Geom_CylindricalSurface aCyl = new(RandomAx3(aRnd), Uniform(aRnd, 1, 3));
                GeomAdaptor_TransformedSurface aTransformedCyl = new(aCyl, aTrsf);
                CheckGrid("TransformedCylinder", new GeomGridEval_Surface(aTransformedCyl), null, null, null, null, null,
                          aTransformedCyl,
                          GridParams(aRnd, 0.0, 2 * M_PI, null), GridParams(aRnd, -2.0, 2.0, null), null);
            }

            // The swept-surface adaptors without a Geom surface: rebuilt from the basis curve
            {
                Geom_BSplineCurve aCurve = RandomBSplineCurve(aRnd);
                GeomAdaptor_SurfaceOfRevolution aRev = new(new GeomAdaptor_Curve(aCurve), Constants.OZ());
                GeomGridEval_Surface anEval = new(aRev);
                Assert.Equal(SurfaceKind.SurfaceOfRevolution, anEval.GetType());
                Geom_SurfaceOfRevolution aRevSurf = new(aCurve, Constants.OZ());
                CheckGrid("RevolutionAdaptor", anEval, aRevSurf.EvalD3, aRevSurf.EvalD0, aRevSurf.EvalD1, aRevSurf.EvalD2,
                          null, null, GridParams(aRnd, 0.0, 2 * M_PI, null),
                          GridParams(aRnd, aCurve.FirstParameter(), aCurve.LastParameter(), null), null, null, false);

                Direction3D aDir = RandomDir(aRnd);
                GeomAdaptor_SurfaceOfLinearExtrusion anExt = new(new GeomAdaptor_Curve(aCurve), aDir);
                GeomGridEval_Surface anEvalE = new(anExt);
                Assert.Equal(SurfaceKind.SurfaceOfExtrusion, anEvalE.GetType());
                Geom_SurfaceOfLinearExtrusion anExtSurf = new(aCurve, aDir);
                CheckGrid("ExtrusionAdaptor", anEvalE, anExtSurf.EvalD3, anExtSurf.EvalD0, anExtSurf.EvalD1,
                          anExtSurf.EvalD2, null, null, GridParams(aRnd, aCurve.FirstParameter(), aCurve.LastParameter(), null),
                          GridParams(aRnd, -2.0, 2.0, null), null, null, false);
            }
        }

        List<string> aFailures = [];
        foreach ((string aKey, Stats aStats) in myStats)
        {
            double? anExpected = Expectation(aKey);
            myOut.WriteLine($"{aKey,-48} {aStats.Identical,7}/{aStats.Count,-7} maxRel {aStats.MaxRel:E2}"
                            + (anExpected is null ? "  (must be bit-identical)" : $"  (tolerance {anExpected:E0})"));
            bool isOk = anExpected is null ? aStats.Identical == aStats.Count : aStats.MaxRel <= anExpected.Value;
            if (!isOk)
            {
                aFailures.Add(aKey);
            }
        }
        Assert.Empty(aFailures);
        Assert.True(myRationalAboveDegreeNonZero); // the rational case really is non-zero there
    }

    /// <summary>
    /// OCCT's GeomGridEval_BSplineSurface::EvaluateGridDN returns zero when NU exceeds the U degree or NV the V
    /// degree, also for a rational surface, whose derivatives there are in general not zero (occt-issues.md).
    /// </summary>
    [Fact]
    public void RationalDN_AboveDegreeIsZero()
    {
        // A quarter of a circle in U (degree 2, rational), extruded linearly in V (degree 1)
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 2);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 2);
        double aW = Math.Sqrt(2.0) / 2.0;
        for (int j = 1; j <= 2; ++j)
        {
            aPoles[1, j] = new Point3D(1, 0, j - 1);
            aPoles[2, j] = new Point3D(1, 1, j - 1);
            aPoles[3, j] = new Point3D(0, 1, j - 1);
            aWeights[1, j] = 1.0;
            aWeights[2, j] = aW;
            aWeights[3, j] = 1.0;
        }
        Geom_BSplineSurface aSurf = new(aPoles, aWeights, NCollection_Array1<double>.From([0.0, 1.0]),
                                        NCollection_Array1<double>.From([0.0, 1.0]), NCollection_Array1<int>.From([3, 3]),
                                        NCollection_Array1<int>.From([2, 2]), 2, 1);
        GeomGridEval_BSplineSurface anEval = new(aSurf);
        NCollection_Array1<double> aParams = NCollection_Array1<double>.From([0.25, 0.5]);

        NCollection_Array2<Vector3D> aGrid = anEval.EvaluateGridDN(aParams, aParams, 3, 0);

        Vector3D aTrue = aSurf.DN(0.25, 0.25, 3, 0);
        Assert.True(aTrue.Magnitude() > 0.1);
        Assert.Equal(0.0, aGrid.Value(1, 1).Magnitude());
    }
}
