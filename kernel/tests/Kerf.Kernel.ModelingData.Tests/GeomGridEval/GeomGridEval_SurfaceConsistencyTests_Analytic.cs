// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The grid evaluators of planes, elementary surfaces, Bezier surfaces and swept surfaces against point
/// evaluation (Geom_Surface.EvalD* / EvalDN and GeomAdaptor_Surface) on random geometry: bit for bit where
/// OCCT's grid algorithm is the point evaluation's, to a relative 1e-13 or 1e-12 where OCCT's grid formula
/// differs (see <see cref="Expectation"/>).
/// </summary>
public class GeomGridEval_AnalyticSurfaceConsistencyTest
{
    private readonly ITestOutputHelper myOut;

    public GeomGridEval_AnalyticSurfaceConsistencyTest(ITestOutputHelper theOut) => myOut = theOut;

    private sealed class Stats
    {
        public double MaxRel;
        public long Count;
        public long Identical;
    }

    private readonly SortedDictionary<string, Stats> myStats = new(StringComparer.Ordinal);
    private bool myRationalAboveDegreeNonZero;
    private int myCurveDNSkipped;

    // Whether OCCT's curve grid DN of this order is one of its errors: any order of a B-spline basis (its span
    // index is a flat-knot index passed with the knots and multiplicities), and orders above the degree of a
    // rational Bezier basis (zero).
    private static bool CurveDNIsOcctError(Curve3D? theBasis, int theOrder)
    {
        return theBasis switch
        {
            BSplineCurve3D => theOrder >= 1,
            BezierCurve3D aBz => aBz.IsRational && theOrder > aBz.Degree,
            _ => false,
        };
    }

    /// <summary>
    /// What each comparison must give: null for bit-identical, else the relative tolerance.
    /// <list type="bullet">
    /// <item>Plane: the derivatives are the axes, bit-identical; the point is (Loc + u X) + v Y, where ElSLib adds
    /// in another order (last-bit differences).</item>
    /// <item>Cylinder, sphere, cone, torus: OCCT's grid formulas (cos/sin of U once per row, then products) group
    /// the terms differently from ElSLib's, so values differ in the last bits (up to 4e-15 relative).</item>
    /// <item>Bezier: D0-D2 use the single-span BSplSLib_Cache built at (0.5, 0.5), as GeomAdaptor_Surface does
    /// (bit-identical to it), where Geom_BezierSurface evaluates through BSplSLib directly (up to 8e-14);
    /// D3 and DN use BSplSLib::D3/DN as both do (bit-identical). A tall column (NbV = 1, NbU &gt;= 8) is
    /// evaluated on the V-isoline Bezier curve (Geom_BezierSurface.VIso) with GeomGridEval_Curve.</item>
    /// <item>Revolution, extrusion: Geom_RevolutionUtils/Geom_ExtrusionUtils on the basis curve's grid, as the
    /// Geom surfaces do on the curve's point evaluation, so bit-identical to Geom wherever the curve's grid is
    /// bit-identical to the curve's point evaluation. GeomAdaptor_Surface evaluates the basis curve through a
    /// GeomAdaptor_Curve (B-spline cache), hence a tolerance against it.</item>
    /// </list>
    /// </summary>
    private static double? Expectation(string theKey)
    {
        string aName = theKey[..theKey.IndexOf(' ')];
        bool isColumn = aName.EndsWith("Column", StringComparison.Ordinal);
        string aKind = aName.Replace("Column", "");
        bool isGeom = theKey.Contains(" vs Geom ");
        string aQty = theKey[(theKey.IndexOf(isGeom ? " vs Geom " : " vs Adaptor ") + (isGeom ? 9 : 12))..];
        bool isPoint = aQty.StartsWith("D0", StringComparison.Ordinal) || aQty.Contains(".P");
        switch (aKind)
        {
            case "Plane":
                return isPoint ? 1e-13 : null;
            case "Cylinder":
            case "Sphere":
            case "Cone":
            case "Torus":
                return 1e-13;
            case "Bezier":
            case "BezierRational":
                if (isColumn && aQty.StartsWith("D0", StringComparison.Ordinal))
                {
                    return 1e-12; // the V-isoline curve
                }
                if (!isGeom || aQty.StartsWith("D3", StringComparison.Ordinal) || aQty.StartsWith("DN", StringComparison.Ordinal))
                {
                    return null;
                }
                return 1e-12;
            default: // Revolution, Extrusion
                if (isGeom && aQty.EndsWith("[curve exact]", StringComparison.Ordinal))
                {
                    return null;
                }
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

    private void Compare(string theKey, Point3D theGrid, Point3D theRef) => Compare(theKey, theGrid.XYZ(), theRef.XYZ());

    private void Compare(string theKey, Vector3D theGrid, Vector3D theRef) => Compare(theKey, theGrid.XYZ(), theRef.XYZ());

    private static double Uniform(Random theRnd, double theA, double theB) => theA + (theB - theA) * theRnd.NextDouble();

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

    private static Point3D RandomPnt(Random theRnd, double theScale) =>
        new(Uniform(theRnd, -theScale, theScale), Uniform(theRnd, -theScale, theScale), Uniform(theRnd, -theScale, theScale));

    private static CoordinateSystem RandomAx3(Random theRnd)
    {
        Direction3D aN = RandomDir(theRnd);
        Direction3D aV;
        do
        {
            aV = RandomDir(theRnd);
        } while (Math.Abs(aN.Dot(aV)) > 0.9);
        CoordinateSystem anAx = new(RandomPnt(theRnd, 10.0), aN, aV);
        return theRnd.Next(2) == 0 ? anAx : anAx.YReversed(); // left-handed half the time
    }

    private static NCollection_Array1<double> SortedParams(Random theRnd, double theA, double theB, int theNb)
    {
        double[] aValues = new double[theNb];
        for (int i = 0; i < theNb; ++i)
        {
            aValues[i] = Uniform(theRnd, theA, theB);
        }
        aValues[0] = theA;
        aValues[theNb - 1] = theB;
        Array.Sort(aValues);
        return NCollection_Array1<double>.From(aValues);
    }

    private static BSplineCurve3D RandomBSplineCurve(Random theRnd, bool theRational)
    {
        int aDeg = theRnd.Next(1, 5);
        int aNbKnots = theRnd.Next(2, 6);
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        double aK = Uniform(theRnd, -2, 2);
        int aNbPoles = 0;
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots[i] = aK;
            aK += Uniform(theRnd, 0.3, 2.0);
            int m = (i == 1 || i == aNbKnots) ? aDeg + 1 : theRnd.Next(1, aDeg + 1);
            aMults[i] = m;
            aNbPoles += m;
        }
        aNbPoles -= aDeg + 1;
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        NCollection_Array1<double> aWeights = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            aPoles[i] = RandomPnt(theRnd, 10.0);
            aWeights[i] = Uniform(theRnd, 0.5, 2.0);
        }
        return theRational ? new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg)
                           : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
    }

    private static BezierCurve3D RandomBezierCurve(Random theRnd, bool theRational)
    {
        int aNb = theRnd.Next(2, 8);
        NCollection_Array1<Point3D> aPoles = new(1, aNb);
        NCollection_Array1<double> aWeights = new(1, aNb);
        for (int i = 1; i <= aNb; ++i)
        {
            aPoles[i] = RandomPnt(theRnd, 10.0);
            aWeights[i] = Uniform(theRnd, 0.5, 2.0);
        }
        return theRational ? new BezierCurve3D(aPoles, aWeights) : new BezierCurve3D(aPoles);
    }

    private static BezierSurface RandomBezierSurface(Random theRnd, bool theRational)
    {
        int aNbU = theRnd.Next(2, 7), aNbV = theRnd.Next(2, 7);
        NCollection_Array2<Point3D> aPoles = new(1, aNbU, 1, aNbV);
        NCollection_Array2<double> aWeights = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                aPoles[i, j] = RandomPnt(theRnd, 10.0);
                aWeights[i, j] = Uniform(theRnd, 0.5, 2.0);
            }
        }
        return theRational ? new BezierSurface(aPoles, aWeights) : new BezierSurface(aPoles);
    }

    private static Curve3D RandomBasisCurve(Random theRnd, int theKind)
    {
        switch (theKind)
        {
            case 0:
                return new LineCurve3D(RandomPnt(theRnd, 10.0), RandomDir(theRnd));
            case 1:
                return new CircleCurve3D(RandomAx3(theRnd).Frame, Uniform(theRnd, 0.5, 10.0));
            case 2:
            case 3:
                return RandomBSplineCurve(theRnd, theKind == 3);
            default:
                return RandomBezierCurve(theRnd, theKind == 5);
        }
    }

    private (string Name, Surface Surf, GeomGridEvalGrid.ISurfaceEvaluator Eval) RandomCase(Random theRnd, int theKind)
    {
        switch (theKind)
        {
            case 0:
            {
                PlaneSurface aS = new(RandomAx3(theRnd));
                return ("Plane", aS, new PlaneGrid(aS));
            }
            case 1:
            {
                CylindricalSurface aS = new(RandomAx3(theRnd), Uniform(theRnd, 0.1, 10.0));
                return ("Cylinder", aS, new CylinderGrid(aS));
            }
            case 2:
            {
                SphericalSurface aS = new(RandomAx3(theRnd), Uniform(theRnd, 0.1, 10.0));
                return ("Sphere", aS, new SphereGrid(aS));
            }
            case 3:
            {
                double aRef = theRnd.Next(3) == 0 ? 0.0 : Uniform(theRnd, 0.1, 10.0);
                double anAng = Uniform(theRnd, -1.4, 1.4);
                if (Math.Abs(anAng) < 0.05)
                {
                    anAng = 0.3;
                }
                ConicalSurface aS = new(RandomAx3(theRnd), anAng, aRef);
                return ("Cone", aS, new ConeGrid(aS));
            }
            case 4:
            {
                double aMaj = Uniform(theRnd, 0.5, 10.0);
                double aMin = theRnd.Next(3) == 0 ? aMaj * Uniform(theRnd, 1.1, 2.0) : aMaj * Uniform(theRnd, 0.1, 0.9);
                ToroidalSurface aS = new(RandomAx3(theRnd), aMaj, aMin);
                return ("Torus", aS, new TorusGrid(aS));
            }
            case 5:
            case 6:
            {
                BezierSurface aS = RandomBezierSurface(theRnd, theKind == 6);
                return (theKind == 6 ? "BezierRational" : "Bezier", aS, new BezierSurfaceGrid(aS));
            }
            case 7:
            {
                int aBasis = theRnd.Next(6);
                Curve3D aC = RandomBasisCurve(theRnd, aBasis);
                RevolutionSurface aS = new(aC, new Axis3D(RandomPnt(theRnd, 10.0), RandomDir(theRnd)));
                return ("Revolution", aS, new SurfaceOfRevolutionGrid(aS));
            }
            default:
            {
                int aBasis = theRnd.Next(6);
                Curve3D aC = RandomBasisCurve(theRnd, aBasis);
                ExtrusionSurface aS = new(aC, RandomDir(theRnd));
                return ("Extrusion", aS, new SurfaceOfExtrusionGrid(aS));
            }
        }
    }

    private static (double U1, double U2, double V1, double V2) Range(Surface theSurf)
    {
        theSurf.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aU1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aU2))
        {
            aU1 = -20.0;
            aU2 = 20.0;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aV1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aV2))
        {
            aV1 = -20.0;
            aV2 = 20.0;
        }
        return (aU1, aU2, aV1, aV2);
    }

    private void CheckGrid(string theName,
                           Surface theSurf,
                           GeomGridEvalGrid.ISurfaceEvaluator theEval,
                           NCollection_Array1<double> theU,
                           NCollection_Array1<double> theV)
    {
        SurfaceAdaptor anAdaptor = new(theSurf);

        // Swept surfaces evaluate their basis curve with GeomGridEval_Curve: their grid can only equal the
        // surface's point evaluation where the curve's grid equals the curve's point evaluation. Record, per
        // curve parameter and order, whether it does.
        Curve3D? aBasis = (theSurf as SweptSurface)?.BasisCurve;
        bool isRevolution = theSurf is RevolutionSurface;
        NCollection_Array1<double> aCurveParams = isRevolution ? theV : theU;
        bool[,] aCurveExact = new bool[aCurveParams.Length() + 1, 4];
        if (aBasis is not null)
        {
            CurveGrid aCurveEval = new(aBasis);
            NCollection_Array1<Point3D> aC0 = aCurveEval.EvaluateGrid(aCurveParams);
            NCollection_Array1<Curve3D.ResD1> aC1 = aCurveEval.EvaluateGridD1(aCurveParams);
            NCollection_Array1<Curve3D.ResD2> aC2 = aCurveEval.EvaluateGridD2(aCurveParams);
            NCollection_Array1<Curve3D.ResD3> aC3 = aCurveEval.EvaluateGridD3(aCurveParams);
            for (int k = 1; k <= aCurveParams.Length(); ++k)
            {
                // Each order against the curve's own evaluator of that order (EvalD3's point may differ from EvalD0's)
                double t = aCurveParams.Value(k);
                Curve3D.ResD1 aRef1 = aBasis.EvalD1(t);
                Curve3D.ResD2 aRef2 = aBasis.EvalD2(t);
                Curve3D.ResD3 aRef = aBasis.EvalD3(t);
                aCurveExact[k, 0] = aC0.Value(k).Equals(aBasis.EvalD0(t));
                aCurveExact[k, 1] = aC1.Value(k).Point.Equals(aRef1.Point) && aC1.Value(k).D1.Equals(aRef1.D1);
                aCurveExact[k, 2] = aC2.Value(k).Point.Equals(aRef2.Point) && aC2.Value(k).D1.Equals(aRef2.D1)
                                    && aC2.Value(k).D2.Equals(aRef2.D2);
                aCurveExact[k, 3] = aC3.Value(k).Point.Equals(aRef.Point) && aC3.Value(k).D1.Equals(aRef.D1)
                                    && aC3.Value(k).D2.Equals(aRef.D2) && aC3.Value(k).D3.Equals(aRef.D3);
            }
        }
        bool isRational = theSurf is BezierSurface aBz && (aBz.IsURational || aBz.IsVRational);
        int aUDeg = (theSurf as BezierSurface)?.UDegree ?? int.MaxValue;
        int aVDeg = (theSurf as BezierSurface)?.VDegree ?? int.MaxValue;
        NCollection_Array2<Point3D> aD0 = theEval.EvaluateGrid(theU, theV);
        NCollection_Array2<Surface.ResD1> aD1 = theEval.EvaluateGridD1(theU, theV);
        NCollection_Array2<Surface.ResD2> aD2 = theEval.EvaluateGridD2(theU, theV);
        NCollection_Array2<Surface.ResD3> aD3 = theEval.EvaluateGridD3(theU, theV);
        (int, int)[] anOrders = [(1, 0), (0, 1), (1, 1), (2, 0), (0, 2), (2, 1), (1, 2), (3, 0), (0, 3), (4, 0), (2, 2), (0, 4), (5, 1)];
        NCollection_Array2<Vector3D>?[] aDN = new NCollection_Array2<Vector3D>?[anOrders.Length];
        for (int k = 0; k < anOrders.Length; ++k)
        {
            // The swept surfaces' DN is the basis curve's grid DN, which OCCT's B-spline curve grid evaluates on the
            // wrong span (sometimes reading past the knots: IndexOutOfRangeException in the port), and which is zero
            // above the degree of a rational curve (occt-issues.md; pinned in GeomGridEval_CurveConsistencyTest).
            // Those values are not compared.
            if (CurveDNIsOcctError(aBasis, isRevolution ? anOrders[k].Item2 : anOrders[k].Item1))
            {
                myCurveDNSkipped++;
                continue;
            }
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
                foreach ((string aRefName, Func<double, double, Surface.ResD3> aRefD3, Func<double, double, Point3D> aRefD0,
                          Func<double, double, Surface.ResD1> aRefD1, Func<double, double, Surface.ResD2> aRefD2,
                          Func<double, double, int, int, Vector3D> aRefDN) in new[]
                         {
                             ("Geom", (Func<double, double, Surface.ResD3>)theSurf.EvalD3,
                              (Func<double, double, Point3D>)theSurf.EvalD0,
                              (Func<double, double, Surface.ResD1>)theSurf.EvalD1,
                              (Func<double, double, Surface.ResD2>)theSurf.EvalD2,
                              (Func<double, double, int, int, Vector3D>)theSurf.EvalDN),
                             ("Adaptor", anAdaptor.EvalD3, anAdaptor.EvalD0, anAdaptor.EvalD1, anAdaptor.EvalD2,
                              anAdaptor.EvalDN),
                         })
                {
                    string aP = theName + " vs " + aRefName + " ";
                    int aC = isRevolution ? j : i;
                    string[] aTag = new string[4];
                    for (int d = 0; d < 4; ++d)
                    {
                        aTag[d] = aBasis is null ? "" : (aCurveExact[aC, d] ? " [curve exact]" : " [curve inexact]");
                    }
                    Compare(aP + "D0" + aTag[0], aD0.Value(i, j), aRefD0(u, v));
                    Surface.ResD1 r1 = aRefD1(u, v);
                    Surface.ResD1 g1 = aD1.Value(i, j);
                    Compare(aP + "D1.P" + aTag[1], g1.Point, r1.Point);
                    Compare(aP + "D1.D1" + aTag[1], g1.D1U, r1.D1U);
                    Compare(aP + "D1.D1" + aTag[1], g1.D1V, r1.D1V);
                    Surface.ResD2 r2 = aRefD2(u, v);
                    Surface.ResD2 g2 = aD2.Value(i, j);
                    Compare(aP + "D2.P" + aTag[2], g2.Point, r2.Point);
                    Compare(aP + "D2.D1" + aTag[2], g2.D1U, r2.D1U);
                    Compare(aP + "D2.D1" + aTag[2], g2.D1V, r2.D1V);
                    Compare(aP + "D2.D2" + aTag[2], g2.D2U, r2.D2U);
                    Compare(aP + "D2.D2" + aTag[2], g2.D2V, r2.D2V);
                    Compare(aP + "D2.D2" + aTag[2], g2.D2UV, r2.D2UV);
                    Surface.ResD3 r3 = aRefD3(u, v);
                    Surface.ResD3 g3 = aD3.Value(i, j);
                    Compare(aP + "D3.P" + aTag[3], g3.Point, r3.Point);
                    Compare(aP + "D3.D1" + aTag[3], g3.D1U, r3.D1U);
                    Compare(aP + "D3.D1" + aTag[3], g3.D1V, r3.D1V);
                    Compare(aP + "D3.D2" + aTag[3], g3.D2U, r3.D2U);
                    Compare(aP + "D3.D2" + aTag[3], g3.D2V, r3.D2V);
                    Compare(aP + "D3.D2" + aTag[3], g3.D2UV, r3.D2UV);
                    Compare(aP + "D3.D3" + aTag[3], g3.D3U, r3.D3U);
                    Compare(aP + "D3.D3" + aTag[3], g3.D3V, r3.D3V);
                    Compare(aP + "D3.D3" + aTag[3], g3.D3UUV, r3.D3UUV);
                    Compare(aP + "D3.D3" + aTag[3], g3.D3UVV, r3.D3UVV);
                    for (int k = 0; k < anOrders.Length; ++k)
                    {
                        if (aDN[k] is null)
                        {
                            continue;
                        }
                        Vector3D aRefVec = aRefDN(u, v, anOrders[k].Item1, anOrders[k].Item2);
                        if (isRational && (anOrders[k].Item1 > aUDeg || anOrders[k].Item2 > aVDeg))
                        {
                            // OCCT returns zero above the degrees, also for rational surfaces (occt-issues.md)
                            Assert.Equal(0.0, aDN[k]!.Value(i, j).Magnitude);
                            myRationalAboveDegreeNonZero |= aRefVec.Magnitude > 1e-3;
                            continue;
                        }
                        Compare(aP + "DN", aDN[k]!.Value(i, j), aRefVec);
                    }
                }
            }
        }
    }

    [Fact]
    public void RandomSurfaces_MatchPointEvaluation()
    {
        Random aRnd = new(20261029);
        for (int aKind = 0; aKind <= 8; ++aKind)
        {
            for (int anIter = 0; anIter < 40; ++anIter)
            {
                (string aName, Surface aSurf, GeomGridEvalGrid.ISurfaceEvaluator anEval) = RandomCase(aRnd, aKind);
                (double aU1, double aU2, double aV1, double aV2) = Range(aSurf);
                NCollection_Array1<double> aU = SortedParams(aRnd, aU1, aU2, aRnd.Next(2, 10));
                NCollection_Array1<double> aV = SortedParams(aRnd, aV1, aV2, aRnd.Next(2, 10));
                CheckGrid(aName, aSurf, anEval, aU, aV);

                // A tall column (the Bezier V-isoline path)
                NCollection_Array1<double> aCol = SortedParams(aRnd, aV1, aV2, 2);
                NCollection_Array1<double> aOneV = NCollection_Array1<double>.From([aCol.Value(1) * 0.3 + aCol.Value(2) * 0.7]);
                NCollection_Array1<double> aManyU = SortedParams(aRnd, aU1, aU2, 12);
                CheckGrid(aName + "Column", aSurf, anEval, aManyU, aOneV);
            }
        }

        List<string> aFailures = [];
        foreach ((string aKey, Stats aStats) in myStats)
        {
            double? anExpected = Expectation(aKey);
            myOut.WriteLine($"{aKey,-62} {aStats.Identical,7}/{aStats.Count,-7} maxRel {aStats.MaxRel:E2}"
                            + (anExpected is null ? "  (must be bit-identical)" : $"  (tolerance {anExpected:E0})"));
            bool isOk = anExpected is null ? aStats.Identical == aStats.Count : aStats.MaxRel <= anExpected.Value;
            if (!isOk)
            {
                aFailures.Add(aKey);
            }
        }
        myOut.WriteLine($"DN grids of swept surfaces not compared (OCCT's curve grid DN errors): {myCurveDNSkipped}");
        Assert.Empty(aFailures);
        Assert.True(myRationalAboveDegreeNonZero); // the rational case really is non-zero there
    }
}
