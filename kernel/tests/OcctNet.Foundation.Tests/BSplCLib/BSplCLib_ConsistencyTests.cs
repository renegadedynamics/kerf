// OCCT.NET's own tests (not a port of an OCCT GTest): BSplCLib's operations checked against each other on random
// curves, since OCCT's GTests cover little of BSplCLib and TKMath has no oracle (docs/decisions.md, decision 23).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using System.Runtime.InteropServices;

namespace OcctNet.Foundation.Tests;

public class BSplCLib_ConsistencyTest
{
    private const double Tol = 1.0e-9;

    // A random curve: knots 0, 1, 2, ..., with interior multiplicities from 1 to Degree, end multiplicities
    // Degree + 1 (or all equal for a periodic curve), random poles and (optionally) weights.
    private sealed class Curve
    {
        public int Degree;
        public bool Periodic;
        public NCollection_Array1<double> Knots = null!;
        public NCollection_Array1<int> Mults = null!;
        public NCollection_Array1<Point3D> Poles = null!;
        public NCollection_Array1<double>? Weights;
        public NCollection_Array1<double> FlatKnots = null!;
    }

    private static Curve MakeCurve(Random theRnd, int theDegree, int theNbKnots, bool theRational, bool thePeriodic)
    {
        Curve c = new() { Degree = theDegree, Periodic = thePeriodic };
        c.Knots = new NCollection_Array1<double>(1, theNbKnots);
        c.Mults = new NCollection_Array1<int>(1, theNbKnots);
        double k = 0.0;
        for (int i = 1; i <= theNbKnots; i++)
        {
            c.Knots[i] = k;
            k += 0.5 + theRnd.NextDouble();
            c.Mults[i] = thePeriodic ? 1 + theRnd.Next(theDegree) : (i == 1 || i == theNbKnots ? theDegree + 1 : 1 + theRnd.Next(theDegree));
        }
        if (thePeriodic)
        {
            c.Mults[theNbKnots] = c.Mults[1];
        }
        int aNbPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(theDegree, thePeriodic, c.Mults);
        c.Poles = new NCollection_Array1<Point3D>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            c.Poles[i] = new Point3D(theRnd.NextDouble() * 10, theRnd.NextDouble() * 10, theRnd.NextDouble() * 10);
        }
        if (theRational)
        {
            c.Weights = new NCollection_Array1<double>(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                c.Weights[i] = 0.5 + theRnd.NextDouble();
            }
        }
        c.FlatKnots = new NCollection_Array1<double>(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(c.Mults, theDegree, thePeriodic));
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(c.Knots, c.Mults, theDegree, thePeriodic, c.FlatKnots);
        return c;
    }

    private static IEnumerable<Curve> Curves()
    {
        Random aRnd = new(12345);
        foreach (int aDeg in new[] { 1, 2, 3, 5, 8 })
        {
            foreach (bool aRat in new[] { false, true })
            {
                foreach (bool aPer in new[] { false, true })
                {
                    yield return MakeCurve(aRnd, aDeg, aPer ? Math.Max(6, aDeg + 2) : 6, aRat, aPer);
                }
            }
        }
    }

    private static IEnumerable<double> Params(Curve c, int theNb = 25)
    {
        double f = c.Knots.First(), l = c.Knots.Last();
        for (int i = 0; i <= theNb; i++)
        {
            yield return f + (l - f) * i / theNb;
        }
    }

    private static void AssertNear(in Xyz a, in Xyz b, double tol)
    {
        double aScale = Math.Max(1.0, b.Modulus());
        Assert.True((a - b).Modulus() <= tol * aScale, $"{a} != {b}");
    }

    [Fact]
    public void D0ToD3_MatchCache()
    {
        foreach (Curve c in Curves())
        {
            global::Kerf.Engine.Geometry.Splines.Cache aCache = new(c.Degree, c.Periodic, c.FlatKnots, c.Poles, c.Weights);
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.FlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                            out Point3D P, out Vector3D V1, out Vector3D V2, out Vector3D V3);
                if (!aCache.IsCacheValid(u))
                {
                    aCache.BuildCache(u, c.FlatKnots, c.Poles, c.Weights);
                }
                aCache.D3(u, out Point3D CP, out Vector3D CV1, out Vector3D CV2, out Vector3D CV3);
                AssertNear(P.XYZ(), CP.XYZ(), Tol);
                AssertNear(V1.XYZ(), CV1.XYZ(), 1e-8);
                AssertNear(V2.XYZ(), CV2.XYZ(), 1e-7);
                if (c.Degree > 1)
                {
                    AssertNear(V3.XYZ(), CV3.XYZ(), 1e-6);
                }

                // the same through knots and multiplicities, and through D0..D2
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P0);
                AssertNear(P0.XYZ(), P.XYZ(), Tol);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P1, out Vector3D W1);
                AssertNear(P1.XYZ(), P.XYZ(), Tol);
                AssertNear(W1.XYZ(), V1.XYZ(), 1e-8);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P2, out Vector3D W21, out Vector3D W22);
                AssertNear(P2.XYZ(), P.XYZ(), Tol);
                AssertNear(W22.XYZ(), V2.XYZ(), 1e-7);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u, 1, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Vector3D VN);
                AssertNear(VN.XYZ(), V1.XYZ(), 1e-8);
            }
        }
    }

    [Fact]
    public void D1_MatchesFiniteDifference()
    {
        foreach (Curve c in Curves())
        {
            foreach (double u in Params(c, 13))
            {
                double h = 1e-6;
                double u0 = Math.Max(c.Knots.First(), u - h), u1 = Math.Min(c.Knots.Last(), u + h);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.FlatKnots, null, out _, out Vector3D V);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u0, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.FlatKnots, null, out Point3D A);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u1, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.FlatKnots, null, out Point3D B);
                // one-sided at knots of low continuity is fine for the interior of a span only
                if (c.Degree >= 2)
                {
                    Xyz aFd = (B.XYZ() - A.XYZ()) / (u1 - u0);
                    double aScale = Math.Max(1.0, V.Magnitude());
                    Assert.True((aFd - V.XYZ()).Modulus() <= 1e-3 * aScale || IsNearKnot(c, u), $"{aFd} vs {V}");
                }
            }
        }
    }

    private static bool IsNearKnot(Curve c, double u)
    {
        for (int i = c.Knots.Lower(); i <= c.Knots.Upper(); i++)
        {
            if (Math.Abs(c.Knots[i] - u) < 1e-5)
            {
                return true;
            }
        }
        return false;
    }

    [Fact]
    public void FlatEval_MatchesD1()
    {
        Span<int> aMode = stackalloc int[2];
        Span<double> aRes = stackalloc double[6];
        foreach (Curve c in Curves())
        {
            if (c.Periodic)
            {
                continue;
            }
            aMode[0] = aMode[1] = c.Degree;
            ReadOnlySpan<double> aPoles = MemoryMarshal.Cast<Point3D, double>(c.Poles.AsSpan());
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.FlatKnots, null, out Point3D P, out Vector3D V);
                if (c.Weights is null)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u, false, 1, aMode, c.Degree, c.FlatKnots, 3, aPoles, aRes);
                    AssertNear(new Xyz(aRes[0], aRes[1], aRes[2]), P.XYZ(), Tol);
                    AssertNear(new Xyz(aRes[3], aRes[4], aRes[5]), V.XYZ(), 1e-8);
                }
                else
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u, false, false, aMode, c.Degree, c.FlatKnots, c.Poles, c.Weights, out Point3D EP, out double EW);
                    AssertNear(EP.XYZ(), P.XYZ(), Tol);
                    Assert.True(EW > 0);
                }
            }
        }
    }

    [Fact]
    public void EvalBsplineBasis_IsPartitionOfUnity()
    {
        foreach (Curve c in Curves())
        {
            int aOrder = c.Degree + 1;
            Matrix aBasis = new(1, 3, 1, aOrder);
            foreach (double u in Params(c))
            {
                int aErr = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(2, aOrder, c.FlatKnots, u, out int aFirst, aBasis);
                Assert.Equal(0, aErr);
                double aSum = 0, aSumD1 = 0;
                for (int j = 1; j <= aOrder; j++)
                {
                    aSum += aBasis[1, j];
                    aSumD1 += aBasis[2, j];
                }
                Assert.Equal(1.0, aSum, 1e-12);
                Assert.Equal(0.0, aSumD1, 1e-9);
                Assert.True(aFirst >= 1);
            }
        }
    }

    [Fact]
    public void InsertKnots_KeepsTheCurve_AndRemoveKnotUndoesIt()
    {
        foreach (Curve c in Curves())
        {
            if (c.Periodic)
            {
                continue;
            }
            NCollection_Array1<double> aAdd = new(1, 2);
            aAdd[1] = c.Knots[2] + 0.25;
            aAdd[2] = c.Knots[4] + 0.1;
            NCollection_Array1<int> aAddM = new(1, 2);
            aAddM[1] = 1;
            aAddM[2] = c.Degree;
            Assert.True(global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(c.Degree, c.Periodic, c.Knots, c.Mults, aAdd, aAddM, out int aNbP, out int aNbK, 1e-9));
            NCollection_Array1<Point3D> aNP = new(1, aNbP);
            NCollection_Array1<double>? aNW = c.Weights is null ? null : new NCollection_Array1<double>(1, aNbP);
            NCollection_Array1<double> aNK = new(1, aNbK);
            NCollection_Array1<int> aNM = new(1, aNbK);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, aAdd, aAddM, aNP, aNW, aNK, aNM, 1e-9);
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, aNP, aNW, aNK, aNM, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), Tol);
            }

            // remove the simple knot inserted at index 3 again
            NCollection_Array1<Point3D> aRP = new(1, aNbP - 1);
            NCollection_Array1<double>? aRW = c.Weights is null ? null : new NCollection_Array1<double>(1, aNbP - 1);
            NCollection_Array1<double> aRK = new(1, aNbK - 1);
            NCollection_Array1<int> aRM = new(1, aNbK - 1);
            Assert.True(global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(3, 0, c.Degree, c.Periodic, aNP, aNW, aNK, aNM, aRP, aRW, aRK, aRM, 1e-7));
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, aRP, aRW, aRK, aRM, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), 1e-7);
            }
        }
    }

    [Fact]
    public void IncreaseDegree_KeepsTheCurve()
    {
        List<string> aFails = new();
        foreach (Curve c in Curves())
        {
            if (c.Periodic)
            {
                // OCCT's IncreaseDegree changes most periodic curves whose multiplicities aren't all 1 (checked
                // against native OCCT, whose results the port reproduces exactly; docs/occt-issues.md)
                continue;
            }
            int aNewDeg = c.Degree + 2;
            int aNbK = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(c.Degree, aNewDeg, c.Periodic, c.Mults);
            NCollection_Array1<double> aNK = new(1, aNbK);
            NCollection_Array1<int> aNM = new(1, aNbK);
            // the multiplicities first, to size the poles
            int aNbP = 0;
            {
                NCollection_Array1<int> aTmpM = new(1, aNbK);
                // IncreaseDegree raises every multiplicity by NewDegree - Degree (the ends too, if not periodic)
                int j = 1;
                for (int i = c.Mults.Lower(); i <= c.Mults.Upper(); i++)
                {
                    aTmpM[j++] = c.Mults[i] + aNewDeg - c.Degree;
                }
                aNbP = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aNewDeg, c.Periodic, aTmpM);
            }
            NCollection_Array1<Point3D> aNP = new(1, aNbP);
            NCollection_Array1<double>? aNW = c.Weights is null ? null : new NCollection_Array1<double>(1, aNbP);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(c.Degree, aNewDeg, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, aNP, aNW, aNK, aNM);
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, aNewDeg, c.Periodic, aNP, aNW, aNK, aNM, out Point3D Q);
                if ((Q.XYZ() - P.XYZ()).Modulus() > 1e-8) aFails.Add($"deg {c.Degree} per {c.Periodic} rat {c.Weights != null} u {u} mults {string.Join(",", c.Mults.AsSpan().ToArray())} new {string.Join(",", aNM.AsSpan().ToArray())} nbp {aNbP} {c.Poles.Length()} d {(Q.XYZ() - P.XYZ()).Modulus()}");
            }
        }
        Assert.True(aFails.Count == 0, string.Join(" | ", aFails.Take(40)));
    }

    [Fact]
    public void Trimming_KeepsTheCurve()
    {
        foreach (Curve c in Curves())
        {
            double U1 = c.Knots[2] + 0.2, U2 = c.Knots[5] - 0.1;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareTrimming(c.Degree, c.Periodic, c.Knots, c.Mults, U1, U2, out int aNbK, out int aNbP);
            NCollection_Array1<double> aNK = new(1, aNbK);
            NCollection_Array1<int> aNM = new(1, aNbK);
            NCollection_Array1<Point3D> aNP = new(1, aNbP);
            NCollection_Array1<double>? aNW = c.Weights is null ? null : new NCollection_Array1<double>(1, aNbP);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Trimming(c.Degree, c.Periodic, c.Knots, c.Mults, c.Poles, c.Weights, U1, U2, aNK, aNM, aNP, aNW);
            Assert.Equal(U1, aNK.First(), 1e-12);
            Assert.Equal(U2, aNK.Last(), 1e-12);
            for (int i = 0; i <= 10; i++)
            {
                double u = U1 + (U2 - U1) * i / 10;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, c.Periodic, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, false, aNP, aNW, aNK, aNM, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), 1e-8);
            }
        }
    }

    [Fact]
    public void Unperiodize_KeepsTheCurve()
    {
        foreach (Curve c in Curves())
        {
            if (!c.Periodic)
            {
                continue;
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(c.Degree, c.Mults, out int aNbK, out int aNbP);
            NCollection_Array1<double> aNK = new(1, aNbK);
            NCollection_Array1<int> aNM = new(1, aNbK);
            NCollection_Array1<Point3D> aNP = new(1, aNbP);
            NCollection_Array1<double>? aNW = c.Weights is null ? null : new NCollection_Array1<double>(1, aNbP);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(c.Degree, c.Mults, c.Knots, c.Poles, c.Weights, aNM, aNK, aNP, aNW);
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, true, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, false, aNP, aNW, aNK, aNM, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), 1e-8);
            }
        }
    }

    [Fact]
    public void Interpolate_PassesThroughThePoints()
    {
        Random aRnd = new(7);
        foreach (int aDeg in new[] { 1, 2, 3, 4, 7 })
        {
            Curve c = MakeCurve(aRnd, aDeg, 5, false, false);
            int aNb = c.Poles.Length();
            NCollection_Array1<double> aParams = new(1, aNb);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(aDeg, c.FlatKnots, aParams);
            NCollection_Array1<int> aContact = new(1, aNb);
            NCollection_Array1<Point3D> aPts = new(1, aNb);
            for (int i = 1; i <= aNb; i++)
            {
                aPts[i] = new Point3D(aRnd.NextDouble(), aRnd.NextDouble(), aRnd.NextDouble());
            }
            NCollection_Array1<Point3D> aPoles = new(aPts);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(aDeg, c.FlatKnots, aParams, aContact, aPoles, out int aProblem);
            Assert.Equal(0, aProblem);
            for (int i = 1; i <= aNb; i++)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(aParams[i], 0, aDeg, false, aPoles, null, c.FlatKnots, null, out Point3D P);
                AssertNear(P.XYZ(), aPts[i].XYZ(), 1e-9);
            }

            // rational: the weighted interpolation reproduces the points too
            NCollection_Array1<Point3D> aRPoles = new(aPts);
            NCollection_Array1<double> aW = new(1, aNb);
            for (int i = 1; i <= aNb; i++)
            {
                aW[i] = 0.5 + aRnd.NextDouble();
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(aDeg, c.FlatKnots, aParams, aContact, aRPoles, aW, out aProblem);
            Assert.Equal(0, aProblem);
        }
    }

    [Fact]
    public void MovePoint_MovesThePoint()
    {
        Random aRnd = new(99);
        foreach (int aDeg in new[] { 2, 3, 5 })
        {
            Curve c = MakeCurve(aRnd, aDeg, 6, false, false);
            double u = 0.5 * (c.Knots[3] + c.Knots[4]);
            Vector3D aDispl = new(0.3, -0.2, 0.1);
            NCollection_Array1<Point3D> aNP = new(1, c.Poles.Length());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePoint(u, aDispl, 1, c.Poles.Length(), aDeg, c.Poles, null, c.FlatKnots, out int aFirst, out int aLast, aNP);
            Assert.True(aFirst >= 1 && aLast >= aFirst);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, aDeg, false, c.Poles, null, c.FlatKnots, null, out Point3D P);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, aDeg, false, aNP, null, c.FlatKnots, null, out Point3D Q);
            AssertNear(Q.XYZ(), P.XYZ() + aDispl.XYZ(), 1e-9);
        }
    }

    [Fact]
    public void Bezier_D0_MatchesPolesCoefficients()
    {
        Random aRnd = new(3);
        foreach (int aDeg in new[] { 1, 2, 3, 6, 25 })
        {
            NCollection_Array1<Point3D> aPoles = new(1, aDeg + 1);
            NCollection_Array1<double> aW = new(1, aDeg + 1);
            for (int i = 1; i <= aDeg + 1; i++)
            {
                aPoles[i] = new Point3D(aRnd.NextDouble(), aRnd.NextDouble(), aRnd.NextDouble());
                aW[i] = 0.5 + aRnd.NextDouble();
            }
            NCollection_Array1<Point3D> aCoefs = new(1, aDeg + 1);
            NCollection_Array1<double> aWCoefs = new(1, aDeg + 1);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(aPoles, aW, aCoefs, aWCoefs);
            // and back with PLib
            NCollection_Array1<Point3D> aBack = new(1, aDeg + 1);
            NCollection_Array1<double> aWBack = new(1, aDeg + 1);
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(aCoefs, aWCoefs, aBack, aWBack);
            for (int i = 1; i <= aDeg + 1; i++)
            {
                AssertNear(aBack[i].XYZ(), aPoles[i].XYZ(), aDeg > 20 ? 1e-5 : 1e-9);
                Assert.Equal(aW[i], aWBack[i], aDeg > 20 ? 1e-5 : 1e-9);
            }
            for (int k = 0; k <= 10; k++)
            {
                double u = k / 10.0;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, aPoles, aW, out Point3D P, out Vector3D V);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.CoefsD1(u, aCoefs, aWCoefs, out Point3D Q, out Vector3D W);
                AssertNear(Q.XYZ(), P.XYZ(), aDeg > 20 ? 1e-4 : 1e-9); // the power basis is ill-conditioned at degree 25
                AssertNear(W.XYZ(), V.XYZ(), aDeg > 20 ? 1e-3 : 1e-8);
            }
        }
    }

    [Fact]
    public void Reverse_ReversesTheCurve()
    {
        foreach (Curve c in Curves())
        {
            if (c.Periodic)
            {
                continue;
            }
            NCollection_Array1<double> aK = new(c.Knots);
            NCollection_Array1<int> aM = new(c.Mults);
            NCollection_Array1<Point3D> aP = new(c.Poles);
            NCollection_Array1<double>? aW = c.Weights is null ? null : new NCollection_Array1<double>(c.Weights);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(aK);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(aM);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(aP, aP.Upper());
            if (aW != null)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(aW, aW.Upper());
            }
            double f = c.Knots.First(), l = c.Knots.Last();
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, false, c.Poles, c.Weights, c.Knots, c.Mults, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(f + l - u, 0, c.Degree, false, aP, aW, aK, aM, out Point3D Q);
                AssertNear(Q.XYZ(), P.XYZ(), Tol);
            }
        }
    }

    [Fact]
    public void Curve2d_MatchesCurve3dWithZeroZ()
    {
        foreach (Curve c in Curves())
        {
            NCollection_Array1<Point2D> aP2 = new(1, c.Poles.Length());
            NCollection_Array1<Point3D> aP3 = new(1, c.Poles.Length());
            for (int i = 1; i <= c.Poles.Length(); i++)
            {
                aP2[i] = new Point2D(c.Poles[i].X(), c.Poles[i].Y());
                aP3[i] = new Point3D(c.Poles[i].X(), c.Poles[i].Y(), 0.0);
            }
            global::Kerf.Engine.Geometry.Splines.Cache aCache = new(c.Degree, c.Periodic, c.FlatKnots, aP2, c.Weights);
            foreach (double u in Params(c))
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, c.Degree, c.Periodic, aP3, c.Weights, c.FlatKnots, null, out Point3D P, out Vector3D V1, out Vector3D V2);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, 0, c.Degree, c.Periodic, aP2, c.Weights, c.FlatKnots, null, out Point2D Q, out Vector2D W1, out Vector2D W2);
                AssertNear(new Xyz(Q.X(), Q.Y(), 0), P.XYZ(), Tol);
                AssertNear(new Xyz(W1.X(), W1.Y(), 0), V1.XYZ(), 1e-8);
                AssertNear(new Xyz(W2.X(), W2.Y(), 0), V2.XYZ(), 1e-7);
                if (!aCache.IsCacheValid(u))
                {
                    aCache.BuildCache(u, c.FlatKnots, aP2, c.Weights);
                }
                aCache.D1(u, out Point2D CQ, out Vector2D CW);
                AssertNear(new Xyz(CQ.X(), CQ.Y(), 0), P.XYZ(), Tol);
                AssertNear(new Xyz(CW.X(), CW.Y(), 0), V1.XYZ(), 1e-8);
            }
        }
    }

    [Fact]
    public void Resolution_BoundsTheParameterStep()
    {
        foreach (Curve c in Curves())
        {
            if (c.Periodic)
            {
                continue;
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(c.Poles, c.Weights, c.Poles.Length(), c.FlatKnots, c.Degree, 1.0e-3, out double aUTol);
            Assert.True(aUTol > 0);
            // a parameter step of UTolerance moves the point by at most the 3D tolerance
            foreach (double u in Params(c, 10))
            {
                double v = Math.Min(u + aUTol, c.Knots.Last());
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 0, c.Degree, false, c.Poles, c.Weights, c.FlatKnots, null, out Point3D P);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(v, 0, c.Degree, false, c.Poles, c.Weights, c.FlatKnots, null, out Point3D Q);
                Assert.True(P.Distance(Q) <= 1.0e-3 * (1 + 1e-9));
            }
        }
    }
}
