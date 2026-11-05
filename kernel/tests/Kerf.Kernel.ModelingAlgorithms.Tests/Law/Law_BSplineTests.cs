// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Law_BSpline against Geom2d_BSplineCurve: a 1D B-spline of poles p_i is the Y coordinate of the planar B-spline
/// of poles (0, p_i) with the same weights, knots and multiplicities, so its values, derivatives and knot
/// operations are checked against the ported (and oracle-tested) 2D curve.
/// </summary>
public class Law_BSplineTests
{
    private const double Tol = 1.0e-10;

    internal sealed record Pair(BSpline Law, BSplineCurve2D Curve);

    /// <summary>A random 1D B-spline and its planar twin.</summary>
    internal static Pair RandomPair(Random theRnd, bool? thePeriodic = null, bool? theRational = null, int theMaxDeg = 5)
    {
        while (true)
        {
            int aDeg = theRnd.Next(1, theMaxDeg + 1);
            bool aPer = thePeriodic ?? theRnd.Next(3) == 0;
            bool aRat = theRational ?? theRnd.Next(2) == 0;
            int aNbKnots = theRnd.Next(2, 6);
            var aKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aMults = new NCollection_Array1<int>(1, aNbKnots);
            aKnots[1] = theRnd.NextDouble() * 4 - 2;
            for (int i = 2; i <= aNbKnots; i++)
            {
                aKnots[i] = aKnots[i - 1] + 0.3 + theRnd.NextDouble();
            }
            for (int i = 2; i < aNbKnots; i++)
            {
                aMults[i] = theRnd.Next(1, aDeg + 1);
            }
            aMults[1] = aMults[aNbKnots] = aPer ? theRnd.Next(1, aDeg + 1) : aDeg + 1;
            int aNbPoles = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, aPer, aMults);
            if (aNbPoles < 2 || (aPer && aNbPoles <= aDeg))
            {
                continue;
            }
            var aPoles = new NCollection_Array1<double>(1, aNbPoles);
            var aPoles2d = new NCollection_Array1<Point2D>(1, aNbPoles);
            var aWeights = new NCollection_Array1<double>(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                aPoles[i] = theRnd.NextDouble() * 20 - 10;
                aPoles2d[i] = new Point2D(0.0, aPoles[i]);
                aWeights[i] = 0.5 + theRnd.NextDouble();
            }
            if (aRat)
            {
                return new Pair(new BSpline(aPoles, aWeights, aKnots, aMults, aDeg, aPer),
                                new BSplineCurve2D(aPoles2d, aWeights, aKnots, aMults, aDeg, aPer));
            }
            return new Pair(new BSpline(aPoles, aKnots, aMults, aDeg, aPer),
                            new BSplineCurve2D(aPoles2d, aKnots, aMults, aDeg, aPer));
        }
    }

    private static void AssertSameFunction(BSpline theLaw, BSplineCurve2D theCurve, string theWhat)
    {
        Assert.Equal(theCurve.FirstParameter, theLaw.FirstParameter(), Tol);
        Assert.Equal(theCurve.LastParameter, theLaw.LastParameter(), Tol);
        double aF = theLaw.FirstParameter(), aL = theLaw.LastParameter();
        for (int k = 0; k <= 20; k++)
        {
            double u = aF + (aL - aF) * k / 20.0;
            Assert.True(Math.Abs(theCurve.Value(u).Y - theLaw.Value(u)) < 1.0e-9, $"{theWhat}: value at {u}: {theCurve.Value(u).Y} vs {theLaw.Value(u)}");
        }
    }

    [Fact]
    public void Evaluation_MatchesThePlanarBSpline()
    {
        var aRnd = new Random(1234);
        for (int aCase = 0; aCase < 300; aCase++)
        {
            Pair p = RandomPair(aRnd);
            Assert.Equal(p.Curve.Degree, p.Law.Degree());
            Assert.Equal(p.Curve.PoleCount, p.Law.PoleCount);
            Assert.Equal(p.Curve.Continuity, p.Law.Continuity());
            Assert.Equal(p.Curve.IsRational, p.Law.IsRational);
            Assert.Equal(p.Curve.KnotDistribution, p.Law.KnotDistribution());
            for (int n = 0; n <= 4; n++)
            {
                Assert.Equal(p.Curve.IsCN(n), p.Law.IsCN(n));
            }
            double aF = p.Law.FirstParameter(), aL = p.Law.LastParameter();
            for (int k = 0; k <= 10; k++)
            {
                double u = aF + (aL - aF) * k / 10.0;
                Curve2D.ResD3 r = p.Curve.EvalD3(u);
                p.Law.D3(u, out double P, out double V1, out double V2, out double V3);
                Assert.Equal(r.Point.Y, P, Tol);
                Assert.Equal(r.D1.Y, V1, 1.0e-8);
                Assert.Equal(r.D2.Y, V2, 1.0e-6);
                Assert.Equal(r.D3.Y, V3, 1.0e-4 * Math.Max(1.0, Math.Abs(V3)));
                // D0 to D3 run different BSplCLib paths, equal to rounding
                Assert.Equal(P, p.Law.Value(u), Tol);
                p.Law.D1(u, out double P1, out double D1);
                p.Law.D2(u, out double P2, out double E1, out double E2);
                Assert.Equal(P, P1, Tol);
                Assert.Equal(P, P2, Tol);
                Assert.Equal(V1, D1, 1.0e-8);
                Assert.Equal(V1, E1, 1.0e-8);
                Assert.Equal(V2, E2, 1.0e-6);
                Assert.Equal(p.Curve.EvalDN(u, 1).Y, p.Law.DN(u, 1), 1.0e-8);
                Assert.Equal(p.Curve.EvalDN(u, 2).Y, p.Law.DN(u, 2), 1.0e-6);
            }
            Assert.Equal(p.Curve.StartPoint.Y, p.Law.StartPoint(), Tol);
            Assert.Equal(p.Curve.EndPoint.Y, p.Law.EndPoint(), Tol);
        }
    }

    [Fact]
    public void LocalEvaluation_UsesTheGivenSpan()
    {
        var aRnd = new Random(77);
        for (int aCase = 0; aCase < 100; aCase++)
        {
            Pair p = RandomPair(aRnd, thePeriodic: false);
            int i1 = p.Law.FirstUKnotIndex(), i2 = p.Law.LastUKnotIndex();
            for (int k = i1; k < i2; k++)
            {
                // at the knot k + 1, the span [k, k + 1] gives the left-hand values
                double u = p.Law.Knot(k + 1);
                p.Curve.LocalD3(u, k, k + 1, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3);
                p.Law.LocalD3(u, k, k + 1, out double LP, out double LV1, out double LV2, out double LV3);
                Assert.Equal(P.Y, LP, Tol);
                Assert.Equal(V1.Y, LV1, 1.0e-8);
                Assert.Equal(V2.Y, LV2, 1.0e-6);
                Assert.Equal(V3.Y, LV3, 1.0e-4 * Math.Max(1.0, Math.Abs(LV3)));
                Assert.Equal(LP, p.Law.LocalValue(u, k, k + 1), Tol);
                p.Law.LocalD0(u, k, k + 1, out double LP0);
                p.Law.LocalD1(u, k, k + 1, out double LP1, out double LW1);
                p.Law.LocalD2(u, k, k + 1, out double LP2, out double LX1, out double LX2);
                Assert.Equal(LP, LP0, Tol);
                Assert.Equal(LP, LP1, Tol);
                Assert.Equal(LP, LP2, Tol);
                Assert.Equal(LV1, LW1, 1.0e-8);
                Assert.Equal(LV1, LX1, 1.0e-8);
                Assert.Equal(LV2, LX2, 1.0e-6);
                Assert.Equal(LV1, p.Law.LocalDN(u, k, k + 1, 1), 1.0e-9);
            }
        }
    }

    [Fact]
    public void KnotOperations_KeepTheFunction()
    {
        var aRnd = new Random(4321);
        int aDropped = 0;
        for (int aCase = 0; aCase < 200; aCase++)
        {
            Pair p = RandomPair(aRnd);
            BSpline aLaw = p.Law;
            double aF = aLaw.FirstParameter(), aL = aLaw.LastParameter();

            // knot insertion, as Geom2d does it
            double u = aF + (aL - aF) * (0.2 + 0.6 * aRnd.NextDouble());
            BSpline anIns = aLaw.KnotInserted(u, 1);
            BSplineCurve2D aCIns = (BSplineCurve2D)p.Curve.Copy();
            aCIns.InsertKnot(u, 1);
            Assert.Equal(aCIns.PoleCount, anIns.PoleCount);
            Assert.Equal(aCIns.KnotCount, anIns.KnotCount);
            AssertSameFunction(anIns, p.Curve, "InsertKnot");

            // degree elevation
            BSpline anInc = aLaw.DegreeIncreased(Math.Min(aLaw.Degree() + 2, BSpline.MaxDegree()));
            BSplineCurve2D aCInc = (BSplineCurve2D)p.Curve.Copy();
            aCInc.IncreaseDegree(anInc.Degree());
            Assert.Equal(aCInc.PoleCount, anInc.PoleCount);
            // the same poles as Geom2d's degree elevation
            for (int k = 1; k <= anInc.PoleCount; k++)
            {
                Assert.Equal(aCInc.Pole(k).Y, anInc.Pole(k), 1.0e-9);
            }
            if (!aLaw.IsPeriodic)
            {
                // (BSplCLib's elevation of some periodic curves changes them, in Geom2d as here; not this unit's)
                AssertSameFunction(anInc, p.Curve, "IncreaseDegree");
            }

            // removing the inserted knot gives back the poles
            int anIndex = 0;
            for (int k = 1; k <= anIns.KnotCount; k++)
            {
                if (anIns.Knot(k) == u)
                {
                    anIndex = k;
                }
            }
            if (anIndex > anIns.FirstUKnotIndex() && anIndex < anIns.LastUKnotIndex())
            {
                BSpline? aRem = anIns.KnotRemoved(anIndex, anIns.Multiplicity(anIndex) - 1, 1.0e-7);
                Assert.NotNull(aRem);
                Assert.Equal(aLaw.PoleCount, aRem.PoleCount);
                AssertSameFunction(aRem, p.Curve, "RemoveKnot");
            }

            // segment
            double u1 = aF + (aL - aF) * 0.25 * aRnd.NextDouble();
            double u2 = aL - (aL - aF) * 0.25 * aRnd.NextDouble();
            BSpline aSeg = aLaw.Segmented(u1, u2);
            Assert.False(aSeg.IsPeriodic);
            Assert.Equal(u1, aSeg.FirstParameter(), 1.0e-12);
            // OCCT locates the end at NewU1 + (U2 - U1), which can round below U2 and then keep the poles of a span
            // that doesn't end at a knot of multiplicity Degree: a wrong curve (occt-issues.md;
            // Segment_CanDropTheLastSpan). Only the segments that end at U2 are checked.
            if (Math.Abs(aSeg.LastParameter() - u2) > 1.0e-12)
            {
                Assert.True(aSeg.LastParameter() < u2);
                aDropped++;
            }
            else
            {
                for (int k = 0; k <= 10; k++)
                {
                    double v = u1 + (u2 - u1) * k / 10.0;
                    Assert.Equal(aLaw.Value(v), aSeg.Value(v), 1.0e-9);
                }
            }

            // reverse
            BSpline aRev = aLaw.Reversed();
            for (int k = 0; k <= 10; k++)
            {
                double v = aF + (aL - aF) * k / 10.0;
                Assert.Equal(aLaw.Value(v), aRev.Value(aLaw.ReversedParameter(v)), 1.0e-9);
            }

            // multiplicities
            if (aLaw.KnotCount > 2)
            {
                BSpline aMult = aLaw.MultiplicityIncreased(2, aLaw.Degree());
                // OCCT inserts M - old with Add = false (occt-issues.md): the multiplicity becomes Max(old, M - old)
                int anOld = aLaw.Multiplicity(2);
                Assert.Equal(Math.Max(anOld, aLaw.Degree() - anOld), aMult.Multiplicity(2));
                AssertSameFunction(aMult, p.Curve, "IncreaseMultiplicity");
                BSpline aMult2 = aLaw.MultiplicityIncremented(2, aLaw.KnotCount - 1, 1);
                AssertSameFunction(aMult2, p.Curve, "IncrementMultiplicity");
            }
        }
        Assert.True(aDropped < 20, $"{aDropped} segments dropped their last span");
    }

    [Fact]
    public void Segment_CanDropTheLastSpan()
    {
        // OCCT: U1 + (U2 - U1) is one ulp below the knot U2 inserted, and the strict test against
        // Epsilon(max(|U1|, |U2|)) misses it, so the segment ends at the previous knot
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = -1.193340764936684;
        aKnots[2] = -0.10065871756554523;
        aKnots[3] = 0.2959691968262052;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults.Init(1);
        var aPoles = new NCollection_Array1<double>(1, 2);
        aPoles[1] = 1.0;
        aPoles[2] = 3.0;
        var aLaw = new BSpline(aPoles, aKnots, aMults, 1, true);
        BSpline aSeg = aLaw.Segmented(-0.8366561267389194, 0.18337401436557388);
        Assert.Equal(-0.8366561267389194, aSeg.FirstParameter());
        Assert.Equal(-0.10065871756554523, aSeg.LastParameter());
        // a range whose end rounds exactly is kept whole
        BSpline aSeg2 = aLaw.Segmented(-0.75, 0.25);
        Assert.Equal(0.25, aSeg2.LastParameter());
    }

    [Fact]
    public void Periodicity_KeepsTheFunction()
    {
        var aRnd = new Random(99);
        for (int aCase = 0; aCase < 100; aCase++)
        {
            Pair p = RandomPair(aRnd, thePeriodic: true);
            BSpline aLaw = p.Law;
            Assert.True(aLaw.IsPeriodic);
            double aF = aLaw.FirstParameter(), aL = aLaw.LastParameter();
            double aT = aL - aF;
            // periodic normalisation
            Assert.Equal(aLaw.Value(aF + 0.3 * aT), aLaw.Value(aF + 1.3 * aT), 1.0e-9);
            Assert.Equal(aLaw.Value(aF + 0.3 * aT), aLaw.Value(aF - 0.7 * aT), 1.0e-9);

            BSpline aNP = aLaw.WithNotPeriodic();
            Assert.False(aNP.IsPeriodic);
            BSplineCurve2D aCNP = (BSplineCurve2D)p.Curve.Copy();
            aCNP.SetNotPeriodic();
            Assert.Equal(aCNP.PoleCount, aNP.PoleCount);
            AssertSameFunction(aNP, p.Curve, "SetNotPeriodic");

            if (aLaw.KnotCount > 2)
            {
                BSpline anOrig = aLaw.WithOrigin(2);
                Assert.Equal(aLaw.Knot(2), anOrig.FirstParameter(), 1.0e-12);
                for (int k = 0; k <= 10; k++)
                {
                    double v = anOrig.FirstParameter() + aT * k / 10.0;
                    Assert.Equal(aLaw.Value(v), anOrig.Value(v), 1.0e-9);
                }
            }

            // a non-periodic curve whose ends join with enough continuity made periodic
            BSpline aPer = aNP.WithPeriodic();
            Assert.True(aPer.IsPeriodic);
        }
    }

    [Fact]
    public void PolesWeightsAndKnots_AreCopiedOut()
    {
        var aRnd = new Random(5);
        Pair p = RandomPair(aRnd, thePeriodic: false, theRational: true);
        BSpline aLaw = p.Law;
        var aPoles = new NCollection_Array1<double>(1, aLaw.PoleCount);
        aLaw.Poles(aPoles);
        Assert.Equal(aPoles.AsSpan().ToArray(), aLaw.Poles().ToArray());
        var aWeights = new NCollection_Array1<double>(1, aLaw.PoleCount);
        aLaw.Weights(aWeights);
        Assert.Equal(aWeights.AsSpan().ToArray(), aLaw.Weights()!.ToArray());
        var aKnots = new NCollection_Array1<double>(1, aLaw.KnotCount);
        aLaw.Knots(aKnots);
        Assert.Equal(aKnots.AsSpan().ToArray(), aLaw.Knots().ToArray());
        var aMults = new NCollection_Array1<int>(1, aLaw.KnotCount);
        aLaw.Multiplicities(aMults);
        Assert.Equal(aMults.AsSpan().ToArray(), aLaw.Multiplicities().ToArray());
        var aSeq = new NCollection_Array1<double>(1, aLaw.KnotSequence().Count);
        aLaw.KnotSequence(aSeq);
        Assert.Equal(aSeq.AsSpan().ToArray(), aLaw.KnotSequence().ToArray());
        aPoles[1] = 1.0e10; // the caller's array, not the curve's
        Assert.NotEqual(1.0e10, aLaw.Pole(1));
        EngineAssert.Throws<DimensionErrorException>(() => aLaw.Poles(new NCollection_Array1<double>(1, aLaw.PoleCount + 1)));
    }

    [Fact]
    public void Poles_AndWeights_SetThroughTwins()
    {
        var aRnd = new Random(6);
        Pair p = RandomPair(aRnd, thePeriodic: false, theRational: false);
        BSpline aLaw = p.Law;
        BSpline aPole = aLaw.WithPole(1, 42.0);
        Assert.Equal(42.0, aPole.Pole(1));
        Assert.Equal(42.0, aPole.StartPoint());
        Assert.NotEqual(42.0, aLaw.Pole(1));
        BSpline aW = aLaw.WithWeight(1, 2.0);
        Assert.True(aW.IsRational);
        Assert.Equal(2.0, aW.Weight(1));
        Assert.False(aW.WithWeight(1, 1.0).IsRational); // all weights equal again
        BSpline aPW = aLaw.WithPole(2, 3.0, 2.0);
        Assert.Equal(3.0, aPW.Pole(2));
        Assert.Equal(2.0, aPW.Weight(2));
        EngineAssert.Throws<ConstructionErrorException>(() => aLaw.WithWeight(1, 0.0));
        EngineAssert.Throws<OutOfRangeException>(() => aLaw.WithPole(0, 1.0));
    }

    [Fact]
    public void Knots_SetThroughTwins()
    {
        var aPoles = new NCollection_Array1<double>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = i * i;
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        var aLaw = new BSpline(aPoles, aKnots, aMults, 3);
        Assert.Equal(BSplKnotDistribution.QuasiUniform, aLaw.KnotDistribution());
        BSpline aK = aLaw.WithKnot(2, 1.5);
        Assert.Equal(1.5, aK.Knot(2));
        Assert.Equal(BSplKnotDistribution.NonUniform, aK.KnotDistribution());
        EngineAssert.Throws<ConstructionErrorException>(() => aLaw.WithKnot(2, 2.0));
        BSpline aKM = aLaw.WithKnot(2, 1.25, 3);
        // OCCT's IncreaseMultiplicity inserts 3 - 1 with Add = false (occt-issues.md): multiplicity 2, not 3
        Assert.Equal(2, aKM.Multiplicity(2));
        Assert.Equal(1.25, aKM.Knot(2));
        Assert.Equal(Continuity.C1, aKM.Continuity());
        Assert.Equal(1, aLaw.MultiplicityIncremented(2, 2, 1).Multiplicity(2)); // Max(1, 1), not 2
        var aNewKnots = new NCollection_Array1<double>(1, 3);
        aNewKnots[1] = -1;
        aNewKnots[2] = 0.5;
        aNewKnots[3] = 4;
        BSpline aKs = aLaw.WithKnots(aNewKnots);
        Assert.Equal(-1.0, aKs.FirstParameter());
        Assert.Equal(4.0, aKs.LastParameter());
        Assert.Equal(aLaw.Pole(1), aKs.Value(-1.0), 1.0e-14);
        Assert.Equal(aLaw.Pole(5), aKs.Value(4.0), 1.0e-14);
    }

    [Fact]
    public void LocateU_FindsTheKnots()
    {
        var aPoles = new NCollection_Array1<double>(1, 5);
        aPoles.Init(1.0);
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        var aLaw = new BSpline(aPoles, aKnots, aMults, 3);
        aLaw.LocateU(0.0, 1.0e-9, out int i1, out int i2);
        Assert.Equal((1, 1), (i1, i2));
        aLaw.LocateU(2.0, 1.0e-9, out i1, out i2);
        Assert.Equal((3, 3), (i1, i2));
        aLaw.LocateU(0.5, 1.0e-9, out i1, out i2);
        Assert.Equal((1, 2), (i1, i2));
        aLaw.LocateU(1.0, 1.0e-9, out i1, out i2);
        Assert.Equal((2, 2), (i1, i2));
        aLaw.LocateU(-1.0, 1.0e-9, out i1, out i2);
        Assert.Equal((0, 1), (i1, i2));
        aLaw.LocateU(3.0, 1.0e-9, out i1, out i2);
        Assert.Equal((3, 4), (i1, i2));
        aLaw.LocateU(0.5, 1.0e-9, out i1, out i2, true);
        Assert.Equal((4, 5), (i1, i2));
        Assert.True(aLaw.IsClosed);
    }

    [Fact]
    public void MovePointAndTangent_MovesValueAndDerivative()
    {
        var aRnd = new Random(8);
        for (int aCase = 0; aCase < 50; aCase++)
        {
            Pair p = RandomPair(aRnd, thePeriodic: false, theMaxDeg: 5);
            BSpline aLaw = p.Law;
            if (aLaw.Degree() < 3 || aLaw.PoleCount < 6)
            {
                continue;
            }
            double u = 0.5 * (aLaw.FirstParameter() + aLaw.LastParameter());
            BSpline aMoved = aLaw.PointAndTangentMoved(u, 7.0, -2.0, 1.0e-9, 1, 1, out int anError);
            if (anError != 0)
            {
                continue;
            }
            aMoved.D1(u, out double P, out double D);
            Assert.Equal(7.0, P, 1.0e-8);
            Assert.Equal(-2.0, D, 1.0e-7);
        }
    }

    [Fact]
    public void Resolution_BoundsTheVariation()
    {
        var aRnd = new Random(9);
        for (int aCase = 0; aCase < 50; aCase++)
        {
            Pair p = RandomPair(aRnd);
            p.Law.Resolution(1.0e-3, out double aUTol);
            p.Curve.Resolution(1.0e-3, out double aCTol);
            Assert.True(aUTol > 0);
            // a bound of the derivative on the poles' Y only is at least as small as on (0, Y)
            Assert.Equal(aCTol, aUTol, 1.0e-12 * Math.Max(1.0, aCTol));
        }
    }

    [Fact]
    public void FrozenCurve_RejectsMutators_TwinsAndCopiesWork()
    {
        var aRnd = new Random(10);
        Pair p = RandomPair(aRnd, thePeriodic: false);
        BSpline aLaw = p.Law;
        aLaw.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aLaw.SetPole(1, 1.0));
        EngineAssert.Throws<ImmutableObjectException>(() => aLaw.InsertKnot(0.5 * (aLaw.FirstParameter() + aLaw.LastParameter())));
        EngineAssert.Throws<ImmutableObjectException>(() => aLaw.Reverse());
        EngineAssert.Throws<ImmutableObjectException>(() => aLaw.Segment(aLaw.FirstParameter(), aLaw.LastParameter()));
        BSpline aCopy = aLaw.Copy();
        Assert.False(aCopy.IsFrozen);
        aCopy.SetPole(1, 1.0);
        Assert.Equal(1.0, aCopy.Pole(1));
        Assert.False(aLaw.WithPole(1, 2.0).IsFrozen);
    }

    [Fact]
    public void Construction_ChecksTheData()
    {
        var aPoles = new NCollection_Array1<double>(1, 3);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = aMults[2] = 3;
        EngineAssert.Throws<ConstructionErrorException>(() => new BSpline(aPoles, aKnots, aMults, 3));
        EngineAssert.Throws<ConstructionErrorException>(() => new BSpline(aPoles, aKnots, aMults, 0));
        var aLaw = new BSpline(aPoles, aKnots, aMults, 2);
        Assert.Equal(BSplKnotDistribution.PiecewiseBezier, aLaw.KnotDistribution());
        Assert.Equal(Continuity.CN, aLaw.Continuity());
        var aWeights = new NCollection_Array1<double>(1, 3);
        aWeights.Init(2.0);
        Assert.False(new BSpline(aPoles, aWeights, aKnots, aMults, 2).IsRational);
        aWeights[2] = -1.0;
        EngineAssert.Throws<ConstructionErrorException>(() => new BSpline(aPoles, aWeights, aKnots, aMults, 2));
    }
}
