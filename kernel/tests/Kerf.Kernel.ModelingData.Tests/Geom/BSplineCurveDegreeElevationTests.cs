// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Raising the degree of a periodic B-spline keeps the curve (occt-issues.md, unit 1's BSplCLib::IncreaseDegree row;
// Kerf unit 10K3, decision 1484). OCCT aligned the poles of its non-periodic working curve with the knots at the
// wrong end, so most periodic curves whose multiplicities aren't all 1 changed shape.
public class BSplineCurveDegreeElevationTests
{
    // a random periodic knot vector: first and last multiplicities equal, every multiplicity at most the degree
    private static void PeriodicKnots(Random theR, int theDegree, out NCollection_Array1<double> theKnots,
                                      out NCollection_Array1<int> theMults)
    {
        int n = theR.Next(3, 8);
        theKnots = new NCollection_Array1<double>(1, n);
        theMults = new NCollection_Array1<int>(1, n);
        double u = theR.NextDouble() * 4 - 2;
        for (int i = 1; i <= n; i++)
        {
            theKnots[i] = u;
            u += 0.1 + theR.NextDouble();
            theMults[i] = theR.Next(1, theDegree + 1);
        }
        theMults[n] = theMults[1];
    }

    private static double Value(Random theR) => theR.NextDouble() * 20 - 10;

    [Fact]
    public void Periodic_curves_keep_their_shape()
    {
        Random aR = new(70);
        int aChecked = 0;
        for (int aCase = 0; aCase < 200; aCase++)
        {
            int aDeg = aR.Next(1, 6);
            PeriodicKnots(aR, aDeg, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
            int aNbPoles = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, true, aMults);
            if (aNbPoles <= aDeg)
            {
                continue;
            }
            NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
            NCollection_Array1<double> aWeights = new(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                aPoles[i] = new Point3D(Value(aR), Value(aR), Value(aR));
                aWeights[i] = 0.5 + aR.NextDouble();
            }
            bool aRational = aR.Next(2) == 0;
            BSplineCurve3D aCurve = aRational ? new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg, true)
                                              : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg, true);
            int aNewDeg = Math.Min(BSplineCurve3D.MaxDegree(), aDeg + aR.Next(1, 4));
            BSplineCurve3D aRaised = aCurve.DegreeIncreased(aNewDeg);
            Assert.Equal(aNewDeg, aRaised.Degree());
            Assert.True(aRaised.IsPeriodic);
            double aF = aCurve.FirstParameter(), aL = aCurve.LastParameter();
            for (int i = 0; i <= 40; i++)
            {
                double u = aF + (aL - aF) * i / 40;
                Point3D aBefore = aCurve.Value(u), anAfter = aRaised.Value(u);
                Assert.True(aBefore.Distance(anAfter) < 1e-9 * (1 + aBefore.Distance(new Point3D(0, 0, 0))),
                            $"case {aCase}: degree {aDeg} to {aNewDeg}, mults {string.Join(",", aMults.AsSpan().ToArray())}, "
                            + $"u = {u}: {aBefore.X()},{aBefore.Y()},{aBefore.Z()} became {anAfter.X()},{anAfter.Y()},{anAfter.Z()}");
            }
            aChecked++;
        }
        Assert.True(aChecked > 150);
    }

    [Fact]
    public void Periodic_2d_curves_keep_their_shape()
    {
        Random aR = new(71);
        for (int aCase = 0; aCase < 200; aCase++)
        {
            int aDeg = aR.Next(1, 6);
            PeriodicKnots(aR, aDeg, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
            int aNbPoles = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, true, aMults);
            if (aNbPoles <= aDeg)
            {
                continue;
            }
            NCollection_Array1<Point2D> aPoles = new(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                aPoles[i] = new Point2D(Value(aR), Value(aR));
            }
            BSplineCurve2D aCurve = new(aPoles, aKnots, aMults, aDeg, true);
            BSplineCurve2D aRaised = aCurve.DegreeIncreased(aDeg + aR.Next(1, 4));
            double aF = aCurve.FirstParameter(), aL = aCurve.LastParameter();
            for (int i = 0; i <= 40; i++)
            {
                double u = aF + (aL - aF) * i / 40;
                Assert.True(aCurve.Value(u).Distance(aRaised.Value(u)) < 1e-9 * 30, $"case {aCase}, u = {u}");
            }
        }
    }

    // the example a user meets: a closed profile with a double knot, raised to cubic (a loft's sections are unified so)
    [Fact]
    public void A_periodic_quadratic_with_a_double_knot_raised_to_cubic()
    {
        NCollection_Array1<Point3D> aPoles = NCollection_Array1<Point3D>.From(
            [new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(-1, 1, 0), new(-1, 0, 0), new(0, -1, 0)]);
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([1, 1, 2, 1, 1, 1]);
        BSplineCurve3D aCurve = new(aPoles, aKnots, aMults, 2, true);
        BSplineCurve3D aRaised = aCurve.DegreeIncreased(3);
        for (int i = 0; i <= 64; i++)
        {
            double u = 5.0 * i / 64;
            Assert.True(aCurve.Value(u).Distance(aRaised.Value(u)) < 1e-12, $"u = {u}");
        }
    }
}
