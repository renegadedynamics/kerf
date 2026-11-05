// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Hermit's a(u) under weights steep enough that the plain Hermite cubic would go negative, so knots must be inserted to keep
/// its sign (unit 11I). Hermit.hxx's promise: a(u)D(u) = 1 at both ends, and a(u) of constant sign; or the tolerances can't
/// be met, which is a DimensionError. HermitTests has the gentle weights.
/// </summary>
public class HermitWeightsTests(ITestOutputHelper output)
{
    /// <summary>Weights of a Bezier-form curve: the end values and steepness of D(u) reach each branch.</summary>
    public static TheoryData<double[]> Steep => new()
    {
        new[] { 1.0, 100.0, 1.0 },
        new[] { 1.0, 0.01, 1.0 },
        new[] { 100.0, 1.0, 100.0 },
        new[] { 1.0, 1000.0, 1.0 },
        new[] { 1000.0, 1.0, 1000.0 },
        new[] { 1.0, 50.0, 2.0 },
        new[] { 2.0, 50.0, 1.0 },
        new[] { 1.0, 0.1, 50.0 },
        new[] { 50.0, 0.1, 1.0 },
        new[] { 1.0, 10.0, 0.01 },
        new[] { 0.01, 10.0, 1.0 },
        new[] { 1.0, 100.0, 100.0, 1.0 },
        new[] { 1.0, 100.0, 1.0, 1.0 },
        new[] { 1.0, 1.0, 100.0, 1.0 },
        new[] { 1.0, 0.01, 100.0, 1.0 },
        new[] { 1.0, 100.0, 0.01, 1.0 },
        new[] { 100.0, 0.01, 0.01, 100.0 },
        new[] { 5.0, 500.0, 0.05, 0.5 },
        new[] { 0.5, 0.05, 500.0, 5.0 },
    };

    private static double Denominator(double[] theWeights, double theU)
    {
        // the Bezier form of the weights' polynomial, by de Casteljau
        double[] aW = (double[])theWeights.Clone();
        for (int aLevel = aW.Length - 1; aLevel > 0; aLevel--)
        {
            for (int i = 0; i < aLevel; i++)
            {
                aW[i] = (1 - theU) * aW[i] + theU * aW[i + 1];
            }
        }
        return aW[0];
    }

    private static BSplineCurve3D Curve3D(double[] theWeights)
    {
        Point3D[] aPoles = theWeights.Select((_, i) => new Point3D(i, i % 2, 0)).ToArray();
        int aDegree = theWeights.Length - 1;
        var aPolesArray = new NCollection_Array1<Point3D>(1, aPoles.Length);
        var aWeightsArray = new NCollection_Array1<double>(1, aPoles.Length);
        for (int i = 0; i < aPoles.Length; i++)
        {
            aPolesArray[i + 1] = aPoles[i];
            aWeightsArray[i + 1] = theWeights[i];
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = aMults[2] = aDegree + 1;
        return new BSplineCurve3D(aPolesArray, aWeightsArray, aKnots, aMults, aDegree);
    }

    private static BSplineCurve2D Curve2D(double[] theWeights)
    {
        int aDegree = theWeights.Length - 1;
        var aPolesArray = new NCollection_Array1<Point2D>(1, theWeights.Length);
        var aWeightsArray = new NCollection_Array1<double>(1, theWeights.Length);
        for (int i = 0; i < theWeights.Length; i++)
        {
            aPolesArray[i + 1] = new Point2D(i, i % 2);
            aWeightsArray[i + 1] = theWeights[i];
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = aMults[2] = aDegree + 1;
        return new BSplineCurve2D(aPolesArray, aWeightsArray, aKnots, aMults, aDegree);
    }

    private void TheReparameterisation(double[] theWeights, Func<BSplineCurve2D> theSolution)
    {
        BSplineCurve2D aResult;
        try
        {
            aResult = theSolution();
        }
        catch (Exception anError) when (Refusal.Unwrapped(anError) is DimensionErrorException)
        {
            // the documented refusal
            output.WriteLine("refused: " + string.Join(' ', theWeights));
            return;
        }

        Point2D aFirst = aResult.Value(aResult.FirstParameter), aLast = aResult.Value(aResult.LastParameter);
        Assert.Equal(1.0, aFirst.Y * Denominator(theWeights, 0), 3);
        Assert.Equal(1.0, aLast.Y * Denominator(theWeights, 1), 3);
        for (int i = 0; i <= 50; i++)
        {
            Point2D aPoint = aResult.Value(aResult.FirstParameter + i * (aResult.LastParameter - aResult.FirstParameter) / 50);
            Assert.True(aPoint.Y > 0, $"a({i / 50.0}) = {aPoint.Y}");
        }
    }

    [Theory]
    [MemberData(nameof(Steep))]
    public void A_stays_positive_and_meets_one_over_the_weights_at_the_ends_3d(double[] theWeights)
        => TheReparameterisation(theWeights, () => Hermit.Solution(Curve3D(theWeights)));

    [Theory]
    [MemberData(nameof(Steep))]
    public void A_stays_positive_and_meets_one_over_the_weights_at_the_ends_2d(double[] theWeights)
        => TheReparameterisation(theWeights, () => Hermit.Solution(Curve2D(theWeights)));

    [Theory]
    [MemberData(nameof(Steep))]
    public void The_knots_to_insert_lie_within_the_curve(double[] theWeights)
    {
        double aKnotMin = 0, aKnotMax = 1;
        try
        {
            Hermit.Solutionbis(Curve3D(theWeights), ref aKnotMin, ref aKnotMax);
        }
        catch (Exception anError) when (Refusal.Unwrapped(anError) is DimensionErrorException)
        {
            return;
        }
        Assert.InRange(aKnotMin, 0.0, 1.0);
        Assert.InRange(aKnotMax, 0.0, 1.0);
    }

    [Theory]
    [MemberData(nameof(Steep))]
    public void Tighter_tolerances_on_the_poles_keep_the_promise_or_refuse(double[] theWeights)
    {
        foreach (double aTolerance in new[] { 0.5, 0.1, 0.01 })
        {
            foreach (double aKnots in new[] { 1e-6, 0.1 })
            {
                TheReparameterisation(theWeights, () => Hermit.Solution(Curve3D(theWeights), aTolerance, aKnots));
                TheReparameterisation(theWeights, () => Hermit.Solution(Curve2D(theWeights), aTolerance, aKnots));
                double aKnotMin = 0, aKnotMax = 1;
                try
                {
                    Hermit.Solutionbis(Curve3D(theWeights), ref aKnotMin, ref aKnotMax, aTolerance, aKnots);
                    Assert.InRange(aKnotMin, 0.0, 1.0);
                    Assert.InRange(aKnotMax, 0.0, 1.0);
                }
                catch (Exception anError) when (Refusal.Unwrapped(anError) is DimensionErrorException)
                {
                }
            }
        }
    }

    [Fact]
    public void Steep_weights_within_the_tolerances_are_solved_not_refused()
    {
        // a(u) would go negative here without the knots it inserts
        BSplineCurve2D aResult = Hermit.Solution(Curve3D([1.0, 100.0, 1.0]));
        Assert.True(aResult.KnotCount > 2);
        Assert.NotNull(Hermit.Solution(Curve3D([1.0, 0.01, 1.0])));
        Assert.NotNull(Hermit.Solution(Curve2D([1.0, 100.0, 100.0, 1.0])));
    }

    [Fact]
    public void Tolerances_that_cannot_be_met_are_refused()
    {
        // end values of a in the ratio 1e8, beyond the tolerance of 1e-6 on the poles' ratio
        EngineAssert.Throws<DimensionErrorException>(() => Hermit.Solution(Curve3D([1e-4, 1.0, 1e4])));
        EngineAssert.Throws<DimensionErrorException>(() => Hermit.Solution(Curve2D([1e-4, 1.0, 1e4])));
        double aMin = 0, aMax = 1;
        Assert.ThrowsAny<Exception>(() => Hermit.Solutionbis(Curve3D([1e-4, 1.0, 1e4]), ref aMin, ref aMax));
    }
}
