// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// A frozen 2D B-spline keeps the boxes of the arcs it was bounded over (<c>BSplineCurve2dBounds.Box</c>): each box is
/// the very one an unfrozen copy of the curve gives, the first time and every time after, at any tolerance, and past
/// the number of arcs kept.
/// </summary>
public class ArcBoxKeptTests
{
    private static BSplineCurve2D Spline(bool thePeriodic)
    {
        int aCount = thePeriodic ? 6 : 7;
        NCollection_Array1<Point2D> aPoles = new(1, aCount);
        for (int i = 1; i <= aCount; i++)
        {
            double anAngle = 2 * Math.PI * i / aCount;
            aPoles.SetValue(i, new Point2D(3 * Math.Cos(anAngle) + 0.1 * i, 2 * Math.Sin(anAngle) - 0.05 * i * i));
        }

        int aKnotCount = thePeriodic ? 7 : 6;
        NCollection_Array1<double> aKnots = new(1, aKnotCount);
        NCollection_Array1<int> aMults = new(1, aKnotCount);
        for (int i = 1; i <= aKnotCount; i++)
        {
            aKnots.SetValue(i, (i - 1) * 0.7);
            aMults.SetValue(i, thePeriodic || (i > 1 && i < aKnotCount) ? 1 : 3);
        }

        return new BSplineCurve2D(aPoles, aKnots, aMults, 2, thePeriodic);
    }

    private static Box2d.Limits Limits(in Box2d theBox) => theBox.Get();

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AFrozenCurvesArcBoxes_AreAnUnfrozenCopysExactly(bool thePeriodic)
    {
        BSplineCurve2D aFrozen = Spline(thePeriodic);
        aFrozen.Freeze();
        var aRandom = new Random(23401);
        double aFirst = aFrozen.FirstParameter;
        double aLength = aFrozen.LastParameter - aFirst;

        // more arcs than a curve keeps, each asked for several times and at several tolerances
        var anArcs = new List<(double, double)>();
        // arcs that share their first end, and arcs that share their last
        for (int i = 0; i < 25; i++)
        {
            double aU1 = aFirst + aRandom.NextDouble() * aLength * (thePeriodic ? 1.5 : 0.5);
            for (int j = 0; j < 4; j++)
            {
                anArcs.Add((aU1, aU1 + (0.1 + aRandom.NextDouble()) * aLength * 0.4));
            }
            anArcs.Add((aU1 - aLength * 0.2, anArcs[^1].Item2));
        }

        for (int aPass = 0; aPass < 3; aPass++)
        {
            foreach ((double aU1, double aU2) in anArcs)
            {
                double aTol = aPass * 1e-3;
                BSplineCurve2D aFresh = (BSplineCurve2D)aFrozen.Copy();
                Assert.False(aFresh.IsFrozen);
                Box2d anExpected = new BSplineCurve2dBounds(aFresh).Box(aU1, aU2, aTol);
                Box2d aKept = new BSplineCurve2dBounds(aFrozen).Box(aU1, aU2, aTol);
                Assert.Equal(Limits(anExpected), Limits(aKept));
                Assert.Equal(anExpected.IsVoid, aKept.IsVoid);
            }
        }

        // the boxes were kept, and answered from (an open curve's arcs are trimmed only at its ends)
        if (!thePeriodic)
        {
            Assert.Contains(anArcs, a => aFrozen.KeptArcBox(a.Item1, Math.Min(a.Item2, aFrozen.LastParameter)) is not null);
        }
    }

    [Fact]
    public void AnUnfrozenCurve_KeepsNoBox()
    {
        BSplineCurve2D aCurve = Spline(false);
        Box2d aBefore = new BSplineCurve2dBounds(aCurve).Box(0.3, 2.1, 0.0);
        Assert.Null(aCurve.KeptArcBox(0.3, 2.1));

        // a change to the curve changes its box, as nothing was kept
        aCurve.Translate(new Vector2D(5.0, 0.0));
        Box2d anAfter = new BSplineCurve2dBounds(aCurve).Box(0.3, 2.1, 0.0);
        Assert.Equal(aBefore.Get().Xmin + 5.0, anAfter.Get().Xmin, 12);
    }
}
