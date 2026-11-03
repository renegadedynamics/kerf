// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// IntAna2d_AnaIntersection (OCCT has no GTests for it): intersection points lie on both curves, on random inputs, and
/// the special cases report the right flags. Native agreement is checked by the oracle tests.
/// </summary>
public class IntAna2d_AnaIntersectionTest
{
    private const double Tol = 1e-8;

    private static Direction2D RandomDir(Random theRandom)
    {
        double anA = theRandom.NextDouble() * 2 * Math.PI;
        return new Direction2D(Math.Cos(anA), Math.Sin(anA));
    }

    private static Point2D RandomPnt(Random theRandom, double theScale = 50)
        => new((theRandom.NextDouble() - 0.5) * 2 * theScale, (theRandom.NextDouble() - 0.5) * 2 * theScale);

    [Fact]
    public void Empty_constructor_is_not_done()
    {
        var anInter = new AnalyticIntersection();
        Assert.False(anInter.IsDone);
        Assert.Throws<NotDoneException>(() => anInter.PointCount);
        Assert.Throws<NotDoneException>(() => anInter.IsEmpty);
    }

    [Fact]
    public void Lines_points_on_both_and_flags()
    {
        var aRandom = new Random(1);
        for (int i = 0; i < 200; i++)
        {
            Line2D aL1 = new(RandomPnt(aRandom), RandomDir(aRandom));
            Line2D aL2 = new(RandomPnt(aRandom), RandomDir(aRandom));
            var anInter = new AnalyticIntersection(aL1, aL2);
            Assert.True(anInter.IsDone);
            Assert.Equal(1, anInter.PointCount);
            IntPoint aP = anInter.Point(1);
            Assert.Equal(0.0, aL1.DistanceTo(aP.Value()), Tol);
            Assert.Equal(0.0, aL2.DistanceTo(aP.Value()), Tol);
            Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aP.ParamOnFirst(), aL1).DistanceTo(aP.Value()), Tol);
            Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aP.ParamOnSecond(), aL2).DistanceTo(aP.Value()), Tol);
            Assert.Throws<OutOfRangeException>(() => anInter.Point(2));
        }

        Line2D aL = new(new Point2D(1, 2), new Direction2D(1, 1));
        var aParallel = new AnalyticIntersection(aL, new Line2D(new Point2D(1, 3), new Direction2D(1, 1)));
        Assert.True(aParallel.ParallelElements());
        Assert.True(aParallel.IsEmpty);
        Assert.False(aParallel.IdenticalElements());

        var anIdentical = new AnalyticIntersection(aL, new Line2D(new Point2D(3, 4), new Direction2D(-1, -1)));
        Assert.True(anIdentical.ParallelElements());
        Assert.True(anIdentical.IdenticalElements());
        Assert.False(anIdentical.IsEmpty);
    }

    [Fact]
    public void Circles_points_on_both()
    {
        var aRandom = new Random(2);
        int aCrossings = 0;
        for (int i = 0; i < 300; i++)
        {
            Circle2D aC1 = new(new Frame2D(RandomPnt(aRandom, 10), RandomDir(aRandom), aRandom.Next(2) == 0), 1 + 20 * aRandom.NextDouble());
            Circle2D aC2 = new(new Frame2D(RandomPnt(aRandom, 10), RandomDir(aRandom), aRandom.Next(2) == 0), 1 + 20 * aRandom.NextDouble());
            var anInter = new AnalyticIntersection(aC1, aC2);
            Assert.True(anInter.IsDone);
            for (int j = 1; j <= anInter.PointCount; j++)
            {
                aCrossings++;
                IntPoint aP = anInter.Point(j);
                Assert.Equal(0.0, aC1.DistanceTo(aP.Value()), Tol);
                Assert.Equal(0.0, aC2.DistanceTo(aP.Value()), Tol);
                // OCCT measures the parameters from the X axes counterclockwise, whatever the sense of the circles
                // (occt-issues.md), so they are right for direct circles only
                if (aC1.IsDirect)
                {
                    Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aP.ParamOnFirst(), aC1).DistanceTo(aP.Value()), 1e-6);
                }
                if (aC2.IsDirect)
                {
                    Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aP.ParamOnSecond(), aC2).DistanceTo(aP.Value()), 1e-6);
                }
            }
        }

        Assert.True(aCrossings > 100);

        Circle2D aC = new(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 5);
        var aSame = new AnalyticIntersection(aC, aC);
        Assert.True(aSame.IdenticalElements());
        var aConcentric = new AnalyticIntersection(aC, new Circle2D(new Axis2D(new Point2D(1, 1), new Direction2D(0, 1)), 3));
        Assert.True(aConcentric.ParallelElements());
        Assert.True(aConcentric.IsEmpty);
    }

    [Fact]
    public void Line_and_circle_agree_with_the_implicit_conic()
    {
        var aRandom = new Random(3);
        for (int i = 0; i < 300; i++)
        {
            Circle2D aC = new(new Frame2D(RandomPnt(aRandom, 10), RandomDir(aRandom), aRandom.Next(2) == 0), 1 + 20 * aRandom.NextDouble());
            Line2D aL = new(RandomPnt(aRandom, 15), RandomDir(aRandom));
            var anInter = new AnalyticIntersection(aL, aC);
            var aConic = new AnalyticIntersection(aL, new Conic(aC));
            Assert.True(anInter.IsDone);
            Assert.True(aConic.IsDone);
            Assert.Equal(anInter.PointCount, aConic.PointCount);
            for (int j = 1; j <= anInter.PointCount; j++)
            {
                IntPoint aP = anInter.Point(j);
                Assert.Equal(0.0, aC.DistanceTo(aP.Value()), Tol);
                Assert.Equal(0.0, aL.DistanceTo(aP.Value()), Tol);
                Assert.True(aConic.Point(j).SecondIsImplicit());
                Assert.Throws<DomainErrorException>(() => aConic.Point(j).ParamOnSecond());
            }
        }
    }

    [Fact]
    public void Conics_against_an_implicit_line_or_circle()
    {
        var aRandom = new Random(4);
        for (int i = 0; i < 300; i++)
        {
            Frame2D anAx = new(RandomPnt(aRandom, 5), RandomDir(aRandom), aRandom.Next(2) == 0);
            Line2D aL = new(RandomPnt(aRandom, 5), RandomDir(aRandom));
            Circle2D aC = new(new Frame2D(RandomPnt(aRandom, 5), RandomDir(aRandom)), 2 + 10 * aRandom.NextDouble());
            Conic anImplicit = i % 2 == 0 ? new Conic(aL) : new Conic(aC);
            double aMinor = 1 + 5 * aRandom.NextDouble();

            var aCirc = new Circle2D(anAx, aMinor);
            Check(new AnalyticIntersection(aCirc, anImplicit), anImplicit, theU => global::Kerf.Engine.Geometry.ElCLib.Value(theU, aCirc));
            var anElips = new Ellipse2D(anAx, aMinor + 3, aMinor);
            Check(new AnalyticIntersection(anElips, anImplicit), anImplicit, theU => global::Kerf.Engine.Geometry.ElCLib.Value(theU, anElips));
            var aParab = new Parabola2D(anAx, aMinor);
            Check(new AnalyticIntersection(aParab, anImplicit), anImplicit, theU => global::Kerf.Engine.Geometry.ElCLib.Value(theU, aParab));
            var aHypr = new Hyperbola2D(anAx, aMinor + 1, aMinor);
            Check(new AnalyticIntersection(aHypr, anImplicit), anImplicit, theU => global::Kerf.Engine.Geometry.ElCLib.Value(theU, aHypr));
        }

        static void Check(AnalyticIntersection theInter, Conic theConic, Func<double, Point2D> theCurve)
        {
            if (!theInter.IsDone)
            {
                // A hyperbola that doesn't meet the conic (occt-issues.md)
                return;
            }
            for (int j = 1; j <= theInter.PointCount; j++)
            {
                IntPoint aP = theInter.Point(j);
                global::Kerf.Engine.Geometry.Xy aGrad = theConic.Grad(aP.Value().X, aP.Value().Y);
                // Distance to the implicit curve, to first order
                Assert.Equal(0.0, theConic.Value(aP.Value().X, aP.Value().Y) / Math.Max(1.0, aGrad.Modulus()), 1e-6);
                Assert.Equal(0.0, theCurve(aP.ParamOnFirst()).DistanceTo(aP.Value()), 1e-6);
            }
        }
    }

    [Fact]
    public void Indirect_circles_get_counterclockwise_parameters()
    {
        // Reproduces OCCT (occt-issues.md): the parameter on an indirect circle is measured counterclockwise
        var aC1 = new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 5, false);
        var aC2 = new Circle2D(new Axis2D(new Point2D(6, 0), new Direction2D(1, 0)), 5);
        var anInter = new AnalyticIntersection(aC1, aC2);
        Assert.Equal(2, anInter.PointCount);
        IntPoint aP = anInter.Point(1);
        Assert.Equal(0.0, aP.Value().DistanceTo(new Point2D(3, 4)), 1e-12);
        Assert.Equal(Math.Atan2(4, 3), aP.ParamOnFirst(), 1e-12);
        Assert.Equal(0.0, global::Kerf.Engine.Geometry.ElCLib.Value(aP.ParamOnFirst(), aC1).DistanceTo(new Point2D(3, -4)), 1e-12);
    }

    [Fact]
    public void Hyperbola_missing_the_conic_is_not_done()
    {
        // Reproduces OCCT (occt-issues.md): with no root, the hyperbola case reports IsDone false, not an empty result
        var aHypr = new Hyperbola2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2, 1);
        var anInter = new AnalyticIntersection(aHypr, new Conic(new Circle2D(new Axis2D(new Point2D(0, 50), new Direction2D(1, 0)), 1)));
        Assert.False(anInter.IsDone);

        // while a parabola that misses it is done and empty
        var aParab = new Parabola2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        var anInterP = new AnalyticIntersection(aParab, new Conic(new Line2D(new Point2D(-1, 0), new Direction2D(0, 1))));
        Assert.True(anInterP.IsDone);
        Assert.True(anInterP.IsEmpty);
    }
}
