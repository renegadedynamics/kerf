// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// IntAna2d_AnaIntersection (OCCT has no GTests for it): intersection points lie on both curves, on random inputs, and
/// the special cases report the right flags. Native agreement is checked by the oracle tests.
/// </summary>
public class IntAna2d_AnaIntersectionTest
{
    private const double Tol = 1e-8;

    private static gp_Dir2d RandomDir(Random theRandom)
    {
        double anA = theRandom.NextDouble() * 2 * Math.PI;
        return new gp_Dir2d(Math.Cos(anA), Math.Sin(anA));
    }

    private static gp_Pnt2d RandomPnt(Random theRandom, double theScale = 50)
        => new((theRandom.NextDouble() - 0.5) * 2 * theScale, (theRandom.NextDouble() - 0.5) * 2 * theScale);

    [Fact]
    public void Empty_constructor_is_not_done()
    {
        var anInter = new IntAna2d_AnaIntersection();
        Assert.False(anInter.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anInter.NbPoints());
        Assert.Throws<StdFail_NotDone>(() => anInter.IsEmpty());
    }

    [Fact]
    public void Lines_points_on_both_and_flags()
    {
        var aRandom = new Random(1);
        for (int i = 0; i < 200; i++)
        {
            gp_Lin2d aL1 = new(RandomPnt(aRandom), RandomDir(aRandom));
            gp_Lin2d aL2 = new(RandomPnt(aRandom), RandomDir(aRandom));
            var anInter = new IntAna2d_AnaIntersection(aL1, aL2);
            Assert.True(anInter.IsDone());
            Assert.Equal(1, anInter.NbPoints());
            IntAna2d_IntPoint aP = anInter.Point(1);
            Assert.Equal(0.0, aL1.Distance(aP.Value()), Tol);
            Assert.Equal(0.0, aL2.Distance(aP.Value()), Tol);
            Assert.Equal(0.0, ElCLib.Value(aP.ParamOnFirst(), aL1).Distance(aP.Value()), Tol);
            Assert.Equal(0.0, ElCLib.Value(aP.ParamOnSecond(), aL2).Distance(aP.Value()), Tol);
            Assert.Throws<Standard_OutOfRange>(() => anInter.Point(2));
        }

        gp_Lin2d aL = new(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 1));
        var aParallel = new IntAna2d_AnaIntersection(aL, new gp_Lin2d(new gp_Pnt2d(1, 3), new gp_Dir2d(1, 1)));
        Assert.True(aParallel.ParallelElements());
        Assert.True(aParallel.IsEmpty());
        Assert.False(aParallel.IdenticalElements());

        var anIdentical = new IntAna2d_AnaIntersection(aL, new gp_Lin2d(new gp_Pnt2d(3, 4), new gp_Dir2d(-1, -1)));
        Assert.True(anIdentical.ParallelElements());
        Assert.True(anIdentical.IdenticalElements());
        Assert.False(anIdentical.IsEmpty());
    }

    [Fact]
    public void Circles_points_on_both()
    {
        var aRandom = new Random(2);
        int aCrossings = 0;
        for (int i = 0; i < 300; i++)
        {
            gp_Circ2d aC1 = new(new gp_Ax22d(RandomPnt(aRandom, 10), RandomDir(aRandom), aRandom.Next(2) == 0), 1 + 20 * aRandom.NextDouble());
            gp_Circ2d aC2 = new(new gp_Ax22d(RandomPnt(aRandom, 10), RandomDir(aRandom), aRandom.Next(2) == 0), 1 + 20 * aRandom.NextDouble());
            var anInter = new IntAna2d_AnaIntersection(aC1, aC2);
            Assert.True(anInter.IsDone());
            for (int j = 1; j <= anInter.NbPoints(); j++)
            {
                aCrossings++;
                IntAna2d_IntPoint aP = anInter.Point(j);
                Assert.Equal(0.0, aC1.Distance(aP.Value()), Tol);
                Assert.Equal(0.0, aC2.Distance(aP.Value()), Tol);
                // OCCT measures the parameters from the X axes counterclockwise, whatever the sense of the circles
                // (occt-issues.md), so they are right for direct circles only
                if (aC1.IsDirect())
                {
                    Assert.Equal(0.0, ElCLib.Value(aP.ParamOnFirst(), aC1).Distance(aP.Value()), 1e-6);
                }
                if (aC2.IsDirect())
                {
                    Assert.Equal(0.0, ElCLib.Value(aP.ParamOnSecond(), aC2).Distance(aP.Value()), 1e-6);
                }
            }
        }

        Assert.True(aCrossings > 100);

        gp_Circ2d aC = new(new gp_Ax2d(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0)), 5);
        var aSame = new IntAna2d_AnaIntersection(aC, aC);
        Assert.True(aSame.IdenticalElements());
        var aConcentric = new IntAna2d_AnaIntersection(aC, new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(1, 1), new gp_Dir2d(0, 1)), 3));
        Assert.True(aConcentric.ParallelElements());
        Assert.True(aConcentric.IsEmpty());
    }

    [Fact]
    public void Line_and_circle_agree_with_the_implicit_conic()
    {
        var aRandom = new Random(3);
        for (int i = 0; i < 300; i++)
        {
            gp_Circ2d aC = new(new gp_Ax22d(RandomPnt(aRandom, 10), RandomDir(aRandom), aRandom.Next(2) == 0), 1 + 20 * aRandom.NextDouble());
            gp_Lin2d aL = new(RandomPnt(aRandom, 15), RandomDir(aRandom));
            var anInter = new IntAna2d_AnaIntersection(aL, aC);
            var aConic = new IntAna2d_AnaIntersection(aL, new IntAna2d_Conic(aC));
            Assert.True(anInter.IsDone());
            Assert.True(aConic.IsDone());
            Assert.Equal(anInter.NbPoints(), aConic.NbPoints());
            for (int j = 1; j <= anInter.NbPoints(); j++)
            {
                IntAna2d_IntPoint aP = anInter.Point(j);
                Assert.Equal(0.0, aC.Distance(aP.Value()), Tol);
                Assert.Equal(0.0, aL.Distance(aP.Value()), Tol);
                Assert.True(aConic.Point(j).SecondIsImplicit());
                Assert.Throws<Standard_DomainError>(() => aConic.Point(j).ParamOnSecond());
            }
        }
    }

    [Fact]
    public void Conics_against_an_implicit_line_or_circle()
    {
        var aRandom = new Random(4);
        for (int i = 0; i < 300; i++)
        {
            gp_Ax22d anAx = new(RandomPnt(aRandom, 5), RandomDir(aRandom), aRandom.Next(2) == 0);
            gp_Lin2d aL = new(RandomPnt(aRandom, 5), RandomDir(aRandom));
            gp_Circ2d aC = new(new gp_Ax22d(RandomPnt(aRandom, 5), RandomDir(aRandom)), 2 + 10 * aRandom.NextDouble());
            IntAna2d_Conic anImplicit = i % 2 == 0 ? new IntAna2d_Conic(aL) : new IntAna2d_Conic(aC);
            double aMinor = 1 + 5 * aRandom.NextDouble();

            var aCirc = new gp_Circ2d(anAx, aMinor);
            Check(new IntAna2d_AnaIntersection(aCirc, anImplicit), anImplicit, theU => ElCLib.Value(theU, aCirc));
            var anElips = new gp_Elips2d(anAx, aMinor + 3, aMinor);
            Check(new IntAna2d_AnaIntersection(anElips, anImplicit), anImplicit, theU => ElCLib.Value(theU, anElips));
            var aParab = new gp_Parab2d(anAx, aMinor);
            Check(new IntAna2d_AnaIntersection(aParab, anImplicit), anImplicit, theU => ElCLib.Value(theU, aParab));
            var aHypr = new gp_Hypr2d(anAx, aMinor + 1, aMinor);
            Check(new IntAna2d_AnaIntersection(aHypr, anImplicit), anImplicit, theU => ElCLib.Value(theU, aHypr));
        }

        static void Check(IntAna2d_AnaIntersection theInter, IntAna2d_Conic theConic, Func<double, gp_Pnt2d> theCurve)
        {
            if (!theInter.IsDone())
            {
                // A hyperbola that doesn't meet the conic (occt-issues.md)
                return;
            }
            for (int j = 1; j <= theInter.NbPoints(); j++)
            {
                IntAna2d_IntPoint aP = theInter.Point(j);
                gp_XY aGrad = theConic.Grad(aP.Value().X(), aP.Value().Y());
                // Distance to the implicit curve, to first order
                Assert.Equal(0.0, theConic.Value(aP.Value().X(), aP.Value().Y()) / Math.Max(1.0, aGrad.Modulus()), 1e-6);
                Assert.Equal(0.0, theCurve(aP.ParamOnFirst()).Distance(aP.Value()), 1e-6);
            }
        }
    }

    [Fact]
    public void Indirect_circles_get_counterclockwise_parameters()
    {
        // Reproduces OCCT (occt-issues.md): the parameter on an indirect circle is measured counterclockwise
        var aC1 = new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 5, false);
        var aC2 = new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(6, 0), new gp_Dir2d(1, 0)), 5);
        var anInter = new IntAna2d_AnaIntersection(aC1, aC2);
        Assert.Equal(2, anInter.NbPoints());
        IntAna2d_IntPoint aP = anInter.Point(1);
        Assert.Equal(0.0, aP.Value().Distance(new gp_Pnt2d(3, 4)), 1e-12);
        Assert.Equal(Math.Atan2(4, 3), aP.ParamOnFirst(), 1e-12);
        Assert.Equal(0.0, ElCLib.Value(aP.ParamOnFirst(), aC1).Distance(new gp_Pnt2d(3, -4)), 1e-12);
    }

    [Fact]
    public void Hyperbola_missing_the_conic_is_not_done()
    {
        // Reproduces OCCT (occt-issues.md): with no root, the hyperbola case reports IsDone false, not an empty result
        var aHypr = new gp_Hypr2d(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2, 1);
        var anInter = new IntAna2d_AnaIntersection(aHypr, new IntAna2d_Conic(new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(0, 50), new gp_Dir2d(1, 0)), 1)));
        Assert.False(anInter.IsDone());

        // while a parabola that misses it is done and empty
        var aParab = new gp_Parab2d(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1);
        var anInterP = new IntAna2d_AnaIntersection(aParab, new IntAna2d_Conic(new gp_Lin2d(new gp_Pnt2d(-1, 0), new gp_Dir2d(0, 1))));
        Assert.True(anInterP.IsDone());
        Assert.True(anInterP.IsEmpty());
    }
}
