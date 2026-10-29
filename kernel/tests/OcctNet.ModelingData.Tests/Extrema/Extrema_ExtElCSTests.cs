// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Extrema_ExtElCS: known cases, and properties on random inputs (each extremum's points lie on the curve and the
/// surface at their parameters, its square distance is theirs, and the minimum is not above the distance between
/// sampled points). There is no native oracle: build123d.NET doesn't bind Extrema_ExtElCS or Extrema_ExtCS.
/// </summary>
public class Extrema_ExtElCSTests
{
    private const int Cases = 300;

    private static gp_Dir RandomDir(Random theRandom)
    {
        while (true)
        {
            var aV = new gp_Vec(theRandom.NextDouble() - 0.5, theRandom.NextDouble() - 0.5, theRandom.NextDouble() - 0.5);
            if (aV.Magnitude() > 0.1)
            {
                return new gp_Dir(aV);
            }
        }
    }

    private static double Real(Random theRandom, double theMin, double theMax)
        => theMin + (theMax - theMin) * theRandom.NextDouble();

    private static gp_Pnt RandomPnt(Random theRandom, double theScale = 10)
        => new(Real(theRandom, -theScale, theScale), Real(theRandom, -theScale, theScale), Real(theRandom, -theScale, theScale));

    private static gp_Ax3 RandomAx3(Random theRandom)
    {
        gp_Dir aN = RandomDir(theRandom);
        gp_Dir aX = RandomDir(theRandom);
        while (aN.IsParallel(aX, 0.1))
        {
            aX = RandomDir(theRandom);
        }
        return new gp_Ax3(RandomPnt(theRandom), aN, aX);
    }

    /// <summary>Checks every extremum: points at their parameters, and the square distance between them.</summary>
    private static void CheckPoints(Extrema_ExtElCS theExt, Func<double, gp_Pnt> theCurve, Func<double, double, gp_Pnt> theSurf,
                                    string theWhat)
    {
        Assert.True(theExt.IsDone(), theWhat);
        if (theExt.IsParallel())
        {
            return;
        }
        for (int i = 1; i <= theExt.NbExt(); i++)
        {
            theExt.Points(i, out Extrema_POnCurv aP1, out Extrema_POnSurf aP2);
            double aScale = Math.Max(1.0, aP1.Value().XYZ().Modulus());
            Assert.True(aP1.Value().Distance(theCurve(aP1.Parameter())) < 1e-9 * aScale, $"{theWhat} extremum {i} on the curve");
            aP2.Parameter(out double aU, out double aV);
            Assert.True(aP2.Value().Distance(theSurf(aU, aV)) < 1e-9 * aScale, $"{theWhat} extremum {i} on the surface");
            Assert.Equal(aP1.Value().SquareDistance(aP2.Value()), theExt.SquareDistance(i), 1e-9 * aScale * aScale);
        }
    }

    private static double MinSqDist(Extrema_ExtElCS theExt)
    {
        double aMin = double.MaxValue;
        for (int i = 1; i <= theExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, theExt.SquareDistance(i));
        }
        return aMin;
    }

    [Fact]
    public void LinePlane()
    {
        var aPln = new gp_Pln(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1));
        var aPar = new Extrema_ExtElCS(new gp_Lin(new gp_Pnt(1, 1, 5), new gp_Dir(1, 1, 0)), aPln);
        Assert.True(aPar.IsDone());
        Assert.True(aPar.IsParallel());
        Assert.Equal(1, aPar.NbExt());
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);

        // A crossing line has no extremum (the distance is 0 at the intersection, which OCCT doesn't report)
        var aCross = new Extrema_ExtElCS(new gp_Lin(new gp_Pnt(1, 1, 5), new gp_Dir(1, 1, 1)), aPln);
        Assert.True(aCross.IsDone());
        Assert.False(aCross.IsParallel());
        Assert.Equal(0, aCross.NbExt());
        Assert.Throws<Standard_OutOfRange>(() => aCross.SquareDistance(1));
    }

    [Fact]
    public void LineCylinder()
    {
        var aCyl = new gp_Cylinder(new gp_Ax3(), 2.0);
        // Parallel to the axis, at distance 5
        var aPar = new Extrema_ExtElCS(new gp_Lin(new gp_Pnt(5, 0, 0), new gp_Dir(0, 0, 1)), aCyl);
        Assert.True(aPar.IsParallel());
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);
        // Outside: the line x = 5 along Y; two extrema from the nearest line point, at distances 3 and 7
        var anOut = new Extrema_ExtElCS(new gp_Lin(new gp_Pnt(5, 0, 1), new gp_Dir(0, 1, 0)), aCyl);
        Assert.False(anOut.IsParallel());
        Assert.Equal(2, anOut.NbExt());
        Assert.Equal(9.0, MinSqDist(anOut), 1e-12);
        CheckPoints(anOut, t => ElCLib.Value(t, new gp_Lin(new gp_Pnt(5, 0, 1), new gp_Dir(0, 1, 0))),
                    (u, v) => ElSLib.Value(u, v, aCyl), "outside");
        // Through: the X axis meets it at x = -2 and 2
        var aThrough = new Extrema_ExtElCS(new gp_Lin(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0)), aCyl);
        Assert.Equal(2, aThrough.NbExt());
        Assert.Equal(0.0, aThrough.SquareDistance(1));
        Assert.Equal(0.0, aThrough.SquareDistance(2));
    }

    [Fact]
    public void LineSphere()
    {
        var aSph = new gp_Sphere(new gp_Ax3(new gp_Pnt(1, 1, 1), new gp_Dir(0, 0, 1)), 2.0);
        // Through the centre: the 2 intersections only (the centre, the foot of the perpendicular, has no extremum
        // on the sphere)
        var aCentre = new Extrema_ExtElCS(new gp_Lin(new gp_Pnt(1, 1, 1), new gp_Dir(1, 0, 0)), aSph);
        Assert.Equal(2, aCentre.NbExt());
        // At distance 1 from the centre: 2 intersections, then the 2 extrema of the foot of the perpendicular
        var aLin = new gp_Lin(new gp_Pnt(1, 1, 2), new gp_Dir(1, 0, 0));
        var aThrough = new Extrema_ExtElCS(aLin, aSph);
        Assert.Equal(4, aThrough.NbExt());
        Assert.Equal(0.0, aThrough.SquareDistance(1), 1e-24);
        Assert.Equal(0.0, aThrough.SquareDistance(2), 1e-24);
        Assert.Equal(1.0, aThrough.SquareDistance(3), 1e-12);
        Assert.Equal(9.0, aThrough.SquareDistance(4), 1e-12);
        CheckPoints(aThrough, t => ElCLib.Value(t, aLin), (u, v) => ElSLib.Value(u, v, aSph), "through");
        // Missing it by 3: distances 3 and 7
        var aMiss = new gp_Lin(new gp_Pnt(1, 1, 6), new gp_Dir(1, 0, 0));
        var anOut = new Extrema_ExtElCS(aMiss, aSph);
        Assert.Equal(2, anOut.NbExt());
        Assert.Equal(9.0, MinSqDist(anOut), 1e-12);
        CheckPoints(anOut, t => ElCLib.Value(t, aMiss), (u, v) => ElSLib.Value(u, v, aSph), "outside");
    }

    [Fact]
    public void NotImplemented()
    {
        Assert.Throws<Standard_NotImplemented>(() => new Extrema_ExtElCS(new gp_Lin(), new gp_Cone()));
        Assert.Throws<Standard_NotImplemented>(() => new Extrema_ExtElCS(new gp_Lin(), new gp_Torus()));
        Assert.Throws<Standard_NotImplemented>(() => new Extrema_ExtElCS(new gp_Circ(), new gp_Cone()));
        Assert.Throws<Standard_NotImplemented>(() => new Extrema_ExtElCS(new gp_Circ(), new gp_Torus()));
        var anEmpty = new Extrema_ExtElCS();
        Assert.False(anEmpty.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anEmpty.NbExt());
        Assert.Throws<StdFail_NotDone>(() => anEmpty.IsParallel());
    }

    [Fact]
    public void CirclePlane()
    {
        var aPln = new gp_Pln();
        // Parallel at height 3
        var aPar = new Extrema_ExtElCS(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), 1), aPln);
        Assert.True(aPar.IsParallel());
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);
        // Tilted 45° at height 3, radius 1: extrema at heights 3 ± sqrt(0.5)
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 1, 1)), 1);
        var aTilt = new Extrema_ExtElCS(aCirc, aPln);
        Assert.Equal(2, aTilt.NbExt());
        double aLow = 3 - Math.Sqrt(0.5), aHigh = 3 + Math.Sqrt(0.5);
        Assert.Equal(aLow * aLow, MinSqDist(aTilt), 1e-12);
        Assert.Equal(aLow * aLow + aHigh * aHigh, aTilt.SquareDistance(1) + aTilt.SquareDistance(2), 1e-11);
        CheckPoints(aTilt, t => ElCLib.Value(t, aCirc), (u, v) => ElSLib.Value(u, v, aPln), "tilted");
        // Crossing the plane: 2 extrema and the 2 intersections
        var aCross = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0.5), new gp_Dir(0, 1, 1)), 1);
        var anInt = new Extrema_ExtElCS(aCross, aPln);
        Assert.Equal(4, anInt.NbExt());
        Assert.Equal(0.0, anInt.SquareDistance(3), 1e-16);
        Assert.Equal(0.0, anInt.SquareDistance(4), 1e-16);
        CheckPoints(anInt, t => ElCLib.Value(t, aCross), (u, v) => ElSLib.Value(u, v, aPln), "crossing");
    }

    [Fact]
    public void CircleCylinder()
    {
        var aCyl = new gp_Cylinder(new gp_Ax3(), 2.0);
        // Coaxial: parallel, at distance |5 - 2|
        var aPar = new Extrema_ExtElCS(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), 5), aCyl);
        Assert.True(aPar.IsParallel());
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);
        // Random circles: the points are on both
        var aRandom = new Random(14031);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            var aC = new gp_Cylinder(anAx, Real(aRandom, 0.5, 5));
            var aCirc = new gp_Circ(new gp_Ax2(anAx.Location().Translated(new gp_Vec(RandomDir(aRandom)) * Real(aRandom, 0, 8)),
                                               RandomDir(aRandom)), Real(aRandom, 0.5, 6));
            var anExt = new Extrema_ExtElCS(aCirc, aC);
            CheckPoints(anExt, t => ElCLib.Value(t, aCirc), (u, v) => ElSLib.Value(u, v, aC), $"case {i}");
        }
    }

    [Fact]
    public void CircleSphere()
    {
        var aSph = new gp_Sphere(new gp_Ax3(), 2.0);
        // On the sphere's axis: parallel
        var aPar = new Extrema_ExtElCS(new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), 4), aSph);
        Assert.True(aPar.IsParallel());
        Assert.Equal(Math.Pow(5 - 2, 2), aPar.SquareDistance(), 1e-12);
        // Away from it, outside: one minimum
        var aFar = new gp_Circ(new gp_Ax2(new gp_Pnt(10, 0, 0), new gp_Dir(0, 0, 1)), 1);
        var anOut = new Extrema_ExtElCS(aFar, aSph);
        Assert.Equal(1, anOut.NbExt());
        Assert.Equal(49.0, anOut.SquareDistance(), 1e-12);
        CheckPoints(anOut, t => ElCLib.Value(t, aFar), (u, v) => ElSLib.Value(u, v, aSph), "far");

        var aRandom = new Random(14032);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            var aS = new gp_Sphere(anAx, Real(aRandom, 0.5, 5));
            var aCirc = new gp_Circ(new gp_Ax2(anAx.Location().Translated(new gp_Vec(RandomDir(aRandom)) * Real(aRandom, 0.5, 8)),
                                               RandomDir(aRandom)), Real(aRandom, 0.5, 6));
            var anExt = new Extrema_ExtElCS(aCirc, aS);
            if (!anExt.IsDone())
            {
                continue;
            }
            CheckPoints(anExt, t => ElCLib.Value(t, aCirc), (u, v) => ElSLib.Value(u, v, aS), $"case {i}");
            if (anExt.IsParallel())
            {
                continue;
            }
            // The minimum is not above the distance from sampled circle points to the sphere
            double aMin = MinSqDist(anExt), aSampled = double.MaxValue;
            for (int j = 0; j < 360; j++)
            {
                double aD = ElCLib.Value(j * M_PI / 180, aCirc).Distance(anAx.Location()) - aS.Radius();
                aSampled = Math.Min(aSampled, aD * aD);
            }
            Assert.True(aMin <= aSampled + 1e-9, $"case {i}: {aMin} > {aSampled}");
        }
    }

    [Fact]
    public void HyperbolaPlane()
    {
        var aPln = new gp_Pln(new gp_Pnt(0, 0, -1), new gp_Dir(0, 0, 1));
        // The hyperbola's plane tilted about its X axis: its points are at heights sinh(t) * r * sin(a); only a
        // plane crossing the major axis direction has an extremum
        var aHypr = new gp_Hypr(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, -1, 1), new gp_Dir(1, 0, 1)), 3, 1);
        var anExt = new Extrema_ExtElCS(aHypr, aPln);
        Assert.True(anExt.IsDone());
        Assert.False(anExt.IsParallel());
        Assert.Equal(1, anExt.NbExt());
        CheckPoints(anExt, t => ElCLib.Value(t, aHypr), (u, v) => ElSLib.Value(u, v, aPln), "hyperbola");
        // The extremum is a local minimum of the height above the plane
        anExt.Points(1, out Extrema_POnCurv aP, out _);
        double aH0 = aPln.Distance(aP.Value());
        Assert.True(aPln.Distance(ElCLib.Value(aP.Parameter() + 1e-3, aHypr)) >= aH0);
        Assert.True(aPln.Distance(ElCLib.Value(aP.Parameter() - 1e-3, aHypr)) >= aH0);

        var aPar = new Extrema_ExtElCS(new gp_Hypr(new gp_Ax2(new gp_Pnt(0, 0, 4), new gp_Dir(0, 0, 1)), 3, 1), aPln);
        Assert.True(aPar.IsParallel());
        Assert.Equal(25.0, aPar.SquareDistance(), 1e-12);
    }

    [Fact]
    public void LinesAndCircles_Random()
    {
        var aRandom = new Random(14033);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 5);
            var aLin = new gp_Lin(anAx.Location().Translated(new gp_Vec(RandomDir(aRandom)) * Real(aRandom, 0, 8)), RandomDir(aRandom));
            var aCirc = new gp_Circ(new gp_Ax2(aLin.Location(), RandomDir(aRandom)), Real(aRandom, 0.5, 6));
            var aPln = new gp_Pln(anAx);
            var aCyl = new gp_Cylinder(anAx, aR);
            var aSph = new gp_Sphere(anAx, aR);
            CheckPoints(new Extrema_ExtElCS(aLin, aCyl), t => ElCLib.Value(t, aLin), (u, v) => ElSLib.Value(u, v, aCyl), $"case {i} line/cylinder");
            CheckPoints(new Extrema_ExtElCS(aLin, aSph), t => ElCLib.Value(t, aLin), (u, v) => ElSLib.Value(u, v, aSph), $"case {i} line/sphere");
            CheckPoints(new Extrema_ExtElCS(aCirc, aPln), t => ElCLib.Value(t, aCirc), (u, v) => ElSLib.Value(u, v, aPln), $"case {i} circle/plane");
        }
    }
}
