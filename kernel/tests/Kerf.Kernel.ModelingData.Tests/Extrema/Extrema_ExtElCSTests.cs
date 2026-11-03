// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Extrema_ExtElCS: known cases, and properties on random inputs (each extremum's points lie on the curve and the
/// surface at their parameters, its square distance is theirs, and the minimum is not above the distance between
/// sampled points). There is no native oracle: build123d.NET doesn't bind Extrema_ExtElCS or Extrema_ExtCS.
/// </summary>
public class Extrema_ExtElCSTests
{
    private const int Cases = 300;

    private static Direction3D RandomDir(Random theRandom)
    {
        while (true)
        {
            var aV = new Vector3D(theRandom.NextDouble() - 0.5, theRandom.NextDouble() - 0.5, theRandom.NextDouble() - 0.5);
            if (aV.Magnitude > 0.1)
            {
                return new Direction3D(aV);
            }
        }
    }

    private static double Real(Random theRandom, double theMin, double theMax)
        => theMin + (theMax - theMin) * theRandom.NextDouble();

    private static Point3D RandomPnt(Random theRandom, double theScale = 10)
        => new(Real(theRandom, -theScale, theScale), Real(theRandom, -theScale, theScale), Real(theRandom, -theScale, theScale));

    private static CoordinateSystem RandomAx3(Random theRandom)
    {
        Direction3D aN = RandomDir(theRandom);
        Direction3D aX = RandomDir(theRandom);
        while (aN.IsParallel(aX, 0.1))
        {
            aX = RandomDir(theRandom);
        }
        return new CoordinateSystem(RandomPnt(theRandom), aN, aX);
    }

    /// <summary>Checks every extremum: points at their parameters, and the square distance between them.</summary>
    private static void CheckPoints(ExtElCS theExt, Func<double, Point3D> theCurve, Func<double, double, Point3D> theSurf,
                                    string theWhat)
    {
        Assert.True(theExt.IsDone, theWhat);
        if (theExt.IsParallel)
        {
            return;
        }
        for (int i = 1; i <= theExt.ExtCount; i++)
        {
            theExt.Points(i, out POnCurv aP1, out POnSurf aP2);
            double aScale = Math.Max(1.0, aP1.Value().XYZ().Modulus());
            Assert.True(aP1.Value().DistanceTo(theCurve(aP1.Parameter())) < 1e-9 * aScale, $"{theWhat} extremum {i} on the curve");
            aP2.Parameter(out double aU, out double aV);
            Assert.True(aP2.Value().DistanceTo(theSurf(aU, aV)) < 1e-9 * aScale, $"{theWhat} extremum {i} on the surface");
            Assert.Equal(aP1.Value().SquareDistanceTo(aP2.Value()), theExt.SquareDistance(i), 1e-9 * aScale * aScale);
        }
    }

    private static double MinSqDist(ExtElCS theExt)
    {
        double aMin = double.MaxValue;
        for (int i = 1; i <= theExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, theExt.SquareDistance(i));
        }
        return aMin;
    }

    [Fact]
    public void LinePlane()
    {
        var aPln = new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1));
        var aPar = new ExtElCS(new Line3D(new Point3D(1, 1, 5), new Direction3D(1, 1, 0)), aPln);
        Assert.True(aPar.IsDone);
        Assert.True(aPar.IsParallel);
        Assert.Equal(1, aPar.ExtCount);
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);

        // A crossing line has no extremum (the distance is 0 at the intersection, which OCCT doesn't report)
        var aCross = new ExtElCS(new Line3D(new Point3D(1, 1, 5), new Direction3D(1, 1, 1)), aPln);
        Assert.True(aCross.IsDone);
        Assert.False(aCross.IsParallel);
        Assert.Equal(0, aCross.ExtCount);
        Assert.Throws<OutOfRangeException>(() => aCross.SquareDistance(1));
    }

    [Fact]
    public void LineCylinder()
    {
        var aCyl = new Cylinder(new CoordinateSystem(), 2.0);
        // Parallel to the axis, at distance 5
        var aPar = new ExtElCS(new Line3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)), aCyl);
        Assert.True(aPar.IsParallel);
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);
        // Outside: the line x = 5 along Y; two extrema from the nearest line point, at distances 3 and 7
        var anOut = new ExtElCS(new Line3D(new Point3D(5, 0, 1), new Direction3D(0, 1, 0)), aCyl);
        Assert.False(anOut.IsParallel);
        Assert.Equal(2, anOut.ExtCount);
        Assert.Equal(9.0, MinSqDist(anOut), 1e-12);
        CheckPoints(anOut, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, new Line3D(new Point3D(5, 0, 1), new Direction3D(0, 1, 0))),
                    (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aCyl), "outside");
        // Through: the X axis meets it at x = -2 and 2
        var aThrough = new ExtElCS(new Line3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)), aCyl);
        Assert.Equal(2, aThrough.ExtCount);
        Assert.Equal(0.0, aThrough.SquareDistance(1));
        Assert.Equal(0.0, aThrough.SquareDistance(2));
    }

    [Fact]
    public void LineSphere()
    {
        var aSph = new Sphere(new CoordinateSystem(new Point3D(1, 1, 1), new Direction3D(0, 0, 1)), 2.0);
        // Through the centre: the 2 intersections only (the centre, the foot of the perpendicular, has no extremum
        // on the sphere)
        var aCentre = new ExtElCS(new Line3D(new Point3D(1, 1, 1), new Direction3D(1, 0, 0)), aSph);
        Assert.Equal(2, aCentre.ExtCount);
        // At distance 1 from the centre: 2 intersections, then the 2 extrema of the foot of the perpendicular
        var aLin = new Line3D(new Point3D(1, 1, 2), new Direction3D(1, 0, 0));
        var aThrough = new ExtElCS(aLin, aSph);
        Assert.Equal(4, aThrough.ExtCount);
        Assert.Equal(0.0, aThrough.SquareDistance(1), 1e-24);
        Assert.Equal(0.0, aThrough.SquareDistance(2), 1e-24);
        Assert.Equal(1.0, aThrough.SquareDistance(3), 1e-12);
        Assert.Equal(9.0, aThrough.SquareDistance(4), 1e-12);
        CheckPoints(aThrough, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aLin), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aSph), "through");
        // Missing it by 3: distances 3 and 7
        var aMiss = new Line3D(new Point3D(1, 1, 6), new Direction3D(1, 0, 0));
        var anOut = new ExtElCS(aMiss, aSph);
        Assert.Equal(2, anOut.ExtCount);
        Assert.Equal(9.0, MinSqDist(anOut), 1e-12);
        CheckPoints(anOut, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aMiss), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aSph), "outside");
    }

    [Fact]
    public void NotImplemented()
    {
        Assert.Throws<MethodNotImplementedException>(() => new ExtElCS(new Line3D(), new Cone()));
        Assert.Throws<MethodNotImplementedException>(() => new ExtElCS(new Line3D(), new Torus()));
        Assert.Throws<MethodNotImplementedException>(() => new ExtElCS(new Circle3D(), new Cone()));
        Assert.Throws<MethodNotImplementedException>(() => new ExtElCS(new Circle3D(), new Torus()));
        var anEmpty = new ExtElCS();
        Assert.False(anEmpty.IsDone);
        Assert.Throws<NotDoneException>(() => anEmpty.ExtCount);
        Assert.Throws<NotDoneException>(() => anEmpty.IsParallel);
    }

    [Fact]
    public void CirclePlane()
    {
        var aPln = new Plane3D();
        // Parallel at height 3
        var aPar = new ExtElCS(new Circle3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), 1), aPln);
        Assert.True(aPar.IsParallel);
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);
        // Tilted 45° at height 3, radius 1: extrema at heights 3 ± sqrt(0.5)
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 1, 1)), 1);
        var aTilt = new ExtElCS(aCirc, aPln);
        Assert.Equal(2, aTilt.ExtCount);
        double aLow = 3 - Math.Sqrt(0.5), aHigh = 3 + Math.Sqrt(0.5);
        Assert.Equal(aLow * aLow, MinSqDist(aTilt), 1e-12);
        Assert.Equal(aLow * aLow + aHigh * aHigh, aTilt.SquareDistance(1) + aTilt.SquareDistance(2), 1e-11);
        CheckPoints(aTilt, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCirc), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aPln), "tilted");
        // Crossing the plane: 2 extrema and the 2 intersections
        var aCross = new Circle3D(new Frame3D(new Point3D(0, 0, 0.5), new Direction3D(0, 1, 1)), 1);
        var anInt = new ExtElCS(aCross, aPln);
        Assert.Equal(4, anInt.ExtCount);
        Assert.Equal(0.0, anInt.SquareDistance(3), 1e-16);
        Assert.Equal(0.0, anInt.SquareDistance(4), 1e-16);
        CheckPoints(anInt, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCross), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aPln), "crossing");
    }

    [Fact]
    public void CircleCylinder()
    {
        var aCyl = new Cylinder(new CoordinateSystem(), 2.0);
        // Coaxial: parallel, at distance |5 - 2|
        var aPar = new ExtElCS(new Circle3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), 5), aCyl);
        Assert.True(aPar.IsParallel);
        Assert.Equal(9.0, aPar.SquareDistance(), 1e-12);
        // Random circles: the points are on both
        var aRandom = new Random(14031);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            var aC = new Cylinder(anAx, Real(aRandom, 0.5, 5));
            var aCirc = new Circle3D(new Frame3D(anAx.Location.Translated(new Vector3D(RandomDir(aRandom)) * Real(aRandom, 0, 8)),
                                               RandomDir(aRandom)), Real(aRandom, 0.5, 6));
            var anExt = new ExtElCS(aCirc, aC);
            CheckPoints(anExt, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCirc), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aC), $"case {i}");
        }
    }

    [Fact]
    public void CircleSphere()
    {
        var aSph = new Sphere(new CoordinateSystem(), 2.0);
        // On the sphere's axis: parallel
        var aPar = new ExtElCS(new Circle3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), 4), aSph);
        Assert.True(aPar.IsParallel);
        Assert.Equal(Math.Pow(5 - 2, 2), aPar.SquareDistance(), 1e-12);
        // Away from it, outside: one minimum
        var aFar = new Circle3D(new Frame3D(new Point3D(10, 0, 0), new Direction3D(0, 0, 1)), 1);
        var anOut = new ExtElCS(aFar, aSph);
        Assert.Equal(1, anOut.ExtCount);
        Assert.Equal(49.0, anOut.SquareDistance(), 1e-12);
        CheckPoints(anOut, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aFar), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aSph), "far");

        var aRandom = new Random(14032);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            var aS = new Sphere(anAx, Real(aRandom, 0.5, 5));
            var aCirc = new Circle3D(new Frame3D(anAx.Location.Translated(new Vector3D(RandomDir(aRandom)) * Real(aRandom, 0.5, 8)),
                                               RandomDir(aRandom)), Real(aRandom, 0.5, 6));
            var anExt = new ExtElCS(aCirc, aS);
            if (!anExt.IsDone)
            {
                continue;
            }
            CheckPoints(anExt, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCirc), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aS), $"case {i}");
            if (anExt.IsParallel)
            {
                continue;
            }
            // The minimum is not above the distance from sampled circle points to the sphere
            double aMin = MinSqDist(anExt), aSampled = double.MaxValue;
            for (int j = 0; j < 360; j++)
            {
                double aD = global::Kerf.Engine.Geometry.ElCLib.Value(j * M_PI / 180, aCirc).DistanceTo(anAx.Location) - aS.Radius;
                aSampled = Math.Min(aSampled, aD * aD);
            }
            Assert.True(aMin <= aSampled + 1e-9, $"case {i}: {aMin} > {aSampled}");
        }
    }

    [Fact]
    public void HyperbolaPlane()
    {
        var aPln = new Plane3D(new Point3D(0, 0, -1), new Direction3D(0, 0, 1));
        // The hyperbola's plane tilted about its X axis: its points are at heights sinh(t) * r * sin(a); only a
        // plane crossing the major axis direction has an extremum
        var aHypr = new Hyperbola3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 1), new Direction3D(1, 0, 1)), 3, 1);
        var anExt = new ExtElCS(aHypr, aPln);
        Assert.True(anExt.IsDone);
        Assert.False(anExt.IsParallel);
        Assert.Equal(1, anExt.ExtCount);
        CheckPoints(anExt, t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aHypr), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aPln), "hyperbola");
        // The extremum is a local minimum of the height above the plane
        anExt.Points(1, out POnCurv aP, out _);
        double aH0 = aPln.DistanceTo(aP.Value());
        Assert.True(aPln.DistanceTo(global::Kerf.Engine.Geometry.ElCLib.Value(aP.Parameter() + 1e-3, aHypr)) >= aH0);
        Assert.True(aPln.DistanceTo(global::Kerf.Engine.Geometry.ElCLib.Value(aP.Parameter() - 1e-3, aHypr)) >= aH0);

        var aPar = new ExtElCS(new Hyperbola3D(new Frame3D(new Point3D(0, 0, 4), new Direction3D(0, 0, 1)), 3, 1), aPln);
        Assert.True(aPar.IsParallel);
        Assert.Equal(25.0, aPar.SquareDistance(), 1e-12);
    }

    [Fact]
    public void LinesAndCircles_Random()
    {
        var aRandom = new Random(14033);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 5);
            var aLin = new Line3D(anAx.Location.Translated(new Vector3D(RandomDir(aRandom)) * Real(aRandom, 0, 8)), RandomDir(aRandom));
            var aCirc = new Circle3D(new Frame3D(aLin.Location, RandomDir(aRandom)), Real(aRandom, 0.5, 6));
            var aPln = new Plane3D(anAx);
            var aCyl = new Cylinder(anAx, aR);
            var aSph = new Sphere(anAx, aR);
            CheckPoints(new ExtElCS(aLin, aCyl), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aLin), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aCyl), $"case {i} line/cylinder");
            CheckPoints(new ExtElCS(aLin, aSph), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aLin), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aSph), $"case {i} line/sphere");
            CheckPoints(new ExtElCS(aCirc, aPln), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCirc), (u, v) => global::Kerf.Engine.Geometry.ElSLib.Value(u, v, aPln), $"case {i} circle/plane");
        }
    }
}
