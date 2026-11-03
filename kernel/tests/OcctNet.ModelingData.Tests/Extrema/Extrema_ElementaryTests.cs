// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// The elementary Extrema classes (OCCT has no GTests for them): the extrema they find are extrema, on random inputs,
/// and the OCCT behaviours the port reproduces (docs/occt-issues.md) stay reproduced. Native agreement is checked by
/// the oracle tests.
/// </summary>
public class Extrema_ElementaryTest
{
    private static Direction3D RandomDir(Random theRandom)
    {
        while (true)
        {
            var aV = new Vector3D(theRandom.NextDouble() - 0.5, theRandom.NextDouble() - 0.5, theRandom.NextDouble() - 0.5);
            if (aV.Magnitude() > 0.1)
            {
                return new Direction3D(aV);
            }
        }
    }

    private static Point3D RandomPnt(Random theRandom, double theScale = 50)
        => new((theRandom.NextDouble() - 0.5) * 2 * theScale, (theRandom.NextDouble() - 0.5) * 2 * theScale,
               (theRandom.NextDouble() - 0.5) * 2 * theScale);

    private static Frame3D RandomAx2(Random theRandom) => new(RandomPnt(theRandom), RandomDir(theRandom), RandomDir(theRandom));

    /// <summary>P - C(u) is normal to the curve at an extremum.</summary>
    private static void AssertNormal(in Point3D theP, in Point3D theC, in Vector3D theTangent)
    {
        var aV = new Vector3D(theC, theP);
        Assert.Equal(0.0, aV.Dot(theTangent) / Math.Max(1.0, aV.Magnitude() * theTangent.Magnitude()), 1e-7);
    }

    [Fact]
    public void POnCurv_and_POnSurf_values()
    {
        var aP = new Extrema_POnCurv(2.5, new Point3D(1, 2, 3));
        Assert.Equal(2.5, aP.Parameter());
        Assert.Equal(new Point3D(1, 2, 3), aP.Value());
        Assert.Equal(new Extrema_POnCurv(), default);

        var aS = new Extrema_POnSurf();
        aS.Parameter(out double aU, out double aV);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aU);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aV);
        aS.SetParameters(1, 2, new Point3D(3, 4, 5));
        aS.Parameter(out aU, out aV);
        Assert.Equal(new Extrema_POnSurf(1, 2, new Point3D(3, 4, 5)), aS);
    }

    [Fact]
    public void ExtPElC_extrema_are_normal_projections()
    {
        var aRandom = new Random(10);
        double aTwoPI = 2 * Math.PI;
        for (int i = 0; i < 200; i++)
        {
            Frame3D anAx = RandomAx2(aRandom);
            Point3D aP = RandomPnt(aRandom);
            double aR = 1 + 30 * aRandom.NextDouble();

            var aLin = new Line3D(anAx.Axis());
            Check(new Extrema_ExtPElC(aP, aLin, 1e-7, -1000, 1000), theU => { global::Kerf.Engine.Geometry.ElCLib.D1(theU, aLin, out Point3D aC, out Vector3D aT); return (aC, aT); }, 1);

            var aCirc = new Circle3D(anAx, aR);
            var anExt = new Extrema_ExtPElC(aP, aCirc, 1e-7, 0, aTwoPI);
            Check(anExt, theU => { global::Kerf.Engine.Geometry.ElCLib.D1(theU, aCirc, out Point3D aC, out Vector3D aT); return (aC, aT); }, 2);
            Assert.True(anExt.IsMin(1));
            Assert.False(anExt.IsMin(2));
            Assert.True(anExt.SquareDistance(1) <= anExt.SquareDistance(2));

            var anElips = new Ellipse3D(anAx, aR + 5, aR);
            Check(new Extrema_ExtPElC(aP, anElips, 1e-7, 0, aTwoPI), theU => { global::Kerf.Engine.Geometry.ElCLib.D1(theU, anElips, out Point3D aC, out Vector3D aT); return (aC, aT); }, -1);

            var aHypr = new Hyperbola3D(anAx, aR + 1, aR);
            Check(new Extrema_ExtPElC(aP, aHypr, 1e-7, -10, 10), theU => { global::Kerf.Engine.Geometry.ElCLib.D1(theU, aHypr, out Point3D aC, out Vector3D aT); return (aC, aT); }, -1);

            var aParab = new Parabola3D(anAx, aR);
            Check(new Extrema_ExtPElC(aP, aParab, 1e-7, -1000, 1000), theU => { global::Kerf.Engine.Geometry.ElCLib.D1(theU, aParab, out Point3D aC, out Vector3D aT); return (aC, aT); }, -1);

            void Check(Extrema_ExtPElC theExt, Func<double, (Point3D, Vector3D)> theD1, int theNb)
            {
                Assert.True(theExt.IsDone());
                if (theNb >= 0)
                {
                    Assert.Equal(theNb, theExt.NbExt());
                }
                for (int j = 1; j <= theExt.NbExt(); j++)
                {
                    Extrema_POnCurv aPOn = theExt.Point(j);
                    (Point3D aC, Vector3D aT) = theD1(aPOn.Parameter());
                    Assert.Equal(0.0, aC.Distance(aPOn.Value()), 1e-7);
                    Assert.Equal(aP.SquareDistance(aPOn.Value()), theExt.SquareDistance(j), 1e-6);
                    AssertNormal(aP, aC, aT);
                }
            }
        }
    }

    [Fact]
    public void ExtPElC_point_on_axis_of_circle_is_not_done()
    {
        var aCirc = new Circle3D(new Frame3D(), 3);
        var anExt = new Extrema_ExtPElC(new Point3D(0, 0, 5), aCirc, 1e-7, 0, 2 * Math.PI);
        Assert.False(anExt.IsDone());
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anExt.NbExt());
    }

    [Fact]
    public void ExtPElC2d_extrema_are_normal_projections()
    {
        var aRandom = new Random(11);
        for (int i = 0; i < 200; i++)
        {
            double anA = aRandom.NextDouble() * 2 * Math.PI;
            var anAx = new Frame2D(new Point2D(aRandom.NextDouble() * 20 - 10, aRandom.NextDouble() * 20 - 10),
                                    new Direction2D(Math.Cos(anA), Math.Sin(anA)), aRandom.Next(2) == 0);
            var aP = new Point2D(aRandom.NextDouble() * 60 - 30, aRandom.NextDouble() * 60 - 30);
            double aR = 1 + 20 * aRandom.NextDouble();

            var aCirc = new Circle2D(anAx, aR);
            var anExt = new Extrema_ExtPElC2d(aP, aCirc, 1e-7, 0, 2 * Math.PI);
            Assert.True(anExt.IsDone());
            Assert.Equal(2, anExt.NbExt());
            // OCCT marks both extrema of a circle as minima (occt-issues.md)
            Assert.True(anExt.IsMin(1) && anExt.IsMin(2));
            for (int j = 1; j <= 2; j++)
            {
                Assert.Equal(0.0, aCirc.Distance(anExt.Point(j).Value()), 1e-9);
                Assert.Equal(aP.SquareDistance(anExt.Point(j).Value()), anExt.SquareDistance(j), 1e-6);
            }

            var anElips = new Ellipse2D(anAx, aR + 4, aR);
            var anExtE = new Extrema_ExtPElC2d(aP, anElips, 1e-7, 0, 2 * Math.PI);
            Assert.True(anExtE.IsDone());
            for (int j = 1; j <= anExtE.NbExt(); j++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(anExtE.Point(j).Parameter(), anElips, out Point2D aC, out Vector2D aT);
                Assert.Equal(0.0, new Vector2D(aC, aP).Dot(aT) / Math.Max(1.0, aC.Distance(aP) * aT.Magnitude()), 1e-7);
                // OCCT never marks an extremum of an ellipse as a minimum (occt-issues.md)
                Assert.False(anExtE.IsMin(j));
            }
        }
    }

    [Fact]
    public void ExtPElS_extrema_are_normal_projections()
    {
        var aRandom = new Random(12);
        for (int i = 0; i < 200; i++)
        {
            var anAx = new CoordinateSystem(RandomAx2(aRandom));
            Point3D aP = RandomPnt(aRandom);
            double aR = 1 + 20 * aRandom.NextDouble();

            double aSemiAngle = 0.1 + 1.2 * aRandom.NextDouble();
            Check(aP, new Extrema_ExtPElS(aP, new Plane3D(anAx), 1e-7), new Plane3D(anAx), 1);
            Check(aP, new Extrema_ExtPElS(aP, new Cylinder(anAx, aR), 1e-7), new Cylinder(anAx, aR), 2);
            Check(aP, new Extrema_ExtPElS(aP, new Cone(anAx, aSemiAngle, aR), 1e-7), new Cone(anAx, aSemiAngle, aR), 2);
            Check(aP, new Extrema_ExtPElS(aP, new Sphere(anAx, aR), 1e-7), new Sphere(anAx, aR), 2);
            Check(aP, new Extrema_ExtPElS(aP, new Torus(anAx, aR + 5, aR), 1e-7), new Torus(anAx, aR + 5, aR), 4);
        }

        static void Check<TSurface>(Point3D theP, Extrema_ExtPElS theExt, TSurface theSurface, int theNb)
        {
            Assert.True(theExt.IsDone());
            Assert.Equal(theNb, theExt.NbExt());
            for (int j = 1; j <= theExt.NbExt(); j++)
            {
                Extrema_POnSurf aPOn = theExt.Point(j);
                aPOn.Parameter(out double aU, out double aV);
                Point3D aS = theSurface switch
                {
                    Plane3D aPln => global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aPln),
                    Cylinder aCyl => global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCyl),
                    Cone aCone => global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCone),
                    Sphere aSph => global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aSph),
                    Torus aTor => global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aTor),
                    _ => throw new InvalidOperationException(),
                };
                Assert.Equal(0.0, aS.Distance(aPOn.Value()), 1e-7);
                Assert.Equal(theP.SquareDistance(aPOn.Value()), theExt.SquareDistance(j), 1e-6);
            }
        }
    }

    [Fact]
    public void ExtElC_between_lines_and_circles()
    {
        var aRandom = new Random(13);
        for (int i = 0; i < 200; i++)
        {
            var aL1 = new Line3D(RandomPnt(aRandom), RandomDir(aRandom));
            var aL2 = new Line3D(RandomPnt(aRandom), RandomDir(aRandom));
            var anExt = new Extrema_ExtElC(aL1, aL2, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.True(anExt.IsDone());
            Assert.False(anExt.IsParallel());
            Assert.Equal(1, anExt.NbExt());
            anExt.Points(1, out Extrema_POnCurv aP1, out Extrema_POnCurv aP2);
            Assert.Equal(aL1.Distance(aL2), Math.Sqrt(anExt.SquareDistance(1)), 1e-8);
            Assert.Equal(0.0, aL1.Distance(aP1.Value()), 1e-8);
            Assert.Equal(0.0, aL2.Distance(aP2.Value()), 1e-8);

            // Line and circle: each pair is a normal pair
            var aCirc = new Circle3D(RandomAx2(aRandom), 1 + 20 * aRandom.NextDouble());
            var anExtLC = new Extrema_ExtElC(aL1, aCirc, global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.True(anExtLC.IsDone());
            for (int j = 1; j <= anExtLC.NbExt(); j++)
            {
                anExtLC.Points(j, out Extrema_POnCurv aPL, out Extrema_POnCurv aPC);
                global::Kerf.Engine.Geometry.ElCLib.D1(aPC.Parameter(), aCirc, out Point3D aC, out Vector3D aT);
                Assert.Equal(0.0, aC.Distance(aPC.Value()), 1e-7);
                AssertNormal(aPL.Value(), aC, aT);
                AssertNormal(aPC.Value(), aPL.Value(), new Vector3D(aL1.Direction()));
            }
        }

        // Coplanar circles crossing each other: 4 common extrema and 2 intersections
        var aC1 = new Circle3D(new Frame3D(), 5);
        var aC2 = new Circle3D(new Frame3D(new Point3D(6, 0, 0), new Direction3D(0, 0, 1)), 3);
        var anExtCC = new Extrema_ExtElC(aC1, aC2);
        Assert.True(anExtCC.IsDone());
        Assert.Equal(6, anExtCC.NbExt());
        Assert.Equal(0.0, anExtCC.SquareDistance(5));
        Assert.Equal(0.0, anExtCC.SquareDistance(6));

        // Non-coplanar circles: not done
        Assert.False(new Extrema_ExtElC(aC1, new Circle3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)), 2)).IsDone());
    }

    [Fact]
    public void ExtElC2d_between_lines_and_conics()
    {
        var aL1 = new Line2D(new Point2D(0, 0), new Direction2D(1, 0));
        var aL2 = new Line2D(new Point2D(2, 3), new Direction2D(1, 1));
        var anExt = new Extrema_ExtElC2d(aL1, aL2, global::Kerf.Engine.Foundation.Precision.Angular());
        Assert.False(anExt.IsParallel());
        anExt.Points(1, out Extrema_POnCurv2d aP1, out Extrema_POnCurv2d aP2);
        Assert.Equal(0.0, aP1.Value().Distance(new Point2D(-1, 0)), 1e-12);
        Assert.Equal(0.0, aP2.Value().Distance(new Point2D(-1, 0)), 1e-12);

        var aParallel = new Extrema_ExtElC2d(aL1, new Line2D(new Point2D(0, 2), new Direction2D(-1, 0)), global::Kerf.Engine.Foundation.Precision.Angular());
        Assert.True(aParallel.IsParallel());
        Assert.Equal(4.0, aParallel.SquareDistance(), 1e-12);

        var aCirc = new Circle2D(new Axis2D(new Point2D(0, 5), new Direction2D(1, 0)), 2);
        var anExtLC = new Extrema_ExtElC2d(aL1, aCirc, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2, anExtLC.NbExt());
        Assert.Equal(9.0, Math.Min(anExtLC.SquareDistance(1), anExtLC.SquareDistance(2)), 1e-12);
        Assert.Equal(49.0, Math.Max(anExtLC.SquareDistance(1), anExtLC.SquareDistance(2)), 1e-12);

        var aCirc2 = new Circle2D(new Axis2D(new Point2D(10, 5), new Direction2D(1, 0)), 3);
        var anExtCC = new Extrema_ExtElC2d(aCirc, aCirc2);
        Assert.Equal(4, anExtCC.NbExt());
        // nearest points (2, 5) and (7, 5), the second pair
        Assert.Equal(25.0, anExtCC.SquareDistance(2), 1e-12);

        var anElips = new Ellipse2D(new Axis2D(new Point2D(20, 5), new Direction2D(1, 0)), 4, 2);
        var anExtCE = new Extrema_ExtElC2d(aCirc, anElips);
        Assert.True(anExtCE.IsDone());
        Assert.True(anExtCE.NbExt() >= 4);
    }

    [Fact]
    public void ExtElSS_planes()
    {
        var aP1 = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aP2 = new Plane3D(new Point3D(1, 2, 3), new Direction3D(0, 0, -1));
        var anExt = new Extrema_ExtElSS(aP1, aP2);
        Assert.True(anExt.IsDone());
        Assert.True(anExt.IsParallel());
        Assert.Equal(9.0, anExt.SquareDistance(), 1e-12);
        // OCCT never fills the points (occt-issues.md)
        Assert.Throws<NullReferenceException>(() => anExt.Points(1, out _, out _));

        var aCrossing = new Extrema_ExtElSS(aP1, new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        Assert.True(aCrossing.IsDone());
        Assert.Equal(0, aCrossing.NbExt());

        Assert.Throws<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => new Extrema_ExtElSS(new Sphere(), new Sphere()));
    }
}
