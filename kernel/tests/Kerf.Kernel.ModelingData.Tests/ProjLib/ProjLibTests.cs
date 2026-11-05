// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Property tests of the elementary projections: a curve lying on the surface projects to a 2D line or conic
/// whose points, mapped back with ElSLib, are the curve's points at the same parameter (or, for lines in general
/// position on a plane, on the projected line).
/// </summary>
public class ProjLibTests
{
    private const int Cases = 200;
    private const double Tol = 1.0e-9;

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
        var anAx = new CoordinateSystem(RandomPnt(theRandom), aN, aX);
        return theRandom.Next(2) == 0 ? anAx : anAx.YReversed();
    }

    private static void AssertSame(in Point3D theExpected, in Point3D theActual, double theTol, string theWhat)
        => Assert.True(theExpected.DistanceTo(theActual) <= theTol,
                       $"{theWhat}: expected {theExpected}, got {theActual} (distance {theExpected.DistanceTo(theActual):E3})");

    // Plane

    [Fact]
    public void Plane_ConicsInThePlane_MapBack()
    {
        var aRandom = new Random(14011);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem aPlAx = RandomAx3(aRandom);
            var aPln = new Plane3D(aPlAx);
            // A conic in the plane: its axis is the plane's normal (either way), its X direction any in-plane one
            double anA = Real(aRandom, 0, 2 * M_PI);
            Point3D aLoc = global::Kerf.Engine.Geometry.ElSLib.Value(Real(aRandom, -10, 10), Real(aRandom, -10, 10), aPln);
            Direction3D aX = new(new Vector3D(aPlAx.XDirection) * Math.Cos(anA) + new Vector3D(aPlAx.YDirection) * Math.Sin(anA));
            Direction3D aN = aRandom.Next(2) == 0 ? aPlAx.Direction : aPlAx.Direction.Reversed();
            var aCAx = new Frame3D(aLoc, aN, aX);
            double aMaj = Real(aRandom, 1, 10), aMin = Real(aRandom, 0.5, aMaj);

            var aCirc = new Circle3D(aCAx, aMaj);
            var anElips = new Ellipse3D(aCAx, aMaj, aMin);
            var aParab = new Parabola3D(aCAx, aMin);
            var aHypr = new Hyperbola3D(aCAx, aMaj, aMin);
            var aLin = new Line3D(aLoc, aX);
            Circle2D aCirc2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, aCirc);
            Ellipse2D anElips2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, anElips);
            Parabola2D aParab2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, aParab);
            Hyperbola2D aHypr2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, aHypr);
            Line2D aLin2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, aLin);
            for (int j = 0; j <= 10; j++)
            {
                double aT = -3 + 0.6 * j;
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc), Back(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc2d)), Tol, $"case {i} circle");
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, anElips), Back(global::Kerf.Engine.Geometry.ElCLib.Value(aT, anElips2d)), Tol, $"case {i} ellipse");
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aParab), Back(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aParab2d)), Tol, $"case {i} parabola");
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aHypr), Back(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aHypr2d)), 1e-8, $"case {i} hyperbola");
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aLin), Back(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aLin2d)), Tol, $"case {i} line");
            }

            Point3D Back(in Point2D theP) => global::Kerf.Engine.Geometry.ElSLib.Value(theP.X, theP.Y, aPln);
        }
    }

    [Fact]
    public void Plane_AnyLine_ProjectsOnItsShadow()
    {
        var aRandom = new Random(14012);
        for (int i = 0; i < Cases; i++)
        {
            var aPln = new Plane3D(RandomAx3(aRandom));
            Direction3D aD = RandomDir(aRandom);
            if (aD.IsParallel(aPln.Axis.Direction, 0.05))
            {
                continue;
            }
            var aLin = new Line3D(RandomPnt(aRandom), aD);
            var aProj = new global::Kerf.Engine.Geometry.Projection.PlaneProjector(aPln, aLin);
            Assert.True(aProj.IsDone);
            Assert.Equal(Line, aProj.GetType());
            Line2D aLin2d = aProj.Line();
            for (int j = 0; j <= 10; j++)
            {
                Point3D aP = global::Kerf.Engine.Geometry.ElCLib.Value(-10 + 2 * j, aLin);
                double aU, aV;
                global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, aP, out aU, out aV);
                Assert.True(aLin2d.DistanceTo(new Point2D(aU, aV)) < Tol, $"case {i}");
            }
        }
    }

    [Fact]
    public void Plane_ThrowsOnOtherType()
    {
        var aProj = new global::Kerf.Engine.Geometry.Projection.PlaneProjector(new Plane3D(), new Line3D(new Point3D(), new Direction3D(1, 0, 0)));
        EngineAssert.Throws<NoSuchObjectException>(() => aProj.Circle());
        EngineAssert.Throws<NoSuchObjectException>(() => aProj.Ellipse());
        EngineAssert.Throws<NoSuchObjectException>(() => aProj.Parabola());
        EngineAssert.Throws<NoSuchObjectException>(() => aProj.Hyperbola());
        var anEmpty = new global::Kerf.Engine.Geometry.Projection.PlaneProjector();
        Assert.False(anEmpty.IsDone);
        Assert.Equal(BSplineCurve, anEmpty.GetType());
        Assert.Null(anEmpty.BSpline());
        Assert.Null(anEmpty.Bezier());
    }

    [Fact]
    public void Plane_LineNormalToThePlane_Throws()
    {
        // The projected direction is null: gp_Dir2d throws, where OCCT's release build (No_Exception) returns a
        // line with a NaN direction (decision 9)
        EngineAssert.Throws<ConstructionErrorException>(() => global::Kerf.Engine.Geometry.Projection.ProjLib.Project(new Plane3D(), new Line3D()));
    }

    // Cylinder

    [Fact]
    public void Cylinder_GeneratricesAndParallels_MapBack()
    {
        var aRandom = new Random(14013);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 10);
            var aCyl = new Cylinder3D(anAx, aR);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -10, 10);
            Point3D aP = global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCyl);

            // A generatrix, either way
            Direction3D aD = aRandom.Next(2) == 0 ? anAx.Direction : anAx.Direction.Reversed();
            var aLin = new Line3D(aP, aD);
            Line2D aLin2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aCyl, aLin);

            // A parallel, either orientation, starting anywhere
            Point3D aC = global::Kerf.Engine.Geometry.ElSLib.Value(0, aV, aCyl).Translated(new Vector3D(anAx.XDirection) * -aR);
            Point3D aStart = global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCyl);
            var aCirc = new Circle3D(new Frame3D(aC, aRandom.Next(2) == 0 ? anAx.Direction : anAx.Direction.Reversed(),
                                               new Direction3D(new Vector3D(aC, aStart))), aR);
            Line2D aCirc2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aCyl, aCirc);
            for (int j = 0; j <= 10; j++)
            {
                double aT = -3 + 0.6 * j;
                Point2D aL = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aLin2d), aCi = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc2d);
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aLin), global::Kerf.Engine.Geometry.ElSLib.Value(aL.X, aL.Y, aCyl), Tol, $"case {i} line");
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc), global::Kerf.Engine.Geometry.ElSLib.Value(aCi.X, aCi.Y, aCyl), Tol, $"case {i} circle");
            }
        }
    }

    [Fact]
    public void Cylinder_SkewLineAndTiltedCircle_NotDone()
    {
        var aCyl = new Cylinder3D(new CoordinateSystem(), 2.0);
        var aLin = new global::Kerf.Engine.Geometry.Projection.CylinderProjector(aCyl, new Line3D(new Point3D(2, 0, 0), new Direction3D(0, 1, 1)));
        Assert.False(aLin.IsDone);
        Assert.Equal(OtherCurve, aLin.GetType());
        EngineAssert.Throws<NoSuchObjectException>(() => aLin.Line());
        var aCirc = new global::Kerf.Engine.Geometry.Projection.CylinderProjector(aCyl, new Circle3D(new Frame3D(new Point3D(), new Direction3D(0, 1, 1)), 2.0));
        Assert.False(aCirc.IsDone);
        var anElips = new global::Kerf.Engine.Geometry.Projection.CylinderProjector(aCyl, new Ellipse3D(new Frame3D(new Point3D(), new Direction3D(0, 1, 1)), 3.0, 2.0));
        Assert.False(anElips.IsDone);
        Assert.Equal(OtherCurve, anElips.GetType());
    }

    // Cone

    [Fact]
    public void Cone_GeneratricesAndParallels_MapBack()
    {
        var aRandom = new Random(14014);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 10), anAng = Real(aRandom, 0.1, 1.4) * (aRandom.Next(2) == 0 ? 1 : -1);
            var aCone = new Cone3D(anAx, anAng, aR);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -10, 10);

            // A generatrix through a point of the cone (the apex sometimes), either way
            Point3D aP = aRandom.Next(8) == 0 ? aCone.Apex : global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCone);
            global::Kerf.Engine.Geometry.ElSLib.ConeD1(aU, aV, aCone.Position, aCone.RefRadius, aCone.SemiAngle, out _, out _, out Vector3D aDv);
            Direction3D aD = aRandom.Next(2) == 0 ? new Direction3D(aDv) : new Direction3D(aDv).Reversed();
            var aLin = new Line3D(aP, aD);
            var aLinProj = new global::Kerf.Engine.Geometry.Projection.ConeProjector(aCone, aLin);
            Assert.True(aLinProj.IsDone, $"case {i} line");

            // A parallel, either orientation, starting anywhere
            double aVc = aV;
            double aRc = aR + aVc * Math.Sin(anAng);
            if (Math.Abs(aRc) < 0.1)
            {
                continue;
            }
            Point3D aC = anAx.Location.Translated(new Vector3D(anAx.Direction) * (aVc * Math.Cos(anAng)));
            Point3D aStart = global::Kerf.Engine.Geometry.ElSLib.Value(aU, aVc, aCone);
            var aCirc = new Circle3D(new Frame3D(aC, aRandom.Next(2) == 0 ? anAx.Direction : anAx.Direction.Reversed(),
                                               new Direction3D(new Vector3D(aC, aStart))), Math.Abs(aRc));
            var aCircProj = new global::Kerf.Engine.Geometry.Projection.ConeProjector(aCone, aCirc);
            Assert.True(aCircProj.IsDone, $"case {i} circle");
            for (int j = 0; j <= 10; j++)
            {
                double aT = -3 + 0.6 * j;
                Point2D aL = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aLinProj.Line()), aCi = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCircProj.Line());
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aLin), global::Kerf.Engine.Geometry.ElSLib.Value(aL.X, aL.Y, aCone), Tol, $"case {i} line");
                AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc), global::Kerf.Engine.Geometry.ElSLib.Value(aCi.X, aCi.Y, aCone), Tol, $"case {i} circle");
            }
        }
    }

    [Fact]
    public void Cone_SkewLine_NotDone()
    {
        var aCone = new Cone3D(new CoordinateSystem(), M_PI / 6, 2.0);
        var aProj = new global::Kerf.Engine.Geometry.Projection.ConeProjector(aCone, new Line3D(new Point3D(2, 0, 0), new Direction3D(0, 1, 0)));
        Assert.False(aProj.IsDone);
        Assert.Equal(OtherCurve, aProj.GetType());
    }

    // Sphere

    [Fact]
    public void Sphere_ParallelsAndMeridians_MapBack()
    {
        var aRandom = new Random(14015);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 10);
            var aSph = new Sphere3D(anAx, aR);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -1.4, 1.4);

            // A parallel (iso V), either orientation, starting anywhere
            Point3D aC = anAx.Location.Translated(new Vector3D(anAx.Direction) * (aR * Math.Sin(aV)));
            Point3D aStart = global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aSph);
            var aPar = new Circle3D(new Frame3D(aC, aRandom.Next(2) == 0 ? anAx.Direction : anAx.Direction.Reversed(),
                                              new Direction3D(new Vector3D(aC, aStart))), aR * Math.Cos(aV));

            // A meridian (iso U): a great circle through the poles, starting anywhere
            Direction3D aMerN = new(new Vector3D(global::Kerf.Engine.Geometry.ElSLib.Value(aU, 0, aSph), global::Kerf.Engine.Geometry.ElSLib.Value(aU + M_PI / 2, 0, aSph)));
            var aMer = new Circle3D(new Frame3D(anAx.Location, aRandom.Next(2) == 0 ? aMerN : aMerN.Reversed(),
                                              new Direction3D(new Vector3D(anAx.Location, aStart))), aR);

            foreach ((Circle3D aCirc, string aWhat) in new[] { (aPar, "parallel"), (aMer, "meridian") })
            {
                var aProj = new global::Kerf.Engine.Geometry.Projection.SphereProjector(aSph, aCirc);
                Assert.True(aProj.IsDone, $"case {i} {aWhat}");
                for (int j = 0; j <= 10; j++)
                {
                    double aT = -3 + 0.6 * j;
                    Point2D aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aProj.Line());
                    AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc), global::Kerf.Engine.Geometry.ElSLib.Value(aP.X, aP.Y, aSph), Tol, $"case {i} {aWhat}");
                }

                // SetInBounds keeps the curve and puts its first point in the natural bounds
                aProj.SetInBounds(0.0);
                Point2D aFirst = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, aProj.Line());
                Assert.InRange(aFirst.Y, -M_PI / 2 - 1e-7, M_PI / 2 + 1e-7);
                Assert.InRange(aFirst.X, -1e-9, 2 * M_PI + 1e-9);
                for (int j = 0; j <= 10; j++)
                {
                    double aT = -3 + 0.6 * j;
                    Point2D aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aProj.Line());
                    AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc), global::Kerf.Engine.Geometry.ElSLib.Value(aP.X, aP.Y, aSph), Tol, $"case {i} {aWhat} in bounds");
                }
            }
        }
    }

    [Fact]
    public void Sphere_OtherCircle_NotDone()
    {
        var aSph = new Sphere3D(new CoordinateSystem(), 2.0);
        var aProj = new global::Kerf.Engine.Geometry.Projection.SphereProjector(aSph, new Circle3D(new Frame3D(new Point3D(0.5, 0, 0), new Direction3D(0, 1, 1)), 1.0));
        Assert.False(aProj.IsDone);
        EngineAssert.Throws<NotDoneException>(() => aProj.SetInBounds(0.0));
        var aLin = new global::Kerf.Engine.Geometry.Projection.SphereProjector(aSph);
        aLin.Project(new Line3D());
        Assert.Equal(OtherCurve, aLin.GetType());
    }

    // Torus

    [Fact]
    public void Torus_ParallelsAndMeridians_MapBack()
    {
        var aRandom = new Random(14016);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aMaj = Real(aRandom, 3, 10), aMin = Real(aRandom, 0.5, aMaj - 1);
            var aTor = new Torus3D(anAx, aMaj, aMin);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, 0, 2 * M_PI);
            Point3D aStart = global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aTor);

            // A parallel (iso V)
            Point3D aC = anAx.Location.Translated(new Vector3D(anAx.Direction) * (aMin * Math.Sin(aV)));
            var aPar = new Circle3D(new Frame3D(aC, aRandom.Next(2) == 0 ? anAx.Direction : anAx.Direction.Reversed(),
                                              new Direction3D(new Vector3D(aC, aStart))), aMaj + aMin * Math.Cos(aV));

            // A meridian (iso U): the circle of the minor radius
            Point3D aMc = global::Kerf.Engine.Geometry.ElSLib.Value(aU, 0, aTor).Translated(new Vector3D(global::Kerf.Engine.Geometry.ElSLib.Value(aU, 0, aTor), global::Kerf.Engine.Geometry.ElSLib.Value(aU, M_PI, aTor)) * 0.5);
            Direction3D aMerN = new(new Vector3D(anAx.Direction).Crossed(new Vector3D(anAx.Location, aMc)));
            var aMer = new Circle3D(new Frame3D(aMc, aRandom.Next(2) == 0 ? aMerN : aMerN.Reversed(),
                                              new Direction3D(new Vector3D(aMc, aStart))), aMin);

            foreach ((Circle3D aCirc, string aWhat) in new[] { (aPar, "parallel"), (aMer, "meridian") })
            {
                var aProj = new global::Kerf.Engine.Geometry.Projection.TorusProjector(aTor, aCirc);
                Assert.True(aProj.IsDone, $"case {i} {aWhat}");
                for (int j = 0; j <= 10; j++)
                {
                    double aT = -3 + 0.6 * j;
                    Point2D aP = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aProj.Line());
                    AssertSame(global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCirc), global::Kerf.Engine.Geometry.ElSLib.Value(aP.X, aP.Y, aTor), 1e-8, $"case {i} {aWhat}");
                }
            }
        }
    }

    // ProjLib's points, IsAnaSurf and the frames

    [Fact]
    public void Points_RoundTrip()
    {
        var aRandom = new Random(14017);
        for (int i = 0; i < Cases; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -1.4, 1.4);
            var aPln = new Plane3D(anAx);
            var aCyl = new Cylinder3D(anAx, 3);
            var aCone = new Cone3D(anAx, 0.5, 3);
            var aSph = new Sphere3D(anAx, 3);
            var aTor = new Torus3D(anAx, 5, 2);
            Check(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aPln)), aU, aV);
            Check(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aCyl, global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCyl)), aU, aV);
            Check(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aCone, global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aCone)), aU, aV);
            Check(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aSph, global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, aSph)), aU, aV);
            Check(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aTor, global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV + 1.5, aTor)), aU, aV + 1.5);
        }

        static void Check(in Point2D theP, double theU, double theV)
        {
            Assert.Equal(theU, theP.X, 1e-9);
            Assert.Equal(theV, theP.Y, 1e-9);
        }
    }

    [Fact]
    public void Projector_Frames()
    {
        var aProj = new global::Kerf.Engine.Geometry.Projection.Projector();
        aProj.SetType(Line);
        aProj.UFrame(0, 1, 0, 2 * M_PI); // the default line (origin, X) is already in the frame
        Assert.Equal(0.0, aProj.Line().Location.X);

        var aCyl = new global::Kerf.Engine.Geometry.Projection.CylinderProjector(new Cylinder3D(new CoordinateSystem(), 1.0),
                                        new Line3D(new Point3D(0, 1, 0), new Direction3D(0, 0, 1)));
        Assert.Equal(M_PI / 2, aCyl.Line().Location.X, 1e-15);
        aCyl.UFrame(0.0, 1.0, 10.0, 2 * M_PI);
        Assert.Equal(M_PI / 2 + 4 * M_PI, aCyl.Line().Location.X, 1e-12);
        aCyl.VFrame(0.0, 1.0, 5.0, 2.0);
        Assert.Equal(6.0, aCyl.Line().Location.Y, 1e-12);
        Assert.False(aCyl.IsPeriodic);
        aCyl.SetPeriodic();
        Assert.True(aCyl.IsPeriodic);
    }
}
