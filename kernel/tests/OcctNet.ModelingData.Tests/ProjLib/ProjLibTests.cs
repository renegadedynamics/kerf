// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Property tests of the elementary projections: a curve lying on the surface projects to a 2D line or conic
/// whose points, mapped back with ElSLib, are the curve's points at the same parameter (or, for lines in general
/// position on a plane, on the projected line).
/// </summary>
public class ProjLibTests
{
    private const int Cases = 200;
    private const double Tol = 1.0e-9;

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
        var anAx = new gp_Ax3(RandomPnt(theRandom), aN, aX);
        return theRandom.Next(2) == 0 ? anAx : anAx.YReversed();
    }

    private static void AssertSame(in gp_Pnt theExpected, in gp_Pnt theActual, double theTol, string theWhat)
        => Assert.True(theExpected.Distance(theActual) <= theTol,
                       $"{theWhat}: expected {theExpected}, got {theActual} (distance {theExpected.Distance(theActual):E3})");

    // Plane

    [Fact]
    public void Plane_ConicsInThePlane_MapBack()
    {
        var aRandom = new Random(14011);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 aPlAx = RandomAx3(aRandom);
            var aPln = new gp_Pln(aPlAx);
            // A conic in the plane: its axis is the plane's normal (either way), its X direction any in-plane one
            double anA = Real(aRandom, 0, 2 * M_PI);
            gp_Pnt aLoc = ElSLib.Value(Real(aRandom, -10, 10), Real(aRandom, -10, 10), aPln);
            gp_Dir aX = new(new gp_Vec(aPlAx.XDirection()) * Math.Cos(anA) + new gp_Vec(aPlAx.YDirection()) * Math.Sin(anA));
            gp_Dir aN = aRandom.Next(2) == 0 ? aPlAx.Direction() : aPlAx.Direction().Reversed();
            var aCAx = new gp_Ax2(aLoc, aN, aX);
            double aMaj = Real(aRandom, 1, 10), aMin = Real(aRandom, 0.5, aMaj);

            var aCirc = new gp_Circ(aCAx, aMaj);
            var anElips = new gp_Elips(aCAx, aMaj, aMin);
            var aParab = new gp_Parab(aCAx, aMin);
            var aHypr = new gp_Hypr(aCAx, aMaj, aMin);
            var aLin = new gp_Lin(aLoc, aX);
            gp_Circ2d aCirc2d = ProjLib.Project(aPln, aCirc);
            gp_Elips2d anElips2d = ProjLib.Project(aPln, anElips);
            gp_Parab2d aParab2d = ProjLib.Project(aPln, aParab);
            gp_Hypr2d aHypr2d = ProjLib.Project(aPln, aHypr);
            gp_Lin2d aLin2d = ProjLib.Project(aPln, aLin);
            for (int j = 0; j <= 10; j++)
            {
                double aT = -3 + 0.6 * j;
                AssertSame(ElCLib.Value(aT, aCirc), Back(ElCLib.Value(aT, aCirc2d)), Tol, $"case {i} circle");
                AssertSame(ElCLib.Value(aT, anElips), Back(ElCLib.Value(aT, anElips2d)), Tol, $"case {i} ellipse");
                AssertSame(ElCLib.Value(aT, aParab), Back(ElCLib.Value(aT, aParab2d)), Tol, $"case {i} parabola");
                AssertSame(ElCLib.Value(aT, aHypr), Back(ElCLib.Value(aT, aHypr2d)), 1e-8, $"case {i} hyperbola");
                AssertSame(ElCLib.Value(aT, aLin), Back(ElCLib.Value(aT, aLin2d)), Tol, $"case {i} line");
            }

            gp_Pnt Back(in gp_Pnt2d theP) => ElSLib.Value(theP.X(), theP.Y(), aPln);
        }
    }

    [Fact]
    public void Plane_AnyLine_ProjectsOnItsShadow()
    {
        var aRandom = new Random(14012);
        for (int i = 0; i < Cases; i++)
        {
            var aPln = new gp_Pln(RandomAx3(aRandom));
            gp_Dir aD = RandomDir(aRandom);
            if (aD.IsParallel(aPln.Axis().Direction(), 0.05))
            {
                continue;
            }
            var aLin = new gp_Lin(RandomPnt(aRandom), aD);
            var aProj = new ProjLib_Plane(aPln, aLin);
            Assert.True(aProj.IsDone());
            Assert.Equal(GeomAbs_Line, aProj.GetType());
            gp_Lin2d aLin2d = aProj.Line();
            for (int j = 0; j <= 10; j++)
            {
                gp_Pnt aP = ElCLib.Value(-10 + 2 * j, aLin);
                double aU, aV;
                ElSLib.Parameters(aPln, aP, out aU, out aV);
                Assert.True(aLin2d.Distance(new gp_Pnt2d(aU, aV)) < Tol, $"case {i}");
            }
        }
    }

    [Fact]
    public void Plane_ThrowsOnOtherType()
    {
        var aProj = new ProjLib_Plane(new gp_Pln(), new gp_Lin(new gp_Pnt(), new gp_Dir(1, 0, 0)));
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Circle());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Ellipse());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Parabola());
        Assert.Throws<Standard_NoSuchObject>(() => aProj.Hyperbola());
        var anEmpty = new ProjLib_Plane();
        Assert.False(anEmpty.IsDone());
        Assert.Equal(GeomAbs_BSplineCurve, anEmpty.GetType());
        Assert.Null(anEmpty.BSpline());
        Assert.Null(anEmpty.Bezier());
    }

    [Fact]
    public void Plane_LineNormalToThePlane_Throws()
    {
        // The projected direction is null: gp_Dir2d throws, where OCCT's release build (No_Exception) returns a
        // line with a NaN direction (decision 9)
        Assert.Throws<Standard_ConstructionError>(() => ProjLib.Project(new gp_Pln(), new gp_Lin()));
    }

    // Cylinder

    [Fact]
    public void Cylinder_GeneratricesAndParallels_MapBack()
    {
        var aRandom = new Random(14013);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 10);
            var aCyl = new gp_Cylinder(anAx, aR);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -10, 10);
            gp_Pnt aP = ElSLib.Value(aU, aV, aCyl);

            // A generatrix, either way
            gp_Dir aD = aRandom.Next(2) == 0 ? anAx.Direction() : anAx.Direction().Reversed();
            var aLin = new gp_Lin(aP, aD);
            gp_Lin2d aLin2d = ProjLib.Project(aCyl, aLin);

            // A parallel, either orientation, starting anywhere
            gp_Pnt aC = ElSLib.Value(0, aV, aCyl).Translated(new gp_Vec(anAx.XDirection()) * -aR);
            gp_Pnt aStart = ElSLib.Value(aU, aV, aCyl);
            var aCirc = new gp_Circ(new gp_Ax2(aC, aRandom.Next(2) == 0 ? anAx.Direction() : anAx.Direction().Reversed(),
                                               new gp_Dir(new gp_Vec(aC, aStart))), aR);
            gp_Lin2d aCirc2d = ProjLib.Project(aCyl, aCirc);
            for (int j = 0; j <= 10; j++)
            {
                double aT = -3 + 0.6 * j;
                gp_Pnt2d aL = ElCLib.Value(aT, aLin2d), aCi = ElCLib.Value(aT, aCirc2d);
                AssertSame(ElCLib.Value(aT, aLin), ElSLib.Value(aL.X(), aL.Y(), aCyl), Tol, $"case {i} line");
                AssertSame(ElCLib.Value(aT, aCirc), ElSLib.Value(aCi.X(), aCi.Y(), aCyl), Tol, $"case {i} circle");
            }
        }
    }

    [Fact]
    public void Cylinder_SkewLineAndTiltedCircle_NotDone()
    {
        var aCyl = new gp_Cylinder(new gp_Ax3(), 2.0);
        var aLin = new ProjLib_Cylinder(aCyl, new gp_Lin(new gp_Pnt(2, 0, 0), new gp_Dir(0, 1, 1)));
        Assert.False(aLin.IsDone());
        Assert.Equal(GeomAbs_OtherCurve, aLin.GetType());
        Assert.Throws<Standard_NoSuchObject>(() => aLin.Line());
        var aCirc = new ProjLib_Cylinder(aCyl, new gp_Circ(new gp_Ax2(new gp_Pnt(), new gp_Dir(0, 1, 1)), 2.0));
        Assert.False(aCirc.IsDone());
        var anElips = new ProjLib_Cylinder(aCyl, new gp_Elips(new gp_Ax2(new gp_Pnt(), new gp_Dir(0, 1, 1)), 3.0, 2.0));
        Assert.False(anElips.IsDone());
        Assert.Equal(GeomAbs_OtherCurve, anElips.GetType());
    }

    // Cone

    [Fact]
    public void Cone_GeneratricesAndParallels_MapBack()
    {
        var aRandom = new Random(14014);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 10), anAng = Real(aRandom, 0.1, 1.4) * (aRandom.Next(2) == 0 ? 1 : -1);
            var aCone = new gp_Cone(anAx, anAng, aR);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -10, 10);

            // A generatrix through a point of the cone (the apex sometimes), either way
            gp_Pnt aP = aRandom.Next(8) == 0 ? aCone.Apex() : ElSLib.Value(aU, aV, aCone);
            ElSLib.ConeD1(aU, aV, aCone.Position(), aCone.RefRadius(), aCone.SemiAngle(), out _, out _, out gp_Vec aDv);
            gp_Dir aD = aRandom.Next(2) == 0 ? new gp_Dir(aDv) : new gp_Dir(aDv).Reversed();
            var aLin = new gp_Lin(aP, aD);
            var aLinProj = new ProjLib_Cone(aCone, aLin);
            Assert.True(aLinProj.IsDone(), $"case {i} line");

            // A parallel, either orientation, starting anywhere
            double aVc = aV;
            double aRc = aR + aVc * Math.Sin(anAng);
            if (Math.Abs(aRc) < 0.1)
            {
                continue;
            }
            gp_Pnt aC = anAx.Location().Translated(new gp_Vec(anAx.Direction()) * (aVc * Math.Cos(anAng)));
            gp_Pnt aStart = ElSLib.Value(aU, aVc, aCone);
            var aCirc = new gp_Circ(new gp_Ax2(aC, aRandom.Next(2) == 0 ? anAx.Direction() : anAx.Direction().Reversed(),
                                               new gp_Dir(new gp_Vec(aC, aStart))), Math.Abs(aRc));
            var aCircProj = new ProjLib_Cone(aCone, aCirc);
            Assert.True(aCircProj.IsDone(), $"case {i} circle");
            for (int j = 0; j <= 10; j++)
            {
                double aT = -3 + 0.6 * j;
                gp_Pnt2d aL = ElCLib.Value(aT, aLinProj.Line()), aCi = ElCLib.Value(aT, aCircProj.Line());
                AssertSame(ElCLib.Value(aT, aLin), ElSLib.Value(aL.X(), aL.Y(), aCone), Tol, $"case {i} line");
                AssertSame(ElCLib.Value(aT, aCirc), ElSLib.Value(aCi.X(), aCi.Y(), aCone), Tol, $"case {i} circle");
            }
        }
    }

    [Fact]
    public void Cone_SkewLine_NotDone()
    {
        var aCone = new gp_Cone(new gp_Ax3(), M_PI / 6, 2.0);
        var aProj = new ProjLib_Cone(aCone, new gp_Lin(new gp_Pnt(2, 0, 0), new gp_Dir(0, 1, 0)));
        Assert.False(aProj.IsDone());
        Assert.Equal(GeomAbs_OtherCurve, aProj.GetType());
    }

    // Sphere

    [Fact]
    public void Sphere_ParallelsAndMeridians_MapBack()
    {
        var aRandom = new Random(14015);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aR = Real(aRandom, 0.5, 10);
            var aSph = new gp_Sphere(anAx, aR);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -1.4, 1.4);

            // A parallel (iso V), either orientation, starting anywhere
            gp_Pnt aC = anAx.Location().Translated(new gp_Vec(anAx.Direction()) * (aR * Math.Sin(aV)));
            gp_Pnt aStart = ElSLib.Value(aU, aV, aSph);
            var aPar = new gp_Circ(new gp_Ax2(aC, aRandom.Next(2) == 0 ? anAx.Direction() : anAx.Direction().Reversed(),
                                              new gp_Dir(new gp_Vec(aC, aStart))), aR * Math.Cos(aV));

            // A meridian (iso U): a great circle through the poles, starting anywhere
            gp_Dir aMerN = new(new gp_Vec(ElSLib.Value(aU, 0, aSph), ElSLib.Value(aU + M_PI / 2, 0, aSph)));
            var aMer = new gp_Circ(new gp_Ax2(anAx.Location(), aRandom.Next(2) == 0 ? aMerN : aMerN.Reversed(),
                                              new gp_Dir(new gp_Vec(anAx.Location(), aStart))), aR);

            foreach ((gp_Circ aCirc, string aWhat) in new[] { (aPar, "parallel"), (aMer, "meridian") })
            {
                var aProj = new ProjLib_Sphere(aSph, aCirc);
                Assert.True(aProj.IsDone(), $"case {i} {aWhat}");
                for (int j = 0; j <= 10; j++)
                {
                    double aT = -3 + 0.6 * j;
                    gp_Pnt2d aP = ElCLib.Value(aT, aProj.Line());
                    AssertSame(ElCLib.Value(aT, aCirc), ElSLib.Value(aP.X(), aP.Y(), aSph), Tol, $"case {i} {aWhat}");
                }

                // SetInBounds keeps the curve and puts its first point in the natural bounds
                aProj.SetInBounds(0.0);
                gp_Pnt2d aFirst = ElCLib.Value(0.0, aProj.Line());
                Assert.InRange(aFirst.Y(), -M_PI / 2 - 1e-7, M_PI / 2 + 1e-7);
                Assert.InRange(aFirst.X(), -1e-9, 2 * M_PI + 1e-9);
                for (int j = 0; j <= 10; j++)
                {
                    double aT = -3 + 0.6 * j;
                    gp_Pnt2d aP = ElCLib.Value(aT, aProj.Line());
                    AssertSame(ElCLib.Value(aT, aCirc), ElSLib.Value(aP.X(), aP.Y(), aSph), Tol, $"case {i} {aWhat} in bounds");
                }
            }
        }
    }

    [Fact]
    public void Sphere_OtherCircle_NotDone()
    {
        var aSph = new gp_Sphere(new gp_Ax3(), 2.0);
        var aProj = new ProjLib_Sphere(aSph, new gp_Circ(new gp_Ax2(new gp_Pnt(0.5, 0, 0), new gp_Dir(0, 1, 1)), 1.0));
        Assert.False(aProj.IsDone());
        Assert.Throws<StdFail_NotDone>(() => aProj.SetInBounds(0.0));
        var aLin = new ProjLib_Sphere(aSph);
        aLin.Project(new gp_Lin());
        Assert.Equal(GeomAbs_OtherCurve, aLin.GetType());
    }

    // Torus

    [Fact]
    public void Torus_ParallelsAndMeridians_MapBack()
    {
        var aRandom = new Random(14016);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aMaj = Real(aRandom, 3, 10), aMin = Real(aRandom, 0.5, aMaj - 1);
            var aTor = new gp_Torus(anAx, aMaj, aMin);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, 0, 2 * M_PI);
            gp_Pnt aStart = ElSLib.Value(aU, aV, aTor);

            // A parallel (iso V)
            gp_Pnt aC = anAx.Location().Translated(new gp_Vec(anAx.Direction()) * (aMin * Math.Sin(aV)));
            var aPar = new gp_Circ(new gp_Ax2(aC, aRandom.Next(2) == 0 ? anAx.Direction() : anAx.Direction().Reversed(),
                                              new gp_Dir(new gp_Vec(aC, aStart))), aMaj + aMin * Math.Cos(aV));

            // A meridian (iso U): the circle of the minor radius
            gp_Pnt aMc = ElSLib.Value(aU, 0, aTor).Translated(new gp_Vec(ElSLib.Value(aU, 0, aTor), ElSLib.Value(aU, M_PI, aTor)) * 0.5);
            gp_Dir aMerN = new(new gp_Vec(anAx.Direction()).Crossed(new gp_Vec(anAx.Location(), aMc)));
            var aMer = new gp_Circ(new gp_Ax2(aMc, aRandom.Next(2) == 0 ? aMerN : aMerN.Reversed(),
                                              new gp_Dir(new gp_Vec(aMc, aStart))), aMin);

            foreach ((gp_Circ aCirc, string aWhat) in new[] { (aPar, "parallel"), (aMer, "meridian") })
            {
                var aProj = new ProjLib_Torus(aTor, aCirc);
                Assert.True(aProj.IsDone(), $"case {i} {aWhat}");
                for (int j = 0; j <= 10; j++)
                {
                    double aT = -3 + 0.6 * j;
                    gp_Pnt2d aP = ElCLib.Value(aT, aProj.Line());
                    AssertSame(ElCLib.Value(aT, aCirc), ElSLib.Value(aP.X(), aP.Y(), aTor), 1e-8, $"case {i} {aWhat}");
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
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aU = Real(aRandom, 0, 2 * M_PI), aV = Real(aRandom, -1.4, 1.4);
            var aPln = new gp_Pln(anAx);
            var aCyl = new gp_Cylinder(anAx, 3);
            var aCone = new gp_Cone(anAx, 0.5, 3);
            var aSph = new gp_Sphere(anAx, 3);
            var aTor = new gp_Torus(anAx, 5, 2);
            Check(ProjLib.Project(aPln, ElSLib.Value(aU, aV, aPln)), aU, aV);
            Check(ProjLib.Project(aCyl, ElSLib.Value(aU, aV, aCyl)), aU, aV);
            Check(ProjLib.Project(aCone, ElSLib.Value(aU, aV, aCone)), aU, aV);
            Check(ProjLib.Project(aSph, ElSLib.Value(aU, aV, aSph)), aU, aV);
            Check(ProjLib.Project(aTor, ElSLib.Value(aU, aV + 1.5, aTor)), aU, aV + 1.5);
        }

        static void Check(in gp_Pnt2d theP, double theU, double theV)
        {
            Assert.Equal(theU, theP.X(), 1e-9);
            Assert.Equal(theV, theP.Y(), 1e-9);
        }
    }

    [Fact]
    public void Projector_Frames()
    {
        var aProj = new ProjLib_Projector();
        aProj.SetType(GeomAbs_Line);
        aProj.UFrame(0, 1, 0, 2 * M_PI); // the default line (origin, X) is already in the frame
        Assert.Equal(0.0, aProj.Line().Location().X());

        var aCyl = new ProjLib_Cylinder(new gp_Cylinder(new gp_Ax3(), 1.0),
                                        new gp_Lin(new gp_Pnt(0, 1, 0), new gp_Dir(0, 0, 1)));
        Assert.Equal(M_PI / 2, aCyl.Line().Location().X(), 1e-15);
        aCyl.UFrame(0.0, 1.0, 10.0, 2 * M_PI);
        Assert.Equal(M_PI / 2 + 4 * M_PI, aCyl.Line().Location().X(), 1e-12);
        aCyl.VFrame(0.0, 1.0, 5.0, 2.0);
        Assert.Equal(6.0, aCyl.Line().Location().Y(), 1e-12);
        Assert.False(aCyl.IsPeriodic());
        aCyl.SetPeriodic();
        Assert.True(aCyl.IsPeriodic());
    }
}
