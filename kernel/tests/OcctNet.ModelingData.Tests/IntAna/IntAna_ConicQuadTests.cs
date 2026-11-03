// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Property tests of IntAna_Int3Pln, IntAna_IntLinTorus and IntAna_IntConicQuad: every intersection point lies on
/// both arguments, at its parameter on the conic. (The oracle tests compare them with native OCCT.)
/// </summary>
public class IntAna_ConicQuadTests
{
    private const int Cases = 300;

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

    private static double Real(Random theRandom, double theMin, double theMax)
        => theMin + (theMax - theMin) * theRandom.NextDouble();

    private static Point3D RandomPnt(Random theRandom, double theScale = 10)
        => new(Real(theRandom, -theScale, theScale), Real(theRandom, -theScale, theScale), Real(theRandom, -theScale, theScale));

    private static CoordinateSystem RandomAx3(Random theRandom, in Point3D theLoc)
    {
        Direction3D aN = RandomDir(theRandom);
        Direction3D aX = RandomDir(theRandom);
        while (aN.IsParallel(aX, 0.1))
        {
            aX = RandomDir(theRandom);
        }
        return new CoordinateSystem(theLoc, aN, aX);
    }

    /// <summary>The value of the quadric's equation at P, relative to the size of its terms.</summary>
    private static double Residual(IntAna_Quadric theQ, in Point3D theP)
    {
        theQ.Coefficients(out double xx, out double yy, out double zz, out double xy, out double xz, out double yz,
                          out double x, out double y, out double z, out double c);
        double X = theP.X(), Y = theP.Y(), Z = theP.Z();
        double aF = xx * X * X + yy * Y * Y + zz * Z * Z + 2 * (xy * X * Y + xz * X * Z + yz * Y * Z)
                    + 2 * (x * X + y * Y + z * Z) + c;
        double aScale = Math.Abs(xx * X * X) + Math.Abs(yy * Y * Y) + Math.Abs(zz * Z * Z)
                        + 2 * (Math.Abs(xy * X * Y) + Math.Abs(xz * X * Z) + Math.Abs(yz * Y * Z))
                        + 2 * (Math.Abs(x * X) + Math.Abs(y * Y) + Math.Abs(z * Z)) + Math.Abs(c);
        return Math.Abs(aF) / Math.Max(1.0, aScale);
    }

    [Fact]
    public void Int3Pln_PointOnThreePlanes()
    {
        var aRandom = new Random(14021);
        for (int i = 0; i < Cases; i++)
        {
            var aP1 = new Plane3D(RandomAx3(aRandom, RandomPnt(aRandom)));
            var aP2 = new Plane3D(RandomAx3(aRandom, RandomPnt(aRandom)));
            var aP3 = new Plane3D(RandomAx3(aRandom, RandomPnt(aRandom)));
            var anInt = new IntAna_Int3Pln(aP1, aP2, aP3);
            Assert.True(anInt.IsDone());
            if (anInt.IsEmpty())
            {
                continue;
            }
            Point3D aP = anInt.Value();
            double aScale = Math.Max(1.0, aP.XYZ().Modulus());
            Assert.True(aP1.Distance(aP) < 1e-9 * aScale, $"case {i}");
            Assert.True(aP2.Distance(aP) < 1e-9 * aScale, $"case {i}");
            Assert.True(aP3.Distance(aP) < 1e-9 * aScale, $"case {i}");
        }
    }

    [Fact]
    public void Int3Pln_KnownCases()
    {
        var anInt = new IntAna_Int3Pln(new Plane3D(new Point3D(1, 0, 0), new Direction3D(1, 0, 0)),
                                       new Plane3D(new Point3D(0, 2, 0), new Direction3D(0, 1, 0)),
                                       new Plane3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)));
        Assert.True(anInt.IsDone());
        Assert.False(anInt.IsEmpty());
        Assert.Equal(new Point3D(1, 2, 3), anInt.Value());

        var aParallel = new IntAna_Int3Pln(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                           new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)),
                                           new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        Assert.True(aParallel.IsEmpty());
        Assert.Throws<DomainErrorException>(() => aParallel.Value());

        var anEmpty = new IntAna_Int3Pln();
        Assert.False(anEmpty.IsDone());
        Assert.Throws<NotDoneException>(() => anEmpty.IsEmpty());
        Assert.Throws<NotDoneException>(() => anEmpty.Value());
    }

    [Fact]
    public void IntLinTorus_PointsOnBoth()
    {
        var aRandom = new Random(14022);
        int aWithPoints = 0;
        for (int i = 0; i < Cases; i++)
        {
            double aMaj = Real(aRandom, 3, 10), aMin = Real(aRandom, 0.5, aMaj - 0.5);
            CoordinateSystem anAx = RandomAx3(aRandom, RandomPnt(aRandom));
            var aTor = new Torus(anAx, aMaj, aMin);
            var aLin = new Line3D(anAx.Location().Translated(new Vector3D(Real(aRandom, -aMaj, aMaj), Real(aRandom, -aMaj, aMaj),
                                                                        Real(aRandom, -aMin, aMin))), RandomDir(aRandom));
            var anInt = new IntAna_IntLinTorus(aLin, aTor);
            if (!anInt.IsDone())
            {
                continue;
            }
            aWithPoints += anInt.NbPoints() > 0 ? 1 : 0;
            for (int j = 1; j <= anInt.NbPoints(); j++)
            {
                Point3D aP = anInt.Value(j);
                Assert.True(aP.Distance(global::Kerf.Engine.Geometry.ElCLib.Value(anInt.ParamOnLine(j), aLin)) < 1e-12 * Math.Max(1, aP.XYZ().Modulus()));
                anInt.ParamOnTorus(j, out double aFi, out double aTheta);
                Assert.True(aP.Distance(global::Kerf.Engine.Geometry.ElSLib.Value(aFi, aTheta, aTor)) < 1e-5, $"case {i}");
            }
        }
        Assert.True(aWithPoints > Cases / 2);
    }

    [Fact]
    public void IntLinTorus_AlongTheAxis_AndThroughTheCentre()
    {
        var aTor = new Torus(new CoordinateSystem(), 5, 1);
        // The axis misses the torus
        var anAxis = new IntAna_IntLinTorus(new Line3D(new Point3D(), new Direction3D(0, 0, 1)), aTor);
        Assert.True(anAxis.IsDone());
        Assert.Equal(0, anAxis.NbPoints());
        // The X axis crosses it 4 times, at -6, -4, 4, 6
        var aX = new IntAna_IntLinTorus(new Line3D(new Point3D(), new Direction3D(1, 0, 0)), aTor);
        Assert.True(aX.IsDone());
        Assert.Equal(4, aX.NbPoints());
        var aParams = Enumerable.Range(1, 4).Select(aX.ParamOnLine).OrderBy(t => t).ToArray();
        Assert.Equal(-6, aParams[0], 1e-9);
        Assert.Equal(-4, aParams[1], 1e-9);
        Assert.Equal(4, aParams[2], 1e-9);
        Assert.Equal(6, aParams[3], 1e-9);
        Assert.Throws<OutOfRangeException>(() => aX.Value(5));
        Assert.Throws<NotDoneException>(() => new IntAna_IntLinTorus().NbPoints());
    }

    [Fact]
    public void IntConicQuad_ConicsAndQuadrics_PointsOnBoth()
    {
        var aRandom = new Random(14023);
        int aWithPoints = 0;
        for (int i = 0; i < Cases; i++)
        {
            double aR = Real(aRandom, 1, 10);
            CoordinateSystem anAx = RandomAx3(aRandom, RandomPnt(aRandom));
            IntAna_Quadric aQuad = (i % 4) switch
            {
                0 => new IntAna_Quadric(new Plane3D(anAx)),
                1 => new IntAna_Quadric(new Cylinder(anAx, aR)),
                2 => new IntAna_Quadric(new Cone(anAx, Real(aRandom, 0.1, 1.4), aR)),
                _ => new IntAna_Quadric(new Sphere(anAx, aR)),
            };
            Point3D aLoc = anAx.Location().Translated(new Vector3D(Real(aRandom, -aR, aR), Real(aRandom, -aR, aR), Real(aRandom, -aR, aR)));
            var aCAx = new Frame3D(aLoc, RandomDir(aRandom));
            double aMaj = Real(aRandom, 0.5, 2 * aR), aMin = Real(aRandom, 0.2, aMaj);

            var aLin = new Line3D(aCAx.Axis());
            var aCirc = new Circle3D(aCAx, aMaj);
            var anElips = new Ellipse3D(aCAx, aMaj, aMin);
            var aParab = new Parabola3D(aCAx, aMin);
            var aHypr = new Hyperbola3D(aCAx, aMaj, aMin);
            Check(new IntAna_IntConicQuad(aLin, aQuad), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aLin), "line");
            Check(new IntAna_IntConicQuad(aCirc, aQuad), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCirc), "circle");
            Check(new IntAna_IntConicQuad(anElips, aQuad), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, anElips), "ellipse");
            Check(new IntAna_IntConicQuad(aParab, aQuad), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aParab), "parabola");
            Check(new IntAna_IntConicQuad(aHypr, aQuad), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aHypr), "hyperbola");

            void Check(IntAna_IntConicQuad theInt, Func<double, Point3D> theValue, string theWhat)
            {
                Assert.True(theInt.IsDone(), $"case {i} {theWhat}");
                if (theInt.IsInQuadric() || theInt.IsParallel())
                {
                    return;
                }
                aWithPoints += theInt.NbPoints() > 0 ? 1 : 0;
                for (int j = 1; j <= theInt.NbPoints(); j++)
                {
                    Point3D aP = theInt.Point(j);
                    double aScale = Math.Max(1.0, aP.XYZ().Modulus());
                    Assert.True(aP.Distance(theValue(theInt.ParamOnConic(j))) < 1e-9 * aScale, $"case {i} {theWhat} point {j}");
                    Assert.True(Residual(aQuad, aP) < 1e-7, $"case {i} {theWhat} point {j}: residual {Residual(aQuad, aP):E2}");
                }
            }
        }
        Assert.True(aWithPoints > Cases);
    }

    [Fact]
    public void IntConicQuad_ConicsAndPlanes_PointsOnBoth()
    {
        var aRandom = new Random(14024);
        for (int i = 0; i < Cases; i++)
        {
            var aPln = new Plane3D(RandomAx3(aRandom, RandomPnt(aRandom)));
            var aCAx = new Frame3D(aPln.Location().Translated(new Vector3D(RandomDir(aRandom)) * Real(aRandom, 0, 8)), RandomDir(aRandom));
            double aMaj = Real(aRandom, 0.5, 10), aMin = Real(aRandom, 0.2, aMaj);
            var aLin = new Line3D(aCAx.Axis());
            var aCirc = new Circle3D(aCAx, aMaj);
            var anElips = new Ellipse3D(aCAx, aMaj, aMin);
            var aParab = new Parabola3D(aCAx, aMin);
            var aHypr = new Hyperbola3D(aCAx, aMaj, aMin);
            Check(new IntAna_IntConicQuad(aLin, aPln, 1e-12), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aLin), "line");
            Check(new IntAna_IntConicQuad(aCirc, aPln, 1e-12, 1e-8), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aCirc), "circle");
            Check(new IntAna_IntConicQuad(anElips, aPln, 1e-12, 1e-8), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, anElips), "ellipse");
            Check(new IntAna_IntConicQuad(aParab, aPln, 1e-12), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aParab), "parabola");
            Check(new IntAna_IntConicQuad(aHypr, aPln, 1e-12), t => global::Kerf.Engine.Geometry.ElCLib.Value(t, aHypr), "hyperbola");

            void Check(IntAna_IntConicQuad theInt, Func<double, Point3D> theValue, string theWhat)
            {
                Assert.True(theInt.IsDone(), $"case {i} {theWhat}");
                if (theInt.IsInQuadric() || theInt.IsParallel())
                {
                    return;
                }
                for (int j = 1; j <= theInt.NbPoints(); j++)
                {
                    Point3D aP = theInt.Point(j);
                    double aScale = Math.Max(1.0, aP.XYZ().Modulus());
                    Assert.True(aP.Distance(theValue(theInt.ParamOnConic(j))) < 1e-8 * aScale, $"case {i} {theWhat} point {j}");
                    Assert.True(aPln.Distance(aP) < 1e-8 * aScale, $"case {i} {theWhat} point {j}");
                }
            }
        }
    }

    [Fact]
    public void IntConicQuad_ParallelAndInQuadric()
    {
        var aPln = new Plane3D();
        // A line in the plane, a line parallel to it, a line crossing it
        var anIn = new IntAna_IntConicQuad(new Line3D(new Point3D(1, 2, 0), new Direction3D(1, 1, 0)), aPln, 1e-12);
        Assert.True(anIn.IsDone());
        Assert.True(anIn.IsParallel() || anIn.IsInQuadric());
        Assert.Throws<DomainErrorException>(() => anIn.NbPoints());
        var aCross = new IntAna_IntConicQuad(new Line3D(new Point3D(1, 2, 3), new Direction3D(0, 0, -1)), aPln, 1e-12);
        Assert.Equal(1, aCross.NbPoints());
        Assert.Equal(3.0, aCross.ParamOnConic(1), 1e-15);
        Assert.True(aCross.Point(1).IsEqual(new Point3D(1, 2, 0), 1e-15));
        Assert.Throws<OutOfRangeException>(() => aCross.Point(2));

        // A circle of the sphere is in it
        var aSph = new IntAna_Quadric(new Sphere(new CoordinateSystem(), 2));
        var aCirc = new IntAna_IntConicQuad(new Circle3D(new Frame3D(new Point3D(), new Direction3D(0, 1, 1)), 2), aSph);
        Assert.True(aCirc.IsDone());
        Assert.True(aCirc.IsInQuadric());

        // A circle parallel to a plane
        var aPar = new IntAna_IntConicQuad(new Circle3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 2), aPln, 1e-12, 1e-8);
        Assert.True(aPar.IsDone());
        Assert.True(aPar.IsParallel());

        // A line through the centre of a sphere: two points at -R and +R
        var aLin = new IntAna_IntConicQuad(new Line3D(new Point3D(), new Direction3D(1, 0, 0)), aSph);
        Assert.Equal(2, aLin.NbPoints());
        Assert.Equal(4.0, Math.Abs(aLin.ParamOnConic(1) - aLin.ParamOnConic(2)), 1e-12);

        Assert.Throws<NotDoneException>(() => new IntAna_IntConicQuad().IsInQuadric());
    }

    [Fact]
    public void Quadric_Coefficients_AndSpecialPoints()
    {
        var aQ = new IntAna_Quadric(new Sphere(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2));
        Assert.Equal(2, aQ.SpecialPoints().Count);
        Assert.True(aQ.SpecialPoints()[0].IsEqual(new Point3D(1, 2, 1), 1e-15));
        Assert.True(aQ.SpecialPoints()[1].IsEqual(new Point3D(1, 2, 5), 1e-15));
        Assert.True(Residual(aQ, new Point3D(3, 2, 3)) < 1e-15);

        var aCone = new IntAna_Quadric(new Cone(new CoordinateSystem(), M_PI / 4, 1));
        Assert.Single(aCone.SpecialPoints());
        Assert.True(aCone.SpecialPoints()[0].IsEqual(new Point3D(0, 0, -1), 1e-12));
        Assert.Empty(new IntAna_Quadric(new Cylinder(new CoordinateSystem(), 1)).SpecialPoints());

        // The empty quadric is 1 = 0
        new IntAna_Quadric().Coefficients(out double xx, out _, out _, out _, out _, out _, out _, out _, out _, out double c);
        Assert.Equal(0.0, xx);
        Assert.Equal(1.0, c);

        // New coefficients in a local system: a cylinder of axis Z is x² + y² - R² in its own system
        var anAx = new CoordinateSystem(new Point3D(1, -2, 4), new Direction3D(1, 1, 1), new Direction3D(1, -1, 0));
        var aCyl = new IntAna_Quadric(new Cylinder(anAx, 3));
        aCyl.Coefficients(out double a, out double b, out double cc, out double d, out double e, out double f, out double g,
                          out double h, out double k, out double l);
        aCyl.NewCoefficients(ref a, ref b, ref cc, ref d, ref e, ref f, ref g, ref h, ref k, ref l, anAx);
        Assert.Equal(1.0, a, 1e-12);
        Assert.Equal(1.0, b, 1e-12);
        Assert.Equal(0.0, cc, 1e-12);
        Assert.Equal(0.0, d, 1e-12);
        Assert.Equal(0.0, e, 1e-12);
        Assert.Equal(0.0, f, 1e-12);
        Assert.Equal(0.0, g, 1e-12);
        Assert.Equal(0.0, h, 1e-12);
        Assert.Equal(0.0, k, 1e-12);
        Assert.Equal(-9.0, l, 1e-11);
    }
}
