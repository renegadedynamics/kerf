// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Property tests of IntAna_IntQuadQuad and IntAna_Curve on random cylinder/quadric and cone/quadric pairs: the
/// points of every curve lie on both surfaces, D1u is tangent to them, and FindParameter finds the points again.
/// There is no oracle: build123d.NET doesn't bind IntAna or IntPatch.
/// </summary>
public class IntAna_IntQuadQuadPropertyTests
{
    private const double Tol = 1.0e-6;

    private static Direction3D RandomDir(Random theRnd)
    {
        while (true)
        {
            double x = theRnd.NextDouble() * 2 - 1, y = theRnd.NextDouble() * 2 - 1, z = theRnd.NextDouble() * 2 - 1;
            double n = Math.Sqrt(x * x + y * y + z * z);
            if (n > 0.1 && n < 1.0)
            {
                return new Direction3D(x / n, y / n, z / n);
            }
        }
    }

    private static Point3D RandomPnt(Random theRnd, double theSize)
        => new((theRnd.NextDouble() * 2 - 1) * theSize, (theRnd.NextDouble() * 2 - 1) * theSize,
               (theRnd.NextDouble() * 2 - 1) * theSize);

    private static CoordinateSystem RandomAx3(Random theRnd, double theSize)
    {
        Direction3D aN = RandomDir(theRnd);
        Direction3D aX;
        do
        {
            aX = RandomDir(theRnd);
        } while (aX.IsParallel(aN, 0.1));
        return new CoordinateSystem(RandomPnt(theRnd, theSize), aN, aN.Crossed(aX).Crossed(aN));
    }

    /// <summary>A random quadric (sphere, cylinder, cone or, for cones only, plane) and the distance to it.</summary>
    private static (Quadric Quad, Func<Point3D, double> Dist, string Name) RandomQuadric(Random theRnd,
                                                                                              bool theWithPlane)
    {
        int aKind = theRnd.Next(theWithPlane ? 4 : 3);
        CoordinateSystem anAx = RandomAx3(theRnd, 2.0);
        switch (aKind)
        {
            case 0:
                {
                    Sphere aS = new(anAx, 1.0 + 3.0 * theRnd.NextDouble());
                    return (new Quadric(aS), P => Math.Abs(P.DistanceTo(aS.Location) - aS.Radius), "sphere");
                }
            case 1:
                {
                    Cylinder aC = new(anAx, 0.5 + 2.5 * theRnd.NextDouble());
                    return (new Quadric(aC), P => Math.Abs(new Line3D(aC.Axis).DistanceTo(P) - aC.Radius),
                            "cylinder");
                }
            case 2:
                {
                    Cone aK = new(anAx, 0.2 + 1.0 * theRnd.NextDouble(), 2.0 * theRnd.NextDouble());
                    return (new Quadric(aK), P => ConeDistance(aK, P), "cone");
                }
            default:
                {
                    Plane3D aPl = new(anAx);
                    return (new Quadric(aPl), P => aPl.DistanceTo(P), "plane");
                }
        }
    }

    // The distance from P to its ElSLib round trip on the cone: zero on the cone, of the order of the distance
    // to the cone near it
    private static double ConeDistance(in Cone theCone, in Point3D theP)
    {
        global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, theP, out double aU, out double aV);
        return theP.DistanceTo(global::Kerf.Engine.Geometry.ElSLib.Value(aU, aV, theCone));
    }

    private static void CheckCurves(IntQuadQuad theInt, Func<Point3D, double> theDist1,
                                    Func<Point3D, double> theDist2, Func<Point3D, Direction3D> theNormal1,
                                    ref int theNbPoints, ref int theNbFound, string theCase)
    {
        for (int i = 1; i <= theInt.CurveCount; i++)
        {
            Curve aC = theInt.Curve(i);
            aC.Domain(out double aFirst, out double aLast);
            Assert.True(aLast > aFirst, theCase);
            const int aNbSamples = 24;
            for (int k = 1; k < aNbSamples; k++)
            {
                double aT = aFirst + (aLast - aFirst) * k / aNbSamples;
                Point3D aP = aC.Value(aT);
                if (aP.XYZ().Modulus() > 1.0e3)
                {
                    continue; // an infinite branch of a cone intersection, clamped by IntAna_Curve
                }
                theNbPoints++;
                Assert.True(theDist1(aP) < Tol, $"{theCase}: curve {i} at {aT} is {theDist1(aP)} from the first surface");
                Assert.True(theDist2(aP) < Tol, $"{theCase}: curve {i} at {aT} is {theDist2(aP)} from the quadric");

                if (aC.D1u(aT, out Point3D aPd, out Vector3D aV))
                {
                    Assert.Equal(aP, aPd);
                    double aMag = aV.Magnitude;
                    if (aMag > 1.0e-3)
                    {
                        // A forward difference of step 1e-6 of the domain: tangent to first order
                        double aCos = Math.Abs(new Vector3D(theNormal1(aP)).Dot(aV)) / aMag;
                        Assert.True(aCos < 1.0e-3, $"{theCase}: D1u of curve {i} at {aT} is not tangent ({aCos})");
                    }
                }

                NCollection_List<double> aParams = new();
                aC.FindParameter(aP, aParams);
                double aPrev = double.NegativeInfinity;
                foreach (double aPar in aParams)
                {
                    Assert.True(aPar >= aPrev, theCase);
                    aPrev = aPar;
                    Assert.True(aC.Value(aPar).DistanceTo(aP) < 1.0e-4, $"{theCase}: FindParameter gave {aPar} for {aT}");
                }
                if (!aParams.IsEmpty())
                {
                    theNbFound++;
                }
            }
        }
    }

    [Fact]
    public void CylinderQuadric_CurvesLieOnBothSurfaces()
    {
        Random aRnd = new(20261029);
        int aNbDone = 0, aNbCurves = 0, aNbPoints = 0, aNbFound = 0;
        for (int anIter = 0; anIter < 400; anIter++)
        {
            Cylinder aCyl = new(RandomAx3(aRnd, 1.0), 0.5 + 2.5 * aRnd.NextDouble());
            (Quadric aQuad, Func<Point3D, double> aDist, string aName) = RandomQuadric(aRnd, false);
            IntQuadQuad anInt = new(aCyl, aQuad, 1.0e-7);
            if (!anInt.IsDone)
            {
                continue;
            }
            aNbDone++;
            Assert.False(anInt.IdenticalElements());
            Assert.Equal(0, anInt.PntCount);
            aNbCurves += anInt.CurveCount;
            Line3D anAxis = new(aCyl.Axis);
            CheckCurves(anInt, P => Math.Abs(anAxis.DistanceTo(P) - aCyl.Radius), aDist,
                        P => CylinderNormal(aCyl, P), ref aNbPoints, ref aNbFound, $"cylinder/{aName} #{anIter}");
            // OCCT links curves only after a cone intersection
            for (int i = 1; i <= anInt.CurveCount; i++)
            {
                Assert.False(anInt.HasNextCurve(i));
                Assert.False(anInt.HasPreviousCurve(i));
            }
        }
        Assert.True(aNbDone > 300, $"{aNbDone} done");
        Assert.True(aNbCurves > 200, $"{aNbCurves} curves");
        Assert.True(aNbFound > 0.95 * aNbPoints, $"FindParameter found {aNbFound} of {aNbPoints}");
    }

    [Fact]
    public void ConeQuadric_CurvesLieOnBothSurfaces()
    {
        Random aRnd = new(1029);
        int aNbDone = 0, aNbCurves = 0, aNbPoints = 0, aNbFound = 0;
        for (int anIter = 0; anIter < 400; anIter++)
        {
            Cone aCone = new(RandomAx3(aRnd, 1.0), 0.2 + 1.0 * aRnd.NextDouble(), 2.0 * aRnd.NextDouble());
            (Quadric aQuad, Func<Point3D, double> aDist, string aName) = RandomQuadric(aRnd, true);
            IntQuadQuad anInt = new(aCone, aQuad, 1.0e-7);
            if (!anInt.IsDone)
            {
                continue;
            }
            aNbDone++;
            aNbCurves += anInt.CurveCount;
            CheckCurves(anInt, P => ConeDistance(aCone, P), aDist, P => ConeNormal(aCone, P), ref aNbPoints,
                        ref aNbFound, $"cone/{aName} #{anIter}");

            // Linked curves are linked back
            for (int i = 1; i <= anInt.CurveCount; i++)
            {
                if (anInt.HasNextCurve(i))
                {
                    int j = anInt.NextCurve(i, out bool anOpposite);
                    Assert.NotEqual(i, j);
                    Assert.True(anOpposite ? anInt.HasNextCurve(j) : anInt.HasPreviousCurve(j));
                    anInt.Curve(i).Domain(out _, out double aLastI);
                    anInt.Curve(j).Domain(out double aFirstJ, out double aLastJ);
                    Point3D aPi = anInt.Curve(i).Value(aLastI);
                    Point3D aPj = anInt.Curve(j).Value(anOpposite ? aLastJ : aFirstJ);
                    Assert.True(aPi.DistanceTo(aPj) < 1.0e-9);
                }
            }
        }
        Assert.True(aNbDone > 300, $"{aNbDone} done");
        Assert.True(aNbCurves > 200, $"{aNbCurves} curves");
        Assert.True(aNbFound > 0.95 * aNbPoints, $"FindParameter found {aNbFound} of {aNbPoints}");
    }

    private static Direction3D CylinderNormal(in Cylinder theCyl, in Point3D theP)
    {
        global::Kerf.Engine.Geometry.ElSLib.Parameters(theCyl, theP, out double aU, out double aV);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(aU, aV, theCyl.Position, theCyl.Radius, out _, out Vector3D aDu, out Vector3D aDv);
        return new Direction3D(aDu.Crossed(aDv));
    }

    private static Direction3D ConeNormal(in Cone theCone, in Point3D theP)
    {
        global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, theP, out double aU, out double aV);
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(aU, aV, theCone.Position, theCone.RefRadius, theCone.SemiAngle, out _, out Vector3D aDu,
                      out Vector3D aDv);
        return new Direction3D(aDu.Crossed(aDv));
    }

    [Fact]
    public void CylinderPlane_NotDone()
    {
        // The quadric's Z² coefficient in the cylinder's frame is zero: OCCT gives up (QuadQuadGeo handles it)
        Cylinder aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), Direction3D.D.Z, Direction3D.D.X), 2.0);
        IntQuadQuad anInt = new(aCyl, new Quadric(new Plane3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 1))),
                                       1.0e-7);
        Assert.False(anInt.IsDone);
        EngineAssert.ThrowsAny<NotDoneException>(() => anInt.CurveCount);
    }

    [Fact]
    public void Curve_EmptyAndTwins()
    {
        Curve anEmpty = new();
        Assert.False(anEmpty.IsOpen);
        EngineAssert.ThrowsAny<DomainErrorException>(() => anEmpty.Domain(out _, out _));
        Assert.Equal(new Point3D(0, 0, 0), anEmpty.Value(0.0));
        Assert.NotEqual(default, anEmpty);

        Cylinder aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), Direction3D.D.Z, Direction3D.D.X), 3.0);
        // The sphere of radius 5 at the origin: z = ±4 on the cylinder of radius 3
        Curve aC = anEmpty.WithCylinderQuadValues(aCyl, 1, 1, 1, 0, 0, 0, 0, 0, 0, -25, 1e-8, 0.0, 2 * M_PI,
                                                         false, true);
        Assert.True(aC.IsOpen);
        aC.Domain(out double aFirst, out double aLast);
        Assert.Equal(0.0, aFirst);
        Assert.Equal(2 * M_PI, aLast);
        Point3D aP = aC.Value(1.0);
        Assert.Equal(4.0, aP.Z, 1e-12);
        Assert.Equal(3.0 * Math.Cos(1.0), aP.X, 1e-12);
        EngineAssert.ThrowsAny<DomainErrorException>(() => aC.Value(7.0));

        Curve aTrimmed = aC.WithDomain(1.0, 2.0).WithIsFirstOpen(true).WithIsLastOpen(true);
        aTrimmed.Domain(out aFirst, out aLast);
        Assert.Equal((1.0, 2.0), (aFirst, aLast));
        Assert.True(aTrimmed.IsFirstOpen && aTrimmed.IsLastOpen);
        Assert.False(aC.IsFirstOpen); // the twins copy
        Assert.NotEqual(aC, aTrimmed);
        Assert.Equal(aC, aC.WithDomain(0.0, 2 * M_PI));
        EngineAssert.ThrowsAny<DomainErrorException>(() => aC.WithDomain(2.0, 1.0));

        NCollection_List<double> aParams = new();
        aC.FindParameter(aP, aParams);
        Assert.Equal(1, aParams.Extent());
        Assert.Equal(1.0, aParams.First(), 1e-12);
    }

    [Fact]
    public void CylinderSphere_Viviani_ReachesThePoles()
    {
        // Viviani's curve: the cylinder touches the sphere's axis, so the sphere's poles (its special points) are
        // intersection points, where the discriminant is zero. AddSpecialPoints keeps them in the curves' domains.
        Cylinder aCyl = new(new CoordinateSystem(new Point3D(1, 0, 0), Direction3D.D.Z, Direction3D.D.X), 1.0);
        Sphere aSph = new(new CoordinateSystem(new Point3D(0, 0, 0), Direction3D.D.Z, Direction3D.D.X), 2.0);
        IntQuadQuad anInt = new(aCyl, new Quadric(aSph), 1.0e-7);
        Assert.True(anInt.IsDone);
        Assert.True(anInt.CurveCount > 0);
        double aMinNorth = double.MaxValue, aMinSouth = double.MaxValue;
        for (int i = 1; i <= anInt.CurveCount; i++)
        {
            Curve aC = anInt.Curve(i);
            aC.Domain(out double aFirst, out double aLast);
            for (int k = 0; k <= 100; k++)
            {
                Point3D aP = aC.Value(aFirst + (aLast - aFirst) * k / 100);
                Assert.Equal(2.0, aP.DistanceTo(aSph.Location), 1e-6);
                Assert.Equal(1.0, new Line3D(aCyl.Axis).DistanceTo(aP), 1e-6);
                aMinNorth = Math.Min(aMinNorth, aP.DistanceTo(new Point3D(0, 0, 2)));
                aMinSouth = Math.Min(aMinSouth, aP.DistanceTo(new Point3D(0, 0, -2)));
            }
        }
        Assert.True(aMinNorth < 1e-6, $"{aMinNorth}");
        Assert.True(aMinSouth < 1e-6, $"{aMinSouth}");
    }
}
