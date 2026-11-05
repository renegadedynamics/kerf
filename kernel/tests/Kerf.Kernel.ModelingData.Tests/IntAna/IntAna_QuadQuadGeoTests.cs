// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// IntAna_QuadQuadGeo (OCCT has no GTests for it): the curves and points it returns lie on both quadrics, on random
/// and special configurations of every pair it handles, and the result types of known cases are OCCT's.
/// </summary>
public class IntAna_QuadQuadGeoTest
{
    private const double Tol = 1.0e-7;

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

    private static Point3D RandomPnt(Random theRandom, double theScale = 10)
        => new((theRandom.NextDouble() - 0.5) * 2 * theScale, (theRandom.NextDouble() - 0.5) * 2 * theScale,
               (theRandom.NextDouble() - 0.5) * 2 * theScale);

    private static CoordinateSystem RandomAx3(Random theRandom)
    {
        Direction3D aN = RandomDir(theRandom);
        Direction3D aX;
        do
        {
            aX = RandomDir(theRandom);
        }
        while (aN.IsParallel(aX, 0.1));
        return new CoordinateSystem(RandomPnt(theRandom), aN, aX);
    }

    private static CoordinateSystem Ax3(in Point3D theP, in Direction3D theN) => new(theP, theN);

    // Distances of a point to the quadrics. Cones are double: OCCT intersects the full quadric.
    private static double Dist(in Plane3D thePln, in Point3D theP) => thePln.DistanceTo(theP);

    private static double Dist(in Cylinder3D theCyl, in Point3D theP)
        => Math.Abs(new Line3D(theCyl.Axis).DistanceTo(theP) - theCyl.Radius);

    private static double Dist(in Sphere3D theSph, in Point3D theP)
        => Math.Abs(theSph.Location.DistanceTo(theP) - theSph.Radius);

    private static double Dist(in Cone3D theCone, in Point3D theP)
    {
        CoordinateSystem aPos = theCone.Position;
        var aV = new Vector3D(aPos.Location, theP);
        double aZ = aV.Dot(new Vector3D(aPos.Direction));
        double aX = aV.Dot(new Vector3D(aPos.XDirection));
        double aY = aV.Dot(new Vector3D(aPos.YDirection));
        double aR = Math.Sqrt(aX * aX + aY * aY);
        double aRz = Math.Abs(theCone.RefRadius + aZ * Math.Tan(theCone.SemiAngle));
        return Math.Abs(aR - aRz) * Math.Cos(theCone.SemiAngle);
    }

    private static double Dist(in Torus3D theTor, in Point3D theP)
    {
        CoordinateSystem aPos = theTor.Position;
        var aV = new Vector3D(aPos.Location, theP);
        double aZ = aV.Dot(new Vector3D(aPos.Direction));
        double aX = aV.Dot(new Vector3D(aPos.XDirection));
        double aY = aV.Dot(new Vector3D(aPos.YDirection));
        double aR = Math.Sqrt(aX * aX + aY * aY) - theTor.MajorRadius;
        return Math.Abs(Math.Sqrt(aR * aR + aZ * aZ) - theTor.MinorRadius);
    }

    /// <summary>Points sampled on every solution, whatever its type.</summary>
    private static List<Point3D> Samples(QuadQuadGeo theInter)
    {
        var aPnts = new List<Point3D>();
        int aNb = theInter.SolutionCount;
        ResultType aType = theInter.TypeInter();
        for (int i = 1; i <= aNb; i++)
        {
            for (int k = 0; k <= 16; k++)
            {
                double t = k / 16.0;
                switch (aType)
                {
                    case Point:
                        aPnts.Add(theInter.Point(i));
                        break;
                    case Line:
                        aPnts.Add(global::Kerf.Engine.Geometry.ElCLib.Value(-10 + 20 * t, theInter.Line(i)));
                        break;
                    case Circle:
                        aPnts.Add(global::Kerf.Engine.Geometry.ElCLib.Value(2 * Math.PI * t, theInter.Circle(i)));
                        break;
                    case Ellipse:
                        aPnts.Add(global::Kerf.Engine.Geometry.ElCLib.Value(2 * Math.PI * t, theInter.Ellipse(i)));
                        break;
                    case Parabola:
                        aPnts.Add(global::Kerf.Engine.Geometry.ElCLib.Value(-5 + 10 * t, theInter.Parabola(i)));
                        break;
                    case Hyperbola:
                        aPnts.Add(global::Kerf.Engine.Geometry.ElCLib.Value(-2 + 4 * t, theInter.Hyperbola(i)));
                        break;
                    case PointAndCircle:
                        if (i == 1)
                        {
                            aPnts.Add(theInter.Point(1));
                            aPnts.Add(global::Kerf.Engine.Geometry.ElCLib.Value(2 * Math.PI * t, theInter.Circle(1)));
                        }
                        break;
                }
            }
        }
        return aPnts;
    }

    private static void AssertOnBoth(QuadQuadGeo theInter, Func<Point3D, double> theD1, Func<Point3D, double> theD2,
                                     double theTol = 1.0e-7)
    {
        foreach (Point3D aP in Samples(theInter))
        {
            double aScale = Math.Max(1.0, aP.XYZ().Modulus());
            Assert.True(theD1(aP) <= theTol * aScale, $"{theInter.TypeInter()}: {theD1(aP)} from the first quadric");
            Assert.True(theD2(aP) <= theTol * aScale, $"{theInter.TypeInter()}: {theD2(aP)} from the second quadric");
        }
    }

    [Fact]
    public void PlanePlane_Random()
    {
        var aRandom = new Random(1401);
        for (int i = 0; i < 300; i++)
        {
            var aP1 = new Plane3D(RandomAx3(aRandom));
            var aP2 = new Plane3D(RandomAx3(aRandom));
            var anInter = new QuadQuadGeo(aP1, aP2, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
            Assert.True(anInter.IsDone);
            Assert.Equal(Line, anInter.TypeInter());
            Assert.Equal(1, anInter.SolutionCount);
            AssertOnBoth(anInter, p => Dist(aP1, p), p => Dist(aP2, p));
        }
    }

    [Fact]
    public void PlanePlane_ParallelAndSame()
    {
        var aP1 = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aP2 = new Plane3D(new Point3D(1, 2, 3), new Direction3D(0, 0, -1));
        Assert.Equal(Empty, new QuadQuadGeo(aP1, aP2, global::Kerf.Engine.Foundation.Precision.Angular(), Tol).TypeInter());
        var aP3 = new Plane3D(new Point3D(5, -2, 0), new Direction3D(0, 0, -1));
        Assert.Equal(Same, new QuadQuadGeo(aP1, aP3, global::Kerf.Engine.Foundation.Precision.Angular(), Tol).TypeInter());
    }

    [Fact]
    public void PlanePlane_SmallAngleRefinesOrigin()
    {
        // Below 2e-6 rad between the normals OCCT refines the origin through IntAna_IntConicQuad
        var aP1 = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aP2 = new Plane3D(new Point3D(3, 1, 1.0e-4), new Direction3D(1.0e-6, 0, 1));
        var anInter = new QuadQuadGeo(aP1, aP2, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
        Assert.True(anInter.IsDone);
        Assert.Equal(Line, anInter.TypeInter());
        Line3D aL = anInter.Line(1);
        Assert.True(Dist(aP1, aL.Location) < 1e-9);
        Assert.True(Dist(aP2, aL.Location) < 1e-9);
    }

    [Fact]
    public void PlaneCylinder_Random()
    {
        var aRandom = new Random(1402);
        int aNbEllipses = 0;
        for (int i = 0; i < 300; i++)
        {
            var aCyl = new Cylinder3D(RandomAx3(aRandom), 0.5 + 5 * aRandom.NextDouble());
            var aPln = new Plane3D(RandomAx3(aRandom));
            var anInter = new QuadQuadGeo(aPln, aCyl, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
            Assert.True(anInter.IsDone);
            ResultType aType = anInter.TypeInter();
            Assert.True(aType == Ellipse || aType == Circle, aType.ToString());
            aNbEllipses += aType == Ellipse ? 1 : 0;
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aCyl, p));
        }
        Assert.True(aNbEllipses > 250);
    }

    [Fact]
    public void PlaneCylinder_ParallelAndPerpendicular()
    {
        var aCyl = new Cylinder3D(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.0);
        // Plane parallel to the axis at distances 1, 2 and 3: 2 lines, 1 line, nothing
        double[] aDists = [1.0, 2.0, 3.0];
        int[] aNbs = [2, 1, 0];
        for (int k = 0; k < 3; k++)
        {
            var aPln = new Plane3D(new Point3D(1 + aDists[k], 2, 0), new Direction3D(1, 0, 0));
            var anInter = new QuadQuadGeo(aPln, aCyl, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
            Assert.True(anInter.IsDone);
            Assert.Equal(aNbs[k] == 0 ? Empty : Line, anInter.TypeInter());
            Assert.Equal(aNbs[k], anInter.SolutionCount);
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aCyl, p));
        }

        // Plane normal to the axis: a circle of the cylinder's radius
        var aPlnN = new Plane3D(new Point3D(0, 0, 7), new Direction3D(0, 0, -1));
        var anInterN = new QuadQuadGeo(aPlnN, aCyl, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
        Assert.Equal(Circle, anInterN.TypeInter());
        Assert.Equal(2.0, anInterN.Circle(1).Radius);
        Assert.True(anInterN.Circle(1).Location.IsEqual(new Point3D(1, 2, 7), 1e-12));
    }

    [Fact]
    public void PlaneSphere_Random()
    {
        var aRandom = new Random(1403);
        for (int i = 0; i < 300; i++)
        {
            var aSph = new Sphere3D(RandomAx3(aRandom), 0.5 + 10 * aRandom.NextDouble());
            var aPln = new Plane3D(RandomAx3(aRandom));
            var anInter = new QuadQuadGeo(aPln, aSph);
            Assert.True(anInter.IsDone);
            double aD = aPln.DistanceTo(aSph.Location);
            Assert.Equal(aD < aSph.Radius ? Circle : Empty, anInter.TypeInter());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aSph, p));
        }
    }

    [Fact]
    public void PlaneSphere_Special()
    {
        var aSph = new Sphere3D(new CoordinateSystem(new Point3D(1, 1, 1), new Direction3D(0, 1, 0)), 3.0);
        var aThrough = new QuadQuadGeo(new Plane3D(new Point3D(1, 1, 1), new Direction3D(1, 1, 0)), aSph);
        Assert.Equal(Circle, aThrough.TypeInter());
        Assert.Equal(3.0, aThrough.Circle(1).Radius, 1e-14);

        var aTangent = new QuadQuadGeo(new Plane3D(new Point3D(1, 1, 4), new Direction3D(0, 0, 1)), aSph);
        Assert.Equal(Point, aTangent.TypeInter());
        Assert.True(aTangent.Point(1).IsEqual(new Point3D(1, 1, 4), 1e-14));
    }

    [Fact]
    public void PlaneCone_Random()
    {
        var aRandom = new Random(1404);
        var aTypes = new HashSet<ResultType>();
        for (int i = 0; i < 400; i++)
        {
            var aCone = new Cone3D(RandomAx3(aRandom), 0.1 + 1.3 * aRandom.NextDouble(), 3 * aRandom.NextDouble());
            var aPln = new Plane3D(RandomAx3(aRandom));
            var anInter = new QuadQuadGeo(aPln, aCone, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
            if (!anInter.IsDone)
            {
                continue; // too large an ellipse or hyperbola: OCCT gives up
            }
            aTypes.Add(anInter.TypeInter());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aCone, p), 1.0e-6);
        }
        Assert.Contains(Ellipse, aTypes);
        Assert.Contains(Hyperbola, aTypes);
    }

    [Fact]
    public void PlaneCone_Special()
    {
        double aSemiAngle = Math.PI / 6;
        var aCone = new Cone3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), aSemiAngle, 1.0);
        double aZApex = -1.0 / Math.Tan(aSemiAngle);

        // Perpendicular plane: a circle
        var aPlnC = new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1));
        var aCirc = new QuadQuadGeo(aPlnC, aCone, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
        Assert.Equal(Circle, aCirc.TypeInter());
        Assert.Equal(1.0 + 2 * Math.Tan(aSemiAngle), aCirc.Circle(1).Radius, 1e-12);

        // Plane containing the axis direction, away from it: two hyperbola branches
        var aPlnH = new Plane3D(new Point3D(0.5, 0, 0), new Direction3D(1, 0, 0));
        var aHyp = new QuadQuadGeo(aPlnH, aCone, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
        Assert.Equal(Hyperbola, aHyp.TypeInter());
        Assert.Equal(2, aHyp.SolutionCount);
        AssertOnBoth(aHyp, p => Dist(aPlnH, p), p => Dist(aCone, p));

        // Plane parallel to a generatrix: a parabola
        Direction3D aN = new(Math.Cos(aSemiAngle), 0, -Math.Sin(aSemiAngle));
        var aPlnP = new Plane3D(new Point3D(0, 0, 1), aN);
        var aPar = new QuadQuadGeo(aPlnP, aCone, 1.0e-9, Tol);
        Assert.Equal(Parabola, aPar.TypeInter());
        AssertOnBoth(aPar, p => Dist(aPlnP, p), p => Dist(aCone, p));

        // Planes through the apex: a point, one line, two lines
        var anApex = new Point3D(0, 0, aZApex);
        Assert.Equal(Point,
                     new QuadQuadGeo(new Plane3D(anApex, new Direction3D(0, 0, 1)), aCone, global::Kerf.Engine.Foundation.Precision.Angular(), Tol).TypeInter());
        var aOne = new QuadQuadGeo(new Plane3D(anApex, aN), aCone, 1.0e-9, Tol);
        Assert.Equal(Line, aOne.TypeInter());
        Assert.Equal(1, aOne.SolutionCount);
        var aPlnA = new Plane3D(anApex, new Direction3D(1, 0, 0));
        var aTwo = new QuadQuadGeo(aPlnA, aCone, global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
        Assert.Equal(Line, aTwo.TypeInter());
        Assert.Equal(2, aTwo.SolutionCount);
        AssertOnBoth(aTwo, p => Dist(aPlnA, p), p => Dist(aCone, p));
    }

    [Fact]
    public void CylinderCylinder_ParallelAxes()
    {
        var aRandom = new Random(1405);
        for (int i = 0; i < 300; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            double aR1 = 0.5 + 3 * aRandom.NextDouble(), aR2 = 0.5 + 3 * aRandom.NextDouble();
            Vector3D anOffset = new Vector3D(anAx.XDirection) * (5 * aRandom.NextDouble())
                              + new Vector3D(anAx.YDirection) * (5 * aRandom.NextDouble())
                              + new Vector3D(anAx.Direction) * (aRandom.NextDouble() - 0.5);
            var aCyl1 = new Cylinder3D(anAx, aR1);
            var aCyl2 = new Cylinder3D(new CoordinateSystem(anAx.Location.Translated(anOffset), anAx.Direction), aR2);
            var anInter = new QuadQuadGeo(aCyl1, aCyl2, Tol);
            Assert.True(anInter.IsDone);
            ResultType aType = anInter.TypeInter();
            Assert.True(aType == Line || aType == Empty, aType.ToString());
            AssertOnBoth(anInter, p => Dist(aCyl1, p), p => Dist(aCyl2, p), 1.0e-6);
        }
    }

    [Fact]
    public void CylinderCylinder_Special()
    {
        var anAx = new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 1, 0));
        Assert.Equal(Same, new QuadQuadGeo(new Cylinder3D(anAx, 2), new Cylinder3D(anAx, 2), Tol).TypeInter());
        Assert.Equal(Empty, new QuadQuadGeo(new Cylinder3D(anAx, 2), new Cylinder3D(anAx, 3), Tol).TypeInter());

        // Equal radii, intersecting axes: two ellipses
        var aCyl1 = new Cylinder3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        var aCyl2 = new Cylinder3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 1)), 2);
        var anInter = new QuadQuadGeo(aCyl1, aCyl2, Tol);
        Assert.Equal(Ellipse, anInter.TypeInter());
        Assert.Equal(2, anInter.SolutionCount);
        AssertOnBoth(anInter, p => Dist(aCyl1, p), p => Dist(aCyl2, p));

        // Skew axes at the distance R1 + R2: a point
        var aCyl3 = new Cylinder3D(new CoordinateSystem(new Point3D(0, 5, 1), new Direction3D(1, 0, 0)), 3);
        var aPoint = new QuadQuadGeo(aCyl1, aCyl3, Tol);
        Assert.Equal(Point, aPoint.TypeInter());
        AssertOnBoth(aPoint, p => Dist(aCyl1, p), p => Dist(aCyl3, p));

        // Intersecting axes, different radii: no geometric solution
        var aCyl4 = new Cylinder3D(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)), 3);
        Assert.Equal(NoGeometricSolution, new QuadQuadGeo(aCyl1, aCyl4, Tol).TypeInter());
    }

    [Fact]
    public void CylinderSphereAndCone_Coaxial()
    {
        var aRandom = new Random(1406);
        for (int i = 0; i < 200; i++)
        {
            CoordinateSystem anAx = RandomAx3(aRandom);
            var aCyl = new Cylinder3D(anAx, 0.5 + 3 * aRandom.NextDouble());
            var aCenter = anAx.Location.Translated(new Vector3D(anAx.Direction) * (10 * aRandom.NextDouble() - 5));
            var aSph = new Sphere3D(new CoordinateSystem(aCenter, RandomDir(aRandom)), 0.5 + 5 * aRandom.NextDouble());
            var anInter = new QuadQuadGeo(aCyl, aSph, Tol);
            Assert.True(anInter.IsDone);
            ResultType aType = anInter.TypeInter();
            Assert.True(aType == Circle || aType == Empty || aType == NoGeometricSolution,
                        aType.ToString());
            if (aType != NoGeometricSolution)
            {
                AssertOnBoth(anInter, p => Dist(aCyl, p), p => Dist(aSph, p), 1.0e-6);
            }

            var aCone = new Cone3D(new CoordinateSystem(aCenter, anAx.Direction), 0.1 + 1.3 * aRandom.NextDouble(),
                                    2 * aRandom.NextDouble());
            var anInterC = new QuadQuadGeo(aCyl, aCone, Tol);
            Assert.True(anInterC.IsDone);
            if (anInterC.TypeInter() == Circle)
            {
                Assert.Equal(2, anInterC.SolutionCount);
                AssertOnBoth(anInterC, p => Dist(aCyl, p), p => Dist(aCone, p), 1.0e-6);
            }
        }

        // Exactly coaxial: circles
        var aCylZ = new Cylinder3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3);
        var aSphZ = new Sphere3D(new CoordinateSystem(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 5);
        var aCS = new QuadQuadGeo(aCylZ, aSphZ, Tol);
        Assert.Equal(Circle, aCS.TypeInter());
        Assert.Equal(2, aCS.SolutionCount);
        Assert.True(aCS.Circle(1).Location.IsEqual(new Point3D(0, 0, 6), 1e-12));
        Assert.True(aCS.Circle(2).Location.IsEqual(new Point3D(0, 0, -2), 1e-12));
        var aConeZ = new Cone3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 4, 1);
        var aCC = new QuadQuadGeo(aCylZ, aConeZ, Tol);
        Assert.Equal(Circle, aCC.TypeInter());
        AssertOnBoth(aCC, p => Dist(aCylZ, p), p => Dist(aConeZ, p));

        // Off-axis sphere
        var aSphOff = new Sphere3D(new CoordinateSystem(new Point3D(1, 0, 2), new Direction3D(0, 0, 1)), 5);
        Assert.Equal(NoGeometricSolution, new QuadQuadGeo(aCylZ, aSphOff, Tol).TypeInter());
    }

    [Fact]
    public void SphereSphere_Random()
    {
        var aRandom = new Random(1407);
        for (int i = 0; i < 300; i++)
        {
            var aSph1 = new Sphere3D(RandomAx3(aRandom), 0.5 + 8 * aRandom.NextDouble());
            var aSph2 = new Sphere3D(RandomAx3(aRandom), 0.5 + 8 * aRandom.NextDouble());
            var anInter = new QuadQuadGeo(aSph1, aSph2, Tol);
            Assert.True(anInter.IsDone);
            double aD = aSph1.Location.DistanceTo(aSph2.Location);
            bool anIntersects = aD < aSph1.Radius + aSph2.Radius && aD > Math.Abs(aSph1.Radius - aSph2.Radius);
            Assert.Equal(anIntersects ? Circle : Empty, anInter.TypeInter());
            AssertOnBoth(anInter, p => Dist(aSph1, p), p => Dist(aSph2, p), 1.0e-6);
        }

        var aS = new Sphere3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        Assert.Equal(Same, new QuadQuadGeo(aS, aS, Tol).TypeInter());
        var aTangent = new QuadQuadGeo(aS, new Sphere3D(new CoordinateSystem(new Point3D(3, 0, 0), new Direction3D(0, 0, 1)), 1), Tol);
        Assert.Equal(Point, aTangent.TypeInter());
        Assert.True(aTangent.Point(1).IsEqual(new Point3D(2, 0, 0), 1e-12));
    }

    [Fact]
    public void SphereCone_Coaxial()
    {
        var aCone = new Cone3D(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 6, 1);
        for (int k = 0; k < 20; k++)
        {
            var aSph = new Sphere3D(new CoordinateSystem(new Point3D(0, 0, -3 + 0.5 * k), new Direction3D(0, 0, 1)), 1 + 0.3 * k);
            var anInter = new QuadQuadGeo(aSph, aCone, Tol);
            Assert.True(anInter.IsDone);
            ResultType aType = anInter.TypeInter();
            Assert.True(aType == Circle || aType == Empty || aType == PointAndCircle,
                        aType.ToString());
            AssertOnBoth(anInter, p => Dist(aSph, p), p => Dist(aCone, p), 1.0e-6);
        }

        // Sphere centered at the apex: the two circles at the same distance
        Point3D anApex = aCone.Apex;
        var aSphA = new Sphere3D(new CoordinateSystem(anApex, new Direction3D(0, 0, 1)), 2);
        var anInterA = new QuadQuadGeo(aSphA, aCone, Tol);
        Assert.Equal(Circle, anInterA.TypeInter());
        Assert.Equal(2, anInterA.SolutionCount);
        Assert.Equal(1.0, anInterA.Circle(1).Radius, 1e-12);
        AssertOnBoth(anInterA, p => Dist(aSphA, p), p => Dist(aCone, p));
    }

    [Fact]
    public void ConeCone_Cases()
    {
        var anAx = new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aCone1 = new Cone3D(anAx, Math.PI / 6, 1);

        // Same axis, different angles: two circles
        var aCone2 = new Cone3D(new CoordinateSystem(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), Math.PI / 4, 1);
        var anInter = new QuadQuadGeo(aCone1, aCone2, Tol);
        Assert.Equal(Circle, anInter.TypeInter());
        AssertOnBoth(anInter, p => Dist(aCone1, p), p => Dist(aCone2, p));

        // Same cone: Same
        Assert.Equal(Same, new QuadQuadGeo(aCone1, aCone1, Tol).TypeInter());

        // Common apex, intersecting axes: lines through the apex
        Point3D anApex = aCone1.Apex;
        var aCone3 = new Cone3D(new CoordinateSystem(anApex, new Direction3D(Math.Sin(0.3), 0, Math.Cos(0.3))), Math.PI / 6, 0);
        var aLines = new QuadQuadGeo(aCone1, aCone3, Tol);
        Assert.Equal(Line, aLines.TypeInter());
        Assert.Equal(2, aLines.SolutionCount);
        AssertOnBoth(aLines, p => Dist(aCone1, p), p => Dist(aCone3, p), 1.0e-6);

        // Parallel axes, equal angles: a conic in a plane
        var aCone4 = new Cone3D(new CoordinateSystem(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 6, 1);
        var aPar = new QuadQuadGeo(aCone1, aCone4, Tol);
        Assert.True(aPar.IsDone);
        Assert.Equal(Hyperbola, aPar.TypeInter());
        AssertOnBoth(aPar, p => Dist(aCone1, p), p => Dist(aCone4, p), 1.0e-6);
    }

    [Fact]
    public void PlaneTorus()
    {
        var aTor = new Torus3D(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 5, 2);
        for (int k = 0; k <= 10; k++)
        {
            var aPln = new Plane3D(new Point3D(0, 0, 3 - 2.5 + 0.5 * k), new Direction3D(0, 0, 1));
            var anInter = new QuadQuadGeo(aPln, aTor, Tol);
            Assert.True(anInter.IsDone);
            ResultType aType = anInter.TypeInter();
            Assert.True(aType == Circle || aType == Empty, aType.ToString());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aTor, p));
        }

        var aPlnAx = new Plane3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 0));
        var anAxial = new QuadQuadGeo(aPlnAx, aTor, Tol);
        Assert.Equal(Circle, anAxial.TypeInter());
        Assert.Equal(2, anAxial.SolutionCount);
        AssertOnBoth(anAxial, p => Dist(aPlnAx, p), p => Dist(aTor, p));

        var aPlnSkew = new Plane3D(new Point3D(1, 2, 3), new Direction3D(1, 0, 1));
        Assert.Equal(NoGeometricSolution, new QuadQuadGeo(aPlnSkew, aTor, Tol).TypeInter());
    }

    [Fact]
    public void QuadricsAndTorus_Coaxial()
    {
        var anAx = new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aTor = new Torus3D(anAx, 5, 2);
        for (int k = 0; k <= 10; k++)
        {
            var aCyl = new Cylinder3D(anAx, 2.5 + 0.6 * k);
            var anInter = new QuadQuadGeo(aCyl, aTor, Tol);
            Assert.True(anInter.IsDone);
            AssertOnBoth(anInter, p => Dist(aCyl, p), p => Dist(aTor, p));

            var aSph = new Sphere3D(new CoordinateSystem(new Point3D(0, 0, -2 + 0.4 * k), new Direction3D(0, 0, 1)), 3.5 + 0.5 * k);
            var anInterS = new QuadQuadGeo(aSph, aTor, Tol);
            Assert.True(anInterS.IsDone);
            AssertOnBoth(anInterS, p => Dist(aSph, p), p => Dist(aTor, p), 1.0e-6);

            var aCone = new Cone3D(new CoordinateSystem(new Point3D(0, 0, -1 + 0.2 * k), new Direction3D(0, 0, 1)), 0.2 + 0.1 * k, 3);
            var anInterC = new QuadQuadGeo(aCone, aTor, Tol);
            Assert.True(anInterC.IsDone);
            AssertOnBoth(anInterC, p => Dist(aCone, p), p => Dist(aTor, p), 1.0e-6);

            var aTor2 = new Torus3D(new CoordinateSystem(new Point3D(0, 0, -1 + 0.2 * k), new Direction3D(0, 0, 1)), 4 + 0.3 * k, 1.5);
            var anInterT = new QuadQuadGeo(aTor, aTor2, Tol);
            Assert.True(anInterT.IsDone);
            AssertOnBoth(anInterT, p => Dist(aTor, p), p => Dist(aTor2, p), 1.0e-6);
        }

        Assert.Equal(Same, new QuadQuadGeo(aTor, aTor, Tol).TypeInter());
        var aCylOff = new Cylinder3D(new CoordinateSystem(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 4);
        Assert.Equal(NoGeometricSolution, new QuadQuadGeo(aCylOff, aTor, Tol).TypeInter());
    }

    [Fact]
    public void Accessors_CheckStateAndRange()
    {
        var anEmpty = new QuadQuadGeo();
        Assert.False(anEmpty.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anEmpty.TypeInter());
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anEmpty.SolutionCount);

        var aPlnPln = new QuadQuadGeo(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                             new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), global::Kerf.Engine.Foundation.Precision.Angular(), Tol);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aPlnPln.Line(2));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aPlnPln.Circle(1));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aPlnPln.Ellipse(1));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aPlnPln.Hyperbola(1));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aPlnPln.Parabola(1));
        Assert.False(aPlnPln.HasCommonGen);
        Assert.Equal(new Point3D(0, 0, 0), aPlnPln.PChar());
    }
}
