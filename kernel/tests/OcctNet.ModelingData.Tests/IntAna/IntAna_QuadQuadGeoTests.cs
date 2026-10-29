// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntAna_ResultType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// IntAna_QuadQuadGeo (OCCT has no GTests for it): the curves and points it returns lie on both quadrics, on random
/// and special configurations of every pair it handles, and the result types of known cases are OCCT's.
/// </summary>
public class IntAna_QuadQuadGeoTest
{
    private const double Tol = 1.0e-7;

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

    private static gp_Pnt RandomPnt(Random theRandom, double theScale = 10)
        => new((theRandom.NextDouble() - 0.5) * 2 * theScale, (theRandom.NextDouble() - 0.5) * 2 * theScale,
               (theRandom.NextDouble() - 0.5) * 2 * theScale);

    private static gp_Ax3 RandomAx3(Random theRandom)
    {
        gp_Dir aN = RandomDir(theRandom);
        gp_Dir aX;
        do
        {
            aX = RandomDir(theRandom);
        }
        while (aN.IsParallel(aX, 0.1));
        return new gp_Ax3(RandomPnt(theRandom), aN, aX);
    }

    private static gp_Ax3 Ax3(in gp_Pnt theP, in gp_Dir theN) => new(theP, theN);

    // Distances of a point to the quadrics. Cones are double: OCCT intersects the full quadric.
    private static double Dist(in gp_Pln thePln, in gp_Pnt theP) => thePln.Distance(theP);

    private static double Dist(in gp_Cylinder theCyl, in gp_Pnt theP)
        => Math.Abs(new gp_Lin(theCyl.Axis()).Distance(theP) - theCyl.Radius());

    private static double Dist(in gp_Sphere theSph, in gp_Pnt theP)
        => Math.Abs(theSph.Location().Distance(theP) - theSph.Radius());

    private static double Dist(in gp_Cone theCone, in gp_Pnt theP)
    {
        gp_Ax3 aPos = theCone.Position();
        var aV = new gp_Vec(aPos.Location(), theP);
        double aZ = aV.Dot(new gp_Vec(aPos.Direction()));
        double aX = aV.Dot(new gp_Vec(aPos.XDirection()));
        double aY = aV.Dot(new gp_Vec(aPos.YDirection()));
        double aR = Math.Sqrt(aX * aX + aY * aY);
        double aRz = Math.Abs(theCone.RefRadius() + aZ * Math.Tan(theCone.SemiAngle()));
        return Math.Abs(aR - aRz) * Math.Cos(theCone.SemiAngle());
    }

    private static double Dist(in gp_Torus theTor, in gp_Pnt theP)
    {
        gp_Ax3 aPos = theTor.Position();
        var aV = new gp_Vec(aPos.Location(), theP);
        double aZ = aV.Dot(new gp_Vec(aPos.Direction()));
        double aX = aV.Dot(new gp_Vec(aPos.XDirection()));
        double aY = aV.Dot(new gp_Vec(aPos.YDirection()));
        double aR = Math.Sqrt(aX * aX + aY * aY) - theTor.MajorRadius();
        return Math.Abs(Math.Sqrt(aR * aR + aZ * aZ) - theTor.MinorRadius());
    }

    /// <summary>Points sampled on every solution, whatever its type.</summary>
    private static List<gp_Pnt> Samples(IntAna_QuadQuadGeo theInter)
    {
        var aPnts = new List<gp_Pnt>();
        int aNb = theInter.NbSolutions();
        IntAna_ResultType aType = theInter.TypeInter();
        for (int i = 1; i <= aNb; i++)
        {
            for (int k = 0; k <= 16; k++)
            {
                double t = k / 16.0;
                switch (aType)
                {
                    case IntAna_Point:
                        aPnts.Add(theInter.Point(i));
                        break;
                    case IntAna_Line:
                        aPnts.Add(ElCLib.Value(-10 + 20 * t, theInter.Line(i)));
                        break;
                    case IntAna_Circle:
                        aPnts.Add(ElCLib.Value(2 * Math.PI * t, theInter.Circle(i)));
                        break;
                    case IntAna_Ellipse:
                        aPnts.Add(ElCLib.Value(2 * Math.PI * t, theInter.Ellipse(i)));
                        break;
                    case IntAna_Parabola:
                        aPnts.Add(ElCLib.Value(-5 + 10 * t, theInter.Parabola(i)));
                        break;
                    case IntAna_Hyperbola:
                        aPnts.Add(ElCLib.Value(-2 + 4 * t, theInter.Hyperbola(i)));
                        break;
                    case IntAna_PointAndCircle:
                        if (i == 1)
                        {
                            aPnts.Add(theInter.Point(1));
                            aPnts.Add(ElCLib.Value(2 * Math.PI * t, theInter.Circle(1)));
                        }
                        break;
                }
            }
        }
        return aPnts;
    }

    private static void AssertOnBoth(IntAna_QuadQuadGeo theInter, Func<gp_Pnt, double> theD1, Func<gp_Pnt, double> theD2,
                                     double theTol = 1.0e-7)
    {
        foreach (gp_Pnt aP in Samples(theInter))
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
            var aP1 = new gp_Pln(RandomAx3(aRandom));
            var aP2 = new gp_Pln(RandomAx3(aRandom));
            var anInter = new IntAna_QuadQuadGeo(aP1, aP2, Precision.Angular(), Tol);
            Assert.True(anInter.IsDone());
            Assert.Equal(IntAna_Line, anInter.TypeInter());
            Assert.Equal(1, anInter.NbSolutions());
            AssertOnBoth(anInter, p => Dist(aP1, p), p => Dist(aP2, p));
        }
    }

    [Fact]
    public void PlanePlane_ParallelAndSame()
    {
        var aP1 = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aP2 = new gp_Pln(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, -1));
        Assert.Equal(IntAna_Empty, new IntAna_QuadQuadGeo(aP1, aP2, Precision.Angular(), Tol).TypeInter());
        var aP3 = new gp_Pln(new gp_Pnt(5, -2, 0), new gp_Dir(0, 0, -1));
        Assert.Equal(IntAna_Same, new IntAna_QuadQuadGeo(aP1, aP3, Precision.Angular(), Tol).TypeInter());
    }

    [Fact]
    public void PlanePlane_SmallAngleRefinesOrigin()
    {
        // Below 2e-6 rad between the normals OCCT refines the origin through IntAna_IntConicQuad
        var aP1 = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aP2 = new gp_Pln(new gp_Pnt(3, 1, 1.0e-4), new gp_Dir(1.0e-6, 0, 1));
        var anInter = new IntAna_QuadQuadGeo(aP1, aP2, Precision.Angular(), Tol);
        Assert.True(anInter.IsDone());
        Assert.Equal(IntAna_Line, anInter.TypeInter());
        gp_Lin aL = anInter.Line(1);
        Assert.True(Dist(aP1, aL.Location()) < 1e-9);
        Assert.True(Dist(aP2, aL.Location()) < 1e-9);
    }

    [Fact]
    public void PlaneCylinder_Random()
    {
        var aRandom = new Random(1402);
        int aNbEllipses = 0;
        for (int i = 0; i < 300; i++)
        {
            var aCyl = new gp_Cylinder(RandomAx3(aRandom), 0.5 + 5 * aRandom.NextDouble());
            var aPln = new gp_Pln(RandomAx3(aRandom));
            var anInter = new IntAna_QuadQuadGeo(aPln, aCyl, Precision.Angular(), Tol);
            Assert.True(anInter.IsDone());
            IntAna_ResultType aType = anInter.TypeInter();
            Assert.True(aType == IntAna_Ellipse || aType == IntAna_Circle, aType.ToString());
            aNbEllipses += aType == IntAna_Ellipse ? 1 : 0;
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aCyl, p));
        }
        Assert.True(aNbEllipses > 250);
    }

    [Fact]
    public void PlaneCylinder_ParallelAndPerpendicular()
    {
        var aCyl = new gp_Cylinder(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 2.0);
        // Plane parallel to the axis at distances 1, 2 and 3: 2 lines, 1 line, nothing
        double[] aDists = [1.0, 2.0, 3.0];
        int[] aNbs = [2, 1, 0];
        for (int k = 0; k < 3; k++)
        {
            var aPln = new gp_Pln(new gp_Pnt(1 + aDists[k], 2, 0), new gp_Dir(1, 0, 0));
            var anInter = new IntAna_QuadQuadGeo(aPln, aCyl, Precision.Angular(), Tol);
            Assert.True(anInter.IsDone());
            Assert.Equal(aNbs[k] == 0 ? IntAna_Empty : IntAna_Line, anInter.TypeInter());
            Assert.Equal(aNbs[k], anInter.NbSolutions());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aCyl, p));
        }

        // Plane normal to the axis: a circle of the cylinder's radius
        var aPlnN = new gp_Pln(new gp_Pnt(0, 0, 7), new gp_Dir(0, 0, -1));
        var anInterN = new IntAna_QuadQuadGeo(aPlnN, aCyl, Precision.Angular(), Tol);
        Assert.Equal(IntAna_Circle, anInterN.TypeInter());
        Assert.Equal(2.0, anInterN.Circle(1).Radius());
        Assert.True(anInterN.Circle(1).Location().IsEqual(new gp_Pnt(1, 2, 7), 1e-12));
    }

    [Fact]
    public void PlaneSphere_Random()
    {
        var aRandom = new Random(1403);
        for (int i = 0; i < 300; i++)
        {
            var aSph = new gp_Sphere(RandomAx3(aRandom), 0.5 + 10 * aRandom.NextDouble());
            var aPln = new gp_Pln(RandomAx3(aRandom));
            var anInter = new IntAna_QuadQuadGeo(aPln, aSph);
            Assert.True(anInter.IsDone());
            double aD = aPln.Distance(aSph.Location());
            Assert.Equal(aD < aSph.Radius() ? IntAna_Circle : IntAna_Empty, anInter.TypeInter());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aSph, p));
        }
    }

    [Fact]
    public void PlaneSphere_Special()
    {
        var aSph = new gp_Sphere(new gp_Ax3(new gp_Pnt(1, 1, 1), new gp_Dir(0, 1, 0)), 3.0);
        var aThrough = new IntAna_QuadQuadGeo(new gp_Pln(new gp_Pnt(1, 1, 1), new gp_Dir(1, 1, 0)), aSph);
        Assert.Equal(IntAna_Circle, aThrough.TypeInter());
        Assert.Equal(3.0, aThrough.Circle(1).Radius(), 1e-14);

        var aTangent = new IntAna_QuadQuadGeo(new gp_Pln(new gp_Pnt(1, 1, 4), new gp_Dir(0, 0, 1)), aSph);
        Assert.Equal(IntAna_Point, aTangent.TypeInter());
        Assert.True(aTangent.Point(1).IsEqual(new gp_Pnt(1, 1, 4), 1e-14));
    }

    [Fact]
    public void PlaneCone_Random()
    {
        var aRandom = new Random(1404);
        var aTypes = new HashSet<IntAna_ResultType>();
        for (int i = 0; i < 400; i++)
        {
            var aCone = new gp_Cone(RandomAx3(aRandom), 0.1 + 1.3 * aRandom.NextDouble(), 3 * aRandom.NextDouble());
            var aPln = new gp_Pln(RandomAx3(aRandom));
            var anInter = new IntAna_QuadQuadGeo(aPln, aCone, Precision.Angular(), Tol);
            if (!anInter.IsDone())
            {
                continue; // too large an ellipse or hyperbola: OCCT gives up
            }
            aTypes.Add(anInter.TypeInter());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aCone, p), 1.0e-6);
        }
        Assert.Contains(IntAna_Ellipse, aTypes);
        Assert.Contains(IntAna_Hyperbola, aTypes);
    }

    [Fact]
    public void PlaneCone_Special()
    {
        double aSemiAngle = Math.PI / 6;
        var aCone = new gp_Cone(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), aSemiAngle, 1.0);
        double aZApex = -1.0 / Math.Tan(aSemiAngle);

        // Perpendicular plane: a circle
        var aPlnC = new gp_Pln(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1));
        var aCirc = new IntAna_QuadQuadGeo(aPlnC, aCone, Precision.Angular(), Tol);
        Assert.Equal(IntAna_Circle, aCirc.TypeInter());
        Assert.Equal(1.0 + 2 * Math.Tan(aSemiAngle), aCirc.Circle(1).Radius(), 1e-12);

        // Plane containing the axis direction, away from it: two hyperbola branches
        var aPlnH = new gp_Pln(new gp_Pnt(0.5, 0, 0), new gp_Dir(1, 0, 0));
        var aHyp = new IntAna_QuadQuadGeo(aPlnH, aCone, Precision.Angular(), Tol);
        Assert.Equal(IntAna_Hyperbola, aHyp.TypeInter());
        Assert.Equal(2, aHyp.NbSolutions());
        AssertOnBoth(aHyp, p => Dist(aPlnH, p), p => Dist(aCone, p));

        // Plane parallel to a generatrix: a parabola
        gp_Dir aN = new(Math.Cos(aSemiAngle), 0, -Math.Sin(aSemiAngle));
        var aPlnP = new gp_Pln(new gp_Pnt(0, 0, 1), aN);
        var aPar = new IntAna_QuadQuadGeo(aPlnP, aCone, 1.0e-9, Tol);
        Assert.Equal(IntAna_Parabola, aPar.TypeInter());
        AssertOnBoth(aPar, p => Dist(aPlnP, p), p => Dist(aCone, p));

        // Planes through the apex: a point, one line, two lines
        var anApex = new gp_Pnt(0, 0, aZApex);
        Assert.Equal(IntAna_Point,
                     new IntAna_QuadQuadGeo(new gp_Pln(anApex, new gp_Dir(0, 0, 1)), aCone, Precision.Angular(), Tol).TypeInter());
        var aOne = new IntAna_QuadQuadGeo(new gp_Pln(anApex, aN), aCone, 1.0e-9, Tol);
        Assert.Equal(IntAna_Line, aOne.TypeInter());
        Assert.Equal(1, aOne.NbSolutions());
        var aPlnA = new gp_Pln(anApex, new gp_Dir(1, 0, 0));
        var aTwo = new IntAna_QuadQuadGeo(aPlnA, aCone, Precision.Angular(), Tol);
        Assert.Equal(IntAna_Line, aTwo.TypeInter());
        Assert.Equal(2, aTwo.NbSolutions());
        AssertOnBoth(aTwo, p => Dist(aPlnA, p), p => Dist(aCone, p));
    }

    [Fact]
    public void CylinderCylinder_ParallelAxes()
    {
        var aRandom = new Random(1405);
        for (int i = 0; i < 300; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            double aR1 = 0.5 + 3 * aRandom.NextDouble(), aR2 = 0.5 + 3 * aRandom.NextDouble();
            gp_Vec anOffset = new gp_Vec(anAx.XDirection()) * (5 * aRandom.NextDouble())
                              + new gp_Vec(anAx.YDirection()) * (5 * aRandom.NextDouble())
                              + new gp_Vec(anAx.Direction()) * (aRandom.NextDouble() - 0.5);
            var aCyl1 = new gp_Cylinder(anAx, aR1);
            var aCyl2 = new gp_Cylinder(new gp_Ax3(anAx.Location().Translated(anOffset), anAx.Direction()), aR2);
            var anInter = new IntAna_QuadQuadGeo(aCyl1, aCyl2, Tol);
            Assert.True(anInter.IsDone());
            IntAna_ResultType aType = anInter.TypeInter();
            Assert.True(aType == IntAna_Line || aType == IntAna_Empty, aType.ToString());
            AssertOnBoth(anInter, p => Dist(aCyl1, p), p => Dist(aCyl2, p), 1.0e-6);
        }
    }

    [Fact]
    public void CylinderCylinder_Special()
    {
        var anAx = new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 1, 0));
        Assert.Equal(IntAna_Same, new IntAna_QuadQuadGeo(new gp_Cylinder(anAx, 2), new gp_Cylinder(anAx, 2), Tol).TypeInter());
        Assert.Equal(IntAna_Empty, new IntAna_QuadQuadGeo(new gp_Cylinder(anAx, 2), new gp_Cylinder(anAx, 3), Tol).TypeInter());

        // Equal radii, intersecting axes: two ellipses
        var aCyl1 = new gp_Cylinder(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2);
        var aCyl2 = new gp_Cylinder(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 1)), 2);
        var anInter = new IntAna_QuadQuadGeo(aCyl1, aCyl2, Tol);
        Assert.Equal(IntAna_Ellipse, anInter.TypeInter());
        Assert.Equal(2, anInter.NbSolutions());
        AssertOnBoth(anInter, p => Dist(aCyl1, p), p => Dist(aCyl2, p));

        // Skew axes at the distance R1 + R2: a point
        var aCyl3 = new gp_Cylinder(new gp_Ax3(new gp_Pnt(0, 5, 1), new gp_Dir(1, 0, 0)), 3);
        var aPoint = new IntAna_QuadQuadGeo(aCyl1, aCyl3, Tol);
        Assert.Equal(IntAna_Point, aPoint.TypeInter());
        AssertOnBoth(aPoint, p => Dist(aCyl1, p), p => Dist(aCyl3, p));

        // Intersecting axes, different radii: no geometric solution
        var aCyl4 = new gp_Cylinder(new gp_Ax3(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0)), 3);
        Assert.Equal(IntAna_NoGeometricSolution, new IntAna_QuadQuadGeo(aCyl1, aCyl4, Tol).TypeInter());
    }

    [Fact]
    public void CylinderSphereAndCone_Coaxial()
    {
        var aRandom = new Random(1406);
        for (int i = 0; i < 200; i++)
        {
            gp_Ax3 anAx = RandomAx3(aRandom);
            var aCyl = new gp_Cylinder(anAx, 0.5 + 3 * aRandom.NextDouble());
            var aCenter = anAx.Location().Translated(new gp_Vec(anAx.Direction()) * (10 * aRandom.NextDouble() - 5));
            var aSph = new gp_Sphere(new gp_Ax3(aCenter, RandomDir(aRandom)), 0.5 + 5 * aRandom.NextDouble());
            var anInter = new IntAna_QuadQuadGeo(aCyl, aSph, Tol);
            Assert.True(anInter.IsDone());
            IntAna_ResultType aType = anInter.TypeInter();
            Assert.True(aType == IntAna_Circle || aType == IntAna_Empty || aType == IntAna_NoGeometricSolution,
                        aType.ToString());
            if (aType != IntAna_NoGeometricSolution)
            {
                AssertOnBoth(anInter, p => Dist(aCyl, p), p => Dist(aSph, p), 1.0e-6);
            }

            var aCone = new gp_Cone(new gp_Ax3(aCenter, anAx.Direction()), 0.1 + 1.3 * aRandom.NextDouble(),
                                    2 * aRandom.NextDouble());
            var anInterC = new IntAna_QuadQuadGeo(aCyl, aCone, Tol);
            Assert.True(anInterC.IsDone());
            if (anInterC.TypeInter() == IntAna_Circle)
            {
                Assert.Equal(2, anInterC.NbSolutions());
                AssertOnBoth(anInterC, p => Dist(aCyl, p), p => Dist(aCone, p), 1.0e-6);
            }
        }

        // Exactly coaxial: circles
        var aCylZ = new gp_Cylinder(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 3);
        var aSphZ = new gp_Sphere(new gp_Ax3(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1)), 5);
        var aCS = new IntAna_QuadQuadGeo(aCylZ, aSphZ, Tol);
        Assert.Equal(IntAna_Circle, aCS.TypeInter());
        Assert.Equal(2, aCS.NbSolutions());
        Assert.True(aCS.Circle(1).Location().IsEqual(new gp_Pnt(0, 0, 6), 1e-12));
        Assert.True(aCS.Circle(2).Location().IsEqual(new gp_Pnt(0, 0, -2), 1e-12));
        var aConeZ = new gp_Cone(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), Math.PI / 4, 1);
        var aCC = new IntAna_QuadQuadGeo(aCylZ, aConeZ, Tol);
        Assert.Equal(IntAna_Circle, aCC.TypeInter());
        AssertOnBoth(aCC, p => Dist(aCylZ, p), p => Dist(aConeZ, p));

        // Off-axis sphere
        var aSphOff = new gp_Sphere(new gp_Ax3(new gp_Pnt(1, 0, 2), new gp_Dir(0, 0, 1)), 5);
        Assert.Equal(IntAna_NoGeometricSolution, new IntAna_QuadQuadGeo(aCylZ, aSphOff, Tol).TypeInter());
    }

    [Fact]
    public void SphereSphere_Random()
    {
        var aRandom = new Random(1407);
        for (int i = 0; i < 300; i++)
        {
            var aSph1 = new gp_Sphere(RandomAx3(aRandom), 0.5 + 8 * aRandom.NextDouble());
            var aSph2 = new gp_Sphere(RandomAx3(aRandom), 0.5 + 8 * aRandom.NextDouble());
            var anInter = new IntAna_QuadQuadGeo(aSph1, aSph2, Tol);
            Assert.True(anInter.IsDone());
            double aD = aSph1.Location().Distance(aSph2.Location());
            bool anIntersects = aD < aSph1.Radius() + aSph2.Radius() && aD > Math.Abs(aSph1.Radius() - aSph2.Radius());
            Assert.Equal(anIntersects ? IntAna_Circle : IntAna_Empty, anInter.TypeInter());
            AssertOnBoth(anInter, p => Dist(aSph1, p), p => Dist(aSph2, p), 1.0e-6);
        }

        var aS = new gp_Sphere(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2);
        Assert.Equal(IntAna_Same, new IntAna_QuadQuadGeo(aS, aS, Tol).TypeInter());
        var aTangent = new IntAna_QuadQuadGeo(aS, new gp_Sphere(new gp_Ax3(new gp_Pnt(3, 0, 0), new gp_Dir(0, 0, 1)), 1), Tol);
        Assert.Equal(IntAna_Point, aTangent.TypeInter());
        Assert.True(aTangent.Point(1).IsEqual(new gp_Pnt(2, 0, 0), 1e-12));
    }

    [Fact]
    public void SphereCone_Coaxial()
    {
        var aCone = new gp_Cone(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), Math.PI / 6, 1);
        for (int k = 0; k < 20; k++)
        {
            var aSph = new gp_Sphere(new gp_Ax3(new gp_Pnt(0, 0, -3 + 0.5 * k), new gp_Dir(0, 0, 1)), 1 + 0.3 * k);
            var anInter = new IntAna_QuadQuadGeo(aSph, aCone, Tol);
            Assert.True(anInter.IsDone());
            IntAna_ResultType aType = anInter.TypeInter();
            Assert.True(aType == IntAna_Circle || aType == IntAna_Empty || aType == IntAna_PointAndCircle,
                        aType.ToString());
            AssertOnBoth(anInter, p => Dist(aSph, p), p => Dist(aCone, p), 1.0e-6);
        }

        // Sphere centered at the apex: the two circles at the same distance
        gp_Pnt anApex = aCone.Apex();
        var aSphA = new gp_Sphere(new gp_Ax3(anApex, new gp_Dir(0, 0, 1)), 2);
        var anInterA = new IntAna_QuadQuadGeo(aSphA, aCone, Tol);
        Assert.Equal(IntAna_Circle, anInterA.TypeInter());
        Assert.Equal(2, anInterA.NbSolutions());
        Assert.Equal(1.0, anInterA.Circle(1).Radius(), 1e-12);
        AssertOnBoth(anInterA, p => Dist(aSphA, p), p => Dist(aCone, p));
    }

    [Fact]
    public void ConeCone_Cases()
    {
        var anAx = new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aCone1 = new gp_Cone(anAx, Math.PI / 6, 1);

        // Same axis, different angles: two circles
        var aCone2 = new gp_Cone(new gp_Ax3(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), Math.PI / 4, 1);
        var anInter = new IntAna_QuadQuadGeo(aCone1, aCone2, Tol);
        Assert.Equal(IntAna_Circle, anInter.TypeInter());
        AssertOnBoth(anInter, p => Dist(aCone1, p), p => Dist(aCone2, p));

        // Same cone: Same
        Assert.Equal(IntAna_Same, new IntAna_QuadQuadGeo(aCone1, aCone1, Tol).TypeInter());

        // Common apex, intersecting axes: lines through the apex
        gp_Pnt anApex = aCone1.Apex();
        var aCone3 = new gp_Cone(new gp_Ax3(anApex, new gp_Dir(Math.Sin(0.3), 0, Math.Cos(0.3))), Math.PI / 6, 0);
        var aLines = new IntAna_QuadQuadGeo(aCone1, aCone3, Tol);
        Assert.Equal(IntAna_Line, aLines.TypeInter());
        Assert.Equal(2, aLines.NbSolutions());
        AssertOnBoth(aLines, p => Dist(aCone1, p), p => Dist(aCone3, p), 1.0e-6);

        // Parallel axes, equal angles: a conic in a plane
        var aCone4 = new gp_Cone(new gp_Ax3(new gp_Pnt(2, 0, 0), new gp_Dir(0, 0, 1)), Math.PI / 6, 1);
        var aPar = new IntAna_QuadQuadGeo(aCone1, aCone4, Tol);
        Assert.True(aPar.IsDone());
        Assert.Equal(IntAna_Hyperbola, aPar.TypeInter());
        AssertOnBoth(aPar, p => Dist(aCone1, p), p => Dist(aCone4, p), 1.0e-6);
    }

    [Fact]
    public void PlaneTorus()
    {
        var aTor = new gp_Torus(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 5, 2);
        for (int k = 0; k <= 10; k++)
        {
            var aPln = new gp_Pln(new gp_Pnt(0, 0, 3 - 2.5 + 0.5 * k), new gp_Dir(0, 0, 1));
            var anInter = new IntAna_QuadQuadGeo(aPln, aTor, Tol);
            Assert.True(anInter.IsDone());
            IntAna_ResultType aType = anInter.TypeInter();
            Assert.True(aType == IntAna_Circle || aType == IntAna_Empty, aType.ToString());
            AssertOnBoth(anInter, p => Dist(aPln, p), p => Dist(aTor, p));
        }

        var aPlnAx = new gp_Pln(new gp_Pnt(1, 2, 3), new gp_Dir(1, 1, 0));
        var anAxial = new IntAna_QuadQuadGeo(aPlnAx, aTor, Tol);
        Assert.Equal(IntAna_Circle, anAxial.TypeInter());
        Assert.Equal(2, anAxial.NbSolutions());
        AssertOnBoth(anAxial, p => Dist(aPlnAx, p), p => Dist(aTor, p));

        var aPlnSkew = new gp_Pln(new gp_Pnt(1, 2, 3), new gp_Dir(1, 0, 1));
        Assert.Equal(IntAna_NoGeometricSolution, new IntAna_QuadQuadGeo(aPlnSkew, aTor, Tol).TypeInter());
    }

    [Fact]
    public void QuadricsAndTorus_Coaxial()
    {
        var anAx = new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aTor = new gp_Torus(anAx, 5, 2);
        for (int k = 0; k <= 10; k++)
        {
            var aCyl = new gp_Cylinder(anAx, 2.5 + 0.6 * k);
            var anInter = new IntAna_QuadQuadGeo(aCyl, aTor, Tol);
            Assert.True(anInter.IsDone());
            AssertOnBoth(anInter, p => Dist(aCyl, p), p => Dist(aTor, p));

            var aSph = new gp_Sphere(new gp_Ax3(new gp_Pnt(0, 0, -2 + 0.4 * k), new gp_Dir(0, 0, 1)), 3.5 + 0.5 * k);
            var anInterS = new IntAna_QuadQuadGeo(aSph, aTor, Tol);
            Assert.True(anInterS.IsDone());
            AssertOnBoth(anInterS, p => Dist(aSph, p), p => Dist(aTor, p), 1.0e-6);

            var aCone = new gp_Cone(new gp_Ax3(new gp_Pnt(0, 0, -1 + 0.2 * k), new gp_Dir(0, 0, 1)), 0.2 + 0.1 * k, 3);
            var anInterC = new IntAna_QuadQuadGeo(aCone, aTor, Tol);
            Assert.True(anInterC.IsDone());
            AssertOnBoth(anInterC, p => Dist(aCone, p), p => Dist(aTor, p), 1.0e-6);

            var aTor2 = new gp_Torus(new gp_Ax3(new gp_Pnt(0, 0, -1 + 0.2 * k), new gp_Dir(0, 0, 1)), 4 + 0.3 * k, 1.5);
            var anInterT = new IntAna_QuadQuadGeo(aTor, aTor2, Tol);
            Assert.True(anInterT.IsDone());
            AssertOnBoth(anInterT, p => Dist(aTor, p), p => Dist(aTor2, p), 1.0e-6);
        }

        Assert.Equal(IntAna_Same, new IntAna_QuadQuadGeo(aTor, aTor, Tol).TypeInter());
        var aCylOff = new gp_Cylinder(new gp_Ax3(new gp_Pnt(1, 0, 0), new gp_Dir(0, 0, 1)), 4);
        Assert.Equal(IntAna_NoGeometricSolution, new IntAna_QuadQuadGeo(aCylOff, aTor, Tol).TypeInter());
    }

    [Fact]
    public void Accessors_CheckStateAndRange()
    {
        var anEmpty = new IntAna_QuadQuadGeo();
        Assert.False(anEmpty.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anEmpty.TypeInter());
        Assert.Throws<StdFail_NotDone>(() => anEmpty.NbSolutions());

        var aPlnPln = new IntAna_QuadQuadGeo(new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                             new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), Precision.Angular(), Tol);
        Assert.Throws<Standard_DomainError>(() => aPlnPln.Line(2));
        Assert.Throws<Standard_DomainError>(() => aPlnPln.Circle(1));
        Assert.Throws<Standard_DomainError>(() => aPlnPln.Ellipse(1));
        Assert.Throws<Standard_DomainError>(() => aPlnPln.Hyperbola(1));
        Assert.Throws<Standard_DomainError>(() => aPlnPln.Parabola(1));
        Assert.False(aPlnPln.HasCommonGen());
        Assert.Equal(new gp_Pnt(0, 0, 0), aPlnPln.PChar());
    }
}
