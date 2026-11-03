// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of GeomAPI_IntCS, and through it IntCurveSurface_HInter's paths: analytic (IntAna), curve/quadric
/// (roots of the quadric's equation), and polygon/polyhedron interference refined by IntImp_IntCS.
/// </summary>
public class GeomAPI_IntCSTests
{
    private static readonly CoordinateSystem XOY = new(Constants.XOY());

    private static CurveSurfaceIntersection Intersect(Curve3D theC, Surface theS)
    {
        CurveSurfaceIntersection anInter = new(theC, theS);
        Assert.True(anInter.IsDone);
        Assert.Equal(0, anInter.SegmentCount);
        return anInter;
    }

    /// <summary>Each point is on the curve at W and on the surface at (U, V).</summary>
    private static void CheckPoints(CurveSurfaceIntersection theInter, Curve3D theC, Surface theS, double theTol)
    {
        for (int i = 1; i <= theInter.PointCount; i++)
        {
            theInter.Parameters(i, out double U, out double V, out double W);
            Point3D aP = theInter.Point(i);
            Assert.True(aP.Distance(theC.Value(W)) <= theTol, $"point {i} off the curve");
            Assert.True(aP.Distance(theS.Value(U, V)) <= theTol, $"point {i} off the surface: {aP} vs {theS.Value(U, V)}");
        }
    }

    [Fact]
    public void LinePlane_OnePointGoingOut()
    {
        LineCurve3D aLine = new(new Point3D(1.0, 2.0, -5.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface aPlane = new(new Plane3D(new Point3D(0.0, 0.0, 2.0), new Direction3D(0.0, 0.0, 1.0)));
        CurveSurfaceIntersection anInter = Intersect(aLine, aPlane);

        Assert.Equal(1, anInter.PointCount);
        anInter.Parameters(1, out double U, out double V, out double W);
        Assert.Equal(7.0, W, 1e-14);
        Assert.Equal(1.0, U, 1e-14);
        Assert.Equal(2.0, V, 1e-14);
        Assert.True(anInter.Point(1).IsEqual(new Point3D(1.0, 2.0, 2.0), 1e-14));

        // The line goes along the plane's normal: out of the surface
        HInter aHInter = new();
        aHInter.Perform(new CurveAdaptor(aLine), new SurfaceAdaptor(aPlane));
        Assert.Equal(TransitionOnCurve.Out, aHInter.Point(1).Transition());
        Assert.False(aHInter.IsParallel);
    }

    [Fact]
    public void LineParallelToPlane_IsParallelWithoutPoints()
    {
        PlaneSurface aPlane = new(XOY);
        HInter aHInter = new();
        aHInter.Perform(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 1), new Direction3D(1, 1, 0))),
                        new SurfaceAdaptor(aPlane));
        Assert.True(aHInter.IsDone);
        Assert.True(aHInter.IsParallel);
        Assert.Equal(0, aHInter.PointCount);

        // and a line in the plane
        aHInter.Perform(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 1, 0))),
                        new SurfaceAdaptor(aPlane));
        Assert.True(aHInter.IsParallel);
        Assert.Equal(0, aHInter.PointCount);
    }

    [Fact]
    public void LineSphere_TwoPointsInThenOut()
    {
        LineCurve3D aLine = new(new Point3D(-10.0, 0.5, 0.0), new Direction3D(1.0, 0.0, 0.0));
        SphericalSurface aSphere = new(XOY, 2.0);
        CurveSurfaceIntersection anInter = Intersect(aLine, aSphere);

        Assert.Equal(2, anInter.PointCount);
        double x = Math.Sqrt(4.0 - 0.25);
        double[] aXs = [anInter.Point(1).X(), anInter.Point(2).X()];
        Array.Sort(aXs);
        Assert.Equal(-x, aXs[0], 1e-12);
        Assert.Equal(x, aXs[1], 1e-12);
        CheckPoints(anInter, aLine, aSphere, 1e-12);

        HInter aHInter = new();
        aHInter.Perform(new CurveAdaptor(aLine), new SurfaceAdaptor(aSphere));
        for (int i = 1; i <= 2; i++)
        {
            CurveSurfaceIntersectionPoint aPt = aHInter.Point(i);
            Assert.Equal(aPt.Pnt().X() < 0 ? TransitionOnCurve.In : TransitionOnCurve.Out, aPt.Transition());
        }
    }

    [Fact]
    public void LineTangentToSphere_TangentTransition()
    {
        LineCurve3D aLine = new(new Point3D(-10.0, 2.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        SphericalSurface aSphere = new(XOY, 2.0);
        HInter aHInter = new();
        aHInter.Perform(new CurveAdaptor(aLine), new SurfaceAdaptor(aSphere));

        Assert.True(aHInter.PointCount >= 1);
        for (int i = 1; i <= aHInter.PointCount; i++)
        {
            Assert.True(aHInter.Point(i).Pnt().IsEqual(new Point3D(0.0, 2.0, 0.0), 1e-6));
            Assert.Equal(TransitionOnCurve.Tangent, aHInter.Point(i).Transition());
        }
    }

    [Fact]
    public void LineTorus_FourPoints()
    {
        LineCurve3D aLine = new(new Point3D(-10.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        ToroidalSurface aTorus = new(XOY, 3.0, 1.0);
        CurveSurfaceIntersection anInter = Intersect(aLine, aTorus);

        Assert.Equal(4, anInter.PointCount);
        double[] aXs = new double[4];
        for (int i = 1; i <= 4; i++)
        {
            aXs[i - 1] = anInter.Point(i).X();
        }
        Array.Sort(aXs);
        Assert.Equal(-4.0, aXs[0], 1e-9);
        Assert.Equal(-2.0, aXs[1], 1e-9);
        Assert.Equal(2.0, aXs[2], 1e-9);
        Assert.Equal(4.0, aXs[3], 1e-9);
        CheckPoints(anInter, aLine, aTorus, 1e-9);
    }

    [Fact]
    public void CircleCylinder_Analytic()
    {
        // A circle of radius 2 in the plane y = 0 centred at (1, 0, 0): it meets the cylinder x^2 + y^2 = 4 at x = 2, z = ±sqrt(3)
        CircleCurve3D aCircle = new(new Frame3D(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0)), 2.0);
        CylindricalSurface aCyl = new(XOY, 2.0);
        CurveSurfaceIntersection anInter = Intersect(aCircle, aCyl);

        Assert.Equal(2, anInter.PointCount);
        CheckPoints(anInter, aCircle, aCyl, 1e-9);
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            anInter.Parameters(i, out double U, out _, out double W);
            Assert.InRange(W, 0.0, 2 * Math.PI);
            Assert.InRange(U, 0.0, 2 * Math.PI);
        }
    }

    /// <summary>A degree 3 B-spline curve with two spans, crossing the plane z = 0.5 twice (the curve/quadric path).</summary>
    private static BSplineCurve3D WaveCurve()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(-1.0, 0.1, 0.0);
        aPoles[2] = new Point3D(-0.5, 0.2, 2.0);
        aPoles[3] = new Point3D(0.0, 0.3, -1.0);
        aPoles[4] = new Point3D(0.5, 0.2, 2.0);
        aPoles[5] = new Point3D(1.0, 0.1, 0.0);
        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.4;
        aKnots[3] = 1.0;
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void BSplineCurvePlane_RootsOfTheQuadric()
    {
        BSplineCurve3D aCurve = WaveCurve();
        PlaneSurface aPlane = new(new Plane3D(new Point3D(0, 0, 0.5), new Direction3D(0, 0, 1)));
        CurveSurfaceIntersection anInter = Intersect(aCurve, aPlane);

        // Count the sign changes of z - 0.5 along the curve
        int aChanges = 0;
        double aPrev = aCurve.Value(0.0).Z() - 0.5;
        for (int i = 1; i <= 1000; i++)
        {
            double z = aCurve.Value(i / 1000.0).Z() - 0.5;
            if (Math.Sign(z) != Math.Sign(aPrev))
            {
                aChanges++;
            }
            aPrev = z;
        }
        Assert.True(aChanges > 0);
        Assert.Equal(aChanges, anInter.PointCount);
        CheckPoints(anInter, aCurve, aPlane, 1e-7);
        for (int i = 1; i <= anInter.PointCount; i++)
        {
            Assert.Equal(0.5, anInter.Point(i).Z(), 1e-7);
        }
    }

    /// <summary>The paraboloid z = x^2 + y^2 on [-1, 1]^2 as a biquadratic B-spline surface (exact).</summary>
    private static BSplineSurface Paraboloid()
    {
        double[] c = [1.0, -1.0, 1.0]; // Bernstein coefficients of (2t - 1)^2
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D(-1.0 + (i - 1), -1.0 + (j - 1), c[i - 1] + c[j - 1]);
            }
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        return new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 2, 2);
    }

    [Fact]
    public void BSplineCurveBSplineSurface_Polyhedron()
    {
        BSplineSurface aSurf = Paraboloid();

        // A vertical line, as a degree 3 B-spline: crosses at z = 0.3^2 + 0.4^2 = 0.25
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aPoles[i] = new Point3D(0.3, 0.4, -1.0 + i);
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 3.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        BSplineCurve3D aLine = new(aPoles, aKnots, aMults, 3);

        CurveSurfaceIntersection anInter = Intersect(aLine, aSurf);
        Assert.Equal(1, anInter.PointCount);
        Assert.True(anInter.Point(1).IsEqual(new Point3D(0.3, 0.4, 0.25), 1e-9));
        anInter.Parameters(1, out double U, out double V, out double W);
        Assert.Equal(0.65, U, 1e-9);
        Assert.Equal(0.7, V, 1e-9);
        Assert.Equal(0.25, W, 1e-9);

        // A curved B-spline crossing the paraboloid twice
        BSplineCurve3D aWave = WaveCurve();
        CurveSurfaceIntersection anInter2 = Intersect(aWave, aSurf);
        Assert.True(anInter2.PointCount >= 2);
        CheckPoints(anInter2, aWave, aSurf, 1e-7);
    }

    [Fact]
    public void LineBSplineSurface_ParallelToTheAxisGoesThroughTheLowestPoint()
    {
        // A vertical line through the apex of the paraboloid: one transversal point at (0, 0, 0)
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, -1.0), new Direction3D(0.0, 0.0, 1.0));
        CurveSurfaceIntersection anInter = Intersect(aLine, Paraboloid());
        Assert.Equal(1, anInter.PointCount);
        Assert.True(anInter.Point(1).IsEqual(new Point3D(0.0, 0.0, 0.0), 1e-9));
    }

    [Fact]
    public void LineExtrusionWithInfiniteBounds()
    {
        // An infinite extrusion of a circle along Z (a cylinder by another name): the line enters and leaves
        CircleCurve3D aCircle = new(Constants.XOY(), 2.0);
        ExtrusionSurface anExtr = new(aCircle, new Direction3D(0.0, 0.0, 1.0));
        LineCurve3D aLine = new(new Point3D(-5.0, 0.5, 3.0), new Direction3D(1.0, 0.0, 0.2));
        CurveSurfaceIntersection anInter = Intersect(aLine, anExtr);
        Assert.Equal(2, anInter.PointCount);
        CheckPoints(anInter, aLine, anExtr, 1e-7);
        for (int i = 1; i <= 2; i++)
        {
            Point3D aP = anInter.Point(i);
            Assert.Equal(2.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1e-7);
        }
    }

    [Fact]
    public void NoIntersection()
    {
        LineCurve3D aLine = new(new Point3D(-10.0, 5.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        CurveSurfaceIntersection anInter = Intersect(aLine, new SphericalSurface(XOY, 2.0));
        Assert.Equal(0, anInter.PointCount);
        Assert.Equal(0, Intersect(aLine, Paraboloid()).PointCount);
    }

    [Fact]
    public void NotDoneBeforePerform()
    {
        HInter aHInter = new();
        Assert.False(aHInter.IsDone);
        Assert.ThrowsAny<NotDoneException>(() => aHInter.PointCount);
        CurveSurfaceIntersection anInter = new();
        Assert.False(anInter.IsDone);
    }
}
