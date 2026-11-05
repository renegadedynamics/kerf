// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeConstruct_ProjectCurveOnSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Construct;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's ShapeConstruct_ProjectCurveOnSurface GTest (the fixture's SetUp is the constructor).</summary>
public class ShapeConstruct_ProjectCurveOnSurfaceTest
{
    private readonly ProjectCurveOnSurface myProjector = new();
    private readonly double myTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();

    // Helper to create a simple B-spline curve lying on Z=0 plane
    private static BSplineCurve3D createPlanarBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(2, 3, 0);
        aPoles[3] = new Point3D(5, 4, 0);
        aPoles[4] = new Point3D(8, 2, 0);
        aPoles[5] = new Point3D(10, 0, 0);

        PointsToBSpline aBuilder = new(aPoles);
        return aBuilder.Curve();
    }

    // Helper to create a helical B-spline on a cylinder
    private static BSplineCurve3D createHelicalBSpline(double theRadius, int theNbPoints)
    {
        NCollection_Array1<Point3D> aPoles = new(1, theNbPoints);
        for (int i = 1; i <= theNbPoints; ++i)
        {
            double anAngle = (i - 1) * Math.PI / 4.0;
            double aZ = (i - 1) * 1.0;
            aPoles[i] = new Point3D(theRadius * Math.Cos(anAngle), theRadius * Math.Sin(anAngle), aZ);
        }
        PointsToBSpline aBuilder = new(aPoles);
        return aBuilder.Curve();
    }

    // Verifies that pcurve correctly maps to 3D points on surface
    private static void verifyProjection(Curve3D theCurve, Curve2D thePCurve, SurfaceAnalysis theSurface,
                                         double theTolerance)
    {
        double aFirst = theCurve.FirstParameter;
        double aLast = theCurve.LastParameter;

        for (double t = 0.0; t <= 1.0; t += 0.25)
        {
            double aParam = aFirst + t * (aLast - aFirst);
            theCurve.D0(aParam, out Point3D a3DPnt);

            Point2D a2DPnt = thePCurve.Value(aParam);
            Point3D aSurfPnt = theSurface.Value(a2DPnt);

            Assert.Equal(0.0, a3DPnt.DistanceTo(aSurfPnt), theTolerance);
        }
    }

    private static PlaneSurface XOY() => new(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));

    private static TrimmedCurve3D Segment(Point3D theP1, Point3D theP2) => new MakeSegment(theP1, theP2).Value();

    //=============================================================================================
    // Basic projection tests - lines and circles on analytical surfaces
    //=============================================================================================

    // Test: Project diagonal line on horizontal plane, verify 2D coordinates match X,Y of 3D points
    [Fact]
    public void LineOnPlane_A1()
    {
        PlaneSurface aPlane = XOY();
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(10, 10, 0));

        SurfaceAnalysis aSAS = new(aPlane);
        myProjector.Init(aSAS, myTolerance);

        double aFirst = aLine.FirstParameter;
        double aLast = aLine.LastParameter;
        bool aResult = myProjector.Perform(aLine, aFirst, aLast, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // Verify start point maps to (0,0) and end point maps to (10,10)
        Point2D aStart = aPCurve.Value(aFirst);
        Point2D aEnd = aPCurve.Value(aLast);
        Assert.Equal(0.0, aStart.X, 0.01);
        Assert.Equal(0.0, aStart.Y, 0.01);
        Assert.Equal(10.0, aEnd.X, 0.01);
        Assert.Equal(10.0, aEnd.Y, 0.01);
    }

    // Test: Project circle lying on cylinder surface, V-coordinate should be constant
    [Fact]
    public void CircleOnCylinder_A2()
    {
        double aRadius = 5.0;
        CylindricalSurface aCylinder = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aRadius);

        // Circle at height Z=3 on the cylinder
        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), aRadius);
        CircleCurve3D aCircle = new(aCirc);

        myProjector.Init(new SurfaceAnalysis(aCylinder), myTolerance);

        bool aResult = myProjector.Perform(aCircle, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // On cylinder parametrization, V = Z, so circle at Z=3 should have V=3
        Point2D aMid = aPCurve.Value(Math.PI);
        Assert.Equal(3.0, aMid.Y, 0.01);
    }

    // Test: Project vertical line on cylinder (isoparametric case), U should be constant
    [Fact]
    public void IsoparametricLineOnCylinder_A3()
    {
        double aRadius = 5.0;
        CylindricalSurface aCylinder = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aRadius);

        // Line along cylinder axis at X=radius, Y=0 (U=0 on cylinder)
        TrimmedCurve3D aLine = Segment(new Point3D(aRadius, 0, 0), new Point3D(aRadius, 0, 10));

        SurfaceAnalysis aSAS = new(aCylinder);
        myProjector.Init(aSAS, myTolerance);

        double aFirst = aLine.FirstParameter;
        double aLast = aLine.LastParameter;
        bool aResult = myProjector.Perform(aLine, aFirst, aLast, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // U should be constant (isoparametric curve)
        Point2D aStart = aPCurve.Value(aFirst);
        Point2D aEnd = aPCurve.Value(aLast);
        Assert.Equal(aEnd.X, aStart.X, 0.01);
    }

    //=============================================================================================
    // B-Spline curve projection tests
    //=============================================================================================

    // Test: Project planar B-spline on plane, endpoints should match original curve
    [Fact]
    public void BSplineOnPlane_B1()
    {
        PlaneSurface aPlane = XOY();
        BSplineCurve3D aBSpline = createPlanarBSpline();
        Assert.NotNull(aBSpline);

        myProjector.Init(new SurfaceAnalysis(aPlane), myTolerance);

        bool aResult = myProjector.Perform(aBSpline, aBSpline.FirstParameter, aBSpline.LastParameter,
                                           out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // Verify endpoints
        Point2D aStart = aPCurve.Value(aBSpline.FirstParameter);
        Point2D aEnd = aPCurve.Value(aBSpline.LastParameter);
        Assert.Equal(0.0, aStart.X, 0.1);
        Assert.Equal(0.0, aStart.Y, 0.1);
        Assert.Equal(10.0, aEnd.X, 0.1);
        Assert.Equal(0.0, aEnd.Y, 0.1);
    }

    // Test: Project helical B-spline on cylinder, verify all sample points lie on surface
    [Fact]
    public void HelicalBSplineOnCylinder_B2()
    {
        double aRadius = 5.0;
        CylindricalSurface aCylinder = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aRadius);

        BSplineCurve3D aBSpline = createHelicalBSpline(aRadius, 9);
        Assert.NotNull(aBSpline);

        SurfaceAnalysis aSAS = new(aCylinder);
        myProjector.Init(aSAS, myTolerance);

        bool aResult = myProjector.Perform(aBSpline, aBSpline.FirstParameter, aBSpline.LastParameter,
                                           out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        verifyProjection(aBSpline, aPCurve, aSAS, 0.1);
    }

    // Test: Project high-degree B-spline with many control points
    [Fact]
    public void HighDegreeBSpline_B3()
    {
        PlaneSurface aPlane = XOY();

        // Create B-spline with 50 control points (sinusoidal shape)
        int aNbPoles = 50;
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            double aX = (i - 1) * 0.5;
            double aY = 5.0 * Math.Sin(aX * Math.PI / 10.0);
            aPoles[i] = new Point3D(aX, aY, 0);
        }

        PointsToBSpline aBuilder = new(aPoles, 3, 8);
        BSplineCurve3D aBSpline = aBuilder.Curve();
        Assert.NotNull(aBSpline);

        SurfaceAnalysis aSAS = new(aPlane);
        myProjector.Init(aSAS, myTolerance);

        bool aResult = myProjector.Perform(aBSpline, aBSpline.FirstParameter, aBSpline.LastParameter,
                                           out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        verifyProjection(aBSpline, aPCurve, aSAS, 0.5);
    }

    //=============================================================================================
    // Periodic surface tests - sphere, torus
    //=============================================================================================

    // Test: Project equator circle on sphere
    [Fact]
    public void EquatorOnSphere_C1()
    {
        double aRadius = 10.0;
        SphericalSurface aSphere = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aRadius);

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), aRadius);
        CircleCurve3D aCircle = new(aCirc);

        myProjector.Init(new SurfaceAnalysis(aSphere), myTolerance);

        bool aResult = myProjector.Perform(aCircle, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);
    }

    // Test: Project latitude circle on sphere at 45 degrees, V should be constant
    [Fact]
    public void LatitudeOnSphere_C2()
    {
        double aRadius = 10.0;
        SphericalSurface aSphere = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aRadius);

        // Latitude at 45 degrees
        double aLatitude = Math.PI / 4.0;
        double aZ = aRadius * Math.Sin(aLatitude);
        double aCircRadius = aRadius * Math.Cos(aLatitude);

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, aZ), new Direction3D(0, 0, 1)), aCircRadius);
        CircleCurve3D aCircle = new(aCirc);

        myProjector.Init(new SurfaceAnalysis(aSphere), myTolerance);

        bool aResult = myProjector.Perform(aCircle, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // V should be constant for latitude circle
        Point2D aStart = aPCurve.Value(0.0);
        Point2D aMid = aPCurve.Value(Math.PI);
        Assert.Equal(aMid.Y, aStart.Y, 0.01);
    }

    // Test: Project on toroidal surface (both U and V periodic)
    [Fact]
    public void CircleOnTorus_C3()
    {
        double aMajorRadius = 10.0;
        double aMinorRadius = 2.0;
        ToroidalSurface aTorus = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aMajorRadius, aMinorRadius);

        // Circle in the minor cross-section plane at U=0
        Circle3D aCirc = new(new Frame3D(new Point3D(aMajorRadius + aMinorRadius, 0, 0), new Direction3D(0, 1, 0)), aMinorRadius);
        CircleCurve3D aCircle = new(aCirc);

        myProjector.Init(new SurfaceAnalysis(aTorus), myTolerance);

        bool aResult = myProjector.Perform(aCircle, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Conical surface tests
    //=============================================================================================

    // Test: Project circle on cone at specific height
    [Fact]
    public void CircleOnCone_D1()
    {
        double aRadius = 5.0;
        double aSemiAngle = Math.PI / 6.0; // 30 degrees
        ConicalSurface aCone = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aSemiAngle, aRadius);

        // Circle at height Z=5, radius increases with height
        double aZ = 5.0;
        double aRadiusAtZ = aRadius + aZ * Math.Tan(aSemiAngle);
        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, aZ), new Direction3D(0, 0, 1)), aRadiusAtZ);
        CircleCurve3D aCircle = new(aCirc);

        myProjector.Init(new SurfaceAnalysis(aCone), myTolerance);

        bool aResult = myProjector.Perform(aCircle, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Trimmed curve tests
    //=============================================================================================

    // Test: Project 90-degree arc on plane
    [Fact]
    public void ArcOnPlane_E1()
    {
        PlaneSurface aPlane = XOY();

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CircleCurve3D aFullCircle = new(aCirc);
        TrimmedCurve3D aArc = new(aFullCircle, 0.0, Math.PI / 2.0);

        SurfaceAnalysis aSAS = new(aPlane);
        myProjector.Init(aSAS, myTolerance);

        double aFirst = aArc.FirstParameter;
        double aLast = aArc.LastParameter;
        bool aResult = myProjector.Perform(aArc, aFirst, aLast, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // Arc from (5,0) to (0,5)
        Point2D aStart = aPCurve.Value(aFirst);
        Point2D aEnd = aPCurve.Value(aLast);
        Assert.Equal(5.0, aStart.X, 0.01);
        Assert.Equal(0.0, aStart.Y, 0.01);
        Assert.Equal(0.0, aEnd.X, 0.01);
        Assert.Equal(5.0, aEnd.Y, 0.01);
    }

    // Test: Project doubly-trimmed curve (tests recursive unwrapping of trimmed curves)
    [Fact]
    public void DoublyTrimmedCurve_E2()
    {
        PlaneSurface aPlane = XOY();
        BSplineCurve3D aBSpline = createPlanarBSpline();
        Assert.NotNull(aBSpline);

        // First trim: take first half
        double aFirst1 = aBSpline.FirstParameter;
        double aLast1 = aBSpline.LastParameter;
        double aMid = (aFirst1 + aLast1) / 2.0;
        TrimmedCurve3D aTrim1 = new(aBSpline, aFirst1, aMid);

        // Second trim (nested): take first quarter of the first trim
        double aFirst2 = aTrim1.FirstParameter;
        double aLast2 = aTrim1.LastParameter;
        double aQuart = aFirst2 + (aLast2 - aFirst2) / 4.0;
        TrimmedCurve3D aTrim2 = new(aTrim1, aFirst2, aQuart);

        myProjector.Init(new SurfaceAnalysis(aPlane), myTolerance);

        bool aResult = myProjector.Perform(aTrim2, aTrim2.FirstParameter, aTrim2.LastParameter, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection of doubly-trimmed curve should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Ellipse projection tests
    //=============================================================================================

    // Test: Project ellipse on plane, verify quadrant points
    [Fact]
    public void EllipseOnPlane_F1()
    {
        PlaneSurface aPlane = XOY();

        // Ellipse with major radius 10, minor radius 5
        Ellipse3D anElips = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 5.0);
        EllipseCurve3D anEllipse = new(anElips);

        SurfaceAnalysis aSAS = new(aPlane);
        myProjector.Init(aSAS, myTolerance);

        bool aResult = myProjector.Perform(anEllipse, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed");
        Assert.NotNull(aPCurve);

        // At parameter 0, ellipse is at (10, 0)
        Point2D aP0 = aPCurve.Value(0.0);
        Assert.Equal(10.0, aP0.X, 0.01);
        Assert.Equal(0.0, aP0.Y, 0.01);

        // At parameter PI/2, ellipse is at (0, 5)
        Point2D aP1 = aPCurve.Value(Math.PI / 2.0);
        Assert.Equal(0.0, aP1.X, 0.01);
        Assert.Equal(5.0, aP1.Y, 0.01);
    }

    //=============================================================================================
    // API method tests - Init, SetSurface, SetPrecision
    //=============================================================================================

    // Test: Init with Geom_Surface directly (not wrapped in ShapeAnalysis_Surface)
    [Fact]
    public void InitWithGeomSurface_G1()
    {
        PlaneSurface aPlane = XOY();
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(10, 10, 0));

        myProjector.Init(aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool aResult = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out Curve2D? aPCurve);

        Assert.True(aResult, "Init with Geom_Surface should work");
        Assert.NotNull(aPCurve);
    }

    // Test: SetSurface + SetPrecision separately
    [Fact]
    public void SetSurfaceAndPrecision_G2()
    {
        PlaneSurface aPlane = XOY();
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(10, 10, 0));

        myProjector.SetSurface(aPlane);
        myProjector.SetPrecision(global::Kerf.Engine.Foundation.Precision.Confusion());

        bool aResult = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out Curve2D? aPCurve);

        Assert.True(aResult, "SetSurface + SetPrecision should work");
        Assert.NotNull(aPCurve);
    }

    // Test: AdjustOverDegenMode accessor returns modifiable reference
    [Fact]
    public void AdjustOverDegenModeAccessor_G3()
    {
        // Default value should be 1
        Assert.Equal(1, myProjector.AdjustOverDegenMode());

        // Modify through reference
        myProjector.AdjustOverDegenMode() = 0;
        Assert.Equal(0, myProjector.AdjustOverDegenMode());

        // Restore
        myProjector.AdjustOverDegenMode() = 1;
        Assert.Equal(1, myProjector.AdjustOverDegenMode());
    }

    // Test: Status method returns meaningful status after projection
    [Fact]
    public void StatusMethod_G4()
    {
        PlaneSurface aPlane = XOY();
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(10, 10, 0));

        myProjector.Init(new SurfaceAnalysis(aPlane), myTolerance);

        myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out _);

        // Status should indicate OK or DONE
        Assert.True(myProjector.Status(ExtendStatus.OK) || myProjector.Status(ExtendStatus.Done),
                    "Status should indicate OK or DONE");
    }

    //=============================================================================================
    // Tolerance and precision tests
    //=============================================================================================

    // Test: Projection with very tight tolerance on exact curve
    [Fact]
    public void TightTolerance_H1()
    {
        PlaneSurface aPlane = XOY();
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(10, 10, 0));

        myProjector.Init(new SurfaceAnalysis(aPlane), 1e-15);

        bool aResult = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out Curve2D? aPCurve);

        Assert.True(aResult, "Tight tolerance should work for exact curve");
        Assert.NotNull(aPCurve);
    }

    // Test: Projection with different endpoint tolerances
    [Fact]
    public void DifferentEndpointTolerances_H2()
    {
        PlaneSurface aPlane = XOY();

        // Curve slightly off plane at end
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(5, 5, 0.05);
        aPoles[3] = new Point3D(10, 10, 0.1);

        PointsToBSpline aBuilder = new(aPoles, 2, 2);
        BSplineCurve3D aBSpline = aBuilder.Curve();
        Assert.NotNull(aBSpline);

        myProjector.Init(new SurfaceAnalysis(aPlane), myTolerance);

        double aTolFirst = 0.01;
        double aTolLast = 0.2;
        bool aResult = myProjector.Perform(aBSpline, aBSpline.FirstParameter, aBSpline.LastParameter,
                                           out Curve2D? aPCurve, aTolFirst, aTolLast);

        Assert.True(aResult, "Projection with different endpoint tolerances should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Partial curve projection tests
    //=============================================================================================

    // Test: Project only first quarter of full circle
    [Fact]
    public void PartialCircle_I1()
    {
        PlaneSurface aPlane = XOY();

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CircleCurve3D aCircle = new(aCirc);

        SurfaceAnalysis aSAS = new(aPlane);
        myProjector.Init(aSAS, myTolerance);

        // Project only 0 to PI/2 (first quadrant)
        bool aResult = myProjector.Perform(aCircle, 0.0, Math.PI / 2.0, out Curve2D? aPCurve);

        Assert.True(aResult, "Partial projection should succeed");
        Assert.NotNull(aPCurve);

        // Verify endpoints: (5,0) to (0,5)
        Point2D aStart = aPCurve.Value(0.0);
        Point2D aEnd = aPCurve.Value(Math.PI / 2.0);
        Assert.Equal(5.0, aStart.X, 0.01);
        Assert.Equal(0.0, aStart.Y, 0.01);
        Assert.Equal(0.0, aEnd.X, 0.01);
        Assert.Equal(5.0, aEnd.Y, 0.01);
    }

    // Test: Project middle portion of B-spline
    [Fact]
    public void MiddlePortionBSpline_I2()
    {
        PlaneSurface aPlane = XOY();
        BSplineCurve3D aBSpline = createPlanarBSpline();
        Assert.NotNull(aBSpline);

        myProjector.Init(new SurfaceAnalysis(aPlane), myTolerance);

        double aFirst = aBSpline.FirstParameter;
        double aLast = aBSpline.LastParameter;
        double aRange = aLast - aFirst;

        // Project middle 50%
        bool aResult = myProjector.Perform(aBSpline, aFirst + 0.25 * aRange, aFirst + 0.75 * aRange, out Curve2D? aPCurve);

        Assert.True(aResult, "Middle portion projection should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Multiple projection and reinitialization tests
    //=============================================================================================

    // Test: Multiple projections on same surface reuse setup correctly
    [Fact]
    public void MultipleProjections_J1()
    {
        PlaneSurface aPlane = XOY();
        SurfaceAnalysis aSAS = new(aPlane);
        myProjector.Init(aSAS, myTolerance);

        // Project 5 different curves
        for (int i = 0; i < 5; ++i)
        {
            double aOffset = i * 10.0;
            TrimmedCurve3D aLine = Segment(new Point3D(aOffset, 0, 0), new Point3D(aOffset + 5, 5, 0));

            bool aResult = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out Curve2D? aPCurve);

            Assert.True(aResult, $"Projection {i} should succeed");
            Assert.NotNull(aPCurve);
        }
    }

    // Test: Surface reinitialization between projections
    [Fact]
    public void SurfaceReinitialization_J2()
    {
        // First surface
        PlaneSurface aPlane1 = XOY();
        myProjector.Init(new SurfaceAnalysis(aPlane1), myTolerance);

        TrimmedCurve3D aLine1 = Segment(new Point3D(0, 0, 0), new Point3D(10, 10, 0));
        myProjector.Perform(aLine1, aLine1.FirstParameter, aLine1.LastParameter, out Curve2D? aPCurve1);
        Assert.NotNull(aPCurve1);

        // Second surface (different plane)
        PlaneSurface aPlane2 = new(new Plane3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)));
        myProjector.Init(new SurfaceAnalysis(aPlane2), myTolerance);

        TrimmedCurve3D aLine2 = Segment(new Point3D(0, 0, 5), new Point3D(10, 10, 5));
        myProjector.Perform(aLine2, aLine2.FirstParameter, aLine2.LastParameter, out Curve2D? aPCurve2);
        Assert.NotNull(aPCurve2);
    }

    //=============================================================================================
    // Near-singularity tests (poles, degenerate regions)
    //=============================================================================================

    // Test: Project small circle near north pole of sphere
    [Fact]
    public void NearPoleOnSphere_K1()
    {
        double aRadius = 10.0;
        SphericalSurface aSphere = new(new CoordinateSystem(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))), aRadius);

        // Small circle very close to north pole (latitude = 89.4 degrees)
        double aLatitude = Math.PI / 2.0 - 0.01;
        double aZ = aRadius * Math.Sin(aLatitude);
        double aCircRadius = aRadius * Math.Cos(aLatitude);

        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, aZ), new Direction3D(0, 0, 1)), aCircRadius);
        CircleCurve3D aCircle = new(aCirc);

        myProjector.Init(new SurfaceAnalysis(aSphere), myTolerance);

        bool aResult = myProjector.Perform(aCircle, 0.0, 2.0 * Math.PI, out Curve2D? aPCurve);

        // Should succeed even near pole
        Assert.True(aResult, "Projection near pole should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Edge case tests
    //=============================================================================================

    // Test: Very short curve projection should not crash
    [Fact]
    public void VeryShortCurve_L1()
    {
        PlaneSurface aPlane = XOY();

        // Very short line segment (1e-6 length)
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(1e-6, 0, 0));

        myProjector.Init(new SurfaceAnalysis(aPlane), 1e-4);

        // Result may vary but should not crash
        _ = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out _);
    }

    // Test: Curve far from surface still projects
    [Fact]
    public void CurveFarFromSurface_L2()
    {
        PlaneSurface aPlane = XOY();

        // Line at Z=100, far from plane at Z=0
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 100), new Point3D(10, 10, 100));

        myProjector.Init(new SurfaceAnalysis(aPlane), myTolerance);

        _ = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out Curve2D? aPCurve);

        // Projection should still produce result (it projects onto the surface)
        Assert.NotNull(aPCurve);
    }

    // Test: Large tolerance projection
    [Fact]
    public void LargeTolerance_L3()
    {
        PlaneSurface aPlane = XOY();

        // Line slightly off plane
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0.1), new Point3D(10, 10, 0.1));

        myProjector.Init(new SurfaceAnalysis(aPlane), 1.0);

        bool aResult = myProjector.Perform(aLine, aLine.FirstParameter, aLine.LastParameter, out Curve2D? aPCurve);

        Assert.True(aResult, "Large tolerance projection should succeed");
        Assert.NotNull(aPCurve);
    }

    //=============================================================================================
    // Regression tests from bug reports
    //=============================================================================================

    // Test: Bug 27569 - B-spline with many knots on B-spline surface
    // This test verifies that normal B-spline curves with many knots
    // (but uniform spacing) project correctly without triggering false
    // positive in knot spacing detection
    [Fact]
    public void Bug27569_ManyKnotsBSpline_M1()
    {
        // Create a B-spline curve with many knots by manually constructing it
        // with degree 3 and 15 knots
        int aDegree = 3;
        int aNbKnots = 15;
        int aNbPoles = aNbKnots + aDegree - 1; // = 17

        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            double t = (i - 1.0) / (aNbPoles - 1.0);
            double aAngle = t * 2.0 * Math.PI;
            double aRadius = 50.0 + 20.0 * Math.Cos(3.0 * aAngle);
            aPoles[i] = new Point3D(aRadius * Math.Cos(aAngle), -30.0 * t, aRadius * Math.Sin(aAngle));
        }

        // Uniform knot vector from 0 to 1
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots[i] = (i - 1.0) / (aNbKnots - 1.0);
            aMults[i] = 1;
        }
        // Clamp end knots
        aMults[1] = aDegree + 1;
        aMults[aNbKnots] = aDegree + 1;

        BSplineCurve3D aBSplineCurve = new(aPoles, aKnots, aMults, aDegree);
        Assert.True(aBSplineCurve.KnotCount >= 10, "Curve should have at least 10 knots");

        // Create a simple plane-like surface containing the curve
        PlaneSurface aPlane = new(new Point3D(0, -15, 0), new Direction3D(0, 1, 0));

        // Test projection onto plane
        myProjector.Init(aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool aResult = myProjector.Perform(aBSplineCurve, aBSplineCurve.FirstParameter, aBSplineCurve.LastParameter,
                                           out Curve2D? aPCurve);

        Assert.True(aResult, "Projection should succeed for B-spline with many knots");
        Assert.NotNull(aPCurve);
    }

    // Test: Bug 27569 - Actual bug reproduction with high-degree B-spline curve on B-spline surface
    // This test uses a B-spline curve of degree 7 with high multiplicities and a section of
    // repeated poles (degenerate section), which is the actual problematic geometry from bug27569.
    // The curve is projected onto a B-spline surface.
    [Fact]
    public void Bug27569_HighMultiplicityBSpline_M2()
    {
        // Degree 7 B-spline with a degenerate section (repeated identical poles)
        int aDegree = 7;
        int aNbKnots = 7;

        // High multiplicities similar to bug27569 (adjusted for valid B-spline)
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        aMults[1] = 8; // degree + 1 for clamped start
        aMults[2] = 7;
        aMults[3] = 7;
        aMults[4] = 7;
        aMults[5] = 7;
        aMults[6] = 7;
        aMults[7] = 8; // degree + 1 for clamped end
        // nbPoles = sum(mults) - degree - 1 = 51 - 7 - 1 = 43

        int aNbPoles = 43;

        // Uniform knot vector
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots[i] = (i - 1); // Knots: 0, 1, 2, 3, 4, 5, 6
        }

        // Create poles with a degenerate section (repeated identical poles in the middle)
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);

        // First section: smooth curve from start
        for (int i = 1; i <= 15; ++i)
        {
            double t = (i - 1.0) / 14.0;
            double aX = 144.0 - t * 74.0; // Decreasing from 144 to 70
            double aY = -t * 0.0002;
            double aZ = 8.3 + t * 51.7; // Increasing from 8.3 to 60
            aPoles[i] = new Point3D(aX, aY, aZ);
        }

        // Middle section: degenerate (repeated identical poles) - like in bug27569
        for (int i = 16; i <= 28; ++i)
        {
            aPoles[i] = new Point3D(70.0, 0.0, 60.0); // All identical
        }

        // Last section: smooth curve to end
        for (int i = 29; i <= aNbPoles; ++i)
        {
            double t = (i - 29.0) / (aNbPoles - 29.0);
            double aX = 70.0 - t * 80.0; // Decreasing from 70 to -10
            double aY = -t * 32.0;
            double aZ = 60.0 - t * 60.0; // Decreasing from 60 to 0
            aPoles[i] = new Point3D(aX, aY, aZ);
        }

        BSplineCurve3D aBSplineCurve;
        try
        {
            aBSplineCurve = new BSplineCurve3D(aPoles, aKnots, aMults, aDegree);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            // OCCT's GTEST_SKIP: "Could not construct B-spline with high multiplicities"
            return;
        }

        // Create B-spline surface (simplified version of bug27569 surface)
        // 8x2 poles B-spline surface
        int aNbUPoles = 8;
        int aNbVPoles = 2;

        NCollection_Array2<Point3D> aSurfPoles = new(1, aNbUPoles, 1, aNbVPoles);
        // V=0 row
        aSurfPoles[1, 1] = new Point3D(70, 0, 60);
        aSurfPoles[2, 1] = new Point3D(85.01, 0, 60);
        aSurfPoles[3, 1] = new Point3D(98.87, 0, 55.07);
        aSurfPoles[4, 1] = new Point3D(111.6, 0, 46.57);
        aSurfPoles[5, 1] = new Point3D(123.15, 0, 35.68);
        aSurfPoles[6, 1] = new Point3D(133.46, 0, 23.56);
        aSurfPoles[7, 1] = new Point3D(142.46, 0, 11.32);
        aSurfPoles[8, 1] = new Point3D(150, 0, 0);
        // V=1 row (same with Y offset)
        aSurfPoles[1, 2] = new Point3D(54.99, 0, 60);
        aSurfPoles[2, 2] = new Point3D(41.13, 0, 55.07);
        aSurfPoles[3, 2] = new Point3D(28.4, 0, 46.57);
        aSurfPoles[4, 2] = new Point3D(16.85, 0, 35.68);
        aSurfPoles[5, 2] = new Point3D(6.54, 0, 23.56);
        aSurfPoles[6, 2] = new Point3D(-2.46, 0, 11.32);
        aSurfPoles[7, 2] = new Point3D(-10, 0, 0);
        aSurfPoles[8, 2] = new Point3D(-10, -35.91, 0);

        NCollection_Array1<double> aUKnots = new(1, 2);
        NCollection_Array1<double> aVKnots = new(1, 2);
        NCollection_Array1<int> aUMults = new(1, 2);
        NCollection_Array1<int> aVMults = new(1, 2);

        aUKnots[1] = 0;
        aUKnots[2] = 1;
        aVKnots[1] = 0;
        aVKnots[2] = 1;
        aUMults[1] = 8;
        aUMults[2] = 8;
        aVMults[1] = 2;
        aVMults[2] = 2;

        BSplineSurface aBSplineSurface;
        try
        {
            aBSplineSurface = new BSplineSurface(aSurfPoles, aUKnots, aVKnots, aUMults, aVMults, 7, 1);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            // Fall back to a simple plane if surface construction fails
            PlaneSurface aPlane = new(new Point3D(70, 0, 30), new Direction3D(0, 1, 0));
            myProjector.Init(aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());

            bool aResult1 = myProjector.Perform(aBSplineCurve, aBSplineCurve.FirstParameter,
                                                aBSplineCurve.LastParameter, out Curve2D? aPCurve1);

            // This test documents expected behavior - projection should succeed
            Assert.True(aResult1, "Projection should succeed");
            Assert.NotNull(aPCurve1);
            return;
        }

        myProjector.Init(aBSplineSurface, global::Kerf.Engine.Foundation.Precision.Confusion());

        bool aResult = myProjector.Perform(aBSplineCurve, aBSplineCurve.FirstParameter, aBSplineCurve.LastParameter,
                                           out Curve2D? aPCurve);

        // This is the key assertion - the projection SHOULD succeed but currently may fail
        // for curves with degenerate sections (repeated poles)
        Assert.True(aResult, "Projection should succeed for B-spline with degenerate section");
        Assert.NotNull(aPCurve);
    }
}
