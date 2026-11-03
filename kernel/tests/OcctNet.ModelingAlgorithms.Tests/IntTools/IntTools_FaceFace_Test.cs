// Port of: src/ModelingAlgorithms/TKBO/GTests/IntTools_FaceFace_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntTools_FaceFace GTest (suite IntTools_FaceFaceTest).</summary>
public class IntTools_FaceFace_Test
{
    //! Create a half-cylinder face.
    private static TopoDS_Face CreateHalfCylinderFace(Point3D theCenter, Direction3D theAxis, double theRadius,
                                                      double theVMin, double theVMax, double theUMin = 0.0,
                                                      double theUMax = Math.PI)
    {
        // Create cylinder surface
        CoordinateSystem anAx3 = new(theCenter, theAxis);
        Geom_CylindricalSurface aCylSurf = new(anAx3, theRadius);

        // Create face with parameter bounds
        BRepBuilderAPI_MakeFace aFaceMaker = new(aCylSurf, theUMin, theUMax, theVMin, theVMax, 1e-7);
        return aFaceMaker.Face();
    }

    //! Create a full cylinder face with finite V range.
    private static TopoDS_Face CreateCylinderFace(Point3D theCenter, Direction3D theAxis, double theRadius, double theVMin,
                                                  double theVMax)
        => CreateHalfCylinderFace(theCenter, theAxis, theRadius, theVMin, theVMax, 0.0, 2.0 * Math.PI);

    //! Create a circular planar face.
    private static TopoDS_Face CreateCircularPlaneFace(Point3D theCenter, Direction3D theNormal, double theRadius)
    {
        // Create plane
        Plane3D aPln = new(theCenter, theNormal);
        Geom_Plane aPlane = new(aPln);

        // Create circular edge on the plane
        Frame3D aCircAx = new(theCenter, theNormal);
        Circle3D aCirc = new(aCircAx, theRadius);

        BRepBuilderAPI_MakeEdge anEdgeMaker = new(aCirc);
        TopoDS_Edge aCircEdge = anEdgeMaker.Edge();

        // Create wire from circular edge
        BRepBuilderAPI_MakeWire aWireMaker = new(aCircEdge);
        TopoDS_Wire aWire = aWireMaker.Wire();

        // Create face from plane with circular wire
        BRepBuilderAPI_MakeFace aFaceMaker = new(aPlane, aWire, true);
        return aFaceMaker.Face();
    }

    //! Regression: cylinder-cylinder analytical gating must not depend on argument order.
    //! Geometry is configured so that only one cylinder contributes a touching V-boundary.
    [Fact]
    public void PerpendicularCylinderBoundaryTouch_OrderIndependent()
    {
        // Cylinder 1: axis along Z, long finite segment.
        Point3D aCyl1Center = new(0.0, 0.0, 0.0);
        Direction3D aCyl1Axis = new(0.0, 0.0, 1.0);
        const double aCyl1Radius = 2.0;
        const double aCyl1VMin = -5.0;
        const double aCyl1VMax = 5.0;

        // Cylinder 2: axis along X, one V-boundary at X=0 touches cylinder 1.
        Point3D aCyl2Center = new(0.0, 2.0, 0.0);
        Direction3D aCyl2Axis = new(1.0, 0.0, 0.0);
        const double aCyl2Radius = 0.5;
        const double aCyl2VMin = 0.0;
        const double aCyl2VMax = 4.0;

        TopoDS_Face aFace1 = CreateCylinderFace(aCyl1Center, aCyl1Axis, aCyl1Radius, aCyl1VMin, aCyl1VMax);
        TopoDS_Face aFace2 = CreateCylinderFace(aCyl2Center, aCyl2Axis, aCyl2Radius, aCyl2VMin, aCyl2VMax);

        Assert.False(aFace1.IsNull(), "Failed to create first cylinder face");
        Assert.False(aFace2.IsNull(), "Failed to create second cylinder face");

        IntTools_FaceFace aFF12 = new();
        aFF12.SetParameters(true, true, true, 1.0e-7);
        aFF12.Perform(aFace1, aFace2);
        Assert.True(aFF12.IsDone(), "Intersection failed for (face1, face2)");

        IntTools_FaceFace aFF21 = new();
        aFF21.SetParameters(true, true, true, 1.0e-7);
        aFF21.Perform(aFace2, aFace1);
        Assert.True(aFF21.IsDone(), "Intersection failed for (face2, face1)");

        int aNbCurves12 = aFF12.Lines().Length();
        int aNbPoints12 = aFF12.Points().Length();
        int aNbCurves21 = aFF21.Lines().Length();
        int aNbPoints21 = aFF21.Points().Length();

        Assert.Equal(aNbCurves12, aNbCurves21); // curve count should not depend on argument order
        Assert.Equal(aNbPoints12, aNbPoints21); // point count should not depend on argument order
    }

    //! Test that a half-cylinder face and a circular plane face that don't actually
    //! intersect (cylinder is outside the circle) correctly return no intersection.
    [Fact]
    public void HalfCylinderOutsideCircularPlane_NoIntersection()
    {
        // Plane at Y = 0, with circular boundary of radius 10, centered at origin
        Point3D aPlaneCenter = new(0.0, 0.0, 0.0);
        Direction3D aPlaneNormal = new(0.0, 1.0, 0.0);
        const double aPlaneRadius = 10.0;

        // Half cylinder with axis along Y, positioned far from plane center
        Point3D aCylCenter = new(20.0, 0.0, 0.0);
        Direction3D aCylAxis = new(0.0, 1.0, 0.0);
        const double aCylRadius = 2.0;
        const double aVMin = -1.0;
        const double aVMax = 1.0;

        TopoDS_Face aCircularPlane = CreateCircularPlaneFace(aPlaneCenter, aPlaneNormal, aPlaneRadius);
        TopoDS_Face aHalfCylinder = CreateHalfCylinderFace(aCylCenter, aCylAxis, aCylRadius, aVMin, aVMax);

        Assert.False(aCircularPlane.IsNull(), "Failed to create circular plane face");
        Assert.False(aHalfCylinder.IsNull(), "Failed to create half-cylinder face");

        IntTools_FaceFace aFF = new();
        aFF.SetParameters(true, true, true, 1.0e-7);
        aFF.Perform(aCircularPlane, aHalfCylinder);

        Assert.True(aFF.IsDone(), "IntTools_FaceFace::Perform() failed");

        // The intersection should be empty because the cylinder is outside the circular boundary
        Assert.Equal(0, aFF.Lines().Length());
        Assert.Equal(0, aFF.Points().Length());
    }

    //! Test that a half-cylinder face and a circular plane face that do intersect
    //! correctly return intersection curves.
    [Fact]
    public void HalfCylinderInsideCircularPlane_HasIntersection()
    {
        Point3D aPlaneCenter = new(0.0, 0.0, 0.0);
        Direction3D aPlaneNormal = new(0.0, 1.0, 0.0);
        const double aPlaneRadius = 20.0;

        Point3D aCylCenter = new(0.0, 0.0, 0.0);
        Direction3D aCylAxis = new(0.0, 1.0, 0.0);
        const double aCylRadius = 5.0;
        const double aVMin = -1.0;
        const double aVMax = 1.0;

        TopoDS_Face aCircularPlane = CreateCircularPlaneFace(aPlaneCenter, aPlaneNormal, aPlaneRadius);
        TopoDS_Face aHalfCylinder = CreateHalfCylinderFace(aCylCenter, aCylAxis, aCylRadius, aVMin, aVMax);

        Assert.False(aCircularPlane.IsNull(), "Failed to create circular plane face");
        Assert.False(aHalfCylinder.IsNull(), "Failed to create half-cylinder face");

        IntTools_FaceFace aFF = new();
        aFF.SetParameters(true, true, true, 1.0e-7);
        aFF.Perform(aCircularPlane, aHalfCylinder);

        Assert.True(aFF.IsDone(), "IntTools_FaceFace::Perform() failed");

        // The intersection should exist - a semicircular arc
        Assert.True(aFF.Lines().Length() >= 1, "Expected at least one intersection curve (semicircular arc)");
    }

    //! Test geometry where the opposite half of the cylinder IS inside the circle.
    [Fact]
    public void OppositeHalfInsideCircle_HasIntersection()
    {
        Point3D aPlaneCenter = new(0.0, 0.0, 0.0);
        Direction3D aPlaneNormal = new(0.0, 1.0, 0.0);
        const double aPlaneRadius = 15.0;

        Point3D aCylCenter = new(10.0, 0.0, -5.0);
        Direction3D aCylAxis = new(0.0, 1.0, 0.0);
        const double aCylRadius = 3.0;
        const double aVMin = -1.0;
        const double aVMax = 1.0;

        // Create the opposite half: U in [PI, 2*PI] (bulges toward +Z, toward plane center)
        TopoDS_Face aCircularPlane = CreateCircularPlaneFace(aPlaneCenter, aPlaneNormal, aPlaneRadius);
        TopoDS_Face aHalfCylinder = CreateHalfCylinderFace(aCylCenter, aCylAxis, aCylRadius, aVMin, aVMax, Math.PI,
                                                           2.0 * Math.PI);

        Assert.False(aCircularPlane.IsNull(), "Failed to create circular plane face");
        Assert.False(aHalfCylinder.IsNull(), "Failed to create half-cylinder face");

        IntTools_FaceFace aFF = new();
        aFF.SetParameters(true, true, true, 1.0e-7);
        aFF.Perform(aCircularPlane, aHalfCylinder);

        Assert.True(aFF.IsDone(), "IntTools_FaceFace::Perform() failed");

        // This intersection SHOULD exist - the semicircle is partially inside the plane circle
        Assert.True(aFF.Lines().Length() >= 1, "Expected intersection for opposite half that bulges toward plane center.");
    }

    //! Test geometry where the cylinder arc partially crosses the circle boundary.
    [Fact]
    public void PartialCrossing_ProperlyTrimmed()
    {
        Point3D aPlaneCenter = new(0.0, 0.0, 0.0);
        Direction3D aPlaneNormal = new(0.0, 1.0, 0.0);
        const double aPlaneRadius = 15.0;

        Point3D aCylCenter = new(12.0, 0.0, -8.0);
        Direction3D aCylAxis = new(0.0, 1.0, 0.0);
        const double aCylRadius = 3.0;
        const double aVMin = -1.0;
        const double aVMax = 1.0;

        TopoDS_Face aCircularPlane = CreateCircularPlaneFace(aPlaneCenter, aPlaneNormal, aPlaneRadius);
        TopoDS_Face aHalfCylinder = CreateHalfCylinderFace(aCylCenter, aCylAxis, aCylRadius, aVMin, aVMax, 0.0,
                                                           Math.PI);

        Assert.False(aCircularPlane.IsNull(), "Failed to create circular plane face");
        Assert.False(aHalfCylinder.IsNull(), "Failed to create half-cylinder face");

        IntTools_FaceFace aFF = new();
        aFF.SetParameters(true, true, true, 1.0e-7);
        aFF.Perform(aCircularPlane, aHalfCylinder);

        Assert.True(aFF.IsDone(), "IntTools_FaceFace::Perform() failed");

        Assert.True(aFF.Lines().Length() >= 1,
                    "Wire-aware classification should produce trimmed intersection curve for partial overlap");
    }

    //! Test with cylinder positioned such that both halves are completely outside the circle.
    [Fact]
    public void BothHalvesCompletelyOutside_NoIntersection()
    {
        Point3D aPlaneCenter = new(0.0, 0.0, 0.0);
        Direction3D aPlaneNormal = new(0.0, 1.0, 0.0);
        const double aPlaneRadius = 5.0;

        Point3D aCylCenter = new(20.0, 0.0, 20.0);
        Direction3D aCylAxis = new(0.0, 1.0, 0.0);
        const double aCylRadius = 2.0;
        const double aVMin = -1.0;
        const double aVMax = 1.0;

        // Test first half
        TopoDS_Face aCircularPlane = CreateCircularPlaneFace(aPlaneCenter, aPlaneNormal, aPlaneRadius);
        TopoDS_Face aHalfCylinder1 = CreateHalfCylinderFace(aCylCenter, aCylAxis, aCylRadius, aVMin, aVMax, 0.0,
                                                            Math.PI);

        Assert.False(aCircularPlane.IsNull());
        Assert.False(aHalfCylinder1.IsNull());

        IntTools_FaceFace aFF1 = new();
        aFF1.SetParameters(true, true, true, 1.0e-7);
        aFF1.Perform(aCircularPlane, aHalfCylinder1);

        Assert.True(aFF1.IsDone());
        Assert.Equal(0, aFF1.Lines().Length()); // First half should have no intersection
        Assert.Equal(0, aFF1.Points().Length());

        // Test opposite half
        TopoDS_Face aHalfCylinder2 = CreateHalfCylinderFace(aCylCenter, aCylAxis, aCylRadius, aVMin, aVMax, Math.PI,
                                                            2.0 * Math.PI);

        Assert.False(aHalfCylinder2.IsNull());

        IntTools_FaceFace aFF2 = new();
        aFF2.SetParameters(true, true, true, 1.0e-7);
        aFF2.Perform(aCircularPlane, aHalfCylinder2);

        Assert.True(aFF2.IsDone());
        Assert.Equal(0, aFF2.Lines().Length()); // Opposite half should also have no intersection
        Assert.Equal(0, aFF2.Points().Length());
    }

    // Test OCC24005: IntTools_FaceFace should complete quickly and correctly when
    // intersecting a slightly off-angle plane with a cylinder. Previously took 7+ seconds.
    [Fact]
    public void OCC24005_PlaneCylinderIntersection()
    {
        // Hardcoded geometry from the original regression test
        Geom_Plane aPlane = new(new CoordinateSystem(new Point3D(-72.948737453424499, 754.30437716359393, 259.52151854671678),
                                           new Direction3D(6.2471473085930200e-007, -0.99999999999980493, 0.00000000000000000),
                                           new Direction3D(0.99999999999980493, 6.2471473085930200e-007, 0.00000000000000000)));

        Geom_CylindricalSurface aCylinder =
            new(new CoordinateSystem(new Point3D(-6.4812490053250649, 753.39408794522092, 279.16400974257465),
                           new Direction3D(1.0000000000000000, 0.0, 0.00000000000000000),
                           new Direction3D(0.0, 1.0000000000000000, 0.00000000000000000)),
                19.712534607908712);

        BRep_Builder aBuilder = new();
        TopoDS_Face aFace1 = new(), aFace2 = new();
        aBuilder.MakeFace(ref aFace1, aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.MakeFace(ref aFace2, aCylinder, global::Kerf.Engine.Foundation.Precision.Confusion());

        IntTools_FaceFace anInters = new();
        anInters.SetParameters(false, true, true, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInters.Perform(aFace1, aFace2);

        Assert.True(anInters.IsDone(), "IntTools_FaceFace::Perform did not complete");

        // The original test verified that intersection completes without hanging.
        // Check that we get at least one intersection curve.
        NCollection_Sequence<IntTools_Curve> aCurves = anInters.Lines();
        Assert.True(aCurves.Length() + anInters.Points().Length() >= 1,
                    "Expected at least one intersection result (curve or point)");
    }
}
