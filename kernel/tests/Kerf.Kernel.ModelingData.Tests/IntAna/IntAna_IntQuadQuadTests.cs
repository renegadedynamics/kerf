// Port of: src/ModelingData/TKGeomBase/GTests/IntAna_IntQuadQuad_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// The three IntAna_QuadQuadGeo tests of the suite (CylinderCylinder*) are in IntAna_IntQuadQuadTests.QuadQuadGeo.cs.
public partial class IntAna_IntQuadQuad_Test
{
    protected readonly Sphere mySphere1;
    protected readonly Cylinder myCylinder1;
    protected readonly Cone myCone1;
    protected readonly Plane3D myPlane1;

    public IntAna_IntQuadQuad_Test()
    {
        // Set up common geometric objects for tests
        Point3D anOrigin = new(0, 0, 0);
        Direction3D aZDir = new(Direction3D.D.Z);
        Direction3D anXDir = new(Direction3D.D.X);
        CoordinateSystem anAxis = new(anOrigin, aZDir, anXDir);

        // Create sphere at origin with radius 5
        mySphere1 = new Sphere(anAxis, 5.0);

        // Create cylinder with axis along Z, radius 3
        myCylinder1 = new Cylinder(anAxis, 3.0);

        // Create cone with semi-angle 30 degrees
        myCone1 = new Cone(anAxis, M_PI / 6, 2.0);

        // Create plane at Z = 2
        Point3D aPlaneOrigin = new(0, 0, 2);
        myPlane1 = new Plane3D(aPlaneOrigin, aZDir);
    }

    // Test basic intersection functionality - Cylinder vs Quadric (sphere)
    [Fact]
    public void CylinderVsSphereIntersection()
    {
        Quadric aSphereQuad = new(mySphere1);

        IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        // Should have intersection curves
        Assert.True(anIntersector.IsDone);
        Assert.False(anIntersector.IdenticalElements());

        // Should have intersection curves for cylinder-sphere case
        Assert.True(anIntersector.CurveCount > 0);

        // Test that we can access all curves without exceptions
        for (int i = 1; i <= anIntersector.CurveCount; i++)
        {
            // This should not throw any exceptions - just verify we can access the curve
            _ = anIntersector.Curve(i);
        }
    }

    // Test NextCurve functionality - the main test for our bug fix
    [Fact]
    public void NextCurveMethodCorrectness()
    {
        Quadric aSphereQuad = new(mySphere1);

        IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        Assert.True(anIntersector.IsDone);

        // Test HasNextCurve and NextCurve for valid indices
        for (int i = 1; i <= anIntersector.CurveCount; i++)
        {
            // These methods should not crash and should return consistent results
            bool aHasNext = anIntersector.HasNextCurve(i);

            if (aHasNext) // Only test NextCurve when HasNextCurve is true
            {
                int aNextIdx = anIntersector.NextCurve(i, out _);

                // Next curve index should be valid
                Assert.True(aNextIdx > 0);
                Assert.True(aNextIdx <= anIntersector.CurveCount);
                Assert.NotEqual(i, aNextIdx); // Should not point to itself
            }
        }
    }

    // Test edge cases and boundary conditions
    [Fact]
    public void NextCurveBoundaryConditions()
    {
        Quadric aSphereQuad = new(mySphere1);

        IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        Assert.True(anIntersector.IsDone);

        // Test with invalid indices (should throw exceptions)
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anIntersector.HasNextCurve(0));
        EngineAssert.ThrowsAny<OutOfRangeException>(() => anIntersector.HasNextCurve(anIntersector.CurveCount + 1));

        // Test NextCurve on curves that don't have next curves
        for (int i = 1; i <= anIntersector.CurveCount; i++)
        {
            if (!anIntersector.HasNextCurve(i)) // Only test exception when HasNextCurve is false
            {
                EngineAssert.ThrowsAny<DomainErrorException>(() => anIntersector.NextCurve(i, out _));
            }
        }
    }

    // Test specific geometric configuration that creates connected curves
    [Fact]
    public void ConnectedCurvesScenario()
    {
        // Create two spheres that intersect in a circle
        Point3D aCenter1 = new(0, 0, 0);
        Point3D aCenter2 = new(3, 0, 0); // Overlapping spheres
        Direction3D aZDir = new(Direction3D.D.Z);
        Direction3D anXDir = new(Direction3D.D.X);
        CoordinateSystem anAxis1 = new(aCenter1, aZDir, anXDir);
        CoordinateSystem anAxis2 = new(aCenter2, aZDir, anXDir);

        Sphere aSphere1 = new(anAxis1, 2.0);
        Sphere aSphere2 = new(anAxis2, 2.0);
        _ = aSphere1;

        Quadric aSphere2Quad = new(aSphere2);

        IntQuadQuad anIntersector = new();
        anIntersector.Perform(new Cylinder(anAxis1, 2.0), aSphere2Quad, 1e-7);

        Assert.True(anIntersector.IsDone);

        Assert.False(anIntersector.IdenticalElements());

        // Verify that NextCurve method works correctly for each curve
        for (int i = 1; i <= anIntersector.CurveCount; i++)
        {
            if (anIntersector.HasNextCurve(i)) // Only test NextCurve when HasNextCurve is true
            {
                int aNextIdx = anIntersector.NextCurve(i, out _);

                // Validate the result
                Assert.True(aNextIdx > 0);
                Assert.True(aNextIdx <= anIntersector.CurveCount);

                // Test consistency: if curve i connects to j, then j should connect back
                // (though possibly with different opposite flag)
                if (anIntersector.HasNextCurve(aNextIdx))
                {
                    int aReverseNext = anIntersector.NextCurve(aNextIdx, out _);
                    // This creates a circular reference which is geometrically possible
                    Assert.True(aReverseNext > 0);
                    Assert.True(aReverseNext <= anIntersector.CurveCount);
                }
            }
        }
    }

    // Test the fix specifically - ensure we don't access out-of-bounds memory
    [Fact]
    public void IndexingConsistencyTest()
    {
        // This test specifically validates that HasNextCurve and NextCurve
        // use consistent indexing (both should use I-1 for 0-based arrays)

        Quadric aSphereQuad = new(mySphere1); // Cylinder vs sphere intersection

        IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        Assert.True(anIntersector.IsDone);

        // Test all valid curve indices
        for (int i = 1; i <= anIntersector.CurveCount; i++)
        {
            // These calls should be consistent and not crash
            bool aHasNext = anIntersector.HasNextCurve(i);

            if (aHasNext) // Only test NextCurve when HasNextCurve is true
            {
                // This should not crash or return invalid values
                int aNextIdx = anIntersector.NextCurve(i, out _);

                Assert.True(aNextIdx > 0);
                Assert.True(aNextIdx <= anIntersector.CurveCount);
            }
        }
    }
}
