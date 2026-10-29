// Port of: src/ModelingData/TKGeomBase/GTests/IntAna_IntQuadQuad_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

// The three IntAna_QuadQuadGeo tests of the suite (CylinderCylinder*) are in IntAna_IntQuadQuadTests.QuadQuadGeo.cs.
public partial class IntAna_IntQuadQuad_Test
{
    protected readonly gp_Sphere mySphere1;
    protected readonly gp_Cylinder myCylinder1;
    protected readonly gp_Cone myCone1;
    protected readonly gp_Pln myPlane1;

    public IntAna_IntQuadQuad_Test()
    {
        // Set up common geometric objects for tests
        gp_Pnt anOrigin = new(0, 0, 0);
        gp_Dir aZDir = new(gp_Dir.D.Z);
        gp_Dir anXDir = new(gp_Dir.D.X);
        gp_Ax3 anAxis = new(anOrigin, aZDir, anXDir);

        // Create sphere at origin with radius 5
        mySphere1 = new gp_Sphere(anAxis, 5.0);

        // Create cylinder with axis along Z, radius 3
        myCylinder1 = new gp_Cylinder(anAxis, 3.0);

        // Create cone with semi-angle 30 degrees
        myCone1 = new gp_Cone(anAxis, M_PI / 6, 2.0);

        // Create plane at Z = 2
        gp_Pnt aPlaneOrigin = new(0, 0, 2);
        myPlane1 = new gp_Pln(aPlaneOrigin, aZDir);
    }

    // Test basic intersection functionality - Cylinder vs Quadric (sphere)
    [Fact]
    public void CylinderVsSphereIntersection()
    {
        IntAna_Quadric aSphereQuad = new(mySphere1);

        IntAna_IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        // Should have intersection curves
        Assert.True(anIntersector.IsDone());
        Assert.False(anIntersector.IdenticalElements());

        // Should have intersection curves for cylinder-sphere case
        Assert.True(anIntersector.NbCurve() > 0);

        // Test that we can access all curves without exceptions
        for (int i = 1; i <= anIntersector.NbCurve(); i++)
        {
            // This should not throw any exceptions - just verify we can access the curve
            _ = anIntersector.Curve(i);
        }
    }

    // Test NextCurve functionality - the main test for our bug fix
    [Fact]
    public void NextCurveMethodCorrectness()
    {
        IntAna_Quadric aSphereQuad = new(mySphere1);

        IntAna_IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        Assert.True(anIntersector.IsDone());

        // Test HasNextCurve and NextCurve for valid indices
        for (int i = 1; i <= anIntersector.NbCurve(); i++)
        {
            // These methods should not crash and should return consistent results
            bool aHasNext = anIntersector.HasNextCurve(i);

            if (aHasNext) // Only test NextCurve when HasNextCurve is true
            {
                int aNextIdx = anIntersector.NextCurve(i, out _);

                // Next curve index should be valid
                Assert.True(aNextIdx > 0);
                Assert.True(aNextIdx <= anIntersector.NbCurve());
                Assert.NotEqual(i, aNextIdx); // Should not point to itself
            }
        }
    }

    // Test edge cases and boundary conditions
    [Fact]
    public void NextCurveBoundaryConditions()
    {
        IntAna_Quadric aSphereQuad = new(mySphere1);

        IntAna_IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        Assert.True(anIntersector.IsDone());

        // Test with invalid indices (should throw exceptions)
        Assert.ThrowsAny<Standard_OutOfRange>(() => anIntersector.HasNextCurve(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => anIntersector.HasNextCurve(anIntersector.NbCurve() + 1));

        // Test NextCurve on curves that don't have next curves
        for (int i = 1; i <= anIntersector.NbCurve(); i++)
        {
            if (!anIntersector.HasNextCurve(i)) // Only test exception when HasNextCurve is false
            {
                Assert.ThrowsAny<Standard_DomainError>(() => anIntersector.NextCurve(i, out _));
            }
        }
    }

    // Test specific geometric configuration that creates connected curves
    [Fact]
    public void ConnectedCurvesScenario()
    {
        // Create two spheres that intersect in a circle
        gp_Pnt aCenter1 = new(0, 0, 0);
        gp_Pnt aCenter2 = new(3, 0, 0); // Overlapping spheres
        gp_Dir aZDir = new(gp_Dir.D.Z);
        gp_Dir anXDir = new(gp_Dir.D.X);
        gp_Ax3 anAxis1 = new(aCenter1, aZDir, anXDir);
        gp_Ax3 anAxis2 = new(aCenter2, aZDir, anXDir);

        gp_Sphere aSphere1 = new(anAxis1, 2.0);
        gp_Sphere aSphere2 = new(anAxis2, 2.0);
        _ = aSphere1;

        IntAna_Quadric aSphere2Quad = new(aSphere2);

        IntAna_IntQuadQuad anIntersector = new();
        anIntersector.Perform(new gp_Cylinder(anAxis1, 2.0), aSphere2Quad, 1e-7);

        Assert.True(anIntersector.IsDone());

        Assert.False(anIntersector.IdenticalElements());

        // Verify that NextCurve method works correctly for each curve
        for (int i = 1; i <= anIntersector.NbCurve(); i++)
        {
            if (anIntersector.HasNextCurve(i)) // Only test NextCurve when HasNextCurve is true
            {
                int aNextIdx = anIntersector.NextCurve(i, out _);

                // Validate the result
                Assert.True(aNextIdx > 0);
                Assert.True(aNextIdx <= anIntersector.NbCurve());

                // Test consistency: if curve i connects to j, then j should connect back
                // (though possibly with different opposite flag)
                if (anIntersector.HasNextCurve(aNextIdx))
                {
                    int aReverseNext = anIntersector.NextCurve(aNextIdx, out _);
                    // This creates a circular reference which is geometrically possible
                    Assert.True(aReverseNext > 0);
                    Assert.True(aReverseNext <= anIntersector.NbCurve());
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

        IntAna_Quadric aSphereQuad = new(mySphere1); // Cylinder vs sphere intersection

        IntAna_IntQuadQuad anIntersector = new(myCylinder1, aSphereQuad, 1e-7);

        Assert.True(anIntersector.IsDone());

        // Test all valid curve indices
        for (int i = 1; i <= anIntersector.NbCurve(); i++)
        {
            // These calls should be consistent and not crash
            bool aHasNext = anIntersector.HasNextCurve(i);

            if (aHasNext) // Only test NextCurve when HasNextCurve is true
            {
                // This should not crash or return invalid values
                int aNextIdx = anIntersector.NextCurve(i, out _);

                Assert.True(aNextIdx > 0);
                Assert.True(aNextIdx <= anIntersector.NbCurve());
            }
        }
    }
}
