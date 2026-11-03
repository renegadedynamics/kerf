// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntPatch_Polyhedron_Test.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntPatch_PolyhedronBVH_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bvh;

using Kerf.Engine.Intersection.Patches;
using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntPatch_Polyhedron GTest.</summary>
public class IntPatch_PolyhedronTest
{
    private static SurfaceAdaptor makePlane()
    {
        CoordinateSystem anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        PlaneSurface aPlane = new(anAxis);
        return new SurfaceAdaptor(aPlane, -1.0, 1.0, -1.0, 1.0);
    }

    private static SurfaceAdaptor makeSphere()
    {
        CoordinateSystem anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        SphericalSurface aSphere = new(anAxis, 1.0);
        return new SurfaceAdaptor(aSphere);
    }

    // Basic construction with auto-computed subdivision.
    [Fact]
    public void DefaultConstructor_ProducesValidMesh()
    {
        SurfaceAdaptor aSurf = makeSphere();
        Polyhedron aPoly = new(aSurf);

        aPoly.Size(out int nbU, out int nbV);
        Assert.True(nbU > 0);
        Assert.True(nbV > 0);
        Assert.True(aPoly.TriangleCount > 0);
        Assert.True(aPoly.PointCount > 0);
    }

    // Regression test for bug #51: zero nbu/nbv must not cause division-by-zero.
    // Constructor clamps to minimum 1.
    [Fact]
    public void ZeroSubdivision_ClampedToMinimum()
    {
        SurfaceAdaptor aSurf = makePlane();

        Polyhedron aPoly = new(aSurf, 0, 0);
        aPoly.Size(out int nbU, out int nbV);
        Assert.True(nbU >= 1);
        Assert.True(nbV >= 1);
        Assert.True(aPoly.TriangleCount > 0);
    }

    // Small subdivision values should work normally.
    [Fact]
    public void SmallSubdivision_ProducesValidMesh()
    {
        SurfaceAdaptor aSurf = makePlane();
        Polyhedron aPoly = new(aSurf, 2, 2);

        aPoly.Size(out int nbU, out int nbV);
        Assert.Equal(2, nbU);
        Assert.Equal(2, nbV);
        Assert.Equal(2 * 2 * 2, aPoly.TriangleCount);
    }

    // Regression test for bug #53: TriConnex must not crash when Pedge == 0.
    [Fact]
    public void TriConnex_PedgeZero_NoCrash()
    {
        SurfaceAdaptor aSurf = makeSphere();
        Polyhedron aPoly = new(aSurf, 4, 4);

        // Get a valid triangle and pivot point.
        aPoly.Triangle(1, out int aP1, out _, out _);

        // Call TriConnex with Pedge = 0 (unknown edge mode).
        int aResult = aPoly.TriConnex(1, aP1, 0, out _, out _);
        // Must not crash; result is a valid triangle index or 0.
        Assert.True(aResult >= 0);
    }

    // TriConnex with various Pedge values must not crash.
    [Fact]
    public void TriConnex_AllVertices_NoCrash()
    {
        SurfaceAdaptor aSurf = makeSphere();
        Polyhedron aPoly = new(aSurf, 3, 3);

        aPoly.Triangle(1, out int aP1, out int aP2, out int aP3);

        // Call TriConnex with each vertex as Pedge and with Pedge=0.
        aPoly.TriConnex(1, aP1, 0, out _, out _);
        aPoly.TriConnex(1, aP1, aP2, out _, out _);
        aPoly.TriConnex(1, aP1, aP3, out _, out _);
        aPoly.TriConnex(1, aP2, aP3, out _, out _);
        // All calls must complete without crash.
    }
}

/// <summary>OCCT's IntPatch_PolyhedronBVH GTest (its fixture IntPatch_PolyhedronBVHTest).</summary>
public class IntPatch_PolyhedronBVHTest
{
    private readonly SurfaceAdaptor mySphereAdaptor;
    private readonly SurfaceAdaptor myCylAdaptor;

    public IntPatch_PolyhedronBVHTest()
    {
        // Create a sphere surface
        Sphere aSphere = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        SphericalSurface aSphereSurf = new(aSphere);
        mySphereAdaptor = new SurfaceAdaptor(aSphereSurf);

        // Create a cylinder surface
        Cylinder aCyl = new(new CoordinateSystem(new Point3D(0.5, 0, 0), new Direction3D(0, 0, 1)), 0.8);
        CylindricalSurface aCylSurf = new(aCyl);
        myCylAdaptor = new SurfaceAdaptor(aCylSurf, 0, 2 * M_PI, -1, 1);
    }

    // Test that PolyhedronBVH can be constructed and has correct size
    [Fact]
    public void Construction()
    {
        const int aNbU = 10;
        const int aNbV = 10;

        Polyhedron aPoly = new(mySphereAdaptor, aNbU, aNbV);
        PolyhedronBvh aBVH = new(aPoly);

        Assert.True(aBVH.IsInitialized);
        Assert.True(aBVH.Size() > 0);
        Assert.True(aBVH.Size() <= PolyhedronTool.NbTriangles(aPoly));
    }

    // Test that Box() returns valid bounding boxes
    [Fact]
    public void Box()
    {
        const int aNbU = 5;
        const int aNbV = 5;

        Polyhedron aPoly = new(mySphereAdaptor, aNbU, aNbV);
        PolyhedronBvh aBVH = new(aPoly);

        // Check that all boxes are valid
        for (int i = 0; i < aBVH.Size(); ++i)
        {
            Box<double, NCollection_Vec3<double>> aBox = aBVH.Box(i);
            Assert.True(aBox.IsValid, $"Box {i} is not valid");
        }
    }

    // Test that Center() returns valid centroid coordinates
    [Fact]
    public void Center()
    {
        const int aNbU = 5;
        const int aNbV = 5;

        Polyhedron aPoly = new(mySphereAdaptor, aNbU, aNbV);
        PolyhedronBvh aBVH = new(aPoly);

        // Check that centroids are within the overall bounding box
        Box aBounding = PolyhedronTool.Bounding(aPoly);
        aBounding.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax,
                      out double aZmax);

        for (int i = 0; i < aBVH.Size(); ++i)
        {
            double aCx = aBVH.Center(i, 0);
            double aCy = aBVH.Center(i, 1);
            double aCz = aBVH.Center(i, 2);

            Assert.True(aCx >= aXmin - 1e-10, $"Center X of triangle {i} is below min");
            Assert.True(aCx <= aXmax + 1e-10, $"Center X of triangle {i} is above max");
            Assert.True(aCy >= aYmin - 1e-10, $"Center Y of triangle {i} is below min");
            Assert.True(aCy <= aYmax + 1e-10, $"Center Y of triangle {i} is above max");
            Assert.True(aCz >= aZmin - 1e-10, $"Center Z of triangle {i} is below min");
            Assert.True(aCz <= aZmax + 1e-10, $"Center Z of triangle {i} is above max");
        }
    }

    // Test that OriginalIndex() returns valid 1-based indices
    [Fact]
    public void OriginalIndex()
    {
        const int aNbU = 5;
        const int aNbV = 5;

        Polyhedron aPoly = new(mySphereAdaptor, aNbU, aNbV);
        PolyhedronBvh aBVH = new(aPoly);

        int aNbTri = aBVH.Size();
        int aNbPolyTri = PolyhedronTool.NbTriangles(aPoly);

        // Before BVH build, indices should be sequential
        for (int i = 0; i < aNbTri; ++i)
        {
            int anOrigIdx = aBVH.OriginalIndex(i);
            Assert.True(anOrigIdx >= 1, "Original index should be >= 1");
            Assert.True(anOrigIdx <= aNbPolyTri, "Original index should be <= NbTriangles");
        }

        // Force BVH build
        aBVH.BVH();

        // After BVH build, each original index should appear exactly once
        bool[] aUsed = new bool[aNbPolyTri + 1];
        for (int i = 0; i < aNbTri; ++i)
        {
            int anOrigIdx = aBVH.OriginalIndex(i);
            Assert.False(aUsed[anOrigIdx], $"Original index {anOrigIdx} used more than once");
            aUsed[anOrigIdx] = true;
        }
    }

    // Test BVH traversal finds overlapping triangles
    [Fact]
    public void Traversal()
    {
        const int aNbU = 10;
        const int aNbV = 10;

        Polyhedron aPoly1 = new(mySphereAdaptor, aNbU, aNbV);
        Polyhedron aPoly2 = new(myCylAdaptor, aNbU, aNbV);

        PolyhedronBvh aSet1 = new(aPoly1);
        PolyhedronBvh aSet2 = new(aPoly2);

        BvhTraversal aTraversal = new();
        int aNbPairs = aTraversal.Perform(aSet1, aSet2, false);

        // The sphere and cylinder should have some overlapping triangles
        Assert.True(aNbPairs > 0, "Expected some overlapping triangle pairs");
        Assert.Equal(aNbPairs, aTraversal.Pairs().Size());

        // Verify all pairs have valid indices
        foreach (BvhTraversal.TrianglePair aPair in aTraversal.Pairs())
        {
            Assert.True(aPair.First >= 1);
            Assert.True(aPair.First <= PolyhedronTool.NbTriangles(aPoly1));
            Assert.True(aPair.Second >= 1);
            Assert.True(aPair.Second <= PolyhedronTool.NbTriangles(aPoly2));
        }
    }

    // Test self-interference mode
    [Fact]
    public void SelfInterference()
    {
        const int aNbU = 5;
        const int aNbV = 5;

        Polyhedron aPoly = new(mySphereAdaptor, aNbU, aNbV);
        PolyhedronBvh aSet = new(aPoly);

        BvhTraversal aTraversal = new();
        aTraversal.Perform(aSet, aSet, true); // self-interference mode

        // In self-interference mode, First < Second for all pairs
        foreach (BvhTraversal.TrianglePair aPair in aTraversal.Pairs())
        {
            Assert.True(aPair.First < aPair.Second, "Self-interference should have First < Second");
        }
    }

    // Test that IntPatch_InterferencePolyhedron works with BVH
    [Fact]
    public void InterferencePolyhedron()
    {
        const int aNbU = 10;
        const int aNbV = 10;

        Polyhedron aPoly1 = new(mySphereAdaptor, aNbU, aNbV);
        Polyhedron aPoly2 = new(myCylAdaptor, aNbU, aNbV);

        // Create interference and check it completes without error
        InterferencePolyhedron anInterf = new(aPoly1, aPoly2);

        // The result should have some section points or lines
        // (exact number depends on geometry, just verify it runs)
        int aNbPoints = anInterf.SectionPointCount;
        int aNbLines = anInterf.SectionLineCount;
        int aNbZones = anInterf.TangentZoneCount;

        // At least some intersection should be found
        Assert.True(aNbPoints > 0 || aNbLines > 0 || aNbZones > 0, "Expected some intersection results");
    }

    // Test with non-overlapping surfaces
    [Fact]
    public void NoOverlap()
    {
        // Create a plane far from the sphere
        Plane3D aPlane = new(new Point3D(10, 10, 10), new Direction3D(1, 0, 0));
        PlaneSurface aPlaneSurf = new(aPlane);
        SurfaceAdaptor aPlaneAdaptor = new(aPlaneSurf, -1, 1, -1, 1);

        const int aNbU = 5;
        const int aNbV = 5;

        Polyhedron aPoly1 = new(mySphereAdaptor, aNbU, aNbV);
        Polyhedron aPoly2 = new(aPlaneAdaptor, aNbU, aNbV);

        PolyhedronBvh aSet1 = new(aPoly1);
        PolyhedronBvh aSet2 = new(aPoly2);

        BvhTraversal aTraversal = new();
        int aNbPairs = aTraversal.Perform(aSet1, aSet2, false);

        // Far-away surfaces should have no overlapping boxes
        Assert.True(aNbPairs == 0, "Expected no overlapping triangle pairs for distant surfaces");
    }
}
