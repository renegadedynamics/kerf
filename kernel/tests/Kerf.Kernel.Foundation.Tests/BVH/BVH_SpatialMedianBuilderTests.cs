// Port of: src/FoundationClasses/TKMath/GTests/BVH_SpatialMedianBuilder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Kernel.Foundation.Tests;

public class BVH_SpatialMedianBuilderTest
{
    // Helper function to compute bounding box for a set
    private static Box<T, TVec> ComputeSetBox<T, TVec>(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> theSet)
        where T : unmanaged, System.Numerics.INumber<T>, System.Numerics.IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        Box<T, TVec> aBox = new();
        int aSize = theSet.Size();
        for (int i = 0; i < aSize; ++i)
        {
            aBox = aBox.Combined(theSet.Box(i));
        }

        return aBox;
    }

    [Fact]
    public void DefaultConstructor()
    {
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Assert.Equal(5, aBuilder.LeafNodeSize());
        Assert.Equal(32, aBuilder.MaxTreeDepth());
    }

    [Fact]
    public void CustomParameters()
    {
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new(10, 20, true);

        Assert.Equal(10, aBuilder.LeafNodeSize());
        Assert.Equal(20, aBuilder.MaxTreeDepth());
    }

    [Fact]
    public void BuildEmptySet()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.Equal(0, aTree.Length());
    }

    [Fact]
    public void BuildSingleTriangle()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.Equal(1, aTree.Length());
        Assert.True(aTree.IsOuter(0));
        Assert.Equal(1, aTree.NbPrimitives(0));
    }

    [Fact]
    public void BuildMultipleTriangles()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 10;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)i;

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.True(aTree.Length() > 0);
        Assert.True(aTree.Depth() >= 1);
    }

    [Fact]
    public void LeafNodeSizeRespected()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 20;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)i;

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        int aLeafSize = 3;
        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new(aLeafSize, 32);

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        // Check all leaf nodes respect the leaf size
        for (int i = 0; i < aTree.Length(); ++i)
        {
            if (aTree.IsOuter(i))
            {
                Assert.True(aTree.NbPrimitives(i) <= aLeafSize);
            }
        }
    }

    [Fact]
    public void MaxDepthRespected()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 100;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)i;

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        int aMaxDepth = 5;
        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new(1, aMaxDepth);

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.True(aTree.Depth() <= aMaxDepth);
    }

    [Fact]
    public void SplitsAtSpatialMedian()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 8;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)i;

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new(2, 32);

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        // For uniformly distributed data, spatial median should create balanced splits
        // Root node should have roughly equal children
        if (aTree.Length() > 0 && !aTree.IsOuter(0))
        {
            int aLeftChild  = aTree.Child(0, 0);
            int aRightChild = aTree.Child(1, 0);

            if (aLeftChild != -1 && aRightChild != -1)
            {
                int aLeftPrims  = aTree.NbPrimitives(aLeftChild);
                int aRightPrims = aTree.NbPrimitives(aRightChild);

                // For 8 triangles, expect roughly 4-4 split (allowing some tolerance)
                Assert.True(aLeftPrims >= 2);
                Assert.True(aLeftPrims <= 6);
                Assert.True(aRightPrims >= 2);
                Assert.True(aRightPrims <= 6);
            }
        }
    }

    [Fact]
    public void Build2D()
    {
        Triangulation<double, BVH_Vec2d> aTriangulation = new();
        int aNumTriangles = 10;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)i;

            aTriangulation.Vertices.Append(new BVH_Vec2d(x, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec2d(x + 0.5, 1.0));
            aTriangulation.Vertices.Append(new BVH_Vec2d(x + 1.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec2d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec2d> aBuilder = new();

        Box<double, BVH_Vec2d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.True(aTree.Length() > 0);
        Assert.True(aTree.Depth() >= 1);
    }

    [Fact]
    public void UseMainAxisFlag()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 20;

        // Create elongated distribution along X axis
        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)(i * 10);

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec3d> aTreeMainAxis = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilderMainAxis = new(5, 32, true);

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilderMainAxis.Build(aTriangulation, aTreeMainAxis, aBox);

        Assert.True(aTreeMainAxis.Length() > 0);
    }

    [Fact]
    public void CompareWithBinnedBuilder()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 50;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)i;

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        // Build with SpatialMedianBuilder (uses 2 bins)
        Tree<double, BVH_Vec3d> aTreeMedian = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilderMedian = new();
        aBuilderMedian.Build(aTriangulation, aTreeMedian, aBox);

        // Build with BinnedBuilder with 2 bins (should be identical)
        Tree<double, BVH_Vec3d> aTreeBinned = new();
        BinnedBuilder<double, BVH_Vec3d> aBuilderBinned = new(bins: 2);
        aBuilderBinned.Build(aTriangulation, aTreeBinned, aBox);

        // Both should produce trees (exact structure may vary but both should be valid)
        Assert.Equal(aTreeBinned.Length(), aTreeMedian.Length());
        Assert.Equal(aTreeBinned.Depth(), aTreeMedian.Depth());
    }

    [Fact]
    public void ClusteredData()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 30;

        // Create two clusters: 15 triangles at x=0, 15 at x=100
        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (i < 15) ? 0.0 : 100.0;
            double y = (double)(i % 15);

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, y, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, y + 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, y, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        // Spatial median should split the two clusters
        Assert.True(aTree.Length() > 0);
        Assert.True(aTree.Depth() >= 1);
    }

    [Fact]
    public void FloatPrecision()
    {
        Triangulation<float, BVH_Vec3f> aTriangulation = new();
        int aNumTriangles = 10;

        for (int i = 0; i < aNumTriangles; ++i)
        {
            float x = (float)i;

            aTriangulation.Vertices.Append(new NCollection_Vec3<float>(x, 0.0f, 0.0f));
            aTriangulation.Vertices.Append(new NCollection_Vec3<float>(x + 0.5f, 1.0f, 0.0f));
            aTriangulation.Vertices.Append(new NCollection_Vec3<float>(x + 1.0f, 0.0f, 0.0f));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<float, BVH_Vec3f> aTree = new();
        SpatialMedianBuilder<float, BVH_Vec3f> aBuilder = new();

        Box<float, BVH_Vec3f> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.True(aTree.Length() > 0);
    }

    [Fact]
    public void IdenticalBoxes()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 10;

        // All triangles at the same position
        for (int i = 0; i < aNumTriangles; ++i)
        {
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        // Should create a single leaf containing all triangles
        Assert.True(aTree.Length() > 0);
    }

    [Fact]
    public void LargeDataSet()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        int aNumTriangles = 500;

        // Random-ish distribution
        for (int i = 0; i < aNumTriangles; ++i)
        {
            double x = (double)((i * 7) % 100);
            double y = (double)((i * 11) % 100);
            double z = (double)((i * 13) % 100);

            aTriangulation.Vertices.Append(new BVH_Vec3d(x, y, z));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, y + 1.0, z));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, y, z + 1.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree<double, BVH_Vec3d> aTree = new();
        SpatialMedianBuilder<double, BVH_Vec3d> aBuilder = new();

        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);
        aBuilder.Build(aTriangulation, aTree, aBox);

        Assert.True(aTree.Length() > 0);
        Assert.True(aTree.Depth() > 5); // Large dataset should produce deeper tree

        // Estimate SAH quality
        double aSAH = aTree.EstimateSAH();
        Assert.True(aSAH > 0.0);
    }
}
