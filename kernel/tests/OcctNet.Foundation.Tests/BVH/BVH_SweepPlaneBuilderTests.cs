// Port of: src/FoundationClasses/TKMath/GTests/BVH_SweepPlaneBuilder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace OcctNet.Foundation.Tests;

public class BVH_SweepPlaneBuilderTest
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
        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new();
        Assert.Equal(BVH_Constants_LeafNodeSizeDefault, aBuilder.LeafNodeSize());
        Assert.Equal(BVH_Constants_MaxTreeDepth, aBuilder.MaxTreeDepth());
    }

    [Fact]
    public void CustomParameters()
    {
        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(5, 20, 1);
        Assert.Equal(5, aBuilder.LeafNodeSize());
        Assert.Equal(20, aBuilder.MaxTreeDepth());
    }

    // =============================================================================
    // BVH Building Tests
    // =============================================================================

    [Fact]
    public void BuildEmptyTriangulation()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();
        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new();

        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = new();

        aBuilder.Build(aTriangulation, aBVH, aBox);

        Assert.Equal(0, aBVH.Length());
    }

    [Fact]
    public void BuildSingleTriangle()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Add vertices
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));

        // Add triangle
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(1, 32, 1);
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        // Single triangle should create one leaf node
        Assert.Equal(1, aBVH.Length());
        Assert.True(aBVH.IsOuter(0));
        Assert.Equal(0, aBVH.BegPrimitive(0));
        Assert.Equal(0, aBVH.EndPrimitive(0));
    }

    [Fact]
    public void BuildTwoTriangles()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create two triangles forming a quad
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 1.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));

        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 2, 3, 0));

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(1, 32, 1);
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        // Two triangles with leaf size 1 should create internal nodes
        Assert.True(aBVH.Length() > 2);
        Assert.True(aBVH.Depth() >= 1);
    }

    [Fact]
    public void BuildMultipleTriangles_Grid()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create a 2x2 grid of quads (8 triangles)
        int aGridSize = 3;
        for (int i = 0; i < aGridSize; ++i)
        {
            for (int j = 0; j < aGridSize; ++j)
            {
                aTriangulation.Vertices.Append(new BVH_Vec3d((double)i, (double)j, 0.0));
            }
        }

        // Add triangles for the grid
        for (int i = 0; i < aGridSize - 1; ++i)
        {
            for (int j = 0; j < aGridSize - 1; ++j)
            {
                int aV0 = i * aGridSize + j;
                int aV1 = i * aGridSize + (j + 1);
                int aV2 = (i + 1) * aGridSize + (j + 1);
                int aV3 = (i + 1) * aGridSize + j;

                aTriangulation.Elements.Append(new BVH_Vec4i(aV0, aV1, aV2, 0));
                aTriangulation.Elements.Append(new BVH_Vec4i(aV0, aV2, aV3, 0));
            }
        }

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(2, 32, 1);
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        int aTotalTriangles = (aGridSize - 1) * (aGridSize - 1) * 2;
        Assert.True(aBVH.Length() > 0);
        Assert.True(aBVH.Depth() >= 1);

        // Verify tree bounds contain all triangles
        Box<double, BVH_Vec3d> aRootBox = new(aBVH.MinPoint(0), aBVH.MaxPoint(0));
        for (int i = 0; i < aTotalTriangles; ++i)
        {
            Box<double, BVH_Vec3d> aTriBox = aTriangulation.Box(i);
            Assert.False(aRootBox.IsOut(aTriBox));
        }
    }

    [Fact]
    public void SplitAlongDifferentAxes()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create triangles spread along X axis (should prefer X-axis split)
        for (int i = 0; i < 10; ++i)
        {
            double x = (double)i * 2.0;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));

            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(1, 32, 1);
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        // Should create splits along X axis
        Assert.True(aBVH.Length() > 1);
        Assert.True(aBVH.Depth() >= 1);
    }

    [Fact]
    public void DegenerateCase_AllSamePosition()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // All triangles at same position (degenerate case)
        for (int i = 0; i < 5; ++i)
        {
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.1, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.1, 0.0));

            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(1, 32, 1);
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        // Should still build valid tree even with degenerate geometry
        Assert.True(aBVH.Length() > 0);
    }

    // =============================================================================
    // SAH Quality Tests
    // =============================================================================

    [Fact]
    public void SAHQuality_VerifyBetter()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create well-separated triangle groups (should result in good SAH)
        // Group 1: left side
        for (int i = 0; i < 5; ++i)
        {
            double x = (double)i * 0.5;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.4, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.2, 0.4, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        // Group 2: right side (spatially separated)
        for (int i = 0; i < 5; ++i)
        {
            double x = 10.0 + (double)i * 0.5;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.4, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.2, 0.4, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i((5 + i) * 3, (5 + i) * 3 + 1, (5 + i) * 3 + 2, 0));
        }

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(2, 32, 1);
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        // Spatially separated groups should create clear split
        Assert.True(aBVH.Depth() >= 1);

        // Root should have two children covering separate regions
        if (!aBVH.IsOuter(0))
        {
            int aLeftChild  = aBVH.Child(0, 0);
            int aRightChild = aBVH.Child(1, 0);

            Box<double, BVH_Vec3d> aLeftBox = new(aBVH.MinPoint(aLeftChild), aBVH.MaxPoint(aLeftChild));
            Box<double, BVH_Vec3d> aRightBox = new(aBVH.MinPoint(aRightChild), aBVH.MaxPoint(aRightChild));

            // Boxes should not significantly overlap for well-separated geometry
            double aOverlap = 0.0;
            for (int i = 0; i < 3; ++i)
            {
                double aMin = Max(aLeftBox.CornerMin()[i], aRightBox.CornerMin()[i]);
                double aMax = Min(aLeftBox.CornerMax()[i], aRightBox.CornerMax()[i]);
                if (aMax > aMin)
                {
                    aOverlap += (aMax - aMin);
                }
            }

            // Overlap should be minimal for well-separated groups
            double aTotalSize = (aBox.CornerMax()[0] - aBox.CornerMin()[0]);
            Assert.True(aOverlap < aTotalSize * 0.1); // Less than 10% overlap
        }
    }

    // =============================================================================
    // Leaf Size Tests
    // =============================================================================

    [Fact]
    public void LeafSize_RespectMaxSize()
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create 20 triangles
        for (int i = 0; i < 20; ++i)
        {
            double x = (double)i;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.9, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 0.9, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        SweepPlaneBuilder<double, BVH_Vec3d> aBuilder = new(5, 32, 1); // Leaf size = 5
        Tree<double, BVH_Vec3d> aBVH = new();
        Box<double, BVH_Vec3d> aBox = ComputeSetBox(aTriangulation);

        aBuilder.Build(aTriangulation, aBVH, aBox);

        // Verify leaf nodes don't exceed the specified size
        for (int i = 0; i < aBVH.Length(); ++i)
        {
            if (aBVH.IsOuter(i))
            {
                int aLeafSize = aBVH.EndPrimitive(i) - aBVH.BegPrimitive(i) + 1;
                Assert.True(aLeafSize <= 5);
            }
        }
    }
}
