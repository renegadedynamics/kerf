// Port of: src/FoundationClasses/TKMath/GTests/BVH_QuickSorter_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class BVH_QuickSorterTest
{
    [Fact]
    public void Constructor()
    {
        BVH_QuickSorter<double, BVH_Vec3d> aSorterX = new(0);
        BVH_QuickSorter<double, BVH_Vec3d> aSorterY = new(1);
        BVH_QuickSorter<double, BVH_Vec3d> aSorterZ = new(2);

        // Constructor should not crash
        Assert.True(true);
    }

    [Fact]
    public void SortEmptySet()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();
        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);

        // Sorting empty set should not crash
        aSorter.Perform(aTriangulation);
        Assert.Equal(0, aTriangulation.Size());
    }

    [Fact]
    public void SortSingleElement()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        // Single element should remain unchanged
        Assert.Equal(1, aTriangulation.Size());
    }

    // =============================================================================
    // Sorting Correctness Tests
    // =============================================================================

    [Fact]
    public void SortAlongXAxis()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create triangles with increasing X centroids: 5, 1, 3
        // Triangle 0: centroid X = 5
        aTriangulation.Vertices.Append(new BVH_Vec3d(4.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(5.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(6.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        // Triangle 1: centroid X = 1
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(3, 4, 5, 0));

        // Triangle 2: centroid X = 3
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(3.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(4.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(6, 7, 8, 0));

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0); // Sort along X
        aSorter.Perform(aTriangulation);

        // After sorting: should be ordered 1, 3, 5 (indices 1, 2, 0)
        Assert.True(aTriangulation.Center(0, 0) < aTriangulation.Center(1, 0));
        Assert.True(aTriangulation.Center(1, 0) < aTriangulation.Center(2, 0));

        // Verify actual values
        Assert.Equal(1.0, aTriangulation.Center(0, 0), 1e-10);
        Assert.Equal(3.0, aTriangulation.Center(1, 0), 1e-10);
        Assert.Equal(5.0, aTriangulation.Center(2, 0), 1e-10);
    }

    [Fact]
    public void SortAlongYAxis()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create triangles with Y centroids: 3, 1, 2
        for (int i = 0; i < 3; ++i)
        {
            double y = (i == 0 ? 3.0 : (i == 1 ? 1.0 : 2.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, y - 0.5, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, y, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.5, y + 0.5, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(1); // Sort along Y
        aSorter.Perform(aTriangulation);

        // Should be ordered by Y centroid
        Assert.True(aTriangulation.Center(0, 1) < aTriangulation.Center(1, 1));
        Assert.True(aTriangulation.Center(1, 1) < aTriangulation.Center(2, 1));
    }

    [Fact]
    public void SortAlongZAxis()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create triangles with Z centroids: 2, 0, 1
        for (int i = 0; i < 3; ++i)
        {
            double z = (i == 0 ? 2.0 : (i == 1 ? 0.0 : 1.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, z));
            aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, z));
            aTriangulation.Vertices.Append(new BVH_Vec3d(0.5, 1.0, z));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(2); // Sort along Z
        aSorter.Perform(aTriangulation);

        // Should be ordered by Z centroid
        Assert.True(aTriangulation.Center(0, 2) < aTriangulation.Center(1, 2));
        Assert.True(aTriangulation.Center(1, 2) < aTriangulation.Center(2, 2));
    }

    [Fact]
    public void SortRangeInSet()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create 5 triangles with X centroids: 0, 1, 2, 3, 4
        for (int i = 0; i < 5; ++i)
        {
            double x = (double)i;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        // Reverse the middle 3 elements (indices 1, 2, 3) manually
        aTriangulation.Swap(1, 3);

        // Now we have centroids: 0, 3, 2, 1, 4
        Assert.Equal(3.5, aTriangulation.Center(1, 0), 1e-10);
        Assert.Equal(2.5, aTriangulation.Center(2, 0), 1e-10);
        Assert.Equal(1.5, aTriangulation.Center(3, 0), 1e-10);

        // Sort only range [1, 3] (middle 3 elements)
        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation, 1, 3);

        // Elements 0 and 4 should remain unchanged
        Assert.Equal(0.5, aTriangulation.Center(0, 0), 1e-10);
        Assert.Equal(4.5, aTriangulation.Center(4, 0), 1e-10);

        // Elements 1-3 should be sorted: 1, 2, 3
        Assert.Equal(1.5, aTriangulation.Center(1, 0), 1e-10);
        Assert.Equal(2.5, aTriangulation.Center(2, 0), 1e-10);
        Assert.Equal(3.5, aTriangulation.Center(3, 0), 1e-10);
    }

    // =============================================================================
    // Edge Cases and Stability Tests
    // =============================================================================

    [Fact]
    public void AlreadySorted()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create already sorted triangles
        for (int i = 0; i < 10; ++i)
        {
            double x = (double)i;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.9, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 0.9, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        // Should remain sorted
        for (int i = 0; i < 9; ++i)
        {
            Assert.True(aTriangulation.Center(i, 0) < aTriangulation.Center(i + 1, 0));
        }
    }

    [Fact]
    public void ReverseSorted()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create reverse sorted triangles
        for (int i = 0; i < 10; ++i)
        {
            double x = (double)(9 - i);
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.9, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 0.9, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        // Should become sorted
        for (int i = 0; i < 9; ++i)
        {
            Assert.True(aTriangulation.Center(i, 0) < aTriangulation.Center(i + 1, 0));
        }
    }

    [Fact]
    public void DuplicateValues()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create triangles with duplicate centroids: 1, 2, 2, 2, 3
        for (int i = 0; i < 5; ++i)
        {
            double x = (i == 0 ? 1.0 : (i <= 3 ? 2.0 : 3.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.9, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 0.9, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        // Should be non-decreasing
        for (int i = 0; i < 4; ++i)
        {
            Assert.True(aTriangulation.Center(i, 0) <= aTriangulation.Center(i + 1, 0));
        }
    }

    [Fact]
    public void AllSameValue()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // All triangles at same position
        for (int i = 0; i < 5; ++i)
        {
            aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(1.9, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(1.5, 0.9, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        // All should have same centroid
        for (int i = 0; i < 5; ++i)
        {
            Assert.Equal(1.467, aTriangulation.Center(i, 0), 1e-2);
        }
    }

    [Fact]
    public void LargeDataSet()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Create 1000 triangles with random-ish order
        for (int i = 0; i < 1000; ++i)
        {
            // Use simple pseudo-random pattern
            double x = (double)((i * 37) % 1000);
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.9, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 0.9, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        // Verify sorted
        for (int i = 0; i < 999; ++i)
        {
            Assert.True(aTriangulation.Center(i, 0) <= aTriangulation.Center(i + 1, 0));
        }
    }

    [Fact]
    public void TwoElements()
    {
        BVH_Triangulation<double, BVH_Vec3d> aTriangulation = new();

        // Two elements out of order
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(3.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.5, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.5, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(3, 4, 5, 0));

        BVH_QuickSorter<double, BVH_Vec3d> aSorter = new(0);
        aSorter.Perform(aTriangulation);

        Assert.True(aTriangulation.Center(0, 0) < aTriangulation.Center(1, 0));
    }
}
