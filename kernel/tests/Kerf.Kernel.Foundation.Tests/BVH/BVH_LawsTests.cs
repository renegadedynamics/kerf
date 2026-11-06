// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Geometry.Bvh;
using Box3d = Kerf.Engine.Geometry.Bvh.Box<double, Kerf.Engine.NCollection_Vec3<double>>;
using Tri3d = Kerf.Engine.Geometry.Bvh.Triangulation<double, Kerf.Engine.NCollection_Vec3<double>>;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// The BVH classes the kernel doesn't reach (decision 19805), by what they must give: the distance field of a closed
/// mesh is the distance to the solid it bounds, whichever triangle is nearest and however the nearest point lies on it;
/// a quad tree holds the same leaves as the binary tree it collapses.
/// </summary>
public class BVH_LawsTests
{
    private sealed class Geometry : BvhGeometry<double, BVH_Vec3d>
    {
    }

    // A cube from (0, 0, 0) to (s, s, s), as 12 triangles facing outwards
    private static Tri3d Cube(double s)
    {
        Tri3d aMesh = new();
        for (int i = 0; i < 8; i++)
        {
            aMesh.Vertices.Append(new BVH_Vec3d((i & 1) * s, ((i >> 1) & 1) * s, ((i >> 2) & 1) * s));
        }
        int[] aQuads =
        [
            0, 2, 3, 1, // z = 0, down
            4, 5, 7, 6, // z = s, up
            0, 1, 5, 4, // y = 0
            2, 6, 7, 3, // y = s
            0, 4, 6, 2, // x = 0
            1, 3, 7, 5, // x = s
        ];
        for (int q = 0; q < 6; q++)
        {
            int a = aQuads[4 * q], b = aQuads[(4 * q) + 1], c = aQuads[(4 * q) + 2], d = aQuads[(4 * q) + 3];
            aMesh.Elements.Append(new BVH_Vec4i(a, b, c, 0));
            aMesh.Elements.Append(new BVH_Vec4i(a, c, d, 0));
        }
        aMesh.MarkDirty();
        return aMesh;
    }

    private static double DistanceToCube(double x, double y, double z, double s)
    {
        double dx = Math.Max(Math.Max(-x, x - s), 0), dy = Math.Max(Math.Max(-y, y - s), 0), dz = Math.Max(Math.Max(-z, z - s), 0);
        double anOutside = Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
        if (anOutside > 0)
        {
            return anOutside;
        }
        return Math.Min(Math.Min(Math.Min(x, s - x), Math.Min(y, s - y)), Math.Min(z, s - z));
    }

    private static bool IsInCube(double x, double y, double z, double s) =>
        x > 0 && x < s && y > 0 && y < s && z > 0 && z < s;

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, false)]
    [InlineData(true, true)]
    public void The_distance_field_of_a_closed_cube_is_the_distance_to_it(bool theSigned, bool theParallel)
    {
        const double aSide = 2;
        var aGeometry = new Geometry();
        aGeometry.Objects().Add(Cube(aSide));
        aGeometry.MarkDirty();
        var aField = new DistanceField<double, BVH_Vec3d>(16, theSigned);
        aField.SetParallel(theParallel);
        Assert.Equal(theParallel, aField.IsParallel);
        Assert.True(aField.Build(aGeometry));
        Assert.Equal(16, aField.DimensionX());
        Assert.Equal(16, aField.DimensionY());
        Assert.Equal(16, aField.DimensionZ());
        Assert.Equal(16 * 16 * 16, aField.PackedData().Length);
        BVH_Vec3d aMin = aField.CornerMin(), aSize = aField.VoxelSize();
        Assert.True(aMin[0] < 0 && aField.CornerMax()[0] > aSide);
        for (int k = 0; k < 16; k++)
        {
            for (int j = 0; j < 16; j++)
            {
                for (int i = 0; i < 16; i++)
                {
                    double x = aMin[0] + (aSize[0] * (i + 0.5)), y = aMin[1] + (aSize[1] * (j + 0.5)), z = aMin[2] + (aSize[2] * (k + 0.5));
                    double anExpected = DistanceToCube(x, y, z, aSide);
                    double aVoxel = aField.Voxel(i, j, k);
                    Assert.Equal(anExpected, Math.Abs(aVoxel), 9);
                    if (theSigned && anExpected > 1e-9)
                    {
                        Assert.Equal(IsInCube(x, y, z, aSide), aVoxel < 0);
                    }
                }
            }
        }
    }

    [Fact]
    public void The_distance_field_of_nothing_is_not_built()
    {
        var aField = new DistanceField<double, BVH_Vec3d>(16, false);
        Assert.False(aField.Build(new Geometry()));
    }

    [Fact]
    public void A_flat_mesh_is_widened_to_at_least_16_voxels_on_every_axis()
    {
        Tri3d aFlat = new();
        aFlat.Vertices.Append(new BVH_Vec3d(0, 0, 0));
        aFlat.Vertices.Append(new BVH_Vec3d(4, 0, 0));
        aFlat.Vertices.Append(new BVH_Vec3d(0, 1, 0.5));
        aFlat.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));
        aFlat.MarkDirty();
        var aGeometry = new Geometry();
        aGeometry.Objects().Add(aFlat);
        aGeometry.MarkDirty();
        var aField = new DistanceField<double, BVH_Vec3d>(32, false);
        Assert.True(aField.Build(aGeometry));
        Assert.Equal(32, aField.DimensionX());
        Assert.Equal(16, aField.DimensionY());
        Assert.Equal(16, aField.DimensionZ());
    }

    [Fact]
    public void Geometry_keeps_its_builder_and_its_box_covers_its_objects()
    {
        var aBuilder = new BinnedBuilder<double, BVH_Vec3d>(1, 32);
        var aGeometry = new Geometry();
        aGeometry.SetBuilder(aBuilder);
        Assert.Same(aBuilder, aGeometry.Builder());
        aGeometry.Objects().Add(Cube(1));
        Tri3d aFar = Cube(1);
        aFar.Vertices.Append(new BVH_Vec3d(5, 5, 5));
        aFar.Elements.Append(new BVH_Vec4i(0, 1, 8, 0));
        aFar.MarkDirty();
        aGeometry.Objects().Add(aFar);
        aGeometry.MarkDirty();
        Assert.True(aGeometry.IsDirty);
        Box3d aBox = aGeometry.Box();
        Assert.Equal(5, aBox.CornerMax()[0], 12);
        Assert.Equal(0, aBox.CornerMin()[0], 12);
        Assert.NotNull(aGeometry.BVH());
        Assert.False(aGeometry.IsDirty);
        Assert.Equal(2, aGeometry.Size());
        var aWithBuilder = new BvhGeometryWithBuilder(aBuilder);
        Assert.Same(aBuilder, aWithBuilder.Builder());
    }

    private sealed class BvhGeometryWithBuilder(Builder<double, BVH_Vec3d> theBuilder) : BvhGeometry<double, BVH_Vec3d>(theBuilder)
    {
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(4)]
    public void A_quad_tree_keeps_the_leaves_and_boxes_of_its_binary_tree(int theLeafSize)
    {
        Tri3d aMesh = Cube(1);
        for (int i = 0; i < 20; i++)
        {
            int aFirst = aMesh.Vertices.Size();
            aMesh.Vertices.Append(new BVH_Vec3d(i * 2, 0, 0));
            aMesh.Vertices.Append(new BVH_Vec3d((i * 2) + 1, 0, 0));
            aMesh.Vertices.Append(new BVH_Vec3d(i * 2, 1, 1));
            aMesh.Elements.Append(new BVH_Vec4i(aFirst, aFirst + 1, aFirst + 2, 0));
        }
        aMesh.SetBuilder(new BinnedBuilder<double, BVH_Vec3d>(theLeafSize, 32));
        aMesh.MarkDirty();
        Tree<double, BVH_Vec3d> aBinary = aMesh.BVH();
        QuadTree<double, BVH_Vec3d> aQuad = aBinary.CollapseToQuadTree();

        Assert.Equal(aBinary.MinPoint(0)[0], aQuad.MinPoint(0)[0], 12);
        Assert.Equal(aBinary.MaxPoint(0)[0], aQuad.MaxPoint(0)[0], 12);

        var aInBinary = new List<int>();
        for (int n = 0; n < aBinary.Length(); n++)
        {
            if (aBinary.IsOuter(n))
            {
                for (int p = aBinary.BegPrimitive(n); p <= aBinary.EndPrimitive(n); p++)
                {
                    aInBinary.Add(p);
                }
            }
        }
        var aInQuad = new List<int>();
        for (int n = 0; n < aQuad.Length(); n++)
        {
            if (aQuad.NodeInfoBuffer()[n][0] == 1)
            {
                for (int p = aQuad.NodeInfoBuffer()[n][1]; p <= aQuad.NodeInfoBuffer()[n][2]; p++)
                {
                    aInQuad.Add(p);
                }
            }
        }
        aInBinary.Sort();
        aInQuad.Sort();
        Assert.Equal(aInBinary, aInQuad);
        Assert.True(aQuad.Depth() <= aBinary.Depth());
    }

    [Fact]
    public void An_empty_tree_collapses_to_an_empty_quad_tree()
    {
        Assert.Equal(0, new Tree<double, BVH_Vec3d>().CollapseToQuadTree().Length());
    }

    [Fact]
    public void A_box_set_by_index_gives_the_elements_it_was_given_after_a_swap()
    {
        var aSet = new IndexedBoxSet<double, BVH_Vec3d>();
        aSet.SetSize(3);
        for (int i = 0; i < 3; i++)
        {
            aSet.Add(i * 10, new Box3d(new BVH_Vec3d(i, 0, 0), new BVH_Vec3d(i + 1, 1, 1)));
        }
        Assert.Equal(3, aSet.Size());
        Assert.Equal(10, aSet.Element(1));
        Assert.Equal(1, aSet.Box(1).CornerMin()[0], 12);
        aSet.Swap(0, 2);
        Assert.Equal(20, aSet.Element(0));
        Assert.Equal(2, aSet.Box(0).CornerMin()[0], 12);
        Assert.Equal(0, aSet.Element(2));
        aSet.Clear();
        Assert.Equal(0, aSet.Size());
        Assert.NotNull(new IndexedBoxSet<double, BVH_Vec3d>(new BinnedBuilder<double, BVH_Vec3d>(1, 32)));
    }

    [Fact]
    public void A_transform_moves_a_box_to_hold_every_corner_moved()
    {
        var aMatrix = NCollection_Mat4<double>.Identity();
        aMatrix.SetValue(0, 3, 5); // a move of 5 along x
        var aTransform = new BvhTransform<double>(aMatrix);
        var aBox = new Box<double, BVH_Vec4d>(new BVH_Vec4d(0, 0, 0, 1), new BVH_Vec4d(1, 2, 3, 1));
        Box<double, BVH_Vec4d> aMoved = aTransform.Apply(aBox);
        Assert.Equal(5, aMoved.CornerMin()[0], 12);
        Assert.Equal(6, aMoved.CornerMax()[0], 12);
        Assert.Equal(3, aMoved.CornerMax()[2], 12);
        Assert.Equal(5, aTransform.Transform().GetValue(0, 3), 12);
        var aDefault = new BvhTransform<double>();
        aDefault.SetTransform(aMatrix);
        Assert.Equal(-5, aDefault.Inversed().GetValue(0, 3), 12);
    }
}
