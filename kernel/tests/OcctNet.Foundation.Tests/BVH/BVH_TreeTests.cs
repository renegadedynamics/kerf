// Port of: src/FoundationClasses/TKMath/GTests/BVH_Tree_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Box3d = Occt.BVH_Box<double, Occt.NCollection_Vec3<double>>;
using Tree3d = Occt.BVH_Tree<double, Occt.NCollection_Vec3<double>>;

namespace OcctNet.Foundation.Tests;

// OCCT's BVH_Tree<T, N, BVH_BinaryTree> is the port's BVH_Tree<T, TVec>; Child<K>(n) is Child(K, n).
public class BVH_TreeTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Tree3d aTree = new();

        Assert.Equal(0, aTree.Length());
        Assert.Equal(0, aTree.Depth());
    }

    [Fact]
    public void AddLeafNode()
    {
        Tree3d aTree = new();

        BVH_Vec3d aMin = new(0.0, 0.0, 0.0);
        BVH_Vec3d aMax = new(1.0, 1.0, 1.0);

        int aNodeIndex = aTree.AddLeafNode(aMin, aMax, 0, 10);

        Assert.Equal(0, aNodeIndex);
        Assert.Equal(1, aTree.Length());
        Assert.True(aTree.IsOuter(0));
        Assert.Equal(0, aTree.BegPrimitive(0));
        Assert.Equal(10, aTree.EndPrimitive(0));
        Assert.Equal(11, aTree.NbPrimitives(0));
    }

    [Fact]
    public void AddInnerNode()
    {
        Tree3d aTree = new();

        int aLeaf1 = aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 5);
        int aLeaf2 = aTree.AddLeafNode(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0), 6, 10);

        int aRoot = aTree.AddInnerNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0), aLeaf1, aLeaf2);

        Assert.Equal(3, aTree.Length());
        Assert.False(aTree.IsOuter(aRoot));
        Assert.Equal(aLeaf1, aTree.Child(0, aRoot));
        Assert.Equal(aLeaf2, aTree.Child(1, aRoot));
    }

    [Fact]
    public void MinMaxPoints()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(1.0, 2.0, 3.0), new BVH_Vec3d(4.0, 5.0, 6.0), 0, 0);

        Assert.Equal(1.0, aTree.MinPoint(0).x(), Precision.Confusion());
        Assert.Equal(2.0, aTree.MinPoint(0).y(), Precision.Confusion());
        Assert.Equal(3.0, aTree.MinPoint(0).z(), Precision.Confusion());
        Assert.Equal(4.0, aTree.MaxPoint(0).x(), Precision.Confusion());
        Assert.Equal(5.0, aTree.MaxPoint(0).y(), Precision.Confusion());
        Assert.Equal(6.0, aTree.MaxPoint(0).z(), Precision.Confusion());
    }

    [Fact]
    public void Clear()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);
        aTree.AddLeafNode(new BVH_Vec3d(1.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0), 1, 1);

        Assert.Equal(2, aTree.Length());

        aTree.Clear();

        Assert.Equal(0, aTree.Length());
        Assert.Equal(0, aTree.Depth());
    }

    [Fact]
    public void Reserve()
    {
        Tree3d aTree = new();

        aTree.Reserve(100);

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);
        Assert.Equal(1, aTree.Length());
    }

    [Fact]
    public void SetOuterInner()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);

        Assert.True(aTree.IsOuter(0));

        aTree.SetInner(0);
        Assert.False(aTree.IsOuter(0));

        aTree.SetOuter(0);
        Assert.True(aTree.IsOuter(0));
    }

    [Fact]
    public void Level()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);

        Assert.Equal(0, aTree.Level(0));

        aTree.Level(0) = 5;
        Assert.Equal(5, aTree.Level(0));
    }

    [Fact]
    public void AddLeafNodeWithBox()
    {
        Tree3d aTree = new();

        Box3d aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));

        int aNodeIndex = aTree.AddLeafNode(aBox, 0, 5);

        Assert.Equal(0, aNodeIndex);
        Assert.True(aTree.IsOuter(0));
        Assert.Equal(0, aTree.BegPrimitive(0));
        Assert.Equal(5, aTree.EndPrimitive(0));
    }

    [Fact]
    public void AddInnerNodeWithBox()
    {
        Tree3d aTree = new();

        Box3d aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
        Box3d aBox2 = new(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0));
        Box3d aRootBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0));

        int aLeaf1 = aTree.AddLeafNode(aBox1, 0, 5);
        int aLeaf2 = aTree.AddLeafNode(aBox2, 6, 10);
        int aRoot = aTree.AddInnerNode(aRootBox, aLeaf1, aLeaf2);

        Assert.False(aTree.IsOuter(aRoot));
        Assert.Equal(aLeaf1, aTree.Child(0, aRoot));
        Assert.Equal(aLeaf2, aTree.Child(1, aRoot));
    }

    [Fact]
    public void EstimateSAH()
    {
        Tree3d aTree = new();

        int aLeaf1 = aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);
        int aLeaf2 = aTree.AddLeafNode(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0), 1, 1);
        aTree.AddInnerNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0), aLeaf1, aLeaf2);

        double aSAH = aTree.EstimateSAH();

        Assert.True(aSAH > 0.0);
    }

    [Fact]
    public void NbPrimitives()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 5, 15);

        Assert.Equal(11, aTree.NbPrimitives(0));
    }

    [Fact]
    public void SinglePrimitive()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 7, 7);

        Assert.Equal(1, aTree.NbPrimitives(0));
        Assert.Equal(7, aTree.BegPrimitive(0));
        Assert.Equal(7, aTree.EndPrimitive(0));
    }

    [Fact]
    public void Float3DTree()
    {
        BVH_Tree<float, BVH_Vec3f> aTree = new();

        aTree.AddLeafNode(new BVH_Vec3f(0.0f, 0.0f, 0.0f), new BVH_Vec3f(1.0f, 1.0f, 1.0f), 0, 5);

        Assert.Equal(1, aTree.Length());
        Assert.Equal(0.0f, aTree.MinPoint(0).x(), 1e-5f);
        Assert.Equal(1.0f, aTree.MaxPoint(0).x(), 1e-5f);
    }

    [Fact]
    public void Tree2D()
    {
        BVH_Tree<double, BVH_Vec2d> aTree = new();

        aTree.AddLeafNode(new BVH_Vec2d(0.0, 0.0), new BVH_Vec2d(1.0, 1.0), 0, 5);

        Assert.Equal(1, aTree.Length());
        Assert.Equal(0.0, aTree.MinPoint(0).x(), Precision.Confusion());
        Assert.Equal(0.0, aTree.MinPoint(0).y(), Precision.Confusion());
    }

    [Fact]
    public void Tree4D()
    {
        BVH_Tree<double, BVH_Vec4d> aTree = new();

        aTree.AddLeafNode(new BVH_Vec4d(0.0, 0.0, 0.0, 0.0), new BVH_Vec4d(1.0, 1.0, 1.0, 1.0), 0, 5);

        Assert.Equal(1, aTree.Length());
        Assert.Equal(0.0, aTree.MinPoint(0).x(), Precision.Confusion());
        Assert.Equal(0.0, aTree.MinPoint(0).w(), Precision.Confusion());
    }

    [Fact]
    public void MultipleLeaves()
    {
        Tree3d aTree = new();

        for (int i = 0; i < 10; ++i)
        {
            BVH_Vec3d aMin = new(i * 1.0, 0.0, 0.0);
            BVH_Vec3d aMax = new(i * 1.0 + 1.0, 1.0, 1.0);
            aTree.AddLeafNode(aMin, aMax, i, i);
        }

        Assert.Equal(10, aTree.Length());

        for (int i = 0; i < 10; ++i)
        {
            Assert.True(aTree.IsOuter(i));
            Assert.Equal(i, aTree.BegPrimitive(i));
            Assert.Equal(i, aTree.EndPrimitive(i));
        }
    }

    [Fact]
    public void DeepTree()
    {
        Tree3d aTree = new();

        int aLeaf1 = aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);
        int aLeaf2 = aTree.AddLeafNode(new BVH_Vec3d(1.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0), 1, 1);
        int aLeaf3 = aTree.AddLeafNode(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0), 2, 2);
        int aLeaf4 = aTree.AddLeafNode(new BVH_Vec3d(3.0, 0.0, 0.0), new BVH_Vec3d(4.0, 1.0, 1.0), 3, 3);

        int aInner1 = aTree.AddInnerNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0), aLeaf1, aLeaf2);
        int aInner2 = aTree.AddInnerNode(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(4.0, 1.0, 1.0), aLeaf3, aLeaf4);

        int aRoot = aTree.AddInnerNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(4.0, 1.0, 1.0), aInner1, aInner2);

        Assert.Equal(7, aTree.Length());
        Assert.False(aTree.IsOuter(aRoot));
        Assert.False(aTree.IsOuter(aInner1));
        Assert.False(aTree.IsOuter(aInner2));
        Assert.True(aTree.IsOuter(aLeaf1));
        Assert.True(aTree.IsOuter(aLeaf4));
    }

    [Fact]
    public void ModifyPrimitiveIndices()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 5);

        aTree.BegPrimitive(0) = 10;
        aTree.EndPrimitive(0) = 20;

        Assert.Equal(10, aTree.BegPrimitive(0));
        Assert.Equal(20, aTree.EndPrimitive(0));
        Assert.Equal(11, aTree.NbPrimitives(0));
    }

    [Fact]
    public void ModifyMinMaxPoints()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);

        aTree.MinPoint(0) = new BVH_Vec3d(-1.0, -1.0, -1.0);
        aTree.MaxPoint(0) = new BVH_Vec3d(2.0, 2.0, 2.0);

        Assert.Equal(-1.0, aTree.MinPoint(0).x(), Precision.Confusion());
        Assert.Equal(2.0, aTree.MaxPoint(0).x(), Precision.Confusion());
    }

    [Fact]
    public void ChangeChild()
    {
        Tree3d aTree = new();

        int aLeaf1 = aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);
        int aLeaf2 = aTree.AddLeafNode(new BVH_Vec3d(1.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0), 1, 1);
        int aLeaf3 = aTree.AddLeafNode(new BVH_Vec3d(2.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 1.0), 2, 2);
        int aRoot = aTree.AddInnerNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(2.0, 1.0, 1.0), aLeaf1, aLeaf2);

        Assert.Equal(aLeaf1, aTree.Child(0, aRoot));
        Assert.Equal(aLeaf2, aTree.Child(1, aRoot));

        aTree.ChangeChild(0, aRoot) = aLeaf3;

        Assert.Equal(aLeaf3, aTree.Child(0, aRoot));
        Assert.Equal(aLeaf2, aTree.Child(1, aRoot));
    }

    [Fact]
    public void NodeInfoBuffer()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 5, 10);

        BVH_Array4i aBuffer = aTree.NodeInfoBuffer();
        Assert.Equal(1, aBuffer.Size());
    }

    [Fact]
    public void MinMaxPointBuffers()
    {
        Tree3d aTree = new();

        aTree.AddLeafNode(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0), 0, 0);

        Assert.Equal(1, aTree.MinPointBuffer().Size());
        Assert.Equal(1, aTree.MaxPointBuffer().Size());
    }
}
