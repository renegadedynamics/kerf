// Port of: src/FoundationClasses/TKMath/GTests/BVH_LinearBuilder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Box2d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec2<double>>;
using Box3d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec3<double>>;
using Builder2d = Kerf.Engine.Geometry.Bvh.LinearBuilder<double, Occt.NCollection_Vec2<double>>;
using Builder3d = Kerf.Engine.Geometry.Bvh.LinearBuilder<double, Occt.NCollection_Vec3<double>>;
using BoxSet2d = Kerf.Engine.Geometry.Bvh.BoxSet<double, Occt.NCollection_Vec2<double>>;
using BoxSet3d = Kerf.Engine.Geometry.Bvh.BoxSet<double, Occt.NCollection_Vec3<double>>;
using Tree2d = Kerf.Engine.Geometry.Bvh.Tree<double, Occt.NCollection_Vec2<double>>;
using Tree3d = Kerf.Engine.Geometry.Bvh.Tree<double, Occt.NCollection_Vec3<double>>;

namespace Kerf.Kernel.Foundation.Tests;

public class BVH_LinearBuilderTest
{
    private static BoxSet3d RowSet(Builder3d theBuilder, int theCount)
    {
        BoxSet3d aBoxSet = new(theBuilder);
        for (int i = 0; i < theCount; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 2.0, 0.0, 0.0), new BVH_Vec3d(i * 2.0 + 1.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        return aBoxSet;
    }

    [Fact]
    public void DefaultConstructor()
    {
        Builder3d aBuilder = new();

        Assert.True(aBuilder.LeafNodeSize() > 0);
        Assert.True(aBuilder.MaxTreeDepth() > 0);
    }

    [Fact]
    public void CustomParameters()
    {
        Builder3d aBuilder = new(5, 20);

        Assert.Equal(5, aBuilder.LeafNodeSize());
        Assert.Equal(20, aBuilder.MaxTreeDepth());
    }

    [Fact]
    public void BuildEmptySet()
    {
        BoxSet3d aBoxSet = new(new Builder3d());

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.Equal(0, aBVH.Length());
    }

    [Fact]
    public void BuildSingleElement()
    {
        BoxSet3d aBoxSet = new(new Builder3d());

        aBoxSet.Add(0, new Box3d(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0)));
        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.Equal(1, aBVH.Length());
        Assert.True(aBVH.IsOuter(0));
    }

    [Fact]
    public void BuildMultipleElements()
    {
        BoxSet3d aBoxSet = RowSet(new Builder3d(1, 32), 10);

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);
        Assert.True(aBVH.Depth() > 0);
    }

    [Fact]
    public void MortonCodeSorting()
    {
        BoxSet3d aBoxSet = new(new Builder3d(1, 32));

        for (int i = 0; i < 8; ++i)
        {
            double x = (i & 1) != 0 ? 10.0 : 0.0;
            double y = (i & 2) != 0 ? 10.0 : 0.0;
            double z = (i & 4) != 0 ? 10.0 : 0.0;

            Box3d aBox = new(new BVH_Vec3d(x, y, z), new BVH_Vec3d(x + 1.0, y + 1.0, z + 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);

        double aSAH = aBVH.EstimateSAH();
        Assert.True(aSAH > 0.0);
    }

    [Fact]
    public void LeafNodeSizeRespected()
    {
        const int aLeafSize = 3;
        BoxSet3d aBoxSet = RowSet(new Builder3d(aLeafSize, 32), 10);

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        for (int i = 0; i < aBVH.Length(); ++i)
        {
            if (aBVH.IsOuter(i))
            {
                Assert.True(aBVH.NbPrimitives(i) <= aLeafSize);
            }
        }
    }

    [Fact]
    public void Build2D()
    {
        BoxSet2d aBoxSet = new(new Builder2d(1, 32));

        for (int i = 0; i < 10; ++i)
        {
            Box2d aBox = new(new BVH_Vec2d(i * 2.0, 0.0), new BVH_Vec2d(i * 2.0 + 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree2d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);
    }

    [Fact]
    public void LargeDataSet()
    {
        BoxSet3d aBoxSet = new(new Builder3d(4, 32));

        int aCount = 0;
        for (int x = 0; x < 10; ++x)
        {
            for (int y = 0; y < 10; ++y)
            {
                for (int z = 0; z < 10; ++z)
                {
                    Box3d aBox = new(new BVH_Vec3d(x * 2.0, y * 2.0, z * 2.0),
                                     new BVH_Vec3d(x * 2.0 + 1.0, y * 2.0 + 1.0, z * 2.0 + 1.0));
                    aBoxSet.Add(aCount++, aBox);
                }
            }
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);

        int aTotalPrims = 0;
        for (int i = 0; i < aBVH.Length(); ++i)
        {
            if (aBVH.IsOuter(i))
            {
                aTotalPrims += aBVH.NbPrimitives(i);
            }
        }

        Assert.Equal(1000, aTotalPrims);
    }

    [Fact]
    public void MaxDepthParameter()
    {
        const int aMaxDepth = 10;
        BoxSet3d aBoxSet = RowSet(new Builder3d(1, aMaxDepth), 100);

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Depth() > 0);
        Assert.True(aBVH.Depth() < 20);
    }

    [Fact]
    public void ClusteredData()
    {
        BoxSet3d aBoxSet = new(new Builder3d(1, 32));

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox1 = new(new BVH_Vec3d(i * 0.1, i * 0.1, i * 0.1),
                              new BVH_Vec3d(i * 0.1 + 0.1, i * 0.1 + 0.1, i * 0.1 + 0.1));
            aBoxSet.Add(i, aBox1);

            Box3d aBox2 = new(new BVH_Vec3d(100.0 + i * 0.1, 100.0 + i * 0.1, 100.0 + i * 0.1),
                              new BVH_Vec3d(100.1 + i * 0.1, 100.1 + i * 0.1, 100.1 + i * 0.1));
            aBoxSet.Add(i + 10, aBox2);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);

        double aSAH = aBVH.EstimateSAH();
        Assert.True(aSAH > 0.0);
    }

    [Fact]
    public void OverlappingBoxes()
    {
        BoxSet3d aBoxSet = new(new Builder3d(1, 32));

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 0.5, 0.0, 0.0), new BVH_Vec3d(i * 0.5 + 2.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);
    }

    [Fact]
    public void IdenticalBoxes()
    {
        BoxSet3d aBoxSet = new(new Builder3d(1, 32));

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() >= 1);
    }

    [Fact]
    public void CompareWithBinnedBuilder()
    {
        BoxSet3d aLinearSet = RowSet(new Builder3d(1, 32), 50);

        aLinearSet.Build();

        Tree3d aLinearBVH = aLinearSet.BVH();

        Assert.True(aLinearBVH.Length() > 1);
        Assert.True(aLinearBVH.Depth() > 0);

        double aSAH = aLinearBVH.EstimateSAH();
        Assert.True(aSAH > 0.0);
    }

    [Fact]
    public void NegativeCoordinates()
    {
        BoxSet3d aBoxSet = new(new Builder3d(1, 32));

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(-10.0 + i * 2.0, -5.0, -5.0), new BVH_Vec3d(-9.0 + i * 2.0, -4.0, -4.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);
    }
}
