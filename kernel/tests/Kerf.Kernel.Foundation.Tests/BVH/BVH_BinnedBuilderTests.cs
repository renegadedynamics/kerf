// Port of: src/FoundationClasses/TKMath/GTests/BVH_BinnedBuilder_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Box2d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec2<double>>;
using Box3d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec3<double>>;
using Builder2d = Kerf.Engine.Geometry.Bvh.BinnedBuilder<double, Occt.NCollection_Vec2<double>>;
using Builder3d = Kerf.Engine.Geometry.Bvh.BinnedBuilder<double, Occt.NCollection_Vec3<double>>;
using BoxSet2d = Kerf.Engine.Geometry.Bvh.BoxSet<double, Occt.NCollection_Vec2<double>>;
using BoxSet3d = Kerf.Engine.Geometry.Bvh.BoxSet<double, Occt.NCollection_Vec3<double>>;
using Tree2d = Kerf.Engine.Geometry.Bvh.Tree<double, Occt.NCollection_Vec2<double>>;
using Tree3d = Kerf.Engine.Geometry.Bvh.Tree<double, Occt.NCollection_Vec3<double>>;

namespace Kerf.Kernel.Foundation.Tests;

public class BVH_BinnedBuilderTest
{
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
        Builder3d aBuilder = new();
        BoxSet3d aBoxSet = new(aBuilder);

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.Equal(0, aBVH.Length());
    }

    [Fact]
    public void BuildSingleElement()
    {
        Builder3d aBuilder = new();
        BoxSet3d aBoxSet = new(aBuilder);

        aBoxSet.Add(0, new Box3d(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0)));
        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.Equal(1, aBVH.Length());
        Assert.True(aBVH.IsOuter(0));
    }

    [Fact]
    public void BuildMultipleElements()
    {
        Builder3d aBuilder = new(1, 32);
        BoxSet3d aBoxSet = new(aBuilder);

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 2.0, 0.0, 0.0), new BVH_Vec3d(i * 2.0 + 1.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);
        Assert.True(aBVH.Depth() > 0);
    }

    [Fact]
    public void SAHOptimization()
    {
        Builder3d aBuilder = new(1, 32);
        BoxSet3d aBoxSet = new(aBuilder);

        // Two clusters: one near origin, one far away
        for (int i = 0; i < 5; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 0.1, 0.0, 0.0), new BVH_Vec3d(i * 0.1 + 0.1, 0.1, 0.1));
            aBoxSet.Add(i, aBox);
        }

        for (int i = 0; i < 5; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(100.0 + i * 0.1, 0.0, 0.0), new BVH_Vec3d(100.0 + i * 0.1 + 0.1, 0.1, 0.1));
            aBoxSet.Add(i + 5, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();

        double aSAH = aBVH.EstimateSAH();
        Assert.True(aSAH > 0.0);
    }

    [Fact]
    public void LeafNodeSizeRespected()
    {
        const int aLeafSize = 3;
        Builder3d aBuilder = new(aLeafSize, 32);
        BoxSet3d aBoxSet = new(aBuilder);

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 2.0, 0.0, 0.0), new BVH_Vec3d(i * 2.0 + 1.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();

        for (int i = 0; i < aBVH.Length(); ++i)
        {
            if (aBVH.IsOuter(i))
            {
                int aNbPrims = aBVH.NbPrimitives(i);
                Assert.True(aNbPrims <= aLeafSize);
            }
        }
    }

    [Fact]
    public void BuildWithDifferentBinCounts()
    {
        // OCCT's template parameter Bins is the constructor's last argument
        Builder3d aBuilder16 = new(1, 32, bins: 16);
        BoxSet3d aBoxSet = new(aBuilder16);

        for (int i = 0; i < 20; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 1.0, 0.0, 0.0), new BVH_Vec3d(i * 1.0 + 0.5, 0.5, 0.5));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() > 1);
    }

    [Fact]
    public void Build2D()
    {
        Builder2d aBuilder = new(1, 32);
        BoxSet2d aBoxSet = new(aBuilder);

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
    public void RandomDistribution()
    {
        Builder3d aBuilder = new(1, 32);
        BoxSet3d aBoxSet = new(aBuilder);

        int aCount = 0;
        for (int x = 0; x < 5; ++x)
        {
            for (int y = 0; y < 5; ++y)
            {
                for (int z = 0; z < 5; ++z)
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

        Assert.Equal(125, aTotalPrims);
    }

    [Fact]
    public void CompareTreeQuality()
    {
        Builder3d aBuilder1 = new(1, 32);
        Builder3d aBuilder4 = new(4, 32);

        BoxSet3d aBoxSet1 = new(aBuilder1);
        BoxSet3d aBoxSet4 = new(aBuilder4);

        for (int i = 0; i < 50; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 2.0, 0.0, 0.0), new BVH_Vec3d(i * 2.0 + 1.0, 1.0, 1.0));
            aBoxSet1.Add(i, aBox);
            aBoxSet4.Add(i, aBox);
        }

        aBoxSet1.Build();
        aBoxSet4.Build();

        Tree3d aBVH1 = aBoxSet1.BVH();
        Tree3d aBVH4 = aBoxSet4.BVH();

        Assert.True(aBVH1.Depth() >= aBVH4.Depth());
    }

    [Fact]
    public void MaxDepthRespected()
    {
        const int aMaxDepth = 5;
        Builder3d aBuilder = new(1, aMaxDepth);
        BoxSet3d aBoxSet = new(aBuilder);

        for (int i = 0; i < 100; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(i * 2.0, 0.0, 0.0), new BVH_Vec3d(i * 2.0 + 1.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Depth() <= aMaxDepth);
    }

    [Fact]
    public void OverlappingBoxes()
    {
        Builder3d aBuilder = new(1, 32);
        BoxSet3d aBoxSet = new(aBuilder);

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
        Builder3d aBuilder = new(1, 32);
        BoxSet3d aBoxSet = new(aBuilder);

        for (int i = 0; i < 10; ++i)
        {
            Box3d aBox = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(1.0, 1.0, 1.0));
            aBoxSet.Add(i, aBox);
        }

        aBoxSet.Build();

        Tree3d aBVH = aBoxSet.BVH();
        Assert.True(aBVH.Length() >= 1);
    }
}
