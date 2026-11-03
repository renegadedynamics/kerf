// Port of: src/FoundationClasses/TKMath/GTests/BVH_Traverse_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Box3d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec3<double>>;
using Tree3d = Kerf.Engine.Geometry.Bvh.Tree<double, Occt.NCollection_Vec3<double>>;

namespace OcctNet.Foundation.Tests;

// Test implementations of BVH_Traverse

/// <summary>Simple traverse implementation that counts all elements.</summary>
internal sealed class BVH_CountAllElements : Traverse<double, BVH_Vec3d, double>
{
    private int myAcceptedCount;

    public override bool RejectNode(in BVH_Vec3d theCornerMin, in BVH_Vec3d theCornerMax, out double theMetric)
    {
        theMetric = 0.0; // All nodes have same metric
        return false;    // Never reject
    }

    public override bool Accept(int theIndex, in double theMetric)
    {
        ++myAcceptedCount;
        return true;
    }

    public int AcceptedCount() => myAcceptedCount;

    public void Reset() => myAcceptedCount = 0;
}

/// <summary>Traverse that rejects nodes outside a bounding box.</summary>
internal sealed class BVH_BoxSelector(in Box3d theBox) : Traverse<double, BVH_Vec3d, double>
{
    private readonly Box3d myBox = theBox;
    private int myAcceptedCount;

    public override bool RejectNode(in BVH_Vec3d theMin, in BVH_Vec3d theMax, out double theMetric)
    {
        // Reject if box doesn't intersect with selection box
        theMetric = 0.0;
        return myBox.IsOut(theMin, theMax);
    }

    public override bool Accept(int theIndex, in double theMetric)
    {
        ++myAcceptedCount;
        return true;
    }

    public int AcceptedCount() => myAcceptedCount;

    public void Reset() => myAcceptedCount = 0;
}

/// <summary>Traverse with distance-based metric and early termination.</summary>
internal sealed class BVH_DistanceSelector(in BVH_Vec3d thePoint, double theMaxDist) : Traverse<double, BVH_Vec3d, double>
{
    private readonly BVH_Vec3d myPoint = thePoint;
    private readonly double myMaxDistSq = theMaxDist * theMaxDist;
    private double myMinDistSq = double.MaxValue;
    private int myAcceptedCount;
    private int myClosestIndex = -1;

    public override bool RejectNode(in BVH_Vec3d theMin, in BVH_Vec3d theMax, out double theMetric)
    {
        // Compute squared distance from point to box
        theMetric = PointBoxSquareDistance(myPoint, theMin, theMax);
        return theMetric > myMaxDistSq;
    }

    public override bool Accept(int theIndex, in double theMetric)
    {
        ++myAcceptedCount;
        if (theMetric < myMinDistSq)
        {
            myMinDistSq = theMetric;
            myClosestIndex = theIndex;
        }

        return true;
    }

    public override bool IsMetricBetter(in double theLeft, in double theRight) => theLeft < theRight; // Closer is better

    public override bool RejectMetric(in double theMetric) => theMetric > myMaxDistSq;

    public int AcceptedCount() => myAcceptedCount;

    public int ClosestIndex() => myClosestIndex;

    public double MinDistance() => Math.Sqrt(myMinDistSq);

    private static double PointBoxSquareDistance(in BVH_Vec3d thePoint, in BVH_Vec3d theMin, in BVH_Vec3d theMax)
    {
        double aDist = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            if (thePoint[i] < theMin[i])
            {
                double d = theMin[i] - thePoint[i];
                aDist += d * d;
            }
            else if (thePoint[i] > theMax[i])
            {
                double d = thePoint[i] - theMax[i];
                aDist += d * d;
            }
        }

        return aDist;
    }
}

/// <summary>Traverse with early stopping after finding N elements.</summary>
internal sealed class BVH_LimitedSelector(int theMaxCount) : Traverse<double, BVH_Vec3d, double>
{
    private readonly int myMaxCount = theMaxCount;
    private int myAcceptedCount;

    public override bool RejectNode(in BVH_Vec3d theCornerMin, in BVH_Vec3d theCornerMax, out double theMetric)
    {
        theMetric = 0.0;
        return false;
    }

    public override bool Accept(int theIndex, in double theMetric)
    {
        ++myAcceptedCount;
        return true;
    }

    public override bool Stop() => myAcceptedCount >= myMaxCount;

    public int AcceptedCount() => myAcceptedCount;
}

// Test implementations of BVH_PairTraverse

/// <summary>Simple pair traverse that counts all pairs.</summary>
internal sealed class BVH_CountAllPairs : PairTraverse<double, BVH_Vec3d, double>
{
    private int myAcceptedCount;

    public override bool RejectNode(in BVH_Vec3d theCornerMin1, in BVH_Vec3d theCornerMax1, in BVH_Vec3d theCornerMin2, in BVH_Vec3d theCornerMax2, out double theMetric)
    {
        theMetric = 0.0;
        return false; // Never reject
    }

    public override bool Accept(int theIndex1, int theIndex2)
    {
        ++myAcceptedCount;
        return true;
    }

    public int AcceptedCount() => myAcceptedCount;
}

/// <summary>Pair traverse that only accepts overlapping boxes.</summary>
internal sealed class BVH_OverlapDetector : PairTraverse<double, BVH_Vec3d, double>
{
    private int myOverlapCount;
    private int myRejectCount;

    public override bool RejectNode(in BVH_Vec3d theMin1, in BVH_Vec3d theMax1, in BVH_Vec3d theMin2, in BVH_Vec3d theMax2, out double theMetric)
    {
        ++myRejectCount;
        theMetric = 0.0;
        // Reject if boxes don't overlap
        Box3d aBox1 = new(theMin1, theMax1);
        bool isOut = aBox1.IsOut(theMin2, theMax2);
        return isOut;
    }

    public override bool Accept(int theIndex1, int theIndex2)
    {
        // Reaching Accept means the bounding boxes overlap
        ++myOverlapCount;
        return true;
    }

    public int OverlapCount() => myOverlapCount;

    public int RejectCount() => myRejectCount;
}

internal static class BVH_TraverseTestTools
{
    /// <summary>Creates a simple triangulation for testing.</summary>
    public static Tree3d CreateSimpleTriangulationBVH(int theNumTriangles)
    {
        Triangulation<double, BVH_Vec3d> aTriangulation = new();

        for (int i = 0; i < theNumTriangles; ++i)
        {
            double x = i * 2;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 1.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 2.0, 0.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Tree3d aBVH = new();
        BinnedBuilder<double, BVH_Vec3d> aBuilder = new();
        aBuilder.Build(aTriangulation, aBVH, aTriangulation.Box());

        return aBVH;
    }

    /// <summary>Two triangulations of three triangles, at x = 0..3 and x = 100..103.</summary>
    public static (Triangulation<double, BVH_Vec3d>, Triangulation<double, BVH_Vec3d>) CreateSeparatedTriangulations()
    {
        Triangulation<double, BVH_Vec3d> aTri1 = new(), aTri2 = new();

        for (int i = 0; i < 3; ++i)
        {
            double x = i;
            aTri1.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTri1.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTri1.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTri1.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        for (int i = 0; i < 3; ++i)
        {
            double x = 100.0 + i;
            aTri2.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTri2.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTri2.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTri2.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        return (aTri1, aTri2);
    }
}

public class BVH_TraverseTest
{
    [Fact]
    public void CountAllElements()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(10);

        BVH_CountAllElements aSelector = new();
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(10, aCount);
        Assert.Equal(10, aSelector.AcceptedCount());
    }

    [Fact]
    public void EmptyTree()
    {
        Tree3d aBVH = new();

        BVH_CountAllElements aSelector = new();
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(0, aCount);
    }

    [Fact]
    public void NullTree()
    {
        Tree3d? aBVH = null;

        BVH_CountAllElements aSelector = new();
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(0, aCount);
    }

    [Fact]
    public void BoxSelection()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(10);

        // Select elements in the first half
        Box3d aBox = new(new BVH_Vec3d(0.0, 0.0, -1.0), new BVH_Vec3d(10.0, 2.0, 1.0));
        BVH_BoxSelector aSelector = new(aBox);
        int aCount = aSelector.Select(aBVH);

        Assert.True(aCount > 0);
        Assert.True(aCount < 10);
    }

    [Fact]
    public void EmptyBoxSelection()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(10);

        Box3d aBox = new(new BVH_Vec3d(100.0, 100.0, 100.0), new BVH_Vec3d(200.0, 200.0, 200.0));
        BVH_BoxSelector aSelector = new(aBox);
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(0, aCount);
    }

    [Fact]
    public void FullBoxSelection()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(10);

        Box3d aBox = new(new BVH_Vec3d(-100.0, -100.0, -100.0), new BVH_Vec3d(100.0, 100.0, 100.0));
        BVH_BoxSelector aSelector = new(aBox);
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(10, aCount);
    }

    [Fact]
    public void DistanceBasedSelection()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(10);

        // Find elements near a point
        BVH_Vec3d aPoint = new(5.0, 0.5, 0.0);
        double aMaxDist = 5.0;
        BVH_DistanceSelector aSelector = new(aPoint, aMaxDist);
        int aCount = aSelector.Select(aBVH);

        Assert.True(aCount > 0);
        Assert.True(aCount <= 10);
        Assert.True(aSelector.ClosestIndex() >= 0);
        Assert.True(aSelector.ClosestIndex() < 10);
    }

    [Fact]
    public void EarlyTermination()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(100);

        // Stop after finding 5 elements
        BVH_LimitedSelector aSelector = new(5);
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(5, aCount);
        Assert.Equal(5, aSelector.AcceptedCount());
    }

    [Fact]
    public void LargeDataSet()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(1000);

        BVH_CountAllElements aSelector = new();
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(1000, aCount);
    }

    [Fact]
    public void MetricBasedPruning()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(50);

        // Very restrictive distance should result in few acceptances
        BVH_Vec3d aPoint = new(1000.0, 1000.0, 1000.0); // Far away
        double aMaxDist = 1.0;                            // Small radius
        BVH_DistanceSelector aSelector = new(aPoint, aMaxDist);
        int aCount = aSelector.Select(aBVH);

        Assert.Equal(0, aCount); // Nothing should be within range
    }
}

public class BVH_PairTraverseTest
{
    [Fact]
    public void IsOutVerification()
    {
        Box3d aBox1 = new(new BVH_Vec3d(0.0, 0.0, 0.0), new BVH_Vec3d(3.0, 1.0, 0.0));
        Box3d aBox2 = new(new BVH_Vec3d(100.0, 0.0, 0.0), new BVH_Vec3d(103.0, 1.0, 0.0));

        Assert.True(aBox1.IsOut(aBox2.CornerMin(), aBox2.CornerMax()));
        Assert.True(aBox2.IsOut(aBox1.CornerMin(), aBox1.CornerMax()));
    }

    [Fact]
    public void TriangulationBoxVerification()
    {
        var (aTri1, aTri2) = BVH_TraverseTestTools.CreateSeparatedTriangulations();

        // Mark as dirty to force bounding box computation
        aTri1.MarkDirty();
        aTri2.MarkDirty();

        Box3d aBox1 = aTri1.Box();
        Box3d aBox2 = aTri2.Box();

        Assert.True(aBox1.IsValid);
        Assert.True(aBox2.IsValid);

        Assert.Equal(0.0, aBox1.CornerMin().X(), 0.01);
        Assert.Equal(3.0, aBox1.CornerMax().X(), 0.01);

        Assert.Equal(100.0, aBox2.CornerMin().X(), 0.01);
        Assert.Equal(103.0, aBox2.CornerMax().X(), 0.01);

        Assert.True(aBox1.IsOut(aBox2));
        Assert.True(aBox2.IsOut(aBox1));
    }

    [Fact]
    public void BVHRootBoxVerification()
    {
        var (aTri1, aTri2) = BVH_TraverseTestTools.CreateSeparatedTriangulations();

        aTri1.MarkDirty();
        aTri2.MarkDirty();

        Tree3d aBVH1 = new();
        Tree3d aBVH2 = new();

        BinnedBuilder<double, BVH_Vec3d> aBuilder = new();
        aBuilder.Build(aTri1, aBVH1, aTri1.Box());
        aBuilder.Build(aTri2, aBVH2, aTri2.Box());

        BVH_Vec3d aMin1 = aBVH1.MinPoint(0);
        BVH_Vec3d aMax1 = aBVH1.MaxPoint(0);
        BVH_Vec3d aMin2 = aBVH2.MinPoint(0);
        BVH_Vec3d aMax2 = aBVH2.MaxPoint(0);

        Assert.Equal(0.0, aMin1.X(), 0.01);
        Assert.Equal(3.0, aMax1.X(), 0.01);
        Assert.Equal(100.0, aMin2.X(), 0.01);
        Assert.Equal(103.0, aMax2.X(), 0.01);

        Box3d aBox1 = new(aMin1, aMax1);
        Assert.True(aBox1.IsOut(aMin2, aMax2));

        BVH_Vec4i aData1 = aBVH1.NodeInfoBuffer()[0];
        BVH_Vec4i aData2 = aBVH2.NodeInfoBuffer()[0];

        // With 3 triangles, the root is a single leaf node
        Assert.Equal(1, aData1.X());
        Assert.Equal(1, aData2.X());
    }

    [Fact]
    public void CountAllPairs()
    {
        Tree3d aBVH1 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(5);
        Tree3d aBVH2 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(5);

        BVH_CountAllPairs aSelector = new();
        int aCount = aSelector.Select(aBVH1, aBVH2);

        Assert.Equal(25, aCount); // 5 x 5 pairs
    }

    [Fact]
    public void EmptyFirstTree()
    {
        Tree3d aBVH1 = new();
        Tree3d aBVH2 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(5);

        BVH_CountAllPairs aSelector = new();
        Assert.Equal(0, aSelector.Select(aBVH1, aBVH2));
    }

    [Fact]
    public void EmptySecondTree()
    {
        Tree3d aBVH1 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(5);
        Tree3d aBVH2 = new();

        BVH_CountAllPairs aSelector = new();
        Assert.Equal(0, aSelector.Select(aBVH1, aBVH2));
    }

    [Fact]
    public void NullTrees()
    {
        BVH_CountAllPairs aSelector = new();
        Assert.Equal(0, aSelector.Select(null, null));
    }

    [Fact]
    public void OverlapDetection_SameTrees()
    {
        Tree3d aBVH = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(10);

        BVH_OverlapDetector aSelector = new();
        int aCount = aSelector.Select(aBVH, aBVH);

        // Self-overlap: all 10 elements overlap with themselves
        Assert.True(aCount >= 10);
    }

    [Fact]
    public void OverlapDetection_NonOverlapping()
    {
        var (aTri1, aTri2) = BVH_TraverseTestTools.CreateSeparatedTriangulations();

        aTri1.MarkDirty();
        aTri2.MarkDirty();

        Tree3d aBVH1 = new();
        Tree3d aBVH2 = new();

        BinnedBuilder<double, BVH_Vec3d> aBuilder = new();
        aBuilder.Build(aTri1, aBVH1, aTri1.Box());
        aBuilder.Build(aTri2, aBVH2, aTri2.Box());

        BVH_OverlapDetector aSelector = new();
        int aCount = aSelector.Select(aBVH1, aBVH2);

        Assert.True(aSelector.RejectCount() > 0, "RejectNode should be called at least once");
        Assert.Equal(0, aCount);
    }

    [Fact]
    public void AsymmetricPairs()
    {
        Tree3d aBVH1 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(3);
        Tree3d aBVH2 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(7);

        BVH_CountAllPairs aSelector = new();
        Assert.Equal(21, aSelector.Select(aBVH1, aBVH2)); // 3 x 7 pairs
    }

    [Fact]
    public void LargeDataSets()
    {
        Tree3d aBVH1 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(50);
        Tree3d aBVH2 = BVH_TraverseTestTools.CreateSimpleTriangulationBVH(50);

        BVH_CountAllPairs aSelector = new();
        Assert.Equal(2500, aSelector.Select(aBVH1, aBVH2)); // 50 x 50 pairs
    }
}
