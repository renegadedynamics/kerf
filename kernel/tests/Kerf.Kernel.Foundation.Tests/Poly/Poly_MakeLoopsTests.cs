// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Link = Kerf.Engine.Meshing.MakeLoops.Link;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>Poly_MakeLoops and NCollection_PackedMapOfInteger (OCCT has no GTests for the Poly package).</summary>
public class Poly_MakeLoopsTest
{
    // nodes 1-4 a unit square, node 5 off its corner 3
    private static readonly Dictionary<int, Point2D> THE_NODES = new()
    {
        [1] = new Point2D(0, 0),
        [2] = new Point2D(1, 0),
        [3] = new Point2D(1, 1),
        [4] = new Point2D(0, 1),
        [5] = new Point2D(2, 2),
    };

    private static readonly Link[] THE_LINKS = [new(1, 2), new(2, 3), new(3, 4), new(4, 1), new(3, 5)];

    private static NCollection_List<Link> Adjacent(int theNode)
        => new(THE_LINKS.Where(aL => aL.node1 == theNode || aL.node2 == theNode));

    private static Vector2D Tangent(in Link theLink)
    {
        Point2D aP1 = THE_NODES[theLink.node1];
        Point2D aP2 = THE_NODES[theLink.node2];
        return theLink.IsReversed ? new Vector2D(aP2, aP1) : new Vector2D(aP1, aP2);
    }

    private sealed class Helper2D : MakeLoops2D.Helper
    {
        public override NCollection_List<Link> GetAdjacentLinks(int theNode) => Adjacent(theNode);

        public override bool GetFirstTangent(in Link theLink, out Direction2D theDir)
        {
            theDir = new Direction2D(Tangent(theLink));
            return true;
        }

        public override bool GetLastTangent(in Link theLink, out Direction2D theDir) => GetFirstTangent(theLink, out theDir);
    }

    private sealed class Helper3D : MakeLoops3D.Helper
    {
        public override NCollection_List<Link> GetAdjacentLinks(int theNode) => Adjacent(theNode);

        public override bool GetFirstTangent(in Link theLink, out Direction3D theDir)
        {
            Vector2D aT = Tangent(theLink);
            theDir = new Direction3D(aT.X(), aT.Y(), 0);
            return true;
        }

        public override bool GetLastTangent(in Link theLink, out Direction3D theDir) => GetFirstTangent(theLink, out theDir);

        public override bool GetNormal(int theNode, out Direction3D theDir)
        {
            theDir = new Direction3D(0, 0, 1);
            return true;
        }
    }

    private static void CheckSquareLoop(MakeLoops theAlgo)
    {
        foreach (Link aLink in THE_LINKS)
        {
            theAlgo.AddLink(aLink);
        }

        int aResult = theAlgo.Perform();
        Assert.Equal((int)(MakeLoops.ResultCode.RC_LoopsDone | MakeLoops.ResultCode.RC_HangingLinks), aResult);
        Assert.Equal(1, theAlgo.GetNbLoops());
        var aLoop = theAlgo.GetLoop(1).Select(aL => (aL.node1, aL.node2)).ToList();
        Assert.Equal([(1, 2), (2, 3), (3, 4), (4, 1)], aLoop);

        Assert.Equal(1, theAlgo.GetNbHanging());
        var aHanging = new NCollection_List<Link>();
        theAlgo.GetHangingLinks(aHanging);
        Assert.Equal(new Link(3, 5), aHanging.First());
    }

    [Fact]
    public void MakeLoops2D_LeftWay()
    {
        CheckSquareLoop(new MakeLoops2D(true, new Helper2D()));
    }

    [Fact]
    public void MakeLoops3D_LeftWay()
    {
        CheckSquareLoop(new MakeLoops3D(new Helper3D()));
    }

    [Fact]
    public void MakeLoops2D_RightWayTakesTheBranch()
    {
        // turning right at node 3 leads to the hanging link 3-5, which ends the contour open and is marked hanging; the
        // next contour from link 1 then has one way at node 3 and closes the loop
        var anAlgo = new MakeLoops2D(false, new Helper2D());
        foreach (Link aLink in THE_LINKS)
        {
            anAlgo.AddLink(aLink);
        }

        int aResult = anAlgo.Perform();
        Assert.Equal((int)(MakeLoops.ResultCode.RC_LoopsDone | MakeLoops.ResultCode.RC_HangingLinks), aResult);
        Assert.Equal(1, anAlgo.GetNbLoops());
        Assert.Equal(4, anAlgo.GetLoop(1).Extent());
        Assert.Equal(1, anAlgo.GetNbHanging());
    }

    [Fact]
    public void Links_FlagsAndLookup()
    {
        var anAlgo = new MakeLoops2D(true, new Helper2D());
        anAlgo.AddLink(new Link(1, 2));
        anAlgo.AddLink(new Link(2, 1) { flags = (int)MakeLoops.LinkFlag.LF_Rev });
        anAlgo.AddLink(new Link(3, 3));

        Link aFound = anAlgo.FindLink(new Link(2, 1));
        Assert.Equal((1, 2), (aFound.node1, aFound.node2));
        Assert.Equal((int)MakeLoops.LinkFlag.LF_Both, aFound.flags);
        Assert.True(anAlgo.FindLink(new Link(3, 3)).IsNull);

        Assert.Equal(MakeLoops.LinkFlag.LF_Both, anAlgo.SetLinkOrientation(new Link(1, 2), MakeLoops.LinkFlag.LF_None));
        Assert.Equal(0, anAlgo.FindLink(new Link(1, 2)).flags);

        anAlgo.ReplaceLink(new Link(1, 2), new Link(2, 4));
        Assert.True(anAlgo.FindLink(new Link(1, 2)).IsNull);
        Assert.Equal((2, 4), (anAlgo.FindLink(new Link(4, 2)).node1, anAlgo.FindLink(new Link(4, 2)).node2));

        var aLink = new Link(7, 8);
        Assert.True(aLink == new Link(8, 7));
        Assert.Equal(aLink.GetHashCode(), new Link(8, 7).GetHashCode());
        Assert.True(aLink.Reversed().IsReversed);
        Assert.False(aLink.IsReversed);
        aLink.Nullify();
        Assert.True(aLink.IsNull);

        anAlgo.Reset(new Helper2D());
        Assert.True(anAlgo.FindLink(new Link(2, 4)).IsNull);
    }

    [Fact]
    public void HeapOfInteger()
    {
        var aHeap = new MakeLoops.HeapOfInteger();
        Assert.True(aHeap.IsEmpty());
        aHeap.Add(7);
        aHeap.Add(-3);
        aHeap.Add(2);
        Assert.False(aHeap.IsEmpty());
        Assert.Equal(2, aHeap.Top());
        aHeap.Remove(2);
        Assert.Equal(7, aHeap.Top());
        Assert.True(aHeap.Contains(-3));
        aHeap.Remove(7);
        Assert.Equal(-3, aHeap.Top());
        aHeap.Remove(-3);
        Assert.True(aHeap.IsEmpty());
    }

    [Fact]
    public void PackedMap_OcctOrder()
    {
        var aMap = new NCollection_PackedMapOfInteger();
        foreach (int aKey in new[] { 5, 1, 40, 33, -1, 5 })
        {
            aMap.Add(aKey);
        }

        Assert.Equal(5, aMap.Extent());
        Assert.Equal(101, aMap.NbBuckets());
        // blocks of 32 values: block 0 in bucket 1, block 1 in bucket 2, -1's block (2^27 - 1) in bucket 40
        Assert.Equal([1, 5, 33, 40, -1], Keys(aMap));

        Assert.True(aMap.Remove(5));
        Assert.False(aMap.Remove(5));
        Assert.False(aMap.Contains(5));
        Assert.True(aMap.Contains(40));
        Assert.True(aMap.Remove(1));
        Assert.Equal([33, 40, -1], Keys(aMap));

        aMap.Clear();
        Assert.True(aMap.IsEmpty());
        Assert.Empty(Keys(aMap));
        Assert.Throws<NoSuchObjectException>(() => new NCollection_PackedMapOfInteger.Iterator(aMap).Key());

        // past the bucket count, the map grows to the next prime
        for (int i = 0; i < 200 * 32; i += 32)
        {
            aMap.Add(i);
        }

        Assert.Equal(1009, aMap.NbBuckets());
        Assert.Equal(200, aMap.Extent());
    }

    private static List<int> Keys(NCollection_PackedMapOfInteger theMap)
    {
        var aKeys = new List<int>();
        for (var anIter = new NCollection_PackedMapOfInteger.Iterator(theMap); anIter.More(); anIter.Next())
        {
            aKeys.Add(anIter.Key());
        }

        return aKeys;
    }
}
