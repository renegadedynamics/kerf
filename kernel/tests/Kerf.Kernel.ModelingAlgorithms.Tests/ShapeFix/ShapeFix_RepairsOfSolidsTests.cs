// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Repairs that need more than one face: small edges merged across the faces that share them, faces pinched at a
/// vertex split in two, and loops taken out of a pcurve. What's checked is what's left: the small edges gone, the area
/// or volume kept, and the caller's shape unchanged.
/// </summary>
public class ShapeFix_RepairsOfSolidsTests
{
    private static Point3D P(double x, double y, double z = 0) => new(x, y, z);

    private static Face PlanarFace(params Point3D[] thePoints)
    {
        Wire aW = ShapeFixPolylines.Polyline(true, thePoints);
        return new MakeFace(aW, true).Face();
    }

    // As PlanarFace, its vertices tolerant enough to reach a nearby edge
    private static Face TolerantFace(params Point3D[] thePoints)
    {
        var aB = new BrepBuilder();
        var aV = new Vertex[thePoints.Length];
        for (int i = 0; i < thePoints.Length; i++)
        {
            aB.MakeVertex(ref aV[i], thePoints[i], 1e-4);
        }
        Wire aW = new();
        aB.MakeWire(ref aW);
        for (int i = 0; i < aV.Length; i++)
        {
            aB.Add(aW, TestShapes.Edge(aV[i], aV[(i + 1) % aV.Length]));
        }
        aW.Closed(true);
        return new MakeFace(aW, true).Face();
    }

    private static double Area(in Shape theS) => MassPropertiesBuilder.SurfaceProperties(theS).Mass();

    private static double Volume(in Shape theS) => MassPropertiesBuilder.VolumeProperties(theS).Mass();

    private static double ShortestEdge(in Shape theS)
    {
        double aMin = double.MaxValue;
        for (var anExp = new Explorer(theS, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            if (!Tool.Degenerated(anE))
            {
                aMin = Math.Min(aMin, MassPropertiesBuilder.LinearProperties(anE).Mass());
            }
        }
        return aMin;
    }

    // A prism over a polygon on z = 0, 5 high
    private static Shape Prism(params Point3D[] thePolygon)
    {
        Face aF = PlanarFace(thePolygon);
        return new ApiMakePrism(aF, new Vector3D(0, 0, 5)).Shape();
    }

    public static TheoryData<string, Point3D[]> SmallEdgeProfiles() => new()
    {
        // along a side, in line with its neighbours
        { "in line", [P(0, 0), P(5, 0), P(5.0001, 0), P(10, 0), P(10, 10), P(0, 10)] },
        // at the corner of the profile, across it
        { "a corner", [P(0, 0), P(10, 0), P(10.0001, 0.0001), P(10.0001, 10), P(0, 10)] },
        // two together
        { "two in a row", [P(0, 0), P(5, 0), P(5.0001, 0), P(5.0002, 0), P(10, 0), P(10, 10), P(0, 10)] },
        // starting the wire, so the wrap-around chooses
        { "first in the wire", [P(0, 0), P(0.0001, 0), P(10, 0), P(10, 10), P(0, 10)] },
    };

    [Theory]
    [MemberData(nameof(SmallEdgeProfiles))]
    public void FixSmallEdges_OnAFace_GoesAndKeepsTheArea(string theCase, Point3D[] theProfile)
    {
        Face aFace = PlanarFace(theProfile);
        double anArea = Area(aFace);
        var aSnap = ShapeSnapshot.Of(aFace);
        var aFix = new WireframeFixer(aFace);
        aFix.SetPrecision(1e-3);
        Assert.True(aFix.FixSmallEdges(), theCase);
        Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aFace);
        Assert.True(ShortestEdge(aResult) > 1e-3, $"{theCase}: an edge of {ShortestEdge(aResult)} is left");
        Assert.Equal(anArea, Area(aResult), 1);
        Assert.True(new Analyzer(aResult).IsValid(), theCase);
    }

    [Theory]
    [MemberData(nameof(SmallEdgeProfiles))]
    public void FixSmallEdges_OnAPrism_MergesAcrossTheFacesSharingThem(string theCase, Point3D[] theProfile)
    {
        // the prism's small edges are in the caps and its short strip's sides
        Shape aPrism = Prism(theProfile);
        double aVolume = Volume(aPrism);
        var aSnap = ShapeSnapshot.Of(aPrism);
        var aFix = new WireframeFixer(aPrism);
        aFix.SetPrecision(1e-3);
        aFix.FixSmallEdges();
        Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aPrism);
        Assert.True(Math.Abs(aVolume - Volume(aResult)) < 1e-2 * aVolume, theCase);
    }

    [Fact]
    public void FixSmallEdges_WithALimitAngle_OnAFace_DropsOnlyWithTheMode()
    {
        // a sharp step: too sharp for the angle limit, so it's kept; with the drop mode it goes all the same
        Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(5, 10), P(5, 10.0001), P(0, 10.0001));
        var aKeep = new WireframeFixer(aFace);
        aKeep.SetPrecision(1e-3);
        aKeep.SetLimitAngle(0.5);
        Assert.False(aKeep.FixSmallEdges());
        Assert.True(ShortestEdge(aKeep.Shape()) < 1e-3);

        var aDrop = new WireframeFixer(aFace);
        aDrop.SetPrecision(1e-3);
        aDrop.SetLimitAngle(0.5);
        aDrop.ModeDropSmallEdges() = true;
        Assert.True(aDrop.FixSmallEdges());
        Assert.True(aDrop.StatusSmallEdges(DONE3));
        Assert.True(ShortestEdge(aDrop.Shape()) > 1e-3);
        Assert.Equal(Area(aFace), Area(aDrop.Shape()), 1);
    }

    [Fact]
    public void FixSmallEdges_WithAHole_MergesInTheInnerWire()
    {
        Face anOuter = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        Wire aHole = ShapeFixPolylines.Polyline(true, P(2, 2), P(2, 8), P(5, 8), P(5.0001, 8), P(8, 8), P(8, 2));
        var aMake = new MakeFace(anOuter);
        aMake.Add(aHole);
        Face aFace = aMake.Face();
        double anArea = Area(aFace);
        var aFix = new WireframeFixer(aFace);
        aFix.SetPrecision(1e-3);
        Assert.True(aFix.FixSmallEdges());
        Assert.True(ShortestEdge(aFix.Shape()) > 1e-3);
        Assert.Equal(anArea, Area(aFix.Shape()), 1);
    }

    [Fact]
    public void FixSmallEdges_ByItsParts_ListsAndMergesThem()
    {
        // CheckSmallEdges finds the small edge and its faces, then MergeSmallEdges works from those lists
        Face aFace = PlanarFace(P(0, 0), P(5, 0), P(5.0001, 0), P(10, 0), P(10, 10), P(0, 10));
        var aFix = new WireframeFixer(aFace);
        aFix.SetPrecision(1e-3);
        var aSmall = new HashSet<Shape>(ShapeHasher.Instance);
        var anEdgeToFaces = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var aFaceWithSmall = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var aMulti = new HashSet<Shape>(ShapeHasher.Instance);
        Assert.True(aFix.CheckSmallEdges(aSmall, anEdgeToFaces, aFaceWithSmall, aMulti));
        Assert.Single(aSmall);
        Assert.Single(aFaceWithSmall);
        Assert.Empty(aMulti);
        Assert.True(aFix.MergeSmallEdges(aSmall, anEdgeToFaces, aFaceWithSmall, aMulti, false, -1));
        Assert.True(ShortestEdge(aFix.Shape()) > 1e-3);
    }

    [Fact]
    public void FixSplitFace_APinchedFace_BecomesTwoFaces_WithTheSameArea()
    {
        // the outline comes back to touch its own first side near (5, 0): a vertex in the middle of another edge, not
        // quite on it (OCCT passes over a vertex at a distance of exactly 0)
        Face aFace = TolerantFace(P(0, 0), P(10, 0), P(10, 10), P(5, 1e-5), P(0, 10));
        double anArea = Area(aFace);
        var aSnap = ShapeSnapshot.Of(aFace);
        var aFix = new FixSmallFaceFixer();
        aFix.Init(aFace);
        aFix.SetPrecision(1e-4);
        Shape aResult = aFix.FixSplitFace(aFace);
        aResult = aFix.Shape();
        aSnap.AssertUnchanged(aFace);
        int aFaces = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces++;
            Assert.True(Area(anExp.Current()) > 1);
        }
        Assert.Equal(2, aFaces);
        Assert.Equal(anArea, Area(aResult), 2);
    }

    [Fact]
    public void FixSplitFace_WithTwoPinches_SplitsAtEach()
    {
        // two vertices touching the first side: three lobes
        Face aFace = TolerantFace(P(0, 0), P(12, 0), P(12, 10), P(8, 1e-5), P(6, 10), P(4, 1e-5), P(0, 10));
        double anArea = Area(aFace);
        var aFix = new FixSmallFaceFixer();
        aFix.Init(aFace);
        aFix.SetPrecision(1e-4);
        aFix.FixSplitFace(aFace);
        Shape aResult = aFix.Shape();
        int aFaces = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces++;
        }
        Assert.Equal(3, aFaces);
        Assert.Equal(anArea, Area(aResult), 2);
    }

    // A wire round a square whose first side is a curve with a loop in it: crossing itself at a point well away from
    // its ends, the loop as high as about theSize
    private static (Wire W, Face F, Edge Looped) WireWithALoop(double theSize)
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, new PlaneSurface(new CoordinateSystem(P(0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), 1e-7);
        var aPoles = new[] { P(0, 0), P(15, theSize), P(-5, theSize), P(10, 0) };
        var aC3d = new BezierCurve3D(aPoles);
        var aC2d = new BezierCurve2D(aPoles.Select(p => new Point2D(p.X, p.Y)).ToArray());
        Vertex aV0 = TestShapes.Vertex(P(0, 0)), aV1 = TestShapes.Vertex(P(10, 0)), aV2 = TestShapes.Vertex(P(10, 10)), aV3 = TestShapes.Vertex(P(0, 10));
        Edge aLooped = TestShapes.Edge(aC3d, 0, 1, aV0, aV1);
        aB.UpdateEdge(aLooped, aC2d, aF, 1e-7);
        aB.Range(aLooped, aF, 0, 1);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aLooped);
        aB.Add(aW, TestShapes.Edge(aV1, aV2));
        aB.Add(aW, TestShapes.Edge(aV2, aV3));
        aB.Add(aW, TestShapes.Edge(aV3, aV0));
        return (aW, TestShapes.Frozen(aF), aLooped);
    }

    [Theory]
    [InlineData(0.01, -1)] // the default: tolerances
    [InlineData(0.01, 0)]
    [InlineData(0.01, 1)] // cutting the loop out
    [InlineData(0.5, -1)]
    [InlineData(0.5, 0)]
    [InlineData(0.5, 1)]
    public void FixSelfIntersection_ALoopInAPCurve_LeavesNoCrossing(double theLoopSize, int theMode)
    {
        (Wire aW, Face aF, _) = WireWithALoop(theLoopSize);
        Wire aFrozen = TestShapes.Frozen(aW);
        var aSnap = ShapeSnapshot.Of(aFrozen);
        var sfw = new WireFixer(aFrozen, aF, 1e-2);
        sfw.SetMaxTolerance(10);
        sfw.ModifyRemoveLoopMode() = theMode;
        sfw.FixSelfIntersectingEdgeMode() = 1;
        sfw.FixIntersectingEdgesMode() = 0;
        sfw.FixNonAdjacentIntersectingEdgesMode() = 0;
        sfw.FixSelfIntersection();
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aFrozen);
        // the ends of the wire's edges still chain round it, at no worse than the vertices' tolerances
        var sae = new EdgeAnalysis();
        var aList = new List<Edge>();
        for (var anIt = new Iterator(aRes); anIt.More(); anIt.Next())
        {
            aList.Add(ShapeCasts.Edge(anIt.Value()));
        }
        Assert.True(aList.Count >= 4);
        for (int i = 0; i < aList.Count; i++)
        {
            Vertex anEnd = sae.LastVertex(aList[i]), aStart = sae.FirstVertex(aList[(i + 1) % aList.Count]);
            Assert.True(Tool.Pnt(anEnd).DistanceTo(Tool.Pnt(aStart)) <= Tool.Tolerance(anEnd) + Tool.Tolerance(aStart) + 1e-9,
                        $"edges {i} and {i + 1} came apart");
        }
        Assert.True(sfw.StatusSelfIntersection(Done), "it should have found the loop");
        {
            // whichever way it was done, the analysis no longer finds a loop outside the vertices' tolerances
            var aLeft = new WireAnalysis(aRes, aF, 1e-2);
            var aPoints2d = new NCollection_Sequence<IntersectionPoint2D>();
            var aPoints3d = new NCollection_Sequence<Point3D>();
            for (int i = 1; i <= aLeft.EdgeCount; i++)
            {
                aPoints2d.Clear();
                aPoints3d.Clear();
                Assert.False(aLeft.CheckSelfIntersectingEdge(i, aPoints2d, aPoints3d), $"edge {i} still crosses itself");
            }
        }
    }

    private static int Wires(in Shape theS)
    {
        int aCount = 0;
        for (var anExp = new Explorer(theS, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            aCount++;
        }
        return aCount;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FixSmallEdges_AHoleOfNothingButSmallEdges_ShrinksAndLeavesTheFace(bool theDrop)
    {
        Face anOuter = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        Wire aHole = ShapeFixPolylines.Polyline(true, P(5, 5), P(5, 5.0001), P(5.0001, 5.0001), P(5.0001, 5));
        var aMake = new MakeFace(anOuter);
        aMake.Add(aHole);
        Face aFace = aMake.Face();
        var aFix = new WireframeFixer(aFace);
        aFix.SetPrecision(1e-3);
        aFix.ModeDropSmallEdges() = theDrop;
        aFix.FixSmallEdges();
        Shape aResult = aFix.Shape();
        // merging stops at the last two edges of a closed loop, whose ends aren't one vertex; the hole's area is nothing
        Assert.Equal(2, Wires(aResult));
        Assert.Equal(100, Area(aResult), 3);
        int anEdges = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            anEdges++;
        }
        Assert.True(anEdges < 8, $"{anEdges} edges");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FixSmallEdges_AFreeWireOfNothingButSmallEdges_HasNoneLeft(bool theDrop)
    {
        Wire aWire = TestShapes.Frozen(ShapeFixPolylines.Polyline(true, P(5, 5), P(5, 5.0001), P(5.0001, 5.0001)));
        var aFix = new WireframeFixer(aWire);
        aFix.SetPrecision(1e-3);
        aFix.ModeDropSmallEdges() = theDrop;
        aFix.FixSmallEdges();
        Shape aResult = aFix.Shape();
        if (!aResult.IsNull)
        {
            // one edge left at most, or none: nothing can be merged with an edge that's still there
            int aEdges = 0;
            for (var anExp = new Explorer(aResult, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                aEdges++;
            }
            Assert.True(aEdges <= 3);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FixSmallEdges_SmallEdgesOfDifferentNeighbours_InABox_KeepItsVolume(bool theDrop)
    {
        // a box with a small chamfer along one edge: the strip's short ends are small edges between faces which differ
        Face aBase = PlanarFace(P(0, 0), P(10, 0), P(10, 9.9999), P(9.9999, 10), P(0, 10));
        Shape aPrism = new ApiMakePrism(aBase, new Vector3D(0, 0, 5)).Shape();
        double aVolume = Volume(aPrism);
        var aFix = new WireframeFixer(aPrism);
        aFix.SetPrecision(1e-3);
        aFix.ModeDropSmallEdges() = theDrop;
        aFix.FixSmallEdges();
        Assert.True(Math.Abs(Volume(aFix.Shape()) - aVolume) < 1e-2 * aVolume);
    }

    [Fact]
    public void FixSmallEdges_KeepsAnInternalVertexOfAnEdgeMerged()
    {
        // the neighbour of the small edge has a vertex inside it, which the merged edge must keep
        Vertex aV0 = TestShapes.Vertex(P(0, 0)), aV1 = TestShapes.Vertex(P(5, 0)), aV2 = TestShapes.Vertex(P(5.0001, 0)),
               aV3 = TestShapes.Vertex(P(10, 0)), aV4 = TestShapes.Vertex(P(10, 10)), aV5 = TestShapes.Vertex(P(0, 10));
        var aB = new BrepBuilder();
        Edge aWithInner = TestShapes.Edge(aV0, aV1);
        Vertex anInner = TestShapes.Vertex(P(2, 0));
        aB.Add(aWithInner, anInner.Oriented(Orientation.Internal));
        aB.UpdateVertex(ShapeCasts.Vertex(anInner.Oriented(Orientation.Internal)), 2, aWithInner, 1e-7);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aWithInner);
        aB.Add(aW, TestShapes.Edge(aV1, aV2));
        aB.Add(aW, TestShapes.Edge(aV2, aV3));
        aB.Add(aW, TestShapes.Edge(aV3, aV4));
        aB.Add(aW, TestShapes.Edge(aV4, aV5));
        aB.Add(aW, TestShapes.Edge(aV5, aV0));
        aW.Closed(true);
        Face aFace = new MakeFace(aW, true).Face();
        var aFix = new WireframeFixer(aFace);
        aFix.SetPrecision(1e-3);
        Assert.True(aFix.FixSmallEdges());
        Shape aResult = aFix.Shape();
        Assert.True(ShortestEdge(aResult) > 1e-3);
        int anInternal = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            for (var anIt = new Iterator(anExp.Current(), false); anIt.More(); anIt.Next())
            {
                anInternal += anIt.Value().Orientation == Orientation.Internal ? 1 : 0;
            }
        }
        Assert.Equal(1, anInternal);
    }
}
