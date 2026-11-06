// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_IntersectionTool (unit 6H2). Its fixes write in place, so they run on unfrozen shapes, as
/// ShapeFix_Wire and ShapeFix_Face run them on their copies.
/// </summary>
public class ShapeFix_IntersectionToolTests
{
    private static Point3D P(double x, double y) => new(x, y, 0.0);

    // An unfrozen planar face bounded by the polygon.
    private static Face PlanarFace(params Point3D[] thePoints)
    {
        var aPoly = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        return new MakeFace(aPoly.InternalWire(), true).InternalFace();
    }

    private static Wire OuterWire(in Face theFace)
    {
        var anIt = new Iterator(theFace);
        return ShapeCasts.Wire(anIt.Value());
    }

    private static int NbEdges(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, ShapeType.Edge, aMap);
        return aMap.Count;
    }

    [Fact]
    public void FixSelfIntersectWire_BowTie_SplitsBothCrossingEdges()
    {
        // (0,0) -> (2,2) -> (2,0) -> (0,2) -> back: two opposite edges cross at (1,1)
        Face aFace = PlanarFace(P(0, 0), P(2, 2), P(2, 0), P(0, 2));
        var sewd = new ExtendWireData(OuterWire(aFace));
        var anOriginals = new List<Edge>();
        for (int i = 1; i <= sewd.EdgeCount; i++)
        {
            anOriginals.Add(sewd.Edge(i));
        }
        var aContext = new ReShapeBuilder();
        var aTool = new IntersectionToolFixer(aContext, 1.0e-7);

        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.True(aTool.FixSelfIntersectWire(sewd, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));

        Assert.Equal(2, aNbSplit);
        Assert.Equal(0, aNbCut);
        Assert.Equal(0, aNbRemoved);
        Assert.Equal(6, sewd.EdgeCount);
        Assert.Equal(2, anOriginals.Count(e => aContext.IsRecorded(e))); // the two crossing edges were replaced

        // the new vertex is shared by the four halves and lies at (1,1)
        var sae = new EdgeAnalysis();
        int aNbAtCross = 0;
        for (int i = 1; i <= sewd.EdgeCount; i++)
        {
            Point3D anEnd = Tool.Pnt(sae.LastVertex(sewd.Edge(i)));
            if (anEnd.DistanceTo(new Point3D(1, 1, 0)) < 1.0e-9)
            {
                aNbAtCross++;
                int j = i == sewd.EdgeCount ? 1 : i + 1;
                Assert.True(sae.LastVertex(sewd.Edge(i)).IsSame(sae.FirstVertex(sewd.Edge(j))));
            }
        }
        Assert.Equal(2, aNbAtCross);
    }

    [Fact]
    public void FixSelfIntersectWire_NoContextOrNoFace_DoesNothing()
    {
        Face aFace = PlanarFace(P(0, 0), P(2, 2), P(2, 0), P(0, 2));
        var sewd = new ExtendWireData(OuterWire(aFace));
        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.False(new IntersectionToolFixer(null, 1.0e-7)
                         .FixSelfIntersectWire(sewd, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.False(new IntersectionToolFixer(new ReShapeBuilder(), 1.0e-7)
                         .FixSelfIntersectWire(sewd, new Face(), ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.Equal(4, sewd.EdgeCount);
    }

    [Fact]
    public void FixSelfIntersectWire_Square_IsUnchanged()
    {
        Face aFace = PlanarFace(P(0, 0), P(2, 0), P(2, 2), P(0, 2));
        var sewd = new ExtendWireData(OuterWire(aFace));
        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.False(new IntersectionToolFixer(new ReShapeBuilder(), 1.0e-7)
                         .FixSelfIntersectWire(sewd, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.Equal(4, sewd.EdgeCount);
    }

    [Fact]
    public void FixIntersectingWires_CrossingWires_SplitsTheirEdgesAndReplacesTheFace()
    {
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        // an inner rectangle crossing the side x = 4
        var aPoly = new MakePolygon(new Point3D(3, 1, 0), new Point3D(5, 1, 0), new Point3D(5, 2, 0),
                                                   new Point3D(3, 2, 0), true);
        Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BrepBuilder().Add(aFace, anInner);
        Face anOriginal = aFace;
        var aContext = new ReShapeBuilder();
        var aTool = new IntersectionToolFixer(aContext, 1.0e-7);

        Assert.True(aTool.FixIntersectingWires(ref aFace));

        Assert.False(ReferenceEquals(aFace.TShape(), anOriginal.TShape()));
        Assert.True(aContext.IsRecorded(anOriginal));
        int aNbWires = 0;
        int aNbEdgesTotal = 0;
        for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next(), aNbWires++)
        {
            aNbEdgesTotal += NbEdges(anIt.Value());
        }
        Assert.Equal(2, aNbWires);
        Assert.True(aNbEdgesTotal > 8);
    }

    [Fact]
    public void FixIntersectingWires_SingleWire_ReturnsFalse()
    {
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        Face anOriginal = aFace;
        Assert.False(new IntersectionToolFixer(new ReShapeBuilder(), 1.0e-7).FixIntersectingWires(ref aFace));
        Assert.True(ReferenceEquals(aFace.TShape(), anOriginal.TShape()));
    }

    [Fact]
    public void SplitEdge_AtAParameter_GivesTwoEdgesSharingTheVertex()
    {
        Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).InternalEdge();
        Vertex aV = new MakeVertex(new Point3D(4, 0, 0)).InternalVertex();
        var aTool = new IntersectionToolFixer(null, 1.0e-7);

        Edge aE1 = new(), aE2 = new();
        Assert.True(aTool.SplitEdge(anEdge, 4.0, aV, aFace, ref aE1, ref aE2, 1.0e-7));
        Tool.Range(aE1, out double f1, out double l1);
        Tool.Range(aE2, out double f2, out double l2);
        Assert.Equal(0.0, f1, 1e-12);
        Assert.Equal(4.0, l1, 1e-12);
        Assert.Equal(4.0, f2, 1e-12);
        Assert.Equal(10.0, l2, 1e-12);
        var sae = new EdgeAnalysis();
        Assert.True(sae.LastVertex(aE1).IsSame(aV));
        Assert.True(sae.FirstVertex(aE2).IsSame(aV));

        // an end vertex or an end parameter: no split
        Assert.False(aTool.SplitEdge(anEdge, 4.0, sae.FirstVertex(anEdge), aFace, ref aE1, ref aE2, 1.0e-7));
        Assert.False(aTool.SplitEdge(anEdge, 10.0, aV, aFace, ref aE1, ref aE2, 1.0e-7));
    }

    [Fact]
    public void SplitEdge_VertexOffTheEdge_RaisesItsTolerance()
    {
        Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).InternalEdge();
        Vertex aV = new MakeVertex(new Point3D(4, 0.01, 0)).InternalVertex();
        Edge aE1 = new(), aE2 = new();
        Assert.True(new IntersectionToolFixer(null, 1.0e-7).SplitEdge(anEdge, 4.0, aV, aFace, ref aE1, ref aE2, 1.0e-7));
        Assert.True(Tool.Tolerance(aV) >= 0.01);
    }

    [Fact]
    public void CutEdge_ChangesTheRangeInPlace_AndRefusesFrozenEdges()
    {
        Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).InternalEdge();
        var aTool = new IntersectionToolFixer(null, 1.0e-7);
        bool isCutLine = false;

        Assert.True(aTool.CutEdge(anEdge, 10.0, 6.0, aFace, ref isCutLine));
        Tool.Range(anEdge, out double f, out double l);
        Assert.Equal(6.0, f);
        Assert.Equal(10.0, l);
        Assert.False(isCutLine);

        // too small a cut, or the whole range: nothing
        Assert.False(aTool.CutEdge(anEdge, 10.0, 10.0, aFace, ref isCutLine));
        Assert.False(aTool.CutEdge(anEdge, 10.0, 6.0, aFace, ref isCutLine));

        Edge aFrozen = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Assert.ThrowsAny<Exception>(() => aTool.CutEdge(aFrozen, 10.0, 6.0, aFace, ref isCutLine));
    }

    // Every vertex of the face made as tolerant as a STEP file's often are
    private static void RaiseVertexTolerances(in Face theFace, double theTol)
    {
        var aB = new BrepBuilder();
        for (var anExp = new Explorer(theFace, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            aB.UpdateVertex(ShapeCasts.Vertex(anExp.Current()), theTol);
        }
    }

    // One pass over the wire: what it did, and that a second pass finds nothing left to do at the same tolerances
    private static (int Split, int Cut, int Removed, ExtendWireData Wire) FixSelf(in Face theFace, double theTol = 1e-2)
    {
        RaiseVertexTolerances(theFace, theTol);
        var sewd = new ExtendWireData(OuterWire(theFace));
        var aTool = new IntersectionToolFixer(new ReShapeBuilder(), 1.0e-7);
        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        aTool.FixSelfIntersectWire(sewd, theFace, ref aNbSplit, ref aNbCut, ref aNbRemoved);
        return (aNbSplit, aNbCut, aNbRemoved, sewd);
    }

    private static void AssertChainConnected(ExtendWireData theWire, bool theClosed = true)
    {
        var sae = new EdgeAnalysis();
        int aLast = theClosed ? theWire.EdgeCount : theWire.EdgeCount - 1;
        for (int i = 1; i <= aLast; i++)
        {
            Edge anA = theWire.Edge(i), aB = theWire.Edge(i == theWire.EdgeCount ? 1 : i + 1);
            Assert.True(sae.LastVertex(anA).IsSame(sae.FirstVertex(aB)) ||
                        Tool.Pnt(sae.LastVertex(anA)).DistanceTo(Tool.Pnt(sae.FirstVertex(aB))) <
                        Tool.Tolerance(sae.LastVertex(anA)) + Tool.Tolerance(sae.FirstVertex(aB)),
                        $"edges {i} and {i + 1} aren't connected");
        }
    }

    [Fact]
    public void FixSelfIntersectWire_CrossingJustBeforeAnEdgesEnd_CutsThatEdgeAndSplitsTheOther()
    {
        // the third edge overruns the first by 1e-3 at its end, within the vertex's tolerance: it's cut back to the
        // crossing, its end vertex taking the crossing in, and the first is split there
        Point3D aA = P(0, 0), aB = P(4, 4), aC = P(4, 0), anEnd = P(1.999, 2.001), aD = P(0, 4);
        Face aFace = PlanarFace(aA, aB, aC, anEnd, aD);
        (int aSplit, int aCut, _, ExtendWireData aW) = FixSelf(aFace);
        Assert.Equal(1, aSplit);
        Assert.Equal(1, aCut);
        Assert.Equal(6, aW.EdgeCount);
        AssertChainConnected(aW);
    }

    [Fact]
    public void FixSelfIntersectWire_TwoCrossingsNearEnds_CutsBothEdges_WithoutSplitting()
    {
        // two edges overrun each other's ends, a corner made by hand with too long sides, at a shared tolerant vertex
        Face aFace = PlanarFace(P(0, 0), P(4.001, 0), P(4.001, -0.001), P(4, -0.001), P(4, 4), P(0, 4));
        (int aSplit, int aCut, _, ExtendWireData aW) = FixSelf(aFace);
        Assert.Equal(0, aSplit);
        Assert.True(aCut >= 1);
        AssertChainConnected(aW);
    }

    [Fact]
    public void FixSelfIntersectWire_AVertexOnAnotherEdge_SplitsThatEdgeUsingTheVertex()
    {
        // the wire's vertex (2, 2) touches the first diagonal in its middle, without crossing it
        Face aFace = PlanarFace(P(0, 0), P(4, 4), P(4, 0), P(2, 2), P(0, 4));
        (int aSplit, _, _, ExtendWireData aW) = FixSelf(aFace);
        Assert.True(aSplit >= 1);
        Assert.Equal(6, aW.EdgeCount);
        var sae = new EdgeAnalysis();
        int aAtTouch = 0;
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            aAtTouch += Tool.Pnt(sae.LastVertex(aW.Edge(i))).DistanceTo(P(2, 2)) < 1e-2 ? 1 : 0;
        }
        Assert.Equal(2, aAtTouch);
        AssertChainConnected(aW);
    }

    [Fact]
    public void FixSelfIntersectWire_TwoVerticesTouching_UnitesThem()
    {
        // a figure of eight: two different vertices at (2, 2)
        Face aFace = PlanarFace(P(0, 0), P(2, 2), P(4, 0), P(4, 4), P(2, 2), P(0, 4));
        (_, _, int aRemoved, ExtendWireData aW) = FixSelf(aFace);
        Assert.Equal(6, aW.EdgeCount);
        var sae = new EdgeAnalysis();
        var aVertices = new List<Vertex>();
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            Vertex aV = sae.LastVertex(aW.Edge(i));
            if (Tool.Pnt(aV).DistanceTo(P(2, 2)) < 1e-2)
            {
                aVertices.Add(aV);
            }
        }
        Assert.Equal(2, aVertices.Count);
        Assert.True(aRemoved >= 0);
        // both corners now use the one vertex
        Assert.True(aVertices[0].IsSame(aVertices[1]), "the two vertices should have become one");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FixSelfIntersectWire_AnEdgeDoublingBackOverAnother_CutsTheOverlap(bool theReversedWire)
    {
        // the last side runs back along the first by 1e-3: the two overlap in a segment
        Point3D[] aPoints = [P(0, 0), P(4, 0), P(4, 4), P(0.001, 4), P(0.001, 0)];
        if (theReversedWire)
        {
            Array.Reverse(aPoints);
        }
        Face aFace = PlanarFace(aPoints);
        (_, int aCut, _, ExtendWireData aW) = FixSelf(aFace);
        Assert.True(aCut >= 1 || aW.EdgeCount >= 5);
        AssertChainConnected(aW);
    }

    [Fact]
    public void FixIntersectingWires_AHoleTouchingTheOutline_SplitsAtTheVertex()
    {
        // the hole's corner lies on the outline's side: T-junction
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        var aPoly = new MakePolygon(new Point3D(4, 2, 0), new Point3D(3, 1, 0), new Point3D(2, 2, 0), new Point3D(3, 3, 0), true);
        Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BrepBuilder().Add(aFace, anInner);
        RaiseVertexTolerances(aFace, 1e-3);
        var aContext = new ReShapeBuilder();
        Assert.True(new IntersectionToolFixer(aContext, 1.0e-7).FixIntersectingWires(ref aFace));
        int aEdges = 0;
        for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next())
        {
            aEdges += NbEdges(anIt.Value());
        }
        Assert.Equal(9, aEdges);
    }

    [Fact]
    public void FixIntersectingWires_TwoCornersTouching_UnitesTheirVertices()
    {
        // the hole's corner at the outline's corner
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        var aPoly = new MakePolygon(new Point3D(4, 4, 0), new Point3D(3, 2, 0), new Point3D(2, 2, 0), new Point3D(2, 3, 0), true);
        Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BrepBuilder().Add(aFace, anInner);
        RaiseVertexTolerances(aFace, 1e-3);
        var aContext = new ReShapeBuilder();
        new IntersectionToolFixer(aContext, 1.0e-7).FixIntersectingWires(ref aFace);
        // the corner is now one vertex
        var aAtCorner = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (var anExp = new Explorer(aFace, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            if (Tool.Pnt(ShapeCasts.Vertex(anExp.Current())).DistanceTo(P(4, 4)) < 1e-2)
            {
                aAtCorner.Add(anExp.Current());
            }
        }
        Assert.Single(aAtCorner);
    }

    [Fact]
    public void FixIntersectingWires_AHoleSharingPartOfASide_CutsTheOverlap()
    {
        // the hole's edge lies along the outline's side, a segment of intersection
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        var aPoly = new MakePolygon(new Point3D(1, 0, 0), new Point3D(3, 0, 0), new Point3D(3, 2, 0), new Point3D(1, 2, 0), true);
        Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BrepBuilder().Add(aFace, anInner);
        RaiseVertexTolerances(aFace, 1e-3);
        new IntersectionToolFixer(new ReShapeBuilder(), 1.0e-7).FixIntersectingWires(ref aFace);
        int aWires = 0;
        for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next())
        {
            aWires++;
        }
        Assert.Equal(2, aWires);
    }

    [Fact]
    public void FixIntersectingWires_CrossingNearAVertex_CutsAndKeepsTheFaceSound()
    {
        // the hole's corner pokes through the side by 1e-3, within its vertices' tolerance
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        var aPoly = new MakePolygon(new Point3D(4.001, 2, 0), new Point3D(3, 1, 0), new Point3D(2, 2, 0), new Point3D(3, 3, 0), true);
        Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BrepBuilder().Add(aFace, anInner);
        RaiseVertexTolerances(aFace, 1e-2);
        Assert.True(new IntersectionToolFixer(new ReShapeBuilder(), 1.0e-7).FixIntersectingWires(ref aFace));
        int aEdges = 0;
        for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next())
        {
            aEdges += NbEdges(anIt.Value());
        }
        Assert.True(aEdges >= 8);
    }

    private static Face EmptyPlaneFace()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), 1e-7);
        return aF;
    }

    private static Vertex Tolerant(in Point3D theP)
    {
        var aB = new BrepBuilder();
        Vertex aV = new();
        aB.MakeVertex(ref aV, theP, 1e-2);
        return aV;
    }

    private static Edge Segment(in Vertex theA, in Vertex theB)
    {
        Point3D aP = Tool.Pnt(theA), aQ = Tool.Pnt(theB);
        var aB = new BrepBuilder();
        Edge anE = new();
        aB.MakeEdge(ref anE, new LineCurve3D(aP, new Direction3D(new Vector3D(aP, aQ))), 1e-7);
        aB.Add(anE, theA.Oriented(Orientation.Forward));
        aB.Add(anE, theB.Oriented(Orientation.Reversed));
        aB.Range(anE, 0, aP.DistanceTo(aQ));
        return anE;
    }

    private static ExtendWireData Loose(params Edge[] theEdges)
    {
        var aData = new ExtendWireData();
        foreach (Edge anE in theEdges)
        {
            aData.Add(anE);
        }
        return aData;
    }

    private static Edge Oriented(in Vertex theT, in Vertex theFar, bool theTouchFirst) =>
        theTouchFirst ? Segment(theT, theFar) : Segment(theFar, theT);

    public static TheoryData<bool, bool, bool, bool> FourFlags()
    {
        var aData = new TheoryData<bool, bool, bool, bool>();
        foreach (bool a in new[] { false, true })
        {
            foreach (bool b in new[] { false, true })
            {
                foreach (bool c in new[] { false, true })
                {
                    foreach (bool d in new[] { false, true })
                    {
                        aData.Add(a, b, c, d);
                    }
                }
            }
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(FourFlags))]
    public void FixSelfIntersectWire_TwoEndsTouching_BecomeOneVertex_InTheirNeighboursToo(
        bool theFirstOfX, bool theFirstOfY, bool theNeighbourBefore, bool theNeighbourFirst)
    {
        // Edges X and Y, not adjacent in the wire, each have an end at (2, 2), at two different vertices. One of Y's
        // neighbours shares Y's vertex there. Whichever ends they are, the wire comes out with one vertex at (2, 2).
        Face aFace = EmptyPlaneFace();
        Vertex aT1 = Tolerant(P(2, 2)), aT2 = Tolerant(P(2, 2));
        Edge aX = Oriented(aT1, Tolerant(P(0, 0)), theFirstOfX);
        Edge aY = Oriented(aT2, Tolerant(P(4, 0)), theFirstOfY);
        Edge aFarM = Segment(Tolerant(P(10, 10)), Tolerant(P(11, 10)));
        Edge aFarN = Segment(Tolerant(P(12, 10)), Tolerant(P(13, 10)));
        Edge aNeighbour = Oriented(aT2, Tolerant(theNeighbourBefore ? P(0, 4) : P(4, 4)), theNeighbourFirst);
        ExtendWireData aW = theNeighbourBefore ? Loose(aX, aNeighbour, aY, aFarN) : Loose(aX, aFarM, aY, aNeighbour);

        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.True(new IntersectionToolFixer(new ReShapeBuilder(), 1e-7).FixSelfIntersectWire(aW, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));

        Assert.Equal(0, aNbSplit);
        Assert.Equal(4, aW.EdgeCount);
        var aAtTouch = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            for (var anExp = new Explorer(aW.Edge(i), ShapeType.Vertex); anExp.More(); anExp.Next())
            {
                if (Tool.Pnt(ShapeCasts.Vertex(anExp.Current())).DistanceTo(P(2, 2)) < 1e-9)
                {
                    aAtTouch.Add(anExp.Current());
                }
            }
        }
        Assert.Single(aAtTouch);
    }

    [Fact]
    public void FixSelfIntersectWire_ALineCrossingAnArcTwice_SplitsAtTheFirstCrossing()
    {
        // two points of intersection: the one to use is the one first along the first curve
        Face aFace = EmptyPlaneFace();
        Edge aLine = Segment(Tolerant(P(0, 0)), Tolerant(P(4, 0)));
        var anArc = new CircleCurve3D(new Frame3D(new Point3D(2, -0.5, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        Edge anArcEdge = TestShapes.Edge(anArc, 0, Math.PI, Tolerant(anArc.Value(0)), Tolerant(anArc.Value(Math.PI)));
        ExtendWireData aW = Loose(aLine, Segment(Tolerant(P(4, 3)), Tolerant(P(0, 3))), anArcEdge, Segment(Tolerant(P(4, 5)), Tolerant(P(0, 5))));

        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.True(new IntersectionToolFixer(new ReShapeBuilder(), 1e-7).FixSelfIntersectWire(aW, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));

        Assert.True(aNbSplit >= 2);
        Assert.True(aW.EdgeCount >= 6);
        // the split edges' ends are at a crossing: on the line, a distance of 1 from the arc's centre
        var sae = new EdgeAnalysis();
        int aOnCrossing = 0;
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            Point3D aEnd = Tool.Pnt(sae.LastVertex(aW.Edge(i)));
            aOnCrossing += Math.Abs(aEnd.Y) < 1e-6 && Math.Abs(aEnd.DistanceTo(new Point3D(2, -0.5, 0)) - 1) < 1e-6 ? 1 : 0;
        }
        Assert.True(aOnCrossing >= 2, $"{aOnCrossing} edges end at a crossing");
    }

    [Fact]
    public void FixSelfIntersectWire_TwoEdgesOverlappingInTheirMiddles_RemovesTheOverlap()
    {
        // The second side retraces the middle of the first, too long a stretch for any vertex's tolerance. It's all
        // common, so the first is split twice around it, and the first's middle and the whole second go.
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 1), P(1, 1), P(1, 0), P(3, 0), P(3, -1), P(0, -1));
        (int aSplit, _, int aRemoved, ExtendWireData aW) = FixSelf(aFace, 1e-3);
        Assert.Equal(2, aRemoved);
        Assert.Equal(2, aSplit);
        Assert.Equal(8, aW.EdgeCount);
        // what remains of the lower sides is outside the common stretch
        var sae = new EdgeAnalysis();
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            Point3D aA = Tool.Pnt(sae.FirstVertex(aW.Edge(i))), aB = Tool.Pnt(sae.LastVertex(aW.Edge(i)));
            if (Math.Abs(aA.Y) < 1e-6 && Math.Abs(aB.Y) < 1e-6)
            {
                Assert.True(Math.Max(aA.X, aB.X) <= 1 + 1e-6 || Math.Min(aA.X, aB.X) >= 3 - 1e-6, $"{aA} to {aB} still overlaps");
            }
        }
    }

    [Fact]
    public void FixSelfIntersectWire_ASmallOverlapInTheirMiddles_SplitsBothAtOneNewVertex()
    {
        // an overlap of 1e-3 within vertex tolerances of 1e-2: it's taken for a touch, not removed
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 1), P(1, 1), P(2, 0), P(2.001, 0), P(3, -1), P(0, -1));
        (int aSplit, _, int aRemoved, ExtendWireData aW) = FixSelf(aFace, 1e-2);
        Assert.Equal(0, aRemoved);
        Assert.True(aSplit >= 1);
        Assert.True(aW.EdgeCount >= 9);
        AssertChainConnected(aW);
    }

    [Theory]
    [InlineData(0, 2)] // the second starts where the first starts
    [InlineData(2, 4)] // ... ends where it ends
    [InlineData(1, 4)]
    [InlineData(0, 3)]
    public void FixSelfIntersectWire_AnOverlapReachingAnEnd_RemovesOrCutsWhatsCommon(double theFrom, double theTo)
    {
        // loose edges: the overlap reaches a vertex of the long edge
        Edge aLong = Segment(Tolerant(P(0, 0)), Tolerant(P(4, 0)));
        Edge aShort = Segment(Tolerant(P(theFrom, 0)), Tolerant(P(theTo, 0)));
        Edge aFar1 = Segment(Tolerant(P(4, 3)), Tolerant(P(5, 3))), aFar2 = Segment(Tolerant(P(4, 5)), Tolerant(P(5, 5)));
        Face aFace = FaceWithWireOf(aLong, aFar1, aShort, aFar2);
        ExtendWireData aW = Loose(aLong, aFar1, aShort, aFar2);

        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.True(new IntersectionToolFixer(new ReShapeBuilder(), 1e-7).FixSelfIntersectWire(aW, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.True(aNbSplit + aNbCut + aNbRemoved > 0);
        // on the line, nothing is covered twice any more, beyond the vertices' tolerance
        var aSpans = new List<(double A, double B)>();
        var sae = new EdgeAnalysis();
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            Assert.True(sae.PCurve(aW.Edge(i), aFace, out Curve2D? aC, out double aF, out double aL));
            if (Math.Abs(aC!.Value(aF).Y) < 1e-6 && Math.Abs(aC.Value(aL).Y) < 1e-6)
            {
                aSpans.Add((Math.Min(aC.Value(aF).X, aC.Value(aL).X), Math.Max(aC.Value(aF).X, aC.Value(aL).X)));
            }
        }
        aSpans.Sort();
        for (int i = 1; i < aSpans.Count; i++)
        {
            Assert.True(aSpans[i].A >= aSpans[i - 1].B - 2e-2, $"{aSpans[i - 1]} overlaps {aSpans[i]}");
        }
    }

    [Theory]
    [InlineData(1, 3, 1e-3)] // a stretch in the side's middle, long
    [InlineData(0, 2, 1e-3)] // from the side's first vertex
    [InlineData(2, 4, 1e-3)] // to its last
    [InlineData(1, 1.001, 1e-2)] // short enough to be a touch
    public void FixIntersectingWires_AHoleAlongTheOutline_LeavesNoSideDoubled(double theFrom, double theTo, double theTol)
    {
        Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        var aPoly = new MakePolygon(new Point3D(theFrom, 0, 0), new Point3D(theTo, 0, 0), new Point3D(theTo, 2, 0), new Point3D(theFrom, 2, 0), true);
        Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BrepBuilder().Add(aFace, anInner);
        RaiseVertexTolerances(aFace, theTol);
        var aTool = new IntersectionToolFixer(new ReShapeBuilder(), 1e-7);
        Assert.True(aTool.FixIntersectingWires(ref aFace));

        var aSpans = new List<(double A, double B)>();
        var sae = new EdgeAnalysis();
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Assert.True(sae.PCurve(anE, aFace, out Curve2D? aC, out double aF, out double aL));
            if (Math.Abs(aC!.Value(aF).Y) < 1e-6 && Math.Abs(aC.Value(aL).Y) < 1e-6)
            {
                aSpans.Add((Math.Min(aC.Value(aF).X, aC.Value(aL).X), Math.Max(aC.Value(aF).X, aC.Value(aL).X)));
            }
        }
        aSpans.Sort();
        for (int i = 1; i < aSpans.Count; i++)
        {
            Assert.True(aSpans[i].A >= aSpans[i - 1].B - 2e-2, $"{aSpans[i - 1]} overlaps {aSpans[i]}");
        }
    }

    // A face on the plane whose vertices are those of the edges (the fix takes its largest tolerance from the face's vertices)
    private static Face FaceWithWireOf(params Edge[] theEdges)
    {
        Face aFace = EmptyPlaneFace();
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        aB.Add(aFace, aW);
        return aFace;
    }

    private static Edge Polyline(params Point3D[] thePoints)
    {
        double[] aKnots = new double[thePoints.Length];
        int[] aMults = new int[thePoints.Length];
        for (int i = 0; i < thePoints.Length; i++)
        {
            aKnots[i] = i;
            aMults[i] = i == 0 || i == thePoints.Length - 1 ? 2 : 1;
        }
        var aCurve = new BSplineCurve3D(thePoints, aKnots, aMults, 1);
        return TestShapes.Edge(aCurve, 0, thePoints.Length - 1, Tolerant(thePoints[0]), Tolerant(thePoints[^1]));
    }

    [Fact]
    public void FixSelfIntersectWire_TwoPolylinesSharingAStretch_SplitsEachInThree_AndRemovesTheCommonParts()
    {
        // neither has a vertex at the common stretch, from (1, 0) to (3, 0)
        Edge aUpper = Polyline(P(0, 1), P(1, 0), P(3, 0), P(4, 1));
        Edge aLower = Polyline(P(4, -1), P(3, 0), P(1, 0), P(0, -1));
        Edge aFar1 = Segment(Tolerant(P(5, 3)), Tolerant(P(6, 3)));
        Edge aFar2 = Segment(Tolerant(P(5, 5)), Tolerant(P(6, 5)));
        Face aFace = FaceWithWireOf(aUpper, aFar1, aLower, aFar2);
        ExtendWireData aW = Loose(aUpper, aFar1, aLower, aFar2);

        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.True(new IntersectionToolFixer(new ReShapeBuilder(), 1e-7).FixSelfIntersectWire(aW, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));

        Assert.Equal(2, aNbRemoved);
        Assert.Equal(4, aNbSplit);
        Assert.Equal(6, aW.EdgeCount);
        // nothing remains along the common stretch
        var sae = new EdgeAnalysis();
        for (int i = 1; i <= aW.EdgeCount; i++)
        {
            Assert.True(sae.PCurve(aW.Edge(i), aFace, out Curve2D? aC, out double aF, out double aL));
            Point2D aMid = aC!.Value((aF + aL) / 2);
            Assert.False(Math.Abs(aMid.Y) < 1e-6 && aMid.X > 1 + 1e-6 && aMid.X < 3 - 1e-6, $"{aMid} is in the common stretch");
        }
    }

    [Fact]
    public void FixIntersectingWires_TwoPolylinesSharingAStretch_ShareOneEdgeThere()
    {
        // the same in two wires of a face, each closed by a straight edge
        Vertex aA = Tolerant(P(0, 1)), aB = Tolerant(P(4, 1)), aC = Tolerant(P(4, -1)), aD = Tolerant(P(0, -1));
        Edge Line(Point3D a0, Point3D a1, Point3D a2, Point3D a3, Vertex aV0, Vertex aV3)
        {
            var aCurve = new BSplineCurve3D(new[] { a0, a1, a2, a3 }, [0.0, 1.0, 2.0, 3.0], [2, 1, 1, 2], 1);
            return TestShapes.Edge(aCurve, 0, 3, aV0, aV3);
        }
        Edge anUpper = Line(P(0, 1), P(1, 0), P(3, 0), P(4, 1), aA, aB);
        Edge aLower = Line(P(4, -1), P(3, 0), P(1, 0), P(0, -1), aC, aD);
        var aBuilder = new BrepBuilder();
        Wire aW1 = new(), aW2 = new();
        aBuilder.MakeWire(ref aW1);
        aBuilder.Add(aW1, anUpper);
        aBuilder.Add(aW1, Segment(aB, aA));
        aBuilder.MakeWire(ref aW2);
        aBuilder.Add(aW2, aLower);
        aBuilder.Add(aW2, Segment(aD, aC));
        Face aFace = EmptyPlaneFace();
        aBuilder.Add(aFace, aW1);
        aBuilder.Add(aFace, aW2);

        Assert.True(new IntersectionToolFixer(new ReShapeBuilder(), 1e-7).FixIntersectingWires(ref aFace));
        // each wire is now its three parts and its closing edge, and the middle part is one edge, shared by both wires
        var aWires = new List<List<Edge>>();
        for (var anIt = new Iterator(aFace); anIt.More(); anIt.Next())
        {
            var anEdges = new List<Edge>();
            for (var anEdge = new Iterator(anIt.Value()); anEdge.More(); anEdge.Next())
            {
                anEdges.Add(ShapeCasts.Edge(anEdge.Value()));
            }
            aWires.Add(anEdges);
        }
        Assert.Equal(2, aWires.Count);
        Assert.Equal(4, aWires[0].Count);
        Assert.Equal(4, aWires[1].Count);
        Edge aShared = Assert.Single(aWires[0], e => aWires[1].Exists(o => o.IsSame(e)));
        Assert.True(new EdgeAnalysis().PCurve(aShared, aFace, out Curve2D? aMiddle, out double aF, out double aL));
        Assert.True(aMiddle!.Value((aF + aL) / 2).DistanceTo(new Point2D(2, 0)) < 1e-6);
    }
}
