// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
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
}
