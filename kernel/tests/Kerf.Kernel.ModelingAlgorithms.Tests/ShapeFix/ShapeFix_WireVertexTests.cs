// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_WireVertex (unit 6H2) on frozen wires: the caller's edges are never written, the result is
/// frozen and holds the caller's own edges where the fixes didn't touch them.
/// </summary>
public class ShapeFix_WireVertexTests
{
    private static Vertex V(double x, double y) => new MakeVertex(new Point3D(x, y, 0)).Vertex();

    private static Edge E(in Vertex v1, in Vertex v2) => new MakeEdge(v1, v2).Edge();

    // A square whose first joint has two distinct vertices 1e-5 apart; the edges are frozen.
    private static Edge[] GappedSquare()
    {
        Vertex v0 = V(0, 0), v1 = V(1, 0), v1b = V(1 + 1.0e-5, 0), v2 = V(1, 1), v3 = V(0, 1);
        return [E(v0, v1), E(v1b, v2), E(v2, v3), E(v3, v0)];
    }

    private static Compound Compound(IEnumerable<Edge> theEdges)
    {
        var aC = new Compound();
        var aB = new BrepBuilder();
        aB.MakeCompound(ref aC);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aC, anE);
        }
        return aC;
    }

    private static ExtendWireData WireData(IEnumerable<Edge> theEdges)
    {
        var sbwd = new ExtendWireData();
        foreach (Edge anE in theEdges)
        {
            sbwd.Add(anE);
        }
        return sbwd;
    }

    private static List<Edge> Edges(in Wire theWire)
    {
        var aList = new List<Edge>();
        for (var anIt = new Iterator(theWire); anIt.More(); anIt.Next())
        {
            aList.Add(ShapeCasts.Edge(anIt.Value()));
        }
        return aList;
    }

    [Fact]
    public void Fix_GappedSquare_SharesEveryVertex_InputUnchanged()
    {
        Edge[] anEdges = GappedSquare();
        Compound anInput = Compound(anEdges);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anInput);
        var sfwv = new WireVertexFixer();
        sfwv.Init(WireData(anEdges), 1.0e-3);
        Assert.Equal(2, sfwv.Analyzer().Status(1)); // close
        Assert.Equal(1, sfwv.Analyzer().Status(2)); // same coords (OCCT: a shared vertex is never "same", see FixSame)

        Assert.Equal(4, sfwv.Fix());

        Wire aWire = sfwv.Wire();
        aSnap.AssertUnchanged(anInput);
        Assert.True(aWire.TShape()!.IsFrozen);
        List<Edge> aResult = Edges(aWire);
        Assert.Equal(4, aResult.Count);
        var sae = new EdgeAnalysis();
        for (int i = 0; i < 4; i++)
        {
            Assert.True(aResult[i].TShape()!.IsFrozen);
            Assert.True(sae.LastVertex(aResult[i]).IsSame(sae.FirstVertex(aResult[(i + 1) % 4])));
            Assert.False(ReferenceEquals(aResult[i].TShape(), anEdges[i].TShape())); // every edge was rebuilt
        }
        Assert.True(Tool.Tolerance(sae.LastVertex(aResult[0])) >= 1.0e-5);
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(0, sfwv.Analyzer().Status(i));
        }
    }

    [Fact]
    public void FixSame_SkipsJointsMarkedSame_KeepsTheCallersUntouchedEdge()
    {
        Edge[] anEdges = GappedSquare();
        Compound anInput = Compound(anEdges);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anInput);
        var sawv = new WireVertexAnalysis();
        sawv.Init(WireData(anEdges), 1.0e-3);
        sawv.SetPrecision(1.0e-3); // Init doesn't take it, as in OCCT
        sawv.Analyze();
        sawv.SetSameVertex(2); // joints edge 2/3 and 3/4 declared done: edge 3 is not touched
        sawv.SetSameVertex(3);
        var sfwv = new WireVertexFixer();
        sfwv.Init(sawv);

        Assert.Equal(2, sfwv.FixSame());

        List<Edge> aResult = Edges(sfwv.Wire());
        aSnap.AssertUnchanged(anInput);
        Assert.True(ReferenceEquals(aResult[2].TShape(), anEdges[2].TShape()));
        Assert.False(ReferenceEquals(aResult[0].TShape(), anEdges[0].TShape()));
        // OCCT: FixSame adds the vertex without removing the edge's own: edge 2 now has three vertices, the first
        // still its old one
        var sae = new EdgeAnalysis();
        Vertex aV1 = sae.LastVertex(aResult[0]);
        int aNbVertices = 0;
        bool hasV1 = false;
        for (var anIt = new Iterator(aResult[1], false, false); anIt.More(); anIt.Next(), aNbVertices++)
        {
            hasV1 |= anIt.Value().IsSame(aV1);
        }
        Assert.Equal(3, aNbVertices);
        Assert.True(hasV1);
        Assert.False(sae.FirstVertex(aResult[1]).IsSame(aV1));
    }

    [Fact]
    public void FixSame_EdgesOfAWire_ThrowsFrozenShape_AsOcct()
    {
        // OCCT: FixSame compares the last vertex of an edge (REVERSED) with the first of the next (FORWARD) with ==,
        // which compares orientations, so a shared vertex is added again; and it adds it without making the edge Free
        // (Fix does), so on the edges of a wire TopoDS_Builder::Add throws TopoDS_FrozenShape.
        Wire aSquare = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0),
                                                             new Point3D(1, 1, 0), new Point3D(0, 1, 0), true).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aSquare);
        var sfwv = new WireVertexFixer();
        sfwv.Init(aSquare, 1.0e-3);
        Assert.Equal(1, sfwv.Analyzer().Status(1));

        Assert.Throws<FrozenShapeException>(() => sfwv.FixSame());
        aSnap.AssertUnchanged(aSquare);
    }

    [Fact]
    public void FixAfterWire_CopiesAgain()
    {
        Edge[] anEdges = GappedSquare();
        var sfwv = new WireVertexFixer();
        sfwv.Init(WireData(anEdges), 1.0e-3);
        Assert.Equal(4, sfwv.Fix());
        Wire aFirst = sfwv.Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        Assert.Equal(0, sfwv.Fix()); // every status is now "same"
        sfwv.Analyzer().SetClose(1);
        Assert.Equal(1, sfwv.Fix());
        Wire aSecond = sfwv.Wire();
        aSnap.AssertUnchanged(aFirst);
        Assert.False(ReferenceEquals(aSecond.TShape(), aFirst.TShape()));
    }

    [Fact]
    public void NotAnalysed_DoesNothing()
    {
        var sfwv = new WireVertexFixer();
        Assert.Equal(0, sfwv.Fix());
        Assert.Equal(0, sfwv.FixSame());
        Assert.Null(sfwv.WireData());
    }

    [Fact]
    public void UnfrozenEdges_AreFixedInPlace()
    {
        var aPoly = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), true);
        Wire aWire = aPoly.InternalWire();
        var sbwd = new ExtendWireData(aWire);
        Edge aFirst = sbwd.Edge(1);
        var sfwv = new WireVertexFixer();
        sfwv.Init(sbwd, 1.0e-3);
        Assert.Equal(3, sfwv.Fix());
        Wire aResult = sfwv.InternalWire();
        Assert.False(aResult.TShape()!.IsFrozen);
        Assert.True(ReferenceEquals(sbwd.Edge(1).TShape(), aFirst.TShape())); // in place: the same edges
        Assert.Equal(ShapeType.Edge, sbwd.Edge(1).ShapeType());
    }
}
