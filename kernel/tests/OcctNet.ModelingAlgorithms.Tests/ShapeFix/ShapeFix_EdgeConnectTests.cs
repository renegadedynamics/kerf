// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_EdgeConnect (unit 6H2): the in-place Build on unfrozen edges, and its functional form Built
/// on frozen ones.
/// </summary>
public class ShapeFix_EdgeConnectTests
{
    private static Compound Compound(params Shape[] theShapes)
    {
        var aC = new Compound();
        var aB = new BrepBuilder();
        aB.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        return aC;
    }

    private static List<Edge> Edges(in Shape theShape)
    {
        var aList = new List<Edge>();
        for (var anExp = new Explorer(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aList.Add(ShapeCasts.Edge(anExp.Current()));
        }
        return aList;
    }

    [Fact]
    public void Built_TwoFrozenEdgesWithAGap_ShareAVertexAtTheMiddle()
    {
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        Compound anInput = Compound(e1, e2);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anInput);
        var aConnect = new EdgeConnectFixer();
        aConnect.Add(e1, e2);

        Shape aResult = aConnect.Built(anInput);

        aSnap.AssertUnchanged(anInput);
        Assert.True(aResult.TShape()!.IsFrozen);
        List<Edge> anEdges = Edges(aResult);
        Assert.Equal(2, anEdges.Count);
        Vertex aShared = Exploration.LastVertex(anEdges[0]);
        Assert.True(aShared.IsSame(Exploration.FirstVertex(anEdges[1])));
        Assert.Equal(1.0001, Tool.Pnt(aShared).X(), 1e-12);
        Assert.Equal(1.0e-4 * 1.0001, Tool.Tolerance(aShared), 1e-12);
        // e1's far vertex is the caller's
        Assert.True(Exploration.FirstVertex(anEdges[0]).IsSame(Exploration.FirstVertex(e1)));
        // e2's vertex list was edited (one removed, one added): KeepUnchanged pairs children by position, so its far
        // vertex stays an (equal) copy
        Assert.Equal(2.0, Tool.Pnt(Exploration.LastVertex(anEdges[1])).X());
    }

    [Fact]
    public void Build_InPlace_OnUnfrozenEdges_AndThrowsOnFrozenOnes()
    {
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).InternalEdge();
        Edge e2 = new MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).InternalEdge();
        var aConnect = new EdgeConnectFixer();
        aConnect.Add(e1, e2);
        aConnect.Build();
        Assert.True(Exploration.LastVertex(e1).IsSame(Exploration.FirstVertex(e2)));

        Edge f1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge f2 = new MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        aConnect.Add(f1, f2);
        Assert.ThrowsAny<Exception>(() => aConnect.Build());
    }

    [Fact]
    public void Built_ClosedWireOfSeparateEdges_ConnectsEveryJoint()
    {
        // a triangle of three edges with their own vertices, in a closed wire
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1, 0, 0), new Point3D(0, 1, 0)).Edge();
        Edge e3 = new MakeEdge(new Point3D(0, 1, 0), new Point3D(0, 0, 0)).Edge();
        var aWire = new Wire();
        var aB = new BrepBuilder();
        aB.MakeWire(ref aWire);
        aB.Add(aWire, e1);
        aB.Add(aWire, e2);
        aB.Add(aWire, e3);
        aWire.Closed(true);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);

        var aConnect = new EdgeConnectFixer();
        aConnect.Add(aWire);
        Shape aResult = aConnect.Built(aWire);

        aSnap.AssertUnchanged(aWire);
        var aVertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, ShapeType.Vertex, aVertices);
        Assert.Equal(3, aVertices.Extent());
        Assert.True(Tool.IsClosed(aResult));
    }

    [Fact]
    public void Clear_ForgetsTheConnections()
    {
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        Compound anInput = Compound(e1, e2);
        var aConnect = new EdgeConnectFixer();
        aConnect.Add(e1, e2);
        aConnect.Clear();
        Shape aResult = aConnect.Built(anInput);
        Assert.True(ReferenceEquals(aResult.TShape(), anInput.TShape())); // nothing to do: the input itself
    }
}
