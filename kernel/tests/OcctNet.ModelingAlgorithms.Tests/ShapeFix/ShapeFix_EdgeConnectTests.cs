// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_EdgeConnect (unit 6H2): the in-place Build on unfrozen edges, and its functional form Built
/// on frozen ones.
/// </summary>
public class ShapeFix_EdgeConnectTests
{
    private static TopoDS_Compound Compound(params TopoDS_Shape[] theShapes)
    {
        var aC = new TopoDS_Compound();
        var aB = new BRep_Builder();
        aB.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        return aC;
    }

    private static List<TopoDS_Edge> Edges(in TopoDS_Shape theShape)
    {
        var aList = new List<TopoDS_Edge>();
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aList.Add(TopoDS.Edge(anExp.Current()));
        }
        return aList;
    }

    [Fact]
    public void Built_TwoFrozenEdgesWithAGap_ShareAVertexAtTheMiddle()
    {
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        TopoDS_Compound anInput = Compound(e1, e2);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anInput);
        var aConnect = new ShapeFix_EdgeConnect();
        aConnect.Add(e1, e2);

        TopoDS_Shape aResult = aConnect.Built(anInput);

        aSnap.AssertUnchanged(anInput);
        Assert.True(aResult.TShape()!.IsFrozen);
        List<TopoDS_Edge> anEdges = Edges(aResult);
        Assert.Equal(2, anEdges.Count);
        TopoDS_Vertex aShared = TopExp.LastVertex(anEdges[0]);
        Assert.True(aShared.IsSame(TopExp.FirstVertex(anEdges[1])));
        Assert.Equal(1.0001, BRep_Tool.Pnt(aShared).X(), 1e-12);
        Assert.Equal(1.0e-4 * 1.0001, BRep_Tool.Tolerance(aShared), 1e-12);
        // e1's far vertex is the caller's
        Assert.True(TopExp.FirstVertex(anEdges[0]).IsSame(TopExp.FirstVertex(e1)));
        // e2's vertex list was edited (one removed, one added): KeepUnchanged pairs children by position, so its far
        // vertex stays an (equal) copy
        Assert.Equal(2.0, BRep_Tool.Pnt(TopExp.LastVertex(anEdges[1])).X());
    }

    [Fact]
    public void Build_InPlace_OnUnfrozenEdges_AndThrowsOnFrozenOnes()
    {
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).InternalEdge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).InternalEdge();
        var aConnect = new ShapeFix_EdgeConnect();
        aConnect.Add(e1, e2);
        aConnect.Build();
        Assert.True(TopExp.LastVertex(e1).IsSame(TopExp.FirstVertex(e2)));

        TopoDS_Edge f1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        TopoDS_Edge f2 = new BRepBuilderAPI_MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        aConnect.Add(f1, f2);
        Assert.ThrowsAny<Exception>(() => aConnect.Build());
    }

    [Fact]
    public void Built_ClosedWireOfSeparateEdges_ConnectsEveryJoint()
    {
        // a triangle of three edges with their own vertices, in a closed wire
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new Point3D(1, 0, 0), new Point3D(0, 1, 0)).Edge();
        TopoDS_Edge e3 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 1, 0), new Point3D(0, 0, 0)).Edge();
        var aWire = new TopoDS_Wire();
        var aB = new BRep_Builder();
        aB.MakeWire(ref aWire);
        aB.Add(aWire, e1);
        aB.Add(aWire, e2);
        aB.Add(aWire, e3);
        aWire.Closed(true);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);

        var aConnect = new ShapeFix_EdgeConnect();
        aConnect.Add(aWire);
        TopoDS_Shape aResult = aConnect.Built(aWire);

        aSnap.AssertUnchanged(aWire);
        var aVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_VERTEX, aVertices);
        Assert.Equal(3, aVertices.Extent());
        Assert.True(BRep_Tool.IsClosed(aResult));
    }

    [Fact]
    public void Clear_ForgetsTheConnections()
    {
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new Point3D(1.0002, 0, 0), new Point3D(2, 0, 0)).Edge();
        TopoDS_Compound anInput = Compound(e1, e2);
        var aConnect = new ShapeFix_EdgeConnect();
        aConnect.Add(e1, e2);
        aConnect.Clear();
        TopoDS_Shape aResult = aConnect.Built(anInput);
        Assert.True(ReferenceEquals(aResult.TShape(), anInput.TShape())); // nothing to do: the input itself
    }
}
