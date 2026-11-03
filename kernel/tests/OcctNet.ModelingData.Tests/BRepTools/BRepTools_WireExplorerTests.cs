// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>BRepTools_WireExplorer: connection order, start of open wires, vertices and their orientations.</summary>
public class BRepTools_WireExplorerTests
{
    private static TopoDS_Wire MakeWire(params TopoDS_Edge[] theEdges) => BRepAdaptor_CompCurveTests.MakeWire(theEdges);

    /// <summary>Checks that each edge starts where the previous one ends, and returns the number of edges.</summary>
    private static int CheckChain(BRepTools_WireExplorer theExp)
    {
        int aCount = 0;
        TopoDS_Vertex aPrevLast = new TopoDS_Vertex();
        for (; theExp.More(); theExp.Next(), aCount++)
        {
            TopoDS_Edge anE = theExp.Current();
            TopoDS_Vertex aFirst = TopExp.FirstVertex(anE, true);
            Assert.True(theExp.CurrentVertex().IsSame(aFirst), $"edge {aCount}: current vertex");
            if (!aPrevLast.IsNull())
            {
                Assert.True(aFirst.IsSame(aPrevLast), $"edge {aCount}: connected");
            }
            // the current vertex is the FORWARD vertex of a FORWARD edge, the REVERSED one of a REVERSED edge
            Assert.Equal(anE.Orientation() == TopAbs_REVERSED ? TopAbs_REVERSED : TopAbs_FORWARD, theExp.Orientation());
            aPrevLast = TopExp.LastVertex(anE, true);
        }
        return aCount;
    }

    [Fact]
    public void ClosedWires_InConnectionOrder()
    {
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        for (var aFaces = new TopExp_Explorer(aBox, TopAbs_FACE); aFaces.More(); aFaces.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aFaces.Current());
            TopoDS_Wire aW = TopoDS.Wire(new TopExp_Explorer(aF, TopAbs_WIRE).Current());
            Assert.Equal(4, CheckChain(new BRepTools_WireExplorer(aW)));
            Assert.Equal(4, CheckChain(new BRepTools_WireExplorer(aW, aF)));
        }
    }

    [Fact]
    public void OpenWire_StartsAtItsFreeVertex()
    {
        TopoDS_Vertex[] aV = [.. Enumerable.Range(0, 4).Select(i => TestShapes.Vertex(new Point3D(i, i % 2, 0)))];
        TopoDS_Edge e01 = TestShapes.Edge(aV[0], aV[1]);
        TopoDS_Edge e12 = TestShapes.Edge(aV[1], aV[2]);
        TopoDS_Edge e32 = TestShapes.Edge(aV[3], aV[2]);
        // shuffled, the last edge reversed
        TopoDS_Wire aW = MakeWire(TopoDS.Edge(e32.Reversed()), e01, e12);
        var anExp = new BRepTools_WireExplorer(aW);
        Assert.True(anExp.Current().IsSame(e01));
        Assert.True(anExp.CurrentVertex().IsSame(aV[0]));
        Assert.Equal(3, CheckChain(anExp));
        Assert.False(anExp.More());
        Assert.True(anExp.Current().IsNull());

        anExp.Init(aW);
        anExp.Next();
        anExp.Next();
        Assert.True(anExp.Current().IsSame(e32));
        Assert.Equal(TopAbs_REVERSED, anExp.Current().Orientation());
        Assert.True(anExp.CurrentVertex().IsSame(aV[2]));

        anExp.Clear();
        Assert.False(anExp.More());
    }

    [Fact]
    public void EmptyAndNullWires()
    {
        var anExp = new BRepTools_WireExplorer();
        Assert.False(anExp.More());
        anExp.Init(new TopoDS_Wire());
        Assert.False(anExp.More());
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new TopoDS_Wire();
        aB.MakeWire(ref aW);
        anExp.Init(aW);
        Assert.False(anExp.More());
    }

    [Fact]
    public void InternalEdgesAreSkipped()
    {
        TopoDS_Vertex[] aV = [.. Enumerable.Range(0, 3).Select(i => TestShapes.Vertex(new Point3D(i, 0, 0)))];
        TopoDS_Edge e01 = TestShapes.Edge(aV[0], aV[1]);
        TopoDS_Edge e12 = TestShapes.Edge(aV[1], aV[2]);
        TopoDS_Edge anInternal = TestShapes.Edge(TestShapes.Vertex(new Point3D(0.5, 1, 0)), aV[1]);
        TopoDS_Wire aW = MakeWire(e01, TopoDS.Edge(anInternal.Oriented(TopAbs_INTERNAL)), e12);
        Assert.Equal(2, CheckChain(new BRepTools_WireExplorer(aW)));
    }

    [Fact]
    public void SeamWire_WithFace()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(1.0, 2.0));
        TopoDS_Wire aW = TopoDS.Wire(new TopExp_Explorer(aFace, TopAbs_WIRE).Current());
        var anExp = new BRepTools_WireExplorer(aW, aFace);
        var anOrder = new List<TopoDS_Edge>();
        for (; anExp.More(); anExp.Next())
        {
            anOrder.Add(anExp.Current());
        }
        Assert.Equal(4, anOrder.Count);
        // bottom, seam FORWARD, top REVERSED, seam REVERSED: each seam once in each orientation
        Assert.True(anOrder[1].IsSame(anOrder[3]));
        Assert.NotEqual(anOrder[1].Orientation(), anOrder[3].Orientation());
    }
}
