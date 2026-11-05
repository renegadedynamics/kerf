// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>BRepTools_WireExplorer: connection order, start of open wires, vertices and their orientations.</summary>
public class BRepTools_WireExplorerTests
{
    private static Wire MakeWire(params Edge[] theEdges) => BRepAdaptor_CompCurveTests.MakeWire(theEdges);

    /// <summary>Checks that each edge starts where the previous one ends, and returns the number of edges.</summary>
    private static int CheckChain(WireExplorer theExp)
    {
        int aCount = 0;
        Vertex aPrevLast = new Vertex();
        for (; theExp.More(); theExp.Next(), aCount++)
        {
            Edge anE = theExp.Current();
            Vertex aFirst = Exploration.FirstVertex(anE, true);
            Assert.True(theExp.CurrentVertex().IsSame(aFirst), $"edge {aCount}: current vertex");
            if (!aPrevLast.IsNull)
            {
                Assert.True(aFirst.IsSame(aPrevLast), $"edge {aCount}: connected");
            }
            // the current vertex is the FORWARD vertex of a FORWARD edge, the REVERSED one of a REVERSED edge
            Assert.Equal(anE.Orientation == Reversed ? Reversed : Forward, theExp.Orientation());
            aPrevLast = Exploration.LastVertex(anE, true);
        }
        return aCount;
    }

    [Fact]
    public void ClosedWires_InConnectionOrder()
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        for (var aFaces = new Explorer(aBox, ShapeType.Face); aFaces.More(); aFaces.Next())
        {
            Face aF = ShapeCasts.Face(aFaces.Current());
            Wire aW = ShapeCasts.Wire(new Explorer(aF, ShapeType.Wire).Current());
            Assert.Equal(4, CheckChain(new WireExplorer(aW)));
            Assert.Equal(4, CheckChain(new WireExplorer(aW, aF)));
        }
    }

    [Fact]
    public void OpenWire_StartsAtItsFreeVertex()
    {
        Vertex[] aV = [.. Enumerable.Range(0, 4).Select(i => TestShapes.Vertex(new Point3D(i, i % 2, 0)))];
        Edge e01 = TestShapes.Edge(aV[0], aV[1]);
        Edge e12 = TestShapes.Edge(aV[1], aV[2]);
        Edge e32 = TestShapes.Edge(aV[3], aV[2]);
        // shuffled, the last edge reversed
        Wire aW = MakeWire(ShapeCasts.Edge(e32.Reversed()), e01, e12);
        var anExp = new WireExplorer(aW);
        Assert.True(anExp.Current().IsSame(e01));
        Assert.True(anExp.CurrentVertex().IsSame(aV[0]));
        Assert.Equal(3, CheckChain(anExp));
        Assert.False(anExp.More());
        Assert.True(anExp.Current().IsNull);

        anExp.Init(aW);
        anExp.Next();
        anExp.Next();
        Assert.True(anExp.Current().IsSame(e32));
        Assert.Equal(Reversed, anExp.Current().Orientation);
        Assert.True(anExp.CurrentVertex().IsSame(aV[2]));

        anExp.Clear();
        Assert.False(anExp.More());
    }

    [Fact]
    public void EmptyAndNullWires()
    {
        var anExp = new WireExplorer();
        Assert.False(anExp.More());
        anExp.Init(new Wire());
        Assert.False(anExp.More());
        var aB = new BrepBuilder();
        Wire aW = new Wire();
        aB.MakeWire(ref aW);
        anExp.Init(aW);
        Assert.False(anExp.More());
    }

    [Fact]
    public void InternalEdgesAreSkipped()
    {
        Vertex[] aV = [.. Enumerable.Range(0, 3).Select(i => TestShapes.Vertex(new Point3D(i, 0, 0)))];
        Edge e01 = TestShapes.Edge(aV[0], aV[1]);
        Edge e12 = TestShapes.Edge(aV[1], aV[2]);
        Edge anInternal = TestShapes.Edge(TestShapes.Vertex(new Point3D(0.5, 1, 0)), aV[1]);
        Wire aW = MakeWire(e01, ShapeCasts.Edge(anInternal.Oriented(Internal)), e12);
        Assert.Equal(2, CheckChain(new WireExplorer(aW)));
    }

    [Fact]
    public void SeamWire_WithFace()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(1.0, 2.0));
        Wire aW = ShapeCasts.Wire(new Explorer(aFace, ShapeType.Wire).Current());
        var anExp = new WireExplorer(aW, aFace);
        var anOrder = new List<Edge>();
        for (; anExp.More(); anExp.Next())
        {
            anOrder.Add(anExp.Current());
        }
        Assert.Equal(4, anOrder.Count);
        // bottom, seam FORWARD, top REVERSED, seam REVERSED: each seam once in each orientation
        Assert.True(anOrder[1].IsSame(anOrder[3]));
        Assert.NotEqual(anOrder[1].Orientation, anOrder[3].Orientation);
    }
}
