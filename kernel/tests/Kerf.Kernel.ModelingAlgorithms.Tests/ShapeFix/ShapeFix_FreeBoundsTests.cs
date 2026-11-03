// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_FreeBounds (unit 6H3): free bounds of shells and of faces, the connection of open wires
/// (which writes into the shape: a frozen input is copied), the split options and the sewing tolerance.
/// </summary>
public class ShapeFix_FreeBoundsTests
{
    private static int Count(in Shape theShape, ShapeType theType)
    {
        if (theShape.IsNull)
        {
            return 0;
        }
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static int NbChildren(in Shape theShape) => theShape.IsNull ? 0 : theShape.ChildCount;

    private static Vertex Vertex(double x, double y, double z = 0)
    {
        var aV = new Vertex();
        var aB = new BrepBuilder();
        aB.MakeVertex(ref aV, new Point3D(x, y, z), 1e-7);
        return aV;
    }

    private static Edge Edge(in Vertex theV1, in Vertex theV2, bool theUnfrozen) =>
        theUnfrozen ? new MakeEdge(theV1, theV2).InternalEdge() : new MakeEdge(theV1, theV2).Edge();

    // A planar square face on z = 0 whose outer wire is built edge by edge: the last edge ends at a vertex of its own,
    // theGap away from the first vertex (so the wire is open, its ends not shared).
    private static Face SquareWithOpenCorner(double theGap, bool theUnfrozen = false)
    {
        Vertex v1 = Vertex(0, 0), v2 = Vertex(1, 0), v3 = Vertex(1, 1), v4 = Vertex(0, 1), v5 = Vertex(theGap, 0);
        var aB = new BrepBuilder();
        var aW = new Wire();
        aB.MakeWire(ref aW);
        aB.Add(aW, Edge(v1, v2, theUnfrozen));
        aB.Add(aW, Edge(v2, v3, theUnfrozen));
        aB.Add(aW, Edge(v3, v4, theUnfrozen));
        aB.Add(aW, Edge(v4, v5, theUnfrozen));
        var aF = new Face();
        aB.MakeFace(ref aF, new PlaneSurface(new Plane3D()), 1e-7);
        aB.Add(aF, aW);
        return aF;
    }

    private static Compound Compound(params Shape[] theShapes)
    {
        var aC = new Compound();
        var aB = new BrepBuilder();
        aB.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void ShellForm_BoxWithoutAFace_OneClosedWire_InputItself()
    {
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        Shell aShell = ShapeCasts.Shell(new Explorer(aBox, ShapeType.Shell).Current());
        // the box's shell without its first face
        var aB = new BrepBuilder();
        var anOpen = new Shell();
        aB.MakeShell(ref anOpen);
        int i = 0;
        for (var anIt = new Iterator(aShell); anIt.More(); anIt.Next(), i++)
        {
            if (i > 0)
            {
                aB.Add(anOpen, anIt.Value());
            }
        }
        Compound aShape = Compound(anOpen);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        var aFB = new FreeBoundsFixer(aShape, 1e-3, false, false);

        aSnap.AssertUnchanged(aShape);
        Assert.Equal(1, NbChildren(aFB.GetClosedWires()));
        Assert.Equal(0, NbChildren(aFB.GetOpenWires()));
        Assert.Equal(4, Count(aFB.GetClosedWires(), global::Kerf.Topology.ShapeType.Edge));
        Assert.True(aFB.GetClosedWires().TShape()!.IsFrozen);
        Assert.True(aFB.GetShape().IsSame(aShape)); // nothing to connect: the input itself
        // the free edges are the caller's own
        var anEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShape, global::Kerf.Topology.ShapeType.Edge, anEdges);
        for (var anExp = new Explorer(aFB.GetClosedWires(), global::Kerf.Topology.ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(anEdges.Contains(anExp.Current()));
        }
    }

    [Fact]
    public void ShellForm_OpenWireConnected_CopiesTheFrozenInput()
    {
        Face aFace = SquareWithOpenCorner(1e-4);
        Compound aShape = Compound(aFace);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        // the gap is beyond the closing tolerance: one open wire, the input itself
        var aFB1 = new FreeBoundsFixer(aShape, 1e-5, false, false);
        Assert.Equal(0, NbChildren(aFB1.GetClosedWires()));
        Assert.Equal(1, NbChildren(aFB1.GetOpenWires()));
        Assert.True(aFB1.GetShape().IsSame(aShape));

        // within it: the open wire is closed, and the shape's edges at the corner get the new vertex
        var aFB2 = new FreeBoundsFixer(aShape, 1e-3, false, false);
        aSnap.AssertUnchanged(aShape);
        Assert.Equal(1, NbChildren(aFB2.GetClosedWires()));
        Assert.Equal(0, NbChildren(aFB2.GetOpenWires()));
        Shape aResult = aFB2.GetShape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.False(aResult.IsSame(aShape));
        Assert.Equal(4, Count(aResult, global::Kerf.Topology.ShapeType.Vertex)); // the two corner vertices merged
        // the two edges not touching the corner are the caller's own
        var anEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShape, global::Kerf.Topology.ShapeType.Edge, anEdges);
        int aKept = 0;
        for (var anExp = new Explorer(aResult, global::Kerf.Topology.ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aKept += anEdges.Contains(anExp.Current()) ? 1 : 0;
        }
        Assert.Equal(2, aKept);
    }

    [Fact]
    public void SewingForm_TwoSquaresSewn_OneClosedWire_SplitOptions()
    {
        // two unit squares side by side with their own edges, 1e-7 apart
        Face aF1 = new MakeFace(new Plane3D(), 0, 1, 0, 1).Face();
        Face aF2 = new MakeFace(new Plane3D(), 1 + 1e-7, 2, 0, 1).Face();
        Compound aShape = Compound(aF1, aF2);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        // without sewing tolerance enough to merge: two closed wires
        var aFB0 = new FreeBoundsFixer(aShape, 1e-9, 1e-9, false, false);
        Assert.Equal(2, NbChildren(aFB0.GetClosedWires()));

        foreach (bool aSplitClosed in new[] { false, true })
        {
            var aFB = new FreeBoundsFixer(aShape, 1e-6, 1e-6, aSplitClosed, true);
            aSnap.AssertUnchanged(aShape);
            Assert.Equal(1, NbChildren(aFB.GetClosedWires()));
            Assert.Equal(6, Count(aFB.GetClosedWires(), global::Kerf.Topology.ShapeType.Edge));
            Assert.Equal(0, NbChildren(aFB.GetOpenWires()));
            Assert.True(aFB.GetShape().IsSame(aShape));
        }
    }

    [Fact]
    public void EmptyConstructor_GivesNullResults()
    {
        var aFB = new FreeBoundsFixer();
        Assert.True(aFB.GetShape().IsNull);
        Assert.True(aFB.GetClosedWires().IsNull);
        Assert.True(aFB.GetOpenWires().IsNull);
    }

    [Fact]
    public void UnfrozenInput_IsConnectedInPlace()
    {
        Face aFace = SquareWithOpenCorner(1e-4, true);
        Assert.False(BrepTools.HoldsFrozen(aFace));
        var aB = new BrepBuilder();
        var aC = new Compound();
        aB.MakeCompound(ref aC);
        aB.Add(aC, aFace);

        var aFB = new FreeBoundsFixer(aC, 1e-3, false, false);
        Assert.True(aFB.GetShape().IsSame(aC)); // the same compound, its edges changed in place
        Assert.Equal(4, Count(aC, global::Kerf.Topology.ShapeType.Vertex));
    }
}
