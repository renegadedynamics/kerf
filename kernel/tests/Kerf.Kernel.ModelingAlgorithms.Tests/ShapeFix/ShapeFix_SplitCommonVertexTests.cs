// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_SplitCommonVertex (unit 6H2) on a frozen face whose two wires share a vertex.</summary>
public class ShapeFix_SplitCommonVertexTests
{
    private static Vertex V(double x, double y) => new MakeVertex(new Point3D(x, y, 0)).Vertex();

    private static IndexedSet<Shape> Map(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap;
    }

    // A square face and, inside it, a triangle touching its corner (0,0): the two wires share that vertex.
    private static Face FaceWithCommonVertex(out Vertex theCommon)
    {
        theCommon = V(0, 0);
        Vertex v1 = V(4, 0), v2 = V(4, 4), v3 = V(0, 4), w1 = V(1, 2), w2 = V(2, 1);
        Wire anOuter = new MakePolygon(theCommon, v1, v2, v3, true).Wire();
        Wire anInner = new MakePolygon(theCommon, w1, w2, true).Wire();
        var aMF = new MakeFace(new MakeFace(anOuter, true).Face());
        aMF.Add(anInner);
        return aMF.Face();
    }

    [Fact]
    public void Perform_SplitsTheCommonVertex_InputUnchanged()
    {
        Face aFace = FaceWithCommonVertex(out Vertex aCommon);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aFix = new SplitCommonVertexFixer();
        aFix.Init(aFace);
        aFix.Perform();
        Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aFace);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Face, aResult.Type);
        var aWires = new List<Shape>();
        for (var anIt = new Iterator(aResult); anIt.More(); anIt.Next())
        {
            aWires.Add(anIt.Value());
        }
        Assert.Equal(2, aWires.Count);
        IndexedSet<Shape> aV1 = Map(aWires[0], ShapeType.Vertex);
        IndexedSet<Shape> aV2 = Map(aWires[1], ShapeType.Vertex);
        for (int i = 0; i < aV1.Count; i++)
        {
            Assert.False(aV2.Contains(aV1[i]));
        }

        // the first wire is the caller's own; the second's edges at the corner were replaced through the context
        Assert.True(Map(aFace, ShapeType.Wire).Contains(aWires[0]));
        Assert.False(Map(aFace, ShapeType.Wire).Contains(aWires[1]));
        Assert.True(Map(aResult, ShapeType.Vertex).Contains(aCommon));
        Assert.Equal(7, Map(aResult, ShapeType.Vertex).Count);
        int aNbRecorded = 0;
        IndexedSet<Shape> anEdges = Map(aFace, ShapeType.Edge);
        for (int i = 0; i < anEdges.Count; i++)
        {
            if (aFix.Context()!.IsRecorded(anEdges[i]))
            {
                aNbRecorded++;
            }
        }
        Assert.Equal(2, aNbRecorded);
    }

    [Fact]
    public void Perform_NoCommonVertex_GivesTheInput()
    {
        Shape aBox = new MakeBox(1, 2, 3).Shape();
        var aFix = new SplitCommonVertexFixer();
        aFix.Init(aBox);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), aBox.TShape()));
    }

    [Fact]
    public void Perform_OnAnEdge_DoesNothing()
    {
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        var aFix = new SplitCommonVertexFixer();
        aFix.Init(anEdge);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), anEdge.TShape()));
    }
}
