// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Unit tests of ShapeFix_SplitCommonVertex (unit 6H2) on a frozen face whose two wires share a vertex.</summary>
public class ShapeFix_SplitCommonVertexTests
{
    private static TopoDS_Vertex V(double x, double y) => new BRepBuilderAPI_MakeVertex(new gp_Pnt(x, y, 0)).Vertex();

    private static NCollection_IndexedMap<TopoDS_Shape> Map(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap;
    }

    // A square face and, inside it, a triangle touching its corner (0,0): the two wires share that vertex.
    private static TopoDS_Face FaceWithCommonVertex(out TopoDS_Vertex theCommon)
    {
        theCommon = V(0, 0);
        TopoDS_Vertex v1 = V(4, 0), v2 = V(4, 4), v3 = V(0, 4), w1 = V(1, 2), w2 = V(2, 1);
        TopoDS_Wire anOuter = new BRepBuilderAPI_MakePolygon(theCommon, v1, v2, v3, true).Wire();
        TopoDS_Wire anInner = new BRepBuilderAPI_MakePolygon(theCommon, w1, w2, true).Wire();
        var aMF = new BRepBuilderAPI_MakeFace(new BRepBuilderAPI_MakeFace(anOuter, true).Face());
        aMF.Add(anInner);
        return aMF.Face();
    }

    [Fact]
    public void Perform_SplitsTheCommonVertex_InputUnchanged()
    {
        TopoDS_Face aFace = FaceWithCommonVertex(out TopoDS_Vertex aCommon);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aFix = new ShapeFix_SplitCommonVertex();
        aFix.Init(aFace);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();

        aSnap.AssertUnchanged(aFace);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_FACE, aResult.ShapeType());
        var aWires = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(aResult); anIt.More(); anIt.Next())
        {
            aWires.Add(anIt.Value());
        }
        Assert.Equal(2, aWires.Count);
        NCollection_IndexedMap<TopoDS_Shape> aV1 = Map(aWires[0], TopAbs_VERTEX);
        NCollection_IndexedMap<TopoDS_Shape> aV2 = Map(aWires[1], TopAbs_VERTEX);
        for (int i = 1; i <= aV1.Extent(); i++)
        {
            Assert.False(aV2.Contains(aV1[i]));
        }

        // the first wire is the caller's own; the second's edges at the corner were replaced through the context
        Assert.True(Map(aFace, TopAbs_WIRE).Contains(aWires[0]));
        Assert.False(Map(aFace, TopAbs_WIRE).Contains(aWires[1]));
        Assert.True(Map(aResult, TopAbs_VERTEX).Contains(aCommon));
        Assert.Equal(7, Map(aResult, TopAbs_VERTEX).Extent());
        int aNbRecorded = 0;
        NCollection_IndexedMap<TopoDS_Shape> anEdges = Map(aFace, TopAbs_EDGE);
        for (int i = 1; i <= anEdges.Extent(); i++)
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
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        var aFix = new ShapeFix_SplitCommonVertex();
        aFix.Init(aBox);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), aBox.TShape()));
    }

    [Fact]
    public void Perform_OnAnEdge_DoesNothing()
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        var aFix = new ShapeFix_SplitCommonVertex();
        aFix.Init(anEdge);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), anEdge.TShape()));
    }
}
