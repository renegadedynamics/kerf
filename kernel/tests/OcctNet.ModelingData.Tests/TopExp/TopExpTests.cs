// Port of: src/ModelingData/TKBRep/GTests/TopExp_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's TopExp GTests, on BRepPrimAPI_MakeBox's (frozen) box: 6 faces, 12 edges and 8 vertices.
/// </summary>
public class TopExpTests
{
    private static TopoDS_Shape MakeBox() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    [Fact]
    public void MapShapes_Faces()
    {
        TopoDS_Shape aBox = MakeBox();
        var aFaceMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, TopAbs_FACE, aFaceMap);
        Assert.Equal(6, aFaceMap.Extent());
    }

    [Fact]
    public void MapShapes_Edges()
    {
        TopoDS_Shape aBox = MakeBox();
        var anEdgeMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, TopAbs_EDGE, anEdgeMap);
        Assert.Equal(12, anEdgeMap.Extent());
    }

    [Fact]
    public void MapShapes_Vertices()
    {
        TopoDS_Shape aBox = MakeBox();
        var aVertexMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, TopAbs_VERTEX, aVertexMap);
        Assert.Equal(8, aVertexMap.Extent());
    }

    [Fact]
    public void FirstVertex_LastVertex()
    {
        TopoDS_Shape aBox = MakeBox();
        var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        TopoDS_Vertex aFirstVertex = TopExp.FirstVertex(anEdge);
        TopoDS_Vertex aLastVertex = TopExp.LastVertex(anEdge);
        Assert.False(aFirstVertex.IsNull());
        Assert.False(aLastVertex.IsNull());
        Assert.False(aFirstVertex.IsSame(aLastVertex));
    }

    [Fact]
    public void Vertices()
    {
        TopoDS_Shape aBox = MakeBox();
        var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.False(aV1.IsNull());
        Assert.False(aV2.IsNull());
    }

    [Fact]
    public void CommonVertex()
    {
        TopoDS_Shape aBox = MakeBox();
        var anEdgeMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aBox, TopAbs_EDGE, anEdgeMap);
        Assert.True(anEdgeMap.Extent() >= 2);

        // Find two adjacent edges (sharing a common vertex)
        bool isFound = false;
        for (int i = 1; i <= anEdgeMap.Extent() && !isFound; ++i)
        {
            TopoDS_Edge anEdge1 = TopoDS.Edge(anEdgeMap.FindKey(i));
            for (int j = i + 1; j <= anEdgeMap.Extent() && !isFound; ++j)
            {
                TopoDS_Edge anEdge2 = TopoDS.Edge(anEdgeMap.FindKey(j));
                TopoDS_Vertex aCommonVertex = new();
                if (TopExp.CommonVertex(anEdge1, anEdge2, ref aCommonVertex))
                {
                    Assert.False(aCommonVertex.IsNull());
                    isFound = true;
                }
            }
        }

        Assert.True(isFound);
    }

    [Fact]
    public void Explorer_NestedFaceEdge_Terminates()
    {
        TopoDS_Shape aBox = MakeBox();

        int aTotalEdgeRefs = 0;
        int aFaceCount = 0;
        for (var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
        {
            ++aFaceCount;
            int anEdgeCount = 0;
            for (var anEdgeExp = new TopExp_Explorer(TopoDS.Face(aFaceExp.Current()), TopAbs_EDGE); anEdgeExp.More();
                 anEdgeExp.Next())
            {
                Assert.False(anEdgeExp.Current().IsNull());
                ++anEdgeCount;
                // Guard against infinite loop
                Assert.True(anEdgeCount < 100);
            }

            Assert.Equal(4, anEdgeCount);
            aTotalEdgeRefs += anEdgeCount;
        }

        Assert.Equal(6, aFaceCount);
        Assert.Equal(24, aTotalEdgeRefs);
    }

    [Fact]
    public void Explorer_ReInit_Terminates()
    {
        TopoDS_Shape aBox = MakeBox();

        var anExp = new TopExp_Explorer();
        for (var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
            anExp.Init(aFace, TopAbs_EDGE);
            int anEdgeCount = 0;
            for (; anExp.More(); anExp.Next())
            {
                ++anEdgeCount;
                Assert.True(anEdgeCount < 100);
            }

            Assert.Equal(4, anEdgeCount);
        }
    }

    [Fact]
    public void MapShapesAndAncestors()
    {
        TopoDS_Shape aBox = MakeBox();

        var anEdgeFaceMap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, anEdgeFaceMap);

        Assert.Equal(12, anEdgeFaceMap.Extent());

        // Each edge of a box is shared by exactly 2 faces
        for (int i = 1; i <= anEdgeFaceMap.Extent(); ++i)
        {
            NCollection_List<TopoDS_Shape> aFaceList = anEdgeFaceMap.FindFromIndex(i);
            Assert.Equal(2, aFaceList.Extent());
        }
    }

    // Additions

    [Fact]
    public void Explorer_with_a_type_to_avoid_and_depth()
    {
        TopoDS_Shape aBox = MakeBox();
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox);
        TopoDS_Vertex aFree = TestShapes.Frozen(TestShapes.Vertex(new gp_Pnt(50, 0, 0)));
        aBuilder.Add(aCompound, aFree);

        int aCount = 0;
        for (var anExp = new TopExp_Explorer(aCompound, TopAbs_VERTEX, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(anExp.Current().IsSame(aFree));
            Assert.Equal(1, anExp.Depth());
            aCount++;
        }

        Assert.Equal(1, aCount);

        var anEdgeExp = new TopExp_Explorer(aCompound, TopAbs_EDGE);
        Assert.Equal(5, anEdgeExp.Depth()); // iterators on the compound, solid, shell, face and wire
        Assert.True(anEdgeExp.ExploredShape().IsEqual(aCompound));

        // a shape simpler than the type to find, and SHAPE, find nothing
        Assert.False(new TopExp_Explorer(aFree, TopAbs_EDGE).More());
        Assert.False(new TopExp_Explorer(aCompound, TopAbs_SHAPE).More());
        // the shape itself when it has the type to find
        var aSelf = new TopExp_Explorer(aFree, TopAbs_VERTEX);
        Assert.True(aSelf.More() && aSelf.Current().IsEqual(aFree) && aSelf.Depth() == 0);
    }

    [Fact]
    public void Vertices_of_wires()
    {
        TopoDS_Shape aBox = MakeBox();
        var aWireExp = new TopExp_Explorer(aBox, TopAbs_WIRE);
        TopoDS_Wire aClosed = TopoDS.Wire(aWireExp.Current());
        TopExp.Vertices(aClosed, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.True(aV1.IsSame(aV2));
        Assert.Equal(TopAbs_FORWARD, aV1.Orientation());
        Assert.Equal(TopAbs_REVERSED, aV2.Orientation());

        // an open wire of two segments
        TopoDS_Vertex aA = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        TopoDS_Vertex aB = TestShapes.Vertex(new gp_Pnt(1, 0, 0));
        TopoDS_Vertex aC = TestShapes.Vertex(new gp_Pnt(1, 1, 0));
        var aBuilder = new BRep_Builder();
        TopoDS_Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aWire, TestShapes.Edge(aA, aB));
        aBuilder.Add(aWire, TestShapes.Edge(aC, aB).Reversed());
        TopExp.Vertices(aWire, out TopoDS_Vertex aFirst, out TopoDS_Vertex aLast);
        Assert.True(aFirst.IsSame(aA));
        Assert.True(aLast.IsSame(aC));
    }
}
