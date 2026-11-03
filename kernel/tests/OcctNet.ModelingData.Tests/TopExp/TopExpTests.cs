// Port of: src/ModelingData/TKBRep/GTests/TopExp_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's TopExp GTests, on BRepPrimAPI_MakeBox's (frozen) box: 6 faces, 12 edges and 8 vertices.
/// </summary>
public class TopExpTests
{
    private static Shape MakeBox() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    [Fact]
    public void MapShapes_Faces()
    {
        Shape aBox = MakeBox();
        var aFaceMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Face, aFaceMap);
        Assert.Equal(6, aFaceMap.Extent());
    }

    [Fact]
    public void MapShapes_Edges()
    {
        Shape aBox = MakeBox();
        var anEdgeMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Edge, anEdgeMap);
        Assert.Equal(12, anEdgeMap.Extent());
    }

    [Fact]
    public void MapShapes_Vertices()
    {
        Shape aBox = MakeBox();
        var aVertexMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Vertex, aVertexMap);
        Assert.Equal(8, aVertexMap.Extent());
    }

    [Fact]
    public void FirstVertex_LastVertex()
    {
        Shape aBox = MakeBox();
        var anExp = new Explorer(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        Vertex aFirstVertex = Exploration.FirstVertex(anEdge);
        Vertex aLastVertex = Exploration.LastVertex(anEdge);
        Assert.False(aFirstVertex.IsNull);
        Assert.False(aLastVertex.IsNull);
        Assert.False(aFirstVertex.IsSame(aLastVertex));
    }

    [Fact]
    public void Vertices()
    {
        Shape aBox = MakeBox();
        var anExp = new Explorer(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
        Assert.False(aV1.IsNull);
        Assert.False(aV2.IsNull);
    }

    [Fact]
    public void CommonVertex()
    {
        Shape aBox = MakeBox();
        var anEdgeMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Edge, anEdgeMap);
        Assert.True(anEdgeMap.Extent() >= 2);

        // Find two adjacent edges (sharing a common vertex)
        bool isFound = false;
        for (int i = 1; i <= anEdgeMap.Extent() && !isFound; ++i)
        {
            Edge anEdge1 = ShapeCasts.Edge(anEdgeMap.FindKey(i));
            for (int j = i + 1; j <= anEdgeMap.Extent() && !isFound; ++j)
            {
                Edge anEdge2 = ShapeCasts.Edge(anEdgeMap.FindKey(j));
                Vertex aCommonVertex = new();
                if (Exploration.CommonVertex(anEdge1, anEdge2, ref aCommonVertex))
                {
                    Assert.False(aCommonVertex.IsNull);
                    isFound = true;
                }
            }
        }

        Assert.True(isFound);
    }

    [Fact]
    public void Explorer_NestedFaceEdge_Terminates()
    {
        Shape aBox = MakeBox();

        int aTotalEdgeRefs = 0;
        int aFaceCount = 0;
        for (var aFaceExp = new Explorer(aBox, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            ++aFaceCount;
            int anEdgeCount = 0;
            for (var anEdgeExp = new Explorer(ShapeCasts.Face(aFaceExp.Current()), ShapeType.Edge); anEdgeExp.More();
                 anEdgeExp.Next())
            {
                Assert.False(anEdgeExp.Current().IsNull);
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
        Shape aBox = MakeBox();

        var anExp = new Explorer();
        for (var aFaceExp = new Explorer(aBox, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExp.Current());
            anExp.Init(aFace, ShapeType.Edge);
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
        Shape aBox = MakeBox();

        var anEdgeFaceMap = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, anEdgeFaceMap);

        Assert.Equal(12, anEdgeFaceMap.Extent());

        // Each edge of a box is shared by exactly 2 faces
        for (int i = 1; i <= anEdgeFaceMap.Extent(); ++i)
        {
            NCollection_List<Shape> aFaceList = anEdgeFaceMap.FindFromIndex(i);
            Assert.Equal(2, aFaceList.Extent());
        }
    }

    // Additions

    [Fact]
    public void Explorer_with_a_type_to_avoid_and_depth()
    {
        Shape aBox = MakeBox();
        var aBuilder = new Builder();
        Compound aCompound = new();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox);
        Vertex aFree = TestShapes.Frozen(TestShapes.Vertex(new Point3D(50, 0, 0)));
        aBuilder.Add(aCompound, aFree);

        int aCount = 0;
        for (var anExp = new Explorer(aCompound, ShapeType.Vertex, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(anExp.Current().IsSame(aFree));
            Assert.Equal(1, anExp.Depth());
            aCount++;
        }

        Assert.Equal(1, aCount);

        var anEdgeExp = new Explorer(aCompound, ShapeType.Edge);
        Assert.Equal(5, anEdgeExp.Depth()); // iterators on the compound, solid, shell, face and wire
        Assert.True(anEdgeExp.ExploredShape().IsEqual(aCompound));

        // a shape simpler than the type to find, and SHAPE, find nothing
        Assert.False(new Explorer(aFree, ShapeType.Edge).More());
        Assert.False(new Explorer(aCompound, ShapeType.Shape).More());
        // the shape itself when it has the type to find
        var aSelf = new Explorer(aFree, ShapeType.Vertex);
        Assert.True(aSelf.More() && aSelf.Current().IsEqual(aFree) && aSelf.Depth() == 0);
    }

    [Fact]
    public void Vertices_of_wires()
    {
        Shape aBox = MakeBox();
        var aWireExp = new Explorer(aBox, ShapeType.Wire);
        Wire aClosed = ShapeCasts.Wire(aWireExp.Current());
        Exploration.Vertices(aClosed, out Vertex aV1, out Vertex aV2);
        Assert.True(aV1.IsSame(aV2));
        Assert.Equal(Forward, aV1.Orientation());
        Assert.Equal(Reversed, aV2.Orientation());

        // an open wire of two segments
        Vertex aA = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aB = TestShapes.Vertex(new Point3D(1, 0, 0));
        Vertex aC = TestShapes.Vertex(new Point3D(1, 1, 0));
        var aBuilder = new BrepBuilder();
        Wire aWire = new();
        aBuilder.MakeWire(ref aWire);
        aBuilder.Add(aWire, TestShapes.Edge(aA, aB));
        aBuilder.Add(aWire, TestShapes.Edge(aC, aB).Reversed());
        Exploration.Vertices(aWire, out Vertex aFirst, out Vertex aLast);
        Assert.True(aFirst.IsSame(aA));
        Assert.True(aLast.IsSame(aC));
    }
}
