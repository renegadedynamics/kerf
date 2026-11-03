// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepBuilderAPI_MakeShapeOnMesh (unit 8H1; compared with native through StlAPI_Reader in RWStlNativeTests).</summary>
public class BRepBuilderAPI_MakeShapeOnMeshTests
{
    private static int Count(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, theType, aMap);
        return aMap.Extent();
    }

    private static Triangulation Quad(params Triangle1[] theExtra)
    {
        var aMesh = new Triangulation(5, 2 + theExtra.Length, false);
        aMesh.SetNode(1, new Point3D(0, 0, 0));
        aMesh.SetNode(2, new Point3D(10, 0, 0));
        aMesh.SetNode(3, new Point3D(10, 10, 0));
        aMesh.SetNode(4, new Point3D(0, 10, 0));
        aMesh.SetNode(5, new Point3D(0, 10, 0)); // at node 4: a geometrically degenerate triangle
        aMesh.SetTriangle(1, new Triangle1(1, 2, 3));
        aMesh.SetTriangle(2, new Triangle1(1, 3, 4));
        for (int i = 0; i < theExtra.Length; ++i)
        {
            aMesh.SetTriangle(3 + i, theExtra[i]);
        }
        return aMesh;
    }

    [Fact]
    public void Triangles_become_planar_faces_sharing_vertices_and_edges()
    {
        var aMaker = new BRepBuilderAPI_MakeShapeOnMesh(Quad());
        aMaker.Build();
        Assert.True(aMaker.IsDone());
        TopoDS_Shape aShape = aMaker.Shape();
        Assert.Equal(TopAbs_COMPOUND, aShape.ShapeType());
        Assert.True(aShape.TShape()!.IsFrozen);
        Assert.Equal(2, Count(aShape, TopAbs_FACE));
        Assert.Equal(5, Count(aShape, TopAbs_EDGE));
        // the vertices of the nodes the triangles use
        Assert.Equal(4, Count(aShape, TopAbs_VERTEX));
        // both faces face +Z, as the triangles' node order says
        for (var anExp = new TopExp_Explorer(aShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aSurf = new BRepAdaptor_Surface(TopoDS.Face(anExp.Current()));
            Direction3D aNorm = aSurf.Plane().Axis().Direction();
            double aSign = anExp.Current().Orientation() == TopAbs_Orientation.TopAbs_REVERSED ? -1 : 1;
            Assert.Equal(1.0, aSign * aNorm.Z(), 12);
        }
        Assert.True(new BRepCheck_Analyzer(aShape).IsValid());
    }

    [Fact]
    public void Degenerate_triangles_are_skipped()
    {
        var aMaker = new BRepBuilderAPI_MakeShapeOnMesh(Quad(new Triangle1(1, 1, 2), new Triangle1(3, 4, 5)));
        TopoDS_Shape aShape = aMaker.Shape();
        Assert.Equal(2, Count(aShape, TopAbs_FACE));
    }

    [Fact]
    public void A_null_or_empty_mesh_builds_nothing()
    {
        var aMaker = new BRepBuilderAPI_MakeShapeOnMesh(null);
        aMaker.Build();
        Assert.False(aMaker.IsDone());
        Assert.ThrowsAny<NotDoneException>(() => aMaker.Shape());
        Assert.False(new BRepBuilderAPI_MakeShapeOnMesh(new Triangulation()).IsDone());
    }

    [Fact]
    public void A_user_break_stops_the_build()
    {
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var aMaker = new BRepBuilderAPI_MakeShapeOnMesh(Quad());
        aMaker.Build(new ProgressIndicator(aSource.Token).Start());
        Assert.False(aMaker.IsDone());
    }
}
