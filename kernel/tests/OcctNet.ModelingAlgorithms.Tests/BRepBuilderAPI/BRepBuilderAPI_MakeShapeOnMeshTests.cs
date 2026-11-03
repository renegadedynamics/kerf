// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepBuilderAPI_MakeShapeOnMesh (unit 8H1; compared with native through StlAPI_Reader in RWStlNativeTests).</summary>
public class BRepBuilderAPI_MakeShapeOnMeshTests
{
    private static int Count(in Shape theS, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theS, theType, aMap);
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
        var aMaker = new MakeShapeOnMesh(Quad());
        aMaker.Build();
        Assert.True(aMaker.IsDone);
        Shape aShape = aMaker.Shape();
        Assert.Equal(ShapeType.Compound, aShape.ShapeType());
        Assert.True(aShape.TShape()!.IsFrozen);
        Assert.Equal(2, Count(aShape, ShapeType.Face));
        Assert.Equal(5, Count(aShape, ShapeType.Edge));
        // the vertices of the nodes the triangles use
        Assert.Equal(4, Count(aShape, ShapeType.Vertex));
        // both faces face +Z, as the triangles' node order says
        for (var anExp = new Explorer(aShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aSurf = new FaceSurfaceAdaptor(ShapeCasts.Face(anExp.Current()));
            Direction3D aNorm = aSurf.Plane().Axis().Direction();
            double aSign = anExp.Current().Orientation() == Orientation.Reversed ? -1 : 1;
            Assert.Equal(1.0, aSign * aNorm.Z(), 12);
        }
        Assert.True(new Analyzer(aShape).IsValid());
    }

    [Fact]
    public void Degenerate_triangles_are_skipped()
    {
        var aMaker = new MakeShapeOnMesh(Quad(new Triangle1(1, 1, 2), new Triangle1(3, 4, 5)));
        Shape aShape = aMaker.Shape();
        Assert.Equal(2, Count(aShape, ShapeType.Face));
    }

    [Fact]
    public void A_null_or_empty_mesh_builds_nothing()
    {
        var aMaker = new MakeShapeOnMesh(null);
        aMaker.Build();
        Assert.False(aMaker.IsDone);
        Assert.ThrowsAny<NotDoneException>(() => aMaker.Shape());
        Assert.False(new MakeShapeOnMesh(new Triangulation()).IsDone);
    }

    [Fact]
    public void A_user_break_stops_the_build()
    {
        using var aSource = new CancellationTokenSource();
        aSource.Cancel();
        var aMaker = new MakeShapeOnMesh(Quad());
        aMaker.Build(new ProgressIndicator(aSource.Token).Start());
        Assert.False(aMaker.IsDone);
    }
}
