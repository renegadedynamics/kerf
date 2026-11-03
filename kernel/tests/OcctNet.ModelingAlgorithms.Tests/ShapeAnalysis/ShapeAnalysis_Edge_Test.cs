// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeAnalysis_Edge_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeAnalysis_Edge.</summary>
public class ShapeAnalysis_EdgeTest
{
    [Fact]
    public void HasCurve3d()
    {
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(10.0, 0.0, 0.0));
        Assert.True(aMakeEdge.IsDone());
        Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.True(anAnalyzer.HasCurve3d(anEdge));
    }

    [Fact]
    public void FirstVertex_LastVertex()
    {
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(10.0, 0.0, 0.0));
        Assert.True(aMakeEdge.IsDone());
        Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Vertex aFirstVertex = anAnalyzer.FirstVertex(anEdge);
        Vertex aLastVertex = anAnalyzer.LastVertex(anEdge);

        Assert.False(aFirstVertex.IsNull);
        Assert.False(aLastVertex.IsNull);
    }

    [Fact]
    public void IsClosed_OpenEdge()
    {
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(10.0, 0.0, 0.0));
        Assert.True(aMakeEdge.IsDone());
        Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.False(anAnalyzer.IsClosed3d(anEdge));
    }

    [Fact]
    public void IsClosed_ClosedEdge()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0.0, 0.0, 0.0), Constants.DZ()), 5.0);
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aCircle);
        Assert.True(aMakeEdge.IsDone());
        Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.True(anAnalyzer.IsClosed3d(anEdge));
    }

    [Fact]
    public void IsSeam_NonSeam()
    {
        var aMakeBox = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aMakeBox.Shape();
        Assert.True(aMakeBox.IsDone());

        var aFaceExp = new Explorer(aBox, ShapeType.Face);
        Assert.True(aFaceExp.More());
        Face aFace = ShapeCasts.Face(aFaceExp.Current());

        var anEdgeExp = new Explorer(aFace, ShapeType.Edge);
        Assert.True(anEdgeExp.More());
        Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.False(anAnalyzer.IsSeam(anEdge, aFace));
    }
}
