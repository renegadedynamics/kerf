// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeAnalysis_Edge_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeAnalysis_Edge.</summary>
public class ShapeAnalysis_EdgeTest
{
    [Fact]
    public void HasCurve3d()
    {
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0.0, 0.0, 0.0), new gp_Pnt(10.0, 0.0, 0.0));
        Assert.True(aMakeEdge.IsDone());
        TopoDS_Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.True(anAnalyzer.HasCurve3d(anEdge));
    }

    [Fact]
    public void FirstVertex_LastVertex()
    {
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0.0, 0.0, 0.0), new gp_Pnt(10.0, 0.0, 0.0));
        Assert.True(aMakeEdge.IsDone());
        TopoDS_Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        TopoDS_Vertex aFirstVertex = anAnalyzer.FirstVertex(anEdge);
        TopoDS_Vertex aLastVertex = anAnalyzer.LastVertex(anEdge);

        Assert.False(aFirstVertex.IsNull());
        Assert.False(aLastVertex.IsNull());
    }

    [Fact]
    public void IsClosed_OpenEdge()
    {
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0.0, 0.0, 0.0), new gp_Pnt(10.0, 0.0, 0.0));
        Assert.True(aMakeEdge.IsDone());
        TopoDS_Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.False(anAnalyzer.IsClosed3d(anEdge));
    }

    [Fact]
    public void IsClosed_ClosedEdge()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), gp.DZ()), 5.0);
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aCircle);
        Assert.True(aMakeEdge.IsDone());
        TopoDS_Edge anEdge = aMakeEdge.Edge();

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.True(anAnalyzer.IsClosed3d(anEdge));
    }

    [Fact]
    public void IsSeam_NonSeam()
    {
        var aMakeBox = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0);
        TopoDS_Shape aBox = aMakeBox.Shape();
        Assert.True(aMakeBox.IsDone());

        var aFaceExp = new TopExp_Explorer(aBox, TopAbs_FACE);
        Assert.True(aFaceExp.More());
        TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());

        var anEdgeExp = new TopExp_Explorer(aFace, TopAbs_EDGE);
        Assert.True(anEdgeExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());

        var anAnalyzer = new ShapeAnalysis_Edge();
        Assert.False(anAnalyzer.IsSeam(anEdge, aFace));
    }
}
