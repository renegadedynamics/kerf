// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix's package statics: SameParameter, EncodeRegularity, FixVertexPosition and LeastEdgeSize, and their twins on
/// frozen inputs.
/// </summary>
public class ShapeFixTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    [Fact]
    public void LeastEdgeSize_OfABox()
    {
        Assert.Equal(10.0, ShapeFix.LeastEdgeSize(Box()), 1e-12);
    }

    [Fact]
    public void SameParametered_ValidBox_IsTheBoxItself()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aRes = ShapeFix.SameParametered(aBox, false, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aBox);
        Assert.Same(aBox.TShape(), aRes.TShape());
    }

    [Fact]
    public void SameParametered_Enforced_LeavesTheInputAndGivesAValidShape()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aRes = ShapeFix.SameParametered(aBox, true, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRes.TShape()!.IsFrozen());
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
        for (var anExp = new TopExp_Explorer(aRes, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(BRep_Tool.SameParameter(TopoDS.Edge(anExp.Current())));
        }
    }

    [Fact]
    public void SameParameter_InPlace_FixesAShiftedPCurve()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, new Geom_Plane(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0))), 1e-7);
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(-10, 0), new gp_Dir2d(1, 0)), aF, 1e-7);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);
        aB.SameParameter(anE, false);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        Assert.True(ShapeFix.SameParameter(aF, false));
        Assert.True(BRep_Tool.SameParameter(anE));
        Assert.True(BRep_Tool.SameRange(anE));
    }

    [Fact]
    public void RegularityEncoded_FrozenBox()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aRes = ShapeFix.RegularityEncoded(aBox);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRes.TShape()!.IsFrozen());
    }

    // two segments from one vertex of tolerance 1 at the origin: (0,0,0)-(10,0,0) and (0,0.5,0)-(0,10,0)
    private static TopoDS_Compound TwoEdgesOnATolerantVertex()
    {
        var aB = new BRep_Builder();
        var aV = new TopoDS_Vertex();
        aB.MakeVertex(ref aV, new gp_Pnt(0, 0, 0), 1.0);
        TopoDS_Edge aE1 = TestShapes.Edge(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 10, aV,
                                          TestShapes.Vertex(new gp_Pnt(10, 0, 0)));
        TopoDS_Edge aE2 = TestShapes.Edge(new Geom_Line(new gp_Pnt(0, 0.5, 0), new gp_Dir(0, 1, 0)), 0, 9.5, aV,
                                          TestShapes.Vertex(new gp_Pnt(0, 10, 0)));
        var aC = new TopoDS_Compound();
        aB.MakeCompound(ref aC);
        aB.Add(aC, aE1);
        aB.Add(aC, aE2);
        return aC;
    }

    [Fact]
    public void VertexPositionFixed_SplitsTheTolerantVertex()
    {
        TopoDS_Compound aC = TestShapes.Frozen(TwoEdgesOnATolerantVertex());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aC);
        var aContext = new ShapeBuild_ReShape();
        TopoDS_Shape aRes = ShapeFix.VertexPositionFixed(aC, 0.1, aContext, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aC);
        Assert.True(aRes.TShape()!.IsFrozen());
        // each edge now starts on its own curve's end, with a tight vertex
        for (var anExp = new TopExp_Explorer(aRes, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            Geom_Curve aCrv = BRep_Tool.Curve(anE, out double f, out _)!;
            TopoDS_Vertex aV = TopExp.FirstVertex(anE);
            Assert.True(BRep_Tool.Pnt(aV).Distance(aCrv.Value(f)) < 1e-9);
            Assert.True(BRep_Tool.Tolerance(aV) < 1e-6);
        }
        // the context answers for the caller's edges
        for (var anExp = new TopExp_Explorer(aC, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(aContext.IsRecorded(anExp.Current()));
        }
    }

    [Fact]
    public void VertexPositionFixed_NothingToDo_IsTheInput()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aRes = ShapeFix.VertexPositionFixed(aBox, 1e-3, new ShapeBuild_ReShape(), out bool isDone);
        Assert.False(isDone);
        Assert.Same(aBox.TShape(), aRes.TShape());
    }

    [Fact]
    public void FixVertexPosition_InPlace()
    {
        TopoDS_Shape aC = TwoEdgesOnATolerantVertex();
        var aContext = new ShapeBuild_ReShape();
        Assert.True(ShapeFix.FixVertexPosition(ref aC, 0.1, aContext));
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aC, TopAbs_VERTEX, aMap);
        Assert.Equal(4, aMap.Extent());
    }
}
