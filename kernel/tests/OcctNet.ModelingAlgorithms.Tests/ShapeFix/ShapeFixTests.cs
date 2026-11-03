// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix's package statics: SameParameter, EncodeRegularity, FixVertexPosition and LeastEdgeSize, and their twins on
/// frozen inputs.
/// </summary>
public class ShapeFixTests
{
    private static Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    [Fact]
    public void LeastEdgeSize_OfABox()
    {
        Assert.Equal(10.0, ShapeFix.LeastEdgeSize(Box()), 1e-12);
    }

    [Fact]
    public void SameParametered_ValidBox_IsTheBoxItself()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aRes = ShapeFix.SameParametered(aBox, false, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aBox);
        Assert.Same(aBox.TShape(), aRes.TShape());
    }

    [Fact]
    public void SameParametered_Enforced_LeavesTheInputAndGivesAValidShape()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aRes = ShapeFix.SameParametered(aBox, true, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRes.TShape()!.IsFrozen);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
        for (var anExp = new Explorer(aRes, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(Tool.SameParameter(ShapeCasts.Edge(anExp.Current())));
        }
    }

    [Fact]
    public void SameParameter_InPlace_FixesAShiftedPCurve()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), 1e-7);
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(-10, 0), new Direction2D(1, 0)), aF, 1e-7);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);
        aB.SameParameter(anE, false);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        Assert.True(ShapeFix.SameParameter(aF, false));
        Assert.True(Tool.SameParameter(anE));
        Assert.True(Tool.SameRange(anE));
    }

    [Fact]
    public void RegularityEncoded_FrozenBox()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aRes = ShapeFix.RegularityEncoded(aBox);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aRes.TShape()!.IsFrozen);
    }

    // two segments from one vertex of tolerance 1 at the origin: (0,0,0)-(10,0,0) and (0,0.5,0)-(0,10,0)
    private static Compound TwoEdgesOnATolerantVertex()
    {
        var aB = new BrepBuilder();
        var aV = new Vertex();
        aB.MakeVertex(ref aV, new Point3D(0, 0, 0), 1.0);
        Edge aE1 = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10, aV,
                                          TestShapes.Vertex(new Point3D(10, 0, 0)));
        Edge aE2 = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0.5, 0), new Direction3D(0, 1, 0)), 0, 9.5, aV,
                                          TestShapes.Vertex(new Point3D(0, 10, 0)));
        var aC = new Compound();
        aB.MakeCompound(ref aC);
        aB.Add(aC, aE1);
        aB.Add(aC, aE2);
        return aC;
    }

    [Fact]
    public void VertexPositionFixed_SplitsTheTolerantVertex()
    {
        Compound aC = TestShapes.Frozen(TwoEdgesOnATolerantVertex());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aC);
        var aContext = new ShapeBuild_ReShape();
        Shape aRes = ShapeFix.VertexPositionFixed(aC, 0.1, aContext, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(aC);
        Assert.True(aRes.TShape()!.IsFrozen);
        // each edge now starts on its own curve's end, with a tight vertex
        for (var anExp = new Explorer(aRes, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Curve3D aCrv = Tool.Curve(anE, out double f, out _)!;
            Vertex aV = Exploration.FirstVertex(anE);
            Assert.True(Tool.Pnt(aV).Distance(aCrv.Value(f)) < 1e-9);
            Assert.True(Tool.Tolerance(aV) < 1e-6);
        }
        // the context answers for the caller's edges
        for (var anExp = new Explorer(aC, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(aContext.IsRecorded(anExp.Current()));
        }
    }

    [Fact]
    public void VertexPositionFixed_NothingToDo_IsTheInput()
    {
        Shape aBox = Box();
        Shape aRes = ShapeFix.VertexPositionFixed(aBox, 1e-3, new ShapeBuild_ReShape(), out bool isDone);
        Assert.False(isDone);
        Assert.Same(aBox.TShape(), aRes.TShape());
    }

    [Fact]
    public void FixVertexPosition_InPlace()
    {
        Shape aC = TwoEdgesOnATolerantVertex();
        var aContext = new ShapeBuild_ReShape();
        Assert.True(ShapeFix.FixVertexPosition(ref aC, 0.1, aContext));
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aC, ShapeType.Vertex, aMap);
        Assert.Equal(4, aMap.Extent());
    }
}
