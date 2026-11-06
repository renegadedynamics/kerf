// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>ShapeFix_SplitTool's splits and cuts of edges on a plane, built unfrozen as its callers' are.</summary>
public class ShapeFix_SplitToolTests
{
    private static Face PlaneFace()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), 1e-7);
        return aF;
    }

    private static Edge Segment() => TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));

    [Fact]
    public void SplitEdge_AtAParameter()
    {
        Face aF = PlaneFace();
        Edge anE = Segment();
        Vertex aV = TestShapes.Vertex(new Point3D(4, 0, 0));
        Edge aE1 = new(), aE2 = new();
        var aTool = new SplitToolFixer();
        Assert.True(aTool.SplitEdge(anE, 4.0, aV, aF, ref aE1, ref aE2, 1e-7, 1e-9));
        Tool.Range(aE1, out double f1, out double l1);
        Tool.Range(aE2, out double f2, out double l2);
        Assert.Equal(0.0, f1, 1e-9);
        Assert.Equal(4.0, l1, 1e-9);
        Assert.Equal(4.0, f2, 1e-9);
        Assert.Equal(10.0, l2, 1e-9);
        Assert.True(Exploration.LastVertex(aE1).IsSame(aV));
        Assert.True(Exploration.FirstVertex(aE2).IsSame(aV));
        // at an end: nothing
        Assert.False(aTool.SplitEdge(anE, 0.0, aV, aF, ref aE1, ref aE2, 1e-7, 1e-9));
    }

    [Fact]
    public void SplitEdge_RaisesAFarVertex()
    {
        Edge anE = Segment();
        Vertex aV = TestShapes.Vertex(new Point3D(4, 0.01, 0));
        Edge aE1 = new(), aE2 = new();
        Assert.True(new SplitToolFixer().SplitEdge(anE, 4.0, aV, PlaneFace(), ref aE1, ref aE2, 1e-7, 1e-9));
        Assert.True(Tool.Tolerance(aV) >= 0.01 - 1e-12);
    }

    [Fact]
    public void CutEdge_CutsTheRange()
    {
        Edge anE = Segment();
        bool isCutLine = true;
        Assert.True(new SplitToolFixer().CutEdge(anE, 0.0, 6.0, PlaneFace(), ref isCutLine));
        Assert.False(isCutLine);
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(0.0, f, 1e-12);
        Assert.Equal(6.0, l, 1e-12);
        Assert.False(new SplitToolFixer().CutEdge(anE, 3.0, 3.0, PlaneFace(), ref isCutLine));
    }

    [Fact]
    public void SplitEdge_BetweenTwoVertices_RecordsTheWireInTheContext()
    {
        Face aF = PlaneFace();
        Edge anE = Segment();
        Vertex aV1 = TestShapes.Vertex(new Point3D(2, 0, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(6, 0, 0));
        var aSeq = new NCollection_Sequence<Shape>();
        int aNum = -1;
        var aContext = new ReShapeBuilder();
        Assert.True(new SplitToolFixer().SplitEdge(anE, 2.0, aV1, 6.0, aV2, aF, aSeq, ref aNum, aContext, 1e-7, 1e-9));
        Assert.Equal(3, aSeq.Length());
        Assert.Equal(2, aNum);
        Assert.Equal(ShapeType.Wire, aContext.InternalValue(anE).Type);
        Tool.Range(ShapeCasts.Edge(aSeq.Value(2)), out double f, out double l);
        Assert.Equal(2.0, f, 1e-9);
        Assert.Equal(6.0, l, 1e-9);
    }

    [Theory]
    [InlineData(4.0, 6.0)]
    [InlineData(6.0, 4.0)]
    public void SplitEdge_OverAStretch_CutsBothPartsAtItsEnds(double theP1, double theP2)
    {
        // split at the stretch's middle: the two parts stop short of it, at its ends, sharing the vertex at the middle
        Face aF = PlaneFace();
        Vertex aV = TestShapes.Vertex(new Point3D(5, 0, 0));
        Edge aE1 = new(), aE2 = new();
        Assert.True(new SplitToolFixer().SplitEdge(Segment(), theP1, theP2, aV, aF, ref aE1, ref aE2, 1e-7, 1e-9));
        Tool.Range(aE1, out double f1, out double l1);
        Tool.Range(aE2, out double f2, out double l2);
        Assert.Equal(0.0, f1, 1e-9);
        Assert.Equal(4.0, l1, 1e-9);
        Assert.Equal(6.0, f2, 1e-9);
        Assert.Equal(10.0, l2, 1e-9);
        Assert.True(Exploration.LastVertex(aE1).IsSame(aV));
        Assert.True(Exploration.FirstVertex(aE2).IsSame(aV));
    }

    [Fact]
    public void SplitEdge_OverAStretch_OfAReversedEdge_KeepsTheOrderAlongTheWire()
    {
        // the first part along a reversed edge is the one nearer the curve's end
        Face aF = PlaneFace();
        Edge anE = ShapeCasts.Edge(Segment().Reversed());
        Vertex aV = TestShapes.Vertex(new Point3D(5, 0, 0));
        Edge aE1 = new(), aE2 = new();
        Assert.True(new SplitToolFixer().SplitEdge(anE, 4.0, 6.0, aV, aF, ref aE1, ref aE2, 1e-7, 1e-9));
        Assert.Equal(Orientation.Reversed, aE1.Orientation);
        Assert.Equal(Orientation.Reversed, aE2.Orientation);
        Tool.Range(aE1, out double f1, out double l1);
        Tool.Range(aE2, out double f2, out double l2);
        Assert.Equal(6.0, f1, 1e-9);
        Assert.Equal(10.0, l1, 1e-9);
        Assert.Equal(0.0, f2, 1e-9);
        Assert.Equal(4.0, l2, 1e-9);
    }

    // An edge whose range and pcurve aren't the same parameter: its pcurve is a trimmed line twice as long
    private static Edge NotSameParameter(in Face theF)
    {
        var aB = new BrepBuilder();
        Edge anE = Segment();
        var aLine = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        aB.UpdateEdge(anE, new TrimmedCurve2D(aLine, 0, 20), theF, 1e-7);
        aB.Range(anE, theF, 0, 20);
        aB.SameParameter(anE, false);
        return anE;
    }

    [Fact]
    public void CutEdge_OfALinePCurveNotSameParameter_ScalesTheCutToThe3dRange()
    {
        Face aF = PlaneFace();
        Edge anE = NotSameParameter(aF);
        bool isCutLine = false;
        // from the beginning: a quarter of the pcurve's range is a quarter of the 3d range
        Assert.True(new SplitToolFixer().CutEdge(anE, 20, 5, aF, ref isCutLine));
        Assert.True(isCutLine);
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(2.5, f, 1e-9);
        Assert.Equal(10, l, 1e-9);

        Edge anE2 = NotSameParameter(aF);
        isCutLine = false;
        Assert.True(new SplitToolFixer().CutEdge(anE2, 0, 15, aF, ref isCutLine));
        Assert.True(isCutLine);
        Tool.Range(anE2, out f, out l);
        Assert.Equal(0, f, 1e-9);
        Assert.Equal(7.5, l, 1e-9);
    }

    [Fact]
    public void CutEdge_OfAnEdgeNotSameParameter_WithoutALine_LeavesIt()
    {
        Face aF = PlaneFace();
        var aB = new BrepBuilder();
        Edge anE = Segment();
        var aSpline = new BSplineCurve2D(new[] { new Point2D(0, 0), new Point2D(10, 0) }, [0.0, 10.0], [2, 2], 1);
        aB.UpdateEdge(anE, aSpline, aF, 1e-7);
        aB.Range(anE, aF, 0, 10);
        aB.SameParameter(anE, false);
        bool isCutLine = true;
        Assert.True(new SplitToolFixer().CutEdge(anE, 0, 5, aF, ref isCutLine));
        Assert.False(isCutLine);
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(0, f, 1e-9);
        Assert.Equal(10, l, 1e-9);
    }

    [Fact]
    public void CutEdge_OnAPeriodicCurve_KeepsTheRangeValid()
    {
        // past the circle's parameter range: the range is brought back, and the edge stays the same arc
        Face aF = PlaneFace();
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 5);
        Edge anE = TestShapes.Edge(aCircle, 1, 3, TestShapes.Vertex(aCircle.Value(1)), TestShapes.Vertex(aCircle.Value(3)));
        bool isCutLine = false;
        Assert.True(new SplitToolFixer().CutEdge(anE, 2, 1.5, aF, ref isCutLine));
        Assert.False(isCutLine);
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(1.5, f, 1e-9);
        Assert.Equal(2, l, 1e-9);
    }

    public static TheoryData<double, double, int, int> StretchesOfTheEdge() => new()
    {
        // from, to, parts, index of the stretch among them
        { 0, 10, 1, 1 }, // all of it
        { 0, 6, 2, 1 }, // up to a vertex at 6
        { 4, 10, 2, 2 }, // from 4 to the end
        { 10, 4, 2, 2 },
        { 6, 0, 2, 1 },
        { 6, 2, 3, 2 }, // given backwards
    };

    [Theory]
    [MemberData(nameof(StretchesOfTheEdge))]
    public void SplitEdge_BetweenTwoVertices_ReusesTheEdgesOwnEnds(double theFrom, double theTo, int theParts, int theStretch)
    {
        Face aF = PlaneFace();
        Edge anE = Segment();
        Vertex aV1 = TestShapes.Vertex(new Point3D(theFrom, 0, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(theTo, 0, 0));
        var aSeq = new NCollection_Sequence<Shape>();
        int aNum = -1;
        var aContext = new ReShapeBuilder();
        Assert.True(new SplitToolFixer().SplitEdge(anE, theFrom, aV1, theTo, aV2, aF, aSeq, ref aNum, aContext, 1e-7, 1e-9));
        Assert.Equal(theParts, aSeq.Length());
        Assert.Equal(theStretch, aNum);
        // the stretch covers from..to, and all the parts together cover the edge, in order and without overlap
        Tool.Range(ShapeCasts.Edge(aSeq.Value(aNum)), out double f, out double l);
        Assert.Equal(Math.Min(theFrom, theTo), f, 1e-9);
        Assert.Equal(Math.Max(theFrom, theTo), l, 1e-9);
        double aExpectedStart = 0;
        for (int i = 1; i <= aSeq.Length(); i++)
        {
            Tool.Range(ShapeCasts.Edge(aSeq.Value(i)), out double fi, out double li);
            Assert.Equal(aExpectedStart, fi, 1e-9);
            aExpectedStart = li;
        }
        Assert.Equal(10, aExpectedStart, 1e-9);
    }

    [Fact]
    public void SplitEdge_BetweenTwoVertices_TooClose_DoesNothing()
    {
        Face aF = PlaneFace();
        Vertex aV = TestShapes.Vertex(new Point3D(5, 0, 0));
        var aSeq = new NCollection_Sequence<Shape>();
        int aNum = -1;
        Assert.False(new SplitToolFixer().SplitEdge(Segment(), 5, aV, 5, aV, aF, aSeq, ref aNum, new ReShapeBuilder(), 1e-7, 1e-9));
        Assert.Equal(0, aSeq.Length());
    }

    [Fact]
    public void SplitEdge_WithoutSameParameter_MeasuresTheVertexOnTheSurface()
    {
        // not same parameter: the point at the parameter comes from the pcurve and the surface
        Face aF = PlaneFace();
        var aB = new BrepBuilder();
        Edge anE = Segment();
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, 1e-7);
        aB.Range(anE, aF, 0, 10);
        aB.SameParameter(anE, false);
        Vertex aV = TestShapes.Vertex(new Point3D(4, 0.02, 0));
        Edge aE1 = new(), aE2 = new();
        Assert.True(new SplitToolFixer().SplitEdge(anE, 4.0, aV, aF, ref aE1, ref aE2, 1e-7, 1e-9));
        Assert.True(Tool.Tolerance(aV) >= 0.02 - 1e-12);
    }
}
