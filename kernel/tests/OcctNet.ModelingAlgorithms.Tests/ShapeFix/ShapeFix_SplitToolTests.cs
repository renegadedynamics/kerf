// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

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
        var aTool = new ShapeFix_SplitTool();
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
        Assert.True(new ShapeFix_SplitTool().SplitEdge(anE, 4.0, aV, PlaneFace(), ref aE1, ref aE2, 1e-7, 1e-9));
        Assert.True(Tool.Tolerance(aV) >= 0.01 - 1e-12);
    }

    [Fact]
    public void CutEdge_CutsTheRange()
    {
        Edge anE = Segment();
        bool isCutLine = true;
        Assert.True(new ShapeFix_SplitTool().CutEdge(anE, 0.0, 6.0, PlaneFace(), ref isCutLine));
        Assert.False(isCutLine);
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(0.0, f, 1e-12);
        Assert.Equal(6.0, l, 1e-12);
        Assert.False(new ShapeFix_SplitTool().CutEdge(anE, 3.0, 3.0, PlaneFace(), ref isCutLine));
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
        var aContext = new ShapeBuild_ReShape();
        Assert.True(new ShapeFix_SplitTool().SplitEdge(anE, 2.0, aV1, 6.0, aV2, aF, aSeq, ref aNum, aContext, 1e-7, 1e-9));
        Assert.Equal(3, aSeq.Length());
        Assert.Equal(2, aNum);
        Assert.Equal(ShapeType.Wire, aContext.InternalValue(anE).ShapeType());
        Tool.Range(ShapeCasts.Edge(aSeq.Value(2)), out double f, out double l);
        Assert.Equal(2.0, f, 1e-9);
        Assert.Equal(6.0, l, 1e-9);
    }
}
