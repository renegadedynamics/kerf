// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeFix_IntersectionTool (unit 6H2). Its fixes write in place, so they run on unfrozen shapes, as
/// ShapeFix_Wire and ShapeFix_Face run them on their copies.
/// </summary>
public class ShapeFix_IntersectionToolTests
{
    private static gp_Pnt P(double x, double y) => new(x, y, 0.0);

    // An unfrozen planar face bounded by the polygon.
    private static TopoDS_Face PlanarFace(params gp_Pnt[] thePoints)
    {
        var aPoly = new BRepBuilderAPI_MakePolygon();
        foreach (gp_Pnt aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        return new BRepBuilderAPI_MakeFace(aPoly.InternalWire(), true).InternalFace();
    }

    private static TopoDS_Wire OuterWire(in TopoDS_Face theFace)
    {
        var anIt = new TopoDS_Iterator(theFace);
        return TopoDS.Wire(anIt.Value());
    }

    private static int NbEdges(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_EDGE, aMap);
        return aMap.Extent();
    }

    [Fact]
    public void FixSelfIntersectWire_BowTie_SplitsBothCrossingEdges()
    {
        // (0,0) -> (2,2) -> (2,0) -> (0,2) -> back: two opposite edges cross at (1,1)
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(2, 2), P(2, 0), P(0, 2));
        var sewd = new ShapeExtend_WireData(OuterWire(aFace));
        var anOriginals = new List<TopoDS_Edge>();
        for (int i = 1; i <= sewd.NbEdges(); i++)
        {
            anOriginals.Add(sewd.Edge(i));
        }
        var aContext = new ShapeBuild_ReShape();
        var aTool = new ShapeFix_IntersectionTool(aContext, 1.0e-7);

        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.True(aTool.FixSelfIntersectWire(sewd, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));

        Assert.Equal(2, aNbSplit);
        Assert.Equal(0, aNbCut);
        Assert.Equal(0, aNbRemoved);
        Assert.Equal(6, sewd.NbEdges());
        Assert.Equal(2, anOriginals.Count(e => aContext.IsRecorded(e))); // the two crossing edges were replaced

        // the new vertex is shared by the four halves and lies at (1,1)
        var sae = new ShapeAnalysis_Edge();
        int aNbAtCross = 0;
        for (int i = 1; i <= sewd.NbEdges(); i++)
        {
            gp_Pnt anEnd = BRep_Tool.Pnt(sae.LastVertex(sewd.Edge(i)));
            if (anEnd.Distance(new gp_Pnt(1, 1, 0)) < 1.0e-9)
            {
                aNbAtCross++;
                int j = i == sewd.NbEdges() ? 1 : i + 1;
                Assert.True(sae.LastVertex(sewd.Edge(i)).IsSame(sae.FirstVertex(sewd.Edge(j))));
            }
        }
        Assert.Equal(2, aNbAtCross);
    }

    [Fact]
    public void FixSelfIntersectWire_NoContextOrNoFace_DoesNothing()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(2, 2), P(2, 0), P(0, 2));
        var sewd = new ShapeExtend_WireData(OuterWire(aFace));
        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.False(new ShapeFix_IntersectionTool(null, 1.0e-7)
                         .FixSelfIntersectWire(sewd, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.False(new ShapeFix_IntersectionTool(new ShapeBuild_ReShape(), 1.0e-7)
                         .FixSelfIntersectWire(sewd, new TopoDS_Face(), ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.Equal(4, sewd.NbEdges());
    }

    [Fact]
    public void FixSelfIntersectWire_Square_IsUnchanged()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(2, 0), P(2, 2), P(0, 2));
        var sewd = new ShapeExtend_WireData(OuterWire(aFace));
        int aNbSplit = 0, aNbCut = 0, aNbRemoved = 0;
        Assert.False(new ShapeFix_IntersectionTool(new ShapeBuild_ReShape(), 1.0e-7)
                         .FixSelfIntersectWire(sewd, aFace, ref aNbSplit, ref aNbCut, ref aNbRemoved));
        Assert.Equal(4, sewd.NbEdges());
    }

    [Fact]
    public void FixIntersectingWires_CrossingWires_SplitsTheirEdgesAndReplacesTheFace()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        // an inner rectangle crossing the side x = 4
        var aPoly = new BRepBuilderAPI_MakePolygon(new gp_Pnt(3, 1, 0), new gp_Pnt(5, 1, 0), new gp_Pnt(5, 2, 0),
                                                   new gp_Pnt(3, 2, 0), true);
        TopoDS_Wire anInner = aPoly.InternalWire();
        anInner.Reverse();
        new BRep_Builder().Add(aFace, anInner);
        TopoDS_Face anOriginal = aFace;
        var aContext = new ShapeBuild_ReShape();
        var aTool = new ShapeFix_IntersectionTool(aContext, 1.0e-7);

        Assert.True(aTool.FixIntersectingWires(ref aFace));

        Assert.False(ReferenceEquals(aFace.TShape(), anOriginal.TShape()));
        Assert.True(aContext.IsRecorded(anOriginal));
        int aNbWires = 0;
        int aNbEdgesTotal = 0;
        for (var anIt = new TopoDS_Iterator(aFace); anIt.More(); anIt.Next(), aNbWires++)
        {
            aNbEdgesTotal += NbEdges(anIt.Value());
        }
        Assert.Equal(2, aNbWires);
        Assert.True(aNbEdgesTotal > 8);
    }

    [Fact]
    public void FixIntersectingWires_SingleWire_ReturnsFalse()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(4, 0), P(4, 4), P(0, 4));
        TopoDS_Face anOriginal = aFace;
        Assert.False(new ShapeFix_IntersectionTool(new ShapeBuild_ReShape(), 1.0e-7).FixIntersectingWires(ref aFace));
        Assert.True(ReferenceEquals(aFace.TShape(), anOriginal.TShape()));
    }

    [Fact]
    public void SplitEdge_AtAParameter_GivesTwoEdgesSharingTheVertex()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).InternalEdge();
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(4, 0, 0)).InternalVertex();
        var aTool = new ShapeFix_IntersectionTool(null, 1.0e-7);

        TopoDS_Edge aE1 = new(), aE2 = new();
        Assert.True(aTool.SplitEdge(anEdge, 4.0, aV, aFace, ref aE1, ref aE2, 1.0e-7));
        BRep_Tool.Range(aE1, out double f1, out double l1);
        BRep_Tool.Range(aE2, out double f2, out double l2);
        Assert.Equal(0.0, f1, 1e-12);
        Assert.Equal(4.0, l1, 1e-12);
        Assert.Equal(4.0, f2, 1e-12);
        Assert.Equal(10.0, l2, 1e-12);
        var sae = new ShapeAnalysis_Edge();
        Assert.True(sae.LastVertex(aE1).IsSame(aV));
        Assert.True(sae.FirstVertex(aE2).IsSame(aV));

        // an end vertex or an end parameter: no split
        Assert.False(aTool.SplitEdge(anEdge, 4.0, sae.FirstVertex(anEdge), aFace, ref aE1, ref aE2, 1.0e-7));
        Assert.False(aTool.SplitEdge(anEdge, 10.0, aV, aFace, ref aE1, ref aE2, 1.0e-7));
    }

    [Fact]
    public void SplitEdge_VertexOffTheEdge_RaisesItsTolerance()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).InternalEdge();
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(4, 0.01, 0)).InternalVertex();
        TopoDS_Edge aE1 = new(), aE2 = new();
        Assert.True(new ShapeFix_IntersectionTool(null, 1.0e-7).SplitEdge(anEdge, 4.0, aV, aFace, ref aE1, ref aE2, 1.0e-7));
        Assert.True(BRep_Tool.Tolerance(aV) >= 0.01);
    }

    [Fact]
    public void CutEdge_ChangesTheRangeInPlace_AndRefusesFrozenEdges()
    {
        TopoDS_Face aFace = PlanarFace(P(0, 0), P(10, 0), P(10, 10), P(0, 10));
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).InternalEdge();
        var aTool = new ShapeFix_IntersectionTool(null, 1.0e-7);
        bool isCutLine = false;

        Assert.True(aTool.CutEdge(anEdge, 10.0, 6.0, aFace, ref isCutLine));
        BRep_Tool.Range(anEdge, out double f, out double l);
        Assert.Equal(6.0, f);
        Assert.Equal(10.0, l);
        Assert.False(isCutLine);

        // too small a cut, or the whole range: nothing
        Assert.False(aTool.CutEdge(anEdge, 10.0, 10.0, aFace, ref isCutLine));
        Assert.False(aTool.CutEdge(anEdge, 10.0, 6.0, aFace, ref isCutLine));

        TopoDS_Edge aFrozen = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).Edge();
        Assert.ThrowsAny<Exception>(() => aTool.CutEdge(aFrozen, 10.0, 6.0, aFace, ref isCutLine));
    }
}
