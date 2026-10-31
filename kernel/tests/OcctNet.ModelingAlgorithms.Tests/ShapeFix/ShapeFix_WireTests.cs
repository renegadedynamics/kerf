// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Wire's fixes one by one (reorder, small, connected, edge curves, degenerated, self-intersection, lacking,
/// gaps in 3d and 2d, shifted, tails), each on a frozen wire: the caller's wire unchanged, the result frozen, the caller's
/// own edges and vertices kept where nothing changed, the context answering for the caller's shapes.
/// </summary>
public class ShapeFix_WireTests
{
    private const double Conf = 1e-7;

    private static Geom_Plane XOY() => new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));

    private static TopoDS_Face PlaneFace()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        return TestShapes.Frozen(aF);
    }

    private static TopoDS_Wire MakeWire(params TopoDS_Edge[] theEdges)
    {
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (TopoDS_Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    private static List<TopoDS_Edge> Edges(in TopoDS_Shape theShape)
    {
        var aList = new List<TopoDS_Edge>();
        for (var anIt = new TopoDS_Iterator(theShape); anIt.More(); anIt.Next())
        {
            aList.Add(TopoDS.Edge(anIt.Value()));
        }
        return aList;
    }

    private static bool Shares(in TopoDS_Shape theA, in TopoDS_Shape theB) => ReferenceEquals(theA.TShape(), theB.TShape());

    // the square [0, 10]^2 in XOY, its vertices shared
    private static TopoDS_Edge[] Square(out TopoDS_Vertex[] theVertices)
    {
        theVertices =
        [
            TestShapes.Vertex(new gp_Pnt(0, 0, 0)), TestShapes.Vertex(new gp_Pnt(10, 0, 0)),
            TestShapes.Vertex(new gp_Pnt(10, 10, 0)), TestShapes.Vertex(new gp_Pnt(0, 10, 0)),
        ];
        var anEdges = new TopoDS_Edge[4];
        for (int i = 0; i < 4; i++)
        {
            anEdges[i] = TestShapes.Edge(theVertices[i], theVertices[(i + 1) % 4]);
        }
        return anEdges;
    }

    private static void AssertConnected(in TopoDS_Wire theWire)
    {
        var sawo = new ShapeAnalysis_WireOrder();
        var saw = new ShapeAnalysis_Wire(theWire, new TopoDS_Face(), Conf);
        saw.CheckOrder(sawo, true, true);
        Assert.False(saw.StatusOrder(ShapeExtend_DONE));
        Assert.True(saw.CheckConnected() == false || !saw.StatusConnected(ShapeExtend_FAIL));
    }

    [Fact]
    public void FixReorder_ShuffledSquare_KeepsTheCallersEdges()
    {
        TopoDS_Edge[] anE = Square(out _);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anE[2], anE[0], anE[3], anE[1]));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, PlaneFace(), Conf);
        Assert.True(sfw.FixReorder());
        Assert.True(sfw.StatusReorder(ShapeExtend_DONE1));
        TopoDS_Wire aRes = sfw.Wire();

        aSnap.AssertUnchanged(aW);
        Assert.True(aRes.TShape()!.IsFrozen());
        List<TopoDS_Edge> aResEdges = Edges(aRes);
        Assert.Equal(4, aResEdges.Count);
        // only the order changed: the edges are the caller's own
        Assert.All(aResEdges, e => Assert.Contains(anE, c => Shares(c, e)));
        for (int i = 0; i < 4; i++)
        {
            Assert.True(Shares(TopExp.LastVertex(aResEdges[i], true), TopExp.FirstVertex(aResEdges[(i + 1) % 4], true)));
        }
    }

    [Fact]
    public void FixSmall_RemovesATinyEdge()
    {
        TopoDS_Edge[] anE = Square(out TopoDS_Vertex[] aV);
        // a 1e-9 edge after the first corner, the next edge starting at its end
        TopoDS_Vertex aV1b = TestShapes.Vertex(new gp_Pnt(10, 1e-9, 0));
        TopoDS_Edge aTiny = TestShapes.Edge(aV[1], aV1b);
        TopoDS_Edge aSide = TestShapes.Edge(aV1b, aV[2]);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anE[0], aTiny, aSide, anE[2], anE[3]));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, PlaneFace(), 1e-7);
        sfw.ModifyTopologyMode() = true;
        Assert.True(sfw.FixSmall(false, 1e-6) > 0);
        Assert.True(sfw.StatusSmall(ShapeExtend_DONE));
        Assert.Equal(4, sfw.NbEdges());
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(4, Edges(aRes).Count);
        AssertConnected(aRes);
    }

    [Fact]
    public void FixConnected_MergesNearVertices_AndTheContextAnswersForTheCallersVertices()
    {
        // four segments, each with its own vertices, the ends 1e-5 apart
        var aP = new[] { new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0), new gp_Pnt(0, 10, 0) };
        var anEdges = new TopoDS_Edge[4];
        for (int i = 0; i < 4; i++)
        {
            anEdges[i] = TestShapes.Edge(aP[i], aP[(i + 1) % 4].Translated(new gp_Vec(1e-5, 0, 0)));
        }
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anEdges));
        var aSnap = ShapeSnapshot.Of(aW);
        TopoDS_Vertex aCallersV = TopExp.LastVertex(anEdges[0], true);

        var aContext = new ShapeBuild_ReShape();
        var sfw = new ShapeFix_Wire();
        sfw.Load(aW);
        sfw.SetContext(aContext); // after the load, as ShapeFix_Shape does
        sfw.SetPrecision(1e-4);
        Assert.True(sfw.FixConnected());
        Assert.True(sfw.StatusConnected(ShapeExtend_DONE));
        TopoDS_Wire aRes = sfw.Wire();

        aSnap.AssertUnchanged(aW);
        List<TopoDS_Edge> aResEdges = Edges(aRes);
        for (int i = 0; i < 4; i++)
        {
            Assert.True(Shares(TopExp.LastVertex(aResEdges[i], true), TopExp.FirstVertex(aResEdges[(i + 1) % 4], true)));
        }
        // the caller's vertex was replaced by a merged one, which is in the result
        TopoDS_Shape aNewV = aContext.Value(aCallersV);
        Assert.False(Shares(aNewV, aCallersV));
        Assert.Contains(aResEdges, e => Shares(TopExp.FirstVertex(e, true), aNewV) || Shares(TopExp.LastVertex(e, true), aNewV));
    }

    [Fact]
    public void FixEdgeCurves_AddsThePCurvesOnACylinder_AndTheSeam()
    {
        const double R = 5.0, H = 10.0;
        TopoDS_Face aF = TestShapes.Frozen(TestShapes.CylinderFace(R, H));
        // the cylinder's boundary rebuilt from 3d curves only
        var aV0 = TestShapes.Vertex(new gp_Pnt(R, 0, 0));
        var aV1 = TestShapes.Vertex(new gp_Pnt(R, 0, H));
        var aCirc0 = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), R);
        var aCirc1 = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, H), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), R);
        TopoDS_Edge aBottom = TestShapes.Edge(aCirc0, 0, 2 * Math.PI, aV0, aV0);
        TopoDS_Edge aTop = TestShapes.Edge(aCirc1, 0, 2 * Math.PI, aV1, aV1);
        TopoDS_Edge aSeam = TestShapes.Edge(new Geom_Line(new gp_Pnt(R, 0, 0), new gp_Dir(0, 0, 1)), 0, H, aV0, aV1);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(aBottom, aSeam, TopoDS.Edge(aTop.Reversed()), TopoDS.Edge(aSeam.Reversed())));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, aF, Conf);
        Assert.True(sfw.FixEdgeCurves());
        Assert.True(sfw.StatusEdgeCurves(ShapeExtend_DONE));
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        foreach (TopoDS_Edge anE in Edges(aRes))
        {
            Assert.NotNull(BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
        }
        TopoDS_Edge aResSeam = Edges(aRes)[1];
        Assert.True(BRep_Tool.IsClosed(aResSeam, aF));
        // the caller's edges have no pcurve still
        Assert.Null(BRep_Tool.CurveOnSurface(aSeam, aF, out _, out _));
    }

    [Fact]
    public void FixDegenerated_AddsTheApexEdgeOfACone()
    {
        TopoDS_Shape aCone = new BRepPrimAPI_MakeCone(0.0, 5.0, 10.0).Shape();
        TopoDS_Face aLateral = new();
        for (var anExp = new TopExp_Explorer(aCone, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_ConicalSurface)
            {
                aLateral = TopoDS.Face(anExp.Current());
            }
        }
        Assert.False(aLateral.IsNull());
        // the lateral face's wire without its degenerated edge
        var aKept = new List<TopoDS_Edge>();
        int aNbDegenerated = 0;
        for (var anIt = new BRepTools_WireExplorer(ShapeAnalysis.OuterWire(aLateral), aLateral); anIt.More(); anIt.Next())
        {
            if (BRep_Tool.Degenerated(anIt.Current()))
            {
                aNbDegenerated++;
            }
            else
            {
                aKept.Add(anIt.Current());
            }
        }
        Assert.Equal(1, aNbDegenerated);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire([.. aKept]));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, aLateral, Conf);
        Assert.True(sfw.FixDegenerated());
        Assert.True(sfw.StatusDegenerated(ShapeExtend_DONE));
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(aKept.Count + 1, Edges(aRes).Count);
        Assert.Contains(Edges(aRes), e => BRep_Tool.Degenerated(e));
    }

    [Fact]
    public void FixSelfIntersection_SplitsABowTie_WithAContext()
    {
        var aV = new[]
        {
            TestShapes.Vertex(new gp_Pnt(0, 0, 0)), TestShapes.Vertex(new gp_Pnt(2, 2, 0)),
            TestShapes.Vertex(new gp_Pnt(2, 0, 0)), TestShapes.Vertex(new gp_Pnt(0, 2, 0)),
        };
        var anEdges = new TopoDS_Edge[4];
        for (int i = 0; i < 4; i++)
        {
            anEdges[i] = TestShapes.Edge(aV[i], aV[(i + 1) % 4]);
        }
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anEdges));
        var aSnap = ShapeSnapshot.Of(aW);

        var aContext = new ShapeBuild_ReShape();
        var sfw = new ShapeFix_Wire(aW, PlaneFace(), Conf);
        sfw.SetContext(aContext);
        Assert.True(sfw.FixSelfIntersection());
        Assert.True(sfw.StatusSelfIntersection(ShapeExtend_DONE5));
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(6, Edges(aRes).Count);
        // the two crossing edges are recorded as split
        Assert.True(aContext.IsRecorded(anEdges[0]));
        Assert.True(aContext.IsRecorded(anEdges[2]));
    }

    [Fact]
    public void FixLacking_AddsAnEdgeAcrossA2dGapInATolerantVertex()
    {
        // the vertex at (10, 0) has a tolerance of 0.1; the second edge starts 0.05 away from it
        var aV0 = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        var aV2 = TestShapes.Vertex(new gp_Pnt(10, 10, 0));
        var aV3 = TestShapes.Vertex(new gp_Pnt(0, 10, 0));
        var aB = new BRep_Builder();
        TopoDS_Vertex aV1 = new();
        aB.MakeVertex(ref aV1, new gp_Pnt(10, 0, 0), 0.1);
        TopoDS_Edge aE0 = TestShapes.Edge(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 10, aV0, aV1);
        TopoDS_Edge aE1 = TestShapes.Edge(new Geom_Line(new gp_Pnt(10, 0.05, 0), new gp_Dir(0, 1, 0)), 0, 9.95, aV1, aV2);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(aE0, aE1, TestShapes.Edge(aV2, aV3), TestShapes.Edge(aV3, aV0)));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, PlaneFace(), Conf);
        sfw.ModifyTopologyMode() = true;
        Assert.True(sfw.FixLacking(true));
        Assert.True(sfw.StatusLacking(ShapeExtend_DONE));
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(5, Edges(aRes).Count);
    }

    [Fact]
    public void FixGaps3d_MovesTheCurveEnds_OnCopiesOfTheCurves()
    {
        // edges sharing a tolerant vertex, their curves ending 1e-3 apart
        var aB = new BRep_Builder();
        TopoDS_Vertex aV1 = new();
        aB.MakeVertex(ref aV1, new gp_Pnt(10, 0, 0), 1e-2);
        var aV0 = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        var aV2 = TestShapes.Vertex(new gp_Pnt(10, 10, 0));
        var aL0 = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        var aL1 = new Geom_Line(new gp_Pnt(10, 1e-3, 0), new gp_Dir(0, 1, 0));
        TopoDS_Edge aE0 = TestShapes.Edge(aL0, 0, 10, aV0, aV1);
        TopoDS_Edge aE1 = TestShapes.Edge(aL1, 0, 10 - 1e-3, aV1, aV2);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(aE0, aE1, TestShapes.Edge(aV2, aV0)));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, new TopoDS_Face(), 1e-5);
        Assert.True(sfw.FixGaps3d());
        Assert.True(sfw.StatusGaps3d(ShapeExtend_DONE));
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        List<TopoDS_Edge> aResEdges = Edges(aRes);
        Geom_Curve aC0 = BRep_Tool.Curve(aResEdges[0], out double f0, out double l0)!;
        Geom_Curve aC1 = BRep_Tool.Curve(aResEdges[1], out double f1, out _)!;
        Assert.True(aC0.Value(l0).Distance(aC1.Value(f1)) < 1e-5);
        // the caller's curves didn't move
        Assert.Equal(1e-3, aL1.Value(0).Y(), 15);
    }

    [Fact]
    public void FixGaps2d_ClosesAGapBetweenPCurves()
    {
        // a square on the plane whose pcurves are stored, one of them ending 1e-3 short of the next
        TopoDS_Edge[] anE = Square(out _);
        var aB = new BRep_Builder();
        TopoDS_Face aF = PlaneFace();
        for (int i = 0; i < 4; i++)
        {
            Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE[i], aF, out double f, out double l)!;
            if (i == 1)
            {
                aPC = new Geom2d_Line(new gp_Pnt2d(10, 1e-3), new gp_Dir2d(0, 1));
                aB.UpdateEdge(anE[i], aPC, aF, 1e-2);
                aB.Range(anE[i], aF, 0, 10 - 1e-3);
            }
            else
            {
                aB.UpdateEdge(anE[i], aPC, aF, Conf);
                aB.Range(anE[i], aF, f, l);
            }
        }
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anE));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new ShapeFix_Wire(aW, aF, 1e-5);
        Assert.True(sfw.FixGaps2d());
        Assert.True(sfw.StatusGaps2d(ShapeExtend_DONE));
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        List<TopoDS_Edge> aResEdges = Edges(aRes);
        Geom2d_Curve aP0 = BRep_Tool.CurveOnSurface(aResEdges[0], aF, out _, out double l0)!;
        Geom2d_Curve aP1 = BRep_Tool.CurveOnSurface(aResEdges[1], aF, out double f1, out _)!;
        Assert.True(aP0.Value(l0).Distance(aP1.Value(f1)) < 1e-5);
    }

    [Fact]
    public void FixShifted_BringsAPCurveBackByAPeriod()
    {
        const double R = 5.0, H = 10.0;
        TopoDS_Face aF = TestShapes.CylinderFace(R, H);
        TopoDS_Wire anOuter = ShapeAnalysis.OuterWire(aF);
        // the wire's edges copied, the pcurve of the top circle moved by 2 pi
        var aEdges = new List<TopoDS_Edge>();
        var aB = new BRep_Builder();
        foreach (TopoDS_Edge anE in Edges(anOuter))
        {
            aEdges.Add(anE);
        }
        TopoDS_Wire aW = MakeWire([.. aEdges]);
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aW, out _);
        List<TopoDS_Edge> aCopied = Edges(aCopy);
        int aShifted = -1;
        for (int i = 0; i < aCopied.Count; i++)
        {
            TopoDS_Edge anE = aCopied[i];
            if (!BRep_Tool.IsClosed(anE, aF) && !BRep_Tool.Degenerated(anE))
            {
                Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE, aF, out double f, out double l)!;
                var aMoved = (Geom2d_Curve)aPC.Translated(new gp_Vec2d(2 * Math.PI, 0));
                aB.UpdateEdge(anE, aMoved, aF, BRep_Tool.Tolerance(anE));
                aB.Range(anE, aF, f, l);
                aShifted = i;
                break;
            }
        }
        Assert.True(aShifted >= 0);
        TopoDS_Wire aShiftedWire = TestShapes.Frozen(TopoDS.Wire(aCopy));
        var aSnap = ShapeSnapshot.Of(aShiftedWire);

        var sfw = new ShapeFix_Wire(aShiftedWire, TestShapes.Frozen(aF), Conf);
        Assert.True(sfw.FixShifted());
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aShiftedWire);
        Geom2d_Curve aResPC = BRep_Tool.CurveOnSurface(Edges(aRes)[aShifted], aF, out double rf, out double rl)!;
        double aU = aResPC.Value((rf + rl) / 2).X();
        Assert.InRange(aU, -1e-9, 2 * Math.PI + 1e-9);
    }

    [Fact]
    public void FixTails_OnTheHealGroupsTriangle_RemovesTheTail()
    {
        // tests/heal/wire_tails_composed/A1: the polygon (-1,0,0) (9,0,0) (1,1,0) back to (-1,0,0) on its plane,
        // fixshape -maxtaila 10 -maxtailw 0.7: 3 vertices, 3 edges, length 9.054743851305652
        var aV = new[]
        {
            TestShapes.Vertex(new gp_Pnt(-1, 0, 0)), TestShapes.Vertex(new gp_Pnt(9, 0, 0)), TestShapes.Vertex(new gp_Pnt(1, 1, 0)),
        };
        var anEdges = new TopoDS_Edge[3];
        for (int i = 0; i < 3; i++)
        {
            anEdges[i] = TestShapes.Edge(aV[i], aV[(i + 1) % 3]);
        }
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anEdges));
        var aSnap = ShapeSnapshot.Of(aW);
        var sfw = new ShapeFix_Wire(aW, PlaneFace(), Conf);
        sfw.SetMaxTailAngle(10 * Math.PI / 180);
        sfw.SetMaxTailWidth(0.7);
        sfw.FixTailMode() = 1;
        sfw.ModifyTopologyMode() = true;
        sfw.Perform();
        TopoDS_Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        double aLength = 0;
        foreach (TopoDS_Edge anE in Edges(aRes))
        {
            var aProps = new GProp_GProps();
            BRepGProp.LinearProperties(anE, aProps);
            aLength += aProps.Mass();
        }
        Assert.Equal(3, Edges(aRes).Count);
        Assert.Equal(9.054743851305652, aLength, 9);
    }

    [Fact]
    public void Perform_PublishedThenFixedAgain_CopiesAgain()
    {
        TopoDS_Edge[] anE = Square(out _);
        TopoDS_Wire aW = TestShapes.Frozen(MakeWire(anE[1], anE[0], anE[2], anE[3]));
        var sfw = new ShapeFix_Wire(aW, PlaneFace(), Conf);
        Assert.True(sfw.Perform());
        TopoDS_Wire aFirst = sfw.Wire();
        Assert.True(aFirst.TShape()!.IsFrozen());
        // the published edges are frozen: a further fix works on copies again, and the first result never changes
        var aSnap = ShapeSnapshot.Of(aFirst);
        sfw.FixConnected(1e-3);
        sfw.FixEdgeCurves();
        TopoDS_Wire aSecond = sfw.Wire();
        aSnap.AssertUnchanged(aFirst);
        Assert.Equal(4, Edges(aSecond).Count);
    }

    [Fact]
    public void Perform_OnUnfrozenEdges_WorksInPlace()
    {
        TopoDS_Edge[] anE = Square(out _);
        TopoDS_Wire aW = MakeWire(anE[1], anE[0], anE[2], anE[3]);
        var sfw = new ShapeFix_Wire(aW, PlaneFace(), Conf);
        Assert.True(sfw.Perform());
        TopoDS_Wire aRes = sfw.InternalWire();
        Assert.False(aRes.TShape()!.IsFrozen());
        Assert.All(Edges(aRes), e => Assert.Contains(anE, c => Shares(c, e)));
    }

    [Fact]
    public void LoadWireData_WithFrozenEdges_IsCopiedAtTheFirstFix()
    {
        TopoDS_Edge[] anE = Square(out _);
        foreach (TopoDS_Edge e in anE)
        {
            TestShapes.Frozen(e);
        }
        var sbwd = new ShapeExtend_WireData();
        sbwd.Add(anE[0]);
        sbwd.Add(anE[2]);
        sbwd.Add(anE[1]);
        sbwd.Add(anE[3]);
        var sfw = new ShapeFix_Wire();
        sfw.Load(sbwd);
        sfw.SetFace(PlaneFace());
        Assert.True(sfw.Perform());
        // between the first fix and the result the wire data holds copies
        Assert.False(sbwd.Edge(1).TShape()!.IsFrozen());
        TopoDS_Wire aRes = sfw.Wire();
        Assert.All(Edges(aRes), e => Assert.Contains(anE, c => Shares(c, e)));
    }
}
