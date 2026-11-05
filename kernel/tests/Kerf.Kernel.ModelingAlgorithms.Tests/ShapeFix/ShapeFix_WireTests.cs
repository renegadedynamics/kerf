// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Wire's fixes one by one (reorder, small, connected, edge curves, degenerated, self-intersection, lacking,
/// gaps in 3d and 2d, shifted, tails), each on a frozen wire: the caller's wire unchanged, the result frozen, the caller's
/// own edges and vertices kept where nothing changed, the context answering for the caller's shapes.
/// </summary>
public class ShapeFix_WireTests
{
    private const double Conf = 1e-7;

    private static PlaneSurface XOY() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static Face PlaneFace()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        return TestShapes.Frozen(aF);
    }

    private static Wire MakeWire(params Edge[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }
        return aW;
    }

    private static List<Edge> Edges(in Shape theShape)
    {
        var aList = new List<Edge>();
        for (var anIt = new Iterator(theShape); anIt.More(); anIt.Next())
        {
            aList.Add(ShapeCasts.Edge(anIt.Value()));
        }
        return aList;
    }

    private static bool Shares(in Shape theA, in Shape theB) => ReferenceEquals(theA.TShape(), theB.TShape());

    // the square [0, 10]^2 in XOY, its vertices shared
    private static Edge[] Square(out Vertex[] theVertices)
    {
        theVertices =
        [
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(10, 0, 0)),
            TestShapes.Vertex(new Point3D(10, 10, 0)), TestShapes.Vertex(new Point3D(0, 10, 0)),
        ];
        var anEdges = new Edge[4];
        for (int i = 0; i < 4; i++)
        {
            anEdges[i] = TestShapes.Edge(theVertices[i], theVertices[(i + 1) % 4]);
        }
        return anEdges;
    }

    private static void AssertConnected(in Wire theWire)
    {
        var sawo = new WireOrderAnalysis();
        var saw = new WireAnalysis(theWire, new Face(), Conf);
        saw.CheckOrder(sawo, true, true);
        Assert.False(saw.StatusOrder(ExtendStatus.Done));
        Assert.True(saw.CheckConnected() == false || !saw.StatusConnected(ExtendStatus.Fail));
    }

    [Fact]
    public void FixReorder_ShuffledSquare_KeepsTheCallersEdges()
    {
        Edge[] anE = Square(out _);
        Wire aW = TestShapes.Frozen(MakeWire(anE[2], anE[0], anE[3], anE[1]));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, PlaneFace(), Conf);
        Assert.True(sfw.FixReorder());
        Assert.True(sfw.StatusReorder(ExtendStatus.DONE1));
        Wire aRes = sfw.Wire();

        aSnap.AssertUnchanged(aW);
        Assert.True(aRes.TShape()!.IsFrozen);
        List<Edge> aResEdges = Edges(aRes);
        Assert.Equal(4, aResEdges.Count);
        // only the order changed: the edges are the caller's own
        Assert.All(aResEdges, e => Assert.Contains(anE, c => Shares(c, e)));
        for (int i = 0; i < 4; i++)
        {
            Assert.True(Shares(Exploration.LastVertex(aResEdges[i], true), Exploration.FirstVertex(aResEdges[(i + 1) % 4], true)));
        }
    }

    [Fact]
    public void FixSmall_RemovesATinyEdge()
    {
        Edge[] anE = Square(out Vertex[] aV);
        // a 1e-9 edge after the first corner, the next edge starting at its end
        Vertex aV1b = TestShapes.Vertex(new Point3D(10, 1e-9, 0));
        Edge aTiny = TestShapes.Edge(aV[1], aV1b);
        Edge aSide = TestShapes.Edge(aV1b, aV[2]);
        Wire aW = TestShapes.Frozen(MakeWire(anE[0], aTiny, aSide, anE[2], anE[3]));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, PlaneFace(), 1e-7);
        sfw.ModifyTopologyMode() = true;
        Assert.True(sfw.FixSmall(false, 1e-6) > 0);
        Assert.True(sfw.StatusSmall(ExtendStatus.Done));
        Assert.Equal(4, sfw.NbEdges());
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(4, Edges(aRes).Count);
        AssertConnected(aRes);
    }

    [Fact]
    public void FixConnected_MergesNearVertices_AndTheContextAnswersForTheCallersVertices()
    {
        // four segments, each with its own vertices, the ends 1e-5 apart
        var aP = new[] { new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0) };
        var anEdges = new Edge[4];
        for (int i = 0; i < 4; i++)
        {
            anEdges[i] = TestShapes.Edge(aP[i], aP[(i + 1) % 4].Translated(new Vector3D(1e-5, 0, 0)));
        }
        Wire aW = TestShapes.Frozen(MakeWire(anEdges));
        var aSnap = ShapeSnapshot.Of(aW);
        Vertex aCallersV = Exploration.LastVertex(anEdges[0], true);

        var aContext = new ReShapeBuilder();
        var sfw = new WireFixer();
        sfw.Load(aW);
        sfw.SetContext(aContext); // after the load, as ShapeFix_Shape does
        sfw.SetPrecision(1e-4);
        Assert.True(sfw.FixConnected());
        Assert.True(sfw.StatusConnected(ExtendStatus.Done));
        Wire aRes = sfw.Wire();

        aSnap.AssertUnchanged(aW);
        List<Edge> aResEdges = Edges(aRes);
        for (int i = 0; i < 4; i++)
        {
            Assert.True(Shares(Exploration.LastVertex(aResEdges[i], true), Exploration.FirstVertex(aResEdges[(i + 1) % 4], true)));
        }
        // the caller's vertex was replaced by a merged one, which is in the result
        Shape aNewV = aContext.Value(aCallersV);
        Assert.False(Shares(aNewV, aCallersV));
        Assert.Contains(aResEdges, e => Shares(Exploration.FirstVertex(e, true), aNewV) || Shares(Exploration.LastVertex(e, true), aNewV));
    }

    [Fact]
    public void FixEdgeCurves_AddsThePCurvesOnACylinder_AndTheSeam()
    {
        const double R = 5.0, H = 10.0;
        Face aF = TestShapes.Frozen(TestShapes.CylinderFace(R, H));
        // the cylinder's boundary rebuilt from 3d curves only
        var aV0 = TestShapes.Vertex(new Point3D(R, 0, 0));
        var aV1 = TestShapes.Vertex(new Point3D(R, 0, H));
        var aCirc0 = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), R);
        var aCirc1 = new CircleCurve3D(new Frame3D(new Point3D(0, 0, H), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), R);
        Edge aBottom = TestShapes.Edge(aCirc0, 0, 2 * Math.PI, aV0, aV0);
        Edge aTop = TestShapes.Edge(aCirc1, 0, 2 * Math.PI, aV1, aV1);
        Edge aSeam = TestShapes.Edge(new LineCurve3D(new Point3D(R, 0, 0), new Direction3D(0, 0, 1)), 0, H, aV0, aV1);
        Wire aW = TestShapes.Frozen(MakeWire(aBottom, aSeam, ShapeCasts.Edge(aTop.Reversed()), ShapeCasts.Edge(aSeam.Reversed())));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, aF, Conf);
        Assert.True(sfw.FixEdgeCurves());
        Assert.True(sfw.StatusEdgeCurves(ExtendStatus.Done));
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        foreach (Edge anE in Edges(aRes))
        {
            Assert.NotNull(Tool.CurveOnSurface(anE, aF, out _, out _));
        }
        Edge aResSeam = Edges(aRes)[1];
        Assert.True(Tool.IsClosed(aResSeam, aF));
        // the caller's edges have no pcurve still
        Assert.Null(Tool.CurveOnSurface(aSeam, aF, out _, out _));
    }

    [Fact]
    public void FixDegenerated_AddsTheApexEdgeOfACone()
    {
        Shape aCone = new ApiMakeCone(0.0, 5.0, 10.0).Shape();
        Face aLateral = new();
        for (var anExp = new Explorer(aCone, ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (Tool.Surface(ShapeCasts.Face(anExp.Current())) is ConicalSurface)
            {
                aLateral = ShapeCasts.Face(anExp.Current());
            }
        }
        Assert.False(aLateral.IsNull);
        // the lateral face's wire without its degenerated edge
        var aKept = new List<Edge>();
        int aNbDegenerated = 0;
        for (var anIt = new WireExplorer(ShapeAnalysisAnalysis.OuterWire(aLateral), aLateral); anIt.More(); anIt.Next())
        {
            if (Tool.Degenerated(anIt.Current()))
            {
                aNbDegenerated++;
            }
            else
            {
                aKept.Add(anIt.Current());
            }
        }
        Assert.Equal(1, aNbDegenerated);
        Wire aW = TestShapes.Frozen(MakeWire([.. aKept]));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, aLateral, Conf);
        Assert.True(sfw.FixDegenerated());
        Assert.True(sfw.StatusDegenerated(ExtendStatus.Done));
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(aKept.Count + 1, Edges(aRes).Count);
        Assert.Contains(Edges(aRes), e => Tool.Degenerated(e));
    }

    [Fact]
    public void FixSelfIntersection_SplitsABowTie_WithAContext()
    {
        var aV = new[]
        {
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(2, 2, 0)),
            TestShapes.Vertex(new Point3D(2, 0, 0)), TestShapes.Vertex(new Point3D(0, 2, 0)),
        };
        var anEdges = new Edge[4];
        for (int i = 0; i < 4; i++)
        {
            anEdges[i] = TestShapes.Edge(aV[i], aV[(i + 1) % 4]);
        }
        Wire aW = TestShapes.Frozen(MakeWire(anEdges));
        var aSnap = ShapeSnapshot.Of(aW);

        var aContext = new ReShapeBuilder();
        var sfw = new WireFixer(aW, PlaneFace(), Conf);
        sfw.SetContext(aContext);
        Assert.True(sfw.FixSelfIntersection());
        Assert.True(sfw.StatusSelfIntersection(ExtendStatus.DONE5));
        Wire aRes = sfw.Wire();
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
        var aV0 = TestShapes.Vertex(new Point3D(0, 0, 0));
        var aV2 = TestShapes.Vertex(new Point3D(10, 10, 0));
        var aV3 = TestShapes.Vertex(new Point3D(0, 10, 0));
        var aB = new BrepBuilder();
        Vertex aV1 = new();
        aB.MakeVertex(ref aV1, new Point3D(10, 0, 0), 0.1);
        Edge aE0 = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10, aV0, aV1);
        Edge aE1 = TestShapes.Edge(new LineCurve3D(new Point3D(10, 0.05, 0), new Direction3D(0, 1, 0)), 0, 9.95, aV1, aV2);
        Wire aW = TestShapes.Frozen(MakeWire(aE0, aE1, TestShapes.Edge(aV2, aV3), TestShapes.Edge(aV3, aV0)));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, PlaneFace(), Conf);
        sfw.ModifyTopologyMode() = true;
        Assert.True(sfw.FixLacking(true));
        Assert.True(sfw.StatusLacking(ExtendStatus.Done));
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        Assert.Equal(5, Edges(aRes).Count);
    }

    [Fact]
    public void FixGaps3d_MovesTheCurveEnds_OnCopiesOfTheCurves()
    {
        // edges sharing a tolerant vertex, their curves ending 1e-3 apart
        var aB = new BrepBuilder();
        Vertex aV1 = new();
        aB.MakeVertex(ref aV1, new Point3D(10, 0, 0), 1e-2);
        var aV0 = TestShapes.Vertex(new Point3D(0, 0, 0));
        var aV2 = TestShapes.Vertex(new Point3D(10, 10, 0));
        var aL0 = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var aL1 = new LineCurve3D(new Point3D(10, 1e-3, 0), new Direction3D(0, 1, 0));
        Edge aE0 = TestShapes.Edge(aL0, 0, 10, aV0, aV1);
        Edge aE1 = TestShapes.Edge(aL1, 0, 10 - 1e-3, aV1, aV2);
        Wire aW = TestShapes.Frozen(MakeWire(aE0, aE1, TestShapes.Edge(aV2, aV0)));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, new Face(), 1e-5);
        Assert.True(sfw.FixGaps3d());
        Assert.True(sfw.StatusGaps3d(ExtendStatus.Done));
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        List<Edge> aResEdges = Edges(aRes);
        Curve3D aC0 = Tool.Curve(aResEdges[0], out double f0, out double l0)!;
        Curve3D aC1 = Tool.Curve(aResEdges[1], out double f1, out _)!;
        Assert.True(aC0.Value(l0).DistanceTo(aC1.Value(f1)) < 1e-5);
        // the caller's curves didn't move
        Assert.Equal(1e-3, aL1.Value(0).Y, 15);
    }

    [Fact]
    public void FixGaps2d_ClosesAGapBetweenPCurves()
    {
        // a square on the plane whose pcurves are stored, one of them ending 1e-3 short of the next
        Edge[] anE = Square(out _);
        var aB = new BrepBuilder();
        Face aF = PlaneFace();
        for (int i = 0; i < 4; i++)
        {
            Curve2D aPC = Tool.CurveOnSurface(anE[i], aF, out double f, out double l)!;
            if (i == 1)
            {
                aPC = new LineCurve2D(new Point2D(10, 1e-3), new Direction2D(0, 1));
                aB.UpdateEdge(anE[i], aPC, aF, 1e-2);
                aB.Range(anE[i], aF, 0, 10 - 1e-3);
            }
            else
            {
                aB.UpdateEdge(anE[i], aPC, aF, Conf);
                aB.Range(anE[i], aF, f, l);
            }
        }
        Wire aW = TestShapes.Frozen(MakeWire(anE));
        var aSnap = ShapeSnapshot.Of(aW);

        var sfw = new WireFixer(aW, aF, 1e-5);
        Assert.True(sfw.FixGaps2d());
        Assert.True(sfw.StatusGaps2d(ExtendStatus.Done));
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        List<Edge> aResEdges = Edges(aRes);
        Curve2D aP0 = Tool.CurveOnSurface(aResEdges[0], aF, out _, out double l0)!;
        Curve2D aP1 = Tool.CurveOnSurface(aResEdges[1], aF, out double f1, out _)!;
        Assert.True(aP0.Value(l0).DistanceTo(aP1.Value(f1)) < 1e-5);
    }

    [Fact]
    public void FixShifted_BringsAPCurveBackByAPeriod()
    {
        const double R = 5.0, H = 10.0;
        Face aF = TestShapes.CylinderFace(R, H);
        Wire anOuter = ShapeAnalysisAnalysis.OuterWire(aF);
        // the wire's edges copied, the pcurve of the top circle moved by 2 pi
        var aEdges = new List<Edge>();
        var aB = new BrepBuilder();
        foreach (Edge anE in Edges(anOuter))
        {
            aEdges.Add(anE);
        }
        Wire aW = MakeWire([.. aEdges]);
        Shape aCopy = TestCopies.ExactCopyForWrite(aW, out _);
        List<Edge> aCopied = Edges(aCopy);
        int aShifted = -1;
        for (int i = 0; i < aCopied.Count; i++)
        {
            Edge anE = aCopied[i];
            if (!Tool.IsClosed(anE, aF) && !Tool.Degenerated(anE))
            {
                Curve2D aPC = Tool.CurveOnSurface(anE, aF, out double f, out double l)!;
                var aMoved = (Curve2D)aPC.Translated(new Vector2D(2 * Math.PI, 0));
                aB.UpdateEdge(anE, aMoved, aF, Tool.Tolerance(anE));
                aB.Range(anE, aF, f, l);
                aShifted = i;
                break;
            }
        }
        Assert.True(aShifted >= 0);
        Wire aShiftedWire = TestShapes.Frozen(ShapeCasts.Wire(aCopy));
        var aSnap = ShapeSnapshot.Of(aShiftedWire);

        var sfw = new WireFixer(aShiftedWire, TestShapes.Frozen(aF), Conf);
        Assert.True(sfw.FixShifted());
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aShiftedWire);
        Curve2D aResPC = Tool.CurveOnSurface(Edges(aRes)[aShifted], aF, out double rf, out double rl)!;
        double aU = aResPC.Value((rf + rl) / 2).X;
        Assert.InRange(aU, -1e-9, 2 * Math.PI + 1e-9);
    }

    [Fact]
    public void FixTails_OnTheHealGroupsTriangle_RemovesTheTail()
    {
        // tests/heal/wire_tails_composed/A1: the polygon (-1,0,0) (9,0,0) (1,1,0) back to (-1,0,0) on its plane,
        // fixshape -maxtaila 10 -maxtailw 0.7: 3 vertices, 3 edges, length 9.054743851305652
        var aV = new[]
        {
            TestShapes.Vertex(new Point3D(-1, 0, 0)), TestShapes.Vertex(new Point3D(9, 0, 0)), TestShapes.Vertex(new Point3D(1, 1, 0)),
        };
        var anEdges = new Edge[3];
        for (int i = 0; i < 3; i++)
        {
            anEdges[i] = TestShapes.Edge(aV[i], aV[(i + 1) % 3]);
        }
        Wire aW = TestShapes.Frozen(MakeWire(anEdges));
        var aSnap = ShapeSnapshot.Of(aW);
        var sfw = new WireFixer(aW, PlaneFace(), Conf);
        sfw.SetMaxTailAngle(10 * Math.PI / 180);
        sfw.SetMaxTailWidth(0.7);
        sfw.FixTailMode() = 1;
        sfw.ModifyTopologyMode() = true;
        sfw.Perform();
        Wire aRes = sfw.Wire();
        aSnap.AssertUnchanged(aW);
        double aLength = 0;
        foreach (Edge anE in Edges(aRes))
        {
            var aProps = new Properties();
            MassPropertiesBuilder.LinearProperties(anE, aProps);
            aLength += aProps.Mass();
        }
        Assert.Equal(3, Edges(aRes).Count);
        Assert.Equal(9.054743851305652, aLength, 9);
    }

    [Fact]
    public void Perform_PublishedThenFixedAgain_CopiesAgain()
    {
        Edge[] anE = Square(out _);
        Wire aW = TestShapes.Frozen(MakeWire(anE[1], anE[0], anE[2], anE[3]));
        var sfw = new WireFixer(aW, PlaneFace(), Conf);
        Assert.True(sfw.Perform());
        Wire aFirst = sfw.Wire();
        Assert.True(aFirst.TShape()!.IsFrozen);
        // the published edges are frozen: a further fix works on copies again, and the first result never changes
        var aSnap = ShapeSnapshot.Of(aFirst);
        sfw.FixConnected(1e-3);
        sfw.FixEdgeCurves();
        Wire aSecond = sfw.Wire();
        aSnap.AssertUnchanged(aFirst);
        Assert.Equal(4, Edges(aSecond).Count);
    }

    [Fact]
    public void Perform_OnUnfrozenEdges_WorksInPlace()
    {
        Edge[] anE = Square(out _);
        Wire aW = MakeWire(anE[1], anE[0], anE[2], anE[3]);
        var sfw = new WireFixer(aW, PlaneFace(), Conf);
        Assert.True(sfw.Perform());
        Wire aRes = sfw.InternalWire();
        Assert.False(aRes.TShape()!.IsFrozen);
        Assert.All(Edges(aRes), e => Assert.Contains(anE, c => Shares(c, e)));
    }

    [Fact]
    public void LoadWireData_WithFrozenEdges_IsFixedInAWorkingOverlay()
    {
        Edge[] anE = Square(out _);
        foreach (Edge e in anE)
        {
            TestShapes.Frozen(e);
        }
        var sbwd = new ExtendWireData();
        sbwd.Add(anE[0]);
        sbwd.Add(anE[2]);
        sbwd.Add(anE[1]);
        sbwd.Add(anE[3]);
        var sfw = new WireFixer();
        sfw.Load(sbwd);
        sfw.SetFace(PlaneFace());
        Assert.True(sfw.Perform());
        // between the first fix and the result the wire data holds the caller's edges, whose changes are in the
        // tool's working overlay (Kerf unit 10H4; a copy at the entry before)
        Assert.True(sbwd.Edge(1).TShape()!.IsFrozen);
        Wire aRes = sfw.Wire();
        Assert.All(Edges(aRes), e => Assert.Contains(anE, c => Shares(c, e)));
    }
}
