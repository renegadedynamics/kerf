// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeAnalysis_Wire's checks on wires and faces built with the makers and BRep_Builder, with the answers OCCT's
/// algorithms give (worked through by hand from the C++ where the order matters).
/// </summary>
public class ShapeAnalysis_WireTests
{
    private const double Prec = 1e-7;

    private static readonly Point3D A = new(0, 0, 0);
    private static readonly Point3D B = new(1, 0, 0);
    private static readonly Point3D C = new(1, 1, 0);
    private static readonly Point3D D = new(0, 1, 0);

    private static Edge Edge(in Point3D P1, in Point3D P2) => new MakeEdge(P1, P2).Edge();

    private static Edge Edge(in Vertex V1, in Vertex V2) => new MakeEdge(V1, V2).Edge();

    private static Vertex Vertex(double X, double Y) => new MakeVertex(new Point3D(X, Y, 0)).Vertex();

    /// <summary>The plane z = 0 as an infinite face (UV = XY).</summary>
    private static Face PlaneFace() => new MakeFace(new Plane3D()).Face();

    /// <summary>The unit square ABCD as four edges sharing their vertices, counter-clockwise.</summary>
    private static Edge[] SquareEdges()
    {
        Vertex[] aV = [Vertex(0, 0), Vertex(1, 0), Vertex(1, 1), Vertex(0, 1)];
        return [Edge(aV[0], aV[1]), Edge(aV[1], aV[2]), Edge(aV[2], aV[3]), Edge(aV[3], aV[0])];
    }

    private static ExtendWireData WireData(params Edge[] theEdges)
    {
        var aWD = new ExtendWireData();
        foreach (Edge anE in theEdges)
        {
            aWD.Add(anE);
        }

        return aWD;
    }

    private static Wire Polygon(params Point3D[] thePoints)
    {
        var aMaker = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aMaker.Add(aP);
        }

        aMaker.Close();
        return aMaker.Wire();
    }

    private static int[] Order(WireOrderAnalysis theOrder, int theNb)
    {
        var anOrder = new int[theNb];
        for (int i = 1; i <= theNb; i++)
        {
            anOrder[i - 1] = theOrder.Ordered(i);
        }

        return anOrder;
    }

    // ---------------------------------------------------------------- CheckOrder

    [Fact]
    public void CheckOrder_OrderedSquare_IsOk()
    {
        Edge[] anE = SquareEdges();
        var aSAW = new WireAnalysis(WireData(anE), PlaneFace(), Prec);
        var anOrder = new WireOrderAnalysis();
        Assert.False(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.OK));
        Assert.Equal([1, 2, 3, 4], Order(anOrder, 4));

        Assert.False(aSAW.CheckOrder());
        Assert.True(aSAW.StatusOrder(ExtendStatus.OK));
    }

    [Fact]
    public void CheckOrder_ShuffledSquare_GivesOcctOrder()
    {
        Edge[] anE = SquareEdges();
        // AB, CD, BC, DA: the chain from AB takes BC (3), then CD (2), then DA (4)
        var aSAW = new WireAnalysis(WireData(anE[0], anE[2], anE[1], anE[3]), PlaneFace(), Prec);
        var anOrder = new WireOrderAnalysis();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));
        Assert.False(aSAW.LastCheckStatus(ExtendStatus.DONE3));
        Assert.Equal([1, 3, 2, 4], Order(anOrder, 4));

        // BC, AB, CD, DA: AB is prepended, then CD and DA follow BC
        aSAW = new WireAnalysis(WireData(anE[1], anE[0], anE[2], anE[3]), PlaneFace(), Prec);
        anOrder = new WireOrderAnalysis();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));
        Assert.Equal([1, 3, 4, 2], Order(anOrder, 4));
    }

    [Fact]
    public void CheckOrder_ReversedEdge_IsDone3()
    {
        Edge[] anE = SquareEdges();
        // AB, BC, CD, then DA reversed (A to D): it must be reversed at the end
        var aSAW = new WireAnalysis(WireData(anE[0], anE[1], anE[2], ShapeCasts.Edge(anE[3].Reversed())), PlaneFace(), Prec);
        var anOrder = new WireOrderAnalysis();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE3));
        Assert.Equal([1, 2, 3, -4], Order(anOrder, 4));

        // the same in 2D, on the pcurves
        anOrder = new WireOrderAnalysis();
        Assert.True(aSAW.CheckOrder(anOrder, true, false));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE3));
        Assert.Equal([1, 2, 3, -4], Order(anOrder, 4));
    }

    [Fact]
    public void CheckOrder_ShuffledAndReversed_IsDone3()
    {
        Edge[] anE = SquareEdges();
        // CB (BC reversed), DA, AB, CD
        var aSAW = new WireAnalysis(WireData(ShapeCasts.Edge(anE[1].Reversed()), anE[3], anE[0], anE[2]), PlaneFace(), Prec);
        var anOrder = new WireOrderAnalysis();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE3));
        // every edge is used once, and the chain closes
        int[] anOrd = Order(anOrder, 4);
        Assert.Equal([1, 2, 3, 4], anOrd.Select(Math.Abs).Order());
    }

    [Fact]
    public void CheckOrder_2dMode_WithoutFace_Fails()
    {
        var aSAW = new WireAnalysis();
        aSAW.Load(WireData(SquareEdges()));
        Assert.False(aSAW.CheckOrder(new WireOrderAnalysis(), true, false));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.FAIL2));
    }

    [Fact]
    public void CheckOrder_FaceWithHoles_2dAsBRepMesh()
    {
        Face aFace = FaceWithHoles(out Wire anOuter, out Wire aHole1, out Wire aHole2);
        foreach (Wire aW in new[] { anOuter, aHole1, aHole2 })
        {
            // as BRepMesh_ShapeVisitor: non-manifold mode, 2D
            var aWD = new ExtendWireData(aW, true, false);
            var aSAW = new WireAnalysis(aWD, aFace, global::Kerf.Engine.Foundation.Precision.Confusion());
            var anOrder = new WireOrderAnalysis();
            aSAW.CheckOrder(anOrder, true, false);
            Assert.False(aSAW.LastCheckStatus(ExtendStatus.Fail));
            Assert.False(aSAW.LastCheckStatus(ExtendStatus.DONE3));
            Assert.Equal(aWD.EdgeCount, anOrder.EdgeCount);
        }
    }

    [Fact]
    public void CheckOrder_SeamAndDegeneratedFaces_2dAsBRepMesh()
    {
        // The primitives' lateral wires are top (reversed), seam at u = 2pi, bottom, seam at u = 0 (reversed): chained
        // in 3D through the seam's vertices, but in 2D the chain from the top edge goes to the seam at u = 0 first. So
        // OCCT's order is 1, 4, 3, 2: the edges reversed in sequence, only shifted (status 3, DONE5), nothing to reverse.
        Face[] aFaces =
        [
            new ApiMakeCylinder(2, 5).Face(),
            new MakeSphere(3).Face(),
            new ApiMakeCone(2, 0, 4).Face(),
        ];
        foreach (Face aFace in aFaces)
        {
            Wire aW = ShapeAnalysisAnalysis.OuterWire(aFace);
            var aWD = new ExtendWireData(aW, true, false);
            var aSAW = new WireAnalysis(aWD, aFace, global::Kerf.Engine.Foundation.Precision.Confusion());
            var anOrder = new WireOrderAnalysis();
            Assert.True(aSAW.CheckOrder(anOrder, true, false));
            Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE5));
            Assert.False(aSAW.LastCheckStatus(ExtendStatus.DONE3));
            Assert.False(aSAW.LastCheckStatus(ExtendStatus.Fail));
            Assert.Equal(aWD.EdgeCount, anOrder.EdgeCount);
            Assert.Equal([1, 4, 3, 2], Order(anOrder, 4));

            // in 3D too: the top edge is closed, and the seam leaving its vertex (edge 4) is preferred to the one
            // arriving there (edge 2)
            anOrder = new WireOrderAnalysis();
            Assert.True(aSAW.CheckOrder(anOrder, true, true));
            Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE5));
            Assert.Equal([1, 4, 3, 2], Order(anOrder, 4));
        }
    }

    // ---------------------------------------------------------------- connection and small edges

    [Fact]
    public void CheckConnected_SharedAndCoincidentVertices()
    {
        Face aFace = PlaneFace();
        // shared vertex: OK
        Edge[] anE = SquareEdges();
        var aSAW = new WireAnalysis(WireData(anE), aFace, Prec);
        Assert.False(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.OK));
        Assert.False(aSAW.CheckConnected());

        // distinct vertices at the same point: DONE1
        aSAW = new WireAnalysis(WireData(Edge(A, B), Edge(B, C)), aFace, Prec);
        Assert.True(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));
        Assert.Equal(0.0, aSAW.MinDistance3d());

        // a gap under the precision: DONE2
        aSAW = new WireAnalysis(WireData(Edge(A, B), Edge(new Point3D(1, 1e-5, 0), C)), aFace, 1e-3);
        Assert.True(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE2));
        Assert.Equal(1e-5, aSAW.MinDistance3d(), 1e-15);

        // a gap under prec only: DONE3
        aSAW = new WireAnalysis(WireData(Edge(A, B), Edge(new Point3D(1, 0.1, 0), C)), aFace, 1e-3);
        Assert.True(aSAW.CheckConnected(2, 0.5));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE3));

        // too far: FAIL1
        Assert.False(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.FAIL1));

        // the second edge the wrong way round: FAIL2
        aSAW = new WireAnalysis(WireData(Edge(A, B), Edge(C, B)), aFace, 1e-3);
        Assert.False(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.FAIL2));
    }

    [Fact]
    public void CheckSmall_DetectsEdgesUnderTheGivenPrecision()
    {
        var aSAW = new WireAnalysis(WireData(Edge(A, B), Edge(B, new Point3D(5, 0, 0))), PlaneFace(), Prec);
        Assert.False(aSAW.CheckSmall(1, 0.5));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.OK));
        Assert.True(aSAW.CheckSmall(1, 2.0));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE2));
        Assert.False(aSAW.CheckSmall(0.5));
        Assert.True(aSAW.CheckSmall(2.0));
        Assert.True(aSAW.StatusSmall(ExtendStatus.DONE2));
    }

    [Fact]
    public void CheckClosed_OpenAndClosedWires()
    {
        Face aFace = PlaneFace();
        var aSAW = new WireAnalysis(WireData(SquareEdges()), aFace, Prec);
        Assert.False(aSAW.CheckClosed());
        Assert.True(aSAW.StatusClosed(ExtendStatus.OK));

        Edge[] anE = SquareEdges();
        aSAW = new WireAnalysis(WireData(anE[0], anE[1], anE[2]), aFace, Prec);
        Assert.False(aSAW.CheckClosed());
        Assert.True(aSAW.StatusClosed(ExtendStatus.FAIL1));
    }

    // ---------------------------------------------------------------- whole checks on good faces

    [Fact]
    public void Perform_OnAGoodPolygonFace_FindsNothing()
    {
        Wire aW = Polygon(new Point3D(0, 0, 0), new Point3D(4, 0, 0), new Point3D(4, 3, 0), new Point3D(0, 3, 0));
        Face aFace = new MakeFace(aW).Face();
        var aSAW = new WireAnalysis(aW, aFace, Prec);
        Assert.True(aSAW.IsLoaded);
        Assert.True(aSAW.IsReady);
        Assert.Equal(4, aSAW.EdgeCount);
        Assert.False(aSAW.Perform());
        Assert.False(aSAW.CheckEdgeCurves());
        Assert.False(aSAW.CheckDegenerated());
        Assert.False(aSAW.CheckLacking());
        Assert.False(aSAW.CheckGaps3d());
        Assert.True(aSAW.MaxDistance3d() < Prec);
        Assert.False(aSAW.CheckGaps2d());
        Assert.False(aSAW.CheckCurveGaps());
        Assert.True(aSAW.MaxDistance3d() < Prec);
        Assert.False(aSAW.CheckOuterBound());
        Assert.False(aSAW.CheckSmallArea(aW));
        for (int i = 1; i <= 4; i++)
        {
            int aShort = 0;
            double aParam = 0;
            Assert.False(aSAW.CheckNotchedEdges(i, ref aShort, ref aParam, 0.01));
            Assert.Equal(0, aShort);
        }
    }

    [Fact]
    public void Perform_OnPrimitiveFaces_FindsNothingWrong()
    {
        Face[] aFaces =
        [
            new ApiMakeCylinder(2, 5).Face(),
            new MakeSphere(3).Face(),
        ];
        foreach (Face aFace in aFaces)
        {
            var aSAW = new WireAnalysis(ShapeAnalysisAnalysis.OuterWire(aFace), aFace, Prec);
            Assert.False(aSAW.CheckDegenerated());
            Assert.False(aSAW.CheckSelfIntersection());
            for (int i = 1; i <= aSAW.EdgeCount; i++)
            {
                Assert.False(aSAW.CheckSeam(i));
            }
        }
    }

    // ---------------------------------------------------------------- self-intersection

    [Fact]
    public void CheckSelfIntersection_FigureEight()
    {
        // (0,0) - (2,2) - (2,0) - (0,2): the first and third edges cross at (1,1)
        Wire aW = Polygon(new Point3D(0, 0, 0), new Point3D(2, 2, 0), new Point3D(2, 0, 0), new Point3D(0, 2, 0));
        var aSAW = new WireAnalysis(aW, PlaneFace(), Prec);
        Assert.True(aSAW.CheckSelfIntersection());
        Assert.True(aSAW.StatusSelfIntersection(ExtendStatus.DONE3));
        Assert.False(aSAW.StatusSelfIntersection(ExtendStatus.DONE1));
        Assert.False(aSAW.StatusSelfIntersection(ExtendStatus.DONE2));

        var aPnts2d = new NCollection_Sequence<IntersectionPoint2D>();
        var aPnts3d = new NCollection_Sequence<Point3D>();
        var anErrors = new NCollection_Sequence<double>();
        Assert.True(aSAW.CheckIntersectingEdges(1, 3, aPnts2d, aPnts3d, anErrors));
        Assert.Equal(1, aPnts3d.Length());
        Assert.True(aPnts3d.Value(1).Distance(new Point3D(1, 1, 0)) < 1e-9);
        Assert.True(anErrors.Value(1) < 1e-9);
        // adjacent edges meet at their common vertex only
        Assert.False(aSAW.CheckIntersectingEdges(2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.OK));
        Assert.False(aSAW.CheckSelfIntersectingEdge(1));
    }

    // ---------------------------------------------------------------- lacking edges

    [Fact]
    public void CheckLacking_GapInsideTheVertexTolerance()
    {
        var aB = new BrepBuilder();
        Vertex aV0 = new(), aV = new(), aV2 = new();
        aB.MakeVertex(ref aV0, new Point3D(0, 0, 0), Prec);
        aB.MakeVertex(ref aV, new Point3D(1.1, 0, 0), 0.3);
        aB.MakeVertex(ref aV2, new Point3D(1.2, 1, 0), Prec);
        Edge anE1 = new(), anE2 = new();
        aB.MakeEdge(ref anE1, new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), Prec);
        aB.Add(anE1, aV0.Oriented(Forward));
        aB.Add(anE1, aV.Oriented(Reversed));
        aB.Range(anE1, 0, 1);
        aB.MakeEdge(ref anE2, new LineCurve3D(new Point3D(1.2, 0, 0), new Direction3D(0, 1, 0)), Prec);
        aB.Add(anE2, aV.Oriented(Forward));
        aB.Add(anE2, aV2.Oriented(Reversed));
        aB.Range(anE2, 0, 1);

        var aSAW = new WireAnalysis(WireData(anE1, anE2), PlaneFace(), Prec);
        Point2D aP1 = new(), aP2 = new();
        Assert.True(aSAW.CheckLacking(2, 0.05, ref aP1, ref aP2));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));
        Assert.False(aSAW.LastCheckStatus(ExtendStatus.DONE2));
        Assert.True(aP1.Distance(new Point2D(1, 0)) < 1e-12);
        Assert.True(aP2.Distance(new Point2D(1.2, 0)) < 1e-12);
        Assert.Equal(0.2, aSAW.MaxDistance2d(), 1e-12);
        // the vertex tolerance (0.3) covers it
        Assert.False(aSAW.CheckLacking(2));
    }

    // ---------------------------------------------------------------- notches, tails, loops, area

    [Fact]
    public void CheckNotchedEdges_EdgeGoingBack()
    {
        Vertex aVA = Vertex(0, 0), aVB = Vertex(5, 0), aVC = Vertex(2, 0);
        var aSAW = new WireAnalysis(WireData(Edge(aVA, aVB), Edge(aVB, aVC)), PlaneFace(), Prec);
        int aShort = 0;
        double aParam = 0;
        Assert.True(aSAW.CheckNotchedEdges(2, ref aShort, ref aParam, 0.01));
        Assert.Equal(2, aShort);
        Assert.Equal(2.0, aParam, 1e-9);
    }

    [Fact]
    public void CheckTail_ThinSpikeOfTwoWholeEdges()
    {
        Vertex aVA = Vertex(0, 0), aVB = Vertex(10, 0), aVC = Vertex(0, 1e-4);
        Edge anE1 = Edge(aVA, aVB), anE2 = Edge(aVB, aVC);
        var aSAW = new WireAnalysis(WireData(anE1, anE2), PlaneFace(), Prec);
        Edge anE11 = new(), anE12 = new(), anE21 = new(), anE22 = new();
        Assert.True(aSAW.CheckTail(anE1, anE2, 0.01, 0.001, 0.1, ref anE11, ref anE12, ref anE21, ref anE22));
        Assert.True(anE11.IsEqual(anE1));
        Assert.True(anE21.IsEqual(anE2));
        Assert.True(anE12.IsNull);
        Assert.True(anE22.IsNull);

        // a right angle is no tail
        anE11 = new Edge();
        Assert.False(aSAW.CheckTail(Edge(aVA, aVB), Edge(aVB, Vertex(10, 5)), 0.01, 0.001, 0.1, ref anE11, ref anE12, ref anE21, ref anE22));
        Assert.True(anE11.IsNull);
    }

    [Fact]
    public void CheckLoop_FindsTheVertexOfTwoTriangles()
    {
        Vertex aO = Vertex(0, 0), aA = Vertex(1, 0), aB = Vertex(1, 1), aC = Vertex(-1, 0), aD = Vertex(-1, -1);
        var aSAW = new WireAnalysis(WireData(Edge(aO, aA), Edge(aA, aB), Edge(aB, aO), Edge(aO, aC), Edge(aC, aD), Edge(aD, aO)),
                                          PlaneFace(), Prec);
        var aLoops = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        var aVE = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var aSmall = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aSeams = new NCollection_Map<Shape>(ShapeHasher.Instance);
        Assert.True(aSAW.CheckLoop(aLoops, aVE, aSmall, aSeams));
        Assert.True(aSAW.StatusLoop(ExtendStatus.DONE1));
        Assert.Equal(1, aLoops.Extent());
        Assert.True(aLoops.FindKey(1).IsSame(aO));
        Assert.Equal(5, aVE.Extent());
        Assert.Equal(4, aVE.Find(aO).Extent());
        Assert.Equal(Prec, aSAW.Precision());
    }

    [Fact]
    public void CheckSmallArea_Sliver()
    {
        Wire aSliver = Polygon(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(5, 1e-9, 0));
        var aSAW = new WireAnalysis(aSliver, PlaneFace(), Prec);
        Assert.True(aSAW.CheckSmallArea(aSliver));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));

        Wire aSquare = Polygon(A, B, C, D);
        aSAW = new WireAnalysis(aSquare, PlaneFace(), Prec);
        Assert.False(aSAW.CheckSmallArea(aSquare));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.OK));
    }

    [Fact]
    public void CheckShapeConnect_FollowingEdgeDirectAndReversed()
    {
        Edge[] anE = SquareEdges();
        var aSAW = new WireAnalysis(WireData(anE[0], anE[1]), PlaneFace(), Prec);
        // C to D follows the wire A-B-C directly
        Assert.True(aSAW.CheckShapeConnect(Edge(C, D)));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE5));
        Assert.Equal(0.0, aSAW.MinDistance3d());

        // D to C follows it reversed
        Assert.True(aSAW.CheckShapeConnect(out double aTH, out double aTT, out double aHT, out double aHH, Edge(D, C)));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE2));
        Assert.Equal(1.0, aTH);
        Assert.Equal(0.0, aTT);
        Assert.Equal(Math.Sqrt(2.0), aHT);
        Assert.Equal(1.0, aHH);

        // far away: FAIL2; not an edge or a wire: FAIL1
        Assert.False(aSAW.CheckShapeConnect(Edge(new Point3D(5, 5, 0), new Point3D(6, 5, 0))));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.FAIL2));
        Assert.False(aSAW.CheckShapeConnect(Vertex(5, 5)));
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.FAIL1));
    }

    [Fact]
    public void CheckOuterBound_HoleWire()
    {
        Face aFace = FaceWithHoles(out Wire anOuter, out Wire aHole1, out _);
        var aSAW = new WireAnalysis(anOuter, aFace, Prec);
        Assert.False(aSAW.CheckOuterBound());
        Assert.False(aSAW.CheckOuterBound(false));
        aSAW = new WireAnalysis(aHole1, aFace, Prec);
        Assert.True(aSAW.CheckOuterBound());
        Assert.True(aSAW.LastCheckStatus(ExtendStatus.DONE1));
    }

    [Fact]
    public void SetSurface_MakesAFrozenFace()
    {
        var aSAW = new WireAnalysis();
        Assert.False(aSAW.IsLoaded);
        Assert.Equal(0, aSAW.EdgeCount);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aSAW.Precision());
        aSAW.SetSurface(new PlaneSurface(new Plane3D()));
        Assert.False(aSAW.Face().IsNull);
        Assert.True(aSAW.Face().TShape()!.IsFrozen);
        Assert.NotNull(aSAW.Surface());
        aSAW.Load(WireData(SquareEdges()));
        Assert.True(aSAW.IsReady);
        aSAW.ClearStatuses();
        Assert.True(aSAW.StatusOrder(ExtendStatus.OK));
    }

    // ---------------------------------------------------------------- helpers

    /// <summary>
    /// A plane face (z = 0) with an outer square 0..10 and two square holes, its wires added hole, outer, hole, so that
    /// the outer wire isn't the first.
    /// </summary>
    internal static Face FaceWithHoles(out Wire theOuter, out Wire theHole1, out Wire theHole2)
    {
        theOuter = Polygon(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        theHole1 = ShapeCasts.Wire(Polygon(new Point3D(2, 2, 0), new Point3D(4, 2, 0), new Point3D(4, 4, 0), new Point3D(2, 4, 0)).Reversed());
        theHole2 = ShapeCasts.Wire(Polygon(new Point3D(6, 6, 0), new Point3D(8, 6, 0), new Point3D(8, 8, 0), new Point3D(6, 8, 0)).Reversed());
        var aB = new BrepBuilder();
        Face aFace = new();
        aB.MakeFace(ref aFace, new PlaneSurface(new Plane3D()), Prec);
        aB.Add(aFace, theHole1);
        aB.Add(aFace, theOuter);
        aB.Add(aFace, theHole2);
        aFace.TShape()!.FreezeGraph();
        return aFace;
    }
}
