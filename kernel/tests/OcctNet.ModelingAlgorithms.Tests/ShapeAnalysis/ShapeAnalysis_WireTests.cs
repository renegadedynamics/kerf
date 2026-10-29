// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeAnalysis_Wire's checks on wires and faces built with the makers and BRep_Builder, with the answers OCCT's
/// algorithms give (worked through by hand from the C++ where the order matters).
/// </summary>
public class ShapeAnalysis_WireTests
{
    private const double Prec = 1e-7;

    private static readonly gp_Pnt A = new(0, 0, 0);
    private static readonly gp_Pnt B = new(1, 0, 0);
    private static readonly gp_Pnt C = new(1, 1, 0);
    private static readonly gp_Pnt D = new(0, 1, 0);

    private static TopoDS_Edge Edge(in gp_Pnt P1, in gp_Pnt P2) => new BRepBuilderAPI_MakeEdge(P1, P2).Edge();

    private static TopoDS_Edge Edge(in TopoDS_Vertex V1, in TopoDS_Vertex V2) => new BRepBuilderAPI_MakeEdge(V1, V2).Edge();

    private static TopoDS_Vertex Vertex(double X, double Y) => new BRepBuilderAPI_MakeVertex(new gp_Pnt(X, Y, 0)).Vertex();

    /// <summary>The plane z = 0 as an infinite face (UV = XY).</summary>
    private static TopoDS_Face PlaneFace() => new BRepBuilderAPI_MakeFace(new gp_Pln()).Face();

    /// <summary>The unit square ABCD as four edges sharing their vertices, counter-clockwise.</summary>
    private static TopoDS_Edge[] SquareEdges()
    {
        TopoDS_Vertex[] aV = [Vertex(0, 0), Vertex(1, 0), Vertex(1, 1), Vertex(0, 1)];
        return [Edge(aV[0], aV[1]), Edge(aV[1], aV[2]), Edge(aV[2], aV[3]), Edge(aV[3], aV[0])];
    }

    private static ShapeExtend_WireData WireData(params TopoDS_Edge[] theEdges)
    {
        var aWD = new ShapeExtend_WireData();
        foreach (TopoDS_Edge anE in theEdges)
        {
            aWD.Add(anE);
        }

        return aWD;
    }

    private static TopoDS_Wire Polygon(params gp_Pnt[] thePoints)
    {
        var aMaker = new BRepBuilderAPI_MakePolygon();
        foreach (gp_Pnt aP in thePoints)
        {
            aMaker.Add(aP);
        }

        aMaker.Close();
        return aMaker.Wire();
    }

    private static int[] Order(ShapeAnalysis_WireOrder theOrder, int theNb)
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
        TopoDS_Edge[] anE = SquareEdges();
        var aSAW = new ShapeAnalysis_Wire(WireData(anE), PlaneFace(), Prec);
        var anOrder = new ShapeAnalysis_WireOrder();
        Assert.False(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_OK));
        Assert.Equal([1, 2, 3, 4], Order(anOrder, 4));

        Assert.False(aSAW.CheckOrder());
        Assert.True(aSAW.StatusOrder(ShapeExtend_OK));
    }

    [Fact]
    public void CheckOrder_ShuffledSquare_GivesOcctOrder()
    {
        TopoDS_Edge[] anE = SquareEdges();
        // AB, CD, BC, DA: the chain from AB takes BC (3), then CD (2), then DA (4)
        var aSAW = new ShapeAnalysis_Wire(WireData(anE[0], anE[2], anE[1], anE[3]), PlaneFace(), Prec);
        var anOrder = new ShapeAnalysis_WireOrder();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));
        Assert.False(aSAW.LastCheckStatus(ShapeExtend_DONE3));
        Assert.Equal([1, 3, 2, 4], Order(anOrder, 4));

        // BC, AB, CD, DA: AB is prepended, then CD and DA follow BC
        aSAW = new ShapeAnalysis_Wire(WireData(anE[1], anE[0], anE[2], anE[3]), PlaneFace(), Prec);
        anOrder = new ShapeAnalysis_WireOrder();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));
        Assert.Equal([1, 3, 4, 2], Order(anOrder, 4));
    }

    [Fact]
    public void CheckOrder_ReversedEdge_IsDone3()
    {
        TopoDS_Edge[] anE = SquareEdges();
        // AB, BC, CD, then DA reversed (A to D): it must be reversed at the end
        var aSAW = new ShapeAnalysis_Wire(WireData(anE[0], anE[1], anE[2], TopoDS.Edge(anE[3].Reversed())), PlaneFace(), Prec);
        var anOrder = new ShapeAnalysis_WireOrder();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE3));
        Assert.Equal([1, 2, 3, -4], Order(anOrder, 4));

        // the same in 2D, on the pcurves
        anOrder = new ShapeAnalysis_WireOrder();
        Assert.True(aSAW.CheckOrder(anOrder, true, false));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE3));
        Assert.Equal([1, 2, 3, -4], Order(anOrder, 4));
    }

    [Fact]
    public void CheckOrder_ShuffledAndReversed_IsDone3()
    {
        TopoDS_Edge[] anE = SquareEdges();
        // CB (BC reversed), DA, AB, CD
        var aSAW = new ShapeAnalysis_Wire(WireData(TopoDS.Edge(anE[1].Reversed()), anE[3], anE[0], anE[2]), PlaneFace(), Prec);
        var anOrder = new ShapeAnalysis_WireOrder();
        Assert.True(aSAW.CheckOrder(anOrder, true, true));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE3));
        // every edge is used once, and the chain closes
        int[] anOrd = Order(anOrder, 4);
        Assert.Equal([1, 2, 3, 4], anOrd.Select(Math.Abs).Order());
    }

    [Fact]
    public void CheckOrder_2dMode_WithoutFace_Fails()
    {
        var aSAW = new ShapeAnalysis_Wire();
        aSAW.Load(WireData(SquareEdges()));
        Assert.False(aSAW.CheckOrder(new ShapeAnalysis_WireOrder(), true, false));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_FAIL2));
    }

    [Fact]
    public void CheckOrder_FaceWithHoles_2dAsBRepMesh()
    {
        TopoDS_Face aFace = FaceWithHoles(out TopoDS_Wire anOuter, out TopoDS_Wire aHole1, out TopoDS_Wire aHole2);
        foreach (TopoDS_Wire aW in new[] { anOuter, aHole1, aHole2 })
        {
            // as BRepMesh_ShapeVisitor: non-manifold mode, 2D
            var aWD = new ShapeExtend_WireData(aW, true, false);
            var aSAW = new ShapeAnalysis_Wire(aWD, aFace, Precision.Confusion());
            var anOrder = new ShapeAnalysis_WireOrder();
            aSAW.CheckOrder(anOrder, true, false);
            Assert.False(aSAW.LastCheckStatus(ShapeExtend_FAIL));
            Assert.False(aSAW.LastCheckStatus(ShapeExtend_DONE3));
            Assert.Equal(aWD.NbEdges(), anOrder.NbEdges());
        }
    }

    [Fact]
    public void CheckOrder_SeamAndDegeneratedFaces_2dAsBRepMesh()
    {
        // The primitives' lateral wires are top (reversed), seam at u = 2pi, bottom, seam at u = 0 (reversed): chained
        // in 3D through the seam's vertices, but in 2D the chain from the top edge goes to the seam at u = 0 first. So
        // OCCT's order is 1, 4, 3, 2: the edges reversed in sequence, only shifted (status 3, DONE5), nothing to reverse.
        TopoDS_Face[] aFaces =
        [
            new BRepPrimAPI_MakeCylinder(2, 5).Face(),
            new BRepPrimAPI_MakeSphere(3).Face(),
            new BRepPrimAPI_MakeCone(2, 0, 4).Face(),
        ];
        foreach (TopoDS_Face aFace in aFaces)
        {
            TopoDS_Wire aW = ShapeAnalysis.OuterWire(aFace);
            var aWD = new ShapeExtend_WireData(aW, true, false);
            var aSAW = new ShapeAnalysis_Wire(aWD, aFace, Precision.Confusion());
            var anOrder = new ShapeAnalysis_WireOrder();
            Assert.True(aSAW.CheckOrder(anOrder, true, false));
            Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE5));
            Assert.False(aSAW.LastCheckStatus(ShapeExtend_DONE3));
            Assert.False(aSAW.LastCheckStatus(ShapeExtend_FAIL));
            Assert.Equal(aWD.NbEdges(), anOrder.NbEdges());
            Assert.Equal([1, 4, 3, 2], Order(anOrder, 4));

            // in 3D too: the top edge is closed, and the seam leaving its vertex (edge 4) is preferred to the one
            // arriving there (edge 2)
            anOrder = new ShapeAnalysis_WireOrder();
            Assert.True(aSAW.CheckOrder(anOrder, true, true));
            Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE5));
            Assert.Equal([1, 4, 3, 2], Order(anOrder, 4));
        }
    }

    // ---------------------------------------------------------------- connection and small edges

    [Fact]
    public void CheckConnected_SharedAndCoincidentVertices()
    {
        TopoDS_Face aFace = PlaneFace();
        // shared vertex: OK
        TopoDS_Edge[] anE = SquareEdges();
        var aSAW = new ShapeAnalysis_Wire(WireData(anE), aFace, Prec);
        Assert.False(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_OK));
        Assert.False(aSAW.CheckConnected());

        // distinct vertices at the same point: DONE1
        aSAW = new ShapeAnalysis_Wire(WireData(Edge(A, B), Edge(B, C)), aFace, Prec);
        Assert.True(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));
        Assert.Equal(0.0, aSAW.MinDistance3d());

        // a gap under the precision: DONE2
        aSAW = new ShapeAnalysis_Wire(WireData(Edge(A, B), Edge(new gp_Pnt(1, 1e-5, 0), C)), aFace, 1e-3);
        Assert.True(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE2));
        Assert.Equal(1e-5, aSAW.MinDistance3d(), 1e-15);

        // a gap under prec only: DONE3
        aSAW = new ShapeAnalysis_Wire(WireData(Edge(A, B), Edge(new gp_Pnt(1, 0.1, 0), C)), aFace, 1e-3);
        Assert.True(aSAW.CheckConnected(2, 0.5));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE3));

        // too far: FAIL1
        Assert.False(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_FAIL1));

        // the second edge the wrong way round: FAIL2
        aSAW = new ShapeAnalysis_Wire(WireData(Edge(A, B), Edge(C, B)), aFace, 1e-3);
        Assert.False(aSAW.CheckConnected(2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_FAIL2));
    }

    [Fact]
    public void CheckSmall_DetectsEdgesUnderTheGivenPrecision()
    {
        var aSAW = new ShapeAnalysis_Wire(WireData(Edge(A, B), Edge(B, new gp_Pnt(5, 0, 0))), PlaneFace(), Prec);
        Assert.False(aSAW.CheckSmall(1, 0.5));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_OK));
        Assert.True(aSAW.CheckSmall(1, 2.0));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE2));
        Assert.False(aSAW.CheckSmall(0.5));
        Assert.True(aSAW.CheckSmall(2.0));
        Assert.True(aSAW.StatusSmall(ShapeExtend_DONE2));
    }

    [Fact]
    public void CheckClosed_OpenAndClosedWires()
    {
        TopoDS_Face aFace = PlaneFace();
        var aSAW = new ShapeAnalysis_Wire(WireData(SquareEdges()), aFace, Prec);
        Assert.False(aSAW.CheckClosed());
        Assert.True(aSAW.StatusClosed(ShapeExtend_OK));

        TopoDS_Edge[] anE = SquareEdges();
        aSAW = new ShapeAnalysis_Wire(WireData(anE[0], anE[1], anE[2]), aFace, Prec);
        Assert.False(aSAW.CheckClosed());
        Assert.True(aSAW.StatusClosed(ShapeExtend_FAIL1));
    }

    // ---------------------------------------------------------------- whole checks on good faces

    [Fact]
    public void Perform_OnAGoodPolygonFace_FindsNothing()
    {
        TopoDS_Wire aW = Polygon(new gp_Pnt(0, 0, 0), new gp_Pnt(4, 0, 0), new gp_Pnt(4, 3, 0), new gp_Pnt(0, 3, 0));
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(aW).Face();
        var aSAW = new ShapeAnalysis_Wire(aW, aFace, Prec);
        Assert.True(aSAW.IsLoaded());
        Assert.True(aSAW.IsReady());
        Assert.Equal(4, aSAW.NbEdges());
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
        TopoDS_Face[] aFaces =
        [
            new BRepPrimAPI_MakeCylinder(2, 5).Face(),
            new BRepPrimAPI_MakeSphere(3).Face(),
        ];
        foreach (TopoDS_Face aFace in aFaces)
        {
            var aSAW = new ShapeAnalysis_Wire(ShapeAnalysis.OuterWire(aFace), aFace, Prec);
            Assert.False(aSAW.CheckDegenerated());
            Assert.False(aSAW.CheckSelfIntersection());
            for (int i = 1; i <= aSAW.NbEdges(); i++)
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
        TopoDS_Wire aW = Polygon(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 2, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(0, 2, 0));
        var aSAW = new ShapeAnalysis_Wire(aW, PlaneFace(), Prec);
        Assert.True(aSAW.CheckSelfIntersection());
        Assert.True(aSAW.StatusSelfIntersection(ShapeExtend_DONE3));
        Assert.False(aSAW.StatusSelfIntersection(ShapeExtend_DONE1));
        Assert.False(aSAW.StatusSelfIntersection(ShapeExtend_DONE2));

        var aPnts2d = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        var aPnts3d = new NCollection_Sequence<gp_Pnt>();
        var anErrors = new NCollection_Sequence<double>();
        Assert.True(aSAW.CheckIntersectingEdges(1, 3, aPnts2d, aPnts3d, anErrors));
        Assert.Equal(1, aPnts3d.Length());
        Assert.True(aPnts3d.Value(1).Distance(new gp_Pnt(1, 1, 0)) < 1e-9);
        Assert.True(anErrors.Value(1) < 1e-9);
        // adjacent edges meet at their common vertex only
        Assert.False(aSAW.CheckIntersectingEdges(2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_OK));
        Assert.False(aSAW.CheckSelfIntersectingEdge(1));
    }

    // ---------------------------------------------------------------- lacking edges

    [Fact]
    public void CheckLacking_GapInsideTheVertexTolerance()
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV0 = new(), aV = new(), aV2 = new();
        aB.MakeVertex(ref aV0, new gp_Pnt(0, 0, 0), Prec);
        aB.MakeVertex(ref aV, new gp_Pnt(1.1, 0, 0), 0.3);
        aB.MakeVertex(ref aV2, new gp_Pnt(1.2, 1, 0), Prec);
        TopoDS_Edge anE1 = new(), anE2 = new();
        aB.MakeEdge(ref anE1, new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), Prec);
        aB.Add(anE1, aV0.Oriented(TopAbs_FORWARD));
        aB.Add(anE1, aV.Oriented(TopAbs_REVERSED));
        aB.Range(anE1, 0, 1);
        aB.MakeEdge(ref anE2, new Geom_Line(new gp_Pnt(1.2, 0, 0), new gp_Dir(0, 1, 0)), Prec);
        aB.Add(anE2, aV.Oriented(TopAbs_FORWARD));
        aB.Add(anE2, aV2.Oriented(TopAbs_REVERSED));
        aB.Range(anE2, 0, 1);

        var aSAW = new ShapeAnalysis_Wire(WireData(anE1, anE2), PlaneFace(), Prec);
        gp_Pnt2d aP1 = new(), aP2 = new();
        Assert.True(aSAW.CheckLacking(2, 0.05, ref aP1, ref aP2));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));
        Assert.False(aSAW.LastCheckStatus(ShapeExtend_DONE2));
        Assert.True(aP1.Distance(new gp_Pnt2d(1, 0)) < 1e-12);
        Assert.True(aP2.Distance(new gp_Pnt2d(1.2, 0)) < 1e-12);
        Assert.Equal(0.2, aSAW.MaxDistance2d(), 1e-12);
        // the vertex tolerance (0.3) covers it
        Assert.False(aSAW.CheckLacking(2));
    }

    // ---------------------------------------------------------------- notches, tails, loops, area

    [Fact]
    public void CheckNotchedEdges_EdgeGoingBack()
    {
        TopoDS_Vertex aVA = Vertex(0, 0), aVB = Vertex(5, 0), aVC = Vertex(2, 0);
        var aSAW = new ShapeAnalysis_Wire(WireData(Edge(aVA, aVB), Edge(aVB, aVC)), PlaneFace(), Prec);
        int aShort = 0;
        double aParam = 0;
        Assert.True(aSAW.CheckNotchedEdges(2, ref aShort, ref aParam, 0.01));
        Assert.Equal(2, aShort);
        Assert.Equal(2.0, aParam, 1e-9);
    }

    [Fact]
    public void CheckTail_ThinSpikeOfTwoWholeEdges()
    {
        TopoDS_Vertex aVA = Vertex(0, 0), aVB = Vertex(10, 0), aVC = Vertex(0, 1e-4);
        TopoDS_Edge anE1 = Edge(aVA, aVB), anE2 = Edge(aVB, aVC);
        var aSAW = new ShapeAnalysis_Wire(WireData(anE1, anE2), PlaneFace(), Prec);
        TopoDS_Edge anE11 = new(), anE12 = new(), anE21 = new(), anE22 = new();
        Assert.True(aSAW.CheckTail(anE1, anE2, 0.01, 0.001, 0.1, ref anE11, ref anE12, ref anE21, ref anE22));
        Assert.True(anE11.IsEqual(anE1));
        Assert.True(anE21.IsEqual(anE2));
        Assert.True(anE12.IsNull());
        Assert.True(anE22.IsNull());

        // a right angle is no tail
        anE11 = new TopoDS_Edge();
        Assert.False(aSAW.CheckTail(Edge(aVA, aVB), Edge(aVB, Vertex(10, 5)), 0.01, 0.001, 0.1, ref anE11, ref anE12, ref anE21, ref anE22));
        Assert.True(anE11.IsNull());
    }

    [Fact]
    public void CheckLoop_FindsTheVertexOfTwoTriangles()
    {
        TopoDS_Vertex aO = Vertex(0, 0), aA = Vertex(1, 0), aB = Vertex(1, 1), aC = Vertex(-1, 0), aD = Vertex(-1, -1);
        var aSAW = new ShapeAnalysis_Wire(WireData(Edge(aO, aA), Edge(aA, aB), Edge(aB, aO), Edge(aO, aC), Edge(aC, aD), Edge(aD, aO)),
                                          PlaneFace(), Prec);
        var aLoops = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aVE = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        var aSmall = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aSeams = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        Assert.True(aSAW.CheckLoop(aLoops, aVE, aSmall, aSeams));
        Assert.True(aSAW.StatusLoop(ShapeExtend_DONE1));
        Assert.Equal(1, aLoops.Extent());
        Assert.True(aLoops.FindKey(1).IsSame(aO));
        Assert.Equal(5, aVE.Extent());
        Assert.Equal(4, aVE.Find(aO).Extent());
        Assert.Equal(Prec, aSAW.Precision());
    }

    [Fact]
    public void CheckSmallArea_Sliver()
    {
        TopoDS_Wire aSliver = Polygon(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(5, 1e-9, 0));
        var aSAW = new ShapeAnalysis_Wire(aSliver, PlaneFace(), Prec);
        Assert.True(aSAW.CheckSmallArea(aSliver));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));

        TopoDS_Wire aSquare = Polygon(A, B, C, D);
        aSAW = new ShapeAnalysis_Wire(aSquare, PlaneFace(), Prec);
        Assert.False(aSAW.CheckSmallArea(aSquare));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_OK));
    }

    [Fact]
    public void CheckShapeConnect_FollowingEdgeDirectAndReversed()
    {
        TopoDS_Edge[] anE = SquareEdges();
        var aSAW = new ShapeAnalysis_Wire(WireData(anE[0], anE[1]), PlaneFace(), Prec);
        // C to D follows the wire A-B-C directly
        Assert.True(aSAW.CheckShapeConnect(Edge(C, D)));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE5));
        Assert.Equal(0.0, aSAW.MinDistance3d());

        // D to C follows it reversed
        Assert.True(aSAW.CheckShapeConnect(out double aTH, out double aTT, out double aHT, out double aHH, Edge(D, C)));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE2));
        Assert.Equal(1.0, aTH);
        Assert.Equal(0.0, aTT);
        Assert.Equal(Math.Sqrt(2.0), aHT);
        Assert.Equal(1.0, aHH);

        // far away: FAIL2; not an edge or a wire: FAIL1
        Assert.False(aSAW.CheckShapeConnect(Edge(new gp_Pnt(5, 5, 0), new gp_Pnt(6, 5, 0))));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_FAIL2));
        Assert.False(aSAW.CheckShapeConnect(Vertex(5, 5)));
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_FAIL1));
    }

    [Fact]
    public void CheckOuterBound_HoleWire()
    {
        TopoDS_Face aFace = FaceWithHoles(out TopoDS_Wire anOuter, out TopoDS_Wire aHole1, out _);
        var aSAW = new ShapeAnalysis_Wire(anOuter, aFace, Prec);
        Assert.False(aSAW.CheckOuterBound());
        Assert.False(aSAW.CheckOuterBound(false));
        aSAW = new ShapeAnalysis_Wire(aHole1, aFace, Prec);
        Assert.True(aSAW.CheckOuterBound());
        Assert.True(aSAW.LastCheckStatus(ShapeExtend_DONE1));
    }

    [Fact]
    public void SetSurface_MakesAFrozenFace()
    {
        var aSAW = new ShapeAnalysis_Wire();
        Assert.False(aSAW.IsLoaded());
        Assert.Equal(0, aSAW.NbEdges());
        Assert.Equal(Precision.Confusion(), aSAW.Precision());
        aSAW.SetSurface(new Geom_Plane(new gp_Pln()));
        Assert.False(aSAW.Face().IsNull());
        Assert.True(aSAW.Face().TShape()!.IsFrozen());
        Assert.NotNull(aSAW.Surface());
        aSAW.Load(WireData(SquareEdges()));
        Assert.True(aSAW.IsReady());
        aSAW.ClearStatuses();
        Assert.True(aSAW.StatusOrder(ShapeExtend_OK));
    }

    // ---------------------------------------------------------------- helpers

    /// <summary>
    /// A plane face (z = 0) with an outer square 0..10 and two square holes, its wires added hole, outer, hole, so that
    /// the outer wire isn't the first.
    /// </summary>
    internal static TopoDS_Face FaceWithHoles(out TopoDS_Wire theOuter, out TopoDS_Wire theHole1, out TopoDS_Wire theHole2)
    {
        theOuter = Polygon(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0), new gp_Pnt(0, 10, 0));
        theHole1 = TopoDS.Wire(Polygon(new gp_Pnt(2, 2, 0), new gp_Pnt(4, 2, 0), new gp_Pnt(4, 4, 0), new gp_Pnt(2, 4, 0)).Reversed());
        theHole2 = TopoDS.Wire(Polygon(new gp_Pnt(6, 6, 0), new gp_Pnt(8, 6, 0), new gp_Pnt(8, 8, 0), new gp_Pnt(6, 8, 0)).Reversed());
        var aB = new BRep_Builder();
        TopoDS_Face aFace = new();
        aB.MakeFace(ref aFace, new Geom_Plane(new gp_Pln()), Prec);
        aB.Add(aFace, theHole1);
        aB.Add(aFace, theOuter);
        aB.Add(aFace, theHole2);
        aFace.TShape()!.FreezeGraph();
        return aFace;
    }
}
