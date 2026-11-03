// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepCheck_Wire and BRepCheck.SelfIntersection on frozen BRep_Builder wires: valid, empty, disconnected, open,
/// redundant, badly oriented, self-intersecting and not closed in 2D; on planes and on a cylinder with a seam.
/// </summary>
public class BRepCheckWireTests
{
    private const double Tol = 1.0e-7;

    private static List<StatusCheck> ToList(NCollection_List<StatusCheck> theList) => [.. theList];

    /// <summary>A face on the plane z = 0 (normal +Z, UV = XY), without wires. Unfrozen.</summary>
    private static Face PlaneFace()
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, Tol);
        return aF;
    }

    private static Vertex V(double x, double y) => TestShapes.Vertex(new Point3D(x, y, 0));

    /// <summary>A segment between two vertices of the plane z = 0, with its pcurve on <paramref name="theFace"/>.</summary>
    private static Edge Seg(in Face theFace, in Vertex theV1, in Vertex theV2)
    {
        var aB = new BrepBuilder();
        Point3D aP1 = Tool.Pnt(theV1), aP2 = Tool.Pnt(theV2);
        Edge anE = TestShapes.Edge(theV1, theV2);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(aP1.X(), aP1.Y()), new Direction2D(aP2.X() - aP1.X(), aP2.Y() - aP1.Y())),
                      theFace, Tol);
        return anE;
    }

    private static Wire WireOf(params Shape[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Shape anE in theEdges)
        {
            aB.Add(aW, anE);
        }

        return aW;
    }

    private static Wire WireOf(Edge[] theEdges) => WireOf([.. theEdges.Select(e => (Shape)e)]);

    /// <summary>The face with the wire added, frozen; the wire as it is in the face.</summary>
    private static Wire InFace(ref Face theFace, in Wire theWire)
    {
        var aB = new BrepBuilder();
        aB.Add(theFace, theWire);
        theFace = TestShapes.Frozen(theFace);
        return ShapeCasts.Wire(new Iterator(theFace).Value());
    }

    /// <summary>A polygon through the corners on <paramref name="theFace"/>: edges i -> i + 1 (closed if asked).</summary>
    private static Edge[] Polygon(in Face theFace, bool theClosed, params (double X, double Y)[] theCorners)
    {
        var aV = theCorners.Select(c => V(c.X, c.Y)).ToArray();
        int n = theClosed ? aV.Length : aV.Length - 1;
        var anEdges = new Edge[n];
        for (int i = 0; i < n; i++)
        {
            anEdges[i] = Seg(theFace, aV[i], aV[(i + 1) % aV.Length]);
        }

        return anEdges;
    }

    private static (Face Face, Wire Wire) Square()
    {
        Face aF = PlaneFace();
        Wire aW = InFace(ref aF, WireOf(Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10))));
        return (aF, aW);
    }

    /// <summary>The statuses of the wire in the face, checked as the analyzer does.</summary>
    private static List<StatusCheck> InFaceStatus(in Wire theWire, in Face theFace, bool theIsParallel = false)
    {
        var aCheck = new WireCheck(theWire);
        aCheck.SetParallel(theIsParallel);
        aCheck.InContext(theFace);
        return ToList(aCheck.StatusOnShape(theFace));
    }

    [Fact]
    public void Closed_square_is_valid()
    {
        (Face aF, Wire aW) = Square();
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.True(aCheck.IsMinimum);
        Assert.True(aCheck.GeometricControls());
        Assert.Equal(StatusCheck.NoError, aCheck.Closed());
        Assert.Equal(StatusCheck.NoError, aCheck.Orientation(aF));
        Assert.Equal(StatusCheck.NoError, aCheck.Closed2d(aF));
        Edge anE1 = new(), anE2 = new();
        Assert.Equal(StatusCheck.NoError, aCheck.SelfIntersect(aF, ref anE1, ref anE2));
        Assert.True(anE1.IsNull);
        Assert.True(anE2.IsNull);
        aCheck.InContext(aF);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.StatusOnShape(aF)));
        // a second InContext does nothing
        aCheck.InContext(aF);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.StatusOnShape(aF)));
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        // the wire on its own, without a face: closed and coherently oriented
        Assert.Equal(StatusCheck.NoError, aCheck.Orientation(new Face()));
    }

    [Fact]
    public void Reversed_face_and_hole_are_valid()
    {
        Face aF = TestShapes.Frozen(ClassificationTestShapes.HoledSquare());
        for (var anExp = new Explorer(aF, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            Assert.Equal([StatusCheck.NoError], InFaceStatus(ShapeCasts.Wire(anExp.Current()), aF));
        }

        Face aR = ShapeCasts.Face(aF.Reversed());
        for (var anExp = new Explorer(aR, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            Assert.Equal([StatusCheck.NoError], InFaceStatus(ShapeCasts.Wire(anExp.Current()), aR));
        }
    }

    [Fact]
    public void Empty_wire()
    {
        Wire aW = TestShapes.Frozen(WireOf());
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.EmptyWire], ToList(aCheck.Status()));
        Face aF = TestShapes.Frozen(PlaneFace());
        Edge anE1 = new(), anE2 = new();
        Assert.Equal(StatusCheck.EmptyWire, aCheck.SelfIntersect(aF, ref anE1, ref anE2));
        // Closed reports the first status of the wire
        Assert.Equal(StatusCheck.EmptyWire, aCheck.Closed());
    }

    [Fact]
    public void Disconnected_wire()
    {
        Face aF = PlaneFace();
        Edge[] aSquare = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        Edge[] anOther = Polygon(aF, true, (20, 0), (30, 0), (30, 10));
        Wire aW = TestShapes.Frozen(WireOf([.. aSquare, .. anOther]));
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.NotConnected], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NotConnected, aCheck.Closed());
    }

    [Fact]
    public void Open_wire_in_a_face_is_not_closed()
    {
        Face aF = PlaneFace();
        Wire aW = InFace(ref aF, WireOf(Polygon(aF, false, (0, 0), (10, 0), (10, 10), (0, 10))));
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NotClosed, aCheck.Closed());
        Assert.Equal([StatusCheck.NotClosed], InFaceStatus(aW, aF));
        // Update adds the status to the wire's own list, replacing NoError
        Assert.Equal(StatusCheck.NotClosed, aCheck.Closed(true));
        Assert.Equal([StatusCheck.NotClosed], ToList(aCheck.Status()));
    }

    [Fact]
    public void Edge_twice_in_the_same_orientation_is_redundant()
    {
        Face aF = PlaneFace();
        Edge[] aSquare = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        Wire aW = InFace(ref aF, WireOf([.. aSquare, aSquare[0]]));
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.RedundantEdge, aCheck.Closed());
        Assert.Equal([StatusCheck.RedundantEdge], InFaceStatus(aW, aF));
    }

    [Fact]
    public void Edge_reversed_in_a_closed_wire_is_badly_oriented()
    {
        Face aF = PlaneFace();
        Edge[] aSquare = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        Wire aW = InFace(ref aF, WireOf(aSquare[0], aSquare[1], aSquare[2].Reversed(), aSquare[3]));
        var aCheck = new WireCheck(aW);
        Assert.Equal(StatusCheck.NoError, aCheck.Closed());
        Assert.Equal(StatusCheck.BadOrientationOfSubshape, aCheck.Orientation(aF));
        Assert.Equal([StatusCheck.BadOrientationOfSubshape], InFaceStatus(aW, aF));
        // without a face too
        Assert.Equal(StatusCheck.BadOrientationOfSubshape, new WireCheck(aW).Orientation(new Face()));
    }

    [Fact]
    public void Figure_eight_intersects_itself()
    {
        Face aF = PlaneFace();
        // (0, 0) -> (10, 10) -> (10, 0) -> (0, 10) -> (0, 0): the first and third edges cross at (5, 5)
        Edge[] anEdges = Polygon(aF, true, (0, 0), (10, 10), (10, 0), (0, 10));
        Wire aW = InFace(ref aF, WireOf(anEdges));
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Edge anE1 = new(), anE2 = new();
        Assert.Equal(StatusCheck.SelfIntersectingWire, aCheck.SelfIntersect(aF, ref anE1, ref anE2));
        Assert.True(anE1.IsSame(anEdges[0]));
        Assert.True(anE2.IsSame(anEdges[2]));
        Assert.Equal([StatusCheck.SelfIntersectingWire], InFaceStatus(aW, aF));

        // the static
        Edge aS1 = new(), aS2 = new();
        Assert.True(BRepCheckCheck.SelfIntersection(aW, aF, ref aS1, ref aS2));
        Assert.True(aS1.IsSame(anEdges[0]));
        Assert.True(aS2.IsSame(anEdges[2]));
        (Face aSqF, Wire aSqW) = Square();
        Edge aN1 = new(), aN2 = new();
        Assert.False(BRepCheckCheck.SelfIntersection(aSqW, aSqF, ref aN1, ref aN2));
        Assert.True(aN1.IsNull && aN2.IsNull);

        // without the geometric controls, InContext doesn't look for self-intersections
        var aBlind = new WireCheck(aW);
        aBlind.GeometricControls(false);
        Assert.False(aBlind.GeometricControls());
        aBlind.InContext(aF);
        Assert.DoesNotContain(StatusCheck.SelfIntersectingWire, ToList(aBlind.StatusOnShape(aF)));
    }

    [Fact]
    public void Edges_meeting_at_their_common_vertices_do_not_intersect()
    {
        Face aF = PlaneFace();
        Wire aW = InFace(ref aF, WireOf(Polygon(aF, true, (0, 0), (10, 0), (5, 8))));
        Edge anE1 = new(), anE2 = new();
        Assert.Equal(StatusCheck.NoError, new WireCheck(aW).SelfIntersect(aF, ref anE1, ref anE2));
    }

    [Fact]
    public void Edge_without_pcurve_on_the_face()
    {
        // pcurves on a plane only; checked on a cylinder (no projection on the fly): the first edge has none
        Face aF = PlaneFace();
        Edge[] anEdges = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        Wire aW = InFace(ref aF, WireOf(anEdges));
        Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        Edge anE1 = new(), anE2 = new();
        Assert.Equal(StatusCheck.SelfIntersectingWire, new WireCheck(aW).SelfIntersect(aCyl, ref anE1, ref anE2));
        Assert.True(anE1.IsSame(anEdges[0]));
        Assert.True(anE2.IsNull);
    }

    [Fact]
    public void Seam_wire_on_a_cylinder_is_valid()
    {
        Face aF = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        Wire aW = ShapeCasts.Wire(new Iterator(aF).Value());
        var aCheck = new WireCheck(aW);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NoError, aCheck.Closed());
        Assert.Equal(StatusCheck.NoError, aCheck.Closed2d(aF));
        Assert.Equal([StatusCheck.NoError], InFaceStatus(aW, aF));
        Assert.Equal([StatusCheck.NoError], InFaceStatus(aW, aF, true));
        // in the reversed face, the wire is reversed too (as the analyzer's TopoDS_Iterator gives it)
        Face aR = ShapeCasts.Face(aF.Reversed());
        Wire aRW = ShapeCasts.Wire(new Iterator(aR).Value());
        Assert.Equal(Reversed, aRW.Orientation());
        Assert.Equal([StatusCheck.NoError], InFaceStatus(aRW, aR));
        // the forward wire in the reversed face takes the other pcurves of the seam: not closed in 2D
        Assert.Equal([StatusCheck.NotClosed], InFaceStatus(aW, aR));
    }

    /// <summary>
    /// A patch [0, 1] x [0, 1] of the cylinder of radius 1 whose left edge (3D: the line at U = 0) has its pcurve at
    /// U = -0.2: closed in 3D, not in the parametric space.
    /// </summary>
    private static (Face, Wire) CylinderPatchOpenIn2d()
    {
        var aB = new BrepBuilder();
        var aSurf = new CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0);
        Face aF = new();
        aB.MakeFace(ref aF, aSurf, Tol);
        Vertex aV00 = TestShapes.Vertex(new Point3D(1, 0, 0));
        Vertex aV10 = TestShapes.Vertex(new Point3D(Math.Cos(1), Math.Sin(1), 0));
        Vertex aV11 = TestShapes.Vertex(new Point3D(Math.Cos(1), Math.Sin(1), 1));
        Vertex aV01 = TestShapes.Vertex(new Point3D(1, 0, 1));
        Edge aBottom = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1),
                                              0, 1, aV00, aV10);
        Edge aTop = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1),
                                           0, 1, aV01, aV11);
        Edge aRight = TestShapes.Edge(new LineCurve3D(new Point3D(Math.Cos(1), Math.Sin(1), 0), new Direction3D(0, 0, 1)), 0, 1, aV10, aV11);
        Edge aLeft = TestShapes.Edge(new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 0, 1, aV00, aV01);
        aB.UpdateEdge(aBottom, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Tol);
        aB.UpdateEdge(aTop, new LineCurve2D(new Point2D(0, 1), new Direction2D(1, 0)), aF, Tol);
        aB.UpdateEdge(aRight, new LineCurve2D(new Point2D(1, 0), new Direction2D(0, 1)), aF, Tol);
        aB.UpdateEdge(aLeft, new LineCurve2D(new Point2D(-0.2, 0), new Direction2D(0, 1)), aF, Tol);
        Wire aW = InFace(ref aF, WireOf(aBottom, aRight, aTop.Reversed(), aLeft.Reversed()));
        return (aF, aW);
    }

    [Fact]
    public void Wire_closed_in_3d_but_not_in_2d()
    {
        (Face aF, Wire aW) = CylinderPatchOpenIn2d();
        var aCheck = new WireCheck(aW);
        Assert.Equal(StatusCheck.NoError, aCheck.Closed());
        Assert.Equal(StatusCheck.NotClosed, aCheck.Closed2d(aF));
        Assert.Equal([StatusCheck.NotClosed], InFaceStatus(aW, aF));
    }

    [Fact]
    public void Wire_not_in_the_context()
    {
        (Face _, Wire aW) = Square();
        (Face anOther, Wire _) = Square();
        Assert.Equal([StatusCheck.SubshapeNotInShape], InFaceStatus(aW, anOther));
    }

    [Fact]
    public void Parallel_mode_gives_the_same_statuses()
    {
        var aCases = new List<(Wire, Face)>();
        aCases.Add((Square().Wire, Square().Face));
        (Face aF, Wire aW) = Square();
        aCases.Add((aW, aF));
        Face aF8 = PlaneFace();
        aCases.Add((InFace(ref aF8, WireOf(Polygon(aF8, true, (0, 0), (10, 10), (10, 0), (0, 10)))), aF8));
        Face anOpen = PlaneFace();
        aCases.Add((InFace(ref anOpen, WireOf(Polygon(anOpen, false, (0, 0), (10, 0), (10, 10)))), anOpen));
        aCases.Add(CylinderPatchOpenIn2d() is var (aPF, aPW) ? (aPW, aPF) : default);
        Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        aCases.Add((ShapeCasts.Wire(new Iterator(aCyl).Value()), aCyl));

        foreach ((Wire aWire, Face aFace) in aCases)
        {
            List<StatusCheck> aSerial = InFaceStatus(aWire, aFace);
            List<StatusCheck> aParallel = InFaceStatus(aWire, aFace, true);
            Assert.Equal(aSerial, aParallel);
        }

        // concurrent contexts on one result, as the analyzer's parallel mode does
        var aCheck = new WireCheck(aW);
        aCheck.SetParallel(true);
        Parallel.For(0, 8, i => aCheck.InContext(i % 2 == 0 ? aF : ShapeCasts.Face(aF.Reversed())));
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.StatusOnShape(aF)));
    }

    [Fact]
    public void SetStatus_adds_to_the_wire_statuses()
    {
        (Face _, Wire aW) = Square();
        var aCheck = new WireCheck(aW);
        aCheck.SetStatus(StatusCheck.SelfIntersectingWire);
        Assert.Equal([StatusCheck.SelfIntersectingWire], ToList(aCheck.Status()));
        aCheck.Blind();
        Assert.True(aCheck.IsBlind);
    }
}
