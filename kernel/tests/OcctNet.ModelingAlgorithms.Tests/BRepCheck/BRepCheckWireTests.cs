// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.BRepCheck_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepCheck_Wire and BRepCheck.SelfIntersection on frozen BRep_Builder wires: valid, empty, disconnected, open,
/// redundant, badly oriented, self-intersecting and not closed in 2D; on planes and on a cylinder with a seam.
/// </summary>
public class BRepCheckWireTests
{
    private const double Tol = 1.0e-7;

    private static List<BRepCheck_Status> ToList(NCollection_List<BRepCheck_Status> theList) => [.. theList];

    /// <summary>A face on the plane z = 0 (normal +Z, UV = XY), without wires. Unfrozen.</summary>
    private static TopoDS_Face PlaneFace()
    {
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, Tol);
        return aF;
    }

    private static TopoDS_Vertex V(double x, double y) => TestShapes.Vertex(new Point3D(x, y, 0));

    /// <summary>A segment between two vertices of the plane z = 0, with its pcurve on <paramref name="theFace"/>.</summary>
    private static TopoDS_Edge Seg(in TopoDS_Face theFace, in TopoDS_Vertex theV1, in TopoDS_Vertex theV2)
    {
        var aB = new BRep_Builder();
        Point3D aP1 = BRep_Tool.Pnt(theV1), aP2 = BRep_Tool.Pnt(theV2);
        TopoDS_Edge anE = TestShapes.Edge(theV1, theV2);
        aB.UpdateEdge(anE, new Geom2d_Line(new Point2D(aP1.X(), aP1.Y()), new Direction2D(aP2.X() - aP1.X(), aP2.Y() - aP1.Y())),
                      theFace, Tol);
        return anE;
    }

    private static TopoDS_Wire WireOf(params TopoDS_Shape[] theEdges)
    {
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (TopoDS_Shape anE in theEdges)
        {
            aB.Add(aW, anE);
        }

        return aW;
    }

    private static TopoDS_Wire WireOf(TopoDS_Edge[] theEdges) => WireOf([.. theEdges.Select(e => (TopoDS_Shape)e)]);

    /// <summary>The face with the wire added, frozen; the wire as it is in the face.</summary>
    private static TopoDS_Wire InFace(ref TopoDS_Face theFace, in TopoDS_Wire theWire)
    {
        var aB = new BRep_Builder();
        aB.Add(theFace, theWire);
        theFace = TestShapes.Frozen(theFace);
        return TopoDS.Wire(new TopoDS_Iterator(theFace).Value());
    }

    /// <summary>A polygon through the corners on <paramref name="theFace"/>: edges i -> i + 1 (closed if asked).</summary>
    private static TopoDS_Edge[] Polygon(in TopoDS_Face theFace, bool theClosed, params (double X, double Y)[] theCorners)
    {
        var aV = theCorners.Select(c => V(c.X, c.Y)).ToArray();
        int n = theClosed ? aV.Length : aV.Length - 1;
        var anEdges = new TopoDS_Edge[n];
        for (int i = 0; i < n; i++)
        {
            anEdges[i] = Seg(theFace, aV[i], aV[(i + 1) % aV.Length]);
        }

        return anEdges;
    }

    private static (TopoDS_Face Face, TopoDS_Wire Wire) Square()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Wire aW = InFace(ref aF, WireOf(Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10))));
        return (aF, aW);
    }

    /// <summary>The statuses of the wire in the face, checked as the analyzer does.</summary>
    private static List<BRepCheck_Status> InFaceStatus(in TopoDS_Wire theWire, in TopoDS_Face theFace, bool theIsParallel = false)
    {
        var aCheck = new BRepCheck_Wire(theWire);
        aCheck.SetParallel(theIsParallel);
        aCheck.InContext(theFace);
        return ToList(aCheck.StatusOnShape(theFace));
    }

    [Fact]
    public void Closed_square_is_valid()
    {
        (TopoDS_Face aF, TopoDS_Wire aW) = Square();
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.True(aCheck.IsMinimum());
        Assert.True(aCheck.GeometricControls());
        Assert.Equal(BRepCheck_NoError, aCheck.Closed());
        Assert.Equal(BRepCheck_NoError, aCheck.Orientation(aF));
        Assert.Equal(BRepCheck_NoError, aCheck.Closed2d(aF));
        TopoDS_Edge anE1 = new(), anE2 = new();
        Assert.Equal(BRepCheck_NoError, aCheck.SelfIntersect(aF, ref anE1, ref anE2));
        Assert.True(anE1.IsNull());
        Assert.True(anE2.IsNull());
        aCheck.InContext(aF);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.StatusOnShape(aF)));
        // a second InContext does nothing
        aCheck.InContext(aF);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.StatusOnShape(aF)));
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        // the wire on its own, without a face: closed and coherently oriented
        Assert.Equal(BRepCheck_NoError, aCheck.Orientation(new TopoDS_Face()));
    }

    [Fact]
    public void Reversed_face_and_hole_are_valid()
    {
        TopoDS_Face aF = TestShapes.Frozen(ClassificationTestShapes.HoledSquare());
        for (var anExp = new TopExp_Explorer(aF, TopAbs_WIRE); anExp.More(); anExp.Next())
        {
            Assert.Equal([BRepCheck_NoError], InFaceStatus(TopoDS.Wire(anExp.Current()), aF));
        }

        TopoDS_Face aR = TopoDS.Face(aF.Reversed());
        for (var anExp = new TopExp_Explorer(aR, TopAbs_WIRE); anExp.More(); anExp.Next())
        {
            Assert.Equal([BRepCheck_NoError], InFaceStatus(TopoDS.Wire(anExp.Current()), aR));
        }
    }

    [Fact]
    public void Empty_wire()
    {
        TopoDS_Wire aW = TestShapes.Frozen(WireOf());
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_EmptyWire], ToList(aCheck.Status()));
        TopoDS_Face aF = TestShapes.Frozen(PlaneFace());
        TopoDS_Edge anE1 = new(), anE2 = new();
        Assert.Equal(BRepCheck_EmptyWire, aCheck.SelfIntersect(aF, ref anE1, ref anE2));
        // Closed reports the first status of the wire
        Assert.Equal(BRepCheck_EmptyWire, aCheck.Closed());
    }

    [Fact]
    public void Disconnected_wire()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Edge[] aSquare = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        TopoDS_Edge[] anOther = Polygon(aF, true, (20, 0), (30, 0), (30, 10));
        TopoDS_Wire aW = TestShapes.Frozen(WireOf([.. aSquare, .. anOther]));
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_NotConnected], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NotConnected, aCheck.Closed());
    }

    [Fact]
    public void Open_wire_in_a_face_is_not_closed()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Wire aW = InFace(ref aF, WireOf(Polygon(aF, false, (0, 0), (10, 0), (10, 10), (0, 10))));
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NotClosed, aCheck.Closed());
        Assert.Equal([BRepCheck_NotClosed], InFaceStatus(aW, aF));
        // Update adds the status to the wire's own list, replacing NoError
        Assert.Equal(BRepCheck_NotClosed, aCheck.Closed(true));
        Assert.Equal([BRepCheck_NotClosed], ToList(aCheck.Status()));
    }

    [Fact]
    public void Edge_twice_in_the_same_orientation_is_redundant()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Edge[] aSquare = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        TopoDS_Wire aW = InFace(ref aF, WireOf([.. aSquare, aSquare[0]]));
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_RedundantEdge, aCheck.Closed());
        Assert.Equal([BRepCheck_RedundantEdge], InFaceStatus(aW, aF));
    }

    [Fact]
    public void Edge_reversed_in_a_closed_wire_is_badly_oriented()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Edge[] aSquare = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        TopoDS_Wire aW = InFace(ref aF, WireOf(aSquare[0], aSquare[1], aSquare[2].Reversed(), aSquare[3]));
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal(BRepCheck_NoError, aCheck.Closed());
        Assert.Equal(BRepCheck_BadOrientationOfSubshape, aCheck.Orientation(aF));
        Assert.Equal([BRepCheck_BadOrientationOfSubshape], InFaceStatus(aW, aF));
        // without a face too
        Assert.Equal(BRepCheck_BadOrientationOfSubshape, new BRepCheck_Wire(aW).Orientation(new TopoDS_Face()));
    }

    [Fact]
    public void Figure_eight_intersects_itself()
    {
        TopoDS_Face aF = PlaneFace();
        // (0, 0) -> (10, 10) -> (10, 0) -> (0, 10) -> (0, 0): the first and third edges cross at (5, 5)
        TopoDS_Edge[] anEdges = Polygon(aF, true, (0, 0), (10, 10), (10, 0), (0, 10));
        TopoDS_Wire aW = InFace(ref aF, WireOf(anEdges));
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        TopoDS_Edge anE1 = new(), anE2 = new();
        Assert.Equal(BRepCheck_SelfIntersectingWire, aCheck.SelfIntersect(aF, ref anE1, ref anE2));
        Assert.True(anE1.IsSame(anEdges[0]));
        Assert.True(anE2.IsSame(anEdges[2]));
        Assert.Equal([BRepCheck_SelfIntersectingWire], InFaceStatus(aW, aF));

        // the static
        TopoDS_Edge aS1 = new(), aS2 = new();
        Assert.True(BRepCheck.SelfIntersection(aW, aF, ref aS1, ref aS2));
        Assert.True(aS1.IsSame(anEdges[0]));
        Assert.True(aS2.IsSame(anEdges[2]));
        (TopoDS_Face aSqF, TopoDS_Wire aSqW) = Square();
        TopoDS_Edge aN1 = new(), aN2 = new();
        Assert.False(BRepCheck.SelfIntersection(aSqW, aSqF, ref aN1, ref aN2));
        Assert.True(aN1.IsNull() && aN2.IsNull());

        // without the geometric controls, InContext doesn't look for self-intersections
        var aBlind = new BRepCheck_Wire(aW);
        aBlind.GeometricControls(false);
        Assert.False(aBlind.GeometricControls());
        aBlind.InContext(aF);
        Assert.DoesNotContain(BRepCheck_SelfIntersectingWire, ToList(aBlind.StatusOnShape(aF)));
    }

    [Fact]
    public void Edges_meeting_at_their_common_vertices_do_not_intersect()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Wire aW = InFace(ref aF, WireOf(Polygon(aF, true, (0, 0), (10, 0), (5, 8))));
        TopoDS_Edge anE1 = new(), anE2 = new();
        Assert.Equal(BRepCheck_NoError, new BRepCheck_Wire(aW).SelfIntersect(aF, ref anE1, ref anE2));
    }

    [Fact]
    public void Edge_without_pcurve_on_the_face()
    {
        // pcurves on a plane only; checked on a cylinder (no projection on the fly): the first edge has none
        TopoDS_Face aF = PlaneFace();
        TopoDS_Edge[] anEdges = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        TopoDS_Wire aW = InFace(ref aF, WireOf(anEdges));
        TopoDS_Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        TopoDS_Edge anE1 = new(), anE2 = new();
        Assert.Equal(BRepCheck_SelfIntersectingWire, new BRepCheck_Wire(aW).SelfIntersect(aCyl, ref anE1, ref anE2));
        Assert.True(anE1.IsSame(anEdges[0]));
        Assert.True(anE2.IsNull());
    }

    [Fact]
    public void Seam_wire_on_a_cylinder_is_valid()
    {
        TopoDS_Face aF = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        TopoDS_Wire aW = TopoDS.Wire(new TopoDS_Iterator(aF).Value());
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NoError, aCheck.Closed());
        Assert.Equal(BRepCheck_NoError, aCheck.Closed2d(aF));
        Assert.Equal([BRepCheck_NoError], InFaceStatus(aW, aF));
        Assert.Equal([BRepCheck_NoError], InFaceStatus(aW, aF, true));
        // in the reversed face, the wire is reversed too (as the analyzer's TopoDS_Iterator gives it)
        TopoDS_Face aR = TopoDS.Face(aF.Reversed());
        TopoDS_Wire aRW = TopoDS.Wire(new TopoDS_Iterator(aR).Value());
        Assert.Equal(TopAbs_REVERSED, aRW.Orientation());
        Assert.Equal([BRepCheck_NoError], InFaceStatus(aRW, aR));
        // the forward wire in the reversed face takes the other pcurves of the seam: not closed in 2D
        Assert.Equal([BRepCheck_NotClosed], InFaceStatus(aW, aR));
    }

    /// <summary>
    /// A patch [0, 1] x [0, 1] of the cylinder of radius 1 whose left edge (3D: the line at U = 0) has its pcurve at
    /// U = -0.2: closed in 3D, not in the parametric space.
    /// </summary>
    private static (TopoDS_Face, TopoDS_Wire) CylinderPatchOpenIn2d()
    {
        var aB = new BRep_Builder();
        var aSurf = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aSurf, Tol);
        TopoDS_Vertex aV00 = TestShapes.Vertex(new Point3D(1, 0, 0));
        TopoDS_Vertex aV10 = TestShapes.Vertex(new Point3D(Math.Cos(1), Math.Sin(1), 0));
        TopoDS_Vertex aV11 = TestShapes.Vertex(new Point3D(Math.Cos(1), Math.Sin(1), 1));
        TopoDS_Vertex aV01 = TestShapes.Vertex(new Point3D(1, 0, 1));
        TopoDS_Edge aBottom = TestShapes.Edge(new Geom_Circle(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1),
                                              0, 1, aV00, aV10);
        TopoDS_Edge aTop = TestShapes.Edge(new Geom_Circle(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1),
                                           0, 1, aV01, aV11);
        TopoDS_Edge aRight = TestShapes.Edge(new Geom_Line(new Point3D(Math.Cos(1), Math.Sin(1), 0), new Direction3D(0, 0, 1)), 0, 1, aV10, aV11);
        TopoDS_Edge aLeft = TestShapes.Edge(new Geom_Line(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 0, 1, aV00, aV01);
        aB.UpdateEdge(aBottom, new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0)), aF, Tol);
        aB.UpdateEdge(aTop, new Geom2d_Line(new Point2D(0, 1), new Direction2D(1, 0)), aF, Tol);
        aB.UpdateEdge(aRight, new Geom2d_Line(new Point2D(1, 0), new Direction2D(0, 1)), aF, Tol);
        aB.UpdateEdge(aLeft, new Geom2d_Line(new Point2D(-0.2, 0), new Direction2D(0, 1)), aF, Tol);
        TopoDS_Wire aW = InFace(ref aF, WireOf(aBottom, aRight, aTop.Reversed(), aLeft.Reversed()));
        return (aF, aW);
    }

    [Fact]
    public void Wire_closed_in_3d_but_not_in_2d()
    {
        (TopoDS_Face aF, TopoDS_Wire aW) = CylinderPatchOpenIn2d();
        var aCheck = new BRepCheck_Wire(aW);
        Assert.Equal(BRepCheck_NoError, aCheck.Closed());
        Assert.Equal(BRepCheck_NotClosed, aCheck.Closed2d(aF));
        Assert.Equal([BRepCheck_NotClosed], InFaceStatus(aW, aF));
    }

    [Fact]
    public void Wire_not_in_the_context()
    {
        (TopoDS_Face _, TopoDS_Wire aW) = Square();
        (TopoDS_Face anOther, TopoDS_Wire _) = Square();
        Assert.Equal([BRepCheck_SubshapeNotInShape], InFaceStatus(aW, anOther));
    }

    [Fact]
    public void Parallel_mode_gives_the_same_statuses()
    {
        var aCases = new List<(TopoDS_Wire, TopoDS_Face)>();
        aCases.Add((Square().Wire, Square().Face));
        (TopoDS_Face aF, TopoDS_Wire aW) = Square();
        aCases.Add((aW, aF));
        TopoDS_Face aF8 = PlaneFace();
        aCases.Add((InFace(ref aF8, WireOf(Polygon(aF8, true, (0, 0), (10, 10), (10, 0), (0, 10)))), aF8));
        TopoDS_Face anOpen = PlaneFace();
        aCases.Add((InFace(ref anOpen, WireOf(Polygon(anOpen, false, (0, 0), (10, 0), (10, 10)))), anOpen));
        aCases.Add(CylinderPatchOpenIn2d() is var (aPF, aPW) ? (aPW, aPF) : default);
        TopoDS_Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        aCases.Add((TopoDS.Wire(new TopoDS_Iterator(aCyl).Value()), aCyl));

        foreach ((TopoDS_Wire aWire, TopoDS_Face aFace) in aCases)
        {
            List<BRepCheck_Status> aSerial = InFaceStatus(aWire, aFace);
            List<BRepCheck_Status> aParallel = InFaceStatus(aWire, aFace, true);
            Assert.Equal(aSerial, aParallel);
        }

        // concurrent contexts on one result, as the analyzer's parallel mode does
        var aCheck = new BRepCheck_Wire(aW);
        aCheck.SetParallel(true);
        Parallel.For(0, 8, i => aCheck.InContext(i % 2 == 0 ? aF : TopoDS.Face(aF.Reversed())));
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.StatusOnShape(aF)));
    }

    [Fact]
    public void SetStatus_adds_to_the_wire_statuses()
    {
        (TopoDS_Face _, TopoDS_Wire aW) = Square();
        var aCheck = new BRepCheck_Wire(aW);
        aCheck.SetStatus(BRepCheck_SelfIntersectingWire);
        Assert.Equal([BRepCheck_SelfIntersectingWire], ToList(aCheck.Status()));
        aCheck.Blind();
        Assert.True(aCheck.IsBlind());
    }
}
