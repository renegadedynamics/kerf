// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.BRepLib_EdgeError;
using static Occt.BRepLib_FaceError;
using static Occt.BRepLib_ShapeModification;
using static Occt.BRepLib_ShellError;
using static Occt.BRepLib_WireError;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of the BRepLib makers: vertices, edges, 2D edges, wires, polygons, faces, shells and solids, with
/// frozen inputs (as callers hold them), frozen results from the public accessors and inputs left unchanged.
/// </summary>
public class BRepLibMakersTests
{
    private const double Conf = 1e-7;

    private static TopoDS_Vertex V(double x, double y, double z) => new BRepLib_MakeVertex(new gp_Pnt(x, y, z)).Vertex();

    private static TopoDS_Edge E(in TopoDS_Vertex v1, in TopoDS_Vertex v2) => new BRepLib_MakeEdge(v1, v2).Edge();

    private static TopoDS_Edge E(double x1, double y1, double z1, double x2, double y2, double z2)
        => new BRepLib_MakeEdge(new gp_Pnt(x1, y1, z1), new gp_Pnt(x2, y2, z2)).Edge();

    private static int Count(in TopoDS_Shape S, TopAbs_ShapeEnum T)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, T, aMap);
        return aMap.Extent();
    }

    private static int CountOccurrences(in TopoDS_Shape S, TopAbs_ShapeEnum T)
    {
        int n = 0;
        for (var anExp = new TopExp_Explorer(S, T); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    private static void AssertPoint(in gp_Pnt theExpected, in gp_Pnt theActual, double theTol = 1e-12)
        => Assert.True(theExpected.Distance(theActual) <= theTol, $"{theExpected} != {theActual}");

    // ---------------------------------------------------------------- vertices

    [Fact]
    public void MakeVertex_is_a_frozen_vertex_at_the_point_with_the_default_precision()
    {
        var aMV = new BRepLib_MakeVertex(new gp_Pnt(1, 2, 3));
        Assert.True(aMV.IsDone());
        TopoDS_Vertex aV = aMV.Vertex();
        Assert.True(aV.TShape()!.IsFrozen());
        Assert.Equal(new gp_Pnt(1, 2, 3), BRep_Tool.Pnt(aV));
        Assert.Equal(BRepLib.Precision(), BRep_Tool.Tolerance(aV));
        Assert.Equal(aV, (TopoDS_Vertex)aMV);
        Assert.Equal((TopoDS_Shape)aV, aMV.Shape());
    }

    [Fact]
    public void Internal_accessors_do_not_freeze()
    {
        var aME = new BRepLib_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        TopoDS_Edge anE = aME.InternalEdge();
        Assert.False(anE.TShape()!.IsFrozen());
        Assert.False(aME.InternalVertex1().TShape()!.IsFrozen());
        Assert.True(aME.Vertex1().TShape()!.IsFrozen());
        Assert.True(anE.TShape()!.IsFrozen()); // Vertex1 publishes the edge with it
    }

    // ---------------------------------------------------------------- edges

    [Fact]
    public void Edge_from_two_points_is_a_segment_parametrised_by_length()
    {
        var aME = new BRepLib_MakeEdge(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 3, 4));
        Assert.True(aME.IsDone());
        Assert.Equal(BRepLib_EdgeDone, aME.Error());
        TopoDS_Edge anE = aME.Edge();
        Assert.True(anE.TShape()!.IsFrozen());
        Geom_Curve? aC = BRep_Tool.Curve(anE, out double f, out double l);
        Assert.IsType<Geom_Line>(aC);
        Assert.Equal(0.0, f);
        Assert.Equal(5.0, l);
        TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.Equal(new gp_Pnt(1, 0, 0), BRep_Tool.Pnt(aV1));
        Assert.Equal(new gp_Pnt(1, 3, 4), BRep_Tool.Pnt(aV2));
        Assert.True(aV1.IsSame(aME.Vertex1()));
        Assert.Equal(TopAbs_FORWARD, aME.Vertex1().Orientation());
        Assert.Equal(TopAbs_REVERSED, aME.Vertex2().Orientation());
        Assert.False(anE.Closed());
        Assert.False(BRep_Tool.Degenerated(anE));
        Assert.Equal(BRepLib.Precision(), BRep_Tool.Tolerance(anE));
    }

    [Fact]
    public void Edge_between_frozen_vertices_shares_them_and_leaves_them_unchanged()
    {
        TopoDS_Vertex aV1 = V(0, 0, 0), aV2 = V(2, 0, 0);
        ShapeSnapshot aS1 = ShapeSnapshot.Of(aV1), aS2 = ShapeSnapshot.Of(aV2);
        TopoDS_Edge anE = E(aV1, aV2);
        TopExp.Vertices(anE, out TopoDS_Vertex aF, out TopoDS_Vertex aL);
        Assert.True(aF.IsSame(aV1));
        Assert.True(aL.IsSame(aV2));
        aS1.AssertUnchanged(aV1);
        aS2.AssertUnchanged(aV2);
    }

    [Fact]
    public void Edge_through_identical_points_fails()
    {
        var aME = new BRepLib_MakeEdge(new gp_Pnt(1, 1, 1), new gp_Pnt(1, 1, 1));
        Assert.False(aME.IsDone());
        Assert.Equal(BRepLib_LineThroughIdenticPoints, aME.Error());
        Assert.Throws<StdFail_NotDone>(() => aME.Edge());
        Assert.Throws<StdFail_NotDone>(() => aME.Vertex1());
    }

    [Fact]
    public void Edge_on_a_line_between_parameters_reorders_them()
    {
        var aL = new gp_Lin(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0));
        TopoDS_Edge anE = new BRepLib_MakeEdge(aL, 3.0, -2.0).Edge();
        BRep_Tool.Range(anE, out double f, out double l);
        Assert.Equal(-2.0, f);
        Assert.Equal(3.0, l);
        TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.Equal(new gp_Pnt(0, -2, 0), BRep_Tool.Pnt(aV1));
        Assert.Equal(new gp_Pnt(0, 3, 0), BRep_Tool.Pnt(aV2));
    }

    [Fact]
    public void Edge_on_a_whole_line_is_infinite_without_vertices()
    {
        var aME = new BRepLib_MakeEdge(new gp_Lin(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)));
        Assert.True(aME.IsDone());
        TopoDS_Edge anE = aME.Edge();
        Assert.True(aME.Vertex1().IsNull());
        Assert.True(aME.Vertex2().IsNull());
        Assert.Equal(0, Count(anE, TopAbs_VERTEX));
        BRep_Tool.Range(anE, out double f, out double l);
        Assert.True(Precision.IsNegativeInfinite(f));
        Assert.True(Precision.IsPositiveInfinite(l));
    }

    [Fact]
    public void Edge_on_a_whole_circle_is_closed_with_one_vertex()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        var aME = new BRepLib_MakeEdge(aCirc);
        TopoDS_Edge anE = aME.Edge();
        Assert.True(anE.Closed());
        Assert.False(BRep_Tool.Degenerated(anE));
        Assert.Equal(1, Count(anE, TopAbs_VERTEX));
        Assert.True(aME.Vertex1().IsSame(aME.Vertex2()));
        BRep_Tool.Range(anE, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(2 * Math.PI, l);
        Assert.True(BRep_Tool.IsClosed(anE));
    }

    [Fact]
    public void Arc_of_circle_between_points_projects_them()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        var aME = new BRepLib_MakeEdge(aCirc, new gp_Pnt(0, 2, 0), new gp_Pnt(-2, 0, 0));
        Assert.True(aME.IsDone());
        BRep_Tool.Range(aME.Edge(), out double f, out double l);
        Assert.Equal(Math.PI / 2, f, 1e-12);
        Assert.Equal(Math.PI, l, 1e-12);
    }

    [Fact]
    public void Arc_of_circle_with_parameters_adjusts_them_into_the_period()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        TopoDS_Edge anE = new BRepLib_MakeEdge(aCirc, 7.0, 8.0).Edge();
        BRep_Tool.Range(anE, out double f, out double l);
        Assert.Equal(7.0 - 2 * Math.PI, f, 1e-12);
        Assert.Equal(8.0 - 2 * Math.PI, l, 1e-12);
    }

    [Fact]
    public void Point_off_the_curve_fails_its_projection()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        var aME = new BRepLib_MakeEdge(aCirc, new gp_Pnt(0, 2.1, 0), new gp_Pnt(-2, 0, 0));
        Assert.False(aME.IsDone());
        Assert.Equal(BRepLib_PointProjectionFailed, aME.Error());
    }

    [Fact]
    public void Ellipse_hyperbola_and_parabola_arcs()
    {
        var anAx = new gp_Ax2(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1));
        TopoDS_Edge anEl = new BRepLib_MakeEdge(new gp_Elips(anAx, 3, 1), 0.0, 1.0).Edge();
        Assert.IsType<Geom_Ellipse>(BRep_Tool.Curve(anEl, out _, out _));
        TopoDS_Edge aHy = new BRepLib_MakeEdge(new gp_Hypr(anAx, 3, 1), -1.0, 1.0).Edge();
        Assert.IsType<Geom_Hyperbola>(BRep_Tool.Curve(aHy, out _, out _));
        TopoDS_Edge aPa = new BRepLib_MakeEdge(new gp_Parab(anAx, 0.5), -2.0, 2.0).Edge();
        Assert.IsType<Geom_Parabola>(BRep_Tool.Curve(aPa, out double f, out double l));
        Assert.Equal(-2.0, f);
        Assert.Equal(2.0, l);
        TopExp.Vertices(aHy, out TopoDS_Vertex aV1, out _);
        AssertPoint(new Geom_Hyperbola(new gp_Hypr(anAx, 3, 1)).Value(-1.0), BRep_Tool.Pnt(aV1));
    }

    private static Geom_BSplineCurve BSpline()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 2, 0);
        aPoles[3] = new gp_Pnt(3, 2, 1);
        aPoles[4] = new gp_Pnt(4, 0, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void BSpline_edge_with_a_range_and_out_of_range_parameters()
    {
        Geom_BSplineCurve aC = BSpline();
        var aME = new BRepLib_MakeEdge(aC, 0.25, 0.75);
        TopoDS_Edge anE = aME.Edge();
        Assert.Same(aC, BRep_Tool.Curve(anE, out double f, out double l));
        Assert.True(aC.IsFrozen()); // frozen when the edge took it
        Assert.Equal(0.25, f);
        Assert.Equal(0.75, l);
        AssertPoint(aC.Value(0.25), BRep_Tool.Pnt(aME.Vertex1()));

        var aBad = new BRepLib_MakeEdge(BSpline(), 0.5, 1.5);
        Assert.Equal(BRepLib_ParameterOutOfRange, aBad.Error());
        Assert.False(aBad.IsDone());
    }

    [Fact]
    public void Trimmed_curves_are_replaced_by_their_basis()
    {
        var aT = new Geom_TrimmedCurve(new Geom_TrimmedCurve(BSpline(), 0.1, 0.9), 0.2, 0.8);
        TopoDS_Edge anE = new BRepLib_MakeEdge(aT).Edge();
        // Geom_TrimmedCurve keeps a copy of its basis; the edge gets that B-spline, not a trimmed curve
        Geom_Curve? anEC = BRep_Tool.Curve(anE, out double f, out double l);
        Assert.Same(aT.BasisCurve(), anEC);
        Assert.IsType<Geom_BSplineCurve>(anEC);
        Assert.Equal(0.2, f, 1e-15);
        Assert.Equal(0.8, l, 1e-15);
    }

    [Fact]
    public void Vertex_away_from_its_parameter_fails()
    {
        TopoDS_Vertex aV1 = V(0, 0, 0), aV2 = V(5, 1, 0);
        var aME = new BRepLib_MakeEdge(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), aV1, aV2, 0.0, 5.0);
        Assert.Equal(BRepLib_DifferentsPointAndParameter, aME.Error());
        Assert.False(aME.IsDone());
    }

    [Fact]
    public void Vertex_at_an_infinite_parameter_fails()
    {
        TopoDS_Vertex aV1 = V(0, 0, 0);
        var aME = new BRepLib_MakeEdge(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), aV1, new TopoDS_Vertex(),
                                       -Precision.Infinite(), 0.0);
        Assert.Equal(BRepLib_PointWithInfiniteParameter, aME.Error());
    }

    [Fact]
    public void Different_vertices_on_a_closed_curve_fail()
    {
        var aCirc = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
        TopoDS_Vertex aV1 = V(1, 0, 0), aV2 = V(1, 0, 0);
        var aME = new BRepLib_MakeEdge(aCirc, aV1, aV2, 0.0, 2 * Math.PI);
        Assert.Equal(BRepLib_DifferentPointsOnClosedCurve, aME.Error());
    }

    [Fact]
    public void Tiny_closed_curve_with_one_vertex_is_degenerated()
    {
        var aCirc = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1e-8);
        TopoDS_Vertex aV = V(1e-8, 0, 0);
        var aME = new BRepLib_MakeEdge(aCirc, aV, aV, 0.0, 2 * Math.PI);
        Assert.True(aME.IsDone());
        Assert.True(BRep_Tool.Degenerated(aME.Edge()));
    }

    [Fact]
    public void Edge_on_a_surface_has_a_pcurve_and_no_3d_curve()
    {
        var aPlane = new Geom_Plane(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1));
        var aL = new Geom2d_Line(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0));
        var aME = new BRepLib_MakeEdge(aL, aPlane, 0.0, 2.0);
        TopoDS_Edge anE = aME.Edge();
        Assert.Null(BRep_Tool.Curve(anE, out _, out _));
        Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(anE, aPlane, new TopLoc_Location(), out double f, out double l);
        Assert.Same(aL, aPC);
        Assert.Equal(0.0, f);
        Assert.Equal(2.0, l);
        Assert.Equal(new gp_Pnt(3, 1, 1), BRep_Tool.Pnt(aME.Vertex2()));
        Assert.Equal(TopAbs_FORWARD, anE.Orientation());

        // reversed parameters reverse the edge
        TopoDS_Edge aR = new BRepLib_MakeEdge(new Geom2d_Line(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0)), aPlane, 2.0, 0.0).Edge();
        Assert.Equal(TopAbs_REVERSED, aR.Orientation());
    }

    [Fact]
    public void Init_after_a_public_read_makes_a_new_edge()
    {
        var aME = new BRepLib_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0));
        TopoDS_Edge aFirst = aME.Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        aME.Init(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 0.0, 2.0);
        TopoDS_Edge aSecond = aME.Edge();
        Assert.False(aFirst.IsSame(aSecond));
        aSnap.AssertUnchanged(aFirst);
    }

    // ---------------------------------------------------------------- 2D edges

    [Fact]
    public void Edge2d_between_points_is_a_pcurve_on_the_default_plane()
    {
        var aME = new BRepLib_MakeEdge2d(new gp_Pnt2d(1, 1), new gp_Pnt2d(4, 5));
        Assert.True(aME.IsDone());
        Assert.Equal(BRepLib_EdgeDone, aME.Error());
        TopoDS_Edge anE = aME.Edge();
        Assert.True(anE.TShape()!.IsFrozen());
        Geom2d_Curve? aPC = BRep_Tool.CurveOnSurface(anE, BRepLib.Plane(), new TopLoc_Location(), out double f, out double l);
        Assert.IsType<Geom2d_Line>(aPC);
        Assert.Equal(0.0, f);
        Assert.Equal(5.0, l);
        Assert.Equal(new gp_Pnt(4, 5, 0), BRep_Tool.Pnt(aME.Vertex2()));
        Assert.Equal(Precision.Confusion(), BRep_Tool.Tolerance(aME.Vertex1()));
    }

    [Fact]
    public void Edge2d_whole_circle_is_closed()
    {
        var aME = new BRepLib_MakeEdge2d(new gp_Circ2d(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.5));
        Assert.True(aME.IsDone());
        Assert.True(aME.Vertex1().IsSame(aME.Vertex2()));
        Assert.Equal(new gp_Pnt(1.5, 0, 0), BRep_Tool.Pnt(aME.Vertex1()));
    }

    [Fact]
    public void Edge2d_through_identical_points_fails()
    {
        var aME = new BRepLib_MakeEdge2d(new gp_Pnt2d(1, 1), new gp_Pnt2d(1, 1));
        Assert.Equal(BRepLib_LineThroughIdenticPoints, aME.Error());
        Assert.False(aME.IsDone());
    }

    // ---------------------------------------------------------------- wires

    [Fact]
    public void Wire_of_edges_sharing_vertices_keeps_them()
    {
        TopoDS_Vertex a = V(0, 0, 0), b = V(1, 0, 0), c = V(1, 1, 0);
        TopoDS_Edge e1 = E(a, b), e2 = E(b, c), e3 = E(c, a);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2);
        var aMW = new BRepLib_MakeWire(e1, e2);
        Assert.Equal(BRepLib_WireDone, aMW.Error());
        TopoDS_Wire aW = aMW.Wire();
        Assert.True(aW.TShape()!.IsFrozen());
        Assert.False(aW.Closed());
        Assert.True(aMW.Edge().IsSame(e2));
        Assert.True(aMW.Vertex().IsSame(b));
        aMW.Add(e3);
        TopoDS_Wire aClosed = aMW.Wire();
        Assert.True(aClosed.Closed());
        Assert.Equal(3, Count(aClosed, TopAbs_EDGE));
        Assert.False(aW.Closed()); // the published wire didn't change
        Assert.Equal(2, Count(aW, TopAbs_EDGE));
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        Assert.Null(aMW.InternalHistory());
    }

    [Fact]
    public void Disconnected_edge_is_refused()
    {
        var aMW = new BRepLib_MakeWire(E(0, 0, 0, 1, 0, 0));
        aMW.Add(E(5, 0, 0, 6, 0, 0));
        Assert.Equal(BRepLib_DisconnectedWire, aMW.Error());
        Assert.False(aMW.IsDone());
        Assert.Throws<StdFail_NotDone>(() => aMW.Wire());
    }

    [Fact]
    public void Empty_maker_reports_an_empty_wire()
    {
        var aMW = new BRepLib_MakeWire();
        Assert.Equal(BRepLib_EmptyWire, aMW.Error());
        Assert.False(aMW.IsDone());
    }

    [Fact]
    public void Edges_with_coincident_vertices_are_connected_by_copies()
    {
        TopoDS_Edge e1 = E(0, 0, 0, 1, 0, 0), e2 = E(1, 0, 0, 1, 1, 0), e3 = E(1, 1, 0, 0, 0, 0);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2), s3 = ShapeSnapshot.Of(e3);
        var aMW = new BRepLib_MakeWire(e1, e2, e3);
        Assert.Equal(BRepLib_WireDone, aMW.Error());
        TopoDS_Wire aW = aMW.Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, TopAbs_EDGE));
        Assert.Equal(3, Count(aW, TopAbs_VERTEX));
        var anExp = new TopExp_Explorer(aW, TopAbs_EDGE);
        Assert.True(anExp.Current().IsSame(e1)); // the first edge is the caller's
        anExp.Next();
        Assert.False(anExp.Current().IsSame(e2)); // the others are copies on the wire's vertices
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        s3.AssertUnchanged(e3);
        Assert.Null(aMW.InternalHistory()); // the merges changed no frozen vertex
    }

    [Fact]
    public void Merging_onto_a_frozen_vertex_that_must_move_copies_the_wire_for_write()
    {
        TopoDS_Edge e1 = E(0, 0, 0, 1, 0, 0);
        TopoDS_Edge e2 = E(1 + 5e-8, 0, 0, 1, 1, 0);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2);
        var aMW = new BRepLib_MakeWire(e1, e2);
        Assert.Equal(BRepLib_WireDone, aMW.Error());
        TopoDS_Wire aW = aMW.Wire();
        Assert.Equal(2, Count(aW, TopAbs_EDGE));
        Assert.Equal(3, Count(aW, TopAbs_VERTEX));
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        BRepTools_History? aH = aMW.InternalHistory();
        Assert.NotNull(aH);
        TopoDS_Shape aE1Copy = aH!.Modified(e1)[0];
        var anExp = new TopExp_Explorer(aW, TopAbs_EDGE);
        Assert.True(anExp.Current().IsSame(aE1Copy));

        // the shared vertex moved half way and grew to cover both
        TopExp.Vertices(TopoDS.Edge(anExp.Current()), out _, out TopoDS_Vertex aMid, true);
        Assert.Equal(1.25e-7, BRep_Tool.Tolerance(aMid), 1e-15);
        AssertPoint(new gp_Pnt(1 + 2.5e-8, 0, 0), BRep_Tool.Pnt(aMid), 1e-15);
    }

    [Fact]
    public void Edge_meeting_the_wire_at_an_inner_vertex_builds_a_non_manifold_wire()
    {
        TopoDS_Vertex a = V(0, 0, 0), b = V(1, 0, 0), c = V(2, 0, 0), d = V(1, 1, 0);
        var aMW = new BRepLib_MakeWire(E(a, b), E(b, c));
        aMW.Add(E(b, d));
        Assert.True(aMW.IsDone());
        TopoDS_Wire aW = aMW.Wire();
        Assert.Equal(3, Count(aW, TopAbs_EDGE));
        Assert.False(aW.Closed());
    }

    [Fact]
    public void List_of_edges_in_any_order_is_connected()
    {
        // a square whose edges have their own coincident vertices, listed out of order
        TopoDS_Edge e1 = E(0, 0, 0, 1, 0, 0), e2 = E(1, 0, 0, 1, 1, 0), e3 = E(1, 1, 0, 0, 1, 0), e4 = E(0, 1, 0, 0, 0, 0);
        var aL = new NCollection_List<TopoDS_Shape>();
        aL.Append(e3);
        aL.Append(e1);
        aL.Append(e4);
        aL.Append(e2);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1);
        var aMW = new BRepLib_MakeWire();
        aMW.Add(aL);
        Assert.True(aMW.IsDone());
        TopoDS_Wire aW = aMW.Wire();
        Assert.Equal(4, Count(aW, TopAbs_EDGE));
        Assert.Equal(4, Count(aW, TopAbs_VERTEX));
        Assert.True(aW.Closed());
        s1.AssertUnchanged(e1);

        // a list that can't be connected
        var aBad = new NCollection_List<TopoDS_Shape>();
        aBad.Append(E(0, 0, 0, 1, 0, 0));
        aBad.Append(E(3, 0, 0, 4, 0, 0));
        var aMW2 = new BRepLib_MakeWire();
        aMW2.Add(aBad);
        Assert.False(aMW2.IsDone());
        Assert.Equal(BRepLib_DisconnectedWire, aMW2.Error());
    }

    [Fact]
    public void List_added_to_a_frozen_wire_grows_its_vertex_on_a_copy()
    {
        // the wire's end vertex (tolerance 1e-7) must grow to reach a list edge's vertex 1.5e-7 away (tolerance 1e-7)
        TopoDS_Wire aW0 = new BRepLib_MakeWire(E(0, 0, 0, 1, 0, 0)).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aW0);
        var aL = new NCollection_List<TopoDS_Shape>();
        aL.Append(E(1 + 1.5e-7, 0, 0, 2, 0, 0));
        var aMW = new BRepLib_MakeWire(aW0);
        aMW.Add(aL);
        Assert.True(aMW.IsDone());
        TopoDS_Wire aW = aMW.Wire();
        Assert.Equal(2, Count(aW, TopAbs_EDGE));
        Assert.Equal(3, Count(aW, TopAbs_VERTEX));
        Assert.NotNull(aMW.InternalHistory());
        aSnap.AssertUnchanged(aW0);
    }

    [Fact]
    public void Wire_of_a_frozen_wire_and_an_edge()
    {
        TopoDS_Wire aW0 = new BRepLib_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0)).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aW0);
        var aMW = new BRepLib_MakeWire(aW0, E(1, 1, 0, 0, 0, 0));
        Assert.True(aMW.IsDone());
        Assert.True(aMW.Wire().Closed());
        aSnap.AssertUnchanged(aW0);
    }

    // ---------------------------------------------------------------- polygons

    [Fact]
    public void Polygon_open_then_closed()
    {
        var aMP = new BRepLib_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(2, 2, 0));
        Assert.True(aMP.IsDone());
        TopoDS_Wire aOpen = aMP.Wire();
        Assert.False(aOpen.Closed());
        Assert.Equal(2, Count(aOpen, TopAbs_EDGE));
        Assert.Equal(new gp_Pnt(0, 0, 0), BRep_Tool.Pnt(aMP.FirstVertex()));
        Assert.Equal(new gp_Pnt(2, 2, 0), BRep_Tool.Pnt(aMP.LastVertex()));
        aMP.Close();
        TopoDS_Wire aClosed = aMP.Wire();
        Assert.True(aClosed.Closed());
        Assert.Equal(3, Count(aClosed, TopAbs_EDGE));
        Assert.False(aOpen.Closed()); // published before: unchanged
        Assert.Equal(2, Count(aOpen, TopAbs_EDGE));
        Assert.True(aMP.Edge().TShape()!.IsFrozen());
    }

    [Fact]
    public void Polygon_of_four_points_closed()
    {
        TopoDS_Wire aW = new BRepLib_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0),
                                                 new gp_Pnt(0, 1, 0), true).Wire();
        Assert.True(aW.Closed());
        Assert.Equal(4, Count(aW, TopAbs_EDGE));
        Assert.Equal(4, Count(aW, TopAbs_VERTEX));
        Assert.Equal(Precision.Confusion(), BRep_Tool.Tolerance(TopoDS.Edge(new TopExp_Explorer(aW, TopAbs_EDGE).Current())));
    }

    [Fact]
    public void Polygon_skips_a_repeated_point_and_closes_on_the_first()
    {
        var aMP = new BRepLib_MakePolygon();
        aMP.Add(new gp_Pnt(0, 0, 0));
        aMP.Add(new gp_Pnt(1, 0, 0));
        Assert.True(aMP.Added());
        aMP.Add(new gp_Pnt(1, 0, 0));
        Assert.False(aMP.Added());
        aMP.Add(new gp_Pnt(1, 1, 0));
        aMP.Add(new gp_Pnt(0, 0, 0));
        Assert.True(aMP.Added());
        TopoDS_Wire aW = aMP.Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, TopAbs_EDGE));
        Assert.True(aMP.LastVertex().IsSame(aMP.FirstVertex()));
    }

    [Fact]
    public void Polygon_of_frozen_vertices_shares_them()
    {
        TopoDS_Vertex a = V(0, 0, 0), b = V(3, 0, 0), c = V(0, 4, 0);
        ShapeSnapshot aSa = ShapeSnapshot.Of(a);
        TopoDS_Wire aW = new BRepLib_MakePolygon(a, b, c, true).Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, TopAbs_VERTEX));
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aW, TopAbs_VERTEX, aMap);
        Assert.True(aMap.Contains(a));
        aSa.AssertUnchanged(a);
    }

    // ---------------------------------------------------------------- faces

    [Fact]
    public void Face_on_a_whole_plane_has_no_wire()
    {
        var aMF = new BRepLib_MakeFace(new gp_Pln(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1)));
        Assert.True(aMF.IsDone());
        Assert.Equal(BRepLib_FaceDone, aMF.Error());
        TopoDS_Face aF = aMF.Face();
        Assert.True(aF.TShape()!.IsFrozen());
        Assert.Equal(0, Count(aF, TopAbs_WIRE));
        Assert.True(BRep_Tool.NaturalRestriction(aF));
        Assert.IsType<Geom_Plane>(BRep_Tool.Surface(aF));
    }

    [Fact]
    public void Face_on_a_bounded_plane_has_a_wire_of_four_iso_edges()
    {
        var aPln = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        TopoDS_Face aF = new BRepLib_MakeFace(aPln, 0, 2, 0, 3).Face();
        Assert.Equal(1, Count(aF, TopAbs_WIRE));
        Assert.Equal(4, Count(aF, TopAbs_EDGE));
        Assert.Equal(4, Count(aF, TopAbs_VERTEX));
        Assert.False(BRep_Tool.NaturalRestriction(aF)); // OCCT: only Init(S, Bound, ...) sets it
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            Assert.NotNull(BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
            Assert.NotNull(BRep_Tool.Curve(anE, out _, out _));
        }
        var aFC = new BRepTopAdaptor_FClass2d(aF, 0);
        Assert.Equal(TopAbs_OUT, aFC.PerformInfinitePoint());
    }

    [Fact]
    public void Face_on_a_whole_cylinder_band_has_a_seam()
    {
        var aCyl = new gp_Cylinder(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        TopoDS_Face aF = new BRepLib_MakeFace(aCyl, 0, 2 * Math.PI, 0, 5).Face();
        Assert.Equal(3, Count(aF, TopAbs_EDGE));
        Assert.Equal(4, CountOccurrences(aF, TopAbs_EDGE));
        int aSeams = 0;
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.IsClosed(TopoDS.Edge(anExp.Current()), aF))
            {
                aSeams++;
            }
        }
        Assert.Equal(2, aSeams); // the seam, met twice
    }

    [Fact]
    public void Face_on_a_whole_sphere_has_degenerated_poles()
    {
        TopoDS_Face aF = new BRepLib_MakeFace(new gp_Sphere(new gp_Ax3(new gp_Pnt(1, 1, 1), new gp_Dir(0, 0, 1)), 3.0)).Face();
        int aDegenerated = 0;
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Degenerated(TopoDS.Edge(anExp.Current())))
            {
                aDegenerated++;
                Assert.Null(BRep_Tool.Curve(TopoDS.Edge(anExp.Current()), out _, out _));
            }
        }
        Assert.Equal(2, aDegenerated);
        Assert.Equal(2, Count(aF, TopAbs_VERTEX));
        Assert.True(BRep_Tool.NaturalRestriction(aF));
    }

    [Fact]
    public void Face_on_a_cone_from_its_apex_is_degenerated_there()
    {
        double aSemi = Math.PI / 6;
        var aCone = new gp_Cone(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), aSemi, 1.0);
        double vApex = -1.0 / Math.Sin(aSemi);
        TopoDS_Face aF = new BRepLib_MakeFace(aCone, 0, 2 * Math.PI, vApex, 1.0).Face();
        int aDegenerated = 0;
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Degenerated(TopoDS.Edge(anExp.Current())))
            {
                aDegenerated++;
            }
        }
        Assert.Equal(1, aDegenerated);
    }

    [Fact]
    public void Face_parameters_out_of_range()
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0, 0, 0);
        aPoles[1, 2] = new gp_Pnt(0, 1, 0);
        aPoles[2, 1] = new gp_Pnt(1, 0, 0);
        aPoles[2, 2] = new gp_Pnt(1, 1, 1);
        var aS = new Geom_BezierSurface(aPoles);
        var aBad = new BRepLib_MakeFace(aS, 2.0, 0.5, 0, 1, Precision.Confusion());
        Assert.Equal(BRepLib_ParametersOutOfRange, aBad.Error());
        Assert.False(aBad.IsDone());
        var aGood = new BRepLib_MakeFace(aS, 0.25, 0.75, 0, 1, Precision.Confusion());
        Assert.True(aGood.IsDone());
        Assert.Equal(4, Count(aGood.Face(), TopAbs_EDGE));
    }

    private static TopoDS_Wire Square(double x0, double y0, double s, bool theCcw)
    {
        var aP = new[] { new gp_Pnt(x0, y0, 0), new gp_Pnt(x0 + s, y0, 0), new gp_Pnt(x0 + s, y0 + s, 0), new gp_Pnt(x0, y0 + s, 0) };
        if (!theCcw)
        {
            Array.Reverse(aP);
        }
        return new BRepLib_MakePolygon(aP[0], aP[1], aP[2], aP[3], true).Wire();
    }

    [Fact]
    public void Face_on_a_plane_from_a_wire_is_oriented_to_bound_a_finite_area()
    {
        var aPln = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        foreach (bool aCcw in new[] { true, false })
        {
            TopoDS_Wire aW = Square(0, 0, 1, aCcw);
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aW);
            var aMF = new BRepLib_MakeFace(aPln, aW);
            Assert.True(aMF.IsDone());
            TopoDS_Face aF = aMF.Face();
            Assert.Equal(TopAbs_OUT, new BRepTopAdaptor_FClass2d(aF, 0).PerformInfinitePoint());
            var anIt = new TopoDS_Iterator(aF);
            Assert.True(anIt.Value().IsSame(aW));
            Assert.Equal(aCcw ? TopAbs_FORWARD : TopAbs_REVERSED, anIt.Value().Orientation());
            aSnap.AssertUnchanged(aW);
        }

        // Inside false keeps the wire as given
        TopoDS_Face aKept = new BRepLib_MakeFace(aPln, Square(0, 0, 1, false), false).Face();
        Assert.Equal(TopAbs_FORWARD, new TopoDS_Iterator(aKept).Value().Orientation());
    }

    [Fact]
    public void Face_with_a_hole_and_Add_after_a_public_read()
    {
        var aPln = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aMF = new BRepLib_MakeFace(aPln, Square(0, 0, 4, true));
        TopoDS_Face aOuterOnly = aMF.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aOuterOnly);
        aMF.Add(Square(1, 1, 1, false));
        TopoDS_Face aF = aMF.Face();
        Assert.Equal(2, Count(aF, TopAbs_WIRE));
        Assert.False(BRep_Tool.NaturalRestriction(aF));
        aSnap.AssertUnchanged(aOuterOnly);
        Assert.Equal(1, Count(aOuterOnly, TopAbs_WIRE));

        var aFC = new BRepClass_FaceClassifier(aF, new gp_Pnt2d(1.5, 1.5), 1e-7);
        Assert.Equal(TopAbs_OUT, aFC.State());
        aFC.Perform(aF, new gp_Pnt2d(0.5, 0.5), 1e-7);
        Assert.Equal(TopAbs_IN, aFC.State());
    }

    [Fact]
    public void Face_loaded_from_a_face_is_a_new_face_with_the_same_wires()
    {
        TopoDS_Face aF0 = new BRepLib_MakeFace(new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), Square(0, 0, 4, true)).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aF0);
        TopoDS_Wire aHole = Square(1, 1, 1, false);
        TopoDS_Face aF = new BRepLib_MakeFace(aF0, aHole).Face();
        Assert.False(aF.IsSame(aF0));
        Assert.Same(BRep_Tool.Surface(aF0), BRep_Tool.Surface(aF));
        Assert.Equal(2, Count(aF, TopAbs_WIRE));
        aSnap.AssertUnchanged(aF0);
    }

    [Fact]
    public void IsDegenerated_on_circles_and_poles()
    {
        Assert.True(BRepLib_MakeFace.IsDegenerated(new Geom_Circle(new gp_Ax2(), 1e-9), 1e-7, out double aTol));
        Assert.Equal(Precision.Confusion(), aTol);
        Assert.False(BRepLib_MakeFace.IsDegenerated(new Geom_Circle(new gp_Ax2(), 1.0), 1e-7, out _));
        Assert.False(BRepLib_MakeFace.IsDegenerated(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 1e-7, out _));
    }

    // ---------------------------------------------------------------- faces through BRepLib's statics

    [Fact]
    public void Face_from_a_frozen_planar_wire_is_built_on_a_copy_for_write()
    {
        TopoDS_Wire aW = Square(1, 2, 3, true);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aW);
        var aMF = new BRepLib_MakeFace(aW, true);
        Assert.True(aMF.IsDone());
        Assert.Equal(BRepLib_FaceDone, aMF.Error());
        TopoDS_Face aF = aMF.Face();
        Assert.True(aF.TShape()!.IsFrozen());
        aSnap.AssertUnchanged(aW);

        // the face holds a copy of the wire, recorded in the history
        BRepTools_History? aH = aMF.InternalHistory();
        Assert.NotNull(aH);
        TopoDS_Shape aFaceWire = new TopoDS_Iterator(aF).Value();
        Assert.False(aFaceWire.IsSame(aW));
        for (var anExp = new TopExp_Explorer(aW, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.Single(aH!.Modified(anExp.Current()));
        }
        Assert.Equal(4, Count(aF, TopAbs_EDGE));
        var aPlane = Assert.IsType<Geom_Plane>(BRep_Tool.Surface(aF));
        Assert.Equal(1.0, Math.Abs(aPlane.Position().Direction().Z()), 1e-12);
        Assert.Equal(TopAbs_OUT, new BRepTopAdaptor_FClass2d(aF, 0).PerformInfinitePoint());
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(BRep_Tool.SameParameter(TopoDS.Edge(anExp.Current())));
        }
    }

    [Fact]
    public void Face_from_a_wire_that_is_not_planar_fails()
    {
        TopoDS_Wire aW = new BRepLib_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0.5),
                                                 new gp_Pnt(0, 1, 0), true).Wire();
        var aMF = new BRepLib_MakeFace(aW);
        Assert.False(aMF.IsDone());
        Assert.Equal(BRepLib_NotPlanar, aMF.Error());
    }

    [Fact]
    public void Face_on_an_offset_surface_is_made_same_parameter()
    {
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        var anOffset = new Geom_OffsetSurface(aCyl, 0.5);
        var aMF = new BRepLib_MakeFace(anOffset, 0, Math.PI, 0, 3, Precision.Confusion());
        Assert.True(aMF.IsDone());
        TopoDS_Face aF = aMF.Face();
        Assert.Equal(4, Count(aF, TopAbs_EDGE));
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            Assert.True(BRep_Tool.SameParameter(anE));
            Assert.True(BRep_Tool.Tolerance(anE) < 1e-5);
        }
    }

    // ---------------------------------------------------------------- shells

    private static Geom_BSplineSurface SurfaceWithAC1Knot()
    {
        // cubic in U with a double interior knot (C1 there), linear in V
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 6, 1, 2);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                aPoles[i, j] = new gp_Pnt(i, 3 * j, (i % 2) * 0.5 + 0.1 * j);
            }
        }
        var aUKnots = new NCollection_Array1<double>(1, 3);
        aUKnots[1] = 0;
        aUKnots[2] = 1;
        aUKnots[3] = 2;
        var aUMults = new NCollection_Array1<int>(1, 3);
        aUMults[1] = 4;
        aUMults[2] = 2;
        aUMults[3] = 4;
        var aVKnots = new NCollection_Array1<double>(1, 2);
        aVKnots[1] = 0;
        aVKnots[2] = 1;
        var aVMults = new NCollection_Array1<int>(1, 2);
        aVMults[1] = 2;
        aVMults[2] = 2;
        return new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 1);
    }

    [Fact]
    public void Shell_of_a_BSpline_surface_is_split_at_its_C1_knot()
    {
        Geom_BSplineSurface aS = SurfaceWithAC1Knot();
        var aMS = new BRepLib_MakeShell(aS);
        Assert.True(aMS.IsDone());
        Assert.Equal(BRepLib_ShellDone, aMS.Error());
        TopoDS_Shell aSh = aMS.Shell();
        Assert.True(aSh.TShape()!.IsFrozen());
        Assert.Equal(2, Count(aSh, TopAbs_FACE));
        Assert.Equal(7, Count(aSh, TopAbs_EDGE)); // the shared edge once
        Assert.Equal(6, Count(aSh, TopAbs_VERTEX));
        Assert.False(aSh.Closed());
        for (var anExp = new TopExp_Explorer(aSh, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.NotNull(BRep_Tool.Curve(TopoDS.Edge(anExp.Current()), out _, out _)); // BuildCurves3d
        }
        // every face is on a copy of the whole surface
        for (var anExp = new TopExp_Explorer(aSh, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aFS = Assert.IsType<Geom_BSplineSurface>(BRep_Tool.Surface(TopoDS.Face(anExp.Current())));
            Assert.NotSame(aS, aFS);
            Assert.Equal(3, aFS.NbUKnots());
        }

        // with Segment, each face has its own segment
        TopoDS_Shell aSeg = new BRepLib_MakeShell(aS, true).Shell();
        for (var anExp = new TopExp_Explorer(aSeg, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aFS = Assert.IsType<Geom_BSplineSurface>(BRep_Tool.Surface(TopoDS.Face(anExp.Current())));
            Assert.Equal(2, aFS.NbUKnots());
        }
        Assert.False(aS.IsFrozen()); // the caller's surface is only copied
    }

    [Fact]
    public void Shell_between_parameters_and_the_empty_maker()
    {
        var aMS = new BRepLib_MakeShell(SurfaceWithAC1Knot(), 0.5, 1.5, 0.2, 0.8);
        Assert.True(aMS.IsDone());
        Assert.Equal(2, Count(aMS.Shell(), TopAbs_FACE));
        Assert.Equal(BRepLib_EmptyShell, new BRepLib_MakeShell().Error());
    }

    // ---------------------------------------------------------------- solids

    [Fact]
    public void Solid_from_a_shell_and_Add_after_a_public_read()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        TopoDS_Shell aShell = TopoDS.Shell(new TopoDS_Iterator(aBox).Value());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        var aMS = new BRepLib_MakeSolid(aShell);
        Assert.True(aMS.IsDone());
        TopoDS_Solid aS1 = aMS.Solid();
        Assert.True(aS1.TShape()!.IsFrozen());
        Assert.Equal(1, Count(aS1, TopAbs_SHELL));
        TopoDS_Shell aShell2 = TopoDS.Shell(new TopoDS_Iterator(TestShapes.Frozen(TestShapes.Box(0.5, 0.5, 0.5))).Value());
        aMS.Add(aShell2);
        TopoDS_Solid aS2 = aMS.Solid();
        Assert.Equal(2, Count(aS2, TopAbs_SHELL));
        Assert.Equal(1, Count(aS1, TopAbs_SHELL));
        aSnap.AssertUnchanged(aShell);
        Assert.Equal(BRepLib_Preserved, aMS.FaceStatus(TopoDS.Face(new TopExp_Explorer(aS2, TopAbs_FACE).Current())));
    }

    [Fact]
    public void Solid_from_shells_and_solids()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shell aSh = TopoDS.Shell(new TopoDS_Iterator(aBox).Value());
        Assert.Equal(2, CountOccurrences(new BRepLib_MakeSolid(aSh, TopoDS.Shell(aSh.Reversed())).Solid(), TopAbs_SHELL));
        Assert.Equal(1, Count(new BRepLib_MakeSolid(aBox).Solid(), TopAbs_SHELL));
        TopoDS_Solid aTwo = new BRepLib_MakeSolid(aBox, TopoDS.Shell(new TopoDS_Iterator(TestShapes.Frozen(TestShapes.Box(2, 2, 2))).Value())).Solid();
        Assert.Equal(2, Count(aTwo, TopAbs_SHELL));
        Assert.Equal(0, Count(new BRepLib_MakeSolid().Solid(), TopAbs_SHELL));
    }

    [Fact]
    public void Solid_from_a_compsolid_drops_the_shared_faces()
    {
        // two boxes sharing a face: a compsolid of two solids whose shells both hold that face
        var aB = new BRep_Builder();
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        TopoDS_Shell aShell = TopoDS.Shell(new TopoDS_Iterator(aBox).Value());
        TopoDS_Face aShared = TopoDS.Face(new TopoDS_Iterator(aShell).Value());
        TopoDS_Shell aOther = new();
        aB.MakeShell(ref aOther);
        aB.Add(aOther, aShared.Reversed());
        TopoDS_Solid aSolid2 = new();
        aB.MakeSolid(ref aSolid2);
        aB.Add(aSolid2, aOther);
        TopoDS_CompSolid aCS = new();
        aB.MakeCompSolid(ref aCS);
        aB.Add(aCS, aBox);
        aB.Add(aCS, aSolid2);
        aCS = TopoDS.CompSolid(TestShapes.Frozen(aCS));

        var aMS = new BRepLib_MakeSolid(aCS);
        TopoDS_Solid aS = aMS.Solid();
        Assert.Equal(1, Count(aS, TopAbs_SHELL));
        Assert.Equal(5, Count(aS, TopAbs_FACE));
        Assert.Equal(BRepLib_Deleted, aMS.FaceStatus(aShared));
        Assert.True(TopoDS.Shell(new TopoDS_Iterator(aS).Value()).Closed() == false);
    }

    // ---------------------------------------------------------------- FindSurface

    [Fact]
    public void FindSurface_fits_a_plane_to_a_planar_polygon_and_orients_it_by_the_wire()
    {
        foreach (bool aCcw in new[] { true, false })
        {
            TopoDS_Wire aW = Square(0, 0, 2, aCcw);
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aW);
            var aFS = new BRepLib_FindSurface(aW);
            Assert.True(aFS.Found());
            Assert.False(aFS.Existed());
            Assert.True(aFS.Location().IsIdentity());
            Assert.Equal(Precision.Confusion(), aFS.Tolerance());
            Assert.True(aFS.ToleranceReached() < 1e-15);
            var aPlane = Assert.IsType<Geom_Plane>(aFS.Surface());
            Assert.True(aPlane.IsFrozen());
            gp_Dir aN = aPlane.Position().Direction();
            Assert.Equal(aCcw ? 1.0 : -1.0, aN.Z(), 1e-12);
            AssertPoint(new gp_Pnt(1, 1, 0), aPlane.Position().Location(), 1e-12);
            aSnap.AssertUnchanged(aW);
        }
    }

    [Fact]
    public void FindSurface_rejects_a_non_planar_polygon_unless_the_tolerance_allows_it()
    {
        TopoDS_Wire aW = new BRepLib_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0.1),
                                                 new gp_Pnt(0, 1, 0), true).Wire();
        var aFS = new BRepLib_FindSurface(aW);
        Assert.False(aFS.Found());
        Assert.True(aFS.ToleranceReached() > 0.01);
        var aLoose = new BRepLib_FindSurface(aW, 0.1);
        Assert.True(aLoose.Found());
        Assert.Equal(0.1, aLoose.Tolerance());
    }

    [Fact]
    public void FindSurface_finds_the_existing_surface_of_the_pcurves()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Face aF = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        TopoDS_Wire aW = TopoDS.Wire(new TopoDS_Iterator(aF).Value());
        var aFS = new BRepLib_FindSurface(aW, -1, true);
        Assert.True(aFS.Found());
        Assert.True(aFS.Existed());
        Assert.Same(BRep_Tool.Surface(aF, out TopLoc_Location aLoc), aFS.Surface());
        Assert.True(aLoc.IsEqual(aFS.Location()));

        // the cylinder's wire has pcurves on a cylinder only
        TopoDS_Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        TopoDS_Wire aCW = TopoDS.Wire(new TopoDS_Iterator(aCyl).Value());
        Assert.IsType<Geom_CylindricalSurface>(new BRepLib_FindSurface(aCW).Surface());
        Assert.False(new BRepLib_FindSurface(aCW, -1, true).Existed());
    }

    [Fact]
    public void FindSurface_needs_edges_and_three_points()
    {
        Assert.False(new BRepLib_FindSurface(V(0, 0, 0)).Found());
        Assert.False(new BRepLib_FindSurface(E(0, 0, 0, 1, 0, 0)).Found()); // two points
        var aEmpty = new BRepLib_FindSurface();
        Assert.False(aEmpty.Found());
        Assert.Equal(0.0, aEmpty.Tolerance());
    }
}
