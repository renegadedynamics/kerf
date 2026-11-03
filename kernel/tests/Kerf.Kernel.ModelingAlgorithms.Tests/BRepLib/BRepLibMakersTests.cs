// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Construction.Library.LibraryEdgeError;
using static Kerf.Engine.Construction.Library.LibraryFaceError;
using static Kerf.Engine.Construction.Library.LibraryShapeModification;
using static Kerf.Engine.Construction.Library.LibraryShellError;
using static Kerf.Engine.Construction.Library.LibraryWireError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of the BRepLib makers: vertices, edges, 2D edges, wires, polygons, faces, shells and solids, with
/// frozen inputs (as callers hold them), frozen results from the public accessors and inputs left unchanged.
/// </summary>
public class BRepLibMakersTests
{
    private const double Conf = 1e-7;

    private static Vertex V(double x, double y, double z) => new LibraryMakeVertex(new Point3D(x, y, z)).Vertex();

    private static Edge E(in Vertex v1, in Vertex v2) => new LibraryMakeEdge(v1, v2).Edge();

    private static Edge E(double x1, double y1, double z1, double x2, double y2, double z2)
        => new LibraryMakeEdge(new Point3D(x1, y1, z1), new Point3D(x2, y2, z2)).Edge();

    private static int Count(in Shape S, ShapeType T)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(S, T, aMap);
        return aMap.Extent();
    }

    private static int CountOccurrences(in Shape S, ShapeType T)
    {
        int n = 0;
        for (var anExp = new Explorer(S, T); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    private static void AssertPoint(in Point3D theExpected, in Point3D theActual, double theTol = 1e-12)
        => Assert.True(theExpected.Distance(theActual) <= theTol, $"{theExpected} != {theActual}");

    // ---------------------------------------------------------------- vertices

    [Fact]
    public void MakeVertex_is_a_frozen_vertex_at_the_point_with_the_default_precision()
    {
        var aMV = new LibraryMakeVertex(new Point3D(1, 2, 3));
        Assert.True(aMV.IsDone);
        Vertex aV = aMV.Vertex();
        Assert.True(aV.TShape()!.IsFrozen);
        Assert.Equal(new Point3D(1, 2, 3), Tool.Pnt(aV));
        Assert.Equal(BrepLibrary.Precision(), Tool.Tolerance(aV));
        Assert.Equal(aV, (Vertex)aMV);
        Assert.Equal((Shape)aV, aMV.Shape());
    }

    [Fact]
    public void Internal_accessors_do_not_freeze()
    {
        var aME = new LibraryMakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Edge anE = aME.InternalEdge();
        Assert.False(anE.TShape()!.IsFrozen);
        Assert.False(aME.InternalVertex1().TShape()!.IsFrozen);
        Assert.True(aME.Vertex1().TShape()!.IsFrozen);
        Assert.True(anE.TShape()!.IsFrozen); // Vertex1 publishes the edge with it
    }

    // ---------------------------------------------------------------- edges

    [Fact]
    public void Edge_from_two_points_is_a_segment_parametrised_by_length()
    {
        var aME = new LibraryMakeEdge(new Point3D(1, 0, 0), new Point3D(1, 3, 4));
        Assert.True(aME.IsDone);
        Assert.Equal(LibraryEdgeError.EdgeDone, aME.Error());
        Edge anE = aME.Edge();
        Assert.True(anE.TShape()!.IsFrozen);
        Curve3D? aC = Tool.Curve(anE, out double f, out double l);
        Assert.IsType<LineCurve3D>(aC);
        Assert.Equal(0.0, f);
        Assert.Equal(5.0, l);
        Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
        Assert.Equal(new Point3D(1, 0, 0), Tool.Pnt(aV1));
        Assert.Equal(new Point3D(1, 3, 4), Tool.Pnt(aV2));
        Assert.True(aV1.IsSame(aME.Vertex1()));
        Assert.Equal(Forward, aME.Vertex1().Orientation());
        Assert.Equal(Reversed, aME.Vertex2().Orientation());
        Assert.False(anE.Closed());
        Assert.False(Tool.Degenerated(anE));
        Assert.Equal(BrepLibrary.Precision(), Tool.Tolerance(anE));
    }

    [Fact]
    public void Edge_between_frozen_vertices_shares_them_and_leaves_them_unchanged()
    {
        Vertex aV1 = V(0, 0, 0), aV2 = V(2, 0, 0);
        ShapeSnapshot aS1 = ShapeSnapshot.Of(aV1), aS2 = ShapeSnapshot.Of(aV2);
        Edge anE = E(aV1, aV2);
        Exploration.Vertices(anE, out Vertex aF, out Vertex aL);
        Assert.True(aF.IsSame(aV1));
        Assert.True(aL.IsSame(aV2));
        aS1.AssertUnchanged(aV1);
        aS2.AssertUnchanged(aV2);
    }

    [Fact]
    public void Edge_through_identical_points_fails()
    {
        var aME = new LibraryMakeEdge(new Point3D(1, 1, 1), new Point3D(1, 1, 1));
        Assert.False(aME.IsDone);
        Assert.Equal(LibraryEdgeError.LineThroughIdenticPoints, aME.Error());
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aME.Edge());
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aME.Vertex1());
    }

    [Fact]
    public void Edge_on_a_line_between_parameters_reorders_them()
    {
        var aL = new Line3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0));
        Edge anE = new LibraryMakeEdge(aL, 3.0, -2.0).Edge();
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(-2.0, f);
        Assert.Equal(3.0, l);
        Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
        Assert.Equal(new Point3D(0, -2, 0), Tool.Pnt(aV1));
        Assert.Equal(new Point3D(0, 3, 0), Tool.Pnt(aV2));
    }

    [Fact]
    public void Edge_on_a_whole_line_is_infinite_without_vertices()
    {
        var aME = new LibraryMakeEdge(new Line3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        Assert.True(aME.IsDone);
        Edge anE = aME.Edge();
        Assert.True(aME.Vertex1().IsNull);
        Assert.True(aME.Vertex2().IsNull);
        Assert.Equal(0, Count(anE, ShapeType.Vertex));
        Tool.Range(anE, out double f, out double l);
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f));
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l));
    }

    [Fact]
    public void Edge_on_a_whole_circle_is_closed_with_one_vertex()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var aME = new LibraryMakeEdge(aCirc);
        Edge anE = aME.Edge();
        Assert.True(anE.Closed());
        Assert.False(Tool.Degenerated(anE));
        Assert.Equal(1, Count(anE, ShapeType.Vertex));
        Assert.True(aME.Vertex1().IsSame(aME.Vertex2()));
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(2 * Math.PI, l);
        Assert.True(Tool.IsClosed(anE));
    }

    [Fact]
    public void Arc_of_circle_between_points_projects_them()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var aME = new LibraryMakeEdge(aCirc, new Point3D(0, 2, 0), new Point3D(-2, 0, 0));
        Assert.True(aME.IsDone);
        Tool.Range(aME.Edge(), out double f, out double l);
        Assert.Equal(Math.PI / 2, f, 1e-12);
        Assert.Equal(Math.PI, l, 1e-12);
    }

    [Fact]
    public void Arc_of_circle_with_parameters_adjusts_them_into_the_period()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Edge anE = new LibraryMakeEdge(aCirc, 7.0, 8.0).Edge();
        Tool.Range(anE, out double f, out double l);
        Assert.Equal(7.0 - 2 * Math.PI, f, 1e-12);
        Assert.Equal(8.0 - 2 * Math.PI, l, 1e-12);
    }

    [Fact]
    public void Point_off_the_curve_fails_its_projection()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var aME = new LibraryMakeEdge(aCirc, new Point3D(0, 2.1, 0), new Point3D(-2, 0, 0));
        Assert.False(aME.IsDone);
        Assert.Equal(LibraryEdgeError.PointProjectionFailed, aME.Error());
    }

    [Fact]
    public void Ellipse_hyperbola_and_parabola_arcs()
    {
        var anAx = new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1));
        Edge anEl = new LibraryMakeEdge(new Ellipse3D(anAx, 3, 1), 0.0, 1.0).Edge();
        Assert.IsType<EllipseCurve3D>(Tool.Curve(anEl, out _, out _));
        Edge aHy = new LibraryMakeEdge(new Hyperbola3D(anAx, 3, 1), -1.0, 1.0).Edge();
        Assert.IsType<HyperbolaCurve3D>(Tool.Curve(aHy, out _, out _));
        Edge aPa = new LibraryMakeEdge(new Parabola3D(anAx, 0.5), -2.0, 2.0).Edge();
        Assert.IsType<ParabolaCurve3D>(Tool.Curve(aPa, out double f, out double l));
        Assert.Equal(-2.0, f);
        Assert.Equal(2.0, l);
        Exploration.Vertices(aHy, out Vertex aV1, out _);
        AssertPoint(new HyperbolaCurve3D(new Hyperbola3D(anAx, 3, 1)).Value(-1.0), Tool.Pnt(aV1));
    }

    private static BSplineCurve3D BSpline()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(3, 2, 1);
        aPoles[4] = new Point3D(4, 0, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void BSpline_edge_with_a_range_and_out_of_range_parameters()
    {
        BSplineCurve3D aC = BSpline();
        var aME = new LibraryMakeEdge(aC, 0.25, 0.75);
        Edge anE = aME.Edge();
        Assert.Same(aC, Tool.Curve(anE, out double f, out double l));
        Assert.True(aC.IsFrozen); // frozen when the edge took it
        Assert.Equal(0.25, f);
        Assert.Equal(0.75, l);
        AssertPoint(aC.Value(0.25), Tool.Pnt(aME.Vertex1()));

        var aBad = new LibraryMakeEdge(BSpline(), 0.5, 1.5);
        Assert.Equal(LibraryEdgeError.ParameterOutOfRange, aBad.Error());
        Assert.False(aBad.IsDone);
    }

    [Fact]
    public void Trimmed_curves_are_replaced_by_their_basis()
    {
        var aT = new TrimmedCurve3D(new TrimmedCurve3D(BSpline(), 0.1, 0.9), 0.2, 0.8);
        Edge anE = new LibraryMakeEdge(aT).Edge();
        // Geom_TrimmedCurve keeps a copy of its basis; the edge gets that B-spline, not a trimmed curve
        Curve3D? anEC = Tool.Curve(anE, out double f, out double l);
        Assert.Same(aT.BasisCurve(), anEC);
        Assert.IsType<BSplineCurve3D>(anEC);
        Assert.Equal(0.2, f, 1e-15);
        Assert.Equal(0.8, l, 1e-15);
    }

    [Fact]
    public void Vertex_away_from_its_parameter_fails()
    {
        Vertex aV1 = V(0, 0, 0), aV2 = V(5, 1, 0);
        var aME = new LibraryMakeEdge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), aV1, aV2, 0.0, 5.0);
        Assert.Equal(LibraryEdgeError.DifferentsPointAndParameter, aME.Error());
        Assert.False(aME.IsDone);
    }

    [Fact]
    public void Vertex_at_an_infinite_parameter_fails()
    {
        Vertex aV1 = V(0, 0, 0);
        var aME = new LibraryMakeEdge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), aV1, new Vertex(),
                                       -global::Kerf.Engine.Foundation.Precision.Infinite(), 0.0);
        Assert.Equal(LibraryEdgeError.PointWithInfiniteParameter, aME.Error());
    }

    [Fact]
    public void Different_vertices_on_a_closed_curve_fail()
    {
        var aCirc = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
        Vertex aV1 = V(1, 0, 0), aV2 = V(1, 0, 0);
        var aME = new LibraryMakeEdge(aCirc, aV1, aV2, 0.0, 2 * Math.PI);
        Assert.Equal(LibraryEdgeError.DifferentPointsOnClosedCurve, aME.Error());
    }

    [Fact]
    public void Tiny_closed_curve_with_one_vertex_is_degenerated()
    {
        var aCirc = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1e-8);
        Vertex aV = V(1e-8, 0, 0);
        var aME = new LibraryMakeEdge(aCirc, aV, aV, 0.0, 2 * Math.PI);
        Assert.True(aME.IsDone);
        Assert.True(Tool.Degenerated(aME.Edge()));
    }

    [Fact]
    public void Edge_on_a_surface_has_a_pcurve_and_no_3d_curve()
    {
        var aPlane = new PlaneSurface(new Point3D(0, 0, 1), new Direction3D(0, 0, 1));
        var aL = new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0));
        var aME = new LibraryMakeEdge(aL, aPlane, 0.0, 2.0);
        Edge anE = aME.Edge();
        Assert.Null(Tool.Curve(anE, out _, out _));
        Curve2D? aPC = Tool.CurveOnSurface(anE, aPlane, new Placement(), out double f, out double l);
        Assert.Same(aL, aPC);
        Assert.Equal(0.0, f);
        Assert.Equal(2.0, l);
        Assert.Equal(new Point3D(3, 1, 1), Tool.Pnt(aME.Vertex2()));
        Assert.Equal(Forward, anE.Orientation());

        // reversed parameters reverse the edge
        Edge aR = new LibraryMakeEdge(new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0)), aPlane, 2.0, 0.0).Edge();
        Assert.Equal(Reversed, aR.Orientation());
    }

    [Fact]
    public void Init_after_a_public_read_makes_a_new_edge()
    {
        var aME = new LibraryMakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        Edge aFirst = aME.Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        aME.Init(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 0.0, 2.0);
        Edge aSecond = aME.Edge();
        Assert.False(aFirst.IsSame(aSecond));
        aSnap.AssertUnchanged(aFirst);
    }

    // ---------------------------------------------------------------- 2D edges

    [Fact]
    public void Edge2d_between_points_is_a_pcurve_on_the_default_plane()
    {
        var aME = new LibraryMakeEdge2d(new Point2D(1, 1), new Point2D(4, 5));
        Assert.True(aME.IsDone);
        Assert.Equal(LibraryEdgeError.EdgeDone, aME.Error());
        Edge anE = aME.Edge();
        Assert.True(anE.TShape()!.IsFrozen);
        Curve2D? aPC = Tool.CurveOnSurface(anE, BrepLibrary.Plane(), new Placement(), out double f, out double l);
        Assert.IsType<LineCurve2D>(aPC);
        Assert.Equal(0.0, f);
        Assert.Equal(5.0, l);
        Assert.Equal(new Point3D(4, 5, 0), Tool.Pnt(aME.Vertex2()));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(aME.Vertex1()));
    }

    [Fact]
    public void Edge2d_whole_circle_is_closed()
    {
        var aME = new LibraryMakeEdge2d(new Circle2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.5));
        Assert.True(aME.IsDone);
        Assert.True(aME.Vertex1().IsSame(aME.Vertex2()));
        Assert.Equal(new Point3D(1.5, 0, 0), Tool.Pnt(aME.Vertex1()));
    }

    [Fact]
    public void Edge2d_through_identical_points_fails()
    {
        var aME = new LibraryMakeEdge2d(new Point2D(1, 1), new Point2D(1, 1));
        Assert.Equal(LibraryEdgeError.LineThroughIdenticPoints, aME.Error());
        Assert.False(aME.IsDone);
    }

    // ---------------------------------------------------------------- wires

    [Fact]
    public void Wire_of_edges_sharing_vertices_keeps_them()
    {
        Vertex a = V(0, 0, 0), b = V(1, 0, 0), c = V(1, 1, 0);
        Edge e1 = E(a, b), e2 = E(b, c), e3 = E(c, a);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2);
        var aMW = new LibraryMakeWire(e1, e2);
        Assert.Equal(LibraryWireError.WireDone, aMW.Error());
        Wire aW = aMW.Wire();
        Assert.True(aW.TShape()!.IsFrozen);
        Assert.False(aW.Closed());
        Assert.True(aMW.Edge().IsSame(e2));
        Assert.True(aMW.Vertex().IsSame(b));
        aMW.Add(e3);
        Wire aClosed = aMW.Wire();
        Assert.True(aClosed.Closed());
        Assert.Equal(3, Count(aClosed, ShapeType.Edge));
        Assert.False(aW.Closed()); // the published wire didn't change
        Assert.Equal(2, Count(aW, ShapeType.Edge));
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        Assert.Null(aMW.InternalHistory());
    }

    [Fact]
    public void Disconnected_edge_is_refused()
    {
        var aMW = new LibraryMakeWire(E(0, 0, 0, 1, 0, 0));
        aMW.Add(E(5, 0, 0, 6, 0, 0));
        Assert.Equal(LibraryWireError.DisconnectedWire, aMW.Error());
        Assert.False(aMW.IsDone);
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aMW.Wire());
    }

    [Fact]
    public void Empty_maker_reports_an_empty_wire()
    {
        var aMW = new LibraryMakeWire();
        Assert.Equal(LibraryWireError.EmptyWire, aMW.Error());
        Assert.False(aMW.IsDone);
    }

    [Fact]
    public void Edges_with_coincident_vertices_are_connected_by_copies()
    {
        Edge e1 = E(0, 0, 0, 1, 0, 0), e2 = E(1, 0, 0, 1, 1, 0), e3 = E(1, 1, 0, 0, 0, 0);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2), s3 = ShapeSnapshot.Of(e3);
        var aMW = new LibraryMakeWire(e1, e2, e3);
        Assert.Equal(LibraryWireError.WireDone, aMW.Error());
        Wire aW = aMW.Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, ShapeType.Edge));
        Assert.Equal(3, Count(aW, ShapeType.Vertex));
        var anExp = new Explorer(aW, ShapeType.Edge);
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
        Edge e1 = E(0, 0, 0, 1, 0, 0);
        Edge e2 = E(1 + 5e-8, 0, 0, 1, 1, 0);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1), s2 = ShapeSnapshot.Of(e2);
        var aMW = new LibraryMakeWire(e1, e2);
        Assert.Equal(LibraryWireError.WireDone, aMW.Error());
        Wire aW = aMW.Wire();
        Assert.Equal(2, Count(aW, ShapeType.Edge));
        Assert.Equal(3, Count(aW, ShapeType.Vertex));
        s1.AssertUnchanged(e1);
        s2.AssertUnchanged(e2);
        History? aH = aMW.InternalHistory();
        Assert.NotNull(aH);
        Shape aE1Copy = aH!.Modified(e1)[0];
        var anExp = new Explorer(aW, ShapeType.Edge);
        Assert.True(anExp.Current().IsSame(aE1Copy));

        // the shared vertex moved half way and grew to cover both
        Exploration.Vertices(ShapeCasts.Edge(anExp.Current()), out _, out Vertex aMid, true);
        Assert.Equal(1.25e-7, Tool.Tolerance(aMid), 1e-15);
        AssertPoint(new Point3D(1 + 2.5e-8, 0, 0), Tool.Pnt(aMid), 1e-15);
    }

    [Fact]
    public void Edge_meeting_the_wire_at_an_inner_vertex_builds_a_non_manifold_wire()
    {
        Vertex a = V(0, 0, 0), b = V(1, 0, 0), c = V(2, 0, 0), d = V(1, 1, 0);
        var aMW = new LibraryMakeWire(E(a, b), E(b, c));
        aMW.Add(E(b, d));
        Assert.True(aMW.IsDone);
        Wire aW = aMW.Wire();
        Assert.Equal(3, Count(aW, ShapeType.Edge));
        Assert.False(aW.Closed());
    }

    [Fact]
    public void List_of_edges_in_any_order_is_connected()
    {
        // a square whose edges have their own coincident vertices, listed out of order
        Edge e1 = E(0, 0, 0, 1, 0, 0), e2 = E(1, 0, 0, 1, 1, 0), e3 = E(1, 1, 0, 0, 1, 0), e4 = E(0, 1, 0, 0, 0, 0);
        var aL = new NCollection_List<Shape>();
        aL.Append(e3);
        aL.Append(e1);
        aL.Append(e4);
        aL.Append(e2);
        ShapeSnapshot s1 = ShapeSnapshot.Of(e1);
        var aMW = new LibraryMakeWire();
        aMW.Add(aL);
        Assert.True(aMW.IsDone);
        Wire aW = aMW.Wire();
        Assert.Equal(4, Count(aW, ShapeType.Edge));
        Assert.Equal(4, Count(aW, ShapeType.Vertex));
        Assert.True(aW.Closed());
        s1.AssertUnchanged(e1);

        // a list that can't be connected
        var aBad = new NCollection_List<Shape>();
        aBad.Append(E(0, 0, 0, 1, 0, 0));
        aBad.Append(E(3, 0, 0, 4, 0, 0));
        var aMW2 = new LibraryMakeWire();
        aMW2.Add(aBad);
        Assert.False(aMW2.IsDone);
        Assert.Equal(LibraryWireError.DisconnectedWire, aMW2.Error());
    }

    [Fact]
    public void List_added_to_a_frozen_wire_grows_its_vertex_on_a_copy()
    {
        // the wire's end vertex (tolerance 1e-7) must grow to reach a list edge's vertex 1.5e-7 away (tolerance 1e-7)
        Wire aW0 = new LibraryMakeWire(E(0, 0, 0, 1, 0, 0)).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aW0);
        var aL = new NCollection_List<Shape>();
        aL.Append(E(1 + 1.5e-7, 0, 0, 2, 0, 0));
        var aMW = new LibraryMakeWire(aW0);
        aMW.Add(aL);
        Assert.True(aMW.IsDone);
        Wire aW = aMW.Wire();
        Assert.Equal(2, Count(aW, ShapeType.Edge));
        Assert.Equal(3, Count(aW, ShapeType.Vertex));
        Assert.NotNull(aMW.InternalHistory());
        aSnap.AssertUnchanged(aW0);
    }

    [Fact]
    public void Wire_of_a_frozen_wire_and_an_edge()
    {
        Wire aW0 = new LibraryMakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0)).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aW0);
        var aMW = new LibraryMakeWire(aW0, E(1, 1, 0, 0, 0, 0));
        Assert.True(aMW.IsDone);
        Assert.True(aMW.Wire().Closed());
        aSnap.AssertUnchanged(aW0);
    }

    // ---------------------------------------------------------------- polygons

    [Fact]
    public void Polygon_open_then_closed()
    {
        var aMP = new LibraryMakePolygon(new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(2, 2, 0));
        Assert.True(aMP.IsDone);
        Wire aOpen = aMP.Wire();
        Assert.False(aOpen.Closed());
        Assert.Equal(2, Count(aOpen, ShapeType.Edge));
        Assert.Equal(new Point3D(0, 0, 0), Tool.Pnt(aMP.FirstVertex()));
        Assert.Equal(new Point3D(2, 2, 0), Tool.Pnt(aMP.LastVertex()));
        aMP.Close();
        Wire aClosed = aMP.Wire();
        Assert.True(aClosed.Closed());
        Assert.Equal(3, Count(aClosed, ShapeType.Edge));
        Assert.False(aOpen.Closed()); // published before: unchanged
        Assert.Equal(2, Count(aOpen, ShapeType.Edge));
        Assert.True(aMP.Edge().TShape()!.IsFrozen);
    }

    [Fact]
    public void Polygon_of_four_points_closed()
    {
        Wire aW = new LibraryMakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0),
                                                 new Point3D(0, 1, 0), true).Wire();
        Assert.True(aW.Closed());
        Assert.Equal(4, Count(aW, ShapeType.Edge));
        Assert.Equal(4, Count(aW, ShapeType.Vertex));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(ShapeCasts.Edge(new Explorer(aW, ShapeType.Edge).Current())));
    }

    [Fact]
    public void Polygon_skips_a_repeated_point_and_closes_on_the_first()
    {
        var aMP = new LibraryMakePolygon();
        aMP.Add(new Point3D(0, 0, 0));
        aMP.Add(new Point3D(1, 0, 0));
        Assert.True(aMP.Added());
        aMP.Add(new Point3D(1, 0, 0));
        Assert.False(aMP.Added());
        aMP.Add(new Point3D(1, 1, 0));
        aMP.Add(new Point3D(0, 0, 0));
        Assert.True(aMP.Added());
        Wire aW = aMP.Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, ShapeType.Edge));
        Assert.True(aMP.LastVertex().IsSame(aMP.FirstVertex()));
    }

    [Fact]
    public void Polygon_of_frozen_vertices_shares_them()
    {
        Vertex a = V(0, 0, 0), b = V(3, 0, 0), c = V(0, 4, 0);
        ShapeSnapshot aSa = ShapeSnapshot.Of(a);
        Wire aW = new LibraryMakePolygon(a, b, c, true).Wire();
        Assert.True(aW.Closed());
        Assert.Equal(3, Count(aW, ShapeType.Vertex));
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aW, ShapeType.Vertex, aMap);
        Assert.True(aMap.Contains(a));
        aSa.AssertUnchanged(a);
    }

    // ---------------------------------------------------------------- faces

    [Fact]
    public void Face_on_a_whole_plane_has_no_wire()
    {
        var aMF = new LibraryMakeFace(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        Assert.True(aMF.IsDone);
        Assert.Equal(LibraryFaceError.FaceDone, aMF.Error());
        Face aF = aMF.Face();
        Assert.True(aF.TShape()!.IsFrozen);
        Assert.Equal(0, Count(aF, ShapeType.Wire));
        Assert.True(Tool.NaturalRestriction(aF));
        Assert.IsType<PlaneSurface>(Tool.Surface(aF));
    }

    [Fact]
    public void Face_on_a_bounded_plane_has_a_wire_of_four_iso_edges()
    {
        var aPln = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        Face aF = new LibraryMakeFace(aPln, 0, 2, 0, 3).Face();
        Assert.Equal(1, Count(aF, ShapeType.Wire));
        Assert.Equal(4, Count(aF, ShapeType.Edge));
        Assert.Equal(4, Count(aF, ShapeType.Vertex));
        Assert.False(Tool.NaturalRestriction(aF)); // OCCT: only Init(S, Bound, ...) sets it
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Assert.NotNull(Tool.CurveOnSurface(anE, aF, out _, out _));
            Assert.NotNull(Tool.Curve(anE, out _, out _));
        }
        var aFC = new FClass2d(aF, 0);
        Assert.Equal(Out, aFC.PerformInfinitePoint());
    }

    [Fact]
    public void Face_on_a_whole_cylinder_band_has_a_seam()
    {
        var aCyl = new Cylinder(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        Face aF = new LibraryMakeFace(aCyl, 0, 2 * Math.PI, 0, 5).Face();
        Assert.Equal(3, Count(aF, ShapeType.Edge));
        Assert.Equal(4, CountOccurrences(aF, ShapeType.Edge));
        int aSeams = 0;
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (Tool.IsClosed(ShapeCasts.Edge(anExp.Current()), aF))
            {
                aSeams++;
            }
        }
        Assert.Equal(2, aSeams); // the seam, met twice
    }

    [Fact]
    public void Face_on_a_whole_sphere_has_degenerated_poles()
    {
        Face aF = new LibraryMakeFace(new Sphere(new CoordinateSystem(new Point3D(1, 1, 1), new Direction3D(0, 0, 1)), 3.0)).Face();
        int aDegenerated = 0;
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (Tool.Degenerated(ShapeCasts.Edge(anExp.Current())))
            {
                aDegenerated++;
                Assert.Null(Tool.Curve(ShapeCasts.Edge(anExp.Current()), out _, out _));
            }
        }
        Assert.Equal(2, aDegenerated);
        Assert.Equal(2, Count(aF, ShapeType.Vertex));
        Assert.True(Tool.NaturalRestriction(aF));
    }

    [Fact]
    public void Face_on_a_cone_from_its_apex_is_degenerated_there()
    {
        double aSemi = Math.PI / 6;
        var aCone = new Cone(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), aSemi, 1.0);
        double vApex = -1.0 / Math.Sin(aSemi);
        Face aF = new LibraryMakeFace(aCone, 0, 2 * Math.PI, vApex, 1.0).Face();
        int aDegenerated = 0;
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (Tool.Degenerated(ShapeCasts.Edge(anExp.Current())))
            {
                aDegenerated++;
            }
        }
        Assert.Equal(1, aDegenerated);
    }

    [Fact]
    public void Face_parameters_out_of_range()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(0, 1, 0);
        aPoles[2, 1] = new Point3D(1, 0, 0);
        aPoles[2, 2] = new Point3D(1, 1, 1);
        var aS = new BezierSurface(aPoles);
        var aBad = new LibraryMakeFace(aS, 2.0, 0.5, 0, 1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(LibraryFaceError.ParametersOutOfRange, aBad.Error());
        Assert.False(aBad.IsDone);
        var aGood = new LibraryMakeFace(aS, 0.25, 0.75, 0, 1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGood.IsDone);
        Assert.Equal(4, Count(aGood.Face(), ShapeType.Edge));
    }

    private static Wire Square(double x0, double y0, double s, bool theCcw)
    {
        var aP = new[] { new Point3D(x0, y0, 0), new Point3D(x0 + s, y0, 0), new Point3D(x0 + s, y0 + s, 0), new Point3D(x0, y0 + s, 0) };
        if (!theCcw)
        {
            Array.Reverse(aP);
        }
        return new LibraryMakePolygon(aP[0], aP[1], aP[2], aP[3], true).Wire();
    }

    [Fact]
    public void Face_on_a_plane_from_a_wire_is_oriented_to_bound_a_finite_area()
    {
        var aPln = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        foreach (bool aCcw in new[] { true, false })
        {
            Wire aW = Square(0, 0, 1, aCcw);
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aW);
            var aMF = new LibraryMakeFace(aPln, aW);
            Assert.True(aMF.IsDone);
            Face aF = aMF.Face();
            Assert.Equal(Out, new FClass2d(aF, 0).PerformInfinitePoint());
            var anIt = new Iterator(aF);
            Assert.True(anIt.Value().IsSame(aW));
            Assert.Equal(aCcw ? Forward : Reversed, anIt.Value().Orientation());
            aSnap.AssertUnchanged(aW);
        }

        // Inside false keeps the wire as given
        Face aKept = new LibraryMakeFace(aPln, Square(0, 0, 1, false), false).Face();
        Assert.Equal(Forward, new Iterator(aKept).Value().Orientation());
    }

    [Fact]
    public void Face_with_a_hole_and_Add_after_a_public_read()
    {
        var aPln = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aMF = new LibraryMakeFace(aPln, Square(0, 0, 4, true));
        Face aOuterOnly = aMF.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aOuterOnly);
        aMF.Add(Square(1, 1, 1, false));
        Face aF = aMF.Face();
        Assert.Equal(2, Count(aF, ShapeType.Wire));
        Assert.False(Tool.NaturalRestriction(aF));
        aSnap.AssertUnchanged(aOuterOnly);
        Assert.Equal(1, Count(aOuterOnly, ShapeType.Wire));

        var aFC = new FaceClassifier(aF, new Point2D(1.5, 1.5), 1e-7);
        Assert.Equal(Out, aFC.State());
        aFC.Perform(aF, new Point2D(0.5, 0.5), 1e-7);
        Assert.Equal(In, aFC.State());
    }

    [Fact]
    public void Face_loaded_from_a_face_is_a_new_face_with_the_same_wires()
    {
        Face aF0 = new LibraryMakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Square(0, 0, 4, true)).Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aF0);
        Wire aHole = Square(1, 1, 1, false);
        Face aF = new LibraryMakeFace(aF0, aHole).Face();
        Assert.False(aF.IsSame(aF0));
        Assert.Same(Tool.Surface(aF0), Tool.Surface(aF));
        Assert.Equal(2, Count(aF, ShapeType.Wire));
        aSnap.AssertUnchanged(aF0);
    }

    [Fact]
    public void IsDegenerated_on_circles_and_poles()
    {
        Assert.True(LibraryMakeFace.IsDegenerated(new CircleCurve3D(new Frame3D(), 1e-9), 1e-7, out double aTol));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aTol);
        Assert.False(LibraryMakeFace.IsDegenerated(new CircleCurve3D(new Frame3D(), 1.0), 1e-7, out _));
        Assert.False(LibraryMakeFace.IsDegenerated(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1e-7, out _));
    }

    // ---------------------------------------------------------------- faces through BRepLib's statics

    [Fact]
    public void Face_from_a_frozen_planar_wire_is_built_on_a_copy_for_write()
    {
        Wire aW = Square(1, 2, 3, true);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aW);
        var aMF = new LibraryMakeFace(aW, true);
        Assert.True(aMF.IsDone);
        Assert.Equal(LibraryFaceError.FaceDone, aMF.Error());
        Face aF = aMF.Face();
        Assert.True(aF.TShape()!.IsFrozen);
        aSnap.AssertUnchanged(aW);

        // the statics changed nothing but the vertices' rounding guard on the copy: the face holds the caller's wire
        History? aH = aMF.InternalHistory();
        Assert.NotNull(aH);
        Shape aFaceWire = new Iterator(aF).Value();
        Assert.True(aFaceWire.IsSame(aW));
        for (var anExp = new Explorer(aW, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.Empty(aH!.Modified(anExp.Current()));
        }
        Assert.Equal(4, Count(aF, ShapeType.Edge));
        var aPlane = Assert.IsType<PlaneSurface>(Tool.Surface(aF));
        Assert.Equal(1.0, Math.Abs(aPlane.Position().Direction().Z()), 1e-12);
        Assert.Equal(Out, new FClass2d(aF, 0).PerformInfinitePoint());
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(Tool.SameParameter(ShapeCasts.Edge(anExp.Current())));
        }
    }

    [Fact]
    public void Faces_from_frozen_wires_sharing_edges_share_them()
    {
        // two squares side by side, sharing the edge x = 1
        var aV = new Vertex[6];
        Point3D[] aP = [new(0, 0, 0), new(1, 0, 0), new(2, 0, 0), new(2, 1, 0), new(1, 1, 0), new(0, 1, 0)];
        for (int i = 0; i < 6; i++)
        {
            aV[i] = TestShapes.Vertex(aP[i]);
        }
        Edge aShared = TestShapes.Edge(aV[1], aV[4]);
        var aW1 = new LibraryMakeWire(TestShapes.Edge(aV[0], aV[1]), aShared, TestShapes.Edge(aV[4], aV[5]), TestShapes.Edge(aV[5], aV[0])).Wire();
        var aW2 = new LibraryMakeWire(TestShapes.Edge(aV[1], aV[2]), TestShapes.Edge(aV[2], aV[3]), TestShapes.Edge(aV[3], aV[4]), aShared).Wire();
        Face aF1 = new LibraryMakeFace(aW1, true).Face();
        Face aF2 = new LibraryMakeFace(aW2, true).Face();

        var aShell = Builder.MakeCompound(aF1, aF2);
        var anEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        var aVertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShell, ShapeType.Edge, anEdges);
        Exploration.MapShapes(aShell, ShapeType.Vertex, aVertices);
        Assert.Equal(7, anEdges.Extent());
        Assert.Equal(6, aVertices.Extent());
        Assert.True(anEdges.Contains(aShared));
    }

    [Fact]
    public void Face_from_a_frozen_wire_keeps_the_edges_its_statics_leave_unchanged()
    {
        // a square whose last vertex is 1e-4 off its edges' ends: UpdateTolerances raises that vertex, and the two
        // edges holding it are rebuilt around the new vertex; the other two edges and vertices stay the caller's
        var aV = new Vertex[4];
        Point3D[] aP = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)];
        for (int i = 0; i < 4; i++)
        {
            aV[i] = TestShapes.Vertex(aP[i]);
        }
        var aB = new BrepBuilder();
        Vertex aOff = new();
        aB.MakeVertex(ref aOff, new Point3D(0, 1.0001, 0), global::Kerf.Engine.Foundation.Precision.Confusion());
        var anEdges = new[]
        {
            TestShapes.Edge(aV[0], aV[1]), TestShapes.Edge(aV[1], aV[2]),
            TestShapes.Edge(new LineCurve3D(aP[2], new Direction3D(-1, 0, 0)), 0, 1, aV[2], aOff),
            TestShapes.Edge(new LineCurve3D(aP[3], new Direction3D(0, -1, 0)), 0, 1, aOff, aV[0]),
        };
        var aWire = new LibraryMakeWire(TestShapes.Frozen(anEdges[0]), TestShapes.Frozen(anEdges[1]),
                                         TestShapes.Frozen(anEdges[2]), TestShapes.Frozen(anEdges[3])).Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        var aMF = new LibraryMakeFace(aWire, true);
        Assert.True(aMF.IsDone);
        Face aF = aMF.Face();
        aSnap.AssertUnchanged(aWire);

        History aH = aMF.InternalHistory()!;
        var aFaceEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aF, ShapeType.Edge, aFaceEdges);
        Assert.True(aFaceEdges.Contains(anEdges[0]));
        Assert.True(aFaceEdges.Contains(anEdges[1]));
        Assert.False(aFaceEdges.Contains(anEdges[2]));
        Assert.False(aFaceEdges.Contains(anEdges[3]));
        Assert.Empty(aH.Modified(anEdges[0]));
        Assert.Single(aH.Modified(anEdges[2]));
        Vertex aNewOff = ShapeCasts.Vertex(Assert.Single(aH.Modified(aOff)));
        Assert.Equal(1e-4, Tool.Tolerance(aNewOff), 1e-9);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(aOff));
        Assert.True(aF.TShape()!.IsFrozen);
    }

    [Fact]
    public void Face_from_a_wire_that_is_not_planar_fails()
    {
        Wire aW = new LibraryMakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0.5),
                                                 new Point3D(0, 1, 0), true).Wire();
        var aMF = new LibraryMakeFace(aW);
        Assert.False(aMF.IsDone);
        Assert.Equal(LibraryFaceError.NotPlanar, aMF.Error());
    }

    [Fact]
    public void Face_on_an_offset_surface_is_made_same_parameter()
    {
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var anOffset = new OffsetSurface(aCyl, 0.5);
        var aMF = new LibraryMakeFace(anOffset, 0, Math.PI, 0, 3, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aMF.IsDone);
        Face aF = aMF.Face();
        Assert.Equal(4, Count(aF, ShapeType.Edge));
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Assert.True(Tool.SameParameter(anE));
            Assert.True(Tool.Tolerance(anE) < 1e-5);
        }
    }

    // ---------------------------------------------------------------- shells

    private static BSplineSurface SurfaceWithAC1Knot()
    {
        // cubic in U with a double interior knot (C1 there), linear in V
        var aPoles = new NCollection_Array2<Point3D>(1, 6, 1, 2);
        for (int i = 1; i <= 6; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                aPoles[i, j] = new Point3D(i, 3 * j, (i % 2) * 0.5 + 0.1 * j);
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
        return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 1);
    }

    [Fact]
    public void Shell_of_a_BSpline_surface_is_split_at_its_C1_knot()
    {
        BSplineSurface aS = SurfaceWithAC1Knot();
        var aMS = new LibraryMakeShell(aS);
        Assert.True(aMS.IsDone);
        Assert.Equal(LibraryShellError.ShellDone, aMS.Error());
        Shell aSh = aMS.Shell();
        Assert.True(aSh.TShape()!.IsFrozen);
        Assert.Equal(2, Count(aSh, ShapeType.Face));
        Assert.Equal(7, Count(aSh, ShapeType.Edge)); // the shared edge once
        Assert.Equal(6, Count(aSh, ShapeType.Vertex));
        Assert.False(aSh.Closed());
        for (var anExp = new Explorer(aSh, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.NotNull(Tool.Curve(ShapeCasts.Edge(anExp.Current()), out _, out _)); // BuildCurves3d
        }
        // every face is on a copy of the whole surface
        for (var anExp = new Explorer(aSh, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aFS = Assert.IsType<BSplineSurface>(Tool.Surface(ShapeCasts.Face(anExp.Current())));
            Assert.NotSame(aS, aFS);
            Assert.Equal(3, aFS.UKnotCount);
        }

        // with Segment, each face has its own segment
        Shell aSeg = new LibraryMakeShell(aS, true).Shell();
        for (var anExp = new Explorer(aSeg, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aFS = Assert.IsType<BSplineSurface>(Tool.Surface(ShapeCasts.Face(anExp.Current())));
            Assert.Equal(2, aFS.UKnotCount);
        }
        Assert.False(aS.IsFrozen); // the caller's surface is only copied
    }

    [Fact]
    public void Shell_between_parameters_and_the_empty_maker()
    {
        var aMS = new LibraryMakeShell(SurfaceWithAC1Knot(), 0.5, 1.5, 0.2, 0.8);
        Assert.True(aMS.IsDone);
        Assert.Equal(2, Count(aMS.Shell(), ShapeType.Face));
        Assert.Equal(LibraryShellError.EmptyShell, new LibraryMakeShell().Error());
    }

    // ---------------------------------------------------------------- solids

    [Fact]
    public void Solid_from_a_shell_and_Add_after_a_public_read()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Shell aShell = ShapeCasts.Shell(new Iterator(aBox).Value());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        var aMS = new LibraryMakeSolid(aShell);
        Assert.True(aMS.IsDone);
        Solid aS1 = aMS.Solid();
        Assert.True(aS1.TShape()!.IsFrozen);
        Assert.Equal(1, Count(aS1, ShapeType.Shell));
        Shell aShell2 = ShapeCasts.Shell(new Iterator(TestShapes.Frozen(TestShapes.Box(0.5, 0.5, 0.5))).Value());
        aMS.Add(aShell2);
        Solid aS2 = aMS.Solid();
        Assert.Equal(2, Count(aS2, ShapeType.Shell));
        Assert.Equal(1, Count(aS1, ShapeType.Shell));
        aSnap.AssertUnchanged(aShell);
        Assert.Equal(LibraryShapeModification.Preserved, aMS.FaceStatus(ShapeCasts.Face(new Explorer(aS2, ShapeType.Face).Current())));
    }

    [Fact]
    public void Solid_from_shells_and_solids()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shell aSh = ShapeCasts.Shell(new Iterator(aBox).Value());
        Assert.Equal(2, CountOccurrences(new LibraryMakeSolid(aSh, ShapeCasts.Shell(aSh.Reversed())).Solid(), ShapeType.Shell));
        Assert.Equal(1, Count(new LibraryMakeSolid(aBox).Solid(), ShapeType.Shell));
        Solid aTwo = new LibraryMakeSolid(aBox, ShapeCasts.Shell(new Iterator(TestShapes.Frozen(TestShapes.Box(2, 2, 2))).Value())).Solid();
        Assert.Equal(2, Count(aTwo, ShapeType.Shell));
        Assert.Equal(0, Count(new LibraryMakeSolid().Solid(), ShapeType.Shell));
    }

    [Fact]
    public void Solid_from_a_compsolid_drops_the_shared_faces()
    {
        // two boxes sharing a face: a compsolid of two solids whose shells both hold that face
        var aB = new BrepBuilder();
        Solid aBox = TestShapes.Box(1, 1, 1);
        Shell aShell = ShapeCasts.Shell(new Iterator(aBox).Value());
        Face aShared = ShapeCasts.Face(new Iterator(aShell).Value());
        Shell aOther = new();
        aB.MakeShell(ref aOther);
        aB.Add(aOther, aShared.Reversed());
        Solid aSolid2 = new();
        aB.MakeSolid(ref aSolid2);
        aB.Add(aSolid2, aOther);
        CompSolid aCS = new();
        aB.MakeCompSolid(ref aCS);
        aB.Add(aCS, aBox);
        aB.Add(aCS, aSolid2);
        aCS = ShapeCasts.CompSolid(TestShapes.Frozen(aCS));

        var aMS = new LibraryMakeSolid(aCS);
        Solid aS = aMS.Solid();
        Assert.Equal(1, Count(aS, ShapeType.Shell));
        Assert.Equal(5, Count(aS, ShapeType.Face));
        Assert.Equal(LibraryShapeModification.Deleted, aMS.FaceStatus(aShared));
        Assert.True(ShapeCasts.Shell(new Iterator(aS).Value()).Closed() == false);
    }

    // ---------------------------------------------------------------- FindSurface

    [Fact]
    public void FindSurface_fits_a_plane_to_a_planar_polygon_and_orients_it_by_the_wire()
    {
        foreach (bool aCcw in new[] { true, false })
        {
            Wire aW = Square(0, 0, 2, aCcw);
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aW);
            var aFS = new FindSurface(aW);
            Assert.True(aFS.Found());
            Assert.False(aFS.Existed());
            Assert.True(aFS.Location().IsIdentity);
            Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), aFS.Tolerance());
            Assert.True(aFS.ToleranceReached() < 1e-15);
            var aPlane = Assert.IsType<PlaneSurface>(aFS.Surface());
            Assert.True(aPlane.IsFrozen);
            Direction3D aN = aPlane.Position().Direction();
            Assert.Equal(aCcw ? 1.0 : -1.0, aN.Z(), 1e-12);
            AssertPoint(new Point3D(1, 1, 0), aPlane.Position().Location(), 1e-12);
            aSnap.AssertUnchanged(aW);
        }
    }

    [Fact]
    public void FindSurface_rejects_a_non_planar_polygon_unless_the_tolerance_allows_it()
    {
        Wire aW = new LibraryMakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0.1),
                                                 new Point3D(0, 1, 0), true).Wire();
        var aFS = new FindSurface(aW);
        Assert.False(aFS.Found());
        Assert.True(aFS.ToleranceReached() > 0.01);
        var aLoose = new FindSurface(aW, 0.1);
        Assert.True(aLoose.Found());
        Assert.Equal(0.1, aLoose.Tolerance());
    }

    [Fact]
    public void FindSurface_finds_the_existing_surface_of_the_pcurves()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Face aF = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        Wire aW = ShapeCasts.Wire(new Iterator(aF).Value());
        var aFS = new FindSurface(aW, -1, true);
        Assert.True(aFS.Found());
        Assert.True(aFS.Existed());
        Assert.Same(Tool.Surface(aF, out Placement aLoc), aFS.Surface());
        Assert.True(aLoc.IsEqual(aFS.Location()));

        // the cylinder's wire has pcurves on a cylinder only
        Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        Wire aCW = ShapeCasts.Wire(new Iterator(aCyl).Value());
        Assert.IsType<CylindricalSurface>(new FindSurface(aCW).Surface());
        Assert.False(new FindSurface(aCW, -1, true).Existed());
    }

    [Fact]
    public void FindSurface_needs_edges_and_three_points()
    {
        Assert.False(new FindSurface(V(0, 0, 0)).Found());
        Assert.False(new FindSurface(E(0, 0, 0, 1, 0, 0)).Found()); // two points
        var aEmpty = new FindSurface();
        Assert.False(aEmpty.Found());
        Assert.Equal(0.0, aEmpty.Tolerance());
    }
}
