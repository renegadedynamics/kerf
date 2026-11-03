// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib's in-place statics (SameRange, BuildCurve3d, SameParameter, UpdateTolerances,
/// UpdateInnerTolerances, UpdateEdgeTol, OrientClosedSolid) on shapes the test owns or on copies for write of frozen
/// shapes, and the ReShape forms on frozen shapes, which leave them unchanged.
/// </summary>
public class BRepLibStaticsTests
{
    private const double Conf = 1e-7;

    private static PlaneSurface XOY() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static BSplineCurve2D Bent2d(double theFirst, double theLast)
        => new(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(2, 0), new Point2D(10, 0)]),
               NCollection_Array1<double>.From([theFirst, theLast]),
               NCollection_Array1<int>.From([3, 3]), 2);

    /// <summary>
    /// A segment from (0, 0, 0) to (10, 0, 0) parametrised by length, on a face of the plane z = 0, whose pcurve is a
    /// quadratic B-spline over [0, 1] that runs along the segment non-uniformly: neither same range nor same parameter.
    /// With <paramref name="theSameRange"/>, the B-spline is over [0, 10]: same range, not same parameter.
    /// </summary>
    private static (Edge Edge, Face Face) BentEdge(bool theSameRange = false)
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        double aLast = theSameRange ? 10 : 1;
        aB.UpdateEdge(anE, Bent2d(0, aLast), aF, Conf);
        aB.Range(anE, aF, 0, aLast);
        aB.SameRange(anE, theSameRange);
        aB.SameParameter(anE, false);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        return (anE, aF);
    }

    private static double MaxDeviation(in Edge E, in Face F)
    {
        Curve3D aC = Tool.Curve(E, out double f, out double l)!;
        Curve2D aPC = Tool.CurveOnSurface(E, F, out double pf, out double pl)!;
        Assert.Equal(f, pf, 1e-12);
        Assert.Equal(l, pl, 1e-12);
        Surface aS = Tool.Surface(F)!;
        double aMax = 0;
        for (int i = 0; i <= 50; i++)
        {
            double t = f + (l - f) * i / 50.0;
            Point2D aUV = aPC.Value(t);
            aMax = Math.Max(aMax, aC.Value(t).Distance(aS.Value(aUV.X(), aUV.Y())));
        }
        return aMax;
    }

    [Fact]
    public void CheckSameRange_and_SameRange_give_the_pcurve_the_3d_range()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        // the line's pcurve over [10, 20]: the same points at parameters shifted by 10
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(-10, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);

        Assert.False(BRepLib.CheckSameRange(anE));
        Assert.True(BRepLib.CheckSameRange(anE, 10.5));

        BRepLib.SameRange(anE);
        Assert.True(BRepLib.CheckSameRange(anE));
        Assert.True(Tool.SameRange(anE));
        Curve2D aPC = Tool.CurveOnSurface(anE, aF, out double f, out double l)!;
        Assert.Equal(0, f, 1e-12);
        Assert.Equal(10, l, 1e-12);
        Point2D aP = aPC.Value(5);
        Assert.Equal(5, aP.X(), 1e-12);
        Assert.Equal(0, aP.Y(), 1e-12);
    }

    [Fact]
    public void BuildCurve3d_on_a_plane_draws_the_pcurve()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(1, 2, 3), new Direction3D(0, 1, 1), new Direction3D(1, 0, 0)));
        aB.MakeFace(ref aF, aPlane, Conf);
        Edge anE = new();
        aB.MakeEdge(ref anE);
        var aCirc = new CircleCurve2D(new Axis2D(new Point2D(0.5, -0.5), new Direction2D(1, 0)), 2);
        aB.UpdateEdge(anE, aCirc, aF, Conf);
        aB.Range(anE, 0.25, 2.5);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);

        Assert.Null(Tool.Curve(anE, out _, out _));
        Assert.True(BRepLib.BuildCurve3d(anE));
        Curve3D? aC = Tool.Curve(anE, out double f, out double l);
        Assert.NotNull(aC);
        Assert.Equal(0.25, f, 1e-12);
        Assert.Equal(2.5, l, 1e-12);
        Assert.True(MaxDeviation(anE, aF) < 1e-12);
        Assert.True(BRepLib.BuildCurve3d(anE)); // existing curve
    }

    [Fact]
    public void BuildCurves3d_approximates_curves_on_a_cylinder()
    {
        var aB = new BrepBuilder();
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3);
        Face aF = new();
        aB.MakeFace(ref aF, aCyl, Conf);
        Wire aW = new();
        aB.MakeWire(ref aW);
        // a helix-like line in UV, and an isoline v = 1
        var anEdges = new List<Edge>();
        foreach (Curve2D aPC in new Curve2D[]
                 {
                     new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0.3)),
                     new LineCurve2D(new Point2D(0, 1), new Direction2D(1, 0)),
                 })
        {
            Edge anE = new();
            aB.MakeEdge(ref anE);
            aB.UpdateEdge(anE, aPC, aF, Conf);
            aB.Range(anE, 0.1, 2.0);
            aB.Add(aW, anE);
            anEdges.Add(anE);
        }
        aB.Add(aF, aW);

        Assert.True(BRepLib.BuildCurves3d(aF, 1e-6));
        foreach (Edge anE in anEdges)
        {
            Assert.NotNull(Tool.Curve(anE, out _, out _));
            Assert.True(Tool.SameParameter(anE));
            Assert.True(MaxDeviation(anE, aF) < 1e-5);
        }
    }

    [Fact]
    public void BuildCurve3d_of_a_degenerated_edge_without_curve_fails()
    {
        var aB = new BrepBuilder();
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3);
        Face aF = new();
        aB.MakeFace(ref aF, aCyl, Conf);
        Edge anE = new();
        aB.MakeEdge(ref anE);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Degenerated(anE, true);
        Assert.False(BRepLib.BuildCurve3d(anE));
    }

    [Fact]
    public void SameParameter_makes_the_pcurve_follow_the_3d_curve()
    {
        (Edge anE, Face aF) = BentEdge();
        Assert.False(BRepLib.CheckSameRange(anE));

        BRepLib.SameParameter(anE, 1e-6);

        Assert.True(Tool.SameParameter(anE));
        Assert.True(Tool.SameRange(anE));
        Assert.True(MaxDeviation(anE, aF) <= Tool.Tolerance(anE));
        Assert.True(Tool.Tolerance(anE) < 1e-5);
        Assert.True(Tool.Tolerance(anE) >= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SameParameter_of_a_shape_also_harmonizes_the_tolerances()
    {
        (Edge anE, Face aF) = BentEdge();
        var aB = new BrepBuilder();
        aB.UpdateFace(aF, 1e-4);

        BRepLib.SameParameter(aF, 1e-6);

        Assert.True(Tool.SameParameter(anE));
        Assert.True(Tool.Tolerance(anE) >= 1e-4);
        Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
        Assert.True(Tool.Tolerance(aV1) >= Tool.Tolerance(anE));
        Assert.True(Tool.Tolerance(aV2) >= Tool.Tolerance(anE));
    }

    [Fact]
    public void SameParameter_skips_same_parameter_edges_unless_forced()
    {
        (Edge anE, Face aF) = BentEdge();
        var aB = new BrepBuilder();
        aB.SameParameter(anE, true); // a false flag: the pcurve is still bent
        Curve2D aBefore = Tool.CurveOnSurface(anE, aF, out _, out _)!;

        BRepLib.SameParameter(aF, 1e-6);
        Assert.Same(aBefore, Tool.CurveOnSurface(anE, aF, out _, out _));

        BRepLib.SameParameter(aF, 1e-6, true);
        Assert.NotSame(aBefore, Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.True(MaxDeviation(anE, aF) <= Tool.Tolerance(anE));
    }

    [Fact]
    public void SameParameter_on_a_new_edge_leaves_the_edge_alone()
    {
        (Edge anE, Face aF) = BentEdge();
        Curve2D aBefore = Tool.CurveOnSurface(anE, aF, out _, out _)!;
        double aNewTol = -1;
        Edge aNE = BRepLib.SameParameter(anE, 1e-6, ref aNewTol, false);

        Assert.False(aNE.IsSame(anE));
        Assert.False(Tool.SameParameter(anE));
        Assert.Same(aBefore, Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.True(Tool.SameParameter(aNE));
        Assert.True(aNewTol > 0);
        Assert.False(aNE.TShape()!.IsFrozen);
    }

    [Fact]
    public void SameParameter_ReShape_form_leaves_a_frozen_shape_unchanged()
    {
        (Edge anE, Face aF) = BentEdge();
        Face aFrozen = TestShapes.Frozen(aF);
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        var aReShape = new ReShape();
        BRepLib.SameParameter(aFrozen, aReShape, 1e-6);
        aSnapshot.AssertUnchanged(aFrozen);

        Face aResult = ShapeCasts.Face(aReShape.Apply(aFrozen));
        Assert.True(aResult.TShape()!.IsFrozen);
        Edge aNE = ShapeCasts.Edge(new Explorer(aResult, ShapeType.Edge).Current());
        Assert.True(Tool.SameParameter(aNE));
        Assert.True(MaxDeviation(aNE, aResult) <= Tool.Tolerance(aNE));
        Assert.False(Tool.SameParameter(anE));
    }

    [Fact]
    public void SameParameter_ReShape_form_forced_leaves_a_frozen_shape_unchanged()
    {
        (Edge anE, Face aF) = BentEdge();
        new BrepBuilder().SameParameter(anE, true);
        Face aFrozen = TestShapes.Frozen(aF);
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        var aReShape = new ReShape();
        BRepLib.SameParameter(aFrozen, aReShape, 1e-6, true);
        aSnapshot.AssertUnchanged(aFrozen);
        Shape aResult = aReShape.Apply(aFrozen);
        Edge aNE = ShapeCasts.Edge(new Explorer(aResult, ShapeType.Edge).Current());
        Assert.False(aNE.IsSame(anE));
        Assert.True(MaxDeviation(aNE, ShapeCasts.Face(aResult)) <= Tool.Tolerance(aNE));
    }

    [Fact]
    public void UpdateTolerances_raises_edges_and_vertices_to_their_faces()
    {
        Solid aBox = TestShapes.Box(2, 3, 4);
        var aB = new BrepBuilder();
        Face aTop = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        aB.UpdateFace(aTop, 1e-3);

        BRepLib.UpdateTolerances(aBox);

        for (var anExp = new Explorer(aTop, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Assert.Equal(1e-3, Tool.Tolerance(anE));
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            Assert.True(Tool.Tolerance(aV1) >= 1e-3);
            Assert.True(Tool.Tolerance(aV2) >= 1e-3);
        }
        Assert.Equal(1e-3, Tool.MaxTolerance(aBox, ShapeType.Edge));
    }

    [Fact]
    public void UpdateTolerances_with_face_verification_sets_face_tolerances_by_size()
    {
        Solid aBox = TestShapes.Box(2, 3, 4);
        var aB = new BrepBuilder();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aB.UpdateFace(ShapeCasts.Face(anExp.Current()), 0.5);
        }

        BRepLib.UpdateTolerances(aBox, true);

        // a plane face: Confusion times its largest box dimension (enlarged by the face's own old tolerance)
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            double aTol = Tool.Tolerance(ShapeCasts.Face(anExp.Current()));
            Assert.True(aTol < 1e-5, $"{aTol}");
            Assert.True(aTol > 1e-7);
        }
    }

    [Fact]
    public void UpdateTolerances_ReShape_form_leaves_a_frozen_shape_unchanged()
    {
        Solid aBox = TestShapes.Box(2, 3, 4);
        Face aTop = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        new BrepBuilder().UpdateFace(aTop, 1e-3);
        Solid aFrozen = TestShapes.Frozen(aBox);
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        var aReShape = new ReShape();
        BRepLib.UpdateTolerances(aFrozen, aReShape);
        aSnapshot.AssertUnchanged(aFrozen);

        Shape aResult = aReShape.Apply(aFrozen);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(1e-3, Tool.MaxTolerance(aResult, ShapeType.Edge));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.MaxTolerance(aFrozen, ShapeType.Edge));
    }

    [Fact]
    public void UpdateInnerTolerances_covers_a_vertex_off_the_curve()
    {
        var aB = new BrepBuilder();
        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 1e-4));
        Vertex aV2 = TestShapes.Vertex(new Point3D(10, 0, 0));
        Edge anE = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10, aV1, aV2);
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);

        BRepLib.UpdateInnerTolerances(aF);

        Assert.True(Tool.Tolerance(aV1) >= 1e-4);
        Assert.True(Tool.Tolerance(aV1) < 1.1e-4);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Tool.Tolerance(aV2));
    }

    [Fact]
    public void UpdateEdgeTol_measures_the_pcurve_distance()
    {
        (Edge anE, Face _) = BentEdge();
        new BrepBuilder().SameRange(anE, true);
        // the 3D curve and the pcurve differ by up to 0.9 at the same parameter (t on [0, 1] against 10 t)
        Assert.False(BRepLib.UpdateEdgeTol(anE, 1e-5, 1e-8));
        Assert.True(BRepLib.UpdateEdgeTolerance(anE, 1e-5, 1.0));
        // not same parameter: the distance is measured along the pcurve, not at equal parameters
        Assert.True(Tool.Tolerance(anE) < 1e-3);
    }

    [Fact]
    public void OrientClosedSolid_puts_matter_inside()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Solid aSolid = aBox;
        Assert.True(BRepLib.OrientClosedSolid(ref aSolid));
        Assert.Equal(Forward, aSolid.Orientation());

        Solid aReversed = ShapeCasts.Solid(aBox.Reversed());
        Assert.True(BRepLib.OrientClosedSolid(ref aReversed));
        Assert.Equal(Forward, aReversed.Orientation());
        Assert.True(aReversed.IsSame(aBox));
    }

    [Fact]
    public void OrientClosedSolid_reverses_an_empty_solid()
    {
        // BRepClass3d_SClassifier::PerformInfinitePoint classifies everything IN a solid without faces
        Solid aSolid = new();
        new BrepBuilder().MakeSolid(ref aSolid);
        Assert.True(BRepLib.OrientClosedSolid(ref aSolid));
        Assert.Equal(Reversed, aSolid.Orientation());
    }

    [Fact]
    public void In_place_statics_throw_on_frozen_shapes_and_work_on_copies_for_write()
    {
        // same range: a copy for write (BRepTools_Modifier, as BRepBuilderAPI_Copy) gives a pcurve the 3D range
        (Edge anE, Face aF) = BentEdge(true);
        Face aFrozen = TestShapes.Frozen(aF);
        Edge aFrozenE = ShapeCasts.Edge(new Explorer(aFrozen, ShapeType.Edge).Current());
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => BRepLib.SameParameter(aFrozen, 1e-6));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => BRepLib.SameParameter(aFrozenE, 1e-6));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => BRepLib.SameRange(aFrozenE));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => BRepLib.UpdateInnerTolerances(aFrozen));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => BRepLib.UpdateEdgeTolerance(aFrozen, 1e-5, 1.0));
        aSnapshot.AssertUnchanged(aFrozen);

        Shape aCopy = BrepTools.CopyForWrite(aFrozen, out History aHistory);
        BRepLib.SameParameter(aCopy, 1e-6);
        BRepLib.UpdateTolerances(aCopy, true);
        BRepLib.UpdateInnerTolerances(aCopy);
        Edge aCopyE = ShapeCasts.Edge(aHistory.Modified(aFrozenE)[0]);
        Assert.True(Tool.SameParameter(aCopyE));
        Assert.True(MaxDeviation(aCopyE, ShapeCasts.Face(aCopy)) <= Tool.Tolerance(aCopyE));
        aSnapshot.AssertUnchanged(aFrozen);
    }

    [Fact]
    public void UpdateTolerances_throws_on_a_frozen_shape_that_needs_it()
    {
        Solid aBox = TestShapes.Box(2, 3, 4);
        Face aTop = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        new BrepBuilder().UpdateFace(aTop, 1e-3);
        Solid aFrozen = TestShapes.Frozen(aBox);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => BRepLib.UpdateTolerances(aFrozen));
        Assert.True(BRepLib.BuildCurves3d(aFrozen)); // every edge has its curve: nothing is written
    }
}
