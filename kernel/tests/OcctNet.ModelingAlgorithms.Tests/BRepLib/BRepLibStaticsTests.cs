// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib's in-place statics (SameRange, BuildCurve3d, SameParameter, UpdateTolerances,
/// UpdateInnerTolerances, UpdateEdgeTol, OrientClosedSolid) on shapes the test owns or on copies for write of frozen
/// shapes, and the ReShape forms on frozen shapes, which leave them unchanged.
/// </summary>
public class BRepLibStaticsTests
{
    private const double Conf = 1e-7;

    private static Geom_Plane XOY() => new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));

    private static Geom2d_BSplineCurve Bent2d(double theFirst, double theLast)
        => new(NCollection_Array1<gp_Pnt2d>.From([new gp_Pnt2d(0, 0), new gp_Pnt2d(2, 0), new gp_Pnt2d(10, 0)]),
               NCollection_Array1<double>.From([theFirst, theLast]),
               NCollection_Array1<int>.From([3, 3]), 2);

    /// <summary>
    /// A segment from (0, 0, 0) to (10, 0, 0) parametrised by length, on a face of the plane z = 0, whose pcurve is a
    /// quadratic B-spline over [0, 1] that runs along the segment non-uniformly: neither same range nor same parameter.
    /// With <paramref name="theSameRange"/>, the B-spline is over [0, 10]: same range, not same parameter.
    /// </summary>
    private static (TopoDS_Edge Edge, TopoDS_Face Face) BentEdge(bool theSameRange = false)
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        double aLast = theSameRange ? 10 : 1;
        aB.UpdateEdge(anE, Bent2d(0, aLast), aF, Conf);
        aB.Range(anE, aF, 0, aLast);
        aB.SameRange(anE, theSameRange);
        aB.SameParameter(anE, false);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        return (anE, aF);
    }

    private static double MaxDeviation(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Geom_Curve aC = BRep_Tool.Curve(E, out double f, out double l)!;
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(E, F, out double pf, out double pl)!;
        Assert.Equal(f, pf, 1e-12);
        Assert.Equal(l, pl, 1e-12);
        Geom_Surface aS = BRep_Tool.Surface(F)!;
        double aMax = 0;
        for (int i = 0; i <= 50; i++)
        {
            double t = f + (l - f) * i / 50.0;
            gp_Pnt2d aUV = aPC.Value(t);
            aMax = Math.Max(aMax, aC.Value(t).Distance(aS.Value(aUV.X(), aUV.Y())));
        }
        return aMax;
    }

    [Fact]
    public void CheckSameRange_and_SameRange_give_the_pcurve_the_3d_range()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        // the line's pcurve over [10, 20]: the same points at parameters shifted by 10
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(-10, 0), new gp_Dir2d(1, 0)), aF, Conf);
        aB.Range(anE, aF, 10, 20);
        aB.SameRange(anE, false);

        Assert.False(BRepLib.CheckSameRange(anE));
        Assert.True(BRepLib.CheckSameRange(anE, 10.5));

        BRepLib.SameRange(anE);
        Assert.True(BRepLib.CheckSameRange(anE));
        Assert.True(BRep_Tool.SameRange(anE));
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(anE, aF, out double f, out double l)!;
        Assert.Equal(0, f, 1e-12);
        Assert.Equal(10, l, 1e-12);
        gp_Pnt2d aP = aPC.Value(5);
        Assert.Equal(5, aP.X(), 1e-12);
        Assert.Equal(0, aP.Y(), 1e-12);
    }

    [Fact]
    public void BuildCurve3d_on_a_plane_draws_the_pcurve()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        Geom_Plane aPlane = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 1, 1), new gp_Dir(1, 0, 0)));
        aB.MakeFace(ref aF, aPlane, Conf);
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE);
        var aCirc = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0.5, -0.5), new gp_Dir2d(1, 0)), 2);
        aB.UpdateEdge(anE, aCirc, aF, Conf);
        aB.Range(anE, 0.25, 2.5);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);

        Assert.Null(BRep_Tool.Curve(anE, out _, out _));
        Assert.True(BRepLib.BuildCurve3d(anE));
        Geom_Curve? aC = BRep_Tool.Curve(anE, out double f, out double l);
        Assert.NotNull(aC);
        Assert.Equal(0.25, f, 1e-12);
        Assert.Equal(2.5, l, 1e-12);
        Assert.True(MaxDeviation(anE, aF) < 1e-12);
        Assert.True(BRepLib.BuildCurve3d(anE)); // existing curve
    }

    [Fact]
    public void BuildCurves3d_approximates_curves_on_a_cylinder()
    {
        var aB = new BRep_Builder();
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aCyl, Conf);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        // a helix-like line in UV, and an isoline v = 1
        var anEdges = new List<TopoDS_Edge>();
        foreach (Geom2d_Curve aPC in new Geom2d_Curve[]
                 {
                     new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0.3)),
                     new Geom2d_Line(new gp_Pnt2d(0, 1), new gp_Dir2d(1, 0)),
                 })
        {
            TopoDS_Edge anE = new();
            aB.MakeEdge(ref anE);
            aB.UpdateEdge(anE, aPC, aF, Conf);
            aB.Range(anE, 0.1, 2.0);
            aB.Add(aW, anE);
            anEdges.Add(anE);
        }
        aB.Add(aF, aW);

        Assert.True(BRepLib.BuildCurves3d(aF, 1e-6));
        foreach (TopoDS_Edge anE in anEdges)
        {
            Assert.NotNull(BRep_Tool.Curve(anE, out _, out _));
            Assert.True(BRep_Tool.SameParameter(anE));
            Assert.True(MaxDeviation(anE, aF) < 1e-5);
        }
    }

    [Fact]
    public void BuildCurve3d_of_a_degenerated_edge_without_curve_fails()
    {
        var aB = new BRep_Builder();
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 3);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aCyl, Conf);
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE);
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Conf);
        aB.Degenerated(anE, true);
        Assert.False(BRepLib.BuildCurve3d(anE));
    }

    [Fact]
    public void SameParameter_makes_the_pcurve_follow_the_3d_curve()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge();
        Assert.False(BRepLib.CheckSameRange(anE));

        BRepLib.SameParameter(anE, 1e-6);

        Assert.True(BRep_Tool.SameParameter(anE));
        Assert.True(BRep_Tool.SameRange(anE));
        Assert.True(MaxDeviation(anE, aF) <= BRep_Tool.Tolerance(anE));
        Assert.True(BRep_Tool.Tolerance(anE) < 1e-5);
        Assert.True(BRep_Tool.Tolerance(anE) >= Precision.Confusion());
    }

    [Fact]
    public void SameParameter_of_a_shape_also_harmonizes_the_tolerances()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge();
        var aB = new BRep_Builder();
        aB.UpdateFace(aF, 1e-4);

        BRepLib.SameParameter(aF, 1e-6);

        Assert.True(BRep_Tool.SameParameter(anE));
        Assert.True(BRep_Tool.Tolerance(anE) >= 1e-4);
        TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        Assert.True(BRep_Tool.Tolerance(aV1) >= BRep_Tool.Tolerance(anE));
        Assert.True(BRep_Tool.Tolerance(aV2) >= BRep_Tool.Tolerance(anE));
    }

    [Fact]
    public void SameParameter_skips_same_parameter_edges_unless_forced()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge();
        var aB = new BRep_Builder();
        aB.SameParameter(anE, true); // a false flag: the pcurve is still bent
        Geom2d_Curve aBefore = BRep_Tool.CurveOnSurface(anE, aF, out _, out _)!;

        BRepLib.SameParameter(aF, 1e-6);
        Assert.Same(aBefore, BRep_Tool.CurveOnSurface(anE, aF, out _, out _));

        BRepLib.SameParameter(aF, 1e-6, true);
        Assert.NotSame(aBefore, BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.True(MaxDeviation(anE, aF) <= BRep_Tool.Tolerance(anE));
    }

    [Fact]
    public void SameParameter_on_a_new_edge_leaves_the_edge_alone()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge();
        Geom2d_Curve aBefore = BRep_Tool.CurveOnSurface(anE, aF, out _, out _)!;
        double aNewTol = -1;
        TopoDS_Edge aNE = BRepLib.SameParameter(anE, 1e-6, ref aNewTol, false);

        Assert.False(aNE.IsSame(anE));
        Assert.False(BRep_Tool.SameParameter(anE));
        Assert.Same(aBefore, BRep_Tool.CurveOnSurface(anE, aF, out _, out _));
        Assert.True(BRep_Tool.SameParameter(aNE));
        Assert.True(aNewTol > 0);
        Assert.False(aNE.TShape()!.IsFrozen());
    }

    [Fact]
    public void SameParameter_ReShape_form_leaves_a_frozen_shape_unchanged()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge();
        TopoDS_Face aFrozen = TestShapes.Frozen(aF);
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        var aReShape = new BRepTools_ReShape();
        BRepLib.SameParameter(aFrozen, aReShape, 1e-6);
        aSnapshot.AssertUnchanged(aFrozen);

        TopoDS_Face aResult = TopoDS.Face(aReShape.Apply(aFrozen));
        Assert.True(aResult.TShape()!.IsFrozen());
        TopoDS_Edge aNE = TopoDS.Edge(new TopExp_Explorer(aResult, TopAbs_EDGE).Current());
        Assert.True(BRep_Tool.SameParameter(aNE));
        Assert.True(MaxDeviation(aNE, aResult) <= BRep_Tool.Tolerance(aNE));
        Assert.False(BRep_Tool.SameParameter(anE));
    }

    [Fact]
    public void SameParameter_ReShape_form_forced_leaves_a_frozen_shape_unchanged()
    {
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge();
        new BRep_Builder().SameParameter(anE, true);
        TopoDS_Face aFrozen = TestShapes.Frozen(aF);
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        var aReShape = new BRepTools_ReShape();
        BRepLib.SameParameter(aFrozen, aReShape, 1e-6, true);
        aSnapshot.AssertUnchanged(aFrozen);
        TopoDS_Shape aResult = aReShape.Apply(aFrozen);
        TopoDS_Edge aNE = TopoDS.Edge(new TopExp_Explorer(aResult, TopAbs_EDGE).Current());
        Assert.False(aNE.IsSame(anE));
        Assert.True(MaxDeviation(aNE, TopoDS.Face(aResult)) <= BRep_Tool.Tolerance(aNE));
    }

    [Fact]
    public void UpdateTolerances_raises_edges_and_vertices_to_their_faces()
    {
        TopoDS_Solid aBox = TestShapes.Box(2, 3, 4);
        var aB = new BRep_Builder();
        TopoDS_Face aTop = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        aB.UpdateFace(aTop, 1e-3);

        BRepLib.UpdateTolerances(aBox);

        for (var anExp = new TopExp_Explorer(aTop, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            Assert.Equal(1e-3, BRep_Tool.Tolerance(anE));
            TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            Assert.True(BRep_Tool.Tolerance(aV1) >= 1e-3);
            Assert.True(BRep_Tool.Tolerance(aV2) >= 1e-3);
        }
        Assert.Equal(1e-3, BRep_Tool.MaxTolerance(aBox, TopAbs_EDGE));
    }

    [Fact]
    public void UpdateTolerances_with_face_verification_sets_face_tolerances_by_size()
    {
        TopoDS_Solid aBox = TestShapes.Box(2, 3, 4);
        var aB = new BRep_Builder();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aB.UpdateFace(TopoDS.Face(anExp.Current()), 0.5);
        }

        BRepLib.UpdateTolerances(aBox, true);

        // a plane face: Confusion times its largest box dimension (enlarged by the face's own old tolerance)
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            double aTol = BRep_Tool.Tolerance(TopoDS.Face(anExp.Current()));
            Assert.True(aTol < 1e-5, $"{aTol}");
            Assert.True(aTol > 1e-7);
        }
    }

    [Fact]
    public void UpdateTolerances_ReShape_form_leaves_a_frozen_shape_unchanged()
    {
        TopoDS_Solid aBox = TestShapes.Box(2, 3, 4);
        TopoDS_Face aTop = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        new BRep_Builder().UpdateFace(aTop, 1e-3);
        TopoDS_Solid aFrozen = TestShapes.Frozen(aBox);
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        var aReShape = new BRepTools_ReShape();
        BRepLib.UpdateTolerances(aFrozen, aReShape);
        aSnapshot.AssertUnchanged(aFrozen);

        TopoDS_Shape aResult = aReShape.Apply(aFrozen);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(1e-3, BRep_Tool.MaxTolerance(aResult, TopAbs_EDGE));
        Assert.Equal(Precision.Confusion(), BRep_Tool.MaxTolerance(aFrozen, TopAbs_EDGE));
    }

    [Fact]
    public void UpdateInnerTolerances_covers_a_vertex_off_the_curve()
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt(0, 0, 1e-4));
        TopoDS_Vertex aV2 = TestShapes.Vertex(new gp_Pnt(10, 0, 0));
        TopoDS_Edge anE = TestShapes.Edge(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 10, aV1, aV2);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Conf);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);

        BRepLib.UpdateInnerTolerances(aF);

        Assert.True(BRep_Tool.Tolerance(aV1) >= 1e-4);
        Assert.True(BRep_Tool.Tolerance(aV1) < 1.1e-4);
        Assert.Equal(Precision.Confusion(), BRep_Tool.Tolerance(aV2));
    }

    [Fact]
    public void UpdateEdgeTol_measures_the_pcurve_distance()
    {
        (TopoDS_Edge anE, TopoDS_Face _) = BentEdge();
        new BRep_Builder().SameRange(anE, true);
        // the 3D curve and the pcurve differ by up to 0.9 at the same parameter (t on [0, 1] against 10 t)
        Assert.False(BRepLib.UpdateEdgeTol(anE, 1e-5, 1e-8));
        Assert.True(BRepLib.UpdateEdgeTolerance(anE, 1e-5, 1.0));
        // not same parameter: the distance is measured along the pcurve, not at equal parameters
        Assert.True(BRep_Tool.Tolerance(anE) < 1e-3);
    }

    [Fact]
    public void OrientClosedSolid_puts_matter_inside()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        TopoDS_Solid aSolid = aBox;
        Assert.True(BRepLib.OrientClosedSolid(ref aSolid));
        Assert.Equal(TopAbs_FORWARD, aSolid.Orientation());

        TopoDS_Solid aReversed = TopoDS.Solid(aBox.Reversed());
        Assert.True(BRepLib.OrientClosedSolid(ref aReversed));
        Assert.Equal(TopAbs_FORWARD, aReversed.Orientation());
        Assert.True(aReversed.IsSame(aBox));
    }

    [Fact]
    public void OrientClosedSolid_reverses_an_empty_solid()
    {
        // BRepClass3d_SClassifier::PerformInfinitePoint classifies everything IN a solid without faces
        TopoDS_Solid aSolid = new();
        new BRep_Builder().MakeSolid(ref aSolid);
        Assert.True(BRepLib.OrientClosedSolid(ref aSolid));
        Assert.Equal(TopAbs_REVERSED, aSolid.Orientation());
    }

    [Fact]
    public void In_place_statics_throw_on_frozen_shapes_and_work_on_copies_for_write()
    {
        // same range: a copy for write (BRepTools_Modifier, as BRepBuilderAPI_Copy) gives a pcurve the 3D range
        (TopoDS_Edge anE, TopoDS_Face aF) = BentEdge(true);
        TopoDS_Face aFrozen = TestShapes.Frozen(aF);
        TopoDS_Edge aFrozenE = TopoDS.Edge(new TopExp_Explorer(aFrozen, TopAbs_EDGE).Current());
        var aSnapshot = ShapeSnapshot.Of(aFrozen);

        Assert.ThrowsAny<Standard_Failure>(() => BRepLib.SameParameter(aFrozen, 1e-6));
        Assert.ThrowsAny<Standard_Failure>(() => BRepLib.SameParameter(aFrozenE, 1e-6));
        Assert.ThrowsAny<Standard_Failure>(() => BRepLib.SameRange(aFrozenE));
        Assert.ThrowsAny<Standard_Failure>(() => BRepLib.UpdateInnerTolerances(aFrozen));
        Assert.ThrowsAny<Standard_Failure>(() => BRepLib.UpdateEdgeTolerance(aFrozen, 1e-5, 1.0));
        aSnapshot.AssertUnchanged(aFrozen);

        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aFrozen, out BRepTools_History aHistory);
        BRepLib.SameParameter(aCopy, 1e-6);
        BRepLib.UpdateTolerances(aCopy, true);
        BRepLib.UpdateInnerTolerances(aCopy);
        TopoDS_Edge aCopyE = TopoDS.Edge(aHistory.Modified(aFrozenE)[0]);
        Assert.True(BRep_Tool.SameParameter(aCopyE));
        Assert.True(MaxDeviation(aCopyE, TopoDS.Face(aCopy)) <= BRep_Tool.Tolerance(aCopyE));
        aSnapshot.AssertUnchanged(aFrozen);
    }

    [Fact]
    public void UpdateTolerances_throws_on_a_frozen_shape_that_needs_it()
    {
        TopoDS_Solid aBox = TestShapes.Box(2, 3, 4);
        TopoDS_Face aTop = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        new BRep_Builder().UpdateFace(aTop, 1e-3);
        TopoDS_Solid aFrozen = TestShapes.Frozen(aBox);
        Assert.ThrowsAny<Standard_Failure>(() => BRepLib.UpdateTolerances(aFrozen));
        Assert.True(BRepLib.BuildCurves3d(aFrozen)); // every edge has its curve: nothing is written
    }
}
