// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/IntSurf_LineOn2S_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's IntSurf_LineOn2S GTest.</summary>
public class IntSurf_LineOn2S_Test
{
    internal static global::Kerf.Engine.Intersection.Walking.PntOn2S buildPoint(in Point3D thePoint, double theU1, double theV1, double theU2, double theV2)
    {
        global::Kerf.Engine.Intersection.Walking.PntOn2S aPnt = new();
        aPnt.SetValue(thePoint, theU1, theV1, theU2, theV2);
        return aPnt;
    }

    [Fact]
    public void EmptyLineBoxesAreNotOut()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        Assert.False(aLine.IsOutBox(new Point3D(10.0, 10.0, 10.0)));
        Assert.False(aLine.IsOutSurf1Box(new Point2D(10.0, 10.0)));
        Assert.False(aLine.IsOutSurf2Box(new Point2D(10.0, 10.0)));
    }

    [Fact]
    public void PointReplacementInvalidatesCachedBoxes()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        aLine.Add(buildPoint(new Point3D(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0));
        aLine.Add(buildPoint(new Point3D(1.0, 1.0, 1.0), 1.0, 1.0, 1.0, 1.0));

        Assert.True(aLine.IsOutBox(new Point3D(100.0, 100.0, 100.0)));
        Assert.True(aLine.IsOutSurf1Box(new Point2D(50.0, 50.0)));

        aLine.SetPoint(2, new Point3D(100.0, 100.0, 100.0));
        Assert.False(aLine.IsOutBox(new Point3D(100.0, 100.0, 100.0)));

        aLine.Value(2, buildPoint(new Point3D(100.0, 100.0, 100.0), 50.0, 50.0, 60.0, 60.0));
        Assert.False(aLine.IsOutSurf1Box(new Point2D(50.0, 50.0)));
        Assert.False(aLine.IsOutSurf2Box(new Point2D(60.0, 60.0)));
    }

    // Split divides line at given index.
    [Fact]
    public void Split_DividesCorrectly()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        aLine.Add(buildPoint(new Point3D(0, 0, 0), 0, 0, 0, 0));
        aLine.Add(buildPoint(new Point3D(1, 0, 0), 1, 0, 1, 0));
        aLine.Add(buildPoint(new Point3D(2, 0, 0), 2, 0, 2, 0));
        aLine.Add(buildPoint(new Point3D(3, 0, 0), 3, 0, 3, 0));

        // NCollection_Sequence::Split(Index, SS) moves items Index..N into SS,
        // keeping 1..Index-1 in original.
        global::Kerf.Engine.Intersection.Walking.LineOn2S aSplit = aLine.Split(2);

        Assert.Equal(1, aLine.PointCount);
        Assert.Equal(3, aSplit.PointCount);
        Assert.Equal(0.0, aLine.Value(1).Value().X, 1e-15);
        Assert.Equal(1.0, aSplit.Value(1).Value().X, 1e-15);
    }
}

/// <summary>
/// The walking-line point container against OCCT's semantics: insertion, removal, reversal, splitting, the
/// parameters set in place, and the lazily computed boxes (computed on the first query, enlarged by 1% of their
/// largest side, grown by additions, forgotten by replacements and removals).
/// </summary>
public class IntSurf_LineOn2STests
{
    private static global::Kerf.Engine.Intersection.Walking.PntOn2S P(double x, double u1, double u2) =>
        IntSurf_LineOn2S_Test.buildPoint(new Point3D(x, 0, 0), u1, 0, u2, 0);

    private static double[] Xs(global::Kerf.Engine.Intersection.Walking.LineOn2S theLine) =>
        Enumerable.Range(1, theLine.PointCount).Select(i => theLine.Value(i).Value().X).ToArray();

    [Fact]
    public void InsertBefore_inserts_or_appends_past_the_end()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        aLine.Add(P(0, 0, 0));
        aLine.Add(P(2, 2, 2));
        aLine.InsertBefore(2, P(1, 1, 1));
        aLine.InsertBefore(4, P(3, 3, 3)); // past the end: appended
        aLine.InsertBefore(99, P(4, 4, 4));
        aLine.InsertBefore(1, P(-1, -1, -1));
        Assert.Equal(new[] { -1.0, 0, 1, 2, 3, 4 }, Xs(aLine));

        EngineAssert.ThrowsAny<OutOfRangeException>(() => aLine.InsertBefore(0, P(9, 9, 9)));
    }

    [Fact]
    public void RemovePoint_Reverse_and_Clear()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        for (int i = 0; i < 5; i++)
        {
            aLine.Add(P(i, i, i));
        }
        aLine.RemovePoint(3);
        Assert.Equal(new[] { 0.0, 1, 3, 4 }, Xs(aLine));
        aLine.Reverse();
        Assert.Equal(new[] { 4.0, 3, 1, 0 }, Xs(aLine));
        EngineAssert.ThrowsAny<OutOfRangeException>(() => aLine.RemovePoint(5));
        aLine.Clear();
        Assert.Equal(0, aLine.PointCount);
        Assert.False(aLine.IsOutBox(new Point3D(1e10, 0, 0)));
    }

    [Fact]
    public void Split_at_the_first_or_past_the_last_point()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        for (int i = 0; i < 3; i++)
        {
            aLine.Add(P(i, i, i));
        }
        global::Kerf.Engine.Intersection.Walking.LineOn2S aNone = aLine.Split(4); // nothing moves
        Assert.Equal(0, aNone.PointCount);
        Assert.Equal(3, aLine.PointCount);

        global::Kerf.Engine.Intersection.Walking.LineOn2S anAll = aLine.Split(1); // everything moves
        Assert.Equal(0, aLine.PointCount);
        Assert.Equal(new[] { 0.0, 1, 2 }, Xs(anAll));

        // The split line has its own boxes: they cover its points only.
        Assert.False(anAll.IsOutBox(new Point3D(2, 0, 0)));
        Assert.True(anAll.IsOutBox(new Point3D(2.1, 0, 0)));
    }

    [Fact]
    public void SetUV_changes_one_surface_s_parameters_in_place()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        aLine.Add(IntSurf_LineOn2S_Test.buildPoint(new Point3D(1, 2, 3), 1, 2, 3, 4));
        aLine.SetUV(1, true, 10, 20);
        aLine.Value(1).Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.Equal((10.0, 20.0, 3.0, 4.0), (u1, v1, u2, v2));
        aLine.SetUV(1, false, 30, 40);
        aLine.Value(1).Parameters(out u1, out v1, out u2, out v2);
        Assert.Equal((10.0, 20.0, 30.0, 40.0), (u1, v1, u2, v2));
        Assert.Equal(new Point3D(1, 2, 3), aLine.Value(1).Value());
    }

    [Fact]
    public void The_boxes_are_enlarged_by_one_percent_of_their_largest_side()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        aLine.Add(IntSurf_LineOn2S_Test.buildPoint(new Point3D(0, 0, 0), 0, 0, 0, 0));
        aLine.Add(IntSurf_LineOn2S_Test.buildPoint(new Point3D(10, 2, 1), 10, 2, 4, 8));

        // 3D: largest side 10, gap 0.1 on every side
        Assert.False(aLine.IsOutBox(new Point3D(10.09, 2.09, 1.09)));
        Assert.True(aLine.IsOutBox(new Point3D(10.11, 0, 0)));
        Assert.True(aLine.IsOutBox(new Point3D(0, 0, -0.11)));
        // surface 1: largest side 10 (u), gap 0.1
        Assert.False(aLine.IsOutSurf1Box(new Point2D(-0.09, 2.09)));
        Assert.True(aLine.IsOutSurf1Box(new Point2D(0, 2.11)));
        // surface 2: largest side 8 (v), gap 0.08
        Assert.False(aLine.IsOutSurf2Box(new Point2D(4.07, 8.07)));
        Assert.True(aLine.IsOutSurf2Box(new Point2D(4.09, 0)));
    }

    [Fact]
    public void Additions_grow_a_computed_box_and_replacements_forget_it()
    {
        global::Kerf.Engine.Intersection.Walking.LineOn2S aLine = new();
        aLine.Add(IntSurf_LineOn2S_Test.buildPoint(new Point3D(0, 0, 0), 0, 0, 0, 0));
        aLine.Add(IntSurf_LineOn2S_Test.buildPoint(new Point3D(10, 0, 0), 10, 0, 10, 0));
        Assert.True(aLine.IsOutBox(new Point3D(20, 0, 0))); // computes the box, gap 0.1

        // Add grows the computed box (keeping its gap) instead of recomputing it.
        aLine.Add(IntSurf_LineOn2S_Test.buildPoint(new Point3D(20, 0, 0), 20, 0, 20, 0));
        Assert.False(aLine.IsOutBox(new Point3D(20.09, 0, 0)));
        Assert.True(aLine.IsOutBox(new Point3D(20.15, 0, 0)));

        // RemovePoint forgets it: the next query recomputes it with a gap of 0.2.
        aLine.RemovePoint(2);
        Assert.False(aLine.IsOutBox(new Point3D(20.15, 0, 0)));

        // SetUV grows the box of its surface only
        Assert.True(aLine.IsOutSurf1Box(new Point2D(30, 0)));
        aLine.SetUV(1, true, 30, 0);
        Assert.False(aLine.IsOutSurf1Box(new Point2D(30, 0)));
    }
}

/// <summary>IntSurf's points, transitions and the package's statics.</summary>
public class IntSurfTests
{
    [Fact]
    public void PntOn2S_sets_and_compares_as_OCCT()
    {
        global::Kerf.Engine.Intersection.Walking.PntOn2S aP = new();
        Assert.Equal(new Point3D(0, 0, 0), aP.Value());
        aP.SetValue(new Point3D(1, 2, 3), true, 4, 5);
        aP.SetValue(false, 6, 7);
        Assert.Equal(new Point2D(4, 5), aP.ValueOnSurface(true));
        Assert.Equal(new Point2D(6, 7), aP.ValueOnSurface(false));
        aP.ParametersOnSurface(false, out double u, out double v);
        Assert.Equal((6.0, 7.0), (u, v));

        global::Kerf.Engine.Intersection.Walking.PntOn2S aQ = aP.WithValue(new Point3D(1, 2, 3.001), 4, 5.01, 6, 7);
        Assert.False(aP.IsSame(aQ)); // 3D tolerance 0
        Assert.True(aP.IsSame(aQ, 0.002)); // 2D not compared
        Assert.False(aP.IsSame(aQ, 0.002, 0.001));
        Assert.True(aP.IsSame(aQ, 0.002, 0.02));
        Assert.Equal(new Point3D(1, 2, 3), aP.Value()); // WithValue copies
    }

    [Fact]
    public void Transition_queries_throw_where_OCCT_s_do()
    {
        global::Kerf.Engine.Intersection.Walking.WalkingTransition anUndecided = new();
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Undecided, anUndecided.TransitionType());
        EngineAssert.ThrowsAny<DomainErrorException>(() => anUndecided.IsTangent);
        EngineAssert.ThrowsAny<DomainErrorException>(() => anUndecided.Situation());

        global::Kerf.Engine.Intersection.Walking.WalkingTransition anIn = new(true, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In);
        Assert.True(anIn.IsTangent);
        EngineAssert.ThrowsAny<DomainErrorException>(() => anIn.IsOpposite);

        global::Kerf.Engine.Intersection.Walking.WalkingTransition aTouch = new(false, global::Kerf.Engine.Intersection.Walking.WalkingSituation.Outside, true);
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Touch, aTouch.TransitionType());
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingSituation.Outside, aTouch.Situation());
        Assert.True(aTouch.IsOpposite);

        // SetValue() makes it undecided, keeping the other fields (OCCT).
        aTouch.SetValue();
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Undecided, aTouch.TransitionType());
        aTouch.SetValue(true, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out);
        Assert.False(aTouch.Equals(new global::Kerf.Engine.Intersection.Walking.WalkingTransition(true, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out))); // the touch fields are still there
    }

    [Fact]
    public void MakeTransition_by_the_mixed_product()
    {
        Direction3D aN = new(0, 0, 1);
        global::Kerf.Engine.Intersection.Walking.WalkingTransition aT1 = new(), aT2 = new();

        // line along x, restriction along y: (y ^ x) . z < 0 -> line out, arc in
        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), aN, ref aT1, ref aT2);
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In), (aT1.TransitionType(), aT2.TransitionType()));
        Assert.False(aT1.IsTangent);

        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(new Vector3D(0, 1, 0), new Vector3D(1, 0, 0), aN, ref aT1, ref aT2);
        Assert.Equal((global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.In, global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Out), (aT1.TransitionType(), aT2.TransitionType()));

        // collinear tangents: touch, unknown, opposite when they point away from each other
        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(new Vector3D(1, 0, 0), new Vector3D(-2, 0, 0), aN, ref aT1, ref aT2);
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Touch, aT1.TransitionType());
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingSituation.Unknown, aT1.Situation());
        Assert.True(aT1.IsOpposite);
        Assert.True(aT2.IsTangent);

        // a null line tangent: undecided
        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(new Vector3D(0, 0, 0), new Vector3D(1, 0, 0), aN, ref aT1, ref aT2);
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Undecided, aT1.TransitionType());

        // tangents in the plane of the normal: |yu| <= 1e-4, undecided
        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(new Vector3D(1, 0, 0), new Vector3D(0, 0, 1), aN, ref aT1, ref aT2);
        Assert.Equal(global::Kerf.Engine.Intersection.Walking.WalkingTypeTrans.Undecided, aT2.TransitionType());
    }

    [Fact]
    public void PathPoint_copies_share_their_parameters_as_OCCT_s_handle()
    {
        global::Kerf.Engine.Intersection.Walking.PathPoint aP = new(new Point3D(1, 2, 3), 0.5, 0.25);
        Assert.True(aP.IsTangent);
        Assert.True(aP.IsPassingPnt);
        EngineAssert.ThrowsAny<UndefinedDerivativeException>(() => aP.Direction3d());

        global::Kerf.Engine.Intersection.Walking.PathPoint aCopy = aP;
        aCopy.AddUV(0.75, 0.125);
        Assert.Equal(1, aP.Multiplicity()); // seen through the shared sequence
        aP.Parameters(1, out double u, out double v);
        Assert.Equal((0.75, 0.125), (u, v));

        aCopy.SetDirections(new Vector3D(1, 0, 0), new Direction2D(0, 1));
        Assert.False(aCopy.IsTangent);
        Assert.True(aP.IsTangent); // the flags are the copy's own
        Assert.Equal(new Direction2D(0, 1), global::Kerf.Engine.Intersection.Walking.PathPointTool.Direction2d(aCopy));

        aCopy.SetValue(new Point3D(), 9, 9); // a new sequence
        Assert.Equal(0, aCopy.Multiplicity());
        Assert.Equal(1, aP.Multiplicity());
        global::Kerf.Engine.Intersection.Walking.PathPointTool.Value2d(aP, out u, out v);
        Assert.Equal((0.5, 0.25), (u, v));
    }

    [Fact]
    public void InteriorPoint_and_its_tool()
    {
        global::Kerf.Engine.Intersection.Walking.InteriorPoint aP = new(new Point3D(1, 2, 3), 4, 5, new Vector3D(0, 0, 2), new Vector2D(3, 4));
        global::Kerf.Engine.Intersection.Walking.InteriorPointTool.Value2d(aP, out double u, out double v);
        Assert.Equal((4.0, 5.0), (u, v));
        Assert.Equal(new Direction2D(0.6, 0.8), global::Kerf.Engine.Intersection.Walking.InteriorPointTool.Direction2d(aP));
        Assert.Equal(new Vector3D(0, 0, 2), global::Kerf.Engine.Intersection.Walking.InteriorPointTool.Direction3d(aP));
    }

    [Fact]
    public void SetPeriod_reads_both_surfaces()
    {
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 2));
        SurfaceAdaptor aPln = new(new PlaneSurface(new Plane3D()));
        Span<double> aPeriods = stackalloc double[4];
        global::Kerf.Engine.Intersection.Walking.IntSurf.SetPeriod(aCyl, aPln, aPeriods);
        Assert.Equal(new[] { 2 * Math.PI, 0, 0, 0 }, aPeriods.ToArray());
    }
}
