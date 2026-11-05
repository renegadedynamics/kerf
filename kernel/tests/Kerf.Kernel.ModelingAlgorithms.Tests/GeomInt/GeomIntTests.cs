// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Patches;
using Kerf.Geometry;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GeomInt's tools that don't need IntPatch_Intersection: the package's AdjustPeriodic, ParameterAndOrientation,
/// GeomInt_IntSS's static curve builders, GeomInt_LineTool's readers and GeomInt_LineConstructor on hand-made lines,
/// each traced through the C++.
/// </summary>
public class GeomIntTests
{
    private const double Tol = 1.0e-12;

    [Fact]
    public void AdjustPeriodic_ShiftsByWholePeriods()
    {
        double aPer = 2.0 * Math.PI;
        // Below the range: dp = 2π + 1, one period up, a positive offset.
        Assert.True(global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(-1.0, 0.0, aPer, aPer, out double aNew, out double aOff));
        Assert.Equal(aPer, aOff);
        Assert.Equal(-1.0 + aPer, aNew);
        // Above the range: dp = -7, modf gives -1, a negative offset (returns false).
        Assert.False(global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(7.0, 0.0, aPer, aPer, out aNew, out aOff));
        Assert.Equal(-aPer, aOff);
        Assert.Equal(7.0 - aPer, aNew);
        // Far below: dp / period = 2.something, two periods.
        Assert.True(global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(-8.0, 0.0, aPer, aPer, out aNew, out aOff));
        Assert.Equal(2.0 * aPer, aOff);
        // Inside, or outside by less than the tolerance: unchanged.
        Assert.False(global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(1.0, 0.0, aPer, aPer, out aNew, out aOff));
        Assert.Equal((1.0, 0.0), (aNew, aOff));
        Assert.False(global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(-1.0e-10, 0.0, aPer, aPer, out aNew, out aOff, 1.0e-9));
        Assert.Equal((-1.0e-10, 0.0), (aNew, aOff));
        // Outside by less than a period from the far bound: modf gives 0, nothing moves.
        Assert.False(global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(-1.0, 0.0, 1.0, 10.0, out aNew, out aOff));
        Assert.Equal((-1.0, 0.0), (aNew, aOff));
    }

    [Fact]
    public void ParameterAndOrientation_ValueSemantics()
    {
        global::Kerf.Engine.Intersection.Surfaces.ParameterAndOrientation aDef = new();
        Assert.Equal(0.0, aDef.Parameter());
        Assert.Equal(Forward, aDef.Orientation1());
        Assert.Equal(Forward, aDef.Orientation2());
        Assert.Equal(default(global::Kerf.Engine.Intersection.Surfaces.ParameterAndOrientation), aDef);

        global::Kerf.Engine.Intersection.Surfaces.ParameterAndOrientation aP = new(2.5, Reversed, Internal);
        Assert.Equal(2.5, aP.Parameter());
        Assert.Equal(Reversed, aP.Orientation1());
        Assert.Equal(Internal, aP.Orientation2());

        global::Kerf.Engine.Intersection.Surfaces.ParameterAndOrientation aQ = aP.WithOrientation1(Forward).WithOrientation2(External);
        Assert.Equal(Reversed, aP.Orientation1()); // the original is unchanged
        Assert.Equal(new global::Kerf.Engine.Intersection.Surfaces.ParameterAndOrientation(2.5, Forward, External), aQ);
        Assert.True(aQ != aP);

        aP.SetOrientation2(Forward);
        Assert.Equal(Forward, aP.Orientation2());
        Assert.Contains("2.5", aP.ToString());
    }

    [Fact]
    public void MakeBSpline_And2d_ThroughTheWalkingPoints()
    {
        WLine aW = new(IntPatch_WLineTests.StraightLine(5), false);

        BSplineCurve3D aC = Assert.IsType<BSplineCurve3D>(global::Kerf.Engine.Intersection.Surfaces.IntSS.MakeBSpline(aW, 2, 4));
        Assert.Equal(1, aC.Degree);
        Assert.Equal(3, aC.PoleCount);
        Assert.Equal(new Point3D(1, 0, 0), aC.Pole(1));
        Assert.Equal(new Point3D(3, 0, 0), aC.Pole(3));
        Assert.Equal([0.0, 1.0, 2.0], aC.Knots.ToArray());
        Assert.Equal([2, 1, 2], aC.Multiplicities.ToArray());
        Assert.False(aC.IsFrozen);

        BSplineCurve2D aC1 = global::Kerf.Engine.Intersection.Surfaces.IntSS.MakeBSpline2d(aW, 2, 4, true);
        BSplineCurve2D aC2 = global::Kerf.Engine.Intersection.Surfaces.IntSS.MakeBSpline2d(aW, 2, 4, false);
        Assert.Equal(new Point2D(1, 0), aC1.Pole(1));
        Assert.Equal(new Point2D(3, 0), aC1.Pole(3));
        Assert.Equal(new Point2D(1, 1), aC2.Pole(1));
        Assert.Equal(new Point2D(3, 1), aC2.Pole(3));
        Assert.Equal([0.0, 1.0, 2.0], aC2.Knots.ToArray());
    }

    [Fact]
    public void BuildPCurves_ProjectsALineOnAPlane()
    {
        PlaneSurface aPlane = new(new Plane3D());
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 1, 0));
        double aTol = 1.0e-7;
        Curve2D? aC2d = null;
        global::Kerf.Engine.Intersection.Surfaces.IntSS.BuildPCurves(0.0, 1.0, ref aTol, aPlane, aLine, ref aC2d);
        Assert.NotNull(aC2d);
        double s = 0.5 / Math.Sqrt(2.0);
        Assert.True(aC2d.Value(0.5).IsEqual(new Point2D(s, s), Tol));

        // A 2D curve already given, or no surface: nothing changes.
        Curve2D? aSame = aC2d;
        global::Kerf.Engine.Intersection.Surfaces.IntSS.BuildPCurves(0.0, 1.0, ref aTol, aPlane, aLine, ref aSame);
        Assert.Same(aC2d, aSame);
        Curve2D? aNone = null;
        global::Kerf.Engine.Intersection.Surfaces.IntSS.BuildPCurves(0.0, 1.0, ref aTol, null, aLine, ref aNone);
        Assert.Null(aNone);
    }

    [Fact]
    public void BuildPCurves_TinyRangeIsASegment()
    {
        // A range shorter than 2e-9: the ends projected by Extrema_ExtPS make a 2-pole B-spline on [f, l].
        PlaneSurface aPlane = new(new Plane3D());
        LineCurve3D aLine = new(new Point3D(1, 2, 0), new Direction3D(1, 0, 0));
        double aTol = 1.0e-7;
        Curve2D? aC2d = null;
        global::Kerf.Engine.Intersection.Surfaces.IntSS.BuildPCurves(0.0, 1.0e-9, -10, 10, -10, 10, ref aTol, aPlane, aLine, ref aC2d);
        BSplineCurve2D aBS = Assert.IsType<BSplineCurve2D>(aC2d);
        Assert.Equal(2, aBS.PoleCount);
        Assert.Equal([0.0, 1.0e-9], aBS.Knots.ToArray());
        Assert.True(aBS.Pole(1).IsEqual(new Point2D(1, 2), 1.0e-9));
        Assert.Equal(1.0e-7, aTol, 1.0e-12);
    }

    [Fact]
    public void BuildPCurves_MovesIntoThePeriodicRange()
    {
        // A circle on a cylinder, projected on [0, π], then moved by a period into u in [2π, 4π].
        CylindricalSurface aCyl = new(new CoordinateSystem(), 1.0);
        CircleCurve3D aCirc = new(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 1.0);
        double aTol = 1.0e-7;
        Curve2D? aC2d = null;
        global::Kerf.Engine.Intersection.Surfaces.IntSS.BuildPCurves(0.0, Math.PI, 2.0 * Math.PI, 4.0 * Math.PI, -10, 10, ref aTol, aCyl, aCirc,
                                   ref aC2d);
        Assert.NotNull(aC2d);
        Point2D aMid = aC2d.Value(0.5 * Math.PI);
        Assert.Equal(2.5 * Math.PI, aMid.X, 1.0e-9);
        Assert.Equal(1.0, aMid.Y, 1.0e-9);
    }

    [Fact]
    public void TrimILineOnSurfBoundaries_SortsTheCrossings()
    {
        // A horizontal 2D line through the box [0, 1] x [0, 1]: it crosses u = 0 at 1 and u = 1 at 2.
        LineCurve2D aL = new(new Point2D(-1, 0.5), new Direction2D(1, 0));
        Box2d aBox1 = new(), aBox2 = new();
        aBox1.Add(new Point2D(0, 0));
        aBox1.Add(new Point2D(1, 1));
        aBox2.Add(new Point2D(5, 5));
        aBox2.Add(new Point2D(6, 6));
        List<double> aPars = new();
        aPars.Add(5.0);
        aPars.Add(0.0);
        global::Kerf.Engine.Intersection.Surfaces.IntSS.TrimILineOnSurfBoundaries(aL, null, aBox1, aBox2, aPars);
        Assert.Equal(4, aPars.Count);
        double[] anExpected = [0.0, 1.0, 2.0, 5.0];
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(anExpected[i], aPars[i], 1.0e-9);
        }
    }

    [Fact]
    public void LineTool_ReadsAWalkingLine()
    {
        WLine aW = new(IntPatch_WLineTests.StraightLine(6), false);
        Assert.Equal(0, global::Kerf.Engine.Intersection.Surfaces.LineTool.NbVertex(aW));
        Assert.Equal(1.0, global::Kerf.Engine.Intersection.Surfaces.LineTool.FirstParameter(aW));
        Assert.Equal(6.0, global::Kerf.Engine.Intersection.Surfaces.LineTool.LastParameter(aW));

        aW.AddVertex(IntPatch_WLineTests.Vtx(1, 2.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(3, 4.0));
        aW.SetFirstPoint(1);
        aW.SetLastPoint(2);
        Assert.Equal(2, global::Kerf.Engine.Intersection.Surfaces.LineTool.NbVertex(aW));
        Assert.Equal(4.0, global::Kerf.Engine.Intersection.Surfaces.LineTool.Vertex(aW, 2).ParameterOnLine());
        Assert.Equal(2.0, global::Kerf.Engine.Intersection.Surfaces.LineTool.FirstParameter(aW));
        Assert.Equal(4.0, global::Kerf.Engine.Intersection.Surfaces.LineTool.LastParameter(aW));

        GLine aLin = new(new Line3D(), false);
        Assert.Equal(-global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Intersection.Surfaces.LineTool.FirstParameter(aLin));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Intersection.Surfaces.LineTool.LastParameter(aLin));
        GLine aCirc = new(new Circle3D(new Frame3D(), 1.0), false);
        Assert.Equal(0.0, global::Kerf.Engine.Intersection.Surfaces.LineTool.FirstParameter(aCirc));
        Assert.Equal(2.0 * Math.PI, global::Kerf.Engine.Intersection.Surfaces.LineTool.LastParameter(aCirc));
    }

    private static (global::Kerf.Engine.Intersection.Surfaces.LineConstructor, SurfaceAdaptor, SurfaceAdaptor) PlanePlane()
    {
        // The same plane twice: the first bounded by [0, 10] x [-1, 1], the second by [0, 4] x [0, 2].
        PlaneSurface aPlane = new(new Plane3D());
        SurfaceAdaptor aHS1 = new(aPlane, 0, 10, -1, 1);
        SurfaceAdaptor aHS2 = new(aPlane, 0, 4, 0, 2);
        global::Kerf.Engine.Intersection.Surfaces.LineConstructor aLC = new();
        aLC.Load(new TopolTool(aHS1), new TopolTool(aHS2), aHS1, aHS2);
        return (aLC, aHS1, aHS2);
    }

    [Fact]
    public void LineConstructor_KeepsTheWalkingPartsInsideBothDomains()
    {
        (global::Kerf.Engine.Intersection.Surfaces.LineConstructor aLC, SurfaceAdaptor aHS1, SurfaceAdaptor aHS2) = PlanePlane();
        Assert.False(aLC.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aLC.PartCount);

        // Points x = 0..9 at (x, 0) on S1 and (x, 1) on S2; vertices at points 1, 5 and 10. The middle of [1, 5] is
        // point 3 (x = 2), inside both; the middle of [5, 10] is point 7 (x = 6), outside the second domain.
        WLine aW = new(IntPatch_WLineTests.StraightLine(10), false);
        aW.AddVertex(IntPatch_WLineTests.Vtx(0, 1.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(4, 5.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(9, 10.0));
        aLC.Perform(aW);
        Assert.True(aLC.IsDone);
        Assert.Equal(1, aLC.PartCount);
        aLC.Part(1, out double aF, out double aL);
        Assert.Equal((1.0, 5.0), (aF, aL));

        // No periodic surface: the line isn't decomposed.
        NCollection_Sequence<PatchLine> aNew = new();
        Assert.False(global::Kerf.Engine.Intersection.Surfaces.LineTool.DecompositionOfWLine(aW, aHS1, aHS2, 2.0e-7, aLC, aNew));
        Assert.Equal(0, aNew.Length());
    }

    [Fact]
    public void LineConstructor_TwoPointPartsTestBothEnds()
    {
        (global::Kerf.Engine.Intersection.Surfaces.LineConstructor aLC, _, _) = PlanePlane();
        // Consecutive vertices at points 4 and 5 (x = 3 and 4, both in), then 5 and 6 (x = 5 is out of S2).
        WLine aW = new(IntPatch_WLineTests.StraightLine(10), false);
        aW.AddVertex(IntPatch_WLineTests.Vtx(3, 4.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(4, 5.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(5, 6.0));
        aLC.Perform(aW);
        Assert.Equal(1, aLC.PartCount);
        aLC.Part(1, out double aF, out double aL);
        Assert.Equal((4.0, 5.0), (aF, aL));

        // An ImpPrm line classifies the middle of the two points instead: x = 4.5 is out of S2 as well.
        aW.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpPrm);
        aLC.Perform(aW);
        Assert.Equal(1, aLC.PartCount);
    }

    [Fact]
    public void LineConstructor_TreatsACircleBySortedVertices()
    {
        // The unit circle in z = 0 on a cylinder of radius 1 (v in [-1, 1]) and on the plane z = 0 bounded by
        // x in [0, 2]: vertices at π/2 and 3π/2 (given in that order), the arc [3π/2, 5π/2] is kept.
        CylindricalSurface aCyl = new(new CoordinateSystem(), 1.0);
        PlaneSurface aPlane = new(new Plane3D());
        SurfaceAdaptor aHS1 = new(aCyl, 0, 2.0 * Math.PI, -1, 1);
        SurfaceAdaptor aHS2 = new(aPlane, 0, 2, -2, 2);
        global::Kerf.Engine.Intersection.Surfaces.LineConstructor aLC = new();
        aLC.Load(new TopolTool(aHS1), new TopolTool(aHS2), aHS1, aHS2);

        GLine aG = new(new Circle3D(new Frame3D(), 1.0), false);
        PatchPoint aV1 = new();
        aV1.SetValue(new Point3D(0, -1, 0), 1.0e-7, false);
        aV1.SetParameter(1.5 * Math.PI);
        PatchPoint aV2 = new();
        aV2.SetValue(new Point3D(0, 1, 0), 1.0e-7, false);
        aV2.SetParameter(0.5 * Math.PI);
        aG.AddVertex(aV1);
        aG.AddVertex(aV2);
        aLC.Perform(aG);
        Assert.True(aLC.IsDone);
        Assert.Equal(1, aLC.PartCount);
        aLC.Part(1, out double aF, out double aL);
        Assert.Equal(1.5 * Math.PI, aF, Tol);
        Assert.Equal(2.5 * Math.PI, aL, Tol);

        // A circle smaller than the tolerance is rejected.
        GLine aMicro = new(new Circle3D(new Frame3D(), 1.0e-12), false);
        aLC.Perform(aMicro);
        Assert.True(aLC.IsDone);
        Assert.Equal(0, aLC.PartCount);
    }

    [Fact]
    public void IntSS_TolFixTangents()
    {
        global::Kerf.Engine.Intersection.Surfaces.IntSS anInt = new();
        anInt.TolFixTangents(out double aTol, out double anAng);
        Assert.Equal((1.0e-7, 1.0e-6), (aTol, anAng));
        anInt.SetTolFixTangents(1.0e-5, 1.0e-4);
        anInt.TolFixTangents(out aTol, out anAng);
        Assert.Equal((1.0e-5, 1.0e-4), (aTol, anAng));
    }
}
