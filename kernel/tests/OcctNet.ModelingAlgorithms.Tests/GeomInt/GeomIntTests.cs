// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace OcctNet.ModelingAlgorithms.Tests;

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
        Assert.True(GeomInt.AdjustPeriodic(-1.0, 0.0, aPer, aPer, out double aNew, out double aOff));
        Assert.Equal(aPer, aOff);
        Assert.Equal(-1.0 + aPer, aNew);
        // Above the range: dp = -7, modf gives -1, a negative offset (returns false).
        Assert.False(GeomInt.AdjustPeriodic(7.0, 0.0, aPer, aPer, out aNew, out aOff));
        Assert.Equal(-aPer, aOff);
        Assert.Equal(7.0 - aPer, aNew);
        // Far below: dp / period = 2.something, two periods.
        Assert.True(GeomInt.AdjustPeriodic(-8.0, 0.0, aPer, aPer, out aNew, out aOff));
        Assert.Equal(2.0 * aPer, aOff);
        // Inside, or outside by less than the tolerance: unchanged.
        Assert.False(GeomInt.AdjustPeriodic(1.0, 0.0, aPer, aPer, out aNew, out aOff));
        Assert.Equal((1.0, 0.0), (aNew, aOff));
        Assert.False(GeomInt.AdjustPeriodic(-1.0e-10, 0.0, aPer, aPer, out aNew, out aOff, 1.0e-9));
        Assert.Equal((-1.0e-10, 0.0), (aNew, aOff));
        // Outside by less than a period from the far bound: modf gives 0, nothing moves.
        Assert.False(GeomInt.AdjustPeriodic(-1.0, 0.0, 1.0, 10.0, out aNew, out aOff));
        Assert.Equal((-1.0, 0.0), (aNew, aOff));
    }

    [Fact]
    public void ParameterAndOrientation_ValueSemantics()
    {
        GeomInt_ParameterAndOrientation aDef = new();
        Assert.Equal(0.0, aDef.Parameter());
        Assert.Equal(TopAbs_FORWARD, aDef.Orientation1());
        Assert.Equal(TopAbs_FORWARD, aDef.Orientation2());
        Assert.Equal(default(GeomInt_ParameterAndOrientation), aDef);

        GeomInt_ParameterAndOrientation aP = new(2.5, TopAbs_REVERSED, TopAbs_INTERNAL);
        Assert.Equal(2.5, aP.Parameter());
        Assert.Equal(TopAbs_REVERSED, aP.Orientation1());
        Assert.Equal(TopAbs_INTERNAL, aP.Orientation2());

        GeomInt_ParameterAndOrientation aQ = aP.WithOrientation1(TopAbs_FORWARD).WithOrientation2(TopAbs_EXTERNAL);
        Assert.Equal(TopAbs_REVERSED, aP.Orientation1()); // the original is unchanged
        Assert.Equal(new GeomInt_ParameterAndOrientation(2.5, TopAbs_FORWARD, TopAbs_EXTERNAL), aQ);
        Assert.True(aQ != aP);

        aP.SetOrientation2(TopAbs_FORWARD);
        Assert.Equal(TopAbs_FORWARD, aP.Orientation2());
        Assert.Contains("2.5", aP.ToString());
    }

    [Fact]
    public void MakeBSpline_And2d_ThroughTheWalkingPoints()
    {
        IntPatch_WLine aW = new(IntPatch_WLineTests.StraightLine(5), false);

        Geom_BSplineCurve aC = Assert.IsType<Geom_BSplineCurve>(GeomInt_IntSS.MakeBSpline(aW, 2, 4));
        Assert.Equal(1, aC.Degree());
        Assert.Equal(3, aC.NbPoles());
        Assert.Equal(new gp_Pnt(1, 0, 0), aC.Pole(1));
        Assert.Equal(new gp_Pnt(3, 0, 0), aC.Pole(3));
        Assert.Equal([0.0, 1.0, 2.0], aC.Knots().AsSpan().ToArray());
        Assert.Equal([2, 1, 2], aC.Multiplicities().AsSpan().ToArray());
        Assert.False(aC.IsFrozen());

        Geom2d_BSplineCurve aC1 = GeomInt_IntSS.MakeBSpline2d(aW, 2, 4, true);
        Geom2d_BSplineCurve aC2 = GeomInt_IntSS.MakeBSpline2d(aW, 2, 4, false);
        Assert.Equal(new gp_Pnt2d(1, 0), aC1.Pole(1));
        Assert.Equal(new gp_Pnt2d(3, 0), aC1.Pole(3));
        Assert.Equal(new gp_Pnt2d(1, 1), aC2.Pole(1));
        Assert.Equal(new gp_Pnt2d(3, 1), aC2.Pole(3));
        Assert.Equal([0.0, 1.0, 2.0], aC2.Knots().AsSpan().ToArray());
    }

    [Fact]
    public void BuildPCurves_ProjectsALineOnAPlane()
    {
        Geom_Plane aPlane = new(new gp_Pln());
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 1, 0));
        double aTol = 1.0e-7;
        Geom2d_Curve? aC2d = null;
        GeomInt_IntSS.BuildPCurves(0.0, 1.0, ref aTol, aPlane, aLine, ref aC2d);
        Assert.NotNull(aC2d);
        double s = 0.5 / Math.Sqrt(2.0);
        Assert.True(aC2d.Value(0.5).IsEqual(new gp_Pnt2d(s, s), Tol));

        // A 2D curve already given, or no surface: nothing changes.
        Geom2d_Curve? aSame = aC2d;
        GeomInt_IntSS.BuildPCurves(0.0, 1.0, ref aTol, aPlane, aLine, ref aSame);
        Assert.Same(aC2d, aSame);
        Geom2d_Curve? aNone = null;
        GeomInt_IntSS.BuildPCurves(0.0, 1.0, ref aTol, null, aLine, ref aNone);
        Assert.Null(aNone);
    }

    [Fact]
    public void BuildPCurves_TinyRangeIsASegment()
    {
        // A range shorter than 2e-9: the ends projected by Extrema_ExtPS make a 2-pole B-spline on [f, l].
        Geom_Plane aPlane = new(new gp_Pln());
        Geom_Line aLine = new(new gp_Pnt(1, 2, 0), new gp_Dir(1, 0, 0));
        double aTol = 1.0e-7;
        Geom2d_Curve? aC2d = null;
        GeomInt_IntSS.BuildPCurves(0.0, 1.0e-9, -10, 10, -10, 10, ref aTol, aPlane, aLine, ref aC2d);
        Geom2d_BSplineCurve aBS = Assert.IsType<Geom2d_BSplineCurve>(aC2d);
        Assert.Equal(2, aBS.NbPoles());
        Assert.Equal([0.0, 1.0e-9], aBS.Knots().AsSpan().ToArray());
        Assert.True(aBS.Pole(1).IsEqual(new gp_Pnt2d(1, 2), 1.0e-9));
        Assert.Equal(1.0e-7, aTol, 1.0e-12);
    }

    [Fact]
    public void BuildPCurves_MovesIntoThePeriodicRange()
    {
        // A circle on a cylinder, projected on [0, π], then moved by a period into u in [2π, 4π].
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(), 1.0);
        Geom_Circle aCirc = new(new gp_Ax2(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1)), 1.0);
        double aTol = 1.0e-7;
        Geom2d_Curve? aC2d = null;
        GeomInt_IntSS.BuildPCurves(0.0, Math.PI, 2.0 * Math.PI, 4.0 * Math.PI, -10, 10, ref aTol, aCyl, aCirc,
                                   ref aC2d);
        Assert.NotNull(aC2d);
        gp_Pnt2d aMid = aC2d.Value(0.5 * Math.PI);
        Assert.Equal(2.5 * Math.PI, aMid.X(), 1.0e-9);
        Assert.Equal(1.0, aMid.Y(), 1.0e-9);
    }

    [Fact]
    public void TrimILineOnSurfBoundaries_SortsTheCrossings()
    {
        // A horizontal 2D line through the box [0, 1] x [0, 1]: it crosses u = 0 at 1 and u = 1 at 2.
        Geom2d_Line aL = new(new gp_Pnt2d(-1, 0.5), new gp_Dir2d(1, 0));
        Bnd_Box2d aBox1 = new(), aBox2 = new();
        aBox1.Add(new gp_Pnt2d(0, 0));
        aBox1.Add(new gp_Pnt2d(1, 1));
        aBox2.Add(new gp_Pnt2d(5, 5));
        aBox2.Add(new gp_Pnt2d(6, 6));
        NCollection_DynamicArray<double> aPars = new();
        aPars.Append(5.0);
        aPars.Append(0.0);
        GeomInt_IntSS.TrimILineOnSurfBoundaries(aL, null, aBox1, aBox2, aPars);
        Assert.Equal(4, aPars.Length());
        double[] anExpected = [0.0, 1.0, 2.0, 5.0];
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(anExpected[i], aPars.Value(i), 1.0e-9);
        }
    }

    [Fact]
    public void LineTool_ReadsAWalkingLine()
    {
        IntPatch_WLine aW = new(IntPatch_WLineTests.StraightLine(6), false);
        Assert.Equal(0, GeomInt_LineTool.NbVertex(aW));
        Assert.Equal(1.0, GeomInt_LineTool.FirstParameter(aW));
        Assert.Equal(6.0, GeomInt_LineTool.LastParameter(aW));

        aW.AddVertex(IntPatch_WLineTests.Vtx(1, 2.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(3, 4.0));
        aW.SetFirstPoint(1);
        aW.SetLastPoint(2);
        Assert.Equal(2, GeomInt_LineTool.NbVertex(aW));
        Assert.Equal(4.0, GeomInt_LineTool.Vertex(aW, 2).ParameterOnLine());
        Assert.Equal(2.0, GeomInt_LineTool.FirstParameter(aW));
        Assert.Equal(4.0, GeomInt_LineTool.LastParameter(aW));

        IntPatch_GLine aLin = new(new gp_Lin(), false);
        Assert.Equal(-Precision.Infinite(), GeomInt_LineTool.FirstParameter(aLin));
        Assert.Equal(Precision.Infinite(), GeomInt_LineTool.LastParameter(aLin));
        IntPatch_GLine aCirc = new(new gp_Circ(new gp_Ax2(), 1.0), false);
        Assert.Equal(0.0, GeomInt_LineTool.FirstParameter(aCirc));
        Assert.Equal(2.0 * Math.PI, GeomInt_LineTool.LastParameter(aCirc));
    }

    private static (GeomInt_LineConstructor, GeomAdaptor_Surface, GeomAdaptor_Surface) PlanePlane()
    {
        // The same plane twice: the first bounded by [0, 10] x [-1, 1], the second by [0, 4] x [0, 2].
        Geom_Plane aPlane = new(new gp_Pln());
        GeomAdaptor_Surface aHS1 = new(aPlane, 0, 10, -1, 1);
        GeomAdaptor_Surface aHS2 = new(aPlane, 0, 4, 0, 2);
        GeomInt_LineConstructor aLC = new();
        aLC.Load(new Adaptor3d_TopolTool(aHS1), new Adaptor3d_TopolTool(aHS2), aHS1, aHS2);
        return (aLC, aHS1, aHS2);
    }

    [Fact]
    public void LineConstructor_KeepsTheWalkingPartsInsideBothDomains()
    {
        (GeomInt_LineConstructor aLC, GeomAdaptor_Surface aHS1, GeomAdaptor_Surface aHS2) = PlanePlane();
        Assert.False(aLC.IsDone());
        Assert.Throws<StdFail_NotDone>(() => aLC.NbParts());

        // Points x = 0..9 at (x, 0) on S1 and (x, 1) on S2; vertices at points 1, 5 and 10. The middle of [1, 5] is
        // point 3 (x = 2), inside both; the middle of [5, 10] is point 7 (x = 6), outside the second domain.
        IntPatch_WLine aW = new(IntPatch_WLineTests.StraightLine(10), false);
        aW.AddVertex(IntPatch_WLineTests.Vtx(0, 1.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(4, 5.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(9, 10.0));
        aLC.Perform(aW);
        Assert.True(aLC.IsDone());
        Assert.Equal(1, aLC.NbParts());
        aLC.Part(1, out double aF, out double aL);
        Assert.Equal((1.0, 5.0), (aF, aL));

        // No periodic surface: the line isn't decomposed.
        NCollection_Sequence<IntPatch_Line> aNew = new();
        Assert.False(GeomInt_LineTool.DecompositionOfWLine(aW, aHS1, aHS2, 2.0e-7, aLC, aNew));
        Assert.Equal(0, aNew.Length());
    }

    [Fact]
    public void LineConstructor_TwoPointPartsTestBothEnds()
    {
        (GeomInt_LineConstructor aLC, _, _) = PlanePlane();
        // Consecutive vertices at points 4 and 5 (x = 3 and 4, both in), then 5 and 6 (x = 5 is out of S2).
        IntPatch_WLine aW = new(IntPatch_WLineTests.StraightLine(10), false);
        aW.AddVertex(IntPatch_WLineTests.Vtx(3, 4.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(4, 5.0));
        aW.AddVertex(IntPatch_WLineTests.Vtx(5, 6.0));
        aLC.Perform(aW);
        Assert.Equal(1, aLC.NbParts());
        aLC.Part(1, out double aF, out double aL);
        Assert.Equal((4.0, 5.0), (aF, aL));

        // An ImpPrm line classifies the middle of the two points instead: x = 4.5 is out of S2 as well.
        aW.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpPrm);
        aLC.Perform(aW);
        Assert.Equal(1, aLC.NbParts());
    }

    [Fact]
    public void LineConstructor_TreatsACircleBySortedVertices()
    {
        // The unit circle in z = 0 on a cylinder of radius 1 (v in [-1, 1]) and on the plane z = 0 bounded by
        // x in [0, 2]: vertices at π/2 and 3π/2 (given in that order), the arc [3π/2, 5π/2] is kept.
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(), 1.0);
        Geom_Plane aPlane = new(new gp_Pln());
        GeomAdaptor_Surface aHS1 = new(aCyl, 0, 2.0 * Math.PI, -1, 1);
        GeomAdaptor_Surface aHS2 = new(aPlane, 0, 2, -2, 2);
        GeomInt_LineConstructor aLC = new();
        aLC.Load(new Adaptor3d_TopolTool(aHS1), new Adaptor3d_TopolTool(aHS2), aHS1, aHS2);

        IntPatch_GLine aG = new(new gp_Circ(new gp_Ax2(), 1.0), false);
        IntPatch_Point aV1 = new();
        aV1.SetValue(new gp_Pnt(0, -1, 0), 1.0e-7, false);
        aV1.SetParameter(1.5 * Math.PI);
        IntPatch_Point aV2 = new();
        aV2.SetValue(new gp_Pnt(0, 1, 0), 1.0e-7, false);
        aV2.SetParameter(0.5 * Math.PI);
        aG.AddVertex(aV1);
        aG.AddVertex(aV2);
        aLC.Perform(aG);
        Assert.True(aLC.IsDone());
        Assert.Equal(1, aLC.NbParts());
        aLC.Part(1, out double aF, out double aL);
        Assert.Equal(1.5 * Math.PI, aF, Tol);
        Assert.Equal(2.5 * Math.PI, aL, Tol);

        // A circle smaller than the tolerance is rejected.
        IntPatch_GLine aMicro = new(new gp_Circ(new gp_Ax2(), 1.0e-12), false);
        aLC.Perform(aMicro);
        Assert.True(aLC.IsDone());
        Assert.Equal(0, aLC.NbParts());
    }

    [Fact]
    public void IntSS_TolFixTangents()
    {
        GeomInt_IntSS anInt = new();
        anInt.TolFixTangents(out double aTol, out double anAng);
        Assert.Equal((1.0e-7, 1.0e-6), (aTol, anAng));
        anInt.SetTolFixTangents(1.0e-5, 1.0e-4);
        anInt.TolFixTangents(out aTol, out anAng);
        Assert.Equal((1.0e-5, 1.0e-4), (aTol, anAng));
    }
}
