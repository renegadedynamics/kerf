// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.LocalAnalysis_StatusErrorType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>LocalAnalysis_CurveContinuity on curves whose continuity is known in closed form, and its Dump.</summary>
public class LocalAnalysis_CurveContinuityTests
{
    // circle of radius 1 about the origin and circle of radius 2 about (-1, 0, 0), both in XY: they touch at (1, 0, 0)
    // at parameter 0, with the same tangent (+y) and the same normal (-x)
    private static readonly Geom_Circle s_c1 = new(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);
    private static readonly Geom_Circle s_c2 = new(new gp_Ax2(new gp_Pnt(-1, 0, 0), new gp_Dir(0, 0, 1)), 2.0);

    [Fact]
    public void C0_IsTheDistanceOfThePoints()
    {
        var aLine = new Geom_Line(new gp_Pnt(1, 0.5, 0), new gp_Dir(0, 1, 0));
        var c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, aLine, 0.0, GeomAbs_C0);
        Assert.True(c.IsDone());
        Assert.Equal(GeomAbs_C0, c.ContinuityStatus());
        Assert.Equal(0.5, c.C0Value(), 1e-15);
        Assert.False(c.IsC0());
        c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, s_c2, 0.0, GeomAbs_C0);
        Assert.Equal(0.0, c.C0Value(), 1e-15);
        Assert.True(c.IsC0());
    }

    [Fact]
    public void C1_AndC2_CompareTheDerivatives()
    {
        var c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, s_c2, 0.0, GeomAbs_C2);
        Assert.True(c.IsDone());
        Assert.Equal(0.0, c.C1Angle(), 1e-15);
        Assert.Equal(0.5, c.C1Ratio(), 1e-15); // |D1| = 1 and 2
        Assert.Equal(0.0, c.C2Angle(), 1e-15);
        Assert.Equal(0.5, c.C2Ratio(), 1e-15); // |D2| = 1 and 2
        Assert.True(c.IsC1()); // C1 tests the angle only
        Assert.False(c.IsC2()); // Lambda1^2 = 0.25 against Lambda2 = 0.5

        // opposite directions: the angle is folded into [0, PI/2]
        var aReversed = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, -1), new gp_Dir(1, 0, 0)), 1.0);
        c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, aReversed, 0.0, GeomAbs_C1);
        Assert.Equal(0.0, c.C1Angle(), 1e-15);
        Assert.Equal(1.0, c.C1Ratio());
        Assert.True(c.IsC1());

        // a line at 30 degrees to the circle's tangent
        var aLine = new Geom_Line(new gp_Pnt(1, 0, 0), new gp_Dir(Math.Sin(Math.PI / 6), Math.Cos(Math.PI / 6), 0));
        c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, aLine, 0.0, GeomAbs_C1);
        Assert.Equal(Math.PI / 6, c.C1Angle(), 1e-14);
        Assert.False(c.IsC1());
        Assert.True(c.IsC0());
    }

    [Fact]
    public void C2_OfTheSameCircle_IsC2()
    {
        var c = new LocalAnalysis_CurveContinuity(s_c1, 0.3, s_c1, 0.3, GeomAbs_C2);
        Assert.True(c.IsC2());
        Assert.Equal(1.0, c.C1Ratio());
        Assert.Equal(1.0, c.C2Ratio());
    }

    [Fact]
    public void G1_AndG2_CompareTangentsNormalsAndCurvatures()
    {
        var c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, s_c2, 0.0, GeomAbs_G2);
        Assert.True(c.IsDone());
        Assert.Equal(0.0, c.G1Angle(), 1e-15);
        Assert.Equal(0.0, c.G2Angle(), 1e-15);
        // |k1 - k2| / sqrt(k1 k2) with curvatures 1 and 0.5
        Assert.Equal(0.5 / Math.Sqrt(0.5), c.G2CurvatureVariation(), 1e-14);
        Assert.True(c.IsG1());
        Assert.False(c.IsG2());
        Assert.True(new LocalAnalysis_CurveContinuity(s_c1, 0.0, s_c2, 0.0, GeomAbs_G2, Percent: 0.8).IsG2());

        var aLine = new Geom_Line(new gp_Pnt(1, 0, 0), new gp_Dir(0, -1, 0));
        c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, aLine, 0.0, GeomAbs_G1);
        Assert.Equal(0.0, c.G1Angle(), 1e-15);
        Assert.True(c.IsG1());
    }

    [Fact]
    public void G2_OfTwoLines_FailsWithNoNormal()
    {
        var aL1 = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        var aL2 = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        var c = new LocalAnalysis_CurveContinuity(aL1, 0.0, aL2, 0.0, GeomAbs_G2);
        Assert.False(c.IsDone());
        Assert.Equal(LocalAnalysis_NormalNotDefined, c.StatusError());
        Assert.Throws<StdFail_NotDone>(() => c.C0Value());
        Assert.Throws<StdFail_NotDone>(() => c.IsC0());
        Assert.Throws<StdFail_NotDone>(() => c.ContinuityStatus());
    }

    [Fact]
    public void NullDerivatives_AreReported()
    {
        // a Bezier curve with a double first pole: its first derivative at 0 is null
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(0, 0, 0);
        aPoles[3] = new gp_Pnt(1, 1, 0);
        aPoles[4] = new gp_Pnt(2, 0, 0);
        var aBez = new Geom_BezierCurve(aPoles);
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));

        var c = new LocalAnalysis_CurveContinuity(aBez, 0.0, aLine, 0.0, GeomAbs_C1);
        Assert.False(c.IsDone());
        Assert.Equal(LocalAnalysis_NullFirstDerivative, c.StatusError());

        // a line's second derivative is null
        c = new LocalAnalysis_CurveContinuity(s_c1, 0.0, aLine, 0.0, GeomAbs_C2);
        Assert.False(c.IsDone());
        Assert.Equal(LocalAnalysis_NullSecondDerivative, c.StatusError());

        // the tangent at the double pole is still defined (CLProps computes the second derivative on demand), so G1 is
        // computed
        c = new LocalAnalysis_CurveContinuity(aBez, 0.0, aLine, 0.0, GeomAbs_G1);
        Assert.True(c.IsDone());
        Assert.Equal(Math.PI / 4, c.G1Angle(), 1e-14);
        Assert.False(c.IsG1());
    }

    [Fact]
    public void Dump_WritesOcctsText()
    {
        var w = new StringWriter();
        LocalAnalysis.Dump(new LocalAnalysis_CurveContinuity(s_c1, 0.0, s_c2, 0.0, GeomAbs_C1), w);
        Assert.Equal(" Continuity Status : C1 \n C0Value = 0\n C1Angle = 0\n C1Ratio = 0.5\n", w.ToString());

        w = new StringWriter();
        LocalAnalysis.Dump(new LocalAnalysis_CurveContinuity(s_c1, 0.0, s_c2, 0.0, GeomAbs_G2), w);
        Assert.Equal(" Continuity Status : G1 \n G0Value = 0\n G1Angle = 0\n G2Angle = 0\n Relative curvature variation = 0.707107\n",
                     w.ToString());

        w = new StringWriter();
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        LocalAnalysis.Dump(new LocalAnalysis_CurveContinuity(aLine, 0.0, aLine, 0.0, GeomAbs_G2), w);
        Assert.Equal("Problem in the computation \none (or both) normal is undefined\n", w.ToString());
    }
}
