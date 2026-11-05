// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Measurement.StatusErrorType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>LocalAnalysis_CurveContinuity on curves whose continuity is known in closed form, and its Dump.</summary>
public class LocalAnalysis_CurveContinuityTests
{
    // circle of radius 1 about the origin and circle of radius 2 about (-1, 0, 0), both in XY: they touch at (1, 0, 0)
    // at parameter 0, with the same tangent (+y) and the same normal (-x)
    private static readonly CircleCurve3D s_c1 = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0);
    private static readonly CircleCurve3D s_c2 = new(new Frame3D(new Point3D(-1, 0, 0), new Direction3D(0, 0, 1)), 2.0);

    [Fact]
    public void C0_IsTheDistanceOfThePoints()
    {
        var aLine = new LineCurve3D(new Point3D(1, 0.5, 0), new Direction3D(0, 1, 0));
        var c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, aLine, 0.0, C0);
        Assert.True(c.IsDone);
        Assert.Equal(C0, c.ContinuityStatus());
        Assert.Equal(0.5, c.C0Value(), 1e-15);
        Assert.False(c.IsC0());
        c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, s_c2, 0.0, C0);
        Assert.Equal(0.0, c.C0Value(), 1e-15);
        Assert.True(c.IsC0());
    }

    [Fact]
    public void C1_AndC2_CompareTheDerivatives()
    {
        var c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, s_c2, 0.0, C2);
        Assert.True(c.IsDone);
        Assert.Equal(0.0, c.C1Angle(), 1e-15);
        Assert.Equal(0.5, c.C1Ratio(), 1e-15); // |D1| = 1 and 2
        Assert.Equal(0.0, c.C2Angle(), 1e-15);
        Assert.Equal(0.5, c.C2Ratio(), 1e-15); // |D2| = 1 and 2
        Assert.True(c.IsC1()); // C1 tests the angle only
        Assert.False(c.IsC2()); // Lambda1^2 = 0.25 against Lambda2 = 0.5

        // opposite directions: the angle is folded into [0, PI/2]
        var aReversed = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, -1), new Direction3D(1, 0, 0)), 1.0);
        c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, aReversed, 0.0, C1);
        Assert.Equal(0.0, c.C1Angle(), 1e-15);
        Assert.Equal(1.0, c.C1Ratio());
        Assert.True(c.IsC1());

        // a line at 30 degrees to the circle's tangent
        var aLine = new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(Math.Sin(Math.PI / 6), Math.Cos(Math.PI / 6), 0));
        c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, aLine, 0.0, C1);
        Assert.Equal(Math.PI / 6, c.C1Angle(), 1e-14);
        Assert.False(c.IsC1());
        Assert.True(c.IsC0());
    }

    [Fact]
    public void C2_OfTheSameCircle_IsC2()
    {
        var c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.3, s_c1, 0.3, C2);
        Assert.True(c.IsC2());
        Assert.Equal(1.0, c.C1Ratio());
        Assert.Equal(1.0, c.C2Ratio());
    }

    [Fact]
    public void G1_AndG2_CompareTangentsNormalsAndCurvatures()
    {
        var c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, s_c2, 0.0, G2);
        Assert.True(c.IsDone);
        Assert.Equal(0.0, c.G1Angle(), 1e-15);
        Assert.Equal(0.0, c.G2Angle(), 1e-15);
        // |k1 - k2| / sqrt(k1 k2) with curvatures 1 and 0.5
        Assert.Equal(0.5 / Math.Sqrt(0.5), c.G2CurvatureVariation(), 1e-14);
        Assert.True(c.IsG1());
        Assert.False(c.IsG2());
        Assert.True(new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, s_c2, 0.0, G2, percent: 0.8).IsG2());

        var aLine = new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0, -1, 0));
        c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, aLine, 0.0, G1);
        Assert.Equal(0.0, c.G1Angle(), 1e-15);
        Assert.True(c.IsG1());
    }

    [Fact]
    public void G2_OfTwoLines_FailsWithNoNormal()
    {
        var aL1 = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var aL2 = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var c = new global::Kerf.Engine.Measurement.CurveContinuity(aL1, 0.0, aL2, 0.0, G2);
        Assert.False(c.IsDone);
        Assert.Equal(global::Kerf.Engine.Measurement.StatusErrorType.NormalNotDefined, c.StatusError());
        EngineAssert.Throws<NotDoneException>(() => c.C0Value());
        EngineAssert.Throws<NotDoneException>(() => c.IsC0());
        EngineAssert.Throws<NotDoneException>(() => c.ContinuityStatus());
    }

    [Fact]
    public void NullDerivatives_AreReported()
    {
        // a Bezier curve with a double first pole: its first derivative at 0 is null
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(0, 0, 0);
        aPoles[3] = new Point3D(1, 1, 0);
        aPoles[4] = new Point3D(2, 0, 0);
        var aBez = new BezierCurve3D(aPoles);
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));

        var c = new global::Kerf.Engine.Measurement.CurveContinuity(aBez, 0.0, aLine, 0.0, C1);
        Assert.False(c.IsDone);
        Assert.Equal(global::Kerf.Engine.Measurement.StatusErrorType.NullFirstDerivative, c.StatusError());

        // a line's second derivative is null
        c = new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, aLine, 0.0, C2);
        Assert.False(c.IsDone);
        Assert.Equal(global::Kerf.Engine.Measurement.StatusErrorType.NullSecondDerivative, c.StatusError());

        // the tangent at the double pole is still defined (CLProps computes the second derivative on demand), so G1 is
        // computed
        c = new global::Kerf.Engine.Measurement.CurveContinuity(aBez, 0.0, aLine, 0.0, G1);
        Assert.True(c.IsDone);
        Assert.Equal(Math.PI / 4, c.G1Angle(), 1e-14);
        Assert.False(c.IsG1());
    }

    [Fact]
    public void Dump_WritesOcctsText()
    {
        var w = new StringWriter();
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, s_c2, 0.0, C1), w);
        Assert.Equal(" Continuity Status : C1 \n C0Value = 0\n C1Angle = 0\n C1Ratio = 0.5\n", w.ToString());

        w = new StringWriter();
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(new global::Kerf.Engine.Measurement.CurveContinuity(s_c1, 0.0, s_c2, 0.0, G2), w);
        Assert.Equal(" Continuity Status : G1 \n G0Value = 0\n G1Angle = 0\n G2Angle = 0\n Relative curvature variation = 0.707107\n",
                     w.ToString());

        w = new StringWriter();
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(new global::Kerf.Engine.Measurement.CurveContinuity(aLine, 0.0, aLine, 0.0, G2), w);
        Assert.Equal("Problem in the computation \none (or both) normal is undefined\n", w.ToString());
    }
}
