// Port of: src/FoundationClasses/TKMath/GTests/Convert_CircleToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace OcctNet.Foundation.Tests;

public class Convert_CircleToBSplineCurveTest
{
    [Fact]
    public void FullCircle_TgtThetaOver2()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 5.0);
        CircleToBSplineCurve aConv = new(aCirc, TgtThetaOver2);

        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.IsPeriodic);
        Assert.True(aConv.PoleCount > 0);
        Assert.True(aConv.KnotCount > 0);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.PoleCount; ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.KnotCount; ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void FullCircle_RationalC1()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0)), 3.0);
        CircleToBSplineCurve aConv = new(aCirc, RationalC1);

        Assert.True(aConv.IsPeriodic);
        Assert.True(aConv.PoleCount > 0);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.PoleCount; ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.KnotCount; ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void Arc_TgtThetaOver2()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 1.0);
        double aU1 = M_PI / 6.0;
        double aU2 = 5.0 * M_PI / 3.0;
        CircleToBSplineCurve aConv = new(aCirc, aU1, aU2, TgtThetaOver2);

        Assert.False(aConv.IsPeriodic);
        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.PoleCount > 0);

        // Verify first pole matches conic point at U1
        NCollection_Array1<Point2D> aPoles = aConv.Poles();
        double aR = aCirc.Radius();
        Point2D aCenter = aCirc.Location();
        Direction2D aXDir = aCirc.XAxis().Direction();
        Direction2D aYDir = aCirc.YAxis().Direction();

        double aFirstX = aCenter.X() + aR * (Math.Cos(aU1) * aXDir.X() + Math.Sin(aU1) * aYDir.X());
        double aFirstY = aCenter.Y() + aR * (Math.Cos(aU1) * aXDir.Y() + Math.Sin(aU1) * aYDir.Y());
        Assert.Equal(aFirstX, aPoles[1].X(), 1.0e-10);
        Assert.Equal(aFirstY, aPoles[1].Y(), 1.0e-10);

        // Verify last pole matches conic point at U2
        double aLastX = aCenter.X() + aR * (Math.Cos(aU2) * aXDir.X() + Math.Sin(aU2) * aYDir.X());
        double aLastY = aCenter.Y() + aR * (Math.Cos(aU2) * aXDir.Y() + Math.Sin(aU2) * aYDir.Y());
        Assert.Equal(aLastX, aPoles[aConv.PoleCount].X(), 1.0e-10);
        Assert.Equal(aLastY, aPoles[aConv.PoleCount].Y(), 1.0e-10);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.PoleCount; ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.KnotCount; ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void Arc_QuasiAngular()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 2.0);
        double aU1 = 0.0;
        double aU2 = M_PI;
        CircleToBSplineCurve aConv = new(aCirc, aU1, aU2, QuasiAngular);

        Assert.False(aConv.IsPeriodic);
        Assert.Equal(6, aConv.Degree());
        Assert.True(aConv.PoleCount > 0);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.PoleCount; ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.KnotCount; ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void Arc_Polynomial()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 1.0);
        double aU1 = 0.0;
        double aU2 = M_PI * 0.5;
        CircleToBSplineCurve aConv = new(aCirc, aU1, aU2, Polynomial);

        Assert.False(aConv.IsPeriodic);
        Assert.Equal(7, aConv.Degree());
        Assert.Equal(8, aConv.PoleCount);
    }

    [Fact]
    public void WeightsArePositive()
    {
        Circle2D aCirc = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 1.0);
        CircleToBSplineCurve aConv = new(aCirc, TgtThetaOver2);

        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.PoleCount; ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }
    }
}
