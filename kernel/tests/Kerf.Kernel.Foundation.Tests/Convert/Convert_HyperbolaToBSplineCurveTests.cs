// Port of: src/FoundationClasses/TKMath/GTests/Convert_HyperbolaToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Convert_HyperbolaToBSplineCurveTest
{
    [Fact]
    public void BasicConversion()
    {
        Hyperbola2D aHypr = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 3.0, 2.0);
        double aU1 = -1.0;
        double aU2 = 1.0;
        HyperbolaToBSplineCurve aConv = new(aHypr, aU1, aU2);

        Assert.False(aConv.IsPeriodic);
        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.PoleCount > 0);
        Assert.True(aConv.KnotCount > 0);

        // Verify first pole matches hyperbola point at U1
        NCollection_Array1<Point2D> aPoles = aConv.Poles();
        double aMajR = aHypr.MajorRadius;
        double aMinR = aHypr.MinorRadius;
        Point2D aCenter = aHypr.Location;
        Direction2D aXDir = aHypr.XAxis.Direction;
        Direction2D aYDir = aHypr.YAxis.Direction;

        double aFirstX =
            aCenter.X + aMajR * Math.Cosh(aU1) * aXDir.X + aMinR * Math.Sinh(aU1) * aYDir.X;
        double aFirstY =
            aCenter.Y + aMajR * Math.Cosh(aU1) * aXDir.Y + aMinR * Math.Sinh(aU1) * aYDir.Y;
        Assert.Equal(aFirstX, aPoles[1].X, 1.0e-10);
        Assert.Equal(aFirstY, aPoles[1].Y, 1.0e-10);

        // Verify last pole matches hyperbola point at U2
        double aLastX =
            aCenter.X + aMajR * Math.Cosh(aU2) * aXDir.X + aMinR * Math.Sinh(aU2) * aYDir.X;
        double aLastY =
            aCenter.Y + aMajR * Math.Cosh(aU2) * aXDir.Y + aMinR * Math.Sinh(aU2) * aYDir.Y;
        Assert.Equal(aLastX, aPoles[aConv.PoleCount].X, 1.0e-10);
        Assert.Equal(aLastY, aPoles[aConv.PoleCount].Y, 1.0e-10);

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

        // Verify multiplicities are within valid range
        NCollection_Array1<int> aMults = aConv.Multiplicities();
        for (int i = 1; i <= aConv.KnotCount; ++i)
        {
            Assert.True(aMults[i] > 0);
            Assert.True(aMults[i] <= aConv.Degree() + 1);
        }
    }

    [Fact]
    public void LargeRange()
    {
        Hyperbola2D aHypr = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 1.0, 1.0);
        double aU1 = -2.0;
        double aU2 = 2.0;
        HyperbolaToBSplineCurve aConv = new(aHypr, aU1, aU2);

        Assert.False(aConv.IsPeriodic);
        Assert.True(aConv.PoleCount > 0);
        Assert.True(aConv.KnotCount > 0);

        // Verify first pole matches hyperbola point at U1
        NCollection_Array1<Point2D> aPoles = aConv.Poles();
        double aMajR = aHypr.MajorRadius;
        double aMinR = aHypr.MinorRadius;
        Point2D aCenter = aHypr.Location;
        Direction2D aXDir = aHypr.XAxis.Direction;
        Direction2D aYDir = aHypr.YAxis.Direction;

        double aFirstX =
            aCenter.X + aMajR * Math.Cosh(aU1) * aXDir.X + aMinR * Math.Sinh(aU1) * aYDir.X;
        double aFirstY =
            aCenter.Y + aMajR * Math.Cosh(aU1) * aXDir.Y + aMinR * Math.Sinh(aU1) * aYDir.Y;
        Assert.Equal(aFirstX, aPoles[1].X, 1.0e-10);
        Assert.Equal(aFirstY, aPoles[1].Y, 1.0e-10);

        // Verify last pole matches hyperbola point at U2
        double aLastX =
            aCenter.X + aMajR * Math.Cosh(aU2) * aXDir.X + aMinR * Math.Sinh(aU2) * aYDir.X;
        double aLastY =
            aCenter.Y + aMajR * Math.Cosh(aU2) * aXDir.Y + aMinR * Math.Sinh(aU2) * aYDir.Y;
        Assert.Equal(aLastX, aPoles[aConv.PoleCount].X, 1.0e-10);
        Assert.Equal(aLastY, aPoles[aConv.PoleCount].Y, 1.0e-10);

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
    public void WeightsArePositive()
    {
        Hyperbola2D aHypr = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 3.0, 2.0);
        HyperbolaToBSplineCurve aConv = new(aHypr, -1.0, 1.0);

        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.PoleCount; ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }
    }
}
