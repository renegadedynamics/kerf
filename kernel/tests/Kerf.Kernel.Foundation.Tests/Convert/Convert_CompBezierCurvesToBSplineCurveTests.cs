// Port of: src/FoundationClasses/TKMath/GTests/Convert_CompBezierCurvesToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Convert_CompBezierCurvesToBSplineCurveTest
{
    [Fact]
    public void SingleLinearBezier()
    {
        CompBezierCurvesToBSplineCurve aConv = new();

        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 1.0, 0.0);

        aConv.AddCurve(aPoles);
        aConv.Perform();

        Assert.True(aConv.Degree() >= 1);
        Assert.Equal(2, aConv.PoleCount);
        Assert.Equal(2, aConv.KnotCount);

        NCollection_Array1<Point3D> aResPoles = new(1, aConv.PoleCount);
        aConv.Poles(aResPoles);
        Assert.Equal(0.0, aResPoles[1].X, 1.0e-15);
        Assert.Equal(1.0, aResPoles[2].X, 1.0e-15);
    }

    [Fact]
    public void SingleCubicBezier()
    {
        CompBezierCurvesToBSplineCurve aConv = new();

        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2] = new Point3D(1.0, 2.0, 0.0);
        aPoles[3] = new Point3D(3.0, 2.0, 0.0);
        aPoles[4] = new Point3D(4.0, 0.0, 0.0);

        aConv.AddCurve(aPoles);
        aConv.Perform();

        Assert.Equal(3, aConv.Degree());
        Assert.Equal(4, aConv.PoleCount);
        Assert.Equal(2, aConv.KnotCount);
    }

    [Fact]
    public void TwoAdjacentBeziers_C0()
    {
        CompBezierCurvesToBSplineCurve aConv = new();

        // First linear segment
        NCollection_Array1<Point3D> aPoles1 = new(1, 2);
        aPoles1[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles1[2] = new Point3D(1.0, 1.0, 0.0);

        // Second linear segment, adjacent but not tangent
        NCollection_Array1<Point3D> aPoles2 = new(1, 2);
        aPoles2[1] = new Point3D(1.0, 1.0, 0.0);
        aPoles2[2] = new Point3D(2.0, 0.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        Assert.True(aConv.Degree() >= 1);
        Assert.Equal(3, aConv.KnotCount);

        NCollection_Array1<Point3D> aResPoles = new(1, aConv.PoleCount);
        NCollection_Array1<double> aKnots = new(1, aConv.KnotCount);
        NCollection_Array1<int> aMults = new(1, aConv.KnotCount);
        aConv.Poles(aResPoles);
        aConv.KnotsAndMults(aKnots, aMults);

        // First and last knot multiplicities should be degree+1
        Assert.Equal(aConv.Degree() + 1, aMults[1]);
        Assert.Equal(aConv.Degree() + 1, aMults[aConv.KnotCount]);
    }

    [Fact]
    public void TwoAdjacentBeziers_C1()
    {
        CompBezierCurvesToBSplineCurve aConv = new();

        // Two cubic Beziers with parallel tangent at junction
        NCollection_Array1<Point3D> aPoles1 = new(1, 4);
        aPoles1[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles1[2] = new Point3D(1.0, 1.0, 0.0);
        aPoles1[3] = new Point3D(2.0, 1.0, 0.0);
        aPoles1[4] = new Point3D(3.0, 0.0, 0.0);

        NCollection_Array1<Point3D> aPoles2 = new(1, 4);
        aPoles2[1] = new Point3D(3.0, 0.0, 0.0);
        aPoles2[2] = new Point3D(4.0, -1.0, 0.0);
        aPoles2[3] = new Point3D(5.0, -1.0, 0.0);
        aPoles2[4] = new Point3D(6.0, 0.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        Assert.Equal(3, aConv.Degree());
        Assert.Equal(3, aConv.KnotCount);

        // Check that junction has multiplicity Degree-1 (C1)
        NCollection_Array1<int> aMults = new(1, aConv.KnotCount);
        NCollection_Array1<double> aKnots = new(1, aConv.KnotCount);
        aConv.KnotsAndMults(aKnots, aMults);
        Assert.Equal(aConv.Degree() - 1, aMults[2]);
    }

    [Fact]
    public void MixedDegreeBeziers()
    {
        CompBezierCurvesToBSplineCurve aConv = new();

        // Linear segment
        NCollection_Array1<Point3D> aPoles1 = new(1, 2);
        aPoles1[1] = new Point3D(0.0, 0.0, 0.0);
        aPoles1[2] = new Point3D(1.0, 0.0, 0.0);

        // Cubic segment
        NCollection_Array1<Point3D> aPoles2 = new(1, 4);
        aPoles2[1] = new Point3D(1.0, 0.0, 0.0);
        aPoles2[2] = new Point3D(2.0, 1.0, 0.0);
        aPoles2[3] = new Point3D(3.0, 1.0, 0.0);
        aPoles2[4] = new Point3D(4.0, 0.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        // Degree should be raised to the maximum
        Assert.Equal(3, aConv.Degree());
    }
}

public class Convert_CompBezierCurves2dToBSplineCurve2dTest
{
    [Fact]
    public void SingleLinear2d()
    {
        CompBezierCurves2dToBSplineCurve2d aConv = new();

        NCollection_Array1<Point2D> aPoles = new(1, 2);
        aPoles[1] = new Point2D(0.0, 0.0);
        aPoles[2] = new Point2D(1.0, 1.0);

        aConv.AddCurve(aPoles);
        aConv.Perform();

        Assert.True(aConv.Degree() >= 1);
        Assert.Equal(2, aConv.PoleCount);
        Assert.Equal(2, aConv.KnotCount);

        NCollection_Array1<Point2D> aResPoles = new(1, aConv.PoleCount);
        aConv.Poles(aResPoles);
        Assert.Equal(0.0, aResPoles[1].X, 1.0e-15);
        Assert.Equal(1.0, aResPoles[2].X, 1.0e-15);
    }

    [Fact]
    public void TwoAdjacent2d_C1()
    {
        CompBezierCurves2dToBSplineCurve2d aConv = new();

        NCollection_Array1<Point2D> aPoles1 = new(1, 4);
        aPoles1[1] = new Point2D(0.0, 0.0);
        aPoles1[2] = new Point2D(1.0, 1.0);
        aPoles1[3] = new Point2D(2.0, 1.0);
        aPoles1[4] = new Point2D(3.0, 0.0);

        NCollection_Array1<Point2D> aPoles2 = new(1, 4);
        aPoles2[1] = new Point2D(3.0, 0.0);
        aPoles2[2] = new Point2D(4.0, -1.0);
        aPoles2[3] = new Point2D(5.0, -1.0);
        aPoles2[4] = new Point2D(6.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        Assert.Equal(3, aConv.Degree());
        Assert.Equal(3, aConv.KnotCount);

        NCollection_Array1<int> aMults = new(1, aConv.KnotCount);
        NCollection_Array1<double> aKnots = new(1, aConv.KnotCount);
        aConv.KnotsAndMults(aKnots, aMults);
        Assert.Equal(aConv.Degree() - 1, aMults[2]);
    }
}
