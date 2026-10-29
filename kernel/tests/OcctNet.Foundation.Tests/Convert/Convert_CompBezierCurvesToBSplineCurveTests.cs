// Port of: src/FoundationClasses/TKMath/GTests/Convert_CompBezierCurvesToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Convert_CompBezierCurvesToBSplineCurveTest
{
    [Fact]
    public void SingleLinearBezier()
    {
        Convert_CompBezierCurvesToBSplineCurve aConv = new();

        NCollection_Array1<gp_Pnt> aPoles = new(1, 2);
        aPoles[1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[2] = new gp_Pnt(1.0, 1.0, 0.0);

        aConv.AddCurve(aPoles);
        aConv.Perform();

        Assert.True(aConv.Degree() >= 1);
        Assert.Equal(2, aConv.NbPoles());
        Assert.Equal(2, aConv.NbKnots());

        NCollection_Array1<gp_Pnt> aResPoles = new(1, aConv.NbPoles());
        aConv.Poles(aResPoles);
        Assert.Equal(0.0, aResPoles[1].X(), 1.0e-15);
        Assert.Equal(1.0, aResPoles[2].X(), 1.0e-15);
    }

    [Fact]
    public void SingleCubicBezier()
    {
        Convert_CompBezierCurvesToBSplineCurve aConv = new();

        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[2] = new gp_Pnt(1.0, 2.0, 0.0);
        aPoles[3] = new gp_Pnt(3.0, 2.0, 0.0);
        aPoles[4] = new gp_Pnt(4.0, 0.0, 0.0);

        aConv.AddCurve(aPoles);
        aConv.Perform();

        Assert.Equal(3, aConv.Degree());
        Assert.Equal(4, aConv.NbPoles());
        Assert.Equal(2, aConv.NbKnots());
    }

    [Fact]
    public void TwoAdjacentBeziers_C0()
    {
        Convert_CompBezierCurvesToBSplineCurve aConv = new();

        // First linear segment
        NCollection_Array1<gp_Pnt> aPoles1 = new(1, 2);
        aPoles1[1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles1[2] = new gp_Pnt(1.0, 1.0, 0.0);

        // Second linear segment, adjacent but not tangent
        NCollection_Array1<gp_Pnt> aPoles2 = new(1, 2);
        aPoles2[1] = new gp_Pnt(1.0, 1.0, 0.0);
        aPoles2[2] = new gp_Pnt(2.0, 0.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        Assert.True(aConv.Degree() >= 1);
        Assert.Equal(3, aConv.NbKnots());

        NCollection_Array1<gp_Pnt> aResPoles = new(1, aConv.NbPoles());
        NCollection_Array1<double> aKnots = new(1, aConv.NbKnots());
        NCollection_Array1<int> aMults = new(1, aConv.NbKnots());
        aConv.Poles(aResPoles);
        aConv.KnotsAndMults(aKnots, aMults);

        // First and last knot multiplicities should be degree+1
        Assert.Equal(aConv.Degree() + 1, aMults[1]);
        Assert.Equal(aConv.Degree() + 1, aMults[aConv.NbKnots()]);
    }

    [Fact]
    public void TwoAdjacentBeziers_C1()
    {
        Convert_CompBezierCurvesToBSplineCurve aConv = new();

        // Two cubic Beziers with parallel tangent at junction
        NCollection_Array1<gp_Pnt> aPoles1 = new(1, 4);
        aPoles1[1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles1[2] = new gp_Pnt(1.0, 1.0, 0.0);
        aPoles1[3] = new gp_Pnt(2.0, 1.0, 0.0);
        aPoles1[4] = new gp_Pnt(3.0, 0.0, 0.0);

        NCollection_Array1<gp_Pnt> aPoles2 = new(1, 4);
        aPoles2[1] = new gp_Pnt(3.0, 0.0, 0.0);
        aPoles2[2] = new gp_Pnt(4.0, -1.0, 0.0);
        aPoles2[3] = new gp_Pnt(5.0, -1.0, 0.0);
        aPoles2[4] = new gp_Pnt(6.0, 0.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        Assert.Equal(3, aConv.Degree());
        Assert.Equal(3, aConv.NbKnots());

        // Check that junction has multiplicity Degree-1 (C1)
        NCollection_Array1<int> aMults = new(1, aConv.NbKnots());
        NCollection_Array1<double> aKnots = new(1, aConv.NbKnots());
        aConv.KnotsAndMults(aKnots, aMults);
        Assert.Equal(aConv.Degree() - 1, aMults[2]);
    }

    [Fact]
    public void MixedDegreeBeziers()
    {
        Convert_CompBezierCurvesToBSplineCurve aConv = new();

        // Linear segment
        NCollection_Array1<gp_Pnt> aPoles1 = new(1, 2);
        aPoles1[1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles1[2] = new gp_Pnt(1.0, 0.0, 0.0);

        // Cubic segment
        NCollection_Array1<gp_Pnt> aPoles2 = new(1, 4);
        aPoles2[1] = new gp_Pnt(1.0, 0.0, 0.0);
        aPoles2[2] = new gp_Pnt(2.0, 1.0, 0.0);
        aPoles2[3] = new gp_Pnt(3.0, 1.0, 0.0);
        aPoles2[4] = new gp_Pnt(4.0, 0.0, 0.0);

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
        Convert_CompBezierCurves2dToBSplineCurve2d aConv = new();

        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 2);
        aPoles[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles[2] = new gp_Pnt2d(1.0, 1.0);

        aConv.AddCurve(aPoles);
        aConv.Perform();

        Assert.True(aConv.Degree() >= 1);
        Assert.Equal(2, aConv.NbPoles());
        Assert.Equal(2, aConv.NbKnots());

        NCollection_Array1<gp_Pnt2d> aResPoles = new(1, aConv.NbPoles());
        aConv.Poles(aResPoles);
        Assert.Equal(0.0, aResPoles[1].X(), 1.0e-15);
        Assert.Equal(1.0, aResPoles[2].X(), 1.0e-15);
    }

    [Fact]
    public void TwoAdjacent2d_C1()
    {
        Convert_CompBezierCurves2dToBSplineCurve2d aConv = new();

        NCollection_Array1<gp_Pnt2d> aPoles1 = new(1, 4);
        aPoles1[1] = new gp_Pnt2d(0.0, 0.0);
        aPoles1[2] = new gp_Pnt2d(1.0, 1.0);
        aPoles1[3] = new gp_Pnt2d(2.0, 1.0);
        aPoles1[4] = new gp_Pnt2d(3.0, 0.0);

        NCollection_Array1<gp_Pnt2d> aPoles2 = new(1, 4);
        aPoles2[1] = new gp_Pnt2d(3.0, 0.0);
        aPoles2[2] = new gp_Pnt2d(4.0, -1.0);
        aPoles2[3] = new gp_Pnt2d(5.0, -1.0);
        aPoles2[4] = new gp_Pnt2d(6.0, 0.0);

        aConv.AddCurve(aPoles1);
        aConv.AddCurve(aPoles2);
        aConv.Perform();

        Assert.Equal(3, aConv.Degree());
        Assert.Equal(3, aConv.NbKnots());

        NCollection_Array1<int> aMults = new(1, aConv.NbKnots());
        NCollection_Array1<double> aKnots = new(1, aConv.NbKnots());
        aConv.KnotsAndMults(aKnots, aMults);
        Assert.Equal(aConv.Degree() - 1, aMults[2]);
    }
}
