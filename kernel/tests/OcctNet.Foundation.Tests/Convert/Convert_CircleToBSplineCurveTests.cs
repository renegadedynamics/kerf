// Port of: src/FoundationClasses/TKMath/GTests/Convert_CircleToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;

namespace OcctNet.Foundation.Tests;

public class Convert_CircleToBSplineCurveTest
{
    [Fact]
    public void FullCircle_TgtThetaOver2()
    {
        gp_Circ2d aCirc = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0);
        Convert_CircleToBSplineCurve aConv = new(aCirc, Convert_TgtThetaOver2);

        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.IsPeriodic());
        Assert.True(aConv.NbPoles() > 0);
        Assert.True(aConv.NbKnots() > 0);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbPoles(); ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.NbKnots(); ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void FullCircle_RationalC1()
    {
        gp_Circ2d aCirc = new(new gp_Ax2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        Convert_CircleToBSplineCurve aConv = new(aCirc, Convert_RationalC1);

        Assert.True(aConv.IsPeriodic());
        Assert.True(aConv.NbPoles() > 0);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbPoles(); ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.NbKnots(); ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void Arc_TgtThetaOver2()
    {
        gp_Circ2d aCirc = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 1.0);
        double aU1 = M_PI / 6.0;
        double aU2 = 5.0 * M_PI / 3.0;
        Convert_CircleToBSplineCurve aConv = new(aCirc, aU1, aU2, Convert_TgtThetaOver2);

        Assert.False(aConv.IsPeriodic());
        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.NbPoles() > 0);

        // Verify first pole matches conic point at U1
        NCollection_Array1<gp_Pnt2d> aPoles = aConv.Poles();
        double aR = aCirc.Radius();
        gp_Pnt2d aCenter = aCirc.Location();
        gp_Dir2d aXDir = aCirc.XAxis().Direction();
        gp_Dir2d aYDir = aCirc.YAxis().Direction();

        double aFirstX = aCenter.X() + aR * (Math.Cos(aU1) * aXDir.X() + Math.Sin(aU1) * aYDir.X());
        double aFirstY = aCenter.Y() + aR * (Math.Cos(aU1) * aXDir.Y() + Math.Sin(aU1) * aYDir.Y());
        Assert.Equal(aFirstX, aPoles[1].X(), 1.0e-10);
        Assert.Equal(aFirstY, aPoles[1].Y(), 1.0e-10);

        // Verify last pole matches conic point at U2
        double aLastX = aCenter.X() + aR * (Math.Cos(aU2) * aXDir.X() + Math.Sin(aU2) * aYDir.X());
        double aLastY = aCenter.Y() + aR * (Math.Cos(aU2) * aXDir.Y() + Math.Sin(aU2) * aYDir.Y());
        Assert.Equal(aLastX, aPoles[aConv.NbPoles()].X(), 1.0e-10);
        Assert.Equal(aLastY, aPoles[aConv.NbPoles()].Y(), 1.0e-10);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbPoles(); ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.NbKnots(); ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void Arc_QuasiAngular()
    {
        gp_Circ2d aCirc = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 2.0);
        double aU1 = 0.0;
        double aU2 = M_PI;
        Convert_CircleToBSplineCurve aConv = new(aCirc, aU1, aU2, Convert_QuasiAngular);

        Assert.False(aConv.IsPeriodic());
        Assert.Equal(6, aConv.Degree());
        Assert.True(aConv.NbPoles() > 0);

        // Verify all weights are positive
        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbPoles(); ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }

        // Verify knots are monotonically increasing
        NCollection_Array1<double> aKnots = aConv.Knots();
        for (int i = 2; i <= aConv.NbKnots(); ++i)
        {
            Assert.True(aKnots[i] > aKnots[i - 1]);
        }
    }

    [Fact]
    public void Arc_Polynomial()
    {
        gp_Circ2d aCirc = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 1.0);
        double aU1 = 0.0;
        double aU2 = M_PI * 0.5;
        Convert_CircleToBSplineCurve aConv = new(aCirc, aU1, aU2, Convert_Polynomial);

        Assert.False(aConv.IsPeriodic());
        Assert.Equal(7, aConv.Degree());
        Assert.Equal(8, aConv.NbPoles());
    }

    [Fact]
    public void WeightsArePositive()
    {
        gp_Circ2d aCirc = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 1.0);
        Convert_CircleToBSplineCurve aConv = new(aCirc, Convert_TgtThetaOver2);

        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbPoles(); ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }
    }
}
