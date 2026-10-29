// Port of: src/FoundationClasses/TKMath/GTests/Convert_EllipseToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;

namespace OcctNet.Foundation.Tests;

public class Convert_EllipseToBSplineCurveTest
{
    [Fact]
    public void FullEllipse_TgtThetaOver2()
    {
        gp_Elips2d anElips = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0, 3.0);
        Convert_EllipseToBSplineCurve aConv = new(anElips, Convert_TgtThetaOver2);

        Assert.True(aConv.IsPeriodic());
        Assert.Equal(2, aConv.Degree());
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

        // Verify multiplicities are within valid range
        NCollection_Array1<int> aMults = aConv.Multiplicities();
        for (int i = 1; i <= aConv.NbKnots(); ++i)
        {
            Assert.True(aMults[i] > 0);
            Assert.True(aMults[i] <= aConv.Degree() + 1);
        }
    }

    [Fact]
    public void Arc_TgtThetaOver2()
    {
        gp_Elips2d anElips = new(new gp_Ax2d(new gp_Pnt2d(1.0, 1.0), new gp_Dir2d(1.0, 0.0)), 4.0, 2.0);
        double aU1 = M_PI / 4.0;
        double aU2 = 3.0 * M_PI / 2.0;
        Convert_EllipseToBSplineCurve aConv = new(anElips, aU1, aU2, Convert_TgtThetaOver2);

        Assert.False(aConv.IsPeriodic());
        Assert.True(aConv.NbPoles() > 0);
        Assert.True(aConv.NbKnots() > 0);

        // Verify first pole matches ellipse point at U1
        NCollection_Array1<gp_Pnt2d> aPoles = aConv.Poles();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();
        gp_Pnt2d aCenter = anElips.Location();
        gp_Dir2d aXDir = anElips.XAxis().Direction();
        gp_Dir2d aYDir = anElips.YAxis().Direction();

        double aFirstX =
            aCenter.X() + aMajR * Math.Cos(aU1) * aXDir.X() + aMinR * Math.Sin(aU1) * aYDir.X();
        double aFirstY =
            aCenter.Y() + aMajR * Math.Cos(aU1) * aXDir.Y() + aMinR * Math.Sin(aU1) * aYDir.Y();
        Assert.Equal(aFirstX, aPoles[1].X(), 1.0e-10);
        Assert.Equal(aFirstY, aPoles[1].Y(), 1.0e-10);

        // Verify last pole matches ellipse point at U2
        double aLastX =
            aCenter.X() + aMajR * Math.Cos(aU2) * aXDir.X() + aMinR * Math.Sin(aU2) * aYDir.X();
        double aLastY =
            aCenter.Y() + aMajR * Math.Cos(aU2) * aXDir.Y() + aMinR * Math.Sin(aU2) * aYDir.Y();
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
    public void FullEllipse_RationalC1()
    {
        gp_Elips2d anElips = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 3.0, 1.0);
        Convert_EllipseToBSplineCurve aConv = new(anElips, Convert_RationalC1);

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
    public void WeightsArePositive()
    {
        gp_Elips2d anElips = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 5.0, 3.0);
        Convert_EllipseToBSplineCurve aConv = new(anElips, Convert_TgtThetaOver2);

        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = 1; i <= aConv.NbPoles(); ++i)
        {
            Assert.True(aWeights[i] > 0.0);
        }
    }
}
