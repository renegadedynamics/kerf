// Port of: src/FoundationClasses/TKMath/GTests/Convert_ParabolaToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Convert_ParabolaToBSplineCurveTest
{
    private static void checkParabolaPoint(Convert_ParabolaToBSplineCurve theConv,
                                           in gp_Parab2d theParab,
                                           double theParam)
    {
        NCollection_Array1<gp_Pnt2d> aPoles = theConv.Poles();
        NCollection_Array1<double> aWeights = theConv.Weights();
        NCollection_Array1<double> aKnots = theConv.Knots();
        NCollection_Array1<int> aMults = theConv.Multiplicities();
        BSplCLib.D0(theParam,
                    0,
                    theConv.Degree(),
                    theConv.IsPeriodic(),
                    aPoles,
                    aWeights,
                    aKnots,
                    aMults,
                    out gp_Pnt2d aBSPnt);

        // Use ElCLib to evaluate the reference point on the parabola
        gp_Pnt2d aExpPnt = ElCLib.Value(theParam, theParab);
        double aExpX = aExpPnt.X();
        double aExpY = aExpPnt.Y();

        Assert.Equal(aExpX, aBSPnt.X(), 1.0e-10);
        Assert.Equal(aExpY, aBSPnt.Y(), 1.0e-10);
    }
    [Fact]
    public void BasicConversion()
    {
        gp_Parab2d aParab = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 1.0);
        double aU1 = -2.0;
        double aU2 = 2.0;
        Convert_ParabolaToBSplineCurve aConv = new(aParab, aU1, aU2);

        Assert.False(aConv.IsPeriodic());
        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.NbPoles() > 0);
        Assert.True(aConv.NbKnots() > 0);

        for (int i = 0; i <= 4; ++i)
        {
            double aParam = aU1 + i * (aU2 - aU1) / 4.0;
            checkParabolaPoint(aConv, aParab, aParam);
        }
    }

    [Fact]
    public void SmallRange()
    {
        gp_Parab2d aParab = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 0.5);
        double aU1 = -0.5;
        double aU2 = 0.5;
        Convert_ParabolaToBSplineCurve aConv = new(aParab, aU1, aU2);

        for (int i = 0; i <= 4; ++i)
        {
            double aParam = aU1 + i * (aU2 - aU1) / 4.0;
            checkParabolaPoint(aConv, aParab, aParam);
        }
    }

    [Fact]
    public void AllWeightsAreOne()
    {
        // Parabola conversion produces non-rational (polynomial) BSpline
        gp_Parab2d aParab = new(new gp_Ax2d(new gp_Pnt2d(0.0, 0.0), new gp_Dir2d(1.0, 0.0)), 1.0);
        Convert_ParabolaToBSplineCurve aConv = new(aParab, -1.0, 1.0);

        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = aWeights.Lower(); i <= aWeights.Upper(); ++i)
        {
            Assert.Equal(1.0, aWeights[i], 1.0e-15);
        }
    }
}
