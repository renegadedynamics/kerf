// Port of: src/FoundationClasses/TKMath/GTests/Convert_ParabolaToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Convert_ParabolaToBSplineCurveTest
{
    private static void checkParabolaPoint(ParabolaToBSplineCurve theConv,
                                           in Parabola2D theParab,
                                           double theParam)
    {
        NCollection_Array1<Point2D> aPoles = theConv.Poles();
        NCollection_Array1<double> aWeights = theConv.Weights();
        NCollection_Array1<double> aKnots = theConv.Knots();
        NCollection_Array1<int> aMults = theConv.Multiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(theParam,
                    0,
                    theConv.Degree(),
                    theConv.IsPeriodic,
                    aPoles,
                    aWeights,
                    aKnots,
                    aMults,
                    out Point2D aBSPnt);

        // Use ElCLib to evaluate the reference point on the parabola
        Point2D aExpPnt = global::Kerf.Engine.Geometry.ElCLib.Value(theParam, theParab);
        double aExpX = aExpPnt.X;
        double aExpY = aExpPnt.Y;

        Assert.Equal(aExpX, aBSPnt.X, 1.0e-10);
        Assert.Equal(aExpY, aBSPnt.Y, 1.0e-10);
    }
    [Fact]
    public void BasicConversion()
    {
        Parabola2D aParab = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 1.0);
        double aU1 = -2.0;
        double aU2 = 2.0;
        ParabolaToBSplineCurve aConv = new(aParab, aU1, aU2);

        Assert.False(aConv.IsPeriodic);
        Assert.Equal(2, aConv.Degree());
        Assert.True(aConv.PoleCount > 0);
        Assert.True(aConv.KnotCount > 0);

        for (int i = 0; i <= 4; ++i)
        {
            double aParam = aU1 + i * (aU2 - aU1) / 4.0;
            checkParabolaPoint(aConv, aParab, aParam);
        }
    }

    [Fact]
    public void SmallRange()
    {
        Parabola2D aParab = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 0.5);
        double aU1 = -0.5;
        double aU2 = 0.5;
        ParabolaToBSplineCurve aConv = new(aParab, aU1, aU2);

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
        Parabola2D aParab = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), 1.0);
        ParabolaToBSplineCurve aConv = new(aParab, -1.0, 1.0);

        NCollection_Array1<double> aWeights = aConv.Weights();
        for (int i = aWeights.Lower(); i <= aWeights.Upper(); ++i)
        {
            Assert.Equal(1.0, aWeights[i], 1.0e-15);
        }
    }
}
