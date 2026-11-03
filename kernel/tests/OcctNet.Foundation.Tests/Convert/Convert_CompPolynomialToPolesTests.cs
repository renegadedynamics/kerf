// Port of: src/FoundationClasses/TKMath/GTests/Convert_CompPolynomialToPoles_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;

namespace OcctNet.Foundation.Tests;

public class Convert_CompPolynomialToPolesTest
{
    [Fact]
    public void SingleLinearPolynomial()
    {
        // Convert f(x) = 2*x + 1 on [-1,1] to BSpline
        // Polynomial domain [-1,1], True interval [-1,1]
        int aDim = 1;
        int aMaxDeg = 1;
        int aDeg = 1;

        NCollection_Array1<double> aCoeffs = new(1, 2);
        aCoeffs[1] = 1.0; // constant term
        aCoeffs[2] = 2.0; // linear term

        NCollection_Array1<double> aPolyIntervals = new(1, 2);
        aPolyIntervals[1] = -1.0;
        aPolyIntervals[2] = 1.0;

        NCollection_Array1<double> aTrueIntervals = new(1, 2);
        aTrueIntervals[1] = -1.0;
        aTrueIntervals[2] = 1.0;

        CompPolynomialToPoles aConv = new(aDim, aMaxDeg, aDeg, aCoeffs, aPolyIntervals, aTrueIntervals);

        Assert.True(aConv.IsDone);
        Assert.Equal(1, aConv.Degree());
        Assert.Equal(2, aConv.NbKnots());
        Assert.True(aConv.NbPoles() > 0);
    }

    [Fact]
    public void SingleQuadraticPolynomial()
    {
        // Convert f(x) = x^2 on [0,1] to BSpline
        int aDim = 1;
        int aMaxDeg = 2;
        int aDeg = 2;

        NCollection_Array1<double> aCoeffs = new(1, 3);
        aCoeffs[1] = 0.0; // x^0
        aCoeffs[2] = 0.0; // x^1
        aCoeffs[3] = 1.0; // x^2

        NCollection_Array1<double> aPolyIntervals = new(1, 2);
        aPolyIntervals[1] = 0.0;
        aPolyIntervals[2] = 1.0;

        NCollection_Array1<double> aTrueIntervals = new(1, 2);
        aTrueIntervals[1] = 0.0;
        aTrueIntervals[2] = 1.0;

        CompPolynomialToPoles aConv = new(aDim, aMaxDeg, aDeg, aCoeffs, aPolyIntervals, aTrueIntervals);

        Assert.True(aConv.IsDone);
        Assert.Equal(2, aConv.Degree());
    }

    [Fact]
    public void TwoSpansUniformContinuity()
    {
        // Two linear polynomials with C0 continuity
        int aNumCurves = 2;
        int aContinuity = 0;
        int aDim = 1;
        int aMaxDeg = 1;

        NCollection_HArray1<int> aNumCoeff = new NCollection_HArray1<int>(1, 2);
        aNumCoeff.SetValue(1, 2); // linear = 2 coefficients
        aNumCoeff.SetValue(2, 2);

        // Coefficients: [curve1: c0, c1] [curve2: c0, c1]
        NCollection_HArray1<double> aCoeffs = new NCollection_HArray1<double>(1, 4);
        aCoeffs.SetValue(1, 0.0);
        aCoeffs.SetValue(2, 1.0);
        aCoeffs.SetValue(3, 1.0);
        aCoeffs.SetValue(4, -1.0);

        NCollection_HArray2<double> aPolyIntervals =
            new NCollection_HArray2<double>(1, 2, 1, 2);
        aPolyIntervals.SetValue(1, 1, 0.0);
        aPolyIntervals.SetValue(1, 2, 1.0);
        aPolyIntervals.SetValue(2, 1, 0.0);
        aPolyIntervals.SetValue(2, 2, 1.0);

        NCollection_HArray1<double> aTrueIntervals = new NCollection_HArray1<double>(1, 3);
        aTrueIntervals.SetValue(1, 0.0);
        aTrueIntervals.SetValue(2, 0.5);
        aTrueIntervals.SetValue(3, 1.0);

        CompPolynomialToPoles aConv = new(aNumCurves,
                                                  aContinuity,
                                                  aDim,
                                                  aMaxDeg,
                                                  aNumCoeff,
                                                  aCoeffs,
                                                  aPolyIntervals,
                                                  aTrueIntervals);

        Assert.True(aConv.IsDone);
        Assert.Equal(1, aConv.Degree());
        Assert.Equal(3, aConv.NbKnots());
    }

    [Fact]
    public void ThreeDimensional()
    {
        // 3D linear curve
        int aDim = 3;
        int aMaxDeg = 1;
        int aDeg = 1;

        // Coefficients for 3D: [x0, y0, z0, x1, y1, z1]
        NCollection_Array1<double> aCoeffs = new(1, 6);
        aCoeffs[1] = 0.0;
        aCoeffs[2] = 0.0;
        aCoeffs[3] = 0.0; // constant
        aCoeffs[4] = 1.0;
        aCoeffs[5] = 2.0;
        aCoeffs[6] = 3.0; // linear

        NCollection_Array1<double> aPolyIntervals = new(1, 2);
        aPolyIntervals[1] = 0.0;
        aPolyIntervals[2] = 1.0;

        NCollection_Array1<double> aTrueIntervals = new(1, 2);
        aTrueIntervals[1] = 0.0;
        aTrueIntervals[2] = 1.0;

        CompPolynomialToPoles aConv = new(aDim, aMaxDeg, aDeg, aCoeffs, aPolyIntervals, aTrueIntervals);

        Assert.True(aConv.IsDone);
        Assert.Equal(1, aConv.Degree());

        NCollection_Array2<double> aPoles = aConv.Poles();
        // Poles should be [1..NbPoles][1..3]
        Assert.Equal(3, aPoles.RowLength());
    }
}
