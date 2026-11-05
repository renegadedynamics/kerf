// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_AHTBezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash, DumpJson_Rational_NoCrash (DumpJson is not ported).
public class Geom2dEval_AHTBezierCurveTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;
    private const double THE_FD_TOL_D3 = 1e-3;

    // Helper to create a curve with basis {1, t, sinh(alpha*t), cosh(alpha*t), sin(beta*t),
    // cos(beta*t)} algDegree=1, alpha=1.0, beta=1.0 => basisDim = 2 + 2 + 2 = 6 poles needed
    private static AhtBezierCurve2D createFullBasisCurve()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 6);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 0.0));
        aPoles.SetValue(3, new Point2D(1.0, 1.0));
        aPoles.SetValue(4, new Point2D(0.0, 1.0));
        aPoles.SetValue(5, new Point2D(0.0, 1.0));
        aPoles.SetValue(6, new Point2D(1.0, 0.0));
        return new AhtBezierCurve2D(aPoles, 1, 1.0, 1.0);
    }

    // Helper to create a pure polynomial curve: basis {1, t, t^2} => 3 poles
    private static AhtBezierCurve2D createPolynomialCurve()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        return new AhtBezierCurve2D(aPoles, 2, 0.0, 0.0);
    }

    // Helper to create a rational polynomial curve with weights
    private static AhtBezierCurve2D createRationalCurve()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 2.0);
        aWeights.SetValue(3, 1.0);
        return new AhtBezierCurve2D(aPoles, aWeights, 2, 0.0, 0.0);
    }

    [Fact]
    public void Construction_FullBasis()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        Assert.Equal(6, aCurve.PoleCount);
        Assert.Equal(1, aCurve.AlgDegree());
        Assert.Equal(1.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCurve.Beta(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Construction_Polynomial()
    {
        AhtBezierCurve2D aCurve = createPolynomialCurve();
        Assert.Equal(3, aCurve.PoleCount);
        Assert.Equal(2, aCurve.AlgDegree());
        Assert.Equal(0.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCurve.Beta(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Construction_HyperbolicOnly()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        AhtBezierCurve2D aCurve = new(aPoles, 0, 2.0, 0.0);
        Assert.Equal(3, aCurve.PoleCount);
        Assert.Equal(0, aCurve.AlgDegree());
        Assert.Equal(2.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCurve.Beta(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Construction_Rational()
    {
        AhtBezierCurve2D aCurve = createRationalCurve();
        Assert.True(aCurve.IsRational);
        Assert.Equal(3, aCurve.PoleCount);
        Assert.Equal(2, aCurve.AlgDegree());
    }

    [Fact]
    public void IsRational_NonRational()
    {
        AhtBezierCurve2D aCurve = createPolynomialCurve();
        Assert.False(aCurve.IsRational);
    }

    [Fact]
    public void Bounds()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        Assert.Equal(0.0, aCurve.FirstParameter, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCurve.LastParameter, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Endpoints()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();

        Point2D aStart = aCurve.EvalD0(aCurve.FirstParameter);
        Point2D aEnd = aCurve.EvalD0(aCurve.LastParameter);

        Assert.Equal(0.0, aStart.DistanceTo(aCurve.StartPoint), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aEnd.DistanceTo(aCurve.EndPoint), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_KnownPoint()
    {
        AhtBezierCurve2D aCurve = createPolynomialCurve();
        Point2D aPnt = aCurve.EvalD0(0.5);
        Assert.Equal(1.0, aPnt.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.5, aPnt.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        double aU = 0.4;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point2D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL_D1);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL_D1);
    }

    [Fact]
    public void EvalD1_Polynomial_ConsistentWithD0()
    {
        AhtBezierCurve2D aCurve = createPolynomialCurve();
        double aU = 0.3;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point2D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL_D1);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL_D1);
    }

    [Fact]
    public void EvalD1_Rational_ConsistentWithD0()
    {
        AhtBezierCurve2D aCurve = createRationalCurve();
        double aU = 0.5;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Point2D aP1 = aCurve.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL_D1);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL_D1);
    }

    [Fact]
    public void Accessors()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 5);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 0.0));
        aPoles.SetValue(3, new Point2D(2.0, 1.0));
        aPoles.SetValue(4, new Point2D(3.0, 1.0));
        aPoles.SetValue(5, new Point2D(4.0, 0.0));
        // algDegree=2 => 3 polynomial basis + 2 trig basis = 5 => beta>0
        AhtBezierCurve2D aCurve = new(aPoles, 2, 0.0, 3.5);
        Assert.Equal(5, aCurve.PoleCount);
        Assert.Equal(2, aCurve.AlgDegree());
        Assert.Equal(0.0, aCurve.Alpha(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.5, aCurve.Beta(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Periodicity()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        Assert.False(aCurve.IsPeriodic);
    }

    [Fact]
    public void Continuity()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        Assert.Equal(global::Kerf.Geometry.Continuity.CN, aCurve.Continuity);
        Assert.True(aCurve.IsCN(0));
        Assert.True(aCurve.IsCN(5));
        Assert.True(aCurve.IsCN(100));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(1.0, 2.0));

        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        Geometry2D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        AhtBezierCurve2D? aCopyCurve = aCopy as AhtBezierCurve2D;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(6, aCopyCurve.PoleCount);
        Assert.Equal(1, aCopyCurve.AlgDegree());

        // Verify the copy evaluates identically at several parameters
        for (double aU = 0.0; aU <= 1.0; aU += 0.25)
        {
            Point2D aOrigPnt = aCurve.EvalD0(aU);
            Point2D aCopyPnt = aCopyCurve.EvalD0(aU);
            Assert.Equal(0.0, aOrigPnt.DistanceTo(aCopyPnt), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        // Verify behavior: transformation is not supported on the copy either.
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(10.0, 10.0));
        AhtBezierCurve2D aCopyHandle = (AhtBezierCurve2D)aCopy;
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCopyHandle.Transform(aTrsf));

        Point2D aOrigMid = aCurve.EvalD0(0.5);
        Point2D aCopyMid = aCopyHandle.EvalD0(0.5);
        Assert.Equal(0.0, aOrigMid.DistanceTo(aCopyMid), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Copy_Rational()
    {
        AhtBezierCurve2D aCurve = createRationalCurve();
        Geometry2D aCopy = aCurve.Copy();
        AhtBezierCurve2D? aCopyCurve = aCopy as AhtBezierCurve2D;
        Assert.NotNull(aCopyCurve);
        Assert.True(aCopyCurve.IsRational);
    }

    [Fact]
    public void Construction_PoleCountMismatch_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 0.0));
        aPoles.SetValue(3, new Point2D(1.0, 1.0));
        aPoles.SetValue(4, new Point2D(0.0, 1.0));
        // algDegree=1, alpha=1.0, beta=1.0 => basisDim = 2+2+2 = 6, but only 4 poles
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new AhtBezierCurve2D(aPoles, 1, 1.0, 1.0));
    }

    [Fact]
    public void Construction_NegativeAlpha_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 0.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new AhtBezierCurve2D(aPoles, 2, -1.0, 0.0));
    }

    [Fact]
    public void Construction_NegativeBeta_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 0.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new AhtBezierCurve2D(aPoles, 2, 0.0, -1.0));
    }

    [Fact]
    public void Construction_WeightCountMismatch_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 2);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new AhtBezierCurve2D(aPoles, aWeights, 2, 0.0, 0.0));
    }

    [Fact]
    public void Construction_NonPositiveWeight_Throws()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        aPoles.SetValue(3, new Point2D(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, -1.0);
        aWeights.SetValue(3, 1.0);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new AhtBezierCurve2D(aPoles, aWeights, 2, 0.0, 0.0));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        double aU = 0.3;

        Curve2D.ResD2 aD2 = aCurve.EvalD2(aU);
        Curve2D.ResD1 aD1Plus = aCurve.EvalD1(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD1 aD1Minus = aCurve.EvalD1(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aD1Plus.D1.XY() - aD1Minus.D1.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD2.D2.X, THE_FD_TOL_D2);
        Assert.Equal(aFD.Y, aD2.D2.Y, THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        double aU = 0.4;

        Curve2D.ResD3 aD3 = aCurve.EvalD3(aU);
        Curve2D.ResD2 aD2Plus = aCurve.EvalD2(aU + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD2 aD2Minus = aCurve.EvalD2(aU - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector2D aFD = new((aD2Plus.D2.XY() - aD2Minus.D2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD3.D3.X, THE_FD_TOL_D3);
        Assert.Equal(aFD.Y, aD3.D3.Y, THE_FD_TOL_D3);
    }

    [Fact]
    public void EvalDN_ConsistentWithD1()
    {
        AhtBezierCurve2D aCurve = createFullBasisCurve();
        double aU = 0.5;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aU);
        Vector2D aDN = aCurve.EvalDN(aU, 1);

        Assert.Equal(aDN.X, aD1.D1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN.Y, aD1.D1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_PurePolynomial_Line()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 2);
        aPoles.SetValue(1, new Point2D(0.0, 0.0));
        aPoles.SetValue(2, new Point2D(1.0, 1.0));
        AhtBezierCurve2D aCurve = new(aPoles, 1, 0.0, 0.0);

        Point2D aPtHalf = aCurve.EvalD0(0.5);
        Assert.Equal(0.5, aPtHalf.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.5, aPtHalf.Y, global::Kerf.Engine.Foundation.Precision.Confusion());

        Point2D aPtEnd = aCurve.EvalD0(1.0);
        Assert.Equal(1.0, aPtEnd.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aPtEnd.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve,
    // for each combination of basis parts, non-rational and rational, including the general EvalDN.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        (int Deg, double Alpha, double Beta)[] aCases =
            [(0, 0.0, 0.0), (3, 0.0, 0.0), (5, 0.0, 0.0), (2, 1.3, 0.0), (1, 0.0, 2.1), (2, 0.9, 1.7), (0, 1.1, 2.4)];
        foreach ((int aDeg, double anAlpha, double aBeta) in aCases)
        {
            int aNb = aDeg + 1 + (anAlpha > 0.0 ? 2 : 0) + (aBeta > 0.0 ? 2 : 0);
            NCollection_Array1<Point2D> aPoles = new(1, aNb);
            NCollection_Array1<double> aWeights = new(1, aNb);
            for (int i = 1; i <= aNb; ++i)
            {
                aPoles.SetValue(i, new Point2D(Math.Cos(1.7 * i) * i, Math.Sin(0.9 * i) + 0.3 * i));
                aWeights.SetValue(i, 1.0 + 0.4 * Math.Sin(2.3 * i));
            }
            AhtBezierCurve2D aCurve = new(aPoles, aDeg, anAlpha, aBeta);
            AhtBezierCurve2D aRational = new(aPoles, aWeights, aDeg, anAlpha, aBeta);
            for (double aU = 0.03; aU < 1.0; aU += 0.17)
            {
                EvalTestUtils.CheckCurveDerivatives(aCurve, aU, 7);
                EvalTestUtils.CheckCurveDerivatives(aRational, aU, 6);
            }
        }
    }
}
