// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_AHTBezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash, DumpJson_Rational_NoCrash (DumpJson is not ported).
public class Geom2dEval_AHTBezierCurveTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;
    private const double THE_FD_TOL_D3 = 1e-3;

    // Helper to create a curve with basis {1, t, sinh(alpha*t), cosh(alpha*t), sin(beta*t),
    // cos(beta*t)} algDegree=1, alpha=1.0, beta=1.0 => basisDim = 2 + 2 + 2 = 6 poles needed
    private static Geom2dEval_AHTBezierCurve createFullBasisCurve()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 6);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(4, new gp_Pnt2d(0.0, 1.0));
        aPoles.SetValue(5, new gp_Pnt2d(0.0, 1.0));
        aPoles.SetValue(6, new gp_Pnt2d(1.0, 0.0));
        return new Geom2dEval_AHTBezierCurve(aPoles, 1, 1.0, 1.0);
    }

    // Helper to create a pure polynomial curve: basis {1, t, t^2} => 3 poles
    private static Geom2dEval_AHTBezierCurve createPolynomialCurve()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        return new Geom2dEval_AHTBezierCurve(aPoles, 2, 0.0, 0.0);
    }

    // Helper to create a rational polynomial curve with weights
    private static Geom2dEval_AHTBezierCurve createRationalCurve()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 2.0);
        aWeights.SetValue(3, 1.0);
        return new Geom2dEval_AHTBezierCurve(aPoles, aWeights, 2, 0.0, 0.0);
    }

    [Fact]
    public void Construction_FullBasis()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.Equal(6, aCurve.NbPoles());
        Assert.Equal(1, aCurve.AlgDegree());
        Assert.Equal(1.0, aCurve.Alpha(), Precision.Confusion());
        Assert.Equal(1.0, aCurve.Beta(), Precision.Confusion());
        Assert.False(aCurve.IsRational());
    }

    [Fact]
    public void Construction_Polynomial()
    {
        Geom2dEval_AHTBezierCurve aCurve = createPolynomialCurve();
        Assert.Equal(3, aCurve.NbPoles());
        Assert.Equal(2, aCurve.AlgDegree());
        Assert.Equal(0.0, aCurve.Alpha(), Precision.Confusion());
        Assert.Equal(0.0, aCurve.Beta(), Precision.Confusion());
        Assert.False(aCurve.IsRational());
    }

    [Fact]
    public void Construction_HyperbolicOnly()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        Geom2dEval_AHTBezierCurve aCurve = new(aPoles, 0, 2.0, 0.0);
        Assert.Equal(3, aCurve.NbPoles());
        Assert.Equal(0, aCurve.AlgDegree());
        Assert.Equal(2.0, aCurve.Alpha(), Precision.Confusion());
        Assert.Equal(0.0, aCurve.Beta(), Precision.Confusion());
    }

    [Fact]
    public void Construction_Rational()
    {
        Geom2dEval_AHTBezierCurve aCurve = createRationalCurve();
        Assert.True(aCurve.IsRational());
        Assert.Equal(3, aCurve.NbPoles());
        Assert.Equal(2, aCurve.AlgDegree());
    }

    [Fact]
    public void IsRational_NonRational()
    {
        Geom2dEval_AHTBezierCurve aCurve = createPolynomialCurve();
        Assert.False(aCurve.IsRational());
    }

    [Fact]
    public void Bounds()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.Equal(0.0, aCurve.FirstParameter(), Precision.Confusion());
        Assert.Equal(1.0, aCurve.LastParameter(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Endpoints()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();

        gp_Pnt2d aStart = aCurve.EvalD0(aCurve.FirstParameter());
        gp_Pnt2d aEnd = aCurve.EvalD0(aCurve.LastParameter());

        Assert.Equal(0.0, aStart.Distance(aCurve.StartPoint()), Precision.Confusion());
        Assert.Equal(0.0, aEnd.Distance(aCurve.EndPoint()), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_KnownPoint()
    {
        Geom2dEval_AHTBezierCurve aCurve = createPolynomialCurve();
        gp_Pnt2d aPnt = aCurve.EvalD0(0.5);
        Assert.Equal(1.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.5, aPnt.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.4;

        Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        gp_Pnt2d aP1 = aCurve.EvalD0(aU + Precision.Confusion());
        gp_Pnt2d aP2 = aCurve.EvalD0(aU - Precision.Confusion());

        gp_Vec2d aFD = new((aP1.XY() - aP2.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL_D1);
    }

    [Fact]
    public void EvalD1_Polynomial_ConsistentWithD0()
    {
        Geom2dEval_AHTBezierCurve aCurve = createPolynomialCurve();
        double aU = 0.3;

        Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        gp_Pnt2d aP1 = aCurve.EvalD0(aU + Precision.Confusion());
        gp_Pnt2d aP2 = aCurve.EvalD0(aU - Precision.Confusion());

        gp_Vec2d aFD = new((aP1.XY() - aP2.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL_D1);
    }

    [Fact]
    public void EvalD1_Rational_ConsistentWithD0()
    {
        Geom2dEval_AHTBezierCurve aCurve = createRationalCurve();
        double aU = 0.5;

        Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        gp_Pnt2d aP1 = aCurve.EvalD0(aU + Precision.Confusion());
        gp_Pnt2d aP2 = aCurve.EvalD0(aU - Precision.Confusion());

        gp_Vec2d aFD = new((aP1.XY() - aP2.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL_D1);
    }

    [Fact]
    public void Accessors()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 5);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 1.0));
        aPoles.SetValue(4, new gp_Pnt2d(3.0, 1.0));
        aPoles.SetValue(5, new gp_Pnt2d(4.0, 0.0));
        // algDegree=2 => 3 polynomial basis + 2 trig basis = 5 => beta>0
        Geom2dEval_AHTBezierCurve aCurve = new(aPoles, 2, 0.0, 3.5);
        Assert.Equal(5, aCurve.NbPoles());
        Assert.Equal(2, aCurve.AlgDegree());
        Assert.Equal(0.0, aCurve.Alpha(), Precision.Confusion());
        Assert.Equal(3.5, aCurve.Beta(), Precision.Confusion());
    }

    [Fact]
    public void Periodicity()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.False(aCurve.IsPeriodic());
    }

    [Fact]
    public void Continuity()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.Equal(GeomAbs_Shape.GeomAbs_CN, aCurve.Continuity());
        Assert.True(aCurve.IsCN(0));
        Assert.True(aCurve.IsCN(5));
        Assert.True(aCurve.IsCN(100));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(1.0, 2.0));

        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Geom2d_Geometry aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        Geom2dEval_AHTBezierCurve? aCopyCurve = aCopy as Geom2dEval_AHTBezierCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(6, aCopyCurve.NbPoles());
        Assert.Equal(1, aCopyCurve.AlgDegree());

        // Verify the copy evaluates identically at several parameters
        for (double aU = 0.0; aU <= 1.0; aU += 0.25)
        {
            gp_Pnt2d aOrigPnt = aCurve.EvalD0(aU);
            gp_Pnt2d aCopyPnt = aCopyCurve.EvalD0(aU);
            Assert.Equal(0.0, aOrigPnt.Distance(aCopyPnt), Precision.Confusion());
        }

        // Verify behavior: transformation is not supported on the copy either.
        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(10.0, 10.0));
        Geom2dEval_AHTBezierCurve aCopyHandle = (Geom2dEval_AHTBezierCurve)aCopy;
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCopyHandle.Transform(aTrsf));

        gp_Pnt2d aOrigMid = aCurve.EvalD0(0.5);
        gp_Pnt2d aCopyMid = aCopyHandle.EvalD0(0.5);
        Assert.Equal(0.0, aOrigMid.Distance(aCopyMid), Precision.Confusion());
    }

    [Fact]
    public void Copy_Rational()
    {
        Geom2dEval_AHTBezierCurve aCurve = createRationalCurve();
        Geom2d_Geometry aCopy = aCurve.Copy();
        Geom2dEval_AHTBezierCurve? aCopyCurve = aCopy as Geom2dEval_AHTBezierCurve;
        Assert.NotNull(aCopyCurve);
        Assert.True(aCopyCurve.IsRational());
    }

    [Fact]
    public void Construction_PoleCountMismatch_Throws()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 4);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(4, new gp_Pnt2d(0.0, 1.0));
        // algDegree=1, alpha=1.0, beta=1.0 => basisDim = 2+2+2 = 6, but only 4 poles
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_AHTBezierCurve(aPoles, 1, 1.0, 1.0));
    }

    [Fact]
    public void Construction_NegativeAlpha_Throws()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_AHTBezierCurve(aPoles, 2, -1.0, 0.0));
    }

    [Fact]
    public void Construction_NegativeBeta_Throws()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_AHTBezierCurve(aPoles, 2, 0.0, -1.0));
    }

    [Fact]
    public void Construction_WeightCountMismatch_Throws()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 2);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 1.0);
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_AHTBezierCurve(aPoles, aWeights, 2, 0.0, 0.0));
    }

    [Fact]
    public void Construction_NonPositiveWeight_Throws()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 1.0));
        aPoles.SetValue(3, new gp_Pnt2d(2.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, -1.0);
        aWeights.SetValue(3, 1.0);
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_AHTBezierCurve(aPoles, aWeights, 2, 0.0, 0.0));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Reverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.3;

        Geom2d_Curve.ResD2 aD2 = aCurve.EvalD2(aU);
        Geom2d_Curve.ResD1 aD1Plus = aCurve.EvalD1(aU + Precision.Confusion());
        Geom2d_Curve.ResD1 aD1Minus = aCurve.EvalD1(aU - Precision.Confusion());

        gp_Vec2d aFD = new((aD1Plus.D1.XY() - aD1Minus.D1.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD2.D2.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Y(), aD2.D2.Y(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.4;

        Geom2d_Curve.ResD3 aD3 = aCurve.EvalD3(aU);
        Geom2d_Curve.ResD2 aD2Plus = aCurve.EvalD2(aU + Precision.Confusion());
        Geom2d_Curve.ResD2 aD2Minus = aCurve.EvalD2(aU - Precision.Confusion());

        gp_Vec2d aFD = new((aD2Plus.D2.XY() - aD2Minus.D2.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD3.D3.X(), THE_FD_TOL_D3);
        Assert.Equal(aFD.Y(), aD3.D3.Y(), THE_FD_TOL_D3);
    }

    [Fact]
    public void EvalDN_ConsistentWithD1()
    {
        Geom2dEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.5;

        Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        gp_Vec2d aDN = aCurve.EvalDN(aU, 1);

        Assert.Equal(aDN.X(), aD1.D1.X(), Precision.Confusion());
        Assert.Equal(aDN.Y(), aD1.D1.Y(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_PurePolynomial_Line()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 2);
        aPoles.SetValue(1, new gp_Pnt2d(0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt2d(1.0, 1.0));
        Geom2dEval_AHTBezierCurve aCurve = new(aPoles, 1, 0.0, 0.0);

        gp_Pnt2d aPtHalf = aCurve.EvalD0(0.5);
        Assert.Equal(0.5, aPtHalf.X(), Precision.Confusion());
        Assert.Equal(0.5, aPtHalf.Y(), Precision.Confusion());

        gp_Pnt2d aPtEnd = aCurve.EvalD0(1.0);
        Assert.Equal(1.0, aPtEnd.X(), Precision.Confusion());
        Assert.Equal(1.0, aPtEnd.Y(), Precision.Confusion());
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
            NCollection_Array1<gp_Pnt2d> aPoles = new(1, aNb);
            NCollection_Array1<double> aWeights = new(1, aNb);
            for (int i = 1; i <= aNb; ++i)
            {
                aPoles.SetValue(i, new gp_Pnt2d(Math.Cos(1.7 * i) * i, Math.Sin(0.9 * i) + 0.3 * i));
                aWeights.SetValue(i, 1.0 + 0.4 * Math.Sin(2.3 * i));
            }
            Geom2dEval_AHTBezierCurve aCurve = new(aPoles, aDeg, anAlpha, aBeta);
            Geom2dEval_AHTBezierCurve aRational = new(aPoles, aWeights, aDeg, anAlpha, aBeta);
            for (double aU = 0.03; aU < 1.0; aU += 0.17)
            {
                EvalTestUtils.CheckCurveDerivatives(aCurve, aU, 7);
                EvalTestUtils.CheckCurveDerivatives(aRational, aU, 6);
            }
        }
    }
}
