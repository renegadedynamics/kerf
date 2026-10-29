// Port of: src/ModelingData/TKG3d/GTests/GeomEval_AHTBezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_AHTBezierCurveTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;

    // Helper to create a curve with basis {1, t, sinh(alpha*t), cosh(alpha*t), sin(beta*t),
    // cos(beta*t)} algDegree=1, alpha=1.0, beta=1.0 => basisDim = 2 + 2 + 2 = 6 poles needed
    private static GeomEval_AHTBezierCurve createFullBasisCurve()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 6);
        aPoles.SetValue(1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(1.0, 0.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt(1.0, 1.0, 0.0));
        aPoles.SetValue(4, new gp_Pnt(0.0, 1.0, 0.0));
        aPoles.SetValue(5, new gp_Pnt(0.0, 1.0, 1.0));
        aPoles.SetValue(6, new gp_Pnt(1.0, 0.0, 1.0));
        return new GeomEval_AHTBezierCurve(aPoles, 1, 1.0, 1.0);
    }

    // Helper to create a pure trigonometric AHT curve: basis {1, sin(t), cos(t)} => 3 poles
    // C(t) = P_0 + P_1*sin(t) + P_2*cos(t) = (cos(t), sin(t), 0), t in [0, 1].
    private static GeomEval_AHTBezierCurve createTrigCircleArc()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(0.0, 1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt(1.0, 0.0, 0.0));
        return new GeomEval_AHTBezierCurve(aPoles, 0, 0.0, 1.0);
    }

    // Helper to create a pure trigonometric AHT ellipse arc: basis {1, sin(t), cos(t)} => 3 poles
    // C(t) = (3*cos(t), 2*sin(t), 0), t in [0, 1].
    private static GeomEval_AHTBezierCurve createTrigEllipseArc()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(0.0, 2.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt(3.0, 0.0, 0.0));
        return new GeomEval_AHTBezierCurve(aPoles, 0, 0.0, 1.0);
    }

    // Helper to create a pure polynomial curve: basis {1, t, t^2} => 3 poles
    private static GeomEval_AHTBezierCurve createPolynomialCurve()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(1.0, 1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt(2.0, 0.0, 0.0));
        return new GeomEval_AHTBezierCurve(aPoles, 2, 0.0, 0.0);
    }

    [Fact]
    public void Construction_FullBasis()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.Equal(6, aCurve.NbPoles());
        Assert.Equal(1, aCurve.AlgDegree());
        Assert.Equal(1.0, aCurve.Alpha(), Precision.Confusion());
        Assert.Equal(1.0, aCurve.Beta(), Precision.Confusion());
        Assert.False(aCurve.IsRational());
    }

    [Fact]
    public void Construction_Polynomial()
    {
        GeomEval_AHTBezierCurve aCurve = createPolynomialCurve();
        Assert.Equal(3, aCurve.NbPoles());
        Assert.Equal(2, aCurve.AlgDegree());
        Assert.Equal(0.0, aCurve.Alpha(), Precision.Confusion());
        Assert.Equal(0.0, aCurve.Beta(), Precision.Confusion());
    }

    [Fact]
    public void ParameterRange()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.Equal(0.0, aCurve.FirstParameter(), Precision.Confusion());
        Assert.Equal(1.0, aCurve.LastParameter(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Endpoints()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();

        gp_Pnt aStart = aCurve.EvalD0(aCurve.FirstParameter());
        gp_Pnt aEnd = aCurve.EvalD0(aCurve.LastParameter());

        Assert.Equal(0.0, aStart.Distance(aCurve.StartPoint()), Precision.Confusion());
        Assert.Equal(0.0, aEnd.Distance(aCurve.EndPoint()), Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.4;

        Geom_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        gp_Pnt aP1 = aCurve.EvalD0(aU + Precision.Confusion());
        gp_Pnt aP2 = aCurve.EvalD0(aU - Precision.Confusion());

        gp_Vec aFD = new((aP1.XYZ() - aP2.XYZ()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL_D1);
        Assert.Equal(aFD.Z(), aD1.D1.Z(), THE_FD_TOL_D1);
    }

    [Fact]
    public void Periodicity()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.False(aCurve.IsPeriodic());
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Reverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void Construction_Rational()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles.SetValue(1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(1.0, 1.0, 0.0));
        aPoles.SetValue(3, new gp_Pnt(2.0, 0.0, 0.0));
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights.SetValue(1, 1.0);
        aWeights.SetValue(2, 2.0);
        aWeights.SetValue(3, 1.0);
        GeomEval_AHTBezierCurve aCurve = new(aPoles, aWeights, 2, 0.0, 0.0);
        Assert.True(aCurve.IsRational());
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(1.0, 2.0, 3.0));

        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        Geom_Geometry aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        GeomEval_AHTBezierCurve? aCopyCurve = aCopy as GeomEval_AHTBezierCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(6, aCopyCurve.NbPoles());
        Assert.Equal(1, aCopyCurve.AlgDegree());
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.3;

        Geom_Curve.ResD2 aD2 = aCurve.EvalD2(aU);
        Geom_Curve.ResD1 aD1Plus = aCurve.EvalD1(aU + Precision.Confusion());
        Geom_Curve.ResD1 aD1Minus = aCurve.EvalD1(aU - Precision.Confusion());

        gp_Vec aFD = new((aD1Plus.D1.XYZ() - aD1Minus.D1.XYZ()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD2.D2.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Y(), aD2.D2.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Z(), aD2.D2.Z(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.4;

        Geom_Curve.ResD3 aD3 = aCurve.EvalD3(aU);
        Geom_Curve.ResD2 aD2Plus = aCurve.EvalD2(aU + Precision.Confusion());
        Geom_Curve.ResD2 aD2Minus = aCurve.EvalD2(aU - Precision.Confusion());

        gp_Vec aFD = new((aD2Plus.D2.XYZ() - aD2Minus.D2.XYZ()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD3.D3.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Y(), aD3.D3.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD.Z(), aD3.D3.Z(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalDN_ConsistentWithD1()
    {
        GeomEval_AHTBezierCurve aCurve = createFullBasisCurve();
        double aU = 0.5;

        Geom_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        gp_Vec aDN = aCurve.EvalDN(aU, 1);

        Assert.Equal(aDN.X(), aD1.D1.X(), Precision.Confusion());
        Assert.Equal(aDN.Y(), aD1.D1.Y(), Precision.Confusion());
        Assert.Equal(aDN.Z(), aD1.D1.Z(), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_PurePolynomial_KnownValues()
    {
        gp_Pnt aP0 = new(0.0, 0.0, 0.0);
        gp_Pnt aP1 = new(0.5, 1.0, 0.0);
        gp_Pnt aP2 = new(1.0, 0.0, 0.0);

        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles.SetValue(1, aP0);
        aPoles.SetValue(2, aP1);
        aPoles.SetValue(3, aP2);
        GeomEval_AHTBezierCurve aCurve = new(aPoles, 2, 0.0, 0.0);

        // P(0) = P0
        gp_Pnt aVal0 = aCurve.EvalD0(0.0);
        Assert.Equal(0.0, aVal0.Distance(aP0), Precision.Confusion());

        // P(0.5) = P0 + 0.5*P1 + 0.25*P2
        gp_Pnt aExpected05 = new(aP0.X() + 0.5 * aP1.X() + 0.25 * aP2.X(),
            aP0.Y() + 0.5 * aP1.Y() + 0.25 * aP2.Y(),
            aP0.Z() + 0.5 * aP1.Z() + 0.25 * aP2.Z());
        gp_Pnt aVal05 = aCurve.EvalD0(0.5);
        Assert.Equal(0.0, aVal05.Distance(aExpected05), Precision.Confusion());

        // P(1) = P0 + P1 + P2
        gp_Pnt aExpected1 = new(aP0.X() + aP1.X() + aP2.X(),
            aP0.Y() + aP1.Y() + aP2.Y(),
            aP0.Z() + aP1.Z() + aP2.Z());
        gp_Pnt aVal1 = aCurve.EvalD0(1.0);
        Assert.Equal(0.0, aVal1.Distance(aExpected1), Precision.Confusion());
    }

    [Fact]
    public void EvalD0_MatchesCircle()
    {
        GeomEval_AHTBezierCurve aAHT = createTrigCircleArc();
        // Geom_Circle (unit 1A) evaluates through ElCLib; the circle is used directly.
        gp_Circ aCircle = new(new gp_Ax2(), 1.0);

        double[] aParams = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0];
        foreach (double aU in aParams)
        {
            gp_Pnt aPAHT = aAHT.EvalD0(aU);
            gp_Pnt aPCirc = ElCLib.Value(aU, aCircle);
            Assert.Equal(0.0, aPAHT.Distance(aPCirc), Precision.Angular());
        }
    }

    [Fact]
    public void EvalD3_MatchesCircle()
    {
        GeomEval_AHTBezierCurve aAHT = createTrigCircleArc();
        // Geom_Circle (unit 1A) evaluates through ElCLib; the circle is used directly.
        gp_Circ aCircle = new(new gp_Ax2(), 1.0);

        double[] aParams = [0.1, 0.25, 0.5, 0.75, 0.9];
        foreach (double aU in aParams)
        {
            Geom_Curve.ResD3 aDT = aAHT.EvalD3(aU);
            Geom_Curve.ResD3 aDC;
            ElCLib.D3(aU, aCircle, out aDC.Point, out aDC.D1, out aDC.D2, out aDC.D3);

            Assert.Equal(0.0, aDT.Point.Distance(aDC.Point), Precision.Angular());
            Assert.Equal(aDC.D1.X(), aDT.D1.X(), Precision.Angular());
            Assert.Equal(aDC.D1.Y(), aDT.D1.Y(), Precision.Angular());
            Assert.Equal(aDC.D1.Z(), aDT.D1.Z(), Precision.Angular());
            Assert.Equal(aDC.D2.X(), aDT.D2.X(), Precision.Angular());
            Assert.Equal(aDC.D2.Y(), aDT.D2.Y(), Precision.Angular());
            Assert.Equal(aDC.D2.Z(), aDT.D2.Z(), Precision.Angular());
            Assert.Equal(aDC.D3.X(), aDT.D3.X(), Precision.Angular());
            Assert.Equal(aDC.D3.Y(), aDT.D3.Y(), Precision.Angular());
            Assert.Equal(aDC.D3.Z(), aDT.D3.Z(), Precision.Angular());
        }
    }

    [Fact]
    public void EvalD2_MatchesEllipse()
    {
        GeomEval_AHTBezierCurve aAHT = createTrigEllipseArc();
        // Geom_Ellipse (unit 1A) evaluates through ElCLib; the ellipse is used directly.
        gp_Elips anEllipse = new(new gp_Ax2(), 3.0, 2.0);

        double[] aParams = [0.1, 0.25, 0.5, 0.75, 0.9];
        foreach (double aU in aParams)
        {
            Geom_Curve.ResD2 aDT = aAHT.EvalD2(aU);
            Geom_Curve.ResD2 aDC;
            ElCLib.D2(aU, anEllipse, out aDC.Point, out aDC.D1, out aDC.D2);

            Assert.Equal(0.0, aDT.Point.Distance(aDC.Point), Precision.Angular());
            Assert.Equal(aDC.D1.X(), aDT.D1.X(), Precision.Angular());
            Assert.Equal(aDC.D1.Y(), aDT.D1.Y(), Precision.Angular());
            Assert.Equal(aDC.D1.Z(), aDT.D1.Z(), Precision.Angular());
            Assert.Equal(aDC.D2.X(), aDT.D2.X(), Precision.Angular());
            Assert.Equal(aDC.D2.Y(), aDT.D2.Y(), Precision.Angular());
            Assert.Equal(aDC.D2.Z(), aDT.D2.Z(), Precision.Angular());
        }
    }

    // OCCT.NET addition: derivatives up to D7 against finite differences for every combination of the basis
    // parts, non-rational and rational (the general EvalDN paths run for N > 3).
    [Theory]
    [InlineData(2, 0.0, 0.0)]
    [InlineData(4, 0.0, 0.0)]
    [InlineData(0, 1.3, 0.0)]
    [InlineData(0, 0.0, 2.1)]
    [InlineData(1, 0.7, 0.0)]
    [InlineData(2, 0.0, 1.7)]
    [InlineData(0, 1.1, 2.3)]
    [InlineData(3, 0.9, 1.9)]
    public void Derivatives_MatchFiniteDifferences(int theAlgDegree, double theAlpha, double theBeta)
    {
        int aNb = theAlgDegree + 1 + (theAlpha > 0.0 ? 2 : 0) + (theBeta > 0.0 ? 2 : 0);
        NCollection_Array1<gp_Pnt> aPoles = new(1, aNb);
        NCollection_Array1<double> aWeights = new(1, aNb);
        for (int i = 1; i <= aNb; ++i)
        {
            aPoles.SetValue(i, new gp_Pnt(Math.Cos(i * 1.3), 0.5 * i, Math.Sin(0.7 * i) - 0.2 * i));
            aWeights.SetValue(i, i == 1 ? 4.0 : 0.5 + 0.1 * i);
        }
        GeomEval_AHTBezierCurve aCurve = new(aPoles, theAlgDegree, theAlpha, theBeta);
        GeomEval_AHTBezierCurve aRational = new(aPoles, aWeights, theAlgDegree, theAlpha, theBeta);
        for (double aU = 0.05; aU < 1.0; aU += 0.15)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aU, 7);
            EvalTestUtils.CheckCurveDerivatives(aRational, aU, 6);
        }
    }
}
