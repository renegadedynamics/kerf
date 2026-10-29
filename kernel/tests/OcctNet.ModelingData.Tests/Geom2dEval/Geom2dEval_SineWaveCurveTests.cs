// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_SineWaveCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class Geom2dEval_SineWaveCurveTest
{
    private const double THE_FD_TOL = 1e-5;
    private const double THE_FD_TOL_DN = 1e-4;

    [Fact]
    public void Construction_ValidParams()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.0);
        Assert.Equal(2.0, aCurve.Amplitude(), Precision.Confusion());
        Assert.Equal(3.0, aCurve.Omega(), Precision.Confusion());
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        gp_Ax2d anAx2d = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_SineWaveCurve(anAx2d, 0.0, 1.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_SineWaveCurve(anAx2d, 1.0, 0.0));
    }

    [Fact]
    public void EvalD0_AtKnownPoints()
    {
        gp_Ax2d anAx2d = new();
        const double aA = 2.0, aOm = 3.0;
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, aA, aOm, 0.0);

        // t=0: C(0) = (0, 0)
        gp_Pnt2d aP0 = aCurve.EvalD0(0.0);
        Assert.Equal(0.0, aP0.X(), Precision.Confusion());
        Assert.Equal(0.0, aP0.Y(), Precision.Confusion());

        // t = Pi/(2*omega): C = (Pi/(2*omega), A)
        double aT1 = M_PI / (2.0 * aOm);
        gp_Pnt2d aP1 = aCurve.EvalD0(aT1);
        Assert.Equal(aT1, aP1.X(), Precision.Confusion());
        Assert.Equal(aA, aP1.Y(), Precision.Confusion());
    }

    [Fact]
    public void IsPeriodic()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.0);
        Assert.False(aCurve.IsPeriodic());
        Assert.ThrowsAny<Standard_NoSuchObject>(() => aCurve.Period());
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Reverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.0);
        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(1.0, 2.0));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.5);
        const double aT = 1.0;

        Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aT);
        gp_Pnt2d aP1 = aCurve.EvalD0(aT + Precision.Confusion());
        gp_Pnt2d aP2 = aCurve.EvalD0(aT - Precision.Confusion());
        gp_Vec2d aFD = new((aP1.XY() - aP2.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL);
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.5);
        const double aT = 1.0;

        Geom2d_Curve.ResD2 aD2 = aCurve.EvalD2(aT);
        Geom2d_Curve.ResD1 aD1p = aCurve.EvalD1(aT + Precision.Confusion());
        Geom2d_Curve.ResD1 aD1m = aCurve.EvalD1(aT - Precision.Confusion());
        gp_Vec2d aFD = new((aD1p.D1.XY() - aD1m.D1.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD2.D2.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD2.D2.Y(), THE_FD_TOL);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.5);
        const double aT = 1.0;

        Geom2d_Curve.ResD3 aD3 = aCurve.EvalD3(aT);
        Geom2d_Curve.ResD2 aD2p = aCurve.EvalD2(aT + Precision.Confusion());
        Geom2d_Curve.ResD2 aD2m = aCurve.EvalD2(aT - Precision.Confusion());
        gp_Vec2d aFD = new((aD2p.D2.XY() - aD2m.D2.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aD3.D3.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD3.D3.Y(), THE_FD_TOL);
    }

    [Fact]
    public void EvalDN_HigherOrder_ConsistentWithPreviousOrder()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.5);
        const double aT = 1.2;
        const int aN = 6;

        gp_Vec2d aDN = aCurve.EvalDN(aT, aN);
        gp_Vec2d aDNm1P = aCurve.EvalDN(aT + Precision.Confusion(), aN - 1);
        gp_Vec2d aDNm1M = aCurve.EvalDN(aT - Precision.Confusion(), aN - 1);
        gp_Vec2d aFD = new((aDNm1P.XY() - aDNm1M.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aDN.X(), THE_FD_TOL_DN);
        Assert.Equal(aFD.Y(), aDN.Y(), THE_FD_TOL_DN);
    }

    [Fact]
    public void Copy_Independent()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 2.0, 3.0, 0.5);
        Geom2d_Geometry aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        Geom2dEval_SineWaveCurve? aCopyCurve = aCopy as Geom2dEval_SineWaveCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(2.0, aCopyCurve.Amplitude(), Precision.Confusion());
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        gp_Ax2d anAx2d = new(new gp_Pnt2d(1.0, -2.0), new gp_Dir2d(1.0, 2.0));
        Geom2dEval_SineWaveCurve aCurve = new(anAx2d, 1.5, 2.5, 0.3);
        for (double aT = -2.0; aT <= 2.0; aT += 0.37)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aT);
        }
    }
}
