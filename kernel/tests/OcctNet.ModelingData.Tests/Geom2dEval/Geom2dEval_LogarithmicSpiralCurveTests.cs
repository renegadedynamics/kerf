// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_LogarithmicSpiralCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class Geom2dEval_LogarithmicSpiralCurveTest
{
    private const double THE_FD_TOL = 1e-5;
    private const double THE_FD_TOL_DN = 1e-4;

    [Fact]
    public void Construction_ValidParams()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.2);
        Assert.Equal(1.0, aCurve.Scale(), Precision.Confusion());
        Assert.Equal(0.2, aCurve.GrowthExponent(), Precision.Confusion());
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        gp_Ax2d anAx2d = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_LogarithmicSpiralCurve(anAx2d, 0.0, 0.2));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_LogarithmicSpiralCurve(anAx2d, -1.0, 0.2));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_LogarithmicSpiralCurve(anAx2d, 1.0, 0.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Geom2dEval_LogarithmicSpiralCurve(anAx2d, 1.0, -0.2));
    }

    [Fact]
    public void SelfSimilarity()
    {
        gp_Ax2d anAx2d = new();
        double aA = 1.0, aB = 0.2;
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, aA, aB);

        // C(t+k) should be a scaled version of C(t) by factor e^(b*k)
        double aK = 2.0;
        double aScaleFactor = Math.Exp(aB * aK);
        gp_Pnt2d anO = anAx2d.Location();

        double[] aParams = [0.0, 1.0, 2.0, M_PI];
        foreach (double aT in aParams)
        {
            gp_Pnt2d aP1 = aCurve.EvalD0(aT);
            gp_Pnt2d aP2 = aCurve.EvalD0(aT + aK);

            // |P2 - O| / |P1 - O| should equal e^(b*k)
            double aDist1 = anO.Distance(aP1);
            double aDist2 = anO.Distance(aP2);
            if (aDist1 > Precision.Confusion())
            {
                Assert.Equal(aScaleFactor, aDist2 / aDist1, 1e-10);
            }
        }
    }

    [Fact]
    public void ConstantAngle()
    {
        gp_Ax2d anAx2d = new();
        double aA = 1.0, aB = 0.2;
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, aA, aB);

        // Angle between tangent and radial direction = atan(1/b)
        double aExpAngle = Math.Atan(1.0 / aB);
        gp_Pnt2d anO = anAx2d.Location();

        double[] aParams = [1.0, 2.0, M_PI, 5.0];
        foreach (double aT in aParams)
        {
            Geom2d_Curve.ResD1 aD1 = aCurve.EvalD1(aT);
            gp_Vec2d aRadial = new(anO, aD1.Point);
            double aDot = aD1.D1.Dot(aRadial);
            double aCross = aD1.D1.Crossed(aRadial);
            double anAngle = Math.Abs(Math.Atan2(Math.Abs(aCross), aDot));
            Assert.Equal(aExpAngle, anAngle, 1e-10);
        }
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 2.0;

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
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 2.0;

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
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 2.0;

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
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 1.2;
        int aN = 6;

        gp_Vec2d aDN = aCurve.EvalDN(aT, aN);
        gp_Vec2d aDNm1P = aCurve.EvalDN(aT + Precision.Confusion(), aN - 1);
        gp_Vec2d aDNm1M = aCurve.EvalDN(aT - Precision.Confusion(), aN - 1);
        gp_Vec2d aFD = new((aDNm1P.XY() - aDNm1M.XY()) / (2.0 * Precision.Confusion()));

        Assert.Equal(aFD.X(), aDN.X(), THE_FD_TOL_DN);
        Assert.Equal(aFD.Y(), aDN.Y(), THE_FD_TOL_DN);
    }

    [Fact]
    public void Properties()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.2);
        Assert.False(aCurve.IsClosed());
        Assert.False(aCurve.IsPeriodic());
        Assert.Equal(GeomAbs_Shape.GeomAbs_CN, aCurve.Continuity());
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.2);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Reverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.2);
        gp_Trsf2d aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec2d(1.0, 2.0));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        gp_Ax2d anAx2d = new();
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 1.0, 0.2);
        Geom2d_Geometry aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        Geom2dEval_LogarithmicSpiralCurve? aCopyCurve = aCopy as Geom2dEval_LogarithmicSpiralCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(1.0, aCopyCurve.Scale(), Precision.Confusion());
        Assert.Equal(0.2, aCopyCurve.GrowthExponent(), Precision.Confusion());
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        gp_Ax2d anAx2d = new(new gp_Pnt2d(1.0, -2.0), new gp_Dir2d(1.0, 2.0));
        Geom2dEval_LogarithmicSpiralCurve aCurve = new(anAx2d, 0.8, 0.15);
        for (double aT = -3.0; aT <= 4.0; aT += 0.47)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aT);
        }
    }
}
