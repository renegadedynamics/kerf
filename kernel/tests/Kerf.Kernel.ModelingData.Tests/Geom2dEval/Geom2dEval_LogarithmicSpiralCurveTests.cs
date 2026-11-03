// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_LogarithmicSpiralCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class Geom2dEval_LogarithmicSpiralCurveTest
{
    private const double THE_FD_TOL = 1e-5;
    private const double THE_FD_TOL_DN = 1e-4;

    [Fact]
    public void Construction_ValidParams()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.2);
        Assert.Equal(1.0, aCurve.Scale(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.2, aCurve.GrowthExponent(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        Axis2D anAx2d = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new LogarithmicSpiralCurve2D(anAx2d, 0.0, 0.2));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new LogarithmicSpiralCurve2D(anAx2d, -1.0, 0.2));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new LogarithmicSpiralCurve2D(anAx2d, 1.0, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new LogarithmicSpiralCurve2D(anAx2d, 1.0, -0.2));
    }

    [Fact]
    public void SelfSimilarity()
    {
        Axis2D anAx2d = new();
        double aA = 1.0, aB = 0.2;
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, aA, aB);

        // C(t+k) should be a scaled version of C(t) by factor e^(b*k)
        double aK = 2.0;
        double aScaleFactor = Math.Exp(aB * aK);
        Point2D anO = anAx2d.Location;

        double[] aParams = [0.0, 1.0, 2.0, M_PI];
        foreach (double aT in aParams)
        {
            Point2D aP1 = aCurve.EvalD0(aT);
            Point2D aP2 = aCurve.EvalD0(aT + aK);

            // |P2 - O| / |P1 - O| should equal e^(b*k)
            double aDist1 = anO.DistanceTo(aP1);
            double aDist2 = anO.DistanceTo(aP2);
            if (aDist1 > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                Assert.Equal(aScaleFactor, aDist2 / aDist1, 1e-10);
            }
        }
    }

    [Fact]
    public void ConstantAngle()
    {
        Axis2D anAx2d = new();
        double aA = 1.0, aB = 0.2;
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, aA, aB);

        // Angle between tangent and radial direction = atan(1/b)
        double aExpAngle = Math.Atan(1.0 / aB);
        Point2D anO = anAx2d.Location;

        double[] aParams = [1.0, 2.0, M_PI, 5.0];
        foreach (double aT in aParams)
        {
            Curve2D.ResD1 aD1 = aCurve.EvalD1(aT);
            Vector2D aRadial = new(anO, aD1.Point);
            double aDot = aD1.D1.Dot(aRadial);
            double aCross = aD1.D1.Crossed(aRadial);
            double anAngle = Math.Abs(Math.Atan2(Math.Abs(aCross), aDot));
            Assert.Equal(aExpAngle, anAngle, 1e-10);
        }
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 2.0;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aT);
        Point2D aP1 = aCurve.EvalD0(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL);
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 2.0;

        Curve2D.ResD2 aD2 = aCurve.EvalD2(aT);
        Curve2D.ResD1 aD1p = aCurve.EvalD1(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD1 aD1m = aCurve.EvalD1(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aFD = new((aD1p.D1.XY() - aD1m.D1.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD2.D2.X, THE_FD_TOL);
        Assert.Equal(aFD.Y, aD2.D2.Y, THE_FD_TOL);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 2.0;

        Curve2D.ResD3 aD3 = aCurve.EvalD3(aT);
        Curve2D.ResD2 aD2p = aCurve.EvalD2(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD2 aD2m = aCurve.EvalD2(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aFD = new((aD2p.D2.XY() - aD2m.D2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD3.D3.X, THE_FD_TOL);
        Assert.Equal(aFD.Y, aD3.D3.Y, THE_FD_TOL);
    }

    [Fact]
    public void EvalDN_HigherOrder_ConsistentWithPreviousOrder()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.3);
        double aT = 1.2;
        int aN = 6;

        Vector2D aDN = aCurve.EvalDN(aT, aN);
        Vector2D aDNm1P = aCurve.EvalDN(aT + global::Kerf.Engine.Foundation.Precision.Confusion(), aN - 1);
        Vector2D aDNm1M = aCurve.EvalDN(aT - global::Kerf.Engine.Foundation.Precision.Confusion(), aN - 1);
        Vector2D aFD = new((aDNm1P.XY() - aDNm1M.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aDN.X, THE_FD_TOL_DN);
        Assert.Equal(aFD.Y, aDN.Y, THE_FD_TOL_DN);
    }

    [Fact]
    public void Properties()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.2);
        Assert.False(aCurve.IsClosed);
        Assert.False(aCurve.IsPeriodic);
        Assert.Equal(Continuity.CN, aCurve.Continuity);
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.2);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.2);
        Transform2D aTrsf = new Transform2D().WithTranslation(new Vector2D(1.0, 2.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        Axis2D anAx2d = new();
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.2);
        Geometry2D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        LogarithmicSpiralCurve2D? aCopyCurve = aCopy as LogarithmicSpiralCurve2D;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(1.0, aCopyCurve.Scale(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.2, aCopyCurve.GrowthExponent(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        Axis2D anAx2d = new(new Point2D(1.0, -2.0), new Direction2D(1.0, 2.0));
        LogarithmicSpiralCurve2D aCurve = new(anAx2d, 0.8, 0.15);
        for (double aT = -3.0; aT <= 4.0; aT += 0.47)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aT);
        }
    }
}
