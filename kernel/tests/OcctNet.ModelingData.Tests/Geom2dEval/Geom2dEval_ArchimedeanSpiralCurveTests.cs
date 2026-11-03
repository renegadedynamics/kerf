// Port of: src/ModelingData/TKG2d/GTests/Geom2dEval_ArchimedeanSpiralCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class Geom2dEval_ArchimedeanSpiralCurveTest
{
    private const double THE_FD_TOL = 1e-5;
    private const double THE_FD_TOL_DN = 1e-4;

    [Fact]
    public void Construction_ValidParams()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 0.0, 1.0);
        Assert.Equal(0.0, aCurve.InitialRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aCurve.GrowthRate(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        Axis2D anAx2d = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ArchimedeanSpiralCurve2D(anAx2d, -1.0, 1.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ArchimedeanSpiralCurve2D(anAx2d, 0.0, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ArchimedeanSpiralCurve2D(anAx2d, 0.0, -1.0));
    }

    [Fact]
    public void EvalD0_DistanceGrowsLinearly()
    {
        Axis2D anAx2d = new();
        double aA = 0.0, aB = 1.0;
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, aA, aB);

        // |C(t) - O| = a + b*t
        Point2D anO = anAx2d.Location();
        double[] aParams = [0.0, 1.0, 2.0, M_PI, 10.0];
        foreach (double aT in aParams)
        {
            Point2D aP = aCurve.EvalD0(aT);
            Assert.Equal(aA + aB * aT, anO.Distance(aP), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void EvalD0_WithInitialRadius()
    {
        Axis2D anAx2d = new();
        double aA = 2.0, aB = 0.5;
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, aA, aB);

        // t=0: C(0) = (a, 0)
        Point2D aP0 = aCurve.EvalD0(0.0);
        Assert.Equal(aA, aP0.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP0.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.5);
        double aT = 3.0;

        Curve2D.ResD1 aD1 = aCurve.EvalD1(aT);
        Point2D aP1 = aCurve.EvalD0(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point2D aP2 = aCurve.EvalD0(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aFD = new((aP1.XY() - aP2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL);
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.5);
        double aT = 3.0;

        Curve2D.ResD2 aD2 = aCurve.EvalD2(aT);
        Curve2D.ResD1 aD1p = aCurve.EvalD1(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD1 aD1m = aCurve.EvalD1(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aFD = new((aD1p.D1.XY() - aD1m.D1.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD2.D2.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD2.D2.Y(), THE_FD_TOL);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.5);
        double aT = 3.0;

        Curve2D.ResD3 aD3 = aCurve.EvalD3(aT);
        Curve2D.ResD2 aD2p = aCurve.EvalD2(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Curve2D.ResD2 aD2m = aCurve.EvalD2(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector2D aFD = new((aD2p.D2.XY() - aD2m.D2.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD3.D3.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD3.D3.Y(), THE_FD_TOL);
    }

    [Fact]
    public void EvalDN_HigherOrder_ConsistentWithPreviousOrder()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 1.0, 0.5);
        double aT = 2.0;
        int aN = 6;

        Vector2D aDN = aCurve.EvalDN(aT, aN);
        Vector2D aDNm1P = aCurve.EvalDN(aT + global::Kerf.Engine.Foundation.Precision.Confusion(), aN - 1);
        Vector2D aDNm1M = aCurve.EvalDN(aT - global::Kerf.Engine.Foundation.Precision.Confusion(), aN - 1);
        Vector2D aFD = new((aDNm1P.XY() - aDNm1M.XY()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aDN.X(), THE_FD_TOL_DN);
        Assert.Equal(aFD.Y(), aDN.Y(), THE_FD_TOL_DN);
    }

    [Fact]
    public void Properties()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 0.0, 1.0);
        Assert.False(aCurve.IsClosed);
        Assert.False(aCurve.IsPeriodic);
        Assert.Equal(Continuity.CN, aCurve.Continuity());
        Assert.Equal(0.0, aCurve.FirstParameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aCurve.LastParameter()));
    }

    [Fact]
    public void Copy_Independent()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 1.0, 2.0);
        Geometry2D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        ArchimedeanSpiralCurve2D? aCopyCurve = aCopy as ArchimedeanSpiralCurve2D;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(1.0, aCopyCurve.InitialRadius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0, aCopyCurve.GrowthRate(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 0.0, 1.0);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        Axis2D anAx2d = new();
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 0.0, 1.0);
        Transform2D aTrsf = new();
        aTrsf.SetTranslation(new Vector2D(1.0, 2.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        Axis2D anAx2d = new(new Point2D(1.0, -2.0), new Direction2D(1.0, 2.0));
        ArchimedeanSpiralCurve2D aCurve = new(anAx2d, 0.7, 0.4);
        for (double aT = 0.05; aT <= 8.0; aT += 0.53)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aT);
        }
    }
}
