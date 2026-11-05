// Port of: src/ModelingData/TKG3d/GTests/GeomEval_CircularHelixCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_CircularHelixCurveTest
{
    private const double THE_FD_TOL = 1e-5;

    [Fact]
    public void Construction_ValidParams()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        Assert.Equal(5.0, aCurve.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aCurve.Pitch(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Construction_InvalidRadius_Throws()
    {
        Frame3D anAx2 = new();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new CircularHelixCurve(anAx2, 0.0, 10.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new CircularHelixCurve(anAx2, -1.0, 10.0));
    }

    [Fact]
    public void Construction_NegativePitch_NoThrow()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, -10.0);
        Assert.Equal(-10.0, aCurve.Pitch());
    }

    [Fact]
    public void EvalD0_AtKnownPoints()
    {
        Frame3D anAx2 = new();
        double aR = 5.0, aP = 10.0;
        CircularHelixCurve aCurve = new(anAx2, aR, aP);

        // t=0: C(0) = (R, 0, 0)
        Point3D aP0 = aCurve.EvalD0(0.0);
        Assert.Equal(aR, aP0.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP0.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP0.Z, global::Kerf.Engine.Foundation.Precision.Confusion());

        // t=Pi/2: C = (0, R, P/4)
        Point3D aP1 = aCurve.EvalD0(M_PI / 2.0);
        Assert.Equal(0.0, aP1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aR, aP1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP / 4.0, aP1.Z, global::Kerf.Engine.Foundation.Precision.Confusion());

        // t=Pi: C = (-R, 0, P/2)
        Point3D aP2 = aCurve.EvalD0(M_PI);
        Assert.Equal(-aR, aP2.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP2.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP / 2.0, aP2.Z, global::Kerf.Engine.Foundation.Precision.Confusion());

        // t=2*Pi: C = (R, 0, P) -- one full turn
        Point3D aP3 = aCurve.EvalD0(2.0 * M_PI);
        Assert.Equal(aR, aP3.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP3.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP, aP3.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConstantSpeed()
    {
        Frame3D anAx2 = new();
        double aR = 5.0, aP = 10.0;
        CircularHelixCurve aCurve = new(anAx2, aR, aP);

        // |D1| should be constant = sqrt(R^2 + (P/(2*Pi))^2)
        double aZRate = aP / (2.0 * M_PI);
        double aExpSpeed = Math.Sqrt(aR * aR + aZRate * aZRate);

        double[] aParams = [0.0, 0.5, 1.0, M_PI, 2.0 * M_PI];
        foreach (double aT in aParams)
        {
            Curve3D.ResD1 aD1 = aCurve.EvalD1(aT);
            Assert.Equal(aExpSpeed, aD1.D1.Magnitude, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        double aT = 1.5;

        Curve3D.ResD1 aD1 = aCurve.EvalD1(aT);
        Point3D aP1 = aCurve.EvalD0(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D aP2 = aCurve.EvalD0(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector3D aFD = new((aP1.XYZ() - aP2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X, aD1.D1.X, THE_FD_TOL);
        Assert.Equal(aFD.Y, aD1.D1.Y, THE_FD_TOL);
        Assert.Equal(aFD.Z, aD1.D1.Z, THE_FD_TOL);
    }

    [Fact]
    public void D1_D2_Orthogonal()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);

        // D2 has no Z component, so D1.D2 = R^2*(-sin*(-cos) + cos*(-sin)) = 0
        double[] aParams = [0.0, 1.0, M_PI, 3.5];
        foreach (double aT in aParams)
        {
            Curve3D.ResD2 aD2 = aCurve.EvalD2(aT);
            // D2 magnitude should be R
            Assert.Equal(5.0, aD2.D2.Magnitude, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void ComparisonWithCircle_ZeroPitch()
    {
        Frame3D anAx2 = new();
        double aR = 5.0;
        CircularHelixCurve aHelix = new(anAx2, aR, 0.0);
        // Geom_Circle (unit 1A) evaluates through ElCLib; the circle is used directly.
        Circle3D aCircle = new(anAx2, aR);

        double[] aParams = [0.0, M_PI / 4.0, M_PI / 2.0, M_PI, 3.0 * M_PI / 2.0];
        foreach (double aT in aParams)
        {
            Point3D aPH = aHelix.EvalD0(aT);
            Point3D aPC = global::Kerf.Engine.Geometry.ElCLib.Value(aT, aCircle);
            Assert.Equal(aPC.X, aPH.X, global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(aPC.Y, aPH.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(aPC.Z, aPH.Z, global::Kerf.Engine.Foundation.Precision.Confusion());

            Curve3D.ResD1 aDH1 = aHelix.EvalD1(aT);
            Curve3D.ResD1 aDC1;
            global::Kerf.Engine.Geometry.ElCLib.D1(aT, aCircle, out aDC1.Point, out aDC1.D1);
            Assert.Equal(aDC1.D1.X, aDH1.D1.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC1.D1.Y, aDH1.D1.Y, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC1.D1.Z, aDH1.D1.Z, global::Kerf.Engine.Foundation.Precision.Angular());

            Curve3D.ResD2 aDH2 = aHelix.EvalD2(aT);
            Curve3D.ResD2 aDC2;
            global::Kerf.Engine.Geometry.ElCLib.D2(aT, aCircle, out aDC2.Point, out aDC2.D1, out aDC2.D2);
            Assert.Equal(aDC2.D2.X, aDH2.D2.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC2.D2.Y, aDH2.D2.Y, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC2.D2.Z, aDH2.D2.Z, global::Kerf.Engine.Foundation.Precision.Angular());

            Curve3D.ResD3 aDH3 = aHelix.EvalD3(aT);
            Curve3D.ResD3 aDC3;
            global::Kerf.Engine.Geometry.ElCLib.D3(aT, aCircle, out aDC3.Point, out aDC3.D1, out aDC3.D2, out aDC3.D3);
            Assert.Equal(aDC3.D3.X, aDH3.D3.X, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC3.D3.Y, aDH3.D3.Y, global::Kerf.Engine.Foundation.Precision.Angular());
            Assert.Equal(aDC3.D3.Z, aDH3.D3.Z, global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }

    [Fact]
    public void EvalDN_CyclesPeriod4()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        double aT = 1.0;

        // DN(t,4) XY-components should equal DN(t,0) XY = D0 XY contribution
        // More specifically: DN cycles with period 4 in XY
        Vector3D aD1 = aCurve.EvalDN(aT, 1);
        Vector3D aD5 = aCurve.EvalDN(aT, 5);
        Assert.Equal(aD5.X, aD1.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aD5.Y, aD1.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        // Z: D1 has Z component, D5 does not
        Assert.Equal(0.0, aD5.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(1.0, 2.0, 3.0));

        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        Geometry3D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        CircularHelixCurve? aCopyCurve = aCopy as CircularHelixCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(5.0, aCopyCurve.Radius(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(10.0, aCopyCurve.Pitch(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(3.0));
    }

    [Fact]
    public void Properties()
    {
        Frame3D anAx2 = new();
        CircularHelixCurve aCurve = new(anAx2, 5.0, 10.0);
        Assert.False(aCurve.IsClosed);
        Assert.False(aCurve.IsPeriodic);
        Assert.Equal(Continuity.CN, aCurve.Continuity);
        Assert.True(aCurve.IsCN(100));
    }

    // Kerf addition: every derivative against central differences of the one below, along the curve, for a
    // right- and a left-handed helix in a tilted coordinate system.
    [Theory]
    [InlineData(10.0)]
    [InlineData(-7.5)]
    public void Derivatives_MatchFiniteDifferences(double thePitch)
    {
        Frame3D anAx2 = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(1.0, 1.0, 1.0), new Direction3D(1.0, -1.0, 0.0));
        CircularHelixCurve aCurve = new(anAx2, 2.5, thePitch);
        for (double aT = -4.0; aT <= 4.0; aT += 0.61)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aT, 6);
        }
    }
}
