// Port of: src/ModelingData/TKG3d/GTests/GeomEval_SineWaveCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_SineWaveCurveTest
{
    private const double THE_FD_TOL = 1e-5;

    [Fact]
    public void Construction_ValidParams()
    {
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 2.0, 3.0, 0.0);
        Assert.Equal(2.0, aCurve.Amplitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCurve.Omega(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCurve.Phase(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        Frame3D anAx2 = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new SineWaveCurve(anAx2, 0.0, 1.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new SineWaveCurve(anAx2, -1.0, 1.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new SineWaveCurve(anAx2, 1.0, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new SineWaveCurve(anAx2, 1.0, -1.0));
    }

    [Fact]
    public void EvalD0_AtKnownPoints()
    {
        Frame3D anAx2 = new();
        const double aA = 2.0, aOm = 3.0;
        SineWaveCurve aCurve = new(anAx2, aA, aOm, 0.0);

        // t=0: C(0) = (0, 0, 0) since sin(0)=0
        Point3D aP0 = aCurve.EvalD0(0.0);
        Assert.Equal(0.0, aP0.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP0.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP0.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // t = Pi/(2*omega): C = (Pi/(2*omega), A, 0) since sin(Pi/2)=1
        double aT1 = M_PI / (2.0 * aOm);
        Point3D aP1 = aCurve.EvalD0(aT1);
        Assert.Equal(aT1, aP1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aA, aP1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP1.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void IsPeriodic()
    {
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 2.0, 3.0, 0.0);
        Assert.False(aCurve.IsPeriodic);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => aCurve.Period());
        Assert.False(aCurve.IsClosed);
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 2.0, 3.0, 0.0);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Reverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.ReversedParameter(0.5));
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 2.0, 3.0, 0.5);
        const double aT = 1.0;

        Curve3D.ResD1 aD1 = aCurve.EvalD1(aT);
        Point3D aP1 = aCurve.EvalD0(aT + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D aP2 = aCurve.EvalD0(aT - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector3D aFD = new((aP1.XYZ() - aP2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD.X(), aD1.D1.X(), THE_FD_TOL);
        Assert.Equal(aFD.Y(), aD1.D1.Y(), THE_FD_TOL);
        Assert.Equal(aFD.Z(), aD1.D1.Z(), THE_FD_TOL);
    }

    [Fact]
    public void EvalD2_Analytical()
    {
        Frame3D anAx2 = new();
        const double aA = 2.0, aOm = 3.0, aPhi = 0.5;
        SineWaveCurve aCurve = new(anAx2, aA, aOm, aPhi);
        const double aT = 1.0;

        Curve3D.ResD2 aD2 = aCurve.EvalD2(aT);

        // D2 = -A*omega^2*sin(omega*t+phi)*YDir = (0, -A*om^2*sin(...), 0)
        double aExpY = -aA * aOm * aOm * Math.Sin(aOm * aT + aPhi);
        Assert.Equal(0.0, aD2.D2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aExpY, aD2.D2.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD2.D2.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ComparisonWithLine_ZeroAmplitude_Like()
    {
        // When amplitude approaches zero, D0 approaches a line.
        // We test D2 and D3 are near zero for very small amplitude.
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 1e-10, 1.0, 0.0);
        const double aT = 1.0;

        Curve3D.ResD3 aD3 = aCurve.EvalD3(aT);
        Assert.Equal(0.0, aD3.D2.Magnitude(), 1e-5);
        Assert.Equal(0.0, aD3.D3.Magnitude(), 1e-5);
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 2.0, 3.0, 0.5);
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(1.0, 2.0, 3.0));

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aCurve.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        Frame3D anAx2 = new();
        SineWaveCurve aCurve = new(anAx2, 2.0, 3.0, 0.5);
        Geometry3D aCopy = aCurve.Copy();
        Assert.NotNull(aCopy);
        SineWaveCurve? aCopyCurve = aCopy as SineWaveCurve;
        Assert.NotNull(aCopyCurve);
        Assert.Equal(2.0, aCopyCurve.Amplitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCopyCurve.Omega(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.5, aCopyCurve.Phase(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCCT.NET addition: every derivative against central differences of the one below, along the curve.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        Frame3D anAx2 = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(1.0, 1.0, 1.0), new Direction3D(1.0, -1.0, 0.0));
        SineWaveCurve aCurve = new(anAx2, 1.5, 2.5, 0.3);
        for (double aT = -2.0; aT <= 2.0; aT += 0.37)
        {
            EvalTestUtils.CheckCurveDerivatives(aCurve, aT);
        }
    }
}
