// Port of: src/ModelingData/TKG3d/GTests/GeomEval_CircularHelicoidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
// TODO(1D): ComparisonWithHelix_ConstantV once GeomEval_CircularHelixCurve is merged into this branch.
public class GeomEval_CircularHelicoidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;

    [Fact]
    public void Construction_ValidParams()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Assert.Equal(10.0, aSurf.Pitch(), Precision.Confusion());
    }

    [Fact]
    public void Construction_ZeroPitch_Throws()
    {
        gp_Ax3 anAx3 = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_CircularHelicoidSurface(anAx3, 0.0));
    }

    [Fact]
    public void Construction_NegativePitch_NoThrow()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, -5.0);
        Assert.Equal(-5.0, aSurf.Pitch());
    }

    [Fact]
    public void EvalD0_AtKnownPoints()
    {
        gp_Ax3 anAx3 = new();
        const double aP = 10.0;
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, aP);

        // S(0, 1) = (1, 0, 0)
        gp_Pnt aP1 = aSurf.EvalD0(0.0, 1.0);
        Assert.Equal(1.0, aP1.X(), Precision.Confusion());
        Assert.Equal(0.0, aP1.Y(), Precision.Confusion());
        Assert.Equal(0.0, aP1.Z(), Precision.Confusion());

        // S(Pi/2, 1) = (0, 1, P/4)
        gp_Pnt aP2 = aSurf.EvalD0(M_PI / 2.0, 1.0);
        Assert.Equal(0.0, aP2.X(), Precision.Confusion());
        Assert.Equal(1.0, aP2.Y(), Precision.Confusion());
        Assert.Equal(aP / 4.0, aP2.Z(), Precision.Confusion());

        // S(u, 0) = (0, 0, P*u/(2*Pi)) -- on the axis
        gp_Pnt aP3 = aSurf.EvalD0(1.0, 0.0);
        Assert.Equal(0.0, aP3.X(), Precision.Confusion());
        Assert.Equal(0.0, aP3.Y(), Precision.Confusion());
        Assert.Equal(aP / (2.0 * M_PI), aP3.Z(), Precision.Confusion());
    }

    [Fact]
    public void D2V_IsZero_RuledSurface()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);

        double[][] aParams = [[0.0, 1.0], [1.0, 2.0], [M_PI, -1.0]];
        foreach (double[] aUV in aParams)
        {
            Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aUV[0], aUV[1]);
            Assert.Equal(0.0, aD2.D2V.Magnitude(), Precision.Confusion());
        }
    }

    [Fact]
    public void D1V_UnitRadial()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);

        // |dS/dv| = |cos(u)*XDir + sin(u)*YDir| = 1
        double[] aParams = [0.0, 1.0, M_PI / 2.0, M_PI, 3.5];
        foreach (double aU in aParams)
        {
            Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, 2.0);
            Assert.Equal(1.0, aD1.D1V.Magnitude(), Precision.Confusion());
        }
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        const double aU = 1.0, aV = 2.0;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
        gp_Pnt aPu1 = aSurf.EvalD0(aU + Precision.Confusion(), aV);
        gp_Pnt aPu2 = aSurf.EvalD0(aU - Precision.Confusion(), aV);
        gp_Pnt aPv1 = aSurf.EvalD0(aU, aV + Precision.Confusion());
        gp_Pnt aPv2 = aSurf.EvalD0(aU, aV - Precision.Confusion());

        gp_Vec aFDU = new((aPu1.XYZ() - aPu2.XYZ()) / (2.0 * Precision.Confusion()));
        gp_Vec aFDV = new((aPv1.XYZ() - aPv2.XYZ()) / (2.0 * Precision.Confusion()));

        ExpectNear(aD1.D1U, aFDU, THE_FD_TOL);
        ExpectNear(aD1.D1V, aFDV, THE_FD_TOL);
    }

    [Fact]
    public void Bounds_Infinite()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        aSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.True(Precision.IsInfinite(aU1));
        Assert.True(Precision.IsInfinite(aU2));
        Assert.True(Precision.IsInfinite(aV1));
        Assert.True(Precision.IsInfinite(aV2));
    }

    [Fact]
    public void Properties()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Assert.False(aSurf.IsUClosed());
        Assert.False(aSurf.IsVClosed());
        Assert.False(aSurf.IsUPeriodic());
        Assert.False(aSurf.IsVPeriodic());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UIso(0.0));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VIso(0.0));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UReverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VReverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UReversedParameter(0.5));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VReversedParameter(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(1.0, 2.0, 3.0));

        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.Transform(aTrsf));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Geom_Geometry aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        GeomEval_CircularHelicoidSurface? aCopySurf = aCopy as GeomEval_CircularHelicoidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(10.0, aCopySurf.Pitch(), Precision.Confusion());
    }

    // OCCT.NET addition: all partial derivatives up to order 4 against finite differences, off the standard frame.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        gp_Ax3 anAx3 = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(1.0, 1.0, 1.0), new gp_Dir(1.0, -1.0, 0.0));
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, -3.5);
        for (double aU = -4.0; aU <= 4.0; aU += 0.7)
        {
            for (double aV = -2.0; aV <= 2.0; aV += 0.7)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV);
            }
        }
    }

    // OCCT.NET addition: the setter's With twin and freezing.
    [Fact]
    public void Setters_WithTwins_Freeze()
    {
        GeomEval_CircularHelicoidSurface aSurf = new(new gp_Ax3(), 10.0);
        GeomEval_CircularHelicoidSurface aCopy = aSurf.WithPitch(-2.0);
        Assert.Equal(10.0, aSurf.Pitch());
        Assert.Equal(-2.0, aCopy.Pitch());
        Assert.ThrowsAny<Standard_ConstructionError>(() => aSurf.WithPitch(0.0));

        aSurf.Freeze();
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.SetPitch(1.0));
    }
}
