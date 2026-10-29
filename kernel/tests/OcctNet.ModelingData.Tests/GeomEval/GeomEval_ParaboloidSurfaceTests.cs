// Port of: src/ModelingData/TKG3d/GTests/GeomEval_ParaboloidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_ParaboloidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;
    private const double THE_FD_TOL_D3 = 1e-4;

    // Test construction with valid parameters
    [Fact]
    public void Construction_ValidParams()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        Assert.Equal(1.0, aSurf.Focal(), Precision.Confusion());
    }

    // Test construction throws for invalid focal
    [Fact]
    public void Construction_InvalidFocal_Throws()
    {
        gp_Ax3 anAx3 = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_ParaboloidSurface(anAx3, -1.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_ParaboloidSurface(anAx3, 0.0));
    }

    // Test EvalD0 at known point: u=0, v=0 should give origin
    [Fact]
    public void EvalD0_Origin()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        gp_Pnt aP = aSurf.EvalD0(0.0, 0.0);
        Assert.Equal(0.0, aP.X(), Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), Precision.Confusion());
        Assert.Equal(0.0, aP.Z(), Precision.Confusion());
    }

    // Test EvalD0 at known point: u=0, v=2, F=1 -> P=(2,0,1)
    [Fact]
    public void EvalD0_KnownPoint()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        gp_Pnt aP = aSurf.EvalD0(0.0, 2.0);
        Assert.Equal(2.0, aP.X(), Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), Precision.Confusion());
        Assert.Equal(1.0, aP.Z(), Precision.Confusion()); // v^2/(4*F) = 4/4 = 1
    }

    // Test derivative consistency: compare EvalD1 vs finite difference of EvalD0
    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 2.0);
        const double aU = 1.0, aV = 1.5;

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

    // Test Bounds and periodicity
    [Fact]
    public void Bounds_Periodicity()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        aSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.0, aU1, Precision.Confusion());
        Assert.Equal(2.0 * M_PI, aU2, Precision.Confusion());
        Assert.True(aSurf.IsUPeriodic());
        Assert.True(aSurf.IsUClosed());
        Assert.False(aSurf.IsVPeriodic());
        Assert.False(aSurf.IsVClosed());
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UIso(0.5));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UReverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VReverse());
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UReversedParameter(0.5));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VReversedParameter(0.5));
    }

    // Test implicit equation at evaluated points
    [Fact]
    public void Coefficients_SatisfiedAtEvalPoints()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.5);
        aSurf.Coefficients(out double aA1, out double aA2, out double aA3, out double aB1, out double aB2,
                           out double aB3, out double aC1, out double aC2, out double aC3, out double aD);

        for (double u = 0.0; u < 6.0; u += 1.0)
        {
            for (double v = -2.0; v <= 2.0; v += 1.0)
            {
                gp_Pnt aP = aSurf.EvalD0(u, v);
                double aVal = Quadric(aP, aA1, aA2, aA3, aB1, aB2, aB3, aC1, aC2, aC3, aD);
                Assert.Equal(0.0, aVal, Precision.Intersection());
            }
        }
    }

    // Test Transform/Transformed are not implemented.
    [Fact]
    public void Transform_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 1.0);
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(1.0, 2.0, 3.0));

        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.Transform(aTrsf));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.Transformed(aTrsf));
    }

    // Test Copy produces independent identical object
    [Fact]
    public void Copy_Independent()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 2.0);
        Geom_Geometry aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        GeomEval_ParaboloidSurface? aCopySurf = aCopy as GeomEval_ParaboloidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(2.0, aCopySurf.Focal(), Precision.Confusion());
    }

    // Test D2 consistency with finite differences of D1
    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 2.0);
        const double aU = 1.0, aV = 1.5;

        Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        Geom_Surface.ResD1 aD1Uf = aSurf.EvalD1(aU + Precision.Confusion(), aV);
        Geom_Surface.ResD1 aD1Ub = aSurf.EvalD1(aU - Precision.Confusion(), aV);
        Geom_Surface.ResD1 aD1Vf = aSurf.EvalD1(aU, aV + Precision.Confusion());
        Geom_Surface.ResD1 aD1Vb = aSurf.EvalD1(aU, aV - Precision.Confusion());

        gp_Vec aFD_D2U = new((aD1Uf.D1U.XYZ() - aD1Ub.D1U.XYZ()) / (2.0 * Precision.Confusion()));
        gp_Vec aFD_D2V = new((aD1Vf.D1V.XYZ() - aD1Vb.D1V.XYZ()) / (2.0 * Precision.Confusion()));
        gp_Vec aFD_D2UV = new((aD1Vf.D1U.XYZ() - aD1Vb.D1U.XYZ()) / (2.0 * Precision.Confusion()));

        ExpectNear(aD2.D2U, aFD_D2U, THE_FD_TOL);
        ExpectNear(aD2.D2V, aFD_D2V, THE_FD_TOL);
        ExpectNear(aD2.D2UV, aFD_D2UV, THE_FD_TOL);
    }

    // Test D3 consistency with finite differences of D2
    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 2.0);
        const double aU = 0.7, aV = 1.2;

        Geom_Surface.ResD3 aD3 = aSurf.EvalD3(aU, aV);

        Geom_Surface.ResD2 aD2Uf = aSurf.EvalD2(aU + Precision.Confusion(), aV);
        Geom_Surface.ResD2 aD2Ub = aSurf.EvalD2(aU - Precision.Confusion(), aV);
        Geom_Surface.ResD2 aD2Vf = aSurf.EvalD2(aU, aV + Precision.Confusion());
        Geom_Surface.ResD2 aD2Vb = aSurf.EvalD2(aU, aV - Precision.Confusion());

        gp_Vec aFD_D3U = new((aD2Uf.D2U.XYZ() - aD2Ub.D2U.XYZ()) / (2.0 * Precision.Confusion()));
        gp_Vec aFD_D3V = new((aD2Vf.D2V.XYZ() - aD2Vb.D2V.XYZ()) / (2.0 * Precision.Confusion()));
        gp_Vec aFD_D3UUV = new((aD2Vf.D2U.XYZ() - aD2Vb.D2U.XYZ()) / (2.0 * Precision.Confusion()));
        gp_Vec aFD_D3UVV = new((aD2Uf.D2V.XYZ() - aD2Ub.D2V.XYZ()) / (2.0 * Precision.Confusion()));

        ExpectNear(aD3.D3U, aFD_D3U, THE_FD_TOL_D3);
        ExpectNear(aD3.D3V, aFD_D3V, THE_FD_TOL_D3);
        ExpectNear(aD3.D3UUV, aFD_D3UUV, THE_FD_TOL_D3);
        ExpectNear(aD3.D3UVV, aFD_D3UVV, THE_FD_TOL_D3);
    }

    // P(u,v) = v*cos(u)*X + v*sin(u)*Y + v^2/(4F)*Z
    // D2UU = -v*cos(u)*X - v*sin(u)*Y => magnitude = v
    [Fact]
    public void EvalD2_KnownAnalytical()
    {
        gp_Ax3 anAx3 = new();
        const double aFocal = 2.0;
        GeomEval_ParaboloidSurface aSurf = new(anAx3, aFocal);
        const double aU = M_PI / 4.0, aV = 2.0;

        Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        // D2UU = (-v*cos(u), -v*sin(u), 0)
        double aExpX = -aV * Math.Cos(aU);
        double aExpY = -aV * Math.Sin(aU);
        double aExpZ = 0.0;

        Assert.Equal(aExpX, aD2.D2U.X(), Precision.Angular());
        Assert.Equal(aExpY, aD2.D2U.Y(), Precision.Angular());
        Assert.Equal(aExpZ, aD2.D2U.Z(), Precision.Angular());
        Assert.Equal(aV, aD2.D2U.Magnitude(), Precision.Angular());
    }

    // Test EvalDN consistency with D1 components
    [Fact]
    public void EvalDN_Consistency()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 2.0);
        const double aU = 1.0, aV = 1.0;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
        gp_Vec aDN10 = aSurf.EvalDN(aU, aV, 1, 0);
        gp_Vec aDN01 = aSurf.EvalDN(aU, aV, 0, 1);

        ExpectNear(aDN10, aD1.D1U, Precision.Confusion());
        ExpectNear(aDN01, aD1.D1V, Precision.Confusion());
    }

    // OCCT.NET addition: all partial derivatives up to order 4 against finite differences, off the standard frame.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        gp_Ax3 anAx3 = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(1.0, 1.0, 1.0), new gp_Dir(1.0, -1.0, 0.0));
        GeomEval_ParaboloidSurface aSurf = new(anAx3, 0.75);
        for (double aU = 0.1; aU < 2.0 * M_PI; aU += 0.7)
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
        GeomEval_ParaboloidSurface aSurf = new(new gp_Ax3(), 2.0);
        GeomEval_ParaboloidSurface aCopy = aSurf.WithFocal(3.0);
        Assert.Equal(2.0, aSurf.Focal());
        Assert.Equal(3.0, aCopy.Focal());
        Assert.ThrowsAny<Standard_ConstructionError>(() => aSurf.WithFocal(0.0));

        aSurf.Freeze();
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.SetFocal(1.0));
    }
}
