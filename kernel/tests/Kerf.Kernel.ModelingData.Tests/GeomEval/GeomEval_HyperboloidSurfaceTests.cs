// Port of: src/ModelingData/TKG3d/GTests/GeomEval_HyperboloidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_HyperboloidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;
    private const double THE_FD_TOL_D3 = 1e-4;

    // Test construction with valid parameters (one-sheet default)
    [Fact]
    public void Construction_ValidParams_OneSheet()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.Equal(2.0, aSurf.R1(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aSurf.R2(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aSurf.Mode() == HyperboloidSurface.SheetMode.OneSheet);
    }

    // Test construction with two-sheet mode
    [Fact]
    public void Construction_ValidParams_TwoSheets()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0, HyperboloidSurface.SheetMode.TwoSheets);
        Assert.True(aSurf.Mode() == HyperboloidSurface.SheetMode.TwoSheets);
    }

    // Test construction throws for invalid radii
    [Fact]
    public void Construction_InvalidRadii_Throws()
    {
        CoordinateSystem anAx3 = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HyperboloidSurface(anAx3, 0.0, 1.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HyperboloidSurface(anAx3, 1.0, 0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HyperboloidSurface(anAx3, -1.0, 1.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HyperboloidSurface(anAx3, 1.0, -1.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HyperboloidSurface(anAx3, -2.0, -3.0));
    }

    [Fact]
    public void Setters_ZeroRadii_Throw()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aSurf.SetR1(0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aSurf.SetR2(0.0));
    }

    // One sheet at u=0, v=0: cosh(0)=1, sinh(0)=0 -> P = (R1, 0, 0)
    [Fact]
    public void EvalD0_OneSheet_Origin()
    {
        CoordinateSystem anAx3 = new();
        const double aR1 = 2.0, aR2 = 3.0;
        HyperboloidSurface aSurf = new(anAx3, aR1, aR2);
        Point3D aP = aSurf.EvalD0(0.0, 0.0);
        Assert.Equal(aR1, aP.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test EvalD0 at u=PI/2, v=0 for one-sheet: P = (0, R1, 0)
    [Fact]
    public void EvalD0_OneSheet_HalfPi()
    {
        CoordinateSystem anAx3 = new();
        const double aR1 = 2.0, aR2 = 3.0;
        HyperboloidSurface aSurf = new(anAx3, aR1, aR2);
        Point3D aP = aSurf.EvalD0(M_PI / 2.0, 0.0);
        Assert.Equal(0.0, aP.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aR1, aP.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Two sheets at u=0, v=0: sinh(0)=0, cosh(0)=1 -> P = (0, 0, R1)
    [Fact]
    public void EvalD0_TwoSheets_Origin()
    {
        CoordinateSystem anAx3 = new();
        const double aR1 = 2.0, aR2 = 3.0;
        HyperboloidSurface aSurf = new(anAx3, aR1, aR2, HyperboloidSurface.SheetMode.TwoSheets);
        Point3D aP = aSurf.EvalD0(0.0, 0.0);
        Assert.Equal(0.0, aP.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aR1, aP.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test derivative consistency: compare EvalD1 vs finite difference of EvalD0
    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        const double aU = 1.0, aV = 0.5;

        Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
        Point3D aPu1 = aSurf.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Point3D aPu2 = aSurf.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Point3D aPv1 = aSurf.EvalD0(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D aPv2 = aSurf.EvalD0(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFDU = new((aPu1.XYZ() - aPu2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFDV = new((aPv1.XYZ() - aPv2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        ExpectNear(aD1.D1U, aFDU, THE_FD_TOL);
        ExpectNear(aD1.D1V, aFDV, THE_FD_TOL);
    }

    // Test Bounds and periodicity
    [Fact]
    public void Bounds_Periodicity()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 1.0, 1.0);
        aSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.0, aU1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(2.0 * M_PI, aU2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aSurf.IsUPeriodic);
        Assert.True(aSurf.IsUClosed);
        Assert.False(aSurf.IsVPeriodic);
        Assert.False(aSurf.IsVClosed);
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UIso(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReversedParameter(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReversedParameter(0.5));
    }

    // Test implicit equation at evaluated points for one-sheet
    [Fact]
    public void Coefficients_OneSheet_SatisfiedAtEvalPoints()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        aSurf.Coefficients(out double aA1, out double aA2, out double aA3, out double aB1, out double aB2,
                           out double aB3, out double aC1, out double aC2, out double aC3, out double aD);

        for (double u = 0.0; u < 6.0; u += 1.0)
        {
            for (double v = -1.0; v <= 1.0; v += 0.5)
            {
                Point3D aP = aSurf.EvalD0(u, v);
                double aVal = Quadric(aP, aA1, aA2, aA3, aB1, aB2, aB3, aC1, aC2, aC3, aD);
                Assert.Equal(0.0, aVal, global::Kerf.Engine.Foundation.Precision.Intersection());
            }
        }
    }

    // Test implicit equation at evaluated points for two-sheet
    [Fact]
    public void Coefficients_TwoSheets_SatisfiedAtEvalPoints()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0, HyperboloidSurface.SheetMode.TwoSheets);
        aSurf.Coefficients(out double aA1, out double aA2, out double aA3, out double aB1, out double aB2,
                           out double aB3, out double aC1, out double aC2, out double aC3, out double aD);

        for (double u = 0.0; u < 6.0; u += 1.0)
        {
            for (double v = -1.0; v <= 1.0; v += 0.5)
            {
                Point3D aP = aSurf.EvalD0(u, v);
                double aVal = Quadric(aP, aA1, aA2, aA3, aB1, aB2, aB3, aC1, aC2, aC3, aD);
                Assert.Equal(0.0, aVal, global::Kerf.Engine.Foundation.Precision.Intersection());
            }
        }
    }

    // Test Transform/Transformed are not implemented.
    [Fact]
    public void Transform_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(1.0, 2.0, 3.0));

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transformed(aTrsf));
    }

    // Test Copy produces independent identical object
    [Fact]
    public void Copy_Independent()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Geometry3D aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        HyperboloidSurface? aCopySurf = aCopy as HyperboloidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(2.0, aCopySurf.R1(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCopySurf.R2(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test D2 consistency with finite differences of D1
    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        CheckD2AgainstD1(aSurf, 0.7, 0.5);
    }

    // Test D3 consistency with finite differences of D2
    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        const double aU = 1.0, aV = 0.3;

        Surface.ResD3 aD3 = aSurf.EvalD3(aU, aV);

        Surface.ResD2 aD2Uf = aSurf.EvalD2(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Surface.ResD2 aD2Ub = aSurf.EvalD2(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Surface.ResD2 aD2Vf = aSurf.EvalD2(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Surface.ResD2 aD2Vb = aSurf.EvalD2(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFD_D3U = new((aD2Uf.D2U.XYZ() - aD2Ub.D2U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFD_D3V = new((aD2Vf.D2V.XYZ() - aD2Vb.D2V.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFD_D3UUV = new((aD2Vf.D2U.XYZ() - aD2Vb.D2U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFD_D3UVV = new((aD2Uf.D2V.XYZ() - aD2Ub.D2V.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        ExpectNear(aD3.D3U, aFD_D3U, THE_FD_TOL_D3);
        ExpectNear(aD3.D3V, aFD_D3V, THE_FD_TOL_D3);
        ExpectNear(aD3.D3UUV, aFD_D3UUV, THE_FD_TOL_D3);
        ExpectNear(aD3.D3UVV, aFD_D3UVV, THE_FD_TOL_D3);
    }

    // Test D2 consistency with finite differences of D1 for TwoSheets mode
    [Fact]
    public void EvalD2_TwoSheets_ConsistentWithD1()
    {
        CoordinateSystem anAx3 = new();
        HyperboloidSurface aSurf = new(anAx3, 2.0, 3.0, HyperboloidSurface.SheetMode.TwoSheets);
        CheckD2AgainstD1(aSurf, 0.7, 0.5);
    }

    // The body shared by EvalD2_ConsistentWithD1 and EvalD2_TwoSheets_ConsistentWithD1.
    private static void CheckD2AgainstD1(HyperboloidSurface aSurf, double aU, double aV)
    {
        Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        Surface.ResD1 aD1Uf = aSurf.EvalD1(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Surface.ResD1 aD1Ub = aSurf.EvalD1(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Surface.ResD1 aD1Vf = aSurf.EvalD1(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Surface.ResD1 aD1Vb = aSurf.EvalD1(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFD_D2U = new((aD1Uf.D1U.XYZ() - aD1Ub.D1U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFD_D2V = new((aD1Vf.D1V.XYZ() - aD1Vb.D1V.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFD_D2UV = new((aD1Vf.D1U.XYZ() - aD1Vb.D1U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        ExpectNear(aD2.D2U, aFD_D2U, THE_FD_TOL);
        ExpectNear(aD2.D2V, aFD_D2V, THE_FD_TOL);
        ExpectNear(aD2.D2UV, aFD_D2UV, THE_FD_TOL);
    }

    // OCCT.NET addition: all partial derivatives up to order 4 against finite differences, both sheet modes,
    // off the standard frame.
    [Theory]
    [InlineData(HyperboloidSurface.SheetMode.OneSheet)]
    [InlineData(HyperboloidSurface.SheetMode.TwoSheets)]
    public void Derivatives_MatchFiniteDifferences(HyperboloidSurface.SheetMode theMode)
    {
        CoordinateSystem anAx3 = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(1.0, 1.0, 1.0), new Direction3D(1.0, -1.0, 0.0));
        HyperboloidSurface aSurf = new(anAx3, 2.0, 1.5, theMode);
        for (double aU = 0.1; aU < 2.0 * M_PI; aU += 0.7)
        {
            for (double aV = -1.5; aV <= 1.5; aV += 0.6)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV);
            }
        }
    }

    // OCCT.NET addition: the setters' With twins and freezing.
    [Fact]
    public void Setters_WithTwins_Freeze()
    {
        HyperboloidSurface aSurf = new(new CoordinateSystem(), 2.0, 3.0);
        HyperboloidSurface aCopy =
            aSurf.WithR1(4.0).WithR2(5.0).WithMode(HyperboloidSurface.SheetMode.TwoSheets);
        Assert.Equal(2.0, aSurf.R1());
        Assert.Equal(HyperboloidSurface.SheetMode.OneSheet, aSurf.Mode());
        Assert.Equal(4.0, aCopy.R1());
        Assert.Equal(5.0, aCopy.R2());
        Assert.Equal(HyperboloidSurface.SheetMode.TwoSheets, aCopy.Mode());

        aSurf.Freeze();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aSurf.SetMode(HyperboloidSurface.SheetMode.TwoSheets));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aSurf.SetR2(1.0));
    }
}
