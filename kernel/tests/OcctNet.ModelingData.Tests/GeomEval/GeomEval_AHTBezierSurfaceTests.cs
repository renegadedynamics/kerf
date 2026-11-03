// Port of: src/ModelingData/TKG3d/GTests/GeomEval_AHTBezierSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_AHTBezierSurfaceTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;

    private static double vecDiff(in Vector3D theV1, in Vector3D theV2)
    {
        return (theV1 - theV2).Magnitude();
    }

    // Helper to create a pure polynomial AHT-Bezier surface.
    // Basis: {1, t, t^2} in each direction => 3x3 poles, algDegree=2, alpha=0, beta=0
    private static GeomEval_AHTBezierSurface createPolynomialSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D((double)(i - 1), (double)(j - 1), (double)((i - 1) * (j - 1)) * 0.1));
            }
        }
        return new GeomEval_AHTBezierSurface(aPoles, 2, 2, 0.0, 0.0, 0.0, 0.0);
    }

    private static GeomEval_AHTBezierSurface createPolynomialSurfaceRationalUniformWeight(double theWeight)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D((double)(i - 1), (double)(j - 1), (double)((i - 1) * (j - 1)) * 0.1));
                aWeights.SetValue(i, j, theWeight);
            }
        }
        return new GeomEval_AHTBezierSurface(aPoles, aWeights, 2, 2, 0.0, 0.0, 0.0, 0.0);
    }

    [Fact]
    public void Construction_ValidParams()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        Assert.Equal(3, aSurf.NbPolesU());
        Assert.Equal(3, aSurf.NbPolesV());
        Assert.Equal(2, aSurf.AlgDegreeU());
        Assert.Equal(2, aSurf.AlgDegreeV());
        Assert.Equal(0.0, aSurf.AlphaU(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aSurf.AlphaV(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aSurf.BetaU(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aSurf.BetaV(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ParameterRange()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        double aU1, aU2, aV1, aV2;
        aSurf.Bounds(out aU1, out aU2, out aV1, out aV2);
        Assert.Equal(0.0, aU1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aU2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aV1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aV2, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD0_Corners()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();

        Point3D aP00 = aSurf.EvalD0(0.0, 0.0);
        Point3D aP10 = aSurf.EvalD0(1.0, 0.0);
        Point3D aP01 = aSurf.EvalD0(0.0, 1.0);
        Point3D aP11 = aSurf.EvalD0(1.0, 1.0);

        // Verify corners are distinct (non-degenerate surface)
        Assert.True(aP00.Distance(aP10) > global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aP00.Distance(aP01) > global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aP00.Distance(aP11) > global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        double aU = 0.3, aV = 0.7;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
        Point3D aPu1 = aSurf.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Point3D aPu2 = aSurf.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Point3D aPv1 = aSurf.EvalD0(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D aPv2 = aSurf.EvalD0(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFDU = new((aPu1.XYZ() - aPu2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFDV = new((aPv1.XYZ() - aPv2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFDU.X(), aD1.D1U.X(), THE_FD_TOL_D1);
        Assert.Equal(aFDU.Y(), aD1.D1U.Y(), THE_FD_TOL_D1);
        Assert.Equal(aFDU.Z(), aD1.D1U.Z(), THE_FD_TOL_D1);
        Assert.Equal(aFDV.X(), aD1.D1V.X(), THE_FD_TOL_D1);
        Assert.Equal(aFDV.Y(), aD1.D1V.Y(), THE_FD_TOL_D1);
        Assert.Equal(aFDV.Z(), aD1.D1V.Z(), THE_FD_TOL_D1);
    }

    [Fact]
    public void Periodicity()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        Assert.False(aSurf.IsUPeriodic());
        Assert.False(aSurf.IsVPeriodic());
        Assert.False(aSurf.IsUClosed());
        Assert.False(aSurf.IsVClosed());
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReversedParameter(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReversedParameter(0.5));
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UIso(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(1.0, 2.0, 3.0));

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        Geom_Geometry aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        GeomEval_AHTBezierSurface? aCopySurf = aCopy as GeomEval_AHTBezierSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(3, aCopySurf.NbPolesU());
        Assert.Equal(3, aCopySurf.NbPolesV());
        Assert.Equal(2, aCopySurf.AlgDegreeU());
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        double aU = 0.3, aV = 0.4;

        Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        // D2U: finite difference of D1U w.r.t. U
        Geom_Surface.ResD1 aD1Uplus = aSurf.EvalD1(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Geom_Surface.ResD1 aD1Uminus = aSurf.EvalD1(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Vector3D aFD_D2U = new((aD1Uplus.D1U.XYZ() - aD1Uminus.D1U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D2U.X(), aD2.D2U.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2U.Y(), aD2.D2U.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2U.Z(), aD2.D2U.Z(), THE_FD_TOL_D2);

        // D2V: finite difference of D1V w.r.t. V
        Geom_Surface.ResD1 aD1Vplus = aSurf.EvalD1(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Geom_Surface.ResD1 aD1Vminus = aSurf.EvalD1(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector3D aFD_D2V = new((aD1Vplus.D1V.XYZ() - aD1Vminus.D1V.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D2V.X(), aD2.D2V.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2V.Y(), aD2.D2V.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2V.Z(), aD2.D2V.Z(), THE_FD_TOL_D2);

        // D2UV: finite difference of D1U w.r.t. V
        Vector3D aFD_D2UV = new((aD1Vplus.D1U.XYZ() - aD1Vminus.D1U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D2UV.X(), aD2.D2UV.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2UV.Y(), aD2.D2UV.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2UV.Z(), aD2.D2UV.Z(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        double aU = 0.4, aV = 0.3;

        Geom_Surface.ResD3 aD3 = aSurf.EvalD3(aU, aV);

        // D3U: finite difference of D2U w.r.t. U
        Geom_Surface.ResD2 aD2Uplus = aSurf.EvalD2(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Geom_Surface.ResD2 aD2Uminus = aSurf.EvalD2(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Vector3D aFD_D3U = new((aD2Uplus.D2U.XYZ() - aD2Uminus.D2U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D3U.X(), aD3.D3U.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3U.Y(), aD3.D3U.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3U.Z(), aD3.D3U.Z(), THE_FD_TOL_D2);

        // D3V: finite difference of D2V w.r.t. V
        Geom_Surface.ResD2 aD2Vplus = aSurf.EvalD2(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Geom_Surface.ResD2 aD2Vminus = aSurf.EvalD2(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());
        Vector3D aFD_D3V = new((aD2Vplus.D2V.XYZ() - aD2Vminus.D2V.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D3V.X(), aD3.D3V.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3V.Y(), aD3.D3V.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3V.Z(), aD3.D3V.Z(), THE_FD_TOL_D2);

        // D3UUV: finite difference of D2U w.r.t. V
        Vector3D aFD_D3UUV = new((aD2Vplus.D2U.XYZ() - aD2Vminus.D2U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D3UUV.X(), aD3.D3UUV.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3UUV.Y(), aD3.D3UUV.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3UUV.Z(), aD3.D3UUV.Z(), THE_FD_TOL_D2);

        // D3UVV: finite difference of D2V w.r.t. U
        Vector3D aFD_D3UVV = new((aD2Uplus.D2V.XYZ() - aD2Uminus.D2V.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D3UVV.X(), aD3.D3UVV.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3UVV.Y(), aD3.D3UVV.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3UVV.Z(), aD3.D3UVV.Z(), THE_FD_TOL_D2);
    }

    [Fact]
    public void EvalDN_ConsistentWithD1()
    {
        GeomEval_AHTBezierSurface aSurf = createPolynomialSurface();
        double aU = 0.5, aV = 0.5;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
        Vector3D aDN_U = aSurf.EvalDN(aU, aV, 1, 0);
        Vector3D aDN_V = aSurf.EvalDN(aU, aV, 0, 1);

        Assert.Equal(aDN_U.X(), aD1.D1U.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN_U.Y(), aD1.D1U.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN_U.Z(), aD1.D1U.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(aDN_V.X(), aD1.D1V.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN_V.Y(), aD1.D1V.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aDN_V.Z(), aD1.D1V.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void RationalUniformScale_Invariant()
    {
        GeomEval_AHTBezierSurface aRatW1 = createPolynomialSurfaceRationalUniformWeight(1.0);
        GeomEval_AHTBezierSurface aRatW5 = createPolynomialSurfaceRationalUniformWeight(5.0);
        double aU = 0.37;
        double aV = 0.62;

        Assert.Equal(0.0, aRatW1.EvalD0(aU, aV).Distance(aRatW5.EvalD0(aU, aV)), 1e-12);

        Geom_Surface.ResD1 aD1W1 = aRatW1.EvalD1(aU, aV);
        Geom_Surface.ResD1 aD1W5 = aRatW5.EvalD1(aU, aV);
        Assert.Equal(0.0, vecDiff(aD1W1.D1U, aD1W5.D1U), 1e-12);
        Assert.Equal(0.0, vecDiff(aD1W1.D1V, aD1W5.D1V), 1e-12);

        Geom_Surface.ResD2 aD2W1 = aRatW1.EvalD2(aU, aV);
        Geom_Surface.ResD2 aD2W5 = aRatW5.EvalD2(aU, aV);
        Assert.Equal(0.0, vecDiff(aD2W1.D2U, aD2W5.D2U), 1e-12);
        Assert.Equal(0.0, vecDiff(aD2W1.D2V, aD2W5.D2V), 1e-12);
        Assert.Equal(0.0, vecDiff(aD2W1.D2UV, aD2W5.D2UV), 1e-12);

        Geom_Surface.ResD3 aD3W1 = aRatW1.EvalD3(aU, aV);
        Geom_Surface.ResD3 aD3W5 = aRatW5.EvalD3(aU, aV);
        Assert.Equal(0.0, vecDiff(aD3W1.D3U, aD3W5.D3U), 1e-12);
        Assert.Equal(0.0, vecDiff(aD3W1.D3V, aD3W5.D3V), 1e-12);
        Assert.Equal(0.0, vecDiff(aD3W1.D3UUV, aD3W5.D3UUV), 1e-12);
        Assert.Equal(0.0, vecDiff(aD3W1.D3UVV, aD3W5.D3UVV), 1e-12);

        Assert.Equal(0.0, vecDiff(aRatW1.EvalDN(aU, aV, 2, 1), aRatW5.EvalDN(aU, aV, 2, 1)), 1e-12);
    }

    // OCCT.NET addition: partial derivatives up to order 5 against finite differences, for combinations of the basis
    // parts in U and V (the cubic x cubic polynomial path included), non-rational and rational, at interior points
    // and on the boundary U = 0 or V = 0 (where the polynomial basis has its own branch).
    [Theory]
    [InlineData(2, 0.0, 0.0, 2, 0.0, 0.0)]
    [InlineData(3, 0.0, 0.0, 3, 0.0, 0.0)]
    [InlineData(4, 0.0, 0.0, 1, 0.0, 0.0)]
    [InlineData(1, 1.3, 0.0, 0, 0.0, 2.1)]
    [InlineData(0, 0.8, 1.7, 2, 1.1, 0.0)]
    [InlineData(2, 0.9, 2.3, 1, 0.7, 1.4)]
    public void Derivatives_MatchFiniteDifferences(int theDegU, double theAlphaU, double theBetaU,
                                                   int theDegV, double theAlphaV, double theBetaV)
    {
        int aNbU = theDegU + 1 + (theAlphaU > 0.0 ? 2 : 0) + (theBetaU > 0.0 ? 2 : 0);
        int aNbV = theDegV + 1 + (theAlphaV > 0.0 ? 2 : 0) + (theBetaV > 0.0 ? 2 : 0);
        NCollection_Array2<Point3D> aPoles = new(1, aNbU, 1, aNbV);
        NCollection_Array2<double> aWeights = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                aPoles.SetValue(i, j, new Point3D(i + 0.3 * Math.Sin(j), j - 0.2 * i, Math.Cos(i * 0.7 + j * 1.1)));
                aWeights.SetValue(i, j, (i == 1 && j == 1) ? 4.0 : 0.5 + 0.1 * i + 0.05 * j);
            }
        }
        GeomEval_AHTBezierSurface aSurf = new(aPoles, theDegU, theDegV, theAlphaU, theAlphaV, theBetaU, theBetaV);
        GeomEval_AHTBezierSurface aRational =
            new(aPoles, aWeights, theDegU, theDegV, theAlphaU, theAlphaV, theBetaU, theBetaV);
        double[] aParams = [0.0, 0.15, 0.5, 0.85];
        foreach (double aU in aParams)
        {
            foreach (double aV in aParams)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV, 5);
                EvalTestUtils.CheckSurfaceDerivatives(aRational, aU, aV, 5);
            }
        }
    }
}
