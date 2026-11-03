// Port of: src/ModelingData/TKG3d/GTests/GeomEval_TBezierSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static OcctNet.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
// The *_MatchesSphere tests compare with ElSLib's sphere evaluators (GeomEval_SurfaceTestHelpers), which are
// what Geom_SphericalSurface's EvalD0 to EvalD3 call (Geom_SphericalSurface is not ported yet).
public class GeomEval_TBezierSurfaceTest
{
    private const double THE_FD_TOL_D1 = 1e-5;
    private const double THE_FD_TOL_D2 = 1e-4;

    // Helper to create a T-Bezier surface representing a unit sphere patch.
    // Basis: {1, sin(u), cos(u)} x {1, sin(v), cos(v)} with alpha=1.
    // S(u,v) = cos(u)*cos(v)*X + sin(u)*cos(v)*Y + sin(v)*Z, domain [0, Pi]x[0, Pi].
    private static TBezierSurface createSpherePatch()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        // All poles default to origin (0,0,0)
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D(0.0, 0.0, 0.0));
            }
        }
        // P(1,2) = (0,0,1) corresponds to 1*sin(v) -> Z component
        aPoles.SetValue(1, 2, new Point3D(0.0, 0.0, 1.0));
        // P(2,3) = (0,1,0) corresponds to sin(u)*cos(v) -> Y component
        aPoles.SetValue(2, 3, new Point3D(0.0, 1.0, 0.0));
        // P(3,3) = (1,0,0) corresponds to cos(u)*cos(v) -> X component
        aPoles.SetValue(3, 3, new Point3D(1.0, 0.0, 0.0));
        return new TBezierSurface(aPoles, 1.0, 1.0);
    }

    // Helper to create a simple 3x3 (order 1 in both directions) T-Bezier surface
    private static TBezierSurface createSimpleSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D((double)(i - 1), (double)(j - 1), 0.0));
            }
        }
        return new TBezierSurface(aPoles, 1.0, 1.0);
    }

    [Fact]
    public void Construction_ValidParams()
    {
        TBezierSurface aSurf = createSimpleSurface();
        Assert.Equal(3, aSurf.UPoleCount);
        Assert.Equal(3, aSurf.VPoleCount);
        Assert.Equal(1, aSurf.OrderU());
        Assert.Equal(1, aSurf.OrderV());
        Assert.Equal(1.0, aSurf.AlphaU(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aSurf.AlphaV(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aSurf.IsRational);
    }

    [Fact]
    public void Construction_InvalidParams_Throws()
    {
        // Even number of poles in U (not odd)
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 3);
        for (int i = 1; i <= 2; ++i)
        {
            for (int j = 1; j <= 3; ++j)
            {
                aPoles.SetValue(i, j, new Point3D((double)(i), (double)(j), 0.0));
            }
        }
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TBezierSurface(aPoles, 1.0, 1.0));
    }

    [Fact]
    public void ParameterRange()
    {
        TBezierSurface aSurf = createSimpleSurface();
        double aU1, aU2, aV1, aV2;
        aSurf.Bounds(out aU1, out aU2, out aV1, out aV2);
        Assert.Equal(0.0, aU1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(M_PI, aU2, global::Kerf.Engine.Foundation.Precision.Confusion()); // Pi/alphaU = Pi/1
        Assert.Equal(0.0, aV1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(M_PI, aV2, global::Kerf.Engine.Foundation.Precision.Confusion()); // Pi/alphaV = Pi/1
    }

    [Fact]
    public void EvalD0_Corners()
    {
        TBezierSurface aSurf = createSimpleSurface();
        double aU1, aU2, aV1, aV2;
        aSurf.Bounds(out aU1, out aU2, out aV1, out aV2);

        // The corner evaluations should be well-defined points
        Point3D aP00 = aSurf.EvalD0(aU1, aV1);
        Point3D aP10 = aSurf.EvalD0(aU2, aV1);
        Point3D aP01 = aSurf.EvalD0(aU1, aV2);
        Point3D aP11 = aSurf.EvalD0(aU2, aV2);

        // Verify points are distinct (surface is not degenerate)
        Assert.True(aP00.Distance(aP10) > global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aP00.Distance(aP01) > global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aP00.Distance(aP11) > global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        TBezierSurface aSurf = createSimpleSurface();
        double aU = M_PI / 3.0, aV = M_PI / 4.0;

        Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
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
        TBezierSurface aSurf = createSimpleSurface();
        Assert.False(aSurf.IsUPeriodic);
        Assert.False(aSurf.IsVPeriodic);
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        TBezierSurface aSurf = createSimpleSurface();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReversedParameter(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReversedParameter(0.5));
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        TBezierSurface aSurf = createSimpleSurface();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UIso(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        TBezierSurface aSurf = createSimpleSurface();
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(1.0, 2.0, 3.0));

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        TBezierSurface aSurf = createSimpleSurface();
        Geometry3D aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        TBezierSurface? aCopySurf = aCopy as TBezierSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(3, aCopySurf.UPoleCount);
        Assert.Equal(3, aCopySurf.VPoleCount);
    }

    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        TBezierSurface aSurf = createSimpleSurface();
        double aU = M_PI / 4.0, aV = M_PI / 4.0;

        Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        // D2U: finite difference of D1U w.r.t. U
        Surface.ResD1 aD1Uplus = aSurf.EvalD1(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Surface.ResD1 aD1Uminus = aSurf.EvalD1(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Vector3D aFD_D2U = new((aD1Uplus.D1U.XYZ() - aD1Uminus.D1U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D2U.X(), aD2.D2U.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2U.Y(), aD2.D2U.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D2U.Z(), aD2.D2U.Z(), THE_FD_TOL_D2);

        // D2V: finite difference of D1V w.r.t. V
        Surface.ResD1 aD1Vplus = aSurf.EvalD1(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Surface.ResD1 aD1Vminus = aSurf.EvalD1(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());
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
        TBezierSurface aSurf = createSimpleSurface();
        double aU = M_PI / 3.0, aV = M_PI / 3.0;

        Surface.ResD3 aD3 = aSurf.EvalD3(aU, aV);

        // D3U: finite difference of D2U w.r.t. U
        Surface.ResD2 aD2Uplus = aSurf.EvalD2(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Surface.ResD2 aD2Uminus = aSurf.EvalD2(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Vector3D aFD_D3U = new((aD2Uplus.D2U.XYZ() - aD2Uminus.D2U.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        Assert.Equal(aFD_D3U.X(), aD3.D3U.X(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3U.Y(), aD3.D3U.Y(), THE_FD_TOL_D2);
        Assert.Equal(aFD_D3U.Z(), aD3.D3U.Z(), THE_FD_TOL_D2);

        // D3V: finite difference of D2V w.r.t. V
        Surface.ResD2 aD2Vplus = aSurf.EvalD2(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Surface.ResD2 aD2Vminus = aSurf.EvalD2(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());
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
        TBezierSurface aSurf = createSimpleSurface();
        double aU = M_PI / 4.0, aV = M_PI / 3.0;

        Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
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
    public void EvalD0_MatchesSphere()
    {
        TBezierSurface aTBez = createSpherePatch();
        CoordinateSystem aSphere = new();

        double[] aUParams =
            [0.1, M_PI / 6.0, M_PI / 4.0, M_PI / 3.0, M_PI / 2.0, 2.0 * M_PI / 3.0, 2.5];
        double[] aVParams = [0.1, M_PI / 6.0, M_PI / 4.0, M_PI / 3.0, 1.2];
        foreach (double aU in aUParams)
        {
            foreach (double aV in aVParams)
            {
                Point3D aPT = aTBez.EvalD0(aU, aV);
                Point3D aPS = SphereEvalD0(aSphere, 1.0, aU, aV);
                Assert.Equal(0.0, aPT.Distance(aPS), global::Kerf.Engine.Foundation.Precision.Angular());
            }
        }
    }

    [Fact]
    public void EvalD1_MatchesSphere()
    {
        TBezierSurface aTBez = createSpherePatch();
        CoordinateSystem aSphere = new();

        double[] aUParams = [M_PI / 6.0, M_PI / 3.0, M_PI / 2.0, 2.0 * M_PI / 3.0];
        double[] aVParams = [0.2, M_PI / 4.0, M_PI / 3.0, 1.0];
        foreach (double aU in aUParams)
        {
            foreach (double aV in aVParams)
            {
                Surface.ResD1 aDT = aTBez.EvalD1(aU, aV);
                Surface.ResD1 aDS = SphereEvalD1(aSphere, 1.0, aU, aV);

                Assert.Equal(0.0, aDT.Point.Distance(aDS.Point), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1U.X(), aDT.D1U.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1U.Y(), aDT.D1U.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1U.Z(), aDT.D1U.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1V.X(), aDT.D1V.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1V.Y(), aDT.D1V.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1V.Z(), aDT.D1V.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
            }
        }
    }

    [Fact]
    public void EvalD2_MatchesSphere()
    {
        TBezierSurface aTBez = createSpherePatch();
        CoordinateSystem aSphere = new();

        double[] aUParams = [M_PI / 6.0, M_PI / 3.0, M_PI / 2.0];
        double[] aVParams = [0.3, M_PI / 4.0, 1.0];
        foreach (double aU in aUParams)
        {
            foreach (double aV in aVParams)
            {
                Surface.ResD2 aDT = aTBez.EvalD2(aU, aV);
                Surface.ResD2 aDS = SphereEvalD2(aSphere, 1.0, aU, aV);

                Assert.Equal(0.0, aDT.Point.Distance(aDS.Point), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1U.X(), aDT.D1U.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1U.Y(), aDT.D1U.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1U.Z(), aDT.D1U.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1V.X(), aDT.D1V.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1V.Y(), aDT.D1V.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D1V.Z(), aDT.D1V.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2U.X(), aDT.D2U.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2U.Y(), aDT.D2U.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2U.Z(), aDT.D2U.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2V.X(), aDT.D2V.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2V.Y(), aDT.D2V.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2V.Z(), aDT.D2V.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2UV.X(), aDT.D2UV.X(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2UV.Y(), aDT.D2UV.Y(), global::Kerf.Engine.Foundation.Precision.Angular());
                Assert.Equal(aDS.D2UV.Z(), aDT.D2UV.Z(), global::Kerf.Engine.Foundation.Precision.Angular());
            }
        }
    }

    private static TBezierSurface createGeneralSurface(int theNbU, int theNbV, bool theRational)
    {
        NCollection_Array2<Point3D> aPoles = new(1, theNbU, 1, theNbV);
        NCollection_Array2<double> aWeights = new(1, theNbU, 1, theNbV);
        for (int i = 1; i <= theNbU; ++i)
        {
            for (int j = 1; j <= theNbV; ++j)
            {
                aPoles.SetValue(i, j, new Point3D(i + 0.3 * Math.Sin(j), j - 0.2 * i, Math.Cos(i * 0.7 + j * 1.1)));
                aWeights.SetValue(i, j, (i == 1 && j == 1) ? 4.0 : 0.5 + 0.1 * i + 0.05 * j);
            }
        }
        return theRational ? new TBezierSurface(aPoles, aWeights, 0.9, 1.3)
                           : new TBezierSurface(aPoles, 0.9, 1.3);
    }

    // OCCT.NET addition: partial derivatives against finite differences, non-rational up to order 4 and rational up
    // to order 3 (OCCT doesn't evaluate rational derivatives above 3).
    [Theory]
    [InlineData(3, 3)]
    [InlineData(5, 3)]
    [InlineData(3, 7)]
    public void Derivatives_MatchFiniteDifferences(int theNbU, int theNbV)
    {
        TBezierSurface aSurf = createGeneralSurface(theNbU, theNbV, false);
        TBezierSurface aRational = createGeneralSurface(theNbU, theNbV, true);
        for (double aU = 0.1; aU < M_PI / 0.9; aU += 0.7)
        {
            for (double aV = 0.1; aV < M_PI / 1.3; aV += 0.55)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV, 4);
                EvalTestUtils.CheckSurfaceDerivatives(aRational, aU, aV, 3);
            }
        }
        Assert.ThrowsAny<UndefinedDerivative3DException>(() => aRational.EvalDN(0.5, 0.5, 2, 2));
    }

    // OCCT.NET addition: Poles() and Weights() are row-major copies of the grids.
    [Fact]
    public void Poles_Weights_RowMajor()
    {
        TBezierSurface aSurf = createGeneralSurface(5, 3, true);
        Assert.Equal(15, aSurf.Poles().Count);
        Assert.Equal(new Point3D(2 + 0.3 * Math.Sin(3), 3 - 0.2 * 2, Math.Cos(2 * 0.7 + 3 * 1.1)), aSurf.Poles()[1 * 3 + 2]);
        Assert.Equal(0.5 + 0.1 * 2 + 0.05 * 3, aSurf.Weights()[1 * 3 + 2]);
        Assert.Empty(createGeneralSurface(3, 3, false).Weights());
    }

    // OCCT.NET addition: OCCT's IsUClosed/IsVClosed compare the first and last rows (columns) of poles, which in
    // the trigonometric basis are the constant and the last cosine terms, not the boundary iso-curves.
    [Fact]
    public void IsClosed_ComparesPoleRows_OcctIssue()
    {
        // Rows: constant, sin(u), cos(u). Constant row = cos row: S(0, v) = 2*P0(v), S(Pi, v) = 0, yet "closed".
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int j = 1; j <= 3; ++j)
        {
            aPoles.SetValue(1, j, new Point3D(1.0, j, 0.0));
            aPoles.SetValue(2, j, new Point3D(0.0, 0.0, 1.0));
            aPoles.SetValue(3, j, new Point3D(1.0, j, 0.0));
        }
        TBezierSurface aSurf = new(aPoles, 1.0, 1.0);
        Assert.True(aSurf.IsUClosed);
        Assert.True(aSurf.EvalD0(0.0, 0.5).Distance(aSurf.EvalD0(M_PI, 0.5)) > 1.0);
    }
}
