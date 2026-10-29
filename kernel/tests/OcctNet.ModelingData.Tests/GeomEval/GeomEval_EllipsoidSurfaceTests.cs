// Port of: src/ModelingData/TKG3d/GTests/GeomEval_EllipsoidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
// The *_MatchesSphere tests compare with ElSLib's sphere evaluators, which are what Geom_SphericalSurface's
// EvalD0 to EvalD3 call (Geom_SphericalSurface is not ported yet).
public class GeomEval_EllipsoidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;

    private static gp_Pnt SphereEvalD0(in gp_Ax3 thePos, double theR, double theU, double theV)
    {
        ElSLib.SphereD0(theU, theV, thePos, theR, out gp_Pnt aP);
        return aP;
    }

    private static Geom_Surface.ResD1 SphereEvalD1(in gp_Ax3 thePos, double theR, double theU, double theV)
    {
        Geom_Surface.ResD1 aR;
        ElSLib.SphereD1(theU, theV, thePos, theR, out aR.Point, out aR.D1U, out aR.D1V);
        return aR;
    }

    private static Geom_Surface.ResD2 SphereEvalD2(in gp_Ax3 thePos, double theR, double theU, double theV)
    {
        Geom_Surface.ResD2 aR;
        ElSLib.SphereD2(theU, theV, thePos, theR, out aR.Point, out aR.D1U, out aR.D1V,
                        out aR.D2U, out aR.D2V, out aR.D2UV);
        return aR;
    }

    private static Geom_Surface.ResD3 SphereEvalD3(in gp_Ax3 thePos, double theR, double theU, double theV)
    {
        Geom_Surface.ResD3 aR;
        ElSLib.SphereD3(theU, theV, thePos, theR, out aR.Point, out aR.D1U, out aR.D1V,
                        out aR.D2U, out aR.D2V, out aR.D2UV, out aR.D3U, out aR.D3V, out aR.D3UUV, out aR.D3UVV);
        return aR;
    }

    // Test construction with valid parameters
    [Fact]
    public void Construction_ValidParams()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        Assert.Equal(3.0, aSurf.SemiAxisA(), Precision.Confusion());
        Assert.Equal(2.0, aSurf.SemiAxisB(), Precision.Confusion());
        Assert.Equal(1.0, aSurf.SemiAxisC(), Precision.Confusion());
    }

    // Test construction throws for invalid semi-axes
    [Fact]
    public void Construction_InvalidAxes_Throws()
    {
        gp_Ax3 anAx3 = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_EllipsoidSurface(anAx3, 0.0, 1.0, 1.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_EllipsoidSurface(anAx3, 1.0, -1.0, 1.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_EllipsoidSurface(anAx3, 1.0, 1.0, 0.0));
    }

    // Test EvalD0 at u=0, v=0: P = (A, 0, 0)
    [Fact]
    public void EvalD0_XAxis()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 3.0, aB = 2.0, aC = 1.0;
        GeomEval_EllipsoidSurface aSurf = new(anAx3, aA, aB, aC);
        gp_Pnt aP = aSurf.EvalD0(0.0, 0.0);
        Assert.Equal(aA, aP.X(), Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), Precision.Confusion());
        Assert.Equal(0.0, aP.Z(), Precision.Confusion());
    }

    // Test EvalD0 at u=PI/2, v=0: P = (0, B, 0)
    [Fact]
    public void EvalD0_YAxis()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 3.0, aB = 2.0, aC = 1.0;
        GeomEval_EllipsoidSurface aSurf = new(anAx3, aA, aB, aC);
        gp_Pnt aP = aSurf.EvalD0(M_PI / 2.0, 0.0);
        Assert.Equal(0.0, aP.X(), Precision.Confusion());
        Assert.Equal(aB, aP.Y(), Precision.Confusion());
        Assert.Equal(0.0, aP.Z(), Precision.Confusion());
    }

    // Test EvalD0 at u=0, v=PI/2: P = (0, 0, C)
    [Fact]
    public void EvalD0_ZAxis()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 3.0, aB = 2.0, aC = 1.0;
        GeomEval_EllipsoidSurface aSurf = new(anAx3, aA, aB, aC);
        gp_Pnt aP = aSurf.EvalD0(0.0, M_PI / 2.0);
        Assert.Equal(0.0, aP.X(), Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), Precision.Confusion());
        Assert.Equal(aC, aP.Z(), Precision.Confusion());
    }

    // Test derivative consistency: compare EvalD1 vs finite difference of EvalD0
    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        const double aU = 1.0, aV = 0.5;

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
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 1.0, 1.0, 1.0);
        aSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal(0.0, aU1, Precision.Confusion());
        Assert.Equal(2.0 * M_PI, aU2, Precision.Confusion());
        Assert.Equal(-M_PI / 2.0, aV1, Precision.Confusion());
        Assert.Equal(M_PI / 2.0, aV2, Precision.Confusion());
        Assert.True(aSurf.IsUPeriodic());
        Assert.True(aSurf.IsUClosed());
        Assert.False(aSurf.IsVPeriodic());
        Assert.False(aSurf.IsVClosed());
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UIso(0.5));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
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
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.5);
        aSurf.Coefficients(out double aA1, out double aA2, out double aA3, out double aB1, out double aB2,
                           out double aB3, out double aC1, out double aC2, out double aC3, out double aD);

        for (double u = 0.0; u < 6.0; u += 1.0)
        {
            for (double v = -1.0; v <= 1.0; v += 0.5)
            {
                gp_Pnt aP = aSurf.EvalD0(u, v);
                double aX = aP.X(), aY = aP.Y(), aZ = aP.Z();
                double aVal = aA1 * aX * aX + aA2 * aY * aY + aA3 * aZ * aZ
                              + 2.0 * (aB1 * aX * aY + aB2 * aX * aZ + aB3 * aY * aZ)
                              + 2.0 * (aC1 * aX + aC2 * aY + aC3 * aZ) + aD;
                Assert.Equal(0.0, aVal, Precision.Intersection());
            }
        }
    }

    // Test Transform/Transformed are not implemented.
    [Fact]
    public void Transform_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
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
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        Geom_Geometry aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        GeomEval_EllipsoidSurface? aCopySurf = aCopy as GeomEval_EllipsoidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(3.0, aCopySurf.SemiAxisA(), Precision.Confusion());
        Assert.Equal(2.0, aCopySurf.SemiAxisB(), Precision.Confusion());
        Assert.Equal(1.0, aCopySurf.SemiAxisC(), Precision.Confusion());
    }

    // When A=B=C=R the ellipsoid degenerates to a sphere; EvalD0 must match
    [Fact]
    public void EvalD0_MatchesSphere()
    {
        gp_Ax3 anAx3 = new();
        const double aR = 5.0;
        GeomEval_EllipsoidSurface anEllipsoid = new(anAx3, aR, aR, aR);

        double[][] aParams =
        [
            [0.0, 0.0], [M_PI / 4.0, M_PI / 4.0], [M_PI / 2.0, 0.0], [M_PI, -M_PI / 4.0],
            [3.0 * M_PI / 2.0, 0.0], [1.0, 0.3],
        ];

        foreach (double[] aUV in aParams)
        {
            double aU = aUV[0];
            double aV = aUV[1];
            gp_Pnt aPE = anEllipsoid.EvalD0(aU, aV);
            gp_Pnt aPS = SphereEvalD0(anAx3, aR, aU, aV);
            ExpectNear(aPE, aPS, Precision.Confusion());
        }
    }

    // When A=B=C=R, EvalD1 must match the sphere
    [Fact]
    public void EvalD1_MatchesSphere()
    {
        gp_Ax3 anAx3 = new();
        const double aR = 5.0;
        GeomEval_EllipsoidSurface anEllipsoid = new(anAx3, aR, aR, aR);

        double[][] aParams = [[0.0, 0.0], [M_PI / 4.0, M_PI / 4.0], [1.0, 0.3], [M_PI, -M_PI / 4.0]];

        foreach (double[] aUV in aParams)
        {
            double aU = aUV[0];
            double aV = aUV[1];
            Geom_Surface.ResD1 aDE = anEllipsoid.EvalD1(aU, aV);
            Geom_Surface.ResD1 aDS = SphereEvalD1(anAx3, aR, aU, aV);

            ExpectNear(aDE.Point, aDS.Point, Precision.Confusion());
            ExpectNear(aDE.D1U, aDS.D1U, Precision.Angular());
            ExpectNear(aDE.D1V, aDS.D1V, Precision.Angular());
        }
    }

    // When A=B=C=R, EvalD2 must match the sphere
    [Fact]
    public void EvalD2_MatchesSphere()
    {
        gp_Ax3 anAx3 = new();
        const double aR = 5.0;
        GeomEval_EllipsoidSurface anEllipsoid = new(anAx3, aR, aR, aR);

        double[][] aParams = [[0.0, 0.0], [M_PI / 4.0, M_PI / 4.0], [1.0, 0.3]];

        foreach (double[] aUV in aParams)
        {
            double aU = aUV[0];
            double aV = aUV[1];
            Geom_Surface.ResD2 aDE = anEllipsoid.EvalD2(aU, aV);
            Geom_Surface.ResD2 aDS = SphereEvalD2(anAx3, aR, aU, aV);

            ExpectNear(aDE.Point, aDS.Point, Precision.Confusion());
            ExpectNear(aDE.D1U, aDS.D1U, Precision.Angular());
            ExpectNear(aDE.D1V, aDS.D1V, Precision.Angular());
            ExpectNear(aDE.D2U, aDS.D2U, Precision.Angular());
            ExpectNear(aDE.D2V, aDS.D2V, Precision.Angular());
            ExpectNear(aDE.D2UV, aDS.D2UV, Precision.Angular());
        }
    }

    // When A=B=C=R, EvalD3 must match the sphere
    [Fact]
    public void EvalD3_MatchesSphere()
    {
        gp_Ax3 anAx3 = new();
        const double aR = 5.0;
        GeomEval_EllipsoidSurface anEllipsoid = new(anAx3, aR, aR, aR);

        double[][] aParams = [[M_PI / 4.0, M_PI / 4.0], [1.0, 0.3]];

        foreach (double[] aUV in aParams)
        {
            double aU = aUV[0];
            double aV = aUV[1];
            Geom_Surface.ResD3 aDE = anEllipsoid.EvalD3(aU, aV);
            Geom_Surface.ResD3 aDS = SphereEvalD3(anAx3, aR, aU, aV);

            ExpectNear(aDE.Point, aDS.Point, Precision.Confusion());
            ExpectNear(aDE.D1U, aDS.D1U, Precision.Angular());
            ExpectNear(aDE.D1V, aDS.D1V, Precision.Angular());
            ExpectNear(aDE.D2U, aDS.D2U, Precision.Angular());
            ExpectNear(aDE.D2V, aDS.D2V, Precision.Angular());
            ExpectNear(aDE.D2UV, aDS.D2UV, Precision.Angular());
            ExpectNear(aDE.D3U, aDS.D3U, Precision.Angular());
            ExpectNear(aDE.D3V, aDS.D3V, Precision.Angular());
            ExpectNear(aDE.D3UUV, aDS.D3UUV, Precision.Angular());
            ExpectNear(aDE.D3UVV, aDS.D3UVV, Precision.Angular());
        }
    }

    // Verify D2 components are consistent with finite differences of D1
    [Fact]
    public void EvalD2_ConsistentWithD1()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        const double aU = 1.0, aV = 0.5;

        Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        // D2U ~ (D1U(u+h,v) - D1U(u-h,v)) / (2h)
        Geom_Surface.ResD1 aD1Up = aSurf.EvalD1(aU + Precision.Confusion(), aV);
        Geom_Surface.ResD1 aD1Um = aSurf.EvalD1(aU - Precision.Confusion(), aV);
        gp_Vec aFD_D2U = new((aD1Up.D1U.XYZ() - aD1Um.D1U.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD2.D2U, aFD_D2U, THE_FD_TOL);

        // D2V ~ (D1V(u,v+h) - D1V(u,v-h)) / (2h)
        Geom_Surface.ResD1 aD1Vp = aSurf.EvalD1(aU, aV + Precision.Confusion());
        Geom_Surface.ResD1 aD1Vm = aSurf.EvalD1(aU, aV - Precision.Confusion());
        gp_Vec aFD_D2V = new((aD1Vp.D1V.XYZ() - aD1Vm.D1V.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD2.D2V, aFD_D2V, THE_FD_TOL);

        // D2UV ~ (D1U(u,v+h) - D1U(u,v-h)) / (2h)
        gp_Vec aFD_D2UV = new((aD1Vp.D1U.XYZ() - aD1Vm.D1U.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD2.D2UV, aFD_D2UV, THE_FD_TOL);
    }

    // Verify D3 components are consistent with finite differences of D2
    [Fact]
    public void EvalD3_ConsistentWithD2()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        const double aU = 1.0, aV = 0.5;

        Geom_Surface.ResD3 aD3 = aSurf.EvalD3(aU, aV);

        // D3U ~ (D2U(u+h,v) - D2U(u-h,v)) / (2h)
        Geom_Surface.ResD2 aD2Up = aSurf.EvalD2(aU + Precision.Confusion(), aV);
        Geom_Surface.ResD2 aD2Um = aSurf.EvalD2(aU - Precision.Confusion(), aV);
        gp_Vec aFD_D3U = new((aD2Up.D2U.XYZ() - aD2Um.D2U.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD3.D3U, aFD_D3U, THE_FD_TOL);

        // D3V ~ (D2V(u,v+h) - D2V(u,v-h)) / (2h)
        Geom_Surface.ResD2 aD2Vp = aSurf.EvalD2(aU, aV + Precision.Confusion());
        Geom_Surface.ResD2 aD2Vm = aSurf.EvalD2(aU, aV - Precision.Confusion());
        gp_Vec aFD_D3V = new((aD2Vp.D2V.XYZ() - aD2Vm.D2V.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD3.D3V, aFD_D3V, THE_FD_TOL);

        // D3UUV ~ d/dv of D2U
        gp_Vec aFD_D3UUV = new((aD2Vp.D2U.XYZ() - aD2Vm.D2U.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD3.D3UUV, aFD_D3UUV, THE_FD_TOL);

        // D3UVV ~ d/du of D2V
        gp_Vec aFD_D3UVV = new((aD2Up.D2V.XYZ() - aD2Um.D2V.XYZ()) / (2.0 * Precision.Confusion()));
        ExpectNear(aD3.D3UVV, aFD_D3UVV, THE_FD_TOL);
    }

    // Verify EvalDN is consistent with D1 from EvalD1
    [Fact]
    public void EvalDN_ConsistentWithD3()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.0);
        const double aU = 1.0, aV = 0.5;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);

        // EvalDN(u, v, 1, 0) must match D1U
        gp_Vec aDN10 = aSurf.EvalDN(aU, aV, 1, 0);
        ExpectNear(aDN10, aD1.D1U, Precision.Angular());

        // EvalDN(u, v, 0, 1) must match D1V
        gp_Vec aDN01 = aSurf.EvalDN(aU, aV, 0, 1);
        ExpectNear(aDN01, aD1.D1V, Precision.Angular());
    }

    // OCCT.NET addition: all partial derivatives up to order 4 against finite differences, off the standard frame.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        gp_Ax3 anAx3 = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(1.0, 1.0, 1.0), new gp_Dir(1.0, -1.0, 0.0));
        GeomEval_EllipsoidSurface aSurf = new(anAx3, 3.0, 2.0, 1.5);
        for (double aU = 0.1; aU < 2.0 * M_PI; aU += 0.7)
        {
            for (double aV = -1.2; aV <= 1.2; aV += 0.6)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV);
            }
        }
    }

    // OCCT.NET addition: the setters, their With twins and freezing.
    [Fact]
    public void Setters_WithTwins_Freeze()
    {
        GeomEval_EllipsoidSurface aSurf = new(new gp_Ax3(), 3.0, 2.0, 1.0);
        GeomEval_EllipsoidSurface aCopy = aSurf.WithSemiAxisA(4.0).WithSemiAxisB(5.0).WithSemiAxisC(6.0);
        Assert.Equal(3.0, aSurf.SemiAxisA());
        Assert.Equal(4.0, aCopy.SemiAxisA());
        Assert.Equal(5.0, aCopy.SemiAxisB());
        Assert.Equal(6.0, aCopy.SemiAxisC());
        Assert.ThrowsAny<Standard_ConstructionError>(() => aSurf.WithSemiAxisC(0.0));

        aSurf.Freeze();
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.SetSemiAxisA(1.0));
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.UReverse());
        Assert.Equal(1.5, aSurf.WithSemiAxisA(1.5).SemiAxisA());
    }
}
