// Port of: src/ModelingData/TKG3d/GTests/GeomEval_HypParaboloidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static OcctNet.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported); EvalD2_MatchesBezierSurface (needs Geom_BezierSurface,
// unit 1C).
public class GeomEval_HypParaboloidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;

    // Test construction with valid parameters
    [Fact]
    public void Construction_ValidParams()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.Equal(2.0, aSurf.SemiAxisA(), Precision.Confusion());
        Assert.Equal(3.0, aSurf.SemiAxisB(), Precision.Confusion());
    }

    // Test construction throws for invalid semi-axes
    [Fact]
    public void Construction_InvalidAxes_Throws()
    {
        gp_Ax3 anAx3 = new();
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_HypParaboloidSurface(anAx3, -1.0, 1.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_HypParaboloidSurface(anAx3, 1.0, 0.0));
        Assert.ThrowsAny<Standard_ConstructionError>(() => new GeomEval_HypParaboloidSurface(anAx3, 0.0, 1.0));
    }

    // Test EvalD0 at u=0, v=0: P = origin
    [Fact]
    public void EvalD0_Origin()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        gp_Pnt aP = aSurf.EvalD0(0.0, 0.0);
        Assert.Equal(0.0, aP.X(), Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), Precision.Confusion());
        Assert.Equal(0.0, aP.Z(), Precision.Confusion());
    }

    // Test EvalD0 at u=1, v=0: P = (1, 0, 1/A^2)
    [Fact]
    public void EvalD0_KnownPoint_U()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, aA, aB);
        gp_Pnt aP = aSurf.EvalD0(1.0, 0.0);
        Assert.Equal(1.0, aP.X(), Precision.Confusion());
        Assert.Equal(0.0, aP.Y(), Precision.Confusion());
        Assert.Equal(1.0 / (aA * aA), aP.Z(), Precision.Confusion()); // u^2/A^2 = 1/4
    }

    // Test EvalD0 at u=0, v=1: P = (0, 1, -1/B^2)
    [Fact]
    public void EvalD0_KnownPoint_V()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, aA, aB);
        gp_Pnt aP = aSurf.EvalD0(0.0, 1.0);
        Assert.Equal(0.0, aP.X(), Precision.Confusion());
        Assert.Equal(1.0, aP.Y(), Precision.Confusion());
        Assert.Equal(-1.0 / (aB * aB), aP.Z(), Precision.Confusion()); // -v^2/B^2 = -1/9
    }

    // Test derivative consistency: compare EvalD1 vs finite difference of EvalD0
    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        const double aU = 1.5, aV = 0.7;

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

    // Test D2 has constant Z component: d2Z/du2 = 2/A^2, d2Z/dv2 = -2/B^2
    [Fact]
    public void EvalD2_ConstantZComponent()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, aA, aB);

        // Evaluate at several parameter values: D2 should be constant
        Geom_Surface.ResD2 aD2a = aSurf.EvalD2(0.0, 0.0);
        Geom_Surface.ResD2 aD2b = aSurf.EvalD2(1.0, 2.0);
        Geom_Surface.ResD2 aD2c = aSurf.EvalD2(-3.0, 5.0);

        double aExpD2U_Z = 2.0 / (aA * aA);
        double aExpD2V_Z = -2.0 / (aB * aB);

        Assert.Equal(aExpD2U_Z, aD2a.D2U.Z(), Precision.Angular());
        Assert.Equal(aExpD2U_Z, aD2b.D2U.Z(), Precision.Angular());
        Assert.Equal(aExpD2U_Z, aD2c.D2U.Z(), Precision.Angular());

        Assert.Equal(aExpD2V_Z, aD2a.D2V.Z(), Precision.Angular());
        Assert.Equal(aExpD2V_Z, aD2b.D2V.Z(), Precision.Angular());
        Assert.Equal(aExpD2V_Z, aD2c.D2V.Z(), Precision.Angular());
    }

    // Test Bounds: all infinite, not periodic, not closed
    [Fact]
    public void Bounds_Periodicity()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 1.0, 1.0);
        aSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.True(aU1 < -1e10);
        Assert.True(aU2 > 1e10);
        Assert.True(aV1 < -1e10);
        Assert.True(aV2 > 1e10);
        Assert.False(aSurf.IsUPeriodic());
        Assert.False(aSurf.IsUClosed());
        Assert.False(aSurf.IsVPeriodic());
        Assert.False(aSurf.IsVClosed());
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.UIso(0.5));
        Assert.ThrowsAny<Standard_NotImplemented>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
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
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        aSurf.Coefficients(out double aA1, out double aA2, out double aA3, out double aB1, out double aB2,
                           out double aB3, out double aC1, out double aC2, out double aC3, out double aD);

        for (double u = -2.0; u <= 2.0; u += 1.0)
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
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
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
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Geom_Geometry aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        GeomEval_HypParaboloidSurface? aCopySurf = aCopy as GeomEval_HypParaboloidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(2.0, aCopySurf.SemiAxisA(), Precision.Confusion());
        Assert.Equal(3.0, aCopySurf.SemiAxisB(), Precision.Confusion());
    }

    // Test D3 components are zero for degree-2 polynomial surface
    [Fact]
    public void EvalD3_Zero()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);

        Geom_Surface.ResD3 aD3 = aSurf.EvalD3(1.0, 1.0);

        Assert.True(aD3.D3U.Magnitude() < Precision.SquareConfusion());
        Assert.True(aD3.D3V.Magnitude() < Precision.SquareConfusion());
        Assert.True(aD3.D3UUV.Magnitude() < Precision.SquareConfusion());
        Assert.True(aD3.D3UVV.Magnitude() < Precision.SquareConfusion());
    }

    // Test D2UV cross derivative is zero since u and v terms are separated
    [Fact]
    public void EvalD2_CrossDerivative()
    {
        gp_Ax3 anAx3 = new();
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);

        double[][] aTestPoints = [[0.0, 0.0], [1.5, -2.0], [-3.0, 4.5]];
        foreach (double[] aPt in aTestPoints)
        {
            Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aPt[0], aPt[1]);
            Assert.Equal(0.0, aD2.D2UV.X(), Precision.SquareConfusion());
            Assert.Equal(0.0, aD2.D2UV.Y(), Precision.SquareConfusion());
            Assert.Equal(0.0, aD2.D2UV.Z(), Precision.SquareConfusion());
        }
    }

    // Test D2 known values including X,Y components (should be zero)
    [Fact]
    public void EvalD2_KnownValues()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, aA, aB);

        Geom_Surface.ResD2 aD2 = aSurf.EvalD2(1.5, -2.7);

        // D2U = (0, 0, 2/A^2)
        Assert.Equal(0.0, aD2.D2U.X(), Precision.SquareConfusion());
        Assert.Equal(0.0, aD2.D2U.Y(), Precision.SquareConfusion());
        Assert.Equal(2.0 / (aA * aA), aD2.D2U.Z(), Precision.Angular());

        // D2V = (0, 0, -2/B^2)
        Assert.Equal(0.0, aD2.D2V.X(), Precision.SquareConfusion());
        Assert.Equal(0.0, aD2.D2V.Y(), Precision.SquareConfusion());
        Assert.Equal(-2.0 / (aB * aB), aD2.D2V.Z(), Precision.Angular());
    }

    // Test EvalDN consistency with D2 and higher-order zero
    [Fact]
    public void EvalDN_HigherOrder()
    {
        gp_Ax3 anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, aA, aB);
        const double aU = 1.0, aV = 1.0;

        Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        // EvalDN(u,v,2,0) should match D2U
        gp_Vec aDN20 = aSurf.EvalDN(aU, aV, 2, 0);
        ExpectNear(aDN20, aD2.D2U, Precision.Confusion());

        // EvalDN(u,v,0,2) should match D2V
        gp_Vec aDN02 = aSurf.EvalDN(aU, aV, 0, 2);
        ExpectNear(aDN02, aD2.D2V, Precision.Confusion());

        // EvalDN(u,v,3,0) should be zero vector
        gp_Vec aDN30 = aSurf.EvalDN(aU, aV, 3, 0);
        Assert.True(aDN30.Magnitude() < Precision.SquareConfusion());
    }

    // OCCT.NET addition: all partial derivatives up to order 4 against finite differences, off the standard frame.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        gp_Ax3 anAx3 = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(1.0, 1.0, 1.0), new gp_Dir(1.0, -1.0, 0.0));
        GeomEval_HypParaboloidSurface aSurf = new(anAx3, 2.0, 1.5);
        for (double aU = -2.0; aU <= 2.0; aU += 0.7)
        {
            for (double aV = -2.0; aV <= 2.0; aV += 0.7)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV);
            }
        }
    }

    // OCCT.NET addition: the setters' With twins and freezing.
    [Fact]
    public void Setters_WithTwins_Freeze()
    {
        GeomEval_HypParaboloidSurface aSurf = new(new gp_Ax3(), 2.0, 3.0);
        GeomEval_HypParaboloidSurface aCopy = aSurf.WithSemiAxisA(4.0).WithSemiAxisB(5.0);
        Assert.Equal(2.0, aSurf.SemiAxisA());
        Assert.Equal(4.0, aCopy.SemiAxisA());
        Assert.Equal(5.0, aCopy.SemiAxisB());
        Assert.ThrowsAny<Standard_ConstructionError>(() => aSurf.WithSemiAxisB(-1.0));

        aSurf.Freeze();
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.SetSemiAxisB(1.0));
    }
}
