// Port of: src/ModelingData/TKG3d/GTests/GeomEval_HypParaboloidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation;
using Kerf.Geometry;
using static Kerf.Kernel.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace Kerf.Kernel.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported); EvalD2_MatchesBezierSurface (needs Geom_BezierSurface,
// unit 1C).
public class GeomEval_HypParaboloidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;

    // Test construction with valid parameters
    [Fact]
    public void Construction_ValidParams()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Assert.Equal(2.0, aSurf.SemiAxisA(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aSurf.SemiAxisB(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test construction throws for invalid semi-axes
    [Fact]
    public void Construction_InvalidAxes_Throws()
    {
        CoordinateSystem anAx3 = new();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HypParaboloidSurface(anAx3, -1.0, 1.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HypParaboloidSurface(anAx3, 1.0, 0.0));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new HypParaboloidSurface(anAx3, 0.0, 1.0));
    }

    // Test EvalD0 at u=0, v=0: P = origin
    [Fact]
    public void EvalD0_Origin()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Point3D aP = aSurf.EvalD0(0.0, 0.0);
        Assert.Equal(0.0, aP.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test EvalD0 at u=1, v=0: P = (1, 0, 1/A^2)
    [Fact]
    public void EvalD0_KnownPoint_U()
    {
        CoordinateSystem anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        HypParaboloidSurface aSurf = new(anAx3, aA, aB);
        Point3D aP = aSurf.EvalD0(1.0, 0.0);
        Assert.Equal(1.0, aP.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0 / (aA * aA), aP.Z, global::Kerf.Engine.Foundation.Precision.Confusion()); // u^2/A^2 = 1/4
    }

    // Test EvalD0 at u=0, v=1: P = (0, 1, -1/B^2)
    [Fact]
    public void EvalD0_KnownPoint_V()
    {
        CoordinateSystem anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        HypParaboloidSurface aSurf = new(anAx3, aA, aB);
        Point3D aP = aSurf.EvalD0(0.0, 1.0);
        Assert.Equal(0.0, aP.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aP.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-1.0 / (aB * aB), aP.Z, global::Kerf.Engine.Foundation.Precision.Confusion()); // -v^2/B^2 = -1/9
    }

    // Test derivative consistency: compare EvalD1 vs finite difference of EvalD0
    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        const double aU = 1.5, aV = 0.7;

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

    // Test D2 has constant Z component: d2Z/du2 = 2/A^2, d2Z/dv2 = -2/B^2
    [Fact]
    public void EvalD2_ConstantZComponent()
    {
        CoordinateSystem anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        HypParaboloidSurface aSurf = new(anAx3, aA, aB);

        // Evaluate at several parameter values: D2 should be constant
        Surface.ResD2 aD2a = aSurf.EvalD2(0.0, 0.0);
        Surface.ResD2 aD2b = aSurf.EvalD2(1.0, 2.0);
        Surface.ResD2 aD2c = aSurf.EvalD2(-3.0, 5.0);

        double aExpD2U_Z = 2.0 / (aA * aA);
        double aExpD2V_Z = -2.0 / (aB * aB);

        Assert.Equal(aExpD2U_Z, aD2a.D2U.Z, global::Kerf.Engine.Foundation.Precision.Angular());
        Assert.Equal(aExpD2U_Z, aD2b.D2U.Z, global::Kerf.Engine.Foundation.Precision.Angular());
        Assert.Equal(aExpD2U_Z, aD2c.D2U.Z, global::Kerf.Engine.Foundation.Precision.Angular());

        Assert.Equal(aExpD2V_Z, aD2a.D2V.Z, global::Kerf.Engine.Foundation.Precision.Angular());
        Assert.Equal(aExpD2V_Z, aD2b.D2V.Z, global::Kerf.Engine.Foundation.Precision.Angular());
        Assert.Equal(aExpD2V_Z, aD2c.D2V.Z, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    // Test Bounds: all infinite, not periodic, not closed
    [Fact]
    public void Bounds_Periodicity()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 1.0, 1.0);
        aSurf.ParameterBounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.True(aU1 < -1e10);
        Assert.True(aU2 > 1e10);
        Assert.True(aV1 < -1e10);
        Assert.True(aV2 > 1e10);
        Assert.False(aSurf.IsUPeriodic);
        Assert.False(aSurf.IsUClosed);
        Assert.False(aSurf.IsVPeriodic);
        Assert.False(aSurf.IsVClosed);
    }

    [Fact]
    public void Iso_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UIso(0.5));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VIso(0.5));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReverse());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReverse());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReversedParameter(0.5));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReversedParameter(0.5));
    }

    // Test implicit equation at evaluated points
    [Fact]
    public void Coefficients_SatisfiedAtEvalPoints()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        aSurf.Coefficients(out double aA1, out double aA2, out double aA3, out double aB1, out double aB2,
                           out double aB3, out double aC1, out double aC2, out double aC3, out double aD);

        for (double u = -2.0; u <= 2.0; u += 1.0)
        {
            for (double v = -2.0; v <= 2.0; v += 1.0)
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
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(1.0, 2.0, 3.0));

        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transform(aTrsf));
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transformed(aTrsf));
    }

    // Test Copy produces independent identical object
    [Fact]
    public void Copy_Independent()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);
        Geometry3D aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        HypParaboloidSurface? aCopySurf = aCopy as HypParaboloidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(2.0, aCopySurf.SemiAxisA(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCopySurf.SemiAxisB(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // Test D3 components are zero for degree-2 polynomial surface
    [Fact]
    public void EvalD3_Zero()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);

        Surface.ResD3 aD3 = aSurf.EvalD3(1.0, 1.0);

        Assert.True(aD3.D3U.Magnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.True(aD3.D3V.Magnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.True(aD3.D3UUV.Magnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.True(aD3.D3UVV.Magnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion());
    }

    // Test D2UV cross derivative is zero since u and v terms are separated
    [Fact]
    public void EvalD2_CrossDerivative()
    {
        CoordinateSystem anAx3 = new();
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 3.0);

        double[][] aTestPoints = [[0.0, 0.0], [1.5, -2.0], [-3.0, 4.5]];
        foreach (double[] aPt in aTestPoints)
        {
            Surface.ResD2 aD2 = aSurf.EvalD2(aPt[0], aPt[1]);
            Assert.Equal(0.0, aD2.D2UV.X, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
            Assert.Equal(0.0, aD2.D2UV.Y, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
            Assert.Equal(0.0, aD2.D2UV.Z, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        }
    }

    // Test D2 known values including X,Y components (should be zero)
    [Fact]
    public void EvalD2_KnownValues()
    {
        CoordinateSystem anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        HypParaboloidSurface aSurf = new(anAx3, aA, aB);

        Surface.ResD2 aD2 = aSurf.EvalD2(1.5, -2.7);

        // D2U = (0, 0, 2/A^2)
        Assert.Equal(0.0, aD2.D2U.X, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.Equal(0.0, aD2.D2U.Y, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.Equal(2.0 / (aA * aA), aD2.D2U.Z, global::Kerf.Engine.Foundation.Precision.Angular());

        // D2V = (0, 0, -2/B^2)
        Assert.Equal(0.0, aD2.D2V.X, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.Equal(0.0, aD2.D2V.Y, global::Kerf.Engine.Foundation.Precision.SquareConfusion());
        Assert.Equal(-2.0 / (aB * aB), aD2.D2V.Z, global::Kerf.Engine.Foundation.Precision.Angular());
    }

    // Test EvalDN consistency with D2 and higher-order zero
    [Fact]
    public void EvalDN_HigherOrder()
    {
        CoordinateSystem anAx3 = new();
        const double aA = 2.0, aB = 3.0;
        HypParaboloidSurface aSurf = new(anAx3, aA, aB);
        const double aU = 1.0, aV = 1.0;

        Surface.ResD2 aD2 = aSurf.EvalD2(aU, aV);

        // EvalDN(u,v,2,0) should match D2U
        Vector3D aDN20 = aSurf.EvalDN(aU, aV, 2, 0);
        ExpectNear(aDN20, aD2.D2U, global::Kerf.Engine.Foundation.Precision.Confusion());

        // EvalDN(u,v,0,2) should match D2V
        Vector3D aDN02 = aSurf.EvalDN(aU, aV, 0, 2);
        ExpectNear(aDN02, aD2.D2V, global::Kerf.Engine.Foundation.Precision.Confusion());

        // EvalDN(u,v,3,0) should be zero vector
        Vector3D aDN30 = aSurf.EvalDN(aU, aV, 3, 0);
        Assert.True(aDN30.Magnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion());
    }

    // Kerf addition: all partial derivatives up to order 4 against finite differences, off the standard frame.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        CoordinateSystem anAx3 = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(1.0, 1.0, 1.0), new Direction3D(1.0, -1.0, 0.0));
        HypParaboloidSurface aSurf = new(anAx3, 2.0, 1.5);
        for (double aU = -2.0; aU <= 2.0; aU += 0.7)
        {
            for (double aV = -2.0; aV <= 2.0; aV += 0.7)
            {
                EvalTestUtils.CheckSurfaceDerivatives(aSurf, aU, aV);
            }
        }
    }

    // Kerf addition: the setters' With twins and freezing.
    [Fact]
    public void Setters_WithTwins_Freeze()
    {
        HypParaboloidSurface aSurf = new(new CoordinateSystem(), 2.0, 3.0);
        HypParaboloidSurface aCopy = aSurf.WithSemiAxisA(4.0).WithSemiAxisB(5.0);
        Assert.Equal(2.0, aSurf.SemiAxisA());
        Assert.Equal(4.0, aCopy.SemiAxisA());
        Assert.Equal(5.0, aCopy.SemiAxisB());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aSurf.WithSemiAxisB(-1.0));

        aSurf.Freeze();
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aSurf.SetSemiAxisB(1.0));
    }
}
