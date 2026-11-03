// Port of: src/ModelingData/TKG3d/GTests/GeomEval_CircularHelicoidSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static OcctNet.ModelingData.Tests.GeomEval_SurfaceTestHelpers;

namespace OcctNet.ModelingData.Tests;

// Left out: DumpJson_NoCrash (DumpJson is not ported).
public class GeomEval_CircularHelicoidSurfaceTest
{
    private const double THE_FD_TOL = 1e-5;

    [Fact]
    public void Construction_ValidParams()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Assert.Equal(10.0, aSurf.Pitch(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Construction_ZeroPitch_Throws()
    {
        CoordinateSystem anAx3 = new();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new GeomEval_CircularHelicoidSurface(anAx3, 0.0));
    }

    [Fact]
    public void Construction_NegativePitch_NoThrow()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, -5.0);
        Assert.Equal(-5.0, aSurf.Pitch());
    }

    [Fact]
    public void EvalD0_AtKnownPoints()
    {
        CoordinateSystem anAx3 = new();
        const double aP = 10.0;
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, aP);

        // S(0, 1) = (1, 0, 0)
        Point3D aP1 = aSurf.EvalD0(0.0, 1.0);
        Assert.Equal(1.0, aP1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP1.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // S(Pi/2, 1) = (0, 1, P/4)
        Point3D aP2 = aSurf.EvalD0(M_PI / 2.0, 1.0);
        Assert.Equal(0.0, aP2.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1.0, aP2.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP / 4.0, aP2.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());

        // S(u, 0) = (0, 0, P*u/(2*Pi)) -- on the axis
        Point3D aP3 = aSurf.EvalD0(1.0, 0.0);
        Assert.Equal(0.0, aP3.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aP3.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aP / (2.0 * M_PI), aP3.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void D2V_IsZero_RuledSurface()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);

        double[][] aParams = [[0.0, 1.0], [1.0, 2.0], [M_PI, -1.0]];
        foreach (double[] aUV in aParams)
        {
            Geom_Surface.ResD2 aD2 = aSurf.EvalD2(aUV[0], aUV[1]);
            Assert.Equal(0.0, aD2.D2V.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void D1V_UnitRadial()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);

        // |dS/dv| = |cos(u)*XDir + sin(u)*YDir| = 1
        double[] aParams = [0.0, 1.0, M_PI / 2.0, M_PI, 3.5];
        foreach (double aU in aParams)
        {
            Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, 2.0);
            Assert.Equal(1.0, aD1.D1V.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void ComparisonWithHelix_ConstantV()
    {
        CoordinateSystem anAx3 = new();
        Frame3D anAx2 = new(anAx3.Location(), anAx3.Direction(), anAx3.XDirection());
        const double aP = 10.0, aR = 3.0;
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, aP);
        GeomEval_CircularHelixCurve aHelix = new(anAx2, aR, aP);

        // S(u, R) should trace the same path as the helix C(u)
        double[] aParams = [0.0, 0.5, 1.0, M_PI, 2.0 * M_PI];
        foreach (double aU in aParams)
        {
            Point3D aPS = aSurf.EvalD0(aU, aR);
            Point3D aPH = aHelix.EvalD0(aU);
            Assert.Equal(0.0, aPS.Distance(aPH), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void EvalD1_ConsistentWithD0()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        const double aU = 1.0, aV = 2.0;

        Geom_Surface.ResD1 aD1 = aSurf.EvalD1(aU, aV);
        Point3D aPu1 = aSurf.EvalD0(aU + global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Point3D aPu2 = aSurf.EvalD0(aU - global::Kerf.Engine.Foundation.Precision.Confusion(), aV);
        Point3D aPv1 = aSurf.EvalD0(aU, aV + global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D aPv2 = aSurf.EvalD0(aU, aV - global::Kerf.Engine.Foundation.Precision.Confusion());

        Vector3D aFDU = new((aPu1.XYZ() - aPu2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));
        Vector3D aFDV = new((aPv1.XYZ() - aPv2.XYZ()) / (2.0 * global::Kerf.Engine.Foundation.Precision.Confusion()));

        ExpectNear(aD1.D1U, aFDU, THE_FD_TOL);
        ExpectNear(aD1.D1V, aFDV, THE_FD_TOL);
    }

    [Fact]
    public void Bounds_Infinite()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        aSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aU1));
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aU2));
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aV1));
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aV2));
    }

    [Fact]
    public void Properties()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Assert.False(aSurf.IsUClosed());
        Assert.False(aSurf.IsVClosed());
        Assert.False(aSurf.IsUPeriodic());
        Assert.False(aSurf.IsVPeriodic());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UIso(0.0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VIso(0.0));
    }

    [Fact]
    public void Reverse_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReverse());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.UReversedParameter(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.VReversedParameter(0.5));
    }

    [Fact]
    public void Transform_NotImplemented()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(1.0, 2.0, 3.0));

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transform(aTrsf));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => aSurf.Transformed(aTrsf));
    }

    [Fact]
    public void Copy_Independent()
    {
        CoordinateSystem anAx3 = new();
        GeomEval_CircularHelicoidSurface aSurf = new(anAx3, 10.0);
        Geom_Geometry aCopy = aSurf.Copy();
        Assert.NotNull(aCopy);
        GeomEval_CircularHelicoidSurface? aCopySurf = aCopy as GeomEval_CircularHelicoidSurface;
        Assert.NotNull(aCopySurf);
        Assert.Equal(10.0, aCopySurf.Pitch(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCCT.NET addition: all partial derivatives up to order 4 against finite differences, off the standard frame.
    [Fact]
    public void Derivatives_MatchFiniteDifferences()
    {
        CoordinateSystem anAx3 = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(1.0, 1.0, 1.0), new Direction3D(1.0, -1.0, 0.0));
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
        GeomEval_CircularHelicoidSurface aSurf = new(new CoordinateSystem(), 10.0);
        GeomEval_CircularHelicoidSurface aCopy = aSurf.WithPitch(-2.0);
        Assert.Equal(10.0, aSurf.Pitch());
        Assert.Equal(-2.0, aCopy.Pitch());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => aSurf.WithPitch(0.0));

        aSurf.Freeze();
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ImmutableObjectException>(() => aSurf.SetPitch(1.0));
    }
}
