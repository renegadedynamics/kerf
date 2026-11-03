// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomAdaptor_TransformedSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>GeomAdaptor_TransformedSurface. All of OCCT's tests are ported.</summary>
public class GeomAdaptor_TransformedSurfaceTest
{
    private const double THE_TOLERANCE = 1.0e-10;

    private static Transform3D createTranslation(double theDx, double theDy, double theDz)
    {
        Transform3D aTrsf = new();
        aTrsf = aTrsf.WithTranslation(new Vector3D(theDx, theDy, theDz));
        return aTrsf;
    }

    private static Transform3D createRotationY(double theAngle)
    {
        Transform3D aTrsf = new();
        aTrsf = aTrsf.WithRotation(new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0)), theAngle);
        return aTrsf;
    }

    private static BSplineSurface createBSplineSurface()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, 1, new Point3D(1.0, 0.0, 0.0));
        aPoles.SetValue(1, 2, new Point3D(0.0, 1.0, 0.0));
        aPoles.SetValue(2, 2, new Point3D(1.0, 1.0, 1.0));

        NCollection_Array1<double> aUKnots = new(1, 2);
        NCollection_Array1<double> aVKnots = new(1, 2);
        NCollection_Array1<int> aUMults = new(1, 2);
        NCollection_Array1<int> aVMults = new(1, 2);
        aUKnots.SetValue(1, 0.0);
        aUKnots.SetValue(2, 1.0);
        aVKnots.SetValue(1, 0.0);
        aVKnots.SetValue(2, 1.0);
        aUMults.SetValue(1, 2);
        aUMults.SetValue(2, 2);
        aVMults.SetValue(1, 2);
        aVMults.SetValue(2, 2);
        return new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
    }

    private static PlaneSurface XOYPlane() => new(new Plane3D(new CoordinateSystem(Constants.XOY())));

    [Fact]
    public void IdentityTransformUsesOriginalSurface()
    {
        PlaneSurface aPlane = XOYPlane();

        TransformedSurface anAdaptor = new(aPlane, new Transform3D());

        Assert.Same(aPlane, anAdaptor.GeomSurfaceOriginal());
        Assert.Same(aPlane, anAdaptor.GeomSurfaceTransformed());
    }

    [Fact]
    public void PlaneCachesTransformedSurface()
    {
        TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 5.0));

        Surface? aFirst = anAdaptor.GeomSurfaceTransformed();
        Surface? aSecond = anAdaptor.GeomSurfaceTransformed();
        Assert.Same(aFirst, aSecond);

        Plane3D aCachedPlane = anAdaptor.Plane();
        Assert.Equal(5.0, aCachedPlane.Location().Z(), THE_TOLERANCE);
    }

    [Fact]
    public void BSplineCachePreservedByShallowCopy()
    {
        TransformedSurface anAdaptor = new(createBSplineSurface(), createTranslation(4.0, 0.0, 0.0));

        Surface? aSurface = anAdaptor.GeomSurfaceTransformed();
        BSplineSurface aBSpline = anAdaptor.BSpline();

        SurfaceAdaptorBase aCopyBase = anAdaptor.ShallowCopy();
        TransformedSurface? aCopy = aCopyBase as TransformedSurface;
        Assert.NotNull(aCopy);

        Assert.Same(aSurface, aCopy.GeomSurfaceTransformed());
        Assert.Same(aBSpline, aCopy.BSpline());
    }

    [Fact]
    public void ShallowCopyFromUnbuiltStateBuildsCacheLazily()
    {
        TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 4.0));

        TransformedSurface? aCopy = anAdaptor.ShallowCopy() as TransformedSurface;
        Assert.NotNull(aCopy);

        Surface? aFirst = aCopy.GeomSurfaceTransformed();
        Surface? aSecond = aCopy.GeomSurfaceTransformed();
        Assert.Same(aFirst, aSecond);
        Assert.Equal(4.0, aCopy.Plane().Location().Z(), THE_TOLERANCE);
    }

    [Fact]
    public void SetTrsfRebuildsCache()
    {
        TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 1.0));

        Surface? aFirst = anAdaptor.GeomSurfaceTransformed();
        anAdaptor.SetTrsf(createTranslation(0.0, 0.0, 3.0));
        Surface? aSecond = anAdaptor.GeomSurfaceTransformed();

        Assert.NotSame(aFirst, aSecond);
        Assert.Equal(3.0, anAdaptor.Plane().Location().Z(), THE_TOLERANCE);
    }

    [Fact]
    public void LoadRebuildsCache()
    {
        TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 2.0));

        Surface? aFirst = anAdaptor.GeomSurfaceTransformed();

        CoordinateSystem aSphereAx = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere = new(aSphereAx, 3.0);
        anAdaptor.Load(aSphere, new Transform3D());

        Surface? aSecond = anAdaptor.GeomSurfaceTransformed();
        Assert.NotSame(aFirst, aSecond);
        Assert.Equal(SurfaceKind.Sphere, anAdaptor.GetType());
        Assert.Equal(3.0, anAdaptor.Sphere().Radius(), THE_TOLERANCE);
    }

    [Fact]
    public void ExtrusionCachesDirectionAndBasisCurve()
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        ExtrusionSurface anExt = new(aLine, new Direction3D(0.0, 0.0, 1.0));

        TransformedSurface anAdaptor = new(anExt, createRotationY(M_PI / 2.0));

        Direction3D aDirection = anAdaptor.Direction();
        Assert.Equal(1.0, aDirection.X(), THE_TOLERANCE);
        Assert.Equal(0.0, aDirection.Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aDirection.Z(), THE_TOLERANCE);

        CurveAdaptorBase aCurve = anAdaptor.BasisCurve();
        Assert.NotNull(aCurve);
        Assert.Equal(CurveKind.Line, aCurve.GetType());
    }

    [Fact]
    public void RevolutionCachesAxisAndBasisCurve()
    {
        LineCurve3D aLine = new(new Point3D(2.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        RevolutionSurface aRev = new(aLine, new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)));

        TransformedSurface anAdaptor = new(aRev, createRotationY(M_PI / 2.0));

        Axis3D aAxis = anAdaptor.AxeOfRevolution();
        Assert.Equal(1.0, aAxis.Direction().X(), THE_TOLERANCE);
        Assert.Equal(0.0, aAxis.Direction().Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aAxis.Direction().Z(), THE_TOLERANCE);

        CurveAdaptorBase aCurve = anAdaptor.BasisCurve();
        Assert.NotNull(aCurve);
        Assert.Equal(CurveKind.Line, aCurve.GetType());
    }

    [Fact]
    public void OffsetCachesBasisSurfaceAndOffsetValue()
    {
        OffsetSurface anOffset = new(XOYPlane(), 2.5);

        TransformedSurface anAdaptor = new(anOffset, createTranslation(0.0, 0.0, 3.0));

        SurfaceAdaptorBase aBasis = anAdaptor.BasisSurface();
        Assert.NotNull(aBasis);
        Assert.Equal(SurfaceKind.Plane, aBasis.GetType());
        Assert.Equal(2.5, anAdaptor.OffsetValue(), THE_TOLERANCE);
    }

    [Fact]
    public void UTrimPreservesRestrictedBounds()
    {
        TransformedSurface anAdaptor = new(XOYPlane(), 2.0, 8.0, 3.0, 9.0, createTranslation(0.0, 0.0, 5.0), 0.1, 0.2);

        SurfaceAdaptor? aTrimmed = anAdaptor.UTrim(4.0, 6.0, 0.05) as SurfaceAdaptor;
        Assert.NotNull(aTrimmed);

        Assert.Equal(4.0, aTrimmed.FirstUParameter(), THE_TOLERANCE);
        Assert.Equal(6.0, aTrimmed.LastUParameter(), THE_TOLERANCE);
        Assert.Equal(3.0, aTrimmed.FirstVParameter(), THE_TOLERANCE);
        Assert.Equal(9.0, aTrimmed.LastVParameter(), THE_TOLERANCE);
        Assert.Equal(0.05, aTrimmed.ToleranceU(), THE_TOLERANCE);
        Assert.Equal(0.2, aTrimmed.ToleranceV(), THE_TOLERANCE);

        Point3D aPoint = aTrimmed.EvalD0(4.0, 3.0);
        Assert.Equal(5.0, aPoint.Z(), THE_TOLERANCE);
    }

    [Fact]
    public void VTrimPreservesRestrictedBounds()
    {
        TransformedSurface anAdaptor = new(XOYPlane(), 2.0, 8.0, 3.0, 9.0, createTranslation(0.0, 0.0, 7.0), 0.1, 0.2);

        SurfaceAdaptor? aTrimmed = anAdaptor.VTrim(4.0, 6.0, 0.15) as SurfaceAdaptor;
        Assert.NotNull(aTrimmed);

        Assert.Equal(2.0, aTrimmed.FirstUParameter(), THE_TOLERANCE);
        Assert.Equal(8.0, aTrimmed.LastUParameter(), THE_TOLERANCE);
        Assert.Equal(4.0, aTrimmed.FirstVParameter(), THE_TOLERANCE);
        Assert.Equal(6.0, aTrimmed.LastVParameter(), THE_TOLERANCE);
        Assert.Equal(0.1, aTrimmed.ToleranceU(), THE_TOLERANCE);
        Assert.Equal(0.15, aTrimmed.ToleranceV(), THE_TOLERANCE);

        Point3D aPoint = aTrimmed.EvalD0(2.0, 4.0);
        Assert.Equal(7.0, aPoint.Z(), THE_TOLERANCE);
    }

    [Fact]
    public void GridEvalUsesTransformedGeometryOnlyOnce()
    {
        PlaneSurface aPlane = new(new Plane3D(new CoordinateSystem(Constants.XOY())));
        TransformedSurface anAdaptor = new(aPlane, createTranslation(0.0, 0.0, 5.0));

        SurfaceGrid anEval = new(anAdaptor);

        NCollection_Array1<double> aUParams = new(1, 2);
        NCollection_Array1<double> aVParams = new(1, 2);
        aUParams.SetValue(1, 0.0);
        aUParams.SetValue(2, 1.0);
        aVParams.SetValue(1, 0.0);
        aVParams.SetValue(2, 1.0);

        NCollection_Array2<Point3D> aGrid = anEval.EvaluateGrid(aUParams, aVParams);
        Assert.Equal(5.0, aGrid.Value(1, 1).Z(), THE_TOLERANCE);
        Assert.Equal(5.0, aGrid.Value(2, 2).Z(), THE_TOLERANCE);
    }
}
