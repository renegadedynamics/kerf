// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomAdaptor_TransformedSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>GeomAdaptor_TransformedSurface. All of OCCT's tests are ported.</summary>
public class GeomAdaptor_TransformedSurfaceTest
{
    private const double THE_TOLERANCE = 1.0e-10;

    private static Transform3D createTranslation(double theDx, double theDy, double theDz)
    {
        Transform3D aTrsf = new();
        aTrsf.SetTranslation(new Vector3D(theDx, theDy, theDz));
        return aTrsf;
    }

    private static Transform3D createRotationY(double theAngle)
    {
        Transform3D aTrsf = new();
        aTrsf.SetRotation(new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0)), theAngle);
        return aTrsf;
    }

    private static Geom_BSplineSurface createBSplineSurface()
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
        return new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
    }

    private static Geom_Plane XOYPlane() => new(new Plane3D(new CoordinateSystem(Constants.XOY())));

    [Fact]
    public void IdentityTransformUsesOriginalSurface()
    {
        Geom_Plane aPlane = XOYPlane();

        GeomAdaptor_TransformedSurface anAdaptor = new(aPlane, new Transform3D());

        Assert.Same(aPlane, anAdaptor.GeomSurfaceOriginal());
        Assert.Same(aPlane, anAdaptor.GeomSurfaceTransformed());
    }

    [Fact]
    public void PlaneCachesTransformedSurface()
    {
        GeomAdaptor_TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 5.0));

        Geom_Surface? aFirst = anAdaptor.GeomSurfaceTransformed();
        Geom_Surface? aSecond = anAdaptor.GeomSurfaceTransformed();
        Assert.Same(aFirst, aSecond);

        Plane3D aCachedPlane = anAdaptor.Plane();
        Assert.Equal(5.0, aCachedPlane.Location().Z(), THE_TOLERANCE);
    }

    [Fact]
    public void BSplineCachePreservedByShallowCopy()
    {
        GeomAdaptor_TransformedSurface anAdaptor = new(createBSplineSurface(), createTranslation(4.0, 0.0, 0.0));

        Geom_Surface? aSurface = anAdaptor.GeomSurfaceTransformed();
        Geom_BSplineSurface aBSpline = anAdaptor.BSpline();

        Adaptor3d_Surface aCopyBase = anAdaptor.ShallowCopy();
        GeomAdaptor_TransformedSurface? aCopy = aCopyBase as GeomAdaptor_TransformedSurface;
        Assert.NotNull(aCopy);

        Assert.Same(aSurface, aCopy.GeomSurfaceTransformed());
        Assert.Same(aBSpline, aCopy.BSpline());
    }

    [Fact]
    public void ShallowCopyFromUnbuiltStateBuildsCacheLazily()
    {
        GeomAdaptor_TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 4.0));

        GeomAdaptor_TransformedSurface? aCopy = anAdaptor.ShallowCopy() as GeomAdaptor_TransformedSurface;
        Assert.NotNull(aCopy);

        Geom_Surface? aFirst = aCopy.GeomSurfaceTransformed();
        Geom_Surface? aSecond = aCopy.GeomSurfaceTransformed();
        Assert.Same(aFirst, aSecond);
        Assert.Equal(4.0, aCopy.Plane().Location().Z(), THE_TOLERANCE);
    }

    [Fact]
    public void SetTrsfRebuildsCache()
    {
        GeomAdaptor_TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 1.0));

        Geom_Surface? aFirst = anAdaptor.GeomSurfaceTransformed();
        anAdaptor.SetTrsf(createTranslation(0.0, 0.0, 3.0));
        Geom_Surface? aSecond = anAdaptor.GeomSurfaceTransformed();

        Assert.NotSame(aFirst, aSecond);
        Assert.Equal(3.0, anAdaptor.Plane().Location().Z(), THE_TOLERANCE);
    }

    [Fact]
    public void LoadRebuildsCache()
    {
        GeomAdaptor_TransformedSurface anAdaptor = new(XOYPlane(), createTranslation(0.0, 0.0, 2.0));

        Geom_Surface? aFirst = anAdaptor.GeomSurfaceTransformed();

        CoordinateSystem aSphereAx = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SphericalSurface aSphere = new(aSphereAx, 3.0);
        anAdaptor.Load(aSphere, new Transform3D());

        Geom_Surface? aSecond = anAdaptor.GeomSurfaceTransformed();
        Assert.NotSame(aFirst, aSecond);
        Assert.Equal(SurfaceKind.Sphere, anAdaptor.GetType());
        Assert.Equal(3.0, anAdaptor.Sphere().Radius(), THE_TOLERANCE);
    }

    [Fact]
    public void ExtrusionCachesDirectionAndBasisCurve()
    {
        Geom_Line aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Geom_SurfaceOfLinearExtrusion anExt = new(aLine, new Direction3D(0.0, 0.0, 1.0));

        GeomAdaptor_TransformedSurface anAdaptor = new(anExt, createRotationY(M_PI / 2.0));

        Direction3D aDirection = anAdaptor.Direction();
        Assert.Equal(1.0, aDirection.X(), THE_TOLERANCE);
        Assert.Equal(0.0, aDirection.Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aDirection.Z(), THE_TOLERANCE);

        Adaptor3d_Curve aCurve = anAdaptor.BasisCurve();
        Assert.NotNull(aCurve);
        Assert.Equal(CurveKind.Line, aCurve.GetType());
    }

    [Fact]
    public void RevolutionCachesAxisAndBasisCurve()
    {
        Geom_Line aLine = new(new Point3D(2.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SurfaceOfRevolution aRev = new(aLine, new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)));

        GeomAdaptor_TransformedSurface anAdaptor = new(aRev, createRotationY(M_PI / 2.0));

        Axis3D aAxis = anAdaptor.AxeOfRevolution();
        Assert.Equal(1.0, aAxis.Direction().X(), THE_TOLERANCE);
        Assert.Equal(0.0, aAxis.Direction().Y(), THE_TOLERANCE);
        Assert.Equal(0.0, aAxis.Direction().Z(), THE_TOLERANCE);

        Adaptor3d_Curve aCurve = anAdaptor.BasisCurve();
        Assert.NotNull(aCurve);
        Assert.Equal(CurveKind.Line, aCurve.GetType());
    }

    [Fact]
    public void OffsetCachesBasisSurfaceAndOffsetValue()
    {
        Geom_OffsetSurface anOffset = new(XOYPlane(), 2.5);

        GeomAdaptor_TransformedSurface anAdaptor = new(anOffset, createTranslation(0.0, 0.0, 3.0));

        Adaptor3d_Surface aBasis = anAdaptor.BasisSurface();
        Assert.NotNull(aBasis);
        Assert.Equal(SurfaceKind.Plane, aBasis.GetType());
        Assert.Equal(2.5, anAdaptor.OffsetValue(), THE_TOLERANCE);
    }

    [Fact]
    public void UTrimPreservesRestrictedBounds()
    {
        GeomAdaptor_TransformedSurface anAdaptor = new(XOYPlane(), 2.0, 8.0, 3.0, 9.0, createTranslation(0.0, 0.0, 5.0), 0.1, 0.2);

        GeomAdaptor_Surface? aTrimmed = anAdaptor.UTrim(4.0, 6.0, 0.05) as GeomAdaptor_Surface;
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
        GeomAdaptor_TransformedSurface anAdaptor = new(XOYPlane(), 2.0, 8.0, 3.0, 9.0, createTranslation(0.0, 0.0, 7.0), 0.1, 0.2);

        GeomAdaptor_Surface? aTrimmed = anAdaptor.VTrim(4.0, 6.0, 0.15) as GeomAdaptor_Surface;
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
        Geom_Plane aPlane = new(new Plane3D(new CoordinateSystem(Constants.XOY())));
        GeomAdaptor_TransformedSurface anAdaptor = new(aPlane, createTranslation(0.0, 0.0, 5.0));

        GeomGridEval_Surface anEval = new(anAdaptor);

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
