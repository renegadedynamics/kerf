// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// ProjLib_ComputeApproxOnPolarSurface on curves lying on spheres, cones, tori, cylinders and on B-spline, Bezier and
/// offset surfaces (some with polar points): the 2D result, mapped back through the surface, is the 3D curve at the
/// same parameter.
/// </summary>
public class ProjLib_ComputeApproxOnPolarSurfacePropertyTests
{
    private static readonly CoordinateSystem Std = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0));

    private static void CheckOnCurve(Curve2D? thePCurve, SurfaceAdaptorBase theSurf, CurveAdaptorBase theCurve,
                                     double theTol, int theNb = 50)
    {
        Assert.NotNull(thePCurve);
        double aFirst = Max(theCurve.FirstParameter(), thePCurve.FirstParameter);
        double aLast = Min(theCurve.LastParameter(), thePCurve.LastParameter);
        Assert.True(aLast - aFirst > 0.99 * (theCurve.LastParameter() - theCurve.FirstParameter()));
        for (int i = 0; i <= theNb; i++)
        {
            double t = aFirst + (aLast - aFirst) * i / theNb;
            Point2D aUV = thePCurve.Value(t);
            Point3D aP = theSurf.Value(aUV.X, aUV.Y);
            double aDist = aP.DistanceTo(theCurve.Value(t));
            Assert.True(aDist <= theTol, $"distance {aDist} at t = {t}");
        }
    }

    private static BSplineCurve2D Project(Curve3D theC, Surface theS, double theTol = 1.0e-6) =>
        Project(new CurveAdaptor(theC), new SurfaceAdaptor(theS), theTol);

    private static BSplineCurve2D Project(CurveAdaptorBase aC, SurfaceAdaptorBase aS, double theTol)
    {
        var aProj = new ComputeApproxOnPolarSurface();
        aProj.SetTolerance(theTol);
        aProj.Perform(aC, aS);
        Assert.True(aProj.IsDone);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        CheckOnCurve(aRes, aS, aC, 1.0e-4);
        return aRes;
    }

    [Fact]
    public void Sphere_Parallel()
    {
        var aSphere = new SphericalSurface(Std, 3.0);
        Curve3D aC = aSphere.VIso(0.7);
        Project(aC, aSphere);
    }

    [Fact]
    public void Sphere_TiltedGreatCircle()
    {
        var aSphere = new SphericalSurface(Std, 3.0);
        var aC = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0.3, 0.2, 1.0)), 3.0);
        Project(aC, aSphere);
    }

    [Fact]
    public void Sphere_SmallCircleNearPole()
    {
        // A circle around a point close to the north pole, not containing it.
        var aSphere = new SphericalSurface(Std, 2.0);
        Direction3D aN = new(0.15, 0.0, 1.0);
        double aR = 0.1, aD = Math.Sqrt(4.0 - aR * aR);
        var aC = new CircleCurve3D(new Frame3D(new Point3D(aD * aN.X, aD * aN.Y, aD * aN.Z), aN), aR);
        Project(aC, aSphere);
    }

    [Fact]
    public void Sphere_ArcEndingAtPole()
    {
        // A meridian arc from the equator up to the north pole.
        var aSphere = new SphericalSurface(Std, 2.0);
        var aC = new TrimmedCurve3D(aSphere.UIso(0.4), 0.0, M_PI / 2.0);
        BSplineCurve2D aRes = Project(aC, aSphere);
        Point2D aEnd = aRes.Value(aRes.LastParameter);
        Assert.Equal(M_PI / 2.0, aEnd.Y, 1.0e-6);
    }

    [Fact]
    public void Cone_Parallel()
    {
        var aCone = new ConicalSurface(Std, M_PI / 6.0, 2.0);
        Project(aCone.VIso(1.5), aCone);
    }

    [Fact]
    public void Cone_GeneratorToApex()
    {
        var aCone = new ConicalSurface(Std, M_PI / 6.0, 2.0);
        Point3D anApex = aCone.Apex;
        Point3D aBase = aCone.Value(0.8, 0.0);
        var aC = new TrimmedCurve3D(new LineCurve3D(aBase, new Direction3D(new Vector3D(aBase, anApex))), 0.0,
                                       aBase.DistanceTo(anApex));
        Project(aC, aCone);
    }

    [Fact]
    public void Torus_MeridianAndParallel()
    {
        var aTorus = new ToroidalSurface(Std, 5.0, 1.5);
        Project(aTorus.UIso(1.0), aTorus);
        Project(aTorus.VIso(2.0), aTorus);
    }

    [Fact]
    public void Cylinder_HelixBSpline()
    {
        var aCyl = new CylindricalSurface(Std, 2.0);
        // A helix: the image of a 2D line through the cylinder, more than one turn.
        var aLine2d = new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.3));
        var aS = new SurfaceAdaptor(aCyl);
        var aHelix = new CurveOnSurface(new CurveAdaptor2D(aLine2d, 0.0, 8.0), aS);
        Project(aHelix, aS, 1.0e-6);
    }

    [Fact]
    public void BSplineSphere_ParallelAndNearPole()
    {
        // A sphere converted to a B-spline surface: its pole rows are collapsed (polar points).
        var aSphere = new SphericalSurface(Std, 2.0);
        BSplineSurface aBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aSphere);
        Project(aBS.VIso(0.5), aBS, 1.0e-5);
        Project(aBS.VIso(1.4), aBS, 1.0e-5);
        Project(new TrimmedCurve3D(aBS.UIso(1.0), 0.0, 1.5), aBS, 1.0e-5);
    }

    [Fact]
    public void BezierSurface_IsoCurves()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i, j, 0.3 * Math.Sin(i + 2.0 * j));
            }
        }
        var aBez = new BezierSurface(aPoles);
        Project(aBez.UIso(0.3), aBez, 1.0e-6);
        Project(aBez.VIso(0.7), aBez, 1.0e-6);
    }

    [Fact]
    public void OffsetSurface_IsoCurve()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D(i, j, 0.2 * ((i - 2) * (i - 2) - (j - 2) * (j - 2)));
            }
        }
        var anOffset = new OffsetSurface(new BezierSurface(aPoles), 0.5);
        Project(anOffset.UIso(0.4), anOffset, 1.0e-6);
    }

    private static (BSplineSurface, BSplineCurve3D) PlanarSurfaceAndC0Curve()
    {
        NCollection_Array2<Point3D> aSP = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aSP[i, j] = new Point3D(5.0 * (i - 1), 5.0 * (j - 1), 0.0);
            }
        }
        NCollection_Array1<double> aSK = new(1, 2);
        aSK[1] = 0.0;
        aSK[2] = 1.0;
        NCollection_Array1<int> aSM = new(1, 2);
        aSM.Init(3);
        var aSurf = new BSplineSurface(aSP, aSK, aSK, aSM, aSM, 2, 2);

        // A degree 2 curve in the plane with a C0 knot in the middle.
        NCollection_Array1<Point3D> aCP = new(1, 5);
        aCP[1] = new Point3D(1.0, 1.0, 0.0);
        aCP[2] = new Point3D(3.0, 4.0, 0.0);
        aCP[3] = new Point3D(5.0, 2.0, 0.0);
        aCP[4] = new Point3D(8.0, 5.0, 0.0);
        aCP[5] = new Point3D(9.0, 9.0, 0.0);
        NCollection_Array1<double> aCK = new(1, 3);
        aCK[1] = 0.0;
        aCK[2] = 1.0;
        aCK[3] = 2.0;
        NCollection_Array1<int> aCM = new(1, 3);
        aCM[1] = 3;
        aCM[2] = 2;
        aCM[3] = 3;
        return (aSurf, new BSplineCurve3D(aCP, aCK, aCM, 2));
    }

    [Fact]
    public void C0BSplineCurve_ProjectedPerC1Interval()
    {
        (BSplineSurface aSurf, BSplineCurve3D aCurve) = PlanarSurfaceAndC0Curve();
        var aC = new CurveAdaptor(aCurve);
        Assert.Equal(2, aC.NbIntervals(C1));
        BSplineCurve2D aRes = Project(aCurve, aSurf);
        // The two pieces are concatenated with a knot of multiplicity = degree at the C0 parameter.
        Assert.Equal(1.0, aRes.Knot(2), 1.0e-12);
        Assert.Equal(aRes.Degree, aRes.Multiplicity(2));
    }

    [Fact]
    public void C0BSplineCurve_WithMatchingInitialCurve()
    {
        (BSplineSurface aSurf, BSplineCurve3D aCurve) = PlanarSurfaceAndC0Curve();
        // The exact pcurve (the surface maps (u, v) to (10 u, 10 v)), with the same C0 knot.
        NCollection_Array1<Point2D> aP2d = new(1, aCurve.PoleCount);
        for (int i = 1; i <= aCurve.PoleCount; i++)
        {
            Point3D aP = aCurve.Pole(i);
            aP2d[i] = new Point2D(aP.X / 10.0, aP.Y / 10.0);
        }
        var anInit = new BSplineCurve2D(aP2d, aCurve.InternalKnots(), aCurve.InternalMultiplicities(), 2);
        var aC = new CurveAdaptor(aCurve);
        var aS = new SurfaceAdaptor(aSurf);
        var aProj = new ComputeApproxOnPolarSurface(new CurveAdaptor2D(anInit), aC, aS, 1.0e-6);
        Assert.True(aProj.IsDone);
        CheckOnCurve(aProj.BSpline(), aS, aC, 1.0e-5);
    }

    [Fact]
    public void Perform_WithInitialCurve_ReturnsNewUnfrozenCurve()
    {
        var aSphere = new SphericalSurface(Std, 3.0);
        var aC = new CurveAdaptor(aSphere.VIso(0.3));
        var aS = new SurfaceAdaptor(aSphere);
        var aProj = new ComputeApproxOnPolarSurface();
        aProj.SetTolerance(1.0e-6);
        Curve2DAdaptorBase? anInit = aProj.BuildInitialCurve2d(aC, aS);
        Assert.NotNull(anInit);
        Assert.True(aProj.IsDone);
        // The initial curve is a degree-1 B-spline, stored as BSpline().
        Assert.Equal(1, aProj.InternalBSpline()!.Degree);

        BSplineCurve2D? aRes = aProj.Perform(anInit, aC, aS);
        Assert.NotNull(aRes);
        Assert.False(aRes.IsFrozen);
        CheckOnCurve(aRes, aS, aC, 1.0e-5);
        // Not stored: BSpline() is still the initial curve.
        Assert.NotSame(aRes, aProj.InternalBSpline());
    }

    [Fact]
    public void BSpline_FrozenWhenHandedOut()
    {
        var aSphere = new SphericalSurface(Std, 3.0);
        var aProj = new ComputeApproxOnPolarSurface(new CurveAdaptor(aSphere.VIso(0.3)),
                                                            new SurfaceAdaptor(aSphere));
        BSplineCurve2D? anInternal = aProj.InternalBSpline();
        Assert.NotNull(anInternal);
        Assert.False(anInternal.IsFrozen);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.Same(anInternal, aRes);
        Assert.True(aRes!.IsFrozen);
        Assert.Same(aRes, aProj.BSpline());
        Assert.Null(aProj.Curve2d());
    }

    [Fact]
    public void SeamConstructor_GivesBothPCurves()
    {
        var aCyl = new CylindricalSurface(Std, 2.0);
        var aSeam = new TrimmedCurve3D(aCyl.UIso(0.0), 0.0, 3.0);
        var aInit = new CurveAdaptor2D(new LineCurve2D(new Point2D(0.0, 0.0), new Direction2D(0.0, 1.0)), 0.0, 3.0);
        var aInitBis =
            new CurveAdaptor2D(new LineCurve2D(new Point2D(2.0 * M_PI, 0.0), new Direction2D(0.0, 1.0)), 0.0, 3.0);
        var aC = new CurveAdaptor(aSeam);
        var aS = new SurfaceAdaptor(aCyl);
        var aProj = new ComputeApproxOnPolarSurface(aInit, aInitBis, aC, aS, 1.0e-6);
        Assert.True(aProj.IsDone);
        BSplineCurve2D? aFirst = aProj.BSpline();
        Curve2D? aSecond = aProj.Curve2d();
        Assert.NotNull(aFirst);
        Assert.NotNull(aSecond);
        Assert.True(aSecond.IsFrozen);
        CheckOnCurve(aFirst, aS, aC, 1.0e-5);
        CheckOnCurve(aSecond, aS, aC, 1.0e-5);
        double aDU = Math.Abs(aFirst.Value(1.0).X - aSecond.Value(1.0).X);
        Assert.Equal(2.0 * M_PI, aDU, 1.0e-9);
    }

    [Fact]
    public void Settings_DegreeSegmentsAndBoundaries()
    {
        var aTorus = new ToroidalSurface(Std, 5.0, 1.5);
        var aC = new CurveAdaptor(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0.5), new Direction3D(0.05, 0.0, 1.0)), 5.5));
        var aS = new SurfaceAdaptor(aTorus);
        var aProj = new ComputeApproxOnPolarSurface();
        aProj.SetTolerance(1.0e-4);
        aProj.SetDegree(3, 5);
        aProj.SetMaxSegments(50);
        aProj.SetBndPnt(PassPoint);
        aProj.Perform(aC, aS);
        Assert.True(aProj.IsDone);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.InRange(aRes.Degree, 3, 5);
        Assert.True(aRes.KnotCount - 1 <= 50);
        Assert.True(aProj.Tolerance() > 0.0);
    }

    [Fact]
    public void MaxDist_StopsFarCurves()
    {
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D(i, j, 0.1 * i * j);
            }
        }
        var aBez = new BezierSurface(aPoles);
        // An iso-curve lifted by 1 along Z.
        Curve3D aLifted = aBez.UIso(0.5).Translated(new Vector3D(0.0, 0.0, 1.0)) as Curve3D ?? throw new InvalidOperationException();
        var aC = new CurveAdaptor(aLifted);
        var aS = new SurfaceAdaptor(aBez);

        var aNear = new ComputeApproxOnPolarSurface();
        aNear.SetTolerance(1.0e-4);
        aNear.SetMaxDist(0.1);
        aNear.Perform(aC, aS);
        Assert.False(aNear.IsDone);
        Assert.Null(aNear.BSpline());

        var aFar = new ComputeApproxOnPolarSurface();
        aFar.SetTolerance(1.0e-4);
        aFar.SetMaxDist(10.0);
        aFar.Perform(aC, aS);
        Assert.True(aFar.IsDone);
        Assert.NotNull(aFar.BSpline());
    }

    [Fact]
    public void UnsupportedSurfaces_Throw()
    {
        // As in OCCT: planes and swept surfaces aren't polar surfaces; BuildInitialCurve2d throws for them.
        var aC = new CurveAdaptor(new CircleCurve3D(new Frame3D(), 1.0));
        var aPlane = new SurfaceAdaptor(new PlaneSurface(new Plane3D()));
        EngineAssert.ThrowsAny<NoSuchObjectException>(() => new ComputeApproxOnPolarSurface(aC, aPlane));
        var aRev = new SurfaceAdaptor(
            new RevolutionSurface(new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), new Axis3D()));
        EngineAssert.ThrowsAny<NoSuchObjectException>(() => new ComputeApproxOnPolarSurface(aC, aRev));
    }

    [Fact]
    public void BilinearSurface_PoleWiseBranchIsDead()
    {
        // OCCT tests Geom_BSplineSurface::MaxDegree() == 1, but MaxDegree() is static (25), so the pole-wise
        // projection of a curve on a bilinear surface (which would set Tolerance() to the 3D tolerance exactly) is
        // never taken: the curve is approximated instead (occt-issues.md).
        NCollection_Array2<Point3D> aSP = new(1, 2, 1, 2);
        aSP[1, 1] = new Point3D(0, 0, 0);
        aSP[2, 1] = new Point3D(10, 0, 0);
        aSP[1, 2] = new Point3D(0, 10, 0);
        aSP[2, 2] = new Point3D(10, 10, 0);
        NCollection_Array1<double> aK = new(1, 2);
        aK[1] = 0.0;
        aK[2] = 1.0;
        NCollection_Array1<int> aM = new(1, 2);
        aM.Init(2);
        var aSurf = new BSplineSurface(aSP, aK, aK, aM, aM, 1, 1);
        Assert.Equal(1, aSurf.UDegree);
        Assert.Equal(25, BSplineSurface.MaxDegree);

        NCollection_Array1<Point3D> aCP = new(1, 4);
        aCP[1] = new Point3D(1, 1, 0);
        aCP[2] = new Point3D(3, 7, 0);
        aCP[3] = new Point3D(6, 2, 0);
        aCP[4] = new Point3D(9, 8, 0);
        var aCurve = new BezierCurve3D(aCP);
        var aC = new CurveAdaptor(aCurve);
        var aS = new SurfaceAdaptor(aSurf);
        var aProj = new ComputeApproxOnPolarSurface();
        aProj.SetTolerance(1.0e-5);
        aProj.Perform(aC, aS);
        Assert.True(aProj.IsDone);
        CheckOnCurve(aProj.BSpline(), aS, aC, 1.0e-4);
        Assert.NotEqual(1.0e-5, aProj.Tolerance());
    }

    [Fact]
    public void Curve2d_NotDone_Throws()
    {
        var aProj = new ComputeApproxOnPolarSurface();
        Assert.False(aProj.IsDone);
        EngineAssert.ThrowsAny<NoSuchObjectException>(() => aProj.Curve2d());
        Assert.Equal(-1.0, aProj.Tolerance());
    }
}
