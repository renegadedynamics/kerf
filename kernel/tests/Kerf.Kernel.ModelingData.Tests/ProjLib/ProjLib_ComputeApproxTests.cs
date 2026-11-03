// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// ProjLib_ComputeApprox: B-spline and Bezier curves on planes (pole projection), and curves lying on planes,
/// cylinders, cones, spheres and tori (approximation, with the seam handling of Function_SetUVBounds). Each result,
/// mapped back through the surface, is the curve at the same parameter.
/// </summary>
public class ProjLib_ComputeApproxTests
{
    private static readonly CoordinateSystem Std = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0));

    private static readonly CoordinateSystem Tilted = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(0.2, -0.3, 1.0), new Direction3D(1.0, 0.0, -0.2));

    // The 2D curve mapped back through the surface lies on the 3D curve at the same parameter; consecutive samples
    // are close in the parametric space (no jump across a seam).
    private static void CheckOnCurve(Curve2D? thePCurve, SurfaceAdaptorBase theSurf, CurveAdaptorBase theCurve,
                                     double theTol, int theNb = 60)
    {
        Assert.NotNull(thePCurve);
        double aFirst = theCurve.FirstParameter(), aLast = theCurve.LastParameter();
        Assert.Equal(aFirst, thePCurve.FirstParameter(), 1.0e-9);
        Assert.Equal(aLast, thePCurve.LastParameter(), 1.0e-9);
        Point2D aPrev = thePCurve.Value(aFirst);
        for (int i = 0; i <= theNb; i++)
        {
            double t = aFirst + (aLast - aFirst) * i / theNb;
            Point2D aUV = thePCurve.Value(t);
            Point3D aP = theSurf.Value(aUV.X(), aUV.Y());
            double aDist = aP.Distance(theCurve.Value(t));
            Assert.True(aDist <= theTol, $"distance {aDist} at t = {t}, tol {theTol}, {aFirst}..{aLast}, knots {(thePCurve as BSplineCurve2D)?.KnotCount}");
            Assert.True(aUV.Distance(aPrev) < 1.0, $"jump at t = {t}");
            aPrev = aUV;
        }
    }

    private static BSplineCurve2D Approx(Curve3D theC, Surface theS, double theTol = 1.0e-6)
    {
        var aC = new CurveAdaptor(theC);
        var aS = new SurfaceAdaptor(theS);
        var aProj = new ComputeApprox(aC, aS, theTol);
        Assert.Null(aProj.Bezier());
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.True(aProj.Tolerance() < 1.0e-4);
        // Within the reached tolerance (an estimate from the 2D error, so with a margin).
        CheckOnCurve(aRes, aS, aC, Max(1.0e-5, 2.0 * aProj.Tolerance()));
        return aRes;
    }

    private static BSplineCurve3D SpaceBSpline(bool theRational, bool thePeriodic)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 5);
        aPoles[1] = new Point3D(0, 0, 1);
        aPoles[2] = new Point3D(1, 2, -1);
        aPoles[3] = new Point3D(3, 2, 2);
        aPoles[4] = new Point3D(4, -1, 0);
        aPoles[5] = new Point3D(2, -2, 3);
        var aWeights = new NCollection_Array1<double>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aWeights[i] = theRational ? 1.0 + 0.2 * i : 1.0;
        }
        if (thePeriodic)
        {
            var aKnots = new NCollection_Array1<double>(1, 6);
            var aMults = new NCollection_Array1<int>(1, 6);
            for (int i = 1; i <= 6; i++)
            {
                aKnots[i] = i - 1;
                aMults[i] = 1;
            }
            return theRational ? new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 3, true)
                               : new BSplineCurve3D(aPoles, aKnots, aMults, 3, true);
        }
        else
        {
            var aKnots = new NCollection_Array1<double>(1, 3);
            var aMults = new NCollection_Array1<int>(1, 3);
            aKnots[1] = 0.0; aKnots[2] = 0.4; aKnots[3] = 1.0;
            aMults[1] = 4; aMults[2] = 1; aMults[3] = 4;
            return theRational ? new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 3)
                               : new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void Plane_BSpline_ProjectsPoles(bool theRational, bool thePeriodic)
    {
        BSplineCurve3D aC = SpaceBSpline(theRational, thePeriodic);
        var aPln = new PlaneSurface(Tilted);
        var aProj = new ComputeApprox(new CurveAdaptor(aC), new SurfaceAdaptor(aPln), 1.0e-7);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.Null(aProj.Bezier());
        Assert.Equal(aC.PoleCount, aRes.PoleCount);
        Assert.Equal(aC.Degree(), aRes.Degree());
        Assert.Equal(thePeriodic, aRes.IsPeriodic);
        Assert.Equal(theRational, aRes.IsRational);
        // No approximation: the tolerance is unchanged.
        Assert.Equal(1.0e-7, aProj.Tolerance());
        Plane3D aP = aPln.Pln();
        for (int i = 0; i <= 20; i++)
        {
            double t = aC.FirstParameter() + (aC.LastParameter() - aC.FirstParameter()) * i / 20;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aP, aC.Value(t), out double u, out double v);
            Point2D aUV = aRes.Value(t);
            Assert.Equal(u, aUV.X(), 1.0e-12);
            Assert.Equal(v, aUV.Y(), 1.0e-12);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Plane_Bezier_GivesBezier(bool theRational)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 1);
        aPoles[3] = new Point3D(3, 3, -2);
        aPoles[4] = new Point3D(4, 0, 5);
        var aWeights = new NCollection_Array1<double>(1, 4);
        aWeights[1] = 1.0; aWeights[2] = 2.0; aWeights[3] = 0.5; aWeights[4] = 1.0;
        BezierCurve3D aC = theRational ? new BezierCurve3D(aPoles, aWeights) : new BezierCurve3D(aPoles);
        var aPln = new PlaneSurface(Tilted);
        var aProj = new ComputeApprox(new CurveAdaptor(aC), new SurfaceAdaptor(aPln), 1.0e-7);
        Assert.Null(aProj.BSpline());
        BezierCurve2D? aRes = aProj.Bezier();
        Assert.NotNull(aRes);
        Assert.Equal(theRational, aRes.IsRational);
        Plane3D aP = aPln.Pln();
        for (int i = 0; i <= 10; i++)
        {
            double t = i / 10.0;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aP, aC.Value(t), out double u, out double v);
            Assert.Equal(u, aRes.Value(t).X(), 1.0e-12);
            Assert.Equal(v, aRes.Value(t).Y(), 1.0e-12);
        }
    }

    [Fact]
    public void Plane_OffsetCurve_Approximated()
    {
        var aBase = new CircleCurve3D(new Frame3D(Tilted.Location(), Tilted.Direction(), Tilted.XDirection()), 2.0);
        var aC = new TrimmedCurve3D(new OffsetCurve3D(aBase, 0.5, Tilted.Direction()), 0.3, 4.0);
        Approx(aC, new PlaneSurface(Tilted));
    }

    [Fact]
    public void Plane_Circle_Approximated()
    {
        var aC = new CircleCurve3D(new Frame3D(Tilted.Location(), Tilted.Direction(), Tilted.XDirection()), 2.0);
        Approx(aC, new PlaneSurface(Tilted));
    }

    [Fact]
    public void Cylinder_VIsoCrossingSeam()
    {
        var aCyl = new CylindricalSurface(Std, 2.0);
        var aC = new TrimmedCurve3D(aCyl.VIso(1.5), 5.0, 7.5);
        BSplineCurve2D aRes = Approx(aC, aCyl);
        // The pcurve is continuous across the seam: from U = 5 to U = 7.5.
        Assert.Equal(5.0, aRes.Value(5.0).X(), 1.0e-6);
        Assert.Equal(7.5, aRes.Value(7.5).X(), 1.0e-6);
    }

    [Fact]
    public void Cylinder_ClosedEllipse()
    {
        var aCyl = new CylindricalSurface(Std, 2.0);
        // The section of the cylinder by a tilted plane through the axis: an ellipse of minor radius 2.
        Direction3D aN = new(0.0, -0.5, 1.0);
        double aMaj = 2.0 / aN.Z();
        var aC = new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 1), aN, new Direction3D(0.0, 1.0, 0.5)), aMaj, 2.0);
        Approx(aC, aCyl);
    }

    [Fact]
    public void Cylinder_OpenEllipseArc()
    {
        var aCyl = new CylindricalSurface(Std, 2.0);
        Direction3D aN = new(0.0, -0.5, 1.0);
        double aMaj = 2.0 / aN.Z();
        var anEl = new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 1), aN, new Direction3D(0.0, 1.0, 0.5)), aMaj, 2.0);
        Approx(new TrimmedCurve3D(anEl, 1.0, 4.0), aCyl);
        Approx(new TrimmedCurve3D(anEl, 4.0, 7.0), aCyl);
    }

    // A B-spline approximation, within 1e-9, of a helix on the cylinder (the 2D line through (u0, 0.12) with
    // direction (1, 0.4), on [0, len]). The method assumes the curve lies on the surface (Function_D1's tangents).
    private static BSplineCurve3D Helix(Surface theSurf, double theU0, double theLen)
        => Helix(theSurf, new Point2D(theU0, 0.12), new Direction2D(1.0, 0.4), theLen);

    // The same for the 2D line through theStart along theDir.
    private static BSplineCurve3D Helix(Surface theSurf, Point2D theStart, Direction2D theDir, double theLen)
    {
        var aLine = new LineCurve2D(theStart, theDir);
        var aCOnS = new CurveOnSurface(new CurveAdaptor2D(aLine, 0.0, theLen), new SurfaceAdaptor(theSurf));
        BSplineCurve3D? aC = new global::Kerf.Engine.Geometry.Conversion.ApproxCurve(aCOnS, 1.0e-9, Continuity.C2, 100, 12).Curve();
        Assert.NotNull(aC);
        return aC;
    }

    [Fact]
    public void Cylinder_BSplineHelixCrossingSeam()
    {
        var aCyl = new CylindricalSurface(Tilted, 1.5);
        // From U = 4 over 5.6 radians: across the seam.
        BSplineCurve3D aC = Helix(aCyl, 4.0, 6.0);
        BSplineCurve2D aRes = Approx(aC, aCyl);
        // Continuous across the seam: the U span is the helix's (6 cos(atan 0.4) = 5.57), not wrapped.
        Assert.Equal(6.0 / Math.Sqrt(1.16), aRes.Value(aC.LastParameter()).X() - aRes.Value(aC.FirstParameter()).X(), 1.0e-6);
    }

    // The pcurve of a curve winding more than a turn follows it: on the curve within the reached tolerance, no jump,
    // and its parametric span the analytic one. OCCT (occt-issues.md, unit 3E's ProjLib_ComputeApprox row) kept a
    // window of 2π around the middle of the curve's U range, so the image wrapped inside the curve and the
    // approximation bridged the jump: a tiny reached tolerance, a result off by up to the diameter (decision 1459).
    private static void CheckWinding(Surface theSurf, Point2D theStart, Direction2D theDir, double theLen)
    {
        BSplineCurve3D aC = Helix(theSurf, theStart, theDir, theLen);
        BSplineCurve2D aRes = Approx(aC, theSurf);
        Vector2D aSpan = new(aRes.Value(aC.FirstParameter()), aRes.Value(aC.LastParameter()));
        Assert.Equal(theLen * theDir.X(), aSpan.X(), 1.0e-6);
        Assert.Equal(theLen * theDir.Y(), aSpan.Y(), 1.0e-6);
    }

    [Fact]
    public void Cylinder_HelixOfMoreThanATurn_FollowsTheHelix()
    {
        var aCyl = new CylindricalSurface(Std, 1.5);
        // 8.4 radians, then three and a half turns, from either side of the seam
        CheckWinding(aCyl, new Point2D(0.3, 0.12), new Direction2D(1.0, 0.4), 9.0);
        CheckWinding(new CylindricalSurface(Tilted, 1.5), new Point2D(5.0, 0.12), new Direction2D(1.0, 0.2), 22.5);
        // turning the other way
        CheckWinding(aCyl, new Point2D(1.0, 0.12), new Direction2D(-1.0, 0.3), 15.0);

        // Under a turn the projection is as before.
        BSplineCurve3D aShort = Helix(aCyl, 0.3, 6.0);
        Approx(aShort, aCyl);
    }

    [Fact]
    public void Cone_SpiralOfMoreThanATurn_FollowsTheSpiral()
        => CheckWinding(new ConicalSurface(Tilted, M_PI / 6.0, 1.0), new Point2D(0.3, 0.5), new Direction2D(1.0, 0.1), 14.0);

    [Fact]
    public void Torus_CoilOfMoreThanATurn_FollowsTheCoil()
    {
        var aTor = new ToroidalSurface(Std, 3.0, 1.0);
        // around the tube five times over a third of the ring, then around the ring more than once
        CheckWinding(aTor, new Point2D(0.3, 0.12), new Direction2D(0.07, 1.0), 31.0);
        CheckWinding(aTor, new Point2D(0.3, 0.12), new Direction2D(1.0, 0.05), 8.0);
    }

    [Fact]
    public void Cone_Generator()
    {
        var aCone = new ConicalSurface(Std, M_PI / 6.0, 2.0);
        var aC = new TrimmedCurve3D(aCone.UIso(2.5), -1.0, 3.0);
        Approx(aC, aCone);
    }

    [Fact]
    public void Cone_ParallelCrossingSeam()
    {
        var aCone = new ConicalSurface(Tilted, M_PI / 5.0, 1.0);
        var aC = new TrimmedCurve3D(aCone.VIso(0.8), 4.5, 8.0);
        Approx(aC, aCone);
    }

    [Fact]
    public void Cone_WholeParallel()
    {
        var aCone = new ConicalSurface(Tilted, M_PI / 5.0, 1.0);
        Approx(aCone.VIso(-0.5), aCone);
    }

    [Fact]
    public void Cone_EllipseSection()
    {
        // A section of a cone x^2 + y^2 = (1 + z tan a)^2 by a slightly tilted plane is an ellipse; its points are
        // projected through ElSLib, so build the ellipse from the plane section directly.
        double aTan = Math.Tan(M_PI / 6.0);
        var aCone = new ConicalSurface(Std, M_PI / 6.0, 1.0);
        // The plane z = 0.2 x + 1: intersection points along x at y = 0.
        double x1 = (1.0 + aTan) / (1.0 - 0.2 * aTan), x2 = -(1.0 + aTan) / (1.0 + 0.2 * aTan);
        // Not an exact section away from the major axis; the ellipse branch of SetUVBounds only needs the type.
        Point3D aCenter = new(0.5 * (x1 + x2), 0.0, 0.2 * 0.5 * (x1 + x2) + 1.0);
        Direction3D aN = new(-0.2, 0.0, 1.0);
        var anEl = new EllipseCurve3D(new Frame3D(aCenter, aN, new Direction3D(1.0, 0.0, 0.2)),
                                    0.5 * (x1 - x2) * Math.Sqrt(1.04), 1.0 + aTan * aCenter.Z());
        var aS = new SurfaceAdaptor(aCone);
        var aProj = new ComputeApprox(new CurveAdaptor(anEl), aS, 1.0e-6);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Cone aCo = aCone.Cone();
        for (int i = 0; i <= 40; i++)
        {
            double t = 2 * M_PI * i / 40;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aCo, anEl.Value(t), out double u, out double v);
            Point2D aUV = aRes.Value(t);
            double du = aUV.X() - u;
            du -= 2 * M_PI * Math.Round(du / (2 * M_PI));
            Assert.True(Math.Abs(du) < 1.0e-4, $"u at t = {t}");
            Assert.Equal(v, aUV.Y(), 1.0e-4);
        }
    }

    [Fact]
    public void Sphere_Parallel()
    {
        var aSph = new SphericalSurface(Tilted, 3.0);
        Approx(aSph.VIso(0.6), aSph);
    }

    [Fact]
    public void Sphere_TiltedGreatCircle()
    {
        // Crosses the seam twice.
        var aSph = new SphericalSurface(Std, 3.0);
        var aC = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0.3, 0.2, 1.0)), 3.0);
        Approx(aC, aSph);
    }

    [Fact]
    public void Sphere_SmallCircleCrossingSeamArc()
    {
        var aSph = new SphericalSurface(Std, 2.0);
        Direction3D aN = new(1.0, 0.1, 0.4);
        double aR = 0.8, aD = Math.Sqrt(4.0 - aR * aR);
        var aCirc = new CircleCurve3D(new Frame3D(new Point3D(aD * aN.X(), aD * aN.Y(), aD * aN.Z()), aN), aR);
        Approx(aCirc, aSph);
        Approx(new TrimmedCurve3D(aCirc, 1.0, 4.0), aSph);
    }

    [Fact]
    public void Sphere_Meridian_VCouture()
    {
        // A great circle through the poles: its plane contains the axis, so D / C is undefined (C = 0) and the
        // general branch handles it; the meridian arc avoids the poles.
        var aSph = new SphericalSurface(Std, 2.0);
        var aC = new TrimmedCurve3D(aSph.UIso(1.0), -1.2, 1.2);
        Approx(aC, aSph);
    }

    [Fact]
    public void Sphere_CircleThroughPole_VCouture()
    {
        // A small circle through the north pole: its plane passes through the pole (|D / C| = R), so VCouture is set
        // (the V window is then widened to +-1e100); past the pole, the image is mirrored (T = PI - T, S = PI + S).
        var aSph = new SphericalSurface(Std, 2.0);
        Direction3D aN = new(1.0, 0.0, 1.0);
        // The plane through the pole (0, 0, 2) with normal aN; its circle has center at the foot from the origin.
        double aDist = 2.0 * aN.Z();
        Point3D aCenter = new(aDist * aN.X(), aDist * aN.Y(), aDist * aN.Z());
        double aR = Math.Sqrt(4.0 - aDist * aDist);
        var aCirc = new CircleCurve3D(new Frame3D(aCenter, aN, new Direction3D(-1.0, 0.0, 1.0)), aR);
        Assert.True(aCirc.Value(0.0).Distance(new Point3D(0, 0, 2)) < 1.0e-12);
        // An arc away from the pole (the pole is at parameter 0).
        var aC = new TrimmedCurve3D(aCirc, 0.5, 2 * M_PI - 0.5);
        var aS = new SurfaceAdaptor(aSph);
        var aProj = new ComputeApprox(new CurveAdaptor(aC), aS, 1.0e-6);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        CheckOnCurve(aRes, aS, new CurveAdaptor(aC), 1.0e-4);
    }

    [Fact]
    public void Sphere_BSplineCircle()
    {
        var aSph = new SphericalSurface(Std, 2.0);
        BSplineCurve3D aC = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aSph.VIso(-0.4), 1.0, 5.5));
        Approx(aC, aSph);
    }

    [Fact]
    public void Torus_Circles()
    {
        var aTor = new ToroidalSurface(Tilted, 3.0, 1.0);
        Approx(aTor.UIso(0.7), aTor);
        Approx(aTor.VIso(2.0), aTor);
        Approx(new TrimmedCurve3D(aTor.VIso(2.0), 5.0, 8.0), aTor);
        Approx(new TrimmedCurve3D(aTor.UIso(0.7), 5.0, 8.0), aTor);
    }

    [Fact]
    public void Torus_BSpline()
    {
        var aTor = new ToroidalSurface(Std, 3.0, 1.0);
        BSplineCurve3D aC = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(new TrimmedCurve3D(aTor.VIso(5.5), 0.5, 4.0));
        Approx(aC, aTor);
    }

    [Fact]
    public void Results_FrozenWhenHandedOut()
    {
        var aCyl = new CylindricalSurface(Std, 2.0);
        var aProj = new ComputeApprox(new CurveAdaptor(aCyl.VIso(1.0)), new SurfaceAdaptor(aCyl), 1.0e-6);
        BSplineCurve2D? anInternal = aProj.InternalBSpline();
        Assert.NotNull(anInternal);
        Assert.False(anInternal.IsFrozen);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.Same(anInternal, aRes);
        Assert.True(aRes!.IsFrozen);
        Assert.Same(aRes, aProj.BSpline());

        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 1);
        aPoles[3] = new Point3D(2, 0, 0);
        var aBez = new ComputeApprox(new CurveAdaptor(new BezierCurve3D(aPoles)),
                                             new SurfaceAdaptor(new PlaneSurface(Std)), 1.0e-6);
        Assert.False(aBez.InternalBezier()!.IsFrozen);
        Assert.True(aBez.Bezier()!.IsFrozen);
        Assert.Same(aBez.InternalBezier(), aBez.Bezier());
    }

    [Fact]
    public void Settings_DegreeSegmentsAndBoundaryConstraint()
    {
        var aSph = new SphericalSurface(Std, 2.0);
        var aC = new CurveAdaptor(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0.3, 0.2, 1.0)), 2.0));
        var aS = new SurfaceAdaptor(aSph);

        var aProj = new ComputeApprox();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.PApproximation(), aProj.Tolerance());
        Assert.Null(aProj.BSpline());
        aProj.SetTolerance(1.0e-5);
        aProj.SetDegree(3, 4);
        aProj.SetMaxSegments(50);
        aProj.Perform(aC, aS);
        BSplineCurve2D? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.True(aRes.Degree() <= 4);
        CheckOnCurve(aRes, aS, aC, 1.0e-3);

        // PassPoint ends: the knots are removed where the tolerance allows (C1 smoothing).
        var aPass = new ComputeApprox();
        aPass.SetTolerance(1.0e-5);
        aPass.SetBndPnt(PassPoint);
        aPass.Perform(aC, aS);
        Assert.NotNull(aPass.BSpline());
        // RemoveKnot's tolerance (the 2D error) isn't added to the reached tolerance.
        CheckOnCurve(aPass.BSpline(), aS, aC, 1.0e-3);

        // The tolerance argument of the constructor is at least PApproximation.
        var aTiny = new ComputeApprox(new CurveAdaptor(SpaceBSpline(false, false)),
                                              new SurfaceAdaptor(new PlaneSurface(Std)), 0.0);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.PApproximation(), aTiny.Tolerance());
    }

    [Fact]
    public void NonAnalyticSurface_Throws()
    {
        // Only planes and the elementary quadrics are handled; ProjLib_ProjectedCurve never calls it otherwise.
        BSplineSurface aS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new TrimmedSurface(
            new CylindricalSurface(Std, 2.0), 0.0, 3.0, 0.0, 2.0));
        var aC = new TrimmedCurve3D(new CylindricalSurface(Std, 2.0).VIso(1.0), 0.5, 2.5);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NoSuchObjectException>(() =>
            new ComputeApprox(new CurveAdaptor(aC), new SurfaceAdaptor(aS), 1.0e-6));
    }
}
