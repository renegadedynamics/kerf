// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// ProjLib_ComputeApprox: B-spline and Bezier curves on planes (pole projection), and curves lying on planes,
/// cylinders, cones, spheres and tori (approximation, with the seam handling of Function_SetUVBounds). Each result,
/// mapped back through the surface, is the curve at the same parameter.
/// </summary>
public class ProjLib_ComputeApproxTests
{
    private static readonly gp_Ax3 Std = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 0.0, 0.0));

    private static readonly gp_Ax3 Tilted = new(new gp_Pnt(1.0, -2.0, 0.5), new gp_Dir(0.2, -0.3, 1.0), new gp_Dir(1.0, 0.0, -0.2));

    // The 2D curve mapped back through the surface lies on the 3D curve at the same parameter; consecutive samples
    // are close in the parametric space (no jump across a seam).
    private static void CheckOnCurve(Geom2d_Curve? thePCurve, Adaptor3d_Surface theSurf, Adaptor3d_Curve theCurve,
                                     double theTol, int theNb = 60)
    {
        Assert.NotNull(thePCurve);
        double aFirst = theCurve.FirstParameter(), aLast = theCurve.LastParameter();
        Assert.Equal(aFirst, thePCurve.FirstParameter(), 1.0e-9);
        Assert.Equal(aLast, thePCurve.LastParameter(), 1.0e-9);
        gp_Pnt2d aPrev = thePCurve.Value(aFirst);
        for (int i = 0; i <= theNb; i++)
        {
            double t = aFirst + (aLast - aFirst) * i / theNb;
            gp_Pnt2d aUV = thePCurve.Value(t);
            gp_Pnt aP = theSurf.Value(aUV.X(), aUV.Y());
            double aDist = aP.Distance(theCurve.Value(t));
            Assert.True(aDist <= theTol, $"distance {aDist} at t = {t}, tol {theTol}, {aFirst}..{aLast}, knots {(thePCurve as Geom2d_BSplineCurve)?.NbKnots()}");
            Assert.True(aUV.Distance(aPrev) < 1.0, $"jump at t = {t}");
            aPrev = aUV;
        }
    }

    private static Geom2d_BSplineCurve Approx(Geom_Curve theC, Geom_Surface theS, double theTol = 1.0e-6)
    {
        var aC = new GeomAdaptor_Curve(theC);
        var aS = new GeomAdaptor_Surface(theS);
        var aProj = new ProjLib_ComputeApprox(aC, aS, theTol);
        Assert.Null(aProj.Bezier());
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.True(aProj.Tolerance() < 1.0e-4);
        // Within the reached tolerance (an estimate from the 2D error, so with a margin).
        CheckOnCurve(aRes, aS, aC, Max(1.0e-5, 2.0 * aProj.Tolerance()));
        return aRes;
    }

    private static Geom_BSplineCurve SpaceBSpline(bool theRational, bool thePeriodic)
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 5);
        aPoles[1] = new gp_Pnt(0, 0, 1);
        aPoles[2] = new gp_Pnt(1, 2, -1);
        aPoles[3] = new gp_Pnt(3, 2, 2);
        aPoles[4] = new gp_Pnt(4, -1, 0);
        aPoles[5] = new gp_Pnt(2, -2, 3);
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
            return theRational ? new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, 3, true)
                               : new Geom_BSplineCurve(aPoles, aKnots, aMults, 3, true);
        }
        else
        {
            var aKnots = new NCollection_Array1<double>(1, 3);
            var aMults = new NCollection_Array1<int>(1, 3);
            aKnots[1] = 0.0; aKnots[2] = 0.4; aKnots[3] = 1.0;
            aMults[1] = 4; aMults[2] = 1; aMults[3] = 4;
            return theRational ? new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, 3)
                               : new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void Plane_BSpline_ProjectsPoles(bool theRational, bool thePeriodic)
    {
        Geom_BSplineCurve aC = SpaceBSpline(theRational, thePeriodic);
        var aPln = new Geom_Plane(Tilted);
        var aProj = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(aC), new GeomAdaptor_Surface(aPln), 1.0e-7);
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.Null(aProj.Bezier());
        Assert.Equal(aC.NbPoles(), aRes.NbPoles());
        Assert.Equal(aC.Degree(), aRes.Degree());
        Assert.Equal(thePeriodic, aRes.IsPeriodic());
        Assert.Equal(theRational, aRes.IsRational());
        // No approximation: the tolerance is unchanged.
        Assert.Equal(1.0e-7, aProj.Tolerance());
        gp_Pln aP = aPln.Pln();
        for (int i = 0; i <= 20; i++)
        {
            double t = aC.FirstParameter() + (aC.LastParameter() - aC.FirstParameter()) * i / 20;
            ElSLib.Parameters(aP, aC.Value(t), out double u, out double v);
            gp_Pnt2d aUV = aRes.Value(t);
            Assert.Equal(u, aUV.X(), 1.0e-12);
            Assert.Equal(v, aUV.Y(), 1.0e-12);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Plane_Bezier_GivesBezier(bool theRational)
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 3, 1);
        aPoles[3] = new gp_Pnt(3, 3, -2);
        aPoles[4] = new gp_Pnt(4, 0, 5);
        var aWeights = new NCollection_Array1<double>(1, 4);
        aWeights[1] = 1.0; aWeights[2] = 2.0; aWeights[3] = 0.5; aWeights[4] = 1.0;
        Geom_BezierCurve aC = theRational ? new Geom_BezierCurve(aPoles, aWeights) : new Geom_BezierCurve(aPoles);
        var aPln = new Geom_Plane(Tilted);
        var aProj = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(aC), new GeomAdaptor_Surface(aPln), 1.0e-7);
        Assert.Null(aProj.BSpline());
        Geom2d_BezierCurve? aRes = aProj.Bezier();
        Assert.NotNull(aRes);
        Assert.Equal(theRational, aRes.IsRational());
        gp_Pln aP = aPln.Pln();
        for (int i = 0; i <= 10; i++)
        {
            double t = i / 10.0;
            ElSLib.Parameters(aP, aC.Value(t), out double u, out double v);
            Assert.Equal(u, aRes.Value(t).X(), 1.0e-12);
            Assert.Equal(v, aRes.Value(t).Y(), 1.0e-12);
        }
    }

    [Fact]
    public void Plane_OffsetCurve_Approximated()
    {
        var aBase = new Geom_Circle(new gp_Ax2(Tilted.Location(), Tilted.Direction(), Tilted.XDirection()), 2.0);
        var aC = new Geom_TrimmedCurve(new Geom_OffsetCurve(aBase, 0.5, Tilted.Direction()), 0.3, 4.0);
        Approx(aC, new Geom_Plane(Tilted));
    }

    [Fact]
    public void Plane_Circle_Approximated()
    {
        var aC = new Geom_Circle(new gp_Ax2(Tilted.Location(), Tilted.Direction(), Tilted.XDirection()), 2.0);
        Approx(aC, new Geom_Plane(Tilted));
    }

    [Fact]
    public void Cylinder_VIsoCrossingSeam()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        var aC = new Geom_TrimmedCurve(aCyl.VIso(1.5), 5.0, 7.5);
        Geom2d_BSplineCurve aRes = Approx(aC, aCyl);
        // The pcurve is continuous across the seam: from U = 5 to U = 7.5.
        Assert.Equal(5.0, aRes.Value(5.0).X(), 1.0e-6);
        Assert.Equal(7.5, aRes.Value(7.5).X(), 1.0e-6);
    }

    [Fact]
    public void Cylinder_ClosedEllipse()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        // The section of the cylinder by a tilted plane through the axis: an ellipse of minor radius 2.
        gp_Dir aN = new(0.0, -0.5, 1.0);
        double aMaj = 2.0 / aN.Z();
        var aC = new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 1), aN, new gp_Dir(0.0, 1.0, 0.5)), aMaj, 2.0);
        Approx(aC, aCyl);
    }

    [Fact]
    public void Cylinder_OpenEllipseArc()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        gp_Dir aN = new(0.0, -0.5, 1.0);
        double aMaj = 2.0 / aN.Z();
        var anEl = new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 1), aN, new gp_Dir(0.0, 1.0, 0.5)), aMaj, 2.0);
        Approx(new Geom_TrimmedCurve(anEl, 1.0, 4.0), aCyl);
        Approx(new Geom_TrimmedCurve(anEl, 4.0, 7.0), aCyl);
    }

    // A B-spline approximation, within 1e-9, of a helix on the cylinder (the 2D line through (u0, 0.12) with
    // direction (1, 0.4), on [0, len]). The method assumes the curve lies on the surface (Function_D1's tangents).
    private static Geom_BSplineCurve Helix(Geom_CylindricalSurface theCyl, double theU0, double theLen)
    {
        var aLine = new Geom2d_Line(new gp_Pnt2d(theU0, 0.12), new gp_Dir2d(1.0, 0.4));
        var aCOnS = new Adaptor3d_CurveOnSurface(new Geom2dAdaptor_Curve(aLine, 0.0, theLen), new GeomAdaptor_Surface(theCyl));
        Geom_BSplineCurve? aC = new GeomConvert_ApproxCurve(aCOnS, 1.0e-9, GeomAbs_Shape.GeomAbs_C2, 100, 12).Curve();
        Assert.NotNull(aC);
        return aC;
    }

    [Fact]
    public void Cylinder_BSplineHelixCrossingSeam()
    {
        var aCyl = new Geom_CylindricalSurface(Tilted, 1.5);
        // From U = 4 over 5.6 radians: across the seam.
        Geom_BSplineCurve aC = Helix(aCyl, 4.0, 6.0);
        Geom2d_BSplineCurve aRes = Approx(aC, aCyl);
        // Continuous across the seam: the U span is the helix's (6 cos(atan 0.4) = 5.57), not wrapped.
        Assert.Equal(6.0 / Math.Sqrt(1.16), aRes.Value(aC.LastParameter()).X() - aRes.Value(aC.FirstParameter()).X(), 1.0e-6);
    }

    [Fact]
    public void Cylinder_HelixOfMoreThanATurn_WrongResult_OcctBug()
    {
        // OCCT: Function_SetUVBounds finds the U range of the curve (here 8.4 radians), then, as it isn't within
        // [0, 2 PI], replaces it with a window of width 2 PI around its middle: the parametric image wraps inside
        // the curve, and the approximation bridges the jump. The reached tolerance is tiny, but the result is off by
        // up to the diameter.
        var aCyl = new Geom_CylindricalSurface(Std, 1.5);
        Geom_BSplineCurve aC = Helix(aCyl, 0.3, 9.0);
        var aS = new GeomAdaptor_Surface(aCyl);
        var aProj = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(aC), aS, 1.0e-6);
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.True(aProj.Tolerance() < 1.0e-9);
        double aMax = 0.0;
        for (int i = 0; i <= 400; i++)
        {
            double t = 9.0 * i / 400;
            gp_Pnt2d aUV = aRes.Value(t);
            aMax = Max(aMax, aS.Value(aUV.X(), aUV.Y()).Distance(aC.Value(t)));
        }
        Assert.True(aMax > 2.9, $"max distance {aMax}");

        // Under a turn the projection is right.
        Geom_BSplineCurve aShort = Helix(aCyl, 0.3, 6.0);
        Approx(aShort, aCyl);
    }

    [Fact]
    public void Cone_Generator()
    {
        var aCone = new Geom_ConicalSurface(Std, M_PI / 6.0, 2.0);
        var aC = new Geom_TrimmedCurve(aCone.UIso(2.5), -1.0, 3.0);
        Approx(aC, aCone);
    }

    [Fact]
    public void Cone_ParallelCrossingSeam()
    {
        var aCone = new Geom_ConicalSurface(Tilted, M_PI / 5.0, 1.0);
        var aC = new Geom_TrimmedCurve(aCone.VIso(0.8), 4.5, 8.0);
        Approx(aC, aCone);
    }

    [Fact]
    public void Cone_WholeParallel()
    {
        var aCone = new Geom_ConicalSurface(Tilted, M_PI / 5.0, 1.0);
        Approx(aCone.VIso(-0.5), aCone);
    }

    [Fact]
    public void Cone_EllipseSection()
    {
        // A section of a cone x^2 + y^2 = (1 + z tan a)^2 by a slightly tilted plane is an ellipse; its points are
        // projected through ElSLib, so build the ellipse from the plane section directly.
        double aTan = Math.Tan(M_PI / 6.0);
        var aCone = new Geom_ConicalSurface(Std, M_PI / 6.0, 1.0);
        // The plane z = 0.2 x + 1: intersection points along x at y = 0.
        double x1 = (1.0 + aTan) / (1.0 - 0.2 * aTan), x2 = -(1.0 + aTan) / (1.0 + 0.2 * aTan);
        // Not an exact section away from the major axis; the ellipse branch of SetUVBounds only needs the type.
        gp_Pnt aCenter = new(0.5 * (x1 + x2), 0.0, 0.2 * 0.5 * (x1 + x2) + 1.0);
        gp_Dir aN = new(-0.2, 0.0, 1.0);
        var anEl = new Geom_Ellipse(new gp_Ax2(aCenter, aN, new gp_Dir(1.0, 0.0, 0.2)),
                                    0.5 * (x1 - x2) * Math.Sqrt(1.04), 1.0 + aTan * aCenter.Z());
        var aS = new GeomAdaptor_Surface(aCone);
        var aProj = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(anEl), aS, 1.0e-6);
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        gp_Cone aCo = aCone.Cone();
        for (int i = 0; i <= 40; i++)
        {
            double t = 2 * M_PI * i / 40;
            ElSLib.Parameters(aCo, anEl.Value(t), out double u, out double v);
            gp_Pnt2d aUV = aRes.Value(t);
            double du = aUV.X() - u;
            du -= 2 * M_PI * Math.Round(du / (2 * M_PI));
            Assert.True(Math.Abs(du) < 1.0e-4, $"u at t = {t}");
            Assert.Equal(v, aUV.Y(), 1.0e-4);
        }
    }

    [Fact]
    public void Sphere_Parallel()
    {
        var aSph = new Geom_SphericalSurface(Tilted, 3.0);
        Approx(aSph.VIso(0.6), aSph);
    }

    [Fact]
    public void Sphere_TiltedGreatCircle()
    {
        // Crosses the seam twice.
        var aSph = new Geom_SphericalSurface(Std, 3.0);
        var aC = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0.3, 0.2, 1.0)), 3.0);
        Approx(aC, aSph);
    }

    [Fact]
    public void Sphere_SmallCircleCrossingSeamArc()
    {
        var aSph = new Geom_SphericalSurface(Std, 2.0);
        gp_Dir aN = new(1.0, 0.1, 0.4);
        double aR = 0.8, aD = Math.Sqrt(4.0 - aR * aR);
        var aCirc = new Geom_Circle(new gp_Ax2(new gp_Pnt(aD * aN.X(), aD * aN.Y(), aD * aN.Z()), aN), aR);
        Approx(aCirc, aSph);
        Approx(new Geom_TrimmedCurve(aCirc, 1.0, 4.0), aSph);
    }

    [Fact]
    public void Sphere_Meridian_VCouture()
    {
        // A great circle through the poles: its plane contains the axis, so D / C is undefined (C = 0) and the
        // general branch handles it; the meridian arc avoids the poles.
        var aSph = new Geom_SphericalSurface(Std, 2.0);
        var aC = new Geom_TrimmedCurve(aSph.UIso(1.0), -1.2, 1.2);
        Approx(aC, aSph);
    }

    [Fact]
    public void Sphere_CircleThroughPole_VCouture()
    {
        // A small circle through the north pole: its plane passes through the pole (|D / C| = R), so VCouture is set
        // (the V window is then widened to +-1e100); past the pole, the image is mirrored (T = PI - T, S = PI + S).
        var aSph = new Geom_SphericalSurface(Std, 2.0);
        gp_Dir aN = new(1.0, 0.0, 1.0);
        // The plane through the pole (0, 0, 2) with normal aN; its circle has center at the foot from the origin.
        double aDist = 2.0 * aN.Z();
        gp_Pnt aCenter = new(aDist * aN.X(), aDist * aN.Y(), aDist * aN.Z());
        double aR = Math.Sqrt(4.0 - aDist * aDist);
        var aCirc = new Geom_Circle(new gp_Ax2(aCenter, aN, new gp_Dir(-1.0, 0.0, 1.0)), aR);
        Assert.True(aCirc.Value(0.0).Distance(new gp_Pnt(0, 0, 2)) < 1.0e-12);
        // An arc away from the pole (the pole is at parameter 0).
        var aC = new Geom_TrimmedCurve(aCirc, 0.5, 2 * M_PI - 0.5);
        var aS = new GeomAdaptor_Surface(aSph);
        var aProj = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(aC), aS, 1.0e-6);
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        CheckOnCurve(aRes, aS, new GeomAdaptor_Curve(aC), 1.0e-4);
    }

    [Fact]
    public void Sphere_BSplineCircle()
    {
        var aSph = new Geom_SphericalSurface(Std, 2.0);
        Geom_BSplineCurve aC = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aSph.VIso(-0.4), 1.0, 5.5));
        Approx(aC, aSph);
    }

    [Fact]
    public void Torus_Circles()
    {
        var aTor = new Geom_ToroidalSurface(Tilted, 3.0, 1.0);
        Approx(aTor.UIso(0.7), aTor);
        Approx(aTor.VIso(2.0), aTor);
        Approx(new Geom_TrimmedCurve(aTor.VIso(2.0), 5.0, 8.0), aTor);
        Approx(new Geom_TrimmedCurve(aTor.UIso(0.7), 5.0, 8.0), aTor);
    }

    [Fact]
    public void Torus_BSpline()
    {
        var aTor = new Geom_ToroidalSurface(Std, 3.0, 1.0);
        Geom_BSplineCurve aC = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aTor.VIso(5.5), 0.5, 4.0));
        Approx(aC, aTor);
    }

    [Fact]
    public void Results_FrozenWhenHandedOut()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        var aProj = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(aCyl.VIso(1.0)), new GeomAdaptor_Surface(aCyl), 1.0e-6);
        Geom2d_BSplineCurve? anInternal = aProj.InternalBSpline();
        Assert.NotNull(anInternal);
        Assert.False(anInternal.IsFrozen());
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.Same(anInternal, aRes);
        Assert.True(aRes!.IsFrozen());
        Assert.Same(aRes, aProj.BSpline());

        var aPoles = new NCollection_Array1<gp_Pnt>(1, 3);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 1, 1);
        aPoles[3] = new gp_Pnt(2, 0, 0);
        var aBez = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(new Geom_BezierCurve(aPoles)),
                                             new GeomAdaptor_Surface(new Geom_Plane(Std)), 1.0e-6);
        Assert.False(aBez.InternalBezier()!.IsFrozen());
        Assert.True(aBez.Bezier()!.IsFrozen());
        Assert.Same(aBez.InternalBezier(), aBez.Bezier());
    }

    [Fact]
    public void Settings_DegreeSegmentsAndBoundaryConstraint()
    {
        var aSph = new Geom_SphericalSurface(Std, 2.0);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0.3, 0.2, 1.0)), 2.0));
        var aS = new GeomAdaptor_Surface(aSph);

        var aProj = new ProjLib_ComputeApprox();
        Assert.Equal(Precision.PApproximation(), aProj.Tolerance());
        Assert.Null(aProj.BSpline());
        aProj.SetTolerance(1.0e-5);
        aProj.SetDegree(3, 4);
        aProj.SetMaxSegments(50);
        aProj.Perform(aC, aS);
        Geom2d_BSplineCurve? aRes = aProj.BSpline();
        Assert.NotNull(aRes);
        Assert.True(aRes.Degree() <= 4);
        CheckOnCurve(aRes, aS, aC, 1.0e-3);

        // PassPoint ends: the knots are removed where the tolerance allows (C1 smoothing).
        var aPass = new ProjLib_ComputeApprox();
        aPass.SetTolerance(1.0e-5);
        aPass.SetBndPnt(AppParCurves_PassPoint);
        aPass.Perform(aC, aS);
        Assert.NotNull(aPass.BSpline());
        // RemoveKnot's tolerance (the 2D error) isn't added to the reached tolerance.
        CheckOnCurve(aPass.BSpline(), aS, aC, 1.0e-3);

        // The tolerance argument of the constructor is at least PApproximation.
        var aTiny = new ProjLib_ComputeApprox(new GeomAdaptor_Curve(SpaceBSpline(false, false)),
                                              new GeomAdaptor_Surface(new Geom_Plane(Std)), 0.0);
        Assert.Equal(Precision.PApproximation(), aTiny.Tolerance());
    }

    [Fact]
    public void NonAnalyticSurface_Throws()
    {
        // Only planes and the elementary quadrics are handled; ProjLib_ProjectedCurve never calls it otherwise.
        Geom_BSplineSurface aS = GeomConvert.SurfaceToBSplineSurface(new Geom_RectangularTrimmedSurface(
            new Geom_CylindricalSurface(Std, 2.0), 0.0, 3.0, 0.0, 2.0));
        var aC = new Geom_TrimmedCurve(new Geom_CylindricalSurface(Std, 2.0).VIso(1.0), 0.5, 2.5);
        Assert.ThrowsAny<Standard_NoSuchObject>(() =>
            new ProjLib_ComputeApprox(new GeomAdaptor_Curve(aC), new GeomAdaptor_Surface(aS), 1.0e-6));
    }
}
