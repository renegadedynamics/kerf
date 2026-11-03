// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.AppParCurves_Constraint;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// ProjLib_ProjectedCurve on every branch of its Perform (elementary projectors, the ComputeApprox fallback, the
/// polar approximation on B-spline surfaces, ProjLib_CompProjectedCurve with Approx_CurveOnSurface on other
/// surfaces, the trimming at poles and the extension to them, the placing into the surface's period), with
/// ProjLib.MakePCurveOfType and the frozen results.
/// </summary>
public class ProjLib_ProjectedCurveTests
{
    private static readonly CoordinateSystem Std = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0));

    private static readonly CoordinateSystem Tilted = new(new Point3D(1.0, -2.0, 0.5), new Direction3D(0.2, -0.3, 1.0), new Direction3D(1.0, 0.0, -0.2));

    // The pcurve mapped through the surface is the 3D curve at the same parameter, within theTol.
    private static void CheckOnCurve(Geom2d_Curve thePCurve, Adaptor3d_Surface theSurf, Adaptor3d_Curve theCurve,
                                     double theFirst, double theLast, double theTol, int theNb = 50)
    {
        for (int i = 0; i <= theNb; i++)
        {
            double t = theFirst + (theLast - theFirst) * i / theNb;
            Point2D aUV = thePCurve.Value(t);
            double aDist = theSurf.Value(aUV.X(), aUV.Y()).Distance(theCurve.Value(t));
            Assert.True(aDist <= theTol, $"distance {aDist} at t = {t} (tolerance {theTol})");
        }
    }

    // A bicubic Bezier-like B-spline surface with a bump, on [0, 1] x [0, 1].
    private static Geom_BSplineSurface BumpSurface()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double z = (i == 2 || i == 3) && (j == 2 || j == 3) ? 1.5 : 0.1 * (i - j);
                aPoles[i, j] = new Point3D(i - 1, j - 1, z);
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    // A curve lying exactly on a surface: the surface along a 2D segment.
    private static Adaptor3d_CurveOnSurface CurveOn(Adaptor3d_Surface theSurf, in Point2D theFrom, in Point2D theTo)
    {
        var aLine = new Geom2d_Line(theFrom, new Direction2D(new Vector2D(theFrom, theTo)));
        var aSeg = new Geom2dAdaptor_Curve(aLine, 0.0, theFrom.Distance(theTo));
        return new Adaptor3d_CurveOnSurface(aSeg, theSurf);
    }

    [Fact]
    public void Plane_Line_IsLine()
    {
        var aS = new GeomAdaptor_Surface(new Geom_Plane(Tilted));
        var aLin = new Line3D(new Point3D(1.0, 2.0, 3.0), new Direction3D(1.0, 1.0, 0.2));
        var aC = new GeomAdaptor_Curve(new Geom_Line(aLin), -2.0, 3.0);
        var aProj = new ProjLib_ProjectedCurve(aS, aC);
        Assert.Equal(CurveKind.Line, aProj.GetType());
        Line2D aExpected = ProjLib.Project(new Plane3D(Tilted), aLin);
        Assert.Equal(aExpected.Location().X(), aProj.Line().Location().X(), 1.0e-14);
        Assert.Equal(aExpected.Direction().Y(), aProj.Line().Direction().Y(), 1.0e-14);
        Assert.Equal(-2.0, aProj.FirstParameter());
        Assert.Equal(3.0, aProj.LastParameter());
        Assert.False(aProj.IsPeriodic());
        Assert.Same(aS, aProj.GetSurface());
        Assert.Same(aC, aProj.GetCurve());

        ProjLib.MakePCurveOfType(aProj, out Geom2d_Curve aC2d);
        Assert.IsType<Geom2d_Line>(aC2d);
    }

    [Fact]
    public void Plane_Conics_MakePCurveOfType()
    {
        var aPln = new Geom_Plane(Std);
        var aS = new GeomAdaptor_Surface(aPln);
        var anAx = new Frame3D(new Point3D(1.0, 2.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 1.0, 0.0));
        (Geom_Curve, Type)[] aCases =
        [
            (new Geom_Circle(anAx, 2.0), typeof(Geom2d_Circle)),
            (new Geom_Ellipse(anAx, 3.0, 1.0), typeof(Geom2d_Ellipse)),
            (new Geom_Parabola(anAx, 0.5), typeof(Geom2d_Parabola)),
            (new Geom_Hyperbola(anAx, 3.0, 1.0), typeof(Geom2d_Hyperbola)),
        ];
        foreach ((Geom_Curve aCurve, Type aType) in aCases)
        {
            var aC = new GeomAdaptor_Curve(aCurve, -1.0, 1.0);
            var aProj = new ProjLib_ProjectedCurve(aS, aC);
            ProjLib.MakePCurveOfType(aProj, out Geom2d_Curve aC2d);
            Assert.IsType(aType, aC2d);
            CheckOnCurve(aC2d, aS, aC, -1.0, 1.0, 1.0e-12);
        }
    }

    [Fact]
    public void Plane_BSpline_FrozenAndShared()
    {
        var aS = new GeomAdaptor_Surface(new Geom_Plane(Tilted));
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 1);
        aPoles[2] = new Point3D(1, 2, -1);
        aPoles[3] = new Point3D(3, 2, 2);
        aPoles[4] = new Point3D(4, -1, 0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        var aBS = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
        var aC = new GeomAdaptor_Curve(aBS);
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-7);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.Equal(3, aProj.Degree());
        Assert.Equal(4, aProj.NbPoles());
        Assert.Equal(2, aProj.NbKnots());
        Assert.False(aProj.IsRational());

        Geom2d_BSplineCurve aRes = aProj.BSpline();
        Assert.True(aRes.IsFrozen);
        Assert.Same(aRes, aProj.BSpline());
        ProjLib.MakePCurveOfType(aProj, out Geom2d_Curve aC2d);
        Assert.Same(aRes, aC2d);
        // The plane's pcurve of a B-spline is the projection of its poles.
        for (int i = 1; i <= 4; i++)
        {
            Point2D aExpected = ProjLib.Project(new Plane3D(Tilted), aPoles[i]);
            Assert.Equal(aExpected.X(), aRes.Pole(i).X(), 1.0e-12);
            Assert.Equal(aExpected.Y(), aRes.Pole(i).Y(), 1.0e-12);
        }

        // The shallow copy shares the result.
        var aCopy = (ProjLib_ProjectedCurve)aProj.ShallowCopy();
        Assert.Same(aRes, aCopy.BSpline());
        Assert.Equal(aProj.GetTolerance(), aCopy.GetTolerance());
    }

    [Fact]
    public void Plane_Bezier_IsBezier_MakePCurveOfTypeThrows()
    {
        var aS = new GeomAdaptor_Surface(new Geom_Plane(Std));
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(0, 0, 1);
        aPoles[2] = new Point3D(1, 2, -1);
        aPoles[3] = new Point3D(3, 2, 2);
        var aC = new GeomAdaptor_Curve(new Geom_BezierCurve(aPoles));
        var aProj = new ProjLib_ProjectedCurve(aS, aC);
        Assert.Equal(CurveKind.BezierCurve, aProj.GetType());
        Assert.Equal(2, aProj.Degree());
        Assert.True(aProj.Bezier().IsFrozen);
        Assert.Throws<NoSuchObjectException>(() => aProj.NbKnots());
        Assert.Throws<MethodNotImplementedException>(() => ProjLib.MakePCurveOfType(aProj, out _));
    }

    [Fact]
    public void NotImplementedQueries_Throw()
    {
        var aS = new GeomAdaptor_Surface(new Geom_Plane(Std));
        var aC = new GeomAdaptor_Curve(new Geom_Line(new Line3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))), 0.0, 1.0);
        var aProj = new ProjLib_ProjectedCurve(aS, aC);
        Assert.Throws<MethodNotImplementedException>(() => aProj.Continuity());
        Assert.Throws<MethodNotImplementedException>(() => aProj.NbIntervals(Continuity.C0));
        Assert.Throws<MethodNotImplementedException>(() => aProj.Intervals(new NCollection_Array1<double>(1, 2), Continuity.C0));
        Assert.Throws<MethodNotImplementedException>(() => aProj.IsClosed());
        Assert.Throws<MethodNotImplementedException>(() => aProj.Period());
        Assert.Throws<MethodNotImplementedException>(() => aProj.Value(0.5));
        Assert.Throws<MethodNotImplementedException>(() => aProj.D0(0.5, out _));
        Assert.Throws<MethodNotImplementedException>(() => aProj.D1(0.5, out _, out _));
        Assert.Throws<MethodNotImplementedException>(() => aProj.D2(0.5, out _, out _, out _));
        Assert.Throws<MethodNotImplementedException>(() => aProj.D3(0.5, out _, out _, out _, out _));
        Assert.Throws<MethodNotImplementedException>(() => aProj.DN(0.5, 1));
        Assert.Throws<MethodNotImplementedException>(() => aProj.Resolution(1.0));
        Assert.Throws<MethodNotImplementedException>(() => aProj.Trim(0.0, 1.0, 1.0e-7));
        Assert.Throws<NoSuchObjectException>(() => aProj.Circle());
        Assert.Throws<NoSuchObjectException>(() => aProj.Degree());
    }

    [Fact]
    public void Cylinder_Parallel_IsLineInFirstPeriod()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        var aS = new GeomAdaptor_Surface(aCyl);
        var aCirc = new Geom_Circle(new Frame3D(new Point3D(0, 0, 1.5), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0);
        double aFirst = 2.0 * M_PI + 0.5, aLast = 2.0 * M_PI + 1.5;
        var aC = new GeomAdaptor_Curve(aCirc, aFirst, aLast);
        var aProj = new ProjLib_ProjectedCurve(aS, aC);
        Assert.Equal(CurveKind.Line, aProj.GetType());
        // UFrame put the start into [0, 2 pi[.
        Point2D aStart = global::Kerf.Engine.Geometry.ElCLib.Value(aFirst, aProj.Line());
        Assert.InRange(aStart.X(), 0.0, 2.0 * M_PI);
        Assert.Equal(0.5, aStart.X(), 1.0e-12);
        Assert.Equal(1.5, aStart.Y(), 1.0e-12);
        ProjLib.MakePCurveOfType(aProj, out Geom2d_Curve aC2d);
        CheckOnCurve(aC2d, aS, aC, aFirst, aLast, 1.0e-12);
    }

    [Fact]
    public void Cylinder_Ellipse_ComputeApproxFallback()
    {
        var aCyl = new Geom_CylindricalSurface(Std, 2.0);
        var aS = new GeomAdaptor_Surface(aCyl);
        // A planar section of the cylinder: ProjLib_Cylinder doesn't project ellipses, ComputeApprox does.
        var anEl = new Geom_Ellipse(new Frame3D(new Point3D(0, 0, 1.0), new Direction3D(0.0, -0.6, 0.8), new Direction3D(0.0, 0.8, 0.6)), 2.5, 2.0);
        var aC = new GeomAdaptor_Curve(anEl, 0.3, 2.5);
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-6);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.True(aProj.GetTolerance() < 1.0e-4);
        Geom2d_BSplineCurve aRes = aProj.BSpline();
        CheckOnCurve(aRes, aS, aC, 0.3, 2.5, 1.0e-4);
        // In the surface's period.
        for (int i = 0; i <= 20; i++)
        {
            Assert.InRange(aRes.Value(0.3 + 2.2 * i / 20).X(), -1.0e-9, 2.0 * M_PI + 1.0e-9);
        }
    }

    [Fact]
    public void Sphere_Parallel_SetInBounds()
    {
        var aSph = new Geom_SphericalSurface(Std, 3.0);
        var aS = new GeomAdaptor_Surface(aSph);
        double aV = 0.4;
        var aCirc = new Geom_Circle(new Frame3D(new Point3D(0, 0, 3.0 * Math.Sin(aV)), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3.0 * Math.Cos(aV));
        var aC = new GeomAdaptor_Curve(aCirc, 1.0, 2.0);
        var aProj = new ProjLib_ProjectedCurve(aS, aC);
        Assert.Equal(CurveKind.Line, aProj.GetType());
        ProjLib.MakePCurveOfType(aProj, out Geom2d_Curve aC2d);
        CheckOnCurve(aC2d, aS, aC, 1.0, 2.0, 1.0e-12);
        Assert.Equal(aV, aC2d.Value(1.5).Y(), 1.0e-12);
    }

    [Fact]
    public void Sphere_BSplineThroughPole_TrimmedAndExtended()
    {
        double aR = 2.0;
        var aSph = new Geom_SphericalSurface(Std, aR);
        var aS = new GeomAdaptor_Surface(aSph);
        // A quarter meridian from the equator to the north pole, as a B-spline (not recognised by ProjLib_Sphere).
        var aCirc = new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 0), new Direction3D(1, 0, 0)), aR);
        Geom_BSplineCurve aBS = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aCirc, 0.0, M_PI / 2.0));
        var aC = new GeomAdaptor_Curve(aBS);
        double aFirst = aBS.FirstParameter(), aLast = aBS.LastParameter();
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-6);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        // The projected curve was trimmed before the pole...
        Assert.True(aProj.GetCurve()!.LastParameter() < aLast);
        Geom2d_BSplineCurve aRes = aProj.BSpline();
        // ... and the pcurve extended back to the whole range, ending on the pole's line V = pi/2.
        Assert.Equal(aFirst, aRes.FirstParameter(), 1.0e-9);
        Assert.Equal(aLast, aRes.LastParameter(), 1.0e-9);
        Assert.Equal(M_PI / 2.0, aRes.Value(aLast).Y(), 1.0e-6);
        CheckOnCurve(aRes, aS, aC, aFirst, aLast, 1.0e-3);
    }

    [Fact]
    public void BSplineSurface_PolarApproximation()
    {
        var aSurf = BumpSurface();
        var aS = new GeomAdaptor_Surface(aSurf);
        var aC = CurveOn(aS, new Point2D(0.1, 0.2), new Point2D(0.85, 0.7));
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-6);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Geom2d_BSplineCurve aRes = aProj.BSpline();
        Assert.True(aRes.IsFrozen);
        CheckOnCurve(aRes, aS, aC, aC.FirstParameter(), aC.LastParameter(), 1.0e-4);
    }

    [Fact]
    public void BSplineSurface_Settings()
    {
        var aSurf = BumpSurface();
        var aS = new GeomAdaptor_Surface(aSurf);
        var aC = CurveOn(aS, new Point2D(0.05, 0.9), new Point2D(0.9, 0.1));
        var aProj = new ProjLib_ProjectedCurve(aS);
        aProj.Load(1.0e-5);
        aProj.SetDegree(2, 6);
        aProj.SetMaxSegments(50);
        aProj.SetBndPnt(AppParCurves_PassPoint);
        aProj.SetMaxDist(1.0);
        aProj.Perform(aC);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.True(aProj.Degree() <= 6);
        CheckOnCurve(aProj.BSpline(), aS, aC, aC.FirstParameter(), aC.LastParameter(), 1.0e-3);
    }

    // A surface of revolution of a Bezier meridian in the XZ plane.
    private static Geom_SurfaceOfRevolution Revolution(bool theFromAxis)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(theFromAxis ? 0.0 : 0.5, 0.0, 0.0);
        aPoles[2] = new Point3D(1.5, 0.0, 0.5);
        aPoles[3] = new Point3D(1.0, 0.0, 1.5);
        aPoles[4] = new Point3D(1.8, 0.0, 2.5);
        return new Geom_SurfaceOfRevolution(new Geom_BezierCurve(aPoles), new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
    }

    [Fact]
    public void Revolution_CompProjectedCurve()
    {
        var aS = new GeomAdaptor_Surface(Revolution(false));
        Assert.Equal(SurfaceKind.SurfaceOfRevolution, aS.GetType());
        var aC = CurveOn(aS, new Point2D(0.3, 0.1), new Point2D(1.6, 0.9));
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-6);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.True(aProj.GetTolerance() < 1.0e-4);
        CheckOnCurve(aProj.BSpline(), aS, aC, aC.FirstParameter(), aC.LastParameter(), 1.0e-4);
    }

    [Fact]
    public void Revolution_CurveFromTheAxis_TrimmedAndExtended()
    {
        var aS = new GeomAdaptor_Surface(Revolution(true));
        // Starts at V = 0, on the axis.
        var aC = CurveOn(aS, new Point2D(0.7, 0.0), new Point2D(1.5, 0.8));
        double aFirst = aC.FirstParameter(), aLast = aC.LastParameter();
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-6);
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.True(aProj.GetCurve()!.FirstParameter() > aFirst);
        Geom2d_BSplineCurve aRes = aProj.BSpline();
        Assert.Equal(aFirst, aRes.FirstParameter(), 1.0e-9);
        Assert.Equal(aLast, aRes.LastParameter(), 1.0e-9);
        // The extension ends on the meridian's parameter of the axis point.
        Assert.Equal(0.0, aRes.Value(aFirst).Y(), 1.0e-6);
        CheckOnCurve(aRes, aS, aC, aFirst, aLast, 1.0e-3);
    }

    [Fact]
    public void Revolution_NoProjection_LeavesNotDone()
    {
        var aS = new GeomAdaptor_Surface(Revolution(false));
        // Far from the surface: CompProjectedCurve finds no part within MaxDist.
        var aC = new GeomAdaptor_Curve(new Geom_Line(new Line3D(new Point3D(50, 50, 50), new Direction3D(1, 0, 0))), 0.0, 1.0);
        var aProj = new ProjLib_ProjectedCurve(aS, aC, 1.0e-6);
        // OCCT's empty projector's type.
        Assert.Equal(CurveKind.BSplineCurve, aProj.GetType());
        Assert.Null((object?)aProj.BSpline());
    }

    [Fact]
    public void IntDataMap_IteratesInOcctBucketOrder()
    {
        // 101 buckets: key k goes to bucket k % 101 + 1, so -1 (sign-extended) goes to bucket 79, after 0 and 1,
        // and shares it with 78 (the newer key first).
        var aMap = new ProjLib_ProjectedCurve.IntDataMap();
        aMap.Bind(-1, 5);
        aMap.Bind(1, 5);
        aMap.Bind(78, 2);
        aMap.Bind(0, 1);
        aMap.ChangeFind(0)++;
        Assert.True(aMap.IsBound(78));
        Assert.False(aMap.IsBound(2));
        Assert.Equal([(0, 2), (1, 5), (78, 2), (-1, 5)], aMap.Pairs().ToArray());
    }

    [Fact]
    public void Projector_BSplineFrozenOnHandOut()
    {
        var aProj = new ProjLib_Projector();
        var aPoles = new NCollection_Array1<Point2D>(1, 2);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        var aBz = new Geom2d_BezierCurve(aPoles);
        aProj.SetBezier(aBz);
        Assert.False(aBz.IsFrozen);
        Assert.Same(aBz, aProj.Bezier());
        Assert.True(aBz.IsFrozen);
    }
}
