// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeCustom_Curve, Curve2d and Surface (unit 7A): their conversions, and the caller's frozen geometry, which OCCT
/// changes in place and the port copies first (decision 974).
/// </summary>
public class ShapeCustom_GeometryTests
{
    private static NCollection_Array1<T> Array<T>(params T[] theValues)
    {
        var anArray = new NCollection_Array1<T>(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            anArray[i + 1] = theValues[i];
        }
        return anArray;
    }

    // A non-periodic cubic B-spline closed by its end poles (a closed loop).
    private static BSplineCurve3D ClosedBSpline()
    {
        NCollection_Array1<Point3D> aPoles = Array(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0),
                                                  new Point3D(-5, 5, 0), new Point3D(0, 0, 0));
        return new BSplineCurve3D(aPoles, Array(0.0, 1.0, 2.0, 3.0), Array(4, 1, 1, 4), 3);
    }

    // An unclamped cubic B-spline whose last three poles repeat the first three: closed, with simple end knots, so OCCT
    // makes it periodic in place, without rearranging its knots.
    private static BSplineCurve3D WrappedBSpline()
    {
        NCollection_Array1<Point3D> aPoles = Array(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0),
                                                  new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0));
        return new BSplineCurve3D(aPoles, Array(0.0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10), Array(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), 3);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Curve_ConvertToPeriodic_OfAFrozenCurve_GivesANewPeriodicCurve_TheCallersIsUnchanged(bool theWrapped)
    {
        BSplineCurve3D aCurve = theWrapped ? WrappedBSpline() : ClosedBSpline();
        aCurve.Freeze();
        var aTool = new ShapeCustom_Curve(aCurve);
        Curve3D? aResult = aTool.ConvertToPeriodic(true);
        Assert.NotNull(aResult);
        Assert.True(aResult!.IsPeriodic);
        Assert.False(aCurve.IsPeriodic);
        Assert.NotSame(aCurve, aResult);
        // a second call converts the substituted (now periodic) curve: nothing to do
        Assert.Null(aTool.ConvertToPeriodic(false));
    }

    [Fact]
    public void Curve_ConvertToPeriodic_OfAnOpenOrNonBSplineCurve_IsNull()
    {
        Assert.Null(new ShapeCustom_Curve(new CircleCurve3D(new Frame3D(), 3)).ConvertToPeriodic(false));
        NCollection_Array1<Point3D> aPoles = Array(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0));
        var anOpen = new BSplineCurve3D(aPoles, Array(0.0, 1.0), Array(4, 4), 3);
        Assert.Null(new ShapeCustom_Curve(anOpen).ConvertToPeriodic(false));
    }

    [Fact]
    public void Curve2d_IsLinear_AndConvertToLine2d()
    {
        double aDeviation = -1;
        Assert.True(ShapeCustom_Curve2d.IsLinear(Array(new Point2D(0, 0), new Point2D(1, 1e-8), new Point2D(3, 0)), 1e-6, ref aDeviation));
        Assert.Equal(1e-8, aDeviation, 12);
        aDeviation = -1;
        Assert.False(ShapeCustom_Curve2d.IsLinear(Array(new Point2D(0, 0), new Point2D(1, 1e-3), new Point2D(3, 0)), 1e-6, ref aDeviation));
        Assert.Equal(-1, aDeviation); // left unchanged
        Assert.False(ShapeCustom_Curve2d.IsLinear(Array(new Point2D(0, 0)), 1e-6, ref aDeviation));

        // a linear B-spline from (1, 1) to (5, 4), parametrised on [2, 7]
        var aBSpline = new BSplineCurve2D(Array(new Point2D(1, 1), new Point2D(3, 2.5), new Point2D(5, 4)), Array(2.0, 7.0), Array(3, 3), 2);
        double cf = 0, cl = 0;
        LineCurve2D? aLine = ShapeCustom_Curve2d.ConvertToLine2d(aBSpline, 2, 7, 1e-7, ref cf, ref cl, ref aDeviation);
        Assert.NotNull(aLine);
        Assert.Equal(0.0, aDeviation, 12);
        // the line's origin is moved to its point at c1, so the curve's ends are at -c1 and -c1 + 5 on it
        Assert.Equal(-2.0, cf, 12);
        Assert.Equal(3.0, cl, 12);
        Assert.True(aLine!.Value(cf).IsEqual(new Point2D(1, 1), 1e-12));
        Assert.True(aLine.Value(cl).IsEqual(new Point2D(5, 4), 1e-12));

        var aBezier = new BezierCurve2D(Array(new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, 0)));
        Assert.Null(ShapeCustom_Curve2d.ConvertToLine2d(aBezier, 0, 1, 1e-7, ref cf, ref cl, ref aDeviation));
        Assert.Null(ShapeCustom_Curve2d.ConvertToLine2d(new CircleCurve2D(new Axis2D(), 1), 0, 1, 1e-7, ref cf, ref cl, ref aDeviation));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Curve2d_SimplifyBSpline2d_RemovesKnotsWhereTheCurveIsSmooth(bool theFrozen)
    {
        // a quintic Bezier as a B-spline, with two simple knots inserted: removable
        var aBSpline = new BSplineCurve2D(Array(new Point2D(0, 0), new Point2D(1, 2), new Point2D(2, -1), new Point2D(3, 3),
                                                     new Point2D(4, 0), new Point2D(5, 1)),
                                               Array(0.0, 1.0), Array(6, 6), 5);
        aBSpline.InsertKnot(0.3);
        aBSpline.InsertKnot(0.6);
        Assert.Equal(4, aBSpline.KnotCount);
        if (theFrozen)
        {
            aBSpline.Freeze();
        }
        BSplineCurve2D aCurve = aBSpline;
        Assert.True(ShapeCustom_Curve2d.SimplifyBSpline2d(ref aCurve, 1e-7));
        Assert.True(aCurve.KnotCount < 4);
        Assert.Equal(theFrozen, !ReferenceEquals(aCurve, aBSpline));
        Assert.Equal(theFrozen ? 4 : aCurve.KnotCount, aBSpline.KnotCount);
        Assert.True(aCurve.Value(0.45).IsEqual(aBSpline.Value(0.45), 1e-6));
    }

    [Fact]
    public void Surface_ConvertToAnalytical_RecognisesAPlaneAndACylinder()
    {
        // a bilinear patch on the plane z = 2x
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(0, 3, 0);
        aPoles[2, 1] = new Point3D(1, 0, 2);
        aPoles[2, 2] = new Point3D(1, 3, 2);
        var aBilinear = new BezierSurface(aPoles);
        var aTool = new ShapeCustom_Surface(aBilinear);
        Surface? aPlane = aTool.ConvertToAnalytical(1e-7, false);
        Assert.IsType<PlaneSurface>(aPlane);
        Assert.True(aTool.Gap() < 1e-7);
        // the same orientation as the patch
        aBilinear.D1(0.5, 0.5, out _, out Vector3D aDu, out Vector3D aDv);
        Assert.True(((PlaneSurface)aPlane!).Position().Direction().Dot(new Direction3D(aDu.Crossed(aDv))) > 0);

        // half a cylinder of radius 2 as a B-spline (closed in U: the whole cylinder)
        var aCylinder = new TrimmedSurface(new CylindricalSurface(new CoordinateSystem(), 2), 0, 2 * Math.PI, 0, 5);
        BSplineSurface aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aCylinder);
        var aTool2 = new ShapeCustom_Surface(aBSpline);
        Surface? aRecognised = aTool2.ConvertToAnalytical(1e-6, true);
        Assert.IsType<CylindricalSurface>(aRecognised);
        Assert.Equal(2.0, ((CylindricalSurface)aRecognised!).Radius(), 6);
    }

    [Fact]
    public void Surface_ConvertToAnalytical_ARejectedSurfaceIsSubstitutedByNull()
    {
        // a B-spline closed in U but not a surface of revolution: rejected after the check, and OCCT's substitute then
        // replaces the tool's surface with the null result (occt-issues.md)
        var aCylinder = new TrimmedSurface(new CylindricalSurface(new CoordinateSystem(), 2), 0, 2 * Math.PI, 0, 5);
        BSplineSurface aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aCylinder);
        aBSpline = (BSplineSurface)aBSpline.Copy();
        Point3D aPole = aBSpline.Pole(2, 2);
        aBSpline.SetPole(2, 2, aPole.Translated(new Vector3D(0.5, 0.5, 0)));
        var aTool = new ShapeCustom_Surface(aBSpline);
        Assert.Null(aTool.ConvertToAnalytical(1e-6, true));
        Assert.True(aTool.Gap() > 1e-6);
        Assert.Null(aTool.ConvertToPeriodic(false)); // the tool's surface is now null
    }

    [Fact]
    public void Surface_ConvertToPeriodic_OfAFrozenSurface_GivesANewSurface_TheCallersIsUnchanged()
    {
        // an unclamped quadratic B-spline surface whose last two U rows of poles repeat the first two: closed in U with
        // simple end knots (no rearrangement), so OCCT makes it periodic in place
        var aPoles = new NCollection_Array2<Point3D>(1, 7, 1, 2);
        for (int i = 1; i <= 7; i++)
        {
            double a = 2 * Math.PI * ((i - 1) % 5) / 5;
            for (int j = 1; j <= 2; j++)
            {
                aPoles[i, j] = new Point3D(3 * Math.Cos(a), 3 * Math.Sin(a), 4 * (j - 1));
            }
        }
        var aSurface = new BSplineSurface(aPoles, Array(0.0, 1, 2, 3, 4, 5, 6, 7, 8, 9), Array(0.0, 1.0),
                                               Array(1, 1, 1, 1, 1, 1, 1, 1, 1, 1), Array(2, 2), 2, 1);
        aSurface.Freeze();
        Assert.False(aSurface.IsUPeriodic);
        Surface? aResult = new ShapeCustom_Surface(aSurface).ConvertToPeriodic(false);
        Assert.NotNull(aResult);
        Assert.True(aResult!.IsUPeriodic);
        Assert.False(aSurface.IsUPeriodic);
    }
}
