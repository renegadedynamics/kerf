// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeCustom_Curve, Curve2d and Surface (unit 7A): their conversions, and the caller's frozen geometry, which OCCT
/// changes in place and the port copies first (decision 8102).
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
    private static Geom_BSplineCurve ClosedBSpline()
    {
        NCollection_Array1<gp_Pnt> aPoles = Array(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0), new gp_Pnt(0, 10, 0),
                                                  new gp_Pnt(-5, 5, 0), new gp_Pnt(0, 0, 0));
        return new Geom_BSplineCurve(aPoles, Array(0.0, 1.0, 2.0, 3.0), Array(4, 1, 1, 4), 3);
    }

    // An unclamped cubic B-spline whose last three poles repeat the first three: closed, with simple end knots, so OCCT
    // makes it periodic in place, without rearranging its knots.
    private static Geom_BSplineCurve WrappedBSpline()
    {
        NCollection_Array1<gp_Pnt> aPoles = Array(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0), new gp_Pnt(0, 10, 0),
                                                  new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), new gp_Pnt(10, 10, 0));
        return new Geom_BSplineCurve(aPoles, Array(0.0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10), Array(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1), 3);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Curve_ConvertToPeriodic_OfAFrozenCurve_GivesANewPeriodicCurve_TheCallersIsUnchanged(bool theWrapped)
    {
        Geom_BSplineCurve aCurve = theWrapped ? WrappedBSpline() : ClosedBSpline();
        aCurve.Freeze();
        var aTool = new ShapeCustom_Curve(aCurve);
        Geom_Curve? aResult = aTool.ConvertToPeriodic(true);
        Assert.NotNull(aResult);
        Assert.True(aResult!.IsPeriodic());
        Assert.False(aCurve.IsPeriodic());
        Assert.NotSame(aCurve, aResult);
        // a second call converts the substituted (now periodic) curve: nothing to do
        Assert.Null(aTool.ConvertToPeriodic(false));
    }

    [Fact]
    public void Curve_ConvertToPeriodic_OfAnOpenOrNonBSplineCurve_IsNull()
    {
        Assert.Null(new ShapeCustom_Curve(new Geom_Circle(new gp_Ax2(), 3)).ConvertToPeriodic(false));
        NCollection_Array1<gp_Pnt> aPoles = Array(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(3, 1, 0));
        var anOpen = new Geom_BSplineCurve(aPoles, Array(0.0, 1.0), Array(4, 4), 3);
        Assert.Null(new ShapeCustom_Curve(anOpen).ConvertToPeriodic(false));
    }

    [Fact]
    public void Curve2d_IsLinear_AndConvertToLine2d()
    {
        double aDeviation = -1;
        Assert.True(ShapeCustom_Curve2d.IsLinear(Array(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1e-8), new gp_Pnt2d(3, 0)), 1e-6, ref aDeviation));
        Assert.Equal(1e-8, aDeviation, 12);
        aDeviation = -1;
        Assert.False(ShapeCustom_Curve2d.IsLinear(Array(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1e-3), new gp_Pnt2d(3, 0)), 1e-6, ref aDeviation));
        Assert.Equal(-1, aDeviation); // left unchanged
        Assert.False(ShapeCustom_Curve2d.IsLinear(Array(new gp_Pnt2d(0, 0)), 1e-6, ref aDeviation));

        // a linear B-spline from (1, 1) to (5, 4), parametrised on [2, 7]
        var aBSpline = new Geom2d_BSplineCurve(Array(new gp_Pnt2d(1, 1), new gp_Pnt2d(3, 2.5), new gp_Pnt2d(5, 4)), Array(2.0, 7.0), Array(3, 3), 2);
        double cf = 0, cl = 0;
        Geom2d_Line? aLine = ShapeCustom_Curve2d.ConvertToLine2d(aBSpline, 2, 7, 1e-7, ref cf, ref cl, ref aDeviation);
        Assert.NotNull(aLine);
        Assert.Equal(0.0, aDeviation, 12);
        // the line's origin is moved to its point at c1, so the curve's ends are at -c1 and -c1 + 5 on it
        Assert.Equal(-2.0, cf, 12);
        Assert.Equal(3.0, cl, 12);
        Assert.True(aLine!.Value(cf).IsEqual(new gp_Pnt2d(1, 1), 1e-12));
        Assert.True(aLine.Value(cl).IsEqual(new gp_Pnt2d(5, 4), 1e-12));

        var aBezier = new Geom2d_BezierCurve(Array(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(2, 0)));
        Assert.Null(ShapeCustom_Curve2d.ConvertToLine2d(aBezier, 0, 1, 1e-7, ref cf, ref cl, ref aDeviation));
        Assert.Null(ShapeCustom_Curve2d.ConvertToLine2d(new Geom2d_Circle(new gp_Ax2d(), 1), 0, 1, 1e-7, ref cf, ref cl, ref aDeviation));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Curve2d_SimplifyBSpline2d_RemovesKnotsWhereTheCurveIsSmooth(bool theFrozen)
    {
        // a quintic Bezier as a B-spline, with two simple knots inserted: removable
        var aBSpline = new Geom2d_BSplineCurve(Array(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(2, -1), new gp_Pnt2d(3, 3),
                                                     new gp_Pnt2d(4, 0), new gp_Pnt2d(5, 1)),
                                               Array(0.0, 1.0), Array(6, 6), 5);
        aBSpline.InsertKnot(0.3);
        aBSpline.InsertKnot(0.6);
        Assert.Equal(4, aBSpline.NbKnots());
        if (theFrozen)
        {
            aBSpline.Freeze();
        }
        Geom2d_BSplineCurve aCurve = aBSpline;
        Assert.True(ShapeCustom_Curve2d.SimplifyBSpline2d(ref aCurve, 1e-7));
        Assert.True(aCurve.NbKnots() < 4);
        Assert.Equal(theFrozen, !ReferenceEquals(aCurve, aBSpline));
        Assert.Equal(theFrozen ? 4 : aCurve.NbKnots(), aBSpline.NbKnots());
        Assert.True(aCurve.Value(0.45).IsEqual(aBSpline.Value(0.45), 1e-6));
    }

    [Fact]
    public void Surface_ConvertToAnalytical_RecognisesAPlaneAndACylinder()
    {
        // a bilinear patch on the plane z = 2x
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0, 0, 0);
        aPoles[1, 2] = new gp_Pnt(0, 3, 0);
        aPoles[2, 1] = new gp_Pnt(1, 0, 2);
        aPoles[2, 2] = new gp_Pnt(1, 3, 2);
        var aBilinear = new Geom_BezierSurface(aPoles);
        var aTool = new ShapeCustom_Surface(aBilinear);
        Geom_Surface? aPlane = aTool.ConvertToAnalytical(1e-7, false);
        Assert.IsType<Geom_Plane>(aPlane);
        Assert.True(aTool.Gap() < 1e-7);
        // the same orientation as the patch
        aBilinear.D1(0.5, 0.5, out _, out gp_Vec aDu, out gp_Vec aDv);
        Assert.True(((Geom_Plane)aPlane!).Position().Direction().Dot(new gp_Dir(aDu.Crossed(aDv))) > 0);

        // half a cylinder of radius 2 as a B-spline (closed in U: the whole cylinder)
        var aCylinder = new Geom_RectangularTrimmedSurface(new Geom_CylindricalSurface(new gp_Ax3(), 2), 0, 2 * Math.PI, 0, 5);
        Geom_BSplineSurface aBSpline = GeomConvert.SurfaceToBSplineSurface(aCylinder);
        var aTool2 = new ShapeCustom_Surface(aBSpline);
        Geom_Surface? aRecognised = aTool2.ConvertToAnalytical(1e-6, true);
        Assert.IsType<Geom_CylindricalSurface>(aRecognised);
        Assert.Equal(2.0, ((Geom_CylindricalSurface)aRecognised!).Radius(), 6);
    }

    [Fact]
    public void Surface_ConvertToAnalytical_ARejectedSurfaceIsSubstitutedByNull()
    {
        // a B-spline closed in U but not a surface of revolution: rejected after the check, and OCCT's substitute then
        // replaces the tool's surface with the null result (occt-issues.md)
        var aCylinder = new Geom_RectangularTrimmedSurface(new Geom_CylindricalSurface(new gp_Ax3(), 2), 0, 2 * Math.PI, 0, 5);
        Geom_BSplineSurface aBSpline = GeomConvert.SurfaceToBSplineSurface(aCylinder);
        aBSpline = (Geom_BSplineSurface)aBSpline.Copy();
        gp_Pnt aPole = aBSpline.Pole(2, 2);
        aBSpline.SetPole(2, 2, aPole.Translated(new gp_Vec(0.5, 0.5, 0)));
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
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 7, 1, 2);
        for (int i = 1; i <= 7; i++)
        {
            double a = 2 * Math.PI * ((i - 1) % 5) / 5;
            for (int j = 1; j <= 2; j++)
            {
                aPoles[i, j] = new gp_Pnt(3 * Math.Cos(a), 3 * Math.Sin(a), 4 * (j - 1));
            }
        }
        var aSurface = new Geom_BSplineSurface(aPoles, Array(0.0, 1, 2, 3, 4, 5, 6, 7, 8, 9), Array(0.0, 1.0),
                                               Array(1, 1, 1, 1, 1, 1, 1, 1, 1, 1), Array(2, 2), 2, 1);
        aSurface.Freeze();
        Assert.False(aSurface.IsUPeriodic());
        Geom_Surface? aResult = new ShapeCustom_Surface(aSurface).ConvertToPeriodic(false);
        Assert.NotNull(aResult);
        Assert.True(aResult!.IsUPeriodic());
        Assert.False(aSurface.IsUPeriodic());
    }
}
