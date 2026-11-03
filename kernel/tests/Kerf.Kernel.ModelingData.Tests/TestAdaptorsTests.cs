// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Sanity checks of the test stand-ins for GeomAdaptor_Curve and Geom2dAdaptor_Curve (TestAdaptors.cs).</summary>
public class TestAdaptorsTests
{
    private static BSplineCurve3D CubicWithDoubleKnot()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new Point3D(i, (i % 2) * 2.0, 0.5 * i);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    private static BSplineCurve2D CubicWithDoubleKnot2d()
    {
        var aPoles = new NCollection_Array1<Point2D>(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new Point2D(i, (i % 2) * 2.0);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        return new BSplineCurve2D(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void Elementary_curves_evaluate_as_the_Geom_curve()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 2.5);
        var anAdaptor = new TestGeomAdaptor_Curve(aCircle);
        Assert.Equal(Circle, anAdaptor.GetType());
        Assert.Equal(2.5, anAdaptor.Circle().Radius());
        Assert.Equal(CN, anAdaptor.Continuity());
        Assert.Equal(1, anAdaptor.NbIntervals(C2));
        Assert.True(anAdaptor.IsPeriodic);
        Assert.True(anAdaptor.IsClosed);
        Assert.Equal(2 * Math.PI, anAdaptor.Period());
        for (double u = 0; u < 6; u += 0.7)
        {
            Assert.Equal(aCircle.Value(u), anAdaptor.Value(u));
            Assert.Equal(aCircle.DN(u, 2), anAdaptor.DN(u, 2));
        }
        Assert.Equal(2 * Math.Asin(1e-7 / 5.0), anAdaptor.Resolution(1e-7));
    }

    [Fact]
    public void Trimmed_curves_are_unwrapped()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        var aTrimmed = new TrimmedCurve3D(aLine, 1.0, 4.0);
        var anAdaptor = new TestGeomAdaptor_Curve(aTrimmed);
        Assert.Equal(Line, anAdaptor.GetType());
        Assert.Equal(1.0, anAdaptor.FirstParameter());
        Assert.Equal(4.0, anAdaptor.LastParameter());
        Assert.IsType<LineCurve3D>(anAdaptor.Curve());
        Assert.Equal(new Point3D(2.5, 0, 0), anAdaptor.Value(2.5));
        Assert.False(anAdaptor.IsClosed);

        var aLine2d = new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1));
        var anAdaptor2d = new TestGeom2dAdaptor_Curve(new TrimmedCurve2D(aLine2d, -1.0, 2.0));
        Assert.Equal(Line, anAdaptor2d.GetType());
        Assert.Equal(-1.0, anAdaptor2d.FirstParameter());
        Assert.Equal(new Point2D(0, 1.5), anAdaptor2d.Value(1.5));
        Assert.Equal(2, anAdaptor2d.SampleCount);
    }

    [Fact]
    public void BSpline_intervals_and_evaluation()
    {
        BSplineCurve3D aCurve = CubicWithDoubleKnot();
        var anAdaptor = new TestGeomAdaptor_Curve(aCurve);
        Assert.Equal(BSplineCurve, anAdaptor.GetType());
        Assert.Equal(C1, anAdaptor.Continuity());
        Assert.Equal(1, anAdaptor.NbIntervals(C1));
        Assert.Equal(2, anAdaptor.NbIntervals(C2));
        var anIntervals = new NCollection_Array1<double>(1, 3);
        anAdaptor.Intervals(anIntervals, C2);
        Assert.Equal(0.0, anIntervals[1]);
        Assert.Equal(1.0, anIntervals[2]);
        Assert.Equal(2.0, anIntervals[3]);
        Assert.Equal(3, anAdaptor.Degree());
        Assert.Equal(6, anAdaptor.PoleCount);
        Assert.False(anAdaptor.IsRational);
        Assert.Same(aCurve, anAdaptor.BSpline());

        // On a sub-range inside one span the curve is CN.
        var aSub = (TestGeomAdaptor_Curve)anAdaptor.Trim(0.2, 0.8, 1e-7);
        Assert.Equal(CN, aSub.Continuity());

        foreach (double u in new[] { 0.0, 0.3, 1.0, 1.4, 2.0 })
        {
            anAdaptor.D2(u, out Point3D aP, out Vector3D aV1, out Vector3D aV2);
            aCurve.D2(u, out Point3D aQ, out Vector3D aW1, out Vector3D aW2);
            Assert.True(aP.Distance(aQ) < 1e-12);
            Assert.True((aV1 - aW1).Magnitude() < 1e-11);
            Assert.True((aV2 - aW2).Magnitude() < 1e-10);
        }
        Assert.Equal(aCurve.Value(0.0), anAdaptor.Value(0.0));
        Assert.Equal(aCurve.Value(2.0), anAdaptor.Value(2.0));
    }

    [Fact]
    public void BSpline_2d_intervals_and_evaluation()
    {
        BSplineCurve2D aCurve = CubicWithDoubleKnot2d();
        var anAdaptor = new TestGeom2dAdaptor_Curve(aCurve);
        Assert.Equal(C1, anAdaptor.Continuity());
        Assert.Equal(2, anAdaptor.NbIntervals(CN));
        Assert.Equal(9, anAdaptor.SampleCount);
        foreach (double u in new[] { 0.0, 0.3, 1.0, 1.4, 2.0 })
        {
            anAdaptor.D1(u, out Point2D aP, out Vector2D aV);
            aCurve.D1(u, out Point2D aQ, out Vector2D aW);
            Assert.True(aP.Distance(aQ) < 1e-12);
            Assert.True((aV - aW).Magnitude() < 1e-11);
        }
    }

    [Fact]
    public void Bezier_uses_the_cache()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(3, 2, 1);
        aPoles[4] = new Point3D(4, 0, 1);
        var aBezier = new BezierCurve3D(aPoles);
        var anAdaptor = new TestGeomAdaptor_Curve(aBezier);
        Assert.Equal(BezierCurve, anAdaptor.GetType());
        Assert.Same(aBezier, anAdaptor.Bezier());
        for (double u = 0; u <= 1; u += 0.125)
        {
            Assert.True(anAdaptor.Value(u).Distance(aBezier.Value(u)) < 1e-14);
        }
        Assert.Throws<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anAdaptor.BSpline());
        Assert.Throws<global::Kerf.Engine.Foundation.NoSuchObjectException>(() => anAdaptor.Circle());
    }

    [Fact]
    public void Load_checks_its_bounds()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Assert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new TestGeomAdaptor_Curve(aLine, 2.0, 1.0));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Parametric(1e-3), new TestGeomAdaptor_Curve(new ParabolaCurve3D(new Frame3D(), 1.0)).Resolution(1e-3));
    }
}
