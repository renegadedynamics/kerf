// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

/// <summary>Sanity checks of the test stand-ins for GeomAdaptor_Curve and Geom2dAdaptor_Curve (TestAdaptors.cs).</summary>
public class TestAdaptorsTests
{
    private static Geom_BSplineCurve CubicWithDoubleKnot()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new gp_Pnt(i, (i % 2) * 2.0, 0.5 * i);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    private static Geom2d_BSplineCurve CubicWithDoubleKnot2d()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new gp_Pnt2d(i, (i % 2) * 2.0);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        aKnots[3] = 2.0;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 4;
        aMults[2] = 2;
        aMults[3] = 4;
        return new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    [Fact]
    public void Elementary_curves_evaluate_as_the_Geom_curve()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 2.5);
        var anAdaptor = new TestGeomAdaptor_Curve(aCircle);
        Assert.Equal(GeomAbs_Circle, anAdaptor.GetType());
        Assert.Equal(2.5, anAdaptor.Circle().Radius());
        Assert.Equal(GeomAbs_CN, anAdaptor.Continuity());
        Assert.Equal(1, anAdaptor.NbIntervals(GeomAbs_C2));
        Assert.True(anAdaptor.IsPeriodic());
        Assert.True(anAdaptor.IsClosed());
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
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        var aTrimmed = new Geom_TrimmedCurve(aLine, 1.0, 4.0);
        var anAdaptor = new TestGeomAdaptor_Curve(aTrimmed);
        Assert.Equal(GeomAbs_Line, anAdaptor.GetType());
        Assert.Equal(1.0, anAdaptor.FirstParameter());
        Assert.Equal(4.0, anAdaptor.LastParameter());
        Assert.IsType<Geom_Line>(anAdaptor.Curve());
        Assert.Equal(new gp_Pnt(2.5, 0, 0), anAdaptor.Value(2.5));
        Assert.False(anAdaptor.IsClosed());

        var aLine2d = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1));
        var anAdaptor2d = new TestGeom2dAdaptor_Curve(new Geom2d_TrimmedCurve(aLine2d, -1.0, 2.0));
        Assert.Equal(GeomAbs_Line, anAdaptor2d.GetType());
        Assert.Equal(-1.0, anAdaptor2d.FirstParameter());
        Assert.Equal(new gp_Pnt2d(0, 1.5), anAdaptor2d.Value(1.5));
        Assert.Equal(2, anAdaptor2d.NbSamples());
    }

    [Fact]
    public void BSpline_intervals_and_evaluation()
    {
        Geom_BSplineCurve aCurve = CubicWithDoubleKnot();
        var anAdaptor = new TestGeomAdaptor_Curve(aCurve);
        Assert.Equal(GeomAbs_BSplineCurve, anAdaptor.GetType());
        Assert.Equal(GeomAbs_C1, anAdaptor.Continuity());
        Assert.Equal(1, anAdaptor.NbIntervals(GeomAbs_C1));
        Assert.Equal(2, anAdaptor.NbIntervals(GeomAbs_C2));
        var anIntervals = new NCollection_Array1<double>(1, 3);
        anAdaptor.Intervals(anIntervals, GeomAbs_C2);
        Assert.Equal(0.0, anIntervals[1]);
        Assert.Equal(1.0, anIntervals[2]);
        Assert.Equal(2.0, anIntervals[3]);
        Assert.Equal(3, anAdaptor.Degree());
        Assert.Equal(6, anAdaptor.NbPoles());
        Assert.False(anAdaptor.IsRational());
        Assert.Same(aCurve, anAdaptor.BSpline());

        // On a sub-range inside one span the curve is CN.
        var aSub = (TestGeomAdaptor_Curve)anAdaptor.Trim(0.2, 0.8, 1e-7);
        Assert.Equal(GeomAbs_CN, aSub.Continuity());

        foreach (double u in new[] { 0.0, 0.3, 1.0, 1.4, 2.0 })
        {
            anAdaptor.D2(u, out gp_Pnt aP, out gp_Vec aV1, out gp_Vec aV2);
            aCurve.D2(u, out gp_Pnt aQ, out gp_Vec aW1, out gp_Vec aW2);
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
        Geom2d_BSplineCurve aCurve = CubicWithDoubleKnot2d();
        var anAdaptor = new TestGeom2dAdaptor_Curve(aCurve);
        Assert.Equal(GeomAbs_C1, anAdaptor.Continuity());
        Assert.Equal(2, anAdaptor.NbIntervals(GeomAbs_CN));
        Assert.Equal(9, anAdaptor.NbSamples());
        foreach (double u in new[] { 0.0, 0.3, 1.0, 1.4, 2.0 })
        {
            anAdaptor.D1(u, out gp_Pnt2d aP, out gp_Vec2d aV);
            aCurve.D1(u, out gp_Pnt2d aQ, out gp_Vec2d aW);
            Assert.True(aP.Distance(aQ) < 1e-12);
            Assert.True((aV - aW).Magnitude() < 1e-11);
        }
    }

    [Fact]
    public void Bezier_uses_the_cache()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 2, 0);
        aPoles[3] = new gp_Pnt(3, 2, 1);
        aPoles[4] = new gp_Pnt(4, 0, 1);
        var aBezier = new Geom_BezierCurve(aPoles);
        var anAdaptor = new TestGeomAdaptor_Curve(aBezier);
        Assert.Equal(GeomAbs_BezierCurve, anAdaptor.GetType());
        Assert.Same(aBezier, anAdaptor.Bezier());
        for (double u = 0; u <= 1; u += 0.125)
        {
            Assert.True(anAdaptor.Value(u).Distance(aBezier.Value(u)) < 1e-14);
        }
        Assert.Throws<Standard_NoSuchObject>(() => anAdaptor.BSpline());
        Assert.Throws<Standard_NoSuchObject>(() => anAdaptor.Circle());
    }

    [Fact]
    public void Load_checks_its_bounds()
    {
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        Assert.Throws<Standard_ConstructionError>(() => new TestGeomAdaptor_Curve(aLine, 2.0, 1.0));
        Assert.Equal(Precision.Parametric(1e-3), new TestGeomAdaptor_Curve(new Geom_Parabola(new gp_Ax2(), 1.0)).Resolution(1e-3));
    }
}
