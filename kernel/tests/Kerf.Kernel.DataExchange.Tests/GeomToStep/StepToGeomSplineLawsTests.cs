// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Exchange.Step.Data.Logical;
using static Kerf.Engine.Exchange.Step.Geometry.StepBSplineCurveForm;
using static Kerf.Engine.Exchange.Step.Geometry.StepBSplineSurfaceForm;
using Converter = Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom;

namespace Kerf.Kernel.DataExchange.Tests.GeomToStep;

/// <summary>
/// STEP's Bezier, uniform and quasi-uniform splines read as B-splines (unit 12Q, decision 1753), by what each is: a
/// Bezier runs from its first pole to its last; a uniform quadratic passes the middle of each pair of poles, at its knots,
/// and so starts and ends there; a quasi-uniform one starts and ends on its end poles. Weights pull the curve as they
/// would: a quadratic quasi-uniform curve weighted (1, cos 45 degrees, 1) is a quarter of a circle. Surfaces over poles on
/// a plane stay on it whatever the form, and a degree out of range is no spline.
/// </summary>
public sealed class StepToGeomSplineLawsTests
{
    private static TCollection_HAsciiString Name() => new("");

    private static StepCartesianPoint Point(double x, double y, double z)
    {
        var aP = new StepCartesianPoint();
        aP.Init3D(Name(), x, y, z);
        return aP;
    }

    private static NCollection_HArray1<StepCartesianPoint> Poles(params (double X, double Y)[] thePoints)
    {
        var aPoles = new NCollection_HArray1<StepCartesianPoint>(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; i++)
        {
            aPoles.SetValue(i + 1, Point(thePoints[i].X, thePoints[i].Y, 0));
        }
        return aPoles;
    }

    private static NCollection_HArray1<double> Weights(params double[] theWeights)
    {
        var aWeights = new NCollection_HArray1<double>(1, theWeights.Length);
        for (int i = 0; i < theWeights.Length; i++)
        {
            aWeights.SetValue(i + 1, theWeights[i]);
        }
        return aWeights;
    }

    private static readonly (double, double)[] Four = [(1, 0), (1, 1), (0, 1), (-1, 1)];

    private static BSplineCurve3D Read(StepBoundedCurve theCurve)
    {
        BoundedCurve3D? aCurve = Converter.MakeBoundedCurve(theCurve);
        return Assert.IsType<BSplineCurve3D>(aCurve);
    }

    private static void AssertAt(Curve3D theCurve, double theU, double theX, double theY) =>
        Assert.True(theCurve.Value(theU).DistanceTo(new Point3D(theX, theY, 0)) < 1e-12, $"{theCurve.Value(theU)} at {theU}");

    [Fact]
    public void A_bezier_runs_from_its_first_pole_to_its_last_through_their_mean()
    {
        var aBezier = new StepBezierCurve();
        aBezier.Init(Name(), 2, Poles((1, 0), (1, 1), (0, 1)), BscfUnspecified, LFalse, LFalse);
        BSplineCurve3D aCurve = Read(aBezier);
        Assert.Equal(2, aCurve.Degree);
        AssertAt(aCurve, aCurve.FirstParameter, 1, 0);
        AssertAt(aCurve, aCurve.LastParameter, 0, 1);
        // at the middle, (P1 + 2 P2 + P3) / 4
        AssertAt(aCurve, 0.5 * (aCurve.FirstParameter + aCurve.LastParameter), 0.75, 0.75);
    }

    [Fact]
    public void A_uniform_quadratic_passes_the_middle_of_each_pair_of_poles()
    {
        var aUniform = new StepUniformCurve();
        aUniform.Init(Name(), 2, Poles(Four), BscfUnspecified, LFalse, LFalse);
        BSplineCurve3D aCurve = Read(aUniform);
        AssertAt(aCurve, aCurve.FirstParameter, 1, 0.5);
        AssertAt(aCurve, aCurve.LastParameter, -0.5, 1);
        AssertAt(aCurve, 0.5 * (aCurve.FirstParameter + aCurve.LastParameter), 0.5, 1);
    }

    [Fact]
    public void A_quasi_uniform_quadratic_starts_and_ends_on_its_end_poles()
    {
        var aQuasi = new StepQuasiUniformCurve();
        aQuasi.Init(Name(), 2, Poles(Four), BscfUnspecified, LFalse, LFalse);
        BSplineCurve3D aCurve = Read(aQuasi);
        AssertAt(aCurve, aCurve.FirstParameter, 1, 0);
        AssertAt(aCurve, aCurve.LastParameter, -1, 1);
        // its one inner knot, half way, is at the middle of the two inner poles
        AssertAt(aCurve, 0.5 * (aCurve.FirstParameter + aCurve.LastParameter), 0.5, 1);
    }

    [Fact]
    public void A_weighted_quasi_uniform_quadratic_is_a_quarter_circle()
    {
        var aQuasi = new StepQuasiUniformCurveAndRationalBSplineCurve();
        aQuasi.Init(Name(), 2, Poles((1, 0), (1, 1), (0, 1)), BscfCircularArc, LFalse, LFalse, Weights(1, Math.Sqrt(0.5), 1));
        BSplineCurve3D aCurve = Read(aQuasi);
        Assert.True(aCurve.IsRational);
        for (int i = 0; i <= 10; i++)
        {
            Point3D aP = aCurve.Value(aCurve.FirstParameter + ((aCurve.LastParameter - aCurve.FirstParameter) * i / 10));
            Assert.Equal(1, Math.Sqrt((aP.X * aP.X) + (aP.Y * aP.Y)), 12);
        }
    }

    [Fact]
    public void A_weighted_uniform_quadratic_passes_the_weighted_middles()
    {
        // at its knots, (w1 P1 + w2 P2) / (w1 + w2): weights 1, 3, 1, 1
        var aUniform = new StepUniformCurveAndRationalBSplineCurve();
        aUniform.Init(Name(), 2, Poles(Four), BscfUnspecified, LFalse, LFalse, Weights(1, 3, 1, 1));
        BSplineCurve3D aCurve = Read(aUniform);
        AssertAt(aCurve, aCurve.FirstParameter, 1, 0.75);
        AssertAt(aCurve, 0.5 * (aCurve.FirstParameter + aCurve.LastParameter), 0.75, 1);
        AssertAt(aCurve, aCurve.LastParameter, -0.5, 1);
    }

    [Fact]
    public void A_degree_out_of_range_is_no_curve()
    {
        // and four poles can't hold a degree 4: it needs five
        foreach (int aDegree in new[] { 0, BSplineCurve3D.MaxDegree + 1, 4 })
        {
            var aBezier = new StepBezierCurve();
            aBezier.Init(Name(), aDegree, Poles(Four), BscfUnspecified, LFalse, LFalse);
            var aUniform = new StepUniformCurve();
            aUniform.Init(Name(), aDegree, Poles(Four), BscfUnspecified, LFalse, LFalse);
            var aQuasi = new StepQuasiUniformCurve();
            aQuasi.Init(Name(), aDegree, Poles(Four), BscfUnspecified, LFalse, LFalse);
            var aRUniform = new StepUniformCurveAndRationalBSplineCurve();
            aRUniform.Init(Name(), aDegree, Poles(Four), BscfUnspecified, LFalse, LFalse, Weights(1, 1, 1, 1));
            var aRQuasi = new StepQuasiUniformCurveAndRationalBSplineCurve();
            aRQuasi.Init(Name(), aDegree, Poles(Four), BscfUnspecified, LFalse, LFalse, Weights(1, 1, 1, 1));
            foreach (StepBoundedCurve aCurve in new StepBoundedCurve[] { aBezier, aUniform, aQuasi, aRUniform, aRQuasi })
            {
                Assert.Null(Converter.MakeBoundedCurve(aCurve));
            }
        }
    }

    // A 4 x 3 grid of poles on the plane z = x + 2 y
    private static NCollection_HArray2<StepCartesianPoint> Grid()
    {
        var aGrid = new NCollection_HArray2<StepCartesianPoint>(1, 4, 1, 3);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aGrid.SetValue(i, j, Point(i, j * 1.5, i + (3.0 * j)));
            }
        }
        return aGrid;
    }

    private static NCollection_HArray2<double> GridWeights()
    {
        var aWeights = new NCollection_HArray2<double>(1, 4, 1, 3);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aWeights.SetValue(i, j, 1 + (0.25 * i * j));
            }
        }
        return aWeights;
    }

    private static BSplineSurface ReadSurface(StepBoundedSurface theSurface) => Assert.IsType<BSplineSurface>(Converter.MakeBoundedSurface(theSurface));

    private static void AssertOnThePlane(BSplineSurface theSurface)
    {
        theSurface.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        for (int i = 0; i <= 6; i++)
        {
            for (int j = 0; j <= 6; j++)
            {
                Point3D aP = theSurface.Value(u1 + ((u2 - u1) * i / 6), v1 + ((v2 - v1) * j / 6));
                Assert.Equal(aP.X + (2 * aP.Y), aP.Z, 10);
            }
        }
    }

    [Fact]
    public void Surfaces_of_each_form_over_a_plane_stay_on_it()
    {
        var aBezier = new StepBezierSurface();
        aBezier.Init(Name(), 3, 2, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse);
        var aUniform = new StepUniformSurface();
        aUniform.Init(Name(), 2, 2, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse);
        var aQuasi = new StepQuasiUniformSurface();
        aQuasi.Init(Name(), 2, 2, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse);
        var aRUniform = new StepUniformSurfaceAndRationalBSplineSurface();
        aRUniform.Init(Name(), 2, 2, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse, GridWeights());
        var aRQuasi = new StepQuasiUniformSurfaceAndRationalBSplineSurface();
        aRQuasi.Init(Name(), 2, 2, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse, GridWeights());
        foreach (StepBoundedSurface aStep in new StepBoundedSurface[] { aBezier, aUniform, aQuasi, aRUniform, aRQuasi })
        {
            AssertOnThePlane(ReadSurface(aStep));
        }

        // a Bezier's and a quasi-uniform surface's corners are their corner poles; a uniform quadratic's the mean of the
        // four poles at the corner
        foreach (StepBoundedSurface aClamped in new StepBoundedSurface[] { aBezier, aQuasi })
        {
            BSplineSurface aSurface = ReadSurface(aClamped);
            aSurface.ParameterBounds(out double u1, out _, out double v1, out _);
            Assert.True(aSurface.Value(u1, v1).DistanceTo(new Point3D(1, 1.5, 4)) < 1e-12);
        }
        BSplineSurface anUnclamped = ReadSurface(aUniform);
        anUnclamped.ParameterBounds(out double uu1, out _, out double vv1, out _);
        Assert.True(anUnclamped.Value(uu1, vv1).DistanceTo(new Point3D(1.5, 2.25, 6)) < 1e-12);
        Assert.True(ReadSurface(aRQuasi).IsURational || ReadSurface(aRQuasi).IsVRational);
    }

    [Fact]
    public void A_surface_degree_out_of_range_is_no_surface()
    {
        // and 4 by 3 poles can't hold a degree 3 in V: it needs 4
        foreach ((int aU, int aV) in new[] { (0, 2), (2, 0), (BSplineSurface.MaxDegree + 1, 2), (2, 3) })
        {
            var aBezier = new StepBezierSurface();
            aBezier.Init(Name(), aU, aV, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse);
            var aUniform = new StepUniformSurface();
            aUniform.Init(Name(), aU, aV, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse);
            var aQuasi = new StepQuasiUniformSurface();
            aQuasi.Init(Name(), aU, aV, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse);
            var aRUniform = new StepUniformSurfaceAndRationalBSplineSurface();
            aRUniform.Init(Name(), aU, aV, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse, GridWeights());
            var aRQuasi = new StepQuasiUniformSurfaceAndRationalBSplineSurface();
            aRQuasi.Init(Name(), aU, aV, Grid(), BssfPlaneSurf, LFalse, LFalse, LFalse, GridWeights());
            foreach (StepBoundedSurface aSurface in new StepBoundedSurface[] { aBezier, aUniform, aQuasi, aRUniform, aRQuasi })
            {
                Assert.Null(Converter.MakeBoundedSurface(aSurface));
            }
        }
    }
}
