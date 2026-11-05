// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema.PointCurve;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using ExtremaCurve = Kerf.Engine.Extrema.PointCurve.Curve;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>The point-curve extrema's other ways in (question 49's coverage): each form of the curve finds the same distance.</summary>
public class ExtremaPC_CurveFormsTests
{
    private const double Tol = 1e-7;
    private static readonly Point3D Point = new(5, 3, 0);
    private static readonly LineCurve3D Line = new(Point3D.Origin, Direction3D.UnitX);

    private static double Distance(ExtremaCurve curve) =>
        Math.Sqrt(curve.Perform(Point, Tol, ExtremaPC.SearchMode.Min).MinSquareDistance());

    [Fact]
    public void A_line_measures_the_same_in_every_form()
    {
        Assert.Equal(3, Distance(new ExtremaCurve(new CurveAdaptor(Line), 0, 10)), 9);
        Assert.Equal(3, Distance(new ExtremaCurve((Curve3D?)Line, 0, 10)), 9);
        Assert.Equal(3, Distance(new ExtremaCurve((Curve3D?)new TrimmedCurve3D(Line, 0, 10))), 9);
        Assert.Equal(3, Distance(new ExtremaCurve((Curve3D?)new TrimmedCurve3D(Line, 0, 10), -5, 20)), 9);
    }

    [Fact]
    public void A_transformed_curve_measures_where_it_is()
    {
        var moved = new TransformedCurve(Line, 0, 10, Transform3D.Translation(new Vector3D(0, 1, 0)));
        Assert.Equal(2, Distance(new ExtremaCurve(moved)), 9);
        Assert.Equal(2, Distance(new ExtremaCurve(moved, 0, 10)), 9);
    }

    [Fact]
    public void A_null_curve_finds_nothing()
    {
        Assert.False(new ExtremaCurve((Curve3D?)null).Perform(Point, Tol, ExtremaPC.SearchMode.Min).IsDone);
    }
}
