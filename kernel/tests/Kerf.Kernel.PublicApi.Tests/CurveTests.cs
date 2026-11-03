// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Curves in 3D and 2D: evaluation, ranges, kinds, and the B-splines' data indexed from 0.</summary>
public class CurveTests
{
    [Fact]
    public void A_line_evaluates_and_is_unbounded()
    {
        var line = new LineCurve3D(new Point3D(1, 0, 0), Direction3D.UnitY);
        Assert.Equal(new Point3D(1, 2, 0), line.Evaluate(2));
        Assert.True(line.Range.IsInfinite);
        Assert.Equal(CurveKind.Line, line.Kind);
        Assert.Equal(Continuity.CN, line.Continuity);
        Assert.Equal(Direction3D.UnitY, line.Position.Direction);
        Assert.False(line.IsClosed);
    }

    [Fact]
    public void A_circle_has_its_derivatives_and_period()
    {
        var circle = new CircleCurve3D(new Circle3D(Frame3D.World, 2));
        Assert.Equal(2, circle.Radius);
        Assert.Equal(CurveKind.Circle, circle.Kind);
        Assert.True(circle.IsPeriodic);
        Assert.Equal(2 * Math.PI, circle.Period, 12);
        Assert.Equal(new Interval(0, 2 * Math.PI), circle.Range);

        CurveDerivatives d = circle.Derivatives(Math.PI / 2, 2);
        Assert.Equal(2, d.Order);
        Assert.True(d.Point.IsEqual(new Point3D(0, 2, 0), 1e-12));
        Assert.True(d.D1.IsEqual(new Vector3D(-2, 0, 0), 1e-12, 1e-12));
        Assert.True(d.D2.IsEqual(new Vector3D(0, -2, 0), 1e-12, 1e-12));
        Assert.Equal(Vector3D.Zero, d.D3);
        Assert.True(circle.DerivativeN(0, 3).IsEqual(new Vector3D(0, -2, 0), 1e-12, 1e-12));
        Assert.Throws<ArgumentOutOfRangeException>(() => circle.Derivatives(0, 4));
        Assert.Throws<ArgumentOutOfRangeException>(() => circle.DerivativeN(0, 0));
        Assert.Equal(new Circle3D(Frame3D.World, 2), circle.Circle);
    }

    [Fact]
    public void A_trimmed_curve_is_its_basis_curves_kind_over_a_range()
    {
        var arc = new TrimmedCurve3D(new CircleCurve3D(Frame3D.World, 1), 0, Math.PI);
        Assert.Equal(CurveKind.Circle, arc.Kind);
        Assert.Equal(new Interval(0, Math.PI), arc.Range);
        Assert.True(arc.StartPoint.IsEqual(new Point3D(1, 0, 0), 1e-12));
        Assert.True(arc.EndPoint.IsEqual(new Point3D(-1, 0, 0), 1e-12));
        Assert.IsType<CircleCurve3D>(arc.BasisCurve);
        Curve3D reversed = arc.Reversed();
        Assert.True(reversed.Evaluate(reversed.Range.Min).IsEqual(new Point3D(-1, 0, 0), 1e-12));
    }

    [Fact]
    public void A_bspline_is_built_from_spans_and_reads_back_from_zero()
    {
        Point3D[] poles = [new(0, 0, 0), new(1, 2, 0), new(2, 0, 0)];
        var curve = new BSplineCurve3D(poles, [0, 1], [3, 3], 2);
        Assert.Equal(2, curve.Degree);
        Assert.Equal(3, curve.PoleCount);
        Assert.Equal(new Point3D(1, 2, 0), curve.Poles[1]);
        Assert.Equal([0.0, 1.0], curve.Knots);
        Assert.Equal([3, 3], curve.Multiplicities);
        Assert.False(curve.IsRational);
        Assert.Equal([1.0, 1.0, 1.0], curve.Weights);
        Assert.Equal(CurveKind.BSplineCurve, curve.Kind);
        Assert.True(curve.Evaluate(0.5).IsEqual(new Point3D(1, 1, 0), 1e-12));
        Assert.Equal(new Interval(0, 1), curve.Range);
    }

    [Fact]
    public void A_rational_bspline_keeps_its_weights()
    {
        Point3D[] poles = [new(1, 0, 0), new(1, 1, 0), new(0, 1, 0)];
        double w = Math.Sqrt(0.5);
        var quarter = new BSplineCurve3D(poles, [0, 1], [3, 3], 2, weights: [1, w, 1]);
        Assert.True(quarter.IsRational);
        Assert.Equal(w, quarter.Weights[1]);
        Point3D mid = quarter.Evaluate(0.5);
        Assert.Equal(1, Math.Sqrt(mid.X * mid.X + mid.Y * mid.Y), 12);
    }

    [Fact]
    public void Invalid_spline_data_is_an_argument_error()
    {
        Point3D[] poles = [new(0, 0, 0), new(1, 2, 0), new(2, 0, 0)];
        Assert.ThrowsAny<ArgumentException>(() => new BSplineCurve3D(poles, [0, 1], [2, 2], 2));
        Assert.ThrowsAny<ArgumentException>(() => new BezierCurve3D([new Point3D(0, 0, 0)]));
    }

    [Fact]
    public void A_bezier_is_built_from_its_poles()
    {
        var bezier = new BezierCurve3D([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)]);
        Assert.Equal(2, bezier.Degree);
        Assert.Equal(new Interval(0, 1), bezier.Range);
        Assert.Equal(CurveKind.BezierCurve, bezier.Kind);
        Assert.True(bezier.Evaluate(0.5).IsEqual(new Point3D(1, 0.5, 0), 1e-12));
        Assert.Equal(new Point3D(2, 0, 0), bezier.Poles[2]);
    }

    [Fact]
    public void Curves_transform_into_new_curves()
    {
        var line = new LineCurve3D(Point3D.Origin, Direction3D.UnitX);
        var moved = (Curve3D)line.Transformed(Transform3D.Translation(new Vector3D(0, 5, 0)));
        Assert.Equal(new Point3D(0, 5, 0), moved.Evaluate(0));
        Assert.Equal(Point3D.Origin, line.Evaluate(0));
    }

    [Fact]
    public void The_2D_curves_work_the_same_way()
    {
        var circle = new CircleCurve2D(Frame2D.World, 3);
        Assert.Equal(CurveKind.Circle, circle.Kind);
        Assert.True(circle.Evaluate(0).IsEqual(new Point2D(3, 0), 1e-12));
        CurveDerivatives2D d = circle.Derivatives(0, 1);
        Assert.True(d.D1.IsEqual(new Vector2D(0, 3), 1e-12, 1e-12));
        Assert.Equal(3, circle.Radius);

        var line = new LineCurve2D(Point2D.Origin, Direction2D.UnitY);
        Assert.Equal(new Point2D(0, 4), line.Evaluate(4));
        Assert.Equal(CurveKind.Line, line.Kind);

        var spline = new BSplineCurve2D([new Point2D(0, 0), new Point2D(1, 1)], [0, 1], [2, 2], 1);
        Assert.Equal(new Point2D(0.5, 0.5), spline.Evaluate(0.5));
        Assert.Equal(new Point2D(1, 1), spline.Poles[1]);
        var bezier = new BezierCurve2D([new Point2D(0, 0), new Point2D(2, 2)]);
        Assert.Equal(new Point2D(1, 1), bezier.Evaluate(0.5));
    }
}
