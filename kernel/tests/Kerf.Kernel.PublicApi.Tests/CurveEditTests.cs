// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The curve and surface classes' With forms and the properties no other test reads (question 49): each change shows in
/// what the new value evaluates to, and the original keeps its own.
/// </summary>
public class CurveEditTests
{
    private static readonly Point3D[] Poles3 = [new(0, 0, 0), new(1, 1, 0), new(2, 0, 0)];
    private static readonly Point2D[] Poles2 = [new(0, 0), new(1, 1), new(2, 0)];

    private static BSplineCurve3D Spline3() =>
        new([new(0, 0, 0), new(1, 1, 0), new(2, 1, 0), new(3, 0, 0)], [0.0, 0.5, 1.0], [3, 1, 3], 2);

    private static BSplineCurve2D Spline2() =>
        new([new(0, 0), new(1, 1), new(2, 1), new(3, 0)], [0.0, 0.5, 1.0], [3, 1, 3], 2);

    private static Grid<Point3D> PolesGrid()
    {
        var grid = new Point3D[9];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                grid[i * 3 + j] = new Point3D(i, j, i == 1 && j == 1 ? 1 : 0);
            }
        }
        return new Grid<Point3D>(grid, 3, 3);
    }

    private static BSplineSurface SplineSurface() =>
        new(PolesGrid(), [0.0, 1.0], [0.0, 1.0], [3, 3], [3, 3], 2, 2);

    [Fact]
    public void A_2D_spline_replaces_poles_weights_and_knots()
    {
        BSplineCurve2D spline = Spline2();
        Assert.Equal(new Point2D(9, 9), spline.WithPole(1, new Point2D(9, 9)).Poles[1]);
        BSplineCurve2D weighted = spline.WithPole(1, new Point2D(9, 9), 2);
        Assert.Equal(new Point2D(9, 9), weighted.Poles[1]);
        Assert.Equal(2, weighted.Weights[1]);
        Assert.True(weighted.IsRational);
        Assert.Equal(3, spline.WithWeight(1, 3).Weights[1]);
        Assert.Equal(0.25, spline.WithKnot(1, 0.25).Knots[1]);
        BSplineCurve2D raised = spline.WithKnot(1, 0.25, 2);
        Assert.Equal(0.25, raised.Knots[1]);
        Assert.Equal(1, spline.Weights[1]);
        Assert.Equal(0.5, spline.Knots[1]);
    }

    [Fact]
    public void A_3D_spline_replaces_poles_and_knots_and_opens()
    {
        BSplineCurve3D spline = Spline3();
        BSplineCurve3D weighted = spline.WithPole(1, new Point3D(9, 9, 9), 2);
        Assert.Equal(new Point3D(9, 9, 9), weighted.Poles[1]);
        Assert.Equal(2, weighted.Weights[1]);
        Assert.Equal(0.25, spline.WithKnot(1, 0.25, 1).Knots[1]);

        BSplineCurve3D periodic = spline.WithPeriodic();
        Assert.True(periodic.IsPeriodic);
        Assert.False(periodic.WithNotPeriodic().IsPeriodic);
    }

    [Fact]
    public void A_Bezier_curve_replaces_poles_and_weights()
    {
        var bezier = new BezierCurve3D(Poles3);
        Assert.Equal(new Point3D(9, 9, 9), bezier.WithPole(1, new Point3D(9, 9, 9)).Poles[1]);
        BezierCurve3D weighted = bezier.WithPole(1, new Point3D(9, 9, 9), 2);
        Assert.Equal(new Point3D(9, 9, 9), weighted.Poles[1]);
        Assert.Equal(2, weighted.Weights[1]);
        Assert.Equal(3, bezier.WithWeight(1, 3).Weights[1]);
        Assert.Equal([0.0, 1.0], bezier.Knots);
        Assert.Equal([3, 3], bezier.Multiplicities);
        Assert.Equal([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], bezier.KnotSequence);
        BezierCurve3D half = bezier.Segmented(0, 0.5);
        Assert.True(half.Evaluate(1).IsEqual(bezier.Evaluate(0.5), 1e-12));

        var flat = new BezierCurve2D(Poles2);
        BezierCurve2D flatWeighted = flat.WithPole(1, new Point2D(9, 9), 2);
        Assert.Equal(new Point2D(9, 9), flatWeighted.Poles[1]);
        Assert.Equal(2, flatWeighted.Weights[1]);
        Assert.Equal(3, flat.WithWeight(1, 3).Weights[1]);
    }

    [Fact]
    public void A_spline_surface_replaces_poles_exchanges_and_closes()
    {
        BSplineSurface surface = SplineSurface();
        Assert.Equal(new Point3D(9, 9, 9), surface.WithPole(1, 1, new Point3D(9, 9, 9)).Poles[1, 1]);
        BSplineSurface weighted = surface.WithPole(1, 1, new Point3D(9, 9, 9), 2);
        Assert.Equal(2, weighted.Weights[1, 1]);
        Assert.Equal(3, surface.WithWeight(1, 1, 3).Weights[1, 1]);
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, surface.VKnotSequence);
        Assert.Equal(0.75, surface.VReversedParameter(0.25));

        // exchanged, u and v swap
        BSplineSurface exchanged = surface.ExchangedUV();
        Assert.True(exchanged.Evaluate(0.25, 0.75).IsEqual(surface.Evaluate(0.75, 0.25), 1e-12));

        BSplineSurface closedU = surface.WithUPeriodic();
        Assert.True(closedU.IsUPeriodic);
        Assert.False(closedU.WithUNotPeriodic().IsUPeriodic);
        BSplineSurface closedV = surface.WithVPeriodic();
        Assert.True(closedV.IsVPeriodic);
        Assert.False(closedV.WithVNotPeriodic().IsVPeriodic);
    }

    [Fact]
    public void A_Bezier_surface_replaces_poles_and_exchanges()
    {
        var surface = new BezierSurface(PolesGrid());
        BezierSurface weighted = surface.WithPole(1, 1, new Point3D(9, 9, 9), 2);
        Assert.Equal(new Point3D(9, 9, 9), weighted.Poles[1, 1]);
        Assert.Equal(2, weighted.Weights[1, 1]);
        Assert.Equal(3, surface.WithWeight(1, 1, 3).Weights[1, 1]);
        Assert.Equal(0.75, surface.UReversedParameter(0.25));
        Assert.Equal(0.75, surface.VReversedParameter(0.25));
        Assert.Equal(new[] { 3, 3 }, surface.VMultiplicities);
        Assert.True(surface.ExchangedUV().Evaluate(0.25, 0.75).IsEqual(surface.Evaluate(0.75, 0.25), 1e-12));
    }

    [Fact]
    public void Conic_curves_move_their_frames()
    {
        var circle = new CircleCurve3D(Frame3D.World, 2);
        var there = new Point3D(1, 2, 3);
        Assert.True(circle.WithLocation(there).Evaluate(0).IsEqual(new Point3D(3, 2, 3), 1e-12));
        Assert.True(circle.WithPosition(new Frame3D(there, Direction3D.UnitZ, Direction3D.UnitX)).Evaluate(0).IsEqual(new Point3D(3, 2, 3), 1e-12));
        Assert.Equal(1, circle.WithAxis(Axis3D.XAxis).Axis.Direction.X, 12);
        Assert.Equal(1, circle.YAxis.Direction.Y, 12);

        var flat = new CircleCurve2D(Frame2D.World, 2);
        Assert.Equal(1, flat.YAxis.Direction.Y, 12);
        var moved = new Frame2D(new Point2D(1, 2), new Direction2D(1, 0));
        Assert.True(flat.WithAxis(moved).Evaluate(0).IsEqual(new Point2D(3, 2), 1e-12));
        Assert.Equal(1, flat.WithXAxis(new Axis2D(Point2D.Origin, new Direction2D(0, 1))).XAxis.Direction.Y, 12);
        Assert.Equal(1, flat.WithYAxis(new Axis2D(Point2D.Origin, new Direction2D(1, 0))).YAxis.Direction.X, 12);
    }

    [Fact]
    public void Line_curves_replace_their_parts()
    {
        var line = new LineCurve3D(Point3D.Origin, Direction3D.UnitX);
        Assert.Equal(new Point3D(0, 2, 0), line.WithDirection(Direction3D.UnitY).Evaluate(2));
        Assert.Equal(new Point3D(0, 0, 2), line.WithPosition(Axis3D.ZAxis).Evaluate(2));
        Assert.Equal(new Point3D(0, 2, 0), line.WithLin(new Line3D(Axis3D.YAxis)).Evaluate(2));

        var flat = new LineCurve2D(Point2D.Origin, new Direction2D(1, 0));
        Assert.Equal(new Point2D(0, 2), flat.WithDirection(new Direction2D(0, 1)).Evaluate(2));
        Assert.Equal(new Point2D(0, 2), flat.WithPosition(Axis2D.YAxis).Evaluate(2));
        Assert.Equal(new Point2D(0, 2), flat.WithLin2d(new Line2D(Axis2D.YAxis)).Evaluate(2));
    }

    [Fact]
    public void Offset_curves_replace_their_parts()
    {
        var basis = new LineCurve3D(Point3D.Origin, Direction3D.UnitX);
        var offset = new OffsetCurve3D(basis, 2, Direction3D.UnitZ);
        Assert.True(offset.Evaluate(1).IsEqual(new Point3D(1, -2, 0), 1e-12));
        Assert.True(offset.WithOffsetValue(3).Evaluate(1).IsEqual(new Point3D(1, -3, 0), 1e-12));
        Assert.True(offset.WithDirection(new Direction3D(0, 0, -1)).Evaluate(1).IsEqual(new Point3D(1, 2, 0), 1e-12));
        Assert.True(offset.WithBasisCurve(new LineCurve3D(new Point3D(0, 1, 0), Direction3D.UnitX)).Evaluate(1).IsEqual(new Point3D(1, -1, 0), 1e-12));
        Assert.Equal(-1, offset.ReversedParameter(1));
        Assert.Equal(Continuity.CN, offset.Continuity);

        var round = new OffsetCurve3D(new CircleCurve3D(Frame3D.World, 2), 1, Direction3D.UnitZ);
        Assert.Equal(2 * Math.PI, round.Period, 12);

        var flat = new OffsetCurve2D(new LineCurve2D(Point2D.Origin, new Direction2D(1, 0)), 2);
        Assert.Equal(Continuity.CN, flat.Continuity);
        Assert.True(flat.WithBasisCurve(new LineCurve2D(new Point2D(0, 1), new Direction2D(1, 0))).Evaluate(1).IsEqual(new Point2D(1, -1), 1e-12));
        var flatRound = new OffsetCurve2D(new CircleCurve2D(Frame2D.World, 2), 1);
        Assert.Equal(2 * Math.PI, flatRound.Period, 12);
    }

    [Fact]
    public void Curves_report_their_range_and_derivatives()
    {
        var flat = new CircleCurve2D(Frame2D.World, 2);
        Assert.Equal(new Interval(0, 2 * Math.PI), flat.Range);
        Assert.True(flat.DerivativeN(0, 1).IsEqual(new Vector2D(0, 2), 1e-12, 1e-12));
        CurveDerivatives2D d = flat.Derivatives(0, 3);
        Assert.Equal(3, d.Order);
        Assert.True(d.Point.IsEqual(new Point2D(2, 0), 1e-12));
        Assert.True(d.D2.IsEqual(new Vector2D(-2, 0), 1e-12, 1e-12));
        Assert.True(d.D3.IsEqual(new Vector2D(0, -2), 1e-12, 1e-12));

        var trimmed = new TrimmedCurve3D(new LineCurve3D(Point3D.Origin, Direction3D.UnitX), 0, 2);
        Assert.Equal(3, trimmed.WithTrim(1, 3).LastParameter);
        Assert.Equal(-3, trimmed.ReversedParameter(3));

        Assert.True(((CircleCurve2D)new CircleCurve2D(new Frame2D(new Point2D(1, 2), new Direction2D(1, 0)), 2).Mirrored(new Point2D(0, 0))).Evaluate(0).IsEqual(new Point2D(-3, -2), 1e-12));
        Assert.True(((CircleCurve2D)flat.Mirrored(Axis2D.YAxis)).Evaluate(0).IsEqual(new Point2D(-2, 0), 1e-12));
        Assert.True(((CircleCurve2D)flat.Rotated(Point2D.Origin, Math.PI / 2)).Evaluate(0).IsEqual(new Point2D(0, 2), 1e-12));
        Assert.Equal(4, ((CircleCurve2D)flat.Scaled(Point2D.Origin, 2)).Radius, 12);
    }

    [Fact]
    public void Surfaces_reverse_their_parameters_and_move_their_frames()
    {
        var plane = new PlaneSurface(new Plane3D(CoordinateSystem.World));
        Assert.Equal(-1, plane.VReversedParameter(1));
        Assert.Equal(1, plane.WithAxis(Axis3D.XAxis).Axis.Direction.X, 12);
        var there = new CoordinateSystem(new Point3D(1, 2, 3), Direction3D.UnitZ, Direction3D.UnitX);
        Assert.Equal(new Point3D(1, 2, 3), plane.WithPosition(there).Evaluate(0, 0));

        var extrusion = new ExtrusionSurface(new LineCurve3D(Point3D.Origin, Direction3D.UnitX), Direction3D.UnitZ);
        Assert.Equal(-2, extrusion.VReversedParameter(2));

        var offset = new OffsetSurface(new SphericalSurface(CoordinateSystem.World, 2), 1);
        Assert.Equal(-1, offset.VReversedParameter(1));
        Assert.Equal(2 * Math.PI, new OffsetSurface(new ToroidalSurface(CoordinateSystem.World, 5, 2), 1).VPeriod, 12);
    }
}
