// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Geometry's operations (docs/api/geometry.md; unit 10L2): arcs, fitting, conversion, projection, intersection
/// and the splines' 0-based copies.</summary>
public class GeometryOperationTests
{
    private static readonly Plane3D XY = new(Point3D.Origin, Direction3D.UnitZ);

    [Fact]
    public void An_arc_through_three_points_is_a_trimmed_circle()
    {
        TrimmedCurve3D arc = TrimmedCurve3D.ArcThrough(new(0, 0, 0), new(5, 5, 0), new(10, 0, 0)).Value;
        Assert.Equal(CurveKind.Circle, arc.Kind);
        Assert.Equal(5, ((CircleCurve3D)arc.BasisCurve).Radius, 9);
        Assert.True(arc.StartPoint.IsEqual(new Point3D(0, 0, 0), 1e-9));
        Assert.True(arc.EndPoint.IsEqual(new Point3D(10, 0, 0), 1e-9));
        Assert.True(GeometryOperations.ArcThrough(new(0, 0, 0), new(5, 5, 0), new(10, 0, 0)).IsSuccess);
    }

    [Fact]
    public void Collinear_points_make_no_arc_and_say_why()
    {
        var arc = TrimmedCurve3D.ArcThrough(new(0, 0, 0), new(5, 0, 0), new(10, 0, 0));
        Assert.False(arc.IsSuccess);
        Assert.Equal(GeometryOperations.ConstructionFailed, arc.Error!.Code);
        Assert.Contains("collinear", arc.Error.Message, StringComparison.Ordinal);
        Assert.Equal(GeometryOperations.ConstructionFailed, TrimmedCurve3D.Segment(Point3D.Origin, Point3D.Origin).Error!.Code);
    }

    [Fact]
    public void Arcs_of_conics_and_segments_are_made_from_their_values()
    {
        var circle = new Circle3D(Frame3D.World, 2);
        TrimmedCurve3D quarter = TrimmedCurve3D.Arc(circle, 0, Math.PI / 2).Value;
        Assert.True(quarter.EndPoint.IsEqual(new Point3D(0, 2, 0), 1e-9));
        TrimmedCurve3D ellipse = TrimmedCurve3D.Arc(new Ellipse3D(Frame3D.World, 3, 1), 0, Math.PI).Value;
        Assert.Equal(CurveKind.Ellipse, ellipse.Kind);
        TrimmedCurve3D segment = TrimmedCurve3D.Segment(Point3D.Origin, new Point3D(3, 4, 0)).Value;
        Assert.Equal(CurveKind.Line, segment.Kind);
        Assert.Equal(5, segment.StartPoint.DistanceTo(segment.EndPoint), 12);
        TrimmedCurve3D tangent = TrimmedCurve3D.ArcTangent(Point3D.Origin, new Vector3D(1, 0, 0), new Point3D(2, 2, 0)).Value;
        Assert.Equal(2, ((CircleCurve3D)tangent.BasisCurve).Radius, 9);
    }

    [Fact]
    public void Interpolation_passes_through_the_points_with_the_tangents()
    {
        Point3D[] points = [new(0, 0, 0), new(10, 5, 0), new(20, 0, 0)];
        BSplineCurve3D curve = BSplineCurve3D.Interpolate(points).Value;
        foreach (Point3D p in points)
        {
            Assert.True(curve.Project(p)[0].Distance < 1e-7);
        }

        var withTangents = BSplineCurve3D.Interpolate(points, new InterpolationOptions { Tangents = [Vector3D.UnitX, Vector3D.UnitX] }).Value;
        Vector3D start = withTangents.Derivatives(withTangents.Range.Min, 1).D1;
        Assert.True(start.Y < 1e-9 && start.X > 0);
        Assert.Throws<ArgumentException>(() => BSplineCurve3D.Interpolate([Point3D.Origin]));
        Assert.Throws<ArgumentException>(() => BSplineCurve3D.Interpolate(points, new InterpolationOptions { Tangents = [Vector3D.UnitX] }));
    }

    [Fact]
    public void Coincident_points_fail_interpolation()
    {
        var curve = BSplineCurve3D.Interpolate([new(0, 0, 0), new(0, 0, 0), new(1, 0, 0)]);
        Assert.False(curve.IsSuccess);
        Assert.Equal(GeometryOperations.InterpolationFailed, curve.Error!.Code);
    }

    [Fact]
    public void Approximation_fits_within_its_tolerance()
    {
        Point3D[] points = [.. Enumerable.Range(0, 20).Select(i => new Point3D(i, Math.Sin(i / 3.0), 0))];
        BSplineCurve3D curve = BSplineCurve3D.Approximate(points, new ApproximationOptions { Tolerance = 1e-3 }).Value;
        Assert.All(points, p => Assert.True(curve.Project(p)[0].Distance < 2e-3));
        Assert.Throws<ArgumentOutOfRangeException>(() => BSplineCurve3D.Approximate(points, new ApproximationOptions { MinDegree = 5, MaxDegree = 3 }));

        var grid = new Grid<Point3D>(Enumerable.Range(0, 16).Select(i => new Point3D(i / 4, i % 4, (i / 4) * (i % 4) * 0.1)).ToArray(), 4, 4);
        BSplineSurface surface = BSplineSurface.Approximate(grid).Value;
        Assert.True(surface.Project(new Point3D(2, 2, 0.4))[0].Distance < 1e-3);
    }

    [Fact]
    public void Curves_join_end_to_end_or_say_which_doesnt_meet()
    {
        TrimmedCurve3D a = TrimmedCurve3D.Segment(new(0, 0, 0), new(1, 0, 0)).Value;
        TrimmedCurve3D b = TrimmedCurve3D.Segment(new(1, 0, 0), new(1, 1, 0)).Value;
        TrimmedCurve3D far = TrimmedCurve3D.Segment(new(5, 5, 0), new(6, 5, 0)).Value;
        BSplineCurve3D joined = BSplineCurve3D.Join([a, b]).Value;
        Assert.True(joined.EndPoint.IsEqual(new Point3D(1, 1, 0), 1e-9));
        var gap = BSplineCurve3D.Join([a, far]);
        Assert.Equal(GeometryOperations.NotConnected, gap.Error!.Code);
        Assert.Throws<ArgumentException>(() => BSplineCurve3D.Join([]));
    }

    [Fact]
    public void Curves_convert_to_BSplines_and_between_2D_and_3D()
    {
        var circle = new CircleCurve3D(new Circle3D(Frame3D.World, 1));
        BSplineCurve3D spline = circle.ToBSpline().Value;
        Assert.True(spline.Evaluate(spline.Range.Middle).DistanceTo(Point3D.Origin) - 1 < 1e-9);
        var line = new LineCurve3D(new Line3D(Point3D.Origin, Direction3D.UnitX));
        Assert.Equal(GeometryOperations.ConversionFailed, line.ToBSpline().Error!.Code);

        Curve2D flat = circle.ToCurve2D(XY).Value;
        Curve3D back = flat.ToCurve3D(XY);
        Assert.True(back.Evaluate(0).IsEqual(circle.Evaluate(0), 1e-12));
    }

    [Fact]
    public void Projections_are_nearest_first()
    {
        var circle = new CircleCurve3D(new Circle3D(Frame3D.World, 1));
        var projected = circle.Project(new Point3D(2, 0, 0));
        Assert.Equal(2, projected.Length);
        Assert.Equal(1, projected[0].Distance, 12);
        Assert.Equal(3, projected[1].Distance, 12);
        Assert.True(circle.TryProjectNearest(new Point3D(0, 3, 0), out CurvePoint nearest));
        Assert.Equal(Math.PI / 2, nearest.Parameter, 9);

        var plane = new PlaneSurface(XY);
        Assert.True(plane.TryProjectNearest(new Point3D(1, 2, 3), out SurfacePoint onPlane));
        Assert.Equal(3, onPlane.Distance, 12);
        Assert.Equal(new Point3D(1, 2, 0), onPlane.Point);
    }

    [Fact]
    public void Intersections_give_points_and_curves()
    {
        var line = new LineCurve3D(new Line3D(new Point3D(0, 0, -5), Direction3D.UnitZ));
        var sphere = new SphericalSurface(new Sphere(CoordinateSystem.World, 2));
        var hits = line.Intersect(sphere).Value;
        Assert.Equal(2, hits.Length);
        Assert.All(hits, h => Assert.Equal(2, Math.Abs(h.Point.Z), 9));

        var plane = new PlaneSurface(XY);
        var circles = plane.Intersect(sphere).Value;
        Assert.Single(circles);
        Assert.Equal(CurveKind.Circle, circles[0].Kind);

        var a = new LineCurve2D(new Line2D(Point2D.Origin, Direction2D.UnitX));
        var b = new LineCurve2D(new Line2D(new Point2D(3, -1), Direction2D.UnitY));
        var crossing = a.Intersect(b).Value;
        Assert.Single(crossing);
        Assert.Equal(3, crossing[0].FirstParameter, 9);
        Assert.Equal(1, crossing[0].SecondParameter, 9);
    }

    [Fact]
    public void A_surface_says_whether_it_is_planar()
    {
        Assert.True(new PlaneSurface(XY).TryGetPlane(Tolerances.Confusion, out Plane3D plane));
        Assert.True(plane.Axis.Direction.IsParallel(Direction3D.UnitZ, 1e-12));
        Assert.False(new SphericalSurface(new Sphere(CoordinateSystem.World, 2)).TryGetPlane(1e-3, out _));
    }

    [Fact]
    public void The_splines_copies_are_indexed_from_zero()
    {
        var spline = new BSplineCurve3D([new(0, 0, 0), new(1, 1, 0), new(2, 0, 0)], [0, 1], [3, 3], 2);
        BSplineCurve3D moved = spline.WithPole(1, new Point3D(1, 2, 0));
        Assert.Equal(new Point3D(1, 2, 0), moved.Poles[1]);
        Assert.Equal(new Point3D(1, 1, 0), spline.Poles[1]);
        BSplineCurve3D weighted = spline.WithWeight(0, 2);
        Assert.True(weighted.IsRational);
        Assert.Equal(2, weighted.Weights[0]);
        Assert.Throws<ArgumentOutOfRangeException>(() => spline.WithPole(3, Point3D.Origin));
        Assert.Throws<ArgumentOutOfRangeException>(() => spline.WithWeight(0, 0));
        Assert.Equal(-1, spline.WithKnot(0, -1).Knots[0]);
        Assert.Throws<ArgumentException>(() => spline.WithKnot(0, 2));

        var bezier = new BezierCurve2D([new(0, 0), new(1, 1), new(2, 0)]);
        Assert.Equal(new Point2D(1, 3), bezier.WithPole(1, new Point2D(1, 3)).Poles[1]);

        var surface = new BezierSurface(new Grid<Point3D>(new Point3D[] { new(0, 0, 0), new(0, 1, 0), new(1, 0, 0), new(1, 1, 0) }, 2, 2));
        Assert.Equal(new Point3D(1, 1, 5), surface.WithPole(1, 1, new Point3D(1, 1, 5)).Poles[1, 1]);
        Assert.Throws<ArgumentOutOfRangeException>(() => surface.WithPole(2, 0, Point3D.Origin));
    }
}
