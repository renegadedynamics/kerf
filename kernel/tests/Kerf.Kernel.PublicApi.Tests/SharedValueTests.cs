// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Intervals, tolerances, bounding boxes and the derivative records (docs/api/README.md, sections 3 and 4).</summary>
public class SharedValueTests
{
    [Fact]
    public void An_interval_measures_and_clamps()
    {
        var range = new Interval(2, 6);
        Assert.Equal(4, range.Length);
        Assert.Equal(4, range.Middle);
        Assert.Equal(3, range.At(0.25));
        Assert.True(range.Contains(6));
        Assert.False(range.Contains(6.1));
        Assert.True(range.Contains(6.1, tolerance: 0.2));
        Assert.Equal(2, range.Clamp(-1));
        Assert.Equal(6, range.Clamp(9));
        Assert.False(range.IsInfinite);
        Assert.True(new Interval(double.NegativeInfinity, 0).IsInfinite);
        Assert.True(new Interval(-Tolerances.Infinite, Tolerances.Infinite).IsInfinite);
        Assert.Equal(new Interval(2, 6), range);
    }

    [Fact]
    public void The_tolerances_are_OCCTs_precisions()
    {
        Assert.Equal(1e-7, Tolerances.Confusion);
        Assert.Equal(1e-12, Tolerances.Angular);
        Assert.Equal(Tolerances.Confusion * 0.01, Tolerances.Parametric);
        Assert.Equal(Tolerances.Confusion * 10.0, Tolerances.Approximation);
        Assert.Equal(Tolerances.Confusion * 0.01, Tolerances.Intersection);
        Assert.Equal(2e100, Tolerances.Infinite);
    }

    [Fact]
    public void An_empty_box_is_the_default_and_the_unit_of_union()
    {
        BoundingBox empty = default;
        Assert.True(empty.IsEmpty);
        Assert.Equal(BoundingBox.Empty, empty);
        Assert.Equal(Vector3D.Zero, empty.Size);
        Assert.False(empty.Contains(Point3D.Origin));
        Assert.Throws<InvalidOperationException>(() => empty.Min);
        Assert.Throws<InvalidOperationException>(() => empty.Centre);
        var box = new BoundingBox(new Point3D(0, 0, 0), new Point3D(1, 2, 3));
        Assert.Equal(box, empty.Union(box));
        Assert.Equal(box, box.Union(empty));
        Assert.Equal(empty, empty.Enlarged(1));
    }

    [Fact]
    public void A_box_grows_moves_and_tests_containment()
    {
        BoundingBox box = BoundingBox.Of(new Point3D(1, 2, 3), new Point3D(-1, 0, 5));
        Assert.Equal(new Point3D(-1, 0, 3), box.Min);
        Assert.Equal(new Point3D(1, 2, 5), box.Max);
        Assert.Equal(new Vector3D(2, 2, 2), box.Size);
        Assert.Equal(new Point3D(0, 1, 4), box.Centre);
        Assert.True(box.Contains(new Point3D(0, 1, 4)));
        Assert.False(box.Contains(new Point3D(0, 1, 6)));
        Assert.Equal(new Point3D(-2, -1, 2), box.Enlarged(1).Min);
        Assert.True(box.Intersects(new BoundingBox(new Point3D(1, 2, 5), new Point3D(9, 9, 9))));
        Assert.False(box.Intersects(new BoundingBox(new Point3D(1.1, 2, 5), new Point3D(9, 9, 9))));
        BoundingBox moved = box.Transformed(Transform3D.Translation(new Vector3D(10, 0, 0)));
        Assert.Equal(new Point3D(9, 0, 3), moved.Min);
        Assert.Equal(new Point3D(1, 2, 5), box.Including(new Point3D(0, 0, 4)).Max);
        Assert.False(box.IsInfinite);
        Assert.Throws<ArgumentException>(() => new BoundingBox(new Point3D(1, 0, 0), Point3D.Origin));
        Assert.Throws<ArgumentOutOfRangeException>(() => box.Enlarged(-1));
    }

    [Fact]
    public void The_derivative_records_hold_what_was_computed()
    {
        var d = new CurveDerivatives(1, new Point3D(1, 0, 0), new Vector3D(0, 1, 0));
        Assert.Equal(1, d.Order);
        Assert.Equal(Vector3D.Zero, d.D2);
        var s = new SurfaceDerivatives(0, Point3D.Origin);
        Assert.Equal(Vector3D.Zero, s.DU);
        var point = new CurvePoint(0.5, new Point3D(1, 1, 1), 2);
        Assert.Equal(2, point.Distance);
        Assert.Equal(0, new SurfacePoint(1, 2, Point3D.Origin).Distance);
    }

    [Fact]
    public void A_grid_is_indexed_from_zero_by_row_and_column()
    {
        var grid = new Grid<int>(new[] { 1, 2, 3, 4, 5, 6 }, 2, 3);
        Assert.Equal(2, grid.RowCount);
        Assert.Equal(3, grid.ColumnCount);
        Assert.Equal(1, grid[0, 0]);
        Assert.Equal(6, grid[1, 2]);
        Assert.Throws<ArgumentOutOfRangeException>(() => grid[2, 0]);
        Assert.True(default(Grid<int>).IsEmpty);
    }
}
