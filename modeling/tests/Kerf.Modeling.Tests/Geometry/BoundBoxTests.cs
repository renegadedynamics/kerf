// Ported from build123d tests/test_direct_api/test_bound_box.py. Boxes of shapes follow with the
// topology port; here boxes are built from their corners, as the Python predicate tests do.

namespace Kerf.Modeling.Tests.Geometry;

using Kerf.Modeling;

public class BoundBoxTests
{
    static BoundingBox Box(Point3D min, Point3D max)
    {
        var box = new global::Kerf.Geometry.BoundingBox(new global::Kerf.Geometry.Point3D(min.X, min.Y, min.Z),
                                                       new global::Kerf.Geometry.Point3D(max.X, max.Y, max.Z));
        return box;
    }

    [Fact]
    public void EmptyBoxPredicates()
    {
        var empty = BoundingBox.Empty;
        var box = Box(new Point3D(0, 0, 0), new Point3D(1, 1, 1));
        Assert.True(empty.IsEmpty);
        Assert.False(empty.Covers(box));
        Assert.False(empty.CoveredBy(box));
        Assert.False(empty.Contains(box));
        Assert.False(empty.ContainsProperly(box));
        Assert.False(empty.Within(box));
        Assert.False(empty.IntersectsNear(box));
        Assert.True(empty.Disjoint(box));
        Assert.False(empty.Touches(box));
        Assert.False(empty.Overlaps(box));
        Assert.Equal(0.0, empty.Diagonal);
        Check.Near(new Vector3D(0, 0, 0), empty.Size);
    }

    [Fact]
    public void Add()
    {
        var bb1 = Box(new Point3D(1, 1, 1), new Point3D(1, 1, 1)).Union(Box(new Point3D(2, 2, 2), new Point3D(2, 2, 2)));
        Check.AlmostEqual(1.0, bb1.Size.X, 1);   // the default gap enlarges the box a little

        var bb2 = Box(new Point3D(0, 0, 0), new Point3D(0, 0, 0)).Union(Box(new Point3D(1, 1, 1), new Point3D(1, 1, 1)));
        var bb3 = bb1.Union(bb2);
        Check.AlmostEqual(new Vector3D(2, 2, 2), bb3.Size, 1);

        bb3 = bb2.Including(new Point3D(3, 3, 3));
        Check.AlmostEqual(new Vector3D(3, 3, 3), bb3.Size, 7);
        Check.AlmostEqual(27, bb3.Measure, 5);

        // The original is unchanged
        Check.AlmostEqual(new Vector3D(1, 1, 1), bb2.Size, 7);

        var flat = Box(new Point3D(0, 0, 0), new Point3D(0, 0, 0)).Union(Box(new Point3D(3, 3, 0), new Point3D(3, 3, 0)));
        Check.AlmostEqual(9, flat.Measure, 5);
    }

    [Fact]
    public void SpatialPredicates()
    {
        var outer = Box(new Point3D(0, 0, 0), new Point3D(3, 3, 3));
        var inner = Box(new Point3D(1, 1, 1), new Point3D(2, 2, 2));
        var partial = Box(new Point3D(2, 2, 2), new Point3D(4, 4, 4));
        var touching = Box(new Point3D(3, 0, 0), new Point3D(4, 1, 1));
        var disjoint = Box(new Point3D(4, 0, 0), new Point3D(5, 1, 1));
        var boundaryFace = Box(new Point3D(0, 0, 0), new Point3D(3, 3, 0));
        var near = Box(new Point3D(3 + 0.5e-6, 0, 0), new Point3D(4, 1, 1));

        Assert.True(outer.Covers(inner));
        Assert.True(inner.CoveredBy(outer));
        Assert.True(outer.Contains(inner));
        Assert.True(inner.Within(outer));
        Assert.True(outer.ContainsProperly(inner));
        Assert.True(outer.Contains(outer));
        Assert.True(outer.Within(outer));
        Assert.False(outer.ContainsProperly(outer));
        Assert.True(outer.Covers(boundaryFace));
        Assert.True(boundaryFace.CoveredBy(outer));
        Assert.False(outer.Contains(boundaryFace));
        Assert.False(boundaryFace.Within(outer));

        Assert.True(outer.IntersectsNear(partial));
        Assert.True(outer.Overlaps(partial));
        Assert.False(outer.Overlaps(outer));
        Assert.False(outer.Contains(partial));
        Assert.False(partial.Contains(outer));

        Assert.True(outer.IntersectsNear(touching));
        Assert.True(outer.Touches(touching));
        Assert.False(outer.Overlaps(touching));
        Assert.True(outer.Disjoint(disjoint));
        Assert.False(outer.IntersectsNear(disjoint));

        Assert.True(outer.IntersectsNear(near));
        Assert.False(outer.Overlaps(near));
        Assert.True(outer.Disjoint(near, tolerance: 0.0));

        var diagonalNear = Box(new Point3D(3 + 0.8e-6, 3 + 0.8e-6, 0), new Point3D(4, 4, 1));
        Assert.False(outer.IntersectsNear(diagonalNear));
        Assert.True(outer.Disjoint(diagonalNear));
    }

    [Fact]
    public void LowerDimensionalPredicates()
    {
        var outerRectangle = Box(new Point3D(0, 0, 0), new Point3D(3, 3, 0));
        var innerRectangle = Box(new Point3D(1, 1, 0), new Point3D(2, 2, 0));
        var partialRectangle = Box(new Point3D(2, 2, 0), new Point3D(4, 4, 0));
        var touchingRectangle = Box(new Point3D(3, 0, 0), new Point3D(4, 1, 0));
        var parallelRectangle = Box(new Point3D(0, 0, 1), new Point3D(3, 3, 1));

        Assert.True(outerRectangle.Contains(innerRectangle));
        Assert.True(outerRectangle.ContainsProperly(innerRectangle));
        Assert.True(innerRectangle.Within(outerRectangle));
        Assert.True(outerRectangle.Overlaps(partialRectangle));
        Assert.True(outerRectangle.Touches(touchingRectangle));
        Assert.True(outerRectangle.Disjoint(parallelRectangle));
        Assert.False(outerRectangle.Overlaps(parallelRectangle));

        var outerLine = Box(new Point3D(0, 0, 0), new Point3D(3, 0, 0));
        var innerLine = Box(new Point3D(1, 0, 0), new Point3D(2, 0, 0));
        var partialLine = Box(new Point3D(2, 0, 0), new Point3D(4, 0, 0));
        var touchingLine = Box(new Point3D(3, 0, 0), new Point3D(4, 0, 0));

        Assert.True(outerLine.Contains(innerLine));
        Assert.True(outerLine.ContainsProperly(innerLine));
        Assert.True(outerLine.Overlaps(partialLine));
        Assert.True(outerLine.Touches(touchingLine));
    }

    [Fact]
    public void Formatting() =>
        Assert.Equal("bbox: 0 <= x <= 1, 0 <= y <= 1, 0 <= z <= 1", Box(new Point3D(0, 0, 0), new Point3D(1, 1, 1)).Describe());

    [Fact]
    public void CenterAndAlignment()
    {
        var box = Box(new Point3D(0, 0, 0), new Point3D(2, 4, 6));
        Check.AlmostEqual(new Point3D(1, 2, 3), box.Center());
        Check.AlmostEqual(new Vector3D(-1, -2, -3), box.ToAlignOffset(Align.Center));
        Check.AlmostEqual(new Vector3D(0, 0, 0), box.ToAlignOffset(Align.Min));
        Check.AlmostEqual(new Vector3D(-2, -4, -6), box.ToAlignOffset(Align.Max));
        Check.AlmostEqual(new Vector3D(0, -2, 0), box.ToAlignOffset(new Alignment(Align.Min, Align.Center, Align.None)));
        Check.AlmostEqual(new Vector3D(-2, 0, 0), box.ToAlignOffset(new Alignment(Align.Max, Align.Min)));
        Check.AlmostEqual(Math.Sqrt(4 + 16 + 36), box.Diagonal);
    }

    [Fact]
    public void ToAndFromLocalCoordinates()
    {
        var local = Plane.XY.Offset(1).ToLocalCoords(Box(new Point3D(0, 0, 0), new Point3D(1, 1, 1)));
        Check.AlmostEqual(new Point3D(0, 0, -1), local.Min);
        Check.AlmostEqual(new Point3D(1, 1, 0), local.Max);
    }
}
