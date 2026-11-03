// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Points, vectors and directions, as a user calls them (docs/api/geometry.md).</summary>
public class PointVectorDirectionTests
{
    [Fact]
    public void A_point_has_its_coordinates_as_properties()
    {
        var p = new Point3D(1, 2, 3);
        Assert.Equal(1, p.X);
        Assert.Equal(2, p.Y);
        Assert.Equal(3, p.Z);
        var (x, y, z) = p;
        Assert.Equal((1.0, 2.0, 3.0), (x, y, z));
        Assert.Equal(new Point3D(1, 5, 3), p.WithY(5));
    }

    [Fact]
    public void Points_measure_distances_and_compare_exactly_or_within_a_tolerance()
    {
        var a = new Point3D(0, 0, 0);
        var b = new Point3D(3, 4, 0);
        Assert.Equal(5, a.DistanceTo(b));
        Assert.Equal(25, a.SquareDistanceTo(b));
        Assert.Equal(Point3D.Origin, a);
        Assert.True(a == new Point3D(0, 0, 0));
        Assert.False(a == new Point3D(1e-9, 0, 0));
        Assert.True(a.IsEqual(new Point3D(1e-9, 0, 0), Tolerances.Confusion));
    }

    [Fact]
    public void Points_and_vectors_combine_with_operators()
    {
        var a = new Point3D(1, 1, 1);
        var b = new Point3D(4, 5, 1);
        Vector3D ab = b - a;
        Assert.Equal(new Vector3D(3, 4, 0), ab);
        Assert.Equal(b, a + ab);
        Assert.Equal(a, b - ab);
        Assert.Equal(5, ab.Magnitude);
        Assert.Equal(25, ab.SquareMagnitude);
    }

    [Fact]
    public void Vectors_have_the_usual_algebra()
    {
        Vector3D x = Vector3D.UnitX;
        Vector3D y = Vector3D.UnitY;
        Assert.Equal(Vector3D.UnitZ, x ^ y);
        Assert.Equal(0, x * y);
        Assert.Equal(new Vector3D(2, 0, 0), 2 * x);
        Assert.Equal(new Vector3D(0.5, 0, 0), x / 2);
        Assert.Equal(new Vector3D(-1, 0, 0), -x);
        Assert.Equal(Math.PI / 2, x.AngleTo(y), 12);
        Assert.Equal(Vector3D.Zero, x - x);
        Assert.Equal(1, new Vector3D(3, 4, 12).Normalized().Magnitude, 12);
    }

    [Fact]
    public void A_direction_is_a_unit_vector()
    {
        var d = new Direction3D(0, 0, 5);
        Assert.Equal(Direction3D.UnitZ, d);
        Assert.Equal(1, d.Z);
        Assert.Equal(new Vector3D(0, 0, 1), d.ToVector());
        Assert.True(Direction3D.UnitX.IsNormal(Direction3D.UnitY, Tolerances.Angular));
        Assert.True(Direction3D.UnitX.IsOpposite(Direction3D.UnitX.Reversed(), Tolerances.Angular));
        Assert.Equal(Math.PI / 2, Direction3D.UnitX.AngleTo(Direction3D.UnitY), 12);
        Assert.Equal(Direction3D.UnitZ, Direction3D.UnitX ^ Direction3D.UnitY);
    }

    [Fact]
    public void Values_convert_to_System_Numerics_for_viewers()
    {
        Assert.Equal(new Vector3(1, 2, 3), new Point3D(1, 2, 3).ToVector3());
        Assert.Equal(new Vector3(1, 2, 3), new Vector3D(1, 2, 3).ToVector3());
        Assert.Equal(Vector3.UnitY, Direction3D.UnitY.ToVector3());
        Assert.Equal(new Vector2(1, 2), new Point2D(1, 2).ToVector2());
        Assert.Equal(new Vector2(1, 2), new Vector2D(1, 2).ToVector2());
    }

    [Fact]
    public void The_2D_twins_work_the_same_way()
    {
        var a = new Point2D(1, 1);
        var b = new Point2D(4, 5);
        Assert.Equal(5, a.DistanceTo(b));
        Vector2D ab = b - a;
        Assert.Equal(new Vector2D(3, 4), ab);
        Assert.Equal(b, a + ab);
        Assert.Equal(a, b - ab);
        Assert.Equal(Point2D.Origin, new Point2D(0, 0));
        var (x, y) = b;
        Assert.Equal((4.0, 5.0), (x, y));
        Assert.Equal(1, (Vector2D.UnitX ^ Vector2D.UnitY));
        Assert.Equal(Direction2D.UnitY, new Direction2D(0, 3));
        Assert.Equal(new Vector2D(0, 1), Direction2D.UnitY.ToVector());
        Assert.Equal(Vector2D.Zero, ab - ab);
    }

    [Fact]
    public void Values_move_without_changing()
    {
        var p = new Point3D(1, 0, 0);
        Point3D rotated = p.Rotated(Axis3D.ZAxis, Math.PI / 2);
        Assert.True(rotated.IsEqual(new Point3D(0, 1, 0), 1e-12));
        Assert.Equal(new Point3D(1, 0, 0), p);
        Assert.Equal(new Point3D(-1, 0, 0), p.Mirrored(Point3D.Origin));
        Assert.Equal(new Point3D(2, 0, 0), p.Scaled(Point3D.Origin, 2));
        Assert.Equal(new Point3D(1, 2, 0), p.Translated(new Vector3D(0, 2, 0)));
    }
}
