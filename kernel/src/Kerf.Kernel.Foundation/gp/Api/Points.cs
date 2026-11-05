// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L1: the public surface the points, vectors and directions add to OCCT's (docs/api/geometry.md).

using System.Numerics;

namespace Kerf.Geometry;

public readonly partial struct Point3D : IGeometry
{
    /// <summary>The origin, (0, 0, 0).</summary>
    public static Point3D Origin => default;

    /// <summary>The coordinates, for deconstruction: <c>var (x, y, z) = point;</c>.</summary>
    public void Deconstruct(out double x, out double y, out double z) => (x, y, z) = (X, Y, Z);

    /// <summary>The point as a single-precision <see cref="Vector3"/>, for viewers.</summary>
    public Vector3 ToVector3() => new((float)X, (float)Y, (float)Z);

    /// <summary>The vector from <paramref name="start"/> to <paramref name="end"/>.</summary>
    public static Vector3D operator -(in Point3D end, in Point3D start) => new(start, end);

    /// <summary>The point moved by <paramref name="offset"/>.</summary>
    public static Point3D operator +(in Point3D point, in Vector3D offset) => point.Translated(offset);

    /// <summary>The point moved by the opposite of <paramref name="offset"/>.</summary>
    public static Point3D operator -(in Point3D point, in Vector3D offset) => point.Translated(-offset);
}

public readonly partial struct Point2D
{
    /// <summary>The origin, (0, 0).</summary>
    public static Point2D Origin => default;

    /// <summary>The coordinates, for deconstruction: <c>var (x, y) = point;</c>.</summary>
    public void Deconstruct(out double x, out double y) => (x, y) = (X, Y);

    /// <summary>The point as a single-precision <see cref="Vector2"/>, for viewers.</summary>
    public Vector2 ToVector2() => new((float)X, (float)Y);

    /// <summary>The vector from <paramref name="start"/> to <paramref name="end"/>.</summary>
    public static Vector2D operator -(in Point2D end, in Point2D start) => new(start, end);

    /// <summary>The point moved by <paramref name="offset"/>.</summary>
    public static Point2D operator +(in Point2D point, in Vector2D offset) => point.Translated(offset);

    /// <summary>The point moved by the opposite of <paramref name="offset"/>.</summary>
    public static Point2D operator -(in Point2D point, in Vector2D offset) => point.Translated(-offset);
}

public readonly partial struct Vector3D
{
    /// <summary>The zero vector.</summary>
    public static Vector3D Zero => default;

    /// <summary>(1, 0, 0).</summary>
    public static Vector3D UnitX => new(1, 0, 0);

    /// <summary>(0, 1, 0).</summary>
    public static Vector3D UnitY => new(0, 1, 0);

    /// <summary>(0, 0, 1).</summary>
    public static Vector3D UnitZ => new(0, 0, 1);

    /// <summary>The components, for deconstruction: <c>var (x, y, z) = vector;</c>.</summary>
    public void Deconstruct(out double x, out double y, out double z) => (x, y, z) = (X, Y, Z);

    /// <summary>The vector as a single-precision <see cref="Vector3"/>, for viewers.</summary>
    public Vector3 ToVector3() => new((float)X, (float)Y, (float)Z);
}

public readonly partial struct Vector2D
{
    /// <summary>The zero vector.</summary>
    public static Vector2D Zero => default;

    /// <summary>(1, 0).</summary>
    public static Vector2D UnitX => new(1, 0);

    /// <summary>(0, 1).</summary>
    public static Vector2D UnitY => new(0, 1);

    /// <summary>The components, for deconstruction: <c>var (x, y) = vector;</c>.</summary>
    public void Deconstruct(out double x, out double y) => (x, y) = (X, Y);

    /// <summary>The vector as a single-precision <see cref="Vector2"/>, for viewers.</summary>
    public Vector2 ToVector2() => new((float)X, (float)Y);
}

public readonly partial struct Direction3D
{
    /// <summary>The vector of unit length along this direction: nothing is lost, so it converts implicitly.</summary>
    public static implicit operator Vector3D(in Direction3D direction) => new(direction);

    /// <summary>The vector along this direction with a length of <paramref name="scalar"/>.</summary>
    public static Vector3D operator *(in Direction3D direction, double scalar) => new Vector3D(direction) * scalar;

    /// <summary>The vector along this direction with a length of <paramref name="scalar"/>.</summary>
    public static Vector3D operator *(double scalar, in Direction3D direction) => new Vector3D(direction) * scalar;

    /// <summary>The direction of the X axis, (1, 0, 0).</summary>
    public static Direction3D UnitX => new(D.X);

    /// <summary>The direction of the Y axis, (0, 1, 0).</summary>
    public static Direction3D UnitY => new(D.Y);

    /// <summary>The direction of the Z axis, (0, 0, 1).</summary>
    public static Direction3D UnitZ => new(D.Z);

    /// <summary>The components, for deconstruction: <c>var (x, y, z) = direction;</c>.</summary>
    public void Deconstruct(out double x, out double y, out double z) => (x, y, z) = (X, Y, Z);

    /// <summary>The direction as a unit vector.</summary>
    public Vector3D ToVector() => new(this);

    /// <summary>The direction as a single-precision <see cref="Vector3"/>, for viewers.</summary>
    public Vector3 ToVector3() => new((float)X, (float)Y, (float)Z);
}

public readonly partial struct Direction2D
{
    /// <summary>The direction of the X axis, (1, 0).</summary>
    public static Direction2D UnitX => new(1, 0);

    /// <summary>The direction of the Y axis, (0, 1).</summary>
    public static Direction2D UnitY => new(0, 1);

    /// <summary>The components, for deconstruction: <c>var (x, y) = direction;</c>.</summary>
    public void Deconstruct(out double x, out double y) => (x, y) = (X, Y);

    /// <summary>The direction as a unit vector.</summary>
    public Vector2D ToVector() => new(this);
}
