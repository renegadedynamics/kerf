// Port of: src/FoundationClasses/TKMath/gp/gp_Lin.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Lin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>An infinite line in 3D space, positioned by an axis (a location and a unit direction).</summary>
/// <remarks><c>new gp_Lin()</c> is the Z axis, as in OCCT; <c>default(gp_Lin)</c> is not valid.</remarks>
public readonly partial struct Line3D : IEquatable<Line3D>
{
    private readonly Axis3D _pos;

    public Line3D() => _pos = new Axis3D();

    public Line3D(in Axis3D axis) => _pos = axis;

    public Line3D(in Point3D point, in Direction3D direction) => _pos = new Axis3D(point, direction);

    // Copies

    public readonly Line3D WithDirection(in Direction3D direction)
    {
        Axis3D pos = _pos;
        pos = pos.WithDirection(direction);
        return new Line3D(pos);
    }

    public readonly Line3D WithLocation(in Point3D point)
    {
        Axis3D pos = _pos;
        pos = pos.WithLocation(point);
        return new Line3D(pos);
    }

    public readonly Line3D WithPosition(in Axis3D axis) => new(axis);

    // Queries

    public readonly Line3D Reversed()
    {
        Axis3D pos = _pos;
        pos = pos.Reversed();
        return new Line3D(pos);
    }

    public readonly Direction3D Direction => _pos.Direction;

    public readonly Point3D Location => _pos.Location;

    public readonly Axis3D Position => _pos;

    /// <summary>The angle between the directions of the two lines, in [0, PI].</summary>
    public readonly double AngleTo(in Line3D other) => _pos.Direction.AngleTo(other._pos.Direction);

    public readonly bool Contains(in Point3D point, double linearTolerance) => DistanceTo(point) <= linearTolerance;

    public readonly double DistanceTo(in Point3D point)
    {
        Xyz coord = point.XYZ();
        coord.Subtract((_pos.Location).XYZ());
        coord.Cross((_pos.Direction).XYZ());
        return coord.Modulus();
    }

    /// <summary>The minimum distance between the two lines.</summary>
    public readonly double DistanceTo(in Line3D other)
    {
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            return other.DistanceTo(_pos.Location);
        }
        else
        {
            Direction3D dir = _pos.Direction.Crossed(other._pos.Direction);
            return Math.Abs(new Vector3D(_pos.Location, other._pos.Location).Dot(new Vector3D(dir)));
        }
    }

    public readonly double SquareDistanceTo(in Point3D point)
    {
        Point3D loc = _pos.Location;
        Vector3D v = new(point.X - loc.X, point.Y - loc.Y, point.Z - loc.Z);
        v = v.Crossed(new Vector3D(_pos.Direction));
        return v.SquareMagnitude;
    }

    public readonly double SquareDistanceTo(in Line3D other)
    {
        double d = DistanceTo(other);
        return d * d;
    }

    /// <summary>The line through <paramref name="point"/> normal to this line and intersecting it.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="point"/> is on this line.</exception>
    public readonly Line3D Normal(in Point3D point)
    {
        Point3D loc = _pos.Location;
        Direction3D v = new(point.X - loc.X, point.Y - loc.Y, point.Z - loc.Z);
        v = _pos.Direction.CrossCrossed(v, _pos.Direction);
        return new Line3D(point, v);
    }

    public readonly Line3D Mirrored(in Point3D point)
    {
        Axis3D pos = _pos;
        pos = pos.Mirrored(point);
        return new Line3D(pos);
    }

    public readonly Line3D Mirrored(in Axis3D axis)
    {
        Axis3D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Line3D(pos);
    }

    public readonly Line3D Mirrored(in Frame3D frame)
    {
        Axis3D pos = _pos;
        pos = pos.Mirrored(frame);
        return new Line3D(pos);
    }

    public readonly Line3D Rotated(in Axis3D axis, double angle)
    {
        Axis3D pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Line3D(pos);
    }

    public readonly Line3D Scaled(in Point3D point, double factor)
    {
        Axis3D pos = _pos;
        pos = pos.Scaled(point, factor);
        return new Line3D(pos);
    }

    public readonly Line3D Transformed(in Transform3D transform)
    {
        Axis3D pos = _pos;
        pos = pos.Transformed(transform);
        return new Line3D(pos);
    }

    public readonly Line3D Translated(in Vector3D vector)
    {
        Axis3D pos = _pos;
        pos = pos.Translated(vector);
        return new Line3D(pos);
    }

    public readonly Line3D Translated(in Point3D point1, in Point3D point2)
    {
        Axis3D pos = _pos;
        pos = pos.Translated(new Vector3D(point1, point2));
        return new Line3D(pos);
    }

    public readonly bool Equals(Line3D other) => _pos.Equals(other._pos);

    public override readonly bool Equals(object? obj) => obj is Line3D other && Equals(other);

    public override readonly int GetHashCode() => _pos.GetHashCode();

    public static bool operator ==(in Line3D left, in Line3D right) => left.Equals(right);

    public static bool operator !=(in Line3D left, in Line3D right) => !left.Equals(right);

    public override readonly string ToString() => $"Line3D({_pos.Location}, {_pos.Direction})";
}
