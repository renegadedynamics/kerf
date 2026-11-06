// Port of: src/FoundationClasses/TKMath/gp/gp_Lin2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Lin2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>An infinite line in 2D space, positioned by an axis (a location and a direction).</summary>
/// <remarks><c>new gp_Lin2d()</c> is the X axis, as in OCCT; <c>default(gp_Lin2d)</c> is not valid.</remarks>
public readonly partial struct Line2D : IEquatable<Line2D>
{
    private readonly Axis2D _pos;

    /// <summary>The X axis.</summary>
    public Line2D() => _pos = new Axis2D();

    /// <summary>The line along <paramref name="axis"/>.</summary>
    public Line2D(in Axis2D axis) => _pos = axis;

    /// <summary>The line through <paramref name="point"/> along <paramref name="direction"/>.</summary>
    public Line2D(in Point2D point, in Direction2D direction) => _pos = new Axis2D(point, direction);

    /// <summary>The line of equation <c>A * X + B * Y + C = 0</c>.</summary>
    /// <exception cref="ConstructionErrorException">A * A + B * B is at most <see cref="Constants.Resolution"/>.</exception>
    public Line2D(double a, double b, double c)
    {
        double norm2 = a * a + b * b;
        Refusal.ArgumentIf(norm2 <= Constants.Resolution(), FailureKind.ConstructionError, " ");
        Point2D p = new(-a * c / norm2, -b * c / norm2);
        Direction2D v = new(-b, a);
        _pos = new Axis2D(p, v);
    }

    // Copies

    /// <summary>A copy with this direction, through the same point.</summary>
    public readonly Line2D WithDirection(in Direction2D direction)
    {
        Axis2D pos = _pos;
        pos = pos.WithDirection(direction);
        return new Line2D(pos);
    }

    /// <summary>A copy through <paramref name="point"/>, with the same direction.</summary>
    public readonly Line2D WithLocation(in Point2D point)
    {
        Axis2D pos = _pos;
        pos = pos.WithLocation(point);
        return new Line2D(pos);
    }

    /// <summary>A copy positioned by <paramref name="axis"/>.</summary>
    public readonly Line2D WithPosition(in Axis2D axis) => new(axis);

    // Queries

    /// <summary>The coefficients of the line's equation <c>A * X + B * Y + C = 0</c>.</summary>
    public readonly void Coefficients(out double a, out double b, out double c)
    {
        a = _pos.Direction.Y;
        b = -_pos.Direction.X;
        c = -(a * _pos.Location.X + b * _pos.Location.Y);
    }

    /// <summary>The unit direction.</summary>
    public readonly Direction2D Direction => _pos.Direction;

    /// <summary>The point the line is positioned by, where its parameter is 0.</summary>
    public readonly Point2D Location => _pos.Location;

    /// <summary>The axis the line is positioned by, which gives its location and sense.</summary>
    public readonly Axis2D Position => _pos;

    /// <summary>The signed angle between the directions of the two lines, in ]-PI, PI].</summary>
    public readonly double AngleTo(in Line2D other) => _pos.Direction.AngleTo(other._pos.Direction);

    /// <summary>True when <paramref name="point"/> is within <paramref name="linearTolerance"/> of it.</summary>
    public readonly bool Contains(in Point2D point, double linearTolerance) => DistanceTo(point) <= linearTolerance;

    /// <summary>The distance from <paramref name="point"/> to the nearest point of the line.</summary>
    public readonly double DistanceTo(in Point2D point)
    {
        Xy coord = point.XY();
        coord.Subtract((_pos.Location).XY());
        return Math.Abs(coord.Crossed(_pos.Direction.XY()));
    }

    /// <summary>The distance between the lines: zero unless they are parallel.</summary>
    public readonly double DistanceTo(in Line2D other)
    {
        double d = 0.0;
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            d = other.DistanceTo(_pos.Location);
        }

        return d;
    }

    /// <summary>The square of the distance, without the square root where it can be avoided.</summary>
    public readonly double SquareDistanceTo(in Point2D point)
    {
        Xy coord = point.XY();
        coord.Subtract((_pos.Location).XY());
        double d = coord.Crossed(_pos.Direction.XY());
        return d * d;
    }

    /// <summary>The square of the distance between the lines: 0 unless they are parallel.</summary>
    public readonly double SquareDistanceTo(in Line2D other)
    {
        double d = 0.0;
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            d = other.SquareDistanceTo(_pos.Location);
        }

        return d;
    }

    /// <summary>The line through <paramref name="point"/> normal to this one.</summary>
    public readonly Line2D Normal(in Point2D point)
        => new(new Axis2D(point, new Direction2D(-(_pos.Direction.Y), _pos.Direction.X)));

    /// <summary>The same line, running the other way.</summary>
    public readonly Line2D Reversed()
    {
        Axis2D pos = _pos;
        pos = pos.Reversed();
        return new Line2D(pos);
    }

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Line2D Mirrored(in Point2D point)
    {
        Axis2D pos = _pos;
        pos = pos.Mirrored(point);
        return new Line2D(pos);
    }

    /// <summary>The reflection in the line <paramref name="axis"/>.</summary>
    public readonly Line2D Mirrored(in Axis2D axis)
    {
        Axis2D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Line2D(pos);
    }

    /// <summary>Rotated about <paramref name="point"/> by <paramref name="angle"/> radians, anticlockwise.</summary>
    public readonly Line2D Rotated(in Point2D point, double angle)
    {
        Axis2D pos = _pos;
        pos = pos.Rotated(point, angle);
        return new Line2D(pos);
    }

    /// <summary>The location scaled about <paramref name="point"/> by <paramref name="factor"/>. The direction stays, unless a negative factor reverses it.</summary>
    public readonly Line2D Scaled(in Point2D point, double factor)
    {
        Axis2D pos = _pos;
        pos = pos.Scaled(point, factor);
        return new Line2D(pos);
    }

    /// <summary>The image under <paramref name="transform"/>. The direction is transformed as a direction, so it stays a unit vector.</summary>
    public readonly Line2D Transformed(in Transform2D transform)
    {
        Axis2D pos = _pos;
        pos = pos.Transformed(transform);
        return new Line2D(pos);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Line2D Translated(in Vector2D vector)
    {
        Axis2D pos = _pos;
        pos = pos.Translated(vector);
        return new Line2D(pos);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Line2D Translated(in Point2D point1, in Point2D point2)
    {
        Axis2D pos = _pos;
        pos = pos.Translated(new Vector2D(point1, point2));
        return new Line2D(pos);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Line2D other) => _pos.Equals(other._pos);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Line2D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Line2D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => _pos.GetHashCode();

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Line2D left, in Line2D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Line2D left, in Line2D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Line2D({_pos.Location}, {_pos.Direction})";
}
