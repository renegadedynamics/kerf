// Port of: src/FoundationClasses/TKMath/gp/gp_Lin2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Lin2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>An infinite line in 2D space, positioned by an axis (a location and a direction).</summary>
/// <remarks><c>new gp_Lin2d()</c> is the X axis, as in OCCT; <c>default(gp_Lin2d)</c> is not valid.</remarks>
public readonly partial struct Line2D : IEquatable<Line2D>
{
    private readonly Axis2D _pos;

    public Line2D() => _pos = new Axis2D();

    public Line2D(in Axis2D a) => _pos = a;

    public Line2D(in Point2D p, in Direction2D v) => _pos = new Axis2D(p, v);

    /// <summary>The line of equation <c>A * X + B * Y + C = 0</c>.</summary>
    /// <exception cref="ConstructionErrorException">A * A + B * B is at most <see cref="Constants.Resolution"/>.</exception>
    public Line2D(double a, double b, double c)
    {
        double norm2 = a * a + b * b;
        Standard_ConstructionError_Raise_if(norm2 <= Constants.Resolution(), " ");
        Point2D p = new(-a * c / norm2, -b * c / norm2);
        Direction2D v = new(-b, a);
        _pos = new Axis2D(p, v);
    }

    // Copies

    public readonly Line2D WithDirection(in Direction2D v)
    {
        Axis2D pos = _pos;
        pos = pos.WithDirection(v);
        return new Line2D(pos);
    }

    public readonly Line2D WithLocation(in Point2D p)
    {
        Axis2D pos = _pos;
        pos = pos.WithLocation(p);
        return new Line2D(pos);
    }

    public readonly Line2D WithPosition(in Axis2D a) => new(a);

    // Queries

    /// <summary>The coefficients of the line's equation <c>A * X + B * Y + C = 0</c>.</summary>
    public readonly void Coefficients(out double a, out double b, out double c)
    {
        a = _pos.Direction.Y;
        b = -_pos.Direction.X;
        c = -(a * _pos.Location.X + b * _pos.Location.Y);
    }

    public readonly Direction2D Direction => _pos.Direction;

    public readonly Point2D Location => _pos.Location;

    public readonly Axis2D Position => _pos;

    /// <summary>The signed angle between the directions of the two lines, in ]-PI, PI].</summary>
    public readonly double AngleTo(in Line2D other) => _pos.Direction.AngleTo(other._pos.Direction);

    public readonly bool Contains(in Point2D p, double linearTolerance) => DistanceTo(p) <= linearTolerance;

    public readonly double DistanceTo(in Point2D p)
    {
        Xy coord = p.XY();
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

    public readonly double SquareDistanceTo(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract((_pos.Location).XY());
        double d = coord.Crossed(_pos.Direction.XY());
        return d * d;
    }

    public readonly double SquareDistanceTo(in Line2D other)
    {
        double d = 0.0;
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            d = other.SquareDistanceTo(_pos.Location);
        }

        return d;
    }

    /// <summary>The line through <paramref name="p"/> normal to this one.</summary>
    public readonly Line2D Normal(in Point2D p)
        => new(new Axis2D(p, new Direction2D(-(_pos.Direction.Y), _pos.Direction.X)));

    public readonly Line2D Reversed()
    {
        Axis2D pos = _pos;
        pos = pos.Reversed();
        return new Line2D(pos);
    }

    public readonly Line2D Mirrored(in Point2D p)
    {
        Axis2D pos = _pos;
        pos = pos.Mirrored(p);
        return new Line2D(pos);
    }

    public readonly Line2D Mirrored(in Axis2D a)
    {
        Axis2D pos = _pos;
        pos = pos.Mirrored(a);
        return new Line2D(pos);
    }

    public readonly Line2D Rotated(in Point2D p, double ang)
    {
        Axis2D pos = _pos;
        pos = pos.Rotated(p, ang);
        return new Line2D(pos);
    }

    public readonly Line2D Scaled(in Point2D p, double s)
    {
        Axis2D pos = _pos;
        pos = pos.Scaled(p, s);
        return new Line2D(pos);
    }

    public readonly Line2D Transformed(in Transform2D t)
    {
        Axis2D pos = _pos;
        pos = pos.Transformed(t);
        return new Line2D(pos);
    }

    public readonly Line2D Translated(in Vector2D v)
    {
        Axis2D pos = _pos;
        pos = pos.Translated(v);
        return new Line2D(pos);
    }

    public readonly Line2D Translated(in Point2D p1, in Point2D p2)
    {
        Axis2D pos = _pos;
        pos = pos.Translated(new Vector2D(p1, p2));
        return new Line2D(pos);
    }

    public readonly bool Equals(Line2D other) => _pos.Equals(other._pos);

    public override readonly bool Equals(object? obj) => obj is Line2D other && Equals(other);

    public override readonly int GetHashCode() => _pos.GetHashCode();

    public static bool operator ==(in Line2D left, in Line2D right) => left.Equals(right);

    public static bool operator !=(in Line2D left, in Line2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Lin2d({_pos.Location}, {_pos.Direction})";
}
