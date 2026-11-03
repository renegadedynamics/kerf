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
public partial struct Line2D : IEquatable<Line2D>
{
    private Axis2D _pos;

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

    // Mutators (internal: see docs/translation-rules.md)

    internal void Reverse() => _pos.Reverse();

    internal void SetDirection(in Direction2D v) => _pos.SetDirection(v);

    internal void SetLocation(in Point2D p) => _pos.SetLocation(p);

    internal void SetPosition(in Axis2D a) => _pos = a;

    internal void Mirror(in Point2D p) => _pos.Mirror(p);

    internal void Mirror(in Axis2D a) => _pos.Mirror(a);

    internal void Rotate(in Point2D p, double ang) => _pos.Rotate(p, ang);

    internal void Scale(in Point2D p, double s) => _pos.Scale(p, s);

    internal void Transform(in Transform2D t) => _pos.Transform(t);

    internal void Translate(in Vector2D v) => _pos.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Line2D WithDirection(in Direction2D v)
    {
        Line2D copy = this;
        copy.SetDirection(v);
        return copy;
    }

    public readonly Line2D WithLocation(in Point2D p)
    {
        Line2D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Line2D WithPosition(in Axis2D a) => new(a);

    // Queries

    /// <summary>The coefficients of the line's equation <c>A * X + B * Y + C = 0</c>.</summary>
    public readonly void Coefficients(out double a, out double b, out double c)
    {
        a = _pos.Direction().Y();
        b = -_pos.Direction().X();
        c = -(a * _pos.Location().X() + b * _pos.Location().Y());
    }

    public readonly Direction2D Direction() => _pos.Direction();

    public readonly Point2D Location() => _pos.Location();

    public readonly Axis2D Position() => _pos;

    /// <summary>The signed angle between the directions of the two lines, in ]-PI, PI].</summary>
    public readonly double Angle(in Line2D other) => _pos.Direction().Angle(other._pos.Direction());

    public readonly bool Contains(in Point2D p, double linearTolerance) => Distance(p) <= linearTolerance;

    public readonly double Distance(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract((_pos.Location()).XY());
        return Math.Abs(coord.Crossed(_pos.Direction().XY()));
    }

    /// <summary>The distance between the lines: zero unless they are parallel.</summary>
    public readonly double Distance(in Line2D other)
    {
        double d = 0.0;
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            d = other.Distance(_pos.Location());
        }

        return d;
    }

    public readonly double SquareDistance(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract((_pos.Location()).XY());
        double d = coord.Crossed(_pos.Direction().XY());
        return d * d;
    }

    public readonly double SquareDistance(in Line2D other)
    {
        double d = 0.0;
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            d = other.SquareDistance(_pos.Location());
        }

        return d;
    }

    /// <summary>The line through <paramref name="p"/> normal to this one.</summary>
    public readonly Line2D Normal(in Point2D p)
        => new(new Axis2D(p, new Direction2D(-(_pos.Direction().Y()), _pos.Direction().X())));

    public readonly Line2D Reversed()
    {
        Line2D l = this;
        l._pos.Reverse();
        return l;
    }

    public readonly Line2D Mirrored(in Point2D p)
    {
        Line2D l = this;
        l._pos.Mirror(p);
        return l;
    }

    public readonly Line2D Mirrored(in Axis2D a)
    {
        Line2D l = this;
        l._pos.Mirror(a);
        return l;
    }

    public readonly Line2D Rotated(in Point2D p, double ang)
    {
        Line2D l = this;
        l._pos.Rotate(p, ang);
        return l;
    }

    public readonly Line2D Scaled(in Point2D p, double s)
    {
        Line2D l = this;
        l._pos.Scale(p, s);
        return l;
    }

    public readonly Line2D Transformed(in Transform2D t)
    {
        Line2D l = this;
        l._pos.Transform(t);
        return l;
    }

    public readonly Line2D Translated(in Vector2D v)
    {
        Line2D l = this;
        l._pos.Translate(v);
        return l;
    }

    public readonly Line2D Translated(in Point2D p1, in Point2D p2)
    {
        Line2D l = this;
        l._pos.Translate(new Vector2D(p1, p2));
        return l;
    }

    public readonly bool Equals(Line2D other) => _pos.Equals(other._pos);

    public override readonly bool Equals(object? obj) => obj is Line2D other && Equals(other);

    public override readonly int GetHashCode() => _pos.GetHashCode();

    public static bool operator ==(in Line2D left, in Line2D right) => left.Equals(right);

    public static bool operator !=(in Line2D left, in Line2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Lin2d({_pos.Location()}, {_pos.Direction()})";
}
