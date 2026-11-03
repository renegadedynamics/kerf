// Port of: src/FoundationClasses/TKMath/gp/gp_Circ2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A circle in 2D space, positioned by a coordinate system whose origin is the centre. It is parametrized
/// from the X axis, counterclockwise when the system is direct.
/// </summary>
/// <remarks>
/// <c>new gp_Circ2d()</c> is OCCT's indefinite circle (radius <see cref="Standard.RealLast"/>);
/// <c>default(gp_Circ2d)</c> is not valid.
/// </remarks>
public partial struct Circle2D : IEquatable<Circle2D>
{
    private Frame2D _pos;
    private double _radius;

    public Circle2D()
    {
        _pos = new Frame2D();
        _radius = RealLast();
    }

    /// <summary>A circle centred on the location of <paramref name="xAxis"/>, direct when <paramref name="isSense"/> is true.</summary>
    /// <exception cref="ConstructionErrorException">The radius is negative.</exception>
    public Circle2D(in Axis2D xAxis, double radius, bool isSense = true)
    {
        _pos = new Frame2D(xAxis, isSense);
        _radius = radius;
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Circ2d() - radius should be positive number");
    }

    /// <exception cref="ConstructionErrorException">The radius is negative.</exception>
    public Circle2D(in Frame2D axis, double radius)
    {
        _pos = axis;
        _radius = radius;
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Circ2d() - radius should be positive number");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in Point2D p) => _pos.SetLocation(p);

    internal void SetXAxis(in Axis2D a) => _pos.SetXAxis(a);

    internal void SetAxis(in Frame2D a) => _pos.SetAxis(a);

    internal void SetYAxis(in Axis2D a) => _pos.SetYAxis(a);

    internal void SetRadius(double radius)
    {
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Circ2d::SetRadius() - radius should be positive number");
        _radius = radius;
    }

    internal void Reverse()
    {
        Direction2D temp = _pos.YDirection();
        temp = temp.Reversed();
        _pos = _pos.WithAxis(new Frame2D(_pos.Location(), _pos.XDirection(), temp));
    }

    internal void Mirror(in Point2D p) => _pos.Mirror(p);

    internal void Mirror(in Axis2D a) => _pos.Mirror(a);

    internal void Rotate(in Point2D p, double ang) => _pos.Rotate(p, ang);

    internal void Scale(in Point2D p, double s)
    {
        _radius *= s;
        if (_radius < 0)
        {
            _radius = -_radius;
        }

        _pos = _pos.Scaled(p, s);
    }

    internal void Transform(in Transform2D t)
    {
        _radius *= t.ScaleFactor();
        if (_radius < 0)
        {
            _radius = -_radius;
        }

        _pos = _pos.Transformed(t);
    }

    internal void Translate(in Vector2D v) => _pos.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Circle2D WithLocation(in Point2D p)
    {
        Circle2D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Circle2D WithXAxis(in Axis2D a)
    {
        Circle2D copy = this;
        copy.SetXAxis(a);
        return copy;
    }

    public readonly Circle2D WithAxis(in Frame2D a)
    {
        Circle2D copy = this;
        copy.SetAxis(a);
        return copy;
    }

    public readonly Circle2D WithYAxis(in Axis2D a)
    {
        Circle2D copy = this;
        copy.SetYAxis(a);
        return copy;
    }

    public readonly Circle2D WithRadius(double radius)
    {
        Circle2D copy = this;
        copy.SetRadius(radius);
        return copy;
    }

    // Queries

    public readonly double Area() => M_PI * _radius * _radius;

    /// <summary>
    /// The coefficients of the circle's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d, out double e, out double f)
    {
        double xc = _pos.Location().X();
        double yc = _pos.Location().Y();
        a = 1.0;
        b = 1.0;
        c = 0.0;
        d = -xc;
        e = -yc;
        f = xc * xc + yc * yc - _radius * _radius;
    }

    public readonly bool Contains(in Point2D p, double linearTolerance) => Distance(p) <= linearTolerance;

    public readonly double Distance(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract(_pos.Location().XY());
        double d = _radius - coord.Modulus();
        if (d < 0)
        {
            d = -d;
        }

        return d;
    }

    public readonly double SquareDistance(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract(_pos.Location().XY());
        double d = _radius - coord.Modulus();
        return d * d;
    }

    public readonly double Length() => 2.0 * M_PI * _radius;

    public readonly Point2D Location() => _pos.Location();

    public readonly double Radius() => _radius;

    public readonly Frame2D Axis() => _pos;

    public readonly Frame2D Position() => _pos;

    public readonly Axis2D XAxis() => _pos.XAxis();

    public readonly Axis2D YAxis() => _pos.YAxis();

    /// <summary>True when the circle's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection().Crossed(_pos.YDirection())) >= 0.0;

    public readonly Circle2D Reversed()
    {
        Circle2D circ = this;
        Direction2D temp = _pos.YDirection();
        temp = temp.Reversed();
        circ._pos = circ._pos.WithAxis(new Frame2D(_pos.Location(), _pos.XDirection(), temp));
        return circ;
    }

    public readonly Circle2D Mirrored(in Point2D p)
    {
        Circle2D c = this;
        c._pos = c._pos.Mirrored(p);
        return c;
    }

    public readonly Circle2D Mirrored(in Axis2D a)
    {
        Circle2D c = this;
        c._pos = c._pos.Mirrored(a);
        return c;
    }

    public readonly Circle2D Rotated(in Point2D p, double ang)
    {
        Circle2D circ = this;
        circ._pos = circ._pos.Rotated(p, ang);
        return circ;
    }

    public readonly Circle2D Scaled(in Point2D p, double s)
    {
        Circle2D circ = this;
        circ._radius *= s;
        if (circ._radius < 0)
        {
            circ._radius = -circ._radius;
        }

        circ._pos = circ._pos.Scaled(p, s);
        return circ;
    }

    public readonly Circle2D Transformed(in Transform2D t)
    {
        Circle2D circ = this;
        circ._radius *= t.ScaleFactor();
        if (circ._radius < 0)
        {
            circ._radius = -circ._radius;
        }

        circ._pos = circ._pos.Transformed(t);
        return circ;
    }

    public readonly Circle2D Translated(in Vector2D v)
    {
        Circle2D circ = this;
        circ._pos = circ._pos.Translated(v);
        return circ;
    }

    public readonly Circle2D Translated(in Point2D p1, in Point2D p2)
    {
        Circle2D circ = this;
        circ._pos = circ._pos.Translated(p1, p2);
        return circ;
    }

    public readonly bool Equals(Circle2D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is Circle2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    public static bool operator ==(in Circle2D left, in Circle2D right) => left.Equals(right);

    public static bool operator !=(in Circle2D left, in Circle2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Circ2d({_pos}, radius {_radius})";
}
