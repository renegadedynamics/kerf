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
public readonly partial struct Circle2D : IEquatable<Circle2D>
{
    private readonly Frame2D _pos;
    private readonly double _radius;

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
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ2d() - radius should be positive number");
    }

    /// <exception cref="ConstructionErrorException">The radius is negative.</exception>
    public Circle2D(in Frame2D axis, double radius)
    {
        _pos = axis;
        _radius = radius;
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ2d() - radius should be positive number");
    }

    // Copies

    public readonly Circle2D WithLocation(in Point2D p)
    {
        Frame2D pos = _pos;
        pos = pos.WithLocation(p);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D WithXAxis(in Axis2D a)
    {
        Frame2D pos = _pos;
        pos = pos.WithXAxis(a);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D WithAxis(in Frame2D a)
    {
        Frame2D pos = _pos;
        pos = pos.WithAxis(a);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D WithYAxis(in Axis2D a)
    {
        Frame2D pos = _pos;
        pos = pos.WithYAxis(a);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D WithRadius(double radius)
    {
        double newRadius = _radius;
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ2d::SetRadius() - radius should be positive number");
        newRadius = radius;
        return new Circle2D(_pos, newRadius);
    }

    // Queries

    public readonly double Area => M_PI * _radius * _radius;

    /// <summary>
    /// The coefficients of the circle's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d, out double e, out double f)
    {
        double xc = _pos.Location.X;
        double yc = _pos.Location.Y;
        a = 1.0;
        b = 1.0;
        c = 0.0;
        d = -xc;
        e = -yc;
        f = xc * xc + yc * yc - _radius * _radius;
    }

    public readonly bool Contains(in Point2D p, double linearTolerance) => DistanceTo(p) <= linearTolerance;

    public readonly double DistanceTo(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract(_pos.Location.XY());
        double d = _radius - coord.Modulus();
        if (d < 0)
        {
            d = -d;
        }

        return d;
    }

    public readonly double SquareDistanceTo(in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract(_pos.Location.XY());
        double d = _radius - coord.Modulus();
        return d * d;
    }

    public readonly double Length => 2.0 * M_PI * _radius;

    public readonly Point2D Location => _pos.Location;

    public readonly double Radius => _radius;

    public readonly Frame2D Axis => _pos;

    public readonly Frame2D Position => _pos;

    public readonly Axis2D XAxis => _pos.XAxis;

    public readonly Axis2D YAxis => _pos.YAxis;

    /// <summary>True when the circle's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection.Crossed(_pos.YDirection)) >= 0.0;

    public readonly Circle2D Reversed()
    {
        Frame2D pos = _pos;
        Direction2D temp = _pos.YDirection;
        temp = temp.Reversed();
        pos = pos.WithAxis(new Frame2D(_pos.Location, _pos.XDirection, temp));
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D Mirrored(in Point2D p)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(p);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D Mirrored(in Axis2D a)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(a);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D Rotated(in Point2D p, double ang)
    {
        Frame2D pos = _pos;
        pos = pos.Rotated(p, ang);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D Scaled(in Point2D p, double s)
    {
        Frame2D pos = _pos;
        double radius = _radius;
        radius *= s;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos = pos.Scaled(p, s);
        return new Circle2D(pos, radius);
    }

    public readonly Circle2D Transformed(in Transform2D t)
    {
        Frame2D pos = _pos;
        double radius = _radius;
        radius *= t.ScaleFactor;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos = pos.Transformed(t);
        return new Circle2D(pos, radius);
    }

    public readonly Circle2D Translated(in Vector2D v)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(v);
        return new Circle2D(pos, _radius);
    }

    public readonly Circle2D Translated(in Point2D p1, in Point2D p2)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(p1, p2);
        return new Circle2D(pos, _radius);
    }

    public readonly bool Equals(Circle2D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is Circle2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    public static bool operator ==(in Circle2D left, in Circle2D right) => left.Equals(right);

    public static bool operator !=(in Circle2D left, in Circle2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Circ2d({_pos}, radius {_radius})";
}
