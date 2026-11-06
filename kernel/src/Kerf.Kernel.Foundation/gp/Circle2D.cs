// Port of: src/FoundationClasses/TKMath/gp/gp_Circ2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>
/// A circle in 2D space, positioned by a coordinate system whose origin is the centre. It is parametrized
/// from the X axis, counterclockwise when the system is direct.
/// </summary>
/// <remarks>
/// <c>new Circle2D()</c> is OCCT's indefinite circle (radius <c>double.MaxValue</c>);
/// <c>default(Circle2D)</c> is not valid.
/// </remarks>
public readonly partial struct Circle2D : IEquatable<Circle2D>
{
    private readonly Frame2D _pos;
    private readonly double _radius;

    /// <summary>An indefinite circle with a radius of <c>double.MaxValue</c>, not a useful value until given one.</summary>
    public Circle2D()
    {
        _pos = new Frame2D();
        _radius = RealLast();
    }

    /// <summary>A circle centred on the location of <paramref name="xAxis"/>, direct when <paramref name="sense"/> is true.</summary>
    /// <exception cref="ArgumentException">The radius is negative.</exception>
    public Circle2D(in Axis2D xAxis, double radius, bool sense = true)
    {
        _pos = new Frame2D(xAxis, sense);
        _radius = radius;
        Refusal.ArgumentIf(!(radius >= 0.0), FailureKind.ConstructionError, "gp_Circ2d() - radius should be positive number");
    }

    /// <exception cref="ArgumentException">The radius is negative.</exception>
    public Circle2D(in Frame2D axis, double radius)
    {
        _pos = axis;
        _radius = radius;
        Refusal.ArgumentIf(!(radius >= 0.0), FailureKind.ConstructionError, "gp_Circ2d() - radius should be positive number");
    }

    // Copies

    /// <summary>A copy moved so that its centre is <paramref name="point"/>; its directions stay.</summary>
    public readonly Circle2D WithLocation(in Point2D point)
    {
        Frame2D pos = _pos;
        pos = pos.WithLocation(point);
        return new Circle2D(pos, _radius);
    }

    /// <summary>A copy whose location and X direction come from <paramref name="axis"/>; the Y direction is recomputed normal to it, keeping the handedness.</summary>
    public readonly Circle2D WithXAxis(in Axis2D axis)
    {
        Frame2D pos = _pos;
        pos = pos.WithXAxis(axis);
        return new Circle2D(pos, _radius);
    }

    /// <summary>A copy positioned by <paramref name="frame"/>, which replaces the whole frame.</summary>
    public readonly Circle2D WithAxis(in Frame2D frame)
    {
        Frame2D pos = _pos;
        pos = pos.WithAxis(frame);
        return new Circle2D(pos, _radius);
    }

    /// <summary>A copy whose location and Y direction come from <paramref name="axis"/>; the X direction is recomputed normal to it, keeping the handedness.</summary>
    public readonly Circle2D WithYAxis(in Axis2D axis)
    {
        Frame2D pos = _pos;
        pos = pos.WithYAxis(axis);
        return new Circle2D(pos, _radius);
    }

    /// <summary>A copy with this radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="radius"/> is negative.</exception>
    public readonly Circle2D WithRadius(double radius)
    {
        double newRadius = _radius;
        Refusal.ArgumentIf(!(radius >= 0.0), FailureKind.ConstructionError, "gp_Circ2d::SetRadius() - radius should be positive number");
        newRadius = radius;
        return new Circle2D(_pos, newRadius);
    }

    // Queries

    /// <summary>The area of the disc.</summary>
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

    /// <summary>True when <paramref name="point"/> is within <paramref name="linearTolerance"/> of the circle itself, not of the disc inside it.</summary>
    public readonly bool Contains(in Point2D point, double linearTolerance) => DistanceTo(point) <= linearTolerance;

    /// <summary>The distance from <paramref name="point"/> to the circle itself: 0 on the circle, and the radius at its centre.</summary>
    public readonly double DistanceTo(in Point2D point)
    {
        Xy coord = point.XY();
        coord.Subtract(_pos.Location.XY());
        double d = _radius - coord.Modulus();
        if (d < 0)
        {
            d = -d;
        }

        return d;
    }

    /// <summary>The square of the distance, without the square root where it can be avoided.</summary>
    public readonly double SquareDistanceTo(in Point2D point)
    {
        Xy coord = point.XY();
        coord.Subtract(_pos.Location.XY());
        double d = _radius - coord.Modulus();
        return d * d;
    }

    /// <summary>The circumference.</summary>
    public readonly double Length => 2.0 * M_PI * _radius;

    /// <summary>The centre.</summary>
    public readonly Point2D Location => _pos.Location;

    /// <summary>The radius.</summary>
    public readonly double Radius => _radius;

    /// <summary>The frame the curve is positioned by, whose X direction is parameter 0's and whose handedness is the sense it runs in.</summary>
    public readonly Frame2D Axis => _pos;

    /// <summary>The frame the curve is positioned by, whose X direction is parameter 0's and whose handedness is the sense it runs in.</summary>
    public readonly Frame2D Position => _pos;

    /// <summary>The axis through the centre along the X direction: where the angular parameter is 0.</summary>
    public readonly Axis2D XAxis => _pos.XAxis;

    /// <summary>The axis through the centre along the Y direction: where the angular parameter is PI/2.</summary>
    public readonly Axis2D YAxis => _pos.YAxis;

    /// <summary>True when the circle's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect => (_pos.XDirection.Crossed(_pos.YDirection)) >= 0.0;

    /// <summary>The same curve, running the other way: the Y direction reverses, which flips the frame's handedness.</summary>
    public readonly Circle2D Reversed()
    {
        Frame2D pos = _pos;
        Direction2D temp = _pos.YDirection;
        temp = temp.Reversed();
        pos = pos.WithAxis(new Frame2D(_pos.Location, _pos.XDirection, temp));
        return new Circle2D(pos, _radius);
    }

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Circle2D Mirrored(in Point2D point)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(point);
        return new Circle2D(pos, _radius);
    }

    /// <summary>The reflection in the line <paramref name="axis"/>.</summary>
    public readonly Circle2D Mirrored(in Axis2D axis)
    {
        Frame2D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Circle2D(pos, _radius);
    }

    /// <summary>Rotated about <paramref name="point"/> by <paramref name="angle"/> radians, anticlockwise.</summary>
    public readonly Circle2D Rotated(in Point2D point, double angle)
    {
        Frame2D pos = _pos;
        pos = pos.Rotated(point, angle);
        return new Circle2D(pos, _radius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: the radius is multiplied by |factor|.</summary>
    public readonly Circle2D Scaled(in Point2D point, double factor)
    {
        Frame2D pos = _pos;
        double radius = _radius;
        radius *= factor;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos = pos.Scaled(point, factor);
        return new Circle2D(pos, radius);
    }

    /// <summary>The image under <paramref name="transform"/>: the radius is multiplied by |ScaleFactor|.</summary>
    public readonly Circle2D Transformed(in Transform2D transform)
    {
        Frame2D pos = _pos;
        double radius = _radius;
        radius *= transform.ScaleFactor;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos = pos.Transformed(transform);
        return new Circle2D(pos, radius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Circle2D Translated(in Vector2D vector)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(vector);
        return new Circle2D(pos, _radius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Circle2D Translated(in Point2D point1, in Point2D point2)
    {
        Frame2D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Circle2D(pos, _radius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Circle2D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Circle2D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Circle2D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Circle2D left, in Circle2D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Circle2D left, in Circle2D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Circle2D({_pos}, radius {_radius})";
}
