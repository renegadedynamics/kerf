// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Geometry;

/// <summary>
/// A circle in 3D space, defined by its radius and positioned by a right-handed coordinate system whose origin is
/// the centre and whose main direction is the normal to the circle's plane. The circle is parameterised by the
/// angle from the X direction: <c>P(U) = O + R * (cos(U) * XDir + sin(U) * YDir)</c>.
/// </summary>
/// <remarks>
/// <c>new Circle3D()</c> is OCCT's indefinite circle (radius <c>double.MaxValue</c>); <c>default(Circle3D)</c>
/// is not valid.
/// </remarks>
public readonly partial struct Circle3D : IEquatable<Circle3D>
{
    private readonly Frame3D _pos;
    private readonly double _radius;

    /// <summary>An indefinite circle with a radius of <c>double.MaxValue</c>, not a useful value until given one.</summary>
    public Circle3D()
    {
        _pos = new Frame3D();
        _radius = RealLast();
    }

    /// <exception cref="ArgumentException"><paramref name="radius"/> is negative.</exception>
    public Circle3D(in Frame3D frame, double radius)
    {
        _pos = frame;
        _radius = radius;
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ() - radius should be positive number");
    }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Circle3D WithAxis(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(axis);
        return new Circle3D(pos, _radius);
    }

    /// <summary>A copy moved so that its centre is <paramref name="point"/>; its directions stay.</summary>
    public readonly Circle3D WithLocation(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.WithLocation(point);
        return new Circle3D(pos, _radius);
    }

    /// <summary>A copy positioned by <paramref name="frame"/>, keeping its size.</summary>
    public readonly Circle3D WithPosition(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = frame;
        return new Circle3D(pos, _radius);
    }

    /// <summary>A copy with this radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="radius"/> is negative.</exception>
    public readonly Circle3D WithRadius(double radius)
    {
        double newRadius = _radius;
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ::SetRadius() - radius should be positive number");
        newRadius = radius;
        return new Circle3D(_pos, newRadius);
    }

    // Queries

    /// <summary>The area of the disc.</summary>
    public readonly double Area => M_PI * _radius * _radius;

    /// <summary>The main axis of the circle: the normal to its plane through its centre.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>The circumference.</summary>
    public readonly double Length => 2.0 * M_PI * _radius;

    /// <summary>The centre of the circle.</summary>
    public readonly Point3D Location => _pos.Location;

    /// <summary>The frame the curve lies in: its main direction is the plane's normal, and its X direction is parameter 0's.</summary>
    public readonly Frame3D Position => _pos;

    /// <summary>The radius.</summary>
    public readonly double Radius => _radius;

    /// <summary>The axis through the centre along the X direction: where the angular parameter is 0.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The axis through the centre along the Y direction: where the angular parameter is PI/2.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    /// <summary>The distance from <paramref name="point"/> to the circle itself: 0 on the circle, and the radius at its centre.</summary>
    public readonly double DistanceTo(in Point3D point) => Math.Sqrt(SquareDistanceTo(point));

    /// <summary>The square of the distance, without the square root where it can be avoided.</summary>
    public readonly double SquareDistanceTo(in Point3D point)
    {
        Vector3D v = new(Location, point);
        double x = v.Dot(new Vector3D(_pos.XDirection));
        double y = v.Dot(new Vector3D(_pos.YDirection));
        double z = v.Dot(new Vector3D(_pos.Direction));
        double t = Math.Sqrt(x * x + y * y) - _radius;
        return (t * t + z * z);
    }

    /// <summary>True when <paramref name="point"/> is within <paramref name="linearTolerance"/> of the circle itself, not of the disc inside it.</summary>
    public readonly bool Contains(in Point3D point, double linearTolerance) => DistanceTo(point) <= linearTolerance;

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Circle3D Mirrored(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(point);
        return new Circle3D(pos, _radius);
    }

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Circle3D Mirrored(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Circle3D(pos, _radius);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Circle3D Mirrored(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(frame);
        return new Circle3D(pos, _radius);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Circle3D Rotated(in Axis3D axis, double angle)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Circle3D(pos, _radius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: the radius is multiplied by |factor|.</summary>
    public readonly Circle3D Scaled(in Point3D point, double factor)
    {
        Frame3D pos = _pos;
        double radius = _radius;
        radius *= factor;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos = pos.Scaled(point, factor);
        return new Circle3D(pos, radius);
    }

    /// <summary>The image under <paramref name="transform"/>: the radius is multiplied by |ScaleFactor|.</summary>
    public readonly Circle3D Transformed(in Transform3D transform)
    {
        Frame3D pos = _pos;
        double radius = _radius;
        radius *= transform.ScaleFactor;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos = pos.Transformed(transform);
        return new Circle3D(pos, radius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Circle3D Translated(in Vector3D vector)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(vector);
        return new Circle3D(pos, _radius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Circle3D Translated(in Point3D point1, in Point3D point2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Circle3D(pos, _radius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Circle3D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Circle3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Circle3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Circle3D left, in Circle3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Circle3D left, in Circle3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => FormattableString.Invariant($"Circle3D({_pos}, {_radius:R})");
}
