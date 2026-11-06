// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// A circle in 3D space, defined by its radius and positioned by a right-handed coordinate system whose origin is
/// the centre and whose main direction is the normal to the circle's plane. The circle is parameterised by the
/// angle from the X direction: <c>P(U) = O + R * (cos(U) * XDir + sin(U) * YDir)</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Circ()</c> is OCCT's indefinite circle (radius <see cref="Standard.RealLast"/>); <c>default(gp_Circ)</c>
/// is not valid.
/// </remarks>
public readonly partial struct Circle3D : IEquatable<Circle3D>
{
    private readonly Frame3D _pos;
    private readonly double _radius;

    public Circle3D()
    {
        _pos = new Frame3D();
        _radius = RealLast();
    }

    /// <exception cref="ConstructionErrorException"><paramref name="radius"/> is negative.</exception>
    public Circle3D(in Frame3D frame, double radius)
    {
        _pos = frame;
        _radius = radius;
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ() - radius should be positive number");
    }

    // Copies

    public readonly Circle3D WithAxis(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(axis);
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D WithLocation(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.WithLocation(point);
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D WithPosition(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = frame;
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D WithRadius(double radius)
    {
        double newRadius = _radius;
        Refusal.ArgumentIf(radius < 0.0, FailureKind.ConstructionError, "gp_Circ::SetRadius() - radius should be positive number");
        newRadius = radius;
        return new Circle3D(_pos, newRadius);
    }

    // Queries

    public readonly double Area => M_PI * _radius * _radius;

    /// <summary>The main axis of the circle: the normal to its plane through its centre.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>The circumference.</summary>
    public readonly double Length => 2.0 * M_PI * _radius;

    /// <summary>The centre of the circle.</summary>
    public readonly Point3D Location => _pos.Location;

    public readonly Frame3D Position => _pos;

    public readonly double Radius => _radius;

    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    public readonly double DistanceTo(in Point3D point) => Math.Sqrt(SquareDistanceTo(point));

    public readonly double SquareDistanceTo(in Point3D point)
    {
        Vector3D v = new(Location, point);
        double x = v.Dot(new Vector3D(_pos.XDirection));
        double y = v.Dot(new Vector3D(_pos.YDirection));
        double z = v.Dot(new Vector3D(_pos.Direction));
        double t = Math.Sqrt(x * x + y * y) - _radius;
        return (t * t + z * z);
    }

    public readonly bool Contains(in Point3D point, double linearTolerance) => DistanceTo(point) <= linearTolerance;

    public readonly Circle3D Mirrored(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(point);
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D Mirrored(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D Mirrored(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(frame);
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D Rotated(in Axis3D axis, double angle)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Circle3D(pos, _radius);
    }

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

    public readonly Circle3D Translated(in Vector3D vector)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(vector);
        return new Circle3D(pos, _radius);
    }

    public readonly Circle3D Translated(in Point3D point1, in Point3D point2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Circle3D(pos, _radius);
    }

    public readonly bool Equals(Circle3D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is Circle3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    public static bool operator ==(in Circle3D left, in Circle3D right) => left.Equals(right);

    public static bool operator !=(in Circle3D left, in Circle3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"Circle3D({_pos}, {_radius:R})");
}
