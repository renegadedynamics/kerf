// Port of: src/FoundationClasses/TKMath/gp/gp_Cylinder.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Geometry;

/// <summary>
/// An infinite cylindrical surface, defined by its radius and positioned by a coordinate system whose main axis
/// is the cylinder's axis. It is parameterised by <c>P(U, V) = O + R * (cos(U) * XDir + sin(U) * YDir) + V * ZDir</c>.
/// </summary>
/// <remarks>
/// <c>new Cylinder3D()</c> is OCCT's indefinite cylinder (radius <c>double.MaxValue</c>);
/// <c>default(Cylinder3D)</c> is not valid.
/// </remarks>
public readonly partial struct Cylinder3D : IEquatable<Cylinder3D>
{
    private readonly CoordinateSystem _pos;
    private readonly double _radius;

    /// <summary>An indefinite cylinder with a radius of <c>double.MaxValue</c>, not a useful value until given one.</summary>
    public Cylinder3D()
    {
        _pos = new CoordinateSystem();
        _radius = RealLast();
    }

    /// <exception cref="ArgumentException"><paramref name="radius"/> is negative.</exception>
    public Cylinder3D(in CoordinateSystem position, double radius)
    {
        _pos = position;
        _radius = radius;
        Refusal.ArgumentIf(!(radius >= 0.0), FailureKind.ConstructionError, "gp_Cylinder() - radius should be positive number");
    }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Cylinder3D WithAxis(in Axis3D axis)
    {
        CoordinateSystem pos = _pos;
        pos = pos.WithAxis(axis);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>A copy moved so that its origin is <paramref name="location"/>; its directions stay.</summary>
    public readonly Cylinder3D WithLocation(in Point3D location)
    {
        CoordinateSystem pos = _pos;
        pos = pos.WithLocation(location);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>A copy positioned by <paramref name="position"/>, keeping its size.</summary>
    public readonly Cylinder3D WithPosition(in CoordinateSystem position)
    {
        CoordinateSystem pos = _pos;
        pos = position;
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>A copy with this radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="r"/> is negative.</exception>
    public readonly Cylinder3D WithRadius(double r)
    {
        double newRadius = _radius;
        Refusal.ArgumentIf(!(r >= 0.0), FailureKind.ConstructionError, "gp_Cylinder::SetRadius() - radius should be positive number");
        newRadius = r;
        return new Cylinder3D(_pos, newRadius);
    }

    /// <summary>The same surface with U running the other way: the Y direction reverses, which flips the coordinate system's handedness.</summary>
    public readonly Cylinder3D UReversed()
    {
        CoordinateSystem pos = _pos;
        pos = pos.YReversed();
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>The same surface with V running the other way: the main direction reverses, which flips the coordinate system's handedness.</summary>
    public readonly Cylinder3D VReversed()
    {
        CoordinateSystem pos = _pos;
        pos = pos.ZReversed();
        return new Cylinder3D(pos, _radius);
    }

    // Queries

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool IsDirect => _pos.IsDirect;

    /// <summary>The symmetry axis.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>
    /// The coefficients of the implicit equation in the global system:
    /// <c>A1.X^2 + A2.Y^2 + A3.Z^2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a1, out double a2, out double a3,
                                      out double b1, out double b2, out double b3,
                                      out double c1, out double c2, out double c3,
                                      out double d)
    {
        // Dans le repere local du cylindre :
        // X**2 + Y**2 - radius = 0.0
        Transform3D t = new Transform3D().WithTransformation(_pos);
        double t11 = t.Value(1, 1);
        double t12 = t.Value(1, 2);
        double t13 = t.Value(1, 3);
        double t14 = t.Value(1, 4);
        double t21 = t.Value(2, 1);
        double t22 = t.Value(2, 2);
        double t23 = t.Value(2, 3);
        double t24 = t.Value(2, 4);
        a1 = t11 * t11 + t21 * t21;
        a2 = t12 * t12 + t22 * t22;
        a3 = t13 * t13 + t23 * t23;
        b1 = t11 * t12 + t21 * t22;
        b2 = t11 * t13 + t21 * t23;
        b3 = t12 * t13 + t22 * t23;
        c1 = t11 * t14 + t21 * t24;
        c2 = t12 * t14 + t22 * t24;
        c3 = t13 * t14 + t23 * t24;
        d = t14 * t14 + t24 * t24 - _radius * _radius;
    }

    /// <summary>The origin, a point on the axis.</summary>
    public readonly Point3D Location => _pos.Location;

    /// <summary>The coordinate system the surface is positioned by.</summary>
    public readonly CoordinateSystem Position => _pos;

    /// <summary>The radius.</summary>
    public readonly double Radius => _radius;

    /// <summary>The axis through the origin along the X direction: where the angular parameter is 0.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The axis through the origin along the Y direction: where the angular parameter is PI/2.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Cylinder3D Mirrored(in Point3D point)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(point);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Cylinder3D Mirrored(in Axis3D axis)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(axis);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Cylinder3D Mirrored(in Frame3D frame)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(frame);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Cylinder3D Rotated(in Axis3D axis, double angle)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: the radius is multiplied by |factor|.</summary>
    public readonly Cylinder3D Scaled(in Point3D point, double factor)
    {
        CoordinateSystem pos = _pos;
        double radius = _radius;
        pos = pos.Scaled(point, factor);
        radius *= factor;
        if (radius < 0)
        {
            radius = -radius;
        }

        return new Cylinder3D(pos, radius);
    }

    /// <summary>The image under <paramref name="transform"/>: the radius is multiplied by |ScaleFactor|.</summary>
    public readonly Cylinder3D Transformed(in Transform3D transform)
    {
        CoordinateSystem pos = _pos;
        double radius = _radius;
        pos = pos.Transformed(transform);
        radius *= transform.ScaleFactor;
        if (radius < 0)
        {
            radius = -radius;
        }

        return new Cylinder3D(pos, radius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Cylinder3D Translated(in Vector3D vector)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Translated(vector);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Cylinder3D Translated(in Point3D point1, in Point3D point2)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Cylinder3D(pos, _radius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Cylinder3D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Cylinder3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Cylinder3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Cylinder3D left, in Cylinder3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Cylinder3D left, in Cylinder3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => FormattableString.Invariant($"Cylinder3D({_pos}, {_radius:R})");
}
