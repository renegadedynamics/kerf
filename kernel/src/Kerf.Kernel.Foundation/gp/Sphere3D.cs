// Port of: src/FoundationClasses/TKMath/gp/gp_Sphere.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Geometry;

/// <summary>
/// A spherical surface, defined by its radius and positioned by a coordinate system whose origin is the centre.
/// It is parameterised by <c>P(U, V) = O + R * cos(V) * (cos(U) * XDir + sin(U) * YDir) + R * sin(V) * ZDir</c>,
/// with U in [0, 2*PI] and V in [-PI/2, PI/2].
/// </summary>
/// <remarks>
/// <c>new Sphere3D()</c> is OCCT's indefinite sphere (radius <c>double.MaxValue</c>);
/// <c>default(Sphere3D)</c> is not valid.
/// </remarks>
public readonly partial struct Sphere3D : IEquatable<Sphere3D>
{
    private readonly CoordinateSystem _pos;
    private readonly double _radius;

    /// <summary>An indefinite sphere with a radius of <c>double.MaxValue</c>, not a useful value until given one.</summary>
    public Sphere3D()
    {
        _pos = new CoordinateSystem();
        _radius = RealLast();
    }

    /// <exception cref="ArgumentException"><paramref name="radius"/> is negative.</exception>
    public Sphere3D(in CoordinateSystem position, double radius)
    {
        _pos = position;
        _radius = radius;
        Refusal.ArgumentIf(!(radius >= 0.0), FailureKind.ConstructionError, "gp_Sphere() - radius should be >= 0");
    }

    // Copies

    /// <summary>A copy moved so that its centre is <paramref name="location"/>; its directions stay.</summary>
    public readonly Sphere3D WithLocation(in Point3D location)
    {
        CoordinateSystem pos = _pos;
        pos = pos.WithLocation(location);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>A copy positioned by <paramref name="position"/>, keeping its size.</summary>
    public readonly Sphere3D WithPosition(in CoordinateSystem position)
    {
        CoordinateSystem pos = _pos;
        pos = position;
        return new Sphere3D(pos, _radius);
    }

    /// <summary>A copy with this radius.</summary>
    /// <exception cref="ArgumentException"><paramref name="r"/> is negative.</exception>
    public readonly Sphere3D WithRadius(double r)
    {
        double newRadius = _radius;
        Refusal.ArgumentIf(!(r >= 0.0), FailureKind.ConstructionError, "gp_Sphere::SetRadius() - radius should be >= 0");
        newRadius = r;
        return new Sphere3D(_pos, newRadius);
    }

    /// <summary>The same surface with U running the other way: the Y direction reverses, which flips the coordinate system's handedness.</summary>
    public readonly Sphere3D UReversed()
    {
        CoordinateSystem pos = _pos;
        pos = pos.YReversed();
        return new Sphere3D(pos, _radius);
    }

    /// <summary>The same surface with V running the other way: the main direction reverses, which flips the coordinate system's handedness.</summary>
    public readonly Sphere3D VReversed()
    {
        CoordinateSystem pos = _pos;
        pos = pos.ZReversed();
        return new Sphere3D(pos, _radius);
    }

    // Queries

    /// <summary>The surface area.</summary>
    public readonly double Area => 4.0 * M_PI * _radius * _radius;

    /// <summary>
    /// The coefficients of the implicit equation in the global system:
    /// <c>A1.X^2 + A2.Y^2 + A3.Z^2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a1, out double a2, out double a3,
                                      out double b1, out double b2, out double b3,
                                      out double c1, out double c2, out double c3,
                                      out double d)
    {
        // Dans le repere local de la sphere :
        // X*X + Y*Y + Z*Z - radius * radius = 0
        Transform3D t = new Transform3D().WithTransformation(_pos);
        double t11 = t.Value(1, 1);
        double t12 = t.Value(1, 2);
        double t13 = t.Value(1, 3);
        double t14 = t.Value(1, 4);
        double t21 = t.Value(2, 1);
        double t22 = t.Value(2, 2);
        double t23 = t.Value(2, 3);
        double t24 = t.Value(2, 4);
        double t31 = t.Value(3, 1);
        double t32 = t.Value(3, 2);
        double t33 = t.Value(3, 3);
        double t34 = t.Value(3, 4);
        a1 = t11 * t11 + t21 * t21 + t31 * t31;
        a2 = t12 * t12 + t22 * t22 + t32 * t32;
        a3 = t13 * t13 + t23 * t23 + t33 * t33;
        b1 = t11 * t12 + t21 * t22 + t31 * t32;
        b2 = t11 * t13 + t21 * t23 + t31 * t33;
        b3 = t12 * t13 + t22 * t23 + t32 * t33;
        c1 = t11 * t14 + t21 * t24 + t31 * t34;
        c2 = t12 * t14 + t22 * t24 + t32 * t34;
        c3 = t13 * t14 + t23 * t24 + t33 * t34;
        d = t14 * t14 + t24 * t24 + t34 * t34 - _radius * _radius;
    }

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool IsDirect => _pos.IsDirect;

    /// <summary>The centre of the sphere.</summary>
    public readonly Point3D Location => _pos.Location;

    /// <summary>The coordinate system the surface is positioned by.</summary>
    public readonly CoordinateSystem Position => _pos;

    /// <summary>The radius.</summary>
    public readonly double Radius => _radius;

    /// <summary>The volume enclosed.</summary>
    public readonly double Volume => (4.0 * M_PI * _radius * _radius * _radius) / 3.0;

    /// <summary>The axis through the centre along the X direction: where the angular parameter is 0.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The axis through the centre along the Y direction: where the angular parameter is PI/2.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Sphere3D Mirrored(in Point3D point)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(point);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Sphere3D Mirrored(in Axis3D axis)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(axis);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Sphere3D Mirrored(in Frame3D frame)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Mirrored(frame);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Sphere3D Rotated(in Axis3D axis, double angle)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: the radius is multiplied by |factor|.</summary>
    public readonly Sphere3D Scaled(in Point3D point, double factor)
    {
        CoordinateSystem pos = _pos;
        double radius = _radius;
        pos = pos.Scaled(point, factor);
        radius *= factor;
        if (radius < 0)
        {
            radius = -radius;
        }

        return new Sphere3D(pos, radius);
    }

    /// <summary>The image under <paramref name="transform"/>: the radius is multiplied by |ScaleFactor|.</summary>
    public readonly Sphere3D Transformed(in Transform3D transform)
    {
        CoordinateSystem pos = _pos;
        double radius = _radius;
        pos = pos.Transformed(transform);
        radius *= transform.ScaleFactor;
        if (radius < 0)
        {
            radius = -radius;
        }

        return new Sphere3D(pos, radius);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Sphere3D Translated(in Vector3D vector)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Translated(vector);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Sphere3D Translated(in Point3D point1, in Point3D point2)
    {
        CoordinateSystem pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Sphere3D(pos, _radius);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Sphere3D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Sphere3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Sphere3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Sphere3D left, in Sphere3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Sphere3D left, in Sphere3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => FormattableString.Invariant($"Sphere3D({_pos}, {_radius:R})");
}
