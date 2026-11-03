// Port of: src/FoundationClasses/TKMath/gp/gp_Sphere.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A spherical surface, defined by its radius and positioned by a coordinate system whose origin is the centre.
/// It is parameterised by <c>P(U, V) = O + R * cos(V) * (cos(U) * XDir + sin(U) * YDir) + R * sin(V) * ZDir</c>,
/// with U in [0, 2*PI] and V in [-PI/2, PI/2].
/// </summary>
/// <remarks>
/// <c>new gp_Sphere()</c> is OCCT's indefinite sphere (radius <see cref="Standard.RealLast"/>);
/// <c>default(gp_Sphere)</c> is not valid.
/// </remarks>
public partial struct Sphere : IEquatable<Sphere>
{
    private CoordinateSystem _pos;
    private double _radius;

    public Sphere()
    {
        _pos = new CoordinateSystem();
        _radius = RealLast();
    }

    /// <exception cref="ConstructionErrorException"><paramref name="radius"/> is negative.</exception>
    public Sphere(in CoordinateSystem a3, double radius)
    {
        _pos = a3;
        _radius = radius;
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Sphere() - radius should be >= 0");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in Point3D loc) => _pos.SetLocation(loc);

    internal void SetPosition(in CoordinateSystem a3) => _pos = a3;

    internal void SetRadius(double r)
    {
        Standard_ConstructionError_Raise_if(r < 0.0, "gp_Sphere::SetRadius() - radius should be >= 0");
        _radius = r;
    }

    /// <summary>Reverses the U parametrisation (the Y direction).</summary>
    internal void UReverse() => _pos.YReverse();

    /// <summary>Reverses the V parametrisation (the main direction).</summary>
    internal void VReverse() => _pos.ZReverse();

    internal void Mirror(in Point3D p) => _pos.Mirror(p);

    internal void Mirror(in Axis3D a1) => _pos.Mirror(a1);

    internal void Mirror(in Frame3D a2) => _pos.Mirror(a2);

    internal void Rotate(in Axis3D a1, double ang) => _pos.Rotate(a1, ang);

    internal void Scale(in Point3D p, double s)
    {
        _pos.Scale(p, s);
        _radius *= s;
        if (_radius < 0)
        {
            _radius = -_radius;
        }
    }

    internal void Transform(in Transform3D t)
    {
        _pos.Transform(t);
        _radius *= t.ScaleFactor();
        if (_radius < 0)
        {
            _radius = -_radius;
        }
    }

    internal void Translate(in Vector3D v) => _pos.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Sphere WithLocation(in Point3D loc)
    {
        Sphere copy = this;
        copy.SetLocation(loc);
        return copy;
    }

    public readonly Sphere WithPosition(in CoordinateSystem a3)
    {
        Sphere copy = this;
        copy.SetPosition(a3);
        return copy;
    }

    public readonly Sphere WithRadius(double r)
    {
        Sphere copy = this;
        copy.SetRadius(r);
        return copy;
    }

    public readonly Sphere UReversed()
    {
        Sphere copy = this;
        copy.UReverse();
        return copy;
    }

    public readonly Sphere VReversed()
    {
        Sphere copy = this;
        copy.VReverse();
        return copy;
    }

    // Queries

    public readonly double Area() => 4.0 * M_PI * _radius * _radius;

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
        Transform3D t = new();
        t.SetTransformation(_pos);
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
    public readonly bool Direct() => _pos.Direct();

    /// <summary>The centre of the sphere.</summary>
    public readonly Point3D Location() => _pos.Location();

    public readonly CoordinateSystem Position() => _pos;

    public readonly double Radius() => _radius;

    public readonly double Volume() => (4.0 * M_PI * _radius * _radius * _radius) / 3.0;

    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Sphere Mirrored(in Point3D p)
    {
        Sphere c = this;
        c._pos.Mirror(p);
        return c;
    }

    public readonly Sphere Mirrored(in Axis3D a1)
    {
        Sphere c = this;
        c._pos.Mirror(a1);
        return c;
    }

    public readonly Sphere Mirrored(in Frame3D a2)
    {
        Sphere c = this;
        c._pos.Mirror(a2);
        return c;
    }

    public readonly Sphere Rotated(in Axis3D a1, double ang)
    {
        Sphere c = this;
        c._pos.Rotate(a1, ang);
        return c;
    }

    public readonly Sphere Scaled(in Point3D p, double s)
    {
        Sphere c = this;
        c._pos.Scale(p, s);
        c._radius *= s;
        if (c._radius < 0)
        {
            c._radius = -c._radius;
        }

        return c;
    }

    public readonly Sphere Transformed(in Transform3D t)
    {
        Sphere c = this;
        c._pos.Transform(t);
        c._radius *= t.ScaleFactor();
        if (c._radius < 0)
        {
            c._radius = -c._radius;
        }

        return c;
    }

    public readonly Sphere Translated(in Vector3D v)
    {
        Sphere c = this;
        c._pos.Translate(v);
        return c;
    }

    public readonly Sphere Translated(in Point3D p1, in Point3D p2)
    {
        Sphere c = this;
        c._pos.Translate(p1, p2);
        return c;
    }

    public readonly bool Equals(Sphere other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is Sphere other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    public static bool operator ==(in Sphere left, in Sphere right) => left.Equals(right);

    public static bool operator !=(in Sphere left, in Sphere right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Sphere({_pos}, {_radius:R})");
}
