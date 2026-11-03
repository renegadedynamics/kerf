// Port of: src/FoundationClasses/TKMath/gp/gp_Cone.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An infinite conical surface, defined by its semi-angle and its reference radius (the radius in the plane of
/// its coordinate system), and positioned by a coordinate system whose main axis is the cone's axis. It is
/// parameterised by <c>P(U, V) = O + (R + V * sin(Ang)) * (cos(U) * XDir + sin(U) * YDir) + V * cos(Ang) * ZDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Cone()</c> is OCCT's indefinite cone (reference radius <see cref="Standard.RealLast"/>, semi-angle
/// PI/4); <c>default(gp_Cone)</c> is not valid.
/// </remarks>
public partial struct Cone : IEquatable<Cone>
{
    private CoordinateSystem _pos;
    private double _radius;
    private double _semiAngle;

    public Cone()
    {
        _pos = new CoordinateSystem();
        _radius = RealLast();
        _semiAngle = M_PI * 0.25;
    }

    /// <exception cref="ConstructionErrorException">
    /// <paramref name="radius"/> is negative, or |<paramref name="ang"/>| is not in (Resolution, PI/2 - Resolution).
    /// </exception>
    public Cone(in CoordinateSystem a3, double ang, double radius)
    {
        _pos = a3;
        _radius = radius;
        _semiAngle = ang;
        double absAng = ang < 0.0 ? -ang : ang;
        Standard_ConstructionError_Raise_if(radius < 0.0 || absAng <= Constants.Resolution()
                                            || M_PI * 0.5 - absAng <= Constants.Resolution(),
                                            "gp_Cone() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1) => _pos.SetAxis(a1);

    internal void SetLocation(in Point3D loc) => _pos.SetLocation(loc);

    internal void SetPosition(in CoordinateSystem a3) => _pos = a3;

    internal void SetRadius(double r)
    {
        Standard_ConstructionError_Raise_if(r < 0.0, "gp_Cone::SetRadius() - radius should be positive number");
        _radius = r;
    }

    internal void SetSemiAngle(double ang)
    {
        Standard_ConstructionError_Raise_if(Math.Abs(ang) <= Constants.Resolution()
                                            || M_PI * 0.5 - Math.Abs(ang) <= Constants.Resolution(),
                                            "gp_Cone::SetSemiAngle() - invalid angle range");
        _semiAngle = ang;
    }

    /// <summary>Reverses the U parametrisation (the Y direction).</summary>
    internal void UReverse() => _pos.YReverse();

    /// <summary>Reverses the V parametrisation (the main direction), which reverses the semi-angle.</summary>
    internal void VReverse()
    {
        _pos = _pos.ZReversed();
        _semiAngle = -_semiAngle;
    }

    internal void Mirror(in Point3D p) => _pos.Mirror(p);

    internal void Mirror(in Axis3D a1) => _pos.Mirror(a1);

    internal void Mirror(in Frame3D a2) => _pos.Mirror(a2);

    internal void Rotate(in Axis3D a1, double ang) => _pos.Rotate(a1, ang);

    internal void Scale(in Point3D p, double s)
    {
        _pos = _pos.Scaled(p, s);
        _radius *= s;
        if (_radius < 0)
        {
            _radius = -_radius;
        }
    }

    internal void Transform(in Transform3D t)
    {
        _pos = _pos.Transformed(t);
        _radius *= t.ScaleFactor();
        if (_radius < 0)
        {
            _radius = -_radius;
        }
    }

    internal void Translate(in Vector3D v) => _pos.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Cone WithAxis(in Axis3D a1)
    {
        Cone copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Cone WithLocation(in Point3D loc)
    {
        Cone copy = this;
        copy.SetLocation(loc);
        return copy;
    }

    public readonly Cone WithPosition(in CoordinateSystem a3)
    {
        Cone copy = this;
        copy.SetPosition(a3);
        return copy;
    }

    public readonly Cone WithRadius(double r)
    {
        Cone copy = this;
        copy.SetRadius(r);
        return copy;
    }

    public readonly Cone WithSemiAngle(double ang)
    {
        Cone copy = this;
        copy.SetSemiAngle(ang);
        return copy;
    }

    public readonly Cone UReversed()
    {
        Cone copy = this;
        copy.UReverse();
        return copy;
    }

    public readonly Cone VReversed()
    {
        Cone copy = this;
        copy.VReverse();
        return copy;
    }

    // Queries

    /// <summary>The apex of the cone, on the negative side of the main axis when the semi-angle is positive.</summary>
    public readonly Point3D Apex()
    {
        Xyz coord = _pos.Direction().XYZ();
        coord.Multiply(-_radius / Math.Tan(_semiAngle));
        coord.Add(_pos.Location().XYZ());
        return new Point3D(coord);
    }

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool Direct() => _pos.Direct();

    /// <summary>The symmetry axis.</summary>
    public readonly Axis3D Axis() => _pos.Axis();

    /// <summary>
    /// The coefficients of the implicit equation in the global system:
    /// <c>A1.X^2 + A2.Y^2 + A3.Z^2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double a1, out double a2, out double a3,
                                      out double b1, out double b2, out double b3,
                                      out double c1, out double c2, out double c3,
                                      out double d)
    {
        // Dans le repere du cone :
        // X**2 + Y**2 - (radius + Z * std::tan(semiAngle))**2 = 0.0
        Transform3D t = new();
        t = t.WithTransformation(_pos);
        double kAng = Math.Tan(_semiAngle);
        double t11 = t.Value(1, 1);
        double t12 = t.Value(1, 2);
        double t13 = t.Value(1, 3);
        double t14 = t.Value(1, 4);
        double t21 = t.Value(2, 1);
        double t22 = t.Value(2, 2);
        double t23 = t.Value(2, 3);
        double t24 = t.Value(2, 4);
        double t31 = t.Value(3, 1) * kAng;
        double t32 = t.Value(3, 2) * kAng;
        double t33 = t.Value(3, 3) * kAng;
        double t34 = t.Value(3, 4) * kAng;
        a1 = t11 * t11 + t21 * t21 - t31 * t31;
        a2 = t12 * t12 + t22 * t22 - t32 * t32;
        a3 = t13 * t13 + t23 * t23 - t33 * t33;
        b1 = t11 * t12 + t21 * t22 - t31 * t32;
        b2 = t11 * t13 + t21 * t23 - t31 * t33;
        b3 = t12 * t13 + t22 * t23 - t32 * t33;
        c1 = t11 * t14 + t21 * t24 - t31 * (_radius + t34);
        c2 = t12 * t14 + t22 * t24 - t32 * (_radius + t34);
        c3 = t13 * t14 + t23 * t24 - t33 * (_radius + t34);
        d = t14 * t14 + t24 * t24 - _radius * _radius - t34 * t34 - 2.0 * _radius * t34;
    }

    public readonly Point3D Location() => _pos.Location();

    public readonly CoordinateSystem Position() => _pos;

    /// <summary>The radius of the cone in the plane of its coordinate system.</summary>
    public readonly double RefRadius() => _radius;

    /// <summary>The half-angle at the apex, in (-PI/2, PI/2), never 0.</summary>
    public readonly double SemiAngle() => _semiAngle;

    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Cone Mirrored(in Point3D p)
    {
        Cone c = this;
        c._pos = c._pos.Mirrored(p);
        return c;
    }

    public readonly Cone Mirrored(in Axis3D a1)
    {
        Cone c = this;
        c._pos = c._pos.Mirrored(a1);
        return c;
    }

    public readonly Cone Mirrored(in Frame3D a2)
    {
        Cone c = this;
        c._pos = c._pos.Mirrored(a2);
        return c;
    }

    public readonly Cone Rotated(in Axis3D a1, double ang)
    {
        Cone cone = this;
        cone._pos = cone._pos.Rotated(a1, ang);
        return cone;
    }

    public readonly Cone Scaled(in Point3D p, double s)
    {
        Cone c = this;
        c._pos = c._pos.Scaled(p, s);
        c._radius *= s;
        if (c._radius < 0)
        {
            c._radius = -c._radius;
        }

        return c;
    }

    public readonly Cone Transformed(in Transform3D t)
    {
        Cone c = this;
        c._pos = c._pos.Transformed(t);
        c._radius *= t.ScaleFactor();
        if (c._radius < 0)
        {
            c._radius = -c._radius;
        }

        return c;
    }

    public readonly Cone Translated(in Vector3D v)
    {
        Cone cone = this;
        cone._pos = cone._pos.Translated(v);
        return cone;
    }

    public readonly Cone Translated(in Point3D p1, in Point3D p2)
    {
        Cone cone = this;
        cone._pos = cone._pos.Translated(p1, p2);
        return cone;
    }

    public readonly bool Equals(Cone other)
        => _pos.Equals(other._pos) && _radius.Equals(other._radius) && _semiAngle.Equals(other._semiAngle);

    public override readonly bool Equals(object? obj) => obj is Cone other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius, _semiAngle);

    public static bool operator ==(in Cone left, in Cone right) => left.Equals(right);

    public static bool operator !=(in Cone left, in Cone right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Cone({_pos}, {_semiAngle:R}, {_radius:R})");
}
