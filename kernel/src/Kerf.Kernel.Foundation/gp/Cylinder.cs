// Port of: src/FoundationClasses/TKMath/gp/gp_Cylinder.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An infinite cylindrical surface, defined by its radius and positioned by a coordinate system whose main axis
/// is the cylinder's axis. It is parameterised by <c>P(U, V) = O + R * (cos(U) * XDir + sin(U) * YDir) + V * ZDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Cylinder()</c> is OCCT's indefinite cylinder (radius <see cref="Standard.RealLast"/>);
/// <c>default(gp_Cylinder)</c> is not valid.
/// </remarks>
public partial struct Cylinder : IEquatable<Cylinder>
{
    private CoordinateSystem _pos;
    private double _radius;

    public Cylinder()
    {
        _pos = new CoordinateSystem();
        _radius = RealLast();
    }

    /// <exception cref="ConstructionErrorException"><paramref name="radius"/> is negative.</exception>
    public Cylinder(in CoordinateSystem a3, double radius)
    {
        _pos = a3;
        _radius = radius;
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Cylinder() - radius should be positive number");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1) => _pos.SetAxis(a1);

    internal void SetLocation(in Point3D loc) => _pos.SetLocation(loc);

    internal void SetPosition(in CoordinateSystem a3) => _pos = a3;

    internal void SetRadius(double r)
    {
        Standard_ConstructionError_Raise_if(r < 0.0, "gp_Cylinder::SetRadius() - radius should be positive number");
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

    public readonly Cylinder WithAxis(in Axis3D a1)
    {
        Cylinder copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Cylinder WithLocation(in Point3D loc)
    {
        Cylinder copy = this;
        copy.SetLocation(loc);
        return copy;
    }

    public readonly Cylinder WithPosition(in CoordinateSystem a3)
    {
        Cylinder copy = this;
        copy.SetPosition(a3);
        return copy;
    }

    public readonly Cylinder WithRadius(double r)
    {
        Cylinder copy = this;
        copy.SetRadius(r);
        return copy;
    }

    public readonly Cylinder UReversed()
    {
        Cylinder copy = this;
        copy.UReverse();
        return copy;
    }

    public readonly Cylinder VReversed()
    {
        Cylinder copy = this;
        copy.VReverse();
        return copy;
    }

    // Queries

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
        // Dans le repere local du cylindre :
        // X**2 + Y**2 - radius = 0.0
        Transform3D t = new();
        t = t.WithTransformation(_pos);
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

    public readonly Point3D Location() => _pos.Location();

    public readonly CoordinateSystem Position() => _pos;

    public readonly double Radius() => _radius;

    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly Cylinder Mirrored(in Point3D p)
    {
        Cylinder c = this;
        c._pos = c._pos.Mirrored(p);
        return c;
    }

    public readonly Cylinder Mirrored(in Axis3D a1)
    {
        Cylinder c = this;
        c._pos = c._pos.Mirrored(a1);
        return c;
    }

    public readonly Cylinder Mirrored(in Frame3D a2)
    {
        Cylinder c = this;
        c._pos = c._pos.Mirrored(a2);
        return c;
    }

    public readonly Cylinder Rotated(in Axis3D a1, double ang)
    {
        Cylinder cyl = this;
        cyl._pos = cyl._pos.Rotated(a1, ang);
        return cyl;
    }

    public readonly Cylinder Scaled(in Point3D p, double s)
    {
        Cylinder cyl = this;
        cyl._pos = cyl._pos.Scaled(p, s);
        cyl._radius *= s;
        if (cyl._radius < 0)
        {
            cyl._radius = -cyl._radius;
        }

        return cyl;
    }

    public readonly Cylinder Transformed(in Transform3D t)
    {
        Cylinder cyl = this;
        cyl._pos = cyl._pos.Transformed(t);
        cyl._radius *= t.ScaleFactor();
        if (cyl._radius < 0)
        {
            cyl._radius = -cyl._radius;
        }

        return cyl;
    }

    public readonly Cylinder Translated(in Vector3D v)
    {
        Cylinder cyl = this;
        cyl._pos = cyl._pos.Translated(v);
        return cyl;
    }

    public readonly Cylinder Translated(in Point3D p1, in Point3D p2)
    {
        Cylinder cyl = this;
        cyl._pos = cyl._pos.Translated(p1, p2);
        return cyl;
    }

    public readonly bool Equals(Cylinder other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is Cylinder other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    public static bool operator ==(in Cylinder left, in Cylinder right) => left.Equals(right);

    public static bool operator !=(in Cylinder left, in Cylinder right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Cylinder({_pos}, {_radius:R})");
}
