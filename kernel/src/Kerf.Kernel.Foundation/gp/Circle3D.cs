// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

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
public partial struct Circle3D : IEquatable<Circle3D>
{
    private Frame3D _pos;
    private double _radius;

    public Circle3D()
    {
        _pos = new Frame3D();
        _radius = RealLast();
    }

    /// <exception cref="ConstructionErrorException"><paramref name="radius"/> is negative.</exception>
    public Circle3D(in Frame3D a2, double radius)
    {
        _pos = a2;
        _radius = radius;
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Circ() - radius should be positive number");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1) => _pos.SetAxis(a1);

    internal void SetLocation(in Point3D p) => _pos.SetLocation(p);

    internal void SetPosition(in Frame3D a2) => _pos = a2;

    internal void SetRadius(double radius)
    {
        Standard_ConstructionError_Raise_if(radius < 0.0, "gp_Circ::SetRadius() - radius should be positive number");
        _radius = radius;
    }

    internal void Mirror(in Point3D p) => _pos.Mirror(p);

    internal void Mirror(in Axis3D a1) => _pos.Mirror(a1);

    internal void Mirror(in Frame3D a2) => _pos.Mirror(a2);

    internal void Rotate(in Axis3D a1, double ang) => _pos.Rotate(a1, ang);

    internal void Scale(in Point3D p, double s)
    {
        _radius *= s;
        if (_radius < 0)
        {
            _radius = -_radius;
        }

        _pos.Scale(p, s);
    }

    internal void Transform(in Transform3D t)
    {
        _radius *= t.ScaleFactor();
        if (_radius < 0)
        {
            _radius = -_radius;
        }

        _pos.Transform(t);
    }

    internal void Translate(in Vector3D v) => _pos.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _pos.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Circle3D WithAxis(in Axis3D a1)
    {
        Circle3D copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Circle3D WithLocation(in Point3D p)
    {
        Circle3D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Circle3D WithPosition(in Frame3D a2)
    {
        Circle3D copy = this;
        copy.SetPosition(a2);
        return copy;
    }

    public readonly Circle3D WithRadius(double radius)
    {
        Circle3D copy = this;
        copy.SetRadius(radius);
        return copy;
    }

    // Queries

    public readonly double Area() => M_PI * _radius * _radius;

    /// <summary>The main axis of the circle: the normal to its plane through its centre.</summary>
    public readonly Axis3D Axis() => _pos.Axis();

    /// <summary>The circumference.</summary>
    public readonly double Length() => 2.0 * M_PI * _radius;

    /// <summary>The centre of the circle.</summary>
    public readonly Point3D Location() => _pos.Location();

    public readonly Frame3D Position() => _pos;

    public readonly double Radius() => _radius;

    public readonly Axis3D XAxis() => new(_pos.Location(), _pos.XDirection());

    public readonly Axis3D YAxis() => new(_pos.Location(), _pos.YDirection());

    public readonly double Distance(in Point3D p) => Math.Sqrt(SquareDistance(p));

    public readonly double SquareDistance(in Point3D p)
    {
        Vector3D v = new(Location(), p);
        double x = v.Dot(new Vector3D(_pos.XDirection()));
        double y = v.Dot(new Vector3D(_pos.YDirection()));
        double z = v.Dot(new Vector3D(_pos.Direction()));
        double t = Math.Sqrt(x * x + y * y) - _radius;
        return (t * t + z * z);
    }

    public readonly bool Contains(in Point3D p, double linearTolerance) => Distance(p) <= linearTolerance;

    public readonly Circle3D Mirrored(in Point3D p)
    {
        Circle3D c = this;
        c._pos.Mirror(p);
        return c;
    }

    public readonly Circle3D Mirrored(in Axis3D a1)
    {
        Circle3D c = this;
        c._pos.Mirror(a1);
        return c;
    }

    public readonly Circle3D Mirrored(in Frame3D a2)
    {
        Circle3D c = this;
        c._pos.Mirror(a2);
        return c;
    }

    public readonly Circle3D Rotated(in Axis3D a1, double ang)
    {
        Circle3D c = this;
        c._pos.Rotate(a1, ang);
        return c;
    }

    public readonly Circle3D Scaled(in Point3D p, double s)
    {
        Circle3D c = this;
        c._radius *= s;
        if (c._radius < 0)
        {
            c._radius = -c._radius;
        }

        c._pos.Scale(p, s);
        return c;
    }

    public readonly Circle3D Transformed(in Transform3D t)
    {
        Circle3D c = this;
        c._radius *= t.ScaleFactor();
        if (c._radius < 0)
        {
            c._radius = -c._radius;
        }

        c._pos.Transform(t);
        return c;
    }

    public readonly Circle3D Translated(in Vector3D v)
    {
        Circle3D c = this;
        c._pos.Translate(v);
        return c;
    }

    public readonly Circle3D Translated(in Point3D p1, in Point3D p2)
    {
        Circle3D c = this;
        c._pos.Translate(p1, p2);
        return c;
    }

    public readonly bool Equals(Circle3D other) => _pos.Equals(other._pos) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is Circle3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pos, _radius);

    public static bool operator ==(in Circle3D left, in Circle3D right) => left.Equals(right);

    public static bool operator !=(in Circle3D left, in Circle3D right) => !left.Equals(right);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Circ({_pos}, {_radius:R})");
}
