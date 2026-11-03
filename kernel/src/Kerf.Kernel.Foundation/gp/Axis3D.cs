// Port of: src/FoundationClasses/TKMath/gp/gp_Ax1.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>An axis in 3D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax1()</c> is the Z axis at the origin, as in OCCT; <c>default(gp_Ax1)</c> is not valid.</remarks>
public partial struct Axis3D : IEquatable<Axis3D>
{
    private Point3D _loc;
    private Direction3D _vdir;

    public Axis3D()
    {
        _loc = new Point3D(0.0, 0.0, 0.0);
        _vdir = new Direction3D(Direction3D.D.Z);
    }

    public Axis3D(in Point3D p, in Direction3D v)
    {
        _loc = p;
        _vdir = v;
    }

    public Axis3D(in Point3D p, Direction3D.D dir)
    {
        _loc = p;
        _vdir = new Direction3D(dir);
    }

    public Axis3D(Direction3D.D dir)
    {
        _loc = new Point3D(0.0, 0.0, 0.0);
        _vdir = new Direction3D(dir);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetDirection(in Direction3D v) => _vdir = v;

    internal void SetLocation(in Point3D p) => _loc = p;

    internal void Reverse() => _vdir.Reverse();

    internal void Mirror(in Point3D p)
    {
        _loc.Mirror(p);
        _vdir.Reverse();
    }

    internal void Mirror(in Axis3D a1)
    {
        _loc.Mirror(a1);
        _vdir.Mirror(a1._vdir);
    }

    internal void Mirror(in Frame3D a2)
    {
        _loc.Mirror(a2);
        _vdir.Mirror(a2);
    }

    internal void Rotate(in Axis3D a1, double angRad)
    {
        _loc.Rotate(a1, angRad);
        _vdir.Rotate(a1, angRad);
    }

    internal void Scale(in Point3D p, double s)
    {
        _loc.Scale(p, s);
        if (s < 0.0)
        {
            _vdir.Reverse();
        }
    }

    internal void Transform(in Transform3D t)
    {
        _loc.Transform(t);
        _vdir.Transform(t);
    }

    internal void Translate(in Vector3D v) => _loc.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _loc.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Axis3D WithDirection(in Direction3D v) => new(_loc, v);

    public readonly Axis3D WithLocation(in Point3D p) => new(p, _vdir);

    // Queries

    public readonly Direction3D Direction() => _vdir;

    public readonly Point3D Location() => _loc;

    public readonly bool IsCoaxial(in Axis3D other, double angularTolerance, double linearTolerance)
    {
        Xyz xyz1 = _loc.XYZ();
        xyz1.Subtract(other._loc.XYZ());
        xyz1.Cross(other._vdir.XYZ());
        double d1 = xyz1.Modulus();
        Xyz xyz2 = other._loc.XYZ();
        xyz2.Subtract(_loc.XYZ());
        xyz2.Cross(_vdir.XYZ());
        double d2 = xyz2.Modulus();
        return _vdir.IsEqual(other._vdir, angularTolerance) && d1 <= linearTolerance && d2 <= linearTolerance;
    }

    public readonly bool IsNormal(in Axis3D other, double angularTolerance) => _vdir.IsNormal(other._vdir, angularTolerance);

    public readonly bool IsOpposite(in Axis3D other, double angularTolerance) => _vdir.IsOpposite(other._vdir, angularTolerance);

    public readonly bool IsParallel(in Axis3D other, double angularTolerance) => _vdir.IsParallel(other._vdir, angularTolerance);

    public readonly double Angle(in Axis3D other) => _vdir.Angle(other._vdir);

    public readonly Axis3D Reversed() => new(_loc, _vdir.Reversed());

    public readonly Axis3D Mirrored(in Point3D p)
    {
        Axis3D a1 = this;
        a1.Mirror(p);
        return a1;
    }

    public readonly Axis3D Mirrored(in Axis3D a1)
    {
        Axis3D a = this;
        a.Mirror(a1);
        return a;
    }

    public readonly Axis3D Mirrored(in Frame3D a2)
    {
        Axis3D a1 = this;
        a1.Mirror(a2);
        return a1;
    }

    public readonly Axis3D Rotated(in Axis3D a1, double angRad)
    {
        Axis3D a = this;
        a.Rotate(a1, angRad);
        return a;
    }

    public readonly Axis3D Scaled(in Point3D p, double s)
    {
        Axis3D a1 = this;
        a1.Scale(p, s);
        return a1;
    }

    public readonly Axis3D Transformed(in Transform3D t)
    {
        Axis3D a1 = this;
        a1.Transform(t);
        return a1;
    }

    public readonly Axis3D Translated(in Vector3D v)
    {
        Axis3D a1 = this;
        a1._loc.Translate(v);
        return a1;
    }

    public readonly Axis3D Translated(in Point3D p1, in Point3D p2)
    {
        Axis3D a1 = this;
        a1._loc.Translate(p1, p2);
        return a1;
    }

    public readonly bool Equals(Axis3D other) => _loc.Equals(other._loc) && _vdir.Equals(other._vdir);

    public override readonly bool Equals(object? obj) => obj is Axis3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_loc, _vdir);

    public static bool operator ==(in Axis3D left, in Axis3D right) => left.Equals(right);

    public static bool operator !=(in Axis3D left, in Axis3D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Ax1({_loc}, {_vdir})";
}
