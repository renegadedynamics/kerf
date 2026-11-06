// Port of: src/FoundationClasses/TKMath/gp/gp_Ax1.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>An axis in 3D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax1()</c> is the Z axis at the origin, as in OCCT; <c>default(gp_Ax1)</c> is not valid.</remarks>
public readonly partial struct Axis3D : IEquatable<Axis3D>
{
    private readonly Point3D _loc;
    private readonly Direction3D _vdir;

    public Axis3D()
    {
        _loc = new Point3D(0.0, 0.0, 0.0);
        _vdir = new Direction3D(Direction3D.D.Z);
    }

    public Axis3D(in Point3D point, in Direction3D direction)
    {
        _loc = point;
        _vdir = direction;
    }

    internal Axis3D(in Point3D p, Direction3D.D dir)
    {
        _loc = p;
        _vdir = new Direction3D(dir);
    }

    internal Axis3D(Direction3D.D dir)
    {
        _loc = new Point3D(0.0, 0.0, 0.0);
        _vdir = new Direction3D(dir);
    }

    // Copies

    public readonly Axis3D WithDirection(in Direction3D direction) => new(_loc, direction);

    public readonly Axis3D WithLocation(in Point3D point) => new(point, _vdir);

    // Queries

    public readonly Direction3D Direction => _vdir;

    public readonly Point3D Location => _loc;

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

    public readonly double AngleTo(in Axis3D other) => _vdir.AngleTo(other._vdir);

    public readonly Axis3D Reversed() => new(_loc, _vdir.Reversed());

    public readonly Axis3D Mirrored(in Point3D point)
    {
        Point3D loc = _loc;
        Direction3D vdir = _vdir;
        loc = loc.Mirrored(point);
        vdir = vdir.Reversed();
        return new Axis3D(loc, vdir);
    }

    public readonly Axis3D Mirrored(in Axis3D axis)
    {
        Point3D loc = _loc;
        Direction3D vdir = _vdir;
        loc = loc.Mirrored(axis);
        vdir = vdir.Mirrored(axis._vdir);
        return new Axis3D(loc, vdir);
    }

    public readonly Axis3D Mirrored(in Frame3D frame)
    {
        Point3D loc = _loc;
        Direction3D vdir = _vdir;
        loc = loc.Mirrored(frame);
        vdir = vdir.Mirrored(frame);
        return new Axis3D(loc, vdir);
    }

    public readonly Axis3D Rotated(in Axis3D axis, double angle)
    {
        Point3D loc = _loc;
        Direction3D vdir = _vdir;
        loc = loc.Rotated(axis, angle);
        vdir = vdir.Rotated(axis, angle);
        return new Axis3D(loc, vdir);
    }

    public readonly Axis3D Scaled(in Point3D point, double factor)
    {
        Point3D loc = _loc;
        Direction3D vdir = _vdir;
        loc = loc.Scaled(point, factor);
        if (factor < 0.0)
        {
            vdir = vdir.Reversed();
        }
        return new Axis3D(loc, vdir);
    }

    public readonly Axis3D Transformed(in Transform3D transform)
    {
        Point3D loc = _loc;
        Direction3D vdir = _vdir;
        loc = loc.Transformed(transform);
        vdir = vdir.Transformed(transform);
        return new Axis3D(loc, vdir);
    }

    public readonly Axis3D Translated(in Vector3D vector)
    {
        Point3D loc = _loc;
        loc = loc.Translated(vector);
        return new Axis3D(loc, _vdir);
    }

    public readonly Axis3D Translated(in Point3D point1, in Point3D point2)
    {
        Point3D loc = _loc;
        loc = loc.Translated(point1, point2);
        return new Axis3D(loc, _vdir);
    }

    public readonly bool Equals(Axis3D other) => _loc.Equals(other._loc) && _vdir.Equals(other._vdir);

    public override readonly bool Equals(object? obj) => obj is Axis3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_loc, _vdir);

    public static bool operator ==(in Axis3D left, in Axis3D right) => left.Equals(right);

    public static bool operator !=(in Axis3D left, in Axis3D right) => !left.Equals(right);

    public override readonly string ToString() => $"Axis3D({_loc}, {_vdir})";
}
