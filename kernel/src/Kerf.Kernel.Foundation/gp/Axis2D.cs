// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>An axis in 2D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax2d()</c> is the X axis at the origin, as in OCCT; <c>default(gp_Ax2d)</c> is not valid.</remarks>
public readonly partial struct Axis2D : IEquatable<Axis2D>
{
    private readonly Point2D _loc;
    private readonly Direction2D _vdir;

    public Axis2D()
    {
        _loc = new Point2D(0.0, 0.0);
        _vdir = new Direction2D();
    }

    public Axis2D(in Point2D p, in Direction2D v)
    {
        _loc = p;
        _vdir = v;
    }

    public Axis2D(in Point2D p, Direction2D.D dir)
    {
        _loc = p;
        _vdir = new Direction2D(dir);
    }

    public Axis2D(Direction2D.D dir)
    {
        _loc = new Point2D(0.0, 0.0);
        _vdir = new Direction2D(dir);
    }

    // Copies

    public readonly Axis2D WithLocation(in Point2D p) => new(p, _vdir);

    public readonly Axis2D WithDirection(in Direction2D v) => new(_loc, v);

    // Queries

    public readonly Point2D Location => _loc;

    public readonly Direction2D Direction => _vdir;

    /// <summary>True when the directions are parallel within the angular tolerance and each location is within the linear tolerance of the other axis.</summary>
    public readonly bool IsCoaxial(in Axis2D other, double angularTolerance, double linearTolerance)
    {
        Xy xy1 = _loc.XY();
        xy1.Subtract(other._loc.XY());
        double d1 = xy1.Crossed(other._vdir.XY());
        if (d1 < 0)
        {
            d1 = -d1;
        }

        Xy xy2 = other._loc.XY();
        xy2.Subtract(_loc.XY());
        double d2 = xy2.Crossed(_vdir.XY());
        if (d2 < 0)
        {
            d2 = -d2;
        }

        return (_vdir.IsParallel(other._vdir, angularTolerance) && d1 <= linearTolerance && d2 <= linearTolerance);
    }

    public readonly bool IsNormal(in Axis2D other, double angularTolerance) => _vdir.IsNormal(other._vdir, angularTolerance);

    public readonly bool IsOpposite(in Axis2D other, double angularTolerance) => _vdir.IsOpposite(other._vdir, angularTolerance);

    public readonly bool IsParallel(in Axis2D other, double angularTolerance) => _vdir.IsParallel(other._vdir, angularTolerance);

    /// <summary>The signed angle between the directions, in ]-PI, PI].</summary>
    public readonly double AngleTo(in Axis2D other) => _vdir.AngleTo(other._vdir);

    public readonly Axis2D Reversed()
    {
        Direction2D vdir = _vdir;
        vdir = vdir.Reversed();
        return new Axis2D(_loc, vdir);
    }

    public readonly Axis2D Mirrored(in Point2D p)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Mirrored(p);
        vdir = vdir.Reversed();
        return new Axis2D(loc, vdir);
    }

    public readonly Axis2D Mirrored(in Axis2D a)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Mirrored(a);
        vdir = vdir.Mirrored(a._vdir);
        return new Axis2D(loc, vdir);
    }

    public readonly Axis2D Rotated(in Point2D p, double ang)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Rotated(p, ang);
        vdir = vdir.Rotated(ang);
        return new Axis2D(loc, vdir);
    }

    public readonly Axis2D Scaled(in Point2D p, double s)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Scaled(p, s);
        if (s < 0.0)
        {
            vdir = vdir.Reversed();
        }
        return new Axis2D(loc, vdir);
    }

    public readonly Axis2D Transformed(in Transform2D t)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Transformed(t);
        vdir = vdir.Transformed(t);
        return new Axis2D(loc, vdir);
    }

    public readonly Axis2D Translated(in Vector2D v)
    {
        return new Axis2D(_loc.Translated(v), _vdir);
    }

    public readonly Axis2D Translated(in Point2D p1, in Point2D p2)
    {
        return new Axis2D(_loc.Translated(p1, p2), _vdir);
    }

    public readonly bool Equals(Axis2D other) => _loc.Equals(other._loc) && _vdir.Equals(other._vdir);

    public override readonly bool Equals(object? obj) => obj is Axis2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_loc, _vdir);

    public static bool operator ==(in Axis2D left, in Axis2D right) => left.Equals(right);

    public static bool operator !=(in Axis2D left, in Axis2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Ax2d({_loc}, {_vdir})";
}
