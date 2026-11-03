// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>An axis in 2D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax2d()</c> is the X axis at the origin, as in OCCT; <c>default(gp_Ax2d)</c> is not valid.</remarks>
public partial struct Axis2D : IEquatable<Axis2D>
{
    private Point2D _loc;
    private Direction2D _vdir;

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

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in Point2D p) => _loc = p;

    internal void SetDirection(in Direction2D v) => _vdir = v;

    internal void Reverse() => _vdir.Reverse();

    internal void Mirror(in Point2D p)
    {
        _loc.Mirror(p);
        _vdir.Reverse();
    }

    internal void Mirror(in Axis2D a)
    {
        _loc.Mirror(a);
        _vdir.Mirror(a._vdir);
    }

    internal void Rotate(in Point2D p, double ang)
    {
        _loc.Rotate(p, ang);
        _vdir.Rotate(ang);
    }

    internal void Scale(in Point2D p, double s)
    {
        _loc.Scale(p, s);
        if (s < 0.0)
        {
            _vdir.Reverse();
        }
    }

    internal void Transform(in Transform2D t)
    {
        _loc.Transform(t);
        _vdir.Transform(t);
    }

    internal void Translate(in Vector2D v) => _loc.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _loc.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Axis2D WithLocation(in Point2D p) => new(p, _vdir);

    public readonly Axis2D WithDirection(in Direction2D v) => new(_loc, v);

    // Queries

    public readonly Point2D Location() => _loc;

    public readonly Direction2D Direction() => _vdir;

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
    public readonly double Angle(in Axis2D other) => _vdir.Angle(other._vdir);

    public readonly Axis2D Reversed()
    {
        Axis2D temp = this;
        temp.Reverse();
        return temp;
    }

    public readonly Axis2D Mirrored(in Point2D p)
    {
        Axis2D a = this;
        a.Mirror(p);
        return a;
    }

    public readonly Axis2D Mirrored(in Axis2D a)
    {
        Axis2D aa = this;
        aa.Mirror(a);
        return aa;
    }

    public readonly Axis2D Rotated(in Point2D p, double ang)
    {
        Axis2D a = this;
        a.Rotate(p, ang);
        return a;
    }

    public readonly Axis2D Scaled(in Point2D p, double s)
    {
        Axis2D a = this;
        a.Scale(p, s);
        return a;
    }

    public readonly Axis2D Transformed(in Transform2D t)
    {
        Axis2D a = this;
        a.Transform(t);
        return a;
    }

    public readonly Axis2D Translated(in Vector2D v)
    {
        Axis2D a = this;
        (a._loc).Translate(v);
        return a;
    }

    public readonly Axis2D Translated(in Point2D p1, in Point2D p2)
    {
        Axis2D a = this;
        (a._loc).Translate(p1, p2);
        return a;
    }

    public readonly bool Equals(Axis2D other) => _loc.Equals(other._loc) && _vdir.Equals(other._vdir);

    public override readonly bool Equals(object? obj) => obj is Axis2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_loc, _vdir);

    public static bool operator ==(in Axis2D left, in Axis2D right) => left.Equals(right);

    public static bool operator !=(in Axis2D left, in Axis2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Ax2d({_loc}, {_vdir})";
}
