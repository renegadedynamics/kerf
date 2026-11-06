// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>An axis in 2D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax2d()</c> is the X axis at the origin, as in OCCT; <c>default(gp_Ax2d)</c> is not valid.</remarks>
public readonly partial struct Axis2D : IEquatable<Axis2D>
{
    private readonly Point2D _loc;
    private readonly Direction2D _vdir;

    /// <summary>The X axis through the origin.</summary>
    public Axis2D()
    {
        _loc = new Point2D(0.0, 0.0);
        _vdir = new Direction2D();
    }

    /// <summary>The axis through <paramref name="point"/> along <paramref name="direction"/>.</summary>
    public Axis2D(in Point2D point, in Direction2D direction)
    {
        _loc = point;
        _vdir = direction;
    }

    internal Axis2D(in Point2D p, Direction2D.D dir)
    {
        _loc = p;
        _vdir = new Direction2D(dir);
    }

    internal Axis2D(Direction2D.D dir)
    {
        _loc = new Point2D(0.0, 0.0);
        _vdir = new Direction2D(dir);
    }

    // Copies

    /// <summary>A copy through <paramref name="point"/>, with the same direction.</summary>
    public readonly Axis2D WithLocation(in Point2D point) => new(point, _vdir);

    /// <summary>A copy with this direction, through the same point.</summary>
    public readonly Axis2D WithDirection(in Direction2D direction) => new(_loc, direction);

    // Queries

    /// <summary>The point the axis passes through.</summary>
    public readonly Point2D Location => _loc;

    /// <summary>The unit direction.</summary>
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

    /// <summary>True when at right angles to <paramref name="other"/>, within <paramref name="angularTolerance"/> radians. Only the directions are compared, not the locations.</summary>
    public readonly bool IsNormal(in Axis2D other, double angularTolerance) => _vdir.IsNormal(other._vdir, angularTolerance);

    /// <summary>True when pointing the opposite way to <paramref name="other"/>, within <paramref name="angularTolerance"/> radians. Only the directions are compared, not the locations.</summary>
    public readonly bool IsOpposite(in Axis2D other, double angularTolerance) => _vdir.IsOpposite(other._vdir, angularTolerance);

    /// <summary>True when parallel to <paramref name="other"/>, in either sense, within <paramref name="angularTolerance"/> radians. Only the directions are compared, not the locations.</summary>
    public readonly bool IsParallel(in Axis2D other, double angularTolerance) => _vdir.IsParallel(other._vdir, angularTolerance);

    /// <summary>The signed angle between the directions, in ]-PI, PI].</summary>
    public readonly double AngleTo(in Axis2D other) => _vdir.AngleTo(other._vdir);

    /// <summary>The same axis, pointing the other way.</summary>
    public readonly Axis2D Reversed()
    {
        Direction2D vdir = _vdir;
        vdir = vdir.Reversed();
        return new Axis2D(_loc, vdir);
    }

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Axis2D Mirrored(in Point2D point)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Mirrored(point);
        vdir = vdir.Reversed();
        return new Axis2D(loc, vdir);
    }

    /// <summary>The reflection in the line <paramref name="axis"/>.</summary>
    public readonly Axis2D Mirrored(in Axis2D axis)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Mirrored(axis);
        vdir = vdir.Mirrored(axis._vdir);
        return new Axis2D(loc, vdir);
    }

    /// <summary>Rotated about <paramref name="point"/> by <paramref name="angle"/> radians, anticlockwise.</summary>
    public readonly Axis2D Rotated(in Point2D point, double angle)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Rotated(point, angle);
        vdir = vdir.Rotated(angle);
        return new Axis2D(loc, vdir);
    }

    /// <summary>The location scaled about <paramref name="point"/> by <paramref name="factor"/>. The direction stays, unless a negative factor reverses it.</summary>
    public readonly Axis2D Scaled(in Point2D point, double factor)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Scaled(point, factor);
        if (factor < 0.0)
        {
            vdir = vdir.Reversed();
        }
        return new Axis2D(loc, vdir);
    }

    /// <summary>The image under <paramref name="transform"/>. The direction is transformed as a direction, so it stays a unit vector.</summary>
    public readonly Axis2D Transformed(in Transform2D transform)
    {
        Point2D loc = _loc;
        Direction2D vdir = _vdir;
        loc = loc.Transformed(transform);
        vdir = vdir.Transformed(transform);
        return new Axis2D(loc, vdir);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Axis2D Translated(in Vector2D vector)
    {
        return new Axis2D(_loc.Translated(vector), _vdir);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Axis2D Translated(in Point2D point1, in Point2D point2)
    {
        return new Axis2D(_loc.Translated(point1, point2), _vdir);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Axis2D other) => _loc.Equals(other._loc) && _vdir.Equals(other._vdir);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Axis2D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Axis2D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_loc, _vdir);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Axis2D left, in Axis2D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Axis2D left, in Axis2D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Axis2D({_loc}, {_vdir})";
}
