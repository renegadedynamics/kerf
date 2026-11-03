// Port of: src/FoundationClasses/TKMath/gp/gp_Lin.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Lin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Occt;

namespace Kerf.Geometry;

/// <summary>An infinite line in 3D space, positioned by an axis (a location and a unit direction).</summary>
/// <remarks><c>new gp_Lin()</c> is the Z axis, as in OCCT; <c>default(gp_Lin)</c> is not valid.</remarks>
public readonly partial struct Line3D : IEquatable<Line3D>
{
    private readonly Axis3D _pos;

    public Line3D() => _pos = new Axis3D();

    public Line3D(in Axis3D a1) => _pos = a1;

    public Line3D(in Point3D p, in Direction3D v) => _pos = new Axis3D(p, v);

    // Copies

    public readonly Line3D WithDirection(in Direction3D v)
    {
        Axis3D pos = _pos;
        pos = pos.WithDirection(v);
        return new Line3D(pos);
    }

    public readonly Line3D WithLocation(in Point3D p)
    {
        Axis3D pos = _pos;
        pos = pos.WithLocation(p);
        return new Line3D(pos);
    }

    public readonly Line3D WithPosition(in Axis3D a1) => new(a1);

    // Queries

    public readonly Line3D Reversed()
    {
        Axis3D pos = _pos;
        pos = pos.Reversed();
        return new Line3D(pos);
    }

    public readonly Direction3D Direction => _pos.Direction;

    public readonly Point3D Location => _pos.Location;

    public readonly Axis3D Position => _pos;

    /// <summary>The angle between the directions of the two lines, in [0, PI].</summary>
    public readonly double AngleTo(in Line3D other) => _pos.Direction.AngleTo(other._pos.Direction);

    public readonly bool Contains(in Point3D p, double linearTolerance) => DistanceTo(p) <= linearTolerance;

    public readonly double DistanceTo(in Point3D p)
    {
        Xyz coord = p.XYZ();
        coord.Subtract((_pos.Location).XYZ());
        coord.Cross((_pos.Direction).XYZ());
        return coord.Modulus();
    }

    /// <summary>The minimum distance between the two lines.</summary>
    public readonly double DistanceTo(in Line3D other)
    {
        if (_pos.IsParallel(other._pos, Constants.Resolution()))
        {
            return other.DistanceTo(_pos.Location);
        }
        else
        {
            Direction3D dir = _pos.Direction.Crossed(other._pos.Direction);
            return Math.Abs(new Vector3D(_pos.Location, other._pos.Location).Dot(new Vector3D(dir)));
        }
    }

    public readonly double SquareDistanceTo(in Point3D p)
    {
        Point3D loc = _pos.Location;
        Vector3D v = new(p.X - loc.X, p.Y - loc.Y, p.Z - loc.Z);
        v = v.Crossed(new Vector3D(_pos.Direction));
        return v.SquareMagnitude;
    }

    public readonly double SquareDistanceTo(in Line3D other)
    {
        double d = DistanceTo(other);
        return d * d;
    }

    /// <summary>The line through <paramref name="p"/> normal to this line and intersecting it.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="p"/> is on this line.</exception>
    public readonly Line3D Normal(in Point3D p)
    {
        Point3D loc = _pos.Location;
        Direction3D v = new(p.X - loc.X, p.Y - loc.Y, p.Z - loc.Z);
        v = _pos.Direction.CrossCrossed(v, _pos.Direction);
        return new Line3D(p, v);
    }

    public readonly Line3D Mirrored(in Point3D p)
    {
        Axis3D pos = _pos;
        pos = pos.Mirrored(p);
        return new Line3D(pos);
    }

    public readonly Line3D Mirrored(in Axis3D a1)
    {
        Axis3D pos = _pos;
        pos = pos.Mirrored(a1);
        return new Line3D(pos);
    }

    public readonly Line3D Mirrored(in Frame3D a2)
    {
        Axis3D pos = _pos;
        pos = pos.Mirrored(a2);
        return new Line3D(pos);
    }

    public readonly Line3D Rotated(in Axis3D a1, double ang)
    {
        Axis3D pos = _pos;
        pos = pos.Rotated(a1, ang);
        return new Line3D(pos);
    }

    public readonly Line3D Scaled(in Point3D p, double s)
    {
        Axis3D pos = _pos;
        pos = pos.Scaled(p, s);
        return new Line3D(pos);
    }

    public readonly Line3D Transformed(in Transform3D t)
    {
        Axis3D pos = _pos;
        pos = pos.Transformed(t);
        return new Line3D(pos);
    }

    public readonly Line3D Translated(in Vector3D v)
    {
        Axis3D pos = _pos;
        pos = pos.Translated(v);
        return new Line3D(pos);
    }

    public readonly Line3D Translated(in Point3D p1, in Point3D p2)
    {
        Axis3D pos = _pos;
        pos = pos.Translated(new Vector3D(p1, p2));
        return new Line3D(pos);
    }

    public readonly bool Equals(Line3D other) => _pos.Equals(other._pos);

    public override readonly bool Equals(object? obj) => obj is Line3D other && Equals(other);

    public override readonly int GetHashCode() => _pos.GetHashCode();

    public static bool operator ==(in Line3D left, in Line3D right) => left.Equals(right);

    public static bool operator !=(in Line3D left, in Line3D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Lin({_pos.Location}, {_pos.Direction})";
}
