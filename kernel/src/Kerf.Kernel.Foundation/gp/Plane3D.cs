// Port of: src/FoundationClasses/TKMath/gp/gp_Pln.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

namespace Kerf.Geometry;

/// <summary>
/// A plane in 3D space, positioned by a coordinate system: the origin is a point of the plane, the X and Y
/// directions span it and the main direction is its normal. It is parameterised by
/// <c>P(U, V) = O + U * XDir + V * YDir</c>.
/// </summary>
/// <remarks><c>new gp_Pln()</c> is the XOY plane, as in OCCT; <c>default(gp_Pln)</c> is not valid.</remarks>
public readonly partial struct Plane3D : IEquatable<Plane3D>
{
    private readonly CoordinateSystem _position;

    /// <summary>The XY plane through the origin.</summary>
    public Plane3D() => _position = new CoordinateSystem();

    /// <summary>The plane through the location of <paramref name="position"/>, normal to its main direction, with U along its X direction and V along its Y direction.</summary>
    public Plane3D(in CoordinateSystem position) => _position = position;

    /// <summary>The plane through <paramref name="point"/> with normal <paramref name="direction"/>; the X direction is chosen.</summary>
    public Plane3D(in Point3D point, in Direction3D direction)
    {
        double a = direction.X;
        double b = direction.Y;
        double c = direction.Z;
        double absA = Math.Abs(a);
        double absB = Math.Abs(b);
        double absC = Math.Abs(c);
        // To determine the X axis:
        // we require that the dot product Vx.V = 0.
        // and we search for the max(A,B,C) to perform the division.
        // one of the coordinates of the vector is zero.
        if (absB <= absA && absB <= absC)
        {
            if (absA > absC)
            {
                _position = new CoordinateSystem(point, direction, new Direction3D(-c, 0.0, a));
            }
            else
            {
                _position = new CoordinateSystem(point, direction, new Direction3D(c, 0.0, -a));
            }
        }
        else if (absA <= absB && absA <= absC)
        {
            if (absB > absC)
            {
                _position = new CoordinateSystem(point, direction, new Direction3D(0.0, -c, b));
            }
            else
            {
                _position = new CoordinateSystem(point, direction, new Direction3D(0.0, c, -b));
            }
        }
        else
        {
            if (absA > absB)
            {
                _position = new CoordinateSystem(point, direction, new Direction3D(-b, a, 0.0));
            }
            else
            {
                _position = new CoordinateSystem(point, direction, new Direction3D(b, -a, 0.0));
            }
        }
    }

    /// <summary>The plane <c>A * X + B * Y + C * Z + D = 0</c>.</summary>
    /// <exception cref="ConstructionErrorException">A, B and C are all zero.</exception>
    public Plane3D(double a, double b, double c, double d)
    {
        double absA = Math.Abs(a);
        double absB = Math.Abs(b);
        double absC = Math.Abs(c);
        if (absB <= absA && absB <= absC)
        {
            if (absA > absC)
            {
                _position = new CoordinateSystem(new Point3D(-d / a, 0.0, 0.0), new Direction3D(a, b, c), new Direction3D(-c, 0.0, a));
            }
            else
            {
                _position = new CoordinateSystem(new Point3D(0.0, 0.0, -d / c), new Direction3D(a, b, c), new Direction3D(c, 0.0, -a));
            }
        }
        else if (absA <= absB && absA <= absC)
        {
            if (absB > absC)
            {
                _position = new CoordinateSystem(new Point3D(0.0, -d / b, 0.0), new Direction3D(a, b, c), new Direction3D(0.0, -c, b));
            }
            else
            {
                _position = new CoordinateSystem(new Point3D(0.0, 0.0, -d / c), new Direction3D(a, b, c), new Direction3D(0.0, c, -b));
            }
        }
        else
        {
            if (absA > absB)
            {
                _position = new CoordinateSystem(new Point3D(-d / a, 0.0, 0.0), new Direction3D(a, b, c), new Direction3D(-b, a, 0.0));
            }
            else
            {
                _position = new CoordinateSystem(new Point3D(0.0, -d / b, 0.0), new Direction3D(a, b, c), new Direction3D(b, -a, 0.0));
            }
        }
    }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Plane3D WithAxis(in Axis3D axis)
    {
        CoordinateSystem position = _position;
        position = position.WithAxis(axis);
        return new Plane3D(position);
    }

    /// <summary>A copy moved so that its origin is <paramref name="location"/>; its directions stay.</summary>
    public readonly Plane3D WithLocation(in Point3D location)
    {
        CoordinateSystem position = _position;
        position = position.WithLocation(location);
        return new Plane3D(position);
    }

    /// <summary>A copy positioned by <paramref name="position"/>.</summary>
    public readonly Plane3D WithPosition(in CoordinateSystem position) => new(position);

    /// <summary>The same plane with U running the other way: the X direction reverses, so the coordinate system becomes left-handed and the normal stays.</summary>
    public readonly Plane3D UReversed()
    {
        CoordinateSystem position = _position;
        position = position.XReversed();
        return new Plane3D(position);
    }

    /// <summary>The same plane with V running the other way: the Y direction reverses, so the coordinate system becomes left-handed and the normal stays.</summary>
    public readonly Plane3D VReversed()
    {
        CoordinateSystem position = _position;
        position = position.YReversed();
        return new Plane3D(position);
    }

    // Queries

    /// <summary>
    /// The coefficients of the plane's equation <c>A * X + B * Y + C * Z + D = 0</c>, with (A, B, C) the normal of
    /// a direct system (reversed for an indirect one).
    /// </summary>
    public readonly void Coefficients(out double a, out double b, out double c, out double d)
    {
        Direction3D dir = _position.Direction;
        if (_position.IsDirect)
        {
            a = dir.X;
            b = dir.Y;
            c = dir.Z;
        }
        else
        {
            a = -dir.X;
            b = -dir.Y;
            c = -dir.Z;
        }

        Point3D p = _position.Location;
        d = -(a * p.X + b * p.Y + c * p.Z);
    }

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool IsDirect => _position.IsDirect;

    /// <summary>The normal axis through the plane's origin.</summary>
    public readonly Axis3D Axis => _position.Axis;

    /// <summary>The origin, where U and V are 0.</summary>
    public readonly Point3D Location => _position.Location;

    /// <summary>The coordinate system the surface is positioned by.</summary>
    public readonly CoordinateSystem Position => _position;

    /// <summary>The distance from <paramref name="point"/> to the plane, with no sign: <see cref="SignedDistance(in Point3D)"/> has one.</summary>
    public readonly double DistanceTo(in Point3D point) => Math.Abs(SignedDistance(point));

    /// <summary>The distance to a line parallel to the plane; 0 when the line is not parallel.</summary>
    public readonly double DistanceTo(in Line3D line) => Math.Abs(SignedDistance(line));

    /// <summary>The distance to a parallel plane; 0 when the planes are not parallel.</summary>
    public readonly double DistanceTo(in Plane3D other) => Math.Abs(SignedDistance(other));

    /// <summary>The signed distance, positive on the side the normal points to.</summary>
    public readonly double SignedDistance(in Point3D point)
    {
        Point3D loc = _position.Location;
        Direction3D dir = _position.Direction;
        return (dir.X * (point.X - loc.X) + dir.Y * (point.Y - loc.Y)
                + dir.Z * (point.Z - loc.Z));
    }

    /// <summary>The distance to the line, positive on the side the normal points to: 0 when the line isn't parallel to the plane.</summary>
    public readonly double SignedDistance(in Line3D line)
    {
        if (!_position.Direction.IsNormal(line.Direction, Constants.Resolution()))
        {
            return 0.0;
        }

        return SignedDistance(line.Location);
    }

    /// <summary>The distance to the plane, positive on the side the normal points to: 0 when the planes aren't parallel.</summary>
    public readonly double SignedDistance(in Plane3D other)
    {
        if (!_position.Direction.IsParallel(other._position.Direction, Constants.Resolution()))
        {
            return 0.0;
        }

        return SignedDistance(other._position.Location);
    }

    /// <summary>The square of the distance, without the square root where it can be avoided.</summary>
    public readonly double SquareDistanceTo(in Point3D point)
    {
        double d = DistanceTo(point);
        return d * d;
    }

    /// <summary>The square of the distance: 0 when the line isn't parallel to the plane.</summary>
    public readonly double SquareDistanceTo(in Line3D line)
    {
        double d = DistanceTo(line);
        return d * d;
    }

    /// <summary>The square of the distance: 0 when the planes aren't parallel.</summary>
    public readonly double SquareDistanceTo(in Plane3D other)
    {
        double d = DistanceTo(other);
        return d * d;
    }

    /// <summary>The X axis: the U parameter direction through the origin.</summary>
    public readonly Axis3D XAxis => new(_position.Location, _position.XDirection);

    /// <summary>The Y axis: the V parameter direction through the origin.</summary>
    public readonly Axis3D YAxis => new(_position.Location, _position.YDirection);

    /// <summary>True when <paramref name="point"/> is within <paramref name="linearTolerance"/> of it.</summary>
    public readonly bool Contains(in Point3D point, double linearTolerance) => DistanceTo(point) <= linearTolerance;

    /// <summary>True when the location of <paramref name="line"/> is within <paramref name="linearTolerance"/> of the plane, and the line is parallel to it within <paramref name="angularTolerance"/> radians.</summary>
    public readonly bool Contains(in Line3D line, double linearTolerance, double angularTolerance)
        => Contains(line.Location, linearTolerance)
           && _position.Direction.IsNormal(line.Direction, angularTolerance);

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Plane3D Mirrored(in Point3D point) => new(_position.Mirrored(point));

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Plane3D Mirrored(in Axis3D axis) => new(_position.Mirrored(axis));

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Plane3D Mirrored(in Frame3D frame) => new(_position.Mirrored(frame));

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Plane3D Rotated(in Axis3D axis, double angle)
    {
        CoordinateSystem position = _position;
        position = position.Rotated(axis, angle);
        return new Plane3D(position);
    }

    /// <summary>The origin scaled about <paramref name="point"/> by <paramref name="factor"/>. A negative factor reverses all three directions.</summary>
    public readonly Plane3D Scaled(in Point3D point, double factor)
    {
        CoordinateSystem position = _position;
        position = position.Scaled(point, factor);
        return new Plane3D(position);
    }

    /// <summary>The image under <paramref name="transform"/>.</summary>
    public readonly Plane3D Transformed(in Transform3D transform)
    {
        CoordinateSystem position = _position;
        position = position.Transformed(transform);
        return new Plane3D(position);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Plane3D Translated(in Vector3D vector)
    {
        CoordinateSystem position = _position;
        position = position.Translated(vector);
        return new Plane3D(position);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Plane3D Translated(in Point3D point1, in Point3D point2)
    {
        CoordinateSystem position = _position;
        position = position.Translated(point1, point2);
        return new Plane3D(position);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Plane3D other) => _position.Equals(other._position);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Plane3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Plane3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => _position.GetHashCode();

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Plane3D left, in Plane3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Plane3D left, in Plane3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Plane3D({_position})";
}
