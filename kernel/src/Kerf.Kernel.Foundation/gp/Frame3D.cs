// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>A right-handed coordinate system in 3D space: a location, a main direction, and X and Y directions.</summary>
/// <remarks><c>new gp_Ax2()</c> is the global system, as in OCCT; <c>default(gp_Ax2)</c> is not valid.</remarks>
public readonly partial struct Frame3D : IEquatable<Frame3D>
{
    private readonly Axis3D _axis;
    private readonly Direction3D _vydir;
    private readonly Direction3D _vxdir;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    private Frame3D(in Axis3D axis, in Direction3D vydir, in Direction3D vxdir)
    {
        _axis = axis;
        _vydir = vydir;
        _vxdir = vxdir;
    }

    /// <summary>The absolute frame: the origin, with the X, Y and Z axes.</summary>
    public Frame3D()
    {
        _axis = new Axis3D();
        _vydir = new Direction3D(Direction3D.D.Y);
        _vxdir = new Direction3D(Direction3D.D.X);
    }

    /// <summary>A system at <paramref name="point"/> with main direction <paramref name="normal"/>, and X direction from <paramref name="xDirection"/> projected onto the plane normal to N.</summary>
    public Frame3D(in Point3D point, in Direction3D normal, in Direction3D xDirection)
    {
        _axis = new Axis3D(point, normal);
        _vydir = normal;
        _vxdir = normal;
        _vxdir = _vxdir.CrossCrossed(xDirection, normal);
        _vydir = _vydir.Crossed(_vxdir);
    }

    internal Frame3D(in Point3D p, Direction3D.D n, Direction3D.D vx)
    {
        _axis = new Axis3D(p, n);
        _vydir = new Direction3D(CrossStandardDir(n, vx));
        _vxdir = new Direction3D(vx);
    }

    /// <summary>A system at <paramref name="point"/> with main direction <paramref name="direction"/>, and X and Y directions chosen by OCCT's rule.</summary>
    public Frame3D(in Point3D point, in Direction3D direction)
    {
        _axis = new Axis3D(point, direction);
        _vydir = new Direction3D();
        _vxdir = new Direction3D();
        double a = direction.X;
        double b = direction.Y;
        double c = direction.Z;
        double aabs = Math.Abs(a);
        double babs = Math.Abs(b);
        double cabs = Math.Abs(c);
        Direction3D d = new();

        // Pick the X direction perpendicular to V from the component of smallest magnitude
        if (babs <= aabs && babs <= cabs)
        {
            if (aabs > cabs)
            {
                d = d.WithCoord(-c, 0.0, a);
            }
            else
            {
                d = d.WithCoord(c, 0.0, -a);
            }
        }
        else if (aabs <= babs && aabs <= cabs)
        {
            if (babs > cabs)
            {
                d = d.WithCoord(0.0, -c, b);
            }
            else
            {
                d = d.WithCoord(0.0, c, -b);
            }
        }
        else
        {
            if (aabs > babs)
            {
                d = d.WithCoord(-b, a, 0.0);
            }
            else
            {
                d = d.WithCoord(b, -a, 0.0);
            }
        }

        this = WithXDirection(d);
    }

    internal Frame3D(in Point3D p, Direction3D.D v)
    {
        _axis = new Axis3D(p, v);
        _vydir = new Direction3D(GetPerpendicularYDir(v));
        _vxdir = new Direction3D(GetPerpendicularXDir(v));
    }

    internal Frame3D(Direction3D.D v) : this(new Point3D(0.0, 0.0, 0.0), v) { }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Frame3D WithAxis(in Axis3D axis)
    {
        Axis3D newAxis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        double a = axis.Direction * vxdir;
        if (Math.Abs(Math.Abs(a) - 1.0) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (a > 0.0)
            {
                vxdir = vydir;
                vydir = newAxis.Direction;
                newAxis = axis;
            }
            else
            {
                vxdir = newAxis.Direction;
                newAxis = axis;
            }
        }
        else
        {
            newAxis = axis;
            vxdir = newAxis.Direction.CrossCrossed(vxdir, newAxis.Direction);
            vydir = newAxis.Direction.Crossed(vxdir);
        }
        return new Frame3D(newAxis, vydir, vxdir);
    }

    /// <summary>A copy with this main direction. The old X direction is projected on to the new XY plane; if it is parallel to <paramref name="direction"/>, the old directions swap roles instead.</summary>
    public readonly Frame3D WithDirection(in Direction3D direction)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        double a = direction * vxdir;
        if (Math.Abs(Math.Abs(a) - 1.0) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (a > 0.0)
            {
                vxdir = vydir;
                vydir = axis.Direction;
                axis = axis.WithDirection(direction);
            }
            else
            {
                vxdir = axis.Direction;
                axis = axis.WithDirection(direction);
            }
        }
        else
        {
            axis = axis.WithDirection(direction);
            vxdir = direction.CrossCrossed(vxdir, direction);
            vydir = direction.Crossed(vxdir);
        }
        return new Frame3D(axis, vydir, vxdir);
    }

    /// <summary>A copy moved so that its origin is <paramref name="point"/>; its directions stay.</summary>
    public readonly Frame3D WithLocation(in Point3D point)
    {
        Axis3D axis = _axis;
        axis = axis.WithLocation(point);
        return new Frame3D(axis, _vydir, _vxdir);
    }

    /// <summary>A copy whose X direction is <paramref name="xDirection"/> projected on to the XY plane; the Y direction is recomputed.</summary>
    /// <exception cref="ArgumentException"><paramref name="xDirection"/> is parallel to the main direction.</exception>
    public readonly Frame3D WithXDirection(in Direction3D xDirection)
    {
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        vxdir = _axis.Direction.CrossCrossed(xDirection, _axis.Direction);
        vydir = _axis.Direction.Crossed(vxdir);
        return new Frame3D(_axis, vydir, vxdir);
    }

    /// <summary>A copy whose Y direction is <paramref name="yDirection"/> projected on to the XY plane; the X direction is recomputed.</summary>
    /// <exception cref="ArgumentException"><paramref name="yDirection"/> is parallel to the main direction.</exception>
    public readonly Frame3D WithYDirection(in Direction3D yDirection)
    {
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        vxdir = yDirection.Crossed(_axis.Direction);
        vydir = _axis.Direction.Crossed(vxdir);
        return new Frame3D(_axis, vydir, vxdir);
    }

    // Queries

    /// <summary>The angle between the main directions, in [0, PI] radians, with no sign.</summary>
    public readonly double AngleTo(in Frame3D other) => _axis.AngleTo(other._axis);

    /// <summary>The main axis: the location and the main direction.</summary>
    public readonly Axis3D Axis => _axis;

    /// <summary>The main direction, normal to the XY plane.</summary>
    public readonly Direction3D Direction => _axis.Direction;

    /// <summary>The origin.</summary>
    public readonly Point3D Location => _axis.Location;

    /// <summary>The X direction, a unit vector.</summary>
    public readonly Direction3D XDirection => _vxdir;

    /// <summary>The Y direction: the main direction cross the X direction.</summary>
    public readonly Direction3D YDirection => _vydir;

    /// <summary>True when the main directions are parallel within <paramref name="angularTolerance"/> radians and the other's location is within <paramref name="linearTolerance"/> of this XY plane.</summary>
    public readonly bool IsCoplanar(in Frame3D other, double linearTolerance, double angularTolerance)
    {
        Direction3D dd = _axis.Direction;
        Point3D pp = _axis.Location;
        Point3D op = other._axis.Location;
        double d1 = dd.X * (op.X - pp.X) + dd.Y * (op.Y - pp.Y) + dd.Z * (op.Z - pp.Z);
        if (d1 < 0)
        {
            d1 = -d1;
        }

        return d1 <= linearTolerance && _axis.IsParallel(other._axis, angularTolerance);
    }

    /// <summary>True when <paramref name="axis"/> is normal to the main direction within <paramref name="angularTolerance"/> radians and its location is within <paramref name="linearTolerance"/> of this XY plane.</summary>
    public readonly bool IsCoplanar(in Axis3D axis, double linearTolerance, double angularTolerance)
    {
        Direction3D dd = _axis.Direction;
        Point3D pp = _axis.Location;
        Point3D ap = axis.Location;
        double d1 = dd.X * (ap.X - pp.X) + dd.Y * (ap.Y - pp.Y) + dd.Z * (ap.Z - pp.Z);
        if (d1 < 0)
        {
            d1 = -d1;
        }

        return d1 <= linearTolerance && _axis.IsNormal(axis, angularTolerance);
    }

    /// <summary>Moved through <paramref name="point"/>, with the X and Y directions reversed and the main direction kept, so it stays right-handed.</summary>
    public readonly Frame3D Mirrored(in Point3D point)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        Point3D temp = axis.Location;
        temp = temp.Mirrored(point);
        axis = axis.WithLocation(temp);
        vxdir = vxdir.Reversed();
        vydir = vydir.Reversed();
        return new Frame3D(axis, vydir, vxdir);
    }

    /// <summary>The half-turn about <paramref name="axis"/>: the X and Y directions are turned and the main direction recomputed.</summary>
    public readonly Frame3D Mirrored(in Axis3D axis)
    {
        Axis3D newAxis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        vydir = vydir.Mirrored(axis);
        vxdir = vxdir.Mirrored(axis);
        Point3D temp = newAxis.Location;
        temp = temp.Mirrored(axis);
        newAxis = newAxis.WithLocation(temp);
        newAxis = newAxis.WithDirection(vxdir.Crossed(vydir));
        return new Frame3D(newAxis, vydir, vxdir);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>: the X and Y directions are reflected and the main direction recomputed, so it stays right-handed.</summary>
    public readonly Frame3D Mirrored(in Frame3D frame)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        vydir = vydir.Mirrored(frame);
        vxdir = vxdir.Mirrored(frame);
        Point3D temp = axis.Location;
        temp = temp.Mirrored(frame);
        axis = axis.WithLocation(temp);
        axis = axis.WithDirection(vxdir.Crossed(vydir));
        return new Frame3D(axis, vydir, vxdir);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Frame3D Rotated(in Axis3D axis, double angle)
    {
        Axis3D newAxis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        Point3D temp = newAxis.Location;
        temp = temp.Rotated(axis, angle);
        newAxis = newAxis.WithLocation(temp);
        vxdir = vxdir.Rotated(axis, angle);
        vydir = vydir.Rotated(axis, angle);
        newAxis = newAxis.WithDirection(vxdir.Crossed(vydir));
        return new Frame3D(newAxis, vydir, vxdir);
    }

    /// <summary>The location scaled about <paramref name="point"/> by <paramref name="factor"/>. A negative factor reverses the X and Y directions, keeping the main one and the handedness.</summary>
    public readonly Frame3D Scaled(in Point3D point, double factor)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        Point3D temp = axis.Location;
        temp = temp.Scaled(point, factor);
        axis = axis.WithLocation(temp);
        if (factor < 0.0)
        {
            vxdir = vxdir.Reversed();
            vydir = vydir.Reversed();
        }
        return new Frame3D(axis, vydir, vxdir);
    }

    /// <summary>The image under <paramref name="transform"/>: the X and Y directions are transformed and the main direction recomputed, so it stays right-handed.</summary>
    public readonly Frame3D Transformed(in Transform3D transform)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        Point3D temp = axis.Location;
        temp = temp.Transformed(transform);
        axis = axis.WithLocation(temp);
        vxdir = vxdir.Transformed(transform);
        vydir = vydir.Transformed(transform);
        axis = axis.WithDirection(vxdir.Crossed(vydir));
        return new Frame3D(axis, vydir, vxdir);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Frame3D Translated(in Vector3D vector)
    {
        Axis3D axis = _axis;
        axis = axis.Translated(vector);
        return new Frame3D(axis, _vydir, _vxdir);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Frame3D Translated(in Point3D point1, in Point3D point2)
    {
        Axis3D axis = _axis;
        axis = axis.Translated(point1, point2);
        return new Frame3D(axis, _vydir, _vxdir);
    }

    internal static Direction3D.D GetPerpendicularXDir(Direction3D.D mainDir)
        => mainDir is Direction3D.D.X or Direction3D.D.NX ? Direction3D.D.Y
           : mainDir is Direction3D.D.Y or Direction3D.D.NY ? Direction3D.D.Z
           : Direction3D.D.X;

    internal static Direction3D.D GetPerpendicularYDir(Direction3D.D mainDir)
        => mainDir == Direction3D.D.Z ? Direction3D.D.Y
           : mainDir == Direction3D.D.NZ ? Direction3D.D.NY
           : mainDir == Direction3D.D.X ? Direction3D.D.Z
           : mainDir == Direction3D.D.NX ? Direction3D.D.NZ
           : mainDir == Direction3D.D.Y ? Direction3D.D.X
           : Direction3D.D.NX;

    /// <summary>The cross product of two of the six axis directions (Y = Z x X).</summary>
    internal static Direction3D.D CrossStandardDir(Direction3D.D a, Direction3D.D b)
    {
        switch (a)
        {
            case Direction3D.D.X:
                return b switch
                {
                    Direction3D.D.Y => Direction3D.D.Z,
                    Direction3D.D.Z => Direction3D.D.NY,
                    Direction3D.D.NY => Direction3D.D.NZ,
                    Direction3D.D.NZ => Direction3D.D.Y,
                    _ => Direction3D.D.Z,
                };
            case Direction3D.D.Y:
                return b switch
                {
                    Direction3D.D.Z => Direction3D.D.X,
                    Direction3D.D.X => Direction3D.D.NZ,
                    Direction3D.D.NX => Direction3D.D.Z,
                    Direction3D.D.NZ => Direction3D.D.NX,
                    _ => Direction3D.D.X,
                };
            case Direction3D.D.Z:
                return b switch
                {
                    Direction3D.D.X => Direction3D.D.Y,
                    Direction3D.D.Y => Direction3D.D.NX,
                    Direction3D.D.NX => Direction3D.D.NY,
                    Direction3D.D.NY => Direction3D.D.X,
                    _ => Direction3D.D.Y,
                };
            case Direction3D.D.NX:
                return b switch
                {
                    Direction3D.D.Y => Direction3D.D.NZ,
                    Direction3D.D.Z => Direction3D.D.Y,
                    Direction3D.D.NY => Direction3D.D.Z,
                    Direction3D.D.NZ => Direction3D.D.NY,
                    _ => Direction3D.D.NZ,
                };
            case Direction3D.D.NY:
                return b switch
                {
                    Direction3D.D.Z => Direction3D.D.NX,
                    Direction3D.D.X => Direction3D.D.Z,
                    Direction3D.D.NX => Direction3D.D.NZ,
                    Direction3D.D.NZ => Direction3D.D.X,
                    _ => Direction3D.D.NX,
                };
            case Direction3D.D.NZ:
                return b switch
                {
                    Direction3D.D.X => Direction3D.D.NY,
                    Direction3D.D.Y => Direction3D.D.X,
                    Direction3D.D.NX => Direction3D.D.Y,
                    Direction3D.D.NY => Direction3D.D.NX,
                    _ => Direction3D.D.NY,
                };
            default:
                return Direction3D.D.Z;
        }
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Frame3D other) => _axis.Equals(other._axis) && _vxdir.Equals(other._vxdir) && _vydir.Equals(other._vydir);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Frame3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Frame3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_axis, _vxdir, _vydir);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Frame3D left, in Frame3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Frame3D left, in Frame3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"Frame3D({_axis.Location}, {_axis.Direction}, {_vxdir})";
}
