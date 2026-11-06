// Port of: src/FoundationClasses/TKMath/gp/gp_Ax3.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>A coordinate system in 3D space, right- or left-handed ("direct" or "indirect").</summary>
/// <remarks><c>new gp_Ax3()</c> is the global system, as in OCCT; <c>default(gp_Ax3)</c> is not valid.</remarks>
public readonly partial struct CoordinateSystem : IEquatable<CoordinateSystem>
{
    private readonly Axis3D _axis;
    private readonly Direction3D _vydir;
    private readonly Direction3D _vxdir;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    private CoordinateSystem(in Axis3D axis, in Direction3D vydir, in Direction3D vxdir)
    {
        _axis = axis;
        _vydir = vydir;
        _vxdir = vxdir;
    }

    /// <summary>The absolute system: the origin, with the X, Y and Z axes, right-handed.</summary>
    public CoordinateSystem()
    {
        _axis = new Axis3D();
        _vydir = new Direction3D(Direction3D.D.Y);
        _vxdir = new Direction3D(Direction3D.D.X);
    }

    /// <summary>The same placement as <paramref name="frame"/>, and so right-handed.</summary>
    public CoordinateSystem(in Frame3D frame)
    {
        _axis = frame.Axis;
        _vydir = frame.YDirection;
        _vxdir = frame.XDirection;
    }

    /// <summary>A right-handed system at <paramref name="point"/> with main direction <paramref name="normal"/>; the X direction is <paramref name="xDirection"/> projected normal to it.</summary>
    /// <exception cref="ArgumentException"><paramref name="xDirection"/> is parallel to <paramref name="normal"/>.</exception>
    public CoordinateSystem(in Point3D point, in Direction3D normal, in Direction3D xDirection)
    {
        _axis = new Axis3D(point, normal);
        _vydir = normal;
        _vxdir = normal;
        _vxdir = _vxdir.CrossCrossed(xDirection, normal);
        _vydir = _vydir.Crossed(_vxdir);
    }

    internal CoordinateSystem(in Point3D p, Direction3D.D n, Direction3D.D vx)
    {
        _axis = new Axis3D(p, n);
        _vydir = new Direction3D(Frame3D.CrossStandardDir(n, vx));
        _vxdir = new Direction3D(vx);
    }

    /// <summary>A right-handed system at <paramref name="point"/> with main direction <paramref name="direction"/>; the X direction is chosen for you, normal to it.</summary>
    public CoordinateSystem(in Point3D point, in Direction3D direction)
    {
        _axis = new Axis3D(point, direction);
        double a = direction.X;
        double b = direction.Y;
        double c = direction.Z;
        double aabs = Math.Abs(a);
        double babs = Math.Abs(b);
        double cabs = Math.Abs(c);
        Direction3D d = new();

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

        _vxdir = d;
        _vydir = direction.Crossed(_vxdir);
    }

    internal CoordinateSystem(in Point3D p, Direction3D.D v)
    {
        _axis = new Axis3D(p, v);
        _vydir = new Direction3D(Frame3D.GetPerpendicularYDir(v));
        _vxdir = new Direction3D(Frame3D.GetPerpendicularXDir(v));
    }

    internal CoordinateSystem(Direction3D.D v) : this(new Point3D(0.0, 0.0, 0.0), v) { }

    // Copies

    /// <summary>A copy with the X direction reversed, which flips the handedness.</summary>
    public readonly CoordinateSystem XReversed()
    {
        Direction3D vxdir = _vxdir;
        vxdir = vxdir.Reversed();
        return new CoordinateSystem(_axis, _vydir, vxdir);
    }

    /// <summary>A copy with the Y direction reversed, which flips the handedness.</summary>
    public readonly CoordinateSystem YReversed()
    {
        Direction3D vydir = _vydir;
        vydir = vydir.Reversed();
        return new CoordinateSystem(_axis, vydir, _vxdir);
    }

    /// <summary>A copy with the main direction reversed, which flips the handedness.</summary>
    public readonly CoordinateSystem ZReversed()
    {
        Axis3D axis = _axis;
        axis = axis.Reversed();
        return new CoordinateSystem(axis, _vydir, _vxdir);
    }

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly CoordinateSystem WithAxis(in Axis3D axis)
    {
        return new CoordinateSystem(_axis.WithLocation(axis.Location), _vydir, _vxdir).WithDirection(axis.Direction);
    }

    /// <summary>A copy with this main direction. The old X direction is projected on to the new XY plane; if it is parallel to <paramref name="direction"/>, the old directions swap roles instead.</summary>
    public readonly CoordinateSystem WithDirection(in Direction3D direction)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        double dot = direction.Dot(vxdir);
        if (1.0 - Math.Abs(dot) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (dot > 0)
            {
                vxdir = vydir;
                vydir = axis.Direction;
            }
            else
            {
                vxdir = axis.Direction;
            }

            axis = axis.WithDirection(direction);
        }
        else
        {
            bool direct = IsDirect;
            axis = axis.WithDirection(direction);
            vxdir = direction.CrossCrossed(vxdir, direction);
            if (direct)
            {
                vydir = direction.Crossed(vxdir);
            }
            else
            {
                vydir = vxdir.Crossed(direction);
            }
        }
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    /// <summary>A copy moved so that its origin is <paramref name="point"/>; its directions stay.</summary>
    public readonly CoordinateSystem WithLocation(in Point3D point)
    {
        Axis3D axis = _axis;
        axis = axis.WithLocation(point);
        return new CoordinateSystem(axis, _vydir, _vxdir);
    }

    /// <summary>A copy whose X direction is <paramref name="xDirection"/> projected on to the XY plane; the Y direction is recomputed, keeping the handedness. If <paramref name="xDirection"/> is parallel to the main direction, the main direction changes instead.</summary>
    public readonly CoordinateSystem WithXDirection(in Direction3D xDirection)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        double dot = xDirection.Dot(axis.Direction);
        if (1.0 - Math.Abs(dot) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (dot > 0)
            {
                axis = axis.WithDirection(vxdir);
                vydir = -vydir;
            }
            else
            {
                axis = axis.WithDirection(vxdir);
            }

            vxdir = xDirection;
        }
        else
        {
            bool direct = IsDirect;
            vxdir = axis.Direction.CrossCrossed(xDirection, axis.Direction);
            if (direct)
            {
                vydir = axis.Direction.Crossed(vxdir);
            }
            else
            {
                vydir = vxdir.Crossed(axis.Direction);
            }
        }
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    /// <summary>A copy whose Y direction is <paramref name="yDirection"/> projected on to the XY plane; the X direction is recomputed, keeping the handedness. If <paramref name="yDirection"/> is parallel to the main direction, the main direction changes instead.</summary>
    public readonly CoordinateSystem WithYDirection(in Direction3D yDirection)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        double dot = yDirection.Dot(axis.Direction);
        if (1.0 - Math.Abs(dot) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (dot > 0)
            {
                axis = axis.WithDirection(vydir);
                vxdir = -vxdir;
            }
            else
            {
                axis = axis.WithDirection(vydir);
            }

            vydir = yDirection;
        }
        else
        {
            bool isDirect = IsDirect;
            vxdir = yDirection.Crossed(axis.Direction);
            vydir = axis.Direction.Crossed(vxdir);
            if (!isDirect)
            {
                vxdir = vxdir.Reversed();
            }
        }
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    // Queries

    /// <summary>The angle between the main directions, in [0, PI] radians, with no sign.</summary>
    public readonly double AngleTo(in CoordinateSystem other) => _axis.AngleTo(other._axis);

    /// <summary>The main axis: the location and the main direction.</summary>
    public readonly Axis3D Axis => _axis;

    /// <summary>The right-handed system with the same location, X direction and (for a direct system) main direction.</summary>
    public readonly Frame3D Frame
    {
        get
        {
            Direction3D zz = _axis.Direction;
            if (!IsDirect)
            {
                zz = zz.Reversed();
            }

            return new Frame3D(_axis.Location, zz, _vxdir);
        }
    }

    /// <summary>The main direction, normal to the XY plane.</summary>
    public readonly Direction3D Direction => _axis.Direction;

    /// <summary>The origin.</summary>
    public readonly Point3D Location => _axis.Location;

    /// <summary>The X direction, a unit vector.</summary>
    public readonly Direction3D XDirection => _vxdir;

    /// <summary>The Y direction, a unit vector: normal to the X direction, on the side set by the handedness.</summary>
    public readonly Direction3D YDirection => _vydir;

    /// <summary>True for a right-handed system.</summary>
    public readonly bool IsDirect => _vxdir.Crossed(_vydir).Dot(_axis.Direction) > 0.0;

    /// <summary>True when the main directions are parallel within <paramref name="angularTolerance"/> radians, and each location is within <paramref name="linearTolerance"/> of the other's XY plane.</summary>
    public readonly bool IsCoplanar(in CoordinateSystem other, double linearTolerance, double angularTolerance)
    {
        Vector3D vec = new(_axis.Location, other._axis.Location);
        double d1 = new Vector3D(_axis.Direction).Dot(vec);
        if (d1 < 0)
        {
            d1 = -d1;
        }

        double d2 = new Vector3D(other._axis.Direction).Dot(vec);
        if (d2 < 0)
        {
            d2 = -d2;
        }

        return d1 <= linearTolerance && d2 <= linearTolerance && _axis.IsParallel(other._axis, angularTolerance);
    }

    /// <summary>True when <paramref name="axis"/> is normal to the main direction within <paramref name="angularTolerance"/> radians and this location is within <paramref name="linearTolerance"/> of the axis, not only of its plane.</summary>
    public readonly bool IsCoplanar(in Axis3D axis, double linearTolerance, double angularTolerance)
    {
        Vector3D vec = new(_axis.Location, axis.Location);
        double d1 = new Vector3D(_axis.Direction).Dot(vec);
        if (d1 < 0)
        {
            d1 = -d1;
        }

        double d2 = new Vector3D(axis.Direction).Crossed(vec).Magnitude;
        if (d2 < 0)
        {
            d2 = -d2;
        }

        return d1 <= linearTolerance && d2 <= linearTolerance && _axis.IsNormal(axis, angularTolerance);
    }

    /// <summary>The central symmetry through <paramref name="point"/>. All three directions reverse, so the handedness flips.</summary>
    public readonly CoordinateSystem Mirrored(in Point3D point)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        axis = axis.Mirrored(point);
        vxdir = vxdir.Reversed();
        vydir = vydir.Reversed();
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    /// <summary>The half-turn about <paramref name="axis"/>, which keeps the handedness.</summary>
    public readonly CoordinateSystem Mirrored(in Axis3D axis)
    {
        Axis3D newAxis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        vydir = vydir.Mirrored(axis);
        vxdir = vxdir.Mirrored(axis);
        newAxis = newAxis.Mirrored(axis);
        return new CoordinateSystem(newAxis, vydir, vxdir);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>, which flips the handedness.</summary>
    public readonly CoordinateSystem Mirrored(in Frame3D frame)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        vydir = vydir.Mirrored(frame);
        vxdir = vxdir.Mirrored(frame);
        axis = axis.Mirrored(frame);
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly CoordinateSystem Rotated(in Axis3D axis, double angle)
    {
        Axis3D newAxis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        newAxis = newAxis.Rotated(axis, angle);
        vxdir = vxdir.Rotated(axis, angle);
        vydir = vydir.Rotated(axis, angle);
        return new CoordinateSystem(newAxis, vydir, vxdir);
    }

    /// <summary>The location scaled about <paramref name="point"/> by <paramref name="factor"/>. A negative factor reverses all three directions, which flips the handedness.</summary>
    public readonly CoordinateSystem Scaled(in Point3D point, double factor)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        axis = axis.Scaled(point, factor);
        if (factor < 0.0)
        {
            vxdir = vxdir.Reversed();
            vydir = vydir.Reversed();
        }
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    /// <summary>The image under <paramref name="transform"/>, each direction transformed on its own, so a mirror flips the handedness.</summary>
    public readonly CoordinateSystem Transformed(in Transform3D transform)
    {
        Axis3D axis = _axis;
        Direction3D vydir = _vydir;
        Direction3D vxdir = _vxdir;
        axis = axis.Transformed(transform);
        vxdir = vxdir.Transformed(transform);
        vydir = vydir.Transformed(transform);
        return new CoordinateSystem(axis, vydir, vxdir);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly CoordinateSystem Translated(in Vector3D vector)
    {
        Axis3D axis = _axis;
        axis = axis.Translated(vector);
        return new CoordinateSystem(axis, _vydir, _vxdir);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly CoordinateSystem Translated(in Point3D point1, in Point3D point2) => Translated(new Vector3D(point1, point2));

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(CoordinateSystem other) => _axis.Equals(other._axis) && _vxdir.Equals(other._vxdir) && _vydir.Equals(other._vydir);

    /// <summary>True when <paramref name="obj"/> is a <see cref="CoordinateSystem"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is CoordinateSystem other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_axis, _vxdir, _vydir);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in CoordinateSystem left, in CoordinateSystem right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in CoordinateSystem left, in CoordinateSystem right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => $"CoordinateSystem({_axis.Location}, {_axis.Direction}, {_vxdir}, {_vydir})";
}
