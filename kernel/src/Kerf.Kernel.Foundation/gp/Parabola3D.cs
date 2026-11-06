// Port of: src/FoundationClasses/TKMath/gp/gp_Parab.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Parab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Geometry;

/// <summary>
/// A parabola in 3D space, defined by its focal length and positioned by a right-handed coordinate system: the
/// origin is the apex, the X direction the axis of symmetry and the main direction the normal to its plane. It is
/// parameterised by <c>P(U) = O + U^2 / (4 * F) * XDir + U * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Parab()</c> is OCCT's indefinite parabola (focal length <see cref="Standard.RealLast"/>);
/// <c>default(gp_Parab)</c> is not valid.
/// </remarks>
public readonly partial struct Parabola3D : IEquatable<Parabola3D>
{
    private readonly Frame3D _pos;
    private readonly double _focalLength;

    /// <summary>An indefinite parabola with a focal length of <c>double.MaxValue</c>, not a useful value until given one.</summary>
    public Parabola3D()
    {
        _pos = new Frame3D();
        _focalLength = RealLast();
    }

    /// <exception cref="ConstructionErrorException"><paramref name="focal"/> is negative.</exception>
    public Parabola3D(in Frame3D frame, double focal)
    {
        _pos = frame;
        _focalLength = focal;
        Refusal.ArgumentIf(focal < 0.0, FailureKind.ConstructionError, "gp_Parab() - focal length should be >= 0");
    }

    /// <summary>The parabola with directrix <paramref name="directrix"/> and focus <paramref name="focus"/>.</summary>
    /// <exception cref="ConstructionErrorException"><paramref name="focus"/> is on <paramref name="directrix"/>.</exception>
    public Parabola3D(in Axis3D directrix, in Point3D focus)
    {
        Line3D droite = new(directrix);
        _focalLength = droite.DistanceTo(focus) / 2.0;
        Axis3D ax = droite.Normal(focus).Position;
        Axis3D ay = droite.Position;
        Direction3D dd = ax.Direction;
        _pos = new Frame3D(new Point3D(focus.X - _focalLength * dd.X,
                                    focus.Y - _focalLength * dd.Y,
                                    focus.Z - _focalLength * dd.Z),
                         ax.Direction.Crossed(ay.Direction),
                         ax.Direction);
    }

    // Copies

    /// <summary>A copy whose main axis is <paramref name="axis"/>. The old X direction is projected on to the new XY plane; if it is parallel to the axis, the old directions swap roles instead.</summary>
    public readonly Parabola3D WithAxis(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.WithAxis(axis);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>A copy with this focal length.</summary>
    /// <exception cref="ArgumentException"><paramref name="focal"/> is negative.</exception>
    public readonly Parabola3D WithFocal(double focal)
    {
        double focalLength = _focalLength;
        Refusal.ArgumentIf(focal < 0.0, FailureKind.ConstructionError, "gp_Parab::SetFocal() - focal length should be >= 0");
        focalLength = focal;
        return new Parabola3D(_pos, focalLength);
    }

    /// <summary>A copy moved so that its apex is <paramref name="point"/>; its directions stay.</summary>
    public readonly Parabola3D WithLocation(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.WithLocation(point);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>A copy positioned by <paramref name="frame"/>, keeping its size.</summary>
    public readonly Parabola3D WithPosition(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = frame;
        return new Parabola3D(pos, _focalLength);
    }

    // Queries

    /// <summary>The main axis: the normal to the parabola's plane through its apex.</summary>
    public readonly Axis3D Axis => _pos.Axis;

    /// <summary>The directrix: parallel to the Y axis, at the focal length from the apex on the negative X side.</summary>
    public readonly Axis3D Directrix
    {
        get
        {
            Point3D pp = _pos.Location;
            Direction3D dd = _pos.XDirection;
            Point3D p = new(pp.X - _focalLength * dd.X,
                            pp.Y - _focalLength * dd.Y,
                            pp.Z - _focalLength * dd.Z);
            return new Axis3D(p, _pos.YDirection);
        }
    }

    /// <summary>The distance between the apex and the focus.</summary>
    public readonly double Focal => _focalLength;

    /// <summary>The focus, at the focal length from the apex along the X axis.</summary>
    public readonly Point3D Focus
    {
        get
        {
            Point3D pp = _pos.Location;
            Direction3D dd = _pos.XDirection;
            return new Point3D(pp.X + _focalLength * dd.X,
                              pp.Y + _focalLength * dd.Y,
                              pp.Z + _focalLength * dd.Z);
        }
    }

    /// <summary>The apex.</summary>
    public readonly Point3D Location => _pos.Location;

    /// <summary>The distance between the focus and the directrix: twice the focal length.</summary>
    public readonly double Parameter => 2.0 * _focalLength;

    /// <summary>The frame the curve lies in: its main direction is the plane's normal, and its X direction is parameter 0's.</summary>
    public readonly Frame3D Position => _pos;

    /// <summary>The axis of symmetry.</summary>
    public readonly Axis3D XAxis => new(_pos.Location, _pos.XDirection);

    /// <summary>The axis through the apex parallel to the directrix.</summary>
    public readonly Axis3D YAxis => new(_pos.Location, _pos.YDirection);

    /// <summary>The central symmetry through <paramref name="point"/>.</summary>
    public readonly Parabola3D Mirrored(in Point3D point)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(point);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>The half-turn about <paramref name="axis"/>.</summary>
    public readonly Parabola3D Mirrored(in Axis3D axis)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(axis);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>The reflection in the XY plane of <paramref name="frame"/>.</summary>
    public readonly Parabola3D Mirrored(in Frame3D frame)
    {
        Frame3D pos = _pos;
        pos = pos.Mirrored(frame);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>Rotated about <paramref name="axis"/> by <paramref name="angle"/> radians, anticlockwise looking against the axis's direction.</summary>
    public readonly Parabola3D Rotated(in Axis3D axis, double angle)
    {
        Frame3D pos = _pos;
        pos = pos.Rotated(axis, angle);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>Scaled about <paramref name="point"/> by <paramref name="factor"/>: the focal length is multiplied by |factor|.</summary>
    public readonly Parabola3D Scaled(in Point3D point, double factor)
    {
        Frame3D pos = _pos;
        double focalLength = _focalLength;
        focalLength *= factor;
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos = pos.Scaled(point, factor);
        return new Parabola3D(pos, focalLength);
    }

    /// <summary>The image under <paramref name="transform"/>: the focal length is multiplied by |ScaleFactor|.</summary>
    public readonly Parabola3D Transformed(in Transform3D transform)
    {
        Frame3D pos = _pos;
        double focalLength = _focalLength;
        focalLength *= transform.ScaleFactor;
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos = pos.Transformed(transform);
        return new Parabola3D(pos, focalLength);
    }

    /// <summary>Moved by <paramref name="vector"/>.</summary>
    public readonly Parabola3D Translated(in Vector3D vector)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(vector);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>Moved by the vector from <paramref name="point1"/> to <paramref name="point2"/>.</summary>
    public readonly Parabola3D Translated(in Point3D point1, in Point3D point2)
    {
        Frame3D pos = _pos;
        pos = pos.Translated(point1, point2);
        return new Parabola3D(pos, _focalLength);
    }

    /// <summary>True when every component is exactly equal: no tolerance is applied.</summary>
    public readonly bool Equals(Parabola3D other) => _pos.Equals(other._pos) && _focalLength.Equals(other._focalLength);

    /// <summary>True when <paramref name="obj"/> is a <see cref="Parabola3D"/> exactly equal to this one.</summary>
    public override readonly bool Equals(object? obj) => obj is Parabola3D other && Equals(other);

    /// <summary>A hash consistent with exact equality, so values that differ only by rounding hash differently.</summary>
    public override readonly int GetHashCode() => HashCode.Combine(_pos, _focalLength);

    /// <summary>Exact equality: no tolerance is applied.</summary>
    public static bool operator ==(in Parabola3D left, in Parabola3D right) => left.Equals(right);

    /// <summary>Exact inequality: no tolerance is applied.</summary>
    public static bool operator !=(in Parabola3D left, in Parabola3D right) => !left.Equals(right);

    /// <summary>A readable form for logs and the debugger, not a format to parse.</summary>
    public override readonly string ToString() => FormattableString.Invariant($"Parabola3D({_pos}, {_focalLength:R})");
}
