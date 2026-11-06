// Port of: src/FoundationClasses/TKMath/gp/gp_Ax22d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax22d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// A coordinate system in 2D space: a location and orthonormal X and Y directions. It is right-handed when
/// X ^ Y is positive, left-handed otherwise.
/// </summary>
/// <remarks><c>new gp_Ax22d()</c> is the global system, as in OCCT; <c>default(gp_Ax22d)</c> is not valid.</remarks>
public readonly partial struct Frame2D : IEquatable<Frame2D>
{
    private readonly Point2D _point;
    private readonly Direction2D _vydir;
    private readonly Direction2D _vxdir;

    /// <summary>A value of the fields as they are (unit 10G: the returning forms build their results with it).</summary>
    private Frame2D(in Direction2D vydir, in Direction2D vxdir, in Point2D point)
    {
        _point = point;
        _vydir = vydir;
        _vxdir = vxdir;
    }

    public Frame2D()
    {
        _point = new Point2D();
        _vydir = new Direction2D(Direction2D.D.Y);
        _vxdir = new Direction2D();
    }

    /// <summary>
    /// A system at <paramref name="point"/> with X direction <paramref name="xDirection"/>; the Y direction is normal to it,
    /// on the side of <paramref name="yDirection"/>.
    /// </summary>
    public Frame2D(in Point2D point, in Direction2D xDirection, in Direction2D yDirection)
    {
        _point = point;
        _vydir = yDirection;
        _vxdir = xDirection;
        double value = xDirection.Crossed(yDirection);
        if (value >= 0.0)
        {
            _vydir = _vydir.WithCoord(-_vxdir.Y, _vxdir.X);
        }
        else
        {
            _vydir = _vydir.WithCoord(_vxdir.Y, -_vxdir.X);
        }
    }

    /// <summary>A system at <paramref name="point"/> with X direction <paramref name="direction"/>, right-handed when <paramref name="sense"/> is true.</summary>
    public Frame2D(in Point2D point, in Direction2D direction, bool sense = true)
    {
        _point = point;
        _vydir = new Direction2D();
        _vxdir = direction;
        if (sense)
        {
            _vydir = _vydir.WithCoord(-direction.Y, direction.X);
        }
        else
        {
            _vydir = _vydir.WithCoord(direction.Y, -direction.X);
        }
    }

    /// <summary>A system on the axis <paramref name="axis"/> (its X axis), right-handed when <paramref name="sense"/> is true.</summary>
    public Frame2D(in Axis2D axis, bool sense = true)
    {
        _point = axis.Location;
        _vydir = new Direction2D();
        _vxdir = axis.Direction;
        if (sense)
        {
            _vydir = _vydir.WithCoord(-_vxdir.Y, _vxdir.X);
        }
        else
        {
            _vydir = _vydir.WithCoord(_vxdir.Y, -_vxdir.X);
        }
    }

    // Copies

    public readonly Frame2D WithAxis(in Frame2D frame)
    {
        Point2D point = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        point = frame.Location;
        vxdir = frame.XDirection;
        vydir = frame.YDirection;
        return new Frame2D(vydir, vxdir, point);
    }

    public readonly Frame2D WithXAxis(in Axis2D axis)
    {
        Point2D point = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        point = axis.Location;
        vxdir = axis.Direction;
        if (isSign)
        {
            vydir = vydir.WithCoord(-vxdir.Y, vxdir.X);
        }
        else
        {
            vydir = vydir.WithCoord(vxdir.Y, -vxdir.X);
        }
        return new Frame2D(vydir, vxdir, point);
    }

    public readonly Frame2D WithYAxis(in Axis2D axis)
    {
        Point2D point = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        point = axis.Location;
        vydir = axis.Direction;
        if (isSign)
        {
            vxdir = vxdir.WithCoord(vydir.Y, -vydir.X);
        }
        else
        {
            vxdir = vxdir.WithCoord(-vydir.Y, vydir.X);
        }
        return new Frame2D(vydir, vxdir, point);
    }

    public readonly Frame2D WithLocation(in Point2D point)
    {
        Point2D newPoint = _point;
        newPoint = point;
        return new Frame2D(_vydir, _vxdir, newPoint);
    }

    public readonly Frame2D WithXDirection(in Direction2D xDirection)
    {
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        vxdir = xDirection;
        if (isSign)
        {
            vydir = vydir.WithCoord(-xDirection.Y, xDirection.X);
        }
        else
        {
            vydir = vydir.WithCoord(xDirection.Y, -xDirection.X);
        }
        return new Frame2D(vydir, vxdir, _point);
    }

    public readonly Frame2D WithYDirection(in Direction2D yDirection)
    {
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        bool isSign = (vxdir.Crossed(vydir)) >= 0.0;
        vydir = yDirection;
        if (isSign)
        {
            vxdir = vxdir.WithCoord(yDirection.Y, -yDirection.X);
        }
        else
        {
            vxdir = vxdir.WithCoord(-yDirection.Y, yDirection.X);
        }
        return new Frame2D(vydir, vxdir, _point);
    }

    // Queries

    public readonly Axis2D XAxis => new(_point, _vxdir);

    public readonly Axis2D YAxis => new(_point, _vydir);

    public readonly Point2D Location => _point;

    public readonly Direction2D XDirection => _vxdir;

    public readonly Direction2D YDirection => _vydir;

    public readonly Frame2D Mirrored(in Point2D point)
    {
        Point2D newPoint = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        Point2D temp = newPoint;
        temp = temp.Mirrored(point);
        newPoint = temp;
        vxdir = vxdir.Reversed();
        vydir = vydir.Reversed();
        return new Frame2D(vydir, vxdir, newPoint);
    }

    public readonly Frame2D Mirrored(in Axis2D axis)
    {
        Point2D point = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        vydir = vydir.Mirrored(axis);
        vxdir = vxdir.Mirrored(axis);
        Point2D temp = point;
        temp = temp.Mirrored(axis);
        point = temp;
        return new Frame2D(vydir, vxdir, point);
    }

    public readonly Frame2D Rotated(in Point2D point, double angle)
    {
        Point2D newPoint = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        Point2D temp = newPoint;
        temp = temp.Rotated(point, angle);
        newPoint = temp;
        vxdir = vxdir.Rotated(angle);
        vydir = vydir.Rotated(angle);
        return new Frame2D(vydir, vxdir, newPoint);
    }

    public readonly Frame2D Scaled(in Point2D point, double factor)
    {
        Point2D newPoint = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        Point2D temp = newPoint;
        temp = temp.Scaled(point, factor);
        newPoint = temp;
        if (factor < 0.0)
        {
            vxdir = vxdir.Reversed();
            vydir = vydir.Reversed();
        }
        return new Frame2D(vydir, vxdir, newPoint);
    }

    public readonly Frame2D Transformed(in Transform2D transform)
    {
        Point2D point = _point;
        Direction2D vydir = _vydir;
        Direction2D vxdir = _vxdir;
        Point2D temp = point;
        temp = temp.Transformed(transform);
        point = temp;
        vxdir = vxdir.Transformed(transform);
        vydir = vydir.Transformed(transform);
        return new Frame2D(vydir, vxdir, point);
    }

    public readonly Frame2D Translated(in Vector2D vector)
    {
        Point2D point = _point;
        point = point.Translated(vector);
        return new Frame2D(_vydir, _vxdir, point);
    }

    public readonly Frame2D Translated(in Point2D point1, in Point2D point2)
    {
        Point2D point = _point;
        point = point.Translated(point1, point2);
        return new Frame2D(_vydir, _vxdir, point);
    }

    public readonly bool Equals(Frame2D other) => _point.Equals(other._point) && _vydir.Equals(other._vydir) && _vxdir.Equals(other._vxdir);

    public override readonly bool Equals(object? obj) => obj is Frame2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_point, _vydir, _vxdir);

    public static bool operator ==(in Frame2D left, in Frame2D right) => left.Equals(right);

    public static bool operator !=(in Frame2D left, in Frame2D right) => !left.Equals(right);

    public override readonly string ToString() => $"Frame2D({_point}, {_vxdir}, {_vydir})";
}
