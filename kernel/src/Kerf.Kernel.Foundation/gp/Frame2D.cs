// Port of: src/FoundationClasses/TKMath/gp/gp_Ax22d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax22d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A coordinate system in 2D space: a location and orthonormal X and Y directions. It is right-handed when
/// X ^ Y is positive, left-handed otherwise.
/// </summary>
/// <remarks><c>new gp_Ax22d()</c> is the global system, as in OCCT; <c>default(gp_Ax22d)</c> is not valid.</remarks>
public partial struct Frame2D : IEquatable<Frame2D>
{
    private Point2D _point;
    private Direction2D _vydir;
    private Direction2D _vxdir;

    public Frame2D()
    {
        _point = new Point2D();
        _vydir = new Direction2D(Direction2D.D.Y);
        _vxdir = new Direction2D();
    }

    /// <summary>
    /// A system at <paramref name="p"/> with X direction <paramref name="vx"/>; the Y direction is normal to it,
    /// on the side of <paramref name="vy"/>.
    /// </summary>
    public Frame2D(in Point2D p, in Direction2D vx, in Direction2D vy)
    {
        _point = p;
        _vydir = vy;
        _vxdir = vx;
        double value = vx.Crossed(vy);
        if (value >= 0.0)
        {
            _vydir = _vydir.WithCoord(-_vxdir.Y(), _vxdir.X());
        }
        else
        {
            _vydir = _vydir.WithCoord(_vxdir.Y(), -_vxdir.X());
        }
    }

    /// <summary>A system at <paramref name="p"/> with X direction <paramref name="v"/>, right-handed when <paramref name="isSense"/> is true.</summary>
    public Frame2D(in Point2D p, in Direction2D v, bool isSense = true)
    {
        _point = p;
        _vydir = new Direction2D();
        _vxdir = v;
        if (isSense)
        {
            _vydir = _vydir.WithCoord(-v.Y(), v.X());
        }
        else
        {
            _vydir = _vydir.WithCoord(v.Y(), -v.X());
        }
    }

    /// <summary>A system on the axis <paramref name="a"/> (its X axis), right-handed when <paramref name="isSense"/> is true.</summary>
    public Frame2D(in Axis2D a, bool isSense = true)
    {
        _point = a.Location();
        _vydir = new Direction2D();
        _vxdir = a.Direction();
        if (isSense)
        {
            _vydir = _vydir.WithCoord(-_vxdir.Y(), _vxdir.X());
        }
        else
        {
            _vydir = _vydir.WithCoord(_vxdir.Y(), -_vxdir.X());
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Frame2D a1)
    {
        _point = a1.Location();
        _vxdir = a1.XDirection();
        _vydir = a1.YDirection();
    }

    internal void SetXAxis(in Axis2D a1)
    {
        bool isSign = (_vxdir.Crossed(_vydir)) >= 0.0;
        _point = a1.Location();
        _vxdir = a1.Direction();
        if (isSign)
        {
            _vydir = _vydir.WithCoord(-_vxdir.Y(), _vxdir.X());
        }
        else
        {
            _vydir = _vydir.WithCoord(_vxdir.Y(), -_vxdir.X());
        }
    }

    internal void SetYAxis(in Axis2D a1)
    {
        bool isSign = (_vxdir.Crossed(_vydir)) >= 0.0;
        _point = a1.Location();
        _vydir = a1.Direction();
        if (isSign)
        {
            _vxdir = _vxdir.WithCoord(_vydir.Y(), -_vydir.X());
        }
        else
        {
            _vxdir = _vxdir.WithCoord(-_vydir.Y(), _vydir.X());
        }
    }

    internal void SetLocation(in Point2D p) => _point = p;

    internal void SetXDirection(in Direction2D vx)
    {
        bool isSign = (_vxdir.Crossed(_vydir)) >= 0.0;
        _vxdir = vx;
        if (isSign)
        {
            _vydir = _vydir.WithCoord(-vx.Y(), vx.X());
        }
        else
        {
            _vydir = _vydir.WithCoord(vx.Y(), -vx.X());
        }
    }

    internal void SetYDirection(in Direction2D vy)
    {
        bool isSign = (_vxdir.Crossed(_vydir)) >= 0.0;
        _vydir = vy;
        if (isSign)
        {
            _vxdir = _vxdir.WithCoord(vy.Y(), -vy.X());
        }
        else
        {
            _vxdir = _vxdir.WithCoord(-vy.Y(), vy.X());
        }
    }

    internal void Mirror(in Point2D p)
    {
        Point2D temp = _point;
        temp = temp.Mirrored(p);
        _point = temp;
        _vxdir = _vxdir.Reversed();
        _vydir = _vydir.Reversed();
    }

    internal void Mirror(in Axis2D a1)
    {
        _vydir = _vydir.Mirrored(a1);
        _vxdir = _vxdir.Mirrored(a1);
        Point2D temp = _point;
        temp = temp.Mirrored(a1);
        _point = temp;
    }

    internal void Rotate(in Point2D p, double ang)
    {
        Point2D temp = _point;
        temp = temp.Rotated(p, ang);
        _point = temp;
        _vxdir = _vxdir.Rotated(ang);
        _vydir = _vydir.Rotated(ang);
    }

    internal void Scale(in Point2D p, double s)
    {
        Point2D temp = _point;
        temp = temp.Scaled(p, s);
        _point = temp;
        if (s < 0.0)
        {
            _vxdir = _vxdir.Reversed();
            _vydir = _vydir.Reversed();
        }
    }

    internal void Transform(in Transform2D t)
    {
        Point2D temp = _point;
        temp = temp.Transformed(t);
        _point = temp;
        _vxdir = _vxdir.Transformed(t);
        _vydir = _vydir.Transformed(t);
    }

    internal void Translate(in Vector2D v) => _point.Translate(v);

    internal void Translate(in Point2D p1, in Point2D p2) => _point.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Frame2D WithAxis(in Frame2D a1)
    {
        Frame2D copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Frame2D WithXAxis(in Axis2D a1)
    {
        Frame2D copy = this;
        copy.SetXAxis(a1);
        return copy;
    }

    public readonly Frame2D WithYAxis(in Axis2D a1)
    {
        Frame2D copy = this;
        copy.SetYAxis(a1);
        return copy;
    }

    public readonly Frame2D WithLocation(in Point2D p)
    {
        Frame2D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Frame2D WithXDirection(in Direction2D vx)
    {
        Frame2D copy = this;
        copy.SetXDirection(vx);
        return copy;
    }

    public readonly Frame2D WithYDirection(in Direction2D vy)
    {
        Frame2D copy = this;
        copy.SetYDirection(vy);
        return copy;
    }

    // Queries

    public readonly Axis2D XAxis() => new(_point, _vxdir);

    public readonly Axis2D YAxis() => new(_point, _vydir);

    public readonly Point2D Location() => _point;

    public readonly Direction2D XDirection() => _vxdir;

    public readonly Direction2D YDirection() => _vydir;

    public readonly Frame2D Mirrored(in Point2D p)
    {
        Frame2D temp = this;
        temp.Mirror(p);
        return temp;
    }

    public readonly Frame2D Mirrored(in Axis2D a1)
    {
        Frame2D temp = this;
        temp.Mirror(a1);
        return temp;
    }

    public readonly Frame2D Rotated(in Point2D p, double ang)
    {
        Frame2D temp = this;
        temp.Rotate(p, ang);
        return temp;
    }

    public readonly Frame2D Scaled(in Point2D p, double s)
    {
        Frame2D temp = this;
        temp.Scale(p, s);
        return temp;
    }

    public readonly Frame2D Transformed(in Transform2D t)
    {
        Frame2D temp = this;
        temp.Transform(t);
        return temp;
    }

    public readonly Frame2D Translated(in Vector2D v)
    {
        Frame2D temp = this;
        temp.Translate(v);
        return temp;
    }

    public readonly Frame2D Translated(in Point2D p1, in Point2D p2)
    {
        Frame2D temp = this;
        temp.Translate(p1, p2);
        return temp;
    }

    public readonly bool Equals(Frame2D other) => _point.Equals(other._point) && _vydir.Equals(other._vydir) && _vxdir.Equals(other._vxdir);

    public override readonly bool Equals(object? obj) => obj is Frame2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_point, _vydir, _vxdir);

    public static bool operator ==(in Frame2D left, in Frame2D right) => left.Equals(right);

    public static bool operator !=(in Frame2D left, in Frame2D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Ax22d({_point}, {_vxdir}, {_vydir})";
}
