// Port of: src/FoundationClasses/TKMath/gp/gp_Ax3.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Geometry;

/// <summary>A coordinate system in 3D space, right- or left-handed ("direct" or "indirect").</summary>
/// <remarks><c>new gp_Ax3()</c> is the global system, as in OCCT; <c>default(gp_Ax3)</c> is not valid.</remarks>
public partial struct CoordinateSystem : IEquatable<CoordinateSystem>
{
    private Axis3D _axis;
    private Direction3D _vydir;
    private Direction3D _vxdir;

    public CoordinateSystem()
    {
        _axis = new Axis3D();
        _vydir = new Direction3D(Direction3D.D.Y);
        _vxdir = new Direction3D(Direction3D.D.X);
    }

    public CoordinateSystem(in Frame3D a)
    {
        _axis = a.Axis();
        _vydir = a.YDirection();
        _vxdir = a.XDirection();
    }

    public CoordinateSystem(in Point3D p, in Direction3D n, in Direction3D vx)
    {
        _axis = new Axis3D(p, n);
        _vydir = n;
        _vxdir = n;
        _vxdir = _vxdir.CrossCrossed(vx, n);
        _vydir = _vydir.Crossed(_vxdir);
    }

    public CoordinateSystem(in Point3D p, Direction3D.D n, Direction3D.D vx)
    {
        _axis = new Axis3D(p, n);
        _vydir = new Direction3D(Frame3D.CrossStandardDir(n, vx));
        _vxdir = new Direction3D(vx);
    }

    public CoordinateSystem(in Point3D p, in Direction3D v)
    {
        _axis = new Axis3D(p, v);
        double a = v.X();
        double b = v.Y();
        double c = v.Z();
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
        _vydir = v.Crossed(_vxdir);
    }

    public CoordinateSystem(in Point3D p, Direction3D.D v)
    {
        _axis = new Axis3D(p, v);
        _vydir = new Direction3D(Frame3D.GetPerpendicularYDir(v));
        _vxdir = new Direction3D(Frame3D.GetPerpendicularXDir(v));
    }

    public CoordinateSystem(Direction3D.D v) : this(new Point3D(0.0, 0.0, 0.0), v) { }

    // Mutators (internal: see docs/translation-rules.md)

    internal void XReverse() => _vxdir.Reverse();

    internal void YReverse() => _vydir.Reverse();

    internal void ZReverse() => _axis.Reverse();

    internal void SetAxis(in Axis3D a1)
    {
        _axis = _axis.WithLocation(a1.Location());
        SetDirection(a1.Direction());
    }

    internal void SetDirection(in Direction3D v)
    {
        double dot = v.Dot(_vxdir);
        if (1.0 - Math.Abs(dot) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (dot > 0)
            {
                _vxdir = _vydir;
                _vydir = _axis.Direction();
            }
            else
            {
                _vxdir = _axis.Direction();
            }

            _axis = _axis.WithDirection(v);
        }
        else
        {
            bool direct = Direct();
            _axis = _axis.WithDirection(v);
            _vxdir = v.CrossCrossed(_vxdir, v);
            if (direct)
            {
                _vydir = v.Crossed(_vxdir);
            }
            else
            {
                _vydir = _vxdir.Crossed(v);
            }
        }
    }

    internal void SetLocation(in Point3D p) => _axis.SetLocation(p);

    internal void SetXDirection(in Direction3D vx)
    {
        double dot = vx.Dot(_axis.Direction());
        if (1.0 - Math.Abs(dot) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (dot > 0)
            {
                _axis = _axis.WithDirection(_vxdir);
                _vydir = -_vydir;
            }
            else
            {
                _axis = _axis.WithDirection(_vxdir);
            }

            _vxdir = vx;
        }
        else
        {
            bool direct = Direct();
            _vxdir = _axis.Direction().CrossCrossed(vx, _axis.Direction());
            if (direct)
            {
                _vydir = _axis.Direction().Crossed(_vxdir);
            }
            else
            {
                _vydir = _vxdir.Crossed(_axis.Direction());
            }
        }
    }

    internal void SetYDirection(in Direction3D vy)
    {
        double dot = vy.Dot(_axis.Direction());
        if (1.0 - Math.Abs(dot) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (dot > 0)
            {
                _axis = _axis.WithDirection(_vydir);
                _vxdir = -_vxdir;
            }
            else
            {
                _axis = _axis.WithDirection(_vydir);
            }

            _vydir = vy;
        }
        else
        {
            bool isDirect = Direct();
            _vxdir = vy.Crossed(_axis.Direction());
            _vydir = _axis.Direction().Crossed(_vxdir);
            if (!isDirect)
            {
                _vxdir = _vxdir.Reversed();
            }
        }
    }

    internal void Mirror(in Point3D p)
    {
        _axis = _axis.Mirrored(p);
        _vxdir = _vxdir.Reversed();
        _vydir = _vydir.Reversed();
    }

    internal void Mirror(in Axis3D a1)
    {
        _vydir = _vydir.Mirrored(a1);
        _vxdir = _vxdir.Mirrored(a1);
        _axis = _axis.Mirrored(a1);
    }

    internal void Mirror(in Frame3D a2)
    {
        _vydir = _vydir.Mirrored(a2);
        _vxdir = _vxdir.Mirrored(a2);
        _axis = _axis.Mirrored(a2);
    }

    internal void Rotate(in Axis3D a1, double ang)
    {
        _axis = _axis.Rotated(a1, ang);
        _vxdir = _vxdir.Rotated(a1, ang);
        _vydir = _vydir.Rotated(a1, ang);
    }

    internal void Scale(in Point3D p, double s)
    {
        _axis = _axis.Scaled(p, s);
        if (s < 0.0)
        {
            _vxdir = _vxdir.Reversed();
            _vydir = _vydir.Reversed();
        }
    }

    internal void Transform(in Transform3D t)
    {
        _axis = _axis.Transformed(t);
        _vxdir = _vxdir.Transformed(t);
        _vydir = _vydir.Transformed(t);
    }

    internal void Translate(in Vector3D v) => _axis.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => Translate(new Vector3D(p1, p2));

    // Copies with a mutator applied

    public readonly CoordinateSystem XReversed()
    {
        CoordinateSystem copy = this;
        copy.XReverse();
        return copy;
    }

    public readonly CoordinateSystem YReversed()
    {
        CoordinateSystem copy = this;
        copy.YReverse();
        return copy;
    }

    public readonly CoordinateSystem ZReversed()
    {
        CoordinateSystem copy = this;
        copy.ZReverse();
        return copy;
    }

    public readonly CoordinateSystem WithAxis(in Axis3D a1)
    {
        CoordinateSystem copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly CoordinateSystem WithDirection(in Direction3D v)
    {
        CoordinateSystem copy = this;
        copy.SetDirection(v);
        return copy;
    }

    public readonly CoordinateSystem WithLocation(in Point3D p)
    {
        CoordinateSystem copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly CoordinateSystem WithXDirection(in Direction3D vx)
    {
        CoordinateSystem copy = this;
        copy.SetXDirection(vx);
        return copy;
    }

    public readonly CoordinateSystem WithYDirection(in Direction3D vy)
    {
        CoordinateSystem copy = this;
        copy.SetYDirection(vy);
        return copy;
    }

    // Queries

    public readonly double Angle(in CoordinateSystem other) => _axis.Angle(other._axis);

    public readonly Axis3D Axis() => _axis;

    /// <summary>The right-handed system with the same location, X direction and (for a direct system) main direction.</summary>
    public readonly Frame3D Ax2()
    {
        Direction3D zz = _axis.Direction();
        if (!Direct())
        {
            zz = zz.Reversed();
        }

        return new Frame3D(_axis.Location(), zz, _vxdir);
    }

    public readonly Direction3D Direction() => _axis.Direction();

    public readonly Point3D Location() => _axis.Location();

    public readonly Direction3D XDirection() => _vxdir;

    public readonly Direction3D YDirection() => _vydir;

    /// <summary>True for a right-handed system.</summary>
    public readonly bool Direct() => _vxdir.Crossed(_vydir).Dot(_axis.Direction()) > 0.0;

    public readonly bool IsCoplanar(in CoordinateSystem other, double linearTolerance, double angularTolerance)
    {
        Vector3D vec = new(_axis.Location(), other._axis.Location());
        double d1 = new Vector3D(_axis.Direction()).Dot(vec);
        if (d1 < 0)
        {
            d1 = -d1;
        }

        double d2 = new Vector3D(other._axis.Direction()).Dot(vec);
        if (d2 < 0)
        {
            d2 = -d2;
        }

        return d1 <= linearTolerance && d2 <= linearTolerance && _axis.IsParallel(other._axis, angularTolerance);
    }

    public readonly bool IsCoplanar(in Axis3D a1, double linearTolerance, double angularTolerance)
    {
        Vector3D vec = new(_axis.Location(), a1.Location());
        double d1 = new Vector3D(_axis.Direction()).Dot(vec);
        if (d1 < 0)
        {
            d1 = -d1;
        }

        double d2 = new Vector3D(a1.Direction()).Crossed(vec).Magnitude();
        if (d2 < 0)
        {
            d2 = -d2;
        }

        return d1 <= linearTolerance && d2 <= linearTolerance && _axis.IsNormal(a1, angularTolerance);
    }

    public readonly CoordinateSystem Mirrored(in Point3D p)
    {
        CoordinateSystem temp = this;
        temp.Mirror(p);
        return temp;
    }

    public readonly CoordinateSystem Mirrored(in Axis3D a1)
    {
        CoordinateSystem temp = this;
        temp.Mirror(a1);
        return temp;
    }

    public readonly CoordinateSystem Mirrored(in Frame3D a2)
    {
        CoordinateSystem temp = this;
        temp.Mirror(a2);
        return temp;
    }

    public readonly CoordinateSystem Rotated(in Axis3D a1, double ang)
    {
        CoordinateSystem temp = this;
        temp.Rotate(a1, ang);
        return temp;
    }

    public readonly CoordinateSystem Scaled(in Point3D p, double s)
    {
        CoordinateSystem temp = this;
        temp.Scale(p, s);
        return temp;
    }

    public readonly CoordinateSystem Transformed(in Transform3D t)
    {
        CoordinateSystem temp = this;
        temp.Transform(t);
        return temp;
    }

    public readonly CoordinateSystem Translated(in Vector3D v)
    {
        CoordinateSystem temp = this;
        temp.Translate(v);
        return temp;
    }

    public readonly CoordinateSystem Translated(in Point3D p1, in Point3D p2) => Translated(new Vector3D(p1, p2));

    public readonly bool Equals(CoordinateSystem other) => _axis.Equals(other._axis) && _vxdir.Equals(other._vxdir) && _vydir.Equals(other._vydir);

    public override readonly bool Equals(object? obj) => obj is CoordinateSystem other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_axis, _vxdir, _vydir);

    public static bool operator ==(in CoordinateSystem left, in CoordinateSystem right) => left.Equals(right);

    public static bool operator !=(in CoordinateSystem left, in CoordinateSystem right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Ax3({_axis.Location()}, {_axis.Direction()}, {_vxdir}, {_vydir})";
}
