// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Geometry;

/// <summary>A right-handed coordinate system in 3D space: a location, a main direction, and X and Y directions.</summary>
/// <remarks><c>new gp_Ax2()</c> is the global system, as in OCCT; <c>default(gp_Ax2)</c> is not valid.</remarks>
public partial struct Frame3D : IEquatable<Frame3D>
{
    private Axis3D _axis;
    private Direction3D _vydir;
    private Direction3D _vxdir;

    public Frame3D()
    {
        _axis = new Axis3D();
        _vydir = new Direction3D(Direction3D.D.Y);
        _vxdir = new Direction3D(Direction3D.D.X);
    }

    /// <summary>A system at <paramref name="p"/> with main direction <paramref name="n"/>, and X direction from <paramref name="vx"/> projected onto the plane normal to N.</summary>
    public Frame3D(in Point3D p, in Direction3D n, in Direction3D vx)
    {
        _axis = new Axis3D(p, n);
        _vydir = n;
        _vxdir = n;
        _vxdir = _vxdir.CrossCrossed(vx, n);
        _vydir = _vydir.Crossed(_vxdir);
    }

    public Frame3D(in Point3D p, Direction3D.D n, Direction3D.D vx)
    {
        _axis = new Axis3D(p, n);
        _vydir = new Direction3D(CrossStandardDir(n, vx));
        _vxdir = new Direction3D(vx);
    }

    /// <summary>A system at <paramref name="p"/> with main direction <paramref name="v"/>, and X and Y directions chosen by OCCT's rule.</summary>
    public Frame3D(in Point3D p, in Direction3D v)
    {
        _axis = new Axis3D(p, v);
        _vydir = new Direction3D();
        _vxdir = new Direction3D();
        double a = v.X();
        double b = v.Y();
        double c = v.Z();
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

        SetXDirection(d);
    }

    public Frame3D(in Point3D p, Direction3D.D v)
    {
        _axis = new Axis3D(p, v);
        _vydir = new Direction3D(GetPerpendicularYDir(v));
        _vxdir = new Direction3D(GetPerpendicularXDir(v));
    }

    public Frame3D(Direction3D.D v) : this(new Point3D(0.0, 0.0, 0.0), v) { }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in Axis3D a1)
    {
        double a = a1.Direction() * _vxdir;
        if (Math.Abs(Math.Abs(a) - 1.0) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (a > 0.0)
            {
                _vxdir = _vydir;
                _vydir = _axis.Direction();
                _axis = a1;
            }
            else
            {
                _vxdir = _axis.Direction();
                _axis = a1;
            }
        }
        else
        {
            _axis = a1;
            _vxdir = _axis.Direction().CrossCrossed(_vxdir, _axis.Direction());
            _vydir = _axis.Direction().Crossed(_vxdir);
        }
    }

    internal void SetDirection(in Direction3D v)
    {
        double a = v * _vxdir;
        if (Math.Abs(Math.Abs(a) - 1.0) <= global::Kerf.Engine.Foundation.Precision.Angular())
        {
            if (a > 0.0)
            {
                _vxdir = _vydir;
                _vydir = _axis.Direction();
                _axis = _axis.WithDirection(v);
            }
            else
            {
                _vxdir = _axis.Direction();
                _axis = _axis.WithDirection(v);
            }
        }
        else
        {
            _axis = _axis.WithDirection(v);
            _vxdir = v.CrossCrossed(_vxdir, v);
            _vydir = v.Crossed(_vxdir);
        }
    }

    internal void SetLocation(in Point3D p) => _axis.SetLocation(p);

    internal void SetXDirection(in Direction3D vx)
    {
        _vxdir = _axis.Direction().CrossCrossed(vx, _axis.Direction());
        _vydir = _axis.Direction().Crossed(_vxdir);
    }

    internal void SetYDirection(in Direction3D vy)
    {
        _vxdir = vy.Crossed(_axis.Direction());
        _vydir = _axis.Direction().Crossed(_vxdir);
    }

    internal void Mirror(in Point3D p)
    {
        Point3D temp = _axis.Location();
        temp = temp.Mirrored(p);
        _axis = _axis.WithLocation(temp);
        _vxdir = _vxdir.Reversed();
        _vydir = _vydir.Reversed();
    }

    internal void Mirror(in Axis3D a1)
    {
        _vydir = _vydir.Mirrored(a1);
        _vxdir = _vxdir.Mirrored(a1);
        Point3D temp = _axis.Location();
        temp = temp.Mirrored(a1);
        _axis = _axis.WithLocation(temp);
        _axis = _axis.WithDirection(_vxdir.Crossed(_vydir));
    }

    internal void Mirror(in Frame3D a2)
    {
        _vydir = _vydir.Mirrored(a2);
        _vxdir = _vxdir.Mirrored(a2);
        Point3D temp = _axis.Location();
        temp = temp.Mirrored(a2);
        _axis = _axis.WithLocation(temp);
        _axis = _axis.WithDirection(_vxdir.Crossed(_vydir));
    }

    internal void Rotate(in Axis3D a1, double ang)
    {
        Point3D temp = _axis.Location();
        temp = temp.Rotated(a1, ang);
        _axis = _axis.WithLocation(temp);
        _vxdir = _vxdir.Rotated(a1, ang);
        _vydir = _vydir.Rotated(a1, ang);
        _axis = _axis.WithDirection(_vxdir.Crossed(_vydir));
    }

    internal void Scale(in Point3D p, double s)
    {
        Point3D temp = _axis.Location();
        temp = temp.Scaled(p, s);
        _axis = _axis.WithLocation(temp);
        if (s < 0.0)
        {
            _vxdir = _vxdir.Reversed();
            _vydir = _vydir.Reversed();
        }
    }

    internal void Transform(in Transform3D t)
    {
        Point3D temp = _axis.Location();
        temp = temp.Transformed(t);
        _axis = _axis.WithLocation(temp);
        _vxdir = _vxdir.Transformed(t);
        _vydir = _vydir.Transformed(t);
        _axis = _axis.WithDirection(_vxdir.Crossed(_vydir));
    }

    internal void Translate(in Vector3D v) => _axis.Translate(v);

    internal void Translate(in Point3D p1, in Point3D p2) => _axis.Translate(p1, p2);

    // Copies with a mutator applied

    public readonly Frame3D WithAxis(in Axis3D a1)
    {
        Frame3D copy = this;
        copy.SetAxis(a1);
        return copy;
    }

    public readonly Frame3D WithDirection(in Direction3D v)
    {
        Frame3D copy = this;
        copy.SetDirection(v);
        return copy;
    }

    public readonly Frame3D WithLocation(in Point3D p)
    {
        Frame3D copy = this;
        copy.SetLocation(p);
        return copy;
    }

    public readonly Frame3D WithXDirection(in Direction3D vx)
    {
        Frame3D copy = this;
        copy.SetXDirection(vx);
        return copy;
    }

    public readonly Frame3D WithYDirection(in Direction3D vy)
    {
        Frame3D copy = this;
        copy.SetYDirection(vy);
        return copy;
    }

    // Queries

    public readonly double Angle(in Frame3D other) => _axis.Angle(other._axis);

    public readonly Axis3D Axis() => _axis;

    public readonly Direction3D Direction() => _axis.Direction();

    public readonly Point3D Location() => _axis.Location();

    public readonly Direction3D XDirection() => _vxdir;

    public readonly Direction3D YDirection() => _vydir;

    public readonly bool IsCoplanar(in Frame3D other, double linearTolerance, double angularTolerance)
    {
        Direction3D dd = _axis.Direction();
        Point3D pp = _axis.Location();
        Point3D op = other._axis.Location();
        double d1 = dd.X() * (op.X() - pp.X()) + dd.Y() * (op.Y() - pp.Y()) + dd.Z() * (op.Z() - pp.Z());
        if (d1 < 0)
        {
            d1 = -d1;
        }

        return d1 <= linearTolerance && _axis.IsParallel(other._axis, angularTolerance);
    }

    public readonly bool IsCoplanar(in Axis3D a, double linearTolerance, double angularTolerance)
    {
        Direction3D dd = _axis.Direction();
        Point3D pp = _axis.Location();
        Point3D ap = a.Location();
        double d1 = dd.X() * (ap.X() - pp.X()) + dd.Y() * (ap.Y() - pp.Y()) + dd.Z() * (ap.Z() - pp.Z());
        if (d1 < 0)
        {
            d1 = -d1;
        }

        return d1 <= linearTolerance && _axis.IsNormal(a, angularTolerance);
    }

    public readonly Frame3D Mirrored(in Point3D p)
    {
        Frame3D temp = this;
        temp.Mirror(p);
        return temp;
    }

    public readonly Frame3D Mirrored(in Axis3D a1)
    {
        Frame3D temp = this;
        temp.Mirror(a1);
        return temp;
    }

    public readonly Frame3D Mirrored(in Frame3D a2)
    {
        Frame3D temp = this;
        temp.Mirror(a2);
        return temp;
    }

    public readonly Frame3D Rotated(in Axis3D a1, double ang)
    {
        Frame3D temp = this;
        temp.Rotate(a1, ang);
        return temp;
    }

    public readonly Frame3D Scaled(in Point3D p, double s)
    {
        Frame3D temp = this;
        temp.Scale(p, s);
        return temp;
    }

    public readonly Frame3D Transformed(in Transform3D t)
    {
        Frame3D temp = this;
        temp.Transform(t);
        return temp;
    }

    public readonly Frame3D Translated(in Vector3D v)
    {
        Frame3D temp = this;
        temp.Translate(v);
        return temp;
    }

    public readonly Frame3D Translated(in Point3D p1, in Point3D p2)
    {
        Frame3D temp = this;
        temp.Translate(p1, p2);
        return temp;
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

    public readonly bool Equals(Frame3D other) => _axis.Equals(other._axis) && _vxdir.Equals(other._vxdir) && _vydir.Equals(other._vydir);

    public override readonly bool Equals(object? obj) => obj is Frame3D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_axis, _vxdir, _vydir);

    public static bool operator ==(in Frame3D left, in Frame3D right) => left.Equals(right);

    public static bool operator !=(in Frame3D left, in Frame3D right) => !left.Equals(right);

    public override readonly string ToString() => $"gp_Ax2({_axis.Location()}, {_axis.Direction()}, {_vxdir})";
}
