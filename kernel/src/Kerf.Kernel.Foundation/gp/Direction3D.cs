// Port of: src/FoundationClasses/TKMath/gp/gp_Dir.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Dir.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A unit vector in 3D space.</summary>
/// <remarks><c>new gp_Dir()</c> is (1, 0, 0), as in OCCT; <c>default(gp_Dir)</c> is all zeros and not a valid direction.</remarks>
public partial struct Direction3D : IEquatable<Direction3D>
{
    /// <summary>The six directions along the axes of the global system.</summary>
    public enum D
    {
        /// <summary>(1, 0, 0)</summary>
        X,
        /// <summary>(0, 1, 0)</summary>
        Y,
        /// <summary>(0, 0, 1)</summary>
        Z,
        /// <summary>(-1, 0, 0)</summary>
        NX,
        /// <summary>(0, -1, 0)</summary>
        NY,
        /// <summary>(0, 0, -1)</summary>
        NZ,
    }

    private Xyz _coord;

    public Direction3D() => _coord = new Xyz(1.0, 0.0, 0.0);

    public Direction3D(D dir)
        => _coord = new Xyz(dir == D.X ? 1.0 : dir == D.NX ? -1.0 : 0.0,
                              dir == D.Y ? 1.0 : dir == D.NY ? -1.0 : 0.0,
                              dir == D.Z ? 1.0 : dir == D.NZ ? -1.0 : 0.0);

    public Direction3D(in Vector3D v) : this(v.XYZ()) { }

    public Direction3D(in Xyz xyz) : this(xyz.X(), xyz.Y(), xyz.Z()) { }

    public Direction3D(double xv, double yv, double zv)
    {
        _coord = default;
        SetCoord(xv, yv, zv);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int index, double xi)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3, "gp_Dir::SetCoord() - index is out of range [1, 3]");
        if (index == 1)
        {
            SetX(xi);
        }
        else if (index == 2)
        {
            SetY(xi);
        }
        else
        {
            SetZ(xi);
        }
    }

    internal void SetCoord(double xv, double yv, double zv)
    {
        double sqMod = xv * xv + yv * yv + zv * zv;
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            _coord.SetCoord(xv, yv, zv);
            return;
        }

        Standard_ConstructionError_Raise_if(sqMod <= Constants.Resolution() * Constants.Resolution(), "gp_Dir::SetCoord() - input vector has zero norm");
        double d = Math.Sqrt(sqMod);
        _coord.SetCoord(xv / d, yv / d, zv / d);
    }

    internal void SetX(double x) => SetCoord(x, _coord.Y(), _coord.Z());

    internal void SetY(double y) => SetCoord(_coord.X(), y, _coord.Z());

    internal void SetZ(double z) => SetCoord(_coord.X(), _coord.Y(), z);

    internal void SetXYZ(in Xyz xyz) => SetCoord(xyz.X(), xyz.Y(), xyz.Z());

    internal void Cross(in Direction3D right)
    {
        _coord.Cross(right._coord);
        double sqMod = _coord.SquareModulus();
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            return;
        }

        Standard_ConstructionError_Raise_if(sqMod <= Constants.Resolution() * Constants.Resolution(), "gp_Dir::Cross() - result vector has zero norm");
        _coord.Divide(Math.Sqrt(sqMod));
    }

    internal void CrossCross(in Direction3D v1, in Direction3D v2)
    {
        _coord.CrossCross(v1._coord, v2._coord);
        double sqMod = _coord.SquareModulus();
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            return;
        }

        Standard_ConstructionError_Raise_if(sqMod <= Constants.Resolution() * Constants.Resolution(), "gp_Dir::CrossCross() - result vector has zero norm");
        _coord.Divide(Math.Sqrt(sqMod));
    }

    internal void Reverse() => _coord.Reverse();

    internal void Mirror(in Direction3D v)
    {
        Xyz xyz = v._coord;
        double a = xyz.X();
        double b = xyz.Y();
        double c = xyz.Z();
        double x = _coord.X();
        double y = _coord.Y();
        double z = _coord.Z();
        double m1 = 2.0 * a * b;
        double m2 = 2.0 * a * c;
        double m3 = 2.0 * b * c;
        double xx = ((2.0 * a * a) - 1.0) * x + m1 * y + m2 * z;
        double yy = m1 * x + ((2.0 * b * b) - 1.0) * y + m3 * z;
        double zz = m2 * x + m3 * y + ((2.0 * c * c) - 1.0) * z;
        _coord.SetCoord(xx, yy, zz);
    }

    internal void Mirror(in Axis3D a1)
    {
        Xyz xyz = a1.Direction()._coord;
        double a = xyz.X();
        double b = xyz.Y();
        // OCCT takes C from Y here, not Z (docs/occt-issues.md); kept so results match OCCT
        double c = xyz.Y();
        double x = _coord.X();
        double y = _coord.Y();
        double z = _coord.Z();
        double m1 = 2.0 * a * b;
        double m2 = 2.0 * a * c;
        double m3 = 2.0 * b * c;
        double xx = ((2.0 * a * a) - 1.0) * x + m1 * y + m2 * z;
        double yy = m1 * x + ((2.0 * b * b) - 1.0) * y + m3 * z;
        double zz = m2 * x + m3 * y + ((2.0 * c * c) - 1.0) * z;
        _coord.SetCoord(xx, yy, zz);
    }

    internal void Mirror(in Frame3D a2)
    {
        Direction3D vz = a2.Direction();
        Mirror(vz);
        Reverse();
    }

    internal void Rotate(in Axis3D a1, double ang)
    {
        Transform3D t = new();
        t.SetRotation(a1, ang);
        _coord.Multiply(t.HVectorialPart());
    }

    internal void Transform(in Transform3D t)
    {
        if (t.Form() == Identity || t.Form() == Translation)
        {
        }
        else if (t.Form() == PntMirror)
        {
            _coord.Reverse();
        }
        else if (t.Form() == Scale)
        {
            if (t.ScaleFactor() < 0.0)
            {
                _coord.Reverse();
            }
        }
        else
        {
            _coord.Multiply(t.HVectorialPart());
            double d = _coord.Modulus();
            _coord.Divide(d);
            if (t.ScaleFactor() < 0.0)
            {
                _coord.Reverse();
            }
        }
    }

    // Copies with a mutator applied

    public readonly Direction3D WithCoord(int index, double xi)
    {
        Direction3D copy = this;
        copy.SetCoord(index, xi);
        return copy;
    }

    public readonly Direction3D WithCoord(double xv, double yv, double zv) => new(xv, yv, zv);

    public readonly Direction3D WithX(double x)
    {
        Direction3D copy = this;
        copy.SetX(x);
        return copy;
    }

    public readonly Direction3D WithY(double y)
    {
        Direction3D copy = this;
        copy.SetY(y);
        return copy;
    }

    public readonly Direction3D WithZ(double z)
    {
        Direction3D copy = this;
        copy.SetZ(z);
        return copy;
    }

    public readonly Direction3D WithXYZ(in Xyz xyz) => new(xyz);

    // Queries

    public readonly double Coord(int index) => _coord.Coord(index);

    public readonly void Coord(out double xv, out double yv, out double zv) => _coord.Coord(out xv, out yv, out zv);

    public readonly double X() => _coord.X();

    public readonly double Y() => _coord.Y();

    public readonly double Z() => _coord.Z();

    public readonly Xyz XYZ() => _coord;

    public readonly bool IsEqual(in Direction3D other, double angularTolerance) => Angle(other) <= angularTolerance;

    public readonly bool IsNormal(in Direction3D other, double angularTolerance)
    {
        double ang = M_PI / 2.0 - Angle(other);
        if (ang < 0)
        {
            ang = -ang;
        }

        return ang <= angularTolerance;
    }

    public readonly bool IsOpposite(in Direction3D other, double angularTolerance) => M_PI - Angle(other) <= angularTolerance;

    public readonly bool IsParallel(in Direction3D other, double angularTolerance)
    {
        double ang = Angle(other);
        return ang <= angularTolerance || M_PI - ang <= angularTolerance;
    }

    public readonly double Angle(in Direction3D other)
    {
        double cosinus = _coord.Dot(other._coord);
        if (cosinus > -0.70710678118655 && cosinus < 0.70710678118655)
        {
            return Math.Acos(cosinus);
        }
        else
        {
            double sinus = _coord.Crossed(other._coord).Modulus();
            if (cosinus < 0.0)
            {
                return M_PI - Math.Asin(sinus);
            }
            else
            {
                return Math.Asin(sinus);
            }
        }
    }

    public readonly double AngleWithRef(in Direction3D other, in Direction3D vref)
    {
        double ang;
        Xyz xyz = _coord.Crossed(other._coord);
        double cosinus = _coord.Dot(other._coord);
        double sinus = xyz.Modulus();
        if (cosinus > -0.70710678118655 && cosinus < 0.70710678118655)
        {
            ang = Math.Acos(cosinus);
        }
        else
        {
            if (cosinus < 0.0)
            {
                ang = M_PI - Math.Asin(sinus);
            }
            else
            {
                ang = Math.Asin(sinus);
            }
        }

        if (xyz.Dot(vref._coord) >= 0.0)
        {
            return ang;
        }
        else
        {
            return -ang;
        }
    }

    public readonly Direction3D Crossed(in Direction3D right)
    {
        Direction3D v = this;
        v.Cross(right);
        return v;
    }

    public readonly Direction3D CrossCrossed(in Direction3D v1, in Direction3D v2)
    {
        Direction3D v = this;
        v.CrossCross(v1, v2);
        return v;
    }

    public readonly double Dot(in Direction3D other) => _coord.Dot(other._coord);

    public readonly double DotCross(in Direction3D v1, in Direction3D v2) => _coord.Dot(v1._coord.Crossed(v2._coord));

    public readonly Direction3D Reversed()
    {
        Direction3D v = this;
        v._coord.Reverse();
        return v;
    }

    public readonly Direction3D Mirrored(in Direction3D v)
    {
        Direction3D vres = this;
        vres.Mirror(v);
        return vres;
    }

    public readonly Direction3D Mirrored(in Axis3D a1)
    {
        Direction3D v = this;
        v.Mirror(a1);
        return v;
    }

    public readonly Direction3D Mirrored(in Frame3D a2)
    {
        Direction3D v = this;
        v.Mirror(a2);
        return v;
    }

    public readonly Direction3D Rotated(in Axis3D a1, double ang)
    {
        Direction3D v = this;
        v.Rotate(a1, ang);
        return v;
    }

    public readonly Direction3D Transformed(in Transform3D t)
    {
        Direction3D v = this;
        v.Transform(t);
        return v;
    }

    public static Direction3D operator -(in Direction3D v) => v.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Direction3D left, in Direction3D right) => left.Dot(right);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static Direction3D operator ^(in Direction3D left, in Direction3D right) => left.Crossed(right);

    public readonly bool Equals(Direction3D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Direction3D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Direction3D left, in Direction3D right) => left.Equals(right);

    public static bool operator !=(in Direction3D left, in Direction3D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Dir", _coord.X(), _coord.Y(), _coord.Z());
}
