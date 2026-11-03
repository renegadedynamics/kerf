// Port of: src/FoundationClasses/TKMath/gp/gp_Dir2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Dir2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

/// <summary>A unit vector in 2D space.</summary>
/// <remarks><c>new gp_Dir2d()</c> is (1, 0), as in OCCT; <c>default(gp_Dir2d)</c> is all zeros and not a valid direction.</remarks>
public partial struct Direction2D : IEquatable<Direction2D>
{
    /// <summary>The four directions along the axes of the global system.</summary>
    public enum D
    {
        /// <summary>(1, 0)</summary>
        X,
        /// <summary>(0, 1)</summary>
        Y,
        /// <summary>(-1, 0)</summary>
        NX,
        /// <summary>(0, -1)</summary>
        NY,
    }

    private Xy _coord;

    public Direction2D() => _coord = new Xy(1.0, 0.0);

    public Direction2D(D dir)
        => _coord = new Xy(dir == D.X ? 1.0 : dir == D.NX ? -1.0 : 0.0,
                             dir == D.Y ? 1.0 : dir == D.NY ? -1.0 : 0.0);

    public Direction2D(in Vector2D v) : this(v.XY()) { }

    public Direction2D(in Xy xy) : this(xy.X(), xy.Y()) { }

    public Direction2D(double xv, double yv)
    {
        _coord = default;
        double sqMod = xv * xv + yv * yv;
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            _coord.SetCoord(xv, yv);
            return;
        }

        Standard_ConstructionError_Raise_if(sqMod <= Constants.Resolution() * Constants.Resolution(), "gp_Dir2d() - input vector has zero norm");
        double d = Math.Sqrt(sqMod);
        _coord.SetCoord(xv / d, yv / d);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int index, double xi)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 2, "gp_Dir2d::SetCoord() - index is out of range [1, 2]");
        if (index == 1)
        {
            SetX(xi);
        }
        else
        {
            SetY(xi);
        }
    }

    internal void SetCoord(double xv, double yv)
    {
        double sqMod = xv * xv + yv * yv;
        if (sqMod >= (1.0 - Constants.Resolution()) && sqMod <= (1.0 + Constants.Resolution()))
        {
            _coord.SetCoord(xv, yv);
            return;
        }

        Standard_ConstructionError_Raise_if(sqMod <= Constants.Resolution() * Constants.Resolution(), "gp_Dir2d::SetCoord() - result vector has zero norm");
        double d = Math.Sqrt(sqMod);
        _coord.SetCoord(xv / d, yv / d);
    }

    internal void SetX(double x) => SetCoord(x, _coord.Y());

    internal void SetY(double y) => SetCoord(_coord.X(), y);

    internal void SetXY(in Xy xy) => SetCoord(xy.X(), xy.Y());

    internal void Reverse() => _coord.Reverse();

    internal void Mirror(in Direction2D v)
    {
        Xy xy = v._coord;
        double a = xy.X();
        double b = xy.Y();
        double x = _coord.X();
        double y = _coord.Y();
        double m1 = 2.0 * a * b;
        double xx = ((2.0 * a * a) - 1.0) * x + m1 * y;
        double yy = m1 * x + ((2.0 * b * b) - 1.0) * y;
        _coord.SetCoord(xx, yy);
    }

    internal void Mirror(in Axis2D a2)
    {
        Xy xy = a2.Direction().XY();
        double a = xy.X();
        double b = xy.Y();
        double x = _coord.X();
        double y = _coord.Y();
        double m1 = 2.0 * a * b;
        double xx = ((2.0 * a * a) - 1.0) * x + m1 * y;
        double yy = m1 * x + ((2.0 * b * b) - 1.0) * y;
        _coord.SetCoord(xx, yy);
    }

    internal void Rotate(double ang)
    {
        Transform2D t = new();
        t.SetRotation(new Point2D(0.0, 0.0), ang);
        _coord.Multiply(t.HVectorialPart());
    }

    internal void Transform(in Transform2D t)
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

    public readonly Direction2D WithCoord(int index, double xi)
    {
        Direction2D copy = this;
        copy.SetCoord(index, xi);
        return copy;
    }

    public readonly Direction2D WithCoord(double xv, double yv)
    {
        Direction2D copy = this;
        copy.SetCoord(xv, yv);
        return copy;
    }

    public readonly Direction2D WithX(double x)
    {
        Direction2D copy = this;
        copy.SetX(x);
        return copy;
    }

    public readonly Direction2D WithY(double y)
    {
        Direction2D copy = this;
        copy.SetY(y);
        return copy;
    }

    public readonly Direction2D WithXY(in Xy xy)
    {
        Direction2D copy = this;
        copy.SetXY(xy);
        return copy;
    }

    // Queries

    public readonly double Coord(int index) => _coord.Coord(index);

    public readonly void Coord(out double xv, out double yv) => _coord.Coord(out xv, out yv);

    public readonly double X() => _coord.X();

    public readonly double Y() => _coord.Y();

    public readonly Xy XY() => _coord;

    public readonly bool IsEqual(in Direction2D other, double angularTolerance)
    {
        double ang = Angle(other);
        if (ang < 0)
        {
            ang = -ang;
        }

        return ang <= angularTolerance;
    }

    public readonly bool IsNormal(in Direction2D other, double angularTolerance)
    {
        double ang = Angle(other);
        if (ang < 0)
        {
            ang = -ang;
        }

        ang = M_PI / 2.0 - ang;
        if (ang < 0)
        {
            ang = -ang;
        }

        return ang <= angularTolerance;
    }

    public readonly bool IsOpposite(in Direction2D other, double angularTolerance)
    {
        double ang = Angle(other);
        if (ang < 0)
        {
            ang = -ang;
        }

        return M_PI - ang <= angularTolerance;
    }

    public readonly bool IsParallel(in Direction2D other, double angularTolerance)
    {
        double ang = Angle(other);
        if (ang < 0)
        {
            ang = -ang;
        }

        return ang <= angularTolerance || M_PI - ang <= angularTolerance;
    }

    /// <summary>The signed angle from this direction to <paramref name="other"/>, in ]-PI, PI].</summary>
    public readonly double Angle(in Direction2D other)
    {
        double cosinus = _coord.Dot(other._coord);
        double sinus = _coord.Crossed(other._coord);
        if (cosinus > -0.70710678118655 && cosinus < 0.70710678118655)
        {
            if (sinus > 0.0)
            {
                return Math.Acos(cosinus);
            }
            else
            {
                return -Math.Acos(cosinus);
            }
        }
        else
        {
            if (cosinus > 0.0)
            {
                return Math.Asin(sinus);
            }
            else
            {
                if (sinus > 0.0)
                {
                    return M_PI - Math.Asin(sinus);
                }
                else
                {
                    return -M_PI - Math.Asin(sinus);
                }
            }
        }
    }

    /// <summary>The scalar cross product.</summary>
    public readonly double Crossed(in Direction2D right) => _coord.Crossed(right._coord);

    public readonly double Dot(in Direction2D other) => _coord.Dot(other._coord);

    public readonly Direction2D Reversed()
    {
        Direction2D v = this;
        v._coord.Reverse();
        return v;
    }

    public readonly Direction2D Mirrored(in Direction2D v)
    {
        Direction2D vres = this;
        vres.Mirror(v);
        return vres;
    }

    public readonly Direction2D Mirrored(in Axis2D a)
    {
        Direction2D v = this;
        v.Mirror(a);
        return v;
    }

    public readonly Direction2D Rotated(double ang)
    {
        Direction2D v = this;
        v.Rotate(ang);
        return v;
    }

    public readonly Direction2D Transformed(in Transform2D t)
    {
        Direction2D v = this;
        v.Transform(t);
        return v;
    }

    public static Direction2D operator -(in Direction2D v) => v.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Direction2D left, in Direction2D right) => left.Dot(right);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in Direction2D left, in Direction2D right) => left.Crossed(right);

    public readonly bool Equals(Direction2D other) => _coord.Equals(other._coord);

    public override readonly bool Equals(object? obj) => obj is Direction2D other && Equals(other);

    public override readonly int GetHashCode() => _coord.GetHashCode();

    public static bool operator ==(in Direction2D left, in Direction2D right) => left.Equals(right);

    public static bool operator !=(in Direction2D left, in Direction2D right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Dir2d", _coord.X(), _coord.Y());
}
