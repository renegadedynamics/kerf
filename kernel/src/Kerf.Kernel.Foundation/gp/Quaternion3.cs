// Port of: src/FoundationClasses/TKMath/gp/gp_Quaternion.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Quaternion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.EulerSequence;

namespace Kerf.Engine.Geometry;

/// <summary>A quaternion, mostly used to represent a rotation in 3D space.</summary>
/// <remarks><c>new gp_Quaternion()</c> is the identity (0, 0, 0, 1), as in OCCT; <c>default(gp_Quaternion)</c> is zero.</remarks>
internal partial struct Quaternion3 : IEquatable<Quaternion3>
{
    private double _x;
    private double _y;
    private double _z;
    private double _w;

    public Quaternion3()
    {
        _x = 0.0;
        _y = 0.0;
        _z = 0.0;
        _w = 1.0;
    }

    public Quaternion3(double x, double y, double z, double w)
    {
        _x = x;
        _y = y;
        _z = z;
        _w = w;
    }

    /// <summary>The shortest rotation from one vector to another.</summary>
    public Quaternion3(in Vector3D vecFrom, in Vector3D vecTo) : this() => SetRotation(vecFrom, vecTo);

    /// <summary>The shortest rotation from one vector to another, about <paramref name="helpCrossVec"/> when they are opposite.</summary>
    public Quaternion3(in Vector3D vecFrom, in Vector3D vecTo, in Vector3D helpCrossVec) : this()
        => SetRotation(vecFrom, vecTo, helpCrossVec);

    /// <summary>The rotation by an angle about an axis.</summary>
    public Quaternion3(in Vector3D axis, double angle) : this() => SetVectorAndAngle(axis, angle);

    /// <summary>The rotation a rotation matrix represents.</summary>
    public Quaternion3(in Matrix3x3 mat) : this() => SetMatrix(mat);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetRotation(in Vector3D vecFrom, in Vector3D vecTo)
    {
        Vector3D vecCross = vecFrom.Crossed(vecTo);
        Set(vecCross.X, vecCross.Y, vecCross.Z, vecFrom.Dot(vecTo));
        Normalize(); // if "from" or "to" not unit, normalize quat
        _w += 1.0; // reducing angle to halfangle
        if (_w <= Constants.Resolution()) // angle close to PI
        {
            if ((vecFrom.Z * vecFrom.Z) > (vecFrom.X * vecFrom.X))
            {
                Set(0.0, vecFrom.Z, -vecFrom.Y, _w); // theVecFrom * gp_Vec(1,0,0)
            }
            else
            {
                Set(vecFrom.Y, -vecFrom.X, 0.0, _w); // theVecFrom * gp_Vec(0,0,1)
            }
        }

        Normalize();
    }

    internal void SetRotation(in Vector3D vecFrom, in Vector3D vecTo, in Vector3D helpCrossVec)
    {
        Vector3D vecCross = vecFrom.Crossed(vecTo);
        Set(vecCross.X, vecCross.Y, vecCross.Z, vecFrom.Dot(vecTo));
        Normalize(); // if "from" or "to" not unit, normalize quat
        _w += 1.0; // reducing angle to halfangle
        if (_w <= Constants.Resolution()) // angle close to PI
        {
            Vector3D theAxis = vecFrom.Crossed(helpCrossVec);
            Set(theAxis.X, theAxis.Y, theAxis.Z, _w);
        }

        Normalize();
    }

    internal void SetVectorAndAngle(in Vector3D axis, double angle)
    {
        Vector3D anAxis = axis.Normalized();
        double angleHalf = 0.5 * angle;
        double sin_a = Math.Sin(angleHalf);
        Set(anAxis.X * sin_a, anAxis.Y * sin_a, anAxis.Z * sin_a, Math.Cos(angleHalf));
    }

    internal void SetMatrix(in Matrix3x3 mat)
    {
        double tr = mat[1, 1] + mat[2, 2] + mat[3, 3]; // trace of martix
        if (tr > 0.0)
        {
            // if trace positive than "w" is biggest component
            Set(mat[3, 2] - mat[2, 3], mat[1, 3] - mat[3, 1], mat[2, 1] - mat[1, 2], tr + 1.0);
            Scale(0.5 / Math.Sqrt(_w)); // "w" contain the "norm * 4"
        }
        else if ((mat[1, 1] > mat[2, 2]) && (mat[1, 1] > mat[3, 3]))
        {
            // Some of vector components is bigger
            Set(1.0 + mat[1, 1] - mat[2, 2] - mat[3, 3],
                mat[1, 2] + mat[2, 1],
                mat[1, 3] + mat[3, 1],
                mat[3, 2] - mat[2, 3]);
            Scale(0.5 / Math.Sqrt(_x));
        }
        else if (mat[2, 2] > mat[3, 3])
        {
            Set(mat[1, 2] + mat[2, 1],
                1.0 + mat[2, 2] - mat[1, 1] - mat[3, 3],
                mat[2, 3] + mat[3, 2],
                mat[1, 3] - mat[3, 1]);
            Scale(0.5 / Math.Sqrt(_y));
        }
        else
        {
            Set(mat[1, 3] + mat[3, 1],
                mat[2, 3] + mat[3, 2],
                1.0 + mat[3, 3] - mat[1, 1] - mat[2, 2],
                mat[2, 1] - mat[1, 2]);
            Scale(0.5 / Math.Sqrt(_z));
        }
    }

    internal void SetEulerAngles(EulerSequence order, double alpha, double beta, double gamma)
    {
        EulerSequenceParameters o = TranslateEulerSequence(order);

        double a = alpha, b = beta, c = gamma;
        if (!o.IsExtrinsic)
        {
            a = gamma;
            c = alpha;
        }

        if (o.isOdd)
        {
            b = -b;
        }

        double ti = 0.5 * a;
        double tj = 0.5 * b;
        double th = 0.5 * c;
        double ci = Math.Cos(ti);
        double cj = Math.Cos(tj);
        double ch = Math.Cos(th);
        double si = Math.Sin(ti);
        double sj = Math.Sin(tj);
        double sh = Math.Sin(th);
        double cc = ci * ch;
        double cs = ci * sh;
        double sc = si * ch;
        double ss = si * sh;

        Span<double> values = stackalloc double[4]; // w, x, y, z
        if (o.IsTwoAxes)
        {
            values[o.i] = cj * (cs + sc);
            values[o.j] = sj * (cc + ss);
            values[o.k] = sj * (cs - sc);
            values[0] = cj * (cc - ss);
        }
        else
        {
            values[o.i] = cj * sc - sj * cs;
            values[o.j] = cj * ss + sj * cc;
            values[o.k] = cj * cs - sj * sc;
            values[0] = cj * cc + sj * ss;
        }

        if (o.isOdd)
        {
            values[o.j] = -values[o.j];
        }

        _x = values[1];
        _y = values[2];
        _z = values[3];
        _w = values[0];
    }

    internal void Set(double x, double y, double z, double w)
    {
        _x = x;
        _y = y;
        _z = z;
        _w = w;
    }

    internal void Set(in Quaternion3 quaternion) => this = quaternion;

    internal void SetIdent()
    {
        _x = _y = _z = 0.0;
        _w = 1.0;
    }

    internal void Reverse()
    {
        _x = -_x;
        _y = -_y;
        _z = -_z;
    }

    internal void Invert()
    {
        double anIn = 1.0 / SquareNorm();
        Set(-_x * anIn, -_y * anIn, -_z * anIn, _w * anIn);
    }

    internal void Scale(double scale)
    {
        _x *= scale;
        _y *= scale;
        _z *= scale;
        _w *= scale;
    }

    /// <summary>Scales by the inverse of the sum of the components' magnitudes, so the norm is near 1, without a square root.</summary>
    internal void StabilizeLength()
    {
        double cs = Math.Abs(_x) + Math.Abs(_y) + Math.Abs(_z) + Math.Abs(_w);
        if (cs > 0.0)
        {
            _x /= cs;
            _y /= cs;
            _z /= cs;
            _w /= cs;
        }
        else
        {
            SetIdent();
        }
    }

    internal void Normalize()
    {
        double magn = Norm();
        if (magn < Constants.Resolution())
        {
            StabilizeLength();
            magn = Norm();
        }

        Scale(1.0 / magn);
    }

    internal void Add(in Quaternion3 q)
    {
        _x += q._x;
        _y += q._y;
        _z += q._z;
        _w += q._w;
    }

    internal void Subtract(in Quaternion3 q)
    {
        _x -= q._x;
        _y -= q._y;
        _z -= q._z;
        _w -= q._w;
    }

    internal void Multiply(in Quaternion3 other) => this = Multiplied(other);

    // Copies with a mutator applied

    public readonly Quaternion3 WithRotation(in Vector3D vecFrom, in Vector3D vecTo) => new(vecFrom, vecTo);

    public readonly Quaternion3 WithRotation(in Vector3D vecFrom, in Vector3D vecTo, in Vector3D helpCrossVec)
        => new(vecFrom, vecTo, helpCrossVec);

    public readonly Quaternion3 WithVectorAndAngle(in Vector3D axis, double angle) => new(axis, angle);

    public readonly Quaternion3 WithMatrix(in Matrix3x3 mat) => new(mat);

    public readonly Quaternion3 WithEulerAngles(EulerSequence order, double alpha, double beta, double gamma)
    {
        Quaternion3 copy = this;
        copy.SetEulerAngles(order, alpha, beta, gamma);
        return copy;
    }

    /// <summary>The rotation given by Euler angles in the given order.</summary>
    public static Quaternion3 FromEulerAngles(EulerSequence order, double alpha, double beta, double gamma)
        => new Quaternion3().WithEulerAngles(order, alpha, beta, gamma);

    public readonly Quaternion3 StabilizedLength()
    {
        Quaternion3 copy = this;
        copy.StabilizeLength();
        return copy;
    }

    // Queries

    /// <summary>True when every component differs by at most <see cref="Constants.Resolution"/>.</summary>
    public readonly bool IsEqual(in Quaternion3 other)
        => Math.Abs(_x - other._x) <= Constants.Resolution()
           && Math.Abs(_y - other._y) <= Constants.Resolution()
           && Math.Abs(_z - other._z) <= Constants.Resolution()
           && Math.Abs(_w - other._w) <= Constants.Resolution();

    public readonly void GetVectorAndAngle(out Vector3D axis, out double angle)
    {
        axis = new Vector3D();
        double vl = Math.Sqrt(_x * _x + _y * _y + _z * _z);
        if (vl > Constants.Resolution())
        {
            double ivl = 1.0 / vl;
            axis = axis.WithCoord(_x * ivl, _y * ivl, _z * ivl);
            if (_w < 0.0)
            {
                angle = 2.0 * Math.Atan2(-vl, -_w); // [-PI, 0]
            }
            else
            {
                angle = 2.0 * Math.Atan2(vl, _w); // [ 0, PI]
            }
        }
        else
        {
            axis = axis.WithCoord(0.0, 0.0, 1.0);
            angle = 0.0;
        }
    }

    public readonly Matrix3x3 GetMatrix()
    {
        double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;
        double s = 2.0 / SquareNorm();
        x2 = _x * s;
        y2 = _y * s;
        z2 = _z * s;
        xx = _x * x2;
        xy = _x * y2;
        xz = _x * z2;
        yy = _y * y2;
        yz = _y * z2;
        zz = _z * z2;
        wx = _w * x2;
        wy = _w * y2;
        wz = _w * z2;

        return new Matrix3x3(1.0 - (yy + zz), xy - wz, xz + wy,
                             xy + wz, 1.0 - (xx + zz), yz - wx,
                             xz - wy, yz + wx, 1.0 - (xx + yy));
    }

    public readonly void GetEulerAngles(EulerSequence order, out double alpha, out double beta, out double gamma)
    {
        Matrix3x3 m = GetMatrix();

        EulerSequenceParameters o = TranslateEulerSequence(order);
        if (o.IsTwoAxes)
        {
            double sy = Math.Sqrt(m[o.i, o.j] * m[o.i, o.j] + m[o.i, o.k] * m[o.i, o.k]);
            if (sy > 16 * RealEpsilon())
            {
                alpha = Math.Atan2(m[o.i, o.j], m[o.i, o.k]);
                gamma = Math.Atan2(m[o.j, o.i], -m[o.k, o.i]);
            }
            else
            {
                alpha = Math.Atan2(-m[o.j, o.k], m[o.j, o.j]);
                gamma = 0.0;
            }

            beta = Math.Atan2(sy, m[o.i, o.i]);
        }
        else
        {
            double cy = Math.Sqrt(m[o.i, o.i] * m[o.i, o.i] + m[o.j, o.i] * m[o.j, o.i]);
            if (cy > 16 * RealEpsilon())
            {
                alpha = Math.Atan2(m[o.k, o.j], m[o.k, o.k]);
                gamma = Math.Atan2(m[o.j, o.i], m[o.i, o.i]);
            }
            else
            {
                alpha = Math.Atan2(-m[o.j, o.k], m[o.j, o.j]);
                gamma = 0.0;
            }

            beta = Math.Atan2(-m[o.k, o.i], cy);
        }

        if (o.isOdd)
        {
            alpha = -alpha;
            beta = -beta;
            gamma = -gamma;
        }

        if (!o.IsExtrinsic)
        {
            double first = alpha;
            alpha = gamma;
            gamma = first;
        }
    }

    public readonly double X() => _x;

    public readonly double Y() => _y;

    public readonly double Z() => _z;

    public readonly double W() => _w;

    public readonly Quaternion3 Reversed() => new(-_x, -_y, -_z, _w);

    public readonly Quaternion3 Inverted()
    {
        double anIn = 1.0 / SquareNorm();
        return new Quaternion3(-_x * anIn, -_y * anIn, -_z * anIn, _w * anIn);
    }

    public readonly double SquareNorm() => _x * _x + _y * _y + _z * _z + _w * _w;

    public readonly double Norm() => Math.Sqrt(SquareNorm());

    public readonly Quaternion3 Scaled(double scale) => new(_x * scale, _y * scale, _z * scale, _w * scale);

    public readonly Quaternion3 Normalized()
    {
        Quaternion3 normilizedQ = this;
        normilizedQ.Normalize();
        return normilizedQ;
    }

    public readonly Quaternion3 Negated() => new(-_x, -_y, -_z, -_w);

    public readonly Quaternion3 Added(in Quaternion3 other) => new(_x + other._x, _y + other._y, _z + other._z, _w + other._w);

    public readonly Quaternion3 Subtracted(in Quaternion3 other) => new(_x - other._x, _y - other._y, _z - other._z, _w - other._w);

    public readonly Quaternion3 Multiplied(in Quaternion3 q)
        => new(_w * q._x + _x * q._w + _y * q._z - _z * q._y,
               _w * q._y + _y * q._w + _z * q._x - _x * q._z,
               _w * q._z + _z * q._w + _x * q._y - _y * q._x,
               _w * q._w - _x * q._x - _y * q._y - _z * q._z);

    public readonly double Dot(in Quaternion3 other) => _x * other._x + _y * other._y + _z * other._z + _w * other._w;

    /// <summary>The rotation angle, in [-PI, PI].</summary>
    public readonly double GetRotationAngle()
    {
        if (_w < 0.0)
        {
            return 2.0 * Math.Atan2(-Math.Sqrt(_x * _x + _y * _y + _z * _z), -_w);
        }
        else
        {
            return 2.0 * Math.Atan2(Math.Sqrt(_x * _x + _y * _y + _z * _z), _w);
        }
    }

    /// <summary>The vector rotated by this quaternion.</summary>
    public readonly Vector3D Multiply(in Vector3D vec)
    {
        Quaternion3 theQ = new(vec.X * _w + vec.Z * _y - vec.Y * _z,
                                 vec.Y * _w + vec.X * _z - vec.Z * _x,
                                 vec.Z * _w + vec.Y * _x - vec.X * _y,
                                 vec.X * _x + vec.Y * _y + vec.Z * _z);
        return new Vector3D(_w * theQ._x + _x * theQ._w + _y * theQ._z - _z * theQ._y,
                          _w * theQ._y + _y * theQ._w + _z * theQ._x - _x * theQ._z,
                          _w * theQ._z + _z * theQ._w + _x * theQ._y - _y * theQ._x)
               * (1.0 / SquareNorm());
    }

    public static Quaternion3 operator -(in Quaternion3 q) => q.Negated();

    public static Quaternion3 operator +(in Quaternion3 left, in Quaternion3 right) => left.Added(right);

    public static Quaternion3 operator -(in Quaternion3 left, in Quaternion3 right) => left.Subtracted(right);

    public static Quaternion3 operator *(in Quaternion3 left, in Quaternion3 right) => left.Multiplied(right);

    public static Quaternion3 operator *(in Quaternion3 q, double scale) => q.Scaled(scale);

    public static Vector3D operator *(in Quaternion3 q, in Vector3D vec) => q.Multiply(vec);

    public readonly bool Equals(Quaternion3 other) => _x.Equals(other._x) && _y.Equals(other._y) && _z.Equals(other._z) && _w.Equals(other._w);

    public override readonly bool Equals(object? obj) => obj is Quaternion3 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_x, _y, _z, _w);

    public static bool operator ==(in Quaternion3 left, in Quaternion3 right) => left.Equals(right);

    public static bool operator !=(in Quaternion3 left, in Quaternion3 right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Quaternion", _x, _y, _z, _w);

    /// <summary>How an Euler sequence maps onto rotation axes (OCCT's gp_EulerSequence_Parameters).</summary>
    private readonly struct EulerSequenceParameters
    {
        public readonly int i; // first rotation axis
        public readonly int j; // next axis of rotation
        public readonly int k; // third axis
        public readonly bool isOdd; // true if order of two first rotation axes is odd permutation, e.g. XZ
        public readonly bool IsTwoAxes; // true if third rotation is about the same axis as first
        public readonly bool IsExtrinsic; // true if rotations are made around fixed axes

        public EulerSequenceParameters(int ax1, bool theisOdd, bool theisTwoAxes, bool theisExtrinsic)
        {
            i = ax1;
            j = 1 + (ax1 + (theisOdd ? 1 : 0)) % 3;
            k = 1 + (ax1 + (theisOdd ? 0 : 1)) % 3;
            isOdd = theisOdd;
            IsTwoAxes = theisTwoAxes;
            IsExtrinsic = theisExtrinsic;
        }
    }

    private static EulerSequenceParameters TranslateEulerSequence(EulerSequence seq)
    {
        const bool f = false;
        const bool t = true;

        return seq switch
        {
            Extrinsic_XYZ => new(1, f, f, t),
            Extrinsic_XZY => new(1, t, f, t),
            Extrinsic_YZX => new(2, f, f, t),
            Extrinsic_YXZ => new(2, t, f, t),
            Extrinsic_ZXY => new(3, f, f, t),
            Extrinsic_ZYX => new(3, t, f, t),
            // Conversion of intrinsic angles is made by the same code as for extrinsic,
            // using equivalence rule: intrinsic rotation is equivalent to extrinsic
            // rotation by the same angles but with inverted order of elemental rotations.
            // Swapping of angles (Alpha <-> Gamma) is done inside conversion procedure;
            // sequence of axes is inverted by setting appropriate parameters here.
            // Note that proper Euler angles (last block below) are symmetric for sequence of axes.
            Intrinsic_XYZ => new(3, t, f, f),
            Intrinsic_XZY => new(2, f, f, f),
            Intrinsic_YZX => new(1, t, f, f),
            Intrinsic_YXZ => new(3, f, f, f),
            Intrinsic_ZXY => new(2, t, f, f),
            Intrinsic_ZYX => new(1, f, f, f),
            Extrinsic_XYX => new(1, f, t, t),
            Extrinsic_XZX => new(1, t, t, t),
            Extrinsic_YZY => new(2, f, t, t),
            Extrinsic_YXY => new(2, t, t, t),
            Extrinsic_ZXZ => new(3, f, t, t),
            Extrinsic_ZYZ => new(3, t, t, t),
            Intrinsic_XYX => new(1, f, t, f),
            Intrinsic_XZX => new(1, t, t, f),
            Intrinsic_YZY => new(2, f, t, f),
            Intrinsic_YXY => new(2, t, t, f),
            Intrinsic_ZXZ => new(3, f, t, f),
            Intrinsic_ZYZ => new(3, t, t, f),
            YawPitchRoll => new(1, f, f, f), // = Intrinsic_ZYX
            _ => new(3, f, t, f), // gp_EulerAngles = Intrinsic_ZXZ
        };
    }
}
