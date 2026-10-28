// Port of: src/FoundationClasses/TKMath/gp/gp_Quaternion.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Quaternion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_EulerSequence;

namespace Occt;

/// <summary>A quaternion, mostly used to represent a rotation in 3D space.</summary>
/// <remarks><c>new gp_Quaternion()</c> is the identity (0, 0, 0, 1), as in OCCT; <c>default(gp_Quaternion)</c> is zero.</remarks>
public partial struct gp_Quaternion : IEquatable<gp_Quaternion>
{
    private double x;
    private double y;
    private double z;
    private double w;

    public gp_Quaternion()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 1.0;
    }

    public gp_Quaternion(double theX, double theY, double theZ, double theW)
    {
        x = theX;
        y = theY;
        z = theZ;
        w = theW;
    }

    /// <summary>The shortest rotation from one vector to another.</summary>
    public gp_Quaternion(in gp_Vec theVecFrom, in gp_Vec theVecTo) : this() => SetRotation(theVecFrom, theVecTo);

    /// <summary>The shortest rotation from one vector to another, about <paramref name="theHelpCrossVec"/> when they are opposite.</summary>
    public gp_Quaternion(in gp_Vec theVecFrom, in gp_Vec theVecTo, in gp_Vec theHelpCrossVec) : this()
        => SetRotation(theVecFrom, theVecTo, theHelpCrossVec);

    /// <summary>The rotation by an angle about an axis.</summary>
    public gp_Quaternion(in gp_Vec theAxis, double theAngle) : this() => SetVectorAndAngle(theAxis, theAngle);

    /// <summary>The rotation a rotation matrix represents.</summary>
    public gp_Quaternion(in gp_Mat theMat) : this() => SetMatrix(theMat);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetRotation(in gp_Vec theVecFrom, in gp_Vec theVecTo)
    {
        gp_Vec aVecCross = theVecFrom.Crossed(theVecTo);
        Set(aVecCross.X(), aVecCross.Y(), aVecCross.Z(), theVecFrom.Dot(theVecTo));
        Normalize(); // if "from" or "to" not unit, normalize quat
        w += 1.0; // reducing angle to halfangle
        if (w <= gp.Resolution()) // angle close to PI
        {
            if ((theVecFrom.Z() * theVecFrom.Z()) > (theVecFrom.X() * theVecFrom.X()))
            {
                Set(0.0, theVecFrom.Z(), -theVecFrom.Y(), w); // theVecFrom * gp_Vec(1,0,0)
            }
            else
            {
                Set(theVecFrom.Y(), -theVecFrom.X(), 0.0, w); // theVecFrom * gp_Vec(0,0,1)
            }
        }

        Normalize();
    }

    internal void SetRotation(in gp_Vec theVecFrom, in gp_Vec theVecTo, in gp_Vec theHelpCrossVec)
    {
        gp_Vec aVecCross = theVecFrom.Crossed(theVecTo);
        Set(aVecCross.X(), aVecCross.Y(), aVecCross.Z(), theVecFrom.Dot(theVecTo));
        Normalize(); // if "from" or "to" not unit, normalize quat
        w += 1.0; // reducing angle to halfangle
        if (w <= gp.Resolution()) // angle close to PI
        {
            gp_Vec theAxis = theVecFrom.Crossed(theHelpCrossVec);
            Set(theAxis.X(), theAxis.Y(), theAxis.Z(), w);
        }

        Normalize();
    }

    internal void SetVectorAndAngle(in gp_Vec theAxis, double theAngle)
    {
        gp_Vec anAxis = theAxis.Normalized();
        double anAngleHalf = 0.5 * theAngle;
        double sin_a = Math.Sin(anAngleHalf);
        Set(anAxis.X() * sin_a, anAxis.Y() * sin_a, anAxis.Z() * sin_a, Math.Cos(anAngleHalf));
    }

    internal void SetMatrix(in gp_Mat theMat)
    {
        double tr = theMat[1, 1] + theMat[2, 2] + theMat[3, 3]; // trace of martix
        if (tr > 0.0)
        {
            // if trace positive than "w" is biggest component
            Set(theMat[3, 2] - theMat[2, 3], theMat[1, 3] - theMat[3, 1], theMat[2, 1] - theMat[1, 2], tr + 1.0);
            Scale(0.5 / Math.Sqrt(w)); // "w" contain the "norm * 4"
        }
        else if ((theMat[1, 1] > theMat[2, 2]) && (theMat[1, 1] > theMat[3, 3]))
        {
            // Some of vector components is bigger
            Set(1.0 + theMat[1, 1] - theMat[2, 2] - theMat[3, 3],
                theMat[1, 2] + theMat[2, 1],
                theMat[1, 3] + theMat[3, 1],
                theMat[3, 2] - theMat[2, 3]);
            Scale(0.5 / Math.Sqrt(x));
        }
        else if (theMat[2, 2] > theMat[3, 3])
        {
            Set(theMat[1, 2] + theMat[2, 1],
                1.0 + theMat[2, 2] - theMat[1, 1] - theMat[3, 3],
                theMat[2, 3] + theMat[3, 2],
                theMat[1, 3] - theMat[3, 1]);
            Scale(0.5 / Math.Sqrt(y));
        }
        else
        {
            Set(theMat[1, 3] + theMat[3, 1],
                theMat[2, 3] + theMat[3, 2],
                1.0 + theMat[3, 3] - theMat[1, 1] - theMat[2, 2],
                theMat[2, 1] - theMat[1, 2]);
            Scale(0.5 / Math.Sqrt(z));
        }
    }

    internal void SetEulerAngles(gp_EulerSequence theOrder, double theAlpha, double theBeta, double theGamma)
    {
        EulerSequenceParameters o = TranslateEulerSequence(theOrder);

        double a = theAlpha, b = theBeta, c = theGamma;
        if (!o.isExtrinsic)
        {
            a = theGamma;
            c = theAlpha;
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
        if (o.isTwoAxes)
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

        x = values[1];
        y = values[2];
        z = values[3];
        w = values[0];
    }

    internal void Set(double theX, double theY, double theZ, double theW)
    {
        x = theX;
        y = theY;
        z = theZ;
        w = theW;
    }

    internal void Set(in gp_Quaternion theQuaternion) => this = theQuaternion;

    internal void SetIdent()
    {
        x = y = z = 0.0;
        w = 1.0;
    }

    internal void Reverse()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    internal void Invert()
    {
        double anIn = 1.0 / SquareNorm();
        Set(-x * anIn, -y * anIn, -z * anIn, w * anIn);
    }

    internal void Scale(double theScale)
    {
        x *= theScale;
        y *= theScale;
        z *= theScale;
        w *= theScale;
    }

    /// <summary>Scales by the inverse of the sum of the components' magnitudes, so the norm is near 1, without a square root.</summary>
    internal void StabilizeLength()
    {
        double cs = Math.Abs(x) + Math.Abs(y) + Math.Abs(z) + Math.Abs(w);
        if (cs > 0.0)
        {
            x /= cs;
            y /= cs;
            z /= cs;
            w /= cs;
        }
        else
        {
            SetIdent();
        }
    }

    internal void Normalize()
    {
        double aMagn = Norm();
        if (aMagn < gp.Resolution())
        {
            StabilizeLength();
            aMagn = Norm();
        }

        Scale(1.0 / aMagn);
    }

    internal void Add(in gp_Quaternion theQ)
    {
        x += theQ.x;
        y += theQ.y;
        z += theQ.z;
        w += theQ.w;
    }

    internal void Subtract(in gp_Quaternion theQ)
    {
        x -= theQ.x;
        y -= theQ.y;
        z -= theQ.z;
        w -= theQ.w;
    }

    internal void Multiply(in gp_Quaternion theOther) => this = Multiplied(theOther);

    // Copies with a mutator applied

    public readonly gp_Quaternion WithRotation(in gp_Vec theVecFrom, in gp_Vec theVecTo) => new(theVecFrom, theVecTo);

    public readonly gp_Quaternion WithRotation(in gp_Vec theVecFrom, in gp_Vec theVecTo, in gp_Vec theHelpCrossVec)
        => new(theVecFrom, theVecTo, theHelpCrossVec);

    public readonly gp_Quaternion WithVectorAndAngle(in gp_Vec theAxis, double theAngle) => new(theAxis, theAngle);

    public readonly gp_Quaternion WithMatrix(in gp_Mat theMat) => new(theMat);

    public readonly gp_Quaternion WithEulerAngles(gp_EulerSequence theOrder, double theAlpha, double theBeta, double theGamma)
    {
        gp_Quaternion aCopy = this;
        aCopy.SetEulerAngles(theOrder, theAlpha, theBeta, theGamma);
        return aCopy;
    }

    /// <summary>The rotation given by Euler angles in the given order.</summary>
    public static gp_Quaternion FromEulerAngles(gp_EulerSequence theOrder, double theAlpha, double theBeta, double theGamma)
        => new gp_Quaternion().WithEulerAngles(theOrder, theAlpha, theBeta, theGamma);

    public readonly gp_Quaternion StabilizedLength()
    {
        gp_Quaternion aCopy = this;
        aCopy.StabilizeLength();
        return aCopy;
    }

    // Queries

    /// <summary>True when every component differs by at most <see cref="gp.Resolution"/>.</summary>
    public readonly bool IsEqual(in gp_Quaternion theOther)
        => Math.Abs(x - theOther.x) <= gp.Resolution()
           && Math.Abs(y - theOther.y) <= gp.Resolution()
           && Math.Abs(z - theOther.z) <= gp.Resolution()
           && Math.Abs(w - theOther.w) <= gp.Resolution();

    public readonly void GetVectorAndAngle(out gp_Vec theAxis, out double theAngle)
    {
        theAxis = new gp_Vec();
        double vl = Math.Sqrt(x * x + y * y + z * z);
        if (vl > gp.Resolution())
        {
            double ivl = 1.0 / vl;
            theAxis.SetCoord(x * ivl, y * ivl, z * ivl);
            if (w < 0.0)
            {
                theAngle = 2.0 * Math.Atan2(-vl, -w); // [-PI, 0]
            }
            else
            {
                theAngle = 2.0 * Math.Atan2(vl, w); // [ 0, PI]
            }
        }
        else
        {
            theAxis.SetCoord(0.0, 0.0, 1.0);
            theAngle = 0.0;
        }
    }

    public readonly gp_Mat GetMatrix()
    {
        double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;
        double s = 2.0 / SquareNorm();
        x2 = x * s;
        y2 = y * s;
        z2 = z * s;
        xx = x * x2;
        xy = x * y2;
        xz = x * z2;
        yy = y * y2;
        yz = y * z2;
        zz = z * z2;
        wx = w * x2;
        wy = w * y2;
        wz = w * z2;

        gp_Mat aMat = new();
        aMat.ChangeValue(1, 1) = 1.0 - (yy + zz);
        aMat.ChangeValue(1, 2) = xy - wz;
        aMat.ChangeValue(1, 3) = xz + wy;
        aMat.ChangeValue(2, 1) = xy + wz;
        aMat.ChangeValue(2, 2) = 1.0 - (xx + zz);
        aMat.ChangeValue(2, 3) = yz - wx;
        aMat.ChangeValue(3, 1) = xz - wy;
        aMat.ChangeValue(3, 2) = yz + wx;
        aMat.ChangeValue(3, 3) = 1.0 - (xx + yy);
        return aMat;
    }

    public readonly void GetEulerAngles(gp_EulerSequence theOrder, out double theAlpha, out double theBeta, out double theGamma)
    {
        gp_Mat M = GetMatrix();

        EulerSequenceParameters o = TranslateEulerSequence(theOrder);
        if (o.isTwoAxes)
        {
            double sy = Math.Sqrt(M[o.i, o.j] * M[o.i, o.j] + M[o.i, o.k] * M[o.i, o.k]);
            if (sy > 16 * RealEpsilon())
            {
                theAlpha = Math.Atan2(M[o.i, o.j], M[o.i, o.k]);
                theGamma = Math.Atan2(M[o.j, o.i], -M[o.k, o.i]);
            }
            else
            {
                theAlpha = Math.Atan2(-M[o.j, o.k], M[o.j, o.j]);
                theGamma = 0.0;
            }

            theBeta = Math.Atan2(sy, M[o.i, o.i]);
        }
        else
        {
            double cy = Math.Sqrt(M[o.i, o.i] * M[o.i, o.i] + M[o.j, o.i] * M[o.j, o.i]);
            if (cy > 16 * RealEpsilon())
            {
                theAlpha = Math.Atan2(M[o.k, o.j], M[o.k, o.k]);
                theGamma = Math.Atan2(M[o.j, o.i], M[o.i, o.i]);
            }
            else
            {
                theAlpha = Math.Atan2(-M[o.j, o.k], M[o.j, o.j]);
                theGamma = 0.0;
            }

            theBeta = Math.Atan2(-M[o.k, o.i], cy);
        }

        if (o.isOdd)
        {
            theAlpha = -theAlpha;
            theBeta = -theBeta;
            theGamma = -theGamma;
        }

        if (!o.isExtrinsic)
        {
            double aFirst = theAlpha;
            theAlpha = theGamma;
            theGamma = aFirst;
        }
    }

    public readonly double X() => x;

    public readonly double Y() => y;

    public readonly double Z() => z;

    public readonly double W() => w;

    public readonly gp_Quaternion Reversed() => new(-x, -y, -z, w);

    public readonly gp_Quaternion Inverted()
    {
        double anIn = 1.0 / SquareNorm();
        return new gp_Quaternion(-x * anIn, -y * anIn, -z * anIn, w * anIn);
    }

    public readonly double SquareNorm() => x * x + y * y + z * z + w * w;

    public readonly double Norm() => Math.Sqrt(SquareNorm());

    public readonly gp_Quaternion Scaled(double theScale) => new(x * theScale, y * theScale, z * theScale, w * theScale);

    public readonly gp_Quaternion Normalized()
    {
        gp_Quaternion aNormilizedQ = this;
        aNormilizedQ.Normalize();
        return aNormilizedQ;
    }

    public readonly gp_Quaternion Negated() => new(-x, -y, -z, -w);

    public readonly gp_Quaternion Added(in gp_Quaternion theOther) => new(x + theOther.x, y + theOther.y, z + theOther.z, w + theOther.w);

    public readonly gp_Quaternion Subtracted(in gp_Quaternion theOther) => new(x - theOther.x, y - theOther.y, z - theOther.z, w - theOther.w);

    public readonly gp_Quaternion Multiplied(in gp_Quaternion theQ)
        => new(w * theQ.x + x * theQ.w + y * theQ.z - z * theQ.y,
               w * theQ.y + y * theQ.w + z * theQ.x - x * theQ.z,
               w * theQ.z + z * theQ.w + x * theQ.y - y * theQ.x,
               w * theQ.w - x * theQ.x - y * theQ.y - z * theQ.z);

    public readonly double Dot(in gp_Quaternion theOther) => x * theOther.x + y * theOther.y + z * theOther.z + w * theOther.w;

    /// <summary>The rotation angle, in [-PI, PI].</summary>
    public readonly double GetRotationAngle()
    {
        if (w < 0.0)
        {
            return 2.0 * Math.Atan2(-Math.Sqrt(x * x + y * y + z * z), -w);
        }
        else
        {
            return 2.0 * Math.Atan2(Math.Sqrt(x * x + y * y + z * z), w);
        }
    }

    /// <summary>The vector rotated by this quaternion.</summary>
    public readonly gp_Vec Multiply(in gp_Vec theVec)
    {
        gp_Quaternion theQ = new(theVec.X() * w + theVec.Z() * y - theVec.Y() * z,
                                 theVec.Y() * w + theVec.X() * z - theVec.Z() * x,
                                 theVec.Z() * w + theVec.Y() * x - theVec.X() * y,
                                 theVec.X() * x + theVec.Y() * y + theVec.Z() * z);
        return new gp_Vec(w * theQ.x + x * theQ.w + y * theQ.z - z * theQ.y,
                          w * theQ.y + y * theQ.w + z * theQ.x - x * theQ.z,
                          w * theQ.z + z * theQ.w + x * theQ.y - y * theQ.x)
               * (1.0 / SquareNorm());
    }

    public static gp_Quaternion operator -(in gp_Quaternion theQ) => theQ.Negated();

    public static gp_Quaternion operator +(in gp_Quaternion theLeft, in gp_Quaternion theRight) => theLeft.Added(theRight);

    public static gp_Quaternion operator -(in gp_Quaternion theLeft, in gp_Quaternion theRight) => theLeft.Subtracted(theRight);

    public static gp_Quaternion operator *(in gp_Quaternion theLeft, in gp_Quaternion theRight) => theLeft.Multiplied(theRight);

    public static gp_Quaternion operator *(in gp_Quaternion theQ, double theScale) => theQ.Scaled(theScale);

    public static gp_Vec operator *(in gp_Quaternion theQ, in gp_Vec theVec) => theQ.Multiply(theVec);

    public readonly bool Equals(gp_Quaternion other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && w.Equals(other.w);

    public override readonly bool Equals(object? obj) => obj is gp_Quaternion other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(x, y, z, w);

    public static bool operator ==(in gp_Quaternion theLeft, in gp_Quaternion theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Quaternion theLeft, in gp_Quaternion theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Quaternion", x, y, z, w);

    /// <summary>How an Euler sequence maps onto rotation axes (OCCT's gp_EulerSequence_Parameters).</summary>
    private readonly struct EulerSequenceParameters
    {
        public readonly int i; // first rotation axis
        public readonly int j; // next axis of rotation
        public readonly int k; // third axis
        public readonly bool isOdd; // true if order of two first rotation axes is odd permutation, e.g. XZ
        public readonly bool isTwoAxes; // true if third rotation is about the same axis as first
        public readonly bool isExtrinsic; // true if rotations are made around fixed axes

        public EulerSequenceParameters(int theAx1, bool theisOdd, bool theisTwoAxes, bool theisExtrinsic)
        {
            i = theAx1;
            j = 1 + (theAx1 + (theisOdd ? 1 : 0)) % 3;
            k = 1 + (theAx1 + (theisOdd ? 0 : 1)) % 3;
            isOdd = theisOdd;
            isTwoAxes = theisTwoAxes;
            isExtrinsic = theisExtrinsic;
        }
    }

    private static EulerSequenceParameters TranslateEulerSequence(gp_EulerSequence theSeq)
    {
        const bool F = false;
        const bool T = true;

        return theSeq switch
        {
            gp_Extrinsic_XYZ => new(1, F, F, T),
            gp_Extrinsic_XZY => new(1, T, F, T),
            gp_Extrinsic_YZX => new(2, F, F, T),
            gp_Extrinsic_YXZ => new(2, T, F, T),
            gp_Extrinsic_ZXY => new(3, F, F, T),
            gp_Extrinsic_ZYX => new(3, T, F, T),
            // Conversion of intrinsic angles is made by the same code as for extrinsic,
            // using equivalence rule: intrinsic rotation is equivalent to extrinsic
            // rotation by the same angles but with inverted order of elemental rotations.
            // Swapping of angles (Alpha <-> Gamma) is done inside conversion procedure;
            // sequence of axes is inverted by setting appropriate parameters here.
            // Note that proper Euler angles (last block below) are symmetric for sequence of axes.
            gp_Intrinsic_XYZ => new(3, T, F, F),
            gp_Intrinsic_XZY => new(2, F, F, F),
            gp_Intrinsic_YZX => new(1, T, F, F),
            gp_Intrinsic_YXZ => new(3, F, F, F),
            gp_Intrinsic_ZXY => new(2, T, F, F),
            gp_Intrinsic_ZYX => new(1, F, F, F),
            gp_Extrinsic_XYX => new(1, F, T, T),
            gp_Extrinsic_XZX => new(1, T, T, T),
            gp_Extrinsic_YZY => new(2, F, T, T),
            gp_Extrinsic_YXY => new(2, T, T, T),
            gp_Extrinsic_ZXZ => new(3, F, T, T),
            gp_Extrinsic_ZYZ => new(3, T, T, T),
            gp_Intrinsic_XYX => new(1, F, T, F),
            gp_Intrinsic_XZX => new(1, T, T, F),
            gp_Intrinsic_YZY => new(2, F, T, F),
            gp_Intrinsic_YXY => new(2, T, T, F),
            gp_Intrinsic_ZXZ => new(3, F, T, F),
            gp_Intrinsic_ZYZ => new(3, T, T, F),
            gp_YawPitchRoll => new(1, F, F, F), // = Intrinsic_ZYX
            _ => new(3, F, T, F), // gp_EulerAngles = Intrinsic_ZXZ
        };
    }
}
