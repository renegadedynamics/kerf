// Port of: src/FoundationClasses/TKMath/gp/gp_XYZ.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_XYZ.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>A triple of reals: the coordinates of a point, vector or direction, without their meaning.</summary>
public struct gp_XYZ : IEquatable<gp_XYZ>
{
    private double x;
    private double y;
    private double z;

    public gp_XYZ(double theX, double theY, double theZ)
    {
        x = theX;
        y = theY;
        z = theZ;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(double theX, double theY, double theZ)
    {
        x = theX;
        y = theY;
        z = theZ;
    }

    internal void SetCoord(int theIndex, double theXi)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3);
        if (theIndex == 1)
        {
            x = theXi;
        }
        else if (theIndex == 2)
        {
            y = theXi;
        }
        else
        {
            z = theXi;
        }
    }

    internal void SetX(double theX) => x = theX;

    internal void SetY(double theY) => y = theY;

    internal void SetZ(double theZ) => z = theZ;

    [UnscopedRef]
    internal ref double ChangeCoord(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3);
        if (theIndex == 1)
        {
            return ref x;
        }
        else if (theIndex == 2)
        {
            return ref y;
        }

        return ref z;
    }

    internal void Add(in gp_XYZ theOther)
    {
        x += theOther.x;
        y += theOther.y;
        z += theOther.z;
    }

    internal void Cross(in gp_XYZ theRight)
    {
        double aXresult = y * theRight.z - z * theRight.y;
        double aYresult = z * theRight.x - x * theRight.z;
        z = x * theRight.y - y * theRight.x;
        x = aXresult;
        y = aYresult;
    }

    internal void CrossCross(in gp_XYZ theCoord1, in gp_XYZ theCoord2)
    {
        double aCrossX = theCoord1.y * theCoord2.z - theCoord1.z * theCoord2.y;
        double aCrossY = theCoord1.z * theCoord2.x - theCoord1.x * theCoord2.z;
        double aCrossZ = theCoord1.x * theCoord2.y - theCoord1.y * theCoord2.x;
        double aXresult = y * aCrossZ - z * aCrossY;
        double aYresult = z * aCrossX - x * aCrossZ;
        z = x * aCrossY - y * aCrossX;
        x = aXresult;
        y = aYresult;
    }

    internal void Divide(double theScalar)
    {
        x /= theScalar;
        y /= theScalar;
        z /= theScalar;
    }

    internal void Multiply(double theScalar)
    {
        x *= theScalar;
        y *= theScalar;
        z *= theScalar;
    }

    internal void Multiply(in gp_XYZ theOther)
    {
        x *= theOther.x;
        y *= theOther.y;
        z *= theOther.z;
    }

    internal void Multiply(in gp_Mat theMatrix)
    {
        double aOrigX = x;
        double aOrigY = y;
        double aOrigZ = z;
        x = theMatrix.m00 * aOrigX + theMatrix.m01 * aOrigY + theMatrix.m02 * aOrigZ;
        y = theMatrix.m10 * aOrigX + theMatrix.m11 * aOrigY + theMatrix.m12 * aOrigZ;
        z = theMatrix.m20 * aOrigX + theMatrix.m21 * aOrigY + theMatrix.m22 * aOrigZ;
    }

    internal void Normalize()
    {
        double aD = Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_XYZ::Normalize() - vector has zero norm");
        x = x / aD;
        y = y / aD;
        z = z / aD;
    }

    internal void Reverse()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    internal void Subtract(in gp_XYZ theOther)
    {
        x -= theOther.x;
        y -= theOther.y;
        z -= theOther.z;
    }

    internal void SetLinearForm(double theA1, in gp_XYZ theXYZ1, double theA2, in gp_XYZ theXYZ2, double theA3, in gp_XYZ theXYZ3, in gp_XYZ theXYZ4)
    {
        x = theA1 * theXYZ1.x + theA2 * theXYZ2.x + theA3 * theXYZ3.x + theXYZ4.x;
        y = theA1 * theXYZ1.y + theA2 * theXYZ2.y + theA3 * theXYZ3.y + theXYZ4.y;
        z = theA1 * theXYZ1.z + theA2 * theXYZ2.z + theA3 * theXYZ3.z + theXYZ4.z;
    }

    internal void SetLinearForm(double theA1, in gp_XYZ theXYZ1, double theA2, in gp_XYZ theXYZ2, double theA3, in gp_XYZ theXYZ3)
    {
        x = theA1 * theXYZ1.x + theA2 * theXYZ2.x + theA3 * theXYZ3.x;
        y = theA1 * theXYZ1.y + theA2 * theXYZ2.y + theA3 * theXYZ3.y;
        z = theA1 * theXYZ1.z + theA2 * theXYZ2.z + theA3 * theXYZ3.z;
    }

    internal void SetLinearForm(double theA1, in gp_XYZ theXYZ1, double theA2, in gp_XYZ theXYZ2, in gp_XYZ theXYZ3)
    {
        x = theA1 * theXYZ1.x + theA2 * theXYZ2.x + theXYZ3.x;
        y = theA1 * theXYZ1.y + theA2 * theXYZ2.y + theXYZ3.y;
        z = theA1 * theXYZ1.z + theA2 * theXYZ2.z + theXYZ3.z;
    }

    internal void SetLinearForm(double theA1, in gp_XYZ theXYZ1, double theA2, in gp_XYZ theXYZ2)
    {
        x = theA1 * theXYZ1.x + theA2 * theXYZ2.x;
        y = theA1 * theXYZ1.y + theA2 * theXYZ2.y;
        z = theA1 * theXYZ1.z + theA2 * theXYZ2.z;
    }

    internal void SetLinearForm(double theA1, in gp_XYZ theXYZ1, in gp_XYZ theXYZ2)
    {
        x = theA1 * theXYZ1.x + theXYZ2.x;
        y = theA1 * theXYZ1.y + theXYZ2.y;
        z = theA1 * theXYZ1.z + theXYZ2.z;
    }

    internal void SetLinearForm(in gp_XYZ theXYZ1, in gp_XYZ theXYZ2)
    {
        x = theXYZ1.x + theXYZ2.x;
        y = theXYZ1.y + theXYZ2.y;
        z = theXYZ1.z + theXYZ2.z;
    }

    // Copies with a mutator applied (the public way to "set")

    public readonly gp_XYZ WithCoord(double theX, double theY, double theZ) => new(theX, theY, theZ);

    public readonly gp_XYZ WithCoord(int theIndex, double theXi)
    {
        gp_XYZ aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_XYZ WithX(double theX) => new(theX, y, z);

    public readonly gp_XYZ WithY(double theY) => new(x, theY, z);

    public readonly gp_XYZ WithZ(double theZ) => new(x, y, theZ);

    // Queries

    public readonly double Coord(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3);
        if (theIndex == 1)
        {
            return x;
        }
        else if (theIndex == 2)
        {
            return y;
        }

        return z;
    }

    public readonly void Coord(out double theX, out double theY, out double theZ)
    {
        theX = x;
        theY = y;
        theZ = z;
    }

    public readonly double X() => x;

    public readonly double Y() => y;

    public readonly double Z() => z;

    public readonly double Modulus() => Math.Sqrt(x * x + y * y + z * z);

    public readonly double SquareModulus() => x * x + y * y + z * z;

    public readonly bool IsEqual(in gp_XYZ theOther, double theTolerance)
        => Math.Abs(x - theOther.x) < theTolerance
           && Math.Abs(y - theOther.y) < theTolerance
           && Math.Abs(z - theOther.z) < theTolerance;

    public readonly gp_XYZ Added(in gp_XYZ theOther) => new(x + theOther.x, y + theOther.y, z + theOther.z);

    public readonly gp_XYZ Crossed(in gp_XYZ theOther)
        => new(y * theOther.z - z * theOther.y, z * theOther.x - x * theOther.z, x * theOther.y - y * theOther.x);

    public readonly double CrossMagnitude(in gp_XYZ theRight) => Math.Sqrt(CrossSquareMagnitude(theRight));

    public readonly double CrossSquareMagnitude(in gp_XYZ theRight)
    {
        double aXresult = y * theRight.z - z * theRight.y;
        double aYresult = z * theRight.x - x * theRight.z;
        double aZresult = x * theRight.y - y * theRight.x;
        return aXresult * aXresult + aYresult * aYresult + aZresult * aZresult;
    }

    public readonly gp_XYZ CrossCrossed(in gp_XYZ theCoord1, in gp_XYZ theCoord2)
    {
        gp_XYZ aCoord0 = this;
        aCoord0.CrossCross(theCoord1, theCoord2);
        return aCoord0;
    }

    public readonly gp_XYZ Divided(double theScalar) => new(x / theScalar, y / theScalar, z / theScalar);

    public readonly double Dot(in gp_XYZ theOther) => x * theOther.x + y * theOther.y + z * theOther.z;

    public readonly double DotCross(in gp_XYZ theCoord1, in gp_XYZ theCoord2)
    {
        double aXresult = theCoord1.y * theCoord2.z - theCoord1.z * theCoord2.y;
        double anYresult = theCoord1.z * theCoord2.x - theCoord1.x * theCoord2.z;
        double aZresult = theCoord1.x * theCoord2.y - theCoord1.y * theCoord2.x;
        return x * aXresult + y * anYresult + z * aZresult;
    }

    public readonly gp_XYZ Multiplied(double theScalar) => new(x * theScalar, y * theScalar, z * theScalar);

    public readonly gp_XYZ Multiplied(in gp_XYZ theOther) => new(x * theOther.x, y * theOther.y, z * theOther.z);

    public readonly gp_XYZ Multiplied(in gp_Mat theMatrix)
        => new(theMatrix.m00 * x + theMatrix.m01 * y + theMatrix.m02 * z,
               theMatrix.m10 * x + theMatrix.m11 * y + theMatrix.m12 * z,
               theMatrix.m20 * x + theMatrix.m21 * y + theMatrix.m22 * z);

    public readonly gp_XYZ Normalized()
    {
        double aD = Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_XYZ::Normalized() - vector has zero norm");
        return new gp_XYZ(x / aD, y / aD, z / aD);
    }

    public readonly gp_XYZ Reversed() => new(-x, -y, -z);

    public readonly gp_XYZ Subtracted(in gp_XYZ theOther) => new(x - theOther.x, y - theOther.y, z - theOther.z);

    /// <summary>Copies the coordinates into a span of at least three doubles.</summary>
    public readonly void CopyTo(Span<double> theDestination)
    {
        theDestination[0] = x;
        theDestination[1] = y;
        theDestination[2] = z;
    }

    public static gp_XYZ operator +(in gp_XYZ theLeft, in gp_XYZ theRight) => theLeft.Added(theRight);

    public static gp_XYZ operator -(in gp_XYZ theLeft, in gp_XYZ theRight) => theLeft.Subtracted(theRight);

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in gp_XYZ theLeft, in gp_XYZ theRight) => theLeft.Dot(theRight);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static gp_XYZ operator ^(in gp_XYZ theLeft, in gp_XYZ theRight) => theLeft.Crossed(theRight);

    public static gp_XYZ operator *(in gp_XYZ theCoord, double theScalar) => theCoord.Multiplied(theScalar);

    public static gp_XYZ operator *(double theScalar, in gp_XYZ theCoord) => theCoord.Multiplied(theScalar);

    public static gp_XYZ operator *(in gp_XYZ theCoord, in gp_Mat theMatrix) => theCoord.Multiplied(theMatrix);

    public static gp_XYZ operator *(in gp_Mat theMatrix, in gp_XYZ theCoord) => theCoord.Multiplied(theMatrix);

    public static gp_XYZ operator /(in gp_XYZ theCoord, double theScalar) => theCoord.Divided(theScalar);

    // Exact equality, for use as a value (OCCT compares with a tolerance: IsEqual)

    public readonly bool Equals(gp_XYZ other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

    public override readonly bool Equals(object? obj) => obj is gp_XYZ other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(x, y, z);

    public static bool operator ==(in gp_XYZ theLeft, in gp_XYZ theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_XYZ theLeft, in gp_XYZ theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_XYZ", x, y, z);
}
