// Port of: src/FoundationClasses/TKMath/gp/gp_XY.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;

namespace Occt;

/// <summary>A pair of reals: the coordinates of a 2D point, vector or direction, without their meaning.</summary>
public struct gp_XY : IEquatable<gp_XY>
{
    private double x;
    private double y;

    public gp_XY(double theX, double theY)
    {
        x = theX;
        y = theY;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 2);
        if (theIndex == 1)
        {
            x = theXi;
        }
        else
        {
            y = theXi;
        }
    }

    internal void SetCoord(double theX, double theY)
    {
        x = theX;
        y = theY;
    }

    internal void SetX(double theX) => x = theX;

    internal void SetY(double theY) => y = theY;

    [UnscopedRef]
    internal ref double ChangeCoord(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 2);
        if (theIndex == 1)
        {
            return ref x;
        }

        return ref y;
    }

    internal void Add(in gp_XY theOther)
    {
        x += theOther.x;
        y += theOther.y;
    }

    internal void Divide(double theScalar)
    {
        x /= theScalar;
        y /= theScalar;
    }

    internal void Multiply(double theScalar)
    {
        x *= theScalar;
        y *= theScalar;
    }

    internal void Multiply(in gp_XY theOther)
    {
        x *= theOther.x;
        y *= theOther.y;
    }

    internal void Multiply(in gp_Mat2d theMatrix)
    {
        double aXresult = theMatrix.m00 * x + theMatrix.m01 * y;
        y = theMatrix.m10 * x + theMatrix.m11 * y;
        x = aXresult;
    }

    internal void Normalize()
    {
        double aD = Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_XY::Normalize() - vector has zero norm");
        x = x / aD;
        y = y / aD;
    }

    internal void Reverse()
    {
        x = -x;
        y = -y;
    }

    internal void SetLinearForm(double theA1, in gp_XY theXY1, double theA2, in gp_XY theXY2)
    {
        x = theA1 * theXY1.x + theA2 * theXY2.x;
        y = theA1 * theXY1.y + theA2 * theXY2.y;
    }

    internal void SetLinearForm(double theA1, in gp_XY theXY1, double theA2, in gp_XY theXY2, in gp_XY theXY3)
    {
        x = theA1 * theXY1.x + theA2 * theXY2.x + theXY3.x;
        y = theA1 * theXY1.y + theA2 * theXY2.y + theXY3.y;
    }

    internal void SetLinearForm(double theA1, in gp_XY theXY1, in gp_XY theXY2)
    {
        x = theA1 * theXY1.x + theXY2.x;
        y = theA1 * theXY1.y + theXY2.y;
    }

    internal void SetLinearForm(in gp_XY theXY1, in gp_XY theXY2)
    {
        x = theXY1.x + theXY2.x;
        y = theXY1.y + theXY2.y;
    }

    internal void Subtract(in gp_XY theOther)
    {
        x -= theOther.x;
        y -= theOther.y;
    }

    // Copies with a mutator applied (the public way to "set")

    public readonly gp_XY WithCoord(int theIndex, double theXi)
    {
        gp_XY aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_XY WithCoord(double theX, double theY) => new(theX, theY);

    public readonly gp_XY WithX(double theX) => new(theX, y);

    public readonly gp_XY WithY(double theY) => new(x, theY);

    // Queries

    public readonly double Coord(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 2);
        if (theIndex == 1)
        {
            return x;
        }

        return y;
    }

    public readonly void Coord(out double theX, out double theY)
    {
        theX = x;
        theY = y;
    }

    public readonly double X() => x;

    public readonly double Y() => y;

    public readonly double Modulus() => Math.Sqrt(SquareModulus());

    public readonly double SquareModulus() => x * x + y * y;

    public readonly bool IsEqual(in gp_XY theOther, double theTolerance)
        => (Math.Abs(x - theOther.x) < theTolerance) && (Math.Abs(y - theOther.y) < theTolerance);

    public readonly gp_XY Added(in gp_XY theOther) => new(x + theOther.X(), y + theOther.Y());

    /// <summary>The scalar cross product <c>x * other.y - y * other.x</c>.</summary>
    public readonly double Crossed(in gp_XY theOther) => x * theOther.y - y * theOther.x;

    public readonly double CrossMagnitude(in gp_XY theRight) => Math.Abs(x * theRight.y - y * theRight.x);

    public readonly double CrossSquareMagnitude(in gp_XY theRight)
    {
        double aZresult = x * theRight.y - y * theRight.x;
        return aZresult * aZresult;
    }

    public readonly gp_XY Divided(double theScalar) => new(x / theScalar, y / theScalar);

    public readonly double Dot(in gp_XY theOther) => x * theOther.x + y * theOther.y;

    public readonly gp_XY Multiplied(double theScalar) => new(x * theScalar, y * theScalar);

    public readonly gp_XY Multiplied(in gp_XY theOther) => new(x * theOther.X(), y * theOther.Y());

    public readonly gp_XY Multiplied(in gp_Mat2d theMatrix)
        => new(theMatrix.m00 * x + theMatrix.m01 * y,
               theMatrix.m10 * x + theMatrix.m11 * y);

    public readonly gp_XY Normalized()
    {
        double aD = Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_XY::Normalized() - vector has zero norm");
        return new gp_XY(x / aD, y / aD);
    }

    public readonly gp_XY Reversed() => new(-x, -y);

    public readonly gp_XY Subtracted(in gp_XY theOther) => new(x - theOther.x, y - theOther.y);

    /// <summary>Copies the coordinates into a span of at least two doubles.</summary>
    public readonly void CopyTo(Span<double> theDestination)
    {
        theDestination[0] = x;
        theDestination[1] = y;
    }

    public static gp_XY operator +(in gp_XY theLeft, in gp_XY theRight) => theLeft.Added(theRight);

    public static gp_XY operator -(in gp_XY theLeft, in gp_XY theRight) => theLeft.Subtracted(theRight);

    public static gp_XY operator -(in gp_XY theCoord) => theCoord.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in gp_XY theLeft, in gp_XY theRight) => theLeft.Dot(theRight);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in gp_XY theLeft, in gp_XY theRight) => theLeft.Crossed(theRight);

    public static gp_XY operator *(in gp_XY theCoord, double theScalar) => theCoord.Multiplied(theScalar);

    public static gp_XY operator *(double theScalar, in gp_XY theCoord) => theCoord.Multiplied(theScalar);

    public static gp_XY operator *(in gp_XY theCoord, in gp_Mat2d theMatrix) => theCoord.Multiplied(theMatrix);

    public static gp_XY operator *(in gp_Mat2d theMatrix, in gp_XY theCoord) => theCoord.Multiplied(theMatrix);

    public static gp_XY operator /(in gp_XY theCoord, double theScalar) => theCoord.Divided(theScalar);

    // Exact equality, for use as a value (OCCT compares with a tolerance: IsEqual)

    public readonly bool Equals(gp_XY other) => x.Equals(other.x) && y.Equals(other.y);

    public override readonly bool Equals(object? obj) => obj is gp_XY other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(x, y);

    public static bool operator ==(in gp_XY theLeft, in gp_XY theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_XY theLeft, in gp_XY theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_XY", x, y);
}
