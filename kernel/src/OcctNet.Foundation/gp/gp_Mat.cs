// Port of: src/FoundationClasses/TKMath/gp/gp_Mat.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Mat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Occt;

/// <summary>A 3x3 matrix. Rows and columns are numbered from 1, as in OCCT.</summary>
/// <remarks>OCCT's <c>double myMat[3][3]</c> is nine fields here; <c>myMat[i][j]</c> is <c>m{i}{j}</c>.</remarks>
[StructLayout(LayoutKind.Sequential)]
public struct gp_Mat : IEquatable<gp_Mat>
{
    internal double m00, m01, m02;
    internal double m10, m11, m12;
    internal double m20, m21, m22;

    public gp_Mat(double theA11, double theA12, double theA13,
                  double theA21, double theA22, double theA23,
                  double theA31, double theA32, double theA33)
    {
        m00 = theA11;
        m01 = theA12;
        m02 = theA13;
        m10 = theA21;
        m11 = theA22;
        m12 = theA23;
        m20 = theA31;
        m21 = theA32;
        m22 = theA33;
    }

    /// <summary>A matrix whose columns are the three triples.</summary>
    public gp_Mat(in gp_XYZ theCol1, in gp_XYZ theCol2, in gp_XYZ theCol3)
    {
        m00 = theCol1.X();
        m10 = theCol1.Y();
        m20 = theCol1.Z();
        m01 = theCol2.X();
        m11 = theCol2.Y();
        m21 = theCol2.Z();
        m02 = theCol3.X();
        m12 = theCol3.Y();
        m22 = theCol3.Z();
    }

    /// <summary>The identity matrix.</summary>
    public static gp_Mat Identity() => new(1, 0, 0, 0, 1, 0, 0, 0, 1);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCol(int theCol, in gp_XYZ theValue)
    {
        Standard_OutOfRange_Raise_if(theCol < 1 || theCol > 3, " ");
        if (theCol == 1)
        {
            m00 = theValue.X();
            m10 = theValue.Y();
            m20 = theValue.Z();
        }
        else if (theCol == 2)
        {
            m01 = theValue.X();
            m11 = theValue.Y();
            m21 = theValue.Z();
        }
        else
        {
            m02 = theValue.X();
            m12 = theValue.Y();
            m22 = theValue.Z();
        }
    }

    internal void SetCols(in gp_XYZ theCol1, in gp_XYZ theCol2, in gp_XYZ theCol3) => this = new gp_Mat(theCol1, theCol2, theCol3);

    internal void SetCross(in gp_XYZ theRef)
    {
        double X = theRef.X();
        double Y = theRef.Y();
        double Z = theRef.Z();
        m00 = m11 = m22 = 0.0;
        m01 = -Z;
        m02 = Y;
        m12 = -X;
        m10 = Z;
        m20 = -Y;
        m21 = X;
    }

    internal void SetDiagonal(double theX1, double theX2, double theX3)
    {
        m00 = theX1;
        m11 = theX2;
        m22 = theX3;
    }

    internal void SetDot(in gp_XYZ theRef)
    {
        double X = theRef.X();
        double Y = theRef.Y();
        double Z = theRef.Z();
        m00 = X * X;
        m11 = Y * Y;
        m22 = Z * Z;
        m01 = X * Y;
        m02 = X * Z;
        m12 = Y * Z;
        m10 = m01;
        m20 = m02;
        m21 = m12;
    }

    internal void SetIdentity()
    {
        m00 = m11 = m22 = 1.0;
        m01 = m02 = m10 = m12 = m20 = m21 = 0.0;
    }

    internal void SetRotation(in gp_XYZ theAxis, double theAng)
    {
        gp_XYZ aV = theAxis.Normalized();
        double A = aV.X();
        double B = aV.Y();
        double C = aV.Z();
        double aCos = Math.Cos(theAng);
        double aSin = Math.Sin(theAng);
        double aOmCos = 1.0 - aCos;
        double A2 = A * A;
        double B2 = B * B;
        double C2 = C * C;
        double AB = A * B;
        double AC = A * C;
        double BC = B * C;
        m00 = 1.0 + aOmCos * (-(B2 + C2));
        m01 = aOmCos * AB - aSin * C;
        m02 = aOmCos * AC + aSin * B;
        m10 = aOmCos * AB + aSin * C;
        m11 = 1.0 + aOmCos * (-(A2 + C2));
        m12 = aOmCos * BC - aSin * A;
        m20 = aOmCos * AC - aSin * B;
        m21 = aOmCos * BC + aSin * A;
        m22 = 1.0 + aOmCos * (-(A2 + B2));
    }

    internal void SetRow(int theRow, in gp_XYZ theValue)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3, " ");
        if (theRow == 1)
        {
            m00 = theValue.X();
            m01 = theValue.Y();
            m02 = theValue.Z();
        }
        else if (theRow == 2)
        {
            m10 = theValue.X();
            m11 = theValue.Y();
            m12 = theValue.Z();
        }
        else
        {
            m20 = theValue.X();
            m21 = theValue.Y();
            m22 = theValue.Z();
        }
    }

    internal void SetRows(in gp_XYZ theRow1, in gp_XYZ theRow2, in gp_XYZ theRow3)
    {
        m00 = theRow1.X();
        m01 = theRow1.Y();
        m02 = theRow1.Z();
        m10 = theRow2.X();
        m11 = theRow2.Y();
        m12 = theRow2.Z();
        m20 = theRow3.X();
        m21 = theRow3.Y();
        m22 = theRow3.Z();
    }

    internal void SetScale(double theS)
    {
        m00 = m11 = m22 = theS;
        m01 = m02 = m10 = m12 = m20 = m21 = 0.0;
    }

    internal void SetValue(int theRow, int theCol, double theValue) => ChangeValue(theRow, theCol) = theValue;

    [UnscopedRef]
    internal ref double ChangeValue(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3 || theCol < 1 || theCol > 3, " ");
        return ref Unsafe.Add(ref m00, (theRow - 1) * 3 + (theCol - 1));
    }

    internal void Add(in gp_Mat theOther)
    {
        m00 += theOther.m00;
        m01 += theOther.m01;
        m02 += theOther.m02;
        m10 += theOther.m10;
        m11 += theOther.m11;
        m12 += theOther.m12;
        m20 += theOther.m20;
        m21 += theOther.m21;
        m22 += theOther.m22;
    }

    internal void Divide(double theScalar)
    {
        Standard_ConstructionError_Raise_if((theScalar < 0.0 ? -theScalar : theScalar) <= gp.Resolution(), "gp_Mat : Divide by 0");
        double anUnSurScalar = 1.0 / theScalar;
        Multiply(anUnSurScalar);
    }

    internal void Invert()
    {
        double a00 = m00, a01 = m01, a02 = m02;
        double a10 = m10, a11 = m11, a12 = m12;
        double a20 = m20, a21 = m21, a22 = m22;

        double adj00 = a11 * a22 - a12 * a21;
        double adj10 = a12 * a20 - a10 * a22;
        double adj20 = a10 * a21 - a11 * a20;
        double adj01 = a02 * a21 - a01 * a22;
        double adj11 = a00 * a22 - a02 * a20;
        double adj21 = a01 * a20 - a00 * a21;
        double adj02 = a01 * a12 - a02 * a11;
        double adj12 = a02 * a10 - a00 * a12;
        double adj22 = a00 * a11 - a01 * a10;

        double aDet = a00 * adj00 + a01 * adj10 + a02 * adj20;
        Standard_ConstructionError_Raise_if(Math.Abs(aDet) <= gp.Resolution(), "gp_Mat::Invert() - matrix has zero determinant");

        double aInvDet = 1.0 / aDet;
        m00 = adj00 * aInvDet;
        m10 = adj10 * aInvDet;
        m20 = adj20 * aInvDet;
        m01 = adj01 * aInvDet;
        m11 = adj11 * aInvDet;
        m21 = adj21 * aInvDet;
        m02 = adj02 * aInvDet;
        m12 = adj12 * aInvDet;
        m22 = adj22 * aInvDet;
    }

    internal void Multiply(in gp_Mat theOther)
    {
        double aT00 = m00 * theOther.m00 + m01 * theOther.m10 + m02 * theOther.m20;
        double aT01 = m00 * theOther.m01 + m01 * theOther.m11 + m02 * theOther.m21;
        double aT02 = m00 * theOther.m02 + m01 * theOther.m12 + m02 * theOther.m22;
        double aT10 = m10 * theOther.m00 + m11 * theOther.m10 + m12 * theOther.m20;
        double aT11 = m10 * theOther.m01 + m11 * theOther.m11 + m12 * theOther.m21;
        double aT12 = m10 * theOther.m02 + m11 * theOther.m12 + m12 * theOther.m22;
        double aT20 = m20 * theOther.m00 + m21 * theOther.m10 + m22 * theOther.m20;
        double aT21 = m20 * theOther.m01 + m21 * theOther.m11 + m22 * theOther.m21;
        double aT22 = m20 * theOther.m02 + m21 * theOther.m12 + m22 * theOther.m22;
        m00 = aT00;
        m01 = aT01;
        m02 = aT02;
        m10 = aT10;
        m11 = aT11;
        m12 = aT12;
        m20 = aT20;
        m21 = aT21;
        m22 = aT22;
    }

    internal void PreMultiply(in gp_Mat theOther)
    {
        double aT00 = theOther.m00 * m00 + theOther.m01 * m10 + theOther.m02 * m20;
        double aT01 = theOther.m00 * m01 + theOther.m01 * m11 + theOther.m02 * m21;
        double aT02 = theOther.m00 * m02 + theOther.m01 * m12 + theOther.m02 * m22;
        double aT10 = theOther.m10 * m00 + theOther.m11 * m10 + theOther.m12 * m20;
        double aT11 = theOther.m10 * m01 + theOther.m11 * m11 + theOther.m12 * m21;
        double aT12 = theOther.m10 * m02 + theOther.m11 * m12 + theOther.m12 * m22;
        double aT20 = theOther.m20 * m00 + theOther.m21 * m10 + theOther.m22 * m20;
        double aT21 = theOther.m20 * m01 + theOther.m21 * m11 + theOther.m22 * m21;
        double aT22 = theOther.m20 * m02 + theOther.m21 * m12 + theOther.m22 * m22;
        m00 = aT00;
        m01 = aT01;
        m02 = aT02;
        m10 = aT10;
        m11 = aT11;
        m12 = aT12;
        m20 = aT20;
        m21 = aT21;
        m22 = aT22;
    }

    internal void Multiply(double theScalar)
    {
        m00 *= theScalar;
        m01 *= theScalar;
        m02 *= theScalar;
        m10 *= theScalar;
        m11 *= theScalar;
        m12 *= theScalar;
        m20 *= theScalar;
        m21 *= theScalar;
        m22 *= theScalar;
    }

    internal void Power(int theN)
    {
        if (theN == 0)
        {
            SetIdentity();
            return;
        }

        if (theN == 1)
        {
            return;
        }

        if (theN == -1)
        {
            Invert();
            return;
        }

        bool isNegative = theN < 0;
        if (isNegative)
        {
            Invert();
        }

        int power = isNegative ? -theN : theN;
        gp_Mat aBase = this;
        SetIdentity();
        while (power > 0)
        {
            if ((power & 1) != 0)
            {
                Multiply(aBase);
            }

            aBase.Multiply(aBase);
            power >>= 1;
        }
    }

    internal void Subtract(in gp_Mat theOther)
    {
        m00 -= theOther.m00;
        m01 -= theOther.m01;
        m02 -= theOther.m02;
        m10 -= theOther.m10;
        m11 -= theOther.m11;
        m12 -= theOther.m12;
        m20 -= theOther.m20;
        m21 -= theOther.m21;
        m22 -= theOther.m22;
    }

    internal void Transpose()
    {
        (m01, m10) = (m10, m01);
        (m02, m20) = (m20, m02);
        (m12, m21) = (m21, m12);
    }

    // Copies with a mutator applied

    public readonly gp_Mat WithCol(int theCol, in gp_XYZ theValue)
    {
        gp_Mat aCopy = this;
        aCopy.SetCol(theCol, theValue);
        return aCopy;
    }

    public readonly gp_Mat WithCols(in gp_XYZ theCol1, in gp_XYZ theCol2, in gp_XYZ theCol3) => new(theCol1, theCol2, theCol3);

    public readonly gp_Mat WithCross(in gp_XYZ theRef)
    {
        gp_Mat aCopy = this;
        aCopy.SetCross(theRef);
        return aCopy;
    }

    public readonly gp_Mat WithDiagonal(double theX1, double theX2, double theX3)
    {
        gp_Mat aCopy = this;
        aCopy.SetDiagonal(theX1, theX2, theX3);
        return aCopy;
    }

    public readonly gp_Mat WithDot(in gp_XYZ theRef)
    {
        gp_Mat aCopy = this;
        aCopy.SetDot(theRef);
        return aCopy;
    }

    public readonly gp_Mat WithIdentity() => Identity();

    public readonly gp_Mat WithRotation(in gp_XYZ theAxis, double theAng)
    {
        gp_Mat aCopy = this;
        aCopy.SetRotation(theAxis, theAng);
        return aCopy;
    }

    public readonly gp_Mat WithRow(int theRow, in gp_XYZ theValue)
    {
        gp_Mat aCopy = this;
        aCopy.SetRow(theRow, theValue);
        return aCopy;
    }

    public readonly gp_Mat WithRows(in gp_XYZ theRow1, in gp_XYZ theRow2, in gp_XYZ theRow3)
    {
        gp_Mat aCopy = this;
        aCopy.SetRows(theRow1, theRow2, theRow3);
        return aCopy;
    }

    public readonly gp_Mat WithScale(double theS)
    {
        gp_Mat aCopy = this;
        aCopy.SetScale(theS);
        return aCopy;
    }

    public readonly gp_Mat WithValue(int theRow, int theCol, double theValue)
    {
        gp_Mat aCopy = this;
        aCopy.SetValue(theRow, theCol, theValue);
        return aCopy;
    }

    // Queries

    public readonly gp_XYZ Column(int theCol)
    {
        Standard_OutOfRange_Raise_if(theCol < 1 || theCol > 3, "gp_Mat::Column() - wrong index");
        if (theCol == 1)
        {
            return new gp_XYZ(m00, m10, m20);
        }

        if (theCol == 2)
        {
            return new gp_XYZ(m01, m11, m21);
        }

        return new gp_XYZ(m02, m12, m22);
    }

    public readonly double Determinant()
    {
        double a00 = m00, a01 = m01, a02 = m02;
        double a10 = m10, a11 = m11, a12 = m12;
        double a20 = m20, a21 = m21, a22 = m22;
        return a00 * (a11 * a22 - a21 * a12) - a01 * (a10 * a22 - a20 * a12) + a02 * (a10 * a21 - a20 * a11);
    }

    public readonly gp_XYZ Diagonal() => new(m00, m11, m22);

    public readonly gp_XYZ Row(int theRow)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3, "gp_Mat::Row() - wrong index");
        if (theRow == 1)
        {
            return new gp_XYZ(m00, m01, m02);
        }

        if (theRow == 2)
        {
            return new gp_XYZ(m10, m11, m12);
        }

        return new gp_XYZ(m20, m21, m22);
    }

    public readonly double Value(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3 || theCol < 1 || theCol > 3, " ");
        return Unsafe.Add(ref Unsafe.AsRef(in m00), (theRow - 1) * 3 + (theCol - 1));
    }

    /// <summary>The element at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    public readonly double this[int theRow, int theCol] => Value(theRow, theCol);

    public readonly bool IsSingular()
    {
        double aDet = Determinant();
        return (aDet < 0.0 ? -aDet : aDet) <= gp.Resolution();
    }

    public readonly gp_Mat Added(in gp_Mat theOther)
    {
        gp_Mat aNewMat = this;
        aNewMat.Add(theOther);
        return aNewMat;
    }

    public readonly gp_Mat Divided(double theScalar)
    {
        gp_Mat aNewMat = this;
        aNewMat.Divide(theScalar);
        return aNewMat;
    }

    public readonly gp_Mat Inverted()
    {
        gp_Mat aNewMat = this;
        aNewMat.Invert();
        return aNewMat;
    }

    public readonly gp_Mat Multiplied(in gp_Mat theOther)
    {
        gp_Mat aNewMat = this;
        aNewMat.Multiply(theOther);
        return aNewMat;
    }

    public readonly gp_Mat Multiplied(double theScalar)
    {
        gp_Mat aNewMat = this;
        aNewMat.Multiply(theScalar);
        return aNewMat;
    }

    public readonly gp_Mat Powered(int theN)
    {
        gp_Mat aMatN = this;
        aMatN.Power(theN);
        return aMatN;
    }

    public readonly gp_Mat Subtracted(in gp_Mat theOther)
    {
        gp_Mat aNewMat = this;
        aNewMat.Subtract(theOther);
        return aNewMat;
    }

    public readonly gp_Mat Transposed()
    {
        gp_Mat aNewMat = this;
        aNewMat.Transpose();
        return aNewMat;
    }

    /// <summary>The nine elements, row by row.</summary>
    public readonly void CopyTo(Span<double> theDestination)
        => MemoryMarshal.CreateReadOnlySpan(ref Unsafe.AsRef(in m00), 9).CopyTo(theDestination);

    public static gp_Mat operator +(in gp_Mat theLeft, in gp_Mat theRight) => theLeft.Added(theRight);

    public static gp_Mat operator -(in gp_Mat theLeft, in gp_Mat theRight) => theLeft.Subtracted(theRight);

    public static gp_Mat operator *(in gp_Mat theLeft, in gp_Mat theRight) => theLeft.Multiplied(theRight);

    public static gp_Mat operator *(in gp_Mat theMat, double theScalar) => theMat.Multiplied(theScalar);

    public static gp_Mat operator *(double theScalar, in gp_Mat theMat) => theMat.Multiplied(theScalar);

    public static gp_Mat operator /(in gp_Mat theMat, double theScalar) => theMat.Divided(theScalar);

    public readonly bool Equals(gp_Mat other)
        => m00.Equals(other.m00) && m01.Equals(other.m01) && m02.Equals(other.m02)
           && m10.Equals(other.m10) && m11.Equals(other.m11) && m12.Equals(other.m12)
           && m20.Equals(other.m20) && m21.Equals(other.m21) && m22.Equals(other.m22);

    public override readonly bool Equals(object? obj) => obj is gp_Mat other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(HashCode.Combine(m00, m01, m02), HashCode.Combine(m10, m11, m12), HashCode.Combine(m20, m21, m22));

    public static bool operator ==(in gp_Mat theLeft, in gp_Mat theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Mat theLeft, in gp_Mat theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Mat", m00, m01, m02, m10, m11, m12, m20, m21, m22);
}
