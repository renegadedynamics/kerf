// Port of: src/FoundationClasses/TKMath/gp/gp_Mat2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Mat2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Occt;

/// <summary>A 2x2 matrix. Rows and columns are numbered from 1, as in OCCT.</summary>
/// <remarks>OCCT's <c>double myMat[2][2]</c> is four fields here; <c>myMat[i][j]</c> is <c>m{i}{j}</c>.</remarks>
[StructLayout(LayoutKind.Sequential)]
public struct gp_Mat2d : IEquatable<gp_Mat2d>
{
    internal double m00, m01;
    internal double m10, m11;

    /// <summary>A matrix whose columns are the two pairs.</summary>
    public gp_Mat2d(in gp_XY theCol1, in gp_XY theCol2)
    {
        m00 = theCol1.X();
        m10 = theCol1.Y();
        m01 = theCol2.X();
        m11 = theCol2.Y();
    }

    /// <summary>The identity matrix.</summary>
    public static gp_Mat2d Identity()
    {
        gp_Mat2d aMat = new();
        aMat.SetIdentity();
        return aMat;
    }

    /// <summary>The matrix of the rotation by an angle in radians.</summary>
    public static gp_Mat2d Rotation(double theAng)
    {
        gp_Mat2d aMat = new();
        aMat.SetRotation(theAng);
        return aMat;
    }

    /// <summary>The matrix of the scale by a factor.</summary>
    public static gp_Mat2d Scale(double theS)
    {
        gp_Mat2d aMat = new();
        aMat.SetScale(theS);
        return aMat;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCol(int theCol, in gp_XY theValue)
    {
        Standard_OutOfRange_Raise_if(theCol < 1 || theCol > 2, "gp_Mat2d::SetCol() - invalid index");
        if (theCol == 1)
        {
            m00 = theValue.X();
            m10 = theValue.Y();
        }
        else
        {
            m01 = theValue.X();
            m11 = theValue.Y();
        }
    }

    internal void SetCols(in gp_XY theCol1, in gp_XY theCol2)
    {
        m00 = theCol1.X();
        m10 = theCol1.Y();
        m01 = theCol2.X();
        m11 = theCol2.Y();
    }

    internal void SetDiagonal(double theX1, double theX2)
    {
        m00 = theX1;
        m11 = theX2;
    }

    internal void SetIdentity()
    {
        m00 = m11 = 1.0;
        m01 = m10 = 0.0;
    }

    internal void SetRotation(double theAng)
    {
        double aSinA = Math.Sin(theAng);
        double aCosA = Math.Cos(theAng);
        m00 = m11 = aCosA;
        m01 = -aSinA;
        m10 = aSinA;
    }

    internal void SetRow(int theRow, in gp_XY theValue)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2, "gp_Mat2d::SetRow() - invalid index");
        if (theRow == 1)
        {
            m00 = theValue.X();
            m01 = theValue.Y();
        }
        else
        {
            m10 = theValue.X();
            m11 = theValue.Y();
        }
    }

    internal void SetRows(in gp_XY theRow1, in gp_XY theRow2)
    {
        m00 = theRow1.X();
        m01 = theRow1.Y();
        m10 = theRow2.X();
        m11 = theRow2.Y();
    }

    internal void SetScale(double theS)
    {
        m00 = m11 = theS;
        m01 = m10 = 0.0;
    }

    internal void SetValue(int theRow, int theCol, double theValue) => ChangeValue(theRow, theCol) = theValue;

    [UnscopedRef]
    internal ref double ChangeValue(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2 || theCol < 1 || theCol > 2, " ");
        return ref Unsafe.Add(ref m00, (theRow - 1) * 2 + (theCol - 1));
    }

    internal void Add(in gp_Mat2d theOther)
    {
        m00 += theOther.m00;
        m01 += theOther.m01;
        m10 += theOther.m10;
        m11 += theOther.m11;
    }

    internal void Divide(double theScalar)
    {
        m00 /= theScalar;
        m01 /= theScalar;
        m10 /= theScalar;
        m11 /= theScalar;
    }

    internal void Invert()
    {
        double aNewMat00 = m11;
        double aNewMat01 = -m01;
        double aNewMat10 = -m10;
        double aNewMat11 = m00;
        double aDet = aNewMat00 * aNewMat11 - aNewMat01 * aNewMat10;
        Standard_ConstructionError_Raise_if(Math.Abs(aDet) <= gp.Resolution(), "gp_Mat2d::Invert() - matrix has zero determinant");
        aDet = 1.0 / aDet;
        m00 = aNewMat00 * aDet;
        m10 = aNewMat10 * aDet;
        m01 = aNewMat01 * aDet;
        m11 = aNewMat11 * aDet;
    }

    internal void Multiply(in gp_Mat2d theOther)
    {
        double aT00 = m00 * theOther.m00 + m01 * theOther.m10;
        double aT10 = m10 * theOther.m00 + m11 * theOther.m10;
        m01 = m00 * theOther.m01 + m01 * theOther.m11;
        m11 = m10 * theOther.m01 + m11 * theOther.m11;
        m00 = aT00;
        m10 = aT10;
    }

    internal void PreMultiply(in gp_Mat2d theOther)
    {
        double aT00 = theOther.m00 * m00 + theOther.m01 * m10;
        m10 = theOther.m10 * m00 + theOther.m11 * m10;
        double aT01 = theOther.m00 * m01 + theOther.m01 * m11;
        m11 = theOther.m10 * m01 + theOther.m11 * m11;
        m00 = aT00;
        m01 = aT01;
    }

    internal void Multiply(double theScalar)
    {
        m00 *= theScalar;
        m01 *= theScalar;
        m10 *= theScalar;
        m11 *= theScalar;
    }

    internal void Power(int theN)
    {
        if (theN == 1)
        {
        }
        else if (theN == 0)
        {
            SetIdentity();
        }
        else if (theN == -1)
        {
            Invert();
        }
        else
        {
            if (theN < 0)
            {
                Invert();
            }

            int Npower = Math.Abs(theN) - 1;
            gp_Mat2d aTemp = this;
            for (;;)
            {
                if (IsOdd(Npower))
                {
                    Multiply(aTemp);
                }

                if (Npower == 1)
                {
                    break;
                }

                // OCCT squares aTemp in place: Multiply reads theOther.myMat[0][1] after writing it (aliasing,
                // docs/occt-issues.md); `in` passes aTemp by reference, so the same result as OCCT
                aTemp.Multiply(aTemp);
                Npower = Npower / 2;
            }
        }
    }

    internal void Subtract(in gp_Mat2d theOther)
    {
        m00 -= theOther.m00;
        m01 -= theOther.m01;
        m10 -= theOther.m10;
        m11 -= theOther.m11;
    }

    internal void Transpose()
    {
        double aTemp = m01;
        m01 = m10;
        m10 = aTemp;
    }

    // Copies with a mutator applied

    public readonly gp_Mat2d WithCol(int theCol, in gp_XY theValue)
    {
        gp_Mat2d aCopy = this;
        aCopy.SetCol(theCol, theValue);
        return aCopy;
    }

    public readonly gp_Mat2d WithCols(in gp_XY theCol1, in gp_XY theCol2) => new(theCol1, theCol2);

    public readonly gp_Mat2d WithDiagonal(double theX1, double theX2)
    {
        gp_Mat2d aCopy = this;
        aCopy.SetDiagonal(theX1, theX2);
        return aCopy;
    }

    public readonly gp_Mat2d WithIdentity() => Identity();

    public readonly gp_Mat2d WithRotation(double theAng) => Rotation(theAng);

    public readonly gp_Mat2d WithRow(int theRow, in gp_XY theValue)
    {
        gp_Mat2d aCopy = this;
        aCopy.SetRow(theRow, theValue);
        return aCopy;
    }

    public readonly gp_Mat2d WithRows(in gp_XY theRow1, in gp_XY theRow2)
    {
        gp_Mat2d aCopy = this;
        aCopy.SetRows(theRow1, theRow2);
        return aCopy;
    }

    public readonly gp_Mat2d WithScale(double theS) => Scale(theS);

    public readonly gp_Mat2d WithValue(int theRow, int theCol, double theValue)
    {
        gp_Mat2d aCopy = this;
        aCopy.SetValue(theRow, theCol, theValue);
        return aCopy;
    }

    // Queries

    public readonly gp_XY Column(int theCol)
    {
        Standard_OutOfRange_Raise_if(theCol < 1 || theCol > 2, "gp_Mat2d::Column() - invalid index");
        if (theCol == 1)
        {
            return new gp_XY(m00, m10);
        }

        return new gp_XY(m01, m11);
    }

    public readonly double Determinant() => m00 * m11 - m10 * m01;

    public readonly gp_XY Diagonal() => new(m00, m11);

    public readonly gp_XY Row(int theRow)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2, "gp_Mat2d::Row() - invalid index");
        if (theRow == 1)
        {
            return new gp_XY(m00, m01);
        }

        return new gp_XY(m10, m11);
    }

    public readonly double Value(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2 || theCol < 1 || theCol > 2, " ");
        return Unsafe.Add(ref Unsafe.AsRef(in m00), (theRow - 1) * 2 + (theCol - 1));
    }

    /// <summary>The element at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    public readonly double this[int theRow, int theCol] => Value(theRow, theCol);

    public readonly bool IsSingular()
    {
        double aDet = Determinant();
        if (aDet < 0)
        {
            aDet = -aDet;
        }

        return aDet <= gp.Resolution();
    }

    public readonly gp_Mat2d Added(in gp_Mat2d theOther)
    {
        gp_Mat2d aNewMat2d = new();
        aNewMat2d.m00 = m00 + theOther.m00;
        aNewMat2d.m01 = m01 + theOther.m01;
        aNewMat2d.m10 = m10 + theOther.m10;
        aNewMat2d.m11 = m11 + theOther.m11;
        return aNewMat2d;
    }

    public readonly gp_Mat2d Divided(double theScalar)
    {
        gp_Mat2d aNewMat2d = new();
        aNewMat2d.m00 = m00 / theScalar;
        aNewMat2d.m01 = m01 / theScalar;
        aNewMat2d.m10 = m10 / theScalar;
        aNewMat2d.m11 = m11 / theScalar;
        return aNewMat2d;
    }

    public readonly gp_Mat2d Inverted()
    {
        gp_Mat2d aNewMat = this;
        aNewMat.Invert();
        return aNewMat;
    }

    public readonly gp_Mat2d Multiplied(in gp_Mat2d theOther)
    {
        gp_Mat2d aNewMat2d = this;
        aNewMat2d.Multiply(theOther);
        return aNewMat2d;
    }

    public readonly gp_Mat2d Multiplied(double theScalar)
    {
        gp_Mat2d aNewMat2d = new();
        aNewMat2d.m00 = m00 * theScalar;
        aNewMat2d.m01 = m01 * theScalar;
        aNewMat2d.m10 = m10 * theScalar;
        aNewMat2d.m11 = m11 * theScalar;
        return aNewMat2d;
    }

    public readonly gp_Mat2d PreMultiplied(in gp_Mat2d theOther)
    {
        gp_Mat2d aNewMat2d = this;
        aNewMat2d.PreMultiply(theOther);
        return aNewMat2d;
    }

    public readonly gp_Mat2d Powered(int theN)
    {
        gp_Mat2d aMat2dN = this;
        aMat2dN.Power(theN);
        return aMat2dN;
    }

    public readonly gp_Mat2d Subtracted(in gp_Mat2d theOther)
    {
        gp_Mat2d aNewMat2d = new();
        aNewMat2d.m00 = m00 - theOther.m00;
        aNewMat2d.m01 = m01 - theOther.m01;
        aNewMat2d.m10 = m10 - theOther.m10;
        aNewMat2d.m11 = m11 - theOther.m11;
        return aNewMat2d;
    }

    public readonly gp_Mat2d Transposed()
    {
        gp_Mat2d aNewMat2d = new();
        aNewMat2d.m10 = m01;
        aNewMat2d.m01 = m10;
        aNewMat2d.m00 = m00;
        aNewMat2d.m11 = m11;
        return aNewMat2d;
    }

    /// <summary>The four elements, row by row.</summary>
    public readonly void CopyTo(Span<double> theDestination)
        => MemoryMarshal.CreateReadOnlySpan(ref Unsafe.AsRef(in m00), 4).CopyTo(theDestination);

    public static gp_Mat2d operator +(in gp_Mat2d theLeft, in gp_Mat2d theRight) => theLeft.Added(theRight);

    public static gp_Mat2d operator -(in gp_Mat2d theLeft, in gp_Mat2d theRight) => theLeft.Subtracted(theRight);

    public static gp_Mat2d operator *(in gp_Mat2d theLeft, in gp_Mat2d theRight) => theLeft.Multiplied(theRight);

    public static gp_Mat2d operator *(in gp_Mat2d theMat, double theScalar) => theMat.Multiplied(theScalar);

    public static gp_Mat2d operator *(double theScalar, in gp_Mat2d theMat) => theMat.Multiplied(theScalar);

    public static gp_Mat2d operator /(in gp_Mat2d theMat, double theScalar) => theMat.Divided(theScalar);

    public readonly bool Equals(gp_Mat2d other)
        => m00.Equals(other.m00) && m01.Equals(other.m01) && m10.Equals(other.m10) && m11.Equals(other.m11);

    public override readonly bool Equals(object? obj) => obj is gp_Mat2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(m00, m01, m10, m11);

    public static bool operator ==(in gp_Mat2d theLeft, in gp_Mat2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Mat2d theLeft, in gp_Mat2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Mat2d", m00, m01, m10, m11);
}
