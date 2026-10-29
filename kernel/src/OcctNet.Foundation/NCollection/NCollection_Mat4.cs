// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Mat4.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;

/// <summary>A 4x4 matrix of numbers (float, double or int), stored column by column as OCCT stores it.</summary>
/// <remarks>
/// OCCT's default constructor makes the identity, and so does <c>new NCollection_Mat4&lt;T&gt;()</c>;
/// <c>default(NCollection_Mat4&lt;T&gt;)</c> is the zero matrix.
/// </remarks>
public partial struct NCollection_Mat4<T> : IEquatable<NCollection_Mat4<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer16<T> myMat;

    public static int Rows() => 4;

    public static int Cols() => 4;

    /// <summary>The identity matrix.</summary>
    public static NCollection_Mat4<T> Identity() => new();

    /// <summary>The zero matrix.</summary>
    public static NCollection_Mat4<T> Zero()
    {
        NCollection_Mat4<T> aMat = new();
        aMat.InitZero();
        return aMat;
    }

    /// <summary>The identity matrix.</summary>
    public NCollection_Mat4() => InitIdentity();

    /// <summary>Converts a matrix of another element type (OCCT's converting constructor).</summary>
    public static NCollection_Mat4<T> From<TOther>(in NCollection_Mat4<TOther> theOtherMat3) where TOther : unmanaged, INumber<TOther>
    {
        NCollection_Mat4<T> aMat = new();
        aMat.ConvertFrom(theOtherMat3);
        return aMat;
    }

    /// <summary>The element at a (0-based) row and column.</summary>
    public readonly T GetValue(int theRow, int theCol) => myMat[theCol * 4 + theRow];

    /// <summary>The element at a (0-based) row and column (OCCT's <c>operator()</c>).</summary>
    public T this[int theRow, int theCol]
    {
        readonly get => GetValue(theRow, theCol);
        internal set => SetValue(theRow, theCol, value);
    }

    public readonly NCollection_Vec4<T> GetRow(int theRow)
        => new(GetValue(theRow, 0), GetValue(theRow, 1), GetValue(theRow, 2), GetValue(theRow, 3));

    public readonly NCollection_Vec4<T> GetColumn(int theCol)
        => new(GetValue(0, theCol), GetValue(1, theCol), GetValue(2, theCol), GetValue(3, theCol));

    public readonly NCollection_Vec4<T> GetDiagonal() => new(GetValue(0, 0), GetValue(1, 1), GetValue(2, 2), GetValue(3, 3));

    /// <summary>The upper-left 3x3 part.</summary>
    public readonly NCollection_Mat3<T> GetMat3()
    {
        NCollection_Mat3<T> aMat = new();
        aMat.SetColumn(0, GetColumn(0).xyz());
        aMat.SetColumn(1, GetColumn(1).xyz());
        aMat.SetColumn(2, GetColumn(2).xyz());
        return aMat;
    }

    // Mutators (internal: see docs/translation-rules.md)

    [UnscopedRef]
    internal ref T ChangeValue(int theRow, int theCol) => ref myMat[theCol * 4 + theRow];

    internal void SetValue(int theRow, int theCol, T theValue) => myMat[theCol * 4 + theRow] = theValue;

    internal void SetRow(int theRow, in NCollection_Vec3<T> theVec)
    {
        SetValue(theRow, 0, theVec.x());
        SetValue(theRow, 1, theVec.y());
        SetValue(theRow, 2, theVec.z());
    }

    internal void SetRow(int theRow, in NCollection_Vec4<T> theVec)
    {
        SetValue(theRow, 0, theVec.x());
        SetValue(theRow, 1, theVec.y());
        SetValue(theRow, 2, theVec.z());
        SetValue(theRow, 3, theVec.w());
    }

    internal void SetColumn(int theCol, in NCollection_Vec3<T> theVec)
    {
        SetValue(0, theCol, theVec.x());
        SetValue(1, theCol, theVec.y());
        SetValue(2, theCol, theVec.z());
    }

    internal void SetColumn(int theCol, in NCollection_Vec4<T> theVec)
    {
        SetValue(0, theCol, theVec.x());
        SetValue(1, theCol, theVec.y());
        SetValue(2, theCol, theVec.z());
        SetValue(3, theCol, theVec.w());
    }

    internal void SetDiagonal(in NCollection_Vec3<T> theVec)
    {
        SetValue(0, 0, theVec.x());
        SetValue(1, 1, theVec.y());
        SetValue(2, 2, theVec.z());
    }

    internal void SetDiagonal(in NCollection_Vec4<T> theVec)
    {
        SetValue(0, 0, theVec.x());
        SetValue(1, 1, theVec.y());
        SetValue(2, 2, theVec.z());
        SetValue(3, 3, theVec.w());
    }

    /// <summary>Multiplies this matrix on the right by a translation.</summary>
    internal void Translate(in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat4<T> aTempMat = new();
        aTempMat.SetColumn(3, theVec);
        Multiply(aTempMat);
    }

    internal void InitZero()
    {
        for (int i = 0; i < 16; ++i)
        {
            myMat[i] = T.Zero;
        }
    }

    internal void InitIdentity()
    {
        for (int i = 0; i < 16; ++i)
        {
            myMat[i] = (i % 5 == 0) ? T.One : T.Zero;
        }
    }

    /// <summary>Multiplies this matrix by another on the right (OCCT's <c>operator*=</c>).</summary>
    internal void Multiply(in NCollection_Mat4<T> theMat) => this = Multiply(this, theMat);

    internal void Multiply(T theFactor)
    {
        for (int i = 0; i < 16; ++i)
        {
            myMat[i] *= theFactor;
        }
    }

    internal void Divide(T theFactor)
    {
        for (int i = 0; i < 16; ++i)
        {
            myMat[i] /= theFactor;
        }
    }

    internal void Add(in NCollection_Mat4<T> theMat)
    {
        for (int i = 0; i < 16; ++i)
        {
            myMat[i] += theMat.myMat[i];
        }
    }

    internal void Subtract(in NCollection_Mat4<T> theMat)
    {
        for (int i = 0; i < 16; ++i)
        {
            myMat[i] -= theMat.myMat[i];
        }
    }

    internal void Transpose() => this = Transposed();

    internal void Convert<TOther>(in NCollection_Mat4<TOther> theFrom) where TOther : unmanaged, INumber<TOther> => ConvertFrom(theFrom);

    internal void ConvertFrom<TOther>(in NCollection_Mat4<TOther> theFrom) where TOther : unmanaged, INumber<TOther>
    {
        for (int anIdx = 0; anIdx < 16; ++anIdx)
        {
            myMat[anIdx] = T.CreateTruncating(theFrom.GetValue(anIdx % 4, anIdx / 4));
        }
    }

    // Copies with a mutator applied

    public readonly NCollection_Mat4<T> WithValue(int theRow, int theCol, T theValue)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetValue(theRow, theCol, theValue);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> WithRow(int theRow, in NCollection_Vec4<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetRow(theRow, theVec);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> WithColumn(int theCol, in NCollection_Vec4<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetColumn(theCol, theVec);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> WithRow(int theRow, in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetRow(theRow, theVec);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> WithColumn(int theCol, in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetColumn(theCol, theVec);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> WithDiagonal(in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetDiagonal(theVec);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> Translated(in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.Translate(theVec);
        return aCopy;
    }

    public readonly NCollection_Mat4<T> WithDiagonal(in NCollection_Vec4<T> theVec)
    {
        NCollection_Mat4<T> aCopy = this;
        aCopy.SetDiagonal(theVec);
        return aCopy;
    }

    // Queries

    public readonly bool IsZero()
    {
        for (int i = 0; i < 16; ++i)
        {
            if (myMat[i] != T.Zero)
            {
                return false;
            }
        }

        return true;
    }

    public readonly bool IsIdentity()
    {
        for (int i = 0; i < 16; ++i)
        {
            if (myMat[i] != ((i % 5 == 0) ? T.One : T.Zero))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each element).</summary>
    public readonly bool IsEqual(in NCollection_Mat4<T> theOther)
    {
        for (int i = 0; i < 16; ++i)
        {
            if (myMat[i] != theOther.myMat[i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>The product of two matrices.</summary>
    public static NCollection_Mat4<T> Multiply(in NCollection_Mat4<T> theMatA, in NCollection_Mat4<T> theMatB)
    {
        NCollection_Mat4<T> aMatRes = new();

        for (int aResElem = 0; aResElem < 16; ++aResElem)
        {
            aMatRes.myMat[aResElem] = T.Zero;
            for (int aInputElem = 0; aInputElem < 4; ++aInputElem)
            {
                aMatRes.myMat[aResElem] += theMatA.GetValue(aResElem % 4, aInputElem) * theMatB.GetValue(aInputElem, aResElem / 4);
            }
        }

        return aMatRes;
    }

    public readonly NCollection_Mat4<T> Multiplied(in NCollection_Mat4<T> theMat)
    {
        NCollection_Mat4<T> aTempMat = this;
        aTempMat.Multiply(theMat);
        return aTempMat;
    }

    public readonly NCollection_Mat4<T> Multiplied(T theFactor)
    {
        NCollection_Mat4<T> aTempMat = this;
        aTempMat.Multiply(theFactor);
        return aTempMat;
    }

    public readonly NCollection_Mat4<T> Divided(T theScalar)
    {
        NCollection_Mat4<T> aTempMat = this;
        aTempMat.Divide(theScalar);
        return aTempMat;
    }

    public readonly NCollection_Mat4<T> Added(in NCollection_Mat4<T> theMat)
    {
        NCollection_Mat4<T> aMat = this;
        aMat.Add(theMat);
        return aMat;
    }

    public readonly NCollection_Mat4<T> Subtracted(in NCollection_Mat4<T> theMat)
    {
        NCollection_Mat4<T> aMat = this;
        aMat.Subtract(theMat);
        return aMat;
    }

    public readonly NCollection_Mat4<T> Negated()
    {
        NCollection_Mat4<T> aMat = new();
        for (int i = 0; i < 16; ++i)
        {
            aMat.myMat[i] = -myMat[i];
        }

        return aMat;
    }

    public readonly NCollection_Mat4<T> Transposed()
    {
        NCollection_Mat4<T> aTempMat = new();
        aTempMat.SetRow(0, GetColumn(0));
        aTempMat.SetRow(1, GetColumn(1));
        aTempMat.SetRow(2, GetColumn(2));
        aTempMat.SetRow(3, GetColumn(3));
        return aTempMat;
    }

    /// <summary>
    /// Computes the inverse and the determinant into <paramref name="theOutMx"/> (written even when the
    /// determinant is zero, as OCCT does). Returns false when the determinant is zero.
    /// </summary>
    public readonly bool Inverted(out NCollection_Mat4<T> theOutMx, out T theDet)
    {
        // use short-cut for better readability (a copy, read before theOutMx is written in case they alias)
        NCollection_Buffer16<T> m = myMat;

        theOutMx = new();
        ref NCollection_Buffer16<T> inv = ref theOutMx.myMat;

        inv[0] = m[5] * (m[10] * m[15] - m[11] * m[14]) - m[9] * (m[6] * m[15] - m[7] * m[14])
                 - m[13] * (m[7] * m[10] - m[6] * m[11]);

        inv[1] = m[1] * (m[11] * m[14] - m[10] * m[15]) - m[9] * (m[3] * m[14] - m[2] * m[15])
                 - m[13] * (m[2] * m[11] - m[3] * m[10]);

        inv[2] = m[1] * (m[6] * m[15] - m[7] * m[14]) - m[5] * (m[2] * m[15] - m[3] * m[14])
                 - m[13] * (m[3] * m[6] - m[2] * m[7]);

        inv[3] = m[1] * (m[7] * m[10] - m[6] * m[11]) - m[5] * (m[3] * m[10] - m[2] * m[11])
                 - m[9] * (m[2] * m[7] - m[3] * m[6]);

        inv[4] = m[4] * (m[11] * m[14] - m[10] * m[15]) - m[8] * (m[7] * m[14] - m[6] * m[15])
                 - m[12] * (m[6] * m[11] - m[7] * m[10]);

        inv[5] = m[0] * (m[10] * m[15] - m[11] * m[14]) - m[8] * (m[2] * m[15] - m[3] * m[14])
                 - m[12] * (m[3] * m[10] - m[2] * m[11]);

        inv[6] = m[0] * (m[7] * m[14] - m[6] * m[15]) - m[4] * (m[3] * m[14] - m[2] * m[15])
                 - m[12] * (m[2] * m[7] - m[3] * m[6]);

        inv[7] = m[0] * (m[6] * m[11] - m[7] * m[10]) - m[4] * (m[2] * m[11] - m[3] * m[10])
                 - m[8] * (m[3] * m[6] - m[2] * m[7]);

        inv[8] = m[4] * (m[9] * m[15] - m[11] * m[13]) - m[8] * (m[5] * m[15] - m[7] * m[13])
                 - m[12] * (m[7] * m[9] - m[5] * m[11]);

        inv[9] = m[0] * (m[11] * m[13] - m[9] * m[15]) - m[8] * (m[3] * m[13] - m[1] * m[15])
                 - m[12] * (m[1] * m[11] - m[3] * m[9]);

        inv[10] = m[0] * (m[5] * m[15] - m[7] * m[13]) - m[4] * (m[1] * m[15] - m[3] * m[13])
                  - m[12] * (m[3] * m[5] - m[1] * m[7]);

        inv[11] = m[0] * (m[7] * m[9] - m[5] * m[11]) - m[4] * (m[3] * m[9] - m[1] * m[11])
                  - m[8] * (m[1] * m[7] - m[3] * m[5]);

        inv[12] = m[4] * (m[10] * m[13] - m[9] * m[14]) - m[8] * (m[6] * m[13] - m[5] * m[14])
                  - m[12] * (m[5] * m[10] - m[6] * m[9]);

        inv[13] = m[0] * (m[9] * m[14] - m[10] * m[13]) - m[8] * (m[1] * m[14] - m[2] * m[13])
                  - m[12] * (m[2] * m[9] - m[1] * m[10]);

        inv[14] = m[0] * (m[6] * m[13] - m[5] * m[14]) - m[4] * (m[2] * m[13] - m[1] * m[14])
                  - m[12] * (m[1] * m[6] - m[2] * m[5]);

        inv[15] = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[4] * (m[1] * m[10] - m[2] * m[9])
                  - m[8] * (m[2] * m[5] - m[1] * m[6]);

        theDet = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        if (theDet == T.Zero)
        {
            return false;
        }

        T aDiv = T.One / theDet;
        for (int i = 0; i < 16; ++i)
        {
            inv[i] *= aDiv;
        }

        return true;
    }

    public readonly bool Inverted(out NCollection_Mat4<T> theOutMx) => Inverted(out theOutMx, out _);

    /// <summary>The inverse; throws Standard_ConstructionError when the determinant is zero.</summary>
    public readonly NCollection_Mat4<T> Inverted()
    {
        if (!Inverted(out NCollection_Mat4<T> anInv))
        {
            throw new Standard_ConstructionError("NCollection_Mat4::Inverted() - matrix has zero determinant");
        }

        return anInv;
    }

    /// <summary>The determinant of the upper-left 3x3 part.</summary>
    public readonly T DeterminantMat3()
        => (GetValue(0, 0) * GetValue(1, 1) * GetValue(2, 2)
            + GetValue(0, 1) * GetValue(1, 2) * GetValue(2, 0)
            + GetValue(0, 2) * GetValue(1, 0) * GetValue(2, 1))
           - (GetValue(0, 2) * GetValue(1, 1) * GetValue(2, 0)
              + GetValue(0, 0) * GetValue(1, 2) * GetValue(2, 1)
              + GetValue(0, 1) * GetValue(1, 0) * GetValue(2, 2));

    /// <summary>The adjoint (adjugate) matrix, as OCCT documents it; see the remark in the code.</summary>
    // OCCT: crossVec4 returns the negated minors, so this is the negated cofactor matrix, neither the adjugate nor
    // Mat3's convention. Nothing in OCCT calls it.
    public readonly NCollection_Mat4<T> Adjoint()
    {
        NCollection_Mat4<T> aMat = new();
        aMat.SetRow(0, crossVec4(GetRow(1), GetRow(2), GetRow(3)));
        aMat.SetRow(1, crossVec4(-GetRow(0), GetRow(2), GetRow(3)));
        aMat.SetRow(2, crossVec4(GetRow(0), GetRow(1), GetRow(3)));
        aMat.SetRow(3, crossVec4(-GetRow(0), GetRow(1), GetRow(2)));
        return aMat;
    }

    private static NCollection_Vec4<T> crossVec4(in NCollection_Vec4<T> theA, in NCollection_Vec4<T> theB, in NCollection_Vec4<T> theC)
    {
        T aD1 = (theB.z() * theC.w()) - (theB.w() * theC.z());
        T aD2 = (theB.y() * theC.w()) - (theB.w() * theC.y());
        T aD3 = (theB.y() * theC.z()) - (theB.z() * theC.y());
        T aD4 = (theB.x() * theC.w()) - (theB.w() * theC.x());
        T aD5 = (theB.x() * theC.z()) - (theB.z() * theC.x());
        T aD6 = (theB.x() * theC.y()) - (theB.y() * theC.x());

        NCollection_Vec4<T> aVec = new();
        aVec[0] = -theA.y() * aD1 + theA.z() * aD2 - theA.w() * aD3;
        aVec[1] = theA.x() * aD1 - theA.z() * aD4 + theA.w() * aD5;
        aVec[2] = -theA.x() * aD2 + theA.y() * aD4 - theA.w() * aD6;
        aVec[3] = theA.x() * aD3 - theA.y() * aD5 + theA.z() * aD6;
        return aVec;
    }

    /// <summary>Copies the 16 elements, column by column, into a span (OCCT's <c>GetData()</c>).</summary>
    public readonly void CopyTo(Span<T> theDestination) => ((ReadOnlySpan<T>)myMat).CopyTo(theDestination);

    public static NCollection_Vec4<T> operator *(in NCollection_Mat4<T> theMat, in NCollection_Vec4<T> theVec)
        => new(theMat.GetValue(0, 0) * theVec.x() + theMat.GetValue(0, 1) * theVec.y() + theMat.GetValue(0, 2) * theVec.z()
                 + theMat.GetValue(0, 3) * theVec.w(),
               theMat.GetValue(1, 0) * theVec.x() + theMat.GetValue(1, 1) * theVec.y() + theMat.GetValue(1, 2) * theVec.z()
                 + theMat.GetValue(1, 3) * theVec.w(),
               theMat.GetValue(2, 0) * theVec.x() + theMat.GetValue(2, 1) * theVec.y() + theMat.GetValue(2, 2) * theVec.z()
                 + theMat.GetValue(2, 3) * theVec.w(),
               theMat.GetValue(3, 0) * theVec.x() + theMat.GetValue(3, 1) * theVec.y() + theMat.GetValue(3, 2) * theVec.z()
                 + theMat.GetValue(3, 3) * theVec.w());

    public static NCollection_Mat4<T> operator *(in NCollection_Mat4<T> theLeft, in NCollection_Mat4<T> theRight) => theLeft.Multiplied(theRight);

    public static NCollection_Mat4<T> operator *(in NCollection_Mat4<T> theMat, T theFactor) => theMat.Multiplied(theFactor);

    public static NCollection_Mat4<T> operator /(in NCollection_Mat4<T> theMat, T theScalar) => theMat.Divided(theScalar);

    public static NCollection_Mat4<T> operator +(in NCollection_Mat4<T> theLeft, in NCollection_Mat4<T> theRight) => theLeft.Added(theRight);

    public static NCollection_Mat4<T> operator -(in NCollection_Mat4<T> theLeft, in NCollection_Mat4<T> theRight) => theLeft.Subtracted(theRight);

    public static NCollection_Mat4<T> operator -(in NCollection_Mat4<T> theMat) => theMat.Negated();

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Mat4<T> other)
    {
        for (int i = 0; i < 16; ++i)
        {
            if (!myMat[i].Equals(other.myMat[i]))
            {
                return false;
            }
        }

        return true;
    }

    public override readonly bool Equals(object? obj) => obj is NCollection_Mat4<T> other && Equals(other);

    public override readonly int GetHashCode()
    {
        var aHash = new HashCode();
        for (int i = 0; i < 16; ++i)
        {
            aHash.Add(myMat[i]);
        }

        return aHash.ToHashCode();
    }

    public static bool operator ==(in NCollection_Mat4<T> theLeft, in NCollection_Mat4<T> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in NCollection_Mat4<T> theLeft, in NCollection_Mat4<T> theRight) => !theLeft.Equals(theRight);

    /// <summary>The elements row by row.</summary>
    public override readonly string ToString()
    {
        Span<T> aRows = stackalloc T[16];
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                aRows[r * 4 + c] = GetValue(r, c);
            }
        }

        return NCollection_VecTools.Format<T>("NCollection_Mat4", aRows);
    }
}
