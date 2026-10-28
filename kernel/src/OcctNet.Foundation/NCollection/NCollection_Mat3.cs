// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Mat3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;

/// <summary>A 3x3 matrix of numbers (float, double or int), stored column by column as OCCT stores it.</summary>
/// <remarks>
/// OCCT's default constructor makes the identity, and so does <c>new NCollection_Mat3&lt;T&gt;()</c>;
/// <c>default(NCollection_Mat3&lt;T&gt;)</c> is the zero matrix.
/// </remarks>
public partial struct NCollection_Mat3<T> : IEquatable<NCollection_Mat3<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer9<T> myMat;

    /// <summary>The identity matrix.</summary>
    public static NCollection_Mat3<T> Identity() => new();

    /// <summary>The zero matrix.</summary>
    public static NCollection_Mat3<T> Zero()
    {
        NCollection_Mat3<T> aMat = new();
        aMat.InitZero();
        return aMat;
    }

    /// <summary>The identity matrix.</summary>
    public NCollection_Mat3() => InitIdentity();

    /// <summary>Converts a matrix of another element type (OCCT's converting constructor).</summary>
    public static NCollection_Mat3<T> From<TOther>(in NCollection_Mat3<TOther> theOtherMat3) where TOther : unmanaged, INumber<TOther>
    {
        NCollection_Mat3<T> aMat = new();
        aMat.ConvertFrom(theOtherMat3);
        return aMat;
    }

    /// <summary>The element at a (0-based) row and column.</summary>
    public readonly T GetValue(int theRow, int theCol) => myMat[theCol * 3 + theRow];

    /// <summary>The element at a (0-based) row and column (OCCT's <c>operator()</c>).</summary>
    public T this[int theRow, int theCol]
    {
        readonly get => GetValue(theRow, theCol);
        internal set => SetValue(theRow, theCol, value);
    }

    public readonly NCollection_Vec3<T> GetRow(int theRow)
        => new(GetValue(theRow, 0), GetValue(theRow, 1), GetValue(theRow, 2));

    public readonly NCollection_Vec3<T> GetColumn(int theCol)
        => new(GetValue(0, theCol), GetValue(1, theCol), GetValue(2, theCol));

    public readonly NCollection_Vec3<T> GetDiagonal() => new(GetValue(0, 0), GetValue(1, 1), GetValue(2, 2));

    // Mutators (internal: see docs/translation-rules.md)

    [UnscopedRef]
    internal ref T ChangeValue(int theRow, int theCol) => ref myMat[theCol * 3 + theRow];

    internal void SetValue(int theRow, int theCol, T theValue) => myMat[theCol * 3 + theRow] = theValue;

    internal void SetRow(int theRow, in NCollection_Vec3<T> theVec)
    {
        SetValue(theRow, 0, theVec.x());
        SetValue(theRow, 1, theVec.y());
        SetValue(theRow, 2, theVec.z());
    }

    internal void SetColumn(int theCol, in NCollection_Vec3<T> theVec)
    {
        SetValue(0, theCol, theVec.x());
        SetValue(1, theCol, theVec.y());
        SetValue(2, theCol, theVec.z());
    }

    internal void SetDiagonal(in NCollection_Vec3<T> theVec)
    {
        SetValue(0, 0, theVec.x());
        SetValue(1, 1, theVec.y());
        SetValue(2, 2, theVec.z());
    }

    internal void InitZero()
    {
        for (int i = 0; i < 9; ++i)
        {
            myMat[i] = T.Zero;
        }
    }

    internal void InitIdentity()
    {
        for (int i = 0; i < 9; ++i)
        {
            myMat[i] = (i % 4 == 0) ? T.One : T.Zero;
        }
    }

    /// <summary>Multiplies this matrix by another on the right (OCCT's <c>operator*=</c>).</summary>
    internal void Multiply(in NCollection_Mat3<T> theMat) => this = Multiply(this, theMat);

    internal void Multiply(T theFactor)
    {
        for (int i = 0; i < 9; ++i)
        {
            myMat[i] *= theFactor;
        }
    }

    internal void Divide(T theFactor)
    {
        for (int i = 0; i < 9; ++i)
        {
            myMat[i] /= theFactor;
        }
    }

    internal void Add(in NCollection_Mat3<T> theMat)
    {
        for (int i = 0; i < 9; ++i)
        {
            myMat[i] += theMat.myMat[i];
        }
    }

    internal void Subtract(in NCollection_Mat3<T> theMat)
    {
        for (int i = 0; i < 9; ++i)
        {
            myMat[i] -= theMat.myMat[i];
        }
    }

    internal void Transpose() => this = Transposed();

    internal void ConvertFrom<TOther>(in NCollection_Mat3<TOther> theFrom) where TOther : unmanaged, INumber<TOther>
    {
        for (int anIdx = 0; anIdx < 9; ++anIdx)
        {
            myMat[anIdx] = T.CreateTruncating(theFrom.GetValue(anIdx % 3, anIdx / 3));
        }
    }

    // Copies with a mutator applied

    public readonly NCollection_Mat3<T> WithValue(int theRow, int theCol, T theValue)
    {
        NCollection_Mat3<T> aCopy = this;
        aCopy.SetValue(theRow, theCol, theValue);
        return aCopy;
    }

    public readonly NCollection_Mat3<T> WithRow(int theRow, in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat3<T> aCopy = this;
        aCopy.SetRow(theRow, theVec);
        return aCopy;
    }

    public readonly NCollection_Mat3<T> WithColumn(int theCol, in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat3<T> aCopy = this;
        aCopy.SetColumn(theCol, theVec);
        return aCopy;
    }

    public readonly NCollection_Mat3<T> WithDiagonal(in NCollection_Vec3<T> theVec)
    {
        NCollection_Mat3<T> aCopy = this;
        aCopy.SetDiagonal(theVec);
        return aCopy;
    }

    // Queries

    public readonly bool IsZero()
    {
        for (int i = 0; i < 9; ++i)
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
        for (int i = 0; i < 9; ++i)
        {
            if (myMat[i] != ((i % 4 == 0) ? T.One : T.Zero))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each element).</summary>
    public readonly bool IsEqual(in NCollection_Mat3<T> theOther)
    {
        for (int i = 0; i < 9; ++i)
        {
            if (myMat[i] != theOther.myMat[i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>The product of two matrices.</summary>
    public static NCollection_Mat3<T> Multiply(in NCollection_Mat3<T> theMatA, in NCollection_Mat3<T> theMatB)
    {
        NCollection_Mat3<T> aMatRes = new();

        for (int aResElem = 0; aResElem < 9; ++aResElem)
        {
            aMatRes.myMat[aResElem] = T.Zero;
            for (int aInputElem = 0; aInputElem < 3; ++aInputElem)
            {
                aMatRes.myMat[aResElem] += theMatA.GetValue(aResElem % 3, aInputElem) * theMatB.GetValue(aInputElem, aResElem / 3);
            }
        }

        return aMatRes;
    }

    public readonly NCollection_Mat3<T> Multiplied(in NCollection_Mat3<T> theMat)
    {
        NCollection_Mat3<T> aTempMat = this;
        aTempMat.Multiply(theMat);
        return aTempMat;
    }

    public readonly NCollection_Mat3<T> Multiplied(T theFactor)
    {
        NCollection_Mat3<T> aTempMat = this;
        aTempMat.Multiply(theFactor);
        return aTempMat;
    }

    public readonly NCollection_Mat3<T> Divided(T theScalar)
    {
        NCollection_Mat3<T> aTempMat = this;
        aTempMat.Divide(theScalar);
        return aTempMat;
    }

    public readonly NCollection_Mat3<T> Added(in NCollection_Mat3<T> theMat)
    {
        NCollection_Mat3<T> aMat = this;
        aMat.Add(theMat);
        return aMat;
    }

    public readonly NCollection_Mat3<T> Subtracted(in NCollection_Mat3<T> theMat)
    {
        NCollection_Mat3<T> aMat = this;
        aMat.Subtract(theMat);
        return aMat;
    }

    public readonly NCollection_Mat3<T> Negated()
    {
        NCollection_Mat3<T> aMat = new();
        for (int i = 0; i < 9; ++i)
        {
            aMat.myMat[i] = -myMat[i];
        }

        return aMat;
    }

    public readonly NCollection_Mat3<T> Transposed()
    {
        NCollection_Mat3<T> aTempMat = new();
        aTempMat.SetRow(0, GetColumn(0));
        aTempMat.SetRow(1, GetColumn(1));
        aTempMat.SetRow(2, GetColumn(2));
        return aTempMat;
    }

    public readonly T Determinant()
        => (GetValue(0, 0) * GetValue(1, 1) * GetValue(2, 2)
            + GetValue(0, 1) * GetValue(1, 2) * GetValue(2, 0)
            + GetValue(0, 2) * GetValue(1, 0) * GetValue(2, 1))
           - (GetValue(0, 2) * GetValue(1, 1) * GetValue(2, 0)
              + GetValue(0, 0) * GetValue(1, 2) * GetValue(2, 1)
              + GetValue(0, 1) * GetValue(1, 0) * GetValue(2, 2));

    /// <summary>The adjoint (adjugate) matrix, as OCCT documents it; see the remark in the code.</summary>
    // OCCT: returns the cofactor matrix, which is the transpose of the adjugate. Inverted() transposes it, so the
    // inverse is right.
    public readonly NCollection_Mat3<T> Adjoint()
    {
        NCollection_Mat3<T> aMat = new();
        aMat.SetRow(0, NCollection_Vec3<T>.Cross(GetRow(1), GetRow(2)));
        aMat.SetRow(1, NCollection_Vec3<T>.Cross(GetRow(2), GetRow(0)));
        aMat.SetRow(2, NCollection_Vec3<T>.Cross(GetRow(0), GetRow(1)));
        return aMat;
    }

    /// <summary>
    /// Computes the inverse and the determinant. Returns false, leaving <paramref name="theInv"/> unchanged,
    /// when the determinant is zero (hence <c>ref</c>).
    /// </summary>
    public readonly bool Inverted(ref NCollection_Mat3<T> theInv, out T theDet)
    {
        NCollection_Mat3<T> aMat = Adjoint();
        theDet = aMat.GetRow(0).Dot(GetRow(0));
        if (theDet == T.Zero)
        {
            return false;
        }

        theInv = aMat.Transposed() / theDet;
        return true;
    }

    public readonly bool Inverted(ref NCollection_Mat3<T> theInv) => Inverted(ref theInv, out _);

    /// <summary>The inverse; throws Standard_ConstructionError when the determinant is zero.</summary>
    public readonly NCollection_Mat3<T> Inverted()
    {
        NCollection_Mat3<T> anInv = new();
        if (!Inverted(ref anInv))
        {
            throw new Standard_ConstructionError("NCollection_Mat3::Inverted() - matrix has zero determinant");
        }

        return anInv;
    }

    /// <summary>Copies the 9 elements, column by column, into a span (OCCT's <c>GetData()</c>).</summary>
    public readonly void CopyTo(Span<T> theDestination) => ((ReadOnlySpan<T>)myMat).CopyTo(theDestination);

    public static NCollection_Vec3<T> operator *(in NCollection_Mat3<T> theMat, in NCollection_Vec3<T> theVec)
        => new(theMat.GetValue(0, 0) * theVec.x() + theMat.GetValue(0, 1) * theVec.y() + theMat.GetValue(0, 2) * theVec.z(),
               theMat.GetValue(1, 0) * theVec.x() + theMat.GetValue(1, 1) * theVec.y() + theMat.GetValue(1, 2) * theVec.z(),
               theMat.GetValue(2, 0) * theVec.x() + theMat.GetValue(2, 1) * theVec.y() + theMat.GetValue(2, 2) * theVec.z());

    public static NCollection_Mat3<T> operator *(in NCollection_Mat3<T> theLeft, in NCollection_Mat3<T> theRight) => theLeft.Multiplied(theRight);

    public static NCollection_Mat3<T> operator *(in NCollection_Mat3<T> theMat, T theFactor) => theMat.Multiplied(theFactor);

    public static NCollection_Mat3<T> operator /(in NCollection_Mat3<T> theMat, T theScalar) => theMat.Divided(theScalar);

    public static NCollection_Mat3<T> operator +(in NCollection_Mat3<T> theLeft, in NCollection_Mat3<T> theRight) => theLeft.Added(theRight);

    public static NCollection_Mat3<T> operator -(in NCollection_Mat3<T> theLeft, in NCollection_Mat3<T> theRight) => theLeft.Subtracted(theRight);

    public static NCollection_Mat3<T> operator -(in NCollection_Mat3<T> theMat) => theMat.Negated();

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Mat3<T> other)
    {
        for (int i = 0; i < 9; ++i)
        {
            if (!myMat[i].Equals(other.myMat[i]))
            {
                return false;
            }
        }

        return true;
    }

    public override readonly bool Equals(object? obj) => obj is NCollection_Mat3<T> other && Equals(other);

    public override readonly int GetHashCode()
    {
        var aHash = new HashCode();
        for (int i = 0; i < 9; ++i)
        {
            aHash.Add(myMat[i]);
        }

        return aHash.ToHashCode();
    }

    public static bool operator ==(in NCollection_Mat3<T> theLeft, in NCollection_Mat3<T> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in NCollection_Mat3<T> theLeft, in NCollection_Mat3<T> theRight) => !theLeft.Equals(theRight);

    /// <summary>The elements row by row.</summary>
    public override readonly string ToString()
    {
        Span<T> aRows = stackalloc T[9];
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                aRows[r * 3 + c] = GetValue(r, c);
            }
        }

        return NCollection_VecTools.Format<T>("NCollection_Mat3", aRows);
    }
}
