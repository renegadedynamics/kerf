// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Mat3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Kerf.Engine;

/// <summary>A 3x3 matrix of numbers (float, double or int), stored column by column as OCCT stores it.</summary>
/// <remarks>
/// OCCT's default constructor makes the identity, and so does <c>new NCollection_Mat3&lt;T&gt;()</c>;
/// <c>default(NCollection_Mat3&lt;T&gt;)</c> is the zero matrix.
/// </remarks>
internal partial struct NCollection_Mat3<T> : IEquatable<NCollection_Mat3<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer9<T> _mat;

    /// <summary>The identity matrix.</summary>
    public static NCollection_Mat3<T> Identity() => new();

    /// <summary>The zero matrix.</summary>
    public static NCollection_Mat3<T> Zero()
    {
        NCollection_Mat3<T> mat = new();
        mat.InitZero();
        return mat;
    }

    /// <summary>The identity matrix.</summary>
    public NCollection_Mat3() => InitIdentity();

    /// <summary>Converts a matrix of another element type (OCCT's converting constructor).</summary>
    public static NCollection_Mat3<T> From<TOther>(in NCollection_Mat3<TOther> otherMat3) where TOther : unmanaged, INumber<TOther>
    {
        NCollection_Mat3<T> mat = new();
        mat.ConvertFrom(otherMat3);
        return mat;
    }

    /// <summary>The element at a (0-based) row and column.</summary>
    public readonly T GetValue(int row, int col) => _mat[col * 3 + row];

    /// <summary>The element at a (0-based) row and column (OCCT's <c>operator()</c>).</summary>
    public T this[int row, int col]
    {
        readonly get => GetValue(row, col);
        internal set => SetValue(row, col, value);
    }

    public readonly NCollection_Vec3<T> GetRow(int row)
        => new(GetValue(row, 0), GetValue(row, 1), GetValue(row, 2));

    public readonly NCollection_Vec3<T> GetColumn(int col)
        => new(GetValue(0, col), GetValue(1, col), GetValue(2, col));

    public readonly NCollection_Vec3<T> GetDiagonal() => new(GetValue(0, 0), GetValue(1, 1), GetValue(2, 2));

    // Mutators (internal: see docs/translation-rules.md)

    [UnscopedRef]
    internal ref T ChangeValue(int row, int col) => ref _mat[col * 3 + row];

    internal void SetValue(int row, int col, T value) => _mat[col * 3 + row] = value;

    internal void SetRow(int row, in NCollection_Vec3<T> vec)
    {
        SetValue(row, 0, vec.X());
        SetValue(row, 1, vec.Y());
        SetValue(row, 2, vec.Z());
    }

    internal void SetColumn(int col, in NCollection_Vec3<T> vec)
    {
        SetValue(0, col, vec.X());
        SetValue(1, col, vec.Y());
        SetValue(2, col, vec.Z());
    }

    internal void SetDiagonal(in NCollection_Vec3<T> vec)
    {
        SetValue(0, 0, vec.X());
        SetValue(1, 1, vec.Y());
        SetValue(2, 2, vec.Z());
    }

    internal void InitZero()
    {
        for (int i = 0; i < 9; ++i)
        {
            _mat[i] = T.Zero;
        }
    }

    internal void InitIdentity()
    {
        for (int i = 0; i < 9; ++i)
        {
            _mat[i] = (i % 4 == 0) ? T.One : T.Zero;
        }
    }

    /// <summary>Multiplies this matrix by another on the right (OCCT's <c>operator*=</c>).</summary>
    internal void Multiply(in NCollection_Mat3<T> mat) => this = Multiply(this, mat);

    internal void Multiply(T factor)
    {
        for (int i = 0; i < 9; ++i)
        {
            _mat[i] *= factor;
        }
    }

    internal void Divide(T factor)
    {
        for (int i = 0; i < 9; ++i)
        {
            _mat[i] /= factor;
        }
    }

    internal void Add(in NCollection_Mat3<T> mat)
    {
        for (int i = 0; i < 9; ++i)
        {
            _mat[i] += mat._mat[i];
        }
    }

    internal void Subtract(in NCollection_Mat3<T> mat)
    {
        for (int i = 0; i < 9; ++i)
        {
            _mat[i] -= mat._mat[i];
        }
    }

    internal void Transpose() => this = Transposed();

    internal void ConvertFrom<TOther>(in NCollection_Mat3<TOther> theFrom) where TOther : unmanaged, INumber<TOther>
    {
        for (int idx = 0; idx < 9; ++idx)
        {
            _mat[idx] = T.CreateTruncating(theFrom.GetValue(idx % 3, idx / 3));
        }
    }

    // Copies with a mutator applied

    public readonly NCollection_Mat3<T> WithValue(int row, int col, T value)
    {
        NCollection_Mat3<T> copy = this;
        copy.SetValue(row, col, value);
        return copy;
    }

    public readonly NCollection_Mat3<T> WithRow(int row, in NCollection_Vec3<T> vec)
    {
        NCollection_Mat3<T> copy = this;
        copy.SetRow(row, vec);
        return copy;
    }

    public readonly NCollection_Mat3<T> WithColumn(int col, in NCollection_Vec3<T> vec)
    {
        NCollection_Mat3<T> copy = this;
        copy.SetColumn(col, vec);
        return copy;
    }

    public readonly NCollection_Mat3<T> WithDiagonal(in NCollection_Vec3<T> vec)
    {
        NCollection_Mat3<T> copy = this;
        copy.SetDiagonal(vec);
        return copy;
    }

    // Queries

    public readonly bool IsZero()
    {
        for (int i = 0; i < 9; ++i)
        {
            if (_mat[i] != T.Zero)
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
            if (_mat[i] != ((i % 4 == 0) ? T.One : T.Zero))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each element).</summary>
    public readonly bool IsEqual(in NCollection_Mat3<T> other)
    {
        for (int i = 0; i < 9; ++i)
        {
            if (_mat[i] != other._mat[i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>The product of two matrices.</summary>
    public static NCollection_Mat3<T> Multiply(in NCollection_Mat3<T> matA, in NCollection_Mat3<T> matB)
    {
        NCollection_Mat3<T> matRes = new();

        for (int resElem = 0; resElem < 9; ++resElem)
        {
            matRes._mat[resElem] = T.Zero;
            for (int inputElem = 0; inputElem < 3; ++inputElem)
            {
                matRes._mat[resElem] += matA.GetValue(resElem % 3, inputElem) * matB.GetValue(inputElem, resElem / 3);
            }
        }

        return matRes;
    }

    public readonly NCollection_Mat3<T> Multiplied(in NCollection_Mat3<T> mat)
    {
        NCollection_Mat3<T> tempMat = this;
        tempMat.Multiply(mat);
        return tempMat;
    }

    public readonly NCollection_Mat3<T> Multiplied(T factor)
    {
        NCollection_Mat3<T> tempMat = this;
        tempMat.Multiply(factor);
        return tempMat;
    }

    public readonly NCollection_Mat3<T> Divided(T scalar)
    {
        NCollection_Mat3<T> tempMat = this;
        tempMat.Divide(scalar);
        return tempMat;
    }

    public readonly NCollection_Mat3<T> Added(in NCollection_Mat3<T> mat)
    {
        NCollection_Mat3<T> aMat = this;
        aMat.Add(mat);
        return aMat;
    }

    public readonly NCollection_Mat3<T> Subtracted(in NCollection_Mat3<T> mat)
    {
        NCollection_Mat3<T> aMat = this;
        aMat.Subtract(mat);
        return aMat;
    }

    public readonly NCollection_Mat3<T> Negated()
    {
        NCollection_Mat3<T> mat = new();
        for (int i = 0; i < 9; ++i)
        {
            mat._mat[i] = -_mat[i];
        }

        return mat;
    }

    public readonly NCollection_Mat3<T> Transposed()
    {
        NCollection_Mat3<T> tempMat = new();
        tempMat.SetRow(0, GetColumn(0));
        tempMat.SetRow(1, GetColumn(1));
        tempMat.SetRow(2, GetColumn(2));
        return tempMat;
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
        NCollection_Mat3<T> mat = new();
        mat.SetRow(0, NCollection_Vec3<T>.Cross(GetRow(1), GetRow(2)));
        mat.SetRow(1, NCollection_Vec3<T>.Cross(GetRow(2), GetRow(0)));
        mat.SetRow(2, NCollection_Vec3<T>.Cross(GetRow(0), GetRow(1)));
        return mat;
    }

    /// <summary>
    /// Computes the inverse and the determinant. Returns false, leaving <paramref name="inv"/> unchanged,
    /// when the determinant is zero (hence <c>ref</c>).
    /// </summary>
    public readonly bool Inverted(ref NCollection_Mat3<T> inv, out T det)
    {
        NCollection_Mat3<T> mat = Adjoint();
        det = mat.GetRow(0).Dot(GetRow(0));
        if (det == T.Zero)
        {
            return false;
        }

        inv = mat.Transposed() / det;
        return true;
    }

    public readonly bool Inverted(ref NCollection_Mat3<T> inv) => Inverted(ref inv, out _);

    /// <summary>The inverse; throws Standard_ConstructionError when the determinant is zero.</summary>
    public readonly NCollection_Mat3<T> Inverted()
    {
        NCollection_Mat3<T> inv = new();
        if (!Inverted(ref inv))
        {
            throw new ConstructionErrorException("NCollection_Mat3::Inverted() - matrix has zero determinant");
        }

        return inv;
    }

    /// <summary>Copies the 9 elements, column by column, into a span (OCCT's <c>GetData()</c>).</summary>
    public readonly void CopyTo(Span<T> destination) => ((ReadOnlySpan<T>)_mat).CopyTo(destination);

    public static NCollection_Vec3<T> operator *(in NCollection_Mat3<T> mat, in NCollection_Vec3<T> vec)
        => new(mat.GetValue(0, 0) * vec.X() + mat.GetValue(0, 1) * vec.Y() + mat.GetValue(0, 2) * vec.Z(),
               mat.GetValue(1, 0) * vec.X() + mat.GetValue(1, 1) * vec.Y() + mat.GetValue(1, 2) * vec.Z(),
               mat.GetValue(2, 0) * vec.X() + mat.GetValue(2, 1) * vec.Y() + mat.GetValue(2, 2) * vec.Z());

    public static NCollection_Mat3<T> operator *(in NCollection_Mat3<T> left, in NCollection_Mat3<T> right) => left.Multiplied(right);

    public static NCollection_Mat3<T> operator *(in NCollection_Mat3<T> mat, T factor) => mat.Multiplied(factor);

    public static NCollection_Mat3<T> operator /(in NCollection_Mat3<T> mat, T scalar) => mat.Divided(scalar);

    public static NCollection_Mat3<T> operator +(in NCollection_Mat3<T> left, in NCollection_Mat3<T> right) => left.Added(right);

    public static NCollection_Mat3<T> operator -(in NCollection_Mat3<T> left, in NCollection_Mat3<T> right) => left.Subtracted(right);

    public static NCollection_Mat3<T> operator -(in NCollection_Mat3<T> mat) => mat.Negated();

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Mat3<T> other)
    {
        for (int i = 0; i < 9; ++i)
        {
            if (!_mat[i].Equals(other._mat[i]))
            {
                return false;
            }
        }

        return true;
    }

    public override readonly bool Equals(object? obj) => obj is NCollection_Mat3<T> other && Equals(other);

    public override readonly int GetHashCode()
    {
        var hash = new HashCode();
        for (int i = 0; i < 9; ++i)
        {
            hash.Add(_mat[i]);
        }

        return hash.ToHashCode();
    }

    public static bool operator ==(in NCollection_Mat3<T> left, in NCollection_Mat3<T> right) => left.Equals(right);

    public static bool operator !=(in NCollection_Mat3<T> left, in NCollection_Mat3<T> right) => !left.Equals(right);

    /// <summary>The elements row by row.</summary>
    public override readonly string ToString()
    {
        Span<T> rows = stackalloc T[9];
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                rows[r * 3 + c] = GetValue(r, c);
            }
        }

        return NCollection_VecTools.Format<T>("NCollection_Mat3", rows);
    }
}
