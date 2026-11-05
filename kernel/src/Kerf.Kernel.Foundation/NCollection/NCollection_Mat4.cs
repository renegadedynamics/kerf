// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Mat4.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;

/// <summary>A 4x4 matrix of numbers (float, double or int), stored column by column as OCCT stores it.</summary>
/// <remarks>
/// OCCT's default constructor makes the identity, and so does <c>new NCollection_Mat4&lt;T&gt;()</c>;
/// <c>default(NCollection_Mat4&lt;T&gt;)</c> is the zero matrix.
/// </remarks>
internal partial struct NCollection_Mat4<T> : IEquatable<NCollection_Mat4<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer16<T> _mat;

    public static int Rows() => 4;

    public static int Cols() => 4;

    /// <summary>The identity matrix.</summary>
    public static NCollection_Mat4<T> Identity() => new();

    /// <summary>The zero matrix.</summary>
    public static NCollection_Mat4<T> Zero()
    {
        NCollection_Mat4<T> mat = new();
        mat.InitZero();
        return mat;
    }

    /// <summary>The identity matrix.</summary>
    public NCollection_Mat4() => InitIdentity();

    /// <summary>Converts a matrix of another element type (OCCT's converting constructor).</summary>
    public static NCollection_Mat4<T> From<TOther>(in NCollection_Mat4<TOther> otherMat3) where TOther : unmanaged, INumber<TOther>
    {
        NCollection_Mat4<T> mat = new();
        mat.ConvertFrom(otherMat3);
        return mat;
    }

    /// <summary>The element at a (0-based) row and column.</summary>
    public readonly T GetValue(int row, int col) => _mat[col * 4 + row];

    /// <summary>The element at a (0-based) row and column (OCCT's <c>operator()</c>).</summary>
    public T this[int row, int col]
    {
        readonly get => GetValue(row, col);
        internal set => SetValue(row, col, value);
    }

    public readonly NCollection_Vec4<T> GetRow(int row)
        => new(GetValue(row, 0), GetValue(row, 1), GetValue(row, 2), GetValue(row, 3));

    public readonly NCollection_Vec4<T> GetColumn(int col)
        => new(GetValue(0, col), GetValue(1, col), GetValue(2, col), GetValue(3, col));

    public readonly NCollection_Vec4<T> GetDiagonal() => new(GetValue(0, 0), GetValue(1, 1), GetValue(2, 2), GetValue(3, 3));

    /// <summary>The upper-left 3x3 part.</summary>
    public readonly NCollection_Mat3<T> GetMat3()
    {
        NCollection_Mat3<T> mat = new();
        mat.SetColumn(0, GetColumn(0).Xyz());
        mat.SetColumn(1, GetColumn(1).Xyz());
        mat.SetColumn(2, GetColumn(2).Xyz());
        return mat;
    }

    // Mutators (internal: see docs/translation-rules.md)

    [UnscopedRef]
    internal ref T ChangeValue(int row, int col) => ref _mat[col * 4 + row];

    internal void SetValue(int row, int col, T value) => _mat[col * 4 + row] = value;

    internal void SetRow(int row, in NCollection_Vec3<T> vec)
    {
        SetValue(row, 0, vec.X());
        SetValue(row, 1, vec.Y());
        SetValue(row, 2, vec.Z());
    }

    internal void SetRow(int row, in NCollection_Vec4<T> vec)
    {
        SetValue(row, 0, vec.X());
        SetValue(row, 1, vec.Y());
        SetValue(row, 2, vec.Z());
        SetValue(row, 3, vec.W());
    }

    internal void SetColumn(int col, in NCollection_Vec3<T> vec)
    {
        SetValue(0, col, vec.X());
        SetValue(1, col, vec.Y());
        SetValue(2, col, vec.Z());
    }

    internal void SetColumn(int col, in NCollection_Vec4<T> vec)
    {
        SetValue(0, col, vec.X());
        SetValue(1, col, vec.Y());
        SetValue(2, col, vec.Z());
        SetValue(3, col, vec.W());
    }

    internal void SetDiagonal(in NCollection_Vec3<T> vec)
    {
        SetValue(0, 0, vec.X());
        SetValue(1, 1, vec.Y());
        SetValue(2, 2, vec.Z());
    }

    internal void SetDiagonal(in NCollection_Vec4<T> vec)
    {
        SetValue(0, 0, vec.X());
        SetValue(1, 1, vec.Y());
        SetValue(2, 2, vec.Z());
        SetValue(3, 3, vec.W());
    }

    /// <summary>Multiplies this matrix on the right by a translation.</summary>
    internal void Translate(in NCollection_Vec3<T> vec)
    {
        NCollection_Mat4<T> tempMat = new();
        tempMat.SetColumn(3, vec);
        Multiply(tempMat);
    }

    internal void InitZero()
    {
        for (int i = 0; i < 16; ++i)
        {
            _mat[i] = T.Zero;
        }
    }

    internal void InitIdentity()
    {
        for (int i = 0; i < 16; ++i)
        {
            _mat[i] = (i % 5 == 0) ? T.One : T.Zero;
        }
    }

    /// <summary>Multiplies this matrix by another on the right (OCCT's <c>operator*=</c>).</summary>
    internal void Multiply(in NCollection_Mat4<T> mat) => this = Multiply(this, mat);

    internal void Multiply(T factor)
    {
        for (int i = 0; i < 16; ++i)
        {
            _mat[i] *= factor;
        }
    }

    internal void Divide(T factor)
    {
        for (int i = 0; i < 16; ++i)
        {
            _mat[i] /= factor;
        }
    }

    internal void Add(in NCollection_Mat4<T> mat)
    {
        for (int i = 0; i < 16; ++i)
        {
            _mat[i] += mat._mat[i];
        }
    }

    internal void Subtract(in NCollection_Mat4<T> mat)
    {
        for (int i = 0; i < 16; ++i)
        {
            _mat[i] -= mat._mat[i];
        }
    }

    internal void Transpose() => this = Transposed();

    internal void Convert<TOther>(in NCollection_Mat4<TOther> theFrom) where TOther : unmanaged, INumber<TOther> => ConvertFrom(theFrom);

    internal void ConvertFrom<TOther>(in NCollection_Mat4<TOther> theFrom) where TOther : unmanaged, INumber<TOther>
    {
        for (int idx = 0; idx < 16; ++idx)
        {
            _mat[idx] = T.CreateTruncating(theFrom.GetValue(idx % 4, idx / 4));
        }
    }

    // Copies with a mutator applied

    public readonly NCollection_Mat4<T> WithValue(int row, int col, T value)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetValue(row, col, value);
        return copy;
    }

    public readonly NCollection_Mat4<T> WithRow(int row, in NCollection_Vec4<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetRow(row, vec);
        return copy;
    }

    public readonly NCollection_Mat4<T> WithColumn(int col, in NCollection_Vec4<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetColumn(col, vec);
        return copy;
    }

    public readonly NCollection_Mat4<T> WithRow(int row, in NCollection_Vec3<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetRow(row, vec);
        return copy;
    }

    public readonly NCollection_Mat4<T> WithColumn(int col, in NCollection_Vec3<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetColumn(col, vec);
        return copy;
    }

    public readonly NCollection_Mat4<T> WithDiagonal(in NCollection_Vec3<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetDiagonal(vec);
        return copy;
    }

    public readonly NCollection_Mat4<T> Translated(in NCollection_Vec3<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.Translate(vec);
        return copy;
    }

    public readonly NCollection_Mat4<T> WithDiagonal(in NCollection_Vec4<T> vec)
    {
        NCollection_Mat4<T> copy = this;
        copy.SetDiagonal(vec);
        return copy;
    }

    // Queries

    public readonly bool IsZero()
    {
        for (int i = 0; i < 16; ++i)
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
        for (int i = 0; i < 16; ++i)
        {
            if (_mat[i] != ((i % 5 == 0) ? T.One : T.Zero))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each element).</summary>
    public readonly bool IsEqual(in NCollection_Mat4<T> other)
    {
        for (int i = 0; i < 16; ++i)
        {
            if (_mat[i] != other._mat[i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>The product of two matrices.</summary>
    public static NCollection_Mat4<T> Multiply(in NCollection_Mat4<T> matA, in NCollection_Mat4<T> matB)
    {
        NCollection_Mat4<T> matRes = new();

        for (int resElem = 0; resElem < 16; ++resElem)
        {
            matRes._mat[resElem] = T.Zero;
            for (int inputElem = 0; inputElem < 4; ++inputElem)
            {
                matRes._mat[resElem] += matA.GetValue(resElem % 4, inputElem) * matB.GetValue(inputElem, resElem / 4);
            }
        }

        return matRes;
    }

    public readonly NCollection_Mat4<T> Multiplied(in NCollection_Mat4<T> mat)
    {
        NCollection_Mat4<T> tempMat = this;
        tempMat.Multiply(mat);
        return tempMat;
    }

    public readonly NCollection_Mat4<T> Multiplied(T factor)
    {
        NCollection_Mat4<T> tempMat = this;
        tempMat.Multiply(factor);
        return tempMat;
    }

    public readonly NCollection_Mat4<T> Divided(T scalar)
    {
        NCollection_Mat4<T> tempMat = this;
        tempMat.Divide(scalar);
        return tempMat;
    }

    public readonly NCollection_Mat4<T> Added(in NCollection_Mat4<T> mat)
    {
        NCollection_Mat4<T> aMat = this;
        aMat.Add(mat);
        return aMat;
    }

    public readonly NCollection_Mat4<T> Subtracted(in NCollection_Mat4<T> mat)
    {
        NCollection_Mat4<T> aMat = this;
        aMat.Subtract(mat);
        return aMat;
    }

    public readonly NCollection_Mat4<T> Negated()
    {
        NCollection_Mat4<T> mat = new();
        for (int i = 0; i < 16; ++i)
        {
            mat._mat[i] = -_mat[i];
        }

        return mat;
    }

    public readonly NCollection_Mat4<T> Transposed()
    {
        NCollection_Mat4<T> tempMat = new();
        tempMat.SetRow(0, GetColumn(0));
        tempMat.SetRow(1, GetColumn(1));
        tempMat.SetRow(2, GetColumn(2));
        tempMat.SetRow(3, GetColumn(3));
        return tempMat;
    }

    /// <summary>
    /// Computes the inverse and the determinant into <paramref name="outMx"/> (written even when the
    /// determinant is zero, as OCCT does). Returns false when the determinant is zero.
    /// </summary>
    public readonly bool Inverted(out NCollection_Mat4<T> outMx, out T det)
    {
        // use short-cut for better readability (a copy, read before theOutMx is written in case they alias)
        NCollection_Buffer16<T> m = _mat;

        outMx = new();
        ref NCollection_Buffer16<T> inv = ref outMx._mat;

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

        det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        if (det == T.Zero)
        {
            return false;
        }

        T div = T.One / det;
        for (int i = 0; i < 16; ++i)
        {
            inv[i] *= div;
        }

        return true;
    }

    public readonly bool Inverted(out NCollection_Mat4<T> outMx) => Inverted(out outMx, out _);

    /// <summary>The inverse; throws Standard_ConstructionError when the determinant is zero.</summary>
    public readonly NCollection_Mat4<T> Inverted()
    {
        if (!Inverted(out NCollection_Mat4<T> inv))
        {
            throw new ConstructionErrorException("NCollection_Mat4::Inverted() - matrix has zero determinant");
        }

        return inv;
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
        NCollection_Mat4<T> mat = new();
        mat.SetRow(0, CrossVec4(GetRow(1), GetRow(2), GetRow(3)));
        mat.SetRow(1, CrossVec4(-GetRow(0), GetRow(2), GetRow(3)));
        mat.SetRow(2, CrossVec4(GetRow(0), GetRow(1), GetRow(3)));
        mat.SetRow(3, CrossVec4(-GetRow(0), GetRow(1), GetRow(2)));
        return mat;
    }

    private static NCollection_Vec4<T> CrossVec4(in NCollection_Vec4<T> a, in NCollection_Vec4<T> b, in NCollection_Vec4<T> c)
    {
        T d1 = (b.Z() * c.W()) - (b.W() * c.Z());
        T d2 = (b.Y() * c.W()) - (b.W() * c.Y());
        T d3 = (b.Y() * c.Z()) - (b.Z() * c.Y());
        T d4 = (b.X() * c.W()) - (b.W() * c.X());
        T d5 = (b.X() * c.Z()) - (b.Z() * c.X());
        T d6 = (b.X() * c.Y()) - (b.Y() * c.X());

        NCollection_Vec4<T> vec = new();
        vec[0] = -a.Y() * d1 + a.Z() * d2 - a.W() * d3;
        vec[1] = a.X() * d1 - a.Z() * d4 + a.W() * d5;
        vec[2] = -a.X() * d2 + a.Y() * d4 - a.W() * d6;
        vec[3] = a.X() * d3 - a.Y() * d5 + a.Z() * d6;
        return vec;
    }

    /// <summary>Copies the 16 elements, column by column, into a span (OCCT's <c>GetData()</c>).</summary>
    public readonly void CopyTo(Span<T> destination) => ((ReadOnlySpan<T>)_mat).CopyTo(destination);

    public static NCollection_Vec4<T> operator *(in NCollection_Mat4<T> mat, in NCollection_Vec4<T> vec)
        => new(mat.GetValue(0, 0) * vec.X() + mat.GetValue(0, 1) * vec.Y() + mat.GetValue(0, 2) * vec.Z()
                 + mat.GetValue(0, 3) * vec.W(),
               mat.GetValue(1, 0) * vec.X() + mat.GetValue(1, 1) * vec.Y() + mat.GetValue(1, 2) * vec.Z()
                 + mat.GetValue(1, 3) * vec.W(),
               mat.GetValue(2, 0) * vec.X() + mat.GetValue(2, 1) * vec.Y() + mat.GetValue(2, 2) * vec.Z()
                 + mat.GetValue(2, 3) * vec.W(),
               mat.GetValue(3, 0) * vec.X() + mat.GetValue(3, 1) * vec.Y() + mat.GetValue(3, 2) * vec.Z()
                 + mat.GetValue(3, 3) * vec.W());

    public static NCollection_Mat4<T> operator *(in NCollection_Mat4<T> left, in NCollection_Mat4<T> right) => left.Multiplied(right);

    public static NCollection_Mat4<T> operator *(in NCollection_Mat4<T> mat, T factor) => mat.Multiplied(factor);

    public static NCollection_Mat4<T> operator /(in NCollection_Mat4<T> mat, T scalar) => mat.Divided(scalar);

    public static NCollection_Mat4<T> operator +(in NCollection_Mat4<T> left, in NCollection_Mat4<T> right) => left.Added(right);

    public static NCollection_Mat4<T> operator -(in NCollection_Mat4<T> left, in NCollection_Mat4<T> right) => left.Subtracted(right);

    public static NCollection_Mat4<T> operator -(in NCollection_Mat4<T> mat) => mat.Negated();

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Mat4<T> other)
    {
        for (int i = 0; i < 16; ++i)
        {
            if (!_mat[i].Equals(other._mat[i]))
            {
                return false;
            }
        }

        return true;
    }

    public override readonly bool Equals(object? obj) => obj is NCollection_Mat4<T> other && Equals(other);

    public override readonly int GetHashCode()
    {
        var hash = new HashCode();
        for (int i = 0; i < 16; ++i)
        {
            hash.Add(_mat[i]);
        }

        return hash.ToHashCode();
    }

    public static bool operator ==(in NCollection_Mat4<T> left, in NCollection_Mat4<T> right) => left.Equals(right);

    public static bool operator !=(in NCollection_Mat4<T> left, in NCollection_Mat4<T> right) => !left.Equals(right);

    /// <summary>The elements row by row.</summary>
    public override readonly string ToString()
    {
        Span<T> rows = stackalloc T[16];
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                rows[r * 4 + c] = GetValue(r, c);
            }
        }

        return NCollection_VecTools.Format<T>("NCollection_Mat4", rows);
    }
}
