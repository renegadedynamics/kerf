// Port of: src/FoundationClasses/TKMath/gp/gp_Mat2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Mat2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Geometry;

/// <summary>A 2x2 matrix. Rows and columns are numbered from 1, as in OCCT.</summary>
/// <remarks>OCCT's <c>double myMat[2][2]</c> is four fields here; <c>myMat[i][j]</c> is <c>m{i}{j}</c>.</remarks>
[StructLayout(LayoutKind.Sequential)]
public struct Matrix2x2 : IEquatable<Matrix2x2>
{
    internal double _m00, _m01;
    internal double _m10, _m11;

    /// <summary>A matrix whose columns are the two pairs.</summary>
    public Matrix2x2(in Xy col1, in Xy col2)
    {
        _m00 = col1.X();
        _m10 = col1.Y();
        _m01 = col2.X();
        _m11 = col2.Y();
    }

    /// <summary>The identity matrix.</summary>
    public static Matrix2x2 Identity()
    {
        Matrix2x2 mat = new();
        mat.SetIdentity();
        return mat;
    }

    /// <summary>The matrix of the rotation by an angle in radians.</summary>
    public static Matrix2x2 Rotation(double ang)
    {
        Matrix2x2 mat = new();
        mat.SetRotation(ang);
        return mat;
    }

    /// <summary>The matrix of the scale by a factor.</summary>
    public static Matrix2x2 Scale(double s)
    {
        Matrix2x2 mat = new();
        mat.SetScale(s);
        return mat;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCol(int col, in Xy value)
    {
        Standard_OutOfRange_Raise_if(col < 1 || col > 2, "gp_Mat2d::SetCol() - invalid index");
        if (col == 1)
        {
            _m00 = value.X();
            _m10 = value.Y();
        }
        else
        {
            _m01 = value.X();
            _m11 = value.Y();
        }
    }

    internal void SetCols(in Xy col1, in Xy col2)
    {
        _m00 = col1.X();
        _m10 = col1.Y();
        _m01 = col2.X();
        _m11 = col2.Y();
    }

    internal void SetDiagonal(double x1, double x2)
    {
        _m00 = x1;
        _m11 = x2;
    }

    internal void SetIdentity()
    {
        _m00 = _m11 = 1.0;
        _m01 = _m10 = 0.0;
    }

    internal void SetRotation(double ang)
    {
        double sinA = Math.Sin(ang);
        double cosA = Math.Cos(ang);
        _m00 = _m11 = cosA;
        _m01 = -sinA;
        _m10 = sinA;
    }

    internal void SetRow(int row, in Xy value)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2, "gp_Mat2d::SetRow() - invalid index");
        if (row == 1)
        {
            _m00 = value.X();
            _m01 = value.Y();
        }
        else
        {
            _m10 = value.X();
            _m11 = value.Y();
        }
    }

    internal void SetRows(in Xy row1, in Xy row2)
    {
        _m00 = row1.X();
        _m01 = row1.Y();
        _m10 = row2.X();
        _m11 = row2.Y();
    }

    internal void SetScale(double s)
    {
        _m00 = _m11 = s;
        _m01 = _m10 = 0.0;
    }

    internal void SetValue(int row, int col, double value) => ChangeValue(row, col) = value;

    [UnscopedRef]
    internal ref double ChangeValue(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2 || col < 1 || col > 2, " ");
        return ref Unsafe.Add(ref _m00, (row - 1) * 2 + (col - 1));
    }

    internal void Add(in Matrix2x2 other)
    {
        _m00 += other._m00;
        _m01 += other._m01;
        _m10 += other._m10;
        _m11 += other._m11;
    }

    internal void Divide(double scalar)
    {
        _m00 /= scalar;
        _m01 /= scalar;
        _m10 /= scalar;
        _m11 /= scalar;
    }

    internal void Invert()
    {
        double newMat00 = _m11;
        double newMat01 = -_m01;
        double newMat10 = -_m10;
        double newMat11 = _m00;
        double det = newMat00 * newMat11 - newMat01 * newMat10;
        Standard_ConstructionError_Raise_if(Math.Abs(det) <= Constants.Resolution(), "gp_Mat2d::Invert() - matrix has zero determinant");
        det = 1.0 / det;
        _m00 = newMat00 * det;
        _m10 = newMat10 * det;
        _m01 = newMat01 * det;
        _m11 = newMat11 * det;
    }

    internal void Multiply(in Matrix2x2 other)
    {
        // other's values are read first: it may be this matrix (M.Multiply(M) squares it; decision 1485)
        double o00 = other._m00, o01 = other._m01, o10 = other._m10, o11 = other._m11;
        double t00 = _m00 * o00 + _m01 * o10;
        double t10 = _m10 * o00 + _m11 * o10;
        _m01 = _m00 * o01 + _m01 * o11;
        _m11 = _m10 * o01 + _m11 * o11;
        _m00 = t00;
        _m10 = t10;
    }

    internal void PreMultiply(in Matrix2x2 other)
    {
        double t00 = other._m00 * _m00 + other._m01 * _m10;
        _m10 = other._m10 * _m00 + other._m11 * _m10;
        double t01 = other._m00 * _m01 + other._m01 * _m11;
        _m11 = other._m10 * _m01 + other._m11 * _m11;
        _m00 = t00;
        _m01 = t01;
    }

    internal void Multiply(double scalar)
    {
        _m00 *= scalar;
        _m01 *= scalar;
        _m10 *= scalar;
        _m11 *= scalar;
    }

    internal void Power(int n)
    {
        if (n == 1)
        {
        }
        else if (n == 0)
        {
            SetIdentity();
        }
        else if (n == -1)
        {
            Invert();
        }
        else
        {
            if (n < 0)
            {
                Invert();
            }

            int npower = Math.Abs(n) - 1;
            Matrix2x2 temp = this;
            for (;;)
            {
                if (IsOdd(npower))
                {
                    Multiply(temp);
                }

                if (npower == 1)
                {
                    break;
                }

                temp.Multiply(temp);
                npower = npower / 2;
            }
        }
    }

    internal void Subtract(in Matrix2x2 other)
    {
        _m00 -= other._m00;
        _m01 -= other._m01;
        _m10 -= other._m10;
        _m11 -= other._m11;
    }

    internal void Transpose()
    {
        double temp = _m01;
        _m01 = _m10;
        _m10 = temp;
    }

    // Copies with a mutator applied

    public readonly Matrix2x2 WithCol(int col, in Xy value)
    {
        Matrix2x2 copy = this;
        copy.SetCol(col, value);
        return copy;
    }

    public readonly Matrix2x2 WithCols(in Xy col1, in Xy col2) => new(col1, col2);

    public readonly Matrix2x2 WithDiagonal(double x1, double x2)
    {
        Matrix2x2 copy = this;
        copy.SetDiagonal(x1, x2);
        return copy;
    }

    public readonly Matrix2x2 WithIdentity() => Identity();

    public readonly Matrix2x2 WithRotation(double ang) => Rotation(ang);

    public readonly Matrix2x2 WithRow(int row, in Xy value)
    {
        Matrix2x2 copy = this;
        copy.SetRow(row, value);
        return copy;
    }

    public readonly Matrix2x2 WithRows(in Xy row1, in Xy row2)
    {
        Matrix2x2 copy = this;
        copy.SetRows(row1, row2);
        return copy;
    }

    public readonly Matrix2x2 WithScale(double s) => Scale(s);

    public readonly Matrix2x2 WithValue(int row, int col, double value)
    {
        Matrix2x2 copy = this;
        copy.SetValue(row, col, value);
        return copy;
    }

    // Queries

    public readonly Xy Column(int col)
    {
        Standard_OutOfRange_Raise_if(col < 1 || col > 2, "gp_Mat2d::Column() - invalid index");
        if (col == 1)
        {
            return new Xy(_m00, _m10);
        }

        return new Xy(_m01, _m11);
    }

    public readonly double Determinant() => _m00 * _m11 - _m10 * _m01;

    public readonly Xy Diagonal() => new(_m00, _m11);

    public readonly Xy Row(int row)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2, "gp_Mat2d::Row() - invalid index");
        if (row == 1)
        {
            return new Xy(_m00, _m01);
        }

        return new Xy(_m10, _m11);
    }

    public readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 2 || col < 1 || col > 2, " ");
        return Unsafe.Add(ref Unsafe.AsRef(in _m00), (row - 1) * 2 + (col - 1));
    }

    /// <summary>The element at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    public readonly double this[int row, int col] => Value(row, col);

    public readonly bool IsSingular()
    {
        double det = Determinant();
        if (det < 0)
        {
            det = -det;
        }

        return det <= Constants.Resolution();
    }

    public readonly Matrix2x2 Added(in Matrix2x2 other)
    {
        Matrix2x2 newMat2d = new();
        newMat2d._m00 = _m00 + other._m00;
        newMat2d._m01 = _m01 + other._m01;
        newMat2d._m10 = _m10 + other._m10;
        newMat2d._m11 = _m11 + other._m11;
        return newMat2d;
    }

    public readonly Matrix2x2 Divided(double scalar)
    {
        Matrix2x2 newMat2d = new();
        newMat2d._m00 = _m00 / scalar;
        newMat2d._m01 = _m01 / scalar;
        newMat2d._m10 = _m10 / scalar;
        newMat2d._m11 = _m11 / scalar;
        return newMat2d;
    }

    public readonly Matrix2x2 Inverted()
    {
        Matrix2x2 newMat = this;
        newMat.Invert();
        return newMat;
    }

    public readonly Matrix2x2 Multiplied(in Matrix2x2 other)
    {
        Matrix2x2 newMat2d = this;
        newMat2d.Multiply(other);
        return newMat2d;
    }

    public readonly Matrix2x2 Multiplied(double scalar)
    {
        Matrix2x2 newMat2d = new();
        newMat2d._m00 = _m00 * scalar;
        newMat2d._m01 = _m01 * scalar;
        newMat2d._m10 = _m10 * scalar;
        newMat2d._m11 = _m11 * scalar;
        return newMat2d;
    }

    public readonly Matrix2x2 PreMultiplied(in Matrix2x2 other)
    {
        Matrix2x2 newMat2d = this;
        newMat2d.PreMultiply(other);
        return newMat2d;
    }

    public readonly Matrix2x2 Powered(int n)
    {
        Matrix2x2 mat2dN = this;
        mat2dN.Power(n);
        return mat2dN;
    }

    public readonly Matrix2x2 Subtracted(in Matrix2x2 other)
    {
        Matrix2x2 newMat2d = new();
        newMat2d._m00 = _m00 - other._m00;
        newMat2d._m01 = _m01 - other._m01;
        newMat2d._m10 = _m10 - other._m10;
        newMat2d._m11 = _m11 - other._m11;
        return newMat2d;
    }

    public readonly Matrix2x2 Transposed()
    {
        Matrix2x2 newMat2d = new();
        newMat2d._m10 = _m01;
        newMat2d._m01 = _m10;
        newMat2d._m00 = _m00;
        newMat2d._m11 = _m11;
        return newMat2d;
    }

    /// <summary>The four elements, row by row.</summary>
    public readonly void CopyTo(Span<double> destination)
        => MemoryMarshal.CreateReadOnlySpan(ref Unsafe.AsRef(in _m00), 4).CopyTo(destination);

    public static Matrix2x2 operator +(in Matrix2x2 left, in Matrix2x2 right) => left.Added(right);

    public static Matrix2x2 operator -(in Matrix2x2 left, in Matrix2x2 right) => left.Subtracted(right);

    public static Matrix2x2 operator *(in Matrix2x2 left, in Matrix2x2 right) => left.Multiplied(right);

    public static Matrix2x2 operator *(in Matrix2x2 mat, double scalar) => mat.Multiplied(scalar);

    public static Matrix2x2 operator *(double scalar, in Matrix2x2 mat) => mat.Multiplied(scalar);

    public static Matrix2x2 operator /(in Matrix2x2 mat, double scalar) => mat.Divided(scalar);

    public readonly bool Equals(Matrix2x2 other)
        => _m00.Equals(other._m00) && _m01.Equals(other._m01) && _m10.Equals(other._m10) && _m11.Equals(other._m11);

    public override readonly bool Equals(object? obj) => obj is Matrix2x2 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_m00, _m01, _m10, _m11);

    public static bool operator ==(in Matrix2x2 left, in Matrix2x2 right) => left.Equals(right);

    public static bool operator !=(in Matrix2x2 left, in Matrix2x2 right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Mat2d", _m00, _m01, _m10, _m11);
}
