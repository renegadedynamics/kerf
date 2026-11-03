// Port of: src/FoundationClasses/TKMath/gp/gp_Mat.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Mat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Kerf.Geometry;

/// <summary>A 3x3 matrix. Rows and columns are numbered from 1, as in OCCT.</summary>
/// <remarks>OCCT's <c>double myMat[3][3]</c> is nine fields here; <c>myMat[i][j]</c> is <c>m{i}{j}</c>.</remarks>
[StructLayout(LayoutKind.Sequential)]
public struct Matrix3x3 : IEquatable<Matrix3x3>
{
    internal double _m00, _m01, _m02;
    internal double _m10, _m11, _m12;
    internal double _m20, _m21, _m22;

    public Matrix3x3(double a11, double a12, double a13,
                  double a21, double a22, double a23,
                  double a31, double a32, double a33)
    {
        _m00 = a11;
        _m01 = a12;
        _m02 = a13;
        _m10 = a21;
        _m11 = a22;
        _m12 = a23;
        _m20 = a31;
        _m21 = a32;
        _m22 = a33;
    }

    /// <summary>A matrix whose columns are the three triples.</summary>
    public Matrix3x3(in Xyz col1, in Xyz col2, in Xyz col3)
    {
        _m00 = col1.X();
        _m10 = col1.Y();
        _m20 = col1.Z();
        _m01 = col2.X();
        _m11 = col2.Y();
        _m21 = col2.Z();
        _m02 = col3.X();
        _m12 = col3.Y();
        _m22 = col3.Z();
    }

    /// <summary>The identity matrix.</summary>
    public static Matrix3x3 Identity() => new(1, 0, 0, 0, 1, 0, 0, 0, 1);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCol(int col, in Xyz value)
    {
        Standard_OutOfRange_Raise_if(col < 1 || col > 3, " ");
        if (col == 1)
        {
            _m00 = value.X();
            _m10 = value.Y();
            _m20 = value.Z();
        }
        else if (col == 2)
        {
            _m01 = value.X();
            _m11 = value.Y();
            _m21 = value.Z();
        }
        else
        {
            _m02 = value.X();
            _m12 = value.Y();
            _m22 = value.Z();
        }
    }

    internal void SetCols(in Xyz col1, in Xyz col2, in Xyz col3) => this = new Matrix3x3(col1, col2, col3);

    internal void SetCross(in Xyz theRef)
    {
        double x = theRef.X();
        double y = theRef.Y();
        double z = theRef.Z();
        _m00 = _m11 = _m22 = 0.0;
        _m01 = -z;
        _m02 = y;
        _m12 = -x;
        _m10 = z;
        _m20 = -y;
        _m21 = x;
    }

    internal void SetDiagonal(double x1, double x2, double x3)
    {
        _m00 = x1;
        _m11 = x2;
        _m22 = x3;
    }

    internal void SetDot(in Xyz theRef)
    {
        double x = theRef.X();
        double y = theRef.Y();
        double z = theRef.Z();
        _m00 = x * x;
        _m11 = y * y;
        _m22 = z * z;
        _m01 = x * y;
        _m02 = x * z;
        _m12 = y * z;
        _m10 = _m01;
        _m20 = _m02;
        _m21 = _m12;
    }

    internal void SetIdentity()
    {
        _m00 = _m11 = _m22 = 1.0;
        _m01 = _m02 = _m10 = _m12 = _m20 = _m21 = 0.0;
    }

    internal void SetRotation(in Xyz axis, double ang)
    {
        Xyz v = axis.Normalized();
        double a = v.X();
        double b = v.Y();
        double c = v.Z();
        double cos = Math.Cos(ang);
        double sin = Math.Sin(ang);
        double omCos = 1.0 - cos;
        double a2 = a * a;
        double b2 = b * b;
        double c2 = c * c;
        double ab = a * b;
        double ac = a * c;
        double bc = b * c;
        _m00 = 1.0 + omCos * (-(b2 + c2));
        _m01 = omCos * ab - sin * c;
        _m02 = omCos * ac + sin * b;
        _m10 = omCos * ab + sin * c;
        _m11 = 1.0 + omCos * (-(a2 + c2));
        _m12 = omCos * bc - sin * a;
        _m20 = omCos * ac - sin * b;
        _m21 = omCos * bc + sin * a;
        _m22 = 1.0 + omCos * (-(a2 + b2));
    }

    internal void SetRow(int row, in Xyz value)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3, " ");
        if (row == 1)
        {
            _m00 = value.X();
            _m01 = value.Y();
            _m02 = value.Z();
        }
        else if (row == 2)
        {
            _m10 = value.X();
            _m11 = value.Y();
            _m12 = value.Z();
        }
        else
        {
            _m20 = value.X();
            _m21 = value.Y();
            _m22 = value.Z();
        }
    }

    internal void SetRows(in Xyz row1, in Xyz row2, in Xyz row3)
    {
        _m00 = row1.X();
        _m01 = row1.Y();
        _m02 = row1.Z();
        _m10 = row2.X();
        _m11 = row2.Y();
        _m12 = row2.Z();
        _m20 = row3.X();
        _m21 = row3.Y();
        _m22 = row3.Z();
    }

    internal void SetScale(double s)
    {
        _m00 = _m11 = _m22 = s;
        _m01 = _m02 = _m10 = _m12 = _m20 = _m21 = 0.0;
    }

    internal void SetValue(int row, int col, double value) => ChangeValue(row, col) = value;

    [UnscopedRef]
    internal ref double ChangeValue(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 3, " ");
        return ref Unsafe.Add(ref _m00, (row - 1) * 3 + (col - 1));
    }

    internal void Add(in Matrix3x3 other)
    {
        _m00 += other._m00;
        _m01 += other._m01;
        _m02 += other._m02;
        _m10 += other._m10;
        _m11 += other._m11;
        _m12 += other._m12;
        _m20 += other._m20;
        _m21 += other._m21;
        _m22 += other._m22;
    }

    internal void Divide(double scalar)
    {
        Standard_ConstructionError_Raise_if((scalar < 0.0 ? -scalar : scalar) <= Constants.Resolution(), "gp_Mat : Divide by 0");
        double unSurScalar = 1.0 / scalar;
        Multiply(unSurScalar);
    }

    internal void Invert()
    {
        double a00 = _m00, a01 = _m01, a02 = _m02;
        double a10 = _m10, a11 = _m11, a12 = _m12;
        double a20 = _m20, a21 = _m21, a22 = _m22;

        double adj00 = a11 * a22 - a12 * a21;
        double adj10 = a12 * a20 - a10 * a22;
        double adj20 = a10 * a21 - a11 * a20;
        double adj01 = a02 * a21 - a01 * a22;
        double adj11 = a00 * a22 - a02 * a20;
        double adj21 = a01 * a20 - a00 * a21;
        double adj02 = a01 * a12 - a02 * a11;
        double adj12 = a02 * a10 - a00 * a12;
        double adj22 = a00 * a11 - a01 * a10;

        double det = a00 * adj00 + a01 * adj10 + a02 * adj20;
        Standard_ConstructionError_Raise_if(Math.Abs(det) <= Constants.Resolution(), "gp_Mat::Invert() - matrix has zero determinant");

        double invDet = 1.0 / det;
        _m00 = adj00 * invDet;
        _m10 = adj10 * invDet;
        _m20 = adj20 * invDet;
        _m01 = adj01 * invDet;
        _m11 = adj11 * invDet;
        _m21 = adj21 * invDet;
        _m02 = adj02 * invDet;
        _m12 = adj12 * invDet;
        _m22 = adj22 * invDet;
    }

    internal void Multiply(in Matrix3x3 other)
    {
        double t00 = _m00 * other._m00 + _m01 * other._m10 + _m02 * other._m20;
        double t01 = _m00 * other._m01 + _m01 * other._m11 + _m02 * other._m21;
        double t02 = _m00 * other._m02 + _m01 * other._m12 + _m02 * other._m22;
        double t10 = _m10 * other._m00 + _m11 * other._m10 + _m12 * other._m20;
        double t11 = _m10 * other._m01 + _m11 * other._m11 + _m12 * other._m21;
        double t12 = _m10 * other._m02 + _m11 * other._m12 + _m12 * other._m22;
        double t20 = _m20 * other._m00 + _m21 * other._m10 + _m22 * other._m20;
        double t21 = _m20 * other._m01 + _m21 * other._m11 + _m22 * other._m21;
        double t22 = _m20 * other._m02 + _m21 * other._m12 + _m22 * other._m22;
        _m00 = t00;
        _m01 = t01;
        _m02 = t02;
        _m10 = t10;
        _m11 = t11;
        _m12 = t12;
        _m20 = t20;
        _m21 = t21;
        _m22 = t22;
    }

    internal void PreMultiply(in Matrix3x3 other)
    {
        double t00 = other._m00 * _m00 + other._m01 * _m10 + other._m02 * _m20;
        double t01 = other._m00 * _m01 + other._m01 * _m11 + other._m02 * _m21;
        double t02 = other._m00 * _m02 + other._m01 * _m12 + other._m02 * _m22;
        double t10 = other._m10 * _m00 + other._m11 * _m10 + other._m12 * _m20;
        double t11 = other._m10 * _m01 + other._m11 * _m11 + other._m12 * _m21;
        double t12 = other._m10 * _m02 + other._m11 * _m12 + other._m12 * _m22;
        double t20 = other._m20 * _m00 + other._m21 * _m10 + other._m22 * _m20;
        double t21 = other._m20 * _m01 + other._m21 * _m11 + other._m22 * _m21;
        double t22 = other._m20 * _m02 + other._m21 * _m12 + other._m22 * _m22;
        _m00 = t00;
        _m01 = t01;
        _m02 = t02;
        _m10 = t10;
        _m11 = t11;
        _m12 = t12;
        _m20 = t20;
        _m21 = t21;
        _m22 = t22;
    }

    internal void Multiply(double scalar)
    {
        _m00 *= scalar;
        _m01 *= scalar;
        _m02 *= scalar;
        _m10 *= scalar;
        _m11 *= scalar;
        _m12 *= scalar;
        _m20 *= scalar;
        _m21 *= scalar;
        _m22 *= scalar;
    }

    internal void Power(int n)
    {
        if (n == 0)
        {
            SetIdentity();
            return;
        }

        if (n == 1)
        {
            return;
        }

        if (n == -1)
        {
            Invert();
            return;
        }

        bool isNegative = n < 0;
        if (isNegative)
        {
            Invert();
        }

        int power = isNegative ? -n : n;
        Matrix3x3 aBase = this;
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

    internal void Subtract(in Matrix3x3 other)
    {
        _m00 -= other._m00;
        _m01 -= other._m01;
        _m02 -= other._m02;
        _m10 -= other._m10;
        _m11 -= other._m11;
        _m12 -= other._m12;
        _m20 -= other._m20;
        _m21 -= other._m21;
        _m22 -= other._m22;
    }

    internal void Transpose()
    {
        (_m01, _m10) = (_m10, _m01);
        (_m02, _m20) = (_m20, _m02);
        (_m12, _m21) = (_m21, _m12);
    }

    // Copies with a mutator applied

    public readonly Matrix3x3 WithCol(int col, in Xyz value)
    {
        Matrix3x3 copy = this;
        copy.SetCol(col, value);
        return copy;
    }

    public readonly Matrix3x3 WithCols(in Xyz col1, in Xyz col2, in Xyz col3) => new(col1, col2, col3);

    public readonly Matrix3x3 WithCross(in Xyz theRef)
    {
        Matrix3x3 copy = this;
        copy.SetCross(theRef);
        return copy;
    }

    public readonly Matrix3x3 WithDiagonal(double x1, double x2, double x3)
    {
        Matrix3x3 copy = this;
        copy.SetDiagonal(x1, x2, x3);
        return copy;
    }

    public readonly Matrix3x3 WithDot(in Xyz theRef)
    {
        Matrix3x3 copy = this;
        copy.SetDot(theRef);
        return copy;
    }

    public readonly Matrix3x3 WithIdentity() => Identity();

    public readonly Matrix3x3 WithRotation(in Xyz axis, double ang)
    {
        Matrix3x3 copy = this;
        copy.SetRotation(axis, ang);
        return copy;
    }

    public readonly Matrix3x3 WithRow(int row, in Xyz value)
    {
        Matrix3x3 copy = this;
        copy.SetRow(row, value);
        return copy;
    }

    public readonly Matrix3x3 WithRows(in Xyz row1, in Xyz row2, in Xyz row3)
    {
        Matrix3x3 copy = this;
        copy.SetRows(row1, row2, row3);
        return copy;
    }

    public readonly Matrix3x3 WithScale(double s)
    {
        Matrix3x3 copy = this;
        copy.SetScale(s);
        return copy;
    }

    public readonly Matrix3x3 WithValue(int row, int col, double value)
    {
        Matrix3x3 copy = this;
        copy.SetValue(row, col, value);
        return copy;
    }

    // Queries

    public readonly Xyz Column(int col)
    {
        Standard_OutOfRange_Raise_if(col < 1 || col > 3, "gp_Mat::Column() - wrong index");
        if (col == 1)
        {
            return new Xyz(_m00, _m10, _m20);
        }

        if (col == 2)
        {
            return new Xyz(_m01, _m11, _m21);
        }

        return new Xyz(_m02, _m12, _m22);
    }

    public readonly double Determinant()
    {
        double a00 = _m00, a01 = _m01, a02 = _m02;
        double a10 = _m10, a11 = _m11, a12 = _m12;
        double a20 = _m20, a21 = _m21, a22 = _m22;
        return a00 * (a11 * a22 - a21 * a12) - a01 * (a10 * a22 - a20 * a12) + a02 * (a10 * a21 - a20 * a11);
    }

    public readonly Xyz Diagonal() => new(_m00, _m11, _m22);

    public readonly Xyz Row(int row)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3, "gp_Mat::Row() - wrong index");
        if (row == 1)
        {
            return new Xyz(_m00, _m01, _m02);
        }

        if (row == 2)
        {
            return new Xyz(_m10, _m11, _m12);
        }

        return new Xyz(_m20, _m21, _m22);
    }

    public readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 3, " ");
        return Unsafe.Add(ref Unsafe.AsRef(in _m00), (row - 1) * 3 + (col - 1));
    }

    /// <summary>The element at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    public readonly double this[int row, int col] => Value(row, col);

    public readonly bool IsSingular()
    {
        double det = Determinant();
        return (det < 0.0 ? -det : det) <= Constants.Resolution();
    }

    public readonly Matrix3x3 Added(in Matrix3x3 other)
    {
        Matrix3x3 newMat = this;
        newMat.Add(other);
        return newMat;
    }

    public readonly Matrix3x3 Divided(double scalar)
    {
        Matrix3x3 newMat = this;
        newMat.Divide(scalar);
        return newMat;
    }

    public readonly Matrix3x3 Inverted()
    {
        Matrix3x3 newMat = this;
        newMat.Invert();
        return newMat;
    }

    public readonly Matrix3x3 Multiplied(in Matrix3x3 other)
    {
        Matrix3x3 newMat = this;
        newMat.Multiply(other);
        return newMat;
    }

    public readonly Matrix3x3 PreMultiplied(in Matrix3x3 other)
    {
        Matrix3x3 newMat = this;
        newMat.PreMultiply(other);
        return newMat;
    }

    public readonly Matrix3x3 Multiplied(double scalar)
    {
        Matrix3x3 newMat = this;
        newMat.Multiply(scalar);
        return newMat;
    }

    public readonly Matrix3x3 Powered(int n)
    {
        Matrix3x3 matN = this;
        matN.Power(n);
        return matN;
    }

    public readonly Matrix3x3 Subtracted(in Matrix3x3 other)
    {
        Matrix3x3 newMat = this;
        newMat.Subtract(other);
        return newMat;
    }

    public readonly Matrix3x3 Transposed()
    {
        Matrix3x3 newMat = this;
        newMat.Transpose();
        return newMat;
    }

    /// <summary>The nine elements, row by row.</summary>
    public readonly void CopyTo(Span<double> destination)
        => MemoryMarshal.CreateReadOnlySpan(ref Unsafe.AsRef(in _m00), 9).CopyTo(destination);

    public static Matrix3x3 operator +(in Matrix3x3 left, in Matrix3x3 right) => left.Added(right);

    public static Matrix3x3 operator -(in Matrix3x3 left, in Matrix3x3 right) => left.Subtracted(right);

    public static Matrix3x3 operator *(in Matrix3x3 left, in Matrix3x3 right) => left.Multiplied(right);

    public static Matrix3x3 operator *(in Matrix3x3 mat, double scalar) => mat.Multiplied(scalar);

    public static Matrix3x3 operator *(double scalar, in Matrix3x3 mat) => mat.Multiplied(scalar);

    public static Matrix3x3 operator /(in Matrix3x3 mat, double scalar) => mat.Divided(scalar);

    public readonly bool Equals(Matrix3x3 other)
        => _m00.Equals(other._m00) && _m01.Equals(other._m01) && _m02.Equals(other._m02)
           && _m10.Equals(other._m10) && _m11.Equals(other._m11) && _m12.Equals(other._m12)
           && _m20.Equals(other._m20) && _m21.Equals(other._m21) && _m22.Equals(other._m22);

    public override readonly bool Equals(object? obj) => obj is Matrix3x3 other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(HashCode.Combine(_m00, _m01, _m02), HashCode.Combine(_m10, _m11, _m12), HashCode.Combine(_m20, _m21, _m22));

    public static bool operator ==(in Matrix3x3 left, in Matrix3x3 right) => left.Equals(right);

    public static bool operator !=(in Matrix3x3 left, in Matrix3x3 right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_Mat", _m00, _m01, _m02, _m10, _m11, _m12, _m20, _m21, _m22);
}
