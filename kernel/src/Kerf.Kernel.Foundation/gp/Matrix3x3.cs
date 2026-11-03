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
public readonly struct Matrix3x3 : IEquatable<Matrix3x3>
{
    internal readonly double _m00, _m01, _m02;
    internal readonly double _m10, _m11, _m12;
    internal readonly double _m20, _m21, _m22;

    /// <summary>The fields, for the returning forms that take a whole value back (unit 10G).</summary>
    private void Deconstruct(out double m00, out double m01, out double m02, out double m10, out double m11, out double m12, out double m20, out double m21, out double m22)
    {
        m00 = _m00;
        m01 = _m01;
        m02 = _m02;
        m10 = _m10;
        m11 = _m11;
        m12 = _m12;
        m20 = _m20;
        m21 = _m21;
        m22 = _m22;
    }

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
    internal Matrix3x3(in Xyz col1, in Xyz col2, in Xyz col3)
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
    public static Matrix3x3 Identity => new(1, 0, 0, 0, 1, 0, 0, 0, 1);

    // Copies

    internal readonly Matrix3x3 WithCol(int col, in Xyz value)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;

        double m22 = _m22;
        Standard_OutOfRange_Raise_if(col < 1 || col > 3, " ");
        if (col == 1)
        {
            m00 = value.X();
            m10 = value.Y();
            m20 = value.Z();
        }
        else if (col == 2)
        {
            m01 = value.X();
            m11 = value.Y();
            m21 = value.Z();
        }
        else
        {
            m02 = value.X();
            m12 = value.Y();
            m22 = value.Z();
        }
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    internal readonly Matrix3x3 WithCols(in Xyz col1, in Xyz col2, in Xyz col3) => new(col1, col2, col3);

    internal readonly Matrix3x3 WithCross(in Xyz theRef)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        double x = theRef.X();
        double y = theRef.Y();
        double z = theRef.Z();
        m00 = m11 = m22 = 0.0;
        m01 = -z;
        m02 = y;
        m12 = -x;
        m10 = z;
        m20 = -y;
        m21 = x;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 WithDiagonal(double x1, double x2, double x3)
    {
        double m00 = _m00;
        double m11 = _m11;
        double m22 = _m22;
        m00 = x1;
        m11 = x2;
        m22 = x3;
        return new Matrix3x3(m00, _m01, _m02, _m10, m11, _m12, _m20, _m21, m22);
    }

    internal readonly Matrix3x3 WithDot(in Xyz theRef)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        double x = theRef.X();
        double y = theRef.Y();
        double z = theRef.Z();
        m00 = x * x;
        m11 = y * y;
        m22 = z * z;
        m01 = x * y;
        m02 = x * z;
        m12 = y * z;
        m10 = m01;
        m20 = m02;
        m21 = m12;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 WithIdentity() => Identity;

    internal readonly Matrix3x3 WithRotation(in Xyz axis, double ang)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
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
        m00 = 1.0 + omCos * (-(b2 + c2));
        m01 = omCos * ab - sin * c;
        m02 = omCos * ac + sin * b;
        m10 = omCos * ab + sin * c;
        m11 = 1.0 + omCos * (-(a2 + c2));
        m12 = omCos * bc - sin * a;
        m20 = omCos * ac - sin * b;
        m21 = omCos * bc + sin * a;
        m22 = 1.0 + omCos * (-(a2 + b2));
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    internal readonly Matrix3x3 WithRow(int row, in Xyz value)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        Standard_OutOfRange_Raise_if(row < 1 || row > 3, " ");
        if (row == 1)
        {
            m00 = value.X();
            m01 = value.Y();
            m02 = value.Z();
        }
        else if (row == 2)
        {
            m10 = value.X();
            m11 = value.Y();
            m12 = value.Z();
        }
        else
        {
            m20 = value.X();
            m21 = value.Y();
            m22 = value.Z();
        }
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    internal readonly Matrix3x3 WithRows(in Xyz row1, in Xyz row2, in Xyz row3)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        m00 = row1.X();
        m01 = row1.Y();
        m02 = row1.Z();
        m10 = row2.X();
        m11 = row2.Y();
        m12 = row2.Z();
        m20 = row3.X();
        m21 = row3.Y();
        m22 = row3.Z();
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 WithScale(double s)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        m00 = m11 = m22 = s;
        m01 = m02 = m10 = m12 = m20 = m21 = 0.0;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    internal readonly Matrix3x3 WithValue(int row, int col, double value)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 3, " ");
        Span<double> m = [_m00, _m01, _m02, _m10, _m11, _m12, _m20, _m21, _m22];
        m[(row - 1) * 3 + (col - 1)] = value;
        return new Matrix3x3(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    }

    // Queries

    internal readonly Xyz Column(int col)
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

    public readonly double Determinant
    {
        get
        {
            double a00 = _m00, a01 = _m01, a02 = _m02;
            double a10 = _m10, a11 = _m11, a12 = _m12;
            double a20 = _m20, a21 = _m21, a22 = _m22;
            return a00 * (a11 * a22 - a21 * a12) - a01 * (a10 * a22 - a20 * a12) + a02 * (a10 * a21 - a20 * a11);
        }
    }

    internal readonly Xyz Diagonal() => new(_m00, _m11, _m22);

    internal readonly Xyz Row(int row)
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

    internal readonly double Value(int row, int col)
    {
        Standard_OutOfRange_Raise_if(row < 1 || row > 3 || col < 1 || col > 3, " ");
        return Unsafe.Add(ref Unsafe.AsRef(in _m00), (row - 1) * 3 + (col - 1));
    }

    /// <summary>The element at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    internal readonly double this[int row, int col] => Value(row, col);

    public readonly bool IsSingular
    {
        get
        {
            double det = Determinant;
            return (det < 0.0 ? -det : det) <= Constants.Resolution();
        }
    }

    public readonly Matrix3x3 Added(in Matrix3x3 other)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        m00 += other._m00;
        m01 += other._m01;
        m02 += other._m02;
        m10 += other._m10;
        m11 += other._m11;
        m12 += other._m12;
        m20 += other._m20;
        m21 += other._m21;
        m22 += other._m22;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 Divided(double scalar)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        Standard_ConstructionError_Raise_if((scalar < 0.0 ? -scalar : scalar) <= Constants.Resolution(), "gp_Mat : Divide by 0");
        double unSurScalar = 1.0 / scalar;
        (m00, m01, m02, m10, m11, m12, m20, m21, m22) = new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22).Multiplied(unSurScalar);
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 Inverted()
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
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

        double det = a00 * adj00 + a01 * adj10 + a02 * adj20;
        Standard_ConstructionError_Raise_if(Math.Abs(det) <= Constants.Resolution(), "gp_Mat::Invert() - matrix has zero determinant");

        double invDet = 1.0 / det;
        m00 = adj00 * invDet;
        m10 = adj10 * invDet;
        m20 = adj20 * invDet;
        m01 = adj01 * invDet;
        m11 = adj11 * invDet;
        m21 = adj21 * invDet;
        m02 = adj02 * invDet;
        m12 = adj12 * invDet;
        m22 = adj22 * invDet;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 Multiplied(in Matrix3x3 other)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        double t00 = m00 * other._m00 + m01 * other._m10 + m02 * other._m20;
        double t01 = m00 * other._m01 + m01 * other._m11 + m02 * other._m21;
        double t02 = m00 * other._m02 + m01 * other._m12 + m02 * other._m22;
        double t10 = m10 * other._m00 + m11 * other._m10 + m12 * other._m20;
        double t11 = m10 * other._m01 + m11 * other._m11 + m12 * other._m21;
        double t12 = m10 * other._m02 + m11 * other._m12 + m12 * other._m22;
        double t20 = m20 * other._m00 + m21 * other._m10 + m22 * other._m20;
        double t21 = m20 * other._m01 + m21 * other._m11 + m22 * other._m21;
        double t22 = m20 * other._m02 + m21 * other._m12 + m22 * other._m22;
        m00 = t00;
        m01 = t01;
        m02 = t02;
        m10 = t10;
        m11 = t11;
        m12 = t12;
        m20 = t20;
        m21 = t21;
        m22 = t22;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 PreMultiplied(in Matrix3x3 other)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        double t00 = other._m00 * m00 + other._m01 * m10 + other._m02 * m20;
        double t01 = other._m00 * m01 + other._m01 * m11 + other._m02 * m21;
        double t02 = other._m00 * m02 + other._m01 * m12 + other._m02 * m22;
        double t10 = other._m10 * m00 + other._m11 * m10 + other._m12 * m20;
        double t11 = other._m10 * m01 + other._m11 * m11 + other._m12 * m21;
        double t12 = other._m10 * m02 + other._m11 * m12 + other._m12 * m22;
        double t20 = other._m20 * m00 + other._m21 * m10 + other._m22 * m20;
        double t21 = other._m20 * m01 + other._m21 * m11 + other._m22 * m21;
        double t22 = other._m20 * m02 + other._m21 * m12 + other._m22 * m22;
        m00 = t00;
        m01 = t01;
        m02 = t02;
        m10 = t10;
        m11 = t11;
        m12 = t12;
        m20 = t20;
        m21 = t21;
        m22 = t22;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 Multiplied(double scalar)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        m00 *= scalar;
        m01 *= scalar;
        m02 *= scalar;
        m10 *= scalar;
        m11 *= scalar;
        m12 *= scalar;
        m20 *= scalar;
        m21 *= scalar;
        m22 *= scalar;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 Powered(int n)
    {
        if (n == 0)
        {
            return Identity;
        }

        if (n == 1)
        {
            return this;
        }

        if (n == -1)
        {
            return Inverted();
        }

        bool isNegative = n < 0;
        Matrix3x3 aBase = isNegative ? Inverted() : this;
        int power = isNegative ? -n : n;
        Matrix3x3 result = Identity;
        while (power > 0)
        {
            if ((power & 1) != 0)
            {
                result = result.Multiplied(aBase);
            }

            aBase = aBase.Multiplied(aBase);
            power >>= 1;
        }

        return result;
    }

    public readonly Matrix3x3 Subtracted(in Matrix3x3 other)
    {
        double m00 = _m00;
        double m01 = _m01;
        double m02 = _m02;
        double m10 = _m10;
        double m11 = _m11;
        double m12 = _m12;
        double m20 = _m20;
        double m21 = _m21;
        double m22 = _m22;
        m00 -= other._m00;
        m01 -= other._m01;
        m02 -= other._m02;
        m10 -= other._m10;
        m11 -= other._m11;
        m12 -= other._m12;
        m20 -= other._m20;
        m21 -= other._m21;
        m22 -= other._m22;
        return new Matrix3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public readonly Matrix3x3 Transposed()
    {
        return new Matrix3x3(_m00, _m10, _m20, _m01, _m11, _m21, _m02, _m12, _m22);
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
