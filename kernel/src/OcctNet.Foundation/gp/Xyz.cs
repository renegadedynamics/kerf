// Port of: src/FoundationClasses/TKMath/gp/gp_XYZ.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_XYZ.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Geometry;

/// <summary>A triple of reals: the coordinates of a point, vector or direction, without their meaning.</summary>
public struct Xyz : IEquatable<Xyz>
{
    private double _x;
    private double _y;
    private double _z;

    public Xyz(double x, double y, double z)
    {
        _x = x;
        _y = y;
        _z = z;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(double x, double y, double z)
    {
        _x = x;
        _y = y;
        _z = z;
    }

    internal void SetCoord(int index, double xi)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3);
        if (index == 1)
        {
            _x = xi;
        }
        else if (index == 2)
        {
            _y = xi;
        }
        else
        {
            _z = xi;
        }
    }

    internal void SetX(double x) => _x = x;

    internal void SetY(double y) => _y = y;

    internal void SetZ(double z) => _z = z;

    [UnscopedRef]
    internal ref double ChangeCoord(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3);
        if (index == 1)
        {
            return ref _x;
        }
        else if (index == 2)
        {
            return ref _y;
        }

        return ref _z;
    }

    internal void Add(in Xyz other)
    {
        _x += other._x;
        _y += other._y;
        _z += other._z;
    }

    internal void Cross(in Xyz right)
    {
        double xresult = _y * right._z - _z * right._y;
        double yresult = _z * right._x - _x * right._z;
        _z = _x * right._y - _y * right._x;
        _x = xresult;
        _y = yresult;
    }

    internal void CrossCross(in Xyz coord1, in Xyz coord2)
    {
        double crossX = coord1._y * coord2._z - coord1._z * coord2._y;
        double crossY = coord1._z * coord2._x - coord1._x * coord2._z;
        double crossZ = coord1._x * coord2._y - coord1._y * coord2._x;
        double xresult = _y * crossZ - _z * crossY;
        double yresult = _z * crossX - _x * crossZ;
        _z = _x * crossY - _y * crossX;
        _x = xresult;
        _y = yresult;
    }

    internal void Divide(double scalar)
    {
        _x /= scalar;
        _y /= scalar;
        _z /= scalar;
    }

    internal void Multiply(double scalar)
    {
        _x *= scalar;
        _y *= scalar;
        _z *= scalar;
    }

    internal void Multiply(in Xyz other)
    {
        _x *= other._x;
        _y *= other._y;
        _z *= other._z;
    }

    internal void Multiply(in Matrix3x3 matrix)
    {
        double origX = _x;
        double origY = _y;
        double origZ = _z;
        _x = matrix._m00 * origX + matrix._m01 * origY + matrix._m02 * origZ;
        _y = matrix._m10 * origX + matrix._m11 * origY + matrix._m12 * origZ;
        _z = matrix._m20 * origX + matrix._m21 * origY + matrix._m22 * origZ;
    }

    internal void Normalize()
    {
        double d = Modulus();
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(), "gp_XYZ::Normalize() - vector has zero norm");
        _x = _x / d;
        _y = _y / d;
        _z = _z / d;
    }

    internal void Reverse()
    {
        _x = -_x;
        _y = -_y;
        _z = -_z;
    }

    internal void Subtract(in Xyz other)
    {
        _x -= other._x;
        _y -= other._y;
        _z -= other._z;
    }

    internal void SetLinearForm(double a1, in Xyz xyz1, double a2, in Xyz xyz2, double a3, in Xyz xyz3, in Xyz xyz4)
    {
        _x = a1 * xyz1._x + a2 * xyz2._x + a3 * xyz3._x + xyz4._x;
        _y = a1 * xyz1._y + a2 * xyz2._y + a3 * xyz3._y + xyz4._y;
        _z = a1 * xyz1._z + a2 * xyz2._z + a3 * xyz3._z + xyz4._z;
    }

    internal void SetLinearForm(double a1, in Xyz xyz1, double a2, in Xyz xyz2, double a3, in Xyz xyz3)
    {
        _x = a1 * xyz1._x + a2 * xyz2._x + a3 * xyz3._x;
        _y = a1 * xyz1._y + a2 * xyz2._y + a3 * xyz3._y;
        _z = a1 * xyz1._z + a2 * xyz2._z + a3 * xyz3._z;
    }

    internal void SetLinearForm(double a1, in Xyz xyz1, double a2, in Xyz xyz2, in Xyz xyz3)
    {
        _x = a1 * xyz1._x + a2 * xyz2._x + xyz3._x;
        _y = a1 * xyz1._y + a2 * xyz2._y + xyz3._y;
        _z = a1 * xyz1._z + a2 * xyz2._z + xyz3._z;
    }

    internal void SetLinearForm(double a1, in Xyz xyz1, double a2, in Xyz xyz2)
    {
        _x = a1 * xyz1._x + a2 * xyz2._x;
        _y = a1 * xyz1._y + a2 * xyz2._y;
        _z = a1 * xyz1._z + a2 * xyz2._z;
    }

    internal void SetLinearForm(double a1, in Xyz xyz1, in Xyz xyz2)
    {
        _x = a1 * xyz1._x + xyz2._x;
        _y = a1 * xyz1._y + xyz2._y;
        _z = a1 * xyz1._z + xyz2._z;
    }

    internal void SetLinearForm(in Xyz xyz1, in Xyz xyz2)
    {
        _x = xyz1._x + xyz2._x;
        _y = xyz1._y + xyz2._y;
        _z = xyz1._z + xyz2._z;
    }

    // Copies with a mutator applied (the public way to "set")

    public readonly Xyz WithCoord(double x, double y, double z) => new(x, y, z);

    public readonly Xyz WithCoord(int index, double xi)
    {
        Xyz copy = this;
        copy.SetCoord(index, xi);
        return copy;
    }

    public readonly Xyz WithX(double x) => new(x, _y, _z);

    public readonly Xyz WithY(double y) => new(_x, y, _z);

    public readonly Xyz WithZ(double z) => new(_x, _y, z);

    // Queries

    public readonly double Coord(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3);
        if (index == 1)
        {
            return _x;
        }
        else if (index == 2)
        {
            return _y;
        }

        return _z;
    }

    public readonly void Coord(out double x, out double y, out double z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    public readonly double X() => _x;

    public readonly double Y() => _y;

    public readonly double Z() => _z;

    public readonly double Modulus() => Math.Sqrt(_x * _x + _y * _y + _z * _z);

    public readonly double SquareModulus() => _x * _x + _y * _y + _z * _z;

    public readonly bool IsEqual(in Xyz other, double tolerance)
        => Math.Abs(_x - other._x) < tolerance
           && Math.Abs(_y - other._y) < tolerance
           && Math.Abs(_z - other._z) < tolerance;

    public readonly Xyz Added(in Xyz other) => new(_x + other._x, _y + other._y, _z + other._z);

    public readonly Xyz Crossed(in Xyz other)
        => new(_y * other._z - _z * other._y, _z * other._x - _x * other._z, _x * other._y - _y * other._x);

    public readonly double CrossMagnitude(in Xyz right) => Math.Sqrt(CrossSquareMagnitude(right));

    public readonly double CrossSquareMagnitude(in Xyz right)
    {
        double xresult = _y * right._z - _z * right._y;
        double yresult = _z * right._x - _x * right._z;
        double zresult = _x * right._y - _y * right._x;
        return xresult * xresult + yresult * yresult + zresult * zresult;
    }

    public readonly Xyz CrossCrossed(in Xyz coord1, in Xyz coord2)
    {
        Xyz coord0 = this;
        coord0.CrossCross(coord1, coord2);
        return coord0;
    }

    public readonly Xyz Divided(double scalar) => new(_x / scalar, _y / scalar, _z / scalar);

    public readonly double Dot(in Xyz other) => _x * other._x + _y * other._y + _z * other._z;

    public readonly double DotCross(in Xyz coord1, in Xyz coord2)
    {
        double xresult = coord1._y * coord2._z - coord1._z * coord2._y;
        double yresult = coord1._z * coord2._x - coord1._x * coord2._z;
        double zresult = coord1._x * coord2._y - coord1._y * coord2._x;
        return _x * xresult + _y * yresult + _z * zresult;
    }

    public readonly Xyz Multiplied(double scalar) => new(_x * scalar, _y * scalar, _z * scalar);

    public readonly Xyz Multiplied(in Xyz other) => new(_x * other._x, _y * other._y, _z * other._z);

    public readonly Xyz Multiplied(in Matrix3x3 matrix)
        => new(matrix._m00 * _x + matrix._m01 * _y + matrix._m02 * _z,
               matrix._m10 * _x + matrix._m11 * _y + matrix._m12 * _z,
               matrix._m20 * _x + matrix._m21 * _y + matrix._m22 * _z);

    public readonly Xyz Normalized()
    {
        double d = Modulus();
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(), "gp_XYZ::Normalized() - vector has zero norm");
        return new Xyz(_x / d, _y / d, _z / d);
    }

    public readonly Xyz Reversed() => new(-_x, -_y, -_z);

    public readonly Xyz Subtracted(in Xyz other) => new(_x - other._x, _y - other._y, _z - other._z);

    /// <summary>Copies the coordinates into a span of at least three doubles.</summary>
    public readonly void CopyTo(Span<double> destination)
    {
        destination[0] = _x;
        destination[1] = _y;
        destination[2] = _z;
    }

    public static Xyz operator +(in Xyz left, in Xyz right) => left.Added(right);

    public static Xyz operator -(in Xyz left, in Xyz right) => left.Subtracted(right);

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Xyz left, in Xyz right) => left.Dot(right);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static Xyz operator ^(in Xyz left, in Xyz right) => left.Crossed(right);

    public static Xyz operator *(in Xyz coord, double scalar) => coord.Multiplied(scalar);

    public static Xyz operator *(double scalar, in Xyz coord) => coord.Multiplied(scalar);

    public static Xyz operator *(in Xyz coord, in Matrix3x3 matrix) => coord.Multiplied(matrix);

    public static Xyz operator *(in Matrix3x3 matrix, in Xyz coord) => coord.Multiplied(matrix);

    public static Xyz operator /(in Xyz coord, double scalar) => coord.Divided(scalar);

    // Exact equality, for use as a value (OCCT compares with a tolerance: IsEqual)

    public readonly bool Equals(Xyz other) => _x.Equals(other._x) && _y.Equals(other._y) && _z.Equals(other._z);

    public override readonly bool Equals(object? obj) => obj is Xyz other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_x, _y, _z);

    public static bool operator ==(in Xyz left, in Xyz right) => left.Equals(right);

    public static bool operator !=(in Xyz left, in Xyz right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("gp_XYZ", _x, _y, _z);
}
