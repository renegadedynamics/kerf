// Port of: src/FoundationClasses/TKMath/gp/gp_XY.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using System.Diagnostics.CodeAnalysis;

namespace Kerf.Engine.Geometry;

/// <summary>A pair of reals: the coordinates of a 2D point, vector or direction, without their meaning.</summary>
internal struct Xy : IEquatable<Xy>
{
    private double _x;
    private double _y;

    public Xy(double x, double y)
    {
        _x = x;
        _y = y;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int index, double xi)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 2);
        if (index == 1)
        {
            _x = xi;
        }
        else
        {
            _y = xi;
        }
    }

    internal void SetCoord(double x, double y)
    {
        _x = x;
        _y = y;
    }

    internal void SetX(double x) => _x = x;

    internal void SetY(double y) => _y = y;

    [UnscopedRef]
    internal ref double ChangeCoord(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 2);
        if (index == 1)
        {
            return ref _x;
        }

        return ref _y;
    }

    internal void Add(in Xy other)
    {
        _x += other._x;
        _y += other._y;
    }

    internal void Divide(double scalar)
    {
        _x /= scalar;
        _y /= scalar;
    }

    internal void Multiply(double scalar)
    {
        _x *= scalar;
        _y *= scalar;
    }

    internal void Multiply(in Xy other)
    {
        _x *= other._x;
        _y *= other._y;
    }

    internal void Multiply(in Matrix2x2 matrix)
    {
        double xresult = matrix._m00 * _x + matrix._m01 * _y;
        _y = matrix._m10 * _x + matrix._m11 * _y;
        _x = xresult;
    }

    internal void Normalize()
    {
        double d = Modulus();
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(), "gp_XY::Normalize() - vector has zero norm");
        _x = _x / d;
        _y = _y / d;
    }

    internal void Reverse()
    {
        _x = -_x;
        _y = -_y;
    }

    internal void SetLinearForm(double a1, in Xy xy1, double a2, in Xy xy2)
    {
        _x = a1 * xy1._x + a2 * xy2._x;
        _y = a1 * xy1._y + a2 * xy2._y;
    }

    internal void SetLinearForm(double a1, in Xy xy1, double a2, in Xy xy2, in Xy xy3)
    {
        _x = a1 * xy1._x + a2 * xy2._x + xy3._x;
        _y = a1 * xy1._y + a2 * xy2._y + xy3._y;
    }

    internal void SetLinearForm(double a1, in Xy xy1, in Xy xy2)
    {
        _x = a1 * xy1._x + xy2._x;
        _y = a1 * xy1._y + xy2._y;
    }

    internal void SetLinearForm(in Xy xy1, in Xy xy2)
    {
        _x = xy1._x + xy2._x;
        _y = xy1._y + xy2._y;
    }

    internal void Subtract(in Xy other)
    {
        _x -= other._x;
        _y -= other._y;
    }

    // Copies with a mutator applied (the public way to "set")

    public readonly Xy WithCoord(int index, double xi)
    {
        Xy copy = this;
        copy.SetCoord(index, xi);
        return copy;
    }

    public readonly Xy WithCoord(double x, double y) => new(x, y);

    public readonly Xy WithX(double x) => new(x, _y);

    public readonly Xy WithY(double y) => new(_x, y);

    // Queries

    public readonly double Coord(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 2);
        if (index == 1)
        {
            return _x;
        }

        return _y;
    }

    public readonly void Coord(out double x, out double y)
    {
        x = _x;
        y = _y;
    }

    public readonly double X() => _x;

    public readonly double Y() => _y;

    public readonly double Modulus() => Math.Sqrt(SquareModulus());

    public readonly double SquareModulus() => _x * _x + _y * _y;

    public readonly bool IsEqual(in Xy other, double tolerance)
        => (Math.Abs(_x - other._x) < tolerance) && (Math.Abs(_y - other._y) < tolerance);

    public readonly Xy Added(in Xy other) => new(_x + other.X(), _y + other.Y());

    /// <summary>The scalar cross product <c>x * other.y - y * other.x</c>.</summary>
    public readonly double Crossed(in Xy other) => _x * other._y - _y * other._x;

    public readonly double CrossMagnitude(in Xy right) => Math.Abs(_x * right._y - _y * right._x);

    public readonly double CrossSquareMagnitude(in Xy right)
    {
        double zresult = _x * right._y - _y * right._x;
        return zresult * zresult;
    }

    public readonly Xy Divided(double scalar) => new(_x / scalar, _y / scalar);

    public readonly double Dot(in Xy other) => _x * other._x + _y * other._y;

    public readonly Xy Multiplied(double scalar) => new(_x * scalar, _y * scalar);

    public readonly Xy Multiplied(in Xy other) => new(_x * other.X(), _y * other.Y());

    public readonly Xy Multiplied(in Matrix2x2 matrix)
        => new(matrix._m00 * _x + matrix._m01 * _y,
               matrix._m10 * _x + matrix._m11 * _y);

    public readonly Xy Normalized()
    {
        double d = Modulus();
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(), "gp_XY::Normalized() - vector has zero norm");
        return new Xy(_x / d, _y / d);
    }

    public readonly Xy Reversed() => new(-_x, -_y);

    public readonly Xy Subtracted(in Xy other) => new(_x - other._x, _y - other._y);

    /// <summary>Copies the coordinates into a span of at least two doubles.</summary>
    public readonly void CopyTo(Span<double> destination)
    {
        destination[0] = _x;
        destination[1] = _y;
    }

    public static Xy operator +(in Xy left, in Xy right) => left.Added(right);

    public static Xy operator -(in Xy left, in Xy right) => left.Subtracted(right);

    public static Xy operator -(in Xy coord) => coord.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in Xy left, in Xy right) => left.Dot(right);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in Xy left, in Xy right) => left.Crossed(right);

    public static Xy operator *(in Xy coord, double scalar) => coord.Multiplied(scalar);

    public static Xy operator *(double scalar, in Xy coord) => coord.Multiplied(scalar);

    public static Xy operator *(in Xy coord, in Matrix2x2 matrix) => coord.Multiplied(matrix);

    public static Xy operator *(in Matrix2x2 matrix, in Xy coord) => coord.Multiplied(matrix);

    public static Xy operator /(in Xy coord, double scalar) => coord.Divided(scalar);

    // Exact equality, for use as a value (OCCT compares with a tolerance: IsEqual)

    public readonly bool Equals(Xy other) => _x.Equals(other._x) && _y.Equals(other._y);

    public override readonly bool Equals(object? obj) => obj is Xy other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_x, _y);

    public static bool operator ==(in Xy left, in Xy right) => left.Equals(right);

    public static bool operator !=(in Xy left, in Xy right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("Xy", _x, _y);
}
