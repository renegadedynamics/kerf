// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Vec3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Kerf.Engine;


/// <summary>A 3-component vector of numbers (float, double or int): an XYZ point or an RGB colour.</summary>
/// <remarks>The default value is the zero vector, as OCCT's default constructor.</remarks>
internal partial struct NCollection_Vec3<T> : INCollection_Vec<T, NCollection_Vec3<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer3<T> _v;

    /// <summary>The number of components.</summary>
    public static int Length() => 3;

    /// <summary>All components set to <paramref name="value"/>.</summary>
    public NCollection_Vec3(T value)
    {
        _v[0] = value;
        _v[1] = value;
        _v[2] = value;
    }

    public NCollection_Vec3(T x, T y, T z)
    {
        _v[0] = x;
        _v[1] = y;
        _v[2] = z;
    }

    /// <summary>From a 2-component vector and a third value.</summary>
    public NCollection_Vec3(in NCollection_Vec2<T> vec2, T z = default)
    {
        _v[0] = vec2[0];
        _v[1] = vec2[1];
        _v[2] = z;
    }

    /// <summary>
    /// Converts a vector of another element type, as OCCT's converting constructor (<c>static_cast</c> of each
    /// component: truncation towards zero from a real to an integer type).
    /// </summary>
    public static NCollection_Vec3<T> From<TOther>(in NCollection_Vec3<TOther> other) where TOther : unmanaged, INumber<TOther>
        => new(T.CreateTruncating(other[0]), T.CreateTruncating(other[1]), T.CreateTruncating(other[2]));

    static NCollection_Vec3<T> INCollection_Vec<T, NCollection_Vec3<T>>.Create(T value) => new(value);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(T x, T y, T z)
    {
        _v[0] = x;
        _v[1] = y;
        _v[2] = z;
    }

    internal void SetValues(in NCollection_Vec2<T> vec2, T z)
    {
        _v[0] = vec2.X();
        _v[1] = vec2.Y();
        _v[2] = z;
    }

    /// <summary>The components, for writing (OCCT's <c>ChangeData()</c> and non-const <c>x()</c>, <c>y()</c>, ...).</summary>
    [UnscopedRef]
    internal Span<T> ChangeData() => _v;

    internal void Add(in NCollection_Vec3<T> add)
    {
        _v[0] += add._v[0];
        _v[1] += add._v[1];
        _v[2] += add._v[2];
    }

    internal void Subtract(in NCollection_Vec3<T> dec)
    {
        _v[0] -= dec._v[0];
        _v[1] -= dec._v[1];
        _v[2] -= dec._v[2];
    }

    /// <summary>Per-component multiplication (OCCT's <c>operator*=</c> with a vector).</summary>
    internal void Multiply(in NCollection_Vec3<T> right)
    {
        _v[0] *= right._v[0];
        _v[1] *= right._v[1];
        _v[2] *= right._v[2];
    }

    internal void Multiply(T factor)
    {
        _v[0] *= factor;
        _v[1] *= factor;
        _v[2] *= factor;
    }

    /// <summary>Per-component division by a scale factor (OCCT's <c>operator/=</c>).</summary>
    internal void Divide(T invFactor)
    {
        // OCCT specialises float and double to multiply by the inverse
        if (NCollection_VecTools.IsReal<T>())
        {
            Multiply(T.One / invFactor);
            return;
        }

        _v[0] /= invFactor;
        _v[1] /= invFactor;
        _v[2] /= invFactor;
    }

    /// <summary>Per-component division (OCCT's <c>operator/=</c> with a vector).</summary>
    internal void Divide(in NCollection_Vec3<T> right)
    {
        _v[0] /= right._v[0];
        _v[1] /= right._v[1];
        _v[2] /= right._v[2];
    }

    internal void Normalize()
    {
        T modulus = Modulus();
        if (modulus != T.Zero) // just avoid divide by zero
        {
            _v[0] = X() / modulus;
            _v[1] = Y() / modulus;
            _v[2] = Z() / modulus;
        }
    }

    // Copies with a mutator applied

    public readonly NCollection_Vec3<T> WithValues(T x, T y, T z) => new(x, y, z);

    /// <summary>A copy with the component of the given (0-based) index replaced.</summary>
    public readonly NCollection_Vec3<T> WithValue(int index, T value)
    {
        NCollection_Vec3<T> copy = this;
        copy[index] = value;
        return copy;
    }

    public readonly NCollection_Vec3<T> Added(in NCollection_Vec3<T> add) => this + add;

    public readonly NCollection_Vec3<T> Subtracted(in NCollection_Vec3<T> dec) => this - dec;

    public readonly NCollection_Vec3<T> Divided(T invFactor) => this / invFactor;

    public readonly NCollection_Vec3<T> Divided(in NCollection_Vec3<T> right) => this / right;

    public readonly NCollection_Vec3<T> Multiplied(in NCollection_Vec3<T> right) => this * right;

    // Queries

    /// <summary>The component of the given (0-based) index.</summary>
    public T this[int index]
    {
        readonly get => _v[index];
        internal set => _v[index] = value;
    }

    public readonly T X() => _v[0];

    public readonly T R() => _v[0];

    public readonly T Y() => _v[1];

    public readonly T G() => _v[1];

    public readonly T Z() => _v[2];

    public readonly T B() => _v[2];

    public readonly NCollection_Vec2<T> Xy() => new(X(), Y());

    public readonly NCollection_Vec2<T> Yx() => new(Y(), X());

    public readonly NCollection_Vec2<T> Xz() => new(X(), Z());

    public readonly NCollection_Vec2<T> Zx() => new(Z(), X());

    public readonly NCollection_Vec2<T> Yz() => new(Y(), Z());

    public readonly NCollection_Vec2<T> Zy() => new(Z(), Y());

    public readonly NCollection_Vec3<T> Xyz() => new(X(), Y(), Z());

    public readonly NCollection_Vec3<T> Xzy() => new(X(), Z(), Y());

    public readonly NCollection_Vec3<T> Yxz() => new(Y(), X(), Z());

    public readonly NCollection_Vec3<T> Yzx() => new(Y(), Z(), X());

    public readonly NCollection_Vec3<T> Zyx() => new(Z(), Y(), X());

    public readonly NCollection_Vec3<T> Zxy() => new(Z(), X(), Y());

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each component).</summary>
    public readonly bool IsEqual(in NCollection_Vec3<T> other) => _v[0] == other._v[0] && _v[1] == other._v[1] && _v[2] == other._v[2];

    public readonly NCollection_Vec3<T> Multiplied(T factor)
    {
        NCollection_Vec3<T> copyVec3 = this;
        copyVec3.Multiply(factor);
        return copyVec3;
    }

    /// <summary>The component-wise minimum of two vectors.</summary>
    public readonly NCollection_Vec3<T> CwiseMin(in NCollection_Vec3<T> vec)
        => new(_v[0] < vec._v[0] ? _v[0] : vec._v[0], _v[1] < vec._v[1] ? _v[1] : vec._v[1], _v[2] < vec._v[2] ? _v[2] : vec._v[2]);

    /// <summary>The component-wise maximum of two vectors.</summary>
    public readonly NCollection_Vec3<T> CwiseMax(in NCollection_Vec3<T> vec)
        => new(_v[0] > vec._v[0] ? _v[0] : vec._v[0], _v[1] > vec._v[1] ? _v[1] : vec._v[1], _v[2] > vec._v[2] ? _v[2] : vec._v[2]);

    /// <summary>The component-wise absolute value.</summary>
    public readonly NCollection_Vec3<T> CwiseAbs() => new(T.Abs(_v[0]), T.Abs(_v[1]), T.Abs(_v[2]));

    /// <summary>The largest component.</summary>
    public readonly T MaxComp() => _v[0] > _v[1] ? (_v[0] > _v[2] ? _v[0] : _v[2]) : (_v[1] > _v[2] ? _v[1] : _v[2]);

    /// <summary>The smallest component.</summary>
    public readonly T MinComp() => _v[0] < _v[1] ? (_v[0] < _v[2] ? _v[0] : _v[2]) : (_v[1] < _v[2] ? _v[1] : _v[2]);

    public readonly T Dot(in NCollection_Vec3<T> other) => X() * other.X() + Y() * other.Y() + Z() * other.Z();

    /// <summary>The vector's length.</summary>
    public readonly T Modulus() => NCollection_VecTools.Sqrt(X() * X() + Y() * Y() + Z() * Z());

    public readonly T SquareModulus() => X() * X() + Y() * Y() + Z() * Z();

    public readonly NCollection_Vec3<T> Normalized()
    {
        NCollection_Vec3<T> copy = this;
        copy.Normalize();
        return copy;
    }

    /// <summary>The cross product.</summary>
    public static NCollection_Vec3<T> Cross(in NCollection_Vec3<T> vec1, in NCollection_Vec3<T> vec2)
        => new(vec1.Y() * vec2.Z() - vec1.Z() * vec2.Y(),
               vec1.Z() * vec2.X() - vec1.X() * vec2.Z(),
               vec1.X() * vec2.Y() - vec1.Y() * vec2.X());

    /// <summary>Linear interpolation between two vectors, <paramref name="t"/> in 0..1.</summary>
    public static NCollection_Vec3<T> GetLERP(in NCollection_Vec3<T> theFrom, in NCollection_Vec3<T> to, T t)
        => theFrom * (T.One - t) + to * t;

    public static NCollection_Vec3<T> DX() => new(T.One, T.Zero, T.Zero);

    public static NCollection_Vec3<T> DY() => new(T.Zero, T.One, T.Zero);

    public static NCollection_Vec3<T> DZ() => new(T.Zero, T.Zero, T.One);

    /// <summary>Copies the components into a span of at least 3 elements.</summary>
    public readonly void CopyTo(Span<T> destination)
    {
        destination[0] = _v[0];
        destination[1] = _v[1];
        destination[2] = _v[2];
    }

    public static NCollection_Vec3<T> operator +(in NCollection_Vec3<T> left, in NCollection_Vec3<T> right)
    {
        NCollection_Vec3<T> result = left;
        result.Add(right);
        return result;
    }

    public static NCollection_Vec3<T> operator -(in NCollection_Vec3<T> left, in NCollection_Vec3<T> right)
    {
        NCollection_Vec3<T> result = left;
        result.Subtract(right);
        return result;
    }

    public static NCollection_Vec3<T> operator *(in NCollection_Vec3<T> left, in NCollection_Vec3<T> right)
    {
        NCollection_Vec3<T> result = left;
        result.Multiply(right);
        return result;
    }

    public static NCollection_Vec3<T> operator /(in NCollection_Vec3<T> left, in NCollection_Vec3<T> right)
    {
        NCollection_Vec3<T> result = left;
        result.Divide(right);
        return result;
    }

    public static NCollection_Vec3<T> operator /(in NCollection_Vec3<T> vec, T invFactor)
    {
        NCollection_Vec3<T> result = vec;
        result.Divide(invFactor);
        return result;
    }

    public static NCollection_Vec3<T> operator -(in NCollection_Vec3<T> vec) => new(-vec.X(), -vec.Y(), -vec.Z());

    public static NCollection_Vec3<T> operator *(in NCollection_Vec3<T> vec, T factor) => vec.Multiplied(factor);

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Vec3<T> other) => _v[0].Equals(other._v[0]) && _v[1].Equals(other._v[1]) && _v[2].Equals(other._v[2]);

    public override readonly bool Equals(object? obj) => obj is NCollection_Vec3<T> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_v[0], _v[1], _v[2]);

    public static bool operator ==(in NCollection_Vec3<T> left, in NCollection_Vec3<T> right) => left.Equals(right);

    public static bool operator !=(in NCollection_Vec3<T> left, in NCollection_Vec3<T> right) => !left.Equals(right);

    public override readonly string ToString() => NCollection_VecTools.Format<T>("NCollection_Vec3", _v);
}
