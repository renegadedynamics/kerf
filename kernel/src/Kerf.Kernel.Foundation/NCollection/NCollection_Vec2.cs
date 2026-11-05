// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Vec2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Kerf.Engine;


/// <summary>A 2D vector of numbers (float, double or int), for low-level arrays and BVH.</summary>
/// <remarks>The default value is the zero vector, as OCCT's default constructor.</remarks>
internal partial struct NCollection_Vec2<T> : INCollection_Vec<T, NCollection_Vec2<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer2<T> _v;

    /// <summary>The number of components.</summary>
    public static int Length() => 2;

    /// <summary>All components set to <paramref name="value"/>.</summary>
    public NCollection_Vec2(T value)
    {
        _v[0] = value;
        _v[1] = value;
    }

    public NCollection_Vec2(T x, T y)
    {
        _v[0] = x;
        _v[1] = y;
    }

    /// <summary>
    /// Converts a vector of another element type, as OCCT's converting constructor (<c>static_cast</c> of each
    /// component: truncation towards zero from a real to an integer type).
    /// </summary>
    public static NCollection_Vec2<T> From<TOther>(in NCollection_Vec2<TOther> other) where TOther : unmanaged, INumber<TOther>
        => new(T.CreateTruncating(other[0]), T.CreateTruncating(other[1]));

    static NCollection_Vec2<T> INCollection_Vec<T, NCollection_Vec2<T>>.Create(T value) => new(value);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(T x, T y)
    {
        _v[0] = x;
        _v[1] = y;
    }

    /// <summary>The components, for writing (OCCT's <c>ChangeData()</c> and non-const <c>x()</c>, <c>y()</c>, ...).</summary>
    [UnscopedRef]
    internal Span<T> ChangeData() => _v;

    internal void Add(in NCollection_Vec2<T> add)
    {
        _v[0] += add._v[0];
        _v[1] += add._v[1];
    }

    internal void Subtract(in NCollection_Vec2<T> dec)
    {
        _v[0] -= dec._v[0];
        _v[1] -= dec._v[1];
    }

    /// <summary>Per-component multiplication (OCCT's <c>operator*=</c> with a vector).</summary>
    internal void Multiply(in NCollection_Vec2<T> right)
    {
        _v[0] *= right._v[0];
        _v[1] *= right._v[1];
    }

    internal void Multiply(T factor)
    {
        _v[0] *= factor;
        _v[1] *= factor;
    }

    /// <summary>Per-component division by a scale factor (OCCT's <c>operator/=</c>).</summary>
    internal void Divide(T invFactor)
    {
        _v[0] /= invFactor;
        _v[1] /= invFactor;
    }

    /// <summary>Per-component division (OCCT's <c>operator/=</c> with a vector).</summary>
    internal void Divide(in NCollection_Vec2<T> right)
    {
        _v[0] /= right._v[0];
        _v[1] /= right._v[1];
    }

    // Copies with a mutator applied

    public readonly NCollection_Vec2<T> WithValues(T x, T y) => new(x, y);

    /// <summary>A copy with the component of the given (0-based) index replaced.</summary>
    public readonly NCollection_Vec2<T> WithValue(int index, T value)
    {
        NCollection_Vec2<T> copy = this;
        copy[index] = value;
        return copy;
    }

    public readonly NCollection_Vec2<T> Added(in NCollection_Vec2<T> add) => this + add;

    public readonly NCollection_Vec2<T> Subtracted(in NCollection_Vec2<T> dec) => this - dec;

    public readonly NCollection_Vec2<T> Divided(T invFactor) => this / invFactor;

    public readonly NCollection_Vec2<T> Divided(in NCollection_Vec2<T> right) => this / right;

    public readonly NCollection_Vec2<T> Multiplied(in NCollection_Vec2<T> right) => this * right;

    // Queries

    /// <summary>The component of the given (0-based) index.</summary>
    public T this[int index]
    {
        readonly get => _v[index];
        internal set => _v[index] = value;
    }

    public readonly T X() => _v[0];

    public readonly T Y() => _v[1];

    public readonly NCollection_Vec2<T> Xy() => new(X(), Y());

    public readonly NCollection_Vec2<T> Yx() => new(Y(), X());

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each component).</summary>
    public readonly bool IsEqual(in NCollection_Vec2<T> other) => _v[0] == other._v[0] && _v[1] == other._v[1];

    public readonly NCollection_Vec2<T> Multiplied(T factor)
    {
        return new(_v[0] * factor, _v[1] * factor);
    }

    /// <summary>The component-wise minimum of two vectors.</summary>
    public readonly NCollection_Vec2<T> CwiseMin(in NCollection_Vec2<T> vec)
        => new(_v[0] < vec._v[0] ? _v[0] : vec._v[0], _v[1] < vec._v[1] ? _v[1] : vec._v[1]);

    /// <summary>The component-wise maximum of two vectors.</summary>
    public readonly NCollection_Vec2<T> CwiseMax(in NCollection_Vec2<T> vec)
        => new(_v[0] > vec._v[0] ? _v[0] : vec._v[0], _v[1] > vec._v[1] ? _v[1] : vec._v[1]);

    /// <summary>The component-wise absolute value.</summary>
    public readonly NCollection_Vec2<T> CwiseAbs() => new(T.Abs(_v[0]), T.Abs(_v[1]));

    /// <summary>The largest component.</summary>
    public readonly T MaxComp() => _v[0] > _v[1] ? _v[0] : _v[1];

    /// <summary>The smallest component.</summary>
    public readonly T MinComp() => _v[0] < _v[1] ? _v[0] : _v[1];

    public readonly T Dot(in NCollection_Vec2<T> other) => X() * other.X() + Y() * other.Y();

    /// <summary>The vector's length.</summary>
    public readonly T Modulus() => NCollection_VecTools.Sqrt(X() * X() + Y() * Y());

    public readonly T SquareModulus() => X() * X() + Y() * Y();

    public static NCollection_Vec2<T> DX() => new(T.One, T.Zero);

    public static NCollection_Vec2<T> DY() => new(T.Zero, T.One);

    /// <summary>Copies the components into a span of at least 2 elements.</summary>
    public readonly void CopyTo(Span<T> destination)
    {
        destination[0] = _v[0];
        destination[1] = _v[1];
    }

    public static NCollection_Vec2<T> operator +(in NCollection_Vec2<T> left, in NCollection_Vec2<T> right) => new(left._v[0] + right._v[0], left._v[1] + right._v[1]);

    public static NCollection_Vec2<T> operator -(in NCollection_Vec2<T> left, in NCollection_Vec2<T> right) => new(left._v[0] - right._v[0], left._v[1] - right._v[1]);

    public static NCollection_Vec2<T> operator *(in NCollection_Vec2<T> left, in NCollection_Vec2<T> right) => new(left._v[0] * right._v[0], left._v[1] * right._v[1]);

    public static NCollection_Vec2<T> operator /(in NCollection_Vec2<T> left, in NCollection_Vec2<T> right) => new(left._v[0] / right._v[0], left._v[1] / right._v[1]);

    public static NCollection_Vec2<T> operator /(in NCollection_Vec2<T> vec, T invFactor) => new(vec._v[0] / invFactor, vec._v[1] / invFactor);

    public static NCollection_Vec2<T> operator -(in NCollection_Vec2<T> vec) => new(-vec.X(), -vec.Y());

    public static NCollection_Vec2<T> operator *(in NCollection_Vec2<T> vec, T factor) => vec.Multiplied(factor);

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Vec2<T> other) => _v[0].Equals(other._v[0]) && _v[1].Equals(other._v[1]);

    public override readonly bool Equals(object? obj) => obj is NCollection_Vec2<T> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_v[0], _v[1]);

    public static bool operator ==(in NCollection_Vec2<T> left, in NCollection_Vec2<T> right) => left.Equals(right);

    public static bool operator !=(in NCollection_Vec2<T> left, in NCollection_Vec2<T> right) => !left.Equals(right);

    public override readonly string ToString() => NCollection_VecTools.Format<T>("NCollection_Vec2", _v);
}
