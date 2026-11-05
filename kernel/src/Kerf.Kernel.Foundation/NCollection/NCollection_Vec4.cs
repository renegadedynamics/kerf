// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Vec4.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;


/// <summary>A 4-component vector of numbers (float, double or int): an XYZW point or an RGBA colour.</summary>
/// <remarks>The default value is the zero vector, as OCCT's default constructor.</remarks>
internal partial struct NCollection_Vec4<T> : INCollection_Vec<T, NCollection_Vec4<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer4<T> _v;

    /// <summary>The number of components.</summary>
    public static int Length() => 4;

    /// <summary>All components set to <paramref name="value"/>.</summary>
    public NCollection_Vec4(T value)
    {
        _v[0] = value;
        _v[1] = value;
        _v[2] = value;
        _v[3] = value;
    }

    public NCollection_Vec4(T x, T y, T z, T w)
    {
        _v[0] = x;
        _v[1] = y;
        _v[2] = z;
        _v[3] = w;
    }

    /// <summary>From a 2-component vector, the last two components zero.</summary>
    public NCollection_Vec4(in NCollection_Vec2<T> vec2)
    {
        _v[0] = vec2[0];
        _v[1] = vec2[1];
        _v[2] = T.Zero;
        _v[3] = T.Zero;
    }

    /// <summary>From a 3-component vector and a fourth value.</summary>
    public NCollection_Vec4(in NCollection_Vec3<T> vec3, T w = default)
    {
        _v[0] = vec3[0];
        _v[1] = vec3[1];
        _v[2] = vec3[2];
        _v[3] = w;
    }

    /// <summary>
    /// Converts a vector of another element type, as OCCT's converting constructor (<c>static_cast</c> of each
    /// component: truncation towards zero from a real to an integer type).
    /// </summary>
    public static NCollection_Vec4<T> From<TOther>(in NCollection_Vec4<TOther> other) where TOther : unmanaged, INumber<TOther>
        => new(T.CreateTruncating(other[0]), T.CreateTruncating(other[1]), T.CreateTruncating(other[2]), T.CreateTruncating(other[3]));

    static NCollection_Vec4<T> INCollection_Vec<T, NCollection_Vec4<T>>.Create(T value) => new(value);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(T x, T y, T z, T w)
    {
        _v[0] = x;
        _v[1] = y;
        _v[2] = z;
        _v[3] = w;
    }

    internal void SetValues(in NCollection_Vec3<T> vec3, T w)
    {
        _v[0] = vec3.X();
        _v[1] = vec3.Y();
        _v[2] = vec3.Z();
        _v[3] = w;
    }

    /// <summary>The components, for writing (OCCT's <c>ChangeData()</c> and non-const <c>x()</c>, <c>y()</c>, ...).</summary>
    [UnscopedRef]
    internal Span<T> ChangeData() => _v;

    internal void Add(in NCollection_Vec4<T> add)
    {
        _v[0] += add._v[0];
        _v[1] += add._v[1];
        _v[2] += add._v[2];
        _v[3] += add._v[3];
    }

    internal void Subtract(in NCollection_Vec4<T> dec)
    {
        _v[0] -= dec._v[0];
        _v[1] -= dec._v[1];
        _v[2] -= dec._v[2];
        _v[3] -= dec._v[3];
    }

    /// <summary>Per-component multiplication (OCCT's <c>operator*=</c> with a vector).</summary>
    internal void Multiply(in NCollection_Vec4<T> right)
    {
        _v[0] *= right._v[0];
        _v[1] *= right._v[1];
        _v[2] *= right._v[2];
        _v[3] *= right._v[3];
    }

    internal void Multiply(T factor)
    {
        _v[0] *= factor;
        _v[1] *= factor;
        _v[2] *= factor;
        _v[3] *= factor;
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
        _v[3] /= invFactor;
    }

    /// <summary>Per-component division (OCCT's <c>operator/=</c> with a vector).</summary>
    internal void Divide(in NCollection_Vec4<T> right)
    {
        _v[0] /= right._v[0];
        _v[1] /= right._v[1];
        _v[2] /= right._v[2];
        _v[3] /= right._v[3];
    }

    // Copies with a mutator applied

    public readonly NCollection_Vec4<T> WithValues(T x, T y, T z, T w) => new(x, y, z, w);

    /// <summary>A copy with the component of the given (0-based) index replaced.</summary>
    public readonly NCollection_Vec4<T> WithValue(int index, T value)
    {
        NCollection_Vec4<T> copy = this;
        copy[index] = value;
        return copy;
    }

    public readonly NCollection_Vec4<T> Added(in NCollection_Vec4<T> add) => this + add;

    public readonly NCollection_Vec4<T> Subtracted(in NCollection_Vec4<T> dec) => this - dec;

    public readonly NCollection_Vec4<T> Divided(T invFactor) => this / invFactor;

    public readonly NCollection_Vec4<T> Divided(in NCollection_Vec4<T> right) => this / right;

    public readonly NCollection_Vec4<T> Multiplied(in NCollection_Vec4<T> right) => this * right;

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

    public readonly T W() => _v[3];

    public readonly T A() => _v[3];

    public readonly NCollection_Vec2<T> Xy() => new(X(), Y());

    public readonly NCollection_Vec2<T> Yx() => new(Y(), X());

    public readonly NCollection_Vec2<T> Xz() => new(X(), Z());

    public readonly NCollection_Vec2<T> Zx() => new(Z(), X());

    public readonly NCollection_Vec2<T> Xw() => new(X(), W());

    public readonly NCollection_Vec2<T> Wx() => new(W(), X());

    public readonly NCollection_Vec2<T> Yz() => new(Y(), Z());

    public readonly NCollection_Vec2<T> Zy() => new(Z(), Y());

    public readonly NCollection_Vec2<T> Yw() => new(Y(), W());

    public readonly NCollection_Vec2<T> Wy() => new(W(), Y());

    public readonly NCollection_Vec2<T> Zw() => new(Z(), W());

    public readonly NCollection_Vec2<T> Wz() => new(W(), Z());

    public readonly NCollection_Vec3<T> Xyz() => new(X(), Y(), Z());

    public readonly NCollection_Vec3<T> Xzy() => new(X(), Z(), Y());

    public readonly NCollection_Vec3<T> Yxz() => new(Y(), X(), Z());

    public readonly NCollection_Vec3<T> Yzx() => new(Y(), Z(), X());

    public readonly NCollection_Vec3<T> Zyx() => new(Z(), Y(), X());

    public readonly NCollection_Vec3<T> Zxy() => new(Z(), X(), Y());

    public readonly NCollection_Vec3<T> Xyw() => new(X(), Y(), W());

    public readonly NCollection_Vec3<T> Xwy() => new(X(), W(), Y());

    public readonly NCollection_Vec3<T> Yxw() => new(Y(), X(), W());

    public readonly NCollection_Vec3<T> Ywx() => new(Y(), W(), X());

    public readonly NCollection_Vec3<T> Wyx() => new(W(), Y(), X());

    public readonly NCollection_Vec3<T> Wxy() => new(W(), X(), Y());

    public readonly NCollection_Vec3<T> Xzw() => new(X(), Z(), W());

    public readonly NCollection_Vec3<T> Xwz() => new(X(), W(), Z());

    public readonly NCollection_Vec3<T> Zxw() => new(Z(), X(), W());

    public readonly NCollection_Vec3<T> Zwx() => new(Z(), W(), X());

    public readonly NCollection_Vec3<T> Wzx() => new(W(), Z(), X());

    public readonly NCollection_Vec3<T> Wxz() => new(W(), X(), Z());

    public readonly NCollection_Vec3<T> Yzw() => new(Y(), Z(), W());

    public readonly NCollection_Vec3<T> Ywz() => new(Y(), W(), Z());

    public readonly NCollection_Vec3<T> Zyw() => new(Z(), Y(), W());

    public readonly NCollection_Vec3<T> Zwy() => new(Z(), W(), Y());

    public readonly NCollection_Vec3<T> Wzy() => new(W(), Z(), Y());

    public readonly NCollection_Vec3<T> Wyz() => new(W(), Y(), Z());

    public readonly NCollection_Vec3<T> Rgb() => new(R(), G(), B());

    public readonly NCollection_Vec3<T> Rbg() => new(R(), B(), G());

    public readonly NCollection_Vec3<T> Grb() => new(G(), R(), B());

    public readonly NCollection_Vec3<T> Gbr() => new(G(), B(), R());

    public readonly NCollection_Vec3<T> Bgr() => new(B(), G(), R());

    public readonly NCollection_Vec3<T> Brg() => new(B(), R(), G());

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each component).</summary>
    public readonly bool IsEqual(in NCollection_Vec4<T> other) => _v[0] == other._v[0] && _v[1] == other._v[1] && _v[2] == other._v[2] && _v[3] == other._v[3];

    public readonly NCollection_Vec4<T> Multiplied(T factor)
    {
        NCollection_Vec4<T> copyVec4 = this;
        copyVec4.Multiply(factor);
        return copyVec4;
    }

    /// <summary>The component-wise minimum of two vectors.</summary>
    public readonly NCollection_Vec4<T> CwiseMin(in NCollection_Vec4<T> vec)
        => new(_v[0] < vec._v[0] ? _v[0] : vec._v[0], _v[1] < vec._v[1] ? _v[1] : vec._v[1], _v[2] < vec._v[2] ? _v[2] : vec._v[2], _v[3] < vec._v[3] ? _v[3] : vec._v[3]);

    /// <summary>The component-wise maximum of two vectors.</summary>
    public readonly NCollection_Vec4<T> CwiseMax(in NCollection_Vec4<T> vec)
        => new(_v[0] > vec._v[0] ? _v[0] : vec._v[0], _v[1] > vec._v[1] ? _v[1] : vec._v[1], _v[2] > vec._v[2] ? _v[2] : vec._v[2], _v[3] > vec._v[3] ? _v[3] : vec._v[3]);

    /// <summary>The component-wise absolute value.</summary>
    public readonly NCollection_Vec4<T> CwiseAbs() => new(T.Abs(_v[0]), T.Abs(_v[1]), T.Abs(_v[2]), T.Abs(_v[3]));

    /// <summary>The largest component.</summary>
    public readonly T MaxComp()
    {
        T max1 = _v[0] > _v[1] ? _v[0] : _v[1];
        T max2 = _v[2] > _v[3] ? _v[2] : _v[3];

        return max1 > max2 ? max1 : max2;
    }

    /// <summary>The smallest component.</summary>
    public readonly T MinComp()
    {
        T min1 = _v[0] < _v[1] ? _v[0] : _v[1];
        T min2 = _v[2] < _v[3] ? _v[2] : _v[3];

        return min1 < min2 ? min1 : min2;
    }

    public readonly T Dot(in NCollection_Vec4<T> other) => X() * other.X() + Y() * other.Y() + Z() * other.Z() + W() * other.W();

    // OCCT's NCollection_Vec4 has no SquareModulus; it is here for INCollection_Vec
    readonly T INCollection_Vec<T, NCollection_Vec4<T>>.SquareModulus() => X() * X() + Y() * Y() + Z() * Z() + W() * W();

    /// <summary>Copies the components into a span of at least 4 elements.</summary>
    public readonly void CopyTo(Span<T> destination)
    {
        destination[0] = _v[0];
        destination[1] = _v[1];
        destination[2] = _v[2];
        destination[3] = _v[3];
    }

    public static NCollection_Vec4<T> operator +(in NCollection_Vec4<T> left, in NCollection_Vec4<T> right)
    {
        NCollection_Vec4<T> result = left;
        result.Add(right);
        return result;
    }

    public static NCollection_Vec4<T> operator -(in NCollection_Vec4<T> left, in NCollection_Vec4<T> right)
    {
        NCollection_Vec4<T> result = left;
        result.Subtract(right);
        return result;
    }

    public static NCollection_Vec4<T> operator *(in NCollection_Vec4<T> left, in NCollection_Vec4<T> right)
    {
        NCollection_Vec4<T> result = left;
        result.Multiply(right);
        return result;
    }

    public static NCollection_Vec4<T> operator /(in NCollection_Vec4<T> left, in NCollection_Vec4<T> right)
    {
        NCollection_Vec4<T> result = left;
        result.Divide(right);
        return result;
    }

    public static NCollection_Vec4<T> operator /(in NCollection_Vec4<T> vec, T invFactor)
    {
        NCollection_Vec4<T> result = vec;
        result.Divide(invFactor);
        return result;
    }

    public static NCollection_Vec4<T> operator -(in NCollection_Vec4<T> vec) => new(-vec.X(), -vec.Y(), -vec.Z(), -vec.W());

    public static NCollection_Vec4<T> operator *(in NCollection_Vec4<T> vec, T factor) => vec.Multiplied(factor);

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Vec4<T> other) => _v[0].Equals(other._v[0]) && _v[1].Equals(other._v[1]) && _v[2].Equals(other._v[2]) && _v[3].Equals(other._v[3]);

    public override readonly bool Equals(object? obj) => obj is NCollection_Vec4<T> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_v[0], _v[1], _v[2], _v[3]);

    public static bool operator ==(in NCollection_Vec4<T> left, in NCollection_Vec4<T> right) => left.Equals(right);

    public static bool operator !=(in NCollection_Vec4<T> left, in NCollection_Vec4<T> right) => !left.Equals(right);

    public override readonly string ToString() => NCollection_VecTools.Format<T>("NCollection_Vec4", _v);
}
