// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Vec4.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;


/// <summary>A 4-component vector of numbers (float, double or int): an XYZW point or an RGBA colour.</summary>
/// <remarks>The default value is the zero vector, as OCCT's default constructor.</remarks>
public partial struct NCollection_Vec4<T> : INCollection_Vec<T, NCollection_Vec4<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer4<T> v;

    /// <summary>The number of components.</summary>
    public static int Length() => 4;

    /// <summary>All components set to <paramref name="theValue"/>.</summary>
    public NCollection_Vec4(T theValue)
    {
        v[0] = theValue;
        v[1] = theValue;
        v[2] = theValue;
        v[3] = theValue;
    }

    public NCollection_Vec4(T theX, T theY, T theZ, T theW)
    {
        v[0] = theX;
        v[1] = theY;
        v[2] = theZ;
        v[3] = theW;
    }

    /// <summary>From a 2-component vector, the last two components zero.</summary>
    public NCollection_Vec4(in NCollection_Vec2<T> theVec2)
    {
        v[0] = theVec2[0];
        v[1] = theVec2[1];
        v[2] = T.Zero;
        v[3] = T.Zero;
    }

    /// <summary>From a 3-component vector and a fourth value.</summary>
    public NCollection_Vec4(in NCollection_Vec3<T> theVec3, T theW = default)
    {
        v[0] = theVec3[0];
        v[1] = theVec3[1];
        v[2] = theVec3[2];
        v[3] = theW;
    }

    /// <summary>
    /// Converts a vector of another element type, as OCCT's converting constructor (<c>static_cast</c> of each
    /// component: truncation towards zero from a real to an integer type).
    /// </summary>
    public static NCollection_Vec4<T> From<TOther>(in NCollection_Vec4<TOther> theOther) where TOther : unmanaged, INumber<TOther>
        => new(T.CreateTruncating(theOther[0]), T.CreateTruncating(theOther[1]), T.CreateTruncating(theOther[2]), T.CreateTruncating(theOther[3]));

    static NCollection_Vec4<T> INCollection_Vec<T, NCollection_Vec4<T>>.Create(T theValue) => new(theValue);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(T theX, T theY, T theZ, T theW)
    {
        v[0] = theX;
        v[1] = theY;
        v[2] = theZ;
        v[3] = theW;
    }

    internal void SetValues(in NCollection_Vec3<T> theVec3, T theW)
    {
        v[0] = theVec3.x();
        v[1] = theVec3.y();
        v[2] = theVec3.z();
        v[3] = theW;
    }

    /// <summary>The components, for writing (OCCT's <c>ChangeData()</c> and non-const <c>x()</c>, <c>y()</c>, ...).</summary>
    [UnscopedRef]
    internal Span<T> ChangeData() => v;

    internal void Add(in NCollection_Vec4<T> theAdd)
    {
        v[0] += theAdd.v[0];
        v[1] += theAdd.v[1];
        v[2] += theAdd.v[2];
        v[3] += theAdd.v[3];
    }

    internal void Subtract(in NCollection_Vec4<T> theDec)
    {
        v[0] -= theDec.v[0];
        v[1] -= theDec.v[1];
        v[2] -= theDec.v[2];
        v[3] -= theDec.v[3];
    }

    /// <summary>Per-component multiplication (OCCT's <c>operator*=</c> with a vector).</summary>
    internal void Multiply(in NCollection_Vec4<T> theRight)
    {
        v[0] *= theRight.v[0];
        v[1] *= theRight.v[1];
        v[2] *= theRight.v[2];
        v[3] *= theRight.v[3];
    }

    internal void Multiply(T theFactor)
    {
        v[0] *= theFactor;
        v[1] *= theFactor;
        v[2] *= theFactor;
        v[3] *= theFactor;
    }

    /// <summary>Per-component division by a scale factor (OCCT's <c>operator/=</c>).</summary>
    internal void Divide(T theInvFactor)
    {
        // OCCT specialises float and double to multiply by the inverse
        if (NCollection_VecTools.IsReal<T>())
        {
            Multiply(T.One / theInvFactor);
            return;
        }

        v[0] /= theInvFactor;
        v[1] /= theInvFactor;
        v[2] /= theInvFactor;
        v[3] /= theInvFactor;
    }

    /// <summary>Per-component division (OCCT's <c>operator/=</c> with a vector).</summary>
    internal void Divide(in NCollection_Vec4<T> theRight)
    {
        v[0] /= theRight.v[0];
        v[1] /= theRight.v[1];
        v[2] /= theRight.v[2];
        v[3] /= theRight.v[3];
    }

    // Copies with a mutator applied

    public readonly NCollection_Vec4<T> WithValues(T theX, T theY, T theZ, T theW) => new(theX, theY, theZ, theW);

    /// <summary>A copy with the component of the given (0-based) index replaced.</summary>
    public readonly NCollection_Vec4<T> WithValue(int theIndex, T theValue)
    {
        NCollection_Vec4<T> aCopy = this;
        aCopy[theIndex] = theValue;
        return aCopy;
    }

    public readonly NCollection_Vec4<T> Added(in NCollection_Vec4<T> theAdd) => this + theAdd;

    public readonly NCollection_Vec4<T> Subtracted(in NCollection_Vec4<T> theDec) => this - theDec;

    public readonly NCollection_Vec4<T> Divided(T theInvFactor) => this / theInvFactor;

    public readonly NCollection_Vec4<T> Divided(in NCollection_Vec4<T> theRight) => this / theRight;

    public readonly NCollection_Vec4<T> Multiplied(in NCollection_Vec4<T> theRight) => this * theRight;

    // Queries

    /// <summary>The component of the given (0-based) index.</summary>
    public T this[int theIndex]
    {
        readonly get => v[theIndex];
        internal set => v[theIndex] = value;
    }

    public readonly T x() => v[0];

    public readonly T r() => v[0];

    public readonly T y() => v[1];

    public readonly T g() => v[1];

    public readonly T z() => v[2];

    public readonly T b() => v[2];

    public readonly T w() => v[3];

    public readonly T a() => v[3];

    public readonly NCollection_Vec2<T> xy() => new(x(), y());

    public readonly NCollection_Vec2<T> yx() => new(y(), x());

    public readonly NCollection_Vec2<T> xz() => new(x(), z());

    public readonly NCollection_Vec2<T> zx() => new(z(), x());

    public readonly NCollection_Vec2<T> xw() => new(x(), w());

    public readonly NCollection_Vec2<T> wx() => new(w(), x());

    public readonly NCollection_Vec2<T> yz() => new(y(), z());

    public readonly NCollection_Vec2<T> zy() => new(z(), y());

    public readonly NCollection_Vec2<T> yw() => new(y(), w());

    public readonly NCollection_Vec2<T> wy() => new(w(), y());

    public readonly NCollection_Vec2<T> zw() => new(z(), w());

    public readonly NCollection_Vec2<T> wz() => new(w(), z());

    public readonly NCollection_Vec3<T> xyz() => new(x(), y(), z());

    public readonly NCollection_Vec3<T> xzy() => new(x(), z(), y());

    public readonly NCollection_Vec3<T> yxz() => new(y(), x(), z());

    public readonly NCollection_Vec3<T> yzx() => new(y(), z(), x());

    public readonly NCollection_Vec3<T> zyx() => new(z(), y(), x());

    public readonly NCollection_Vec3<T> zxy() => new(z(), x(), y());

    public readonly NCollection_Vec3<T> xyw() => new(x(), y(), w());

    public readonly NCollection_Vec3<T> xwy() => new(x(), w(), y());

    public readonly NCollection_Vec3<T> yxw() => new(y(), x(), w());

    public readonly NCollection_Vec3<T> ywx() => new(y(), w(), x());

    public readonly NCollection_Vec3<T> wyx() => new(w(), y(), x());

    public readonly NCollection_Vec3<T> wxy() => new(w(), x(), y());

    public readonly NCollection_Vec3<T> xzw() => new(x(), z(), w());

    public readonly NCollection_Vec3<T> xwz() => new(x(), w(), z());

    public readonly NCollection_Vec3<T> zxw() => new(z(), x(), w());

    public readonly NCollection_Vec3<T> zwx() => new(z(), w(), x());

    public readonly NCollection_Vec3<T> wzx() => new(w(), z(), x());

    public readonly NCollection_Vec3<T> wxz() => new(w(), x(), z());

    public readonly NCollection_Vec3<T> yzw() => new(y(), z(), w());

    public readonly NCollection_Vec3<T> ywz() => new(y(), w(), z());

    public readonly NCollection_Vec3<T> zyw() => new(z(), y(), w());

    public readonly NCollection_Vec3<T> zwy() => new(z(), w(), y());

    public readonly NCollection_Vec3<T> wzy() => new(w(), z(), y());

    public readonly NCollection_Vec3<T> wyz() => new(w(), y(), z());

    public readonly NCollection_Vec3<T> rgb() => new(r(), g(), b());

    public readonly NCollection_Vec3<T> rbg() => new(r(), b(), g());

    public readonly NCollection_Vec3<T> grb() => new(g(), r(), b());

    public readonly NCollection_Vec3<T> gbr() => new(g(), b(), r());

    public readonly NCollection_Vec3<T> bgr() => new(b(), g(), r());

    public readonly NCollection_Vec3<T> brg() => new(b(), r(), g());

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each component).</summary>
    public readonly bool IsEqual(in NCollection_Vec4<T> theOther) => v[0] == theOther.v[0] && v[1] == theOther.v[1] && v[2] == theOther.v[2] && v[3] == theOther.v[3];

    public readonly NCollection_Vec4<T> Multiplied(T theFactor)
    {
        NCollection_Vec4<T> aCopyVec4 = this;
        aCopyVec4.Multiply(theFactor);
        return aCopyVec4;
    }

    /// <summary>The component-wise minimum of two vectors.</summary>
    public readonly NCollection_Vec4<T> cwiseMin(in NCollection_Vec4<T> theVec)
        => new(v[0] < theVec.v[0] ? v[0] : theVec.v[0], v[1] < theVec.v[1] ? v[1] : theVec.v[1], v[2] < theVec.v[2] ? v[2] : theVec.v[2], v[3] < theVec.v[3] ? v[3] : theVec.v[3]);

    /// <summary>The component-wise maximum of two vectors.</summary>
    public readonly NCollection_Vec4<T> cwiseMax(in NCollection_Vec4<T> theVec)
        => new(v[0] > theVec.v[0] ? v[0] : theVec.v[0], v[1] > theVec.v[1] ? v[1] : theVec.v[1], v[2] > theVec.v[2] ? v[2] : theVec.v[2], v[3] > theVec.v[3] ? v[3] : theVec.v[3]);

    /// <summary>The component-wise absolute value.</summary>
    public readonly NCollection_Vec4<T> cwiseAbs() => new(T.Abs(v[0]), T.Abs(v[1]), T.Abs(v[2]), T.Abs(v[3]));

    /// <summary>The largest component.</summary>
    public readonly T maxComp()
    {
        T aMax1 = v[0] > v[1] ? v[0] : v[1];
        T aMax2 = v[2] > v[3] ? v[2] : v[3];

        return aMax1 > aMax2 ? aMax1 : aMax2;
    }

    /// <summary>The smallest component.</summary>
    public readonly T minComp()
    {
        T aMin1 = v[0] < v[1] ? v[0] : v[1];
        T aMin2 = v[2] < v[3] ? v[2] : v[3];

        return aMin1 < aMin2 ? aMin1 : aMin2;
    }

    public readonly T Dot(in NCollection_Vec4<T> theOther) => x() * theOther.x() + y() * theOther.y() + z() * theOther.z() + w() * theOther.w();

    // OCCT's NCollection_Vec4 has no SquareModulus; it is here for INCollection_Vec
    readonly T INCollection_Vec<T, NCollection_Vec4<T>>.SquareModulus() => x() * x() + y() * y() + z() * z() + w() * w();

    /// <summary>Copies the components into a span of at least 4 elements.</summary>
    public readonly void CopyTo(Span<T> theDestination)
    {
        theDestination[0] = v[0];
        theDestination[1] = v[1];
        theDestination[2] = v[2];
        theDestination[3] = v[3];
    }

    public static NCollection_Vec4<T> operator +(in NCollection_Vec4<T> theLeft, in NCollection_Vec4<T> theRight)
    {
        NCollection_Vec4<T> aResult = theLeft;
        aResult.Add(theRight);
        return aResult;
    }

    public static NCollection_Vec4<T> operator -(in NCollection_Vec4<T> theLeft, in NCollection_Vec4<T> theRight)
    {
        NCollection_Vec4<T> aResult = theLeft;
        aResult.Subtract(theRight);
        return aResult;
    }

    public static NCollection_Vec4<T> operator *(in NCollection_Vec4<T> theLeft, in NCollection_Vec4<T> theRight)
    {
        NCollection_Vec4<T> aResult = theLeft;
        aResult.Multiply(theRight);
        return aResult;
    }

    public static NCollection_Vec4<T> operator /(in NCollection_Vec4<T> theLeft, in NCollection_Vec4<T> theRight)
    {
        NCollection_Vec4<T> aResult = theLeft;
        aResult.Divide(theRight);
        return aResult;
    }

    public static NCollection_Vec4<T> operator /(in NCollection_Vec4<T> theVec, T theInvFactor)
    {
        NCollection_Vec4<T> aResult = theVec;
        aResult.Divide(theInvFactor);
        return aResult;
    }

    public static NCollection_Vec4<T> operator -(in NCollection_Vec4<T> theVec) => new(-theVec.x(), -theVec.y(), -theVec.z(), -theVec.w());

    public static NCollection_Vec4<T> operator *(in NCollection_Vec4<T> theVec, T theFactor) => theVec.Multiplied(theFactor);

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Vec4<T> other) => v[0].Equals(other.v[0]) && v[1].Equals(other.v[1]) && v[2].Equals(other.v[2]) && v[3].Equals(other.v[3]);

    public override readonly bool Equals(object? obj) => obj is NCollection_Vec4<T> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(v[0], v[1], v[2], v[3]);

    public static bool operator ==(in NCollection_Vec4<T> theLeft, in NCollection_Vec4<T> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in NCollection_Vec4<T> theLeft, in NCollection_Vec4<T> theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => NCollection_VecTools.Format<T>("NCollection_Vec4", v);
}
