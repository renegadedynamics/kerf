// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Vec3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;


/// <summary>A 3-component vector of numbers (float, double or int): an XYZ point or an RGB colour.</summary>
/// <remarks>The default value is the zero vector, as OCCT's default constructor.</remarks>
public partial struct NCollection_Vec3<T> : INCollection_Vec<T, NCollection_Vec3<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer3<T> v;

    /// <summary>The number of components.</summary>
    public static int Length() => 3;

    /// <summary>All components set to <paramref name="theValue"/>.</summary>
    public NCollection_Vec3(T theValue)
    {
        v[0] = theValue;
        v[1] = theValue;
        v[2] = theValue;
    }

    public NCollection_Vec3(T theX, T theY, T theZ)
    {
        v[0] = theX;
        v[1] = theY;
        v[2] = theZ;
    }

    /// <summary>From a 2-component vector and a third value.</summary>
    public NCollection_Vec3(in NCollection_Vec2<T> theVec2, T theZ = default)
    {
        v[0] = theVec2[0];
        v[1] = theVec2[1];
        v[2] = theZ;
    }

    /// <summary>
    /// Converts a vector of another element type, as OCCT's converting constructor (<c>static_cast</c> of each
    /// component: truncation towards zero from a real to an integer type).
    /// </summary>
    public static NCollection_Vec3<T> From<TOther>(in NCollection_Vec3<TOther> theOther) where TOther : unmanaged, INumber<TOther>
        => new(T.CreateTruncating(theOther[0]), T.CreateTruncating(theOther[1]), T.CreateTruncating(theOther[2]));

    static NCollection_Vec3<T> INCollection_Vec<T, NCollection_Vec3<T>>.Create(T theValue) => new(theValue);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(T theX, T theY, T theZ)
    {
        v[0] = theX;
        v[1] = theY;
        v[2] = theZ;
    }

    internal void SetValues(in NCollection_Vec2<T> theVec2, T theZ)
    {
        v[0] = theVec2.x();
        v[1] = theVec2.y();
        v[2] = theZ;
    }

    /// <summary>The components, for writing (OCCT's <c>ChangeData()</c> and non-const <c>x()</c>, <c>y()</c>, ...).</summary>
    [UnscopedRef]
    internal Span<T> ChangeData() => v;

    internal void Add(in NCollection_Vec3<T> theAdd)
    {
        v[0] += theAdd.v[0];
        v[1] += theAdd.v[1];
        v[2] += theAdd.v[2];
    }

    internal void Subtract(in NCollection_Vec3<T> theDec)
    {
        v[0] -= theDec.v[0];
        v[1] -= theDec.v[1];
        v[2] -= theDec.v[2];
    }

    /// <summary>Per-component multiplication (OCCT's <c>operator*=</c> with a vector).</summary>
    internal void Multiply(in NCollection_Vec3<T> theRight)
    {
        v[0] *= theRight.v[0];
        v[1] *= theRight.v[1];
        v[2] *= theRight.v[2];
    }

    internal void Multiply(T theFactor)
    {
        v[0] *= theFactor;
        v[1] *= theFactor;
        v[2] *= theFactor;
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
    }

    /// <summary>Per-component division (OCCT's <c>operator/=</c> with a vector).</summary>
    internal void Divide(in NCollection_Vec3<T> theRight)
    {
        v[0] /= theRight.v[0];
        v[1] /= theRight.v[1];
        v[2] /= theRight.v[2];
    }

    internal void Normalize()
    {
        T aModulus = Modulus();
        if (aModulus != T.Zero) // just avoid divide by zero
        {
            v[0] = x() / aModulus;
            v[1] = y() / aModulus;
            v[2] = z() / aModulus;
        }
    }

    // Copies with a mutator applied

    public readonly NCollection_Vec3<T> WithValues(T theX, T theY, T theZ) => new(theX, theY, theZ);

    /// <summary>A copy with the component of the given (0-based) index replaced.</summary>
    public readonly NCollection_Vec3<T> WithValue(int theIndex, T theValue)
    {
        NCollection_Vec3<T> aCopy = this;
        aCopy[theIndex] = theValue;
        return aCopy;
    }

    public readonly NCollection_Vec3<T> Added(in NCollection_Vec3<T> theAdd) => this + theAdd;

    public readonly NCollection_Vec3<T> Subtracted(in NCollection_Vec3<T> theDec) => this - theDec;

    public readonly NCollection_Vec3<T> Divided(T theInvFactor) => this / theInvFactor;

    public readonly NCollection_Vec3<T> Divided(in NCollection_Vec3<T> theRight) => this / theRight;

    public readonly NCollection_Vec3<T> Multiplied(in NCollection_Vec3<T> theRight) => this * theRight;

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

    public readonly NCollection_Vec2<T> xy() => new(x(), y());

    public readonly NCollection_Vec2<T> yx() => new(y(), x());

    public readonly NCollection_Vec2<T> xz() => new(x(), z());

    public readonly NCollection_Vec2<T> zx() => new(z(), x());

    public readonly NCollection_Vec2<T> yz() => new(y(), z());

    public readonly NCollection_Vec2<T> zy() => new(z(), y());

    public readonly NCollection_Vec3<T> xyz() => new(x(), y(), z());

    public readonly NCollection_Vec3<T> xzy() => new(x(), z(), y());

    public readonly NCollection_Vec3<T> yxz() => new(y(), x(), z());

    public readonly NCollection_Vec3<T> yzx() => new(y(), z(), x());

    public readonly NCollection_Vec3<T> zyx() => new(z(), y(), x());

    public readonly NCollection_Vec3<T> zxy() => new(z(), x(), y());

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each component).</summary>
    public readonly bool IsEqual(in NCollection_Vec3<T> theOther) => v[0] == theOther.v[0] && v[1] == theOther.v[1] && v[2] == theOther.v[2];

    public readonly NCollection_Vec3<T> Multiplied(T theFactor)
    {
        NCollection_Vec3<T> aCopyVec3 = this;
        aCopyVec3.Multiply(theFactor);
        return aCopyVec3;
    }

    /// <summary>The component-wise minimum of two vectors.</summary>
    public readonly NCollection_Vec3<T> cwiseMin(in NCollection_Vec3<T> theVec)
        => new(v[0] < theVec.v[0] ? v[0] : theVec.v[0], v[1] < theVec.v[1] ? v[1] : theVec.v[1], v[2] < theVec.v[2] ? v[2] : theVec.v[2]);

    /// <summary>The component-wise maximum of two vectors.</summary>
    public readonly NCollection_Vec3<T> cwiseMax(in NCollection_Vec3<T> theVec)
        => new(v[0] > theVec.v[0] ? v[0] : theVec.v[0], v[1] > theVec.v[1] ? v[1] : theVec.v[1], v[2] > theVec.v[2] ? v[2] : theVec.v[2]);

    /// <summary>The component-wise absolute value.</summary>
    public readonly NCollection_Vec3<T> cwiseAbs() => new(T.Abs(v[0]), T.Abs(v[1]), T.Abs(v[2]));

    /// <summary>The largest component.</summary>
    public readonly T maxComp() => v[0] > v[1] ? (v[0] > v[2] ? v[0] : v[2]) : (v[1] > v[2] ? v[1] : v[2]);

    /// <summary>The smallest component.</summary>
    public readonly T minComp() => v[0] < v[1] ? (v[0] < v[2] ? v[0] : v[2]) : (v[1] < v[2] ? v[1] : v[2]);

    public readonly T Dot(in NCollection_Vec3<T> theOther) => x() * theOther.x() + y() * theOther.y() + z() * theOther.z();

    /// <summary>The vector's length.</summary>
    public readonly T Modulus() => NCollection_VecTools.Sqrt(x() * x() + y() * y() + z() * z());

    public readonly T SquareModulus() => x() * x() + y() * y() + z() * z();

    public readonly NCollection_Vec3<T> Normalized()
    {
        NCollection_Vec3<T> aCopy = this;
        aCopy.Normalize();
        return aCopy;
    }

    /// <summary>The cross product.</summary>
    public static NCollection_Vec3<T> Cross(in NCollection_Vec3<T> theVec1, in NCollection_Vec3<T> theVec2)
        => new(theVec1.y() * theVec2.z() - theVec1.z() * theVec2.y(),
               theVec1.z() * theVec2.x() - theVec1.x() * theVec2.z(),
               theVec1.x() * theVec2.y() - theVec1.y() * theVec2.x());

    /// <summary>Linear interpolation between two vectors, <paramref name="theT"/> in 0..1.</summary>
    public static NCollection_Vec3<T> GetLERP(in NCollection_Vec3<T> theFrom, in NCollection_Vec3<T> theTo, T theT)
        => theFrom * (T.One - theT) + theTo * theT;

    public static NCollection_Vec3<T> DX() => new(T.One, T.Zero, T.Zero);

    public static NCollection_Vec3<T> DY() => new(T.Zero, T.One, T.Zero);

    public static NCollection_Vec3<T> DZ() => new(T.Zero, T.Zero, T.One);

    /// <summary>Copies the components into a span of at least 3 elements.</summary>
    public readonly void CopyTo(Span<T> theDestination)
    {
        theDestination[0] = v[0];
        theDestination[1] = v[1];
        theDestination[2] = v[2];
    }

    public static NCollection_Vec3<T> operator +(in NCollection_Vec3<T> theLeft, in NCollection_Vec3<T> theRight)
    {
        NCollection_Vec3<T> aResult = theLeft;
        aResult.Add(theRight);
        return aResult;
    }

    public static NCollection_Vec3<T> operator -(in NCollection_Vec3<T> theLeft, in NCollection_Vec3<T> theRight)
    {
        NCollection_Vec3<T> aResult = theLeft;
        aResult.Subtract(theRight);
        return aResult;
    }

    public static NCollection_Vec3<T> operator *(in NCollection_Vec3<T> theLeft, in NCollection_Vec3<T> theRight)
    {
        NCollection_Vec3<T> aResult = theLeft;
        aResult.Multiply(theRight);
        return aResult;
    }

    public static NCollection_Vec3<T> operator /(in NCollection_Vec3<T> theLeft, in NCollection_Vec3<T> theRight)
    {
        NCollection_Vec3<T> aResult = theLeft;
        aResult.Divide(theRight);
        return aResult;
    }

    public static NCollection_Vec3<T> operator /(in NCollection_Vec3<T> theVec, T theInvFactor)
    {
        NCollection_Vec3<T> aResult = theVec;
        aResult.Divide(theInvFactor);
        return aResult;
    }

    public static NCollection_Vec3<T> operator -(in NCollection_Vec3<T> theVec) => new(-theVec.x(), -theVec.y(), -theVec.z());

    public static NCollection_Vec3<T> operator *(in NCollection_Vec3<T> theVec, T theFactor) => theVec.Multiplied(theFactor);

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Vec3<T> other) => v[0].Equals(other.v[0]) && v[1].Equals(other.v[1]) && v[2].Equals(other.v[2]);

    public override readonly bool Equals(object? obj) => obj is NCollection_Vec3<T> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(v[0], v[1], v[2]);

    public static bool operator ==(in NCollection_Vec3<T> theLeft, in NCollection_Vec3<T> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in NCollection_Vec3<T> theLeft, in NCollection_Vec3<T> theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => NCollection_VecTools.Format<T>("NCollection_Vec3", v);
}
