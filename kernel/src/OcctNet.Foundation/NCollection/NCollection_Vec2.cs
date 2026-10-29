// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Vec2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;

namespace Occt;


/// <summary>A 2D vector of numbers (float, double or int), for low-level arrays and BVH.</summary>
/// <remarks>The default value is the zero vector, as OCCT's default constructor.</remarks>
public partial struct NCollection_Vec2<T> : INCollection_Vec<T, NCollection_Vec2<T>>
    where T : unmanaged, INumber<T>
{
    private NCollection_Buffer2<T> v;

    /// <summary>The number of components.</summary>
    public static int Length() => 2;

    /// <summary>All components set to <paramref name="theValue"/>.</summary>
    public NCollection_Vec2(T theValue)
    {
        v[0] = theValue;
        v[1] = theValue;
    }

    public NCollection_Vec2(T theX, T theY)
    {
        v[0] = theX;
        v[1] = theY;
    }

    /// <summary>
    /// Converts a vector of another element type, as OCCT's converting constructor (<c>static_cast</c> of each
    /// component: truncation towards zero from a real to an integer type).
    /// </summary>
    public static NCollection_Vec2<T> From<TOther>(in NCollection_Vec2<TOther> theOther) where TOther : unmanaged, INumber<TOther>
        => new(T.CreateTruncating(theOther[0]), T.CreateTruncating(theOther[1]));

    static NCollection_Vec2<T> INCollection_Vec<T, NCollection_Vec2<T>>.Create(T theValue) => new(theValue);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(T theX, T theY)
    {
        v[0] = theX;
        v[1] = theY;
    }

    /// <summary>The components, for writing (OCCT's <c>ChangeData()</c> and non-const <c>x()</c>, <c>y()</c>, ...).</summary>
    [UnscopedRef]
    internal Span<T> ChangeData() => v;

    internal void Add(in NCollection_Vec2<T> theAdd)
    {
        v[0] += theAdd.v[0];
        v[1] += theAdd.v[1];
    }

    internal void Subtract(in NCollection_Vec2<T> theDec)
    {
        v[0] -= theDec.v[0];
        v[1] -= theDec.v[1];
    }

    /// <summary>Per-component multiplication (OCCT's <c>operator*=</c> with a vector).</summary>
    internal void Multiply(in NCollection_Vec2<T> theRight)
    {
        v[0] *= theRight.v[0];
        v[1] *= theRight.v[1];
    }

    internal void Multiply(T theFactor)
    {
        v[0] *= theFactor;
        v[1] *= theFactor;
    }

    /// <summary>Per-component division by a scale factor (OCCT's <c>operator/=</c>).</summary>
    internal void Divide(T theInvFactor)
    {
        v[0] /= theInvFactor;
        v[1] /= theInvFactor;
    }

    /// <summary>Per-component division (OCCT's <c>operator/=</c> with a vector).</summary>
    internal void Divide(in NCollection_Vec2<T> theRight)
    {
        v[0] /= theRight.v[0];
        v[1] /= theRight.v[1];
    }

    // Copies with a mutator applied

    public readonly NCollection_Vec2<T> WithValues(T theX, T theY) => new(theX, theY);

    /// <summary>A copy with the component of the given (0-based) index replaced.</summary>
    public readonly NCollection_Vec2<T> WithValue(int theIndex, T theValue)
    {
        NCollection_Vec2<T> aCopy = this;
        aCopy[theIndex] = theValue;
        return aCopy;
    }

    public readonly NCollection_Vec2<T> Added(in NCollection_Vec2<T> theAdd) => this + theAdd;

    public readonly NCollection_Vec2<T> Subtracted(in NCollection_Vec2<T> theDec) => this - theDec;

    public readonly NCollection_Vec2<T> Divided(T theInvFactor) => this / theInvFactor;

    public readonly NCollection_Vec2<T> Divided(in NCollection_Vec2<T> theRight) => this / theRight;

    public readonly NCollection_Vec2<T> Multiplied(in NCollection_Vec2<T> theRight) => this * theRight;

    // Queries

    /// <summary>The component of the given (0-based) index.</summary>
    public T this[int theIndex]
    {
        readonly get => v[theIndex];
        internal set => v[theIndex] = value;
    }

    public readonly T x() => v[0];

    public readonly T y() => v[1];

    public readonly NCollection_Vec2<T> xy() => new(x(), y());

    public readonly NCollection_Vec2<T> yx() => new(y(), x());

    /// <summary>Exact comparison, without tolerance (C++ <c>==</c> on each component).</summary>
    public readonly bool IsEqual(in NCollection_Vec2<T> theOther) => v[0] == theOther.v[0] && v[1] == theOther.v[1];

    public readonly NCollection_Vec2<T> Multiplied(T theFactor)
    {
        return new(v[0] * theFactor, v[1] * theFactor);
    }

    /// <summary>The component-wise minimum of two vectors.</summary>
    public readonly NCollection_Vec2<T> cwiseMin(in NCollection_Vec2<T> theVec)
        => new(v[0] < theVec.v[0] ? v[0] : theVec.v[0], v[1] < theVec.v[1] ? v[1] : theVec.v[1]);

    /// <summary>The component-wise maximum of two vectors.</summary>
    public readonly NCollection_Vec2<T> cwiseMax(in NCollection_Vec2<T> theVec)
        => new(v[0] > theVec.v[0] ? v[0] : theVec.v[0], v[1] > theVec.v[1] ? v[1] : theVec.v[1]);

    /// <summary>The component-wise absolute value.</summary>
    public readonly NCollection_Vec2<T> cwiseAbs() => new(T.Abs(v[0]), T.Abs(v[1]));

    /// <summary>The largest component.</summary>
    public readonly T maxComp() => v[0] > v[1] ? v[0] : v[1];

    /// <summary>The smallest component.</summary>
    public readonly T minComp() => v[0] < v[1] ? v[0] : v[1];

    public readonly T Dot(in NCollection_Vec2<T> theOther) => x() * theOther.x() + y() * theOther.y();

    /// <summary>The vector's length.</summary>
    public readonly T Modulus() => NCollection_VecTools.Sqrt(x() * x() + y() * y());

    public readonly T SquareModulus() => x() * x() + y() * y();

    public static NCollection_Vec2<T> DX() => new(T.One, T.Zero);

    public static NCollection_Vec2<T> DY() => new(T.Zero, T.One);

    /// <summary>Copies the components into a span of at least 2 elements.</summary>
    public readonly void CopyTo(Span<T> theDestination)
    {
        theDestination[0] = v[0];
        theDestination[1] = v[1];
    }

    public static NCollection_Vec2<T> operator +(in NCollection_Vec2<T> theLeft, in NCollection_Vec2<T> theRight) => new(theLeft.v[0] + theRight.v[0], theLeft.v[1] + theRight.v[1]);

    public static NCollection_Vec2<T> operator -(in NCollection_Vec2<T> theLeft, in NCollection_Vec2<T> theRight) => new(theLeft.v[0] - theRight.v[0], theLeft.v[1] - theRight.v[1]);

    public static NCollection_Vec2<T> operator *(in NCollection_Vec2<T> theLeft, in NCollection_Vec2<T> theRight) => new(theLeft.v[0] * theRight.v[0], theLeft.v[1] * theRight.v[1]);

    public static NCollection_Vec2<T> operator /(in NCollection_Vec2<T> theLeft, in NCollection_Vec2<T> theRight) => new(theLeft.v[0] / theRight.v[0], theLeft.v[1] / theRight.v[1]);

    public static NCollection_Vec2<T> operator /(in NCollection_Vec2<T> theVec, T theInvFactor) => new(theVec.v[0] / theInvFactor, theVec.v[1] / theInvFactor);

    public static NCollection_Vec2<T> operator -(in NCollection_Vec2<T> theVec) => new(-theVec.x(), -theVec.y());

    public static NCollection_Vec2<T> operator *(in NCollection_Vec2<T> theVec, T theFactor) => theVec.Multiplied(theFactor);

    // Exact equality, for use as a value

    public readonly bool Equals(NCollection_Vec2<T> other) => v[0].Equals(other.v[0]) && v[1].Equals(other.v[1]);

    public override readonly bool Equals(object? obj) => obj is NCollection_Vec2<T> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(v[0], v[1]);

    public static bool operator ==(in NCollection_Vec2<T> theLeft, in NCollection_Vec2<T> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in NCollection_Vec2<T> theLeft, in NCollection_Vec2<T> theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => NCollection_VecTools.Format<T>("NCollection_Vec2", v);
}
