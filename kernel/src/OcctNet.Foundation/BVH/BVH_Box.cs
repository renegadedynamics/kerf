// Port of: src/FoundationClasses/TKMath/BVH/BVH_Box.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>An axis-aligned bounding box (AABB) of BVH vectors.</summary>
/// <remarks>
/// OCCT's default constructor makes an invalid (empty) box, with the minimum corner at the largest value and the
/// maximum corner at the lowest; so does <c>new BVH_Box&lt;T, TVec&gt;()</c>. <c>default(BVH_Box&lt;T, TVec&gt;)</c>
/// is a valid box of one point at the origin: ported code must write <c>new()</c>, and arrays of boxes must be
/// filled explicitly. <see cref="BVH_BaseBox.Transform{T}"/> and <see cref="BVH_BaseBox.Transformed{T}"/>, which
/// OCCT defines only for three dimensions, are extension methods.
/// </remarks>
/// <typeparam name="T">The element type (float or double).</typeparam>
/// <typeparam name="TVec">The vector type, which fixes the dimension (OCCT's <c>N</c>).</typeparam>
public partial struct BVH_Box<T, TVec> : IEquatable<BVH_Box<T, TVec>>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    private TVec myMinPoint; //!< Minimum point of bounding box (max<T> when invalid)
    private TVec myMaxPoint; //!< Maximum point of bounding box (lowest<T> when invalid)

    // The minimum point sentinel value for an invalid box.
    private static T minSentinel() => T.MaxValue;

    // The maximum point sentinel value for an invalid box.
    private static T maxSentinel() => T.MinValue;

    /// <summary>An invalid (empty) box.</summary>
    public BVH_Box()
    {
        myMinPoint = TVec.Create(minSentinel());
        myMaxPoint = TVec.Create(maxSentinel());
    }

    /// <summary>The box of one point.</summary>
    public BVH_Box(in TVec thePoint)
    {
        myMinPoint = thePoint;
        myMaxPoint = thePoint;
    }

    /// <summary>The box with the given corners.</summary>
    public BVH_Box(in TVec theMinPoint, in TVec theMaxPoint)
    {
        myMinPoint = theMinPoint;
        myMaxPoint = theMaxPoint;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Clear()
    {
        myMinPoint = TVec.Create(minSentinel());
        myMaxPoint = TVec.Create(maxSentinel());
    }

    /// <summary>Appends a point to the box.</summary>
    internal void Add(in TVec thePoint)
    {
        BVH.CwiseMin<T, TVec>(ref myMinPoint, thePoint);
        BVH.CwiseMax<T, TVec>(ref myMaxPoint, thePoint);
    }

    /// <summary>Combines the box with another one.</summary>
    internal void Combine(in BVH_Box<T, TVec> theBox)
    {
        if (theBox.IsValid())
        {
            BVH.CwiseMin<T, TVec>(ref myMinPoint, theBox.myMinPoint);
            BVH.CwiseMax<T, TVec>(ref myMaxPoint, theBox.myMaxPoint);
        }
    }

    /// <summary>The minimum corner, for writing (OCCT's non-const <c>CornerMin()</c>).</summary>
    [UnscopedRef]
    internal ref TVec ChangeCornerMin() => ref myMinPoint;

    /// <summary>The maximum corner, for writing (OCCT's non-const <c>CornerMax()</c>).</summary>
    [UnscopedRef]
    internal ref TVec ChangeCornerMax() => ref myMaxPoint;

    // Copies with a mutator applied

    /// <summary>An invalid (empty) box, as <c>Clear</c> makes.</summary>
    public readonly BVH_Box<T, TVec> Cleared() => new();

    public readonly BVH_Box<T, TVec> Added(in TVec thePoint)
    {
        BVH_Box<T, TVec> aCopy = this;
        aCopy.Add(thePoint);
        return aCopy;
    }

    public readonly BVH_Box<T, TVec> Combined(in BVH_Box<T, TVec> theBox)
    {
        BVH_Box<T, TVec> aCopy = this;
        aCopy.Combine(theBox);
        return aCopy;
    }

    public readonly BVH_Box<T, TVec> WithCornerMin(in TVec theMinPoint) => new(theMinPoint, myMaxPoint);

    public readonly BVH_Box<T, TVec> WithCornerMax(in TVec theMaxPoint) => new(myMinPoint, theMaxPoint);

    // Queries

    /// <summary>True if the box is valid (not empty). Only the first component is checked, as in OCCT.</summary>
    public readonly bool IsValid() => myMinPoint[0] <= myMaxPoint[0];

    public readonly TVec CornerMin() => myMinPoint;

    public readonly TVec CornerMax() => myMaxPoint;

    /// <summary>The surface area of the box; the perimeter if the box is degenerated into a line.</summary>
    public readonly T Area() => !IsValid() ? T.Zero : BVH.SurfaceArea<T, TVec>(myMaxPoint - myMinPoint);

    /// <summary>The diagonal of the box.</summary>
    public readonly TVec Size() => myMaxPoint - myMinPoint;

    /// <summary>The centre of the box.</summary>
    public readonly TVec Center() => (myMinPoint + myMaxPoint) * T.CreateTruncating(0.5);

    /// <summary>The centre of the box along the given axis.</summary>
    public readonly T Center(int theAxis) => BVH.CenterAxis(this, theAxis);

    /// <summary>True if this box is out of the other box (or the other box is invalid).</summary>
    public readonly bool IsOut(in BVH_Box<T, TVec> theOther)
    {
        if (!theOther.IsValid())
        {
            return true;
        }

        return IsOut(theOther.myMinPoint, theOther.myMaxPoint);
    }

    /// <summary>True if this box is out of the box defined by two points.</summary>
    public readonly bool IsOut(in TVec theMinPoint, in TVec theMaxPoint)
    {
        if (!IsValid())
        {
            return true;
        }

        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (myMinPoint[i] > theMaxPoint[i] || myMaxPoint[i] < theMinPoint[i])
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>True if this box fully contains the other box; <paramref name="hasOverlap"/> tells whether they overlap.</summary>
    public readonly bool Contains(in BVH_Box<T, TVec> theOther, out bool hasOverlap)
    {
        hasOverlap = false;
        if (!theOther.IsValid())
        {
            return false;
        }

        return Contains(theOther.myMinPoint, theOther.myMaxPoint, out hasOverlap);
    }

    /// <summary>True if this box fully contains the box defined by two points.</summary>
    public readonly bool Contains(in TVec theMinPoint, in TVec theMaxPoint, out bool hasOverlap)
    {
        hasOverlap = false;
        if (!IsValid())
        {
            return false;
        }

        bool isInside = true;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            hasOverlap = (myMinPoint[i] <= theMaxPoint[i] && myMaxPoint[i] >= theMinPoint[i]);
            if (!hasOverlap)
            {
                return false;
            }

            isInside = isInside && (myMinPoint[i] <= theMinPoint[i] && myMaxPoint[i] >= theMaxPoint[i]);
        }

        return isInside;
    }

    /// <summary>True if the point is out of the box (or the box is invalid).</summary>
    public readonly bool IsOut(in TVec thePoint)
    {
        if (!IsValid())
        {
            return true;
        }

        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (thePoint[i] < myMinPoint[i] || thePoint[i] > myMaxPoint[i])
            {
                return true;
            }
        }

        return false;
    }

    // Exact equality, for use as a value (DumpJson and InitFromJson are not ported: ToString replaces them)

    public readonly bool Equals(BVH_Box<T, TVec> other) => myMinPoint.Equals(other.myMinPoint) && myMaxPoint.Equals(other.myMaxPoint);

    public override readonly bool Equals(object? obj) => obj is BVH_Box<T, TVec> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myMinPoint, myMaxPoint);

    public static bool operator ==(in BVH_Box<T, TVec> theLeft, in BVH_Box<T, TVec> theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BVH_Box<T, TVec> theLeft, in BVH_Box<T, TVec> theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"BVH_Box({myMinPoint}, {myMaxPoint})";
}

/// <summary>
/// OCCT's <c>BVH_BaseBox&lt;T, 3, BVH_Box&gt;</c>: the members OCCT gives only three-dimensional boxes, as
/// extension methods.
/// </summary>
public static class BVH_BaseBox
{
    /// <summary>Transforms this box with the given transformation.</summary>
    internal static void Transform<T>(ref this BVH_Box<T, NCollection_Vec3<T>> theBox, in NCollection_Mat4<T> theTransform)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
    {
        if (theTransform.IsIdentity())
        {
            return;
        }

        if (!theBox.IsValid())
        {
            return;
        }

        BVH_Box<T, NCollection_Vec3<T>> aBox = theBox.Transformed(theTransform);

        theBox.ChangeCornerMin() = aBox.CornerMin();
        theBox.ChangeCornerMax() = aBox.CornerMax();
    }

    /// <summary>
    /// The box enclosing this box transformed by an affine transformation (James Arvo's algorithm: for each row of
    /// the matrix, the smaller and larger products with the corners).
    /// </summary>
    public static BVH_Box<T, NCollection_Vec3<T>> Transformed<T>(this in BVH_Box<T, NCollection_Vec3<T>> theBox, in NCollection_Mat4<T> theTransform)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
    {
        if (theTransform.IsIdentity())
        {
            return theBox;
        }

        if (!theBox.IsValid())
        {
            return theBox;
        }

        // Untransformed AABB min and max points
        NCollection_Vec3<T> anOldMinPnt = theBox.CornerMin();
        NCollection_Vec3<T> anOldMaxPnt = theBox.CornerMax();

        // Define an empty AABB located in the transformation translation point
        NCollection_Vec4<T> aTranslation = theTransform.GetColumn(3);
        NCollection_Vec3<T> aNewMinPnt = new(aTranslation.x(), aTranslation.y(), aTranslation.z());
        NCollection_Vec3<T> aNewMaxPnt = new(aTranslation.x(), aTranslation.y(), aTranslation.z());

        for (int aCol = 0; aCol < 3; ++aCol)
        {
            for (int aRow = 0; aRow < 3; ++aRow)
            {
                T aMatValue = theTransform.GetValue(aRow, aCol);
                T anOffset1 = aMatValue * anOldMinPnt[aCol];
                T anOffset2 = aMatValue * anOldMaxPnt[aCol];

                aNewMinPnt.ChangeData()[aRow] += BVH.Min(anOffset1, anOffset2);
                aNewMaxPnt.ChangeData()[aRow] += BVH.Max(anOffset1, anOffset2);
            }
        }

        return new BVH_Box<T, NCollection_Vec3<T>>(aNewMinPnt, aNewMaxPnt);
    }
}

public static partial class BVH
{
    /// <summary>The centre of the box along the given axis (<c>BVH::CenterAxis</c>).</summary>
    public static T CenterAxis<T, TVec>(in BVH_Box<T, TVec> theBox, int theAxis)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
        => (theBox.CornerMin()[theAxis] + theBox.CornerMax()[theAxis]) * T.CreateTruncating(0.5);

    /// <summary>
    /// The surface area of a box of the given size (<c>BVH::SurfaceCalculator</c>): the area in 2D, the 3D surface
    /// area from X, Y and Z otherwise (the W component is ignored); the perimeter for a degenerate box.
    /// </summary>
    public static T SurfaceArea<T, TVec>(in TVec theSize)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        if (TVec.Length() == 2)
        {
            T anArea2 = T.Abs(theSize[0] * theSize[1]);
            if (anArea2 < Epsilon<T>())
            {
                return T.Abs(theSize[0]) + T.Abs(theSize[1]);
            }

            return anArea2;
        }

        // For N >= 3, compute standard 3D surface area.
        T anArea = (T.Abs(theSize[0] * theSize[1]) + T.Abs(theSize[0] * theSize[2])
                    + T.Abs(theSize[2] * theSize[1]))
                   * T.CreateTruncating(2.0);
        if (anArea < Epsilon<T>())
        {
            return T.Abs(theSize[0]) + T.Abs(theSize[1]) + T.Abs(theSize[2]);
        }

        return anArea;
    }

    /// <summary>Component-wise minimum in place (<c>BVH::BoxMinMax::CwiseMin</c>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CwiseMin<T, TVec>(ref TVec theVec1, in TVec theVec2)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        for (int i = 0; i < TVec.Length(); ++i)
        {
            ref T aComp = ref NCollection_VecTools.Change<T, TVec>(ref theVec1, i);
            aComp = Min(aComp, theVec2[i]);
        }
    }

    /// <summary>Component-wise maximum in place (<c>BVH::BoxMinMax::CwiseMax</c>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CwiseMax<T, TVec>(ref TVec theVec1, in TVec theVec2)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        for (int i = 0; i < TVec.Length(); ++i)
        {
            ref T aComp = ref NCollection_VecTools.Change<T, TVec>(ref theVec1, i);
            aComp = Max(aComp, theVec2[i]);
        }
    }
}
