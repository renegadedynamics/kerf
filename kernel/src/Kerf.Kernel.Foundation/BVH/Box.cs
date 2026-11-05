// Port of: src/FoundationClasses/TKMath/BVH/BVH_Box.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>An axis-aligned bounding box (AABB) of BVH vectors.</summary>
/// <remarks>
/// OCCT's default constructor makes an invalid (empty) box, with the minimum corner at the largest value and the
/// maximum corner at the lowest; so does <c>new BVH_Box&lt;T, TVec&gt;()</c>. <c>default(BVH_Box&lt;T, TVec&gt;)</c>
/// is a valid box of one point at the origin: ported code must write <c>new()</c>, and arrays of boxes must be
/// filled explicitly. <see cref="BaseBox.Transform{T}"/> and <see cref="BaseBox.Transformed{T}"/>, which
/// OCCT defines only for three dimensions, are extension methods.
/// </remarks>
/// <typeparam name="T">The element type (float or double).</typeparam>
/// <typeparam name="TVec">The vector type, which fixes the dimension (OCCT's <c>N</c>).</typeparam>
internal partial struct Box<T, TVec> : IEquatable<Box<T, TVec>>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    private TVec _minPoint; //!< Minimum point of bounding box (max<T> when invalid)
    private TVec _maxPoint; //!< Maximum point of bounding box (lowest<T> when invalid)

    // The minimum point sentinel value for an invalid box.
    private static T MinSentinel() => T.MaxValue;

    // The maximum point sentinel value for an invalid box.
    private static T MaxSentinel() => T.MinValue;

    /// <summary>An invalid (empty) box.</summary>
    public Box()
    {
        _minPoint = TVec.Create(MinSentinel());
        _maxPoint = TVec.Create(MaxSentinel());
    }

    /// <summary>The box of one point.</summary>
    public Box(in TVec point)
    {
        _minPoint = point;
        _maxPoint = point;
    }

    /// <summary>The box with the given corners.</summary>
    public Box(in TVec minPoint, in TVec maxPoint)
    {
        _minPoint = minPoint;
        _maxPoint = maxPoint;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Clear()
    {
        _minPoint = TVec.Create(MinSentinel());
        _maxPoint = TVec.Create(MaxSentinel());
    }

    /// <summary>Appends a point to the box.</summary>
    internal void Add(in TVec point)
    {
        BvhTools.CwiseMin<T, TVec>(ref _minPoint, point);
        BvhTools.CwiseMax<T, TVec>(ref _maxPoint, point);
    }

    /// <summary>Combines the box with another one.</summary>
    internal void Combine(in Box<T, TVec> box)
    {
        if (box.IsValid)
        {
            BvhTools.CwiseMin<T, TVec>(ref _minPoint, box._minPoint);
            BvhTools.CwiseMax<T, TVec>(ref _maxPoint, box._maxPoint);
        }
    }

    /// <summary>The minimum corner, for writing (OCCT's non-const <c>CornerMin()</c>).</summary>
    [UnscopedRef]
    internal ref TVec ChangeCornerMin() => ref _minPoint;

    /// <summary>The maximum corner, for writing (OCCT's non-const <c>CornerMax()</c>).</summary>
    [UnscopedRef]
    internal ref TVec ChangeCornerMax() => ref _maxPoint;

    // Copies with a mutator applied

    /// <summary>An invalid (empty) box, as <c>Clear</c> makes.</summary>
    public readonly Box<T, TVec> Cleared() => new();

    public readonly Box<T, TVec> Added(in TVec point)
    {
        Box<T, TVec> copy = this;
        copy.Add(point);
        return copy;
    }

    public readonly Box<T, TVec> Combined(in Box<T, TVec> box)
    {
        Box<T, TVec> copy = this;
        copy.Combine(box);
        return copy;
    }

    public readonly Box<T, TVec> WithCornerMin(in TVec minPoint) => new(minPoint, _maxPoint);

    public readonly Box<T, TVec> WithCornerMax(in TVec maxPoint) => new(_minPoint, maxPoint);

    // Queries

    /// <summary>True if the box is valid (not empty). Only the first component is checked, as in OCCT.</summary>
    public readonly bool IsValid => _minPoint[0] <= _maxPoint[0];

    public readonly TVec CornerMin() => _minPoint;

    public readonly TVec CornerMax() => _maxPoint;

    /// <summary>The surface area of the box; the perimeter if the box is degenerated into a line.</summary>
    public readonly T Area() => !IsValid ? T.Zero : BvhTools.SurfaceArea<T, TVec>(_maxPoint - _minPoint);

    /// <summary>The diagonal of the box.</summary>
    public readonly TVec Size() => _maxPoint - _minPoint;

    /// <summary>The centre of the box.</summary>
    public readonly TVec Center() => (_minPoint + _maxPoint) * T.CreateTruncating(0.5);

    /// <summary>The centre of the box along the given axis.</summary>
    public readonly T Center(int axis) => BvhTools.CenterAxis(this, axis);

    /// <summary>True if this box is out of the other box (or the other box is invalid).</summary>
    public readonly bool IsOut(in Box<T, TVec> other)
    {
        if (!other.IsValid)
        {
            return true;
        }

        return IsOut(other._minPoint, other._maxPoint);
    }

    /// <summary>True if this box is out of the box defined by two points.</summary>
    public readonly bool IsOut(in TVec minPoint, in TVec maxPoint)
    {
        if (!IsValid)
        {
            return true;
        }

        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (_minPoint[i] > maxPoint[i] || _maxPoint[i] < minPoint[i])
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>True if this box fully contains the other box; <paramref name="hasOverlap"/> tells whether they overlap.</summary>
    public readonly bool Contains(in Box<T, TVec> other, out bool hasOverlap)
    {
        hasOverlap = false;
        if (!other.IsValid)
        {
            return false;
        }

        return Contains(other._minPoint, other._maxPoint, out hasOverlap);
    }

    /// <summary>True if this box fully contains the box defined by two points.</summary>
    public readonly bool Contains(in TVec minPoint, in TVec maxPoint, out bool hasOverlap)
    {
        hasOverlap = false;
        if (!IsValid)
        {
            return false;
        }

        bool isInside = true;
        for (int i = 0; i < TVec.Length(); ++i)
        {
            hasOverlap = (_minPoint[i] <= maxPoint[i] && _maxPoint[i] >= minPoint[i]);
            if (!hasOverlap)
            {
                return false;
            }

            isInside = isInside && (_minPoint[i] <= minPoint[i] && _maxPoint[i] >= maxPoint[i]);
        }

        return isInside;
    }

    /// <summary>True if the point is out of the box (or the box is invalid).</summary>
    public readonly bool IsOut(in TVec point)
    {
        if (!IsValid)
        {
            return true;
        }

        for (int i = 0; i < TVec.Length(); ++i)
        {
            if (point[i] < _minPoint[i] || point[i] > _maxPoint[i])
            {
                return true;
            }
        }

        return false;
    }

    // Exact equality, for use as a value (DumpJson and InitFromJson are not ported: ToString replaces them)

    public readonly bool Equals(Box<T, TVec> other) => _minPoint.Equals(other._minPoint) && _maxPoint.Equals(other._maxPoint);

    public override readonly bool Equals(object? obj) => obj is Box<T, TVec> other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_minPoint, _maxPoint);

    public static bool operator ==(in Box<T, TVec> left, in Box<T, TVec> right) => left.Equals(right);

    public static bool operator !=(in Box<T, TVec> left, in Box<T, TVec> right) => !left.Equals(right);

    public override readonly string ToString() => $"BVH_Box({_minPoint}, {_maxPoint})";
}

/// <summary>
/// OCCT's <c>BVH_BaseBox&lt;T, 3, BVH_Box&gt;</c>: the members OCCT gives only three-dimensional boxes, as
/// extension methods.
/// </summary>
internal static class BaseBox
{
    /// <summary>Transforms this box with the given transformation.</summary>
    internal static void Transform<T>(ref this Box<T, NCollection_Vec3<T>> box, in NCollection_Mat4<T> transform)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
    {
        if (transform.IsIdentity())
        {
            return;
        }

        if (!box.IsValid)
        {
            return;
        }

        Box<T, NCollection_Vec3<T>> aBox = box.Transformed(transform);

        box.ChangeCornerMin() = aBox.CornerMin();
        box.ChangeCornerMax() = aBox.CornerMax();
    }

    /// <summary>
    /// The box enclosing this box transformed by an affine transformation (James Arvo's algorithm: for each row of
    /// the matrix, the smaller and larger products with the corners).
    /// </summary>
    public static Box<T, NCollection_Vec3<T>> Transformed<T>(this in Box<T, NCollection_Vec3<T>> box, in NCollection_Mat4<T> transform)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
    {
        if (transform.IsIdentity())
        {
            return box;
        }

        if (!box.IsValid)
        {
            return box;
        }

        // Untransformed AABB min and max points
        NCollection_Vec3<T> oldMinPnt = box.CornerMin();
        NCollection_Vec3<T> oldMaxPnt = box.CornerMax();

        // Define an empty AABB located in the transformation translation point
        NCollection_Vec4<T> translation = transform.GetColumn(3);
        NCollection_Vec3<T> newMinPnt = new(translation.X(), translation.Y(), translation.Z());
        NCollection_Vec3<T> newMaxPnt = new(translation.X(), translation.Y(), translation.Z());

        for (int col = 0; col < 3; ++col)
        {
            for (int row = 0; row < 3; ++row)
            {
                T matValue = transform.GetValue(row, col);
                T offset1 = matValue * oldMinPnt[col];
                T offset2 = matValue * oldMaxPnt[col];

                newMinPnt.ChangeData()[row] += BvhTools.Min(offset1, offset2);
                newMaxPnt.ChangeData()[row] += BvhTools.Max(offset1, offset2);
            }
        }

        return new Box<T, NCollection_Vec3<T>>(newMinPnt, newMaxPnt);
    }
}

internal static partial class BvhTools
{
    /// <summary>The centre of the box along the given axis (<c>BVH::CenterAxis</c>).</summary>
    public static T CenterAxis<T, TVec>(in Box<T, TVec> box, int axis)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
        => (box.CornerMin()[axis] + box.CornerMax()[axis]) * T.CreateTruncating(0.5);

    /// <summary>
    /// The surface area of a box of the given size (<c>BVH::SurfaceCalculator</c>): the area in 2D, the 3D surface
    /// area from X, Y and Z otherwise (the W component is ignored); the perimeter for a degenerate box.
    /// </summary>
    public static T SurfaceArea<T, TVec>(in TVec size)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        if (TVec.Length() == 2)
        {
            T area2 = T.Abs(size[0] * size[1]);
            if (area2 < Epsilon<T>())
            {
                return T.Abs(size[0]) + T.Abs(size[1]);
            }

            return area2;
        }

        // For N >= 3, compute standard 3D surface area.
        T area = (T.Abs(size[0] * size[1]) + T.Abs(size[0] * size[2])
                    + T.Abs(size[2] * size[1]))
                   * T.CreateTruncating(2.0);
        if (area < Epsilon<T>())
        {
            return T.Abs(size[0]) + T.Abs(size[1]) + T.Abs(size[2]);
        }

        return area;
    }

    /// <summary>Component-wise minimum in place (<c>BVH::BoxMinMax::CwiseMin</c>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CwiseMin<T, TVec>(ref TVec vec1, in TVec vec2)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        for (int i = 0; i < TVec.Length(); ++i)
        {
            ref T comp = ref NCollection_VecTools.Change<T, TVec>(ref vec1, i);
            comp = Min(comp, vec2[i]);
        }
    }

    /// <summary>Component-wise maximum in place (<c>BVH::BoxMinMax::CwiseMax</c>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void CwiseMax<T, TVec>(ref TVec vec1, in TVec vec2)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        for (int i = 0; i < TVec.Length(); ++i)
        {
            ref T comp = ref NCollection_VecTools.Change<T, TVec>(ref vec1, i);
            comp = Max(comp, vec2[i]);
        }
    }
}
