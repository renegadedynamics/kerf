// Port of: src/FoundationClasses/TKMath/BVH/BVH_QuickSorter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// Sorts an abstract set by the centroids of its primitives along the given axis (X - 0, Y - 1, Z - 2), with an
/// O(n log n) sort of indices followed by an in-place permutation of the set.
/// </summary>
/// <remarks>
/// OCCT sorts with <c>std::sort</c>, the port with <see cref="MemoryExtensions.Sort{T}(Span{T}, Comparison{T})"/>.
/// Both are unstable introsorts, but not the same one, so primitives with equal centroids can end up in a different
/// order than in OCCT (and the trees built from them can differ accordingly).
/// </remarks>
internal class QuickSorter<T, TVec> : Sorter<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Axis used to arrange the primitives (X - 0, Y - 1, Z - 2).</summary>
    protected int _axis;

    /// <summary>Creates a new quick sorter for the given axis.</summary>
    public QuickSorter(int axis = 0)
    {
        _axis = axis;
    }

    /// <summary>Sorts the set.</summary>
    public override void Perform(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set) => Perform(set, 0, set.Size() - 1);

    /// <summary>Sorts the given (inclusive) range of the set.</summary>
    public override void Perform(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, int start, int final)
    {
        int size = final - start + 1;
        if (size <= 1)
        {
            return;
        }

        // Create index array for sorting
        int[] indices = new int[size];
        for (int i = 0; i < size; ++i)
        {
            indices[i] = i;
        }

        // Sort indices by center value (the comparison returns OCCT's strict "less" as an ordering)
        int axis = _axis;
        indices.AsSpan().Sort((a, b) =>
        {
            T centerA = set.Center(start + a, axis);
            T centerB = set.Center(start + b, axis);
            return centerA < centerB ? -1 : (centerB < centerA ? 1 : 0);
        });

        // Compute inverse permutation: invPerm[i] = where element i should go
        int[] invPerm = new int[size];
        for (int i = 0; i < size; ++i)
        {
            invPerm[indices[i]] = i;
        }

        // Apply permutation using cycle-based algorithm - O(n) swaps total
        for (int i = 0; i < size; ++i)
        {
            // Follow the cycle starting at position i
            while (invPerm[i] != i)
            {
                int j = invPerm[i];
                set.Swap(start + i, start + j);
                (invPerm[i], invPerm[j]) = (invPerm[j], invPerm[i]);
            }
        }
    }
}
