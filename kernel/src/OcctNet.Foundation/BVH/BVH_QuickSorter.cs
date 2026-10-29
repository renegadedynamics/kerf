// Port of: src/FoundationClasses/TKMath/BVH/BVH_QuickSorter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// Sorts an abstract set by the centroids of its primitives along the given axis (X - 0, Y - 1, Z - 2), with an
/// O(n log n) sort of indices followed by an in-place permutation of the set.
/// </summary>
/// <remarks>
/// OCCT sorts with <c>std::sort</c>, the port with <see cref="MemoryExtensions.Sort{T}(Span{T}, Comparison{T})"/>.
/// Both are unstable introsorts, but not the same one, so primitives with equal centroids can end up in a different
/// order than in OCCT (and the trees built from them can differ accordingly).
/// </remarks>
public class BVH_QuickSorter<T, TVec> : BVH_Sorter<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Axis used to arrange the primitives (X - 0, Y - 1, Z - 2).</summary>
    protected int myAxis;

    /// <summary>Creates a new quick sorter for the given axis.</summary>
    public BVH_QuickSorter(int theAxis = 0)
    {
        myAxis = theAxis;
    }

    /// <summary>Sorts the set.</summary>
    public override void Perform(BVH_Set<T, TVec> theSet) => Perform(theSet, 0, theSet.Size() - 1);

    /// <summary>Sorts the given (inclusive) range of the set.</summary>
    public override void Perform(BVH_Set<T, TVec> theSet, int theStart, int theFinal)
    {
        int aSize = theFinal - theStart + 1;
        if (aSize <= 1)
        {
            return;
        }

        // Create index array for sorting
        int[] anIndices = new int[aSize];
        for (int i = 0; i < aSize; ++i)
        {
            anIndices[i] = i;
        }

        // Sort indices by center value (the comparison returns OCCT's strict "less" as an ordering)
        int anAxis = myAxis;
        anIndices.AsSpan().Sort((a, b) =>
        {
            T aCenterA = theSet.Center(theStart + a, anAxis);
            T aCenterB = theSet.Center(theStart + b, anAxis);
            return aCenterA < aCenterB ? -1 : (aCenterB < aCenterA ? 1 : 0);
        });

        // Compute inverse permutation: invPerm[i] = where element i should go
        int[] anInvPerm = new int[aSize];
        for (int i = 0; i < aSize; ++i)
        {
            anInvPerm[anIndices[i]] = i;
        }

        // Apply permutation using cycle-based algorithm - O(n) swaps total
        for (int i = 0; i < aSize; ++i)
        {
            // Follow the cycle starting at position i
            while (anInvPerm[i] != i)
            {
                int j = anInvPerm[i];
                theSet.Swap(theStart + i, theStart + j);
                (anInvPerm[i], anInvPerm[j]) = (anInvPerm[j], anInvPerm[i]);
            }
        }
    }
}
