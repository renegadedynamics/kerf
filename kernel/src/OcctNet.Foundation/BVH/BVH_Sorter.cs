// Port of: src/FoundationClasses/TKMath/BVH/BVH_Sorter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A tool to sort an abstract primitive set.</summary>
public abstract class BVH_Sorter<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    private bool myIsParallel;

    protected BVH_Sorter()
    {
        myIsParallel = false;
    }

    /// <summary>Sorts the set.</summary>
    public abstract void Perform(BVH_Set<T, TVec> theSet);

    /// <summary>Sorts the given (inclusive) range of the set.</summary>
    public abstract void Perform(BVH_Set<T, TVec> theSet, int theStart, int theFinal);

    /// <summary>The parallel flag.</summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>Sets the flag allowing parallel execution.</summary>
    public void SetParallel(bool isParallel) => myIsParallel = isParallel;
}
