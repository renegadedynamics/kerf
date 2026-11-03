// Port of: src/FoundationClasses/TKMath/BVH/BVH_Sorter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A tool to sort an abstract primitive set.</summary>
public abstract class Sorter<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    private bool _isParallel;

    protected Sorter()
    {
        _isParallel = false;
    }

    /// <summary>Sorts the set.</summary>
    public abstract void Perform(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set);

    /// <summary>Sorts the given (inclusive) range of the set.</summary>
    public abstract void Perform(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, int start, int final);

    /// <summary>The parallel flag.</summary>
    public bool IsParallel => _isParallel;

    /// <summary>Sets the flag allowing parallel execution.</summary>
    public void SetParallel(bool isParallel) => _isParallel = isParallel;
}
