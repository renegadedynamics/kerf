// Port of: src/FoundationClasses/TKMath/BVH/BVH_Distance.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// The minimum distance between an object and the elements of a BVH tree. A derived class defines
/// <c>RejectNode</c> (the distance from the object to a box) and <c>Accept</c> (the distance to an element).
/// </summary>
/// <typeparam name="T">The element type, which is also the metric (a distance).</typeparam>
/// <typeparam name="TVec">The vector type (dimension).</typeparam>
/// <typeparam name="TObject">The type of the object the distance is computed to.</typeparam>
/// <typeparam name="TSet">The type of the set the tree is built on.</typeparam>
internal abstract class BvhDistance<T, TVec, TObject, TSet> : Traverse<T, TVec, TSet, T>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
{
    protected T _distance;        //!< Distance
    protected bool _isDone;       //!< State of the algorithm
    protected TObject _object;    //!< Object to compute the distance to

    protected BvhDistance()
    {
        _distance = T.MaxValue;
        _isDone = false;
        _object = default!;
    }

    /// <summary>Sets the object the distance is computed to.</summary>
    public void SetObject(in TObject theObject) => _object = theObject;

    /// <summary>Computes the distance between the object and the tree.</summary>
    public T ComputeDistance()
    {
        _isDone = Select() > 0;
        return _distance;
    }

    /// <summary>True if the computation found an element.</summary>
    public bool IsDone => _isDone;

    /// <summary>The computed distance.</summary>
    public T Distance() => _distance;

    /// <summary>The smaller metric is the better.</summary>
    public override bool IsMetricBetter(in T left, in T right) => left < right;

    /// <summary>Rejects the branch whose metric exceeds the current distance.</summary>
    public override bool RejectMetric(in T metric) => metric > _distance;

    /// <summary>Stops when the distance is zero.</summary>
    public override bool Stop() => _distance == T.Zero;
}
