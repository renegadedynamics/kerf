// Port of: src/FoundationClasses/TKMath/BVH/BVH_PairDistance.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// The minimum (square) distance between the elements of two BVH trees. A derived class defines only <c>Accept</c>,
/// the distance between two elements.
/// </summary>
public abstract class PairDistance<T, TVec, TSet> : PairTraverse<T, TVec, TSet, T>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
{
    protected T _distance;   //!< Square distance
    protected bool _isDone;  //!< State of the algorithm

    protected PairDistance()
    {
        _distance = T.MaxValue;
        _isDone = false;
    }

    /// <summary>Computes the distance between the two trees.</summary>
    public T ComputeDistance()
    {
        _isDone = Select() > 0;
        return _distance;
    }

    /// <summary>True if the computation found a pair.</summary>
    public bool IsDone => _isDone;

    /// <summary>The computed (square) distance.</summary>
    public T Distance() => _distance;

    /// <summary>The smaller metric is the better.</summary>
    public override bool IsMetricBetter(in T left, in T right) => left < right;

    /// <summary>The square distance between the boxes of the nodes; rejects the pair beyond the current distance.</summary>
    public override bool RejectNode(in TVec cornerMin1, in TVec cornerMax1, in TVec cornerMin2, in TVec cornerMax2, out T metric)
    {
        metric = Tools<T, TVec>.BoxBoxSquareDistance(cornerMin1, cornerMax1, cornerMin2, cornerMax2);
        return metric > _distance;
    }

    /// <summary>Rejects the branch whose metric exceeds the current distance.</summary>
    public override bool RejectMetric(in T metric) => metric > _distance;

    /// <summary>Stops when the distance is zero.</summary>
    public override bool Stop() => _distance == T.Zero;
}
