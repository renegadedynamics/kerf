// Port of: src/FoundationClasses/TKMath/BVH/BVH_PairDistance.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// The minimum (square) distance between the elements of two BVH trees. A derived class defines only <c>Accept</c>,
/// the distance between two elements.
/// </summary>
public abstract class BVH_PairDistance<T, TVec, TSet> : BVH_PairTraverse<T, TVec, TSet, T>
    where T : unmanaged, IFloatingPointIeee754<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
{
    protected T myDistance;   //!< Square distance
    protected bool myIsDone;  //!< State of the algorithm

    protected BVH_PairDistance()
    {
        myDistance = T.MaxValue;
        myIsDone = false;
    }

    /// <summary>Computes the distance between the two trees.</summary>
    public T ComputeDistance()
    {
        myIsDone = Select() > 0;
        return myDistance;
    }

    /// <summary>True if the computation found a pair.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The computed (square) distance.</summary>
    public T Distance() => myDistance;

    /// <summary>The smaller metric is the better.</summary>
    public override bool IsMetricBetter(in T theLeft, in T theRight) => theLeft < theRight;

    /// <summary>The square distance between the boxes of the nodes; rejects the pair beyond the current distance.</summary>
    public override bool RejectNode(in TVec theCornerMin1, in TVec theCornerMax1, in TVec theCornerMin2, in TVec theCornerMax2, out T theMetric)
    {
        theMetric = BVH_Tools<T, TVec>.BoxBoxSquareDistance(theCornerMin1, theCornerMax1, theCornerMin2, theCornerMax2);
        return theMetric > myDistance;
    }

    /// <summary>Rejects the branch whose metric exceeds the current distance.</summary>
    public override bool RejectMetric(in T theMetric) => theMetric > myDistance;

    /// <summary>Stops when the distance is zero.</summary>
    public override bool Stop() => myDistance == T.Zero;
}
