// Port of: src/FoundationClasses/TKMath/BVH/BVH_Distance.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>
/// The minimum distance between an object and the elements of a BVH tree. A derived class defines
/// <c>RejectNode</c> (the distance from the object to a box) and <c>Accept</c> (the distance to an element).
/// </summary>
/// <typeparam name="T">The element type, which is also the metric (a distance).</typeparam>
/// <typeparam name="TVec">The vector type (dimension).</typeparam>
/// <typeparam name="TObject">The type of the object the distance is computed to.</typeparam>
/// <typeparam name="TSet">The type of the set the tree is built on.</typeparam>
public abstract class BVH_Distance<T, TVec, TObject, TSet> : BVH_Traverse<T, TVec, TSet, T>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
    where TSet : class
{
    protected T myDistance;        //!< Distance
    protected bool myIsDone;       //!< State of the algorithm
    protected TObject myObject;    //!< Object to compute the distance to

    protected BVH_Distance()
    {
        myDistance = T.MaxValue;
        myIsDone = false;
        myObject = default!;
    }

    /// <summary>Sets the object the distance is computed to.</summary>
    public void SetObject(in TObject theObject) => myObject = theObject;

    /// <summary>Computes the distance between the object and the tree.</summary>
    public T ComputeDistance()
    {
        myIsDone = Select() > 0;
        return myDistance;
    }

    /// <summary>True if the computation found an element.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The computed distance.</summary>
    public T Distance() => myDistance;

    /// <summary>The smaller metric is the better.</summary>
    public override bool IsMetricBetter(in T theLeft, in T theRight) => theLeft < theRight;

    /// <summary>Rejects the branch whose metric exceeds the current distance.</summary>
    public override bool RejectMetric(in T theMetric) => theMetric > myDistance;

    /// <summary>Stops when the distance is zero.</summary>
    public override bool Stop() => myDistance == T.Zero;
}
