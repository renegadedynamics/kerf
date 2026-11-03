// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Interference.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Interference_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using System.Diagnostics.CodeAnalysis;

namespace Occt;

/// <summary>
/// An interference of an edge with a face for the hidden-line removal: the intersection, the coincidence with the
/// boundary, and the orientation, transition and boundary transition (OCCT's instantiation of TopBas_Interference).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (OCCT's default constructor leaves
/// the orientations unset). The setters (OCCT's overloads of the getters' names) and the <c>Change</c> accessors
/// (references to the stored parts) are internal.
/// </remarks>
public partial struct HLRAlgo_Interference : IEquatable<HLRAlgo_Interference>
{
    private HLRAlgo_Intersection myIntersection;
    private HLRAlgo_Coincidence myBoundary;
    private Orientation myOrientation;
    private Orientation myTransition;
    private Orientation myBTransition;

    public HLRAlgo_Interference()
    {
        myIntersection = new HLRAlgo_Intersection();
        myBoundary = new HLRAlgo_Coincidence();
    }

    public HLRAlgo_Interference(in HLRAlgo_Intersection Inters, in HLRAlgo_Coincidence Bound,
                                Orientation Orient, Orientation Trans, Orientation BTrans)
    {
        myIntersection = Inters;
        myBoundary = Bound;
        myOrientation = Orient;
        myTransition = Trans;
        myBTransition = BTrans;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Intersection(in HLRAlgo_Intersection I) => myIntersection = I;

    internal void Boundary(in HLRAlgo_Coincidence B) => myBoundary = B;

    internal void Orientation(Orientation Or) => myOrientation = Or;

    internal void Transition(Orientation Or) => myTransition = Or;

    internal void BoundaryTransition(Orientation Or) => myBTransition = Or;

    [UnscopedRef]
    internal ref HLRAlgo_Intersection ChangeIntersection() => ref myIntersection;

    [UnscopedRef]
    internal ref HLRAlgo_Coincidence ChangeBoundary() => ref myBoundary;

    // Queries

    public readonly HLRAlgo_Intersection Intersection() => myIntersection;

    public readonly HLRAlgo_Coincidence Boundary() => myBoundary;

    public readonly Orientation Orientation() => myOrientation;

    public readonly Orientation Transition() => myTransition;

    public readonly Orientation BoundaryTransition() => myBTransition;

    public readonly HLRAlgo_Interference WithIntersection(in HLRAlgo_Intersection I)
    {
        HLRAlgo_Interference aCopy = this;
        aCopy.Intersection(I);
        return aCopy;
    }

    public readonly HLRAlgo_Interference WithBoundary(in HLRAlgo_Coincidence B)
    {
        HLRAlgo_Interference aCopy = this;
        aCopy.Boundary(B);
        return aCopy;
    }

    public readonly HLRAlgo_Interference WithOrientation(Orientation Or)
    {
        HLRAlgo_Interference aCopy = this;
        aCopy.Orientation(Or);
        return aCopy;
    }

    public readonly HLRAlgo_Interference WithTransition(Orientation Or)
    {
        HLRAlgo_Interference aCopy = this;
        aCopy.Transition(Or);
        return aCopy;
    }

    public readonly HLRAlgo_Interference WithBoundaryTransition(Orientation Or)
    {
        HLRAlgo_Interference aCopy = this;
        aCopy.BoundaryTransition(Or);
        return aCopy;
    }

    public readonly bool Equals(HLRAlgo_Interference other)
        => myIntersection.Equals(other.myIntersection) && myBoundary.Equals(other.myBoundary)
           && myOrientation == other.myOrientation && myTransition == other.myTransition
           && myBTransition == other.myBTransition;

    public override readonly bool Equals(object? obj) => obj is HLRAlgo_Interference other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myIntersection, myBoundary, myOrientation, myTransition, myBTransition);

    public static bool operator ==(in HLRAlgo_Interference theLeft, in HLRAlgo_Interference theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in HLRAlgo_Interference theLeft, in HLRAlgo_Interference theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"HLRAlgo_Interference({myIntersection}, {myBoundary}, {myOrientation}, {myTransition}, {myBTransition})";
}
