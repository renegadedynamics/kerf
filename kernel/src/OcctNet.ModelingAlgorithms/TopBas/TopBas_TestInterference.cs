// Port of: src/ModelingAlgorithms/TKHLR/TopBas/TopBas_TestInterference.hxx
// Port of: src/ModelingAlgorithms/TKHLR/TopBas/TopBas_TestInterference_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>
/// An interference for tests: an intersection parameter, a boundary index, and the orientation, transition and
/// boundary transition (OCCT's instantiation of TopBas_Interference over a real and an integer).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (OCCT's default constructor leaves
/// the fields unset). The setters (OCCT's overloads of the getters' names) and the <c>Change</c> accessors are
/// internal.
/// </remarks>
public partial struct TopBas_TestInterference : IEquatable<TopBas_TestInterference>
{
    private double myIntersection;
    private int myBoundary;
    private TopAbs_Orientation myOrientation;
    private TopAbs_Orientation myTransition;
    private TopAbs_Orientation myBTransition;

    public TopBas_TestInterference(double Inters, int Bound, TopAbs_Orientation Orient, TopAbs_Orientation Trans,
                                   TopAbs_Orientation BTrans)
    {
        myIntersection = Inters;
        myBoundary = Bound;
        myOrientation = Orient;
        myTransition = Trans;
        myBTransition = BTrans;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Intersection(double I) => myIntersection = I;

    internal void Boundary(int B) => myBoundary = B;

    internal void Orientation(TopAbs_Orientation Or) => myOrientation = Or;

    internal void Transition(TopAbs_Orientation Or) => myTransition = Or;

    internal void BoundaryTransition(TopAbs_Orientation Or) => myBTransition = Or;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref double ChangeIntersection() => ref myIntersection;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref int ChangeBoundary() => ref myBoundary;

    // Queries

    public readonly double Intersection() => myIntersection;

    public readonly int Boundary() => myBoundary;

    public readonly TopAbs_Orientation Orientation() => myOrientation;

    public readonly TopAbs_Orientation Transition() => myTransition;

    public readonly TopAbs_Orientation BoundaryTransition() => myBTransition;

    public readonly TopBas_TestInterference WithIntersection(double I)
    {
        TopBas_TestInterference aCopy = this;
        aCopy.Intersection(I);
        return aCopy;
    }

    public readonly TopBas_TestInterference WithBoundary(int B)
    {
        TopBas_TestInterference aCopy = this;
        aCopy.Boundary(B);
        return aCopy;
    }

    public readonly TopBas_TestInterference WithOrientation(TopAbs_Orientation Or)
    {
        TopBas_TestInterference aCopy = this;
        aCopy.Orientation(Or);
        return aCopy;
    }

    public readonly TopBas_TestInterference WithTransition(TopAbs_Orientation Or)
    {
        TopBas_TestInterference aCopy = this;
        aCopy.Transition(Or);
        return aCopy;
    }

    public readonly TopBas_TestInterference WithBoundaryTransition(TopAbs_Orientation Or)
    {
        TopBas_TestInterference aCopy = this;
        aCopy.BoundaryTransition(Or);
        return aCopy;
    }

    public readonly bool Equals(TopBas_TestInterference other)
        => myIntersection.Equals(other.myIntersection) && myBoundary == other.myBoundary
           && myOrientation == other.myOrientation && myTransition == other.myTransition
           && myBTransition == other.myBTransition;

    public override readonly bool Equals(object? obj) => obj is TopBas_TestInterference other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myIntersection, myBoundary, myOrientation, myTransition, myBTransition);

    public static bool operator ==(in TopBas_TestInterference theLeft, in TopBas_TestInterference theRight)
        => theLeft.Equals(theRight);

    public static bool operator !=(in TopBas_TestInterference theLeft, in TopBas_TestInterference theRight)
        => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{Constants.Format("TopBas_TestInterference", myIntersection)} {myBoundary} {myOrientation} {myTransition} {myBTransition}";
}
