// Port of: src/ModelingAlgorithms/TKHLR/TopBas/TopBas_TestInterference.hxx
// Port of: src/ModelingAlgorithms/TKHLR/TopBas/TopBas_TestInterference_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy;

/// <summary>
/// An interference for tests: an intersection parameter, a boundary index, and the orientation, transition and
/// boundary transition (OCCT's instantiation of TopBas_Interference over a real and an integer).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (OCCT's default constructor leaves
/// the fields unset). The setters (OCCT's overloads of the getters' names) and the <c>Change</c> accessors are
/// internal.
/// </remarks>
internal partial struct TestInterference : IEquatable<TestInterference>
{
    private double _intersection;
    private int _boundary;
    private Orientation _orientation;
    private Orientation _transition;
    private Orientation _bTransition;

    public TestInterference(double inters, int bound, Orientation orient, Orientation trans,
                                   Orientation bTrans)
    {
        _intersection = inters;
        _boundary = bound;
        _orientation = orient;
        _transition = trans;
        _bTransition = bTrans;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Intersection(double i) => _intersection = i;

    internal void Boundary(int b) => _boundary = b;

    internal void Orientation(Orientation or) => _orientation = or;

    internal void Transition(Orientation or) => _transition = or;

    internal void BoundaryTransition(Orientation or) => _bTransition = or;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref double ChangeIntersection() => ref _intersection;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref int ChangeBoundary() => ref _boundary;

    // Queries

    public readonly double Intersection() => _intersection;

    public readonly int Boundary() => _boundary;

    public readonly Orientation Orientation() => _orientation;

    public readonly Orientation Transition() => _transition;

    public readonly Orientation BoundaryTransition() => _bTransition;

    public readonly TestInterference WithIntersection(double i)
    {
        TestInterference copy = this;
        copy.Intersection(i);
        return copy;
    }

    public readonly TestInterference WithBoundary(int b)
    {
        TestInterference copy = this;
        copy.Boundary(b);
        return copy;
    }

    public readonly TestInterference WithOrientation(Orientation or)
    {
        TestInterference copy = this;
        copy.Orientation(or);
        return copy;
    }

    public readonly TestInterference WithTransition(Orientation or)
    {
        TestInterference copy = this;
        copy.Transition(or);
        return copy;
    }

    public readonly TestInterference WithBoundaryTransition(Orientation or)
    {
        TestInterference copy = this;
        copy.BoundaryTransition(or);
        return copy;
    }

    public readonly bool Equals(TestInterference other)
        => _intersection.Equals(other._intersection) && _boundary == other._boundary
           && _orientation == other._orientation && _transition == other._transition
           && _bTransition == other._bTransition;

    public override readonly bool Equals(object? obj) => obj is TestInterference other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_intersection, _boundary, _orientation, _transition, _bTransition);

    public static bool operator ==(in TestInterference left, in TestInterference right)
        => left.Equals(right);

    public static bool operator !=(in TestInterference left, in TestInterference right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("TopBas_TestInterference", _intersection)} {_boundary} {_orientation} {_transition} {_bTransition}";
}
