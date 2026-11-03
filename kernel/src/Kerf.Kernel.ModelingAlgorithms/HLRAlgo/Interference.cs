// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Interference.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_Interference_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using System.Diagnostics.CodeAnalysis;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>
/// An interference of an edge with a face for the hidden-line removal: the intersection, the coincidence with the
/// boundary, and the orientation, transition and boundary transition (OCCT's instantiation of TopBas_Interference).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same (OCCT's default constructor leaves
/// the orientations unset). The setters (OCCT's overloads of the getters' names) and the <c>Change</c> accessors
/// (references to the stored parts) are internal.
/// </remarks>
public partial struct Interference : IEquatable<Interference>
{
    private AlgoIntersection _intersection;
    private Coincidence _boundary;
    private Orientation _orientation;
    private Orientation _transition;
    private Orientation _bTransition;

    public Interference()
    {
        _intersection = new AlgoIntersection();
        _boundary = new Coincidence();
    }

    public Interference(in AlgoIntersection inters, in Coincidence bound,
                                Orientation orient, Orientation trans, Orientation bTrans)
    {
        _intersection = inters;
        _boundary = bound;
        _orientation = orient;
        _transition = trans;
        _bTransition = bTrans;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Intersection(in AlgoIntersection i) => _intersection = i;

    internal void Boundary(in Coincidence b) => _boundary = b;

    internal void Orientation(Orientation or) => _orientation = or;

    internal void Transition(Orientation or) => _transition = or;

    internal void BoundaryTransition(Orientation or) => _bTransition = or;

    [UnscopedRef]
    internal ref AlgoIntersection ChangeIntersection() => ref _intersection;

    [UnscopedRef]
    internal ref Coincidence ChangeBoundary() => ref _boundary;

    // Queries

    public readonly AlgoIntersection Intersection() => _intersection;

    public readonly Coincidence Boundary() => _boundary;

    public readonly Orientation Orientation() => _orientation;

    public readonly Orientation Transition() => _transition;

    public readonly Orientation BoundaryTransition() => _bTransition;

    public readonly Interference WithIntersection(in AlgoIntersection i)
    {
        Interference copy = this;
        copy.Intersection(i);
        return copy;
    }

    public readonly Interference WithBoundary(in Coincidence b)
    {
        Interference copy = this;
        copy.Boundary(b);
        return copy;
    }

    public readonly Interference WithOrientation(Orientation or)
    {
        Interference copy = this;
        copy.Orientation(or);
        return copy;
    }

    public readonly Interference WithTransition(Orientation or)
    {
        Interference copy = this;
        copy.Transition(or);
        return copy;
    }

    public readonly Interference WithBoundaryTransition(Orientation or)
    {
        Interference copy = this;
        copy.BoundaryTransition(or);
        return copy;
    }

    public readonly bool Equals(Interference other)
        => _intersection.Equals(other._intersection) && _boundary.Equals(other._boundary)
           && _orientation == other._orientation && _transition == other._transition
           && _bTransition == other._bTransition;

    public override readonly bool Equals(object? obj) => obj is Interference other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_intersection, _boundary, _orientation, _transition, _bTransition);

    public static bool operator ==(in Interference left, in Interference right)
        => left.Equals(right);

    public static bool operator !=(in Interference left, in Interference right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"HLRAlgo_Interference({_intersection}, {_boundary}, {_orientation}, {_transition}, {_bTransition})";
}
