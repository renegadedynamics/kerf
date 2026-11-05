// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Transition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Transition.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// The transition at the intersection between an intersection line and a restriction curve on a surface: in, out,
/// touch (with the situation and whether the curves are opposite) or undecided, and whether it is tangent.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new()</c> is OCCT's empty constructor (undecided); <c>default</c> is an in
/// transition (zero), so ported code writes <c>new()</c>. The <c>SetValue</c> overloads are internal mutators; the
/// constructors are their twins.
/// </remarks>
internal partial struct WalkingTransition : IEquatable<WalkingTransition>
{
    private bool _tangent;
    private WalkingTypeTrans _typetra;
    private WalkingSituation _situat;
    private bool _oppos;

    /// <summary>An undecided transition.</summary>
    public WalkingTransition()
    {
        _tangent = false;
        _typetra = WalkingTypeTrans.Undecided;
        _situat = WalkingSituation.Unknown;
        _oppos = false;
    }

    /// <summary>An in or out transition.</summary>
    public WalkingTransition(bool tangent, WalkingTypeTrans type)
    {
        _tangent = tangent;
        _typetra = type;
        _situat = WalkingSituation.Unknown;
        _oppos = false;
    }

    /// <summary>A touch transition.</summary>
    public WalkingTransition(bool tangent, WalkingSituation situ, bool oppos)
    {
        _tangent = tangent;
        _typetra = WalkingTypeTrans.Touch;
        _situat = situ;
        _oppos = oppos;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values of an in or out transition.</summary>
    internal void SetValue(bool tangent, WalkingTypeTrans type)
    {
        _tangent = tangent;
        _typetra = type;
    }

    /// <summary>Sets the values of a touch transition.</summary>
    internal void SetValue(bool tangent, WalkingSituation situ, bool oppos)
    {
        _tangent = tangent;
        _typetra = WalkingTypeTrans.Touch;
        _situat = situ;
        _oppos = oppos;
    }

    /// <summary>Sets an undecided transition (the other fields are kept, as in OCCT).</summary>
    internal void SetValue() => _typetra = WalkingTypeTrans.Undecided;

    // Queries

    /// <summary>
    /// The type of the transition of the intersection line compared to the arc of restriction (in: the line goes
    /// into the part of the plane limited by the arc).
    /// </summary>
    public readonly WalkingTypeTrans TransitionType() => _typetra;

    /// <summary>True if the point is tangent to the arc. Throws Standard_DomainError if undecided.</summary>
    public readonly bool IsTangent
    {
        get
        {
            if (_typetra == WalkingTypeTrans.Undecided)
            {
                throw new DomainErrorException();
            }
            return _tangent;
        }
    }

    /// <summary>
    /// For a touch transition, whether the line stays inside or outside the arc, or unknown. Throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public readonly WalkingSituation Situation()
    {
        if (_typetra != WalkingTypeTrans.Touch)
        {
            throw new DomainErrorException();
        }
        return _situat;
    }

    /// <summary>
    /// For a touch transition, true when the two curves locally bound two different parts of the space. Throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public readonly bool IsOpposite
    {
        get
        {
            if (_typetra != WalkingTypeTrans.Touch)
            {
                throw new DomainErrorException();
            }
            return _oppos;
        }
    }

    public readonly bool Equals(WalkingTransition other)
        => _tangent == other._tangent && _typetra == other._typetra && _situat == other._situat && _oppos == other._oppos;

    public override readonly bool Equals(object? obj) => obj is WalkingTransition other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_tangent, _typetra, _situat, _oppos);

    public static bool operator ==(in WalkingTransition left, in WalkingTransition right) => left.Equals(right);

    public static bool operator !=(in WalkingTransition left, in WalkingTransition right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntSurf_Transition({_typetra}, tangent {_tangent}, {_situat}, opposite {_oppos})";
}
