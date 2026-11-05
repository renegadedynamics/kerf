// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Transition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Transition.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// The type of transition near an intersection point between two curves: a "true transition" (one curve goes
/// inside or outside the area defined by the other), a "touch transition" (the first curve does not cross the
/// other), or an "undecided" transition (the curves are superposed).
/// </summary>
/// <remarks>
/// <c>new IntRes2d_Transition()</c> is OCCT's default (tangent, Middle, Undecided, Unknown, not opposite);
/// <c>default(IntRes2d_Transition)</c> is all zeros (not tangent, Head, In, Inside). The <c>SetValue</c> and
/// <c>SetPosition</c> mutators are internal; the constructors and <see cref="WithPosition"/> are their twins.
/// OCCT's <c>operator&lt;&lt;</c> (French text) is left out; <see cref="ToString"/> replaces it.
/// </remarks>
internal partial struct Transition : IEquatable<Transition>
{
    private bool _tangent;
    private Position2D _posit;
    private TypeTrans _typetra;
    private Situation _situat;
    private bool _oppos;

    /// <summary>Empty constructor: an undecided transition in the middle of the curve.</summary>
    public Transition()
    {
        _tangent = true;
        _posit = Position2D.Middle;
        _typetra = TypeTrans.Undecided;
        _situat = global::Kerf.Engine.Intersection.TwoD.Situation.Unknown;
        _oppos = false;
    }

    /// <summary>Creates an IN or OUT transition.</summary>
    public Transition(bool tangent, Position2D pos, TypeTrans type)
    {
        _tangent = tangent;
        _posit = pos;
        _typetra = type;
        _situat = global::Kerf.Engine.Intersection.TwoD.Situation.Unknown;
        _oppos = false;
    }

    /// <summary>Creates a TOUCH transition.</summary>
    public Transition(bool tangent, Position2D pos, Situation situ, bool oppos)
    {
        _tangent = tangent;
        _posit = pos;
        _typetra = TypeTrans.Touch;
        _situat = situ;
        _oppos = oppos;
    }

    /// <summary>Creates an UNDECIDED transition.</summary>
    public Transition(Position2D pos)
    {
        _tangent = true;
        _posit = pos;
        _typetra = TypeTrans.Undecided;
        _situat = global::Kerf.Engine.Intersection.TwoD.Situation.Unknown;
        _oppos = false;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values of an IN or OUT transition (the situation and opposite flag are kept).</summary>
    internal void SetValue(bool tangent, Position2D pos, TypeTrans type)
    {
        _tangent = tangent;
        _posit = pos;
        _typetra = type;
    }

    /// <summary>Sets the values of a TOUCH transition.</summary>
    internal void SetValue(bool tangent, Position2D pos, Situation situ, bool oppos)
    {
        _tangent = tangent;
        _posit = pos;
        _typetra = TypeTrans.Touch;
        _situat = situ;
        _oppos = oppos;
    }

    /// <summary>Sets the values of an UNDECIDED transition (the other fields are kept).</summary>
    internal void SetValue(Position2D pos)
    {
        _posit = pos;
        _typetra = TypeTrans.Undecided;
    }

    /// <summary>Sets the value of the position.</summary>
    internal void SetPosition(Position2D pos)
    {
        _posit = pos;
    }

    // Queries

    /// <summary>
    /// Whether the intersection is at the beginning (Head), at the end (End) or in the middle (Middle) of the curve.
    /// </summary>
    public readonly Position2D PositionOnCurve() => _posit;

    /// <summary>
    /// The type of transition: IN, OUT, TOUCH, or UNDECIDED if the two first derivatives are not enough to give
    /// the tangent to one of the two curves.
    /// </summary>
    public readonly TypeTrans TransitionType() => _typetra;

    /// <summary>True when the two curves are tangent at the intersection point.</summary>
    /// <exception cref="DomainErrorException">The transition is UNDECIDED.</exception>
    public readonly bool IsTangent
    {
        get
        {
            if (_typetra == TypeTrans.Undecided)
            {
                throw new DomainErrorException();
            }

            return _tangent;
        }
    }

    /// <summary>
    /// For a TOUCH transition: INSIDE when the curve remains inside the other one, OUTSIDE when it remains outside,
    /// UNKNOWN when the second derivatives cannot tell.
    /// </summary>
    /// <exception cref="DomainErrorException">The transition is not TOUCH.</exception>
    public readonly Situation Situation()
    {
        if (_typetra != TypeTrans.Touch)
        {
            throw new DomainErrorException();
        }

        return _situat;
    }

    /// <summary>For a TOUCH transition: true when the two curves locally define two different parts of the space.</summary>
    /// <exception cref="DomainErrorException">The transition is not TOUCH.</exception>
    public readonly bool IsOpposite
    {
        get
        {
            if (_typetra != TypeTrans.Touch)
            {
                throw new DomainErrorException();
            }

            return _oppos;
        }
    }

    // Non-mutating twins

    /// <summary>A copy with the position changed (the twin of SetPosition).</summary>
    public readonly Transition WithPosition(Position2D pos)
    {
        Transition copy = this;
        copy.SetPosition(pos);
        return copy;
    }

    public readonly bool Equals(Transition other)
        => _tangent == other._tangent && _posit == other._posit && _typetra == other._typetra && _situat == other._situat
           && _oppos == other._oppos;

    public override readonly bool Equals(object? obj) => obj is Transition other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_tangent, _posit, _typetra, _situat, _oppos);

    public static bool operator ==(in Transition left, in Transition right) => left.Equals(right);

    public static bool operator !=(in Transition left, in Transition right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntRes2d_Transition({_posit}, {_typetra}, {_situat}, tangent: {_tangent}, opposite: {_oppos})";
}
