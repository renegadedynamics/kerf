// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Transition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Transition.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;

namespace Occt;

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
public partial struct IntRes2d_Transition : IEquatable<IntRes2d_Transition>
{
    private bool tangent;
    private IntRes2d_Position posit;
    private IntRes2d_TypeTrans typetra;
    private IntRes2d_Situation situat;
    private bool oppos;

    /// <summary>Empty constructor: an undecided transition in the middle of the curve.</summary>
    public IntRes2d_Transition()
    {
        tangent = true;
        posit = IntRes2d_Middle;
        typetra = IntRes2d_Undecided;
        situat = IntRes2d_Unknown;
        oppos = false;
    }

    /// <summary>Creates an IN or OUT transition.</summary>
    public IntRes2d_Transition(bool Tangent, IntRes2d_Position Pos, IntRes2d_TypeTrans Type)
    {
        tangent = Tangent;
        posit = Pos;
        typetra = Type;
        situat = IntRes2d_Unknown;
        oppos = false;
    }

    /// <summary>Creates a TOUCH transition.</summary>
    public IntRes2d_Transition(bool Tangent, IntRes2d_Position Pos, IntRes2d_Situation Situ, bool Oppos)
    {
        tangent = Tangent;
        posit = Pos;
        typetra = IntRes2d_Touch;
        situat = Situ;
        oppos = Oppos;
    }

    /// <summary>Creates an UNDECIDED transition.</summary>
    public IntRes2d_Transition(IntRes2d_Position Pos)
    {
        tangent = true;
        posit = Pos;
        typetra = IntRes2d_Undecided;
        situat = IntRes2d_Unknown;
        oppos = false;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values of an IN or OUT transition (the situation and opposite flag are kept).</summary>
    internal void SetValue(bool Tangent, IntRes2d_Position Pos, IntRes2d_TypeTrans Type)
    {
        tangent = Tangent;
        posit = Pos;
        typetra = Type;
    }

    /// <summary>Sets the values of a TOUCH transition.</summary>
    internal void SetValue(bool Tangent, IntRes2d_Position Pos, IntRes2d_Situation Situ, bool Oppos)
    {
        tangent = Tangent;
        posit = Pos;
        typetra = IntRes2d_Touch;
        situat = Situ;
        oppos = Oppos;
    }

    /// <summary>Sets the values of an UNDECIDED transition (the other fields are kept).</summary>
    internal void SetValue(IntRes2d_Position Pos)
    {
        posit = Pos;
        typetra = IntRes2d_Undecided;
    }

    /// <summary>Sets the value of the position.</summary>
    internal void SetPosition(IntRes2d_Position Pos)
    {
        posit = Pos;
    }

    // Queries

    /// <summary>
    /// Whether the intersection is at the beginning (Head), at the end (End) or in the middle (Middle) of the curve.
    /// </summary>
    public readonly IntRes2d_Position PositionOnCurve() => posit;

    /// <summary>
    /// The type of transition: IN, OUT, TOUCH, or UNDECIDED if the two first derivatives are not enough to give
    /// the tangent to one of the two curves.
    /// </summary>
    public readonly IntRes2d_TypeTrans TransitionType() => typetra;

    /// <summary>True when the two curves are tangent at the intersection point.</summary>
    /// <exception cref="DomainErrorException">The transition is UNDECIDED.</exception>
    public readonly bool IsTangent()
    {
        if (typetra == IntRes2d_Undecided)
        {
            throw new DomainErrorException();
        }

        return tangent;
    }

    /// <summary>
    /// For a TOUCH transition: INSIDE when the curve remains inside the other one, OUTSIDE when it remains outside,
    /// UNKNOWN when the second derivatives cannot tell.
    /// </summary>
    /// <exception cref="DomainErrorException">The transition is not TOUCH.</exception>
    public readonly IntRes2d_Situation Situation()
    {
        if (typetra != IntRes2d_Touch)
        {
            throw new DomainErrorException();
        }

        return situat;
    }

    /// <summary>For a TOUCH transition: true when the two curves locally define two different parts of the space.</summary>
    /// <exception cref="DomainErrorException">The transition is not TOUCH.</exception>
    public readonly bool IsOpposite()
    {
        if (typetra != IntRes2d_Touch)
        {
            throw new DomainErrorException();
        }

        return oppos;
    }

    // Non-mutating twins

    /// <summary>A copy with the position changed (the twin of SetPosition).</summary>
    public readonly IntRes2d_Transition WithPosition(IntRes2d_Position Pos)
    {
        IntRes2d_Transition aCopy = this;
        aCopy.SetPosition(Pos);
        return aCopy;
    }

    public readonly bool Equals(IntRes2d_Transition other)
        => tangent == other.tangent && posit == other.posit && typetra == other.typetra && situat == other.situat
           && oppos == other.oppos;

    public override readonly bool Equals(object? obj) => obj is IntRes2d_Transition other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(tangent, posit, typetra, situat, oppos);

    public static bool operator ==(in IntRes2d_Transition theLeft, in IntRes2d_Transition theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntRes2d_Transition theLeft, in IntRes2d_Transition theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntRes2d_Transition({posit}, {typetra}, {situat}, tangent: {tangent}, opposite: {oppos})";
}
