// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Transition.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Transition.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Transition.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// The transition at the intersection between an intersection line and a restriction curve on a surface: in, out,
/// touch (with the situation and whether the curves are opposite) or undecided, and whether it is tangent.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new()</c> is OCCT's empty constructor (undecided); <c>default</c> is an in
/// transition (zero), so ported code writes <c>new()</c>. The <c>SetValue</c> overloads are internal mutators; the
/// constructors are their twins.
/// </remarks>
public partial struct IntSurf_Transition : IEquatable<IntSurf_Transition>
{
    private bool tangent;
    private IntSurf_TypeTrans typetra;
    private IntSurf_Situation situat;
    private bool oppos;

    /// <summary>An undecided transition.</summary>
    public IntSurf_Transition()
    {
        tangent = false;
        typetra = IntSurf_Undecided;
        situat = IntSurf_Unknown;
        oppos = false;
    }

    /// <summary>An in or out transition.</summary>
    public IntSurf_Transition(bool Tangent, IntSurf_TypeTrans Type)
    {
        tangent = Tangent;
        typetra = Type;
        situat = IntSurf_Unknown;
        oppos = false;
    }

    /// <summary>A touch transition.</summary>
    public IntSurf_Transition(bool Tangent, IntSurf_Situation Situ, bool Oppos)
    {
        tangent = Tangent;
        typetra = IntSurf_Touch;
        situat = Situ;
        oppos = Oppos;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values of an in or out transition.</summary>
    internal void SetValue(bool Tangent, IntSurf_TypeTrans Type)
    {
        tangent = Tangent;
        typetra = Type;
    }

    /// <summary>Sets the values of a touch transition.</summary>
    internal void SetValue(bool Tangent, IntSurf_Situation Situ, bool Oppos)
    {
        tangent = Tangent;
        typetra = IntSurf_Touch;
        situat = Situ;
        oppos = Oppos;
    }

    /// <summary>Sets an undecided transition (the other fields are kept, as in OCCT).</summary>
    internal void SetValue() => typetra = IntSurf_Undecided;

    // Queries

    /// <summary>
    /// The type of the transition of the intersection line compared to the arc of restriction (in: the line goes
    /// into the part of the plane limited by the arc).
    /// </summary>
    public readonly IntSurf_TypeTrans TransitionType() => typetra;

    /// <summary>True if the point is tangent to the arc. Throws Standard_DomainError if undecided.</summary>
    public readonly bool IsTangent()
    {
        if (typetra == IntSurf_Undecided)
        {
            throw new Standard_DomainError();
        }
        return tangent;
    }

    /// <summary>
    /// For a touch transition, whether the line stays inside or outside the arc, or unknown. Throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public readonly IntSurf_Situation Situation()
    {
        if (typetra != IntSurf_Touch)
        {
            throw new Standard_DomainError();
        }
        return situat;
    }

    /// <summary>
    /// For a touch transition, true when the two curves locally bound two different parts of the space. Throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public readonly bool IsOpposite()
    {
        if (typetra != IntSurf_Touch)
        {
            throw new Standard_DomainError();
        }
        return oppos;
    }

    public readonly bool Equals(IntSurf_Transition other)
        => tangent == other.tangent && typetra == other.typetra && situat == other.situat && oppos == other.oppos;

    public override readonly bool Equals(object? obj) => obj is IntSurf_Transition other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(tangent, typetra, situat, oppos);

    public static bool operator ==(in IntSurf_Transition theLeft, in IntSurf_Transition theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntSurf_Transition theLeft, in IntSurf_Transition theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntSurf_Transition({typetra}, tangent {tangent}, {situat}, opposite {oppos})";
}
