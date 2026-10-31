// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedCirc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// A circle with a qualifier: whether the solutions computed from it enclose it (GccEnt_enclosing), are enclosed by
/// it (GccEnt_enclosed), are outside it (GccEnt_outside), or may be any of these (GccEnt_unqualified).
/// </summary>
/// <remarks>A value: OCCT gives it no setters. <c>default</c> is an unqualified circle of radius 0 at the origin.</remarks>
public readonly struct GccEnt_QualifiedCirc : IEquatable<GccEnt_QualifiedCirc>
{
    private readonly gp_Circ2d TheQualified;
    private readonly GccEnt_Position TheQualifier;

    /// <summary>Assigns the qualifier to the circle.</summary>
    public GccEnt_QualifiedCirc(in gp_Circ2d Qualified, GccEnt_Position Qualifier)
    {
        TheQualified = Qualified;
        TheQualifier = Qualifier;
    }

    /// <summary>The circle to which the qualifier is assigned.</summary>
    public gp_Circ2d Qualified()
    {
        return TheQualified;
    }

    /// <summary>The qualifier.</summary>
    public GccEnt_Position Qualifier()
    {
        return TheQualifier;
    }

    public bool IsUnqualified()
    {
        return TheQualifier == GccEnt_unqualified;
    }

    /// <summary>True if the solutions enclose the circle.</summary>
    public bool IsEnclosing()
    {
        return TheQualifier == GccEnt_enclosing;
    }

    /// <summary>True if the circle encloses the solutions.</summary>
    public bool IsEnclosed()
    {
        return TheQualifier == GccEnt_enclosed;
    }

    /// <summary>True if the solutions and the circle are outside each other.</summary>
    public bool IsOutside()
    {
        return TheQualifier == GccEnt_outside;
    }

    public bool Equals(GccEnt_QualifiedCirc other)
        => TheQualified.Equals(other.TheQualified) && TheQualifier == other.TheQualifier;

    public override bool Equals(object? obj) => obj is GccEnt_QualifiedCirc other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(TheQualified, TheQualifier);

    public static bool operator ==(in GccEnt_QualifiedCirc theLeft, in GccEnt_QualifiedCirc theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in GccEnt_QualifiedCirc theLeft, in GccEnt_QualifiedCirc theRight) => !theLeft.Equals(theRight);

    public override string ToString()
        => "GccEnt_QualifiedCirc(" + TheQualified.ToString() + ", " + GccEnt.PositionToString(TheQualifier) + ")";
}
