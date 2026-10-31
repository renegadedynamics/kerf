// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedLin.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedLin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// A line with a qualifier: whether the solutions computed from it lie on its left, its inside (GccEnt_enclosed), on
/// its right (GccEnt_outside), or on either side (GccEnt_unqualified).
/// </summary>
/// <remarks>A value: OCCT gives it no setters. <c>default</c> is an unqualified line along X through the origin.</remarks>
public readonly struct GccEnt_QualifiedLin : IEquatable<GccEnt_QualifiedLin>
{
    private readonly GccEnt_Position TheQualifier;
    private readonly gp_Lin2d TheQualified;

    /// <summary>Assigns the qualifier to the line.</summary>
    public GccEnt_QualifiedLin(in gp_Lin2d Qualified, GccEnt_Position Qualifier)
    {
        TheQualified = Qualified;
        TheQualifier = Qualifier;
    }

    /// <summary>The line to which the qualifier is assigned.</summary>
    public gp_Lin2d Qualified()
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

    /// <summary>True if the solutions are on the left of the line (its inside).</summary>
    public bool IsEnclosed()
    {
        return (TheQualifier == GccEnt_enclosed);
    }

    /// <summary>True if the solutions are on the right of the line (its outside).</summary>
    public bool IsOutside()
    {
        return TheQualifier == GccEnt_outside;
    }

    public bool Equals(GccEnt_QualifiedLin other)
        => TheQualified.Equals(other.TheQualified) && TheQualifier == other.TheQualifier;

    public override bool Equals(object? obj) => obj is GccEnt_QualifiedLin other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(TheQualified, TheQualifier);

    public static bool operator ==(in GccEnt_QualifiedLin theLeft, in GccEnt_QualifiedLin theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in GccEnt_QualifiedLin theLeft, in GccEnt_QualifiedLin theRight) => !theLeft.Equals(theRight);

    public override string ToString()
        => "GccEnt_QualifiedLin(" + TheQualified.ToString() + ", " + GccEnt.PositionToString(TheQualifier) + ")";
}
