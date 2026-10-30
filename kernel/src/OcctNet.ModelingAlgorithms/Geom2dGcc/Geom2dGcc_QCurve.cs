// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_QCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_QCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// A qualified curve as the Geo and Iter solvers take it: an adapted curve and its qualifier. The same as
/// Geom2dGcc_QualifiedCurve, which OCCT keeps for the public constructors.
/// </summary>
/// <remarks>A value; the adaptor is held and returned as shallow copies, as Geom2dGcc_QualifiedCurve (decision 5971).</remarks>
public readonly struct Geom2dGcc_QCurve : IEquatable<Geom2dGcc_QCurve>
{
    private readonly GccEnt_Position TheQualifier;
    private readonly Geom2dAdaptor_Curve TheQualified;

    /// <summary>Assigns the qualifier to the curve (copied).</summary>
    public Geom2dGcc_QCurve(Geom2dAdaptor_Curve Curve, GccEnt_Position Qualifier)
    {
        TheQualified = (Geom2dAdaptor_Curve)Curve.ShallowCopy();
        TheQualifier = Qualifier;
    }

    /// <summary>The curve to which the qualifier is assigned: a new adaptor on the same curve and range.</summary>
    public Geom2dAdaptor_Curve Qualified()
    {
        return (Geom2dAdaptor_Curve)TheQualified.ShallowCopy();
    }

    /// <summary>The qualifier.</summary>
    public GccEnt_Position Qualifier()
    {
        return TheQualifier;
    }

    /// <summary>True if the solutions are unqualified.</summary>
    public bool IsUnqualified()
    {
        return TheQualifier == GccEnt_unqualified;
    }

    /// <summary>True if the solutions enclose the curve.</summary>
    public bool IsEnclosing()
    {
        return TheQualifier == GccEnt_enclosing;
    }

    /// <summary>True if the curve encloses the solutions.</summary>
    public bool IsEnclosed()
    {
        return TheQualifier == GccEnt_enclosed;
    }

    /// <summary>True if the solutions and the curve are outside each other.</summary>
    public bool IsOutside()
    {
        return TheQualifier == GccEnt_outside;
    }

    public bool Equals(Geom2dGcc_QCurve other)
    {
        if (TheQualifier != other.TheQualifier)
        {
            return false;
        }
        if (TheQualified is null || other.TheQualified is null)
        {
            return TheQualified is null && other.TheQualified is null;
        }
        return ReferenceEquals(TheQualified.Curve(), other.TheQualified.Curve())
               && TheQualified.FirstParameter().Equals(other.TheQualified.FirstParameter())
               && TheQualified.LastParameter().Equals(other.TheQualified.LastParameter());
    }

    public override bool Equals(object? obj) => obj is Geom2dGcc_QCurve other && Equals(other);

    public override int GetHashCode()
        => TheQualified is null
               ? HashCode.Combine(TheQualifier)
               : HashCode.Combine(TheQualifier, TheQualified.Curve(), TheQualified.FirstParameter(), TheQualified.LastParameter());

    public static bool operator ==(in Geom2dGcc_QCurve theLeft, in Geom2dGcc_QCurve theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Geom2dGcc_QCurve theLeft, in Geom2dGcc_QCurve theRight) => !theLeft.Equals(theRight);

    public override string ToString()
        => "Geom2dGcc_QCurve(" + (TheQualified is null
                                       ? "null"
                                       : TheQualified.GetType().ToString() + " "
                                         + gp.Format("range", TheQualified.FirstParameter(), TheQualified.LastParameter()))
           + ", " + GccEnt.PositionToString(TheQualifier) + ")";
}
