// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_QualifiedCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_QualifiedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// A 2D curve with a qualifier: whether the solutions computed from it enclose it (GccEnt_enclosing), are enclosed
/// by it (GccEnt_enclosed), are outside it (GccEnt_outside), or may be any of these (GccEnt_unqualified). The interior
/// of a curve is the left-hand side of the curve in relation to its orientation.
/// </summary>
/// <remarks>
/// A value, as OCCT uses it (no setters). OCCT stores a copy of the adaptor and returns copies: the port keeps a
/// shallow copy (<see cref="Adaptor2d_Curve2d.ShallowCopy"/>: the curve is shared, the evaluation caches are not)
/// and <see cref="Qualified"/> returns a new one each time, so no caller or solver shares the evaluation caches of
/// another (decision 797). Two qualified curves are equal when they have the same qualifier, the same Geom2d curve
/// object and the same parameter range. <c>default</c> has no curve.
/// </remarks>
public readonly struct Geom2dGcc_QualifiedCurve : IEquatable<Geom2dGcc_QualifiedCurve>
{
    private readonly GccEnt_Position TheQualifier;
    private readonly Geom2dAdaptor_Curve TheQualified;

    /// <summary>Assigns the qualifier to the curve (copied).</summary>
    public Geom2dGcc_QualifiedCurve(Geom2dAdaptor_Curve Curve, GccEnt_Position Qualifier)
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

    public bool Equals(Geom2dGcc_QualifiedCurve other)
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

    public override bool Equals(object? obj) => obj is Geom2dGcc_QualifiedCurve other && Equals(other);

    public override int GetHashCode()
        => TheQualified is null
               ? HashCode.Combine(TheQualifier)
               : HashCode.Combine(TheQualifier, TheQualified.Curve(), TheQualified.FirstParameter(), TheQualified.LastParameter());

    public static bool operator ==(in Geom2dGcc_QualifiedCurve theLeft, in Geom2dGcc_QualifiedCurve theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Geom2dGcc_QualifiedCurve theLeft, in Geom2dGcc_QualifiedCurve theRight) => !theLeft.Equals(theRight);

    public override string ToString()
        => "Geom2dGcc_QualifiedCurve(" + (TheQualified is null
                                       ? "null"
                                       : TheQualified.GetType().ToString() + " "
                                         + Constants.Format("range", TheQualified.FirstParameter(), TheQualified.LastParameter()))
           + ", " + GccEnt.PositionToString(TheQualifier) + ")";
}
