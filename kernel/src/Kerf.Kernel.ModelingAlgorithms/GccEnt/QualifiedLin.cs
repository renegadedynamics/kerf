// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedLin.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedLin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A line with a qualifier: whether the solutions computed from it lie on its left, its inside (GccEnt_enclosed), on
/// its right (GccEnt_outside), or on either side (GccEnt_unqualified).
/// </summary>
/// <remarks>A value: OCCT gives it no setters. <c>default</c> is an unqualified line along X through the origin.</remarks>
internal readonly struct QualifiedLin : IEquatable<QualifiedLin>
{
    private readonly Qualifier _theQualifier;
    private readonly Line2D _theQualified;

    /// <summary>Assigns the qualifier to the line.</summary>
    public QualifiedLin(in Line2D qualified, Qualifier qualifier)
    {
        _theQualified = qualified;
        _theQualifier = qualifier;
    }

    /// <summary>The line to which the qualifier is assigned.</summary>
    public Line2D Qualified()
    {
        return _theQualified;
    }

    /// <summary>The qualifier.</summary>
    public Qualifier Qualifier()
    {
        return _theQualifier;
    }

    public bool IsUnqualified
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified;
        }
    }

    /// <summary>True if the solutions are on the left of the line (its inside).</summary>
    public bool IsEnclosed
    {
        get
        {
            return (_theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed);
        }
    }

    /// <summary>True if the solutions are on the right of the line (its outside).</summary>
    public bool IsOutside
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        }
    }

    public bool Equals(QualifiedLin other)
        => _theQualified.Equals(other._theQualified) && _theQualifier == other._theQualifier;

    public override bool Equals(object? obj) => obj is QualifiedLin other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_theQualified, _theQualifier);

    public static bool operator ==(in QualifiedLin left, in QualifiedLin right) => left.Equals(right);

    public static bool operator !=(in QualifiedLin left, in QualifiedLin right) => !left.Equals(right);

    public override string ToString()
        => "GccEnt_QualifiedLin(" + _theQualified.ToString() + ", " + global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(_theQualifier) + ")";
}
