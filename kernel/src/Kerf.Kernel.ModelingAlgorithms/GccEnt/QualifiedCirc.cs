// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedCirc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt_QualifiedCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A circle with a qualifier: whether the solutions computed from it enclose it (GccEnt_enclosing), are enclosed by
/// it (GccEnt_enclosed), are outside it (GccEnt_outside), or may be any of these (GccEnt_unqualified).
/// </summary>
/// <remarks>A value: OCCT gives it no setters. <c>default</c> is an unqualified circle of radius 0 at the origin.</remarks>
internal readonly struct QualifiedCirc : IEquatable<QualifiedCirc>
{
    private readonly Circle2D _theQualified;
    private readonly Qualifier _theQualifier;

    /// <summary>Assigns the qualifier to the circle.</summary>
    public QualifiedCirc(in Circle2D qualified, Qualifier qualifier)
    {
        _theQualified = qualified;
        _theQualifier = qualifier;
    }

    /// <summary>The circle to which the qualifier is assigned.</summary>
    public Circle2D Qualified()
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

    /// <summary>True if the solutions enclose the circle.</summary>
    public bool IsEnclosing
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing;
        }
    }

    /// <summary>True if the circle encloses the solutions.</summary>
    public bool IsEnclosed
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed;
        }
    }

    /// <summary>True if the solutions and the circle are outside each other.</summary>
    public bool IsOutside
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        }
    }

    public bool Equals(QualifiedCirc other)
        => _theQualified.Equals(other._theQualified) && _theQualifier == other._theQualifier;

    public override bool Equals(object? obj) => obj is QualifiedCirc other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(_theQualified, _theQualifier);

    public static bool operator ==(in QualifiedCirc left, in QualifiedCirc right) => left.Equals(right);

    public static bool operator !=(in QualifiedCirc left, in QualifiedCirc right) => !left.Equals(right);

    public override string ToString()
        => "GccEnt_QualifiedCirc(" + _theQualified.ToString() + ", " + global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(_theQualifier) + ")";
}
