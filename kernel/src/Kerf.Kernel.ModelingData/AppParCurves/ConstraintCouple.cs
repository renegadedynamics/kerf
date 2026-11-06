// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_ConstraintCouple.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_ConstraintCouple.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>Associates the index of a point with its constraint.</summary>
/// <remarks>
/// A value type (section 3 of the translation rules): OCCT copies it by value. Its default (parameterless
/// constructor) is index -1 with no constraint, so ported code writes <c>new AppParCurves_ConstraintCouple()</c>;
/// <c>default</c> has index 0.
/// </remarks>
internal partial struct ConstraintCouple : IEquatable<ConstraintCouple>
{
    private int _index;
    private Constraint _constraint;

    /// <summary>An indefinite couple: index -1, no constraint.</summary>
    public ConstraintCouple()
    {
        _index = -1;
        _constraint = NoConstraint;
    }

    /// <summary>The point <paramref name="theIndex"/> has the constraint <paramref name="cons"/>.</summary>
    public ConstraintCouple(int theIndex, Constraint cons)
    {
        _index = theIndex;
        _constraint = cons;
    }

    /// <summary>The index of the constrained point.</summary>
    public readonly int Index() => _index;

    /// <summary>The constraint of the point.</summary>
    public readonly Constraint Constraint() => _constraint;

    internal void SetIndex(int theIndex) => _index = theIndex;

    internal void SetConstraint(Constraint cons) => _constraint = cons;

    /// <summary>A copy with the index <paramref name="theIndex"/> (the twin of <c>SetIndex</c>).</summary>
    public readonly ConstraintCouple WithIndex(int theIndex)
    {
        ConstraintCouple copy = this;
        copy.SetIndex(theIndex);
        return copy;
    }

    /// <summary>A copy with the constraint <paramref name="cons"/> (the twin of <c>SetConstraint</c>).</summary>
    public readonly ConstraintCouple WithConstraint(Constraint cons)
    {
        ConstraintCouple copy = this;
        copy.SetConstraint(cons);
        return copy;
    }

    public readonly bool Equals(ConstraintCouple other) =>
        _index == other._index && _constraint == other._constraint;

    public override readonly bool Equals(object? obj) => obj is ConstraintCouple other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_index, _constraint);

    public static bool operator ==(ConstraintCouple left, ConstraintCouple right) => left.Equals(right);

    public static bool operator !=(ConstraintCouple left, ConstraintCouple right) => !left.Equals(right);

    public override readonly string ToString() =>
        "AppParCurves_ConstraintCouple(" + _index.ToString(CultureInfo.InvariantCulture) + ", " + _constraint + ")";
}
