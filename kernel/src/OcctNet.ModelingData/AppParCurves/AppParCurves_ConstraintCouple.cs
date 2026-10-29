// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_ConstraintCouple.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_ConstraintCouple.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>Associates the index of a point with its constraint.</summary>
/// <remarks>
/// A value type (section 3 of the translation rules): OCCT copies it by value. Its default (parameterless
/// constructor) is index -1 with no constraint, so ported code writes <c>new AppParCurves_ConstraintCouple()</c>;
/// <c>default</c> has index 0.
/// </remarks>
public partial struct AppParCurves_ConstraintCouple : IEquatable<AppParCurves_ConstraintCouple>
{
    private int myIndex;
    private AppParCurves_Constraint myConstraint;

    /// <summary>An indefinite couple: index -1, no constraint.</summary>
    public AppParCurves_ConstraintCouple()
    {
        myIndex = -1;
        myConstraint = AppParCurves_NoConstraint;
    }

    /// <summary>The point <paramref name="TheIndex"/> has the constraint <paramref name="Cons"/>.</summary>
    public AppParCurves_ConstraintCouple(int TheIndex, AppParCurves_Constraint Cons)
    {
        myIndex = TheIndex;
        myConstraint = Cons;
    }

    /// <summary>The index of the constrained point.</summary>
    public readonly int Index() => myIndex;

    /// <summary>The constraint of the point.</summary>
    public readonly AppParCurves_Constraint Constraint() => myConstraint;

    internal void SetIndex(int TheIndex) => myIndex = TheIndex;

    internal void SetConstraint(AppParCurves_Constraint Cons) => myConstraint = Cons;

    /// <summary>A copy with the index <paramref name="TheIndex"/> (the twin of <c>SetIndex</c>).</summary>
    public readonly AppParCurves_ConstraintCouple WithIndex(int TheIndex)
    {
        AppParCurves_ConstraintCouple aCopy = this;
        aCopy.SetIndex(TheIndex);
        return aCopy;
    }

    /// <summary>A copy with the constraint <paramref name="Cons"/> (the twin of <c>SetConstraint</c>).</summary>
    public readonly AppParCurves_ConstraintCouple WithConstraint(AppParCurves_Constraint Cons)
    {
        AppParCurves_ConstraintCouple aCopy = this;
        aCopy.SetConstraint(Cons);
        return aCopy;
    }

    public readonly bool Equals(AppParCurves_ConstraintCouple other) =>
        myIndex == other.myIndex && myConstraint == other.myConstraint;

    public override readonly bool Equals(object? obj) => obj is AppParCurves_ConstraintCouple other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myIndex, myConstraint);

    public static bool operator ==(AppParCurves_ConstraintCouple left, AppParCurves_ConstraintCouple right) => left.Equals(right);

    public static bool operator !=(AppParCurves_ConstraintCouple left, AppParCurves_ConstraintCouple right) => !left.Equals(right);

    public override readonly string ToString() =>
        "AppParCurves_ConstraintCouple(" + myIndex.ToString(CultureInfo.InvariantCulture) + ", " + myConstraint + ")";
}
