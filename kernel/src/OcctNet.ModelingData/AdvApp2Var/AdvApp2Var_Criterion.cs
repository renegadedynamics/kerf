// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Criterion.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Criterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A criterion that the patches of <see cref="AdvApp2Var_ApproxAFunc2Var"/> must satisfy.</summary>
/// <remarks>
/// Subclasses (GeomPlate's G0 and G1 criteria) set the protected fields in their constructors, compute the
/// criterion's value on a patch in <see cref="Value"/> (storing it with <see cref="AdvApp2Var_Patch.SetCritValue"/>)
/// and test it in <see cref="IsSatisfied"/>.
/// </remarks>
public abstract class AdvApp2Var_Criterion
{
    protected double myMaxValue;
    protected AdvApp2Var_CriterionType myType;
    protected AdvApp2Var_CriterionRepartition myRepartition;

    /// <summary>Computes the criterion on <paramref name="P"/> and stores it in the patch.</summary>
    public abstract void Value(AdvApp2Var_Patch P, AdvApp2Var_Context C);

    /// <summary>True if <paramref name="P"/> satisfies the criterion.</summary>
    public abstract bool IsSatisfied(AdvApp2Var_Patch P);

    public double MaxValue() => myMaxValue;

    public AdvApp2Var_CriterionType Type() => myType;

    public AdvApp2Var_CriterionRepartition Repartition() => myRepartition;
}
