// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Criterion.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Criterion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>A criterion that the patches of <see cref="ApproxAFunc2Var"/> must satisfy.</summary>
/// <remarks>
/// Subclasses (GeomPlate's G0 and G1 criteria) set the protected fields in their constructors, compute the
/// criterion's value on a patch in <see cref="Value"/> (storing it with <see cref="Patch.SetCritValue"/>)
/// and test it in <see cref="IsSatisfied"/>.
/// </remarks>
internal abstract class Criterion
{
    protected double _maxValue;
    protected CriterionType _type;
    protected CriterionRepartition _repartition;

    /// <summary>Computes the criterion on <paramref name="p"/> and stores it in the patch.</summary>
    public abstract void Value(Patch p, Context c);

    /// <summary>True if <paramref name="p"/> satisfies the criterion.</summary>
    public abstract bool IsSatisfied(Patch p);

    public double MaxValue() => _maxValue;

    public CriterionType Type() => _type;

    public CriterionRepartition Repartition() => _repartition;
}
