// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunction.hxx
// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A function of several variables.</summary>
public abstract class math_MultipleVarFunction
{
    /// <summary>The number of variables of the function.</summary>
    public abstract int NbVariables();

    /// <summary>Computes the value <paramref name="F"/> at <paramref name="X"/>; false if it could not be computed.</summary>
    public abstract bool Value(math_Vector X, out double F);

    /// <summary>
    /// The state of the function at the latest call, which algorithms record when they find a solution. 0 (the
    /// default) means no state is saved.
    /// </summary>
    public virtual int GetStateNumber()
    {
        return 0;
    }
}
