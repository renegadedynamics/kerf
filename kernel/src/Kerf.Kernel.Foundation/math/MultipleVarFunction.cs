// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunction.hxx
// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>A function of several variables.</summary>
internal abstract class MultipleVarFunction
{
    /// <summary>The number of variables of the function.</summary>
    public abstract int VariableCount { get; }

    /// <summary>Computes the value <paramref name="f"/> at <paramref name="x"/>; false if it could not be computed.</summary>
    public abstract bool Value(math_Vector x, out double f);

    /// <summary>
    /// The state of the function at the latest call, which algorithms record when they find a solution. 0 (the
    /// default) means no state is saved.
    /// </summary>
    public virtual int GetStateNumber()
    {
        return 0;
    }
}
