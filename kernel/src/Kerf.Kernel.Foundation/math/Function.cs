// Port of: src/FoundationClasses/TKMath/math/math_Function.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Function.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Numerics;

/// <summary>A function of a single variable.</summary>
internal abstract class Function
{
    /// <summary>Computes the value <paramref name="f"/> of the function at <paramref name="x"/>; false if it could not be computed.</summary>
    public abstract bool Value(double x, out double f);

    /// <summary>
    /// The state of the function at the latest call, which algorithms record when they find a solution. 0 (the
    /// default) means no state is saved.
    /// </summary>
    public virtual int GetStateNumber()
    {
        return 0;
    }
}
