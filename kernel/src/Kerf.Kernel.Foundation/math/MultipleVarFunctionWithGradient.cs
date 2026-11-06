// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunctionWithGradient.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Numerics;

/// <summary>A function of several variables whose gradient is available.</summary>
internal abstract class MultipleVarFunctionWithGradient : MultipleVarFunction
{
    /// <summary>Computes the gradient <paramref name="g"/> at <paramref name="x"/>; false if it could not be computed.</summary>
    public abstract bool Gradient(math_Vector x, math_Vector g);

    /// <summary>Computes the value <paramref name="f"/> and the gradient <paramref name="g"/> at <paramref name="x"/>.</summary>
    public abstract bool Values(math_Vector x, out double f, math_Vector g);
}
