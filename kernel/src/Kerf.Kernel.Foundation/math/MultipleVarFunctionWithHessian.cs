// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunctionWithHessian.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Numerics;

/// <summary>A function of several variables whose gradient and hessian are available.</summary>
internal abstract class MultipleVarFunctionWithHessian : MultipleVarFunctionWithGradient
{
    /// <summary>Computes the value <paramref name="f"/>, the gradient <paramref name="g"/> and the hessian <paramref name="h"/> at <paramref name="x"/>.</summary>
    public abstract bool Values(math_Vector x, out double f, math_Vector g, Matrix h);
}
