// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunctionWithHessian.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A function of several variables whose gradient and hessian are available.</summary>
public abstract class math_MultipleVarFunctionWithHessian : math_MultipleVarFunctionWithGradient
{
    /// <summary>Computes the value <paramref name="F"/>, the gradient <paramref name="G"/> and the hessian <paramref name="H"/> at <paramref name="X"/>.</summary>
    public abstract bool Values(math_Vector X, out double F, math_Vector G, math_Matrix H);
}
