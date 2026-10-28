// Port of: src/FoundationClasses/TKMath/math/math_MultipleVarFunctionWithGradient.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A function of several variables whose gradient is available.</summary>
public abstract class math_MultipleVarFunctionWithGradient : math_MultipleVarFunction
{
    /// <summary>Computes the gradient <paramref name="G"/> at <paramref name="X"/>; false if it could not be computed.</summary>
    public abstract bool Gradient(math_Vector X, math_Vector G);

    /// <summary>Computes the value <paramref name="F"/> and the gradient <paramref name="G"/> at <paramref name="X"/>.</summary>
    public abstract bool Values(math_Vector X, out double F, math_Vector G);
}
