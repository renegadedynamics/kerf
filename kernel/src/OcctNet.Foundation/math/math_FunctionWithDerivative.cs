// Port of: src/FoundationClasses/TKMath/math/math_FunctionWithDerivative.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionWithDerivative.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A function of a single variable whose first derivative is available.</summary>
public abstract class math_FunctionWithDerivative : math_Function
{
    /// <summary>Computes the derivative <paramref name="D"/> at <paramref name="X"/>; false if it could not be computed.</summary>
    public abstract bool Derivative(double X, out double D);

    /// <summary>Computes the value <paramref name="F"/> and the derivative <paramref name="D"/> at <paramref name="X"/>.</summary>
    public abstract bool Values(double X, out double F, out double D);
}
