// Port of: src/FoundationClasses/TKMath/math/math_FunctionWithDerivative.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionWithDerivative.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>A function of a single variable whose first derivative is available.</summary>
internal abstract class FunctionWithDerivative : Function
{
    /// <summary>Computes the derivative <paramref name="d"/> at <paramref name="x"/>; false if it could not be computed.</summary>
    public abstract bool Derivative(double x, out double d);

    /// <summary>Computes the value <paramref name="f"/> and the derivative <paramref name="d"/> at <paramref name="x"/>.</summary>
    public abstract bool Values(double x, out double f, out double d);
}
