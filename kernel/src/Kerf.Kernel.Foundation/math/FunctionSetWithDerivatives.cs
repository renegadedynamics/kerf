// Port of: src/FoundationClasses/TKMath/math/math_FunctionSetWithDerivatives.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>A set of N functions of M independent variables whose derivatives are available.</summary>
internal abstract class FunctionSetWithDerivatives : FunctionSet
{
    /// <summary>Computes the derivatives <paramref name="d"/> (the Jacobian, N rows by M columns) at <paramref name="x"/>.</summary>
    public abstract bool Derivatives(math_Vector x, Matrix d);

    /// <summary>Computes the values <paramref name="f"/> and the derivatives <paramref name="d"/> at <paramref name="x"/>.</summary>
    public abstract bool Values(math_Vector x, math_Vector f, Matrix d);
}
