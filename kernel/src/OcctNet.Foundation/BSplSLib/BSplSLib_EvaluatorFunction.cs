// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib_EvaluatorFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// History - C function pointer converted to a virtual class
// in order to get rid of usage of static functions and static data

namespace Occt;

/// <summary>
/// A function of two parameters (u, v) for <see cref="BSplSLib.FunctionMultiply"/>, to be defined by a descendant.
/// </summary>
/// <remarks>OCCT's function-call <c>operator()</c> is left out: call <see cref="Evaluate"/>.</remarks>
public abstract class BSplSLib_EvaluatorFunction
{
    /// <summary>Evaluates the function (or its derivative of the requested order) at (u, v).</summary>
    public abstract void Evaluate(int theDerivativeRequest, double theUParameter, double theVParameter,
                                  out double theResult, out int theErrorCode);
}
