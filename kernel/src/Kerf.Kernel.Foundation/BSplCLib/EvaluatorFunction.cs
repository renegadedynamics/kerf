// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_EvaluatorFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Geometry.Splines;

/// <summary>
/// A one-dimensional function of the parameter, used to multiply a vectorial B-spline by a function
/// (<c>BSplCLib.FunctionMultiply</c>) or to reparametrise one (<c>BSplCLib.FunctionReparameterise</c>).
/// </summary>
/// <remarks>
/// OCCT's <c>operator()</c> shortcut is left out: call <see cref="Evaluate"/>. As for math's functions
/// (docs/decisions.md), an implementation must assign <c>theResult</c> even when it fails.
/// </remarks>
internal abstract class EvaluatorFunction
{
    /// <summary>
    /// Evaluates the function, or its derivative of order <paramref name="derivativeRequest"/>, at
    /// <paramref name="parameter"/>. <paramref name="startEnd"/> holds the two ends of the parameter range.
    /// <paramref name="errorCode"/> is 0 on success.
    /// </summary>
    public abstract void Evaluate(int derivativeRequest, ReadOnlySpan<double> startEnd, double parameter,
                                  out double result, out int errorCode);
}
