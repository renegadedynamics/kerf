// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_EvaluatorFunc2Var.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>The function of two variables that <see cref="ApproxAFunc2Var"/> approximates.</summary>
/// <remarks>
/// OCCT passes every argument through a pointer; none of its implementations writes an input, so the inputs are
/// by value and the arrays spans (decision 193, as decision 145 for AdvApprox_EvaluatorFunction).
/// <c>operator()</c> is left out. Implementations must always assign the error code.
/// </remarks>
internal abstract class EvaluatorFunc2Var
{
    /// <summary>
    /// Evaluates the function, or its derivative of order (<paramref name="uOrder"/>, <paramref name="vOrder"/>),
    /// at <paramref name="nbParams"/> points of an iso-curve.
    /// </summary>
    /// <param name="dimension">The dimension of the function's values.</param>
    /// <param name="uStartEnd">The first and last parameters in U.</param>
    /// <param name="vStartEnd">The first and last parameters in V.</param>
    /// <param name="favorIso">The constant parameter: 1 for U, 2 for V.</param>
    /// <param name="constParam">The value of the constant parameter.</param>
    /// <param name="nbParams">The number N of points.</param>
    /// <param name="parameters">The N values of the other parameter.</param>
    /// <param name="uOrder">The order of derivation in U.</param>
    /// <param name="vOrder">The order of derivation in V.</param>
    /// <param name="result">The values: <c>theResult[i * theDimension + d]</c> for point i and coordinate d.</param>
    /// <param name="errorCode">0 on success.</param>
    public abstract void Evaluate(int                 dimension,
                                  ReadOnlySpan<double> uStartEnd,
                                  ReadOnlySpan<double> vStartEnd,
                                  int                 favorIso,
                                  double              constParam,
                                  int                 nbParams,
                                  ReadOnlySpan<double> parameters,
                                  int                 uOrder,
                                  int                 vOrder,
                                  Span<double>        result,
                                  out int             errorCode);
}
