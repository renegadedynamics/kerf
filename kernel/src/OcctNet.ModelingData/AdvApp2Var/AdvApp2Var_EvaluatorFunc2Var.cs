// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_EvaluatorFunc2Var.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The function of two variables that <see cref="AdvApp2Var_ApproxAFunc2Var"/> approximates.</summary>
/// <remarks>
/// OCCT passes every argument through a pointer; none of its implementations writes an input, so the inputs are
/// by value and the arrays spans (decision 1902, as decision 145 for AdvApprox_EvaluatorFunction).
/// <c>operator()</c> is left out. Implementations must always assign the error code.
/// </remarks>
public abstract class AdvApp2Var_EvaluatorFunc2Var
{
    /// <summary>
    /// Evaluates the function, or its derivative of order (<paramref name="theUOrder"/>, <paramref name="theVOrder"/>),
    /// at <paramref name="theNbParams"/> points of an iso-curve.
    /// </summary>
    /// <param name="theDimension">The dimension of the function's values.</param>
    /// <param name="theUStartEnd">The first and last parameters in U.</param>
    /// <param name="theVStartEnd">The first and last parameters in V.</param>
    /// <param name="theFavorIso">The constant parameter: 1 for U, 2 for V.</param>
    /// <param name="theConstParam">The value of the constant parameter.</param>
    /// <param name="theNbParams">The number N of points.</param>
    /// <param name="theParameters">The N values of the other parameter.</param>
    /// <param name="theUOrder">The order of derivation in U.</param>
    /// <param name="theVOrder">The order of derivation in V.</param>
    /// <param name="theResult">The values: <c>theResult[i * theDimension + d]</c> for point i and coordinate d.</param>
    /// <param name="theErrorCode">0 on success.</param>
    public abstract void Evaluate(int                 theDimension,
                                  ReadOnlySpan<double> theUStartEnd,
                                  ReadOnlySpan<double> theVStartEnd,
                                  int                 theFavorIso,
                                  double              theConstParam,
                                  int                 theNbParams,
                                  ReadOnlySpan<double> theParameters,
                                  int                 theUOrder,
                                  int                 theVOrder,
                                  Span<double>        theResult,
                                  out int             theErrorCode);
}
