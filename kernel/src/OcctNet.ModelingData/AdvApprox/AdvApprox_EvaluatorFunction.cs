// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_EvaluatorFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A function to be approximated by <see cref="AdvApprox_ApproxAFunction"/>: a vector-valued function of one
/// parameter, with its derivatives.
/// </summary>
/// <remarks>
/// OCCT passes every argument through a pointer; the inputs, which no implementation writes, are by value here,
/// the start and end of the range are a span of two values, and the result is a span of
/// <c>Dimension</c> values (decision 72). OCCT's <c>operator()</c> shortcut is left out: call
/// <see cref="Evaluate"/>. As for math's functions, an implementation must assign <c>ErrorCode</c>, 0 on success.
/// </remarks>
public abstract class AdvApprox_EvaluatorFunction
{
    /// <summary>
    /// Evaluates the function, or its derivative of order <paramref name="DerivativeRequest"/> (0, 1 or 2), at
    /// <paramref name="Parameter"/> into the first <paramref name="Dimension"/> values of
    /// <paramref name="Result"/>. <paramref name="StartEnd"/> holds the ends of the interval being approximated
    /// (a hint: an implementation may use it to choose a piece). <paramref name="ErrorCode"/> is 0 on success.
    /// </summary>
    public abstract void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter, int DerivativeRequest,
                                  Span<double> Result, out int ErrorCode);
}
