// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Foundation;

/// <summary>
/// An area, volume or centre that couldn't be confirmed to the precision asked for (decision 2460): thrown by the public
/// measures that return a number, which a caller sees as an <see cref="ArithmeticException"/> whose message is the
/// diagnostic's, and turned back into that diagnostic by <see cref="Diagnostic.FromException"/>. The kernel's own checks
/// and proofs catch it and fail, so no figure that isn't known passes one.
/// </summary>
internal sealed class UnconfirmedMeasureException(Diagnostic diagnostic) : ArithmeticException(diagnostic.ToString())
{
    /// <summary>The error a <see cref="Result{T}"/> would carry: <c>measurement.unconfirmed</c> and why.</summary>
    public Diagnostic Diagnostic { get; } = diagnostic;
}
