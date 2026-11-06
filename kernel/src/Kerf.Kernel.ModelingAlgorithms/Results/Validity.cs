// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Checks;
using Kerf.Healing;
using Kerf.Topology;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The check behind "no operation reports success for an invalid result" (decision 1742): OCCT's <c>BRepCheck_Analyzer</c>
/// with its defaults, and the error a failure carrying the shape reports.
/// </summary>
internal static class Validity
{
    /// <summary>The error for <paramref name="shape"/> if it isn't valid (what it's called in <paramref name="what"/>), or null.</summary>
    public static Diagnostic? Problem(Shape shape, string what)
    {
        if (shape.IsNull)
        {
            return null;
        }
        try
        {
            // The report is only worth building for a fault
            return new Analyzer(shape, true, true).IsValid() ? null : shape.Validate().ToDiagnostic(what);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            return Diagnostic.Error(DiagnosticCode.InvalidResult, $"{what} couldn't be checked: {FailureDiagnostics.PublicText(e)}");
        }
    }

    /// <summary><paramref name="made"/>, if it's a success that isn't valid as a failure carrying it.</summary>
    public static Result<T> Checked<T>(Result<T> made, bool check, string what)
        where T : struct, IShape<T> =>
        check && made.TryGetValue(out T value) && Problem(value.AsShape(), what) is { } problem
            ? Result<T>.Invalid(value, problem, made.Diagnostics)
            : made;
}
