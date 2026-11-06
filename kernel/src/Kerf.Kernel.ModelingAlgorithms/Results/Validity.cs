// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
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

    /// <summary>
    /// The largest tolerance of a vertex or edge of <paramref name="shape"/>. A fillet's result with one past its radius is
    /// wrong though the check passes: a tolerance that large swallows the geometry (#1639, #1695, #1701; decision 20904).
    /// </summary>
    public static double LargestTolerance(Shape shape)
    {
        double largest = 0.0;
        for (var edges = new Explorer(shape, ShapeType.Edge); edges.More(); edges.Next())
        {
            largest = Math.Max(largest, Tool.Tolerance(ShapeCasts.Edge(edges.Current())));
        }

        for (var vertices = new Explorer(shape, ShapeType.Vertex); vertices.More(); vertices.Next())
        {
            largest = Math.Max(largest, Tool.Tolerance(ShapeCasts.Vertex(vertices.Current())));
        }

        return largest;
    }

    /// <summary>The error if a tolerance of <paramref name="shape"/> is past <paramref name="limit"/>, or null.</summary>
    public static Diagnostic? ToleranceProblem(Shape shape, double limit, string what)
    {
        double largest = shape.IsNull ? 0.0 : LargestTolerance(shape);
        return largest > limit
            ? Diagnostic.Error(DiagnosticCode.InvalidResult,
                               FormattableString.Invariant($"{what} isn't reliable: it has a tolerance of {largest:G4}, past the {limit:G4} that the operation could need."))
            : null;
    }

    /// <summary><paramref name="made"/>, if it's a success that isn't valid as a failure carrying it.</summary>
    public static Result<T> Checked<T>(Result<T> made, bool check, string what)
        where T : struct, IShape<T> =>
        check && made.TryGetValue(out T value) && Problem(value.AsShape(), what) is { } problem
            ? Result<T>.Invalid(value, problem, made.Diagnostics)
            : made;
}
