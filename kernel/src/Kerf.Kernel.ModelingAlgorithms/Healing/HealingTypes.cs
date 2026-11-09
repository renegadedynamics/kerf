// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Healing;

/// <summary>How a shape is checked (OCCT's <c>BRepCheck_Analyzer</c>).</summary>
public sealed record ValidationOptions
{
    /// <summary>Every check, in parallel.</summary>
    public static ValidationOptions Default { get; } = new();

    /// <summary>Check the geometry too (curves on surfaces, tolerances); false checks only the topology.</summary>
    public bool Geometry { get; init; } = true;

    /// <summary>Check the sub-shapes on several threads.</summary>
    public bool Parallel { get; init; } = true;

    /// <summary>Measure the edges on faces exactly rather than by sampling (slower).</summary>
    public bool Exact { get; init; }
}

/// <summary>What checking a shape found (an invalid shape is a finding, not a failed operation).</summary>
public sealed record ValidationReport
{
    /// <summary>True if no fault was found on the shape or its sub-shapes.</summary>
    public required bool IsValid { get; init; }

    /// <summary>The faults found, each on its sub-shape; empty for a valid shape.</summary>
    public required ImmutableArray<ShapeProblem> Problems { get; init; }

    /// <summary>
    /// The error (<c>result.invalid</c>) for a result with these faults, naming the sub-shapes:
    /// <c>The fillet's result isn't valid: a shape's orientation is inconsistent (and 2 more faults).</c>
    /// </summary>
    /// <param name="what">What to call the result, starting with a capital.</param>
    public Diagnostic ToDiagnostic(string what)
    {
        KernelArgumentNullException.ThrowIfNull(what);
        string[] kinds = [.. Problems.Select(static p => p.Message.TrimEnd('.')).Distinct()];
        string said = kinds.Length == 0 ? "a fault was found" : kinds[0];
        string more = (kinds.Length - 1) switch
        {
            <= 0 => "",
            1 => " (and 1 more kind of fault)",
            int n => $" (and {n} more kinds of fault)",
        };
        return Diagnostic.Error(DiagnosticCode.InvalidResult, $"{what} isn't valid: {said}{more}.")
                         .WithShapes(Problems.Select(static p => p.Shape).Where(static s => !s.IsNull).Distinct());
    }
}

/// <summary>A fault of a sub-shape, on its own or in its context (an edge on one of its faces).</summary>
/// <param name="Shape">The sub-shape at fault.</param>
/// <param name="Context">The shape it was checked in, or a null shape for a fault of its own.</param>
/// <param name="Code">The fault, stable for programs: <c>check.</c> and OCCT's <c>BRepCheck_Status</c> in words
/// (<c>check.not-closed</c>, <c>check.self-intersecting-wire</c>).</param>
/// <param name="Message">The fault in plain English.</param>
public readonly record struct ShapeProblem(Shape Shape, Shape Context, DiagnosticCode Code, string Message);

/// <summary>How a shape is fixed (OCCT's <c>ShapeFix_Shape</c>). A null mode lets the fixer decide, as OCCT's -1.</summary>
public sealed record FixOptions
{
    /// <summary>OCCT's defaults.</summary>
    public static FixOptions Default { get; } = new();

    /// <summary>The basic precision of the fixes.</summary>
    public double Precision { get; init; } = Tolerances.Confusion;

    /// <summary>The smallest tolerance a fix may set.</summary>
    public double MinTolerance { get; init; } = Tolerances.Confusion;

    /// <summary>The largest tolerance a fix may set.</summary>
    public double MaxTolerance { get; init; } = Tolerances.Confusion;

    /// <summary>Fix solids (their shells' orientation, solids from shells).</summary>
    public bool? FixSolids { get; init; }

    /// <summary>Fix shells that are not in solids.</summary>
    public bool? FixShells { get; init; }

    /// <summary>Fix faces that are not in shells.</summary>
    public bool? FixFaces { get; init; }

    /// <summary>Fix wires that are not in faces.</summary>
    public bool? FixWires { get; init; }

    /// <summary>Make the edges' curves agree with their curves on faces (SameParameter).</summary>
    public bool? FixSameParameter { get; init; }

    /// <summary>
    /// Whether <c>Fix</c> checks its result and fails (<c>result.invalid</c>), carrying it, if it is still invalid: true by
    /// default. The check costs a validation of the result; a caller that checks a later step can leave it out.
    /// </summary>
    public bool CheckResult { get; init; } = true;
}

/// <summary>How <c>HealingOperations.FixWireframe</c> fixes a wireframe (OCCT's <c>ShapeFix_Wireframe</c>).</summary>
public sealed record WireframeFixOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>OCCT's defaults: its own maximum tolerance, the gaps fixed before the small edges.</summary>
    public static WireframeFixOptions Default { get; } = new();

    /// <summary>The largest tolerance a fix may give an edge or vertex; null for OCCT's own.</summary>
    public double? MaxTolerance { get; init; }

    /// <summary>Fix the small edges before the gaps, rather than after.</summary>
    public bool SmallEdgesFirst { get; init; }
}

/// <summary>How faces are sewn (OCCT's <c>BRepBuilderAPI_Sewing</c>).</summary>
public sealed record SewOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>OCCT's defaults.</summary>
    public static SewOptions Default { get; } = new();

    /// <summary>Edges closer than this are sewn.</summary>
    public double Tolerance
    {
        get;
        init => field = value > 0 ? value : throw new KernelArgumentOutOfRangeException(nameof(value), value, "The tolerance must be more than zero.");
    } = 1e-6;

    /// <summary>Edges and faces smaller than this are degenerate; null for OCCT's, a ten-thousandth of
    /// <see cref="Tolerance"/>.</summary>
    public double? MinTolerance { get; init; }

    /// <summary>The largest gap closed, edges farther apart staying free; null for no limit.</summary>
    public double? MaxTolerance { get; init; }

    /// <summary>Allow edges shared by more than two faces.</summary>
    public bool NonManifold { get; init; }
}

/// <summary>How faces and edges on the same surface or curve are merged (OCCT's <c>ShapeUpgrade_UnifySameDomain</c>).</summary>
public sealed record UnifyOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>Faces and edges merged, B-splines concatenated.</summary>
    public static UnifyOptions Default { get; } = new();

    /// <summary>Merge the edges.</summary>
    public bool Edges { get; init; } = true;

    /// <summary>Merge the faces.</summary>
    public bool Faces { get; init; } = true;

    /// <summary>Concatenate B-spline curves into one.</summary>
    public bool ConcatBSplines { get; init; } = true;

    /// <summary>Keep the edges inside merged faces that separate nothing (false removes them).</summary>
    public bool AllowInternalEdges { get; init; }

    /// <summary>The linear tolerance of "the same surface or curve".</summary>
    public double LinearTolerance { get; init; } = Tolerances.Confusion;

    /// <summary>The angular tolerance, in radians.</summary>
    public double AngularTolerance { get; init; } = Tolerances.Angular;
}

/// <summary>
/// How <c>HealingOperations.ApproximateBSplines</c> approximates a shape's B-spline and Bezier geometry, and the extrusions,
/// revolutions and offsets based on it (OCCT's <c>ShapeCustom::BSplineRestriction</c>; its defaults).
/// </summary>
public sealed record BSplineApproximationOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>OCCT's defaults.</summary>
    public static BSplineApproximationOptions Default { get; } = new();

    /// <summary>The 3D tolerance of the approximation.</summary>
    public double Tolerance3D { get; init; } = 0.01;

    /// <summary>The 2D tolerance of the approximation of curves on faces.</summary>
    public double Tolerance2D { get; init; } = 1e-6;

    /// <summary>The highest degree.</summary>
    public int MaxDegree { get; init; } = 9;

    /// <summary>The most spans.</summary>
    public int MaxSegments { get; init; } = 10000;

    /// <summary>The continuity of the 3D curves and surfaces.</summary>
    public Continuity Continuity3D { get; init; } = Continuity.C1;

    /// <summary>The continuity of the curves on faces.</summary>
    public Continuity Continuity2D { get; init; } = Continuity.C2;

    /// <summary>Keep the degree within <see cref="MaxDegree"/> first, rather than the spans within <see cref="MaxSegments"/>.</summary>
    public bool PreferDegree { get; init; } = true;

    /// <summary>Make rational geometry polynomial.</summary>
    public bool Polynomial { get; init; }
}

/// <summary>Which surfaces <c>HealingOperations.ConvertToBSpline</c> converts (OCCT's <c>ShapeCustom::ConvertToBSpline</c>).</summary>
public sealed record BSplineConversionOptions
{
    /// <summary>Check the result, and fail with <c>result.invalid</c>, carrying it, if it isn't valid: true by default.</summary>
    public bool CheckResult { get; init; } = true;

    /// <summary>Extrusions, revolutions and offset surfaces; not planes.</summary>
    public static BSplineConversionOptions Default { get; } = new();

    /// <summary>Surfaces of linear extrusion.</summary>
    public bool Extrusions { get; init; } = true;

    /// <summary>Surfaces of revolution.</summary>
    public bool Revolutions { get; init; } = true;

    /// <summary>Whether offset surfaces are converted: on by default.</summary>
    public bool Offsets { get; init; } = true;

    /// <summary>Whether planes are converted: off by default.</summary>
    public bool Planes { get; init; }
}
