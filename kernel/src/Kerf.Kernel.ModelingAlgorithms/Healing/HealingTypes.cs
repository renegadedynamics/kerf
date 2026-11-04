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
}

/// <summary>How faces are sewn (OCCT's <c>BRepBuilderAPI_Sewing</c>).</summary>
public sealed record SewOptions
{
    /// <summary>OCCT's defaults.</summary>
    public static SewOptions Default { get; } = new();

    /// <summary>Edges closer than this are sewn.</summary>
    public double Tolerance
    {
        get;
        init => field = value > 0 ? value : throw new ArgumentOutOfRangeException(nameof(value), value, "The tolerance must be more than zero.");
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

/// <summary>Which surfaces <c>HealingOperations.ConvertToBSpline</c> converts (OCCT's <c>ShapeCustom::ConvertToBSpline</c>).</summary>
public sealed record BSplineConversionOptions
{
    /// <summary>Extrusions, revolutions and offset surfaces; not planes.</summary>
    public static BSplineConversionOptions Default { get; } = new();

    /// <summary>Surfaces of linear extrusion.</summary>
    public bool Extrusions { get; init; } = true;

    /// <summary>Surfaces of revolution.</summary>
    public bool Revolutions { get; init; } = true;

    /// <summary>Offset surfaces.</summary>
    public bool Offsets { get; init; } = true;

    /// <summary>Planes.</summary>
    public bool Planes { get; init; }
}
