// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Step.Parser;

/// <summary>
/// Limits for hostile input to the STEP parser (<see cref="StepParser"/>), which OCCT's flex and bison parser
/// doesn't have. Reaching one stops the parse as a syntax error that can't be recovered does: the parser returns 1,
/// with a message in <see cref="ReadData.GetLastError"/>.
/// </summary>
/// <remarks>
/// The defaults are far above any real file's (unit 8P's decision). A token (a string with its quotes, a name, a
/// number, a run of blanks inside a comment) is limited only by <see cref="MaxFileSize"/> (OCCT's scanner failed on one
/// of 16,384 bytes or more: Kerf decision 1511).
/// </remarks>
internal sealed record ParserReadLimits
{
    /// <summary>The defaults: 4 GiB, 100 million records, nesting 10,000 deep.</summary>
    public static ParserReadLimits Default { get; } = new();

    /// <summary>The most bytes read from the stream.</summary>
    public long MaxFileSize { get; init; } = 4L << 30;

    /// <summary>The most records (entities, sub-lists, scope marks, the header's entities).</summary>
    public int MaxRecords { get; init; } = 100_000_000;

    /// <summary>The deepest nesting of lists in a parameter list, plus scopes.</summary>
    public int MaxDepth { get; init; } = 10_000;

    /// <summary>
    /// Kerf (decision 23105): where a parse that stops at a limit says which, for its caller to tell a refused file
    /// from a malformed one; null for no report. The caller's own object, for its read alone.
    /// </summary>
    public ParserLimitReport? Report { get; init; }
}

/// <summary>Which of <see cref="ParserReadLimits"/> a parse stopped at (decision 23105).</summary>
internal sealed class ParserLimitReport
{
    /// <summary>The limit reached ("record count", "nesting depth", "file size"), or null if none was.</summary>
    public string? Exceeded { get; set; }
}
