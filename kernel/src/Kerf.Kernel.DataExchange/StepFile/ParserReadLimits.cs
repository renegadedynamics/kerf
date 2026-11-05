// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

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
}
