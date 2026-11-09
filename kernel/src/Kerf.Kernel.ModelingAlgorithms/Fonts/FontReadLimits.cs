// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Fonts;

/// <summary>
/// What the font reader refuses rather than allocate or loop for, reading a file that may be damaged or hostile, as the
/// exchange readers' <c>ReadLimits</c> do: a font past one fails with <see cref="Font.LimitExceeded"/>. The defaults
/// pass every font a system ships, the largest CJK collections among them.
/// </summary>
public sealed record FontReadLimits
{
    /// <summary>256 MB, 1,024 fonts in a collection, 100,000 points to a glyph, components 16 deep, 100,000 charstring operations.</summary>
    public static FontReadLimits Default { get; } = new();

    /// <summary>The most bytes read.</summary>
    public long MaxBytes { get; init; } = 256L << 20;

    /// <summary>The most fonts in a collection (<c>.ttc</c>).</summary>
    public int MaxFaces { get; init; } = 1024;

    /// <summary>The most points in one glyph's outline with its components, and the most components it loads.</summary>
    public int MaxOutlinePoints { get; init; } = 100_000;

    /// <summary>The deepest a TrueType glyph's components may nest; a loop of components fails here.</summary>
    public int MaxCompositeDepth { get; init; } = 16;

    /// <summary>The most operations one CFF glyph's charstring may run, with its subroutines.</summary>
    public int MaxCharStringOperations { get; init; } = 100_000;
}
