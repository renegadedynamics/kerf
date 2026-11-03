using Occt;

namespace Kerf.Modeling;

/// <summary>
/// Limits for importing data that may be hostile or broken: a reader refuses data past them with an error, rather than
/// allocate without bound. The defaults allow any reasonable model.
/// </summary>
/// <remarks>
/// Every limit applies to the format it names, checked as the data is parsed.
/// </remarks>
/// <param name="MaxBytes">The most bytes read (every format).</param>
/// <param name="MaxEntities">The most STEP records.</param>
/// <param name="MaxDepth">The deepest nesting: STEP's parameter lists, glTF's node hierarchy.</param>
/// <param name="MaxTriangles">The most triangles (STL, glTF).</param>
/// <param name="MaxNodes">The most glTF nodes.</param>
public sealed record ImportLimits(long MaxBytes = 1L << 30, int MaxEntities = 20_000_000, int MaxDepth = 1_000,
                                  long MaxTriangles = 50_000_000, int MaxNodes = 1_000_000)
{
    /// <summary>The default limits: a gigabyte, 20 million STEP records, 50 million triangles.</summary>
    public static ImportLimits Default { get; } = new();

    internal ExchangeReadLimits ToExchange() => new(MaxBytes, MaxEntities, MaxDepth, MaxTriangles, MaxNodes);
}
