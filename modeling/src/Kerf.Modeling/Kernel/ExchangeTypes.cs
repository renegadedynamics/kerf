using Kerf.Topology;
using Occt;

namespace Kerf.Modeling;

/// <summary>
/// A node of an assembly tree, as the glTF reader gives it (<see cref="Extras.ReadGltf"/>; the writers and STEP take
/// the kernel's <c>Kerf.Exchange.ModelNode</c>): a part (<see cref="Shape"/>, placed by its own
/// location) or an assembly (<see cref="Children"/>, placed by <see cref="Location"/>), with a name and a colour.
/// </summary>
/// <param name="Name">The name, or null.</param>
/// <param name="Color">The colour as linear RGBA (OCCT's <c>Quantity_TOC_RGB</c>), or null to inherit the parent's.</param>
/// <param name="Shape">A part's shape, at its place; null for an assembly.</param>
/// <param name="Location">An assembly's place relative to its parent; null for a part or no move.</param>
/// <param name="Children">An assembly's children.</param>
internal sealed record ExchangeNode(string? Name, (double R, double G, double B, double A)? Color, global::Kerf.Topology.Shape? Shape,
                                  Placement? Location, IReadOnlyList<ExchangeNode> Children)
{
    /// <summary>A part: a shape at its place.</summary>
    public static ExchangeNode Part(global::Kerf.Topology.Shape shape, string? name = null, (double R, double G, double B, double A)? color = null)
        => new(name, color, shape, null, []);

    /// <summary>An assembly of children at a place.</summary>
    public static ExchangeNode Assembly(IReadOnlyList<ExchangeNode> children, Placement? location = null, string? name = null,
                                       (double R, double G, double B, double A)? color = null)
        => new(name, color, null, location, children);
}

/// <summary>
/// Limits for reading data that may be hostile (bento-integration.md, section 7): what a reader refuses rather than
/// allocate for.
/// </summary>
/// <param name="MaxBytes">The most bytes read from the stream.</param>
/// <param name="MaxEntities">The most STEP records.</param>
/// <param name="MaxDepth">The deepest nesting: STEP's parameter lists, glTF's node hierarchy.</param>
/// <param name="MaxTriangles">The most triangles (STL, glTF).</param>
/// <param name="MaxNodes">The most glTF nodes.</param>
internal sealed record ExchangeReadLimits(long MaxBytes, int MaxEntities, int MaxDepth, long MaxTriangles, int MaxNodes);

/// <summary>Stream helpers for the exchange code.</summary>
internal static class ExchangeIO
{
    /// <summary>Copies a stream, refusing more than <paramref name="maxBytes"/>.</summary>
    internal static void CopyWithin(Stream from, Stream to, long maxBytes, string name, CancellationToken cancellationToken)
    {
        var buffer = new byte[81920];
        long total = 0;
        int n;
        while ((n = from.Read(buffer, 0, buffer.Length)) > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            total += n;
            if (total > maxBytes)
                throw new InvalidDataException($"{name} is larger than the limit of {maxBytes} bytes");
            to.Write(buffer, 0, n);
        }
    }
}
