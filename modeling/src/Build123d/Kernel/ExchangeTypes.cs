namespace Build123d.Occt;

/// <summary>
/// A node of an assembly tree, as the mesh writers take it and the glTF reader gives it (<see cref="Extras.WriteGltf"/>,
/// <see cref="Extras.Write3mf"/>, <see cref="Extras.ReadGltf"/>): a part (<see cref="Shape"/>, placed by its own
/// location) or an assembly (<see cref="Children"/>, placed by <see cref="Location"/>), with a name and a colour.
/// Shared by both backends: it holds no native state of its own.
/// </summary>
/// <param name="Name">The name, or null.</param>
/// <param name="Color">The colour as linear RGBA (OCCT's <c>Quantity_TOC_RGB</c>), or null to inherit the parent's.</param>
/// <param name="Shape">A part's shape, at its place; null for an assembly.</param>
/// <param name="Location">An assembly's place relative to its parent; null for a part or no move.</param>
/// <param name="Children">An assembly's children.</param>
public sealed record ExchangeNode(string? Name, (double R, double G, double B, double A)? Color, TopoDS_Shape? Shape,
                                  TopLoc_Location? Location, IReadOnlyList<ExchangeNode> Children)
{
    /// <summary>A part: a shape at its place.</summary>
    public static ExchangeNode Part(TopoDS_Shape shape, string? name = null, (double R, double G, double B, double A)? color = null)
        => new(name, color, shape, null, []);

    /// <summary>An assembly of children at a place.</summary>
    public static ExchangeNode Assembly(IReadOnlyList<ExchangeNode> children, TopLoc_Location? location = null, string? name = null,
                                       (double R, double G, double B, double A)? color = null)
        => new(name, color, null, location, children);
}

/// <summary>
/// Limits for reading data that may be hostile (bento-integration.md, section 7): what a reader refuses rather than
/// allocate for. Shared by both backends; each applies what its readers can.
/// </summary>
/// <param name="MaxBytes">The most bytes read from the stream.</param>
/// <param name="MaxEntities">The most STEP records.</param>
/// <param name="MaxDepth">The deepest nesting: STEP's parameter lists, glTF's node hierarchy.</param>
/// <param name="MaxTriangles">The most triangles (STL, glTF).</param>
/// <param name="MaxNodes">The most glTF nodes.</param>
public sealed record ExchangeReadLimits(long MaxBytes, int MaxEntities, int MaxDepth, long MaxTriangles, int MaxNodes);

/// <summary>
/// STEP writing parameters, OCCT's statics: the shapes' length unit, which the file is written in, whether to write
/// pcurves (<c>write.surfacecurve.mode</c>) and the precision mode (<c>write.precision.mode</c>).
/// </summary>
/// <param name="UnitInMillimetres">The unit's size in millimetres (OCCT's <c>write.step.unit</c>, and the shapes' unit).</param>
/// <param name="WritePcurves">Whether to write the edges' curves on their faces.</param>
/// <param name="PrecisionMode">-1 least, 0 average, 1 greatest, 2 session.</param>
public sealed record StepWriteSettings(double UnitInMillimetres = 1.0, bool WritePcurves = true, int PrecisionMode = 0)
{
    /// <summary>OCCT's defaults.</summary>
    public static StepWriteSettings Default { get; } = new();
}

/// <summary>Stream helpers shared by both backends' exchange code.</summary>
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
