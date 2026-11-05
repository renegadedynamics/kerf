using System.Text;
using Kerf.Exchange;
using Kerf.Meshing;

namespace Kerf.Modeling;

/// <summary>How <c>ExportStep</c> writes STEP.</summary>
public sealed record StepExportOptions
{
    /// <summary>The options by default: millimetres, with pcurves, at the average precision, stamped with the time of writing.</summary>
    public static StepExportOptions Default { get; } = new();

    /// <summary>The length unit written; a unit that isn't a length throws <see cref="ArgumentException"/> when used.</summary>
    public Unit Unit { get; init; } = Unit.Millimeter;

    /// <summary>Whether edges keep their curves on surfaces.</summary>
    public bool WritePcurves { get; init; } = true;

    /// <summary>The precision the file declares.</summary>
    public PrecisionMode PrecisionMode { get; init; } = PrecisionMode.Average;

    /// <summary>An ISO 8601 time stamp in place of the time of writing in the header, for repeatable output.</summary>
    public string? Timestamp { get; init; }
}

/// <summary>How <c>ExportStl</c> meshes and writes STL.</summary>
public sealed record StlExportOptions
{
    /// <summary>The options by default: binary, at a linear deflection of 0.001 and an angular one of 0.1.</summary>
    public static StlExportOptions Default { get; } = new();

    /// <summary>The mesh's linear deflection, relative to each edge's size, as build123d meshes.</summary>
    public double Tolerance { get; init; } = 1e-3;

    /// <summary>The mesh's angular deflection.</summary>
    public double AngularTolerance { get; init; } = 0.1;

    /// <summary>Whether to write ASCII rather than binary.</summary>
    public bool Ascii { get; init; }
}

/// <summary>How <c>ExportGltf</c> meshes and writes glTF.</summary>
public sealed record GltfExportOptions
{
    /// <summary>The options by default: JSON, from millimetres, at a linear deflection of 0.001 and an angular one of 0.1.</summary>
    public static GltfExportOptions Default { get; } = new();

    /// <summary>The shape's length unit, turned into metres as glTF wants; a unit that isn't a length throws <see cref="ArgumentException"/> when used.</summary>
    public Unit Unit { get; init; } = Unit.Millimeter;

    /// <summary>Whether to write binary (<c>.glb</c>) rather than JSON (<c>.gltf</c>).</summary>
    public bool Binary { get; init; }

    /// <summary>The mesh's linear deflection, relative to each edge's size.</summary>
    public double LinearDeflection { get; init; } = 0.001;

    /// <summary>The mesh's angular deflection.</summary>
    public double AngularDeflection { get; init; } = 0.1;
}

/// <summary>How <c>Export3mf</c> meshes and writes 3MF.</summary>
public sealed record ThreeMfExportOptions
{
    /// <summary>The options by default: millimetres, at a linear deflection of 0.001 and an angular one of 0.1.</summary>
    public static ThreeMfExportOptions Default { get; } = new();

    /// <summary>The length unit written; a unit that isn't a length throws <see cref="ArgumentException"/> when used.</summary>
    public Unit Unit { get; init; } = Unit.Millimeter;

    /// <summary>The mesh's linear deflection, relative to each edge's size.</summary>
    public double LinearDeflection { get; init; } = 0.001;

    /// <summary>The mesh's angular deflection.</summary>
    public double AngularDeflection { get; init; } = 0.1;
}

/// <summary>
/// build123d's 3D exporters (<c>exporters3d.py</c>): BREP, STEP, STL and glTF, and 3MF (build123d's <c>Mesher</c>).
/// Each writes a stream, with cancellation; the file forms write a file. None changes the shape it's given: the mesh
/// formats mesh private copies, as <c>Tessellate</c> does.
/// </summary>
/// <remarks>
/// <para>
/// Each returns a <see cref="Result"/> (unit 10P2), as the importers do: an empty shape is
/// <c>exchange.nothing-done</c>, a mesh that can't be made is the mesher's diagnostic, a writer's failure the exchange's
/// (<c>exchange.write-failed</c>, <c>exchange.failed</c>), and a file or stream that can't be written
/// <c>exchange.write-failed</c>. Misuse throws: a null argument, a unit that isn't a length
/// (<see cref="ArgumentException"/>), a folder that doesn't exist (<see cref="DirectoryNotFoundException"/>, as a
/// missing file is for the importers). Cancellation throws <see cref="OperationCanceledException"/>.
/// </para>
/// <para>
/// STEP is written as build123d writes it: through an assembly document (Kerf's STEPCAFControl over its XdeDoc
/// tree), a product per part and assembly, named by its label and coloured by its colour. The FILE_NAME header carries
/// the shape's label, the time stamp given and "Kerf.Modeling" as the originating system, where build123d names itself.
/// </para>
/// <para>
/// STL, glTF and 3MF are written by Kerf's writers. glTF and 3MF keep the assembly tree, its labels and colours.
/// </para>
/// </remarks>
public static class Exporters3D
{
    private static readonly DiagnosticCode NothingDone = new("exchange.nothing-done");
    private static readonly DiagnosticCode WriteFailed = new("exchange.write-failed");

    /// <summary>Writes a shape as BREP (OCCT's text format, with any triangulations).</summary>
    public static Result ExportBrep(IShape toExport, Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        return Export(toExport, () =>
        {
            stream.Write(Extras.WriteBRep(toExport.AsShape(), withTriangles: true));
            return Result.Success();
        });
    }

    /// <summary>Writes a shape to a BREP file.</summary>
    public static Result ExportBrep(IShape toExport, string filePath) => ToFile(filePath, s => ExportBrep(toExport, s));

    /// <summary>
    /// Writes a shape or assembly as STEP (AP214, as OCCT writes by default). As build123d's <c>export_step</c>: an
    /// assembly (a compound with <see cref="Assembly.Children"/>) is written as one, each child a component at
    /// its location, children of one shape, label and colour written once; labels name the products and colours colour
    /// them.
    /// </summary>
    public static Result ExportStep(IShape toExport, Stream stream, StepExportOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        options ??= StepExportOptions.Default;
        var writeOptions = new StepWriteOptions
        {
            Unit = ExchangeUnits.LengthUnit(options.Unit, "unit"),
            WritePcurves = options.WritePcurves,
            Precision = (StepPrecision)(int)options.PrecisionMode,
        };
        return Export(toExport, () =>
        {
            using var text = new MemoryStream();
            var written = Step.Write(StepTree(toExport), text, writeOptions, cancellationToken: cancellationToken);
            if (written.IsFailure)
                return written;
            stream.Write(StepHeader.WithFileName(text.ToArray(), toExport is ModelShape { Label.Length: > 0 } labelled ? labelled.Label : null, options.Timestamp, "Kerf.Modeling"));
            return written;
        });
    }

    /// <summary>Writes a shape to a STEP file.</summary>
    public static Result ExportStep(IShape toExport, string filePath, StepExportOptions? options = null, CancellationToken cancellationToken = default)
        => ToFile(filePath, s => ExportStep(toExport, s, options, cancellationToken));

    /// <summary>Writes a shape's mesh as STL, binary or ASCII.</summary>
    public static Result ExportStl(IShape toExport, Stream stream, StlExportOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        options ??= StlExportOptions.Default;
        return Export(toExport, () =>
        {
            var mesh = Failures.Get(Mesher.Mesh(toExport.AsShape(), MeshOptions(options.Tolerance, options.AngularTolerance), cancellationToken: cancellationToken),
                                    () => "mesh");
            return Stl.Write(mesh, stream, new StlWriteOptions { Ascii = options.Ascii }, cancellationToken);
        });
    }

    /// <summary>Writes a shape's mesh to an STL file, whose folder must exist.</summary>
    public static Result ExportStl(IShape toExport, string filePath, StlExportOptions? options = null, CancellationToken cancellationToken = default)
        => ToFile(filePath, s => ExportStl(toExport, s, options, cancellationToken));

    /// <summary>
    /// Writes a shape or assembly as glTF 2.0: binary (<c>.glb</c>), or JSON (<c>.gltf</c>) with its buffer embedded
    /// as a <c>data:</c> URI, since one stream can't carry the <c>.bin</c> file beside it. Z-up turned Y-up, and the
    /// unit into metres, as glTF wants; a material per colour; a node per assembly child, named by its label.
    /// </summary>
    public static Result ExportGltf(IShape toExport, Stream stream, GltfExportOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var writeOptions = GltfOptions(options ?? GltfExportOptions.Default);
        return Export(toExport, () => Gltf.Write(MeshTree(toExport), stream, writeOptions, cancellationToken));
    }

    /// <summary>
    /// Writes a shape or assembly to a glTF 2.0 file: binary, or JSON with its buffer in a <c>.bin</c> file of the
    /// same name beside it, as build123d's.
    /// </summary>
    public static Result ExportGltf(IShape toExport, string filePath, GltfExportOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(filePath);
        options ??= GltfExportOptions.Default;
        if (options.Binary)
            return ToFile(filePath, s => ExportGltf(toExport, s, options, cancellationToken));
        var writeOptions = GltfOptions(options);
        var binPath = Path.ChangeExtension(filePath, ".bin");
        using var bin = new MemoryStream();
        var done = ToFile(filePath, s => Export(toExport, () => Gltf.Write(MeshTree(toExport), s, bin, Path.GetFileName(binPath), writeOptions, cancellationToken)));
        if (done.IsFailure || bin.Length == 0)
            return done;
        try
        {
            File.WriteAllBytes(binPath, bin.ToArray());
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(WriteFailed, $"{binPath} couldn't be written: {e.Message}"));
        }
        return done;
    }

    /// <summary>
    /// Writes a shape or assembly as 3MF (3MF Core: a mesh object per part, a components object per assembly, a base
    /// material per colour). build123d writes 3MF through its <c>Mesher</c> (lib3mf); this is its <c>add_shape</c> and
    /// <c>write</c> in one, with <c>add_shape</c>'s deflections.
    /// </summary>
    public static Result Export3mf(IShape toExport, Stream stream, ThreeMfExportOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        options ??= ThreeMfExportOptions.Default;
        var writeOptions = new ThreeMfWriteOptions { Unit = ExchangeUnits.LengthUnit(options.Unit, "unit"), Mesh = MeshOptions(options.LinearDeflection, options.AngularDeflection) };
        return Export(toExport, () => ThreeMf.Write(MeshTree(toExport), stream, writeOptions, cancellationToken));
    }

    /// <summary>Writes a shape or assembly to a 3MF file.</summary>
    public static Result Export3mf(IShape toExport, string filePath, ThreeMfExportOptions? options = null, CancellationToken cancellationToken = default)
        => ToFile(filePath, s => Export3mf(toExport, s, options, cancellationToken));

    // An export's failures as its result: nothing to write, a failure carried up from the kernel, or a stream or
    // file that can't be written
    private static Result Export(IShape shape, Func<Result> write)
    {
        if (shape.IsEmpty)
            return Result.Failure(Diagnostic.Error(NothingDone, $"There is nothing to export: the {shape.GetType().Name} is empty"));
        Result<Result> done;
        try
        {
            done = Failures.Try(write);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(WriteFailed, $"The data couldn't be written: {e.Message}"));
        }
        return done.TryGetValue(out var result) ? result : Result.Failure(done.Diagnostics);
    }

    // The data is made in memory and written only if it's whole, so a failure leaves no partial file. The folder must
    // exist, as build123d's export_stl requires.
    private static Result ToFile(string filePath, Func<Stream, Result> write)
    {
        ArgumentNullException.ThrowIfNull(filePath);
        var folder = Path.GetDirectoryName(Path.GetFullPath(filePath));
        if (folder is not null && !Directory.Exists(folder))
            throw new DirectoryNotFoundException($"No such folder: {folder}");
        using var buffer = new MemoryStream();
        var written = write(buffer);
        if (written.IsFailure)
            return written;
        try
        {
            using var file = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.None);
            buffer.Position = 0;
            buffer.CopyTo(file);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(WriteFailed, $"{filePath} couldn't be written: {e.Message}"));
        }
        return written;
    }

    // The tree the mesh writers take (they mesh each part with the options' deflections, relative to each edge's size,
    // as build123d meshes): the tree STEP is written from
    private static ModelNode MeshTree(IShape shape) => StepTree(shape);

    // The tree STEP is written from, as build123d's _create_xde: an assembly per compound with children, the other
    // shapes as parts, labels and colours as they are (the writers resolve a part's inherited colour)
    private static ModelNode StepTree(IShape shape)
    {
        var model = shape as ModelShape;
        string? name = model is { Label.Length: > 0 } ? model.Label : null;
        var color = model?.Color;
        if (model is Assembly { Children.Count: > 0 } assembly)
        {
            var children = assembly.Children.Where(c => !c.IsEmpty).Select(c => StepTree(c)).ToList();
            return ModelNode.Assembly(children, name, assembly.Shape.Placement, color);
        }
        return ModelNode.Part(shape.AsShape(), name, color);
    }

    private static global::Kerf.Meshing.MeshOptions MeshOptions(double linearDeflection, double angularDeflection) =>
        new() { LinearDeflection = linearDeflection, AngularDeflection = angularDeflection, Relative = true, Parallel = true };

    private static GltfWriteOptions GltfOptions(GltfExportOptions options) =>
        new()
        {
            Binary = options.Binary,
            Unit = ExchangeUnits.LengthUnit(options.Unit, "unit"),
            Mesh = MeshOptions(options.LinearDeflection, options.AngularDeflection),
            Generator = "Kerf",
        };

}

/// <summary>
/// The header of a STEP file: OCCT writes FILE_NAME from the time of writing and its own names; build123d sets the
/// model's name, the time stamp and the originating system. The record is rewritten in the text, as it was when the
/// native backend's header maker wasn't bound.
/// </summary>
internal static class StepHeader
{
    /// <summary>
    /// The STEP text with FILE_NAME's name, time stamp and originating system replaced by those given (null keeps
    /// OCCT's). The other arguments, and the layout, stay as OCCT wrote them.
    /// </summary>
    internal static byte[] WithFileName(byte[] step, string? name, string? timestamp, string? originatingSystem)
    {
        // Part 21 text is 8-bit; Latin-1 maps each byte to one char and back, so nothing else changes
        var text = Encoding.Latin1.GetString(step);
        int start = text.IndexOf("FILE_NAME(", StringComparison.Ordinal);
        int data = text.IndexOf("DATA;", StringComparison.Ordinal);
        if (start < 0 || (data >= 0 && start > data))
            return step;
        var spans = Arguments(text, start + "FILE_NAME(".Length);
        if (spans.Count != 7)
            return step;
        var replacements = new (int Index, string? Value)[] { (5, originatingSystem), (1, timestamp), (0, name) };
        foreach (var (index, value) in replacements)
        {
            if (value is null)
                continue;
            var (from, to) = spans[index];
            text = string.Concat(text.AsSpan(0, from), Encode(value), text.AsSpan(to));
        }
        return Encoding.Latin1.GetBytes(text);
    }

    // The spans of the record's top-level arguments, trimmed of the white space around them
    private static List<(int From, int To)> Arguments(string text, int i)
    {
        var spans = new List<(int, int)>();
        int depth = 0, argument = i;
        bool quoted = false;
        for (; i < text.Length; i++)
        {
            char c = text[i];
            if (quoted)
            {
                if (c == '\'')
                {
                    if (i + 1 < text.Length && text[i + 1] == '\'')
                        i++;
                    else
                        quoted = false;
                }
                continue;
            }
            if (c == '\'')
                quoted = true;
            else if (c == '(')
                depth++;
            else if (c == ')' && depth > 0)
                depth--;
            else if (c is ',' or ')' && depth == 0)
            {
                spans.Add(Trimmed(text, argument, i));
                if (c == ')')
                    return spans;
                argument = i + 1;
            }
        }
        return [];
    }

    private static (int, int) Trimmed(string text, int from, int to)
    {
        while (from < to && char.IsWhiteSpace(text[from]))
            from++;
        while (to > from && char.IsWhiteSpace(text[to - 1]))
            to--;
        return (from, to);
    }

    /// <summary>A Part 21 string: quotes and backslashes doubled, other than printable ASCII as <c>\X2\</c> or <c>\X4\</c>.</summary>
    internal static string Encode(string value)
    {
        var s = new StringBuilder("'");
        foreach (var rune in value.EnumerateRunes())
        {
            int v = rune.Value;
            if (v == '\'')
                s.Append("''");
            else if (v == '\\')
                s.Append("\\\\");
            else if (v is >= 0x20 and < 0x7F)
                s.Append((char)v);
            else if (v <= 0xFFFF)
                s.Append("\\X2\\").Append(v.ToString("X4", System.Globalization.CultureInfo.InvariantCulture)).Append("\\X0\\");
            else
                s.Append("\\X4\\").Append(v.ToString("X8", System.Globalization.CultureInfo.InvariantCulture)).Append("\\X0\\");
        }
        return s.Append('\'').ToString();
    }
}
