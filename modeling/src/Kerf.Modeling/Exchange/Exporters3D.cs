using System.Text;
using Occt;

namespace Kerf.Modeling;

/// <summary>
/// build123d's 3D exporters (<c>exporters3d.py</c>): BREP, STEP, STL and glTF, and 3MF (build123d's <c>Mesher</c>).
/// Each writes a stream, with cancellation; the file forms write a file. None changes the shape it's given: the mesh
/// formats mesh private copies, as <see cref="Shape.Tessellate"/> does.
/// </summary>
/// <remarks>
/// <para>
/// STEP is written as build123d writes it: through an assembly document (OCCT.NET's STEPCAFControl over its XdeDoc
/// tree), a product per part and assembly, named by its label and coloured by its colour. The FILE_NAME header carries
/// the shape's label, the time stamp given and "build123d" as the originating system, as build123d's.
/// </para>
/// <para>
/// STL, glTF and 3MF are written by OCCT.NET's writers. glTF and 3MF keep the assembly tree, its labels and colours.
/// </para>
/// </remarks>
public static class Exporters3D
{
    /// <summary>Writes a shape as BREP (OCCT's text format, with any triangulations). True on success.</summary>
    public static bool ExportBrep(Shape toExport, Stream stream)
    {
        ArgumentNullException.ThrowIfNull(toExport);
        ArgumentNullException.ThrowIfNull(stream);
        NothingToExport(toExport);
        stream.Write(Extras.WriteBRep(toExport.TopoShape, withTriangles: true));
        return true;
    }

    /// <summary>Writes a shape to a BREP file.</summary>
    public static bool ExportBrep(Shape toExport, string filePath) => ToFile(filePath, s => ExportBrep(toExport, s));

    /// <summary>
    /// Writes a shape or assembly as STEP (AP214, as OCCT writes by default), in <paramref name="unit"/>, with or
    /// without pcurves, at a precision mode; <paramref name="timestamp"/> (ISO 8601) replaces the time of writing in the
    /// header, for repeatable output. As build123d's <c>export_step</c>: an assembly (a compound with
    /// <see cref="Compound.AssemblyChildren"/>) is written as one, each child a component at its location, children of
    /// one shape, label and colour written once; labels name the products and colours colour them.
    /// </summary>
    /// <exception cref="InvalidOperationException">The shape is empty, or OCCT couldn't translate it.</exception>
    public static bool ExportStep(Shape toExport, Stream stream, Unit unit = Unit.Millimeter, bool writePcurves = true,
                                  PrecisionMode precisionMode = PrecisionMode.Average, string? timestamp = null,
                                  CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(toExport);
        ArgumentNullException.ThrowIfNull(stream);
        NothingToExport(toExport);
        var settings = new StepWriteSettings(ExchangeUnits.Millimetres(unit, "unit"), writePcurves, (int)precisionMode);
        using var text = new MemoryStream();
        if (!Extras.WriteStep(StepTree(toExport), text, settings, cancellationToken))
            throw new InvalidOperationException("Failed to write STEP file");
        stream.Write(StepHeader.WithFileName(text.ToArray(), toExport.Label.Length > 0 ? toExport.Label : null, timestamp, "build123d"));
        return true;
    }

    /// <summary>Writes a shape to a STEP file.</summary>
    public static bool ExportStep(Shape toExport, string filePath, Unit unit = Unit.Millimeter, bool writePcurves = true,
                                  PrecisionMode precisionMode = PrecisionMode.Average, string? timestamp = null,
                                  CancellationToken cancellationToken = default)
        => ToFile(filePath, s => ExportStep(toExport, s, unit, writePcurves, precisionMode, timestamp, cancellationToken));

    /// <summary>
    /// Writes a shape's mesh as STL, binary or ASCII. <paramref name="tolerance"/> and
    /// <paramref name="angularTolerance"/> are the mesh's deflections (relative to each edge's size, as build123d
    /// meshes).
    /// </summary>
    public static bool ExportStl(Shape toExport, Stream stream, double tolerance = 1e-3, double angularTolerance = 0.1,
                                 bool asciiFormat = false, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(toExport);
        ArgumentNullException.ThrowIfNull(stream);
        NothingToExport(toExport);
        return Extras.WriteStl(toExport.MeshedCopy(tolerance, angularTolerance), stream, asciiFormat, cancellationToken);
    }

    /// <summary>Writes a shape's mesh to an STL file, whose folder must exist.</summary>
    public static bool ExportStl(Shape toExport, string filePath, double tolerance = 1e-3, double angularTolerance = 0.1,
                                 bool asciiFormat = false, CancellationToken cancellationToken = default)
        => ToFile(filePath, s => ExportStl(toExport, s, tolerance, angularTolerance, asciiFormat, cancellationToken));

    /// <summary>
    /// Writes a shape or assembly as glTF 2.0: binary (<c>.glb</c>), or JSON (<c>.gltf</c>) with its buffer embedded
    /// as a <c>data:</c> URI, since one stream can't carry the <c>.bin</c> file beside it. Z-up turned Y-up, and
    /// <paramref name="unit"/> into metres, as glTF wants; a material per colour; a node per assembly child, named by
    /// its label.
    /// </summary>
    public static bool ExportGltf(Shape toExport, Stream stream, Unit unit = Unit.Millimeter, bool binary = false,
                                  double linearDeflection = 0.001, double angularDeflection = 0.1,
                                  CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(toExport);
        ArgumentNullException.ThrowIfNull(stream);
        NothingToExport(toExport);
        var metres = ExchangeUnits.Millimetres(unit, "unit") / 1000.0;
        return Extras.WriteGltf(MeshTree(toExport, linearDeflection, angularDeflection), stream, null, null, binary, metres, cancellationToken);
    }

    /// <summary>
    /// Writes a shape or assembly to a glTF 2.0 file: binary, or JSON with its buffer in a <c>.bin</c> file of the
    /// same name beside it, as build123d's.
    /// </summary>
    public static bool ExportGltf(Shape toExport, string filePath, Unit unit = Unit.Millimeter, bool binary = false,
                                  double linearDeflection = 0.001, double angularDeflection = 0.1,
                                  CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(toExport);
        ArgumentNullException.ThrowIfNull(filePath);
        NothingToExport(toExport);
        if (binary)
            return ToFile(filePath, s => ExportGltf(toExport, s, unit, binary, linearDeflection, angularDeflection, cancellationToken));
        var metres = ExchangeUnits.Millimetres(unit, "unit") / 1000.0;
        var tree = MeshTree(toExport, linearDeflection, angularDeflection);
        var binPath = Path.ChangeExtension(filePath, ".bin");
        using var bin = new MemoryStream();
        var done = ToFile(filePath, s => Extras.WriteGltf(tree, s, bin, Path.GetFileName(binPath), false, metres, cancellationToken));
        if (done && bin.Length > 0)
            File.WriteAllBytes(binPath, bin.ToArray());
        return done;
    }

    /// <summary>
    /// Writes a shape or assembly as 3MF (3MF Core: a mesh object per part, a components object per assembly, a base
    /// material per colour), in <paramref name="unit"/>. build123d writes 3MF through its <c>Mesher</c> (lib3mf);
    /// this is its <c>add_shape</c> and <c>write</c> in one, with <c>add_shape</c>'s deflections.
    /// </summary>
    public static bool Export3mf(Shape toExport, Stream stream, Unit unit = Unit.Millimeter, double linearDeflection = 0.001,
                                 double angularDeflection = 0.1, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(toExport);
        ArgumentNullException.ThrowIfNull(stream);
        NothingToExport(toExport);
        var metres = ExchangeUnits.Millimetres(unit, "unit") / 1000.0;
        return Extras.Write3mf(MeshTree(toExport, linearDeflection, angularDeflection), stream, metres, cancellationToken);
    }

    /// <summary>Writes a shape or assembly to a 3MF file.</summary>
    public static bool Export3mf(Shape toExport, string filePath, Unit unit = Unit.Millimeter, double linearDeflection = 0.001,
                                 double angularDeflection = 0.1, CancellationToken cancellationToken = default)
        => ToFile(filePath, s => Export3mf(toExport, s, unit, linearDeflection, angularDeflection, cancellationToken));

    private static void NothingToExport(Shape shape)
    {
        if (shape.IsEmpty)
            throw new InvalidOperationException($"There is nothing to export: the {shape.GetType().Name} is empty");
    }

    // The data is made in memory and written only if it's whole, so a failure leaves no partial file. The folder must
    // exist, as build123d's export_stl requires.
    private static bool ToFile(string filePath, Func<Stream, bool> write)
    {
        ArgumentNullException.ThrowIfNull(filePath);
        var folder = Path.GetDirectoryName(Path.GetFullPath(filePath));
        if (folder is not null && !Directory.Exists(folder))
            throw new DirectoryNotFoundException($"No such folder: {folder}");
        using var buffer = new MemoryStream();
        if (!write(buffer))
            return false;
        using var file = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.None);
        buffer.Position = 0;
        buffer.CopyTo(file);
        return true;
    }

    // The tree the mesh writers take: an assembly per compound with children, a meshed copy per part, labels and
    // colours as they are (the writers resolve a part's inherited colour).
    private static ExchangeNode MeshTree(Shape shape, double linearDeflection, double angularDeflection)
    {
        string? name = shape.Label.Length > 0 ? shape.Label : null;
        var color = Linear(shape.Color);
        if (shape is Compound { AssemblyChildren.Count: > 0 } assembly)
        {
            var children = assembly.AssemblyChildren.Where(c => !c.IsEmpty)
                                   .Select(c => MeshTree(c, linearDeflection, angularDeflection)).ToList();
            return ExchangeNode.Assembly(children, assembly.TopoShape.Placement, name, color);
        }
        return ExchangeNode.Part(shape.MeshedCopy(linearDeflection, angularDeflection), name, color);
    }

    // The tree STEP is written from, as build123d's _create_xde: an assembly per compound with children, the other
    // shapes as parts, labels and colours as they are
    private static ExchangeNode StepTree(Shape shape)
    {
        string? name = shape.Label.Length > 0 ? shape.Label : null;
        var color = Linear(shape.Color);
        if (shape is Compound { AssemblyChildren.Count: > 0 } assembly)
        {
            var children = assembly.AssemblyChildren.Where(c => !c.IsEmpty).Select(StepTree).ToList();
            return ExchangeNode.Assembly(children, assembly.TopoShape.Placement, name, color);
        }
        return ExchangeNode.Part(shape.TopoShape, name, color);
    }

    private static (double R, double G, double B, double A)? Linear(Color? color)
    {
        if (color is not { } c)
            return null;
        var rgba = c.ToQuantityColorRgba();
        var rgb = rgba.GetRGB();
        return (rgb.Red(), rgb.Green(), rgb.Blue(), rgba.Alpha());
    }
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
