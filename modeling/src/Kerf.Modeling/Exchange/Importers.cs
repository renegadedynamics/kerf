using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Modeling;

/// <summary>
/// build123d's importers (<c>importers.py</c>): shapes from BREP, STEP and STL data, and from glTF (not in build123d).
/// Each reads a stream, with limits for hostile data and cancellation; the file forms open the file and read it as a
/// stream. Bad data throws <see cref="ArgumentException"/> (build123d's ValueError) or, for BREP, OCCT's
/// <see cref="FailureException"/> (<c>Storage_StreamFormatError</c>); data past a limit throws
/// <see cref="InvalidDataException"/>.
/// </summary>
/// <remarks>STEP keeps its names, colours and assemblies (OCCT.NET's STEPCAFControl).</remarks>
public static class Importers
{
    /// <summary>A shape from BREP data (OCCT's text format).</summary>
    public static Shape ImportBrep(Stream stream, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var bytes = ReadAll(stream, (limits ?? ImportLimits.Default).MaxBytes, "The BREP data", cancellationToken);
        return Shape.Cast(Extras.ReadBRep(bytes));
    }

    /// <summary>A shape from a BREP file.</summary>
    public static Shape ImportBrep(string fileName, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportBrep(file, limits, cancellationToken);
    }

    /// <summary>
    /// The model of STEP data, as build123d's <c>import_step</c>: the file's assembly tree as compounds whose
    /// <see cref="Compound.AssemblyChildren"/> are its components, each placed by its location, with the product names
    /// as labels and the colours (a shape's own, else its product's); one free shape is returned as it is, several in a
    /// compound of them. <paramref name="name"/> names the data in OCCT's messages. Data that refers to
    /// other STEP files is refused, as there is no folder to read them from.
    /// </summary>
    public static Shape ImportStep(Stream stream, ImportLimits? limits = null, CancellationToken cancellationToken = default,
                                   string name = "STEP data")
        => ImportStep(stream, limits, cancellationToken, name, externalFiles: false);

    /// <summary>
    /// The model of a STEP file (see the stream form). Other STEP files it refers to are read from its folder, as OCCT
    /// reads them.
    /// </summary>
    public static Shape ImportStep(string fileName, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportStep(file, limits, cancellationToken, Path.GetFullPath(fileName), externalFiles: true);
    }

    private static Shape ImportStep(Stream stream, ImportLimits? limits, CancellationToken cancellationToken, string name, bool externalFiles)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var exchangeLimits = (limits ?? ImportLimits.Default).ToExchange();
        var status = Extras.ReadStep(name, stream, exchangeLimits, externalFiles, out var root, cancellationToken);
        if (ExchangeDiagnostics.ToDiagnostic(status) is { } error)
            throw new ArgumentException($"Could not import {name}: it isn't readable STEP data. {error.Message}", new KerfException(error));
        return root is null ? new Compound() : Shape.Result(FromNode(root));
    }

    /// <summary>
    /// A face holding the triangles of STL data (binary or ASCII), with no surface: fast to read and to show, as
    /// build123d's. <paramref name="modelUnit"/> is the data's unit, scaled to millimetres (Blender writes metres).
    ///
    /// </summary>
    public static Face ImportStl(Stream stream, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                 CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var scale = ExchangeUnits.Millimetres(modelUnit, "model_scale");
        return new Face(Extras.ReadStl(stream, scale, (limits ?? ImportLimits.Default).ToExchange(), cancellationToken));
    }

    /// <summary>A face holding the triangles of an STL file.</summary>
    public static Face ImportStl(string fileName, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                 CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportStl(file, modelUnit, limits, cancellationToken);
    }

    /// <summary>
    /// The model of glTF 2.0 data (<c>.glb</c>, or <c>.gltf</c> with its buffers embedded), Y-up turned Z-up, in
    /// <paramref name="modelUnit"/>: a part is a compound of faces holding only triangles, with the mesh's name and
    /// colour; a scene of several parts or a node hierarchy is an assembly (<see cref="Compound.AssemblyChildren"/>).
    /// Not in build123d.
    /// </summary>
    public static Shape ImportGltf(Stream stream, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                   CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var metres = ExchangeUnits.Millimetres(modelUnit, "model_unit") / 1000.0;
        var root = Extras.ReadGltf(stream, metres, (limits ?? ImportLimits.Default).ToExchange(), cancellationToken);
        return FromNode(root);
    }

    /// <summary>The model of a glTF 2.0 file. External buffers aren't read.</summary>
    public static Shape ImportGltf(string fileName, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                   CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportGltf(file, modelUnit, limits, cancellationToken);
    }

    private static Shape FromNode(ExchangeNode node)
    {
        var label = node.Name ?? "";
        Color? color = null;
        if (node.Color is { } c)
        {
            var rgb = new global::Kerf.Engine.Foundation.Color(c.R, c.G, c.B, ColorSpace.TocRgb);
            var rgba = new ColorWithAlpha(rgb, (float)c.A);
            color = new Color(rgba);
        }
        if (node.Shape is { } shape)
            return Shape.Cast(shape).WithLabel(label).WithColor(color);
        var children = node.Children.Select(FromNode).ToList();
        var assembly = Compound.Assembly(children, label, color);
        return node.Location is { } location ? assembly.Located(new Location(location)) : assembly;
    }

    internal static FileStream OpenRead(string fileName)
    {
        ArgumentNullException.ThrowIfNull(fileName);
        if (!File.Exists(fileName))
            throw new FileNotFoundException($"No such file: {fileName}", fileName);
        return new FileStream(fileName, FileMode.Open, FileAccess.Read, FileShare.Read);
    }

    private static byte[] ReadAll(Stream stream, long maxBytes, string name, CancellationToken cancellationToken)
    {
        using var memory = new MemoryStream();
        var buffer = new byte[81920];
        long total = 0;
        int n;
        while ((n = stream.Read(buffer, 0, buffer.Length)) > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            total += n;
            if (total > maxBytes)
                throw new InvalidDataException($"{name} is larger than the limit of {maxBytes} bytes");
            memory.Write(buffer, 0, n);
        }
        return memory.ToArray();
    }
}

/// <summary>build123d's length units in millimetres, for the importers and exporters.</summary>
internal static class ExchangeUnits
{
    /// <summary>The size of a length unit in millimetres; a mass unit is refused, naming the parameter as build123d does.</summary>
    internal static double Millimetres(Unit unit, string parameter) => unit switch
    {
        Unit.Micrometer => 0.001,
        Unit.Millimeter => 1.0,
        Unit.Centimeter => 10.0,
        Unit.Meter => 1000.0,
        Unit.Inch => 25.4,
        Unit.Foot => 304.8,
        _ => throw new ArgumentException($"{parameter} must be one of: Micrometer, Millimeter, Centimeter, Meter, Inch, Foot"),
    };
}
