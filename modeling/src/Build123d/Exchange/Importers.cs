using Build123d.Occt;

namespace Build123d;

/// <summary>
/// build123d's importers (<c>importers.py</c>): shapes from BREP, STEP and STL data, and from glTF (not in build123d).
/// Each reads a stream, with limits for hostile data and cancellation; the file forms open the file and read it as a
/// stream. Bad data throws <see cref="ArgumentException"/> (build123d's ValueError) or, for BREP, OCCT's
/// <see cref="OcctException"/> (<c>Storage_StreamFormatError</c>); data past a limit throws
/// <see cref="InvalidDataException"/>.
/// </summary>
/// <remarks>
/// STEP is read without names, colours or assemblies for now: a model's shapes come back as one shape, or a compound
/// of them (OCCT's <c>STEPControl_Reader</c>). STL and glTF need the OCCT.NET backend.
/// </remarks>
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
    /// The shapes of STEP data: the one shape, or a compound of them. <paramref name="name"/> names the data in OCCT's
    /// messages.
    /// </summary>
    public static Shape ImportStep(Stream stream, ImportLimits? limits = null, CancellationToken cancellationToken = default,
                                   string name = "STEP data")
    {
        ArgumentNullException.ThrowIfNull(stream);
        using var reader = new STEPControl_Reader();
        var status = reader.ReadStream(name, stream, (limits ?? ImportLimits.Default).ToExchange(), cancellationToken);
        if (status != IFSelect_ReturnStatus.IFSelect_RetDone)
            throw new ArgumentException($"Could not import {name}: it isn't readable STEP data ({status})");
        reader.TransferRoots(cancellationToken);
        var shape = reader.OneShape();
        if (shape.IsNull())
            return new Compound();
        return Shape.Result(Shape.Cast(shape));
    }

    /// <summary>The shapes of a STEP file: the one shape, or a compound of them.</summary>
    public static Shape ImportStep(string fileName, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportStep(file, limits, cancellationToken, fileName);
    }

    /// <summary>
    /// A face holding the triangles of STL data (binary or ASCII), with no surface: fast to read and to show, as
    /// build123d's. <paramref name="modelUnit"/> is the data's unit, scaled to millimetres (Blender writes metres).
    /// Needs the OCCT.NET backend.
    /// </summary>
    public static Face ImportStl(Stream stream, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                 CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var scale = ExchangeUnits.Millimetres(modelUnit, "model_scale");
        return new Face(Extras.ReadStl(stream, scale, (limits ?? ImportLimits.Default).ToExchange(), cancellationToken));
    }

    /// <summary>A face holding the triangles of an STL file. Needs the OCCT.NET backend.</summary>
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
    /// Not in build123d. Needs the OCCT.NET backend.
    /// </summary>
    public static Shape ImportGltf(Stream stream, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                   CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var metres = ExchangeUnits.Millimetres(modelUnit, "model_unit") / 1000.0;
        var root = Extras.ReadGltf(stream, metres, (limits ?? ImportLimits.Default).ToExchange(), cancellationToken);
        return FromNode(root);
    }

    /// <summary>The model of a glTF 2.0 file. External buffers aren't read. Needs the OCCT.NET backend.</summary>
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
            using var rgb = new Quantity_Color(c.R, c.G, c.B, Quantity_TypeOfColor.Quantity_TOC_RGB);
            using var rgba = new Quantity_ColorRGBA(rgb, (float)c.A);
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
