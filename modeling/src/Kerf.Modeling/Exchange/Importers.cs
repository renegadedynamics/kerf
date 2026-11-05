using Kerf.Exchange;

namespace Kerf.Modeling;

/// <summary>
/// build123d's importers (<c>importers.py</c>): shapes from BREP, STEP and STL data, and from glTF (not in build123d).
/// Each reads a stream, with limits for hostile data and cancellation; the file forms open the file and read it as a
/// stream. Data that can't be read is a failed <see cref="Result{T}"/> with the kernel's diagnostic (unit 10M3):
/// <c>exchange.malformed</c> or <c>exchange.invalid-data</c> for data that isn't the format, <c>exchange.limit-exceeded</c>
/// for data past a limit, <c>exchange.external-references</c> for STEP data that refers to other files. A missing file
/// throws <see cref="FileNotFoundException"/>, and a unit that isn't a length <see cref="ArgumentException"/>.
/// </summary>
/// <remarks>STEP keeps its names, colours and assemblies (Kerf's STEPCAFControl).</remarks>
public static class Importers
{
    /// <summary>A shape from BREP data (OCCT's text format).</summary>
    public static Result<Shape> ImportBrep(Stream stream, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var maxBytes = (limits ?? ImportLimits.Default).MaxBytes;
        if (ReadAll(stream, maxBytes, cancellationToken) is not { } bytes)
            return Result<Shape>.Failure(Diagnostic.Error(LimitExceeded, $"The BREP data is larger than the limit of {maxBytes} bytes"));
        return Extras.ReadBRep(bytes);
    }

    /// <summary>A shape from a BREP file.</summary>
    public static Result<Shape> ImportBrep(string fileName, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportBrep(file, limits, cancellationToken);
    }

    /// <summary>
    /// The model of STEP data, as build123d's <c>import_step</c>: the file's assembly tree as compounds whose
    /// <see cref="Assembly.Children"/> are its components, each placed by its location, with the product names
    /// as labels and the colours (a shape's own, else its product's); one free shape is returned as it is, several in a
    /// compound of them. <paramref name="name"/> names the data in OCCT's messages. Data that refers to
    /// other STEP files is refused, as there is no folder to read them from.
    /// </summary>
    public static Result<ModelShape> ImportStep(Stream stream, ImportLimits? limits = null, CancellationToken cancellationToken = default,
                                   string name = "STEP data")
        => ImportStep(stream, limits, cancellationToken, name, externalFiles: false);

    /// <summary>
    /// The model of a STEP file (see the stream form). Other STEP files it refers to are read from its folder, as OCCT
    /// reads them.
    /// </summary>
    public static Result<ModelShape> ImportStep(string fileName, ImportLimits? limits = null, CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportStep(file, limits, cancellationToken, Path.GetFullPath(fileName), externalFiles: true);
    }

    private static Result<ModelShape> ImportStep(Stream stream, ImportLimits? limits, CancellationToken cancellationToken, string name, bool externalFiles)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var options = new StepReadOptions { Limits = (limits ?? ImportLimits.Default).ToReadLimits(), ExternalReferences = externalFiles };
        var read = externalFiles ? Step.Read(name, options, cancellationToken: cancellationToken)
                                 : Step.Read(stream, options, cancellationToken: cancellationToken);
        if (read.Error is { } error)
        {
            // Data refused for its references says so; other data that can't be read says which and why
            if (error.Code.Value is "exchange.external-references")
                return Result<ModelShape>.Failure(read.Diagnostics);
            return Result<ModelShape>.Failure(error with { Message = $"Could not import {name}: it isn't readable STEP data. {error.Message}" });
        }
        var root = read.Value;
        return root.Shape is null && root.Children.IsEmpty ? new Assembly(TopoUtil.EmptyCompound()) : FromModel(root);
    }

    private static ModelShape FromModel(ModelNode node)
    {
        var label = node.Name ?? "";
        Color? color = node.Color;
        if (node.Shape is { } shape)
            return Leaf(shape, label, color);
        // A mesh read (glTF): the faces holding only its triangles, as they were read
        if (node.Mesh is { } mesh)
            return Leaf(mesh.TriangulatedShape, label, color);
        var children = node.Children.Select(FromModel).ToList();
        var assembly = new Assembly(children, label, color);
        return node.Placement.IsIdentity ? assembly : assembly.Located(new Location(node.Placement));
    }

    // The object for a part of the tree: a kernel shape can't hold a label or a colour
    private static ModelShape Leaf(Shape shape, string label, Color? color) => shape.Dim switch
    {
        3 => new Part(shape, label, color),
        2 => new Sketch(shape, label, color),
        1 => new Curve(shape, label, color),
        _ => new Assembly(shape, label, color),
    };

    /// <summary>
    /// A face holding the triangles of STL data (binary or ASCII), with no surface: fast to read and to show, as
    /// build123d's. <paramref name="modelUnit"/> is the data's unit, scaled to millimetres (Blender writes metres).
    ///
    /// </summary>
    public static Result<Face> ImportStl(Stream stream, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                 CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var options = new StlReadOptions
        {
            Limits = (limits ?? ImportLimits.Default).ToReadLimits(),
            Unit = ExchangeUnits.LengthUnit(modelUnit, "model_scale"),
        };
        // The face holding the triangles read, with no surface
        return Stl.Read(stream, options, cancellationToken).Map(mesh => (Face)mesh.TriangulatedShape);
    }

    /// <summary>A face holding the triangles of an STL file.</summary>
    public static Result<Face> ImportStl(string fileName, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                 CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportStl(file, modelUnit, limits, cancellationToken);
    }

    /// <summary>
    /// The model of glTF 2.0 data (<c>.glb</c>, or <c>.gltf</c> with its buffers embedded), Y-up turned Z-up, in
    /// <paramref name="modelUnit"/>: a part is a compound of faces holding only triangles, with the mesh's name and
    /// colour; a scene of several parts or a node hierarchy is an assembly (<see cref="Assembly.Children"/>).
    /// Not in build123d.
    /// </summary>
    public static Result<ModelShape> ImportGltf(Stream stream, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                   CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var options = new GltfReadOptions
        {
            Limits = (limits ?? ImportLimits.Default).ToReadLimits(),
            Unit = ExchangeUnits.LengthUnit(modelUnit, "model_unit"),
        };
        return Gltf.Read(stream, options, cancellationToken).Map(FromModel);
    }

    /// <summary>The model of a glTF 2.0 file. External buffers aren't read.</summary>
    public static Result<ModelShape> ImportGltf(string fileName, Unit modelUnit = Unit.Millimeter, ImportLimits? limits = null,
                                   CancellationToken cancellationToken = default)
    {
        using var file = OpenRead(fileName);
        return ImportGltf(file, modelUnit, limits, cancellationToken);
    }


    internal static FileStream OpenRead(string fileName)
    {
        ArgumentNullException.ThrowIfNull(fileName);
        if (!File.Exists(fileName))
            throw new FileNotFoundException($"No such file: {fileName}", fileName);
        return new FileStream(fileName, FileMode.Open, FileAccess.Read, FileShare.Read);
    }

    private static readonly DiagnosticCode LimitExceeded = new("exchange.limit-exceeded");

    // The stream's bytes, or null past the limit
    private static byte[]? ReadAll(Stream stream, long maxBytes, CancellationToken cancellationToken)
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
                return null;
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

    /// <summary>The kernel's length unit of a build123d length unit; a mass unit is refused, as <see cref="Millimetres"/>.</summary>
    internal static LengthUnit LengthUnit(Unit unit, string parameter)
    {
        Millimetres(unit, parameter);
        return unit switch
        {
            Unit.Micrometer => global::Kerf.Exchange.LengthUnit.Micrometre,
            Unit.Centimeter => global::Kerf.Exchange.LengthUnit.Centimetre,
            Unit.Meter => global::Kerf.Exchange.LengthUnit.Metre,
            Unit.Inch => global::Kerf.Exchange.LengthUnit.Inch,
            Unit.Foot => global::Kerf.Exchange.LengthUnit.Foot,
            _ => global::Kerf.Exchange.LengthUnit.Millimetre,
        };
    }
}
