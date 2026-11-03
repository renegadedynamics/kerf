using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Exchange.ThreeMf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;

using Occt;

namespace Build123d;

// The mesh formats over OCCT.NET's writers and readers (STL: RWStl and StlAPI; glTF and 3MF: the writers over the
// assembly tree XdeDoc_Tree, and the glTF reader), and STEP through STEPCAFControl, with OCCT.NET's limits and
// cancellation.

internal static partial class Extras
{
    /// <summary>Writes a meshed shape as STL (OCCT's <c>StlAPI_Writer</c>): binary, or ASCII with LF line ends.</summary>
    public static bool WriteStl(global::Kerf.Topology.Shape meshed, Stream stream, bool ascii, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var progress = new ProgressIndicator(cancellationToken);
        var writer = new ApiWriter();
        writer.SetASCIIMode(ascii);
        var done = writer.Write(meshed, stream, progress.Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done;
    }

    /// <summary>
    /// A face holding the triangulation read from STL data (OCCT's <c>RWStl::ReadFile</c>, binary or ASCII, nodes
    /// merged), its nodes scaled by <paramref name="scale"/>, within the limits (bytes, triangles).
    /// </summary>
    public static global::Kerf.Topology.Face ReadStl(Stream stream, double scale, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        var progress = new ProgressIndicator(cancellationToken);
        var stlLimits = new global::Kerf.Engine.Exchange.Stl.StlReadLimits
        {
            MaxBytes = limits.MaxBytes,
            MaxTriangles = (int)Math.Min(limits.MaxTriangles, int.MaxValue),
        };
        var mesh = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(stream, Math.PI / 2.0, progress.Start(), stlLimits);
        cancellationToken.ThrowIfCancellationRequested();
        if (mesh is null || mesh.TriangleCount == 0)
            throw new InvalidDataException("The STL data holds no triangles, or is larger than the limits");
        if (scale != 1.0)
        {
            // RWStl's triangulations are published (frozen): scale a copy, as build123d scales the nodes it read
            mesh = mesh.Copy();
            for (int i = 1; i <= mesh.NodeCount; i++)
            {
                var p = mesh.Node(i);
                mesh.SetNode(i, new Point3D(p.X() * scale, p.Y() * scale, p.Z() * scale));
            }
        }
        var face = new global::Kerf.Topology.Face();
        new BrepBuilder().MakeFace(ref face, mesh);
        face.TShape()!.FreezeGraph();
        return face;
    }

    // The placeholder URI of a buffer embedded afterwards, in a glTF written to one stream
    private const string EmbeddedBuffer = "b3d-embedded-buffer.bin";

    /// <summary>
    /// Writes a meshed tree as glTF 2.0 (OCCT.NET's <c>RWGltf_TreeWriter</c>): binary (<c>.glb</c>) to
    /// <paramref name="stream"/>, or JSON with its buffer in <paramref name="buffer"/> as <paramref name="bufferUri"/>,
    /// or embedded as a <c>data:</c> URI if there's no buffer stream (one stream can't carry two files).
    /// </summary>
    public static bool WriteGltf(ExchangeNode root, Stream stream, Stream? buffer, string? bufferUri, bool binary,
                                 double unitInMetres, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(root);
        ArgumentNullException.ThrowIfNull(stream);
        var progress = new ProgressIndicator(cancellationToken);
        var writer = new GltfTreeWriter(new GltfWriteOptions { InputLengthUnit = unitInMetres, Generator = "build123d.NET" });
        var tree = ToTree(root);
        bool done;
        if (binary)
            done = writer.WriteGlb(tree, stream, progress.Start());
        else if (buffer is not null)
            done = writer.WriteGltf(tree, stream, buffer, bufferUri ?? "buffer.bin", progress.Start());
        else
        {
            using var json = new MemoryStream();
            using var bin = new MemoryStream();
            done = writer.WriteGltf(tree, json, bin, EmbeddedBuffer, progress.Start());
            if (done)
            {
                var text = Encoding.UTF8.GetString(json.ToArray());
                if (bin.Length > 0)
                    text = text.Replace($"\"{EmbeddedBuffer}\"", "\"data:application/octet-stream;base64,"
                                        + Convert.ToBase64String(bin.GetBuffer(), 0, (int)bin.Length) + "\"", StringComparison.Ordinal);
                stream.Write(Encoding.UTF8.GetBytes(text));
            }
        }
        cancellationToken.ThrowIfCancellationRequested();
        return done;
    }

    /// <summary>Writes a meshed tree as 3MF (OCCT.NET's <c>RW3mf_TreeWriter</c>, 3MF Core).</summary>
    public static bool Write3mf(ExchangeNode root, Stream stream, double unitInMetres, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(root);
        ArgumentNullException.ThrowIfNull(stream);
        var progress = new ProgressIndicator(cancellationToken);
        var writer = new TreeWriter(new WriteOptions { InputLengthUnit = unitInMetres });
        var done = writer.Write(ToTree(root), stream, progress.Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done;
    }

    /// <summary>
    /// The tree read from glTF 2.0 data (OCCT.NET's <c>RWGltf_TreeReader</c>: <c>.glb</c>, or <c>.gltf</c> with its
    /// buffers embedded), Y-up turned Z-up, in a unit of <paramref name="unitInMetres"/>, within the limits (bytes,
    /// nodes, depth, triangles). A part is a compound of faces holding only triangulations.
    /// </summary>
    public static ExchangeNode ReadGltf(Stream stream, double unitInMetres, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        var progress = new ProgressIndicator(cancellationToken);
        var options = new ReadOptions
        {
            OutputLengthUnit = unitInMetres,
            Limits = ReadLimits.Default with
            {
                MaxBytes = limits.MaxBytes,
                MaxNodes = limits.MaxNodes,
                MaxDepth = limits.MaxDepth,
                MaxTriangles = limits.MaxTriangles,
            },
        };
        var reader = new TreeReader(options);
        var tree = reader.Read(stream, null, progress.Start());
        cancellationToken.ThrowIfCancellationRequested();
        if (tree is null)
            throw new InvalidDataException($"The glTF data couldn't be read: {reader.Error}");
        var roots = tree.Roots.Select(r => FromInstance(r, productNames: false)).ToList();
        return roots.Count == 1 ? roots[0] : ExchangeNode.Assembly(roots);
    }

    /// <summary>
    /// Reads STEP data into a tree (OCCT.NET's <c>STEPCAFControl_Reader</c> into an <c>XdeDoc_Document</c>, frozen
    /// into an <c>XdeDoc_Tree</c>): the free shapes as roots, assemblies with their components placed, names (an
    /// instance's, else its product's) and colours (surface, else generic, else curve), within the limits and
    /// cancellable. <paramref name="root"/> is null when the data holds no shape. STEP data referring to other files
    /// (external references) is refused unless <paramref name="externalFiles"/>, when they are read from the folder of
    /// <paramref name="name"/>, and only from inside it.
    /// </summary>
    public static ReturnStatus ReadStep(string name, Stream stream, ExchangeReadLimits limits, bool externalFiles,
                                                 out ExchangeNode? root, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(name);
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        root = null;
        var stepLimits = new ParserReadLimits { MaxFileSize = limits.MaxBytes, MaxRecords = limits.MaxEntities, MaxDepth = limits.MaxDepth };
        var reader = new CafReader();
        // build123d's modes: colours, names and layers
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);
        // OCCT.NET reads no external file from a stream unless given a resolver (its decision 1285): these are read
        // from the file's folder and refused outside it
        if (externalFiles)
            reader.SetExternFileResolver(CafReader.FolderResolver(Path.GetDirectoryName(Path.GetFullPath(name))!));
        var status = reader.ReadStream(name, stream, new ProgressIndicator(cancellationToken).Start(), stepLimits);
        cancellationToken.ThrowIfCancellationRequested();
        if (status != ReturnStatus.RetDone)
            return (ReturnStatus)status;
        if (!externalFiles)
        {
            // OCCT reads a file's external references from the disk while transferring: never for data from a stream
            var references = new ExternRefs(reader.Reader().WS()!);
            if (references.LoadExternRefs() && references.ExternRefCount > 0)
                throw new InvalidDataException($"{name} refers to {references.ExternRefCount} other STEP file(s), which aren't read from a stream");
        }
        var document = new ExchangeDocument();
        reader.Transfer(document, new ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        var roots = document.ToTree().Roots.Select(r => FromInstance(r, productNames: true)).ToList();
        root = roots.Count switch
        {
            0 => null,
            1 => roots[0],
            _ => ExchangeNode.Assembly(roots),
        };
        return ReturnStatus.RetDone;
    }

    /// <summary>
    /// Writes a tree as STEP (OCCT.NET's <c>STEPCAFControl_Writer</c> over the tree's document, AP214 by default): a
    /// product per part and assembly, the parts of one shape, name and colour written once and placed by each of their
    /// instances, with names and colours. The shapes are written as they are, without OCCT's shape processing, as
    /// OCCT's STEPCAFControl writes them. Cancellable.
    /// </summary>
    public static bool WriteStep(ExchangeNode root, Stream stream, StepWriteSettings settings, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(root);
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(settings);
        // The shapes' unit is the file's (XCAF's length unit, in metres)
        var unit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthUnitByFactorValue(settings.UnitInMillimetres);
        var tree = ToTree(root).WithLengthUnit(new DocumentLengthUnit(global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(unit), settings.UnitInMillimetres / 1000.0));
        var document = ExchangeDocument.FromTree(tree);
        var writer = new CafWriter();
        writer.SetColorMode(true);
        writer.SetNameMode(true);
        writer.SetLayerMode(true);
        var model = writer.Writer().Model();
        var parameters = model.InternalParameters;
        parameters.InitFromStatic(writer.Writer().WS().Statics());
        parameters.WriteUnit = unit;
        parameters.WriteSurfaceCurMode = settings.WritePcurves;
        parameters.WritePrecisionMode = (Parameters.WriteMode_PrecisionMode)settings.PrecisionMode;
        var done = writer.Transfer(document, in parameters, ModelType.AsIs, null,
                                   new ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done && writer.WriteStream(stream) == ReturnStatus.RetDone;
    }

    // The writers' tree: a product per part or assembly, those of the same shape, name and colour shared (as XCAF's
    // AddShape shares a compound's equal shapes), so a part placed many times is written once. Parts of one shape with
    // other names or colours get copies of it: STEP's writer maps shapes to products, so two products of one shape
    // would be written as one, with the last name.
    private static Tree ToTree(ExchangeNode root) => new(ToInstance(root, new Products()));

    private readonly record struct ProductKey(object TShape, Orientation Orientation, string? Name, (double, double, double, double)? Color);

    private sealed class Products
    {
        public readonly Dictionary<ProductKey, DocumentProduct> ByKey = [];
        public readonly HashSet<object> Shapes = new(ReferenceEqualityComparer.Instance);
    }

    private static Instance ToInstance(ExchangeNode node, Products products)
    {
        if (node.Shape is not { } shape)
        {
            var location = node.Location ?? new Placement();
            var assembly = DocumentProduct.Assembly(node.Name, node.Children.Select(child => ToInstance(child, products)));
            if (node.Color is { } c)
                assembly = assembly.WithColor(ColorType.ColorSurf, Rgba(c));
            return new Instance(assembly, location);
        }
        // A part: its shape at the origin, placed by the instance, as XCAF holds a located shape
        var origin = shape.Located(new Placement());
        var key = new ProductKey(origin.TShape()!, origin.Orientation(), node.Name, node.Color);
        if (!products.ByKey.TryGetValue(key, out var product))
        {
            if (!products.Shapes.Add(key.TShape))
                origin = new Copy(origin).Shape();
            product = DocumentProduct.Part(origin, node.Name);
            if (node.Color is { } c)
            {
                // A colour of faces, or of curves for a shape without faces
                bool faces = new Explorer(origin, global::Kerf.Topology.ShapeType.Face).More();
                product = product.WithColor(faces ? ColorType.ColorSurf : ColorType.ColorCurv, Rgba(c));
            }
            products.ByKey[key] = product;
        }
        return new Instance(product, shape.Location());
    }

    private static ColorWithAlpha Rgba((double R, double G, double B, double A) c)
        => new((float)c.R, (float)c.G, (float)c.B, (float)c.A);

    // A node of the tree read: named by its instance, else its product (glTF's nodes name instances), or by its
    // product, else its instance, as build123d names STEP's components (STEP's instance names are the occurrences'
    // identifiers)
    private static ExchangeNode FromInstance(Instance instance, bool productNames)
    {
        var name = productNames ? instance.Product.Name ?? instance.Name : instance.DisplayName;
        var product = instance.Product;
        var colour = instance.Attributes.SurfaceColor ?? instance.Attributes.GenericColor ?? instance.Attributes.CurveColor
                     ?? product.Attributes.SurfaceColor ?? product.Attributes.GenericColor ?? product.Attributes.CurveColor;
        (double, double, double, double)? color = null;
        if (colour is { } rgba)
        {
            var rgb = rgba.GetRGB();
            color = (rgb.Red(), rgb.Green(), rgb.Blue(), rgba.Alpha());
        }
        if (!product.IsAssembly)
        {
            var placed = product.Shape.Moved(instance.Location);
            return ExchangeNode.Part(placed, name, color);
        }
        var children = product.Components.Select(c => FromInstance(c, productNames)).ToList();
        return ExchangeNode.Assembly(children, instance.Location, name, color);
    }
}
