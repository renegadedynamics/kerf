using System.Text;

using Occt;

namespace Build123d;

// The mesh formats over OCCT.NET's writers and readers (STL: RWStl and StlAPI; glTF and 3MF: the writers over the
// assembly tree XdeDoc_Tree, and the glTF reader), and STEP through STEPCAFControl, with OCCT.NET's limits and
// cancellation.

internal static partial class Extras
{
    /// <summary>Writes a meshed shape as STL (OCCT's <c>StlAPI_Writer</c>): binary, or ASCII with LF line ends.</summary>
    public static bool WriteStl(TopoDS_Shape meshed, Stream stream, bool ascii, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var progress = new Message_ProgressIndicator(cancellationToken);
        var writer = new StlAPI_Writer();
        writer.SetASCIIMode(ascii);
        var done = writer.Write(meshed, stream, progress.Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done;
    }

    /// <summary>
    /// A face holding the triangulation read from STL data (OCCT's <c>RWStl::ReadFile</c>, binary or ASCII, nodes
    /// merged), its nodes scaled by <paramref name="scale"/>, within the limits (bytes, triangles).
    /// </summary>
    public static TopoDS_Face ReadStl(Stream stream, double scale, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        var progress = new Message_ProgressIndicator(cancellationToken);
        var stlLimits = new RWStl_ReadLimits
        {
            MaxBytes = limits.MaxBytes,
            MaxTriangles = (int)Math.Min(limits.MaxTriangles, int.MaxValue),
        };
        var mesh = RWStl.ReadFile(stream, Math.PI / 2.0, progress.Start(), stlLimits);
        cancellationToken.ThrowIfCancellationRequested();
        if (mesh is null || mesh.NbTriangles() == 0)
            throw new InvalidDataException("The STL data holds no triangles, or is larger than the limits");
        if (scale != 1.0)
        {
            // RWStl's triangulations are published (frozen): scale a copy, as build123d scales the nodes it read
            mesh = mesh.Copy();
            for (int i = 1; i <= mesh.NbNodes(); i++)
            {
                var p = mesh.Node(i);
                mesh.SetNode(i, new gp_Pnt(p.X() * scale, p.Y() * scale, p.Z() * scale));
            }
        }
        var face = new TopoDS_Face();
        new BRep_Builder().MakeFace(ref face, mesh);
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
        var progress = new Message_ProgressIndicator(cancellationToken);
        var writer = new RWGltf_TreeWriter(new RWGltf_WriteOptions { InputLengthUnit = unitInMetres, Generator = "build123d.NET" });
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
        var progress = new Message_ProgressIndicator(cancellationToken);
        var writer = new RW3mf_TreeWriter(new RW3mf_WriteOptions { InputLengthUnit = unitInMetres });
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
        var progress = new Message_ProgressIndicator(cancellationToken);
        var options = new RWGltf_ReadOptions
        {
            OutputLengthUnit = unitInMetres,
            Limits = RWGltf_ReadLimits.Default with
            {
                MaxBytes = limits.MaxBytes,
                MaxNodes = limits.MaxNodes,
                MaxDepth = limits.MaxDepth,
                MaxTriangles = limits.MaxTriangles,
            },
        };
        var reader = new RWGltf_TreeReader(options);
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
    public static IFSelect_ReturnStatus ReadStep(string name, Stream stream, ExchangeReadLimits limits, bool externalFiles,
                                                 out ExchangeNode? root, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(name);
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        root = null;
        var stepLimits = new StepFile_ReadLimits { MaxFileSize = limits.MaxBytes, MaxRecords = limits.MaxEntities, MaxDepth = limits.MaxDepth };
        var reader = new STEPCAFControl_Reader();
        // build123d's modes: colours, names and layers
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);
        // OCCT.NET reads no external file from a stream unless given a resolver (its decision 1285): these are read
        // from the file's folder and refused outside it
        if (externalFiles)
            reader.SetExternFileResolver(STEPCAFControl_Reader.FolderResolver(Path.GetDirectoryName(Path.GetFullPath(name))!));
        var status = reader.ReadStream(name, stream, new Message_ProgressIndicator(cancellationToken).Start(), stepLimits);
        cancellationToken.ThrowIfCancellationRequested();
        if (status != IFSelect_ReturnStatus.IFSelect_RetDone)
            return (IFSelect_ReturnStatus)status;
        if (!externalFiles)
        {
            // OCCT reads a file's external references from the disk while transferring: never for data from a stream
            var references = new STEPConstruct_ExternRefs(reader.Reader().WS()!);
            if (references.LoadExternRefs() && references.NbExternRefs() > 0)
                throw new InvalidDataException($"{name} refers to {references.NbExternRefs()} other STEP file(s), which aren't read from a stream");
        }
        var document = new XdeDoc_Document();
        reader.Transfer(document, new Message_ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        var roots = document.ToTree().Roots.Select(r => FromInstance(r, productNames: true)).ToList();
        root = roots.Count switch
        {
            0 => null,
            1 => roots[0],
            _ => ExchangeNode.Assembly(roots),
        };
        return IFSelect_ReturnStatus.IFSelect_RetDone;
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
        var unit = UnitsMethods.GetLengthUnitByFactorValue(settings.UnitInMillimetres);
        var tree = ToTree(root).WithLengthUnit(new XdeDoc_LengthUnit(UnitsMethods.DumpLengthUnit(unit), settings.UnitInMillimetres / 1000.0));
        var document = XdeDoc_Document.FromTree(tree);
        var writer = new STEPCAFControl_Writer();
        writer.SetColorMode(true);
        writer.SetNameMode(true);
        writer.SetLayerMode(true);
        var model = writer.Writer().Model();
        var parameters = model.InternalParameters;
        parameters.InitFromStatic(writer.Writer().WS().Statics());
        parameters.WriteUnit = unit;
        parameters.WriteSurfaceCurMode = settings.WritePcurves;
        parameters.WritePrecisionMode = (DESTEP_Parameters.WriteMode_PrecisionMode)settings.PrecisionMode;
        var done = writer.Transfer(document, in parameters, STEPControl_StepModelType.STEPControl_AsIs, null,
                                   new Message_ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done && writer.WriteStream(stream) == IFSelect_ReturnStatus.IFSelect_RetDone;
    }

    // The writers' tree: a product per part or assembly, those of the same shape, name and colour shared (as XCAF's
    // AddShape shares a compound's equal shapes), so a part placed many times is written once. Parts of one shape with
    // other names or colours get copies of it: STEP's writer maps shapes to products, so two products of one shape
    // would be written as one, with the last name.
    private static XdeDoc_Tree ToTree(ExchangeNode root) => new(ToInstance(root, new Products()));

    private readonly record struct ProductKey(object TShape, TopAbs_Orientation Orientation, string? Name, (double, double, double, double)? Color);

    private sealed class Products
    {
        public readonly Dictionary<ProductKey, XdeDoc_Product> ByKey = [];
        public readonly HashSet<object> Shapes = new(ReferenceEqualityComparer.Instance);
    }

    private static XdeDoc_Instance ToInstance(ExchangeNode node, Products products)
    {
        if (node.Shape is not { } shape)
        {
            var location = node.Location ?? new TopLoc_Location();
            var assembly = XdeDoc_Product.Assembly(node.Name, node.Children.Select(child => ToInstance(child, products)));
            if (node.Color is { } c)
                assembly = assembly.WithColor(XdeDoc_ColorType.XdeDoc_ColorSurf, Rgba(c));
            return new XdeDoc_Instance(assembly, location);
        }
        // A part: its shape at the origin, placed by the instance, as XCAF holds a located shape
        var origin = shape.Located(new TopLoc_Location());
        var key = new ProductKey(origin.TShape()!, origin.Orientation(), node.Name, node.Color);
        if (!products.ByKey.TryGetValue(key, out var product))
        {
            if (!products.Shapes.Add(key.TShape))
                origin = new BRepBuilderAPI_Copy(origin).Shape();
            product = XdeDoc_Product.Part(origin, node.Name);
            if (node.Color is { } c)
            {
                // A colour of faces, or of curves for a shape without faces
                bool faces = new TopExp_Explorer(origin, TopAbs_ShapeEnum.TopAbs_FACE).More();
                product = product.WithColor(faces ? XdeDoc_ColorType.XdeDoc_ColorSurf : XdeDoc_ColorType.XdeDoc_ColorCurv, Rgba(c));
            }
            products.ByKey[key] = product;
        }
        return new XdeDoc_Instance(product, shape.Location());
    }

    private static Quantity_ColorRGBA Rgba((double R, double G, double B, double A) c)
        => new((float)c.R, (float)c.G, (float)c.B, (float)c.A);

    // A node of the tree read: named by its instance, else its product (glTF's nodes name instances), or by its
    // product, else its instance, as build123d names STEP's components (STEP's instance names are the occurrences'
    // identifiers)
    private static ExchangeNode FromInstance(XdeDoc_Instance instance, bool productNames)
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
