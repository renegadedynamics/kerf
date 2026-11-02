using System.Runtime.CompilerServices;
using System.Text;
using O = global::Occt;

namespace Build123d.Occt;

// The stream forms of the STEP reader and writer, with OCCT.NET's limits and cancellation, and the mesh formats over
// OCCT.NET's writers and readers (STL: RWStl and StlAPI; glTF and 3MF: the writers over the assembly tree XdeDoc_Tree,
// and the glTF reader). The native binding has the same members (src/Build123d.Occt/Runtime/Exchange.Native.cs).

public partial class STEPControl_Reader
{
    /// <summary>
    /// Reads STEP data from a stream, within the limits (OCCT.NET's <c>StepFile_ReadLimits</c>: bytes, records,
    /// nesting) and cancellable while it parses.
    /// </summary>
    public IFSelect_ReturnStatus ReadStream(string name, Stream stream, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        var stepLimits = new O.StepFile_ReadLimits { MaxFileSize = limits.MaxBytes, MaxRecords = limits.MaxEntities, MaxDepth = limits.MaxDepth };
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        try
        {
            var status = Impl.ReadStream(name, stream, progress.Start(), stepLimits);
            cancellationToken.ThrowIfCancellationRequested();
            return (IFSelect_ReturnStatus)status;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>Transfers the roots read, cancellable while it runs.</summary>
    public int TransferRoots(CancellationToken cancellationToken)
    {
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        try
        {
            var n = Impl.TransferRoots(progress.Start());
            cancellationToken.ThrowIfCancellationRequested();
            return n;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }
}

public partial class STEPControl_Writer
{
    /// <summary>
    /// Translates a shape with writing parameters (the unit, pcurves, precision mode: the session's statics, as OCCT
    /// reads them, with these set), cancellable while it runs.
    /// </summary>
    public IFSelect_ReturnStatus Transfer(TopoDS_Shape sh, STEPControl_StepModelType mode, StepWriteSettings settings,
                                          CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(sh);
        ArgumentNullException.ThrowIfNull(settings);
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        try
        {
            var model = Impl.Model();
            var parameters = model.InternalParameters;
            parameters.InitFromStatic(Impl.WS().Statics());
            parameters.WriteUnit = O.UnitsMethods.GetLengthUnitByFactorValue(settings.UnitInMillimetres);
            parameters.WriteSurfaceCurMode = settings.WritePcurves;
            parameters.WritePrecisionMode = (O.DESTEP_Parameters.WriteMode_PrecisionMode)settings.PrecisionMode;
            // The shapes' unit: OCCT takes it from xstep.cascade.unit (millimetres) if the model has none
            model.SetLocalLengthUnit(settings.UnitInMillimetres);
            var status = Impl.Transfer(sh.V, (O.STEPControl_StepModelType)mode, in parameters, true, progress.Start());
            cancellationToken.ThrowIfCancellationRequested();
            return (IFSelect_ReturnStatus)status;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>Writes the STEP model to a stream.</summary>
    public IFSelect_ReturnStatus WriteStream(Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        try
        {
            return (IFSelect_ReturnStatus)Impl.WriteStream(stream);
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }
}

public static partial class Extras
{
    /// <summary>Writes a meshed shape as STL (OCCT's <c>StlAPI_Writer</c>): binary, or ASCII with LF line ends.</summary>
    public static bool WriteStl(TopoDS_Shape meshed, Stream stream, bool ascii, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(meshed);
        ArgumentNullException.ThrowIfNull(stream);
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        try
        {
            var writer = new O.StlAPI_Writer();
            writer.SetASCIIMode(ascii);
            var done = writer.Write(meshed.V, stream, progress.Start());
            cancellationToken.ThrowIfCancellationRequested();
            return done;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>
    /// A face holding the triangulation read from STL data (OCCT's <c>RWStl::ReadFile</c>, binary or ASCII, nodes
    /// merged), its nodes scaled by <paramref name="scale"/>, within the limits (bytes, triangles).
    /// </summary>
    public static TopoDS_Face ReadStl(Stream stream, double scale, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        var stlLimits = new O.RWStl_ReadLimits
        {
            MaxBytes = limits.MaxBytes,
            MaxTriangles = (int)Math.Min(limits.MaxTriangles, int.MaxValue),
        };
        try
        {
            var mesh = O.RWStl.ReadFile(stream, Math.PI / 2.0, progress.Start(), stlLimits);
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
                    mesh.SetNode(i, new O.gp_Pnt(p.X() * scale, p.Y() * scale, p.Z() * scale));
                }
            }
            var face = new O.TopoDS_Face();
            ExchangeInternals.MakeFace(ExchangeInternals.NewBRepBuilder(), ref face, mesh);
            Internals.FreezeGraph(face.TShape()!);
            return new TopoDS_Face(face);
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
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
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        try
        {
            var writer = new O.RWGltf_TreeWriter(new O.RWGltf_WriteOptions { InputLengthUnit = unitInMetres, Generator = "build123d.NET" });
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
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>Writes a meshed tree as 3MF (OCCT.NET's <c>RW3mf_TreeWriter</c>, 3MF Core).</summary>
    public static bool Write3mf(ExchangeNode root, Stream stream, double unitInMetres, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(root);
        ArgumentNullException.ThrowIfNull(stream);
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        try
        {
            var writer = new O.RW3mf_TreeWriter(new O.RW3mf_WriteOptions { InputLengthUnit = unitInMetres });
            var done = writer.Write(ToTree(root), stream, progress.Start());
            cancellationToken.ThrowIfCancellationRequested();
            return done;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
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
        var progress = new O.Message_ProgressIndicator(cancellationToken);
        var options = new O.RWGltf_ReadOptions
        {
            OutputLengthUnit = unitInMetres,
            Limits = O.RWGltf_ReadLimits.Default with
            {
                MaxBytes = limits.MaxBytes,
                MaxNodes = limits.MaxNodes,
                MaxDepth = limits.MaxDepth,
                MaxTriangles = limits.MaxTriangles,
            },
        };
        try
        {
            var reader = new O.RWGltf_TreeReader(options);
            var tree = reader.Read(stream, null, progress.Start());
            cancellationToken.ThrowIfCancellationRequested();
            if (tree is null)
                throw new InvalidDataException($"The glTF data couldn't be read: {reader.Error}");
            var roots = tree.Roots.Select(r => FromInstance(r, productNames: false)).ToList();
            return roots.Count == 1 ? roots[0] : ExchangeNode.Assembly(roots);
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    /// <summary>Whether STEP keeps names, colours and assemblies: on OCCT.NET, through its STEPCAFControl.</summary>
    public static bool StepKeepsAssemblies => true;

    /// <summary>
    /// Reads STEP data into a tree (OCCT.NET's <c>STEPCAFControl_Reader</c> into an <c>XdeDoc_Document</c>, frozen
    /// into an <c>XdeDoc_Tree</c>): the free shapes as roots, assemblies with their components placed, names (an
    /// instance's, else its product's) and colours (surface, else generic, else curve), within the limits and
    /// cancellable. <paramref name="root"/> is null when the data holds no shape. STEP data referring to other files
    /// (external references) is refused unless <paramref name="externalFiles"/>, when they are read from the folder of
    /// <paramref name="name"/>.
    /// </summary>
    public static IFSelect_ReturnStatus ReadStep(string name, Stream stream, ExchangeReadLimits limits, bool externalFiles,
                                                 out ExchangeNode? root, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(name);
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        root = null;
        var stepLimits = new O.StepFile_ReadLimits { MaxFileSize = limits.MaxBytes, MaxRecords = limits.MaxEntities, MaxDepth = limits.MaxDepth };
        try
        {
            var reader = new O.STEPCAFControl_Reader();
            // build123d's modes: colours, names and layers
            reader.SetColorMode(true);
            reader.SetNameMode(true);
            reader.SetLayerMode(true);
            var status = reader.ReadStream(name, stream, new O.Message_ProgressIndicator(cancellationToken).Start(), stepLimits);
            cancellationToken.ThrowIfCancellationRequested();
            if (status != O.IFSelect_ReturnStatus.IFSelect_RetDone)
                return (IFSelect_ReturnStatus)status;
            if (!externalFiles)
            {
                // OCCT reads a file's external references from the disk while transferring: never for data from a stream
                var references = new O.STEPConstruct_ExternRefs(reader.Reader().WS()!);
                if (references.LoadExternRefs() && references.NbExternRefs() > 0)
                    throw new InvalidDataException($"{name} refers to {references.NbExternRefs()} other STEP file(s), which aren't read from a stream");
            }
            var document = new O.XdeDoc_Document();
            reader.Transfer(document, new O.Message_ProgressIndicator(cancellationToken).Start());
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
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
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
        try
        {
            // The shapes' unit is the file's (XCAF's length unit, in metres)
            var unit = O.UnitsMethods.GetLengthUnitByFactorValue(settings.UnitInMillimetres);
            var tree = ToTree(root).WithLengthUnit(new O.XdeDoc_LengthUnit(O.UnitsMethods.DumpLengthUnit(unit), settings.UnitInMillimetres / 1000.0));
            var document = O.XdeDoc_Document.FromTree(tree);
            var writer = new O.STEPCAFControl_Writer();
            writer.SetColorMode(true);
            writer.SetNameMode(true);
            writer.SetLayerMode(true);
            var model = writer.Writer().Model();
            var parameters = model.InternalParameters;
            parameters.InitFromStatic(writer.Writer().WS().Statics());
            parameters.WriteUnit = unit;
            parameters.WriteSurfaceCurMode = settings.WritePcurves;
            parameters.WritePrecisionMode = (O.DESTEP_Parameters.WriteMode_PrecisionMode)settings.PrecisionMode;
            var done = writer.Transfer(document, in parameters, O.STEPControl_StepModelType.STEPControl_AsIs, null,
                                       new O.Message_ProgressIndicator(cancellationToken).Start());
            cancellationToken.ThrowIfCancellationRequested();
            return done && writer.WriteStream(stream) == O.IFSelect_ReturnStatus.IFSelect_RetDone;
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    // The writers' tree: a product per part or assembly, those of the same shape, name and colour shared (as XCAF's
    // AddShape shares a compound's equal shapes), so a part placed many times is written once
    private static O.XdeDoc_Tree ToTree(ExchangeNode root) => new(ToInstance(root, new Dictionary<ProductKey, O.XdeDoc_Product>()));

    private readonly record struct ProductKey(object TShape, O.TopAbs_Orientation Orientation, string? Name, (double, double, double, double)? Color);

    private static O.XdeDoc_Instance ToInstance(ExchangeNode node, Dictionary<ProductKey, O.XdeDoc_Product> products)
    {
        if (node.Shape is not { } shape)
        {
            var location = node.Location?.V ?? new O.TopLoc_Location();
            var assembly = O.XdeDoc_Product.Assembly(node.Name, node.Children.Select(child => ToInstance(child, products)));
            if (node.Color is { } c)
                assembly = assembly.WithColor(O.XdeDoc_ColorType.XdeDoc_ColorSurf, Rgba(c));
            return new O.XdeDoc_Instance(assembly, location);
        }
        // A part: its shape at the origin, placed by the instance, as XCAF holds a located shape
        var origin = shape.V.Located(new O.TopLoc_Location());
        var key = new ProductKey(origin.TShape()!, origin.Orientation(), node.Name, node.Color);
        if (!products.TryGetValue(key, out var product))
        {
            product = O.XdeDoc_Product.Part(origin, node.Name);
            if (node.Color is { } c)
            {
                // A colour of faces, or of curves for a shape without faces
                bool faces = new O.TopExp_Explorer(origin, O.TopAbs_ShapeEnum.TopAbs_FACE).More();
                product = product.WithColor(faces ? O.XdeDoc_ColorType.XdeDoc_ColorSurf : O.XdeDoc_ColorType.XdeDoc_ColorCurv, Rgba(c));
            }
            products[key] = product;
        }
        return new O.XdeDoc_Instance(product, shape.V.Location());
    }

    private static O.Quantity_ColorRGBA Rgba((double R, double G, double B, double A) c)
        => new((float)c.R, (float)c.G, (float)c.B, (float)c.A);

    // A node of the tree read: named by its instance, else its product (glTF's nodes name instances), or by its
    // product, else its instance, as build123d names STEP's components (STEP's instance names are the occurrences'
    // identifiers)
    private static ExchangeNode FromInstance(O.XdeDoc_Instance instance, bool productNames)
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
            return ExchangeNode.Part(new TopoDS_Shape(placed), name, color);
        }
        var children = product.Components.Select(c => FromInstance(c, productNames)).ToList();
        return ExchangeNode.Assembly(children, new TopLoc_Location(instance.Location), name, color);
    }
}

/// <summary>OCCT.NET's internal face builder, reached without reflection (as <see cref="Internals"/>).</summary>
internal static class ExchangeInternals
{
    private const string BRepBuilder = "Occt.BRep_Builder, OcctNet.ModelingData";

    [UnsafeAccessor(UnsafeAccessorKind.Constructor)]
    [return: UnsafeAccessorType(BRepBuilder)]
    internal static extern object NewBRepBuilder();

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "MakeFace")]
    internal static extern void MakeFace([UnsafeAccessorType(BRepBuilder)] object builder, ref O.TopoDS_Face face, O.Poly_Triangulation? triangulation);
}
