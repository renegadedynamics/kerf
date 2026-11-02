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
            var roots = tree.Roots.Select(r => FromInstance(r)).ToList();
            return roots.Count == 1 ? roots[0] : ExchangeNode.Assembly(roots);
        }
        catch (O.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    private static O.XdeDoc_Tree ToTree(ExchangeNode root) => new(ToInstance(root));

    private static O.XdeDoc_Instance ToInstance(ExchangeNode node)
    {
        O.XdeDoc_Product product;
        O.TopLoc_Location location;
        if (node.Shape is { } shape)
        {
            // A part: its shape at the origin, placed by the instance, as XCAF holds a located shape
            location = shape.V.Location();
            product = O.XdeDoc_Product.Part(shape.V.Located(new O.TopLoc_Location()), node.Name);
        }
        else
        {
            location = node.Location?.V ?? new O.TopLoc_Location();
            product = O.XdeDoc_Product.Assembly(node.Name, node.Children.Select(ToInstance));
        }
        if (node.Color is { } c)
            product = product.WithColor(O.XdeDoc_ColorType.XdeDoc_ColorSurf, new O.Quantity_ColorRGBA((float)c.R, (float)c.G, (float)c.B, (float)c.A));
        return new O.XdeDoc_Instance(product, location);
    }

    private static ExchangeNode FromInstance(O.XdeDoc_Instance instance)
    {
        var product = instance.Product;
        var colour = instance.Attributes.SurfaceColor ?? instance.Attributes.GenericColor
                     ?? product.Attributes.SurfaceColor ?? product.Attributes.GenericColor;
        (double, double, double, double)? color = null;
        if (colour is { } rgba)
        {
            var rgb = rgba.GetRGB();
            color = (rgb.Red(), rgb.Green(), rgb.Blue(), rgba.Alpha());
        }
        if (!product.IsAssembly)
        {
            var placed = product.Shape.Moved(instance.Location);
            return ExchangeNode.Part(new TopoDS_Shape(placed), instance.DisplayName, color);
        }
        var children = product.Components.Select(c => FromInstance(c)).ToList();
        return ExchangeNode.Assembly(children, new TopLoc_Location(instance.Location), instance.DisplayName, color);
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
