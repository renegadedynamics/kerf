using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Occt;

namespace Kerf.Modeling;

// The mesh readers over OCCT.NET's (STL: RWStl; glTF: the reader of the assembly tree XdeDoc_Tree), with OCCT.NET's
// limits and cancellation: they give faces holding only triangles, which the kernel's public API doesn't make (its
// readers give meshes, decision 1349's question 59), so they stay here until 10M. STEP and the writers are on
// Kerf.Exchange (unit 10L3).

internal static partial class Extras
{
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
                mesh.SetNode(i, new Point3D(p.X * scale, p.Y * scale, p.Z * scale));
            }
        }
        var face = new global::Kerf.Topology.Face();
        new BrepBuilder().MakeFace(ref face, mesh);
        face.TShape()!.FreezeGraph();
        return face;
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
            Limits = GltfReadLimits.Default with
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
