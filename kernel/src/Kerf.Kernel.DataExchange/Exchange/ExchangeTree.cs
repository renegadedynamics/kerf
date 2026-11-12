// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Exchange;

/// <summary>
/// The public model tree and the engine's assembly tree (XdeDoc's <see cref="Tree"/>, which the STEP, glTF, 3MF and OBJ
/// readers and writers take), with the shared helpers of the format classes: diagnostics, limits on streams.
/// </summary>
internal static class ExchangeTree
{
    /// <summary>A reader's or writer's failure for malformed data: <c>exchange.malformed</c>.</summary>
    public static DiagnosticCode Malformed { get; } = new("exchange.malformed");

    /// <summary>Data past a reader's limits: <c>exchange.limit-exceeded</c>.</summary>
    public static DiagnosticCode LimitExceeded { get; } = new("exchange.limit-exceeded");

    /// <summary>A writer that couldn't write: <c>exchange.write-failed</c>.</summary>
    public static DiagnosticCode WriteFailed { get; } = new("exchange.write-failed");

    /// <summary>STEP data that refers to other files, read from a stream: <c>exchange.external-references</c>.</summary>
    public static DiagnosticCode ExternalReferences { get; } = new("exchange.external-references");

    /// <summary>A warning that entities of the data couldn't be read and were left out: <c>exchange.entity-skipped</c>.</summary>
    public static DiagnosticCode EntitySkipped { get; } = new("exchange.entity-skipped");

    /// <summary>A warning that faces of the model read are invalid: <c>exchange.invalid-face</c> (decision 33502).</summary>
    public static DiagnosticCode InvalidFace { get; } = new("exchange.invalid-face");

    /// <summary>Whether meshing a part for a writer stopped at a limit: the mesh's size, or a face refused (decision 1942).</summary>
    internal static bool IsMeshRefused(Exception e) => e is MeshTooLargeException or MeshFaceRefusedException;

    /// <summary>The exceptions a reader throws on bad data (rather than misuse).</summary>
    public static bool IsBadData(Exception e)
        => (Refusal.IsFailure(e) || e is InvalidDataException or FormatException or EndOfStreamException or OverflowException
                or IndexOutOfRangeException or IOException or InsufficientExecutionStackException);

    /// <summary>The diagnostic of a reader's exception on bad data.</summary>
    public static Diagnostic BadData(Exception e, string format)
        => e is InsufficientExecutionStackException
            ? Diagnostic.Error(LimitExceeded, $"The {format} data is nested too deep to read: its curves, surfaces or representations refer to each other in too long a chain.")
            : e is InvalidDataException
            ? Diagnostic.Error(LimitExceeded, $"The {format} data is larger than the limits: {FailureDiagnostics.PublicText(e)}")
            : Diagnostic.Error(Malformed, $"The {format} data couldn't be read: it is malformed or truncated ({FailureDiagnostics.PublicText(e)}).");

    /// <summary>The stream read to its end, refusing more than <paramref name="maxBytes"/>.</summary>
    /// <exception cref="InvalidDataException">The stream holds more.</exception>
    public static MemoryStream ReadAll(Stream stream, long maxBytes, CancellationToken cancellationToken)
    {
        var memory = new MemoryStream();
        var buffer = new byte[81920];
        long total = 0;
        int n;
        while ((n = stream.Read(buffer, 0, buffer.Length)) > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            total += n;
            if (total > maxBytes)
            {
                throw new InvalidDataException($"more than {maxBytes} bytes");
            }

            memory.Write(buffer, 0, n);
        }

        memory.Position = 0;
        return memory;
    }

    /// <summary>
    /// The engine's tree of a model: a product per part and assembly, the parts of one shape, name and colour shared (as
    /// XCAF's AddShape shares a compound's equal shapes), so a part placed many times is written once. Parts of one
    /// shape with other names or colours get copies of it: STEP's writer maps shapes to products. With
    /// <paramref name="mesh"/>, each part's shape is meshed first (the mesh formats); a mesh part is written as faces
    /// holding its triangles. A part whose faces are meshed further from their surfaces than the deflection adds a
    /// warning to <paramref name="warnings"/> (<c>meshing.deflection-exceeded</c>, decision 2071).
    /// </summary>
    public static Tree ToTree(ModelNode root, MeshOptions? mesh, List<Diagnostic>? warnings = null)
        => new(ToInstance(root, new Products(), mesh, warnings));

    private readonly record struct ProductKey(object TShape, Orientation Orientation, string? Name, Color? Color);

    private sealed class Products
    {
        public readonly Dictionary<ProductKey, DocumentProduct> ByKey = [];
        public readonly HashSet<object> Shapes = new(ReferenceEqualityComparer.Instance);
    }

    private static Instance ToInstance(ModelNode node, Products products, MeshOptions? mesh, List<Diagnostic>? warnings)
    {
        Shape? part = node.Shape;
        if (node.Mesh is { } shapeMesh)
        {
            part = shapeMesh.ToMeshOnlyShape();
        }
        else if (part is { } shape && mesh is not null)
        {
            ToolParameters parameters = mesh.ToEngine();
            part = IncrementalMesh.Meshed(shape, parameters, out _, out ImmutableArray<ExceededFace> exceeded);
            if (parameters.Budget is { RefusedFaces: > 0 and int refused })
            {
                // Kerf: a part written with faces missing would be a success for an invalid result (decision 1942)
                throw new MeshFaceRefusedException($"{refused} {(refused == 1 ? "face" : "faces")} of {node.Name ?? "a part"} couldn't be meshed: "
                                                   + "their mesh would take far more memory than their size and the deflection justify, as a damaged surface or range does.");
            }

            if (!exceeded.IsEmpty)
            {
                warnings?.Add(Mesher.DeflectionWarning(exceeded, mesh.Relative, node.Name ?? "a part"));
            }
        }

        if (part is not { } partShape)
        {
            var assembly = DocumentProduct.Assembly(node.Name, node.Children.Select(child => ToInstance(child, products, mesh, warnings)));
            if (node.Color is { } c)
            {
                assembly = assembly.WithColor(ColorType.ColorSurf, c.ToEngine());
            }

            return new Instance(assembly, node.Placement);
        }

        // A part: its shape at the origin, placed by the instance, as XCAF holds a located shape
        Shape origin = partShape.Located(new Placement());
        var key = new ProductKey(origin.TShape()!, origin.Orientation, node.Name, node.Color);
        if (!products.ByKey.TryGetValue(key, out DocumentProduct? product))
        {
            if (!products.Shapes.Add(key.TShape))
            {
                origin = new Copy(origin).Shape();
            }

            product = DocumentProduct.Part(origin, node.Name);
            if (node.Color is { } c)
            {
                // A colour of faces, or of curves for a shape without faces
                bool faces = new Explorer(origin, ShapeType.Face).More();
                product = product.WithColor(faces ? ColorType.ColorSurf : ColorType.ColorCurv, c.ToEngine());
            }

            products.ByKey[key] = product;
        }

        return new Instance(product, partShape.Placement);
    }

    /// <summary>
    /// The model of an engine tree read: its roots, one as it is or several in an assembly; null for none. Nodes are
    /// named by their products, else their instances (<paramref name="productNames"/>, STEP), or the other way (glTF,
    /// whose nodes name instances); coloured by the instance's surface, generic or curve colour, else the product's.
    /// With <paramref name="meshes"/>, parts are meshes (the mesh formats).
    /// </summary>
    public static ModelNode? FromTree(Tree tree, bool productNames, bool meshes)
    {
        List<ModelNode> roots = [.. tree.Roots.Select(r => FromInstance(r, productNames, meshes, 0))];
        return roots.Count switch
        {
            0 => null,
            1 => roots[0],
            _ => ModelNode.Assembly(roots),
        };
    }

    private static ModelNode FromInstance(Instance instance, bool productNames, bool meshes, int depth)
    {
        if (depth > ModelNode.MaxNesting)
        {
            throw new InvalidDataException($"Its assemblies are nested more than {ModelNode.MaxNesting} deep.");
        }

        string? name = productNames ? instance.Product.Name ?? instance.Name : instance.DisplayName;
        DocumentProduct product = instance.Product;
        ColorWithAlpha? colour = instance.Attributes.SurfaceColor ?? instance.Attributes.GenericColor ?? instance.Attributes.CurveColor
                                 ?? product.Attributes.SurfaceColor ?? product.Attributes.GenericColor ?? product.Attributes.CurveColor;
        Color? color = colour is { } rgba ? Color.FromEngine(rgba) : null;
        if (!product.IsAssembly)
        {
            Shape placed = product.Shape.Moved(instance.Location);
            return meshes
                ? ModelNode.Part(ShapeMesh.FromMeshOnlyShape(placed), name, color)
                : ModelNode.Part(placed, name, color);
        }

        return ModelNode.Assembly(product.Components.Select(c => FromInstance(c, productNames, meshes, depth + 1)), name, instance.Location, color);
    }
}
