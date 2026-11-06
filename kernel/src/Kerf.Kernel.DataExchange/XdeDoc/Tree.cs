// New in Kerf (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Exchange.Obj;
using Kerf.Engine.Exchange.ThreeMf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Collections.Immutable;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// An immutable assembly tree: what an XCAF document holds for data exchange (decision 1166: no OCAF or XCAF),
/// without labels. Its roots are instances (XCAF's free shapes: a product, placed if the free label is a reference);
/// products are shared definitions (parts or assemblies) placed by instances; products, instances and parts'
/// sub-shapes carry names, colours, visibility, layers and validation properties (<see cref="Attributes"/>).
/// </summary>
/// <remarks>
/// <para>
/// Immutable and thread-safe: the <c>With...</c> methods make new trees sharing the products they don't change.
/// Shapes are frozen. <see cref="ExchangeDocument"/> gives the same content XCAF's label interface (the shape, colour
/// and layer tools' calls) for code ported from STEPCAFControl: a reader fills a document and freezes it with
/// <see cref="ExchangeDocument.ToTree"/>, a writer opens a tree with <see cref="ExchangeDocument.FromTree"/>.
/// </para>
/// <para>
/// <see cref="Flatten"/> gives the placed parts with their resolved styles, for meshing and viewing; the mesh writers
/// (<see cref="GltfTreeWriter"/>, <see cref="ObjTreeWriter"/>, <see cref="TreeWriter"/>) and the glTF
/// reader (<see cref="TreeReader"/>) work on trees.
/// </para>
/// </remarks>
internal sealed class Tree
{
    /// <summary>A tree with no root.</summary>
    public static Tree Empty { get; } = new([]);

    /// <summary>
    /// The tree of the roots, with the layers (in order; layers the attributes name and this list doesn't are added
    /// at the end, visible) and the length unit (null if unknown).
    /// </summary>
    public Tree(IEnumerable<Instance> roots,
                       IEnumerable<Layer>? layers = null,
                       DocumentLengthUnit? lengthUnit = null)
    {
        Roots = roots.ToImmutableArray();
        LengthUnit = lengthUnit;

        var aLayers = ImmutableArray.CreateBuilder<Layer>();
        var names = new HashSet<string>(StringComparer.Ordinal);
        foreach (Layer layer in layers ?? [])
        {
            if (names.Add(layer.Name))
            {
                aLayers.Add(layer);
            }
        }
        void AddUsed(Attributes attributes)
        {
            foreach (string name in attributes.Layers)
            {
                if (names.Add(name))
                {
                    aLayers.Add(new Layer(name));
                }
            }
        }
        foreach (Instance root in Roots)
        {
            AddUsed(root.Attributes);
        }
        foreach (DocumentProduct product in Products())
        {
            AddUsed(product.Attributes);
            foreach (Instance component in product.Components)
            {
                AddUsed(component.Attributes);
            }
            foreach (SubShape sub in product.SubShapes)
            {
                AddUsed(sub.Attributes);
            }
        }
        Layers = aLayers.ToImmutable();
    }

    /// <summary>The tree of one root.</summary>
    public Tree(Instance root) : this([root]) { }

    /// <summary>A tree of one part: the shape (frozen) with a name, at its place.</summary>
    public static Tree FromShape(in Shape shape, string? name = null)
        => new(new Instance(DocumentProduct.Part(shape, name)));

    /// <summary>The roots (XCAF's free shapes).</summary>
    public ImmutableArray<Instance> Roots { get; }

    /// <summary>The layers.</summary>
    public ImmutableArray<Layer> Layers { get; }

    /// <summary>The length unit of the shapes, or null if unknown.</summary>
    public DocumentLengthUnit? LengthUnit { get; }

    /// <summary>The tree with other roots.</summary>
    public Tree WithRoots(IEnumerable<Instance> roots) => new(roots, Layers, LengthUnit);

    /// <summary>The tree with a root added at the end.</summary>
    public Tree WithRoot(Instance root) => new(Roots.Add(root), Layers, LengthUnit);

    /// <summary>The tree with other layers.</summary>
    public Tree WithLayers(IEnumerable<Layer> layers) => new(Roots, layers, LengthUnit);

    /// <summary>The tree with another length unit.</summary>
    public Tree WithLengthUnit(DocumentLengthUnit? unit) => new(Roots, Layers, unit);

    /// <summary>The layer of the name, or null.</summary>
    public Layer? FindLayer(string name)
    {
        foreach (Layer layer in Layers)
        {
            if (layer.Name == name)
            {
                return layer;
            }
        }
        return null;
    }

    /// <summary>
    /// The distinct products, each once (compared by reference), in depth-first order: a product before its
    /// components' products, roots in order.
    /// </summary>
    public IEnumerable<DocumentProduct> Products()
    {
        var seen = new HashSet<DocumentProduct>(ReferenceEqualityComparer.Instance);
        var stack = new Stack<DocumentProduct>();
        var result = new List<DocumentProduct>();
        for (int i = Roots.Length - 1; i >= 0; i--)
        {
            stack.Push(Roots[i].Product);
        }
        while (stack.Count > 0)
        {
            DocumentProduct product = stack.Pop();
            if (!seen.Add(product))
            {
                continue;
            }
            result.Add(product);
            for (int i = product.Components.Length - 1; i >= 0; i--)
            {
                stack.Push(product.Components[i].Product);
            }
        }
        return result;
    }

    /// <summary>
    /// The placed parts, in depth-first order: each part once per path from a root, with its world location and its
    /// style (XCAF's presentation rules: a parent's style is inherited, a product's attributes override it, an
    /// instance's override its product's). Hidden parts (by their attributes or a hidden layer) are left out unless
    /// <paramref name="includeHidden"/>; their style then says they are hidden.
    /// </summary>
    public IReadOnlyList<FlatItem> Flatten(bool includeHidden = false)
    {
        var result = new List<FlatItem>();
        var path = new List<Instance>();
        foreach (Instance root in Roots)
        {
            FlattenCore(root, new Placement(), Style.Default, path, result, includeHidden);
        }
        return result;
    }

    private void FlattenCore(Instance instance,
                         Placement parentLocation,
                         Style parentStyle,
                         List<Instance> path,
                         List<FlatItem> result,
                         bool includeHidden)
    {
        DocumentProduct product = instance.Product;
        Placement location = parentLocation.Multiplied(instance.Location);
        Style style = parentStyle.Merged(product.Attributes).Merged(instance.Attributes);
        if (IsInHiddenLayer(product.Attributes) || IsInHiddenLayer(instance.Attributes))
        {
            style = style with { IsVisible = false };
        }
        if (!style.IsVisible && !includeHidden)
        {
            return;
        }

        path.Add(instance);
        if (product.IsAssembly)
        {
            foreach (Instance component in product.Components)
            {
                FlattenCore(component, location, style, path, result, includeHidden);
            }
        }
        else
        {
            result.Add(new FlatItem(product, [.. path], location, style));
        }
        path.RemoveAt(path.Count - 1);
    }

    /// <summary>True if one of the attributes' layers is hidden.</summary>
    public bool IsInHiddenLayer(Attributes attributes)
    {
        foreach (string name in attributes.Layers)
        {
            if (FindLayer(name) is { IsVisible: false })
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// The tree with every part meshed (<see cref="IncrementalMesh.Meshed(in Shape, in ToolParameters, out History)"/>),
    /// sharing what meshing leaves unchanged: faces already meshed, and products whose faces all are. Sub-shape
    /// attributes follow their sub-shapes into the meshed parts. Null if cancelled.
    /// </summary>
    public Tree? Meshed(in ToolParameters parameters, ProgressRange progress = default)
    {
        var parts = Products().Where(p => !p.IsAssembly).ToList();
        var map = new Dictionary<DocumentProduct, DocumentProduct>(ReferenceEqualityComparer.Instance);
        using (var ps = new ProgressScope(progress, "Meshing", Math.Max(1, parts.Count)))
        {
            foreach (DocumentProduct part in parts)
            {
                if (!ps.More())
                {
                    return null;
                }
                ps.Next();
                Shape meshed = IncrementalMesh.Meshed(part.Shape, parameters, out History history);
                if (meshed.IsEqual(part.Shape))
                {
                    map[part] = part;
                    continue;
                }
                var subs = new List<SubShape>(part.SubShapes.Length);
                foreach (SubShape sub in part.SubShapes)
                {
                    var modified = history.Modified(sub.Shape);
                    if (modified.Count == 0)
                    {
                        subs.Add(sub);
                    }
                    else
                    {
                        foreach (Shape aNew in modified)
                        {
                            subs.Add(new SubShape(aNew, sub.Attributes));
                        }
                    }
                }
                map[part] = DocumentProduct.Part(meshed, part.Attributes, subs);
            }
        }
        var done = new Dictionary<DocumentProduct, DocumentProduct>(ReferenceEqualityComparer.Instance);
        DocumentProduct Remap(DocumentProduct product)
        {
            if (done.TryGetValue(product, out DocumentProduct? known))
            {
                return known;
            }
            DocumentProduct aNew;
            if (!product.IsAssembly)
            {
                aNew = map[product];
            }
            else
            {
                var components = product.Components.Select(c => RemapInstance(c)).ToImmutableArray();
                bool isSame = true;
                for (int i = 0; i < components.Length && isSame; i++)
                {
                    isSame = ReferenceEquals(components[i], product.Components[i]);
                }
                aNew = isSame ? product : product.WithComponents(components);
            }
            done[product] = aNew;
            return aNew;
        }
        Instance RemapInstance(Instance instance)
        {
            DocumentProduct aProduct = Remap(instance.Product);
            return ReferenceEquals(aProduct, instance.Product) ? instance : instance.WithProduct(aProduct);
        }
        return new Tree(Roots.Select(RemapInstance), Layers, LengthUnit);
    }
}
