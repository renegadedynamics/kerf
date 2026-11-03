// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Collections.Immutable;

namespace Occt;

/// <summary>
/// An immutable assembly tree: what an XCAF document holds for data exchange (decision 1166: no OCAF or XCAF),
/// without labels. Its roots are instances (XCAF's free shapes: a product, placed if the free label is a reference);
/// products are shared definitions (parts or assemblies) placed by instances; products, instances and parts'
/// sub-shapes carry names, colours, visibility, layers and validation properties (<see cref="XdeDoc_Attributes"/>).
/// </summary>
/// <remarks>
/// <para>
/// Immutable and thread-safe: the <c>With...</c> methods make new trees sharing the products they don't change.
/// Shapes are frozen. <see cref="XdeDoc_Document"/> gives the same content XCAF's label interface (the shape, colour
/// and layer tools' calls) for code ported from STEPCAFControl: a reader fills a document and freezes it with
/// <see cref="XdeDoc_Document.ToTree"/>, a writer opens a tree with <see cref="XdeDoc_Document.FromTree"/>.
/// </para>
/// <para>
/// <see cref="Flatten"/> gives the placed parts with their resolved styles, for meshing and viewing; the mesh writers
/// (<see cref="RWGltf_TreeWriter"/>, <see cref="RWObj_TreeWriter"/>, <see cref="RW3mf_TreeWriter"/>) and the glTF
/// reader (<see cref="RWGltf_TreeReader"/>) work on trees.
/// </para>
/// </remarks>
public sealed class XdeDoc_Tree
{
    /// <summary>A tree with no root.</summary>
    public static XdeDoc_Tree Empty { get; } = new([]);

    /// <summary>
    /// The tree of the roots, with the layers (in order; layers the attributes name and this list doesn't are added
    /// at the end, visible) and the length unit (null if unknown).
    /// </summary>
    public XdeDoc_Tree(IEnumerable<XdeDoc_Instance> theRoots,
                       IEnumerable<XdeDoc_Layer>? theLayers = null,
                       XdeDoc_LengthUnit? theLengthUnit = null)
    {
        Roots = theRoots.ToImmutableArray();
        LengthUnit = theLengthUnit;

        var aLayers = ImmutableArray.CreateBuilder<XdeDoc_Layer>();
        var aNames = new HashSet<string>(StringComparer.Ordinal);
        foreach (XdeDoc_Layer aLayer in theLayers ?? [])
        {
            if (aNames.Add(aLayer.Name))
            {
                aLayers.Add(aLayer);
            }
        }
        void addUsed(XdeDoc_Attributes theAttributes)
        {
            foreach (string aName in theAttributes.Layers)
            {
                if (aNames.Add(aName))
                {
                    aLayers.Add(new XdeDoc_Layer(aName));
                }
            }
        }
        foreach (XdeDoc_Instance aRoot in Roots)
        {
            addUsed(aRoot.Attributes);
        }
        foreach (XdeDoc_Product aProduct in Products())
        {
            addUsed(aProduct.Attributes);
            foreach (XdeDoc_Instance aComponent in aProduct.Components)
            {
                addUsed(aComponent.Attributes);
            }
            foreach (XdeDoc_SubShape aSub in aProduct.SubShapes)
            {
                addUsed(aSub.Attributes);
            }
        }
        Layers = aLayers.ToImmutable();
    }

    /// <summary>The tree of one root.</summary>
    public XdeDoc_Tree(XdeDoc_Instance theRoot) : this([theRoot]) { }

    /// <summary>A tree of one part: the shape (frozen) with a name, at its place.</summary>
    public static XdeDoc_Tree FromShape(in Shape theShape, string? theName = null)
        => new(new XdeDoc_Instance(XdeDoc_Product.Part(theShape, theName)));

    /// <summary>The roots (XCAF's free shapes).</summary>
    public ImmutableArray<XdeDoc_Instance> Roots { get; }

    /// <summary>The layers.</summary>
    public ImmutableArray<XdeDoc_Layer> Layers { get; }

    /// <summary>The length unit of the shapes, or null if unknown.</summary>
    public XdeDoc_LengthUnit? LengthUnit { get; }

    /// <summary>The tree with other roots.</summary>
    public XdeDoc_Tree WithRoots(IEnumerable<XdeDoc_Instance> theRoots) => new(theRoots, Layers, LengthUnit);

    /// <summary>The tree with a root added at the end.</summary>
    public XdeDoc_Tree WithRoot(XdeDoc_Instance theRoot) => new(Roots.Add(theRoot), Layers, LengthUnit);

    /// <summary>The tree with other layers.</summary>
    public XdeDoc_Tree WithLayers(IEnumerable<XdeDoc_Layer> theLayers) => new(Roots, theLayers, LengthUnit);

    /// <summary>The tree with another length unit.</summary>
    public XdeDoc_Tree WithLengthUnit(XdeDoc_LengthUnit? theUnit) => new(Roots, Layers, theUnit);

    /// <summary>The layer of the name, or null.</summary>
    public XdeDoc_Layer? FindLayer(string theName)
    {
        foreach (XdeDoc_Layer aLayer in Layers)
        {
            if (aLayer.Name == theName)
            {
                return aLayer;
            }
        }
        return null;
    }

    /// <summary>
    /// The distinct products, each once (compared by reference), in depth-first order: a product before its
    /// components' products, roots in order.
    /// </summary>
    public IEnumerable<XdeDoc_Product> Products()
    {
        var aSeen = new HashSet<XdeDoc_Product>(ReferenceEqualityComparer.Instance);
        var aStack = new Stack<XdeDoc_Product>();
        var aResult = new List<XdeDoc_Product>();
        for (int i = Roots.Length - 1; i >= 0; i--)
        {
            aStack.Push(Roots[i].Product);
        }
        while (aStack.Count > 0)
        {
            XdeDoc_Product aProduct = aStack.Pop();
            if (!aSeen.Add(aProduct))
            {
                continue;
            }
            aResult.Add(aProduct);
            for (int i = aProduct.Components.Length - 1; i >= 0; i--)
            {
                aStack.Push(aProduct.Components[i].Product);
            }
        }
        return aResult;
    }

    /// <summary>
    /// The placed parts, in depth-first order: each part once per path from a root, with its world location and its
    /// style (XCAF's presentation rules: a parent's style is inherited, a product's attributes override it, an
    /// instance's override its product's). Hidden parts (by their attributes or a hidden layer) are left out unless
    /// <paramref name="includeHidden"/>; their style then says they are hidden.
    /// </summary>
    public IReadOnlyList<XdeDoc_FlatItem> Flatten(bool includeHidden = false)
    {
        var aResult = new List<XdeDoc_FlatItem>();
        var aPath = new List<XdeDoc_Instance>();
        foreach (XdeDoc_Instance aRoot in Roots)
        {
            flatten(aRoot, new Placement(), XdeDoc_Style.Default, aPath, aResult, includeHidden);
        }
        return aResult;
    }

    private void flatten(XdeDoc_Instance theInstance,
                         Placement theParentLocation,
                         XdeDoc_Style theParentStyle,
                         List<XdeDoc_Instance> thePath,
                         List<XdeDoc_FlatItem> theResult,
                         bool includeHidden)
    {
        XdeDoc_Product aProduct = theInstance.Product;
        Placement aLocation = theParentLocation.Multiplied(theInstance.Location);
        XdeDoc_Style aStyle = theParentStyle.Merged(aProduct.Attributes).Merged(theInstance.Attributes);
        if (IsInHiddenLayer(aProduct.Attributes) || IsInHiddenLayer(theInstance.Attributes))
        {
            aStyle = aStyle with { IsVisible = false };
        }
        if (!aStyle.IsVisible && !includeHidden)
        {
            return;
        }

        thePath.Add(theInstance);
        if (aProduct.IsAssembly)
        {
            foreach (XdeDoc_Instance aComponent in aProduct.Components)
            {
                flatten(aComponent, aLocation, aStyle, thePath, theResult, includeHidden);
            }
        }
        else
        {
            theResult.Add(new XdeDoc_FlatItem(aProduct, [.. thePath], aLocation, aStyle));
        }
        thePath.RemoveAt(thePath.Count - 1);
    }

    /// <summary>True if one of the attributes' layers is hidden.</summary>
    public bool IsInHiddenLayer(XdeDoc_Attributes theAttributes)
    {
        foreach (string aName in theAttributes.Layers)
        {
            if (FindLayer(aName) is { IsVisible: false })
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
    public XdeDoc_Tree? Meshed(in ToolParameters theParameters, ProgressRange theProgress = default)
    {
        var aParts = Products().Where(p => !p.IsAssembly).ToList();
        var aMap = new Dictionary<XdeDoc_Product, XdeDoc_Product>(ReferenceEqualityComparer.Instance);
        using (var aPS = new ProgressScope(theProgress, "Meshing", Math.Max(1, aParts.Count)))
        {
            foreach (XdeDoc_Product aPart in aParts)
            {
                if (!aPS.More())
                {
                    return null;
                }
                aPS.Next();
                Shape aMeshed = IncrementalMesh.Meshed(aPart.Shape, theParameters, out History aHistory);
                if (aMeshed.IsEqual(aPart.Shape))
                {
                    aMap[aPart] = aPart;
                    continue;
                }
                var aSubs = new List<XdeDoc_SubShape>(aPart.SubShapes.Length);
                foreach (XdeDoc_SubShape aSub in aPart.SubShapes)
                {
                    var aModified = aHistory.Modified(aSub.Shape);
                    if (aModified.Count == 0)
                    {
                        aSubs.Add(aSub);
                    }
                    else
                    {
                        foreach (Shape aNew in aModified)
                        {
                            aSubs.Add(new XdeDoc_SubShape(aNew, aSub.Attributes));
                        }
                    }
                }
                aMap[aPart] = XdeDoc_Product.Part(aMeshed, aPart.Attributes, aSubs);
            }
        }
        var aDone = new Dictionary<XdeDoc_Product, XdeDoc_Product>(ReferenceEqualityComparer.Instance);
        XdeDoc_Product remap(XdeDoc_Product theProduct)
        {
            if (aDone.TryGetValue(theProduct, out XdeDoc_Product? aKnown))
            {
                return aKnown;
            }
            XdeDoc_Product aNew;
            if (!theProduct.IsAssembly)
            {
                aNew = aMap[theProduct];
            }
            else
            {
                var aComponents = theProduct.Components.Select(c => remapInstance(c)).ToImmutableArray();
                bool isSame = true;
                for (int i = 0; i < aComponents.Length && isSame; i++)
                {
                    isSame = ReferenceEquals(aComponents[i], theProduct.Components[i]);
                }
                aNew = isSame ? theProduct : theProduct.WithComponents(aComponents);
            }
            aDone[theProduct] = aNew;
            return aNew;
        }
        XdeDoc_Instance remapInstance(XdeDoc_Instance theInstance)
        {
            XdeDoc_Product aProduct = remap(theInstance.Product);
            return ReferenceEquals(aProduct, theInstance.Product) ? theInstance : theInstance.WithProduct(aProduct);
        }
        return new XdeDoc_Tree(Roots.Select(remapInstance), Layers, LengthUnit);
    }
}
