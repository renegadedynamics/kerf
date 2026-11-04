// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using System.Collections.Immutable;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// A product of an assembly tree: a shared definition, placed by <see cref="Instance"/>s. Either a part (a
/// shape, as XCAF's simple shape labels) or an assembly (components, as XCAF's assembly labels), with its own
/// attributes and the attributes of its sub-shapes (XCAF's sub-shape labels: colours and names on faces, edges...).
/// </summary>
/// <remarks>
/// Immutable: the <c>With...</c> methods make new products, and products are shared between instances and between
/// trees (a product placed twice is one object, as a label referred to twice in XCAF). A part's shape is frozen when
/// the product is made (decision 1230); an assembly's shape is the compound of its components, made once when first
/// asked for. Thread-safe.
/// </remarks>
internal sealed class DocumentProduct
{
    private readonly Shape _shape;
    private StrongBox<Shape>? _assemblyShape; // published once, without a lock (unit 10I)

    private DocumentProduct(Shape shape,
                           bool isAssembly,
                           ImmutableArray<Instance> components,
                           ImmutableArray<SubShape> subShapes,
                           Attributes attributes)
    {
        _shape = shape;
        IsAssembly = isAssembly;
        Components = components;
        SubShapes = subShapes;
        Attributes = attributes;
    }

    /// <summary>A part: the shape (frozen here, if it isn't yet), its attributes and its sub-shapes' attributes.</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static DocumentProduct Part(in Shape shape,
                                      Attributes? attributes = null,
                                      IEnumerable<SubShape>? subShapes = null)
    {
        if (shape.IsNull)
        {
            throw new ArgumentException("A part needs a shape", nameof(shape));
        }
        shape.TShape()?.FreezeGraph();
        return new DocumentProduct(shape, false, [], subShapes?.ToImmutableArray() ?? [],
                                  attributes ?? global::Kerf.Engine.Exchange.Documents.Attributes.Empty);
    }

    /// <summary>A part with a name.</summary>
    public static DocumentProduct Part(in Shape shape, string? name)
        => Part(shape, global::Kerf.Engine.Exchange.Documents.Attributes.Empty.WithName(name));

    /// <summary>An assembly of the components (possibly none), with its attributes.</summary>
    public static DocumentProduct Assembly(IEnumerable<Instance> components,
                                          Attributes? attributes = null)
        => new(new Shape(), true, components.ToImmutableArray(), [],
               attributes ?? global::Kerf.Engine.Exchange.Documents.Attributes.Empty);

    /// <summary>An assembly with a name.</summary>
    public static DocumentProduct Assembly(string? name, params IEnumerable<Instance> components)
        => Assembly(components, global::Kerf.Engine.Exchange.Documents.Attributes.Empty.WithName(name));

    /// <summary>True for an assembly, false for a part.</summary>
    public bool IsAssembly { get; }

    /// <summary>The components of an assembly (empty for a part).</summary>
    public ImmutableArray<Instance> Components { get; }

    /// <summary>The attributes of the part's sub-shapes (empty for an assembly).</summary>
    public ImmutableArray<SubShape> SubShapes { get; }

    /// <summary>The product's own attributes.</summary>
    public Attributes Attributes { get; }

    /// <summary>The product's name, or null.</summary>
    public string? Name => Attributes.Name;

    /// <summary>
    /// The shape: a part's own, or for an assembly the compound of its components placed (as XCAF's
    /// <c>UpdateAssemblies</c> makes it), frozen. An empty assembly gives an empty compound.
    /// </summary>
    public Shape Shape
    {
        get
        {
            if (!IsAssembly)
            {
                return _shape;
            }
            if (Volatile.Read(ref _assemblyShape) is { } made)
            {
                return made.Value;
            }
            var builder = new BrepBuilder();
            Compound compound = new();
            builder.MakeCompound(ref compound);
            foreach (Instance component in Components)
            {
                Shape sub = component.Shape;
                if (!sub.IsNull)
                {
                    builder.Add(compound, sub);
                }
            }
            compound.TShape()!.FreezeGraph();
            // Racing threads build equal compounds; the first one stored wins
            var box = new StrongBox<Shape>(compound);
            return (Interlocked.CompareExchange(ref _assemblyShape, box, null) ?? box).Value;
        }
    }

    /// <summary>The attributes of a sub-shape (compared with IsSame), or null.</summary>
    public SubShape? FindSubShape(in Shape subShape)
    {
        foreach (SubShape sub in SubShapes)
        {
            if (sub.Shape.IsSame(subShape))
            {
                return sub;
            }
        }
        return null;
    }

    /// <summary>The product with other attributes.</summary>
    public DocumentProduct WithAttributes(Attributes attributes)
        => new(_shape, IsAssembly, Components, SubShapes, attributes);

    /// <summary>The product with another name.</summary>
    public DocumentProduct WithName(string? name) => WithAttributes(Attributes.WithName(name));

    /// <summary>The product with the colour of the type replaced (removed if null).</summary>
    public DocumentProduct WithColor(ColorType type, ColorWithAlpha? color)
        => WithAttributes(Attributes.WithColor(type, color));

    /// <summary>The assembly with other components.</summary>
    /// <exception cref="InvalidOperationException">The product is a part.</exception>
    public DocumentProduct WithComponents(IEnumerable<Instance> components)
        => IsAssembly
             ? new(_shape, true, components.ToImmutableArray(), [], Attributes)
             : throw new InvalidOperationException("A part has no components");

    /// <summary>The part with other sub-shape attributes.</summary>
    /// <exception cref="InvalidOperationException">The product is an assembly.</exception>
    public DocumentProduct WithSubShapes(IEnumerable<SubShape> subShapes)
        => !IsAssembly
             ? new(_shape, false, [], subShapes.ToImmutableArray(), Attributes)
             : throw new InvalidOperationException("An assembly has no sub-shapes");

    /// <summary>
    /// The part with the attributes of a sub-shape set (replacing the sub-shape's entry if it has one, compared with
    /// IsSame). The sub-shape is frozen.
    /// </summary>
    public DocumentProduct WithSubShape(in Shape subShape, Attributes attributes)
    {
        var sub = new SubShape(subShape, attributes);
        var list = SubShapes.ToBuilder();
        for (int i = 0; i < list.Count; i++)
        {
            if (list[i].Shape.IsSame(subShape))
            {
                list[i] = sub;
                return WithSubShapes(list.ToImmutable());
            }
        }
        list.Add(sub);
        return WithSubShapes(list.ToImmutable());
    }

    /// <summary>The product placed at the location, as a new instance.</summary>
    public Instance Placed(in Placement location, Attributes? attributes = null)
        => new(this, location, attributes);

    /// <inheritdoc/>
    public override string ToString() => $"{(IsAssembly ? "Assembly" : "Part")} {Name}";
}
