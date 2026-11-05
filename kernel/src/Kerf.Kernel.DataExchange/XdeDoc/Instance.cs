// New in Kerf (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// An instance of a product: the product placed at a location, as XCAF's reference labels (an assembly's components,
/// or a free shape placed at the top). It has attributes of its own (XCAF's on the component label), which override
/// the product's for this placement only.
/// </summary>
/// <remarks>Immutable; shares its product.</remarks>
internal sealed class Instance
{
    /// <summary>The product placed at the location, with the instance's own attributes.</summary>
    public Instance(DocumentProduct product, in Placement location, Attributes? attributes = null)
    {
        Product = product ?? throw new ArgumentNullException(nameof(product));
        Location = location;
        Attributes = attributes ?? global::Kerf.Engine.Exchange.Documents.Attributes.Empty;
    }

    /// <summary>The product at its own place (identity location), with no attribute of its own.</summary>
    public Instance(DocumentProduct product) : this(product, new Placement()) { }

    /// <summary>The product.</summary>
    public DocumentProduct Product { get; }

    /// <summary>The location, in the parent's coordinates (the world's for a root).</summary>
    public Placement Location { get; }

    /// <summary>The instance's own attributes (override the product's).</summary>
    public Attributes Attributes { get; }

    /// <summary>The instance's own name, or null (see <see cref="DisplayName"/>).</summary>
    public string? Name => Attributes.Name;

    /// <summary>The instance's name, or the product's if it has none.</summary>
    public string? DisplayName => Attributes.Name ?? Product.Name;

    /// <summary>The product's shape moved to the location (XCAF's <c>GetShape</c> of a reference).</summary>
    public Shape Shape
    {
        get
        {
            Shape shape = Product.Shape;
            return shape.IsNull || Location.IsIdentity ? shape : shape.Moved(Location);
        }
    }

    /// <summary>The instance with another location.</summary>
    public Instance WithLocation(in Placement location) => new(Product, location, Attributes);

    /// <summary>The instance with other attributes.</summary>
    public Instance WithAttributes(Attributes attributes) => new(Product, Location, attributes);

    /// <summary>The instance with another name.</summary>
    public Instance WithName(string? name) => WithAttributes(Attributes.WithName(name));

    /// <summary>The instance of another product, at the same place with the same attributes.</summary>
    public Instance WithProduct(DocumentProduct product) => new(product, Location, Attributes);

    /// <inheritdoc/>
    public override string ToString() => $"Instance {DisplayName}";
}
