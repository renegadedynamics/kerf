// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An instance of a product: the product placed at a location, as XCAF's reference labels (an assembly's components,
/// or a free shape placed at the top). It has attributes of its own (XCAF's on the component label), which override
/// the product's for this placement only.
/// </summary>
/// <remarks>Immutable; shares its product.</remarks>
public sealed class XdeDoc_Instance
{
    /// <summary>The product placed at the location, with the instance's own attributes.</summary>
    public XdeDoc_Instance(XdeDoc_Product theProduct, in TopLoc_Location theLocation, XdeDoc_Attributes? theAttributes = null)
    {
        Product = theProduct ?? throw new ArgumentNullException(nameof(theProduct));
        Location = theLocation;
        Attributes = theAttributes ?? XdeDoc_Attributes.Empty;
    }

    /// <summary>The product at its own place (identity location), with no attribute of its own.</summary>
    public XdeDoc_Instance(XdeDoc_Product theProduct) : this(theProduct, new TopLoc_Location()) { }

    /// <summary>The product.</summary>
    public XdeDoc_Product Product { get; }

    /// <summary>The location, in the parent's coordinates (the world's for a root).</summary>
    public TopLoc_Location Location { get; }

    /// <summary>The instance's own attributes (override the product's).</summary>
    public XdeDoc_Attributes Attributes { get; }

    /// <summary>The instance's own name, or null (see <see cref="DisplayName"/>).</summary>
    public string? Name => Attributes.Name;

    /// <summary>The instance's name, or the product's if it has none.</summary>
    public string? DisplayName => Attributes.Name ?? Product.Name;

    /// <summary>The product's shape moved to the location (XCAF's <c>GetShape</c> of a reference).</summary>
    public TopoDS_Shape Shape
    {
        get
        {
            TopoDS_Shape aShape = Product.Shape;
            return aShape.IsNull() || Location.IsIdentity() ? aShape : aShape.Moved(Location);
        }
    }

    /// <summary>The instance with another location.</summary>
    public XdeDoc_Instance WithLocation(in TopLoc_Location theLocation) => new(Product, theLocation, Attributes);

    /// <summary>The instance with other attributes.</summary>
    public XdeDoc_Instance WithAttributes(XdeDoc_Attributes theAttributes) => new(Product, Location, theAttributes);

    /// <summary>The instance with another name.</summary>
    public XdeDoc_Instance WithName(string? theName) => WithAttributes(Attributes.WithName(theName));

    /// <summary>The instance of another product, at the same place with the same attributes.</summary>
    public XdeDoc_Instance WithProduct(XdeDoc_Product theProduct) => new(theProduct, Location, Attributes);

    /// <inheritdoc/>
    public override string ToString() => $"Instance {DisplayName}";
}
