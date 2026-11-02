// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;

namespace Occt;

/// <summary>
/// A product of an assembly tree: a shared definition, placed by <see cref="XdeDoc_Instance"/>s. Either a part (a
/// shape, as XCAF's simple shape labels) or an assembly (components, as XCAF's assembly labels), with its own
/// attributes and the attributes of its sub-shapes (XCAF's sub-shape labels: colours and names on faces, edges...).
/// </summary>
/// <remarks>
/// Immutable: the <c>With...</c> methods make new products, and products are shared between instances and between
/// trees (a product placed twice is one object, as a label referred to twice in XCAF). A part's shape is frozen when
/// the product is made (decision 1230); an assembly's shape is the compound of its components, made once when first
/// asked for. Thread-safe.
/// </remarks>
public sealed class XdeDoc_Product
{
    private readonly TopoDS_Shape myShape;
    private TopoDS_Shape? myAssemblyShape;
    private readonly Lock myLock = new();

    private XdeDoc_Product(TopoDS_Shape theShape,
                           bool isAssembly,
                           ImmutableArray<XdeDoc_Instance> theComponents,
                           ImmutableArray<XdeDoc_SubShape> theSubShapes,
                           XdeDoc_Attributes theAttributes)
    {
        myShape = theShape;
        IsAssembly = isAssembly;
        Components = theComponents;
        SubShapes = theSubShapes;
        Attributes = theAttributes;
    }

    /// <summary>A part: the shape (frozen here, if it isn't yet), its attributes and its sub-shapes' attributes.</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static XdeDoc_Product Part(in TopoDS_Shape theShape,
                                      XdeDoc_Attributes? theAttributes = null,
                                      IEnumerable<XdeDoc_SubShape>? theSubShapes = null)
    {
        if (theShape.IsNull())
        {
            throw new ArgumentException("A part needs a shape", nameof(theShape));
        }
        theShape.TShape()?.FreezeGraph();
        return new XdeDoc_Product(theShape, false, [], theSubShapes?.ToImmutableArray() ?? [],
                                  theAttributes ?? XdeDoc_Attributes.Empty);
    }

    /// <summary>A part with a name.</summary>
    public static XdeDoc_Product Part(in TopoDS_Shape theShape, string? theName)
        => Part(theShape, XdeDoc_Attributes.Empty.WithName(theName));

    /// <summary>An assembly of the components (possibly none), with its attributes.</summary>
    public static XdeDoc_Product Assembly(IEnumerable<XdeDoc_Instance> theComponents,
                                          XdeDoc_Attributes? theAttributes = null)
        => new(new TopoDS_Shape(), true, theComponents.ToImmutableArray(), [],
               theAttributes ?? XdeDoc_Attributes.Empty);

    /// <summary>An assembly with a name.</summary>
    public static XdeDoc_Product Assembly(string? theName, params IEnumerable<XdeDoc_Instance> theComponents)
        => Assembly(theComponents, XdeDoc_Attributes.Empty.WithName(theName));

    /// <summary>True for an assembly, false for a part.</summary>
    public bool IsAssembly { get; }

    /// <summary>The components of an assembly (empty for a part).</summary>
    public ImmutableArray<XdeDoc_Instance> Components { get; }

    /// <summary>The attributes of the part's sub-shapes (empty for an assembly).</summary>
    public ImmutableArray<XdeDoc_SubShape> SubShapes { get; }

    /// <summary>The product's own attributes.</summary>
    public XdeDoc_Attributes Attributes { get; }

    /// <summary>The product's name, or null.</summary>
    public string? Name => Attributes.Name;

    /// <summary>
    /// The shape: a part's own, or for an assembly the compound of its components placed (as XCAF's
    /// <c>UpdateAssemblies</c> makes it), frozen. An empty assembly gives an empty compound.
    /// </summary>
    public TopoDS_Shape Shape
    {
        get
        {
            if (!IsAssembly)
            {
                return myShape;
            }
            lock (myLock)
            {
                if (myAssemblyShape is TopoDS_Shape aShape)
                {
                    return aShape;
                }
            }
            var aBuilder = new BRep_Builder();
            TopoDS_Compound aCompound = new();
            aBuilder.MakeCompound(ref aCompound);
            foreach (XdeDoc_Instance aComponent in Components)
            {
                TopoDS_Shape aSub = aComponent.Shape;
                if (!aSub.IsNull())
                {
                    aBuilder.Add(aCompound, aSub);
                }
            }
            aCompound.TShape()!.FreezeGraph();
            // Racing threads build equal compounds; the first one stored wins
            lock (myLock)
            {
                myAssemblyShape ??= aCompound;
                return myAssemblyShape.Value;
            }
        }
    }

    /// <summary>The attributes of a sub-shape (compared with IsSame), or null.</summary>
    public XdeDoc_SubShape? FindSubShape(in TopoDS_Shape theSubShape)
    {
        foreach (XdeDoc_SubShape aSub in SubShapes)
        {
            if (aSub.Shape.IsSame(theSubShape))
            {
                return aSub;
            }
        }
        return null;
    }

    /// <summary>The product with other attributes.</summary>
    public XdeDoc_Product WithAttributes(XdeDoc_Attributes theAttributes)
        => new(myShape, IsAssembly, Components, SubShapes, theAttributes);

    /// <summary>The product with another name.</summary>
    public XdeDoc_Product WithName(string? theName) => WithAttributes(Attributes.WithName(theName));

    /// <summary>The product with the colour of the type replaced (removed if null).</summary>
    public XdeDoc_Product WithColor(XdeDoc_ColorType theType, Quantity_ColorRGBA? theColor)
        => WithAttributes(Attributes.WithColor(theType, theColor));

    /// <summary>The assembly with other components.</summary>
    /// <exception cref="InvalidOperationException">The product is a part.</exception>
    public XdeDoc_Product WithComponents(IEnumerable<XdeDoc_Instance> theComponents)
        => IsAssembly
             ? new(myShape, true, theComponents.ToImmutableArray(), [], Attributes)
             : throw new InvalidOperationException("A part has no components");

    /// <summary>The part with other sub-shape attributes.</summary>
    /// <exception cref="InvalidOperationException">The product is an assembly.</exception>
    public XdeDoc_Product WithSubShapes(IEnumerable<XdeDoc_SubShape> theSubShapes)
        => !IsAssembly
             ? new(myShape, false, [], theSubShapes.ToImmutableArray(), Attributes)
             : throw new InvalidOperationException("An assembly has no sub-shapes");

    /// <summary>
    /// The part with the attributes of a sub-shape set (replacing the sub-shape's entry if it has one, compared with
    /// IsSame). The sub-shape is frozen.
    /// </summary>
    public XdeDoc_Product WithSubShape(in TopoDS_Shape theSubShape, XdeDoc_Attributes theAttributes)
    {
        var aSub = new XdeDoc_SubShape(theSubShape, theAttributes);
        var aList = SubShapes.ToBuilder();
        for (int i = 0; i < aList.Count; i++)
        {
            if (aList[i].Shape.IsSame(theSubShape))
            {
                aList[i] = aSub;
                return WithSubShapes(aList.ToImmutable());
            }
        }
        aList.Add(aSub);
        return WithSubShapes(aList.ToImmutable());
    }

    /// <summary>The product placed at the location, as a new instance.</summary>
    public XdeDoc_Instance Placed(in TopLoc_Location theLocation, XdeDoc_Attributes? theAttributes = null)
        => new(this, theLocation, theAttributes);

    /// <inheritdoc/>
    public override string ToString() => $"{(IsAssembly ? "Assembly" : "Part")} {Name}";
}
