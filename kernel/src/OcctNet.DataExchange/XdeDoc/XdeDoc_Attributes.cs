// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;

namespace Occt;

/// <summary>
/// What XCAF attaches to a label besides the shape: a name (<c>TDataStd_Name</c>), colours (<c>XCAFDoc_ColorTool</c>,
/// with alpha), visibility, layers (<c>XCAFDoc_LayerTool</c>, by name), validation properties and external references.
/// Products, instances and sub-shapes each carry a set. Immutable: the <c>With...</c> methods and <c>with</c>
/// expressions make new sets.
/// </summary>
/// <remarks>Materials, GD&amp;T, views and user metadata are left for later (decision 1166); the record grows then.</remarks>
public sealed record XdeDoc_Attributes
{
    /// <summary>No attribute: no name, no colour, visible, no layer.</summary>
    public static XdeDoc_Attributes Empty { get; } = new();

    /// <summary>The name, or null.</summary>
    public string? Name { get; init; }

    /// <summary>The generic colour (<see cref="XdeDoc_ColorType.XdeDoc_ColorGen"/>), or null.</summary>
    public Quantity_ColorRGBA? GenericColor { get; init; }

    /// <summary>The surfaces' colour (<see cref="XdeDoc_ColorType.XdeDoc_ColorSurf"/>), or null.</summary>
    public Quantity_ColorRGBA? SurfaceColor { get; init; }

    /// <summary>The curves' colour (<see cref="XdeDoc_ColorType.XdeDoc_ColorCurv"/>), or null.</summary>
    public Quantity_ColorRGBA? CurveColor { get; init; }

    /// <summary>False if the shape is hidden (<c>XCAFDoc_ColorTool::SetVisibility</c>).</summary>
    public bool IsVisible { get; init; } = true;

    /// <summary>The names of the layers the shape is in, in the order it was put in them.</summary>
    public ImmutableArray<string> Layers { get; init; } = [];

    /// <summary>The validation properties, or null.</summary>
    public XdeDoc_ValidationProps? ValidationProps { get; init; }

    /// <summary>The names of external files the shape was read from (<c>XCAFDoc_ShapeTool::SetExternRefs</c>).</summary>
    public ImmutableArray<string> ExternRefs { get; init; } = [];

    /// <summary>The colour of the type, or null.</summary>
    public Quantity_ColorRGBA? Color(XdeDoc_ColorType theType) => theType switch
    {
        XdeDoc_ColorType.XdeDoc_ColorGen => GenericColor,
        XdeDoc_ColorType.XdeDoc_ColorSurf => SurfaceColor,
        XdeDoc_ColorType.XdeDoc_ColorCurv => CurveColor,
        _ => throw new ArgumentOutOfRangeException(nameof(theType)),
    };

    /// <summary>The set with the colour of the type replaced (removed if null).</summary>
    public XdeDoc_Attributes WithColor(XdeDoc_ColorType theType, Quantity_ColorRGBA? theColor) => theType switch
    {
        XdeDoc_ColorType.XdeDoc_ColorGen => this with { GenericColor = theColor },
        XdeDoc_ColorType.XdeDoc_ColorSurf => this with { SurfaceColor = theColor },
        XdeDoc_ColorType.XdeDoc_ColorCurv => this with { CurveColor = theColor },
        _ => throw new ArgumentOutOfRangeException(nameof(theType)),
    };

    /// <summary>The set with the name replaced.</summary>
    public XdeDoc_Attributes WithName(string? theName) => this with { Name = theName };

    /// <summary>The set with the shape put in the layer as well (unchanged if it is already in it).</summary>
    public XdeDoc_Attributes WithLayer(string theLayer)
        => Layers.Contains(theLayer) ? this : this with { Layers = Layers.Add(theLayer) };

    /// <summary>True if any colour is set.</summary>
    public bool HasColor => GenericColor is not null || SurfaceColor is not null || CurveColor is not null;

    /// <summary>True if nothing is set.</summary>
    public bool IsEmpty => Equals(Empty);

    /// <summary>Equality by value, the lists compared element by element.</summary>
    public bool Equals(XdeDoc_Attributes? other)
        => other is not null
           && Name == other.Name
           && GenericColor == other.GenericColor
           && SurfaceColor == other.SurfaceColor
           && CurveColor == other.CurveColor
           && IsVisible == other.IsVisible
           && Layers.SequenceEqual(other.Layers)
           && Equals(ValidationProps, other.ValidationProps)
           && ExternRefs.SequenceEqual(other.ExternRefs);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        var aHash = new HashCode();
        aHash.Add(Name);
        aHash.Add(GenericColor);
        aHash.Add(SurfaceColor);
        aHash.Add(CurveColor);
        aHash.Add(IsVisible);
        foreach (string aLayer in Layers)
        {
            aHash.Add(aLayer);
        }
        aHash.Add(ValidationProps);
        foreach (string aRef in ExternRefs)
        {
            aHash.Add(aRef);
        }
        return aHash.ToHashCode();
    }
}
