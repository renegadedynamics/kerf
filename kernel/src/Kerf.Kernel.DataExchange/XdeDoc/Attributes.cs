// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Collections.Immutable;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// What XCAF attaches to a label besides the shape: a name (<c>TDataStd_Name</c>), colours (<c>XCAFDoc_ColorTool</c>,
/// with alpha), visibility, layers (<c>XCAFDoc_LayerTool</c>, by name), validation properties and external references.
/// Products, instances and sub-shapes each carry a set. Immutable: the <c>With...</c> methods and <c>with</c>
/// expressions make new sets.
/// </summary>
/// <remarks>Materials, GD&amp;T, views and user metadata are left for later (decision 1166); the record grows then.</remarks>
internal sealed record Attributes
{
    /// <summary>No attribute: no name, no colour, visible, no layer.</summary>
    public static Attributes Empty { get; } = new();

    /// <summary>The name, or null.</summary>
    public string? Name { get; init; }

    /// <summary>The generic colour (<see cref="ColorType.ColorGen"/>), or null.</summary>
    public ColorWithAlpha? GenericColor { get; init; }

    /// <summary>The surfaces' colour (<see cref="ColorType.ColorSurf"/>), or null.</summary>
    public ColorWithAlpha? SurfaceColor { get; init; }

    /// <summary>The curves' colour (<see cref="ColorType.ColorCurv"/>), or null.</summary>
    public ColorWithAlpha? CurveColor { get; init; }

    /// <summary>False if the shape is hidden (<c>XCAFDoc_ColorTool::SetVisibility</c>).</summary>
    public bool IsVisible { get; init; } = true;

    /// <summary>The names of the layers the shape is in, in the order it was put in them.</summary>
    public ImmutableArray<string> Layers { get; init; } = [];

    /// <summary>The validation properties, or null.</summary>
    public DocumentValidationProps? ValidationProps { get; init; }

    /// <summary>The names of external files the shape was read from (<c>XCAFDoc_ShapeTool::SetExternRefs</c>).</summary>
    public ImmutableArray<string> ExternRefs { get; init; } = [];

    /// <summary>The colour of the type, or null.</summary>
    public ColorWithAlpha? Color(ColorType type) => type switch
    {
        ColorType.ColorGen => GenericColor,
        ColorType.ColorSurf => SurfaceColor,
        ColorType.ColorCurv => CurveColor,
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    /// <summary>The set with the colour of the type replaced (removed if null).</summary>
    public Attributes WithColor(ColorType type, ColorWithAlpha? color) => type switch
    {
        ColorType.ColorGen => this with { GenericColor = color },
        ColorType.ColorSurf => this with { SurfaceColor = color },
        ColorType.ColorCurv => this with { CurveColor = color },
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    /// <summary>The set with the name replaced.</summary>
    public Attributes WithName(string? name) => this with { Name = name };

    /// <summary>The set with the shape put in the layer as well (unchanged if it is already in it).</summary>
    public Attributes WithLayer(string layer)
        => Layers.Contains(layer) ? this : this with { Layers = Layers.Add(layer) };

    /// <summary>True if any colour is set.</summary>
    public bool HasColor => GenericColor is not null || SurfaceColor is not null || CurveColor is not null;

    /// <summary>True if nothing is set.</summary>
    public bool IsEmpty => Equals(Empty);

    /// <summary>Equality by value, the lists compared element by element.</summary>
    public bool Equals(Attributes? other)
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
        var hash = new HashCode();
        hash.Add(Name);
        hash.Add(GenericColor);
        hash.Add(SurfaceColor);
        hash.Add(CurveColor);
        hash.Add(IsVisible);
        foreach (string layer in Layers)
        {
            hash.Add(layer);
        }
        hash.Add(ValidationProps);
        foreach (string aRef in ExternRefs)
        {
            hash.Add(aRef);
        }
        return hash.ToHashCode();
    }
}
