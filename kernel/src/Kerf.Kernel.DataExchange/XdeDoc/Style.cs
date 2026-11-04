// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// The resolved style of a shape where it is shown, as XCAF's presentation (<c>XCAFPrs_Style</c>): a surface colour, a
/// curve colour and visibility, each inherited down the tree and overridden by more specific attributes.
/// </summary>
/// <param name="SurfaceColor">The surfaces' colour, or null for the viewer's default.</param>
/// <param name="CurveColor">The curves' colour, or null for the viewer's default.</param>
/// <param name="IsVisible">False if the shape is hidden.</param>
internal readonly record struct Style(ColorWithAlpha? SurfaceColor, ColorWithAlpha? CurveColor, bool IsVisible = true)
{
    /// <summary>No colour, visible.</summary>
    public static Style Default => new(null, null, true);

    /// <summary>
    /// This style overridden by the attributes, as <c>XCAFPrs_DocumentExplorer</c>'s merge: the generic colour sets both
    /// colours, then the surface and curve colours set their own; hiding is inherited.
    /// </summary>
    public Style Merged(Attributes attributes)
    {
        ColorWithAlpha? surf = SurfaceColor;
        ColorWithAlpha? curv = CurveColor;
        if (attributes.GenericColor is ColorWithAlpha gen)
        {
            surf = gen;
            curv = new ColorWithAlpha(gen.GetRGB());
        }
        if (attributes.SurfaceColor is ColorWithAlpha s)
        {
            surf = s;
        }
        if (attributes.CurveColor is ColorWithAlpha c)
        {
            curv = new ColorWithAlpha(c.GetRGB());
        }
        return new Style(surf, curv, IsVisible && attributes.IsVisible);
    }
}
