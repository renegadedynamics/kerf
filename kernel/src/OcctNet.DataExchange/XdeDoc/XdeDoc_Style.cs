// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The resolved style of a shape where it is shown, as XCAF's presentation (<c>XCAFPrs_Style</c>): a surface colour, a
/// curve colour and visibility, each inherited down the tree and overridden by more specific attributes.
/// </summary>
/// <param name="SurfaceColor">The surfaces' colour, or null for the viewer's default.</param>
/// <param name="CurveColor">The curves' colour, or null for the viewer's default.</param>
/// <param name="IsVisible">False if the shape is hidden.</param>
public readonly record struct XdeDoc_Style(ColorWithAlpha? SurfaceColor, ColorWithAlpha? CurveColor, bool IsVisible = true)
{
    /// <summary>No colour, visible.</summary>
    public static XdeDoc_Style Default => new(null, null, true);

    /// <summary>
    /// This style overridden by the attributes, as <c>XCAFPrs_DocumentExplorer</c>'s merge: the generic colour sets both
    /// colours, then the surface and curve colours set their own; hiding is inherited.
    /// </summary>
    public XdeDoc_Style Merged(XdeDoc_Attributes theAttributes)
    {
        ColorWithAlpha? aSurf = SurfaceColor;
        ColorWithAlpha? aCurv = CurveColor;
        if (theAttributes.GenericColor is ColorWithAlpha aGen)
        {
            aSurf = aGen;
            aCurv = new ColorWithAlpha(aGen.GetRGB());
        }
        if (theAttributes.SurfaceColor is ColorWithAlpha aS)
        {
            aSurf = aS;
        }
        if (theAttributes.CurveColor is ColorWithAlpha aC)
        {
            aCurv = new ColorWithAlpha(aC.GetRGB());
        }
        return new XdeDoc_Style(aSurf, aCurv, IsVisible && theAttributes.IsVisible);
    }
}
