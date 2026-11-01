// Port of: src/FoundationClasses/TKernel/Quantity/Quantity_TypeOfColor.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The colour spaces a <see cref="Quantity_Color"/> is given or read in.</summary>
public enum Quantity_TypeOfColor
{
    /// <summary>Normalized linear RGB (red, green, blue) values within range [0..1] for each component.</summary>
    Quantity_TOC_RGB,

    /// <summary>Normalized non-linear gamma-shifted RGB (red, green, blue) values within range [0..1] for each component.</summary>
    Quantity_TOC_sRGB,

    /// <summary>
    /// Hue (degrees in [0, 360], 0 being red; -1 reserved for greyscale, with saturation 0), lightness [0, 1] and
    /// saturation [0, 1].
    /// </summary>
    Quantity_TOC_HLS,

    /// <summary>
    /// CIE L*a*b* (D65, 2 degrees): L in [0, 100], a approximately in [-90, 100], b approximately in [-110, 95]. When
    /// converted to RGB, a and b may be reduced (in the same proportion) to fit the RGB range.
    /// </summary>
    Quantity_TOC_CIELab,

    /// <summary>
    /// CIE L*c*h*, L*a*b* in cylindrical coordinates: L in [0, 100], chroma approximately in [0, 135], hue in [0, 360].
    /// When converted to RGB, the chroma may be reduced to fit the RGB range.
    /// </summary>
    Quantity_TOC_CIELch,
}
