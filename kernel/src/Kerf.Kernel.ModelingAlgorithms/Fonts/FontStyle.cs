// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Fonts;

/// <summary>A font's style within its family (OCCT's <c>Font_FontAspect</c>, build123d's <c>FontStyle</c>).</summary>
public enum FontStyle
{
    /// <summary>Upright, normal weight.</summary>
    Regular,

    /// <summary>Upright, bold.</summary>
    Bold,

    /// <summary>Italic (or oblique), normal weight.</summary>
    Italic,

    /// <summary>Italic (or oblique), bold.</summary>
    BoldItalic,
}
