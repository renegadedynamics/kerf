// Port of: src/ModelingData/TKBRep/TopTools/TopTools_FormatVersion.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>The versions of the TopTools (text BRep) format.</summary>
/// <remarks>
/// An unscoped enum in C++; ported code imports it with <c>using static Occt.TopTools_FormatVersion;</c>. OCCT's
/// anonymous enum of bounds is <see cref="FormatVersionBounds"/>.
/// </remarks>
internal enum FormatVersion
{
    /// <summary>Doesn't write the UV points of curves on surfaces; reading calls Check(). The default version.</summary>
    FormatVersion_VERSION_1 = 1,

    /// <summary>Stores the UV points of curves on surfaces; reading recognises the format from its version string.</summary>
    FormatVersion_VERSION_2 = 2,

    /// <summary>Stores per-vertex normals of triangulation-only faces, which have no geometry to restore them from.</summary>
    FormatVersion_VERSION_3 = 3,

    /// <summary>The current version.</summary>
    FormatVersion_CURRENT = FormatVersion_VERSION_3,
}

/// <summary>The lowest and highest supported <see cref="FormatVersion"/> (OCCT's anonymous enum).</summary>
internal static class FormatVersionBounds
{
    public const FormatVersion TopTools_FormatVersion_LOWER = FormatVersion.FormatVersion_VERSION_1;

    public const FormatVersion TopTools_FormatVersion_UPPER = FormatVersion.FormatVersion_VERSION_3;
}
