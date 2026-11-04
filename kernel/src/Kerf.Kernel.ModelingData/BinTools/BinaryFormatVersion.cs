// Port of: src/ModelingData/TKBRep/BinTools/BinTools_FormatVersion.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>The versions of the BinTools (binary BRep) format.</summary>
/// <remarks>
/// An unscoped enum in C++; ported code imports it with <c>using static Occt.BinTools_FormatVersion;</c>. OCCT's
/// anonymous enum of bounds is <see cref="BinaryFormatVersionBounds"/>.
/// </remarks>
internal enum BinaryFormatVersion
{
    /// <summary>Doesn't write the UV points of curves on surfaces; reading calls Check().</summary>
    FormatVersion_VERSION_1 = 1,

    /// <summary>Stores the UV points of curves on surfaces; reading recognises the format from its version string.</summary>
    FormatVersion_VERSION_2 = 2,

    /// <summary>(OCCT 6.2.1) Reads points on curves, on surfaces and on curves on surfaces correctly.</summary>
    FormatVersion_VERSION_3 = 3,

    /// <summary>Stores per-vertex normals of triangulation-only faces, which have no geometry to restore them from.</summary>
    FormatVersion_VERSION_4 = 4,

    /// <summary>The current version.</summary>
    FormatVersion_CURRENT = FormatVersion_VERSION_4,
}

/// <summary>The lowest and highest supported <see cref="BinaryFormatVersion"/> (OCCT's anonymous enum).</summary>
internal static class BinaryFormatVersionBounds
{
    public const BinaryFormatVersion BinTools_FormatVersion_LOWER = BinaryFormatVersion.FormatVersion_VERSION_1;

    public const BinaryFormatVersion BinTools_FormatVersion_UPPER = BinaryFormatVersion.FormatVersion_VERSION_4;
}
