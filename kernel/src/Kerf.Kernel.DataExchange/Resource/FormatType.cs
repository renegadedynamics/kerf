// Port of: src/FoundationClasses/TKernel/Resource/Resource_FormatType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Foundation.Resources;

/// <summary>List of non ASCII format types.</summary>
internal enum FormatType
{
    /// <summary>SJIS (Shift Japanese Industrial Standards) encoding.</summary>
    FormatType_SJIS,
    /// <summary>EUC (Extended Unix Code) multi-byte encoding primarily for Japanese, Korean, and simplified Chinese.</summary>
    FormatType_EUC,
    /// <summary>Format type indicating non-conversion behavior.</summary>
    FormatType_NoConversion,
    /// <summary>GB (Guobiao) encoding for Simplified Chinese.</summary>
    FormatType_GB,
    /// <summary>Multi-byte UTF-8 encoding.</summary>
    FormatType_UTF8,
    /// <summary>Active system-defined locale; this value is strongly NOT recommended to use.</summary>
    FormatType_SystemLocale,

    // Windows-native ("ANSI") 8-bit code pages
    FormatType_CP1250,
    FormatType_CP1251,
    FormatType_CP1252,
    FormatType_CP1253,
    FormatType_CP1254,
    FormatType_CP1255,
    FormatType_CP1256,
    FormatType_CP1257,
    FormatType_CP1258,

    // ISO8859 8-bit code pages
    FormatType_iso8859_1,
    FormatType_iso8859_2,
    FormatType_iso8859_3,
    FormatType_iso8859_4,
    FormatType_iso8859_5,
    FormatType_iso8859_6,
    FormatType_iso8859_7,
    FormatType_iso8859_8,
    FormatType_iso8859_9,

    // Addition code pages
    FormatType_CP850,
    FormatType_GBK,
    FormatType_Big5,

    // old aliases
    FormatType_ANSI = FormatType_NoConversion,
    Sjis = FormatType_SJIS,
    Euc = FormatType_EUC,
    Ansi = FormatType_ANSI,
    GB = FormatType_GB,
}
