// Port of: src/FoundationClasses/TKernel/Resource/Resource_FormatType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>List of non ASCII format types.</summary>
public enum Resource_FormatType
{
    /// <summary>SJIS (Shift Japanese Industrial Standards) encoding.</summary>
    Resource_FormatType_SJIS,
    /// <summary>EUC (Extended Unix Code) multi-byte encoding primarily for Japanese, Korean, and simplified Chinese.</summary>
    Resource_FormatType_EUC,
    /// <summary>Format type indicating non-conversion behavior.</summary>
    Resource_FormatType_NoConversion,
    /// <summary>GB (Guobiao) encoding for Simplified Chinese.</summary>
    Resource_FormatType_GB,
    /// <summary>Multi-byte UTF-8 encoding.</summary>
    Resource_FormatType_UTF8,
    /// <summary>Active system-defined locale; this value is strongly NOT recommended to use.</summary>
    Resource_FormatType_SystemLocale,

    // Windows-native ("ANSI") 8-bit code pages
    Resource_FormatType_CP1250,
    Resource_FormatType_CP1251,
    Resource_FormatType_CP1252,
    Resource_FormatType_CP1253,
    Resource_FormatType_CP1254,
    Resource_FormatType_CP1255,
    Resource_FormatType_CP1256,
    Resource_FormatType_CP1257,
    Resource_FormatType_CP1258,

    // ISO8859 8-bit code pages
    Resource_FormatType_iso8859_1,
    Resource_FormatType_iso8859_2,
    Resource_FormatType_iso8859_3,
    Resource_FormatType_iso8859_4,
    Resource_FormatType_iso8859_5,
    Resource_FormatType_iso8859_6,
    Resource_FormatType_iso8859_7,
    Resource_FormatType_iso8859_8,
    Resource_FormatType_iso8859_9,

    // Addition code pages
    Resource_FormatType_CP850,
    Resource_FormatType_GBK,
    Resource_FormatType_Big5,

    // old aliases
    Resource_FormatType_ANSI = Resource_FormatType_NoConversion,
    Resource_SJIS = Resource_FormatType_SJIS,
    Resource_EUC = Resource_FormatType_EUC,
    Resource_ANSI = Resource_FormatType_ANSI,
    Resource_GB = Resource_FormatType_GB,
}
