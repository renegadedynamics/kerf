// Port of: src/DataExchange/TKXSBase/Interface/Interface_Version.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// The version texts data exchange writes into files (OCCT's macros <c>XSTEP_PROCESSOR_VERSION</c>...), with OCCT's
/// <c>OCC_VERSION_STRING</c> of the ported version, "8.0".
/// </summary>
public static class InterfaceVersion
{
    /// <summary>OCCT's <c>OCC_VERSION_STRING</c> (major.minor) for V8_0_1.</summary>
    public const string OCC_VERSION_STRING = "8.0";

    /// <summary>A format with one <c>%s</c> for the norm's name: "Open CASCADE %s processor 8.0".</summary>
    public const string XSTEP_PROCESSOR_VERSION = "Open CASCADE %s processor " + OCC_VERSION_STRING;

    public const string XSTEP_SYSTEM_VERSION = "Open CASCADE " + OCC_VERSION_STRING;

    public const string XSTEP_Config = OCC_VERSION_STRING;

    public const string XSTEP_ULNames = OCC_VERSION_STRING;
}
