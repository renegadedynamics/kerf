// Port of: src/DataExchange/TKXSBase/Interface/Interface_Version.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// The version texts data exchange writes into files (OCCT's macros <c>XSTEP_PROCESSOR_VERSION</c>...), with OCCT's
/// <c>OCC_VERSION_STRING</c> of the ported version, "8.0". They name Kerf, a modified version that Open Cascade SAS hasn't
/// endorsed, and keep that version (unit 11L); nothing reads them back.
/// </summary>
internal static class InterfaceVersion
{
    /// <summary>OCCT's <c>OCC_VERSION_STRING</c> (major.minor) for V8_0_1.</summary>
    public const string OCC_VERSION_STRING = "8.0";

    /// <summary>A format with one <c>%s</c> for the norm's name: "Kerf %s processor 8.0".</summary>
    public const string XSTEP_PROCESSOR_VERSION = "Kerf %s processor " + OCC_VERSION_STRING;

    public const string XSTEP_SYSTEM_VERSION = "Kerf " + OCC_VERSION_STRING;

    /// <summary>The truthful note a STEP file's description carries, as the licence and the README describe Kerf.</summary>
    public const string XSTEP_DERIVATION = "Kerf is derived from Open CASCADE Technology " + OCC_VERSION_STRING;

    public const string XSTEP_Config = OCC_VERSION_STRING;

    public const string XSTEP_ULNames = OCC_VERSION_STRING;
}
