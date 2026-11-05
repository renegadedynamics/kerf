// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>The HeaderSection package: its protocol.</summary>
internal static class HeaderSection
{
    private static readonly Lazy<HeaderProtocol> Proto = new(() => new HeaderProtocol());

    /// <summary>Returns the Protocol (created once per process).</summary>
    public static HeaderProtocol Protocol()
    {
        return Proto.Value;
    }
}
