// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The HeaderSection package: its protocol.</summary>
public static class HeaderSection
{
    private static readonly Lazy<HeaderSection_Protocol> proto = new(() => new HeaderSection_Protocol());

    /// <summary>Returns the Protocol (created once per process).</summary>
    public static HeaderSection_Protocol Protocol()
    {
        return proto.Value;
    }
}
