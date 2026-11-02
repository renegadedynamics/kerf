// Port of: src/DataExchange/TKDESTEP/StepAP214/StepAP214.hxx
// Port of: src/DataExchange/TKDESTEP/StepAP214/StepAP214.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Written by hand (tools/stepgen/overrides/StepAP214.cs, decision 12803): the generator copies this file.

namespace Occt;

/// <summary>The StepAP214 package: its protocol.</summary>
public static class StepAP214
{
    // OCCT's StaticHandle and InitHandleVoid: one protocol per process, created on first use
    private static readonly Lazy<StepAP214_Protocol> proto = new(() => new StepAP214_Protocol());

    /// <summary>Returns the Protocol (created once per process).</summary>
    public static StepAP214_Protocol Protocol()
    {
        return proto.Value;
    }
}
