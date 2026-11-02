// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeWireError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error status of TopoDSToStep_MakeWire.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.TopoDSToStep_MakeWireError;</c>.</remarks>
public enum TopoDSToStep_MakeWireError
{
    TopoDSToStep_WireDone,
    TopoDSToStep_NonManifoldWire,
    TopoDSToStep_WireOther,
}
