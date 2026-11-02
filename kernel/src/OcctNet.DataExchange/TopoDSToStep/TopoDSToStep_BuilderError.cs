// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_BuilderError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error status of TopoDSToStep_Builder.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.TopoDSToStep_BuilderError;</c>.</remarks>
public enum TopoDSToStep_BuilderError
{
    TopoDSToStep_BuilderDone,
    TopoDSToStep_NoFaceMapped,
    TopoDSToStep_BuilderOther,
}
