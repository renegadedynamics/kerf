// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeFaceError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>The error status of TopoDSToStep_MakeFace.</summary>
internal enum MakeFaceError
{
    FaceDone,
    InfiniteFace,
    NonManifoldFace,
    NoWireMapped,
    FaceOther,
}
