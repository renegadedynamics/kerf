// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Operation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Features.Local;

/// <summary>The kind of a local operation.</summary>
internal enum FeatureOperation
{
    Fuse,
    Cut,
    Invalid,
}
