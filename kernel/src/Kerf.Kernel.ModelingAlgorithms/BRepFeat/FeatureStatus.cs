// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Features;

/// <summary>The status of BRepFeat_MakeCylindricalHole.</summary>
internal enum FeatureStatus
{
    NoError,
    InvalidPlacement,
    HoleTooLong,
}
