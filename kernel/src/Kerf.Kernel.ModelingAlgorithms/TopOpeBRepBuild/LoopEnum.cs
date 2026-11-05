// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopEnum.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>Which loops of an area a loop is compared with: any, the boundaries (shapes) or the blocks of elements.</summary>
internal enum LoopEnum
{
    Anyloop,
    Boundary,
    Block,
}
