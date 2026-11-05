// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Operation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans;

/// <summary>The type of a Boolean operation.</summary>
internal enum Operation
{
    Common,
    Fuse,
    Cut,
    CUT21,
    Section,
    Unknown,
}
