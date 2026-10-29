// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Operation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The type of a Boolean operation.</summary>
public enum BOPAlgo_Operation
{
    BOPAlgo_COMMON,
    BOPAlgo_FUSE,
    BOPAlgo_CUT,
    BOPAlgo_CUT21,
    BOPAlgo_SECTION,
    BOPAlgo_UNKNOWN,
}
