// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_ErrorStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error statuses of the draft modification: which kind of shape could not be recomputed.</summary>
public enum Draft_ErrorStatus
{
    Draft_NoError,
    Draft_FaceRecomputation,
    Draft_EdgeRecomputation,
    Draft_VertexRecomputation
}
