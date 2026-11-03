// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_ContactType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Offsets.Blend;

/// <summary>The kinds of the rolling ball's contacts on the two sides of a blend.</summary>
public enum ContactType
{
    FaceFace,
    FaceEdge,
    FaceVertex,
    EdgeEdge,
    EdgeVertex,
    VertexVertex
}
