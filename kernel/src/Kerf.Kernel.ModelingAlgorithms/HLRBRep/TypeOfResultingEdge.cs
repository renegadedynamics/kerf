// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TypeOfResultingEdge.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.HiddenLines;

/// <summary>The kinds of the edges the hidden-line removal produces (HLRBRep_HLRToShape's compounds).</summary>
internal enum TypeOfResultingEdge
{
    Undefined,

    /// <summary>Isoparametric line.</summary>
    IsoLine,

    /// <summary>Outline ("silhouette").</summary>
    OutLine,

    /// <summary>Smooth edge of G1-continuity between two surfaces.</summary>
    Rg1Line,

    /// <summary>Sewn edge of CN-continuity on one surface.</summary>
    RgNLine,

    /// <summary>Sharp edge (of C0-continuity).</summary>
    Sharp,
}
