// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TypeOfResultingEdge.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kinds of the edges the hidden-line removal produces (HLRBRep_HLRToShape's compounds).</summary>
public enum HLRBRep_TypeOfResultingEdge
{
    HLRBRep_Undefined,

    /// <summary>Isoparametric line.</summary>
    HLRBRep_IsoLine,

    /// <summary>Outline ("silhouette").</summary>
    HLRBRep_OutLine,

    /// <summary>Smooth edge of G1-continuity between two surfaces.</summary>
    HLRBRep_Rg1Line,

    /// <summary>Sewn edge of CN-continuity on one surface.</summary>
    HLRBRep_RgNLine,

    /// <summary>Sharp edge (of C0-continuity).</summary>
    HLRBRep_Sharp,
}
