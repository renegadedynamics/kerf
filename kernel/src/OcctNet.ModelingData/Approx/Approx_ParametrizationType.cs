// Port of: src/ModelingData/TKGeomBase/Approx/Approx_ParametrizationType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>How the approximation algorithms parametrise the points of a MultiLine.</summary>
public enum Approx_ParametrizationType
{
    /// <summary>The parameters of the points are proportionate to the distances between them.</summary>
    Approx_ChordLength,

    /// <summary>The parameters of the points are proportionate to the square roots of the distances between them.</summary>
    Approx_Centripetal,

    /// <summary>The parameters of the points are distributed uniformly.</summary>
    Approx_IsoParametric,
}
