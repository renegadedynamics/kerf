// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs_Orientation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Topology;

/// <summary>
/// The orientation of a shape: FORWARD and REVERSED for boundaries, INTERNAL (both sides are inside) and
/// EXTERNAL (both sides are outside).
/// </summary>
public enum Orientation
{
    /// <summary>The same sense as the underlying geometry.</summary>
    Forward,
    /// <summary>The opposite sense to the underlying geometry.</summary>
    Reversed,
    /// <summary>Inside the region, with material on both sides, such as an edge embedded in a face.</summary>
    Internal,
    /// <summary>Outside the region, with no material on either side.</summary>
    External,
}
