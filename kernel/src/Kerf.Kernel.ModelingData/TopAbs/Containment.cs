// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs_State.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Topology;

/// <summary>The position of a point relative to a region (inside, outside, on the boundary, unknown).</summary>
public enum Containment
{
    /// <summary>Strictly inside the region.</summary>
    In,
    /// <summary>Strictly outside the region.</summary>
    Out,
    /// <summary>On the region's boundary, within the classifier's tolerance.</summary>
    On,
    /// <summary>Not classified, or the classification failed.</summary>
    Unknown,
}
