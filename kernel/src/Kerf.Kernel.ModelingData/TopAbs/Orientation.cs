// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs_Orientation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Topology;

/// <summary>
/// The orientation of a shape: FORWARD and REVERSED for boundaries, INTERNAL (both sides are inside) and
/// EXTERNAL (both sides are outside).
/// </summary>
public enum Orientation
{
    Forward,
    Reversed,
    Internal,
    External,
}
