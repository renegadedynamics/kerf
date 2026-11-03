// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TransitionStyle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Filling;

/// <summary>How a sweep treats the discontinuities of its spine's tangent.</summary>
public enum TransitionStyle
{
    /// <summary>The section is swept through the corner with a discontinuous trihedron.</summary>
    Modified,

    /// <summary>The pieces of the sweep are extended and cut at the corner (sharp corners).</summary>
    Right,

    /// <summary>The pieces are joined by a revolved piece (round corners).</summary>
    Round,
}
