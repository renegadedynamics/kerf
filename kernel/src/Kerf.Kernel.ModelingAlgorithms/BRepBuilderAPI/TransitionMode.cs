// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_TransitionMode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;

namespace Kerf.Construction;

/// <summary>How a sweep treats the discontinuities of its spine.</summary>
public enum TransitionMode
{
    /// <summary>Discontinuities are treated by modification of the sweeping mode.</summary>
    Transformed,

    /// <summary>Discontinuities are treated like right corners: the pipes are extended and intersected.</summary>
    RightCorner,

    /// <summary>Discontinuities are treated like round corners: the pipes are filled.</summary>
    RoundCorner,
}
