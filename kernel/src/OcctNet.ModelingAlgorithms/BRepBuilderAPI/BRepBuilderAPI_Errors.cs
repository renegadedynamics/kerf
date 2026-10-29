// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_EdgeError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FaceError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ShellError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_WireError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_PipeError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ShapeModification.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_TransitionMode.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The construction status of <see cref="BRepBuilderAPI_MakeEdge"/> and <see cref="BRepBuilderAPI_MakeEdge2d"/>.</summary>
public enum BRepBuilderAPI_EdgeError
{
    /// <summary>The edge is built.</summary>
    BRepBuilderAPI_EdgeDone,

    /// <summary>A point or vertex couldn't be projected on the curve.</summary>
    BRepBuilderAPI_PointProjectionFailed,

    /// <summary>A parameter is outside the curve's range.</summary>
    BRepBuilderAPI_ParameterOutOfRange,

    /// <summary>The two points of a closed curve differ (they must coincide).</summary>
    BRepBuilderAPI_DifferentPointsOnClosedCurve,

    /// <summary>A point is given at an infinite parameter.</summary>
    BRepBuilderAPI_PointWithInfiniteParameter,

    /// <summary>A point and the parameter given with it don't agree.</summary>
    BRepBuilderAPI_DifferentsPointAndParameter,

    /// <summary>A line through two identical points.</summary>
    BRepBuilderAPI_LineThroughIdenticPoints,
}

/// <summary>The construction status of <see cref="BRepBuilderAPI_MakeFace"/>.</summary>
public enum BRepBuilderAPI_FaceError
{
    /// <summary>The face is built.</summary>
    BRepBuilderAPI_FaceDone,

    /// <summary>No face: no surface or no wire.</summary>
    BRepBuilderAPI_NoFace,

    /// <summary>The wire isn't planar (OnlyPlane).</summary>
    BRepBuilderAPI_NotPlanar,

    /// <summary>A curve couldn't be projected on the surface.</summary>
    BRepBuilderAPI_CurveProjectionFailed,

    /// <summary>The parameters are outside the surface's bounds.</summary>
    BRepBuilderAPI_ParametersOutOfRange,
}

/// <summary>The construction status of <see cref="BRepBuilderAPI_MakeShell"/>.</summary>
public enum BRepBuilderAPI_ShellError
{
    /// <summary>The shell is built.</summary>
    BRepBuilderAPI_ShellDone,

    /// <summary>No shell was built.</summary>
    BRepBuilderAPI_EmptyShell,

    /// <summary>The shell isn't connected.</summary>
    BRepBuilderAPI_DisconnectedShell,

    /// <summary>The parameters are outside the surface's bounds.</summary>
    BRepBuilderAPI_ShellParametersOutOfRange,
}

/// <summary>The construction status of <see cref="BRepBuilderAPI_MakeWire"/>.</summary>
public enum BRepBuilderAPI_WireError
{
    /// <summary>The wire is built.</summary>
    BRepBuilderAPI_WireDone,

    /// <summary>No edge was given.</summary>
    BRepBuilderAPI_EmptyWire,

    /// <summary>The last edge added isn't connected to the wire.</summary>
    BRepBuilderAPI_DisconnectedWire,

    /// <summary>The wire is non-manifold.</summary>
    BRepBuilderAPI_NonManifoldWire,
}

/// <summary>The status of the pipe algorithms (BRepOffsetAPI's).</summary>
public enum BRepBuilderAPI_PipeError
{
    BRepBuilderAPI_PipeDone,
    BRepBuilderAPI_PipeNotDone,
    BRepBuilderAPI_PlaneNotIntersectGuide,
    BRepBuilderAPI_ImpossibleContact,
}

/// <summary>The state of a face after a shape construction (BRepBuilderAPI_MakeShape's descendants).</summary>
public enum BRepBuilderAPI_ShapeModification
{
    BRepBuilderAPI_Preserved,
    BRepBuilderAPI_Deleted,
    BRepBuilderAPI_Trimmed,
    BRepBuilderAPI_Merged,
    BRepBuilderAPI_BoundaryModified,
}

/// <summary>How a sweep treats the discontinuities of its spine.</summary>
public enum BRepBuilderAPI_TransitionMode
{
    /// <summary>Discontinuities are treated by modification of the sweeping mode.</summary>
    BRepBuilderAPI_Transformed,

    /// <summary>Discontinuities are treated like right corners: the pipes are extended and intersected.</summary>
    BRepBuilderAPI_RightCorner,

    /// <summary>Discontinuities are treated like round corners: the pipes are filled.</summary>
    BRepBuilderAPI_RoundCorner,
}
