// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_EdgeError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FaceError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ShellError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_WireError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_PipeError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ShapeModification.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>The construction status of <see cref="MakeEdge"/> and <see cref="MakeEdge2d"/>.</summary>
public enum EdgeError
{
    /// <summary>The edge is built.</summary>
    EdgeDone,

    /// <summary>A point or vertex couldn't be projected on the curve.</summary>
    PointProjectionFailed,

    /// <summary>A parameter is outside the curve's range.</summary>
    ParameterOutOfRange,

    /// <summary>The two points of a closed curve differ (they must coincide).</summary>
    DifferentPointsOnClosedCurve,

    /// <summary>A point is given at an infinite parameter.</summary>
    PointWithInfiniteParameter,

    /// <summary>A point and the parameter given with it don't agree.</summary>
    DifferentsPointAndParameter,

    /// <summary>A line through two identical points.</summary>
    LineThroughIdenticPoints,
}

/// <summary>The construction status of <see cref="MakeFace"/>.</summary>
public enum FaceError
{
    /// <summary>The face is built.</summary>
    FaceDone,

    /// <summary>No face: no surface or no wire.</summary>
    NoFace,

    /// <summary>The wire isn't planar (OnlyPlane).</summary>
    NotPlanar,

    /// <summary>A curve couldn't be projected on the surface.</summary>
    CurveProjectionFailed,

    /// <summary>The parameters are outside the surface's bounds.</summary>
    ParametersOutOfRange,
}

/// <summary>The construction status of <see cref="MakeShell"/>.</summary>
public enum ShellError
{
    /// <summary>The shell is built.</summary>
    ShellDone,

    /// <summary>No shell was built.</summary>
    EmptyShell,

    /// <summary>The shell isn't connected.</summary>
    DisconnectedShell,

    /// <summary>The parameters are outside the surface's bounds.</summary>
    ShellParametersOutOfRange,
}

/// <summary>The construction status of <see cref="MakeWire"/>.</summary>
public enum WireError
{
    /// <summary>The wire is built.</summary>
    WireDone,

    /// <summary>No edge was given.</summary>
    EmptyWire,

    /// <summary>The last edge added isn't connected to the wire.</summary>
    DisconnectedWire,

    /// <summary>The wire is non-manifold.</summary>
    NonManifoldWire,
}

/// <summary>The status of the pipe algorithms (BRepOffsetAPI's).</summary>
public enum PipeError
{
    PipeDone,
    PipeNotDone,
    PlaneNotIntersectGuide,
    ImpossibleContact,
}

/// <summary>The state of a face after a shape construction (BRepBuilderAPI_MakeShape's descendants).</summary>
internal enum ShapeModification
{
    Preserved,
    Deleted,
    Trimmed,
    Merged,
    BoundaryModified,
}
