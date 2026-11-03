// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_EdgeError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FaceError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ShellError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_WireError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ShapeModification.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Construction.Library;

/// <summary>Errors of <see cref="LibraryMakeEdge"/> and <see cref="LibraryMakeEdge2d"/>.</summary>
public enum LibraryEdgeError
{
    EdgeDone,
    PointProjectionFailed,
    ParameterOutOfRange,
    DifferentPointsOnClosedCurve,
    PointWithInfiniteParameter,
    DifferentsPointAndParameter,
    LineThroughIdenticPoints,
}

/// <summary>Errors of <see cref="LibraryMakeFace"/>.</summary>
public enum LibraryFaceError
{
    FaceDone,
    NoFace,
    NotPlanar,
    CurveProjectionFailed,
    ParametersOutOfRange,
}

/// <summary>Errors of <see cref="LibraryMakeShell"/>.</summary>
public enum LibraryShellError
{
    ShellDone,
    EmptyShell,
    DisconnectedShell,
    ShellParametersOutOfRange,
}

/// <summary>Errors of <see cref="LibraryMakeWire"/>.</summary>
public enum LibraryWireError
{
    WireDone,
    EmptyWire,
    DisconnectedWire,
    NonManifoldWire,
}

/// <summary>Modification type after a topological operation.</summary>
public enum LibraryShapeModification
{
    Preserved,
    Deleted,
    Trimmed,
    Merged,
    BoundaryModified,
}
