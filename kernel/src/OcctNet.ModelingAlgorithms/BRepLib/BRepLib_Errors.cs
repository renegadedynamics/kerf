// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_EdgeError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FaceError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ShellError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_WireError.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ShapeModification.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Errors of <see cref="BRepLib_MakeEdge"/> and <see cref="BRepLib_MakeEdge2d"/>.</summary>
public enum BRepLib_EdgeError
{
    BRepLib_EdgeDone,
    BRepLib_PointProjectionFailed,
    BRepLib_ParameterOutOfRange,
    BRepLib_DifferentPointsOnClosedCurve,
    BRepLib_PointWithInfiniteParameter,
    BRepLib_DifferentsPointAndParameter,
    BRepLib_LineThroughIdenticPoints,
}

/// <summary>Errors of <see cref="BRepLib_MakeFace"/>.</summary>
public enum BRepLib_FaceError
{
    BRepLib_FaceDone,
    BRepLib_NoFace,
    BRepLib_NotPlanar,
    BRepLib_CurveProjectionFailed,
    BRepLib_ParametersOutOfRange,
}

/// <summary>Errors of <see cref="BRepLib_MakeShell"/>.</summary>
public enum BRepLib_ShellError
{
    BRepLib_ShellDone,
    BRepLib_EmptyShell,
    BRepLib_DisconnectedShell,
    BRepLib_ShellParametersOutOfRange,
}

/// <summary>Errors of <see cref="BRepLib_MakeWire"/>.</summary>
public enum BRepLib_WireError
{
    BRepLib_WireDone,
    BRepLib_EmptyWire,
    BRepLib_DisconnectedWire,
    BRepLib_NonManifoldWire,
}

/// <summary>Modification type after a topological operation.</summary>
public enum BRepLib_ShapeModification
{
    BRepLib_Preserved,
    BRepLib_Deleted,
    BRepLib_Trimmed,
    BRepLib_Merged,
    BRepLib_BoundaryModified,
}
