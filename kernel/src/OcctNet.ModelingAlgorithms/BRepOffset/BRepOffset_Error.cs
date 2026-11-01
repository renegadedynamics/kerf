// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Error.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error statuses of the 3D offset.</summary>
public enum BRepOffset_Error
{
    BRepOffset_NoError,
    BRepOffset_UnknownError,
    BRepOffset_BadNormalsOnGeometry,
    BRepOffset_C0Geometry,
    BRepOffset_NullOffset,
    BRepOffset_NotConnectedShell,
    /// <summary>An exception while trimming edges.</summary>
    BRepOffset_CannotTrimEdges,
    /// <summary>An exception while fusing vertices.</summary>
    BRepOffset_CannotFuseVertices,
    /// <summary>An exception while extending edges.</summary>
    BRepOffset_CannotExtentEdge,
    /// <summary>The user broke off.</summary>
    BRepOffset_UserBreak,
    /// <summary>Different connectivity of faces along an edge: partially C0 and tangent.</summary>
    BRepOffset_MixedConnectivity,
}
