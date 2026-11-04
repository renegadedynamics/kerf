// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Error.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Offsets;

/// <summary>The error statuses of the 3D offset.</summary>
internal enum Error
{
    NoError,
    UnknownError,
    BadNormalsOnGeometry,
    C0Geometry,
    NullOffset,
    NotConnectedShell,
    /// <summary>An exception while trimming edges.</summary>
    CannotTrimEdges,
    /// <summary>An exception while fusing vertices.</summary>
    CannotFuseVertices,
    /// <summary>An exception while extending edges.</summary>
    CannotExtentEdge,
    /// <summary>The user broke off.</summary>
    UserBreak,
    /// <summary>Different connectivity of faces along an edge: partially C0 and tangent.</summary>
    MixedConnectivity,
}
