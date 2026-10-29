// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Status.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The statuses of the discrete model: flags, combined in a status mask.</summary>
public enum IMeshData_Status
{
    /// <summary>Mesh generation is successful.</summary>
    IMeshData_NoError = 0x0,

    /// <summary>An open wire, which can lead to incorrect results.</summary>
    IMeshData_OpenWire = 0x1,

    /// <summary>Self-intersections on a discretized wire, which can lead to incorrect results.</summary>
    IMeshData_SelfIntersectingWire = 0x2,

    /// <summary>Failed to generate the mesh of some faces.</summary>
    IMeshData_Failure = 0x4,

    /// <summary>The deflection of some edges has been decreased because of interference in the discrete model.</summary>
    IMeshData_ReMesh = 0x8,

    /// <summary>A badly oriented wire, which can lead to incorrect results.</summary>
    IMeshData_UnorientedWire = 0x10,

    /// <summary>The discrete model has too few boundary points to generate a mesh.</summary>
    IMeshData_TooFewPoints = 0x20,

    /// <summary>The existing triangulation of some faces has a greater deflection than asked.</summary>
    IMeshData_Outdated = 0x40,

    /// <summary>The existing triangulation of some faces is reused, as it fits the asked deflection.</summary>
    IMeshData_Reused = 0x80,

    /// <summary>User break.</summary>
    IMeshData_UserBreak = 0x100,
}
