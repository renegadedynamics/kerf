// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_ConstructionError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The status of a 2D fillet or chamfer construction (<see cref="ChFi2d_Builder"/>).</summary>
public enum ChFi2d_ConstructionError
{
    /// <summary>The face is not planar.</summary>
    ChFi2d_NotPlanar,
    /// <summary>The face is null.</summary>
    ChFi2d_NoFace,
    /// <summary>The two faces used for the initialisation are incompatible.</summary>
    ChFi2d_InitialisationError,
    /// <summary>The parameters (distances or angle of a chamfer) are less than or equal to zero.</summary>
    ChFi2d_ParametersError,
    /// <summary>The initialisation has been successful.</summary>
    ChFi2d_Ready,
    ChFi2d_IsDone,
    /// <summary>The algorithm could not find a solution.</summary>
    ChFi2d_ComputationError,
    /// <summary>The vertex given to locate the fillet or the chamfer is not connected to two edges.</summary>
    ChFi2d_ConnexionError,
    /// <summary>The two edges connected to the vertex are tangent.</summary>
    ChFi2d_TangencyError,
    /// <summary>The first edge is degenerated.</summary>
    ChFi2d_FirstEdgeDegenerated,
    /// <summary>The last edge is degenerated.</summary>
    ChFi2d_LastEdgeDegenerated,
    /// <summary>The two edges are degenerated.</summary>
    ChFi2d_BothEdgesDegenerated,
    /// <summary>
    /// One or both of the edges connected to the vertex is a fillet or a chamfer, or is not a line or a circle.
    /// </summary>
    ChFi2d_NotAuthorized,
}
