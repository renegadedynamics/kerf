// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_ConstructionError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>The status of a 2D fillet or chamfer construction (<see cref="PlanarBuilder"/>).</summary>
internal enum ConstructionError
{
    /// <summary>The face is not planar.</summary>
    NotPlanar,
    /// <summary>The face is null.</summary>
    NoFace,
    /// <summary>The two faces used for the initialisation are incompatible.</summary>
    InitialisationError,
    /// <summary>The parameters (distances or angle of a chamfer) are less than or equal to zero.</summary>
    ParametersError,
    /// <summary>The initialisation has been successful.</summary>
    Ready,
    IsDone,
    /// <summary>The algorithm could not find a solution.</summary>
    ComputationError,
    /// <summary>The vertex given to locate the fillet or the chamfer is not connected to two edges.</summary>
    ConnexionError,
    /// <summary>The two edges connected to the vertex are tangent.</summary>
    TangencyError,
    /// <summary>The first edge is degenerated.</summary>
    FirstEdgeDegenerated,
    /// <summary>The last edge is degenerated.</summary>
    LastEdgeDegenerated,
    /// <summary>The two edges are degenerated.</summary>
    BothEdgesDegenerated,
    /// <summary>
    /// One or both of the edges connected to the vertex is a fillet or a chamfer, or is not a line or a circle.
    /// </summary>
    NotAuthorized,
}
