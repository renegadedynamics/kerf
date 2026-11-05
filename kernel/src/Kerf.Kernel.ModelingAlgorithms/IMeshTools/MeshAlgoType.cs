// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgoType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>The built-in 2D triangulation algorithms (factories of <see cref="MeshAlgoFactory"/>).</summary>
internal enum MeshAlgoType
{
    /// <summary>The global default: <see cref="MeshAlgoType_Watson"/> (OCCT.NET doesn't read <c>CSF_MeshAlgo</c>, decision 492).</summary>
    MeshAlgoType_DEFAULT = -1,

    /// <summary>2D Delaunay triangulation by Watson's algorithm (BRepMesh_MeshAlgoFactory).</summary>
    MeshAlgoType_Watson = 0,

    /// <summary>2D Delaunay triangulation by the Delabella algorithm (BRepMesh_DelabellaMeshAlgoFactory).</summary>
    MeshAlgoType_Delabella,
}
