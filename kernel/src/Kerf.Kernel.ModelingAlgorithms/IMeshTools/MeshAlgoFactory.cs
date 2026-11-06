// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>A factory of triangulation algorithms, chosen by the type of the face's surface.</summary>
internal abstract class MeshAlgoFactory : Transient
{
    /// <summary>A factory.</summary>
    protected MeshAlgoFactory()
    {
    }

    /// <summary>A meshing algorithm for the given type of surface.</summary>
    public abstract MeshAlgo? GetAlgo(SurfaceKind surfaceType, in ToolParameters parameters);
}
