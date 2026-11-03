// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>A factory of triangulation algorithms, chosen by the type of the face's surface.</summary>
public abstract class IMeshTools_MeshAlgoFactory : Transient
{
    /// <summary>A factory.</summary>
    protected IMeshTools_MeshAlgoFactory()
    {
    }

    /// <summary>A meshing algorithm for the given type of surface.</summary>
    public abstract IMeshTools_MeshAlgo? GetAlgo(SurfaceKind theSurfaceType, in IMeshTools_Parameters theParameters);
}
