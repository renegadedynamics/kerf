// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Occt;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>An algorithm making the mesh of a discrete face.</summary>
internal abstract class MeshAlgo : Transient
{
    /// <summary>An algorithm.</summary>
    protected MeshAlgo()
    {
    }

    /// <summary>Meshes the face.</summary>
    public abstract void Perform(FaceModel dFace, in ToolParameters parameters, ProgressRange range);
}
