// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An algorithm making the mesh of a discrete face.</summary>
public abstract class IMeshTools_MeshAlgo : Standard_Transient
{
    /// <summary>An algorithm.</summary>
    protected IMeshTools_MeshAlgo()
    {
    }

    /// <summary>Meshes the face.</summary>
    public abstract void Perform(IMeshData_Face theDFace, in IMeshTools_Parameters theParameters, Message_ProgressRange theRange);
}
