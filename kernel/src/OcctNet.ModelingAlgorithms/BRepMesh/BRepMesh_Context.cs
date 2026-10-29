// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Context.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshTools_MeshAlgoType;

namespace Occt;

/// <summary>
/// The default mesher context: the model builder, edge discretizer, healer, pre-processor, face discretizer (with the
/// factory of the meshing algorithm type) and post-processor of BRepMesh.
/// </summary>
/// <remarks>
/// OCCT reads the environment variable <c>CSF_MeshAlgo</c> to choose the default algorithm. The port doesn't: a
/// result must not depend on the process environment, so the default is Watson's (decision 492).
/// </remarks>
public class BRepMesh_Context : IMeshTools_Context
{
    /// <summary>A context with the tools of BRepMesh and the factory of the given meshing algorithm type.</summary>
    public BRepMesh_Context(IMeshTools_MeshAlgoType theMeshType = IMeshTools_MeshAlgoType_DEFAULT)
    {
        if (theMeshType == IMeshTools_MeshAlgoType_DEFAULT)
        {
            // OCCT.NET: CSF_MeshAlgo isn't read (decision 492)
            theMeshType = IMeshTools_MeshAlgoType_Watson;
        }

        IMeshTools_MeshAlgoFactory? aAlgoFactory = null;
        switch (theMeshType)
        {
            case IMeshTools_MeshAlgoType_DEFAULT:
            case IMeshTools_MeshAlgoType_Watson:
                aAlgoFactory = new BRepMesh_MeshAlgoFactory();
                break;
            case IMeshTools_MeshAlgoType_Delabella:
                aAlgoFactory = new BRepMesh_DelabellaMeshAlgoFactory();
                break;
        }

        SetModelBuilder(new BRepMesh_ModelBuilder());
        SetEdgeDiscret(new BRepMesh_EdgeDiscret());
        SetModelHealer(new BRepMesh_ModelHealer());
        SetPreProcessor(new BRepMesh_ModelPreProcessor());
        SetFaceDiscret(new BRepMesh_FaceDiscret(aAlgoFactory));
        SetPostProcessor(new BRepMesh_ModelPostProcessor());
    }
}
