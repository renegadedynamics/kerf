// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Context.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Tools;
using Occt;
using static Kerf.Engine.Meshing.Tools.MeshAlgoType;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The default mesher context: the model builder, edge discretizer, healer, pre-processor, face discretizer (with the
/// factory of the meshing algorithm type) and post-processor of BRepMesh.
/// </summary>
/// <remarks>
/// OCCT reads the environment variable <c>CSF_MeshAlgo</c> to choose the default algorithm. The port doesn't: a
/// result must not depend on the process environment, so the default is Watson's (decision 492).
/// </remarks>
internal class MeshContext : ToolContext
{
    /// <summary>A context with the tools of BRepMesh and the factory of the given meshing algorithm type.</summary>
    public MeshContext(MeshAlgoType meshType = MeshAlgoType.MeshAlgoType_DEFAULT)
    {
        if (meshType == MeshAlgoType.MeshAlgoType_DEFAULT)
        {
            // Kerf: CSF_MeshAlgo isn't read (decision 492)
            meshType = MeshAlgoType.MeshAlgoType_Watson;
        }

        MeshAlgoFactory? algoFactory = null;
        switch (meshType)
        {
            case MeshAlgoType.MeshAlgoType_DEFAULT:
            case MeshAlgoType.MeshAlgoType_Watson:
                algoFactory = new DefaultMeshAlgoFactory();
                break;
            case MeshAlgoType.MeshAlgoType_Delabella:
                algoFactory = new DelabellaMeshAlgoFactory();
                break;
        }

        SetModelBuilder(new MeshModelBuilder());
        SetEdgeDiscret(new EdgeDiscret());
        SetModelHealer(new ModelHealer());
        SetPreProcessor(new ModelPreProcessor());
        SetFaceDiscret(new FaceDiscret(algoFactory));
        SetPostProcessor(new ModelPostProcessor());
    }
}
