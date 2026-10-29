// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaMeshAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaMeshAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The factory of Delabella-based face algorithms, of a complexity depending on the type of the face's surface.
/// </summary>
/// <remarks>
/// OCCT's local <c>NodeInsertionMeshAlgo&lt;RS&gt;::Type</c> and <c>DeflectionControlMeshAlgo&lt;RS&gt;::Type</c> over
/// <c>BRepMesh_CustomDelaunayBaseMeshAlgo&lt;BRepMesh_DelabellaBaseMeshAlgo&gt;</c> are the <c>...OfDelabella</c>
/// generics; its <c>Default...</c> types are the Watson ones.
/// </remarks>
public class BRepMesh_DelabellaMeshAlgoFactory : IMeshTools_MeshAlgoFactory
{
    /// <summary>A factory.</summary>
    public BRepMesh_DelabellaMeshAlgoFactory()
    {
    }

    /// <summary>A new meshing algorithm for the type of surface.</summary>
    public override IMeshTools_MeshAlgo? GetAlgo(GeomAbs_SurfaceType theSurfaceType,
                                                 in IMeshTools_Parameters theParameters)
    {
        switch (theSurfaceType)
        {
            case GeomAbs_Plane:
                return theParameters.InternalVerticesMode
                         ? new BRepMesh_DelaunayNodeInsertionMeshAlgoOfDelabella<BRepMesh_DefaultRangeSplitter>()
                         : new BRepMesh_DelabellaBaseMeshAlgo();

            case GeomAbs_Sphere:
            {
                BRepMesh_DelaunayNodeInsertionMeshAlgoOfDelabella<BRepMesh_SphereRangeSplitter> aMeshAlgo = new();
                aMeshAlgo.SetPreProcessSurfaceNodes(true);
                return aMeshAlgo;
            }

            case GeomAbs_Cylinder:
                return theParameters.InternalVerticesMode
                         ? new BRepMesh_DelaunayNodeInsertionMeshAlgo<BRepMesh_CylinderRangeSplitter>()
                         : new BRepMesh_DelaunayBaseMeshAlgo();

            case GeomAbs_Cone:
            {
                BRepMesh_DelaunayNodeInsertionMeshAlgoOfDelabella<BRepMesh_ConeRangeSplitter> aMeshAlgo = new();
                aMeshAlgo.SetPreProcessSurfaceNodes(true);
                return aMeshAlgo;
            }

            case GeomAbs_Torus:
            {
                BRepMesh_DelaunayNodeInsertionMeshAlgoOfDelabella<BRepMesh_TorusRangeSplitter> aMeshAlgo = new();
                aMeshAlgo.SetPreProcessSurfaceNodes(true);
                return aMeshAlgo;
            }

            case GeomAbs_SurfaceOfRevolution:
            {
                BRepMesh_DelaunayDeflectionControlMeshAlgoOfDelabella<BRepMesh_BoundaryParamsRangeSplitter> aMeshAlgo = new();
                aMeshAlgo.SetPreProcessSurfaceNodes(true);
                return aMeshAlgo;
            }

            default:
            {
                BRepMesh_DelaunayDeflectionControlMeshAlgoOfDelabella<BRepMesh_NURBSRangeSplitter> aMeshAlgo = new();
                aMeshAlgo.SetPreProcessSurfaceNodes(true);
                return aMeshAlgo;
            }
        }
    }
}
