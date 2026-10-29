// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The default factory of face algorithms (Watson's Delaunay), of a complexity depending on the type of the face's
/// surface.
/// </summary>
public class BRepMesh_MeshAlgoFactory : IMeshTools_MeshAlgoFactory
{
    /// <summary>A factory.</summary>
    public BRepMesh_MeshAlgoFactory()
    {
    }

    /// <summary>A new meshing algorithm for the type of surface.</summary>
    public override IMeshTools_MeshAlgo? GetAlgo(GeomAbs_SurfaceType theSurfaceType,
                                                 in IMeshTools_Parameters theParameters)
    {
        switch (theSurfaceType)
        {
            case GeomAbs_Plane:
                return theParameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_DefaultRangeSplitter>()
                         : (theParameters.InternalVerticesMode
                              ? new BRepMesh_DelaunayNodeInsertionMeshAlgo<BRepMesh_DefaultRangeSplitter>()
                              : new BRepMesh_DelaunayBaseMeshAlgo());

            case GeomAbs_Sphere:
                return theParameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_SphereRangeSplitter>()
                         : new BRepMesh_DelaunayNodeInsertionMeshAlgo<BRepMesh_SphereRangeSplitter>();

            case GeomAbs_Cylinder:
                return theParameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_CylinderRangeSplitter>()
                         : (theParameters.InternalVerticesMode
                              ? new BRepMesh_DelaunayNodeInsertionMeshAlgo<BRepMesh_CylinderRangeSplitter>()
                              : new BRepMesh_DelaunayBaseMeshAlgo());

            case GeomAbs_Cone:
                return theParameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_ConeRangeSplitter>()
                         : new BRepMesh_DelaunayNodeInsertionMeshAlgo<BRepMesh_ConeRangeSplitter>();

            case GeomAbs_Torus:
                return theParameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_TorusRangeSplitter>()
                         : new BRepMesh_DelaunayNodeInsertionMeshAlgo<BRepMesh_TorusRangeSplitter>();

            case GeomAbs_SurfaceOfRevolution:
                return new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_BoundaryParamsRangeSplitter>();

            case GeomAbs_SurfaceOfExtrusion:
                return new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_ExtrusionRangeSplitter>();

            case GeomAbs_BezierSurface:
            case GeomAbs_BSplineSurface:
                return new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_NURBSRangeSplitter>();

            case GeomAbs_OffsetSurface:
            case GeomAbs_OtherSurface:
            default:
                return new BRepMesh_DelaunayDeflectionControlMeshAlgo<BRepMesh_UndefinedRangeSplitter>();
        }
    }
}
