// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The default factory of face algorithms (Watson's Delaunay), of a complexity depending on the type of the face's
/// surface.
/// </summary>
internal class DefaultMeshAlgoFactory : MeshAlgoFactory
{
    /// <summary>A factory.</summary>
    public DefaultMeshAlgoFactory()
    {
    }

    /// <summary>A new meshing algorithm for the type of surface.</summary>
    public override MeshAlgo? GetAlgo(SurfaceKind surfaceType,
                                                 in ToolParameters parameters)
    {
        switch (surfaceType)
        {
            case Plane:
                return parameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new DelaunayDeflectionControlMeshAlgo<DefaultRangeSplitter>()
                         : (parameters.InternalVerticesMode
                              ? new DelaunayNodeInsertionMeshAlgo<DefaultRangeSplitter>()
                              : new DelaunayBaseMeshAlgo());

            case SurfaceKind.Sphere:
                return parameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new DelaunayDeflectionControlMeshAlgo<SphereRangeSplitter>()
                         : new DelaunayNodeInsertionMeshAlgo<SphereRangeSplitter>();

            case SurfaceKind.Cylinder:
                return parameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new DelaunayDeflectionControlMeshAlgo<CylinderRangeSplitter>()
                         : (parameters.InternalVerticesMode
                              ? new DelaunayNodeInsertionMeshAlgo<CylinderRangeSplitter>()
                              : new DelaunayBaseMeshAlgo());

            case SurfaceKind.Cone:
                return parameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new DelaunayDeflectionControlMeshAlgo<ConeRangeSplitter>()
                         : new DelaunayNodeInsertionMeshAlgo<ConeRangeSplitter>();

            case SurfaceKind.Torus:
                return parameters.EnableControlSurfaceDeflectionAllSurfaces
                         ? new DelaunayDeflectionControlMeshAlgo<TorusRangeSplitter>()
                         : new DelaunayNodeInsertionMeshAlgo<TorusRangeSplitter>();

            case SurfaceOfRevolution:
                return new DelaunayDeflectionControlMeshAlgo<BoundaryParamsRangeSplitter>();

            case SurfaceOfExtrusion:
                return new DelaunayDeflectionControlMeshAlgo<ExtrusionRangeSplitter>();

            case SurfaceKind.BezierSurface:
            case SurfaceKind.BSplineSurface:
                return new DelaunayDeflectionControlMeshAlgo<NurbsRangeSplitter>();

            case SurfaceKind.OffsetSurface:
            case OtherSurface:
            default:
                return new DelaunayDeflectionControlMeshAlgo<UndefinedRangeSplitter>();
        }
    }
}
