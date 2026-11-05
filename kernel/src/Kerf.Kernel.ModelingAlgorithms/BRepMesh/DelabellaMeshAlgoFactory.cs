// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaMeshAlgoFactory.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaMeshAlgoFactory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The factory of Delabella-based face algorithms, of a complexity depending on the type of the face's surface.
/// </summary>
/// <remarks>
/// OCCT's local <c>NodeInsertionMeshAlgo&lt;RS&gt;::Type</c> and <c>DeflectionControlMeshAlgo&lt;RS&gt;::Type</c> over
/// <c>BRepMesh_CustomDelaunayBaseMeshAlgo&lt;BRepMesh_DelabellaBaseMeshAlgo&gt;</c> are the <c>...OfDelabella</c>
/// generics; its <c>Default...</c> types are the Watson ones.
/// </remarks>
internal class DelabellaMeshAlgoFactory : MeshAlgoFactory
{
    /// <summary>A factory.</summary>
    public DelabellaMeshAlgoFactory()
    {
    }

    /// <summary>A new meshing algorithm for the type of surface.</summary>
    public override MeshAlgo? GetAlgo(SurfaceKind surfaceType,
                                                 in ToolParameters parameters)
    {
        switch (surfaceType)
        {
            case Plane:
                return parameters.InternalVerticesMode
                         ? new DelaunayNodeInsertionMeshAlgoOfDelabella<DefaultRangeSplitter>()
                         : new DelabellaBaseMeshAlgo();

            case SurfaceKind.Sphere:
            {
                DelaunayNodeInsertionMeshAlgoOfDelabella<SphereRangeSplitter> meshAlgo = new();
                meshAlgo.SetPreProcessSurfaceNodes(true);
                return meshAlgo;
            }

            case SurfaceKind.Cylinder:
                return parameters.InternalVerticesMode
                         ? new DelaunayNodeInsertionMeshAlgo<CylinderRangeSplitter>()
                         : new DelaunayBaseMeshAlgo();

            case SurfaceKind.Cone:
            {
                DelaunayNodeInsertionMeshAlgoOfDelabella<ConeRangeSplitter> meshAlgo = new();
                meshAlgo.SetPreProcessSurfaceNodes(true);
                return meshAlgo;
            }

            case SurfaceKind.Torus:
            {
                DelaunayNodeInsertionMeshAlgoOfDelabella<TorusRangeSplitter> meshAlgo = new();
                meshAlgo.SetPreProcessSurfaceNodes(true);
                return meshAlgo;
            }

            case SurfaceOfRevolution:
            {
                DelaunayDeflectionControlMeshAlgoOfDelabella<BoundaryParamsRangeSplitter> meshAlgo = new();
                meshAlgo.SetPreProcessSurfaceNodes(true);
                return meshAlgo;
            }

            default:
            {
                // OCCT: surfaces of extrusion, offset and other surfaces get the NURBS splitter too, which asks a
                // surface of extrusion for its V poles (Standard_NoSuchObject): the face is left unmeshed
                DelaunayDeflectionControlMeshAlgoOfDelabella<NurbsRangeSplitter> meshAlgo = new();
                meshAlgo.SetPreProcessSurfaceNodes(true);
                return meshAlgo;
            }
        }
    }
}
