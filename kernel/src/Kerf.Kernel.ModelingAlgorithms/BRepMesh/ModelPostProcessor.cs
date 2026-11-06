// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPostProcessor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPostProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Post-processes the discrete model: finishes each new triangulation (its deflection and the parameters it was made
/// with) and attaches it to its face, then stores the edges' polygons on the faces' triangulations (and the free edges'
/// 3D polygons) into the shape.
/// </summary>
/// <remarks>
/// <para>
/// Kerf (unit 10L5): OCCT's face mesher attaches each triangulation, this commits the edges' polygons, then estimates
/// each triangulation's deflection from it and its degenerated edges' polygons and sets it in place. Here the face
/// mesher leaves its triangulation with the face's model (<see cref="FaceModel.MadeTriangulation"/>), and this
/// estimates the deflection first, taking the degenerated edges' nodes from the model (the nodes of the polygons it
/// commits), so that a triangulation is finished before it is attached and is attached frozen, as all geometry is: the
/// same numbers, in a different order. Writes into the shape being meshed, as OCCT does, through
/// <see cref="ShapeTool"/>. Faces that failed or whose triangulation was reused (frozen, from the input) are skipped,
/// so a reused triangulation is never written.
/// </para>
/// <para>
/// The edge loop is serial, as OCCT forces it ("data races on edges sharing the same TShape"). The deflections are
/// estimated in parallel when the parameters ask for it: each body reads only its face's triangulation, and they share
/// one immutable <see cref="TriangulationParameters"/>, as OCCT does. The triangulations are attached serially.
/// </para>
/// </remarks>
internal class ModelPostProcessor : ModelAlgo
{
    /// <summary>A post-processor.</summary>
    public ModelPostProcessor()
    {
    }

    /// <summary>Post-processes the model; false without a model.</summary>
    protected override bool PerformInternal(ModelModel? model,
                                            in ToolParameters parameters,
                                            ProgressRange range)
    {
        _ = range;
        if (model is null)
        {
            return false;
        }

        // Kerf: each triangulation finished, then attached, before the polygons on it (see the remarks)
        OsdParallel.For(0,
                         model.FacesNb(),
                         new DeflectionEstimator(model, parameters).Process,
                         !parameters.InParallel);
        for (int faceIndex = 0; faceIndex < model.FacesNb(); ++faceIndex)
        {
            FaceModel dFace = model.GetFace(faceIndex);
            if (dFace.MadeTriangulation is { } made)
            {
                dFace.MadeTriangulation = null;
                ShapeTool.AttachToFace(dFace.GetFace(), made);
            }
        }

        // TODO: Force single threaded solution due to data races on edges sharing the same TShape
        OsdParallel.For(0,
                         model.EdgesNb(),
                         new PolygonCommitter(model).Process,
                         true /*!theParameters.InParallel*/);
        return true;
    }

    // Commits 3D polygons and polygons on triangulations for corresponding edges.
    private sealed class PolygonCommitter
    {
        private readonly ModelModel _model;

        public PolygonCommitter(ModelModel model)
        {
            _model = model;
        }

        // Main functor.
        public void Process(int edgeIndex)
        {
            EdgeModel dEdge = _model.GetEdge(edgeIndex);
            if (dEdge.GetCurve().ParametersNb() == 0)
            {
                return;
            }

            if (dEdge.IsFree)
            {
                if (!dEdge.IsSet(StatusModel.Reused))
                {
                    CommitPolygon3D(dEdge);
                }
            }
            else
            {
                CommitPolygons(dEdge);
            }
        }

        // Commits 3d polygon to topological edge
        private static void CommitPolygon3D(EdgeModel dEdge)
        {
            CurveModel curve = dEdge.GetCurve();

            NCollection_Array1<Point3D> nodes = new(1, curve.ParametersNb());
            NCollection_Array1<double> uvNodes = new(1, curve.ParametersNb());
            for (int i = 1; i <= curve.ParametersNb(); ++i)
            {
                nodes[i] = curve.GetPoint(i - 1);
                uvNodes[i] = curve.GetParameter(i - 1);
            }

            Polygon3D poly3D = new(nodes, uvNodes);
            poly3D.Deflection(dEdge.GetDeflection());

            ShapeTool.UpdateEdge(dEdge.GetEdge(), poly3D);
        }

        // Commits all polygons on triangulations correspondent to the given edge.
        private static void CommitPolygons(EdgeModel dEdge)
        {
            // Collect pcurves associated with the given edge on the specific surface.
            IndexedMap<FaceModel, NCollection_List<PCurveModel>> mapOfPCurves =
                new(ReferenceEqualityComparer.Instance);
            for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
            {
                PCurveModel pCurve = dEdge.GetPCurve(pCurveIt);
                FaceModel dFacePtr = pCurve.GetFace();
                FaceModel dFace = dFacePtr;
                if (dFace.IsSet(StatusModel.Failure) || dFace.IsSet(StatusModel.Reused))
                {
                    continue;
                }

                if (!mapOfPCurves.ContainsKey(dFacePtr))
                {
                    mapOfPCurves.Add(dFacePtr, new NCollection_List<PCurveModel>());
                }

                NCollection_List<PCurveModel> pCurves = mapOfPCurves.GetValue(dFacePtr);
                pCurves.Append(pCurve);
            }

            // Commit polygons related to separate face.
            Edge edge = dEdge.GetEdge();
            for (int polygonIt = 0; polygonIt < mapOfPCurves.Count; ++polygonIt)
            {
                Face face = mapOfPCurves.KeyAt(polygonIt).GetFace();

                Triangulation? triangulation = Tool.Triangulation(face, out Placement loc);

                if (triangulation is not null)
                {
                    NCollection_List<PCurveModel> pCurves = mapOfPCurves.ValueAt(polygonIt);
                    // OCCT: the first pcurve becomes the forward edge's polygon, the last the reversed edge's, whatever
                    // their orientations (occt-issues.md).
                    if (pCurves.Size() == 2)
                    {
                        ShapeTool.UpdateEdge(edge,
                                                      CollectPolygon(pCurves.First(), dEdge.GetDeflection()),
                                                      CollectPolygon(pCurves.Last(), dEdge.GetDeflection()),
                                                      triangulation,
                                                      loc);
                    }
                    else
                    {
                        ShapeTool.UpdateEdge(edge,
                                                      CollectPolygon(pCurves.First(), dEdge.GetDeflection()),
                                                      triangulation,
                                                      loc);
                    }
                }
            }
        }

        // Collects polygonal data for the given pcurve
        private static PolygonOnTriangulation CollectPolygon(PCurveModel pCurve, double deflection)
        {
            NCollection_Array1<int> nodes = new(1, pCurve.ParametersNb());
            NCollection_Array1<double> aParams = new(1, pCurve.ParametersNb());
            for (int i = 1; i <= pCurve.ParametersNb(); ++i)
            {
                nodes[i] = pCurve.GetIndex(i - 1);
                aParams[i] = pCurve.GetParameter(i - 1);
            }

            PolygonOnTriangulation polygon = new(nodes, aParams);

            polygon.Deflection(deflection);
            return polygon;
        }
    }

    // Estimates and updates deflection of triangulations for corresponding faces.
    private sealed class DeflectionEstimator
    {
        private readonly ModelModel _model;
        private readonly TriangulationParameters _params;
        private readonly Dictionary<FaceModel, HashSet<int>> _degeneratedNodes;

        public DeflectionEstimator(ModelModel model, in ToolParameters theParams)
        {
            _model = model;
            _params = new TriangulationParameters(theParams.Deflection,
                                                        theParams.Angle,
                                                        theParams.MinSize);
            _degeneratedNodes = DegeneratedNodes(model);
        }

        // Main functor.
        public void Process(int faceIndex)
        {
            FaceModel dFace = _model.GetFace(faceIndex);
            if (dFace.IsSet(StatusModel.Failure) || dFace.IsSet(StatusModel.Reused))
            {
                return;
            }

            // Kerf: the triangulation the face's mesher made, not yet attached (BRepLib::UpdateDeflection of the face,
            // on it)
            Triangulation? triangulation = dFace.MadeTriangulation;
            if (triangulation is null)
            {
                return;
            }

            Face face = dFace.GetFace();
            if (Tool.Surface(face) is not null && triangulation.HasUVNodes)
            {
                triangulation.Deflection(BrepLibrary.EstimateDeflection(face, triangulation, face.Placement,
                    _degeneratedNodes.GetValueOrDefault(dFace) ?? []));
            }

            triangulation.Parameters(_params);
        }

        // Kerf: the nodes of each face's degenerated edges, as the polygons PolygonCommitter commits hold them (the
        // first pcurve on the face), which BRepLib::UpdateDeflection reads off the attached polygons
        private static Dictionary<FaceModel, HashSet<int>> DegeneratedNodes(ModelModel model)
        {
            var nodes = new Dictionary<FaceModel, HashSet<int>>(ReferenceEqualityComparer.Instance);
            for (int edgeIndex = 0; edgeIndex < model.EdgesNb(); ++edgeIndex)
            {
                EdgeModel dEdge = model.GetEdge(edgeIndex);
                if (dEdge.IsFree || dEdge.GetCurve().ParametersNb() == 0 || !Tool.Degenerated(dEdge.GetEdge()))
                {
                    continue;
                }

                var seen = new HashSet<FaceModel>(ReferenceEqualityComparer.Instance);
                for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
                {
                    PCurveModel pCurve = dEdge.GetPCurve(pCurveIt);
                    FaceModel dFace = pCurve.GetFace();
                    if (!seen.Add(dFace))
                    {
                        continue;
                    }

                    if (!nodes.TryGetValue(dFace, out HashSet<int>? faceNodes))
                    {
                        faceNodes = [];
                        nodes.Add(dFace, faceNodes);
                    }

                    for (int i = 0; i < pCurve.ParametersNb(); ++i)
                    {
                        faceNodes.Add(pCurve.GetIndex(i));
                    }
                }
            }

            return nodes;
        }
    }
}
