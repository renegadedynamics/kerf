// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPostProcessor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelPostProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Post-processes the discrete model: stores the edges' polygons on the faces' triangulations (and the free edges' 3D
/// polygons) into the shape, then each new triangulation's deflection and the parameters it was made with.
/// </summary>
/// <remarks>
/// <para>
/// Writes into the shape being meshed, as OCCT does, through <see cref="ShapeTool"/>'s non-freezing attach
/// (decision 488); the polygons and triangulations freeze with the result when it is published. Faces that failed or
/// whose triangulation was reused (frozen, from the input) are skipped, so a reused triangulation is never written.
/// </para>
/// <para>
/// The edge loop is serial, as OCCT forces it ("data races on edges sharing the same TShape"). The face loop runs in
/// parallel when the parameters ask for it: each body updates only its face's triangulation, and shares one
/// immutable <see cref="TriangulationParameters"/>, as OCCT does.
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

        // TODO: Force single threaded solution due to data races on edges sharing the same TShape
        OsdParallel.For(0,
                         model.EdgesNb(),
                         new PolygonCommitter(model).Process,
                         true /*!theParameters.InParallel*/);

        // Estimate deflection here due to BRepLib::EstimateDeflection requires
        // existence of both Poly_Triangulation and Poly_PolygonOnTriangulation.
        OsdParallel.For(0,
                         model.FacesNb(),
                         new DeflectionEstimator(model, parameters).Process,
                         !parameters.InParallel);
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

        public DeflectionEstimator(ModelModel model, in ToolParameters theParams)
        {
            _model = model;
            _params = new TriangulationParameters(theParams.Deflection,
                                                        theParams.Angle,
                                                        theParams.MinSize);
        }

        // Main functor.
        public void Process(int faceIndex)
        {
            FaceModel dFace = _model.GetFace(faceIndex);
            if (dFace.IsSet(StatusModel.Failure) || dFace.IsSet(StatusModel.Reused))
            {
                return;
            }

            BrepLibrary.UpdateDeflection(dFace.GetFace());

            Triangulation? triangulation = Tool.Triangulation(dFace.GetFace(), out Placement _);

            if (triangulation is not null)
            {
                triangulation.Parameters(_params);
            }
        }
    }
}
