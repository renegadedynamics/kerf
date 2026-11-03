// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelHealer.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelHealer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Heals the discrete model: checks that each face's wires are closed and free of self-intersections. Ends of adjacent
/// edges forming gaps are connected in the face's parametric space; edges found in self-intersections are
/// re-discretized with a smaller deflection (up to five times), and faces still intersecting fail.
/// </summary>
/// <remarks>
/// <para>
/// The faces, then the edges to amplify, then the faces to check again are processed in parallel when the parameters
/// ask for it (decision 490's audit):
/// </para>
/// <list type="bullet">
/// <item>A face's body writes only the points of the pcurves on that face (each discrete pcurve belongs to one face),
/// the face's and its wires' statuses and deflections, and the face's own slot of the intersections map; the map's
/// keys are all bound before the loop, so the body only looks its slot up, which is a read of the map.</item>
/// <item>An edge's body writes only its discrete edge (3D curve and all its pcurves) and reads the outer wire of each
/// of its faces; its tessellator evaluates its own adaptors.</item>
/// </list>
/// <para>
/// The edges and faces gathered in maps compared by reference iterate in insertion order, where OCCT's iterate in
/// address order; each body is independent of the others, so the order doesn't change the result.
/// </para>
/// </remarks>
public class ModelHealer : ModelAlgo
{
    private ModelModel? _model;
    private ToolParameters _parameters = new();
    private Dictionary<FaceModel, HashSet<EdgeModel>?>? _faceIntersectingEdges;

    /// <summary>A healer.</summary>
    public ModelHealer()
    {
    }

    /// <summary>Heals the model; false without a model.</summary>
    protected override bool PerformInternal(ModelModel? model,
                                            in ToolParameters parameters,
                                            ProgressRange range)
    {
        _ = range;
        _model = model;
        _parameters = parameters;
        if (_model is null)
        {
            return false;
        }

        // MinSize is made as a constant. It is connected with
        // the fact that too rude discretisation can lead to
        // self-intersecting polygon, which cannot be fixed.
        // As result the face will not be triangulated at all.
        // E.g. see "Test mesh standard_mesh C7", the face #17.
        _parameters.MinSize = global::Kerf.Engine.Foundation.Precision.Confusion();

        _faceIntersectingEdges =
            new Dictionary<FaceModel, HashSet<EdgeModel>?>(ReferenceEqualityComparer.Instance);
        for (int faceIt = 0; faceIt < _model.FacesNb(); ++faceIt)
        {
            _faceIntersectingEdges[_model.GetFace(faceIt)] = null;
        }

        // TODO: Here we can process edges in order to remove close discrete points.
        OsdParallel.For(0, _model.FacesNb(), Process, !IsParallel());
        AmplifyEdges();

        foreach (KeyValuePair<FaceModel, HashSet<EdgeModel>?> faceIt in _faceIntersectingEdges)
        {
            if (faceIt.Value is not null)
            {
                FaceModel dFace = faceIt.Key;
                dFace.SetStatus(StatusModel.SelfIntersectingWire);
                dFace.SetStatus(StatusModel.Failure);
            }
        }

        _faceIntersectingEdges = null;
        _model = null; // Do not hold link to model.
        return true;
    }

    // Amplifies discretization of edges in case if self-intersection problem has been found.
    private void AmplifyEdges()
    {
        int ampIt = 0;
        const double iterNb = 5;
        HashSet<EdgeModel> edgesToUpdate = new(ReferenceEqualityComparer.Instance);
        EdgeAmplifier edgeAmplifier = new(_parameters);

        while (ampIt++ < iterNb && PopEdgesToUpdate(edgesToUpdate))
        {
            // Try to update discretization by decreasing deflection of problematic edges.
            OsdParallel.ForEach(edgesToUpdate,
                                 edgeAmplifier.Process,
                                 !_parameters.InParallel || edgesToUpdate.Count <= 1,
                                 edgesToUpdate.Count);

            HashSet<FaceModel> facesToCheck = new(ReferenceEqualityComparer.Instance);
            foreach (EdgeModel dEdge in edgesToUpdate)
            {
                for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
                {
                    facesToCheck.Add(dEdge.GetPCurve(pCurveIt).GetFace());
                }
            }

            OsdParallel.ForEach(facesToCheck,
                                 Process,
                                 !_parameters.InParallel || facesToCheck.Count <= 1,
                                 facesToCheck.Count);

            edgesToUpdate.Clear();
        }
    }

    // Collects unique edges to be updated from face map. Clears data stored in face map.
    private bool PopEdgesToUpdate(HashSet<EdgeModel> edgesToUpdate)
    {
        foreach (FaceModel dFace in _faceIntersectingEdges!.Keys)
        {
            ref HashSet<EdgeModel>? intersections = ref _faceIntersectingEdges.GetValueRef(dFace);
            if (intersections is not null)
            {
                NCollection_MapAlgo.Unite(edgesToUpdate, intersections);
                intersections = null;
            }
        }

        return edgesToUpdate.Count != 0;
    }

    // Returns True if check can be done in parallel.
    private bool IsParallel() => (_parameters.InParallel && _model!.FacesNb() > 1);

    // Checks existing discretization of the face and updates data model.
    private void Process(int faceIndex)
    {
        FaceModel dFace = _model!.GetFace(faceIndex);
        Process(dFace);
    }

    // Checks existing discretization of the face and updates data model.
    private void Process(FaceModel dFace)
    {
        try
        {
            ref HashSet<EdgeModel>? intersections = ref _faceIntersectingEdges!.GetValueRef(dFace);
            intersections = null;

            FixFaceBoundaries(dFace);

            if (!dFace.IsSet(StatusModel.Failure))
            {
                FaceChecker checker = new(dFace, _parameters);
                if (!checker.Perform())
                {
                    intersections = checker.GetIntersectingEdges();
                }
                else
                {
                    if (dFace.WiresNb() == 1)
                    {
                        WireModel dWire = dFace.GetWire(0);

                        if (dWire.EdgesNb() == 2)
                        {
                            EdgeModel dEdge0 = dWire.GetEdge(0);
                            EdgeModel dEdge1 = dWire.GetEdge(1);

                            PCurveModel pCurve0 = dEdge0.GetPCurve(dFace, dWire.GetEdgeOrientation(0));
                            PCurveModel pCurve1 = dEdge1.GetPCurve(dFace, dWire.GetEdgeOrientation(1));

                            if (pCurve0.ParametersNb() == 2 && pCurve1.ParametersNb() == 2)
                            {
                                intersections = new HashSet<EdgeModel>(ReferenceEqualityComparer.Instance);
                                // a kind of degenerated face - 1 wire, 2 edges and both edges are very small
                                intersections.Add(dEdge0);
                                intersections.Add(dEdge1);
                            }
                        }
                    }
                }
            }
        }
        catch (Exception exception) when (IsCaughtFailure(exception))
        {
            dFace.SetStatus(StatusModel.Failure);
        }
    }

    // Connects ends of pcurves of face's wires according to topological coherency.
    private void FixFaceBoundaries(FaceModel dFace)
    {
        for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
        {
            WireModel dWire = dFace.GetWire(wireIt);
            Deflection.ComputeDeflection(dWire, _parameters);
            for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
            {
                int prevEdgeIt = (edgeIt + dWire.EdgesNb() - 1) % dWire.EdgesNb();
                int nextEdgeIt = (edgeIt + 1) % dWire.EdgesNb();

                EdgeModel prevEdge = dWire.GetEdge(prevEdgeIt);
                EdgeModel currEdge = dWire.GetEdge(edgeIt);
                EdgeModel nextEdge = dWire.GetEdge(nextEdgeIt);

                bool isConnected = !GetCommonVertex(currEdge, nextEdge).IsNull
                                   && !GetCommonVertex(prevEdge, currEdge).IsNull;

                if (isConnected)
                {
                    PCurveModel prevPCurve =
                        prevEdge.GetPCurve(dFace, dWire.GetEdgeOrientation(prevEdgeIt));

                    PCurveModel currPCurve =
                        currEdge.GetPCurve(dFace, dWire.GetEdgeOrientation(edgeIt));

                    PCurveModel nextPCurve =
                        nextEdge.GetPCurve(dFace, dWire.GetEdgeOrientation(nextEdgeIt));

                    isConnected = ConnectClosestPoints(prevPCurve, currPCurve, nextPCurve);
                }

                if (!isConnected || currEdge.IsSet(StatusModel.Outdated))
                {
                    // We have to clean face from triangulation.
                    dFace.SetStatus(StatusModel.Outdated);

                    if (!isConnected)
                    {
                        // Just mark wire as open, but continue fixing other inconsistencies
                        // in hope that this data could be suitable to build mesh somehow.
                        dWire.SetStatus(StatusModel.OpenWire);
                    }
                }
            }
        }

        Deflection.ComputeDeflection(dFace, _parameters);
    }

    // Returns common vertex of two edges or null ptr in case if there is no such vertex.
    private static Vertex GetCommonVertex(EdgeModel edge1, EdgeModel edge2)
    {
        Exploration.Vertices(edge1.GetEdge(), out Vertex vertex1_1, out Vertex vertex1_2);

        // Test bugs moddata_2 bug428.
        //   restore [locate_data_file OCC428.brep] rr
        //   explode rr f
        //   explode rr_91 w
        //   explode rr_91_2 e
        //   nbshapes rr_91_2_2
        //   # 0 vertices; 1 edge

        // This shape is invalid and can lead to exception in this code.

        if (vertex1_1.IsNull || vertex1_2.IsNull)
        {
            return new Vertex();
        }

        if (edge1.GetEdge().IsSame(edge2.GetEdge()))
        {
            return vertex1_1.IsSame(vertex1_2) ? vertex1_1 : new Vertex();
        }

        Exploration.Vertices(edge2.GetEdge(), out Vertex vertex2_1, out Vertex vertex2_2);

        if (vertex2_1.IsNull || vertex2_2.IsNull)
        {
            return new Vertex();
        }

        if (IsSameWithSomeOf(vertex1_1, vertex2_1, vertex2_2))
        {
            return vertex1_1;
        }
        else if (IsSameWithSomeOf(vertex1_2, vertex2_1, vertex2_2))
        {
            return vertex1_2;
        }

        Point3D pnt1_1 = Tool.Pnt(vertex1_1);
        Point3D pnt1_2 = Tool.Pnt(vertex1_2);
        double tol1_1 = Tool.Tolerance(vertex1_1);
        double tol1_2 = Tool.Tolerance(vertex1_2);

        Point3D pnt2_1 = Tool.Pnt(vertex2_1);
        Point3D pnt2_2 = Tool.Pnt(vertex2_2);
        double tol2_1 = Tool.Tolerance(vertex2_1);
        double tol2_2 = Tool.Tolerance(vertex2_2);

        if (IsInToleranceWithSomeOf(pnt1_1, pnt2_1, pnt2_2, tol1_1 + Max(tol2_1, tol2_2)))
        {
            return vertex1_1;
        }
        else if (IsInToleranceWithSomeOf(pnt1_2, pnt2_1, pnt2_2, tol1_2 + Max(tol2_1, tol2_2)))
        {
            return vertex1_2;
        }

        return new Vertex();
    }

    // Connects pcurves of previous and current edge on the specified face according to topological connectivity. Uses
    // next edge in order to identify closest point in case of single vertex shared between both ends of edge
    // (degenerative edge).
    private static bool ConnectClosestPoints(PCurveModel prevDEdge,
                                             PCurveModel currDEdge,
                                             PCurveModel nextDEdge)
    {
        if (prevDEdge.IsInternal || currDEdge.IsInternal || nextDEdge.IsInternal)
        {
            return true;
        }

        PntRef prevFirstUV = new(prevDEdge, 0);
        PntRef prevLastUV = new(prevDEdge, prevDEdge.ParametersNb() - 1);

        if (prevDEdge == currDEdge)
        {
            // Wire consists of a single edge.
            prevFirstUV.Value() = prevLastUV.Value();
            return true;
        }

        PntRef currFirstUV = new(currDEdge, 0);
        PntRef currLastUV = new(currDEdge, currDEdge.ParametersNb() - 1);

        double prevSqDist =
            ClosestPoints(prevFirstUV, prevLastUV, currFirstUV, currLastUV, out PntRef prevUV, out PntRef currPrevUV);

        PntRef nextUV, currNextUV;
        if (prevDEdge == nextDEdge)
        {
            // Wire consists of two edges. Connect both ends.
            nextUV = (prevUV == prevFirstUV) ? prevLastUV : prevFirstUV;
            currNextUV = (currPrevUV == currFirstUV) ? currLastUV : currFirstUV;

            nextUV.Value() = currNextUV.Value();
            prevUV.Value() = currPrevUV.Value();
            return true;
        }

        PntRef nextFirstUV = new(nextDEdge, 0);
        PntRef nextLastUV = new(nextDEdge, nextDEdge.ParametersNb() - 1);

        double nextSqDist =
            ClosestPoints(nextFirstUV, nextLastUV, currFirstUV, currLastUV, out nextUV, out currNextUV);

        // Connect closest points first. This can help to identify
        // which ends should be connected in case of gap.
        if (prevSqDist - nextSqDist > Constants.Resolution())
        {
            AdjustSamePoints(ref currNextUV,
                             ref nextUV,
                             ref currPrevUV,
                             ref prevUV,
                             currFirstUV,
                             currLastUV,
                             prevFirstUV,
                             prevLastUV);
        }
        else
        {
            AdjustSamePoints(ref currPrevUV,
                             ref prevUV,
                             ref currNextUV,
                             ref nextUV,
                             currFirstUV,
                             currLastUV,
                             nextFirstUV,
                             nextLastUV);
        }

        return true;
    }

    // Chooses the most closest point to reference one from the given pair. Returns square distance between reference
    // point and closest one as well as pointer to closest point.
    private static double ClosestPoint(PntRef refPnt, PntRef fristPnt, PntRef secondPnt, out PntRef closestPnt)
    {
        // Find the most closest end-points.
        double sqDist1 = refPnt.Value().SquareDistanceTo(fristPnt.Value());
        double sqDist2 = refPnt.Value().SquareDistanceTo(secondPnt.Value());
        if (sqDist1 < sqDist2)
        {
            closestPnt = fristPnt;
            return sqDist1;
        }

        closestPnt = secondPnt;
        return sqDist2;
    }

    // Chooses the most closest points among the given to reference one from the given pair. Returns square distance
    // between reference point and closest one as well as pointer to closest point.
    private static double ClosestPoints(PntRef firstPnt1,
                                        PntRef secondPnt1,
                                        PntRef firstPnt2,
                                        PntRef secondPnt2,
                                        out PntRef closestPnt1,
                                        out PntRef closestPnt2)
    {
        double sqDist1 = ClosestPoint(firstPnt1, firstPnt2, secondPnt2, out PntRef currPrevUV1);
        double sqDist2 = ClosestPoint(secondPnt1, firstPnt2, secondPnt2, out PntRef currPrevUV2);
        if (sqDist1 - sqDist2 < Constants.Resolution())
        {
            closestPnt1 = firstPnt1;
            closestPnt2 = currPrevUV1;
            return sqDist1;
        }

        closestPnt1 = secondPnt1;
        closestPnt2 = currPrevUV2;
        return sqDist2;
    }

    // Adjusts the given pair of points supposed to be the same. In addition, adjusts another end-point of an edge in
    // order to perform correct matching in case of gap.
    private static void AdjustSamePoints(ref PntRef majorSamePnt1,
                                         ref PntRef minorSamePnt1,
                                         ref PntRef majorSamePnt2,
                                         ref PntRef minorSamePnt2,
                                         PntRef majorFirstPnt,
                                         PntRef majorLastPnt,
                                         PntRef minorFirstPnt,
                                         PntRef minorLastPnt)
    {
        if (majorSamePnt2 == majorSamePnt1)
        {
            majorSamePnt2 = (majorSamePnt2 == majorFirstPnt) ? majorLastPnt : majorFirstPnt;
            ClosestPoint(majorSamePnt2, minorFirstPnt, minorLastPnt, out minorSamePnt2);
        }

        majorSamePnt1.Value() = minorSamePnt1.Value();
        majorSamePnt2.Value() = minorSamePnt2.Value();
    }

    // Returns True if some of two vertcies is same with reference one.
    private static bool IsSameWithSomeOf(in Vertex refVertex,
                                         in Vertex vertex1,
                                         in Vertex vertex2)
    {
        return (refVertex.IsSame(vertex1) || refVertex.IsSame(vertex2));
    }

    // Returns True if some of two vertcies is within tolerance of reference one.
    private static bool IsInToleranceWithSomeOf(in Point3D refPoint,
                                                in Point3D point1,
                                                in Point3D point2,
                                                double tol)
    {
        double sqTol = tol * tol;
        return (refPoint.SquareDistanceTo(point1) < sqTol
                || refPoint.SquareDistanceTo(point2) < sqTol);
    }

    // OCCT's `gp_Pnt2d*` into a discrete pcurve: the pcurve and the point's index. Two are equal when they designate the
    // same point, as OCCT's pointers are (the first and last points of a one-point pcurve are the same).
    private readonly struct PntRef : IEquatable<PntRef>
    {
        private readonly PCurveModel _pCurve;
        private readonly int _index;

        public PntRef(PCurveModel pCurve, int index)
        {
            _pCurve = pCurve;
            _index = index;
        }

        // The point, for reading and writing (*ptr).
        public ref Point2D Value() => ref _pCurve.GetPoint(_index);

        public bool Equals(PntRef other) => ReferenceEquals(_pCurve, other._pCurve) && _index == other._index;

        public override bool Equals(object? obj) => obj is PntRef aRef && Equals(aRef);

        public override int GetHashCode() => HashCode.Combine(RuntimeHelpers.GetHashCode(_pCurve), _index);

        public static bool operator ==(PntRef left, PntRef right) => left.Equals(right);

        public static bool operator !=(PntRef left, PntRef right) => !left.Equals(right);
    }

    // Decreases deflection of the given edge and tries to update discretization.
    private sealed class EdgeAmplifier
    {
        private readonly ToolParameters _parameters;

        public EdgeAmplifier(in ToolParameters parameters)
        {
            _parameters = parameters;
        }

        // Main operator.
        public void Process(EdgeModel dEdge)
        {
            EdgeModel aDEdge = dEdge;

            int pointsNb = aDEdge.GetCurve().ParametersNb();

            aDEdge.Clear(true);
            aDEdge.SetDeflection(Max(aDEdge.GetDeflection() / 3.0, global::Kerf.Engine.Foundation.Precision.Confusion()));

            for (int pCurveIt = 0; pCurveIt < aDEdge.PCurvesNb(); ++pCurveIt)
            {
                PCurveModel pCurve = aDEdge.GetPCurve(pCurveIt);
                FaceModel dFace = pCurve.GetFace();

                // Check that outer wire contains 2 edges or less and add an additional point.
                WireModel dWire = dFace.GetWire(0);
                if (dWire.EdgesNb() <= 2)
                {
                    ++pointsNb;
                    break;
                }
            }

            {
                PCurveModel pCurve = aDEdge.GetPCurve(0);
                FaceModel dFace = pCurve.GetFace();
                CurveTessellator tessellator =
                    EdgeDiscret.CreateEdgeTessellator(aDEdge,
                                                               pCurve.GetOrientation(),
                                                               dFace,
                                                               _parameters,
                                                               pointsNb);

                EdgeDiscret.Tessellate3d(aDEdge, tessellator, false);
                EdgeDiscret.Tessellate2d(aDEdge, false);
            }
        }
    }
}
