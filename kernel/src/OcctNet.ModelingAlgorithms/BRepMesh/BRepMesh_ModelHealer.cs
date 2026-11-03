// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelHealer.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelHealer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Occt.IMeshData_Status;

namespace Occt;

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
public class BRepMesh_ModelHealer : IMeshTools_ModelAlgo
{
    private IMeshData_Model? myModel;
    private IMeshTools_Parameters myParameters = new();
    private NCollection_DataMap<IMeshData_Face, NCollection_Map<IMeshData_Edge>?>? myFaceIntersectingEdges;

    /// <summary>A healer.</summary>
    public BRepMesh_ModelHealer()
    {
    }

    /// <summary>Heals the model; false without a model.</summary>
    protected override bool performInternal(IMeshData_Model? theModel,
                                            in IMeshTools_Parameters theParameters,
                                            ProgressRange theRange)
    {
        _ = theRange;
        myModel = theModel;
        myParameters = theParameters;
        if (myModel is null)
        {
            return false;
        }

        // MinSize is made as a constant. It is connected with
        // the fact that too rude discretisation can lead to
        // self-intersecting polygon, which cannot be fixed.
        // As result the face will not be triangulated at all.
        // E.g. see "Test mesh standard_mesh C7", the face #17.
        myParameters.MinSize = global::Kerf.Engine.Foundation.Precision.Confusion();

        myFaceIntersectingEdges =
            new NCollection_DataMap<IMeshData_Face, NCollection_Map<IMeshData_Edge>?>(ReferenceEqualityComparer.Instance);
        for (int aFaceIt = 0; aFaceIt < myModel.FacesNb(); ++aFaceIt)
        {
            myFaceIntersectingEdges.Bind(myModel.GetFace(aFaceIt), null);
        }

        // TODO: Here we can process edges in order to remove close discrete points.
        OsdParallel.For(0, myModel.FacesNb(), process, !isParallel());
        amplifyEdges();

        foreach (KeyValuePair<IMeshData_Face, NCollection_Map<IMeshData_Edge>?> aFaceIt in myFaceIntersectingEdges)
        {
            if (aFaceIt.Value is not null)
            {
                IMeshData_Face aDFace = aFaceIt.Key;
                aDFace.SetStatus(IMeshData_SelfIntersectingWire);
                aDFace.SetStatus(IMeshData_Failure);
            }
        }

        myFaceIntersectingEdges = null;
        myModel = null; // Do not hold link to model.
        return true;
    }

    // Amplifies discretization of edges in case if self-intersection problem has been found.
    private void amplifyEdges()
    {
        int aAmpIt = 0;
        const double aIterNb = 5;
        NCollection_Map<IMeshData_Edge> aEdgesToUpdate = new(ReferenceEqualityComparer.Instance);
        EdgeAmplifier anEdgeAmplifier = new(myParameters);

        while (aAmpIt++ < aIterNb && popEdgesToUpdate(aEdgesToUpdate))
        {
            // Try to update discretization by decreasing deflection of problematic edges.
            OsdParallel.ForEach(aEdgesToUpdate,
                                 anEdgeAmplifier.Process,
                                 !myParameters.InParallel || aEdgesToUpdate.Size() <= 1,
                                 aEdgesToUpdate.Size());

            NCollection_Map<IMeshData_Face> aFacesToCheck = new(ReferenceEqualityComparer.Instance);
            foreach (IMeshData_Edge aDEdge in aEdgesToUpdate)
            {
                for (int aPCurveIt = 0; aPCurveIt < aDEdge.PCurvesNb(); ++aPCurveIt)
                {
                    aFacesToCheck.Add(aDEdge.GetPCurve(aPCurveIt).GetFace());
                }
            }

            OsdParallel.ForEach(aFacesToCheck,
                                 process,
                                 !myParameters.InParallel || aFacesToCheck.Size() <= 1,
                                 aFacesToCheck.Size());

            aEdgesToUpdate.Clear();
        }
    }

    // Collects unique edges to be updated from face map. Clears data stored in face map.
    private bool popEdgesToUpdate(NCollection_Map<IMeshData_Edge> theEdgesToUpdate)
    {
        foreach (IMeshData_Face aDFace in myFaceIntersectingEdges!.Keys)
        {
            ref NCollection_Map<IMeshData_Edge>? aIntersections = ref myFaceIntersectingEdges.ChangeFind(aDFace);
            if (aIntersections is not null)
            {
                NCollection_MapAlgo.Unite(theEdgesToUpdate, aIntersections);
                aIntersections = null;
            }
        }

        return !theEdgesToUpdate.IsEmpty();
    }

    // Returns True if check can be done in parallel.
    private bool isParallel() => (myParameters.InParallel && myModel!.FacesNb() > 1);

    // Checks existing discretization of the face and updates data model.
    private void process(int theFaceIndex)
    {
        IMeshData_Face aDFace = myModel!.GetFace(theFaceIndex);
        process(aDFace);
    }

    // Checks existing discretization of the face and updates data model.
    private void process(IMeshData_Face theDFace)
    {
        try
        {
            ref NCollection_Map<IMeshData_Edge>? aIntersections = ref myFaceIntersectingEdges!.ChangeFind(theDFace);
            aIntersections = null;

            fixFaceBoundaries(theDFace);

            if (!theDFace.IsSet(IMeshData_Failure))
            {
                BRepMesh_FaceChecker aChecker = new(theDFace, myParameters);
                if (!aChecker.Perform())
                {
                    aIntersections = aChecker.GetIntersectingEdges();
                }
                else
                {
                    if (theDFace.WiresNb() == 1)
                    {
                        IMeshData_Wire aDWire = theDFace.GetWire(0);

                        if (aDWire.EdgesNb() == 2)
                        {
                            IMeshData_Edge aDEdge0 = aDWire.GetEdge(0);
                            IMeshData_Edge aDEdge1 = aDWire.GetEdge(1);

                            IMeshData_PCurve aPCurve0 = aDEdge0.GetPCurve(theDFace, aDWire.GetEdgeOrientation(0));
                            IMeshData_PCurve aPCurve1 = aDEdge1.GetPCurve(theDFace, aDWire.GetEdgeOrientation(1));

                            if (aPCurve0.ParametersNb() == 2 && aPCurve1.ParametersNb() == 2)
                            {
                                aIntersections = new NCollection_Map<IMeshData_Edge>(ReferenceEqualityComparer.Instance);
                                // a kind of degenerated face - 1 wire, 2 edges and both edges are very small
                                aIntersections.Add(aDEdge0);
                                aIntersections.Add(aDEdge1);
                            }
                        }
                    }
                }
            }
        }
        catch (Exception anException) when (IsCaughtFailure(anException))
        {
            theDFace.SetStatus(IMeshData_Failure);
        }
    }

    // Connects ends of pcurves of face's wires according to topological coherency.
    private void fixFaceBoundaries(IMeshData_Face theDFace)
    {
        for (int aWireIt = 0; aWireIt < theDFace.WiresNb(); ++aWireIt)
        {
            IMeshData_Wire aDWire = theDFace.GetWire(aWireIt);
            BRepMesh_Deflection.ComputeDeflection(aDWire, myParameters);
            for (int aEdgeIt = 0; aEdgeIt < aDWire.EdgesNb(); ++aEdgeIt)
            {
                int aPrevEdgeIt = (aEdgeIt + aDWire.EdgesNb() - 1) % aDWire.EdgesNb();
                int aNextEdgeIt = (aEdgeIt + 1) % aDWire.EdgesNb();

                IMeshData_Edge aPrevEdge = aDWire.GetEdge(aPrevEdgeIt);
                IMeshData_Edge aCurrEdge = aDWire.GetEdge(aEdgeIt);
                IMeshData_Edge aNextEdge = aDWire.GetEdge(aNextEdgeIt);

                bool isConnected = !getCommonVertex(aCurrEdge, aNextEdge).IsNull
                                   && !getCommonVertex(aPrevEdge, aCurrEdge).IsNull;

                if (isConnected)
                {
                    IMeshData_PCurve aPrevPCurve =
                        aPrevEdge.GetPCurve(theDFace, aDWire.GetEdgeOrientation(aPrevEdgeIt));

                    IMeshData_PCurve aCurrPCurve =
                        aCurrEdge.GetPCurve(theDFace, aDWire.GetEdgeOrientation(aEdgeIt));

                    IMeshData_PCurve aNextPCurve =
                        aNextEdge.GetPCurve(theDFace, aDWire.GetEdgeOrientation(aNextEdgeIt));

                    isConnected = connectClosestPoints(aPrevPCurve, aCurrPCurve, aNextPCurve);
                }

                if (!isConnected || aCurrEdge.IsSet(IMeshData_Outdated))
                {
                    // We have to clean face from triangulation.
                    theDFace.SetStatus(IMeshData_Outdated);

                    if (!isConnected)
                    {
                        // Just mark wire as open, but continue fixing other inconsistencies
                        // in hope that this data could be suitable to build mesh somehow.
                        aDWire.SetStatus(IMeshData_OpenWire);
                    }
                }
            }
        }

        BRepMesh_Deflection.ComputeDeflection(theDFace, myParameters);
    }

    // Returns common vertex of two edges or null ptr in case if there is no such vertex.
    private static Vertex getCommonVertex(IMeshData_Edge theEdge1, IMeshData_Edge theEdge2)
    {
        Exploration.Vertices(theEdge1.GetEdge(), out Vertex aVertex1_1, out Vertex aVertex1_2);

        // Test bugs moddata_2 bug428.
        //   restore [locate_data_file OCC428.brep] rr
        //   explode rr f
        //   explode rr_91 w
        //   explode rr_91_2 e
        //   nbshapes rr_91_2_2
        //   # 0 vertices; 1 edge

        // This shape is invalid and can lead to exception in this code.

        if (aVertex1_1.IsNull || aVertex1_2.IsNull)
        {
            return new Vertex();
        }

        if (theEdge1.GetEdge().IsSame(theEdge2.GetEdge()))
        {
            return aVertex1_1.IsSame(aVertex1_2) ? aVertex1_1 : new Vertex();
        }

        Exploration.Vertices(theEdge2.GetEdge(), out Vertex aVertex2_1, out Vertex aVertex2_2);

        if (aVertex2_1.IsNull || aVertex2_2.IsNull)
        {
            return new Vertex();
        }

        if (isSameWithSomeOf(aVertex1_1, aVertex2_1, aVertex2_2))
        {
            return aVertex1_1;
        }
        else if (isSameWithSomeOf(aVertex1_2, aVertex2_1, aVertex2_2))
        {
            return aVertex1_2;
        }

        Point3D aPnt1_1 = Tool.Pnt(aVertex1_1);
        Point3D aPnt1_2 = Tool.Pnt(aVertex1_2);
        double aTol1_1 = Tool.Tolerance(aVertex1_1);
        double aTol1_2 = Tool.Tolerance(aVertex1_2);

        Point3D aPnt2_1 = Tool.Pnt(aVertex2_1);
        Point3D aPnt2_2 = Tool.Pnt(aVertex2_2);
        double aTol2_1 = Tool.Tolerance(aVertex2_1);
        double aTol2_2 = Tool.Tolerance(aVertex2_2);

        if (isInToleranceWithSomeOf(aPnt1_1, aPnt2_1, aPnt2_2, aTol1_1 + Max(aTol2_1, aTol2_2)))
        {
            return aVertex1_1;
        }
        else if (isInToleranceWithSomeOf(aPnt1_2, aPnt2_1, aPnt2_2, aTol1_2 + Max(aTol2_1, aTol2_2)))
        {
            return aVertex1_2;
        }

        return new Vertex();
    }

    // Connects pcurves of previous and current edge on the specified face according to topological connectivity. Uses
    // next edge in order to identify closest point in case of single vertex shared between both ends of edge
    // (degenerative edge).
    private static bool connectClosestPoints(IMeshData_PCurve thePrevDEdge,
                                             IMeshData_PCurve theCurrDEdge,
                                             IMeshData_PCurve theNextDEdge)
    {
        if (thePrevDEdge.IsInternal() || theCurrDEdge.IsInternal() || theNextDEdge.IsInternal())
        {
            return true;
        }

        PntRef aPrevFirstUV = new(thePrevDEdge, 0);
        PntRef aPrevLastUV = new(thePrevDEdge, thePrevDEdge.ParametersNb() - 1);

        if (thePrevDEdge == theCurrDEdge)
        {
            // Wire consists of a single edge.
            aPrevFirstUV.Value() = aPrevLastUV.Value();
            return true;
        }

        PntRef aCurrFirstUV = new(theCurrDEdge, 0);
        PntRef aCurrLastUV = new(theCurrDEdge, theCurrDEdge.ParametersNb() - 1);

        double aPrevSqDist =
            closestPoints(aPrevFirstUV, aPrevLastUV, aCurrFirstUV, aCurrLastUV, out PntRef aPrevUV, out PntRef aCurrPrevUV);

        PntRef aNextUV, aCurrNextUV;
        if (thePrevDEdge == theNextDEdge)
        {
            // Wire consists of two edges. Connect both ends.
            aNextUV = (aPrevUV == aPrevFirstUV) ? aPrevLastUV : aPrevFirstUV;
            aCurrNextUV = (aCurrPrevUV == aCurrFirstUV) ? aCurrLastUV : aCurrFirstUV;

            aNextUV.Value() = aCurrNextUV.Value();
            aPrevUV.Value() = aCurrPrevUV.Value();
            return true;
        }

        PntRef aNextFirstUV = new(theNextDEdge, 0);
        PntRef aNextLastUV = new(theNextDEdge, theNextDEdge.ParametersNb() - 1);

        double aNextSqDist =
            closestPoints(aNextFirstUV, aNextLastUV, aCurrFirstUV, aCurrLastUV, out aNextUV, out aCurrNextUV);

        // Connect closest points first. This can help to identify
        // which ends should be connected in case of gap.
        if (aPrevSqDist - aNextSqDist > Constants.Resolution())
        {
            adjustSamePoints(ref aCurrNextUV,
                             ref aNextUV,
                             ref aCurrPrevUV,
                             ref aPrevUV,
                             aCurrFirstUV,
                             aCurrLastUV,
                             aPrevFirstUV,
                             aPrevLastUV);
        }
        else
        {
            adjustSamePoints(ref aCurrPrevUV,
                             ref aPrevUV,
                             ref aCurrNextUV,
                             ref aNextUV,
                             aCurrFirstUV,
                             aCurrLastUV,
                             aNextFirstUV,
                             aNextLastUV);
        }

        return true;
    }

    // Chooses the most closest point to reference one from the given pair. Returns square distance between reference
    // point and closest one as well as pointer to closest point.
    private static double closestPoint(PntRef theRefPnt, PntRef theFristPnt, PntRef theSecondPnt, out PntRef theClosestPnt)
    {
        // Find the most closest end-points.
        double aSqDist1 = theRefPnt.Value().SquareDistance(theFristPnt.Value());
        double aSqDist2 = theRefPnt.Value().SquareDistance(theSecondPnt.Value());
        if (aSqDist1 < aSqDist2)
        {
            theClosestPnt = theFristPnt;
            return aSqDist1;
        }

        theClosestPnt = theSecondPnt;
        return aSqDist2;
    }

    // Chooses the most closest points among the given to reference one from the given pair. Returns square distance
    // between reference point and closest one as well as pointer to closest point.
    private static double closestPoints(PntRef theFirstPnt1,
                                        PntRef theSecondPnt1,
                                        PntRef theFirstPnt2,
                                        PntRef theSecondPnt2,
                                        out PntRef theClosestPnt1,
                                        out PntRef theClosestPnt2)
    {
        double aSqDist1 = closestPoint(theFirstPnt1, theFirstPnt2, theSecondPnt2, out PntRef aCurrPrevUV1);
        double aSqDist2 = closestPoint(theSecondPnt1, theFirstPnt2, theSecondPnt2, out PntRef aCurrPrevUV2);
        if (aSqDist1 - aSqDist2 < Constants.Resolution())
        {
            theClosestPnt1 = theFirstPnt1;
            theClosestPnt2 = aCurrPrevUV1;
            return aSqDist1;
        }

        theClosestPnt1 = theSecondPnt1;
        theClosestPnt2 = aCurrPrevUV2;
        return aSqDist2;
    }

    // Adjusts the given pair of points supposed to be the same. In addition, adjusts another end-point of an edge in
    // order to perform correct matching in case of gap.
    private static void adjustSamePoints(ref PntRef theMajorSamePnt1,
                                         ref PntRef theMinorSamePnt1,
                                         ref PntRef theMajorSamePnt2,
                                         ref PntRef theMinorSamePnt2,
                                         PntRef theMajorFirstPnt,
                                         PntRef theMajorLastPnt,
                                         PntRef theMinorFirstPnt,
                                         PntRef theMinorLastPnt)
    {
        if (theMajorSamePnt2 == theMajorSamePnt1)
        {
            theMajorSamePnt2 = (theMajorSamePnt2 == theMajorFirstPnt) ? theMajorLastPnt : theMajorFirstPnt;
            closestPoint(theMajorSamePnt2, theMinorFirstPnt, theMinorLastPnt, out theMinorSamePnt2);
        }

        theMajorSamePnt1.Value() = theMinorSamePnt1.Value();
        theMajorSamePnt2.Value() = theMinorSamePnt2.Value();
    }

    // Returns True if some of two vertcies is same with reference one.
    private static bool isSameWithSomeOf(in Vertex theRefVertex,
                                         in Vertex theVertex1,
                                         in Vertex theVertex2)
    {
        return (theRefVertex.IsSame(theVertex1) || theRefVertex.IsSame(theVertex2));
    }

    // Returns True if some of two vertcies is within tolerance of reference one.
    private static bool isInToleranceWithSomeOf(in Point3D theRefPoint,
                                                in Point3D thePoint1,
                                                in Point3D thePoint2,
                                                double theTol)
    {
        double aSqTol = theTol * theTol;
        return (theRefPoint.SquareDistance(thePoint1) < aSqTol
                || theRefPoint.SquareDistance(thePoint2) < aSqTol);
    }

    // OCCT's `gp_Pnt2d*` into a discrete pcurve: the pcurve and the point's index. Two are equal when they designate the
    // same point, as OCCT's pointers are (the first and last points of a one-point pcurve are the same).
    private readonly struct PntRef : IEquatable<PntRef>
    {
        private readonly IMeshData_PCurve myPCurve;
        private readonly int myIndex;

        public PntRef(IMeshData_PCurve thePCurve, int theIndex)
        {
            myPCurve = thePCurve;
            myIndex = theIndex;
        }

        // The point, for reading and writing (*ptr).
        public ref Point2D Value() => ref myPCurve.GetPoint(myIndex);

        public bool Equals(PntRef theOther) => ReferenceEquals(myPCurve, theOther.myPCurve) && myIndex == theOther.myIndex;

        public override bool Equals(object? theObj) => theObj is PntRef aRef && Equals(aRef);

        public override int GetHashCode() => HashCode.Combine(RuntimeHelpers.GetHashCode(myPCurve), myIndex);

        public static bool operator ==(PntRef theLeft, PntRef theRight) => theLeft.Equals(theRight);

        public static bool operator !=(PntRef theLeft, PntRef theRight) => !theLeft.Equals(theRight);
    }

    // Decreases deflection of the given edge and tries to update discretization.
    private sealed class EdgeAmplifier
    {
        private readonly IMeshTools_Parameters myParameters;

        public EdgeAmplifier(in IMeshTools_Parameters theParameters)
        {
            myParameters = theParameters;
        }

        // Main operator.
        public void Process(IMeshData_Edge theDEdge)
        {
            IMeshData_Edge aDEdge = theDEdge;

            int aPointsNb = aDEdge.GetCurve().ParametersNb();

            aDEdge.Clear(true);
            aDEdge.SetDeflection(Max(aDEdge.GetDeflection() / 3.0, global::Kerf.Engine.Foundation.Precision.Confusion()));

            for (int aPCurveIt = 0; aPCurveIt < aDEdge.PCurvesNb(); ++aPCurveIt)
            {
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(aPCurveIt);
                IMeshData_Face aDFace = aPCurve.GetFace();

                // Check that outer wire contains 2 edges or less and add an additional point.
                IMeshData_Wire aDWire = aDFace.GetWire(0);
                if (aDWire.EdgesNb() <= 2)
                {
                    ++aPointsNb;
                    break;
                }
            }

            {
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(0);
                IMeshData_Face aDFace = aPCurve.GetFace();
                IMeshTools_CurveTessellator aTessellator =
                    BRepMesh_EdgeDiscret.CreateEdgeTessellator(aDEdge,
                                                               aPCurve.GetOrientation(),
                                                               aDFace,
                                                               myParameters,
                                                               aPointsNb);

                BRepMesh_EdgeDiscret.Tessellate3d(aDEdge, aTessellator, false);
                BRepMesh_EdgeDiscret.Tessellate2d(aDEdge, false);
            }
        }
    }
}
