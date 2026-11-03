// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityDistTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityDistTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Kerf.Engine.Meshing;
using BVH_PrjState = Kerf.Engine.Geometry.Bvh.Tools<double, Occt.NCollection_Vec3<double>>.BVH_PrjStateInTriangle;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The proximity distance from a first primitive set to a second: the largest of the smallest perpendicular
/// distances from the first set's vertices to the second set (or the smallest distance when there is no
/// perpendicular one). Results are approximate and depend on the tessellations.
/// </summary>
/// <remarks>A working object (decision 21), used by <see cref="BRepExtrema_ProximityValueTool"/>.</remarks>
public class BRepExtrema_ProximityDistTool : BvhDistance<double, BVH_Vec3d, BVH_Vec3d, BRepExtrema_TriangleSet>
{
    /// <summary>The status of a proximity point: on the border or inside its shape.</summary>
    public enum ProxPnt_Status
    {
        ProxPnt_Status_BORDER,
        ProxPnt_Status_MIDDLE,
        ProxPnt_Status_UNKNOWN,
    }

    /// <summary>Where the projection giving a proximity point of the second shape lies on its triangle.</summary>
    public readonly struct PrjState
    {
        private readonly int myTrgIdx; //!< Index of triangle on which the projection is located
        private readonly BVH_PrjState myPrjState; //!< Position of a projection on the triangle (vertex, edge, inner)
        private readonly int myNumberOfFirstNode; //!< The 1st vtx of the triangle edge on which the projection is located
        private readonly int myNumberOfLastNode; //!< The 2nd vtx of the triangle edge on which the projection is located

        public PrjState()
        {
            myTrgIdx = 0;
            myPrjState = BVH_PrjState.PrjStateInTriangle_INNER;
            myNumberOfFirstNode = 0;
            myNumberOfLastNode = 0;
        }

        public PrjState(int theTrgIdx, BVH_PrjState thePrjState, int theNumberOfFirstNode, int theNumberOfLastNode)
        {
            myTrgIdx = theTrgIdx;
            myPrjState = thePrjState;
            myNumberOfFirstNode = theNumberOfFirstNode;
            myNumberOfLastNode = theNumberOfLastNode;
        }

        public int GetTrgIdx() => myTrgIdx;

        public BVH_PrjState GetPrjState() => myPrjState;

        public int GetNumberOfFirstNode() => myNumberOfFirstNode;

        public int GetNumberOfLastNode() => myNumberOfLastNode;
    }

    protected double myMinDistance; //!< Minimal distance from point to BVH, could be not equal to myDistance
    protected BVH_Vec3d myMinDistPoint; //!< Point on BVH providing the minimal distance
    protected BVH_Vec3d myExtremaPoint; //!< Point on BVH providing the extrema
    protected double myProxDist; //!< Proximity distance
    protected BVH_Vec3d myPnt1, myPnt2; //!< Proximity points
    protected ProxPnt_Status myPntStatus1, myPntStatus2; //!< Proximity points' status

    private BRepExtrema_TriangleSet? mySet1;
    private BRepExtrema_TriangleSet? mySet2;
    private NCollection_DynamicArray<TopoDS_Shape> myShapeList1 = new();
    private NCollection_DynamicArray<TopoDS_Shape> myShapeList2 = new();
    private readonly int myNbSamples1; //!< Number of samples points on the first shape
    private bool myIsProxVtx1FromAddSet; //!< Is the vertex of the proximity point of the 1st shape from the additional set
    private NCollection_LinearVector<BVH_Vec3d> myAddVertices1 = new(); //!< Additional vertices on the 1st shape
    private NCollection_DynamicArray<ProxPnt_Status> myAddStatus1 = new(); //!< Status of additional vertices on the 1st shape
    private int myProxVtxIdx1; //!< Vertex index from 1st BVH corresponding to proximity point of 1st shape
    private PrjState myProxPrjState = new(); //!< Projection state from the 2nd BVH giving the proximity point of the 2nd shape
    private PrjState myExtPrjState = new(); //!< Projection state from the 2nd BVH giving the extrema
    private PrjState myMinPrjState = new(); //!< Projection state from the 2nd BVH giving the minimal distance

    /// <summary>An uninitialized tool.</summary>
    public BRepExtrema_ProximityDistTool()
    {
        myMinDistance = double.MaxValue;
        myProxDist = -1.0;
        myPntStatus1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        myNbSamples1 = 0;
        myProxVtxIdx1 = -1;
    }

    /// <summary>A tool for the given element sets.</summary>
    public BRepExtrema_ProximityDistTool(BRepExtrema_TriangleSet theSet1,
                                         int theNbSamples1,
                                         NCollection_LinearVector<BVH_Vec3d> theAddVertices1,
                                         NCollection_DynamicArray<ProxPnt_Status> theAddStatus1,
                                         BRepExtrema_TriangleSet theSet2,
                                         NCollection_DynamicArray<TopoDS_Shape> theShapeList1,
                                         NCollection_DynamicArray<TopoDS_Shape> theShapeList2)
    {
        myMinDistance = double.MaxValue;
        myProxDist = -1.0;
        myPntStatus1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        myNbSamples1 = theNbSamples1;
        myProxVtxIdx1 = -1;
        LoadTriangleSets(theSet1, theSet2);
        LoadShapeLists(theShapeList1, theShapeList2);
        LoadAdditionalPointsFirstSet(theAddVertices1, theAddStatus1);
    }

    /// <summary>Loads the element sets.</summary>
    public void LoadTriangleSets(BRepExtrema_TriangleSet theSet1, BRepExtrema_TriangleSet theSet2)
    {
        mySet1 = theSet1;
        mySet2 = theSet2;
    }

    /// <summary>Loads the lists of sub-shapes (copies, as OCCT assigns by value).</summary>
    public void LoadShapeLists(NCollection_DynamicArray<TopoDS_Shape> theShapeList1, NCollection_DynamicArray<TopoDS_Shape> theShapeList2)
    {
        myShapeList1 = new NCollection_DynamicArray<TopoDS_Shape>(theShapeList1);
        myShapeList2 = new NCollection_DynamicArray<TopoDS_Shape>(theShapeList2);
    }

    /// <summary>Loads the additional vertices of the first set and their statuses (copies).</summary>
    public void LoadAdditionalPointsFirstSet(NCollection_LinearVector<BVH_Vec3d> theAddVertices1,
                                             NCollection_DynamicArray<ProxPnt_Status> theAddStatus1)
    {
        myAddVertices1 = new NCollection_LinearVector<BVH_Vec3d>(theAddVertices1);
        myAddStatus1 = new NCollection_DynamicArray<ProxPnt_Status>(theAddStatus1);
    }

    //=======================================================================
    // function : goThroughtSet1
    // purpose  : Goes through vertices from the 1st set
    //=======================================================================
    private void goThroughtSet1(NCollection_LinearVector<BVH_Vec3d> theVertices1, bool theIsAdditionalSet)
    {
        int aVtxSize = theVertices1.Size();
        int aVtxStep = Max(myNbSamples1 <= 0 ? 1 : aVtxSize / myNbSamples1, 1);
        for (int aVtxIdx = 0; aVtxIdx < aVtxSize; aVtxIdx += aVtxStep)
        {
            _distance = double.MaxValue;
            myMinDistance = double.MaxValue;
            _isDone = false;
            SetObject(theVertices1[aVtxIdx]);

            ComputeDistance();

            if (!IsDone && myProxDist < 0.0)
            {
                return;
            }

            if (IsDone && _distance > myProxDist)
            {
                myPnt1 = theVertices1[aVtxIdx];
                myPnt2 = myExtremaPoint;
                myProxDist = _distance;
                myProxVtxIdx1 = aVtxIdx;
                myIsProxVtx1FromAddSet = theIsAdditionalSet;
                myProxPrjState = myExtPrjState;
            }
        }
    }

    /// <summary>Searches for the proximity distance.</summary>
    public void Perform()
    {
        SetBVHSet(mySet2);

        goThroughtSet1(mySet1!.GetVertices(), false);
        goThroughtSet1(myAddVertices1, true);

        _isDone = myProxDist > -1.0;

        if (_isDone)
        {
            DefineStatusProxPnt();
        }
    }

    private static double pointBoxSquareMaxDistance(in BVH_Vec3d thePoint, in BVH_Vec3d theCMin, in BVH_Vec3d theCMax)
    {
        double aDist = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (thePoint[i] <= 0.5 * (theCMax[i] + theCMin[i]))
            {
                double d = theCMax[i] - thePoint[i];
                d *= d;
                aDist += d;
            }
            else
            {
                double d = thePoint[i] - theCMin[i];
                d *= d;
                aDist += d;
            }
        }
        return aDist;
    }

    /// <summary>Rejects a node by its bounding box.</summary>
    public override bool RejectNode(in BVH_Vec3d theCornerMin, in BVH_Vec3d theCornerMax, out double theMetric)
    {
        theMetric = Math.Sqrt(Tools<double, BVH_Vec3d>.PointBoxSquareDistance(_object, theCornerMin, theCornerMax));

        double aMaxMetric = Math.Sqrt(pointBoxSquareMaxDistance(_object, theCornerMin, theCornerMax));

        return theMetric > _distance || aMaxMetric < myProxDist;
    }

    /// <summary>Accepts a leaf element (a triangle of the second set).</summary>
    public override bool Accept(int theTrgIdx, in double theMetric)
    {
        BVH_PrjState aBVH_PrjState = default;
        int aNumberOfFirstNode = -1;
        int aNumberOfLastNode = -1;

        mySet2!.GetVertices(theTrgIdx, out BVH_Vec3d aTrgVert1, out BVH_Vec3d aTrgVert2, out BVH_Vec3d aTrgVert3);

        BVH_Vec3d aNearestPnt = Tools<double, BVH_Vec3d>.PointTriangleProjection(_object,
                                                                                    aTrgVert1,
                                                                                    aTrgVert2,
                                                                                    aTrgVert3,
                                                                                    ref aBVH_PrjState,
                                                                                    ref aNumberOfFirstNode,
                                                                                    ref aNumberOfLastNode);

        var aPrjState = new PrjState(theTrgIdx, aBVH_PrjState, aNumberOfFirstNode, aNumberOfLastNode);
        BVH_Vec3d aDirect = _object - aNearestPnt;
        double aSqDistance = aDirect.Dot(aDirect);

        if (aSqDistance > global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // point belongs to triangle
        {
            BVH_Vec3d aAB = aTrgVert2 - aTrgVert1;

            BVH_Vec3d aNorm;
            if (aTrgVert2.IsEqual(aTrgVert3)) // is this degenerate triangle (= segment)
            {
                BVH_Vec3d aAP = _object - aTrgVert1;
                aNorm = BVH_Vec3d.Cross(BVH_Vec3d.Cross(aAP, aAB), aAB);
            }
            else
            {
                BVH_Vec3d aAC = aTrgVert3 - aTrgVert1;
                aNorm = BVH_Vec3d.Cross(aAB, aAC);
            }

            double aNormSqLen = aNorm.Dot(aNorm);

            // check if the distance is under perpendicular
            BVH_Vec3d aCrossCross = BVH_Vec3d.Cross(aDirect, aNorm);
            double aCrossCrossSqLen = aCrossCross.Dot(aCrossCross);
            if (aCrossCrossSqLen > global::Kerf.Engine.Foundation.Precision.SquareConfusion() * aSqDistance * aNormSqLen)
            {
                // the distance is not under perpendicular
                if (myMinDistance - Math.Sqrt(aSqDistance) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    myMinDistance = Math.Sqrt(aSqDistance);
                    myMinDistPoint = aNearestPnt;
                    myMinPrjState = aPrjState;
                }

                return false;
            }
        }

        // the distance is under perpendicular
        if (_distance - Math.Sqrt(aSqDistance) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _distance = Math.Sqrt(aSqDistance);
            myExtremaPoint = aNearestPnt;
            myExtPrjState = aPrjState;

            return true;
        }

        return false;
    }

    /// <summary>
    /// Computes the distance between the object and the BVH tree, falling back on the smallest non-perpendicular
    /// distance (hides BVH_Distance's, as OCCT's does).
    /// </summary>
    protected new double ComputeDistance()
    {
        _isDone = Select() > 0;

        if (!_isDone)
        {
            if (myMinDistance < double.MaxValue)
            {
                myExtremaPoint = myMinDistPoint;
                myExtPrjState = myMinPrjState;
                _isDone = true;
            }

            _distance = myMinDistance;
        }

        return _distance;
    }

    /// <summary>True if the node is on the border of the triangulation.</summary>
    public static bool IsNodeOnBorder(int theNodeIdx, Triangulation theTr)
    {
        var aPolyConnect = new Connect(theTr);

        int aContTrg; // index of triangle containing exploring node
        Span<int> aContTrgNodes = stackalloc int[3];
        Span<int> aAdjTrg = stackalloc int[3];
        for (aPolyConnect.Initialize(theNodeIdx); aPolyConnect.More(); aPolyConnect.Next())
        {
            aContTrg = aPolyConnect.Value();

            theTr.Triangle(aContTrg).Get(out aContTrgNodes[0], out aContTrgNodes[1], out aContTrgNodes[2]); // indices of nodes of the triangle

            aPolyConnect.Triangles(aContTrg, out aAdjTrg[0], out aAdjTrg[1], out aAdjTrg[2]); // indices of adjacent triangles

            for (int j = 0; j < 3; j++)
            {
                int k = (j + 1) % 3;
                if (aAdjTrg[j] == 0) // free segment of triangle
                {
                    // aContTrgNodes[j], aContTrgNodes[k] are ends of free segment and it is a part of border
                    if (aContTrgNodes[j] == theNodeIdx || aContTrgNodes[k] == theNodeIdx)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /// <summary>True if the edge (given by the numbers of its nodes in the triangle) is on the border.</summary>
    public static bool IsEdgeOnBorder(int theTrgIdx, int theFirstEdgeNodeIdx, int theSecondEdgeNodeIdx, Triangulation theTr)
    {
        var aPolyConnect = new Connect(theTr);

        Span<int> aAdjTrg = stackalloc int[3];
        aPolyConnect.Triangles(theTrgIdx, out aAdjTrg[0], out aAdjTrg[1], out aAdjTrg[2]); // indices of adjacent triangles

        for (int j = 0; j < 3; j++)
        {
            int k = (j + 1) % 3;
            if (aAdjTrg[j] == 0) // free segment of triangle
            {
                // are ends of free segment and it is a part of border
                if (j == theFirstEdgeNodeIdx && k == theSecondEdgeNodeIdx)
                {
                    return true;
                }
            }
        }

        return false;
    }

    private void defineStatusProxPnt1()
    {
        if (myIsProxVtx1FromAddSet)
        {
            myPntStatus1 = myAddStatus1[myProxVtxIdx1];
            return;
        }

        int aFaceID1 = mySet1!.GetShapeIDOfVtx(myProxVtxIdx1);

        if (myShapeList1[aFaceID1].ShapeType() == TopAbs_EDGE)
        {
            NCollection_LinearVector<BVH_Vec3d> aVertices1 = mySet1.GetVertices();
            int aVtxSize = aVertices1.Size();
            int aLastIdx = aVtxSize - 1;

            if ((aVertices1[0] - aVertices1[aLastIdx]).Modulus() < global::Kerf.Engine.Foundation.Precision.Confusion()) // if closed
            {
                myPntStatus1 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                return;
            }

            if (myProxVtxIdx1 == 0 || myProxVtxIdx1 == aLastIdx)
            {
                myPntStatus1 = ProxPnt_Status.ProxPnt_Status_BORDER;
            }
            else
            {
                myPntStatus1 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
        }
        else if (myShapeList1[aFaceID1].ShapeType() == TopAbs_FACE)
        {
            int aNodeIdx = mySet1.GetVtxIdxInShape(myProxVtxIdx1) + 1;

            TopoDS_Face aF = TopoDS.Face(myShapeList1[aFaceID1]);
            Triangulation aTr = BRep_Tool.Triangulation(aF, out _)!;

            if (IsNodeOnBorder(aNodeIdx, aTr))
            {
                myPntStatus1 = ProxPnt_Status.ProxPnt_Status_BORDER;
            }
            else
            {
                myPntStatus1 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
        }
    }

    private void defineStatusProxPnt2()
    {
        int aTrgIdx = myProxPrjState.GetTrgIdx();
        int aFaceID2 = mySet2!.GetFaceID(aTrgIdx);

        if (myShapeList2[aFaceID2].ShapeType() == TopAbs_EDGE)
        {
            if (myProxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_INNER)
            {
                return;
            }
            else
            {
                NCollection_LinearVector<BVH_Vec3d> aVertices2 = mySet2.GetVertices();
                int aVtxSize = aVertices2.Size();
                int aLastIdx = aVtxSize - 1;

                if ((aVertices2[0] - aVertices2[aLastIdx]).Modulus() < global::Kerf.Engine.Foundation.Precision.Confusion()) // if closed
                {
                    myPntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    return;
                }

                mySet2.GetVtxIndices(aTrgIdx, out NCollection_Array1<int> aVtxIndicesOfTrg);

                int aFirstNodeNum = myProxPrjState.GetNumberOfFirstNode();
                int aFirstVtxIdx = aVtxIndicesOfTrg[aFirstNodeNum];

                if (myProxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_VERTEX)
                {
                    if (aFirstVtxIdx == 0 || aFirstVtxIdx == aLastIdx)
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
                else if (myProxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_EDGE)
                {
                    int aLastNodeNum = myProxPrjState.GetNumberOfLastNode();
                    int aLastVtxIdx = aVtxIndicesOfTrg[aLastNodeNum];

                    // it could be border only in case projection is on a degenerated edge
                    if (aFirstVtxIdx == aLastVtxIdx && (aFirstVtxIdx == 0 || aFirstVtxIdx == aLastIdx))
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
            }
        }
        else if (myShapeList2[aFaceID2].ShapeType() == TopAbs_FACE)
        {
            if (myProxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_INNER)
            {
                myPntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
            else
            {
                TopoDS_Face aF = TopoDS.Face(myShapeList2[aFaceID2]);
                Triangulation aTr = BRep_Tool.Triangulation(aF, out _)!;

                mySet2.GetVtxIndices(aTrgIdx, out NCollection_Array1<int> aVtxIndicesOfTrg);

                if (myProxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_VERTEX)
                {
                    int aNodeNum = myProxPrjState.GetNumberOfFirstNode();
                    int aNodeIdx = mySet2.GetVtxIdxInShape(aVtxIndicesOfTrg[aNodeNum]) + 1;

                    if (IsNodeOnBorder(aNodeIdx, aTr))
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
                else if (myProxPrjState.GetPrjState() == BVH_PrjState.PrjStateInTriangle_EDGE)
                {
                    int aTrgIdxInShape = mySet2.GetTrgIdxInShape(aTrgIdx) + 1;

                    if (IsEdgeOnBorder(aTrgIdxInShape,
                                       myProxPrjState.GetNumberOfFirstNode(),
                                       myProxPrjState.GetNumberOfLastNode(),
                                       aTr))
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_BORDER;
                    }
                    else
                    {
                        myPntStatus2 = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                    }
                }
            }
        }
    }

    /// <summary>Defines the status of the proximity points.</summary>
    protected void DefineStatusProxPnt()
    {
        // define the status of proximity point from 1st BVH
        defineStatusProxPnt1();

        // define the status of proximity point from 2nd BVH
        defineStatusProxPnt2();
    }

    /// <summary>The points of the two sets giving the proximity distance.</summary>
    public void ProximityPoints(out BVH_Vec3d thePoint1, out BVH_Vec3d thePoint2)
    {
        thePoint1 = myPnt1;
        thePoint2 = myPnt2;
    }

    /// <summary>The statuses of the points giving the proximity distance.</summary>
    public void ProximityPointsStatus(out ProxPnt_Status thePointStatus1, out ProxPnt_Status thePointStatus2)
    {
        thePointStatus1 = myPntStatus1;
        thePointStatus2 = myPntStatus2;
    }

    /// <summary>The proximity distance.</summary>
    public double ProximityDistance() => myProxDist;
}
