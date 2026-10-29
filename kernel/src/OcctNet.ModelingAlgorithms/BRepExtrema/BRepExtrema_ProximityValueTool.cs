// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityValueTool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ProximityValueTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2022-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using ProxPnt_Status = Occt.BRepExtrema_ProximityDistTool.ProxPnt_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The CellFilter inspector of gp_XYZ points in 3D space, used to find coincident points within a tolerance.
/// </summary>
public sealed class BRepExtrema_VertexInspector : INCollection_CellFilter_Inspector<int, gp_XYZ>
{
    private double myTol;
    private bool myIsNeedAdd;
    private readonly NCollection_DynamicArray<gp_XYZ> myPoints = new();
    private gp_XYZ myCurrent;

    public static int Dimension => 3;

    public static double Coord(int i, gp_XYZ thePnt) => thePnt.Coord(i + 1);

    public static gp_XYZ Shift(in gp_XYZ thePnt, double theTol) => new(thePnt.X() + theTol, thePnt.Y() + theTol, thePnt.Z() + theTol);

    /// <summary>An inspector with the square tolerance Precision::SquareConfusion().</summary>
    public BRepExtrema_VertexInspector()
    {
        myTol = Precision.SquareConfusion();
        myIsNeedAdd = true;
    }

    /// <summary>Keeps a point to compare with.</summary>
    public void Add(in gp_XYZ thePnt) => myPoints.Append(thePnt);

    /// <summary>Sets the (square) tolerance of the comparison of coordinates.</summary>
    public void SetTol(double theTol) => myTol = theTol;

    /// <summary>Sets the point to search coincidences for.</summary>
    public void SetCurrent(in gp_XYZ theCurPnt)
    {
        myCurrent = theCurPnt;
        myIsNeedAdd = true;
    }

    /// <summary>True if the current point coincides with no kept point.</summary>
    public bool IsNeedAdd() => myIsNeedAdd;

    /// <summary>
    /// Compares the current point with the kept point <paramref name="theTarget"/> (from 1).
    /// </summary>
    /// <remarks>
    /// OCCT: each call sets the flag again before comparing, so only the last target inspected decides
    /// (occt-issues.md).
    /// </remarks>
    public NCollection_CellFilter_Action Inspect(int theTarget)
    {
        myIsNeedAdd = true;

        gp_XYZ aPnt = myPoints.Value(theTarget - 1);
        double aDx, aDy, aDz;
        aDx = myCurrent.X() - aPnt.X();
        aDy = myCurrent.Y() - aPnt.Y();
        aDz = myCurrent.Z() - aPnt.Z();

        if ((aDx * aDx <= myTol) && (aDy * aDy <= myTol) && (aDz * aDz <= myTol))
        {
            myIsNeedAdd = false;
        }

        return NCollection_CellFilter_Action.CellFilter_Keep;
    }
}

/// <summary>
/// The proximity value from one BVH primitive set to another, solving the max(min) problem, for edge/edge and
/// face/face cases only: the coarser mesh is refined first. Results are approximate and depend on the tessellations.
/// </summary>
/// <remarks>A working object (decision 21), used by <see cref="BRepExtrema_ShapeProximity"/>. It only reads the meshes.</remarks>
public sealed class BRepExtrema_ProximityValueTool
{
    private BRepExtrema_TriangleSet? mySet1;
    private BRepExtrema_TriangleSet? mySet2;
    private NCollection_DynamicArray<TopoDS_Shape> myShapeList1 = new();
    private NCollection_DynamicArray<TopoDS_Shape> myShapeList2 = new();
    private TopoDS_Shape myShape1;
    private TopoDS_Shape myShape2;
    private readonly NCollection_LinearVector<BVH_Vec3d> myAddVertices1 = new(); //!< Additional vertices on the 1st shape if its mesh is coarser.
    private readonly NCollection_LinearVector<BVH_Vec3d> myAddVertices2 = new(); //!< Additional vertices on the 2nd shape if its mesh is coarser.
    private readonly NCollection_DynamicArray<ProxPnt_Status> myAddStatus1 = new(); //!< Status of additional vertices on the 1st shape.
    private readonly NCollection_DynamicArray<ProxPnt_Status> myAddStatus2 = new(); //!< Status of additional vertices on the 2nd shape.
    private bool myIsInitS1; //!< Is the 1st shape initialized?
    private bool myIsInitS2; //!< Is the 2nd shape initialized?
    private bool myIsRefinementRequired1; //!< Flag about the need to refine the 1st shape.
    private bool myIsRefinementRequired2; //!< Flag about the need to refine the 2nd shape.
    private int myNbNodes1; //!< Number of nodes in triangulation of the 1st shape.
    private int myNbNodes2; //!< Number of nodes in triangulation of the 2nd shape.
    private double myStep1; //!< Step for getting vertices on the 1st shape.
    private double myStep2; //!< Step for getting vertices on the 2nd shape.
    private readonly NCollection_CellFilter<BRepExtrema_VertexInspector, int, gp_XYZ> myCells = new();
    private BRepExtrema_VertexInspector myInspector = new();
    private TopAbs_ShapeEnum myShapeType1; //!< 1st shape type.
    private TopAbs_ShapeEnum myShapeType2; //!< 2nd shape type.
    private double myDistance; //!< Distance
    private bool myIsDone; //!< State of the algorithm
    private int myNbSamples1; //!< Number of samples points on the first shape
    private int myNbSamples2; //!< Number of samples points on the second shape
    private gp_Pnt myPnt1, myPnt2; //!< Proximity points
    private ProxPnt_Status myPntStatus1, myPntStatus2; //!< Proximity points' status

    /// <summary>An uninitialized proximity tool.</summary>
    public BRepExtrema_ProximityValueTool()
    {
        myIsRefinementRequired1 = false;
        myIsRefinementRequired2 = false;
        myDistance = double.MaxValue;
        myIsDone = false;
        myNbSamples1 = 0;
        myNbSamples2 = 0;
        // Should be initialized later
        myIsInitS1 = myIsInitS2 = false;
    }

    /// <summary>A proximity tool for the given element sets and lists of sub-shapes.</summary>
    public BRepExtrema_ProximityValueTool(BRepExtrema_TriangleSet theSet1,
                                          BRepExtrema_TriangleSet theSet2,
                                          NCollection_DynamicArray<TopoDS_Shape> theShapeList1,
                                          NCollection_DynamicArray<TopoDS_Shape> theShapeList2)
    {
        myIsRefinementRequired1 = false;
        myIsRefinementRequired2 = false;
        myDistance = double.MaxValue;
        myIsDone = false;
        myNbSamples1 = 0;
        myNbSamples2 = 0;
        LoadShapeLists(theShapeList1, theShapeList2);
        LoadTriangleSets(theSet1, theSet2);
    }

    /// <summary>Loads the element sets.</summary>
    public void LoadTriangleSets(BRepExtrema_TriangleSet theSet1, BRepExtrema_TriangleSet theSet2)
    {
        mySet1 = theSet1;
        mySet2 = theSet2;

        MarkDirty();
    }

    /// <summary>True if the proximity test is completed.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>Marks the results as outdated.</summary>
    public void MarkDirty() => myIsDone = false;

    /// <summary>The computed distance.</summary>
    public double Distance() => myDistance;

    /// <summary>The points of the two sets giving the proximity distance.</summary>
    public void ProximityPoints(out gp_Pnt thePoint1, out gp_Pnt thePoint2)
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

    //=======================================================================
    // function : calcEdgeRefinementStep
    // purpose  : Calculates the edge refinement step
    //=======================================================================
    private static double calcEdgeRefinementStep(in TopoDS_Edge theEdge, int theNbNodes)
    {
        if (theNbNodes < 2)
        {
            return 0;
        }

        var aBAC = new BRepAdaptor_Curve(theEdge);
        double aLen = GCPnts_AbscissaPoint.Length(aBAC);
        return aLen / (double)(theNbNodes - 1);
    }

    //=======================================================================
    // function : calcFaceRefinementStep
    // purpose  : Calculates the face refinement step as an approximate square
    // (Shape area / number triangles) * 2
    //=======================================================================
    private static double calcFaceRefinementStep(in TopoDS_Face theFace, int theNbTrg)
    {
        if (theNbTrg < 1)
        {
            return 0;
        }

        var props = new GProp_GProps();
        BRepGProp.SurfaceProperties(theFace, props);
        double aArea = props.Mass();
        return 2 * (aArea / (double)theNbTrg);
    }

    //=======================================================================
    // function : getInfoForRefinement
    // purpose  : Gets shape data for further refinement
    //=======================================================================
    private static bool getInfoForRefinement(in TopoDS_Shape theShape,
                                             ref TopAbs_ShapeEnum theShapeType,
                                             ref int theNbNodes,
                                             ref double theStep)
    {
        if (theShape.ShapeType() == TopAbs_FACE)
        {
            theShapeType = TopAbs_FACE;
            TopoDS_Face aF = TopoDS.Face(theShape);

            Poly_Triangulation? aTriangulation = BRep_Tool.Triangulation(aF, out _);

            if (aTriangulation is null)
            {
                return false;
            }

            theNbNodes = aTriangulation.NbNodes();
            int aNbTrg = aTriangulation.NbTriangles();
            theStep = calcFaceRefinementStep(aF, aNbTrg);
        }
        else if (theShape.ShapeType() == TopAbs_EDGE)
        {
            theShapeType = TopAbs_EDGE;
            TopoDS_Edge aE = TopoDS.Edge(theShape);

            Poly_Polygon3D? aPolygon = BRep_Tool.Polygon3D(aE, out _);

            if (aPolygon is null)
            {
                return false;
            }

            theNbNodes = aPolygon.NbNodes();
            theStep = calcEdgeRefinementStep(aE, theNbNodes);
        }
        else
        {
            return false;
        }

        if (theStep < Precision.Confusion())
        {
            return false;
        }

        return true;
    }

    /// <summary>Loads the lists of sub-shapes (copies, as OCCT assigns by value); the first of each is measured.</summary>
    public void LoadShapeLists(NCollection_DynamicArray<TopoDS_Shape> theShapeList1, NCollection_DynamicArray<TopoDS_Shape> theShapeList2)
    {
        myShapeList1 = new NCollection_DynamicArray<TopoDS_Shape>(theShapeList1);
        myShapeList2 = new NCollection_DynamicArray<TopoDS_Shape>(theShapeList2);

        myShape1 = theShapeList1[0];
        myIsInitS1 = getInfoForRefinement(myShape1, ref myShapeType1, ref myNbNodes1, ref myStep1);

        myShape2 = theShapeList2[0];
        myIsInitS2 = getInfoForRefinement(myShape2, ref myShapeType2, ref myNbNodes2, ref myStep2);

        MarkDirty();
    }

    /// <summary>The numbers of sample points of each shape; zero or less means every triangulation node.</summary>
    public void SetNbSamplePoints(int theSamples1 = 0, int theSamples2 = 0)
    {
        myNbSamples1 = theSamples1;
        myNbSamples2 = theSamples2;

        MarkDirty();
    }

    //=======================================================================
    // function : computeProximityValue
    // purpose  : Returns the computed proximity value from first BVH to another one
    //=======================================================================
    private static double computeProximityDist(BRepExtrema_TriangleSet theSet1,
                                               int theNbSamples1,
                                               NCollection_LinearVector<BVH_Vec3d> theAddVertices1,
                                               NCollection_DynamicArray<ProxPnt_Status> theAddStatus1,
                                               BRepExtrema_TriangleSet theSet2,
                                               NCollection_DynamicArray<TopoDS_Shape> theShapeList1,
                                               NCollection_DynamicArray<TopoDS_Shape> theShapeList2,
                                               ref BVH_Vec3d thePoint1,
                                               ref BVH_Vec3d thePoint2,
                                               ref ProxPnt_Status thePointStatus1,
                                               ref ProxPnt_Status thePointStatus2)
    {
        var aProxDistTool = new BRepExtrema_ProximityDistTool(theSet1,
                                                              theNbSamples1,
                                                              theAddVertices1,
                                                              theAddStatus1,
                                                              theSet2,
                                                              theShapeList1,
                                                              theShapeList2);
        aProxDistTool.Perform();

        if (!aProxDistTool.IsDone())
        {
            return -1.0;
        }

        aProxDistTool.ProximityPoints(out thePoint1, out thePoint2);
        aProxDistTool.ProximityPointsStatus(out thePointStatus1, out thePointStatus2);

        return aProxDistTool.ProximityDistance();
    }

    //=======================================================================
    // function : getEdgeAdditionalVertices
    // purpose  : Gets additional vertices and their statuses on the edge with the input step
    //=======================================================================
    private static bool getEdgeAdditionalVertices(in TopoDS_Edge theEdge,
                                                  double theStep,
                                                  NCollection_LinearVector<BVH_Vec3d> theAddVertices,
                                                  NCollection_DynamicArray<ProxPnt_Status> theAddStatuses)
    {
        var aBAC = new BRepAdaptor_Curve(theEdge);

        if (!aBAC.Is3DCurve() || theStep < Precision.Confusion())
        {
            return false;
        }

        double aLen = GCPnts_AbscissaPoint.Length(aBAC);
        int aNbSamplePoints = (int)(aLen / theStep) + 1;

        var aGCPnts = new GCPnts_QuasiUniformAbscissa(aBAC, Max(3, aNbSamplePoints));

        if (!aGCPnts.IsDone())
        {
            return false;
        }

        int aNbNodes = aGCPnts.NbPoints();
        for (int aVertIdx = 2; aVertIdx < aNbNodes; ++aVertIdx) // don't add extreme points
        {
            double aPar = aGCPnts.Parameter(aVertIdx);
            gp_Pnt aP = aBAC.Value(aPar);

            theAddVertices.Append(new BVH_Vec3d(aP.X(), aP.Y(), aP.Z()));
            theAddStatuses.Append(ProxPnt_Status.ProxPnt_Status_MIDDLE);
        }

        return true;
    }

    //=======================================================================
    // function : doRecurTrgSplit
    // purpose  : Splits the triangle into two ones recursively, halving the longest side
    //           until the area of the current triangle > input step
    //=======================================================================
    private void doRecurTrgSplit(ReadOnlySpan<gp_Pnt> theTrg,
                                 ReadOnlySpan<ProxPnt_Status> theEdgesStatus,
                                 double theTol,
                                 double theStep,
                                 NCollection_LinearVector<BVH_Vec3d> theAddVertices,
                                 NCollection_DynamicArray<ProxPnt_Status> theAddStatuses)
    {
        gp_XYZ aTrgSide1 = theTrg[1].Coord() - theTrg[0].Coord();
        gp_XYZ aTrgSide2 = theTrg[2].Coord() - theTrg[0].Coord();
        double aTrgArea = 0.5 * aTrgSide1.CrossMagnitude(aTrgSide2);

        if (aTrgArea - theStep < Precision.SquareConfusion())
        {
            return;
        }

        ReadOnlySpan<double> aD = [theTrg[0].Distance(theTrg[1]),
                                   theTrg[1].Distance(theTrg[2]),
                                   theTrg[2].Distance(theTrg[0])];

        int aBisectedEdgeIdx = aD[0] > aD[1] ? (aD[0] > aD[2] ? 0 : 2) : (aD[1] > aD[2] ? 1 : 2);
        var aCenterOfMaxSide = new gp_Pnt(theTrg[aBisectedEdgeIdx].Coord());
        aCenterOfMaxSide.BaryCenter(0.5, theTrg[(aBisectedEdgeIdx + 1) % 3], 0.5);

        var aBox = new Bnd_Box();
        aBox.Add(aCenterOfMaxSide);
        aBox.Enlarge(theTol);
        myInspector.SetCurrent(aCenterOfMaxSide.Coord());
        myCells.Inspect(aBox.CornerMin().XYZ(), aBox.CornerMax().XYZ(), ref myInspector);

        if (myInspector.IsNeedAdd()) // is point aCenterOfMaxSide unique
        {
            var aBisectingPnt = new BVH_Vec3d(aCenterOfMaxSide.X(), aCenterOfMaxSide.Y(), aCenterOfMaxSide.Z());
            theAddVertices.Append(aBisectingPnt);
            theAddStatuses.Append(theEdgesStatus[aBisectedEdgeIdx]);
            myInspector.Add(aCenterOfMaxSide.Coord());
            myCells.Add(theAddVertices.Size(), aBox.CornerMin().XYZ(), aBox.CornerMax().XYZ());
        }

        Span<gp_Pnt> aTrg1 = [theTrg[0], theTrg[1], theTrg[2]];
        Span<gp_Pnt> aTrg2 = [theTrg[0], theTrg[1], theTrg[2]];
        Span<ProxPnt_Status> aEdgesStatus1 = [theEdgesStatus[0], theEdgesStatus[1], theEdgesStatus[2]];
        Span<ProxPnt_Status> aEdgesStatus2 = [theEdgesStatus[0], theEdgesStatus[1], theEdgesStatus[2]];
        switch (aBisectedEdgeIdx)
        {
            case 0:
                aTrg1[0] = aTrg2[1] = aCenterOfMaxSide;
                aEdgesStatus1[2] = aEdgesStatus2[1] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                break;
            case 1:
                aTrg1[1] = aTrg2[2] = aCenterOfMaxSide;
                aEdgesStatus1[0] = aEdgesStatus2[2] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                break;
            case 2:
                aTrg1[2] = aTrg2[0] = aCenterOfMaxSide;
                aEdgesStatus1[1] = aEdgesStatus2[0] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
                break;
        }

        doRecurTrgSplit(aTrg1, aEdgesStatus1, theTol, theStep, theAddVertices, theAddStatuses);
        doRecurTrgSplit(aTrg2, aEdgesStatus2, theTol, theStep, theAddVertices, theAddStatuses);
    }

    private static double getModelRange(in TopLoc_Location theLocation, Poly_Triangulation theTr)
    {
        var aBox = new Bnd_Box();
        theTr.MinMax(ref aBox, theLocation.Transformation());
        double aXm = 0.0, aYm = 0.0, aZm = 0.0, aXM = 0.0, aYM = 0.0, aZM = 0.0;
        aBox.Get(out aXm, out aYm, out aZm, out aXM, out aYM, out aZM);
        double aRange = aXM - aXm;
        aRange = Max(aRange, aYM - aYm);
        aRange = Max(aRange, aZM - aZm);

        return aRange;
    }

    private static void getNodesOfTrg(int theTriIdx, in TopLoc_Location theLocation, Poly_Triangulation theTr, Span<gp_Pnt> theTrg)
    {
        theTr.Triangle(theTriIdx).Get(out int aVtxIdx1, out int aVtxIdx2, out int aVtxIdx3);

        gp_Pnt aVtx1 = theTr.Node(aVtxIdx1);
        aVtx1.Transform(theLocation.Transformation());
        theTrg[0] = aVtx1;

        gp_Pnt aVtx2 = theTr.Node(aVtxIdx2);
        aVtx2.Transform(theLocation.Transformation());
        theTrg[1] = aVtx2;

        gp_Pnt aVtx3 = theTr.Node(aVtxIdx3);
        aVtx3.Transform(theLocation.Transformation());
        theTrg[2] = aVtx3;
    }

    // Gets status of triangle edges - on the border or middle of the face
    private static void getEdgesStatus(int theTriIdx, Poly_Triangulation theTr, Span<ProxPnt_Status> theEdgesStatus1)
    {
        for (int j = 0; j < 3; j++)
        {
            int k = (j + 1) % 3;

            if (BRepExtrema_ProximityDistTool.IsEdgeOnBorder(theTriIdx, j, k, theTr))
            {
                theEdgesStatus1[j] = ProxPnt_Status.ProxPnt_Status_BORDER;
            }
            else
            {
                theEdgesStatus1[j] = ProxPnt_Status.ProxPnt_Status_MIDDLE;
            }
        }
    }

    //=======================================================================
    // function : getFaceAdditionalVertices
    // purpose  : Gets additional vertices and their statuses on the face with the input step (triangle
    // square)
    //=======================================================================
    private bool getFaceAdditionalVertices(in TopoDS_Face theFace,
                                           double theStep,
                                           NCollection_LinearVector<BVH_Vec3d> theAddVertices,
                                           NCollection_DynamicArray<ProxPnt_Status> theAddStatuses)
    {
        double aTol = Precision.Confusion();

        Poly_Triangulation? aTr = BRep_Tool.Triangulation(theFace, out TopLoc_Location aLocation);

        if (aTr is null)
        {
            return false;
        }

        myCells.Reset(Max(aTol, getModelRange(aLocation, aTr) / IntegerLast()));

        Span<gp_Pnt> aTrg = stackalloc gp_Pnt[3];
        Span<ProxPnt_Status> aEdgesStatus = stackalloc ProxPnt_Status[3];
        for (int aTriIdx = 1; aTriIdx <= aTr.NbTriangles(); ++aTriIdx)
        {
            getNodesOfTrg(aTriIdx, aLocation, aTr, aTrg);
            getEdgesStatus(aTriIdx, aTr, aEdgesStatus);
            doRecurTrgSplit(aTrg, aEdgesStatus, aTol, theStep, theAddVertices, theAddStatuses);
        }

        return true;
    }

    //=======================================================================
    // function : getShapesVertices
    // purpose  : Gets additional vertices on shapes with refining a coarser one if it's needed
    //=======================================================================
    private bool getShapesAdditionalVertices()
    {
        // estimate the density of meshes of shapes to add points to a coarcer one
        // target steps for refinement
        double aStep1 = myStep1;
        double aStep2 = myStep2;

        if ((myShapeType1 == TopAbs_EDGE) && (myShapeType2 == TopAbs_EDGE))
        {
            if (myNbSamples1 > myNbNodes1) // 1st edge needs refinement
            {
                aStep1 = calcEdgeRefinementStep(TopoDS.Edge(myShape1), myNbSamples1);
                myIsRefinementRequired1 = true;
            }

            if (myNbSamples2 > myNbNodes2) // 2nd edge needs refinement
            {
                aStep2 = calcEdgeRefinementStep(TopoDS.Edge(myShape2), myNbSamples2);
                myIsRefinementRequired2 = true;
            }

            if (aStep1 / aStep2 > 2.0) // 1st edge needs refinement
            {
                myIsRefinementRequired1 = true;
                aStep1 = aStep2;
            }
            else if (aStep2 / aStep1 > 2.0) // 2nd edge needs refinement
            {
                myIsRefinementRequired2 = true;
                aStep2 = aStep1;
            }

            if (myIsRefinementRequired1)
            {
                if (!getEdgeAdditionalVertices(TopoDS.Edge(myShape1), aStep1, myAddVertices1, myAddStatus1))
                {
                    return false;
                }
            }

            if (myIsRefinementRequired2)
            {
                if (!getEdgeAdditionalVertices(TopoDS.Edge(myShape2), aStep2, myAddVertices2, myAddStatus2))
                {
                    return false;
                }
            }
        }
        else if ((myShapeType1 == TopAbs_FACE) && (myShapeType2 == TopAbs_FACE))
        {
            if (aStep1 / aStep2 > 2) // 1st face needs refinement
            {
                myIsRefinementRequired1 = true;
                aStep1 = myStep2;
            }
            else if (aStep2 / aStep1 > 2.0) // 2nd face needs refinement
            {
                myIsRefinementRequired2 = true;
                aStep2 = myStep1;
            }

            if (myIsRefinementRequired1)
            {
                return getFaceAdditionalVertices(TopoDS.Face(myShape1), aStep1, myAddVertices1, myAddStatus1);
            }

            if (myIsRefinementRequired2)
            {
                return getFaceAdditionalVertices(TopoDS.Face(myShape2), aStep2, myAddVertices2, myAddStatus2);
            }
        }

        return true;
    }

    //=======================================================================
    // function : Perform
    // purpose  : Performs the computation of the proximity value
    //=======================================================================
    /// <summary>Computes the proximity value; when done, <paramref name="theTolerance"/> receives it.</summary>
    public void Perform(ref double theTolerance)
    {
        if (!myIsInitS1 || !myIsInitS2 || (myShapeType1 != myShapeType2))
        {
            return;
        }

        // get vertices on shapes with refining a coarser mesh if it's needed
        if (!getShapesAdditionalVertices())
        {
            return;
        }

        // max(min) dist from the 1st shape to the 2nd one
        BVH_Vec3d aP1_1 = default, aP1_2 = default;
        ProxPnt_Status aPointStatus1_1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        ProxPnt_Status aPointStatus1_2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;

        double aProximityDist1 = computeProximityDist(mySet1!,
                                                      myNbSamples1,
                                                      myAddVertices1,
                                                      myAddStatus1,
                                                      mySet2!,
                                                      myShapeList1,
                                                      myShapeList2,
                                                      ref aP1_1,
                                                      ref aP1_2,
                                                      ref aPointStatus1_1,
                                                      ref aPointStatus1_2);

        if (aProximityDist1 < 0.0)
        {
            return;
        }

        // max(min) dist from the 2nd shape to t he 1st one
        BVH_Vec3d aP2_1 = default, aP2_2 = default;
        ProxPnt_Status aPointStatus2_1 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;
        ProxPnt_Status aPointStatus2_2 = ProxPnt_Status.ProxPnt_Status_UNKNOWN;

        double aProximityDist2 = computeProximityDist(mySet2!,
                                                      myNbSamples2,
                                                      myAddVertices2,
                                                      myAddStatus2,
                                                      mySet1!,
                                                      myShapeList2,
                                                      myShapeList1,
                                                      ref aP2_2,
                                                      ref aP2_1,
                                                      ref aPointStatus2_2,
                                                      ref aPointStatus2_1);

        if (aProximityDist2 < 0.0)
        {
            return;
        }

        // min dist of the two max(min) dists
        if (aProximityDist1 < aProximityDist2)
        {
            myDistance = aProximityDist1;
            myPnt1 = new gp_Pnt(aP1_1.x(), aP1_1.y(), aP1_1.z());
            myPnt2 = new gp_Pnt(aP1_2.x(), aP1_2.y(), aP1_2.z());
            myPntStatus1 = aPointStatus1_1;
            myPntStatus2 = aPointStatus1_2;
        }
        else
        {
            myDistance = aProximityDist2;
            myPnt1 = new gp_Pnt(aP2_1.x(), aP2_1.y(), aP2_1.z());
            myPnt2 = new gp_Pnt(aP2_2.x(), aP2_2.y(), aP2_2.z());
            myPntStatus1 = aPointStatus2_1;
            myPntStatus2 = aPointStatus2_2;
        }

        myIsDone = true;
        theTolerance = myDistance;
    }
}
