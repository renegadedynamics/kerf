// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_GlueEnum;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The intersection of a pair of faces run by the pave filler's parallel loop (<c>BOPAlgo_PaveFiller_6.cxx</c>'s
/// file-local class).
/// </summary>
/// <remarks>
/// OCCT derives it from IntTools_FaceFace and BOPAlgo_ParallelAlgo; C# has one base, so the parallel algorithm's part
/// (the progress range, the report, the parallel flag) is a <see cref="BOPAlgo_Options"/> member with OCCT's
/// members forwarding to it. OCCT runs the vector without a context (<c>BOPTools_Parallel::Perform(myRunParallel,
/// aVFaceFace)</c>), so each face/face intersection makes its own context, as IntTools_FaceFace does natively: the
/// class is a plain solver, not a context solver.
/// </remarks>
internal sealed class BOPAlgo_FaceFace : IntTools_FaceFace, BOPTools_Parallel.ISolver
{
    private readonly BOPAlgo_Options myOptions = new();
    private Message_ProgressRange myProgressRange;

    private int myIF1;
    private int myIF2;
    private double myTolFF;
    private TopoDS_Face myF1 = new();
    private TopoDS_Face myF2 = new();
    private Bnd_Box myBox1 = new();
    private Bnd_Box myBox2 = new();
    private gp_Trsf myTrsf = new();

    public BOPAlgo_FaceFace()
    {
        myIF1 = -1;
        myIF2 = -1;
        myTolFF = 1.0e-7;
    }

    //
    public void SetIndices(int nF1, int nF2)
    {
        myIF1 = nF1;
        myIF2 = nF2;
    }

    //
    public void Indices(out int nF1, out int nF2)
    {
        nF1 = myIF1;
        nF2 = myIF2;
    }

    //
    public void SetFaces(in TopoDS_Face aF1, in TopoDS_Face aF2)
    {
        myF1 = aF1;
        myF2 = aF2;
    }

    //
    public void SetBoxes(in Bnd_Box theBox1, in Bnd_Box theBox2)
    {
        myBox1 = theBox1;
        myBox2 = theBox2;
    }

    //
    public new TopoDS_Face Face1() => myF1;

    //
    public new TopoDS_Face Face2() => myF2;

    //
    public void SetTolFF(double aTolFF) => myTolFF = aTolFF;

    //
    public double TolFF() => myTolFF;

    //
    public gp_Trsf Trsf() => myTrsf;

    // BOPAlgo_ParallelAlgo's and BOPAlgo_Options' members used by the filler.
    public void SetProgressRange(Message_ProgressRange theRange) => myProgressRange = theRange;

    public void SetRunParallel(bool theFlag) => myOptions.SetRunParallel(theFlag);

    public bool HasErrors() => myOptions.HasErrors();

    //
    public void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (myOptions.UserBreak(aPS))
        {
            return;
        }
        try
        {
            gp_Trsf aTrsf = new();
            TopoDS_Face aF1 = myF1, aF2 = myF2;
            if (BOPAlgo_Tools.TrsfToPoint(myBox1, myBox2, ref aTrsf))
            {
                // Shapes are located far from origin, move the shapes to the origin,
                // to increase the accuracy of intersection.
                TopLoc_Location aLoc = new(aTrsf);
                aF1.Move(aLoc);
                aF2.Move(aLoc);

                // The starting point is initialized only with the UV parameters
                // on the faces - 3D point is not set (see GetEFPnts method),
                // so no need to transform anything.

                myTrsf = aTrsf.Inverted();
            }

            Perform(aF1, aF2, myOptions.RunParallel());
        }
        catch (Standard_Failure)
        {
            myOptions.AddError(new BOPAlgo_AlertIntersectionFailed());
        }
    }

    //
    public void ApplyTrsf()
    {
        if (IsDone())
        {
            // Update curves
            for (int i = 1; i <= mySeqOfCurve.Length(); ++i)
            {
                ref IntTools_Curve aIC = ref mySeqOfCurve.ChangeValue(i);
                aIC.Curve()!.Transform(myTrsf);
            }
            // Update points
            for (int i = 1; i <= myPnts.Length(); ++i)
            {
                ref IntTools_PntOn2Faces aP2F = ref myPnts.ChangeValue(i);
                IntTools_PntOnFace aPOnF1 = aP2F.P1(), aPOnF2 = aP2F.P2();
                aPOnF1.SetPnt(aPOnF1.Pnt().Transformed(myTrsf));
                aPOnF2.SetPnt(aPOnF2.Pnt().Transformed(myTrsf));
                aP2F.SetP1(aPOnF1);
                aP2F.SetP2(aPOnF2);
            }
        }
    }
}

public partial class BOPAlgo_PaveFiller
{
    private static bool IsPlaneFF(Geom_Surface theSurface)
    {
        if (theSurface is Geom_Plane)
        {
            return true;
        }
        else if (theSurface is Geom_OffsetSurface anOffsetSurface)
        {
            return anOffsetSurface.BasisSurface() is Geom_Plane;
        }
        else if (theSurface is Geom_RectangularTrimmedSurface aTrimmedSurface)
        {
            return aTrimmedSurface.BasisSurface() is Geom_Plane;
        }

        return false;
    }

    //=================================================================================================

    private static bool IsClosedFF(in TopoDS_Edge theEdge,
                                   Geom_Surface? theSurface,
                                   Poly_Triangulation? theTriangulation,
                                   in TopLoc_Location theLocation,
                                   bool theIsPlane)
    {
        if (!theIsPlane)
        {
            // Check surface
            TopLoc_Location aLocation = theLocation.Predivided(theEdge.Location());

            // find the representation
            BRep_TEdge aTEdge = (BRep_TEdge)theEdge.TShape()!;
            for (NCollection_List<BRep_CurveRepresentation>.Iterator anIter = new(aTEdge.Curves());
                 anIter.More();
                 anIter.Next())
            {
                BRep_CurveRepresentation aCurveRepresentation = anIter.Value();
                if (aCurveRepresentation.IsCurveOnSurface(theSurface, aLocation)
                    && aCurveRepresentation.IsCurveOnClosedSurface())
                {
                    return true;
                }
            }
        }

        // Check triangulation
        return BRep_Tool.IsClosed(theEdge, theTriangulation, theLocation);
    }

    //=================================================================================================

    /// <summary>The face/face intersections: the section curves and points of each pair of faces.</summary>
    protected virtual void PerformFF(Message_ProgressRange theRange)
    {
        // Update face info for all Face/Face intersection pairs
        // and also for the rest of the faces with FaceInfo already initialized,
        // i.e. anyhow touched faces.
        myIterator!.Initialize(TopAbs_FACE, TopAbs_FACE);
        int iSize = myIterator.ExpectedLength();

        // Collect faces from intersection pairs
        NCollection_IntMap aMIFence = new();
        int nF1, nF2;
        for (; myIterator.More(); myIterator.Next())
        {
            myIterator.Value(out nF1, out nF2);
            aMIFence.Add(nF1);
            aMIFence.Add(nF2);
        }
        // Collect the rest of the touched faces
        for (int i = 0; i < myDS!.NbSourceShapes(); ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() == TopAbs_FACE && aSI.HasReference())
            {
                aMIFence.Add(i);
            }
        }
        // Update face info
        myDS.UpdateFaceInfoOn(aMIFence);
        myDS.UpdateFaceInfoIn(aMIFence);

        if (iSize == 0)
        {
            // no intersection pairs found
            return;
        }

        using var aPSOuter = new Message_ProgressScope(theRange, null, 1);

        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS.InterfFF();
        //
        // Options for the intersection algorithm
        bool bApprox = mySectionAttribute.Approximation(), bCompC2D1 = mySectionAttribute.PCurveOnS1(),
             bCompC2D2 = mySectionAttribute.PCurveOnS2();
        double anApproxTol = 1.0e-7;
        // Post-processing options
        bool bSplitCurve = false;
        //
        // Collect all pairs of Edge/Edge interferences to check if
        // some faces have to be moved to obtain more precise intersection
        NCollection_DataMap<BOPDS_Pair, NCollection_List<int>> aEEMap = new();
        NCollection_DynamicArray<BOPDS_InterfEE> aVEEs = myDS.InterfEE();
        for (int iEE = 0; iEE < aVEEs.Length(); ++iEE)
        {
            BOPDS_Interf aEE = aVEEs[iEE];
            if (!aEE.HasIndexNew())
            {
                continue;
            }
            aEE.Indices(out int nE1, out int nE2);

            int nVN = aEE.IndexNew();

            BOPDS_Pair aPair = new(nE1, nE2);
            if (aEEMap.Find(aPair, out NCollection_List<int> pPoints))
            {
                pPoints.Append(nVN);
            }
            else
            {
                pPoints = aEEMap.Bound(new BOPDS_Pair(nE1, nE2), new NCollection_List<int>());
                pPoints.Append(nVN);
            }
        }

        // Prepare the pairs of faces for intersection
        NCollection_DynamicArray<BOPAlgo_FaceFace> aVFaceFace = new();
        myIterator.Initialize(TopAbs_FACE, TopAbs_FACE);
        for (; myIterator.More(); myIterator.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            myIterator.Value(out nF1, out nF2);

            if (myGlue == BOPAlgo_GlueOff)
            {
                TopoDS_Face aF1 = TopoDS.Face(myDS.Shape(nF1));
                TopoDS_Face aF2 = TopoDS.Face(myDS.Shape(nF2));
                //
                BRepAdaptor_Surface aBAS1 = myContext!.SurfaceAdaptor(aF1);
                BRepAdaptor_Surface aBAS2 = myContext.SurfaceAdaptor(aF2);
                if (aBAS1.GetType() == GeomAbs_Plane && aBAS2.GetType() == GeomAbs_Plane)
                {
                    // Check if the planes are really interfering
                    bool bToIntersect = CheckPlanes(nF1, nF2);
                    if (!bToIntersect)
                    {
                        BOPDS_InterfFF aFF0 = aFFs.Append(new BOPDS_InterfFF());
                        aFF0.SetIndices(nF1, nF2);
                        aFF0.Init(0, 0);
                        continue;
                    }
                }

                // Check if there is an intersection between edges of the faces.
                // If there is an intersection, check if there is a shift between the edges
                // (intersection point is on some distance from the edges), and move one of
                // the faces to the point of exact edges intersection. This should allow
                // obtaining more precise intersection curves between the faces
                // (at least the curves should reach the boundary).
                // Note, that currently this check considers only closed edges (seam edges).
                TopoDS_Face aFShifted1 = aF1, aFShifted2 = aF2;
                // Keep shift value to use it as the tolerance for intersection curves
                double aShiftValue = 0.0;

                if (aBAS1.GetType() != GeomAbs_Plane || aBAS2.GetType() != GeomAbs_Plane)
                {
                    Geom_Surface aSurface1 = BRep_Tool.Surface(aF1, out TopLoc_Location aLocation1)!;
                    Poly_Triangulation? aTriangulation1 = BRep_Tool.Triangulation(aF1, out aLocation1);
                    bool anIsPlane1 = IsPlaneFF(aSurface1);

                    Geom_Surface aSurface2 = BRep_Tool.Surface(aF2, out TopLoc_Location aLocation2)!;
                    Poly_Triangulation? aTriangulation2 = BRep_Tool.Triangulation(aF2, out aLocation2);
                    bool anIsPlane2 = IsPlaneFF(aSurface2);

                    bool anIsFound = false;
                    for (TopoDS_Iterator aItW1 = new(aF1); !anIsFound && aItW1.More(); aItW1.Next())
                    {
                        for (TopoDS_Iterator aItE1 = new(aItW1.Value()); !anIsFound && aItE1.More(); aItE1.Next())
                        {
                            TopoDS_Edge anEdge1 = TopoDS.Edge(aItE1.Value());
                            int anEdgeIndex1 = myDS.Index(anEdge1);

                            for (TopoDS_Iterator aItW2 = new(aF2); !anIsFound && aItW2.More(); aItW2.Next())
                            {
                                for (TopoDS_Iterator aItE2 = new(aItW2.Value()); !anIsFound && aItE2.More(); aItE2.Next())
                                {
                                    TopoDS_Edge anEdge2 = TopoDS.Edge(aItE2.Value());
                                    int anEdgeIndex2 = myDS.Index(anEdge2);

                                    bool anIsClosed1 =
                                        IsClosedFF(anEdge1, aSurface1, aTriangulation1, aLocation1, anIsPlane1);
                                    bool anIsClosed2 =
                                        IsClosedFF(anEdge2, aSurface2, aTriangulation2, aLocation2, anIsPlane2);
                                    if (!anIsClosed1 && !anIsClosed2)
                                    {
                                        continue;
                                    }

                                    if (!aEEMap.Find(new BOPDS_Pair(anEdgeIndex1, anEdgeIndex2), out NCollection_List<int> aVertexIndices))
                                    {
                                        continue;
                                    }

                                    for (NCollection_List<int>.Iterator itEEP = new(aVertexIndices); itEEP.More();
                                         itEEP.Next())
                                    {
                                        int aVertexIndex = itEEP.Value();
                                        TopoDS_Vertex aVertex = TopoDS.Vertex(myDS.Shape(aVertexIndex));
                                        gp_Pnt aVertexPoint = BRep_Tool.Pnt(aVertex);

                                        // Compute points exactly on the edges
                                        GeomAPI_ProjectPointOnCurve aProjPC1 = myContext.ProjPC(anEdge1);
                                        GeomAPI_ProjectPointOnCurve aProjPC2 = myContext.ProjPC(anEdge2);
                                        aProjPC1.Perform(aVertexPoint);
                                        aProjPC2.Perform(aVertexPoint);
                                        if (aProjPC1.NbPoints() == 0 && aProjPC2.NbPoints() == 0)
                                        {
                                            continue;
                                        }
                                        gp_Pnt aP1 =
                                            aProjPC1.NbPoints() > 0 ? aProjPC1.NearestPoint() : aVertexPoint;
                                        gp_Pnt aP2 =
                                            aProjPC2.NbPoints() > 0 ? aProjPC2.NearestPoint() : aVertexPoint;

                                        double aShiftDist = aP1.Distance(aP2);
                                        if (aShiftDist > BRep_Tool.Tolerance(aVertex))
                                        {
                                            // Move one of the faces to the point of exact intersection of edges
                                            gp_Trsf aTrsf = new();
                                            aTrsf.SetTranslation(anIsClosed1 ? new gp_Vec(aP1, aP2) : new gp_Vec(aP2, aP1));
                                            TopLoc_Location aLoc = new(aTrsf);
                                            if (anIsClosed1)
                                            {
                                                aFShifted1.Move(aLoc);
                                            }
                                            else
                                            {
                                                aFShifted2.Move(aLoc);
                                            }
                                            aShiftValue = aShiftDist;
                                            anIsFound = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                //
                BOPAlgo_FaceFace aFaceFace = aVFaceFace.Append(new BOPAlgo_FaceFace());
                //
                aFaceFace.SetRunParallel(myRunParallel);
                aFaceFace.SetIndices(nF1, nF2);
                aFaceFace.SetFaces(aFShifted1, aFShifted2);
                aFaceFace.SetBoxes(myDS.ShapeInfo(nF1).Box(), myDS.ShapeInfo(nF2).Box());
                // Note: in case of faces with closed edges it should not be less than value of the shift
                double aTolFF = Max(aShiftValue, ToleranceFF(aBAS1, aBAS2));
                aFaceFace.SetTolFF(aTolFF);
                //
                NCollection_List<IntSurf_PntOn2S> aListOfPnts = new();
                GetEFPnts(nF1, nF2, aListOfPnts);
                int aNbLP = aListOfPnts.Extent();
                if (aNbLP != 0)
                {
                    aFaceFace.SetList(aListOfPnts);
                }
                //
                aFaceFace.SetParameters(bApprox, bCompC2D1, bCompC2D2, anApproxTol);
                aFaceFace.SetFuzzyValue(myFuzzyValue);
            }
            else
            {
                // for the Glue mode just add all interferences of that type
                BOPDS_InterfFF aFF0 = aFFs.Append(new BOPDS_InterfFF());
                aFF0.SetIndices(nF1, nF2);
                aFF0.SetTangentFaces(false);
                aFF0.Init(0, 0);
            }
        } // for (; myIterator->More(); myIterator->Next()) {
        //
        int k, aNbFaceFace = aVFaceFace.Length();
        using var aPS = new Message_ProgressScope(aPSOuter.Next(), "Performing Face-Face intersection", aNbFaceFace);
        for (k = 0; k < aNbFaceFace; k++)
        {
            BOPAlgo_FaceFace aFaceFace = aVFaceFace[k];
            aFaceFace.SetProgressRange(aPS.Next());
        }
        //======================================================
        // Perform intersection
        BOPTools_Parallel.Perform(myRunParallel, aVFaceFace);
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //======================================================
        // Treatment of the results

        for (k = 0; k < aNbFaceFace; ++k)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_FaceFace aFaceFace = aVFaceFace[k];
            aFaceFace.Indices(out nF1, out nF2);
            if (!aFaceFace.IsDone() || aFaceFace.HasErrors())
            {
                BOPDS_InterfFF aFF0 = aFFs.Append(new BOPDS_InterfFF());
                aFF0.SetIndices(nF1, nF2);
                aFF0.Init(0, 0);
                // Warn about failed intersection of faces
                AddIntersectionFailedWarning(aFaceFace.Face1(), aFaceFace.Face2());
                continue;
            }
            //
            bool bTangentFaces = aFaceFace.TangentFaces();
            double aTolFF = aFaceFace.TolFF();
            //
            aFaceFace.PrepareLines3D(bSplitCurve);
            //
            aFaceFace.ApplyTrsf();
            //
            // The curves are kept unfrozen in the data structure: InternalLines() (Lines() copies and freezes).
            NCollection_Sequence<IntTools_Curve> aCvsX = aFaceFace.InternalLines();
            NCollection_Sequence<IntTools_PntOn2Faces> aPntsX = aFaceFace.Points();
            //
            int aNbCurves = aCvsX.Length();
            int aNbPoints = aPntsX.Length();
            //
            if (aNbCurves != 0 || aNbPoints != 0)
            {
                myDS.AddInterf(nF1, nF2);
            }
            //
            BOPDS_InterfFF aFF = aFFs.Append(new BOPDS_InterfFF());
            aFF.SetIndices(nF1, nF2);
            aFF.SetTangentFaces(bTangentFaces);
            aFF.Init(aNbCurves, aNbPoints);
            //
            // Curves
            // Fix bounding box expanding coefficient.
            double aBoxExpandValue = aTolFF;
            if (aNbCurves > 0)
            {
                // Modify geometric expanding coefficient by topology value,
                // since this bounding box used in sharing (vertex or edge).
                double aMaxVertexTol = Max(BRep_Tool.MaxTolerance(aFaceFace.Face1(), TopAbs_VERTEX),
                                           BRep_Tool.MaxTolerance(aFaceFace.Face2(), TopAbs_VERTEX));
                aBoxExpandValue += aMaxVertexTol;
            }
            //
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
            for (int i = 1; i <= aNbCurves; ++i)
            {
                if (UserBreak(aPSOuter))
                {
                    return;
                }
                Bnd_Box aBox = new();
                IntTools_Curve aIC = aCvsX[i];
                bool bIsValid = IntTools_Tools.CheckCurve(aIC, ref aBox);
                if (bIsValid)
                {
                    BOPDS_Curve aNC = aVNC.Append(new BOPDS_Curve());
                    aNC.SetCurve(aIC);
                    // make sure that the bounding box has the maximal gap
                    aBox.Enlarge(aBoxExpandValue);
                    aNC.SetBox(aBox);
                    aNC.SetTolerance(Max(aIC.Tolerance(), aTolFF));
                }
            }
            //
            // Points
            NCollection_DynamicArray<BOPDS_Point> aVNP = aFF.ChangePoints();
            for (int i = 1; i <= aNbPoints; ++i)
            {
                IntTools_PntOn2Faces aPi = aPntsX[i];
                gp_Pnt aP = aPi.P1().Pnt();
                //
                ref BOPDS_Point aNP = ref aVNP.Append(new BOPDS_Point());
                aNP.SetPnt(aP);
            }
        }
    }

    //=======================================================================
    // function : UpdateSavedTolerance
    // purpose  : Updates the saved tolerance of the vertices of the edge
    //           with new tolerance of edge
    //=======================================================================
    private static void UpdateSavedTolerance(BOPDS_DS theDS,
                                             int theNE,
                                             double theTolNew,
                                             NCollection_IntDataMap<double> theMVTol)
    {
        NCollection_List<int> aSubShapes = theDS.ShapeInfo(theNE).SubShapes();
        NCollection_List<int>.Iterator itSS = new(aSubShapes);
        for (; itSS.More(); itSS.Next())
        {
            int nV = itSS.Value();
            if (theMVTol.IsBound(nV))
            {
                ref double pTolSaved = ref theMVTol.ChangeFind(nV);
                if (pTolSaved < theTolNew)
                {
                    pTolSaved = theTolNew;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Builds the section edges from the face/face interferences, with their post-treatment.</summary>
    protected void MakeBlocks(Message_ProgressRange theRange)
    {
        using var aPSOuter = new Message_ProgressScope(theRange, null, 4);
        if (myGlue != BOPAlgo_GlueOff)
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFF = aFFs.Length();
        using var aPS = new Message_ProgressScope(aPSOuter.Next(), "Building section edges", aNbFF);
        if (aNbFF == 0)
        {
            return;
        }
        //
        bool bExist, bValid2D;
        int i, nF1, nF2, aNbC, aNbP, j;
        int nV1, nV2;
        double aT1, aT2;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new();
        TopoDS_Edge aES;
        BOPDS_PaveBlock? aPBOut = null;
        //
        //-----------------------------------------------------scope f
        //
        // Per-iteration collections (OCCT: a temporary allocator, cleared and reset each iteration).
        NCollection_List<int> aLSE = new();
        NCollection_List<int> aLBV = new();
        NCollection_IntMap aMVOnIn = new(100);
        NCollection_IntMap aMVCommon = new(100);
        NCollection_IntMap aMVStick = new(100);
        NCollection_IntMap aMVEF = new(100);
        NCollection_IntMap aMVBounds = new(100);
        NCollection_IntMap aMI = new(100);
        NCollection_IndexedMap<BOPDS_PaveBlock> aMPBOnIn = new();
        NCollection_Map<BOPDS_PaveBlock> aMPBCommon = new();
        NCollection_IntDataMap<NCollection_List<int>> aDMBV = new(100);
        NCollection_IntDataMap<double> aMVTol = new(100);
        //
        // Cross-iteration collections (persist through entire loop)
        NCollection_Map<BOPDS_PaveBlock> aMPBAdd = new();
        NCollection_List<BOPDS_PaveBlock> aLPB = new();
        NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMSCPB = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, int> aMVI = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> aDMExEdges = new();
        NCollection_IntDataMap<int> aDMNewSD = new(100);
        NCollection_IntDataMap<NCollection_List<int>> aDMVLV = new(100);
        NCollection_IndexedMap<BOPDS_PaveBlock> aMicroPB = new();
        NCollection_IndexedMap<TopoDS_Shape> aVertsOnRejectedPB = new(TopTools_ShapeMapHasher.Instance);
        // Map of PaveBlocks with the faces to which it has to be added
        NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> aPBFacesMap = new();
        //
        // The vector aFFToRecheck contains indices of potentially problematic Face-Face intersections
        NCollection_DynamicArray<int> aFFToRecheck = new();
        // aNbFF may be increased while processing this loop, because it is necessary to recheck
        // some of Face-Face intersections to avoid missing section edges
        // aNbFF will be increased to the number of potentially problematic Face-Face intersections
        int aNbFFPrev = aNbFF;
        for (i = 0; i < aNbFF; ++i, aPS.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            // after passing through all of Face-Face intersections it is necessary to return
            // to potentially problematic Face-Face intersections and process them one more time
            int aCurInd = i < aNbFFPrev ? i : aFFToRecheck[i - aNbFFPrev];
            //
            BOPDS_InterfFF aFF = aFFs[aCurInd];
            aFF.Indices(out nF1, out nF2);
            //
            NCollection_DynamicArray<BOPDS_Point> aVP = aFF.ChangePoints();
            aNbP = aVP.Length();
            NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.ChangeCurves();
            aNbC = aVC.Length();
            if (aNbP == 0 && aNbC == 0)
            {
                continue;
            }
            //
            TopoDS_Face aF1 = TopoDS.Face(myDS.Shape(nF1));
            TopoDS_Face aF2 = TopoDS.Face(myDS.Shape(nF2));
            //
            double aTolFF = Max(BRep_Tool.Tolerance(aF1), BRep_Tool.Tolerance(aF2));
            //
            BOPDS_FaceInfo aFI1 = myDS.ChangeFaceInfo(nF1);
            BOPDS_FaceInfo aFI2 = myDS.ChangeFaceInfo(nF2);
            //
            // Clear per-iteration collections
            aMVOnIn.Clear();
            aMVCommon.Clear();
            aMPBOnIn.Clear();
            aMPBCommon.Clear();
            aDMBV.Clear();
            aMVTol.Clear();
            aLSE.Clear();
            aLBV.Clear();
            aMVStick.Clear();
            aMVEF.Clear();
            aMVBounds.Clear();
            aMI.Clear();
            //
            myDS.SubShapesOnIn(nF1, nF2, aMVOnIn, aMVCommon, aMPBOnIn, aMPBCommon);
            myDS.SharedEdges(nF1, nF2, aLSE);
            //
            // 1. Treat Points
            for (j = 0; j < aNbP; ++j)
            {
                TopoDS_Vertex aV;
                BOPDS_CoupleOfPaveBlocks aCPB = new();
                //
                ref BOPDS_Point aNP = ref aVP.ChangeValue(j);
                gp_Pnt aP = aNP.Pnt();
                //
                bExist = IsExistingVertex(aP, aTolFF, aMVOnIn);
                if (!bExist)
                {
                    BOPTools_AlgoTools.MakeNewVertex(aP, aTolFF, out aV);
                    //
                    aCPB.SetIndexInterf(i);
                    aCPB.SetIndex(j);
                    aMSCPB.Add(aV, aCPB);
                }
            }
            //
            // 2. Treat Curves
            GetStickVertices(nF1, nF2, aMVStick, aMVEF, aMI);
            //
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVC[j];
                // DEBt
                aNC.InitPaveBlock1();
                //
                // In order to avoid problems connected with
                // extending tolerance of vertex while putting
                // (e.g. see "bugs modalg_6 bug26789_1" test case),
                // all not-common vertices will be checked by
                // BndBoxes before putting. For common-vertices,
                // filtering by BndBoxes is not necessary.
                PutPavesOnCurve(aMVOnIn, aMVCommon, aNC, aMI, aMVEF, aMVTol, aDMVLV);
            }

            // if some E-F vertex was put on a curve due to large E-F intersection range,
            // and it also was put on another curve correctly then remove this vertex from
            // the first curve. Detect such case if the distance to curve exceeds aTolR3D.
            FilterPavesOnCurves(aVC, aMVTol);

            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVC[j];
                IntTools_Curve aIC = aNC.Curve();
                //
                PutStickPavesOnCurve(aF1, aF2, aMI, aVC, j, aMVStick, aMVTol, aDMVLV);
                // 904/F7
                if (aNbC == 1)
                {
                    PutEFPavesOnCurve(aVC, j, aMI, aMVEF, aMVTol, aDMVLV);
                }
                //
                if (aIC.HasBounds())
                {
                    aLBV.Clear();
                    //
                    PutBoundPaveOnCurve(aF1, aF2, aNC, aLBV);
                    //
                    if (!aLBV.IsEmpty())
                    {
                        // OCCT binds a copy of the list (aLBV is cleared and reused).
                        aDMBV.Bind(j, new NCollection_List<int>(aLBV));
                        NCollection_List<int>.Iterator aItI = new(aLBV);
                        for (; aItI.More(); aItI.Next())
                        {
                            aMVBounds.Add(aItI.Value());
                        }
                    }
                }
            } // for (j=0; j<aNbC; ++j) {

            // Put closing pave if needed
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVC[j];
                PutClosingPaveOnCurve(aNC);
            }
            //

            BOPTools_BoxTree aPBTree = new();
            {
                // Fill the tree with boxes of pave blocks ON/IN
                // Tree will be build on first selection from the tree.
                int aNbPB = aMPBOnIn.Extent();
                aPBTree.SetSize(aNbPB);
                for (int iPB = 1; iPB <= aNbPB; ++iPB)
                {
                    BOPDS_PaveBlock aPB = aMPBOnIn[iPB];
                    if (!aPB.HasEdge())
                    {
                        continue;
                    }

                    if (myDS.ShapeInfo(aPB.OriginalEdge()).HasFlag())
                    {
                        continue;
                    }

                    aPBTree.Add(iPB, Bnd_Tools.Bnd2BVH(myDS.ShapeInfo(aPB.Edge()).Box()));
                }
            }

            // Added additional check of Face-Face intersection to avoid missing section edges
            // because of sequence of Face-Face interference processing
            bool isToRecheck = aNbC > 0 && i < aNbFFPrev;
            //
            // 3. Make section edges
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVC[j];
                IntTools_Curve aIC = aNC.Curve();
                double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
                //
                NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.ChangePaveBlocks();
                BOPDS_PaveBlock aPB1 = aNC.ChangePaveBlock1();
                //
                aLPB.Clear();
                aPB1.Update(aLPB, false);
                //
                if (aLPB.Extent() != 0)
                {
                    isToRecheck = false;
                }

                aItLPB.Initialize(aLPB);
                for (; aItLPB.More(); aItLPB.Next())
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    aPB.Indices(out nV1, out nV2);
                    aPB.Range(out aT1, out aT2);
                    //
                    if (Math.Abs(aT1 - aT2) < Precision.PConfusion())
                    {
                        continue;
                    }

                    // Check validity of the block for the faces:
                    // classify bounding and middle points on the curve
                    // relatively both faces
                    bValid2D = myContext!.IsValidBlockForFaces(aT1, aT2, aIC, aF1, aF2, aTolR3D);
                    if (!bValid2D)
                    {
                        continue;
                    }
                    //
                    int nEOut = 0; // OCCT: uninitialised, read only when IsExistingPaveBlock writes it
                    double aTolNew = -1.0;
                    bExist = IsExistingPaveBlock(aPB, aNC, aLSE, ref nEOut, ref aTolNew);
                    if (bExist)
                    {
                        // Update edge with new tolerance
                        UpdateEdgeTolerance(nEOut, aTolNew);
                        // Update aMVTol map with new tolerances of vertices
                        UpdateSavedTolerance(myDS, nEOut, aTolNew, aMVTol);
                        continue;
                    }
                    //
                    TopoDS_Vertex aV1 = TopoDS.Vertex(myDS.Shape(nV1));
                    TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(nV2));
                    //
                    // check if the pave block has a valid range
                    double aFirst = 0.0, aLast = 0.0;
                    if (!BRepLib.FindValidRange(new GeomAdaptor_Curve(aIC.Curve()!),
                                                aTolR3D,
                                                aT1,
                                                BRep_Tool.Pnt(aV1),
                                                Max(aTolR3D, BRep_Tool.Tolerance(aV1)),
                                                aT2,
                                                BRep_Tool.Pnt(aV2),
                                                Max(aTolR3D, BRep_Tool.Tolerance(aV2)),
                                                ref aFirst,
                                                ref aLast))
                    {
                        // If the pave block does not have valid range, i.e. it is completely
                        // covered by the tolerance spheres of its vertices, it will be
                        // passed into post treatment process to fuse its vertices.
                        // The pave block itself will not be kept.
                        if (!aMVBounds.Contains(nV1) && !aMVBounds.Contains(nV2))
                        {
                            aMicroPB.Add(aPB);
                            // keep vertices for post treatment
                            aMVI.Bind(aV1, nV1);
                            aMVI.Bind(aV2, nV2);
                        }
                        continue;
                    }
                    //
                    bExist =
                        IsExistingPaveBlock(aPB, aNC, aTolR3D, aMPBOnIn, aPBTree, aMPBCommon, ref aPBOut, ref aTolNew);
                    if (bExist)
                    {
                        bool bInF1 =
                            (aFI1.PaveBlocksOn().Contains(aPBOut!) || aFI1.PaveBlocksIn().Contains(aPBOut!));
                        bool bInF2 =
                            (aFI2.PaveBlocksOn().Contains(aPBOut!) || aFI2.PaveBlocksIn().Contains(aPBOut!));
                        if (!bInF1 || !bInF2)
                        {
                            // Update edge to touch both faces
                            int nE = aPBOut!.Edge();
                            TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                            double aTolE = BRep_Tool.Tolerance(aE);
                            if (aTolNew < aNC.Tolerance())
                            {
                                aTolNew = aNC.Tolerance(); // use real tolerance of intersection
                            }
                            if (aTolNew > aTolE)
                            {
                                UpdateEdgeTolerance(nE, aTolNew);
                                // Update aMVTol map with new tolerances of vertices
                                UpdateSavedTolerance(myDS, nE, aTolNew, aMVTol);
                            }

                            // Face without pave block
                            int nF = bInF1 ? nF2 : nF1;
                            if (!aPBFacesMap.Find(aPBOut, out NCollection_List<int> pFaces))
                            {
                                pFaces = aPBFacesMap.Bound(aPBOut, new NCollection_List<int>());
                            }
                            // List is expected to be short, so we allow the check here
                            if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                            {
                                pFaces.Append(nF);
                            }

                            // Try fusing the vertices of the existing pave block
                            // with the vertices put on the real section curve (except
                            // for technological vertices, which will be removed)
                            aPBOut.Indices(out int nVOut1, out int nVOut2);
                            if (nV1 != nVOut1 && nV1 != nVOut2 && !aMVBounds.Contains(nV1))
                            {
                                aVertsOnRejectedPB.Add(aV1);
                            }
                            if (nV2 != nVOut1 && nV2 != nVOut2 && !aMVBounds.Contains(nV2))
                            {
                                aVertsOnRejectedPB.Add(aV2);
                            }

                            if (aMPBAdd.Add(aPBOut))
                            {
                                // Add edge for processing as the section edge
                                PreparePostTreatFF(aCurInd, j, aPBOut, aMSCPB, aMVI, aLPBC);
                            }
                        }
                        continue;
                    }
                    //
                    // Make Edge
                    BOPTools_AlgoTools.MakeEdge(aIC, aV1, aT1, aV2, aT2, aTolR3D, out aES);
                    // Make p-curves
                    BOPTools_AlgoTools.MakePCurve(aES,
                                                  aF1,
                                                  aF2,
                                                  aIC,
                                                  mySectionAttribute.PCurveOnS1(),
                                                  mySectionAttribute.PCurveOnS2(),
                                                  myContext);
                    //
                    // Append the Pave Block to the Curve j
                    aLPBC.Append(aPB);
                    //
                    // Keep info for post treatment
                    BOPDS_CoupleOfPaveBlocks aCPB = new();
                    aCPB.SetIndexInterf(aCurInd);
                    aCPB.SetIndex(j);
                    aCPB.SetPaveBlock1(aPB);
                    //
                    aMSCPB.Add(aES, aCPB);
                    aMVI.Bind(aV1, nV1);
                    aMVI.Bind(aV2, nV2);
                    //
                    aMVTol.UnBind(nV1);
                    aMVTol.UnBind(nV2);

                    // Add existing pave blocks for post treatment
                    ProcessExistingPaveBlocks(aCurInd,
                                              j,
                                              nF1,
                                              nF2,
                                              aES,
                                              aMPBOnIn,
                                              aPBTree,
                                              aMSCPB,
                                              aMVI,
                                              aLPBC,
                                              aPBFacesMap,
                                              aMPBAdd);
                }
                //
                aLPBC.RemoveFirst();
            } // for (j=0; j<aNbC; ++j) {
            if (isToRecheck)
            {
                aFFToRecheck.Append(aCurInd);
                ++aNbFF;
            }
            //
            // back to previous tolerance values for unused vertices
            // and forget about SD groups of such vertices
            foreach (KeyValuePair<int, double> aItMV in aMVTol)
            {
                nV1 = aItMV.Key;
                double aTol = aItMV.Value;
                //
                TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV1));
                BRep_TVertex TV = (BRep_TVertex)aV.TShape()!;
                // OCCT.NET: frozen. OCCT writes the saved tolerance back even into a vertex it never raised (a
                // vertex of both faces put on the curve as an existing pave): the port skips writing an unchanged
                // value, so an argument vertex shared by several arguments (faces given separately to the volume
                // maker or the cells builder) is not written. A raised argument vertex still throws.
                if (TV.Tolerance() != aTol)
                {
                    TV.Tolerance(aTol);
                }
                // reset bnd box
                BOPDS_ShapeInfo aSIDS = myDS.ChangeShapeInfo(nV1);
                ref Bnd_Box aBoxDS = ref aSIDS.ChangeBox();
                aBoxDS = new Bnd_Box();
                BRepBndLib.Add(aV, ref aBoxDS);
                aBoxDS.SetGap(aBoxDS.GetGap() + Precision.Confusion());
                //
                if (aDMVLV.IsBound(nV1))
                {
                    aDMVLV.UnBind(nV1);
                }
            }
            //
            ProcessExistingPaveBlocks(aCurInd,
                                      nF1,
                                      nF2,
                                      aMPBOnIn,
                                      aPBTree,
                                      aDMBV,
                                      aMSCPB,
                                      aMVI,
                                      aPBFacesMap,
                                      aMPBAdd);
        } // for (i=0; i<aNbFF; ++i) {

        // Remove "micro" section edges
        RemoveMicroSectionEdges(aMSCPB, aMicroPB);

        // post treatment
        MakeSDVerticesFF(aDMVLV, aDMNewSD);
        PostTreatFF(aMSCPB,
                    aDMExEdges,
                    aDMNewSD,
                    aMicroPB,
                    aVertsOnRejectedPB,
                    aPSOuter.Next(2));
        if (HasErrors())
        {
            return;
        }
        // reduce tolerances of section edges where it is appropriate
        CorrectToleranceOfSE();
        //
        // update face info
        UpdateFaceInfo(aDMExEdges, aDMNewSD, aPBFacesMap);
        // Update all pave blocks
        UpdatePaveBlocks(aDMNewSD);
        //
        // Treat possible common zones by trying to put each section edge
        // into all faces, not participated in creation of that edge, as IN edge

        PutSEInOtherFaces(aPSOuter.Next());
    }

    //=================================================================================================

    /// <summary>Makes a same-domain vertex for each group of coinciding vertices.</summary>
    protected void MakeSDVerticesFF(NCollection_IntDataMap<NCollection_List<int>> theDMVLV,
                                    NCollection_IntDataMap<int> theDMNewSD)
    {
        // Create a new SD vertex for each group of coinciding vertices
        // and put new substitutions to theDMNewSD.
        // The groups are visited in the map's order: the new vertices' indices depend on it (decision 784).
        foreach (KeyValuePair<int, NCollection_List<int>> aItG in theDMVLV)
        {
            NCollection_List<int> aList = aItG.Value;
            // make SD vertices w/o creation of interfs
            int nSD = MakeSDVertices(aList, false);
            // update theDMNewSD
            NCollection_List<int>.Iterator aItL = new(aList);
            for (; aItL.More(); aItL.Next())
            {
                int nV = aItL.Value();
                theDMNewSD.Bind(nV, nSD);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// The post-treatment of the section edges and vertices: they are intersected among themselves (and with the
    /// existing edges and vertices involved) by a nested pave filler, and the results are put into the data structure.
    /// </summary>
    protected void PostTreatFF(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB,
                               NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> aDMExEdges,
                               NCollection_IntDataMap<int> aDMNewSD,
                               NCollection_IndexedMap<BOPDS_PaveBlock> theMicroPB,
                               NCollection_IndexedMap<TopoDS_Shape> theVertsOnRejectedPB,
                               Message_ProgressRange theRange)
    {
        int aNbS = theMSCPB.Extent();
        if (aNbS == 0)
        {
            return;
        }
        //
        bool bHasPaveBlocks, bOld;
        int nSx, nVSD = 0, iX, iP, iC, j, nV, iV = 0, iE, k;
        int aNbLPBx;
        TopAbs_ShapeEnum aType;
        TopoDS_Shape aV, aE;
        NCollection_List<TopoDS_Shape>.Iterator aItLS = new();
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new();
        BOPDS_DS aPDS;
        BOPDS_PaveBlock aPB1;
        Span<BOPDS_Pave> aPave = stackalloc BOPDS_Pave[2];
        BOPDS_ShapeInfo aSI = new();
        //
        NCollection_List<TopoDS_Shape> aLS = new();
        BOPAlgo_PaveFiller aPF = new();
        aPF.SetIsPrimary(false);
        aPF.SetNonDestructive(myNonDestructive);
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFF = aFFs.Length();
        //

        // Find unused vertices
        NCollection_IndexedMap<TopoDS_Shape> VertsUnused = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IntMap IndMap = new();
        for (int i = 0; i < aNbFF; i++)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            aFF.Indices(out int nF1, out int nF2);

            NCollection_IntMap aMV = new(), aMVEF = new(), aMI = new();
            GetStickVertices(nF1, nF2, aMV, aMVEF, aMI);
            NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.ChangeCurves();
            RemoveUsedVertices(aVC, aMV);

            // The vertices are visited in the map's order, which orders VertsUnused and so the nested filler's
            // arguments (decision 784).
            foreach (int indV in aMV)
            {
                TopoDS_Shape aVertex = myDS.Shape(indV);
                if (IndMap.Add(indV))
                {
                    VertsUnused.Add(aVertex);
                }
                else
                {
                    VertsUnused.RemoveKey(aVertex);
                }
            }
        }
        /////////////////////

        int aNbME = theMicroPB.Extent();
        int aNbVOnRPB = theVertsOnRejectedPB.Extent();
        // 0
        if (aNbS == 1 && (aNbME == 0) && (aNbVOnRPB == 0) && VertsUnused.IsEmpty())
        {
            TopoDS_Shape aS = theMSCPB.FindKey(1);
            BOPDS_CoupleOfPaveBlocks aCPB = theMSCPB.FindFromIndex(1);
            //
            aType = aS.ShapeType();
            if (aType == TopAbs_VERTEX)
            {
                aSI.SetShapeType(aType);
                aSI.SetShape(aS);
                iV = myDS.Append(aSI);
                //
                iX = aCPB.IndexInterf();
                iP = aCPB.Index();
                BOPDS_InterfFF aFF = aFFs[iX];
                NCollection_DynamicArray<BOPDS_Point> aVNP = aFF.ChangePoints();
                ref BOPDS_Point aNP = ref aVNP[iP];
                aNP.SetIndex(iV);
            }
            else if (aType == TopAbs_EDGE)
            {
                aPB1 = aCPB.PaveBlock1()!;
                //
                if (aPB1.HasEdge())
                {
                    NCollection_List<BOPDS_PaveBlock> aLPBx = new();
                    aLPBx.Append(aPB1);
                    aDMExEdges.Bind(aPB1, aLPBx);
                }
                else
                {
                    aSI.SetShapeType(aType);
                    aSI.SetShape(aS);
                    iE = myDS.Append(aSI);
                    //
                    aPB1.SetEdge(iE);
                }
            }
            return;
        }
        //
        // 1 prepare arguments
        // Avoid intersection of existing edges among themselves
        TopoDS_Compound anExistingEdges = new();
        new BRep_Builder().MakeCompound(ref anExistingEdges);
        NCollection_Map<TopoDS_Shape> anAddedSD = new(TopTools_ShapeMapHasher.Instance);
        for (k = aNbS; k > 0; --k)
        {
            TopoDS_Shape aS = theMSCPB.FindKey(k);
            BOPDS_PaveBlock? aPB = theMSCPB[k].PaveBlock1();
            if (aPB is not null && aPB.HasEdge())
            {
                new BRep_Builder().Add(anExistingEdges, aS);
            }
            else
            {
                aLS.Append(aS);
            }
            // add vertices-candidates for SD from the map aDMNewSD,
            // so that they took part in fuse operation.
            TopoDS_Iterator itV = new(aS);
            for (; itV.More(); itV.Next())
            {
                TopoDS_Shape aVer = itV.Value();
                int iVer = myDS.Index(aVer);
                if (aDMNewSD.Find(iVer, out int pSD))
                {
                    TopoDS_Shape aVSD = myDS.Shape(pSD);
                    if (anAddedSD.Add(aVSD))
                    {
                        aLS.Append(aVSD);
                    }
                }
            }
        }
        if (anExistingEdges.NbChildren() > 0)
        {
            aLS.Append(anExistingEdges);
        }
        //
        // The section edges considered as a micro should be
        // specially treated - their vertices should be united and
        // the edge itself should be removed. Thus, we add only
        // its vertices into operation.
        //
        BRep_Builder aBB = new();
        Span<int> nVerts = stackalloc int[2];
        TopoDS_Vertex[] aVerts = new TopoDS_Vertex[2];
        for (k = 1; k <= aNbME; ++k)
        {
            theMicroPB[k].Indices(out nVerts[0], out nVerts[1]);
            for (int i = 0; i < 2; ++i)
            {
                bool isSD = aDMNewSD.Find(nVerts[i], out int pSD);
                aVerts[i] = TopoDS.Vertex(myDS.Shape(isSD ? pSD : nVerts[i]));
                if (anAddedSD.Add(aVerts[i]))
                {
                    aLS.Append(aVerts[i]);
                }
            }
            //
            if (aVerts[0].IsSame(aVerts[1]))
            {
                continue;
            }
            //
            // make sure these vertices will be united
            gp_Pnt aP1 = BRep_Tool.Pnt(aVerts[0]);
            gp_Pnt aP2 = BRep_Tool.Pnt(aVerts[1]);
            //
            double aDist = aP1.Distance(aP2);
            double aTolV1 = BRep_Tool.Tolerance(aVerts[0]);
            double aTolV2 = BRep_Tool.Tolerance(aVerts[1]);
            //
            aDist -= (aTolV1 + aTolV2);
            if (aDist > 0.0)
            {
                aDist /= 2.0;
                aBB.UpdateVertex(aVerts[0], aTolV1 + aDist);
                aBB.UpdateVertex(aVerts[1], aTolV2 + aDist);
            }
        }

        // Add vertices put on the real section curves to unify them with the
        // vertices of the edges, by which these sections curves have been rejected
        // and with unused vertices
        NCollection_IndexedMap<TopoDS_Shape>[] VerMap = [theVertsOnRejectedPB, VertsUnused];
        for (int imap = 0; imap < 2; imap++)
        {
            int NbVer = VerMap[imap].Extent();
            for (int i = 1; i <= NbVer; ++i)
            {
                TopoDS_Shape aVer = VerMap[imap].FindKey(i);
                int iVer = myDS.Index(aVer);
                if (aDMNewSD.Find(iVer, out int pSD))
                {
                    aVer = myDS.Shape(pSD);
                }

                if (anAddedSD.Add(aVer))
                {
                    aLS.Append(aVer);
                }
            }
        }
        //
        using var aPS = new Message_ProgressScope(theRange, "Intersection of section edges", 1);

        // 2 Fuse shapes
        aPF.SetRunParallel(myRunParallel);
        aPF.SetArguments(aLS);
        aPF.Perform(aPS.Next());
        if (aPF.HasErrors())
        {
            AddError(new BOPAlgo_AlertPostTreatFF());
            return;
        }
        aPDS = aPF.PDS()!;
        //
        // Map to store the real tolerance of the common block
        // and avoid repeated computation of it
        NCollection_DataMap<BOPDS_CommonBlock, double> aMCBTol = new();
        // Map to avoid creation of different pave blocks for
        // the same intersection edge
        NCollection_IntDataMap<BOPDS_PaveBlock> aMEPB = new();
        //
        aItLS.Initialize(aLS);
        for (; aItLS.More(); aItLS.Next())
        {
            TopoDS_Shape aSx = aItLS.Value();
            if (aSx.ShapeType() == TopAbs_COMPOUND)
            {
                for (TopoDS_Iterator itC = new(aSx); itC.More(); itC.Next())
                {
                    aLS.Append(itC.Value());
                }
                continue;
            }
            nSx = aPDS.Index(aSx);
            BOPDS_ShapeInfo aSIx = aPDS.ShapeInfo(nSx);
            //
            aType = aSIx.ShapeType();
            //
            if (aType == TopAbs_VERTEX)
            {
                bool bIntersectionPoint = theMSCPB.Contains(aSx);
                //
                if (aPDS.HasShapeSD(nSx, ref nVSD))
                {
                    aV = aPDS.Shape(nVSD);
                }
                else
                {
                    aV = aSx;
                }
                // index of new vertex in theDS -> iV
                iV = myDS.Index(aV);
                if (iV < 0)
                {
                    aSI.SetShapeType(aType);
                    aSI.SetShape(aV);
                    iV = myDS.Append(aSI);
                }
                //
                if (!bIntersectionPoint)
                {
                    // save SD connection
                    nSx = myDS.Index(aSx);
                    if (nSx != iV)
                    {
                        aDMNewSD.Bind(nSx, iV);
                        myDS.AddShapeSD(nSx, iV);
                    }
                }
                else
                {
                    // update FF interference
                    BOPDS_CoupleOfPaveBlocks aCPB = theMSCPB.FindFromKey(aSx);
                    iX = aCPB.IndexInterf();
                    iP = aCPB.Index();
                    BOPDS_InterfFF aFF = aFFs[iX];
                    NCollection_DynamicArray<BOPDS_Point> aVNP = aFF.ChangePoints();
                    ref BOPDS_Point aNP = ref aVNP[iP];
                    aNP.SetIndex(iV);
                }
            } // if (aType==TopAbs_VERTEX) {
            //
            else if (aType == TopAbs_EDGE)
            {
                bHasPaveBlocks = aPDS.HasPaveBlocks(nSx);
                BOPDS_CoupleOfPaveBlocks aCPB = theMSCPB.FindFromKey(aSx);
                iX = aCPB.IndexInterf();
                iC = aCPB.Index();
                aPB1 = aCPB.PaveBlock1()!;
                //
                bOld = aPB1.HasEdge();
                if (bOld)
                {
                    NCollection_List<BOPDS_PaveBlock> aLPBx = new();
                    aDMExEdges.Bind(aPB1, aLPBx);
                }
                //
                if (!bHasPaveBlocks)
                {
                    if (bOld)
                    {
                        aDMExEdges.ChangeFind(aPB1).Append(aPB1);
                    }
                    else
                    {
                        aSI.SetShapeType(aType);
                        aSI.SetShape(aSx);
                        iE = myDS.Append(aSI);
                        //
                        aPB1.SetEdge(iE);
                    }
                }
                else
                {
                    BOPDS_InterfFF aFF = aFFs[iX];
                    NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
                    BOPDS_Curve aNC = aVNC[iC];
                    NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.ChangePaveBlocks();
                    //
                    // check if edge occurred to be micro edge;
                    // note we check not the edge aSx itself, but its image in aPDS
                    NCollection_List<BOPDS_PaveBlock> aLPBx = aPDS.PaveBlocks(nSx);
                    aNbLPBx = aLPBx.Extent();
                    if (aNbLPBx == 0 || (aNbLPBx == 1 && !aLPBx.First().HasShrunkData()))
                    {
                        NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aLPBC);
                        for (; it.More(); it.Next())
                        {
                            if (it.Value() == aPB1)
                            {
                                aLPBC.Remove(it);
                                break;
                            }
                        }

                        // The edge became micro edge, check vertices for SD
                        TopoDS_Iterator itV = new(aSx);
                        for (; itV.More(); itV.Next())
                        {
                            aLS.Append(itV.Value());
                        }

                        continue;
                    }
                    //
                    if (bOld && aNbLPBx == 0)
                    {
                        aDMExEdges.ChangeFind(aPB1).Append(aPB1);
                        continue;
                    }
                    //
                    if (!bOld)
                    {
                        aItLPB.Initialize(aLPBC);
                        for (; aItLPB.More(); aItLPB.Next())
                        {
                            BOPDS_PaveBlock aPBC = aItLPB.Value();
                            if (aPBC == aPB1)
                            {
                                aLPBC.Remove(aItLPB);
                                break;
                            }
                        }
                    }
                    //
                    if (aNbLPBx != 0)
                    {
                        aItLPB.Initialize(aLPBx);
                        for (; aItLPB.More(); aItLPB.Next())
                        {
                            BOPDS_PaveBlock aPBx = aItLPB.Value();
                            BOPDS_PaveBlock aPBRx = aPDS.RealPaveBlock(aPBx);
                            //
                            // update vertices of paves
                            aPave[0] = aPBx.Pave1();
                            aPave[1] = aPBx.Pave2();
                            for (j = 0; j < 2; ++j)
                            {
                                nV = aPave[j].Index();
                                aV = aPDS.Shape(nV);
                                // index of new vertex in myDS -> iV
                                iV = myDS.Index(aV);
                                if (iV < 0)
                                {
                                    aSI.SetShapeType(TopAbs_VERTEX);
                                    aSI.SetShape(aV);
                                    iV = myDS.Append(aSI);
                                }
                                BOPDS_Pave aP1 = j == 0 ? aPB1.Pave1() : aPB1.Pave2();
                                if (aP1.Index() != iV)
                                {
                                    if (aP1.Parameter() == aPave[j].Parameter())
                                    {
                                        aDMNewSD.Bind(aP1.Index(), iV);
                                        myDS.AddShapeSD(aP1.Index(), iV);
                                    }
                                    else
                                    {
                                        // check aPDS to have the SD connection between these vertices
                                        TopoDS_Shape aVPave = myDS.Shape(aP1.Index());
                                        int nVnewSD = 0, nVnew = aPDS.Index(aVPave);
                                        if (aPDS.HasShapeSD(nVnew, ref nVnewSD))
                                        {
                                            if (nVnewSD == nV)
                                            {
                                                aDMNewSD.Bind(aP1.Index(), iV);
                                                myDS.AddShapeSD(aP1.Index(), iV);
                                            }
                                        }
                                    }
                                }
                                //
                                aPave[j].SetIndex(iV);
                            }
                            //
                            // add edge
                            aE = aPDS.Shape(aPBRx.Edge());
                            iE = myDS.Index(aE);
                            if (iE < 0)
                            {
                                aSI.SetShapeType(aType);
                                aSI.SetShape(aE);
                                iE = myDS.Append(aSI);
                            }
                            //
                            // update real edge tolerance according to distances in common block if any
                            if (aPDS.IsCommonBlock(aPBRx))
                            {
                                BOPDS_CommonBlock aCB = aPDS.CommonBlock(aPBRx)!;
                                if (!aMCBTol.Find(aCB, out double pTol))
                                {
                                    double aTol = BOPAlgo_Tools.ComputeToleranceOfCB(aCB, aPDS, aPF.Context());
                                    pTol = aMCBTol.Bound(aCB, aTol);
                                }
                                //
                                if (aNC.Tolerance() < pTol)
                                {
                                    aNC.SetTolerance(pTol);
                                }
                            }
                            // append new PaveBlock to aLPBC
                            if (!aMEPB.Find(iE, out BOPDS_PaveBlock pPBC))
                            {
                                pPBC = aMEPB.Bound(iE, new BOPDS_PaveBlock());
                                BOPDS_Pave aPaveR1, aPaveR2;
                                aPaveR1 = aPBRx.Pave1();
                                aPaveR2 = aPBRx.Pave2();
                                aPaveR1.SetIndex(myDS.Index(aPDS.Shape(aPaveR1.Index())));
                                aPaveR2.SetIndex(myDS.Index(aPDS.Shape(aPaveR2.Index())));
                                //
                                pPBC.SetPave1(aPaveR1);
                                pPBC.SetPave2(aPaveR2);
                                pPBC.SetEdge(iE);
                            }
                            //
                            if (bOld)
                            {
                                pPBC.SetOriginalEdge(aPB1.OriginalEdge());
                                aDMExEdges.ChangeFind(aPB1).Append(pPBC);
                            }
                            else
                            {
                                aLPBC.Append(pPBC);
                            }
                        }
                    }
                }
            } // else if (aType==TopAbs_EDGE)
        } // for (; aItLS.More(); aItLS.Next()) {

        // Update SD for vertices that did not participate in operation
        // OCCT assigns through the map's iterator; the port visits a snapshot of the keys in the map's order and
        // assigns through the map (the same values in the same order; the order matters for chains, decision 784).
        NCollection_List<int> aKeys = new();
        foreach (KeyValuePair<int, int> aKV in aDMNewSD)
        {
            aKeys.Append(aKV.Key);
        }
        for (NCollection_List<int>.Iterator itDM = new(aKeys); itDM.More(); itDM.Next())
        {
            int aKey = itDM.Value();
            if (aDMNewSD.Find(aDMNewSD.Find(aKey), out int pSD))
            {
                aDMNewSD.ChangeFind(aKey) = pSD;
                myDS.AddShapeSD(aKey, pSD);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Updates the face information with the section edges and vertices, and with the pave blocks and vertices
    /// that the post-treatment replaced.
    /// </summary>
    protected void UpdateFaceInfo(NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> theDME,
                                  NCollection_IntDataMap<int> theDMV,
                                  NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap)
    {
        int i, j, nV1, nF1, nF2, aNbFF, aNbC, aNbP;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new();
        NCollection_IntMap aMF = new();

        // Unify pave blocks of the existing edges united on the post-treat stage
        NCollection_IntDataMap<NCollection_List<BOPDS_PaveBlock>> anEdgeLPB = new();

        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        aNbFF = aFFs.Length();
        // 1. Sections (curves, points);
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            aFF.Indices(out nF1, out nF2);
            //
            BOPDS_FaceInfo aFI1 = myDS.ChangeFaceInfo(nF1);
            BOPDS_FaceInfo aFI2 = myDS.ChangeFaceInfo(nF2);
            //
            // 1.1. Section edges
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
            aNbC = aVNC.Length();
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVNC[j];
                NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.ChangePaveBlocks();
                //
                // Add section edges to face info
                aItLPB.Initialize(aLPBC);
                for (; aItLPB.More();)
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    //
                    // Treat existing pave blocks
                    if (theDME.IsBound(aPB))
                    {
                        NCollection_List<BOPDS_PaveBlock> aLPB = theDME.ChangeFind(aPB);
                        UpdateExistingPaveBlocks(aPB, aLPB, thePBFacesMap);

                        NCollection_List<BOPDS_PaveBlock>.Iterator itLPB = new(aLPB);
                        for (; itLPB.More(); itLPB.Next())
                        {
                            int nE1 = itLPB.Value().Edge();
                            if (!anEdgeLPB.Find(nE1, out NCollection_List<BOPDS_PaveBlock> pLPBOnE1))
                            {
                                pLPBOnE1 = anEdgeLPB.Bound(nE1, new NCollection_List<BOPDS_PaveBlock>());
                            }
                            pLPBOnE1.Append(itLPB.Value());
                        }

                        aLPBC.Remove(aItLPB);
                        continue;
                    }
                    //
                    aFI1.ChangePaveBlocksSc().Add(aPB);
                    aFI2.ChangePaveBlocksSc().Add(aPB);
                    // Add edge-PB connection
                    int nE = aPB.Edge();
                    if (!anEdgeLPB.Find(nE, out NCollection_List<BOPDS_PaveBlock> pLPBOnE))
                    {
                        pLPBOnE = anEdgeLPB.Bound(nE, new NCollection_List<BOPDS_PaveBlock>());
                    }
                    pLPBOnE.Append(aPB);

                    aItLPB.Next();
                }
            }
            //
            // 1.2. Section vertices
            NCollection_DynamicArray<BOPDS_Point> aVNP = aFF.Points();
            aNbP = aVNP.Length();
            for (j = 0; j < aNbP; ++j)
            {
                BOPDS_Point aNP = aVNP[j];
                nV1 = aNP.Index();
                if (nV1 < 0)
                {
                    continue;
                }
                aFI1.ChangeVerticesSc().Add(nV1);
                aFI2.ChangeVerticesSc().Add(nV1);
            }
            //
            aMF.Add(nF1);
            aMF.Add(nF2);
        }

        bool bNewCB = false;
        {
            // Unify pave blocks of the existing edges united on the post-treat stage
            // by making new Common blocks from them
            // (the edges are visited in the map's order, which orders the new common blocks, decision 784)
            foreach (KeyValuePair<int, NCollection_List<BOPDS_PaveBlock>> itDM in anEdgeLPB)
            {
                NCollection_List<BOPDS_PaveBlock> aLPB = itDM.Value;
                if (aLPB.Extent() == 1)
                {
                    continue;
                }

                bNewCB = true;

                // Find or create common block
                BOPDS_CommonBlock? aCB = null;
                // Collect all faces attached to common blocks
                NCollection_IntMap aMFaces = new();
                // Collect all pave blocks attached to common blocks
                NCollection_IndexedMap<BOPDS_PaveBlock> aMPaveBlocks = new();

                NCollection_List<BOPDS_PaveBlock>.Iterator itLPB = new(aLPB);
                for (; itLPB.More(); itLPB.Next())
                {
                    BOPDS_PaveBlock aPB = itLPB.Value();
                    aMPaveBlocks.Add(aPB);

                    if (myDS.IsCommonBlock(aPB))
                    {
                        BOPDS_CommonBlock aPBCB = myDS.CommonBlock(aPB)!;
                        // Get pave blocks
                        NCollection_List<BOPDS_PaveBlock> aLPBOnCB = aPBCB.PaveBlocks();
                        for (NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aLPBOnCB); it.More();
                             it.Next())
                        {
                            aMPaveBlocks.Add(it.Value());
                        }

                        // Get faces
                        NCollection_List<int> aLFacesOnCB = aPBCB.Faces();
                        for (NCollection_List<int>.Iterator it = new(aLFacesOnCB); it.More(); it.Next())
                        {
                            aMFaces.Add(it.Value());
                        }

                        aCB ??= aPBCB;
                    }
                }

                if (aCB is null)
                {
                    // None of the pave blocks in the list is a common block,
                    // so create the new one.
                    aCB = new BOPDS_CommonBlock();
                    aCB.SetPaveBlocks(aLPB);
                    itLPB.Initialize(aLPB);
                    for (; itLPB.More(); itLPB.Next())
                    {
                        BOPDS_PaveBlock aPB = itLPB.Value();
                        myDS.SetCommonBlock(aPB, aCB);
                    }
                }
                else
                {
                    // Update common block with new pave blocks
                    NCollection_List<BOPDS_PaveBlock> aLPBNew = new();
                    {
                        int aNbPB = aMPaveBlocks.Extent();
                        for (int iPB = 1; iPB <= aNbPB; ++iPB)
                        {
                            BOPDS_PaveBlock aPB = aMPaveBlocks[iPB];
                            myDS.SetCommonBlock(aPB, aCB);
                            aLPBNew.Append(aPB);
                        }
                    }
                    aCB.SetPaveBlocks(aLPBNew);

                    // Update faces of the common block
                    // (in the map's order, decision 784)
                    NCollection_List<int> aLFaces = new();
                    foreach (int nF in aMFaces)
                    {
                        aLFaces.Append(nF);
                    }
                    aCB.SetFaces(aLFaces);
                }
            }
        }

        bool bVerts = theDMV.Extent() > 0;
        bool bEdges = theDME.Extent() > 0 || bNewCB;
        //
        if (!bVerts && !bEdges)
        {
            return;
        }
        //
        // 2. Update Face Info information with new vertices and new
        //    pave blocks created in PostTreatFF from existing ones
        int nV2;
        //
        foreach (int nF in aMF)
        {
            nF1 = nF;
            //
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF1);
            //
            // 2.1. Update information about vertices
            if (bVerts)
            {
                NCollection_IntMap aMVOn = aFI.ChangeVerticesOn();
                NCollection_IntMap aMVIn = aFI.ChangeVerticesIn();
                //
                foreach (KeyValuePair<int, int> aItMV in theDMV)
                {
                    nV1 = aItMV.Key;
                    nV2 = aItMV.Value;
                    //
                    if (aMVOn.Remove(nV1))
                    {
                        aMVOn.Add(nV2);
                    }
                    //
                    if (aMVIn.Remove(nV1))
                    {
                        aMVIn.Add(nV2);
                    }
                } // for (; aItMV.More(); aItMV.Next()) {
            } // if (bVerts) {
            //
            // 2.2. Update information about pave blocks
            if (bEdges)
            {
                NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();
                NCollection_IndexedMap<BOPDS_PaveBlock>[] pMPB = [aFI.ChangePaveBlocksOn(),
                                                                  aFI.ChangePaveBlocksIn(),
                                                                  aFI.ChangePaveBlocksSc()];
                for (i = 0; i < 3; ++i)
                {
                    NCollection_IndexedMap<BOPDS_PaveBlock> aMPBCopy = new(pMPB[i]);
                    pMPB[i].Clear();
                    int aNbPB = aMPBCopy.Extent();
                    for (j = 1; j <= aNbPB; ++j)
                    {
                        BOPDS_PaveBlock aPB = aMPBCopy[j];
                        if (theDME.Find(aPB, out NCollection_List<BOPDS_PaveBlock> pLPB) && !pLPB.IsEmpty())
                        {
                            aItLPB.Initialize(pLPB);
                            for (; aItLPB.More(); aItLPB.Next())
                            {
                                BOPDS_PaveBlock aPB1 = aItLPB.Value();
                                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB1);
                                if (aMPBFence.Add(aPBR))
                                {
                                    pMPB[i].Add(aPBR);
                                }
                            }
                        }
                        else
                        {
                            BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);
                            if (aMPBFence.Add(aPBR))
                            {
                                pMPB[i].Add(aPBR);
                            }
                        }
                    } // for (j = 1; j <= aNbPB; ++j) {
                } // for (i = 0; i < 2; ++i) {
            } // if (bEdges) {
        }
    }

    //=================================================================================================

    /// <summary>Whether the point coincides with one of the vertices <paramref name="aMVOnIn"/>.</summary>
    protected bool IsExistingVertex(in gp_Pnt aP, double theTolR3D, NCollection_IntMap aMVOnIn)
    {
        bool bRet;
        int nV, iFlag;
        double aTolCheck;
        Bnd_Box aBoxP = new();
        //
        aTolCheck = theTolR3D + myFuzzyValue;
        bRet = true;
        //
        aBoxP.Add(aP);
        aBoxP.Enlarge(theTolR3D);
        //
        foreach (int aIt in aMVOnIn)
        {
            nV = aIt;
            BOPDS_ShapeInfo aSIV = myDS!.ShapeInfo(nV);
            TopoDS_Vertex aV = TopoDS.Vertex(aSIV.Shape());
            Bnd_Box aBoxV = aSIV.Box();
            //
            if (!aBoxP.IsOut(aBoxV))
            {
                iFlag = BOPTools_AlgoTools.ComputeVV(aV, aP, aTolCheck);
                if (iFlag == 0)
                {
                    return bRet;
                }
            }
        }
        return !bRet;
    }

    //=================================================================================================

    /// <summary>Whether the pave block of the section curve coincides with one of the shared edges <paramref name="theLSE"/>.</summary>
    protected bool IsExistingPaveBlock(BOPDS_PaveBlock thePB,
                                       BOPDS_Curve theNC,
                                       NCollection_List<int> theLSE,
                                       ref int theNEOut,
                                       ref double theTolNew)
    {
        if (theLSE.IsEmpty())
        {
            return false;
        }

        double aT1, aT2, aTm, aTx = 0.0, aTolE, aTolCheck, aTol, aDist = 0.0;
        int nE, iFlag, nV1, nV2;
        gp_Pnt aPm = new();
        Bnd_Box aBoxPm = new();
        NCollection_List<int>.Iterator aItLI = new();
        //
        thePB.Range(out aT1, out aT2);
        thePB.Indices(out nV1, out nV2);
        TopoDS_Vertex aV1 = TopoDS.Vertex(myDS!.Shape(nV1)),
                      aV2 = TopoDS.Vertex(myDS.Shape(nV2));
        double aTolV1 = BRep_Tool.Tolerance(aV1), aTolV2 = BRep_Tool.Tolerance(aV2);

        aTol = Max(aTolV1, aTolV2);

        aTm = IntTools_Tools.IntermediatePoint(aT1, aT2);
        theNC.Curve().D0(aTm, ref aPm);
        aBoxPm.Add(aPm);
        aBoxPm.Enlarge(aTol);
        //
        aItLI.Initialize(theLSE);
        for (; aItLI.More(); aItLI.Next())
        {
            nE = aItLI.Value();
            if (nE < 0)
            {
                continue;
            }
            BOPDS_ShapeInfo aSIE = myDS.ChangeShapeInfo(nE);
            Bnd_Box aBoxE = aSIE.Box();
            if (!aBoxE.IsOut(aBoxPm))
            {
                TopoDS_Edge aE = TopoDS.Edge(aSIE.Shape());
                aTolE = BRep_Tool.Tolerance(aE);
                aTolCheck = Max(aTolE, aTol) + myFuzzyValue;
                iFlag = myContext!.ComputePE(aPm, aTolCheck, aE, ref aTx, ref aDist);
                if (iFlag == 0)
                {
                    theNEOut = nE;
                    theTolNew = aDist;
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Looks for the existing pave block (ON/IN the faces) closest to the pave block of the section curve; returns
    /// it in <paramref name="aPBOut"/> with the new tolerance.
    /// </summary>
    protected bool IsExistingPaveBlock(BOPDS_PaveBlock thePB,
                                       BOPDS_Curve theNC,
                                       double theTolR3D,
                                       NCollection_IndexedMap<BOPDS_PaveBlock> theMPBOnIn,
                                       BOPTools_BoxTree thePBTree,
                                       NCollection_Map<BOPDS_PaveBlock> theMPBCommon,
                                       ref BOPDS_PaveBlock? aPBOut,
                                       ref double theTolNew)
    {
        IntTools_Curve aIC = theNC.Curve();

        thePB.Range(out double aT1, out double aT2);

        thePB.Indices(out int nV11, out int nV12);

        // first point
        Bnd_Box aBoxP1 = new();
        gp_Pnt aP1 = new();
        aIC.D0(aT1, ref aP1);
        aBoxP1.Add(aP1);
        double aTolV11 = BRep_Tool.Tolerance(TopoDS.Vertex(myDS!.Shape(nV11)));
        aBoxP1.Enlarge(aTolV11);

        // Find edges intersecting by AABB with the first point
        BOPTools_BoxTreeSelector aSelector = new();
        aSelector.SetBox(Bnd_Tools.Bnd2BVH(aBoxP1));
        aSelector.SetBVHSet(thePBTree);
        if (aSelector.Select() == 0)
        {
            return false;
        }

        // intermediate point
        Bnd_Box aBoxPm = new();
        double aTm = IntTools_Tools.IntermediatePoint(aT1, aT2);
        Geom_Curve aC3d = aIC.Curve()!;
        aC3d.D1(aTm, out gp_Pnt aPm, out gp_Vec aVTgt1);
        aBoxPm.Add(aPm);
        bool isVtgt1Valid = aVTgt1.SquareMagnitude() > gp.Resolution();
        if (isVtgt1Valid)
        {
            aVTgt1.Normalize();
        }

        // last point
        Bnd_Box aBoxP2 = new();
        gp_Pnt aP2 = new();
        aIC.D0(aT2, ref aP2);
        aBoxP2.Add(aP2);
        double aTolV12 = BRep_Tool.Tolerance(TopoDS.Vertex(myDS.Shape(nV12)));
        aBoxP2.Enlarge(aTolV12);

        double aTolV1 = Max(aTolV11, aTolV12) + myFuzzyValue;

        double aTolCheck = theTolR3D + myFuzzyValue;

        // Some limit values to define "thin" face when iflag1=iflag2=2 and
        // edge has no common block with any face
        double aMaxTolAdd = 0.001; // Maximal tolerance of edge allowed
        const double aCoeffTolAdd = 10.0; // Coeff to define max. tolerance with help of aTolCheck
        aMaxTolAdd = Min(aMaxTolAdd, aCoeffTolAdd * aTolCheck);

        // Look for the existing pave block closest to the section curve
        bool bFound = false;
        theTolNew = RealLast();

        for (NCollection_List<int>.Iterator it = new(aSelector.Indices()); it.More(); it.Next())
        {
            BOPDS_PaveBlock aPB = theMPBOnIn[it.Value()];

            aPB.Indices(out int nV21, out int nV22);

            double aTolV21 = BRep_Tool.Tolerance(TopoDS.Vertex(myDS.Shape(nV21)));
            double aTolV22 = BRep_Tool.Tolerance(TopoDS.Vertex(myDS.Shape(nV22)));
            double aTolV2 = Max(aTolV21, aTolV22) + myFuzzyValue;

            BOPDS_ShapeInfo aSISp = myDS.ChangeShapeInfo(aPB.Edge());
            TopoDS_Edge aSp = TopoDS.Edge(aSISp.Shape());
            Bnd_Box aBoxSp = aSISp.Box();

            int iFlag1 = (nV11 == nV21 || nV11 == nV22) ? 2 : 1;
            int iFlag2 = (nV12 == nV21 || nV12 == nV22) ? 2 : (!aBoxSp.IsOut(aBoxP2) ? 1 : 0);
            if (iFlag2 == 0)
            {
                continue;
            }

            double aDist = 0.0;
            double aCoeff = 1.0; // Coeff for taking in account deflections between edge and theNC
            // when aPB is not common block
            double aDistm1m2 = 0.0;
            int aPEStatus = 1;

            double aRealTol = aTolCheck;
            if (myDS.IsCommonBlock(aPB))
            {
                aRealTol = Max(aRealTol, Max(aTolV1, aTolV2));
                if (theMPBCommon.Contains(aPB))
                {
                    // for an edge, which is a common block with a face,
                    // increase the chance to coincide with section curve
                    aRealTol *= 2.0;
                }
            }
            else if (iFlag1 == 2 && iFlag2 == 2)
            {
                // Check, if edge could be common block with section curve
                //  and increase the chance to coincide with section curve
                // skip processing if one edge is closed, but other is not closed
                // such configurations can give iFlag1 == 2 && iFlag2 == 2
                bool bSkipProcessing =
                    ((nV11 == nV12) && (nV21 != nV22)) || ((nV11 != nV12) && (nV21 == nV22));

                if (!bSkipProcessing)
                {

                    if (isVtgt1Valid)
                    {
                        BRepAdaptor_Curve aBAC2 = new(aSp);
                        if (aIC.Type() != GeomAbs_Line || aBAC2.GetType() != GeomAbs_Line)
                        {
                            double aTldp = 0.0; // OCCT: uninitialised, read only when ComputePE succeeds
                            double aTolAdd =
                                2.0 * Min(aMaxTolAdd, Max(aRealTol, Max(aTolV1, aTolV2)));
                            aPEStatus = myContext!.ComputePE(aPm, aTolAdd, aSp, ref aTldp, ref aDistm1m2);

                            if (aPEStatus == 0)
                            {
                                aBAC2.D1(aTldp, out gp_Pnt aPm2, out gp_Vec aVTgt2);
                                if (aVTgt2.SquareMagnitude() > gp.Resolution())
                                {
                                    // The angle should be close to zero
                                    double aCos = aVTgt1.Dot(aVTgt2.Normalized());
                                    if (Math.Abs(aCos) >= 0.9063)
                                    {
                                        aRealTol = aTolAdd;
                                        aCoeff = 2.0;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Bnd_Box aBoxTmp = aBoxPm;
            aBoxTmp.Enlarge(aRealTol);

            double aDistToSp = 0.0;
            double aTx = 0.0; // OCCT: uninitialised, written by ComputePE and never read
            if (aBoxSp.IsOut(aBoxTmp) || aPEStatus < 0)
            {
                continue;
            }
            else if (aPEStatus == 0) // aPEStatus == 0 for case iflag1 == iflag2 == 2
            {
                aDistToSp = aDistm1m2;
            }
            else if (aPEStatus == 1) // Projection has not been done yet
            {
                aPEStatus = myContext!.ComputePE(aPm, aRealTol, aSp, ref aTx, ref aDistToSp);
                if (aPEStatus < 0)
                {
                    continue;
                }
            }
            //
            if (iFlag1 == 1)
            {
                iFlag1 = myContext!.ComputePE(aP1, aRealTol, aSp, ref aTx, ref aDist) == 0 ? 1 : 0;
                if (iFlag1 != 0 && aDistToSp < aDist)
                {
                    aDistToSp = aDist;
                }
            }
            //
            if (iFlag2 == 1)
            {
                iFlag2 = myContext!.ComputePE(aP2, aRealTol, aSp, ref aTx, ref aDist) == 0 ? 1 : 0;
                if (iFlag2 != 0 && aDistToSp < aDist)
                {
                    aDistToSp = aDist;
                }
            }
            //
            if (iFlag1 != 0 && iFlag2 != 0)
            {
                if (aDistToSp < theTolNew)
                {
                    aPBOut = aPB;
                    theTolNew = aCoeff * aDistToSp;
                    bFound = true;
                }
            }
        }
        return bFound;
    }

    //=================================================================================================

    private static void getBoundPaves(BOPDS_DS theDS, BOPDS_Curve theNC, Span<int> theNV)
    {
        theNV[0] = theNV[1] = -1;

        // get extreme paves
        BOPDS_PaveBlock aPB = theNC.PaveBlocks().First();
        NCollection_List<BOPDS_Pave> aLP = aPB.ExtPaves();
        int aNbEP = aLP.Extent();
        if (aNbEP == 0)
        {
            return;
        }
        double aTmin = RealLast();
        double aTmax = -aTmin;
        for (NCollection_List<BOPDS_Pave>.Iterator aItLP = new(aLP); aItLP.More(); aItLP.Next())
        {
            BOPDS_Pave aPv = aItLP.Value();
            int nV;
            double aTV;
            aPv.Contents(out nV, out aTV);
            if (aTV < aTmin)
            {
                theNV[0] = aPv.Index();
                aTmin = aTV;
            }
            if (aTV > aTmax)
            {
                theNV[1] = aPv.Index();
                aTmax = aTV;
            }
        }

        // compare extreme vertices with ends of the curve
        IntTools_Curve aIC = theNC.Curve();
        double aT0 = 0.0, aT1 = 0.0;
        gp_Pnt[] aP = [new gp_Pnt(), new gp_Pnt()];
        aIC.Bounds(ref aT0, ref aT1, ref aP[0], ref aP[1]);
        double aTol = Max(theNC.Tolerance(), theNC.TangentialTolerance());
        aTol += Precision.Confusion();
        for (int j = 0; j < 2; ++j)
        {
            BOPDS_ShapeInfo aSIV = theDS.ShapeInfo(theNV[j]);
            TopoDS_Vertex aV = TopoDS.Vertex(aSIV.Shape());
            int iFlag = BOPTools_AlgoTools.ComputeVV(aV, aP[j], aTol);
            if (iFlag != 0)
            {
                theNV[j] = -1;
            }
        }
    }

    //=================================================================================================

    protected void PutBoundPaveOnCurve(in TopoDS_Face aF1, in TopoDS_Face aF2, BOPDS_Curve aNC, NCollection_List<int> aLVB)
    {
        IntTools_Curve aIC = aNC.Curve();
        Span<double> aT = stackalloc double[2];
        gp_Pnt[] aP = [new gp_Pnt(), new gp_Pnt()];
        aIC.Bounds(ref aT[0], ref aT[1], ref aP[0], ref aP[1]);
        double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
        BOPDS_PaveBlock aPB = aNC.ChangePaveBlock1();
        // Get numbers of vertices assigned to the ends of the curve
        Span<int> aBndNV = stackalloc int[2];
        getBoundPaves(myDS!, aNC, aBndNV);
        //
        double aTolVnew = Precision.Confusion();
        bool isClosed = aP[1].IsEqual(aP[0], aTolVnew);
        if (isClosed && (aBndNV[0] > 0 || aBndNV[1] > 0))
        {
            return;
        }

        for (int j = 0; j < 2; ++j)
        {
            if (aBndNV[j] < 0)
            {
                // no vertex on this end
                if (j != 0 && isClosed)
                {
                    // if curve is closed, process only one bound
                    continue;
                }
                bool bVF = myContext!.IsValidPointForFaces(aP[j], aF1, aF2, aTolR3D);
                if (!bVF)
                {
                    continue;
                }
                BOPTools_AlgoTools.MakeNewVertex(aP[j], aTolR3D, out TopoDS_Vertex aVn);
                BOPTools_AlgoTools.UpdateVertex(aIC, aT[j], aVn);
                aTolVnew = BRep_Tool.Tolerance(aVn);

                BOPDS_ShapeInfo aSIVn = new();
                aSIVn.SetShapeType(TopAbs_VERTEX);
                aSIVn.SetShape(aVn);

                ref Bnd_Box aBox = ref aSIVn.ChangeBox();
                BRepBndLib.Add(aVn, ref aBox);
                aBox.SetGap(aBox.GetGap() + Precision.Confusion());

                int nVn = myDS!.Append(aSIVn);

                BOPDS_Pave aPn = new();
                aPn.SetIndex(nVn);
                aPn.SetParameter(aT[j]);
                aPB.AppendExtPave(aPn);

                aLVB.Append(nVn);
            }
        }
    }

    //=================================================================================================

    protected void PutPavesOnCurve(NCollection_IntMap theMVOnIn,
                                   NCollection_IntMap theMVCommon,
                                   BOPDS_Curve theNC,
                                   NCollection_IntMap theMI,
                                   NCollection_IntMap theMVEF,
                                   NCollection_IntDataMap<double> theMVTol,
                                   NCollection_IntDataMap<NCollection_List<int>> theDMVLV)
    {
        //
        Bnd_Box aBoxC = theNC.Box();
        double aTolR3D = Max(theNC.Tolerance(), theNC.TangentialTolerance());
        //
        // Put EF vertices first
        // (decision 784: the order of the map decides the order of the extra paves and of the lists of theDMVLV)
        foreach (int nV in theMVEF)
        {
            PutPaveOnCurve(nV, aTolR3D, theNC, theMI, theMVTol, theDMVLV, 2);
        }

        // Put all other vertices
        foreach (int nV in theMVOnIn)
        {
            if (theMVEF.Contains(nV))
            {
                continue;
            }

            if (!theMVCommon.Contains(nV))
            {
                BOPDS_ShapeInfo aSIV = myDS!.ShapeInfo(nV);
                Bnd_Box aBoxV = aSIV.Box();
                //
                if (aBoxC.IsOut(aBoxV))
                {
                    continue;
                }
                if (!myDS.IsNewShape(nV))
                {
                    continue;
                }
            }
            //
            PutPaveOnCurve(nV, aTolR3D, theNC, theMI, theMVTol, theDMVLV, 1);
        }
    }

    //=================================================================================================

    private struct PaveBlockDist
    {
        public BOPDS_PaveBlock PB;
        public double SquareDist; // square distance from vertex to the paveblock
        public double SinAngle;   // sinus of angle between projection vector
        // and tangent at projection point
        public double Tolerance; // tolerance of the section curve
    }

    protected void FilterPavesOnCurves(NCollection_DynamicArray<BOPDS_Curve> theVNC, NCollection_IntDataMap<double> theMVTol)
    {
        // For each vertex found in ExtPaves of pave blocks of section curves
        // collect list of pave blocks with distance to the curve
        NCollection_IndexedDataMap<int, NCollection_List<PaveBlockDist>> aIDMVertPBs = new();
        int i;
        double anEps = gp.Resolution();
        for (i = 0; i < theVNC.Length(); ++i)
        {
            BOPDS_Curve aNC = theVNC[i];
            IntTools_Curve aIC = aNC.Curve();
            double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
            GeomAdaptor_Curve aGAC = new(aIC.Curve()!);
            BOPDS_PaveBlock aPB = aNC.PaveBlocks().First();
            NCollection_List<BOPDS_Pave> aPaves = aPB.ExtPaves();
            NCollection_List<BOPDS_Pave>.Iterator itPaves = new(aPaves);
            for (; itPaves.More(); itPaves.Next())
            {
                BOPDS_Pave aPave = itPaves.Value();
                int nV = aPave.Index();
                TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
                // compute distance from vertex to the point on curve with vertex parameter
                gp_Pnt aPV = BRep_Tool.Pnt(aV);
                double aPar = aPave.Parameter();
                aGAC.D1(aPar, out gp_Pnt aPonC, out gp_Vec aD1);
                gp_Vec aProjVec = new(aPV, aPonC);
                double aSqDist = aProjVec.SquareMagnitude();
                double aSqD1Mod = aD1.SquareMagnitude();
                double aSin = aProjVec.CrossSquareMagnitude(aD1);
                if (aSqDist > anEps && aSqD1Mod > anEps)
                {
                    aSin = Math.Sqrt(aSin / aSqDist / aSqD1Mod);
                }
                int anIndex = aIDMVertPBs.FindIndex(nV);
                NCollection_List<PaveBlockDist> pList = anIndex > 0
                    ? aIDMVertPBs.ChangeFromIndex(anIndex)
                    : aIDMVertPBs.ChangeFromIndex(aIDMVertPBs.Add(nV, new NCollection_List<PaveBlockDist>()));
                PaveBlockDist aPBD = new() { PB = aPB, SquareDist = aSqDist, SinAngle = aSin, Tolerance = aTolR3D };
                pList.Append(aPBD);
            }
        }

        // Process each vertex
        const double aSinAngleMin = 0.5; // angle below which projection is suspicious
        for (i = 1; i <= aIDMVertPBs.Extent(); i++)
        {
            int nV = aIDMVertPBs.FindKey(i);
            NCollection_List<PaveBlockDist> aList = aIDMVertPBs[i];
            // Find a pave with minimal distance
            double aMinDist = RealLast();
            BOPDS_PaveBlock? aPBMinDist = null;
            NCollection_List<PaveBlockDist>.Iterator itL = new(aList);
            for (; itL.More(); itL.Next())
            {
                PaveBlockDist aPBD = itL.Value();
                if (aPBD.SquareDist < aMinDist)
                {
                    aMinDist = aPBD.SquareDist;
                    aPBMinDist = aPBD.PB;
                }
            }
            _ = aPBMinDist; // OCCT finds it and never reads it
            // Remove a vertex from a pave block if the distance is greater than the tolerance
            // and there are other pave blocks for which the distance is less than the current.
            // Do not remove a vertex if it is projected on the curve with quite large angle
            // (see test bugs modalg_6 bug27761).

            // Reduce tolerance for the vertex to the value of maximal distance to
            // to section curve on which it will be kept.
            double aMaxDistKept = -1;
            bool isRemoved = false;
            for (itL.Initialize(aList); itL.More(); itL.Next())
            {
                PaveBlockDist aPBD = itL.Value();
                double aCheckDist = 100.0 * Max(aPBD.Tolerance * aPBD.Tolerance, aMinDist);
                if (aPBD.SquareDist > aCheckDist && aPBD.SinAngle < aSinAngleMin)
                {
                    aPBD.PB.RemoveExtPave(nV);
                    isRemoved = true;
                }
                else if (aPBD.SquareDist > aMaxDistKept)
                {
                    aMaxDistKept = aPBD.SquareDist;
                }
            }

            if (isRemoved && aMaxDistKept > 0)
            {
                if (theMVTol.Find(nV, out double pTol))
                {
                    TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
                    double aRealTol = Max(pTol, Math.Sqrt(aMaxDistKept) + Precision.Confusion());
                    // OCCT writes the tolerance into the vertex's TShape directly (a frozen vertex throws).
                    // OCCT.NET: an identical write is skipped, as in MakeBlocks (unit 6G).
                    if (((BRep_TVertex)aV.TShape()!).Tolerance() != aRealTol)
                    {
                        ((BRep_TVertex)aV.TShape()!).Tolerance(aRealTol);
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected bool ExtendedTolerance(int nV, NCollection_IntMap aMI, ref double aTolVExt, int aType = 0)
    {
        bool bFound = false;
        if (!(myDS!.IsNewShape(nV)))
        {
            return bFound;
        }
        //
        int i, k, aNbLines, aNbInt;
        double aT11, aT12, aD1, aD2, aD;
        TopoDS_Vertex aV;
        gp_Pnt aPV, aP11, aP12;
        //
        k = 0;
        aNbInt = 2;
        if (aType == 1)
        {
            aNbInt = 1;
        }
        else if (aType == 2)
        {
            k = 1;
        }
        //
        aV = TopoDS.Vertex(myDS.Shape(nV));
        aPV = BRep_Tool.Pnt(aV);
        //
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();
        //
        for (; k < aNbInt; ++k)
        {
            aNbLines = k == 0 ? aEEs.Length() : aEFs.Length();
            for (i = 0; i < aNbLines; ++i)
            {
                BOPDS_Interf aInt = k == 0 ? aEEs[i] : aEFs[i];
                if (aInt.IndexNew() == nV)
                {
                    if (aMI.Contains(aInt.Index1()) && aMI.Contains(aInt.Index2()))
                    {
                        IntTools_CommonPrt aComPrt = k == 0 ? aEEs[i].CommonPart() : aEFs[i].CommonPart();
                        //
                        TopoDS_Edge aE1 = aComPrt.Edge1();
                        aComPrt.Range1(out aT11, out aT12);
                        BOPTools_AlgoTools.PointOnEdge(aE1, aT11, out aP11);
                        BOPTools_AlgoTools.PointOnEdge(aE1, aT12, out aP12);
                        aD1 = aPV.Distance(aP11);
                        aD2 = aPV.Distance(aP12);
                        aD = (aD1 > aD2) ? aD1 : aD2;
                        if (aD > aTolVExt)
                        {
                            aTolVExt = aD;
                        }
                        return !bFound;
                    } // if (aMI.Contains(aEF.Index1()) && aMI.Contains(aEF.Index2())) {
                } // if (aInt->IndexNew() == nV) {
            } // for (i = 0; i < aNbLines; ++i) {
        } // for (k=0; k<2; ++k) {
        return bFound;
    }

    //=================================================================================================

    protected void GetEFPnts(int nF1, int nF2, NCollection_List<IntSurf_PntOn2S> aListOfPnts)
    {
        int nE, nF, nFOpposite, aNbEFs, i;
        double U1, U2, V1, V2, f, l;
        NCollection_IntMap aMI = new();
        //
        // collect indexes of all shapes from nF1 and nF2.
        GetFullShapeMap(nF1, aMI);
        GetFullShapeMap(nF2, aMI);
        //
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS!.InterfEF();
        aNbEFs = aEFs.Length();
        //
        for (i = 0; i < aNbEFs; ++i)
        {
            BOPDS_InterfEF aEF = aEFs[i];
            if (aEF.HasIndexNew())
            {
                aEF.Indices(out nE, out nFOpposite);
                if (aMI.Contains(nE) && aMI.Contains(nFOpposite))
                {
                    IntTools_CommonPrt aCP = aEF.CommonPart();
                    double aPar = aCP.VertexParameter1();
                    TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                    TopoDS_Face aFOpposite = TopoDS.Face(myDS.Shape(nFOpposite));
                    //
                    Geom_Curve aCurve = BRep_Tool.Curve(aE, out f, out l)!;
                    //
                    nF = (nFOpposite == nF1) ? nF2 : nF1;
                    TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));
                    Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(aE, aF, out f, out l);
                    //
                    GeomAPI_ProjectPointOnSurf aProj = myContext!.ProjPS(aFOpposite);
                    //
                    aCurve.D0(aPar, out gp_Pnt aPoint);
                    IntSurf_PntOn2S aPnt = new();
                    if (aPCurve is not null)
                    {
                        gp_Pnt2d aP2d = aPCurve.Value(aPar);
                        aProj.Perform(aPoint);
                        if (aProj.IsDone())
                        {
                            aProj.LowerDistanceParameters(out U1, out V1);
                            if (nF == nF1)
                            {
                                aPnt.SetValue(aP2d.X(), aP2d.Y(), U1, V1);
                            }
                            else
                            {
                                aPnt.SetValue(U1, V1, aP2d.X(), aP2d.Y());
                            }
                            aListOfPnts.Append(aPnt);
                        }
                    }
                    else
                    {
                        GeomAPI_ProjectPointOnSurf aProj1 = myContext.ProjPS(aF);
                        aProj1.Perform(aPoint);
                        aProj.Perform(aPoint);
                        if (aProj1.IsDone() && aProj.IsDone())
                        {
                            aProj1.LowerDistanceParameters(out U1, out V1);
                            aProj.LowerDistanceParameters(out U2, out V2);
                            if (nF == nF1)
                            {
                                aPnt.SetValue(U1, V1, U2, V2);
                            }
                            else
                            {
                                aPnt.SetValue(U2, V2, U1, V1);
                            }
                            aListOfPnts.Append(aPnt);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void PutEFPavesOnCurve(NCollection_DynamicArray<BOPDS_Curve> theVC,
                                     int theIndex,
                                     NCollection_IntMap aMI,
                                     NCollection_IntMap aMVEF,
                                     NCollection_IntDataMap<double> aMVTol,
                                     NCollection_IntDataMap<NCollection_List<int>> aDMVLV)
    {
        if (aMVEF.Extent() == 0)
        {
            return;
        }
        //
        BOPDS_Curve aNC = theVC.Value(theIndex);
        IntTools_Curve aIC = aNC.Curve();
        GeomAbs_CurveType aTypeC;
        aTypeC = aIC.Type();
        if (aTypeC != GeomAbs_BezierCurve && aTypeC != GeomAbs_BSplineCurve)
        {
            return;
        }
        //
        NCollection_IntMap aMV = new(aMVEF);
        //
        RemoveUsedVertices(theVC, aMV);
        if (aMV.Extent() == 0)
        {
            return;
        }
        //
        double aDist;
        //
        Geom_Curve aC3D = aIC.Curve()!;
        GeomAPI_ProjectPointOnCurve aProjPT = myContext!.ProjPT(aC3D);
        //
        // (decision 784: the order of the map decides the order of the extra paves)
        foreach (int nV in aMV)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
            gp_Pnt aPV = BRep_Tool.Pnt(aV);
            aProjPT.Perform(aPV);
            int aNbPoints = aProjPT.NbPoints();
            if (aNbPoints != 0)
            {
                aDist = aProjPT.LowerDistance();
                PutPaveOnCurve(nV, aDist, aNC, aMI, aMVTol, aDMVLV);
            }
        }
    }

    //=================================================================================================

    protected void PutStickPavesOnCurve(in TopoDS_Face aF1,
                                        in TopoDS_Face aF2,
                                        NCollection_IntMap aMI,
                                        NCollection_DynamicArray<BOPDS_Curve> theVC,
                                        int theIndex,
                                        NCollection_IntMap aMVStick,
                                        NCollection_IntDataMap<double> aMVTol,
                                        NCollection_IntDataMap<NCollection_List<int>> aDMVLV)
    {
        BOPDS_Curve aNC = theVC.Value(theIndex);
        // Get numbers of vertices assigned to the ends of the curve
        Span<int> aBndNV = stackalloc int[2];
        getBoundPaves(myDS!, aNC, aBndNV);
        if (aBndNV[0] >= 0 && aBndNV[1] >= 0)
        {
            // both curve ends already have assigned vertices
            return;
        }
        NCollection_IntMap aMV = new(aMVStick);
        RemoveUsedVertices(theVC, aMV);
        //
        if (aMV.Extent() == 0)
        {
            return;
        }
        //
        Geom_Surface aS1 = BRep_Tool.Surface(aF1)!;
        Geom_Surface aS2 = BRep_Tool.Surface(aF2)!;
        //
        IntTools_Curve aIC = aNC.Curve();
        Geom2d_Curve?[] aC2D = new Geom2d_Curve?[2];
        //
        aC2D[0] = aIC.FirstCurve2d();
        aC2D[1] = aIC.SecondCurve2d();
        if (aC2D[0] is not null && aC2D[1] is not null)
        {
            int m, n;
            double aD, aD2, u, v, aDT2, aScPr, aDScPr;
            Span<double> aTC = stackalloc double[2];
            gp_Pnt[] aPC = [new gp_Pnt(), new gp_Pnt()];
            gp_Pnt aPV;
            gp_Dir[] aDN = [new gp_Dir(), new gp_Dir()];
            gp_Pnt2d aP2D;
            //
            aDT2 = 2e-7;  // the rich criteria
            aDScPr = 5.0e-9; // the creasing criteria
            aIC.Bounds(ref aTC[0], ref aTC[1], ref aPC[0], ref aPC[1]);
            //
            // (decision 784: the order of the map decides which vertex is put first)
            foreach (int nV in aMV)
            {
                TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
                aPV = BRep_Tool.Pnt(aV);
                //
                for (m = 0; m < 2; ++m)
                {
                    if (aBndNV[m] >= 0)
                    {
                        continue;
                    }
                    aD2 = aPC[m].SquareDistance(aPV);
                    if (aD2 > aDT2)
                    { // no rich
                        continue;
                    }
                    //
                    for (n = 0; n < 2; ++n)
                    {
                        Geom_Surface aS = (n == 0) ? aS1 : aS2;
                        aC2D[n]!.D0(aTC[m], out aP2D);
                        aP2D.Coord(out u, out v);
                        BOPTools_AlgoTools3D.GetNormalToSurface(aS, u, v, ref aDN[n]);
                    }
                    //
                    aScPr = aDN[0] * aDN[1];
                    if (aScPr < 0.0)
                    {
                        aScPr = -aScPr;
                    }
                    aScPr = 1.0 - aScPr;
                    //
                    if (aScPr > aDScPr)
                    {
                        continue;
                    }
                    //
                    // The intersection curve aIC is vanishing curve (the crease)
                    aD = Math.Sqrt(aD2);
                    //
                    PutPaveOnCurve(nV, aD, aNC, aMI, aMVTol, aDMVLV);
                }
            } // for (jVU=1; jVU=aNbVU; ++jVU) {
        }
    }

    //=================================================================================================

    protected void GetStickVertices(int nF1, int nF2, NCollection_IntMap aMVStick, NCollection_IntMap aMVEF, NCollection_IntMap aMI)
    {
        int nS1, nS2, nVNew, aTypeInt, i;
        //
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = myDS!.InterfVV();
        NCollection_DynamicArray<BOPDS_InterfVE> aVEs = myDS.InterfVE();
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        NCollection_DynamicArray<BOPDS_InterfVF> aVFs = myDS.InterfVF();
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();
        //
        Span<int> aNbLines = [aVVs.Length(), aVEs.Length(), aEEs.Length(), aVFs.Length(), aEFs.Length()];
        // collect indices of all shapes from nF1 and nF2.
        aMI.Clear();
        GetFullShapeMap(nF1, aMI);
        GetFullShapeMap(nF2, aMI);
        //
        // collect VV, VE, EE, VF interferences
        for (aTypeInt = 0; aTypeInt < 4; ++aTypeInt)
        {
            for (i = 0; i < aNbLines[aTypeInt]; ++i)
            {
                BOPDS_Interf aInt = (aTypeInt == 0)
                                        ? aVVs[i]
                                        : ((aTypeInt == 1) ? aVEs[i]
                                                           : ((aTypeInt == 2) ? aEEs[i]
                                                                              : aVFs[i]));
                if (aInt.HasIndexNew())
                {
                    aInt.Indices(out nS1, out nS2);
                    if (aMI.Contains(nS1) && aMI.Contains(nS2))
                    {
                        nVNew = aInt.IndexNew();
                        myDS.HasShapeSD(nVNew, ref nVNew);
                        aMVStick.Add(nVNew);
                    }
                }
            }
        }
        // collect EF interferences
        for (i = 0; i < aNbLines[4]; ++i)
        {
            BOPDS_InterfEF aInt = aEFs[i];
            if (aInt.HasIndexNew())
            {
                aInt.Indices(out nS1, out nS2);
                if (aMI.Contains(nS1) && aMI.Contains(nS2))
                {
                    nVNew = aInt.IndexNew();
                    myDS.HasShapeSD(nVNew, ref nVNew);
                    aMVStick.Add(nVNew);
                    aMVEF.Add(nVNew);
                }
            }
        }
    }

    //=================================================================================================

    protected void GetFullShapeMap(int nF, NCollection_IntMap aMI)
    {
        NCollection_List<int>.Iterator aIt = new();
        int nS;
        //
        BOPDS_ShapeInfo aSI = myDS!.ShapeInfo(nF);
        NCollection_List<int> aLI = aSI.SubShapes();
        //
        aMI.Add(nF);
        aIt.Initialize(aLI);
        for (; aIt.More(); aIt.Next())
        {
            nS = aIt.Value();
            aMI.Add(nS);
        }
    }

    //=================================================================================================

    protected void RemoveUsedVertices(NCollection_DynamicArray<BOPDS_Curve> aVC, NCollection_IntMap aMV)
    {
        if (aMV.IsEmpty())
        {
            return;
        }

        for (int i = 0; i < aVC.Length(); ++i)
        {
            BOPDS_Curve aNC = aVC.Value(i);
            NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.PaveBlocks();
            NCollection_List<BOPDS_PaveBlock>.Iterator itPB = new(aLPBC);
            for (; itPB.More(); itPB.Next())
            {
                BOPDS_PaveBlock aPB = itPB.Value();
                NCollection_List<BOPDS_Pave> aLP = aPB.ExtPaves();
                NCollection_List<BOPDS_Pave>.Iterator itLP = new(aLP);
                for (; itLP.More(); itLP.Next())
                {
                    aMV.Remove(itLP.Value().Index());
                }

                aMV.Remove(aPB.Pave1().Index());
                aMV.Remove(aPB.Pave2().Index());
            }
        }
    }

    //=================================================================================================

    protected void PutPaveOnCurve(int nV,
                                  double aTolR3D,
                                  BOPDS_Curve aNC,
                                  NCollection_IntMap aMI,
                                  NCollection_IntDataMap<double> aMVTol,
                                  NCollection_IntDataMap<NCollection_List<int>> aDMVLV,
                                  int iCheckExtend = 0)
    {
        bool bIsVertexOnLine;
        double aT = 0.0; // OCCT: uninitialised, written by IsVertexOnLine when it returns true
        //
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        BOPDS_PaveBlock aPB = aNC.PaveBlocks().First();
        IntTools_Curve aIC = aNC.Curve();
        //
        double aTolV = (aMVTol.IsBound(nV) ? aMVTol.Find(nV) : BRep_Tool.Tolerance(aV));

        bIsVertexOnLine = myContext!.IsVertexOnLine(aV, aTolV, aIC, aTolR3D + myFuzzyValue, ref aT);
        if (!bIsVertexOnLine && iCheckExtend != 0 && !myVertsToAvoidExtension.Contains(nV))
        {
            double anExtraTol = aTolV;
            if (ExtendedTolerance(nV, aMI, ref anExtraTol, iCheckExtend))
            {
                bIsVertexOnLine = myContext.IsVertexOnLine(aV, anExtraTol, aIC, aTolR3D + myFuzzyValue, ref aT);
                if (bIsVertexOnLine)
                {
                    gp_Pnt aPOnC = new();
                    aIC.D0(aT, ref aPOnC);
                    aTolV = aPOnC.Distance(BRep_Tool.Pnt(aV));
                }
            }
        }
        //
        if (bIsVertexOnLine)
        {
            // check if aPB contains the parameter aT
            bool bExist;
            int nVUsed = 0; // OCCT: uninitialised, written by ContainsParameter when it returns true
            double aPTol, aDTol;
            //
            aDTol = BOPTools_AlgoTools.DTolerance();
            //
            GeomAdaptor_Curve aGAC = new(aIC.Curve()!);
            aPTol = aGAC.Resolution(Max(aTolR3D, aTolV));
            //
            bExist = aPB.ContainsParameter(aT, aPTol, ref nVUsed);
            if (bExist)
            {
                // use existing pave
                if (!aDMVLV.Find(nVUsed, out NCollection_List<int> pList))
                {
                    pList = aDMVLV.Bound(nVUsed, new NCollection_List<int>());
                    pList.Append(nVUsed);
                    if (!aMVTol.IsBound(nVUsed))
                    {
                        TopoDS_Vertex aVUsed = TopoDS.Vertex(myDS.Shape(nVUsed));
                        aTolV = BRep_Tool.Tolerance(aVUsed);
                        aMVTol.Bind(nVUsed, aTolV);
                    }
                }
                // avoid repeated elements in the list
                NCollection_List<int>.Iterator aItLI = new(pList);
                for (; aItLI.More(); aItLI.Next())
                {
                    if (aItLI.Value() == nV)
                    {
                        break;
                    }
                }
                if (!aItLI.More())
                {
                    pList.Append(nV);
                }
                // save initial tolerance for the vertex
                if (!aMVTol.IsBound(nV))
                {
                    aTolV = BRep_Tool.Tolerance(aV);
                    aMVTol.Bind(nV, aTolV);
                }
            }
            else
            {
                // add new pave
                BOPDS_Pave aPave = new();
                aPave.SetIndex(nV);
                aPave.SetParameter(aT);
                aPB.AppendExtPave(aPave);
                //
                gp_Pnt aP1 = aGAC.Value(aT);
                aTolV = BRep_Tool.Tolerance(aV);
                gp_Pnt aP2 = BRep_Tool.Pnt(aV);
                double aDist = aP1.Distance(aP2);
                if (aTolV < aDist + aDTol)
                {
                    // OCCT writes the vertex in place, also an argument vertex shared by both faces (theMVCommon of
                    // PutPavesOnCurve) in non-destructive mode: a frozen vertex throws here.
                    new BRep_Builder().UpdateVertex(aV, aDist + aDTol);
                    //
                    if (!aMVTol.IsBound(nV))
                    {
                        aMVTol.Bind(nV, aTolV);
                    }
                    //
                    BOPDS_ShapeInfo aSIDS = myDS.ChangeShapeInfo(nV);
                    ref Bnd_Box aBoxDS = ref aSIDS.ChangeBox();
                    BRepBndLib.Add(aV, ref aBoxDS);
                    aBoxDS.SetGap(aBoxDS.GetGap() + Precision.Confusion());
                }
            }
        }
    }

    //=================================================================================================

    protected void ProcessExistingPaveBlocks(int theInt,
                                             int theCur,
                                             int nF1,
                                             int nF2,
                                             in TopoDS_Edge theES,
                                             NCollection_IndexedMap<BOPDS_PaveBlock> theMPBOnIn,
                                             BOPTools_BoxTree thePBTree,
                                             NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB,
                                             NCollection_DataMap<TopoDS_Shape, int> theMVI,
                                             NCollection_List<BOPDS_PaveBlock> theLPBC,
                                             NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap,
                                             NCollection_Map<BOPDS_PaveBlock> theMPB)
    {
        Bnd_Box aBoxES = new();
        BRepBndLib.Add(theES, ref aBoxES, false);

        BOPTools_BoxTreeSelector aSelector = new();
        aSelector.SetBox(Bnd_Tools.Bnd2BVH(aBoxES));
        aSelector.SetBVHSet(thePBTree);
        if (aSelector.Select() == 0)
        {
            return;
        }

        double aTolES = BRep_Tool.Tolerance(theES);

        BOPDS_FaceInfo aFI1 = myDS!.FaceInfo(nF1);
        BOPDS_FaceInfo aFI2 = myDS.FaceInfo(nF2);

        for (NCollection_List<int>.Iterator itPB = new(aSelector.Indices()); itPB.More(); itPB.Next())
        {
            BOPDS_PaveBlock aPBF = theMPBOnIn[itPB.Value()];
            if (theMPB.Contains(aPBF))
            {
                continue;
            }

            bool bInF1 = (aFI1.PaveBlocksOn().Contains(aPBF) || aFI1.PaveBlocksIn().Contains(aPBF));
            bool bInF2 = (aFI2.PaveBlocksOn().Contains(aPBF) || aFI2.PaveBlocksIn().Contains(aPBF));
            if (bInF1 && bInF2)
            {
                // Add all common edges for post treatment
                theMPB.Add(aPBF);
                PreparePostTreatFF(theInt, theCur, aPBF, theMSCPB, theMVI, theLPBC);
                continue;
            }

            int nF = bInF1 ? nF2 : nF1;
            if (!myDistances.Find(new BOPDS_Pair(aPBF.OriginalEdge(), nF), out NCollection_List<EdgeRangeDistance> pList))
            {
                continue;
            }

            aPBF.Range(out double aT1, out double aT2);

            double aDist = RealLast();
            for (NCollection_List<EdgeRangeDistance>.Iterator itR = new(pList); itR.More(); itR.Next())
            {
                EdgeRangeDistance aRangeDist = itR.Value();
                if ((aT1 <= aRangeDist.First && aRangeDist.First <= aT2)
                    || (aT1 <= aRangeDist.Last && aRangeDist.Last <= aT2)
                    || (aRangeDist.First <= aT1 && aT1 <= aRangeDist.Last)
                    || (aRangeDist.First <= aT2 && aT2 <= aRangeDist.Last))
                {
                    aDist = aRangeDist.Distance;
                    break;
                }
            }
            if (aDist < RealLast())
            {
                TopoDS_Edge aEF = TopoDS.Edge(myDS.Shape(aPBF.Edge()));
                double aTolSum = aTolES + BRep_Tool.Tolerance(aEF);

                if (aDist <= aTolSum)
                {
                    theMPB.Add(aPBF);
                    PreparePostTreatFF(theInt, theCur, aPBF, theMSCPB, theMVI, theLPBC);

                    if (!thePBFacesMap.Find(aPBF, out NCollection_List<int> pFaces))
                    {
                        pFaces = thePBFacesMap.Bound(aPBF, new NCollection_List<int>());
                    }
                    if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                    {
                        pFaces.Append(nF);
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void ProcessExistingPaveBlocks(int theInt,
                                             int nF1,
                                             int nF2,
                                             NCollection_IndexedMap<BOPDS_PaveBlock> aMPBOnIn,
                                             BOPTools_BoxTree thePBTree,
                                             NCollection_IntDataMap<NCollection_List<int>> aDMBV,
                                             NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMSCPB,
                                             NCollection_DataMap<TopoDS_Shape, int> aMVI,
                                             NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap,
                                             NCollection_Map<BOPDS_PaveBlock> aMPB)
    {
        if (aDMBV.IsEmpty())
        {
            return;
        }
        //
        double aT = 0.0, dummy = 0.0;
        int nV, nE, iC, iFlag;
        NCollection_List<int>.Iterator aItLI = new();
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        BOPDS_InterfFF aFF = aFFs[theInt];
        NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.ChangeCurves();
        //
        BOPDS_FaceInfo aFI1 = myDS.FaceInfo(nF1);
        BOPDS_FaceInfo aFI2 = myDS.FaceInfo(nF2);
        //
        // (decision 784: OCCT iterates the DataMap<int, ...> in bucket order; the order of the curves decides the
        // order of the post-treated pave blocks)
        foreach (KeyValuePair<int, NCollection_List<int>> aItBV in aDMBV)
        {
            iC = aItBV.Key;
            NCollection_List<int> aLBV = aItBV.Value;
            //
            BOPDS_Curve aNC = aVC[iC];
            NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.ChangePaveBlocks();
            //
            aItLI.Initialize(aLBV);
            for (; aItLI.More(); aItLI.Next())
            {
                nV = aItLI.Value();
                BOPDS_ShapeInfo aSIV = myDS.ShapeInfo(nV);
                Bnd_Box aBoxV = aSIV.Box();
                TopoDS_Vertex aV = TopoDS.Vertex(aSIV.Shape());
                if (!aMVI.IsBound(aV))
                {
                    continue;
                }
                //
                BOPTools_BoxTreeSelector aSelector = new();
                aSelector.SetBox(Bnd_Tools.Bnd2BVH(aBoxV));
                aSelector.SetBVHSet(thePBTree);
                if (aSelector.Select() == 0)
                {
                    continue;
                }

                for (NCollection_List<int>.Iterator it = new(aSelector.Indices()); it.More(); it.Next())
                {
                    BOPDS_PaveBlock aPB = aMPBOnIn[it.Value()];
                    if (aPB.Pave1().Index() == nV || aPB.Pave2().Index() == nV)
                    {
                        continue;
                    }
                    //
                    if (aMPB.Contains(aPB))
                    {
                        continue;
                    }
                    nE = aPB.Edge();
                    BOPDS_ShapeInfo aSIE = myDS.ShapeInfo(nE);
                    TopoDS_Edge aE = TopoDS.Edge(aSIE.Shape());
                    //
                    iFlag = myContext!.ComputeVE(aV, aE, ref aT, ref dummy, myFuzzyValue);
                    if (iFlag == 0)
                    {
                        aMPB.Add(aPB);
                        PreparePostTreatFF(theInt, iC, aPB, aMSCPB, aMVI, aLPBC);

                        // Add faces to PB
                        bool bInF1 = (aFI1.PaveBlocksOn().Contains(aPB) || aFI1.PaveBlocksIn().Contains(aPB));
                        bool bInF2 = (aFI2.PaveBlocksOn().Contains(aPB) || aFI2.PaveBlocksIn().Contains(aPB));
                        if (!bInF1 || !bInF2)
                        {
                            // Face without pave block
                            int nF = bInF1 ? nF2 : nF1;
                            if (!thePBFacesMap.Find(aPB, out NCollection_List<int> pFaces))
                            {
                                pFaces = thePBFacesMap.Bound(aPB, new NCollection_List<int>());
                            }
                            // List is expected to be short, so we allow the check here
                            if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                            {
                                pFaces.Append(nF);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void UpdateExistingPaveBlocks(BOPDS_PaveBlock aPBf,
                                            NCollection_List<BOPDS_PaveBlock> aLPB,
                                            NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap)
    {
        if (aLPB.Extent() == 0)
        {
            return;
        }
        //
        int nE;
        bool bCB;
        BOPDS_PaveBlock aPB, aPB1, aPB2, aPB2n;
        BOPDS_CommonBlock? aCB;
        NCollection_List<BOPDS_PaveBlock>.Iterator aIt = new(), aIt1 = new(), aIt2 = new();
        //
        // 1. Remove old pave blocks
        BOPDS_CommonBlock? aCB1 = myDS!.CommonBlock(aPBf);
        bCB = aCB1 is not null;
        NCollection_List<BOPDS_PaveBlock> aLPB1 = new();
        //
        if (bCB)
        {
            aLPB1.Assign(aCB1!.PaveBlocks());
        }
        else
        {
            aLPB1.Append(aPBf);
        }
        aIt1.Initialize(aLPB1);
        for (; aIt1.More(); aIt1.Next())
        {
            aPB1 = aIt1.Value();
            nE = aPB1.OriginalEdge();
            //
            NCollection_List<BOPDS_PaveBlock> aLPB2 = myDS.ChangePaveBlocks(nE);
            aIt2.Initialize(aLPB2);
            for (; aIt2.More(); aIt2.Next())
            {
                aPB2 = aIt2.Value();
                if (aPB1 == aPB2)
                {
                    aLPB2.Remove(aIt2);
                    break;
                }
            }
        }
        //
        // 2. Update pave blocks
        if (bCB)
        {
            // Create new common blocks
            NCollection_List<BOPDS_PaveBlock> aLPBNew = new();
            NCollection_List<int> aFaces = aCB1!.Faces();
            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                BOPDS_PaveBlock aPBValue = aIt.Value();
                BOPDS_Pave aPBValuePave0 = aPBValue.Pave1(), aPBValuePave1 = aPBValue.Pave2();
                //
                aCB = new BOPDS_CommonBlock();
                aIt1.Initialize(aLPB1);
                for (; aIt1.More(); aIt1.Next())
                {
                    aPB2 = aIt1.Value();
                    nE = aPB2.OriginalEdge();
                    //
                    // Create new pave block
                    aPB2n = new BOPDS_PaveBlock();
                    if (aPBValue.OriginalEdge() == nE)
                    {
                        aPB2n.SetPave1(aPBValuePave0);
                        aPB2n.SetPave2(aPBValuePave1);
                    }
                    else
                    {
                        // For the different original edge compute the parameters of paves
                        BOPDS_Pave[] aPave = [new BOPDS_Pave(), new BOPDS_Pave()];

                        if (aPBValuePave0.Index() == aPBValuePave1.Index()
                            && aPB2.Pave1().Index() == aPB2.Pave2().Index())
                        {
                            // still closed
                            aPave[0].SetIndex(aPBValuePave0.Index());
                            aPave[0].SetParameter(aPB2.Pave1().Parameter());
                            aPave[1].SetIndex(aPBValuePave1.Index());
                            aPave[1].SetParameter(aPB2.Pave2().Parameter());
                        }
                        else
                        {
                            for (int i = 0; i < 2; ++i)
                            {
                                int nV = (i == 0 ? aPBValuePave0 : aPBValuePave1).Index();
                                aPave[i].SetIndex(nV);
                                if (nV == aPB2.Pave1().Index())
                                {
                                    aPave[i].SetParameter(aPB2.Pave1().Parameter());
                                }
                                else if (nV == aPB2.Pave2().Index())
                                {
                                    aPave[i].SetParameter(aPB2.Pave2().Parameter());
                                }
                                else
                                {
                                    // Compute the parameter by projecting the point
                                    TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
                                    TopoDS_Edge aEOr = TopoDS.Edge(myDS.Shape(nE));
                                    double aTOut = 0.0, aDist = 0.0;
                                    int iErr = myContext!.ComputeVE(aV, aEOr, ref aTOut, ref aDist, myFuzzyValue);
                                    if (iErr == 0)
                                    {
                                        aPave[i].SetParameter(aTOut);
                                    }
                                    else
                                    {
                                        // Unable to project - set the parameter of the closest boundary
                                        TopoDS_Vertex aV1 = TopoDS.Vertex(myDS.Shape(aPB2.Pave1().Index()));
                                        TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(aPB2.Pave2().Index()));
                                        //
                                        gp_Pnt aP = BRep_Tool.Pnt(aV);
                                        gp_Pnt aP1 = BRep_Tool.Pnt(aV1);
                                        gp_Pnt aP2 = BRep_Tool.Pnt(aV2);
                                        //
                                        double aDist1 = aP.SquareDistance(aP1);
                                        double aDist2 = aP.SquareDistance(aP2);
                                        //
                                        aPave[i].SetParameter(aDist1 < aDist2 ? aPB2.Pave1().Parameter()
                                                                              : aPB2.Pave2().Parameter());
                                    }
                                }
                            }
                        }
                        //
                        if (aPave[1].Parameter() < aPave[0].Parameter())
                        {
                            BOPDS_Pave aPaveTmp = aPave[0];
                            aPave[0] = aPave[1];
                            aPave[1] = aPaveTmp;
                        }
                        //
                        aPB2n.SetPave1(aPave[0]);
                        aPB2n.SetPave2(aPave[1]);
                    }
                    //
                    aPB2n.SetEdge(aPBValue.Edge());
                    aPB2n.SetOriginalEdge(nE);
                    aCB.AddPaveBlock(aPB2n);
                    myDS.SetCommonBlock(aPB2n, aCB);
                    myDS.ChangePaveBlocks(nE).Append(aPB2n);
                }
                aCB.SetFaces(aFaces);
                //
                BOPDS_PaveBlock aPBNew = aCB.PaveBlocks().First();
                aLPBNew.Append(aPBNew);
            }
            //
            aLPB.Assign(aLPBNew);
        }
        else
        {
            nE = aPBf.OriginalEdge();
            NCollection_List<BOPDS_PaveBlock> aLPBE = myDS.ChangePaveBlocks(nE);
            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                aPB = aIt.Value();
                aLPBE.Append(aPB);
            }
        }

        // Try to project the edge on the faces
        if (!thePBFacesMap.Find(aPBf, out NCollection_List<int> pLFaces))
        {
            return;
        }
        NCollection_List<int>.Iterator itLF = new(pLFaces);
        for (; itLF.More(); itLF.Next())
        {
            int nF = itLF.Value();
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));

            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                aPB = aIt.Value();
                if (aFI.PaveBlocksOn().Contains(aPB) || aFI.PaveBlocksIn().Contains(aPB))
                {
                    continue;
                }

                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(aPB.Edge()));
                //
                IntTools_EdgeFace anEF = new();
                anEF.SetEdge(aE);
                anEF.SetFace(aF);
                anEF.SetFuzzyValue(myFuzzyValue);
                anEF.SetRange(aPB.Pave1().Parameter(), aPB.Pave2().Parameter());
                anEF.SetContext(myContext);
                anEF.Perform();
                //
                NCollection_Sequence<IntTools_CommonPrt> aCPrts = anEF.CommonParts();
                bool bCoincide = (aCPrts.Length() == 1 && aCPrts.Value(1).Type() == TopAbs_EDGE);
                if (bCoincide)
                {
                    aCB = myDS.CommonBlock(aPB);
                    if (aCB is null)
                    {
                        aCB = new BOPDS_CommonBlock();
                        aCB.AddPaveBlock(aPB);
                        myDS.SetCommonBlock(aPB, aCB);
                    }
                    aCB.AddFace(nF);
                    aFI.ChangePaveBlocksIn().Add(aPB);
                }
            }
        }
    }

    //=================================================================================================

    protected void PutClosingPaveOnCurve(BOPDS_Curve aNC)
    {
        IntTools_Curve aIC = aNC.Curve();
        Geom_Curve? aC3D = aIC.Curve();
        // check 3d curve
        if (aC3D is null)
        {
            return;
        }

        // check bounds
        if (!aIC.HasBounds())
        {
            return;
        }

        // check closeness
        Span<double> aT = stackalloc double[2];
        gp_Pnt[] aP = [new gp_Pnt(), new gp_Pnt()];
        aIC.Bounds(ref aT[0], ref aT[1], ref aP[0], ref aP[1]);

        // Find the pave which has been put at one of the ends
        BOPDS_Pave aPave = new();
        // Index of the vertex put at one of the ends
        int nV = -1;
        // Keep the opposite parameter
        double aTOp = 0.0;
        // Keep the opposite bounding point
        gp_Pnt aPOp = new();

        BOPDS_PaveBlock aPB = aNC.ChangePaveBlock1();
        NCollection_List<BOPDS_Pave> aLP = aPB.ChangeExtPaves();
        NCollection_List<BOPDS_Pave>.Iterator aItLP = new(aLP);
        for (; aItLP.More() && (nV < 0); aItLP.Next())
        {
            aPave = aItLP.Value();
            double aTC = aPave.Parameter();
            for (int j = 0; j < 2; ++j)
            {
                if (Math.Abs(aTC - aT[j]) < Precision.PConfusion())
                {
                    nV = aPave.Index();
                    aTOp = (j == 0) ? aT[1] : aT[0];
                    aPOp = (j == 0) ? aP[1] : aP[0];
                    break;
                }
            }
        }

        if (nV < 0)
        {
            // No paves on the bounds of the curve
            return;
        }

        // Check if the curve is closed using the tolerance
        // of found vertex
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        double aTolV = BRep_Tool.Tolerance(aV);
        gp_Pnt aPV = BRep_Tool.Pnt(aV);
        // Tolerance for the point on the curve
        double aTolP = Max(aNC.Tolerance(), aNC.TangentialTolerance());
        aTolP += Precision.Confusion();

        double aDistVP = aPV.Distance(aPOp);
        if (aDistVP > aTolV + aTolP)
        {
            // Curve is not closed
            return;
        }

        // Check if there will be valid range on the curve
        double aFirst = 0.0, aLast = 0.0;
        double aNewTolV = Max(aTolV, aDistVP + BOPTools_AlgoTools.DTolerance());
        if (!BRepLib.FindValidRange(new GeomAdaptor_Curve(aC3D),
                                    aIC.Tolerance(),
                                    aT[0],
                                    aP[0],
                                    aNewTolV,
                                    aT[1],
                                    aP[1],
                                    aNewTolV,
                                    ref aFirst,
                                    ref aLast))
        {
            // No valid range
            return;
        }

        if (aNewTolV > aTolV)
        {
            int nVn = UpdateVertex(nV, aNewTolV);
            if (nVn != nV)
            {
                aPave.SetIndex(nVn);
                nV = nVn;
            }
            aTolV = BRep_Tool.Tolerance(TopoDS.Vertex(myDS.Shape(nV)));
        }

        // Add closing pave to the curve
        BOPDS_Pave aNewPave = new();
        aNewPave.SetIndex(nV);
        aNewPave.SetParameter(aTOp);
        aLP.Append(aNewPave);
    }

    //=================================================================================================

    protected void PreparePostTreatFF(int aInt,
                                      int aCur,
                                      BOPDS_PaveBlock aPB,
                                      NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMSCPB,
                                      NCollection_DataMap<TopoDS_Shape, int> aMVI,
                                      NCollection_List<BOPDS_PaveBlock> aLPBC)
    {
        int nV1, nV2;
        //
        aLPBC.Append(aPB);
        //
        aPB.Indices(out nV1, out nV2);
        TopoDS_Vertex aV1 = TopoDS.Vertex(myDS!.Shape(nV1));
        TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(nV2));
        TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(aPB.Edge()));
        // Keep info for post treatment
        BOPDS_CoupleOfPaveBlocks aCPB = new();
        aCPB.SetIndexInterf(aInt);
        aCPB.SetIndex(aCur);
        aCPB.SetPaveBlock1(aPB);
        //
        aMSCPB.Add(aE, aCPB);
        aMVI.Bind(aV1, nV1);
        aMVI.Bind(aV2, nV2);
    }

    //=================================================================================================

    protected bool CheckPlanes(int nF1, int nF2)
    {
        bool bToIntersect;
        int i, iCnt;
        //
        bToIntersect = false;
        //
        BOPDS_FaceInfo aFI1 = myDS!.ChangeFaceInfo(nF1);
        BOPDS_FaceInfo aFI2 = myDS.ChangeFaceInfo(nF2);
        //
        NCollection_IntMap aMVIn1 = aFI1.VerticesIn();
        NCollection_IntMap aMVOn1 = aFI1.VerticesOn();
        //
        iCnt = 0;
        for (i = 0; (i < 2 && !bToIntersect); ++i)
        {
            NCollection_IntMap aMV2 = (i == 0) ? aFI2.VerticesIn() : aFI2.VerticesOn();
            //
            foreach (int nV2 in aMV2)
            {
                if (aMVIn1.Contains(nV2) || aMVOn1.Contains(nV2))
                {
                    ++iCnt;
                    if (iCnt > 1)
                    {
                        bToIntersect = !bToIntersect;
                        break;
                    }
                }
            }
        }
        //
        return bToIntersect;
    }

    //=================================================================================================

    protected void UpdatePaveBlocks(NCollection_IntDataMap<int> aDMNewSD)
    {
        if (aDMNewSD.IsEmpty())
        {
            return;
        }
        //
        int nSp, aNbPBP, i, j;
        Span<int> nV = stackalloc int[2];
        Span<double> aT = stackalloc double[2];
        bool bCB, bRebuild;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        NCollection_Map<BOPDS_PaveBlock> aMPB = new();
        NCollection_IntMap aMicroEdges = new();
        //
        NCollection_List<BOPDS_PaveBlock> anAllPBs = new();

        // Get pave blocks of section edges
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFF = aFFs.Length();
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.Curves();
            int aNbC = aVNC.Length();
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVNC[j];
                NCollection_List<BOPDS_PaveBlock> aLPBC = aNC.PaveBlocks();
                aItPB.Initialize(aLPBC);
                for (; aItPB.More(); aItPB.Next())
                {
                    anAllPBs.Append(aItPB.Value());
                }
            }
        }

        // Get pave blocks from the pool
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS.ChangePaveBlocksPool();
        aNbPBP = aPBP.Length();
        for (i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                anAllPBs.Append(aItPB.Value());
            }
        }

        // Process all pave blocks
        aItPB.Initialize(anAllPBs);
        for (; aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);
            bCB = aCB is not null;
            if (bCB)
            {
                aPB = aCB!.PaveBlock1();
            }
            //
            if (aMPB.Add(aPB))
            {
                bRebuild = false;
                aPB.Indices(out nV[0], out nV[1]);
                aPB.Range(out aT[0], out aT[1]);
                // remember the fact if the edge had different vertices before substitution
                bool wasRegularEdge = (nV[0] != nV[1]);
                //
                for (j = 0; j < 2; ++j)
                {
                    if (aDMNewSD.IsBound(nV[j]))
                    {
                        BOPDS_Pave aPave = new();
                        //
                        nV[j] = aDMNewSD.Find(nV[j]);
                        aPave.SetIndex(nV[j]);
                        aPave.SetParameter(aT[j]);
                        //
                        bRebuild = true;
                        if (j == 0)
                        {
                            aPB.SetPave1(aPave);
                        }
                        else
                        {
                            aPB.SetPave2(aPave);
                        }
                    }
                }
                //
                if (bRebuild)
                {
                    int nE = aPB.Edge();
                    // Check if the Pave Block has the edge set
                    if (nE < 0)
                    {
                        // untouched edge
                        nE = aPB.OriginalEdge();
                    }
                    bool isDegEdge = myDS.ShapeInfo(nE).HasFlag();
                    if (wasRegularEdge && !isDegEdge && nV[0] == nV[1])
                    {
                        // now edge has the same vertex on both ends;
                        // check if it is not a regular closed curve.
                        FillShrunkData(aPB);
                        if (!aPB.HasShrunkData())
                        {
                            // micro edge, so mark it for removal
                            aMicroEdges.Add(nE);
                            continue;
                        }
                    }
                    nSp = SplitEdge(nE, nV[0], aT[0], nV[1], aT[1]);
                    if (bCB)
                    {
                        aCB!.SetEdge(nSp);
                    }
                    else
                    {
                        aPB.SetEdge(nSp);
                    }
                } // if (bRebuild) {
            } // if (aMPB.Add(aPB)) {
        } // for (; aItPB.More(); aItPB.Next()) {
        aMPB.Clear();

        if (aMicroEdges.Extent() != 0)
        {
            RemovePaveBlocks(aMicroEdges);
        }
    }

    //=================================================================================================

    protected void RemovePaveBlocks(NCollection_IntMap theEdges)
    {
        // Remove all pave blocks referring to input edges:
        //
        // 1. from the Pave Blocks Pool
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        int aNbPBP = aPBP.Length(), i;
        for (i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            //
            NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPB);
            while (aItPB.More())
            {
                BOPDS_PaveBlock aPB = aItPB.Value();
                if (theEdges.Contains(aPB.Edge()))
                {
                    aLPB.Remove(aItPB);
                }
                else
                {
                    aItPB.Next();
                }
            }
        }

        // 2. from section curves
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS.InterfFF();
        int aNbFF = aFFs.Length(), j;
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            // remove from Section pave blocks
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
            int aNbC = aVNC.Length();
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVNC[j];
                NCollection_List<BOPDS_PaveBlock> aLPB = aNC.ChangePaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPB);
                while (aItPB.More())
                {
                    BOPDS_PaveBlock aPB = aItPB.Value();
                    if (theEdges.Contains(aPB.Edge()))
                    {
                        aLPB.Remove(aItPB);
                    }
                    else
                    {
                        aItPB.Next();
                    }
                }
            }
        }

        // 3. From Face Info
        for (i = 0; i < myDS.NbSourceShapes(); ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            if (!aSI.HasReference())
            {
                continue;
            }

            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(i);
            NCollection_IndexedMap<BOPDS_PaveBlock>[] aIMPB = [aFI.ChangePaveBlocksIn(),
                                                               aFI.ChangePaveBlocksOn(),
                                                               aFI.ChangePaveBlocksSc()];
            for (int k = 0; k < 3; k++)
            {
                int aNbPB = aIMPB[k].Extent(), m;
                for (m = 1; m <= aNbPB; ++m)
                {
                    BOPDS_PaveBlock aPB = aIMPB[k].FindKey(m);
                    if (theEdges.Contains(aPB.Edge()))
                    {
                        break;
                    }
                }
                if (m <= aNbPB)
                {
                    NCollection_IndexedMap<BOPDS_PaveBlock> aMPBCopy = new(aIMPB[k]);
                    aIMPB[k].Clear();
                    for (m = 1; m <= aNbPB; ++m)
                    {
                        BOPDS_PaveBlock aPB = aMPBCopy[m];
                        if (!theEdges.Contains(aPB.Edge()))
                        {
                            aIMPB[k].Add(aPB);
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : ToleranceFF
    // purpose  : Computes the TolFF according to the tolerance value and
    //           types of the faces.
    //=======================================================================
    private static double ToleranceFF(BRepAdaptor_Surface aBAS1, BRepAdaptor_Surface aBAS2)
    {
        double aTol1 = aBAS1.Tolerance();
        double aTol2 = aBAS2.Tolerance();
        double aTolFF = Max(aTol1, aTol2);
        //
        bool isAna1, isAna2;
        isAna1 = (aBAS1.GetType() == GeomAbs_Plane || aBAS1.GetType() == GeomAbs_Cylinder
                  || aBAS1.GetType() == GeomAbs_Cone || aBAS1.GetType() == GeomAbs_Sphere
                  || aBAS1.GetType() == GeomAbs_Torus);
        //
        isAna2 = (aBAS2.GetType() == GeomAbs_Plane || aBAS2.GetType() == GeomAbs_Cylinder
                  || aBAS2.GetType() == GeomAbs_Cone || aBAS2.GetType() == GeomAbs_Sphere
                  || aBAS2.GetType() == GeomAbs_Torus);
        //
        if (!isAna1 || !isAna2)
        {
            aTolFF = Max(aTolFF, 5.0e-6);
        }
        return aTolFF;
    }

    //=================================================================================================

    protected void UpdateBlocksWithSharedVertices()
    {
        if (!myNonDestructive)
        {
            return;
        }
        //
        int aNbFF;
        //
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        aNbFF = aFFs.Length();
        if (aNbFF == 0)
        {
            return;
        }
        //
        bool bOnCurve, bHasShapeSD;
        int i, nF1, nF2, aNbC, j, nVSD = 0;
        double aTolV;
        NCollection_IntMap aMF = new();
        //
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            //
            NCollection_DynamicArray<BOPDS_Curve> aVC = aFF.ChangeCurves();
            aNbC = aVC.Length();
            if (aNbC == 0)
            {
                continue;
            }
            //
            aFF.Indices(out nF1, out nF2);
            //
            if (aMF.Add(nF1))
            {
                myDS.UpdateFaceInfoOn(nF1);
            }
            if (aMF.Add(nF2))
            {
                myDS.UpdateFaceInfoOn(nF2);
            }
            //
            // Collect old vertices that are shared for nF1, nF2 ->aMI;
            NCollection_IntMap aMI = new();
            //
            BOPDS_FaceInfo aFI1 = myDS.ChangeFaceInfo(nF1);
            BOPDS_FaceInfo aFI2 = myDS.ChangeFaceInfo(nF2);
            //
            NCollection_IntMap aMVOn1 = aFI1.VerticesOn();
            NCollection_IntMap aMVIn1 = aFI1.VerticesIn();
            NCollection_IntMap aMVOn2 = aFI2.VerticesOn();
            NCollection_IntMap aMVIn2 = aFI2.VerticesIn();
            //
            for (j = 0; j < 2; ++j)
            {
                NCollection_IntMap aMV1 = (j == 0) ? aMVOn1 : aMVIn1;
                foreach (int nV in aMV1)
                {
                    if (myDS.IsNewShape(nV))
                    {
                        continue;
                    }
                    if (aMVOn2.Contains(nV) || aMVIn2.Contains(nV))
                    {
                        aMI.Add(nV);
                    }
                }
            }
            //
            // Try to put vertices aMI on curves
            for (j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aNC = aVC[j];
                double aTolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
                //
                // (decision 784: the order of aMI decides the order of the new vertices UpdateVertex appends)
                foreach (int nV in aMI)
                {
                    //
                    bHasShapeSD = myDS.HasShapeSD(nV, ref nVSD);
                    if (bHasShapeSD)
                    {
                        continue;
                    }
                    //
                    bOnCurve = EstimatePaveOnCurve(nV, aNC, aTolR3D);
                    if (!bOnCurve)
                    {
                        continue;
                    }
                    //
                    TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
                    aTolV = BRep_Tool.Tolerance(aV);
                    //
                    UpdateVertex(nV, aTolV);
                    myDS.InitPaveBlocksForVertex(nV);
                }
            } // for (j=0; j<aNbC; ++j) {
        } // for (i=0; i<aNbFF; ++i) {
        //
        UpdateCommonBlocksWithSDVertices();
    }

    //=================================================================================================

    protected bool EstimatePaveOnCurve(int nV, BOPDS_Curve aNC, double aTolR3D)
    {
        bool bIsVertexOnLine;
        double aT = 0.0;
        //
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        IntTools_Curve aIC = aNC.Curve();
        //
        bIsVertexOnLine = myContext!.IsVertexOnLine(aV, aIC, aTolR3D, ref aT);
        return bIsVertexOnLine;
    }

    //=================================================================================================

    protected void CorrectToleranceOfSE()
    {
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        NCollection_IndexedDataMap<int, NCollection_List<BOPDS_PaveBlock>> aMVIPBs = new();
        NCollection_IntMap aMVIToReduce = new();
        // Fence map to avoid repeated checking of the same edge
        NCollection_Map<BOPDS_PaveBlock> aMPB = new();
        //
        // 1. iterate on all sections F-F
        int aNb = aFFs.Length(), i;
        for (i = 0; i < aNb; ++i)
        {
            BOPDS_InterfFF aFF = aFFs[i];
            //
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.ChangeCurves();
            int aNbC = aVNC.Length(), k;
            for (k = 0; k < aNbC; ++k)
            {
                BOPDS_Curve aNC = aVNC[k];
                NCollection_List<BOPDS_PaveBlock> aLPB = aNC.ChangePaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
                for (; aItLPB.More();)
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    if (!aPB.HasEdge(out int nE))
                    {
                        aLPB.Remove(aItLPB);
                        continue;
                    }
                    //
                    if (!aMPB.Add(aPB))
                    {
                        aItLPB.Next();
                        continue;
                    }
                    //
                    bool bIsReduced = false;
                    if (aPB.OriginalEdge() < 0)
                    {
                        // It is possible that due to small angle between faces the
                        // common zone between faces can be large and the tangential
                        // tolerance of the curve will be large as well.
                        // Here we're trying to reduce the tolerance of the section
                        // edge using the valid tolerance of the edge.
                        // Note, that if the pave block has created common block with
                        // other edges its valid tolerance could have been changed to
                        // cover all edges in common block (see PostTreatFF() method).
                        double aTolC = aNC.Tolerance();
                        double aTolTang = aNC.TangentialTolerance();
                        if (aTolC < aTolTang)
                        {
                            TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                            double aTolE = BRep_Tool.Tolerance(aE);
                            if (aTolC < aTolE)
                            {
                                // reduce edge tolerance (a new section edge: never an argument's)
                                ((BRep_TEdge)aE.TShape()!).Tolerance(aTolC);
                                bIsReduced = true;
                            }
                        }
                    }
                    //
                    // fill in the map vertex index - pave blocks
                    for (int j = 0; j < 2; j++)
                    {
                        int nV = (j == 0 ? aPB.Pave1().Index() : aPB.Pave2().Index());
                        myDS.HasShapeSD(nV, ref nV);
                        int anIndex = aMVIPBs.FindIndex(nV);
                        NCollection_List<BOPDS_PaveBlock> pPBList = anIndex > 0
                            ? aMVIPBs.ChangeFromIndex(anIndex)
                            : aMVIPBs.ChangeFromIndex(aMVIPBs.Add(nV, new NCollection_List<BOPDS_PaveBlock>()));
                        pPBList.Append(aPB);
                        if (bIsReduced)
                        {
                            aMVIToReduce.Add(nV);
                        }
                    }
                    aItLPB.Next();
                }
            }
        }
        //
        if (aMVIToReduce.IsEmpty())
        {
            return;
        }
        //
        // 2. try to reduce tolerances of connected vertices
        // 2.1 find all other edges containing these connected vertices to avoid
        //     reducing the tolerance to the value less than the tolerances of edges,
        //     i.e. minimal tolerance for the vertex is the max tolerance of the
        //     edges containing this vertex
        NCollection_IntDataMap<double> aMVITol = new();
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS.ChangePaveBlocksPool();
        aNb = aPBP.Length();
        Span<int> nVs = stackalloc int[2];
        for (i = 0; i < aNb; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                if (!aPB.HasEdge(out int nE))
                {
                    continue;
                }
                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                double aTolE = BRep_Tool.Tolerance(aE);
                //
                aPB.Indices(out nVs[0], out nVs[1]);
                //
                for (int j = 0; j < 2; j++)
                {
                    if (aMVIToReduce.Contains(nVs[j]))
                    {
                        if (!aMVITol.IsBound(nVs[j]))
                        {
                            aMVITol.Bind(nVs[j], aTolE);
                        }
                        else if (aTolE > aMVITol.Find(nVs[j]))
                        {
                            aMVITol.ChangeFind(nVs[j]) = aTolE;
                        }
                        NCollection_List<BOPDS_PaveBlock> aPBList = aMVIPBs.ChangeFromKey(nVs[j]);
                        aPBList.Append(aPB);
                    }
                }
            }
        }
        //
        // 2.2 reduce tolerances if possible
        aNb = aMVIPBs.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            int nV = aMVIPBs.FindKey(i);
            if (!aMVIToReduce.Contains(nV))
            {
                continue;
            }
            //
            TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
            double aTolV = BRep_Tool.Tolerance(aV);
            double aMaxTol = aMVITol.Find(nV, out double pMaxTol) ? pMaxTol : 0.0;
            // it makes no sense to compute the real tolerance if it is
            // impossible to reduce the tolerance at least 0.1% of the current value
            if (aTolV - aMaxTol < 0.001 * aTolV)
            {
                continue;
            }
            //
            // compute the maximal distance from the vertex to the adjacent edges
            gp_Pnt aP = BRep_Tool.Pnt(aV);
            //
            // Avoid repeated checks
            NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();
            //
            NCollection_List<BOPDS_PaveBlock> aLPB = aMVIPBs.FindFromIndex(i);
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                if (!aMPBFence.Add(aPB))
                {
                    continue;
                }
                int nE = aPB.Edge();
                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                BRepAdaptor_Curve aC = new(aE);
                for (int iPave = 0; iPave < 2; ++iPave)
                {
                    BOPDS_Pave aPave = iPave == 0 ? aPB.Pave1() : aPB.Pave2();
                    int nVSD = aPave.Index();
                    myDS.HasShapeSD(nVSD, ref nVSD);
                    if (nVSD != nV)
                    {
                        continue;
                    }
                    //
                    gp_Pnt aPonE = aC.Value(aPave.Parameter());
                    double aDist = aP.Distance(aPonE);
                    aDist += BRep_Tool.Tolerance(aE);
                    if (aDist > aMaxTol)
                    {
                        aMaxTol = aDist;
                    }
                }
            }
            //
            if (aMaxTol < aTolV)
            {
                // OCCT reduces the vertex's tolerance in place, even for an argument vertex in non-destructive mode
                // (a vertex of a section edge that is an untouched argument vertex): a frozen vertex throws here.
                ((BRep_TVertex)aV.TShape()!).Tolerance(aMaxTol);
            }
        }
    }

    //=================================================================================================

    protected void PutSEInOtherFaces(Message_ProgressRange theRange)
    {
        // Try to intersect each section edge with the faces
        // not participated in its creation

        // Get all section edges
        NCollection_IndexedMap<BOPDS_PaveBlock> aMPBScAll = new();

        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFF = aFFs.Length();
        using var aPS = new Message_ProgressScope(theRange, null, 1);
        for (int i = 0; i < aNbFF; ++i)
        {
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFFs[i].Curves();
            int aNbC = aVNC.Length();
            for (int j = 0; j < aNbC; ++j)
            {
                NCollection_List<BOPDS_PaveBlock> aLPBC = aVNC[j].PaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPBC);
                for (; aItPB.More(); aItPB.Next())
                {
                    aMPBScAll.Add(aItPB.Value());
                }
            }
        }
        // Perform intersection of collected pave blocks
        ForceInterfEF(aMPBScAll, aPS.Next(), false);
    }

    //=================================================================================================

    protected void RemoveMicroSectionEdges(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB,
                                           NCollection_IndexedMap<BOPDS_PaveBlock> theMicroPB)
    {
        if (theMSCPB.IsEmpty())
        {
            // no section edges
            return;
        }

        // Get all F/F interferences
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();

        // Build the new map of section edges avoiding the micro edges
        NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aSEPBMap = new(TopTools_ShapeMapHasher.Instance);
        // Analyze all section edges
        int aNbCPB = theMSCPB.Extent();
        for (int i = 1; i <= aNbCPB; ++i)
        {
            TopoDS_Shape aSI = theMSCPB.FindKey(i);
            BOPDS_CoupleOfPaveBlocks aCPB = theMSCPB[i];

            if (aSI.ShapeType() != TopAbs_EDGE)
            {
                // Not an edge
                aSEPBMap.Add(aSI, aCPB);
                continue;
            }

            // Get pave block for analysis
            BOPDS_PaveBlock aPB = aCPB.PaveBlock1()!;
            if (aPB.HasEdge())
            {
                // Not a real section edge
                aSEPBMap.Add(aSI, aCPB);
                continue;
            }

            if (!BOPTools_AlgoTools.IsMicroEdge(TopoDS.Edge(aSI), myContext, false))
            {
                // Normal edge
                aSEPBMap.Add(aSI, aCPB);
                continue;
            }

            // Micro edge is found, avoid it in the <theMSCPB> map
            // and remove from the F/F Intersection info structure

            // Get F/F interference which created this micro edge
            BOPDS_InterfFF aFF = aFFs[aCPB.IndexInterf()];
            // Get curve from which this edge has been created
            BOPDS_Curve aCurve = aFF.ChangeCurves()[aCPB.Index()];
            // Get all section pave blocks created from this curve
            NCollection_List<BOPDS_PaveBlock> aLPBC = aCurve.ChangePaveBlocks();
            // Remove pave block from the list
            for (NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aLPBC); it.More(); it.Next())
            {
                if (it.Value() == aPB)
                {
                    aLPBC.Remove(it);
                    break;
                }
            }

            // Add the pave block of "micro" edge into outgoing map for
            // unification of its vertices in the PostTreatFF method
            theMicroPB.Add(aPB);
        }

        // Overwrite the old map if necessary (OCCT's assignment: the caller's map takes the new contents)
        if (aSEPBMap.Extent() != theMSCPB.Extent())
        {
            theMSCPB.Clear();
            for (int i = 1; i <= aSEPBMap.Extent(); ++i)
            {
                theMSCPB.Add(aSEPBMap.FindKey(i), aSEPBMap.FindFromIndex(i));
            }
        }
    }

    //=================================================================================================

    protected void RemoveMicroEdges()
    {
        // Fence map
        NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();
        // Resulting map of micro edges
        NCollection_IntMap aMicroEdges = new();
        // Check all pave blocks from the pool to find the micro edges
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        int aNbPBP = aPBP.Length();
        for (int i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            if (aLPB.Extent() < 2)
            {
                // No splits
                continue;
            }

            if (myDS.ShapeInfo(aLPB.First().OriginalEdge()).HasFlag())
            {
                continue;
            }

            NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aLPB);
            for (; it.More(); it.Next())
            {
                BOPDS_PaveBlock aPB = it.Value();
                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);

                if (aMPBFence.Add(aPBR))
                {
                    aPBR.Indices(out int nV1, out int nV2);
                    if (nV1 == nV2)
                    {
                        // Check if it has the valid range
                        FillShrunkData(aPBR);
                        if (!aPBR.HasShrunkData())
                        {
                            aMicroEdges.Add(aPBR.Edge());
                        }
                    }
                }
            }
        }
        RemovePaveBlocks(aMicroEdges);
    }
}
