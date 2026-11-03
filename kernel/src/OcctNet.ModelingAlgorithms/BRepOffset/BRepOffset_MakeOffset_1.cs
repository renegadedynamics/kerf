// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// This is the implementation of the extension of the 3D offset algorithm
// to work in mode Complete and Join Type Intersection.
// Currently only the Planar cases are supported.

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

// OFFSET_DEBUG blocks (the BRepAlgoAPI_Check of the edges and the compounds built only to be shown) are left out.

//=======================================================================
// function : BRepOffset_BuildOffsetFaces
// purpose  : Auxiliary local class that is used here for building splits
//           of offset faces, that are further used for building volumes.
//=======================================================================

/// <summary>
/// Builds the splits of the offset faces in the Complete mode with the Intersection join (planar shapes), used to
/// build the volumes of the offset. A local class of BRepOffset_MakeOffset_1.cxx.
/// </summary>
/// <remarks>
/// A working object of the offset (decision 21): it writes into the offset's own working shapes (the AsDes edges, the
/// splits it builds, vertex tolerances of micro edges), never into the caller's. Maps that OCCT holds by value inside
/// other maps (lists, maps and indexed maps of shapes) are copied wherever OCCT copies them. Shape-keyed maps iterate in
/// insertion order where OCCT's iterate by address (decision 1032).
/// </remarks>
internal sealed class BRepOffset_BuildOffsetFaces
{
    //=======================================================================
    // function : AddToContainer
    // purpose  : Set of methods to add a shape into container
    //=======================================================================
    // OCCT.NET: the overloads of OCCT's templates (TakeModified) dispatch on the container's type at run time.
    private static bool AddToContainer(in TopoDS_Shape theS, object theContainer)
    {
        switch (theContainer)
        {
            case NCollection_List<TopoDS_Shape> aList:
                aList.Append(theS);
                return true;
            case NCollection_Map<TopoDS_Shape> aMap:
                return aMap.Add(theS);
            case NCollection_IndexedMap<TopoDS_Shape> anIMap:
            {
                int aNb = anIMap.Extent();
                int anInd = anIMap.Add(theS);
                return anInd > aNb;
            }
            case TopoDS_Shape aSOut:
                new BRep_Builder().Add(aSOut, theS);
                return true;
            case TopoDS_Compound aCOut:
                new BRep_Builder().Add(aCOut, theS);
                return true;
            default:
                throw new ProgramErrorException("BRepOffset_BuildOffsetFaces::AddToContainer");
        }
    }

    private static void AddToContainer(in TopoDS_Shape theKey, in TopoDS_Shape theValue,
                                       NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        if (theMap.Find(theKey, out NCollection_List<TopoDS_Shape> pList))
        {
            pList.Append(theValue);
        }
        else
        {
            NCollection_List<TopoDS_Shape> aList = new();
            aList.Append(theValue);
            theMap.Bind(theKey, aList);
        }
    }

    //=======================================================================
    // function : TakeModified
    // purpose  : Check if the shape has images in the given images map.
    //           Puts in the output map either the images or the shape itself.
    //=======================================================================
    private static bool TakeModified(in TopoDS_Shape theS,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages,
                                     object theContainer, object? theMFence)
    {
        if (theImages.Find(theS, out NCollection_List<TopoDS_Shape> pLSIm))
        {
            NCollection_List<TopoDS_Shape>.Iterator itLSIm = new(pLSIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                TopoDS_Shape aSIm = itLSIm.Value();
                if (theMFence is null || AddToContainer(aSIm, theMFence))
                {
                    AddToContainer(aSIm, theContainer);
                }
            }
            return true;
        }
        else
        {
            if (theMFence is null || AddToContainer(theS, theMFence))
            {
                AddToContainer(theS, theContainer);
            }
            return false;
        }
    }

    private static bool TakeModified(in TopoDS_Shape theS,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages,
                                     object theMapOut)
    {
        return TakeModified(theS, theImages, theMapOut, null);
    }

    //=======================================================================
    // function : hasData
    // purpose  : Checks if container has any data in it
    //=======================================================================
    private static bool hasData<TK, TV>(NCollection_DataMap<TK, TV>? theData) where TK : notnull =>
        theData is not null && !theData.IsEmpty();

    private static bool hasData(NCollection_List<TopoDS_Shape>? theData) => theData is not null && !theData.IsEmpty();

    //=======================================================================
    // function : AppendToList
    // purpose  : Add to a list only unique elements
    //=======================================================================
    private static void AppendToList(NCollection_List<TopoDS_Shape> theList, in TopoDS_Shape theShape)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(theList); it.More(); it.Next())
        {
            TopoDS_Shape aS = it.Value();
            if (aS.IsSame(theShape))
            {
                return;
            }
        }
        theList.Append(theShape);
    }

    //=======================================================================
    // function : ProcessMicroEdge
    // purpose  : Checking if the edge is micro edge
    //=======================================================================
    private static bool ProcessMicroEdge(in TopoDS_Edge theEdge, IntTools_Context theCtx)
    {
        TopExp.Vertices(theEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        if (aV1.IsNull() || aV2.IsNull())
        {
            return false;
        }

        bool bMicro = BOPTools_AlgoTools.IsMicroEdge(theEdge, theCtx);
        if (bMicro && new BRepAdaptor_Curve(theEdge).GetType() == Line)
        {
            double aLen = BRep_Tool.Pnt(aV1).Distance(BRep_Tool.Pnt(aV2));
            new BRep_Builder().UpdateVertex(aV1, aLen / 2.0);
            new BRep_Builder().UpdateVertex(aV2, aLen / 2.0);
        }

        return bMicro;
    }

    // OCCT's BOPAlgo_Builder::Modified (a const reference to the history's list), as a list of its own.
    private static NCollection_List<TopoDS_Shape> ModifiedOf(BOPAlgo_BuilderShape theGF, in TopoDS_Shape theS) =>
        new(theGF.Modified(theS));

    //=================================================================================================

    private static void UpdateOrigins(NCollection_List<TopoDS_Shape> theLA,
                                      NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theOrigins,
                                      BOPAlgo_Builder theGF)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator aItA = new(theLA); aItA.More(); aItA.Next())
        {
            TopoDS_Shape aS = aItA.Value();

            NCollection_List<TopoDS_Shape> aLSIm = ModifiedOf(theGF, aS);
            if (aLSIm.IsEmpty())
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> aLSEmpt = new();
            if (!theOrigins.Find(aS, out NCollection_List<TopoDS_Shape> pLS))
            {
                pLS = aLSEmpt;
                pLS.Append(aS);
            }

            for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLSIm); aIt.More(); aIt.Next())
            {
                TopoDS_Shape aSIm = aIt.Value();
                if (theOrigins.Find(aSIm, out NCollection_List<TopoDS_Shape> pLSOr))
                {
                    // merge two lists
                    for (NCollection_List<TopoDS_Shape>.Iterator aIt1 = new(pLS); aIt1.More(); aIt1.Next())
                    {
                        AppendToList(pLSOr, aIt1.Value());
                    }
                }
                else
                {
                    theOrigins.Bind(aSIm, new NCollection_List<TopoDS_Shape>(pLS));
                }
            }
        }
    }

    //=================================================================================================

    private static void UpdateImages(NCollection_List<TopoDS_Shape> theLA,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages,
                                     BOPAlgo_Builder theGF, NCollection_Map<TopoDS_Shape> theModified)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLA); aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();

            if (!theImages.Find(aS, out NCollection_List<TopoDS_Shape> pLSIm))
            {
                NCollection_List<TopoDS_Shape> aLSIm = ModifiedOf(theGF, aS);
                if (aLSIm.Extent() != 0)
                {
                    theImages.Bind(aS, aLSIm);
                    theModified.Add(aS);
                }
                continue;
            }

            NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape> aLSImNew = new();

            bool bModified = false;

            // check modifications of the images
            for (NCollection_List<TopoDS_Shape>.Iterator aIt1 = new(pLSIm); aIt1.More(); aIt1.Next())
            {
                TopoDS_Shape aSIm = aIt1.Value();
                bModified |= TakeModified(aSIm, theGF.Images(), aLSImNew, aMFence);
            }

            if (bModified)
            {
                pLSIm.Assign(aLSImNew);
                theModified.Add(aS);
            }
        }
    }

    //=======================================================================
    // function : FindCommonParts
    // purpose  : Looking for the parts of type <theType> contained in both lists
    //=======================================================================
    private static void FindCommonParts(NCollection_List<TopoDS_Shape> theLS1, NCollection_List<TopoDS_Shape> theLS2,
                                        NCollection_List<TopoDS_Shape> theLSC, TopAbs_ShapeEnum theType = TopAbs_EDGE)
    {
        // map shapes in the first list
        NCollection_IndexedMap<TopoDS_Shape> aMS1 = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLS1); aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            TopExp.MapShapes(aS, theType, aMS1);
        }
        if (aMS1.IsEmpty())
        {
            return;
        }

        // check for such shapes in the other list
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLS2); aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();

            TopExp_Explorer aExp = new(aS, theType);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aST = aExp.Current();
                if (aMS1.Contains(aST) && aMFence.Add(aST))
                {
                    theLSC.Append(aST);
                }
            }
        }
    }

    //=======================================================================
    // function : NbPoints
    // purpose  : Defines number of sample points to get average direction of the edge
    //=======================================================================
    private static int NbPoints(in TopoDS_Edge theEdge)
    {
        BRepAdaptor_Curve aBAC = new(theEdge);
        switch (aBAC.GetType())
        {
            case Line:
                return 1;
            default:
                return 11;
        }
    }

    //=======================================================================
    // function : FindShape
    // purpose  : Looking for the same sub-shape in the shape
    //=======================================================================
    private static bool FindShape(in TopoDS_Shape theSWhat, in TopoDS_Shape theSWhere, BRepOffset_Analyse? theAnalyse,
                                  ref TopoDS_Shape theRes)
    {
        bool bFound = false;
        TopAbs_ShapeEnum aType = theSWhat.ShapeType();
        TopExp_Explorer aExp = new(theSWhere, aType);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aS = aExp.Current();
            if (aS.IsSame(theSWhat))
            {
                theRes = aS;
                bFound = true;
                break;
            }
        }

        if (!bFound && theAnalyse is not null)
        {
            NCollection_List<TopoDS_Shape>? pLD = theAnalyse.Descendants(theSWhere);
            if (pLD is not null)
            {
                for (NCollection_List<TopoDS_Shape>.Iterator it = new(pLD); it.More(); it.Next())
                {
                    TopoDS_Shape aS = it.Value();
                    if (aS.IsSame(theSWhat))
                    {
                        theRes = aS;
                        bFound = true;
                        break;
                    }
                }
            }
        }

        return bFound;
    }

    //=======================================================================
    // function : BuildSplitsOfTrimmedFace
    // purpose  : Building the splits of offset face
    //=======================================================================
    private static void BuildSplitsOfTrimmedFace(in TopoDS_Face theFace, in TopoDS_Shape theEdges,
                                                 NCollection_List<TopoDS_Shape> theLFImages,
                                                 ProgressRange theRange)
    {
        BOPAlgo_Splitter aSplitter = new();
        //
        aSplitter.AddArgument(theFace);
        aSplitter.AddArgument(theEdges);
        aSplitter.SetToFillHistory(false);
        aSplitter.Perform(theRange);
        if (aSplitter.HasErrors())
        {
            return;
        }
        //
        // splits of the offset shape
        for (TopExp_Explorer anExp = new(aSplitter.InternalShape(), TopAbs_FACE); anExp.More(); anExp.Next())
        {
            theLFImages.Append(anExp.Current());
        }
    }

    //=======================================================================
    // function : BuildSplitsOfFace
    // purpose  : Building the splits of offset face
    //=======================================================================
    private static void BuildSplitsOfFace(in TopoDS_Face theFace, in TopoDS_Shape theEdges,
                                          NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theFacesOrigins,
                                          NCollection_List<TopoDS_Shape> theLFImages)
    {
        theLFImages.Clear();
        //
        // take edges to split the face
        NCollection_List<TopoDS_Shape> aLE = new();
        TopExp_Explorer aExp = new(theEdges, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            aE.Orientation(TopAbs_FORWARD);
            aLE.Append(aE);
            aE.Orientation(TopAbs_REVERSED);
            aLE.Append(aE);
        }
        //
        TopoDS_Face aFF = theFace;
        TopAbs_Orientation anOr = theFace.Orientation();
        aFF.Orientation(TopAbs_FORWARD);
        //
        // build pcurves for edges on the face
        BRepLib.BuildPCurveForEdgesOnPlane(aLE, aFF);
        //
        // build splits of faces
        BOPAlgo_BuilderFace aBF = new();
        aBF.SetFace(aFF);
        aBF.SetShapes(aLE);
        aBF.Perform();
        if (aBF.HasErrors())
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape> aLFSp = aBF.Areas();
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLFSp);
        for (; aItLF.More(); aItLF.Next())
        {
            ref TopoDS_Shape aFSp = ref aItLF.ChangeValue();
            aFSp.Orientation(anOr);
            theLFImages.Append(aFSp);
            //
            theFacesOrigins.Bind(aFSp, theFace);
        }
    }

    //=======================================================================
    // function : GetAverageTangent
    // purpose  : Computes average tangent vector along the curve
    //=======================================================================
    private static Vector3D GetAverageTangent(in TopoDS_Shape theS, int theNbP)
    {
        Vector3D aVA = new();
        TopExp_Explorer aExp = new(theS, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            //
            Geom_Curve aC = BRep_Tool.Curve(aE, out double aT1, out double aT2)!;
            //
            Vector3D aVSum = new();
            double aT = aT1;
            double aDt = (aT2 - aT1) / theNbP;
            while (aT <= aT2)
            {
                aC.D1(aT, out Point3D aP, out Vector3D aV);
                aVSum += aV.Normalized();
                aT += aDt;
            }
            //
            if (aE.Orientation() == TopAbs_REVERSED)
            {
                aVSum.Reverse();
            }
            //
            aVA += aVSum;
        }
        return aVA;
    }

    // Input data
    private NCollection_List<TopoDS_Shape>? myFaces; //!< Input faces which have to be split
    private BRepAlgo_AsDes? myAsDes; //!< Ascendants/descendants of the edges faces
    private BRepOffset_Analyse? myAnalyzer; //!< Analyzer of the input parameters

    //!< Origins of the offset edges (binding between offset edge and original edge)
    private NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>? myEdgesOrigins;
    //!< Origins of the offset faces (binding between offset face and original face)
    private NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>? myFacesOrigins;
    //!< Binding between trimmed and infinite edge
    private NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>? myETrimEInf;

    // Intermediate data
    //!< Images of the offset edges
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOEImages = new(TopTools_ShapeMapHasher.Instance);
    //!< Origins of the splits of offset edges
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOEOrigins = new(TopTools_ShapeMapHasher.Instance);
    //!< Images of the offset edges
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOFImages = new(TopTools_ShapeMapHasher.Instance);

    //!< Edges considered invalid (orientation is changed) in some split of face
    private NCollection_IndexedMap<TopoDS_Shape> myInvalidEdges = new(TopTools_ShapeMapHasher.Instance);
    //!< Edges considered valid (orientation is kept) in some split of face
    private readonly NCollection_IndexedMap<TopoDS_Shape> myValidEdges = new(TopTools_ShapeMapHasher.Instance);
    //!< Edges considered inverted (vertices swapped) in some split of face
    private readonly NCollection_IndexedMap<TopoDS_Shape> myInvertedEdges = new(TopTools_ShapeMapHasher.Instance);
    //!< Splits of edges to be avoided when building splits of faces
    private NCollection_IndexedMap<TopoDS_Shape> myEdgesToAvoid = new(TopTools_ShapeMapHasher.Instance);
    //!< Edges marked invalid on the current step and to be avoided on the next step
    private readonly NCollection_Map<TopoDS_Shape> myLastInvEdges = new(TopTools_ShapeMapHasher.Instance);
    //!< Edges to be used for building splits
    private readonly NCollection_Map<TopoDS_Shape> myModifiedEdges = new(TopTools_ShapeMapHasher.Instance);
    //!< Edges located fully inside solids built from the splits of offset faces
    private readonly NCollection_IndexedMap<TopoDS_Shape> myInsideEdges = new(TopTools_ShapeMapHasher.Instance);

    //!< Invalid faces - splits of offset faces consisting of invalid edges
    private NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myInvalidFaces = new(TopTools_ShapeMapHasher.Instance);
    //!< Artificially invalid faces - valid faces intentionally marked invalid to be rebuilt on the future steps in the
    //!  situations when invalid edges are present, but invalid faces not
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> myArtInvalidFaces = new(TopTools_ShapeMapHasher.Instance);
    //!< Count number of the same face being marked invalid to avoid infinite rebuilding of the same face
    private readonly NCollection_DataMap<TopoDS_Shape, int> myAlreadyInvFaces = new(TopTools_ShapeMapHasher.Instance);
    //!< Images of the hole faces of the original face
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myFNewHoles = new(TopTools_ShapeMapHasher.Instance);

    //!< Intersection information, used to collect intersection pairs during rebuild
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySSInterfs = new(TopTools_ShapeMapHasher.Instance);
    //!< Intersection information, used to collect intersection pairs during rebuild
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySSInterfsArt = new(TopTools_ShapeMapHasher.Instance);
    //!< All possible intersection pairs, used to avoid some of the intersections
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>>> myIntersectionPairs = new(TopTools_ShapeMapHasher.Instance);

    //!< Faces that have to be rebuilt (invalid and close to invalid faces)
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myFacesToRebuild = new(TopTools_ShapeMapHasher.Instance);
    //!< Faces that have to be avoided when rebuilding splits of the same offset face
    private readonly NCollection_Map<TopoDS_Shape> myFSelfRebAvoid = new(TopTools_ShapeMapHasher.Instance);

    private TopoDS_Shape mySolids = new(); //!< Solids built from the splits of faces

    // Auxiliary tools
    private readonly IntTools_Context myContext;

    // Output
    private readonly BRepAlgo_Image myImage; //!< History of modifications

    //! Constructor, taking the history tool to be filled
    public BRepOffset_BuildOffsetFaces(BRepAlgo_Image theImage)
    {
        myImage = theImage;
        myContext = new IntTools_Context();
    }

    //! Sets faces to build splits
    public void SetFaces(NCollection_List<TopoDS_Shape> theFaces) => myFaces = theFaces;

    //! Sets ascendants/descendants information
    public void SetAsDesInfo(BRepAlgo_AsDes theAsDes) => myAsDes = theAsDes;

    //! Sets the analysis info of the input shape
    public void SetAnalysis(BRepOffset_Analyse theAnalyse) => myAnalyzer = theAnalyse;

    //! Sets origins of the offset edges (from original shape)
    public void SetEdgesOrigins(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgesOrigins) =>
        myEdgesOrigins = theEdgesOrigins;

    //! Sets origins of the offset faces (from original shape)
    public void SetFacesOrigins(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theFacesOrigins) =>
        myFacesOrigins = theFacesOrigins;

    //! Sets infinite (extended) edges for the trimmed ones
    public void SetInfEdges(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theETrimEInf) => myETrimEInf = theETrimEInf;

    //=================================================================================================

    //! Build splits of already trimmed faces
    public void BuildSplitsOfTrimmedFaces(ProgressRange theRange)
    {
        if (!hasData(myFaces))
        {
            return;
        }

        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEdgesOrigins = new(TopTools_ShapeMapHasher.Instance);
        if (myEdgesOrigins is null)
        {
            myEdgesOrigins = anEdgesOrigins;
        }

        using ProgressScope aPS = new(theRange, "Building splits of trimmed faces", 5);

        // Fuse all edges
        IntersectTrimmedEdges(aPS.Next(1));

        using ProgressScope aPSLoop = new(aPS.Next(4), null, myFaces!.Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator aItLF = new(myFaces); aItLF.More(); aItLF.Next())
        {
            if (!aPSLoop.More())
            {
                return;
            }
            TopoDS_Face aF = TopoDS.Face(aItLF.Value());

            TopoDS_Shape aCE = new();
            bool bFound = GetEdges(aF, ref aCE);

            // split the face by the edges
            if (!bFound)
            {
                if (!myImage.HasImage(aF))
                {
                    myOFImages[myOFImages.Add(aF, new NCollection_List<TopoDS_Shape>())].Append(aF);
                }
                continue;
            }

            NCollection_List<TopoDS_Shape> aLFImages = new();
            BuildSplitsOfTrimmedFace(aF, aCE, aLFImages, aPSLoop.Next());

            myOFImages.Add(aF, aLFImages);
        }
        // Fill history for faces and edges
        FillHistory();
    }

    //=================================================================================================

    //! Building splits of not-trimmed offset faces.
    //! For the cases in which invalidities will be found, these invalidities will be rebuilt.
    public void BuildSplitsOfExtendedFaces(ProgressRange theRange)
    {
        // Check input data
        if (!hasData(myFaces) || !hasData(myEdgesOrigins) || !hasData(myFacesOrigins) || !hasData(myETrimEInf))
        {
            return;
        }

        using ProgressScope aPS = new(theRange, "Building splits of extended faces", 100.0);
        // Scope has to be added into a loop of undefined size.
        // In general there are about 2 to 5 loops performed, each time
        // decreasing complexity. So reserve for each next loop smaller time.
        // Reserve also 4% on filling gaps after the faces are built.
        double aWhole = 100.0 - 4.0;

        // Fusing all trimmed offset edges to avoid self-intersections in the splits
        IntersectTrimmedEdges(aPS.Next());
        if (!aPS.More())
        {
            return;
        }
        // vertices to avoid
        NCollection_Map<TopoDS_Shape> aVertsToAvoid = new(TopTools_ShapeMapHasher.Instance);

        // Limit total number of attempts by 10. This should be extra, as each invalid face can be
        // rebuilt only three times. So, in general, there are about 2-5 loops done.
        const int aNbMaxAttempts = 10;
        // First progress portion is the half of the whole. Each next step is half of the previous:
        // 48%, 24%, 12%, 6% and so on. This way in three loops it will already be 84%,
        // and in four - 90%. So even if the loop will stop earlier, the not advanced scope
        // will be acceptable.
        double aPart = aWhole / 2.0;
        for (int iCount = 1; iCount <= aNbMaxAttempts; ++iCount, aPart /= 2.0)
        {
            if (!aPS.More())
            {
                return;
            }
            // Clear the data before further faces construction
            myInvalidFaces.Clear();
            myArtInvalidFaces.Clear();
            myInvalidEdges.Clear();
            myInvertedEdges.Clear();
            mySSInterfs.Clear();
            mySSInterfsArt.Clear();
            myIntersectionPairs.Clear();
            mySolids.Nullify();
            myFacesToRebuild.Clear();
            myFSelfRebAvoid.Clear();

            // Split progress range on
            // * building faces basing on currently available edges and
            // * rebuilding faces basing on edges classification
            using ProgressScope aPSLoop = new(aPS.Next(aPart), null, 10.0);

            // Build splits of the faces having new intersection edges
            BuildSplitsOfFaces(aPSLoop.Next(7.0));
            if (myInvalidFaces.IsEmpty())
            {
                break;
            }

            // Find faces to rebuild
            FindFacesToRebuild();
            if (myFacesToRebuild.IsEmpty())
            {
                break;
            }

            // Perform new intersections
            myModifiedEdges.Clear();
            IntersectFaces(aVertsToAvoid, aPSLoop.Next(3.0));
        }

        // Fill possible gaps in the splits of offset faces to increase possibility of
        // creating closed volume from these splits
        FillGaps(aPS.Next(4.0));

        // Fill history for faces and edges
        FillHistory();
    }

    //=======================================================================
    // function : UpdateIntersectedEdges
    // purpose  : Saving connection from trimmed edges to not trimmed ones
    //=======================================================================
    private void UpdateIntersectedEdges(NCollection_List<TopoDS_Shape> theLA, BOPAlgo_Builder theGF)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator aItA = new(theLA); aItA.More(); aItA.Next())
        {
            TopoDS_Shape aS = aItA.Value();
            if (!myETrimEInf!.Find(aS, out TopoDS_Shape pEInf))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> aLSIm = ModifiedOf(theGF, aS);
            if (aLSIm.IsEmpty())
            {
                continue;
            }

            for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLSIm); aIt.More(); aIt.Next())
            {
                TopoDS_Shape aEIm = aIt.Value();
                if (!myETrimEInf.IsBound(aEIm))
                {
                    myETrimEInf.Bind(aEIm, pEInf);
                }
            }
        }
    }

    //=================================================================================================

    //! Intersection of the trimmed edges among themselves
    private void IntersectTrimmedEdges(ProgressRange theRange)
    {
        // get edges to intersect from descendants of the offset faces
        NCollection_List<TopoDS_Shape> aLS = new();
        //
        using ProgressScope aPS = new(theRange, null, 2);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(myFaces!);
        for (; aItLF.More(); aItLF.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face aF = TopoDS.Face(aItLF.Value());
            //
            NCollection_List<TopoDS_Shape> aLE = myAsDes!.Descendant(aF);
            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
            for (; aItLE.More(); aItLE.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aItLE.Value());
                //
                if (ProcessMicroEdge(aE, myContext))
                {
                    continue;
                }
                //
                if (myModifiedEdges.Add(aE))
                {
                    aLS.Append(aE);
                }
            }
        }
        //
        if (aLS.Extent() < 2)
        {
            // nothing to intersect
            return;
        }
        //
        // perform intersection of the edges
        BOPAlgo_Builder aGFE = new();
        aGFE.SetArguments(aLS);
        aGFE.Perform(aPS.Next());
        if (aGFE.HasErrors())
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape> aLA = new();
        // fill map with edges images
        using ProgressScope aPSLoop = new(aPS.Next(), null, aLS.Extent());
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLS); aIt.More(); aIt.Next(), aPSLoop.Next())
        {
            if (!aPSLoop.More())
            {
                return;
            }
            TopoDS_Shape aE = aIt.Value();
            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(aGFE, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }
            //
            aLA.Append(aE);
            // save images
            myOEImages.Bind(aE, new NCollection_List<TopoDS_Shape>(aLEIm));
            // save origins
            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLEIm);
            for (; aItLE.More(); aItLE.Next())
            {
                TopoDS_Shape aEIm = aItLE.Value();
                if (myOEOrigins.Find(aEIm, out NCollection_List<TopoDS_Shape> pLEOr))
                {
                    AppendToList(pLEOr, aE);
                }
                else
                {
                    NCollection_List<TopoDS_Shape> aL = new();
                    aL.Append(aE);
                    myOEOrigins.Bind(aEIm, aL);
                }
            }
        }
        //
        UpdateOrigins(aLA, myEdgesOrigins!, aGFE);
        UpdateIntersectedEdges(aLA, aGFE);
    }

    //=======================================================================
    // function : CheckConnectionsOfFace
    // purpose  : Checks number of connections for theFace with theLF
    //           Returns true if number of connections more than 1
    //=======================================================================
    private static bool checkConnectionsOfFace(in TopoDS_Shape theFace, NCollection_List<TopoDS_Shape> theLF)
    {
        NCollection_IndexedMap<TopoDS_Shape> aShapeVert = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator aFImIterator = new(theLF); aFImIterator.More(); aFImIterator.Next())
        {
            TopoDS_Shape aShape = aFImIterator.Value();
            if (aShape.IsSame(theFace))
            {
                continue;
            }
            TopExp.MapShapes(aShape, TopAbs_VERTEX, aShapeVert);
        }
        int aNbConnections = 0;
        NCollection_IndexedMap<TopoDS_Shape> aFaceVertices = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theFace, TopAbs_VERTEX, aFaceVertices);
        for (int iV = 1; iV <= aFaceVertices.Extent(); ++iV)
        {
            TopoDS_Shape aVert = aFaceVertices[iV];
            if (aShapeVert.Contains(aVert))
            {
                ++aNbConnections;
            }
            if (aNbConnections > 1)
            {
                return true;
            }
        }
        return false;
    }

    //=======================================================================
    // function : BuildSplitsOfFaces
    // purpose  : Building the splits of offset faces and
    //           looking for the invalid splits
    //=======================================================================
    private void BuildSplitsOfFaces(ProgressRange theRange)
    {
        BRep_Builder aBB = new();
        int i, aNb;
        //
        // processed faces
        NCollection_List<TopoDS_Shape> aLFDone = new();
        // extended face - map of neutral edges, i.e. in one split - valid and in other - invalid
        NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> aDMFMNE = new(TopTools_ShapeMapHasher.Instance);
        // map of valid edges for each face
        NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> aDMFMVE = new(TopTools_ShapeMapHasher.Instance);
        // map of invalid edges for each face
        NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> aDMFMIE = new(TopTools_ShapeMapHasher.Instance);
        // map of valid inverted edges for the face
        NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> aDMFMVIE = new(TopTools_ShapeMapHasher.Instance);
        // map of splits to check for internals
        NCollection_IndexedMap<TopoDS_Shape> aMFToCheckInt = new(TopTools_ShapeMapHasher.Instance);
        // map of edges created from vertex and marked as invalid
        NCollection_Map<TopoDS_Shape> aMEdgeInvalidByVertex = new(TopTools_ShapeMapHasher.Instance);
        // map of edges created from vertex and marked as valid
        NCollection_Map<TopoDS_Shape> aMEdgeValidByVertex = new(TopTools_ShapeMapHasher.Instance);
        // connection map from old edges to new ones
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEOrLEIm = new(TopTools_ShapeMapHasher.Instance);
        //
        // Outer range
        using ProgressScope aPSOuter = new(theRange, null, 10.0);
        // build splits of faces
        using ProgressScope aPSBF = new(aPSOuter.Next(3.0), "Building faces", 2 * myFaces!.Extent());
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(myFaces);
        for (; aItLF.More(); aItLF.Next(), aPSBF.Next())
        {
            if (!aPSBF.More())
            {
                return;
            }
            TopoDS_Face aF = TopoDS.Face(aItLF.Value());
            //
            NCollection_List<TopoDS_Shape>? pLFIm = myOFImages.FindFromKey(aF, out NCollection_List<TopoDS_Shape> aFound) ? aFound : null;
            if (pLFIm is not null && pLFIm.IsEmpty())
            {
                continue;
            }
            // get edges by which the face should be split
            TopoDS_Shape aCE = new();
            NCollection_IndexedMap<TopoDS_Shape> aMapEInv = new(TopTools_ShapeMapHasher.Instance);
            bool bFound = GetEdges(aF, ref aCE, aMapEInv);
            if (!bFound)
            {
                continue;
            }
            //
            // OFFSET_DEBUG: the self-intersection check of the edges is left out.
            // build splits
            NCollection_List<TopoDS_Shape> aLFImages = new();
            BuildSplitsOfFace(aF, aCE, myFacesOrigins!, aLFImages);
            //
            if (aMapEInv.Extent() != 0)
            {
                // check if all possible faces are built
                NCollection_Map<TopoDS_Shape> aMENInv = new(TopTools_ShapeMapHasher.Instance);
                bool bArtificialCase = aLFImages.IsEmpty() || CheckIfArtificial(aF, aLFImages, aCE, aMapEInv, aMENInv);
                //
                // try to build splits using invalid edges
                TopoDS_Compound aCE1 = new();
                aBB.MakeCompound(ref aCE1);
                aBB.Add(aCE1, aCE);
                for (i = 1; i <= aMapEInv.Extent(); ++i)
                {
                    aBB.Add(aCE1, aMapEInv[i]);
                }
                //
                NCollection_List<TopoDS_Shape> aLFImages1 = new();
                BuildSplitsOfFace(aF, aCE1, myFacesOrigins!, aLFImages1);
                //
                // check if the rebuilding has added some new faces to the splits
                for (NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFImages1); aItLFIm.More();)
                {
                    bool bAllInv = true;
                    // Additional check for artificial case
                    // if current image face consist only of edges from aMapEInv and aMENInv
                    // then recheck current face for the further processing
                    bool aToReCheckFace = bArtificialCase;
                    TopoDS_Shape aFIm = aItLFIm.Value();
                    TopExp_Explorer aExpE = new(aFIm, TopAbs_EDGE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        TopoDS_Shape aE = aExpE.Current();
                        if (!aMapEInv.Contains(aE))
                        {
                            bAllInv = false;
                            if (!aMENInv.Contains(aE))
                            {
                                aToReCheckFace = false;
                                break;
                            }
                        }
                    }
                    // if current image face is to recheck then check number of connections for this face
                    // with other image faces for current face
                    if (!aExpE.More() && aToReCheckFace)
                    {
                        aToReCheckFace = checkConnectionsOfFace(aFIm, aLFImages1);
                    }
                    // do not delete image face from further processing if aToReCheckFace is true
                    if (!aExpE.More() && !aToReCheckFace)
                    {
                        if (bAllInv)
                        {
                            aMFToCheckInt.Add(aFIm);
                        }
                        aLFImages1.Remove(aItLFIm);
                    }
                    else
                    {
                        aItLFIm.Next();
                    }
                }
                //
                if (bArtificialCase)
                {
                    if (aLFImages.Extent() == aLFImages1.Extent())
                    {
                        bArtificialCase = false;
                    }
                    else
                    {
                        aLFImages.Assign(aLFImages1);
                    }
                }
                //
                if (bArtificialCase)
                {
                    // make the face invalid
                    NCollection_IndexedMap<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
                    //
                    // OCCT: "*pLFIm = aLFImages" dereferences a null pointer when the face has no splits yet
                    // (an access violation natively, a NullReferenceException here; occt-issues.md).
                    pLFIm!.Assign(aLFImages);
                    NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFImages);
                    for (; aItLFIm.More(); aItLFIm.Next())
                    {
                        TopoDS_Shape aFIm = aItLFIm.Value();
                        TopExp_Explorer aExpE = new(aFIm, TopAbs_EDGE);
                        for (; aExpE.More(); aExpE.Next())
                        {
                            TopoDS_Shape aE = aExpE.Current();
                            if (aMapEInv.Contains(aE))
                            {
                                myInvalidEdges.Add(aE);
                                aMEInv.Add(aE);
                            }
                            else
                            {
                                myValidEdges.Add(aE);
                            }
                        }
                    }
                    //
                    myArtInvalidFaces.Bind(aF, new NCollection_IndexedMap<TopoDS_Shape>(aMEInv));
                    aDMFMIE.Bind(aF, new NCollection_IndexedMap<TopoDS_Shape>(aMEInv));
                    aLFDone.Append(aF);
                    //
                    continue;
                }
            }

            // find invalid edges
            FindInvalidEdges(aF, aLFImages, aDMFMVE, aDMFMNE, aDMFMIE, aDMFMVIE, aDMEOrLEIm, aMEdgeInvalidByVertex,
                             aMEdgeValidByVertex, aPSBF.Next());

            // save the new splits
            if (pLFIm is null)
            {
                pLFIm = myOFImages[myOFImages.Add(aF, new NCollection_List<TopoDS_Shape>())];
            }
            else
            {
                pLFIm.Clear();
            }
            pLFIm.Append(aLFImages);
            //
            aLFDone.Append(aF);
        }
        //
        if (myInvalidEdges.IsEmpty() && myArtInvalidFaces.IsEmpty() && aDMFMIE.IsEmpty())
        {
            return;
        }

        // Additional step to find invalid edges by checking unclassified edges
        // in the splits of SD faces
        FindInvalidEdges(aLFDone, aDMFMIE, aDMFMVE, aDMFMNE);

        // Additional step to mark inverted edges located inside loops
        // of invalid edges as invalid as well
        MakeInvertedEdgesInvalid(aLFDone);

        // OFFSET_DEBUG: the compounds of invalid, valid and inverted edges, and of all splits, are left out.

        // Build Edge-Face connectivity map to find faces which removal
        // may potentially lead to creation of the holes in the faces
        // preventing from obtaining closed volume in the result
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEFMap = new(TopTools_ShapeMapHasher.Instance);
        int aNbF = myOFImages.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            NCollection_List<TopoDS_Shape>.Iterator itLFIm = new(myOFImages[i]);
            for (; itLFIm.More(); itLFIm.Next())
            {
                TopExp.MapShapesAndAncestors(itLFIm.Value(), TopAbs_EDGE, TopAbs_FACE, anEFMap);
            }
        }

        NCollection_Map<TopoDS_Shape> anEmptyMap = new(TopTools_ShapeMapHasher.Instance);
        // invalid faces inside the holes
        NCollection_IndexedMap<TopoDS_Shape> aMFInvInHole = new(TopTools_ShapeMapHasher.Instance);
        // all hole faces
        TopoDS_Compound aFHoles = new();
        aBB.MakeCompound(ref aFHoles);
        // Find the faces containing only the inverted edges and the invalid ones
        NCollection_List<TopoDS_Shape> anInvertedFaces = new();

        using ProgressScope aPSIF = new(aPSOuter.Next(2.0), "Checking validity of faces", aLFDone.Extent());
        // find invalid faces
        // considering faces containing only invalid edges as invalid
        aItLF.Initialize(aLFDone);
        for (; aItLF.More(); aItLF.Next(), aPSIF.Next())
        {
            if (!aPSIF.More())
            {
                return;
            }
            TopoDS_Face aF = TopoDS.Face(aItLF.Value());
            NCollection_List<TopoDS_Shape> aLFImages = myOFImages.ChangeFromKey(aF);
            //
            NCollection_List<TopoDS_Shape> aLFInv = new();
            bool bArtificialCase = myArtInvalidFaces.IsBound(aF);
            if (bArtificialCase)
            {
                aLFInv.Assign(aLFImages);
            }
            else
            {
                // neutral edges
                if (!aDMFMNE.Find(aF, out NCollection_Map<TopoDS_Shape> pMNE))
                {
                    pMNE = anEmptyMap;
                }
                // find faces inside holes wires
                NCollection_Map<TopoDS_Shape> aMFHoles = new(TopTools_ShapeMapHasher.Instance);
                TopoDS_Face aFOr = TopoDS.Face(myFacesOrigins!.Find(aF));
                FindFacesInsideHoleWires(aFOr, aF, aLFImages, aDMEOrLEIm, anEFMap, aMFHoles);
                //
                foreach (TopoDS_Shape aItMH in aMFHoles)
                {
                    aBB.Add(aFHoles, aItMH);
                }
                //
                // find invalid faces
                FindInvalidFaces(aLFImages, aDMFMVE, aDMFMIE, pMNE, aMEdgeInvalidByVertex, aMEdgeValidByVertex, aMFHoles,
                                 aMFInvInHole, aLFInv, anInvertedFaces);
            }
            //
            if (aLFInv.Extent() != 0)
            {
                if (myAlreadyInvFaces.IsBound(aF))
                {
                    if (myAlreadyInvFaces.Find(aF) > 2)
                    {
                        if (aLFInv.Extent() == aLFImages.Extent() && !bArtificialCase)
                        {
                            aLFImages.Clear();
                        }
                        //
                        aLFInv.Clear();
                    }
                }
                myInvalidFaces.Add(aF, aLFInv);
            }
        }
        //
        if (myInvalidFaces.IsEmpty())
        {
            myInvalidEdges.Clear();
            return;
        }
        //
        // OFFSET_DEBUG: the compound of invalid faces is left out.
        //
        NCollection_IndexedMap<TopoDS_Shape> aMERemoved = new(TopTools_ShapeMapHasher.Instance);
        // remove invalid splits of faces using inverted edges
        RemoveInvalidSplitsByInvertedEdges(aMERemoved);
        if (myInvalidFaces.IsEmpty())
        {
            myInvalidEdges.Clear();
            return;
        }
        //
        // remove invalid splits from valid splits
        RemoveInvalidSplitsFromValid(aDMFMVIE);
        //
        // remove inside faces
        RemoveInsideFaces(anInvertedFaces, aMFToCheckInt, aMFInvInHole, aFHoles, aMERemoved, myInsideEdges,
                          aPSOuter.Next(5.0));
        //
        // make compound of valid splits
        TopoDS_Compound aCFIm = new();
        aBB.MakeCompound(ref aCFIm);
        //
        aNb = myOFImages.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages[i];
            aItLF.Initialize(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aFIm = aItLF.Value();
                aBB.Add(aCFIm, aFIm);
            }
        }
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aCFIm, TopAbs_EDGE, TopAbs_FACE, aDMEF);
        //
        // filter maps of images and origins
        FilterEdgesImages(aCFIm);
        //
        // filter invalid faces
        FilterInvalidFaces(aDMEF, aMERemoved.Extent() != 0 ? myInsideEdges : aMERemoved);
        aNb = myInvalidFaces.Extent();
        if (aNb == 0)
        {
            myInvalidEdges.Clear();
            return;
        }
        //
        // OFFSET_DEBUG: the compound of invalid faces is left out.
        //
        // filter invalid edges
        NCollection_Map<TopoDS_Shape> aMEUseInRebuild = new(TopTools_ShapeMapHasher.Instance);
        FilterInvalidEdges(aDMFMIE, aMERemoved, aMERemoved.Extent() != 0 ? myInsideEdges : aMERemoved, aMEUseInRebuild);
        //
        // Check additionally validity of edges originated from vertices.
        CheckEdgesCreatedByVertex();

        // OFFSET_DEBUG: the compound of invalid edges is left out.
        //
        myLastInvEdges.Clear();
        aNb = myInvalidEdges.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aE = myInvalidEdges[i];
            myLastInvEdges.Add(aE);
            if (!aMEUseInRebuild.Contains(aE))
            {
                myEdgesToAvoid.Add(aE);
            }
        }
        //
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            if (myAlreadyInvFaces.IsBound(aF))
            {
                myAlreadyInvFaces.ChangeFind(aF)++;
            }
            else
            {
                myAlreadyInvFaces.Bind(aF, 1);
            }
        }
    }

    //=======================================================================
    // function : GetEdges
    // purpose  : Getting edges from AsDes map to build the splits of faces
    //=======================================================================
    private bool GetEdges(in TopoDS_Face theFace, ref TopoDS_Shape theEdges,
                          NCollection_IndexedMap<TopoDS_Shape>? theInvMap = null)
    {
        // get boundary edges
        NCollection_Map<TopoDS_Shape> aMFBounds = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer aExp = new(theFace, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aE = aExp.Current();
            if (myOEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
            {
                for (NCollection_List<TopoDS_Shape>.Iterator aItLE0 = new(pLEIm); aItLE0.More(); aItLE0.Next())
                {
                    aMFBounds.Add(aItLE0.Value());
                }
            }
            else
            {
                aMFBounds.Add(aE);
            }
        }

        BRep_Builder aBB = new();
        bool bFound = false, bUpdate = false;
        // the resulting edges
        TopoDS_Compound anEdges = new();
        aBB.MakeCompound(ref anEdges);
        // Fence map
        NCollection_Map<TopoDS_Shape> aMEFence = new(TopTools_ShapeMapHasher.Instance);
        // the edges by which the offset face should be split
        NCollection_List<TopoDS_Shape> aLE = myAsDes!.Descendant(theFace);
        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aItLE.Value());
            //
            if (!bUpdate)
            {
                bUpdate = myModifiedEdges.Contains(aE);
            }
            //
            if (myOEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
            {
                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    TopoDS_Edge aEIm = TopoDS.Edge(aItLEIm.Value());
                    //
                    if (!aMEFence.Add(aEIm))
                    {
                        continue;
                    }

                    if (myEdgesToAvoid.Contains(aEIm))
                    {
                        theInvMap?.Add(aEIm);
                        if (!bUpdate)
                        {
                            bUpdate = myLastInvEdges.Contains(aEIm);
                        }
                        continue;
                    }
                    // check for micro edge
                    if (ProcessMicroEdge(aEIm, myContext))
                    {
                        continue;
                    }
                    //
                    aBB.Add(anEdges, aEIm);
                    if (!bFound)
                    {
                        bFound = !aMFBounds.Contains(aEIm);
                    }
                    //
                    if (!bUpdate)
                    {
                        bUpdate = myModifiedEdges.Contains(aEIm);
                    }
                }
            }
            else
            {
                if (myEdgesToAvoid.Contains(aE))
                {
                    theInvMap?.Add(aE);
                    if (!bUpdate)
                    {
                        bUpdate = myLastInvEdges.Contains(aE);
                    }
                    continue;
                }
                //
                if (ProcessMicroEdge(aE, myContext))
                {
                    continue;
                }
                aBB.Add(anEdges, aE);
                if (!bFound)
                {
                    bFound = !aMFBounds.Contains(aE);
                }
            }
        }
        //
        theEdges = anEdges;
        return bFound && bUpdate;
    }

    //=======================================================================
    // function : CheckIfArtificial
    // purpose  : Checks if the face is artificially invalid
    //=======================================================================
    private bool CheckIfArtificial(in TopoDS_Shape theF, NCollection_List<TopoDS_Shape> theLFImages, in TopoDS_Shape theCE,
                                   NCollection_IndexedMap<TopoDS_Shape> theMapEInv, NCollection_Map<TopoDS_Shape> theMENInv)
    {
        // all boundary edges should be used
        NCollection_IndexedMap<TopoDS_Shape> aMEUsed = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(theLFImages);
        for (; aItLFIm.More(); aItLFIm.Next())
        {
            TopoDS_Shape aFIm = aItLFIm.Value();
            TopExp.MapShapes(aFIm, TopAbs_EDGE, aMEUsed);
            TopExp.MapShapes(aFIm, TopAbs_VERTEX, aMEUsed);
        }
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theCE, TopAbs_VERTEX, TopAbs_EDGE, aMVE);
        //
        int i, aNb = theMapEInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aEInv = theMapEInv[i];
            TopExp_Explorer aExpV = new(aEInv, TopAbs_VERTEX);
            for (; aExpV.More(); aExpV.Next())
            {
                TopoDS_Shape aVEInv = aExpV.Current();
                if (aMVE.FindFromKey(aVEInv, out NCollection_List<TopoDS_Shape> pLENInv))
                {
                    NCollection_List<TopoDS_Shape>.Iterator aItLEInv = new(pLENInv);
                    for (; aItLEInv.More(); aItLEInv.Next())
                    {
                        TopoDS_Shape aENInv = aItLEInv.Value();
                        if (!aMEUsed.Contains(aENInv))
                        {
                            theMENInv.Add(aENInv);
                        }
                    }
                }
            }
        }
        //
        if (theMENInv.IsEmpty())
        {
            return false;
        }
        //
        NCollection_IndexedMap<TopoDS_Shape> aMEFound = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theCE, TopAbs_EDGE, aMEFound);
        //
        NCollection_List<TopoDS_Shape> aLE = myAsDes!.Descendant(theF);
        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aItLE.Value());
            //
            if (myOEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
            {
                bool bChecked = false;
                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    TopoDS_Edge aEIm = TopoDS.Edge(aItLEIm.Value());
                    if (!aMEFound.Contains(aEIm) || theMENInv.Contains(aEIm))
                    {
                        continue;
                    }
                    //
                    bChecked = true;
                    if (aMEUsed.Contains(aEIm))
                    {
                        break;
                    }
                }
                //
                if (bChecked && !aItLEIm.More())
                {
                    break;
                }
            }
            else
            {
                if (aMEFound.Contains(aE) && !theMENInv.Contains(aE) && !aMEUsed.Contains(aE))
                {
                    break;
                }
            }
        }
        //
        return aItLE.More();
    }

    //=================================================================================================

    //! Looks for the invalid edges (edges with changed orientation)
    //! in the splits of offset faces
    private void FindInvalidEdges(in TopoDS_Face theF, NCollection_List<TopoDS_Shape> theLFImages,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMFMVE,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMFMNE,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> theDMFMIE,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMFMVIE,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEOrLEIm,
                                  NCollection_Map<TopoDS_Shape> theEdgesInvalidByVertex,
                                  NCollection_Map<TopoDS_Shape> theEdgesValidByVertex, ProgressRange theRange)
    {
        // Edge is considered as invalid in the following cases:
        // 1. Its orientation on the face has changed comparing to the originals edge and face;
        // 2. The vertices of the edge have changed places comparing to the originals edge and face.
        //
        // The edges created from vertices, i.e. as intersection between two faces connected only
        // by VERTEX, will also be checked on validity. For these edges the correct orientation will
        // be defined by the edges on the original face adjacent to the connection vertex

        // original face
        TopoDS_Face aFOr = TopoDS.Face(myFacesOrigins!.Find(theF));
        // invalid edges
        NCollection_IndexedMap<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
        // valid edges
        NCollection_Map<TopoDS_Shape> aMEVal = new(TopTools_ShapeMapHasher.Instance);
        // internal edges
        NCollection_Map<TopoDS_Shape> aMEInt = new(TopTools_ShapeMapHasher.Instance);
        //
        // maps for checking the inverted edges
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVE = new(TopTools_ShapeMapHasher.Instance),
                                                                                 aDMEF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMEdges = new(TopTools_ShapeMapHasher.Instance);
        // back map from the original shapes to their offset images
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anImages = new(TopTools_ShapeMapHasher.Instance);
        //
        using ProgressScope aPS = new(theRange, "Checking validity of edges", 2 * theLFImages.Extent());
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(theLFImages);
        for (; aItLF.More(); aItLF.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
            //
            TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aE = aExp.Current();
                // keep all edges
                aMEdges.Add(aE);
                //
                // keep connection from edges to faces
                if (!aDMEF.FindFromKey(aE, out NCollection_List<TopoDS_Shape> pLF))
                {
                    pLF = aDMEF[aDMEF.Add(aE, new NCollection_List<TopoDS_Shape>())];
                }
                AppendToList(pLF, aFIm);
                //
                // keep connection from vertices to edges
                TopoDS_Iterator aItV = new(aE);
                for (; aItV.More(); aItV.Next())
                {
                    TopoDS_Shape aV = aItV.Value();
                    //
                    if (!aDMVE.FindFromKey(aV, out NCollection_List<TopoDS_Shape> pLE))
                    {
                        pLE = aDMVE[aDMVE.Add(aV, new NCollection_List<TopoDS_Shape>())];
                    }
                    AppendToList(pLE, aE);
                }

                // back map from original edges to their offset images
                if (!myEdgesOrigins!.Find(aE, out NCollection_List<TopoDS_Shape> pLOr))
                {
                    continue;
                }
                for (NCollection_List<TopoDS_Shape>.Iterator itOr = new(pLOr); itOr.More(); itOr.Next())
                {
                    TopoDS_Shape aSOr = itOr.Value();
                    TopoDS_Shape aSInF = new();
                    if (!FindShape(aSOr, aFOr, myAnalyzer, ref aSInF))
                    {
                        continue;
                    }
                    if (!anImages.Find(aSInF, out NCollection_List<TopoDS_Shape> pImages))
                    {
                        pImages = new NCollection_List<TopoDS_Shape>();
                        anImages.Bind(aSInF, pImages);
                    }
                    AppendToList(pImages, aE);
                }
            }
        }
        //
        // the map will be used to find the edges on the original face
        // adjacent to the same vertex. It will be filled at first necessity;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVEFOr = new(TopTools_ShapeMapHasher.Instance);
        //
        aItLF.Initialize(theLFImages);
        for (; aItLF.More(); aItLF.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
            //
            // valid edges for this split
            NCollection_Map<TopoDS_Shape> aMVE = new(TopTools_ShapeMapHasher.Instance);
            // invalid edges for this split
            NCollection_IndexedMap<TopoDS_Shape> aMIE = new(TopTools_ShapeMapHasher.Instance);
            //
            TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge aEIm = TopoDS.Edge(aExp.Current());
                //
                if (aEIm.Orientation() == TopAbs_INTERNAL)
                {
                    aMEInt.Add(aEIm);
                    continue;
                }
                //
                if (!myEdgesOrigins!.Find(aEIm, out NCollection_List<TopoDS_Shape> pLEOr) || pLEOr.IsEmpty())
                {
                    continue;
                }
                //
                int aNbVOr = 0;
                NCollection_List<TopoDS_Shape>.Iterator aItLEO = new(pLEOr);
                for (; aItLEO.More(); aItLEO.Next())
                {
                    if (aItLEO.Value().ShapeType() == TopAbs_VERTEX)
                    {
                        ++aNbVOr;
                    }
                }
                if (aNbVOr > 1 && (pLEOr.Extent() - aNbVOr) > 1)
                {
                    continue;
                }
                //
                NCollection_Map<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance), aMV = new(TopTools_ShapeMapHasher.Instance),
                                              aMF = new(TopTools_ShapeMapHasher.Instance);
                bool bInvalid = false, bChecked = false;
                int aNbP = NbPoints(aEIm), aNbInv = 0;
                bool bUseVertex = aNbVOr == 0 ? false
                                              : (aNbVOr == 1 && aDMEF.FindFromKey(aEIm).Extent() == 1
                                                 && !myOEOrigins.IsBound(aEIm));
                //
                aItLEO.Initialize(pLEOr);
                for (; aItLEO.More(); aItLEO.Next())
                {
                    TopoDS_Shape aSOr = aItLEO.Value();
                    bool bVertex = (aSOr.ShapeType() == TopAbs_VERTEX);
                    //
                    TopoDS_Shape aEOrF = new();
                    if (bVertex)
                    {
                        // for some cases it is impossible to check the validity of the edge
                        if (!bUseVertex)
                        {
                            continue;
                        }
                        // find edges on the original face adjacent to this vertex
                        if (aDMVEFOr.IsEmpty())
                        {
                            // fill the map
                            TopExp.MapShapesAndAncestors(aFOr, TopAbs_VERTEX, TopAbs_EDGE, aDMVEFOr);
                        }
                        //
                        if (aDMVEFOr.FindFromKey(aSOr, out NCollection_List<TopoDS_Shape> pLEFOr))
                        {
                            TopoDS_Compound aCEOr = new();
                            new BRep_Builder().MakeCompound(ref aCEOr);
                            // Avoid classification of edges originated from vertices
                            // located between tangent edges
                            bool bAllTgt = true;
                            NCollection_List<TopoDS_Shape>.Iterator aItLEFOr = new(pLEFOr);
                            Vector3D aVRef = GetAverageTangent(aItLEFOr.Value(), aNbP);
                            for (; aItLEFOr.More(); aItLEFOr.Next())
                            {
                                TopoDS_Shape aEOr = aItLEFOr.Value();
                                new BRep_Builder().Add(aCEOr, aEOr);

                                Vector3D aVCur = GetAverageTangent(aEOr, aNbP);
                                if (!aVRef.IsParallel(aVCur, global::Kerf.Engine.Foundation.Precision.Angular()))
                                {
                                    bAllTgt = false;
                                }
                            }
                            if (!bAllTgt)
                            {
                                aEOrF = aCEOr;
                            }
                        }
                    }
                    else
                    {
                        FindShape(aSOr, aFOr, myAnalyzer, ref aEOrF);
                        //
                        if (!theDMEOrLEIm.Find(aSOr, out NCollection_List<TopoDS_Shape> pLEIm))
                        {
                            pLEIm = new NCollection_List<TopoDS_Shape>();
                            theDMEOrLEIm.Bind(aSOr, pLEIm);
                        }
                        AppendToList(pLEIm, aEIm);
                    }
                    //
                    if (aEOrF.IsNull())
                    {
                        // the edge has not been found
                        continue;
                    }

                    if (bVertex)
                    {
                        NCollection_Map<TopoDS_Shape> aMVTotal = new(TopTools_ShapeMapHasher.Instance);
                        int aNbChecked = 0;
                        // Just check if the original edges sharing the vertex do not share it any more.
                        for (TopoDS_Iterator it = new(aEOrF); it.More(); it.Next())
                        {
                            TopoDS_Shape aEOr = it.Value();
                            if (!anImages.Find(aEOr, out NCollection_List<TopoDS_Shape> aLIm))
                            {
                                continue;
                            }
                            ++aNbChecked;
                            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVLoc = new(TopTools_ShapeMapHasher.Instance);
                            for (NCollection_List<TopoDS_Shape>.Iterator itLIM = new(aLIm); itLIM.More(); itLIM.Next())
                            {
                                TopExp.MapShapesAndAncestors(itLIM.Value(), TopAbs_VERTEX, TopAbs_EDGE, aMVLoc);
                            }
                            for (int i0 = 1; i0 <= aMVLoc.Extent(); ++i0)
                            {
                                if (aMVLoc[i0].Extent() > 1 && !aMVTotal.Add(aMVLoc.FindKey(i0)))
                                {
                                    bInvalid = true;
                                    theEdgesInvalidByVertex.Add(aEIm);
                                    break;
                                }
                            }
                            if (bInvalid)
                            {
                                break;
                            }
                        }
                        if (!bInvalid && aNbChecked < 2)
                        {
                            continue;
                        }
                        else
                        {
                            theEdgesValidByVertex.Add(aEIm);
                        }
                    }
                    else
                    {
                        //
                        // Check orientations of the image edge and original edge.
                        // In case the 3d curves are having the same direction the orientations
                        // must be the same. Otherwise the orientations should also be different.
                        //
                        // get average tangent vector for each curve taking into account
                        // the orientations of the edges, i.e. the edge is reversed
                        // the vector is reversed as well
                        Vector3D aVSum1 = GetAverageTangent(aEIm, aNbP);
                        Vector3D aVSum2 = GetAverageTangent(aEOrF, aNbP);
                        //
                        aVSum1.Normalize();
                        aVSum2.Normalize();
                        //
                        double aCos = aVSum1.Dot(aVSum2);
                        if (Math.Abs(aCos) < 0.9999)
                        {
                            continue;
                        }
                        //
                        aME.Add(aEOrF);
                        TopExp_Explorer aExpE = new(aEOrF, TopAbs_VERTEX);
                        for (; aExpE.More(); aExpE.Next())
                        {
                            TopoDS_Shape aV = aExpE.Current();
                            aMV.Add(aV);
                        }
                        if (myAnalyzer is not null)
                        {
                            for (NCollection_List<TopoDS_Shape>.Iterator itFA = new(myAnalyzer.Ancestors(aEOrF)); itFA.More(); itFA.Next())
                            {
                                aMF.Add(itFA.Value());
                            }
                        }
                        //
                        if (aCos < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            bInvalid = true;
                            aNbInv++;
                        }
                    }
                    bChecked = true;
                }
                //
                if (!bChecked)
                {
                    continue;
                }
                //
                bool bLocalOnly = (aNbVOr > 1 && (pLEOr.Extent() - aNbVOr) > 1);
                int aNbE = aME.Extent(), aNbV = aMV.Extent();
                if (aNbE > 1 && aNbV == 2 * aNbE)
                {
                    bool bSkip = true;

                    // It seems the edge originated from not connected edges and cannot be
                    // considered as correctly classified as it may fill some undesired parts.
                    // Still, allow the edge to be accounted for local analysis if it is:
                    // * originated from more than two faces
                    // * unanimously considered valid or invalid
                    // * not a boundary edge in the splits
                    if (aMF.Extent() > 2 && (aNbInv == 0 || aNbInv == aNbE))
                    {
                        if (theLFImages.Extent() > 2)
                        {
                            TopoDS_Iterator itV = new(aEIm);
                            for (; itV.More(); itV.Next())
                            {
                                NCollection_List<TopoDS_Shape>.Iterator itE = new(aDMVE.FindFromKey(itV.Value()));
                                for (; itE.More(); itE.Next())
                                {
                                    if (aDMEF.FindFromKey(itE.Value()).Extent() < 2)
                                    {
                                        break;
                                    }
                                }
                                if (itE.More())
                                {
                                    break;
                                }
                            }
                            bSkip = itV.More();
                        }
                    }
                    if (bSkip)
                    {
                        continue;
                    }
                    else
                    {
                        bLocalOnly = true;
                    }
                }
                //
                if (bInvalid)
                {
                    if (!bLocalOnly)
                    {
                        myInvalidEdges.Add(aEIm);
                    }
                    aMIE.Add(aEIm);
                    aMEInv.Add(aEIm);
                    continue;
                }
                //
                // check if the edge has been inverted
                bool bInverted = aNbE == 0 || bLocalOnly ? false : CheckInverted(aEIm, aFOr, aDMVE, aMEdges);
                //
                if (!bInverted || aNbVOr == 0)
                {
                    if (!bLocalOnly)
                    {
                        myValidEdges.Add(aEIm);
                    }
                    aMVE.Add(aEIm);
                    aMEVal.Add(aEIm);
                }
            }
            //
            // valid edges
            if (aMVE.Extent() != 0)
            {
                theDMFMVE.Bind(aFIm, aMVE);
            }
            //
            // invalid edges
            if (aMIE.Extent() != 0)
            {
                theDMFMIE.Bind(aFIm, aMIE);
            }
        }
        //
        // process invalid edges:
        // check for the inverted edges
        NCollection_Map<TopoDS_Shape> aMVIE = new(TopTools_ShapeMapHasher.Instance);
        // fill neutral edges
        NCollection_Map<TopoDS_Shape> aMNE = new(TopTools_ShapeMapHasher.Instance);
        //
        int i, aNbEInv = aMEInv.Extent();
        for (i = 1; i <= aNbEInv; ++i)
        {
            TopoDS_Shape aEIm = aMEInv[i];
            //
            // neutral edges - on the splits of the same offset face
            // it is valid for one split and invalid for other
            if (aMEVal.Contains(aEIm))
            {
                aMNE.Add(aEIm);
                continue;
            }
            //
            // the inverted images of the origins of invalid edges should also be invalid
            if (!myInvertedEdges.Contains(aEIm))
            {
                continue;
            }
            //
            if (!myOEOrigins.Find(aEIm, out NCollection_List<TopoDS_Shape> pLOEOr))
            {
                continue;
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLOEOr = new(pLOEOr);
            for (; aItLOEOr.More(); aItLOEOr.Next())
            {
                TopoDS_Shape aOEOr = aItLOEOr.Value();
                NCollection_List<TopoDS_Shape> aLEIm1 = myOEImages.Find(aOEOr);
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLEIm1 = new(aLEIm1);
                for (; aItLEIm1.More(); aItLEIm1.Next())
                {
                    TopoDS_Shape aEIm1 = aItLEIm1.Value();
                    if (aMEdges.Contains(aEIm1) && !aMEInv.Contains(aEIm1) && !aMEInt.Contains(aEIm1)
                        && myInvertedEdges.Contains(aEIm1))
                    {
                        myInvalidEdges.Add(aEIm1);
                        aMVIE.Add(aEIm1);
                    }
                }
            }
        }
        //
        if (aMNE.Extent() != 0)
        {
            theDMFMNE.Bind(theF, aMNE);
        }
        //
        if (aMVIE.Extent() != 0)
        {
            theDMFMVIE.Bind(theF, aMVIE);
        }
    }

    //=======================================================================
    // function : addAsNeutral
    // purpose  : Adds as the edge into corresponding maps making it neutral
    //=======================================================================
    private static void addAsNeutral(in TopoDS_Shape theE, in TopoDS_Shape theFInv, in TopoDS_Shape theFVal,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> theLocInvEdges,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theLocValidEdges)
    {
        if (!theLocInvEdges.Find(theFInv, out NCollection_IndexedMap<TopoDS_Shape> pMEInv))
        {
            pMEInv = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            theLocInvEdges.Bind(theFInv, pMEInv);
        }
        pMEInv.Add(theE);

        if (!theLocValidEdges.Find(theFVal, out NCollection_Map<TopoDS_Shape> pMEVal))
        {
            pMEVal = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            theLocValidEdges.Bind(theFVal, pMEVal);
        }
        pMEVal.Add(theE);
    }

    //=======================================================================
    // function : FindInvalidEdges
    // purpose  : Additional method to look for invalid edges
    //=======================================================================
    private void FindInvalidEdges(NCollection_List<TopoDS_Shape> theLFOffset,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> theLocInvEdges,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theLocValidEdges,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theNeutralEdges)
    {
        // 1. Find edges unclassified in faces
        // 2. Find SD faces in which the same edge is classified
        // 3. Check if the edge is neutral in face in which it wasn't classified

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> aMEUnclassified = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aFSplitFOffset = new(TopTools_ShapeMapHasher.Instance);

        // Avoid artificial faces
        NCollection_Map<TopoDS_Shape> aNewFaces = new(TopTools_ShapeMapHasher.Instance);
        if (myAnalyzer is not null)
        {
            NCollection_Map<TopoDS_Shape> aMapNewTmp = new(TopTools_ShapeMapHasher.Instance);
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(myAnalyzer.NewFaces()); it.More(); it.Next())
            {
                aMapNewTmp.Add(it.Value());
            }

            for (NCollection_List<TopoDS_Shape>.Iterator it = new(theLFOffset); it.More(); it.Next())
            {
                TopoDS_Shape aFOffset = it.Value();
                TopoDS_Shape aFOrigin = myFacesOrigins!.Find(aFOffset);
                if (aMapNewTmp.Contains(aFOrigin))
                {
                    aNewFaces.Add(aFOffset);
                }
            }
        }

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEFMap = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator itLFO = new(theLFOffset); itLFO.More(); itLFO.Next())
        {
            TopoDS_Shape aF = itLFO.Value();
            if (aNewFaces.Contains(aF))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> aLFImages = myOFImages.FindFromKey(aF);
            for (NCollection_List<TopoDS_Shape>.Iterator itLF = new(aLFImages); itLF.More(); itLF.Next())
            {
                TopoDS_Shape aFIm = itLF.Value();

                TopExp.MapShapesAndAncestors(aFIm, TopAbs_EDGE, TopAbs_FACE, anEFMap);

                theLocInvEdges.Find(aFIm, out NCollection_IndexedMap<TopoDS_Shape>? pMEInvalid);
                theLocValidEdges.Find(aFIm, out NCollection_Map<TopoDS_Shape>? pMEValid);

                for (TopExp_Explorer expE = new(aFIm, TopAbs_EDGE); expE.More(); expE.Next())
                {
                    TopoDS_Shape aE = expE.Current();
                    if (myInvalidEdges.Contains(aE) != myValidEdges.Contains(aE))
                    {
                        // edge is classified in some face

                        if ((pMEInvalid is null || !pMEInvalid.Contains(aE)) && (pMEValid is null || !pMEValid.Contains(aE)))
                        {
                            // but not in the current one
                            if (!aMEUnclassified.FindFromKey(aE, out NCollection_Map<TopoDS_Shape> pMap))
                            {
                                pMap = aMEUnclassified[aMEUnclassified.Add(aE, new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance))];
                            }
                            pMap.Add(aFIm);

                            aFSplitFOffset.Bind(aFIm, aF);
                        }
                    }
                }
            }
        }

        if (aMEUnclassified.IsEmpty())
        {
            return;
        }

        // Analyze unclassified edges
        int aNbE = aMEUnclassified.Extent();
        for (int iE = 1; iE <= aNbE; ++iE)
        {
            TopoDS_Shape aE = aMEUnclassified.FindKey(iE);
            NCollection_Map<TopoDS_Shape> aMFUnclassified = aMEUnclassified[iE];

            NCollection_List<TopoDS_Shape> aLF = anEFMap.FindFromKey(aE);

            for (NCollection_List<TopoDS_Shape>.Iterator itLF = new(aLF); itLF.More(); itLF.Next())
            {
                TopoDS_Shape aFClassified = itLF.Value();
                if (aMFUnclassified.Contains(aFClassified))
                {
                    continue;
                }

                BOPTools_Set anEdgeSetClass = new();
                anEdgeSetClass.Add(aFClassified, TopAbs_EDGE);

                TopoDS_Shape aEClassified = new();
                FindShape(aE, aFClassified, null, ref aEClassified);
                TopAbs_Orientation anOriClass = aEClassified.Orientation();

                BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(TopoDS.Edge(aEClassified), TopoDS.Face(aFClassified),
                                                           out Direction3D aDNClass);

                theLocInvEdges.Find(aFClassified, out NCollection_IndexedMap<TopoDS_Shape>? pMEInvalid);
                bool isInvalid = pMEInvalid is not null && pMEInvalid.Contains(aE);

                // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
                foreach (TopoDS_Shape aFUnclassified in aMFUnclassified)
                {
                    BOPTools_Set anEdgeSetUnclass = new();
                    anEdgeSetUnclass.Add(aFUnclassified, TopAbs_EDGE);

                    if (anEdgeSetClass.IsEqual(anEdgeSetUnclass))
                    {
                        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(TopoDS.Edge(aE), TopoDS.Face(aFUnclassified),
                                                                   out Direction3D aDNUnclass);

                        bool isSameOri = aDNClass.IsEqual(aDNUnclass, global::Kerf.Engine.Foundation.Precision.Angular());

                        // Among other splits of the same face find those where the edge is contained with
                        // different orientation
                        TopoDS_Shape aFOffset = aFSplitFOffset.Find(aFUnclassified);
                        NCollection_List<TopoDS_Shape> aLFSplits = myOFImages.FindFromKey(aFOffset);
                        NCollection_List<TopoDS_Shape>.Iterator itLFSp = new(aLFSplits);
                        for (; itLFSp.More(); itLFSp.Next())
                        {
                            TopoDS_Shape aFSp = itLFSp.Value();

                            if (!aFSp.IsSame(aFUnclassified) && aMFUnclassified.Contains(aFSp))
                            {
                                TopoDS_Shape aEUnclassified = new();
                                FindShape(aE, aFSp, null, ref aEUnclassified);

                                TopAbs_Orientation anOriUnclass = aEUnclassified.Orientation();
                                if (!isSameOri)
                                {
                                    anOriUnclass = TopAbs.Reverse(anOriUnclass);
                                }

                                if (anOriClass != anOriUnclass)
                                {
                                    // make the edge neutral for the face
                                    if (!theNeutralEdges.Find(aFOffset, out NCollection_Map<TopoDS_Shape> pMENeutral))
                                    {
                                        pMENeutral = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                                        theNeutralEdges.Bind(aFOffset, pMENeutral);
                                    }
                                    pMENeutral.Add(aE);

                                    if (isInvalid && isSameOri)
                                    {
                                        // make edge invalid in aFUnclassified and valid in aFSp
                                        addAsNeutral(aE, aFClassified, aFSp, theLocInvEdges, theLocValidEdges);
                                    }
                                    else
                                    {
                                        // make edge invalid in aFSp and valid in aFUnclassified
                                        addAsNeutral(aE, aFSp, aFClassified, theLocInvEdges, theLocValidEdges);
                                    }
                                }
                            }
                        }
                        if (itLFSp.More())
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : MakeInvertedEdgesInvalid
    // purpose  : Makes inverted edges located inside loop of invalid edges, invalid as well
    //=======================================================================
    private void MakeInvertedEdgesInvalid(NCollection_List<TopoDS_Shape> theLFOffset)
    {
        if (myInvalidEdges.IsEmpty() || myInvertedEdges.IsEmpty())
        {
            return;
        }

        // Map all invalid edges
        TopoDS_Compound aCBEInv = new();
        new BRep_Builder().MakeCompound(ref aCBEInv);
        for (int i = 1; i <= myInvalidEdges.Extent(); ++i)
        {
            new BRep_Builder().Add(aCBEInv, myInvalidEdges[i]);
        }

        // Make loops of invalid edges
        NCollection_List<TopoDS_Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCBEInv, TopAbs_VERTEX, TopAbs_EDGE, aLCB);

        // Analyze each loop on closeness and use only closed ones
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDMVCB = new(TopTools_ShapeMapHasher.Instance);

        for (NCollection_List<TopoDS_Shape>.Iterator itLCB = new(aLCB); itLCB.More(); itLCB.Next())
        {
            TopoDS_Shape aCB = itLCB.Value();

            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVE = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(aCB, TopAbs_VERTEX, TopAbs_EDGE, aDMVE);
            bool isClosed = true;
            for (int iV = 1; iV <= aDMVE.Extent(); ++iV)
            {
                if (aDMVE[iV].Extent() != 2)
                {
                    isClosed = false;
                    break;
                }
            }
            if (!isClosed)
            {
                continue;
            }

            // Bind loop to each vertex of the loop
            for (int iV = 1; iV <= aDMVE.Extent(); ++iV)
            {
                aDMVCB.Bind(aDMVE.FindKey(iV), aCB);
            }
        }

        // Check if any inverted edges of offset faces are locked inside the loops of invalid edges.
        // Make such edges invalid as well.
        for (NCollection_List<TopoDS_Shape>.Iterator itLF = new(theLFOffset); itLF.More(); itLF.Next())
        {
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(itLF.Value());
            for (NCollection_List<TopoDS_Shape>.Iterator itLFIm = new(aLFIm); itLFIm.More(); itLFIm.Next())
            {
                for (TopExp_Explorer expE = new(itLFIm.Value(), TopAbs_EDGE); expE.More(); expE.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(expE.Current());
                    if (!myInvalidEdges.Contains(aE) && myInvertedEdges.Contains(aE))
                    {
                        bool bCB1 = aDMVCB.Find(TopExp.FirstVertex(aE), out TopoDS_Shape pCB1);
                        bool bCB2 = aDMVCB.Find(TopExp.LastVertex(aE), out TopoDS_Shape pCB2);
                        if (bCB1 && bCB2 && pCB1.IsSame(pCB2))
                        {
                            myInvalidEdges.Add(aE);
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FindInvalidFaces
    // purpose  : Looking for the invalid faces by analyzing their invalid edges
    //=======================================================================
    private void FindInvalidFaces(NCollection_List<TopoDS_Shape> theLFImages,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMFMVE,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> theDMFMIE,
                                  NCollection_Map<TopoDS_Shape> theMENeutral,
                                  NCollection_Map<TopoDS_Shape> theEdgesInvalidByVertex,
                                  NCollection_Map<TopoDS_Shape> theEdgesValidByVertex,
                                  NCollection_Map<TopoDS_Shape> theMFHoles,
                                  NCollection_IndexedMap<TopoDS_Shape> theMFInvInHole,
                                  NCollection_List<TopoDS_Shape> theInvFaces,
                                  NCollection_List<TopoDS_Shape> theInvertedFaces)
    {
        // The face should be considered as invalid in the following cases:
        // 1. It has been reverted, i.e. at least two not connected edges
        //    have changed orientation (i.e. invalid). In this case all edges,
        //    should be invalid for that face, because edges have also been reverted;
        // 2. All checked edges of the face are invalid for this face;
        // The face should be removed from the splits in the following cases:
        // 1. All checked edges of the face are invalid for this one, but valid for
        //    some other face in this list of splits.
        // The face will be kept in the following cases:
        // 1. Some of the edges are valid for this face.
        bool bHasValid, bAllValid, bAllInvalid, bHasReallyInvalid, bAllInvNeutral;
        bool bValid, bValidLoc, bInvalid, bInvalidLoc, bNeutral, bInverted;
        bool bIsInvalidByInverted, bHasInverted;
        int aNbChecked;
        //
        bool bTreatInvertedAsInvalid = (theLFImages.Extent() == 1);
        //
        // neutral edges to remove
        NCollection_IndexedMap<TopoDS_Shape> aMENRem = new(TopTools_ShapeMapHasher.Instance);
        //
        // faces for post treat
        NCollection_List<TopoDS_Shape> aLFPT = new();
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(theLFImages);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
            TopExp.MapShapesAndAncestors(aFIm, TopAbs_EDGE, TopAbs_FACE, aDMEF);
        }

        aItLF.Initialize(theLFImages);
        for (; aItLF.More();)
        {
            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
            //
            // valid edges for this split
            theDMFMVE.Find(aFIm, out NCollection_Map<TopoDS_Shape>? pMVE);
            // invalid edges for this split
            theDMFMIE.Find(aFIm, out NCollection_IndexedMap<TopoDS_Shape>? pMIE);
            //
            bHasValid = false;
            bAllValid = true;
            bAllInvalid = true;
            bHasReallyInvalid = false;
            bAllInvNeutral = true;
            bIsInvalidByInverted = true;
            bHasInverted = false;
            aNbChecked = 0;
            //
            TopoDS_Wire aWIm = BRepTools.OuterWire(aFIm);
            TopExp_Explorer aExp = new(aWIm, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aEIm = aExp.Current();
                //
                bValid = myValidEdges.Contains(aEIm);
                bInvalid = myInvalidEdges.Contains(aEIm);
                bNeutral = theMENeutral.Contains(aEIm);
                //
                if (!bValid && !bInvalid && !bNeutral)
                {
                    // edge has not been checked for some reason
                    continue;
                }

                // skip not-boundary edges originated from vertex
                if ((theEdgesInvalidByVertex.Contains(aEIm) || theEdgesValidByVertex.Contains(aEIm))
                    && aDMEF.FindFromKey(aEIm).Extent() != 1)
                {
                    continue;
                }

                ++aNbChecked;
                //
                bInvalidLoc = pMIE is not null && pMIE.Contains(aEIm);
                bHasReallyInvalid = bInvalid && bInvalidLoc && !bValid && !theEdgesInvalidByVertex.Contains(aEIm);
                if (bHasReallyInvalid)
                {
                    break;
                }
                //
                bValidLoc = pMVE is not null && pMVE.Contains(aEIm);
                bInverted = myInvertedEdges.Contains(aEIm);
                if (!bInvalid && !bInvalidLoc && bTreatInvertedAsInvalid)
                {
                    bInvalid = bInverted;
                }
                //
                if (bValidLoc && bNeutral)
                {
                    bHasValid = true;
                }
                //
                bAllValid &= bValidLoc;
                bAllInvalid &= (bInvalid || bInvalidLoc);
                bAllInvNeutral &= (bAllInvalid && bNeutral);
                bIsInvalidByInverted &= (bInvalidLoc || bInverted);
                bHasInverted |= bInverted;
            }
            //
            if (aNbChecked == 0)
            {
                aItLF.Next();
                continue;
            }
            //
            if (!bHasReallyInvalid && (bAllInvNeutral && !bHasValid) && (aNbChecked > 1))
            {
                if (bHasInverted)
                {
                    // The part seems to be filled due to overlapping of parts rather than
                    // due to multi-connection of faces. No need to remove the part.
                    aItLF.Next();
                    continue;
                }
                // remove edges from neutral
                TopExp.MapShapes(aFIm, TopAbs_EDGE, aMENRem);
                // remove face
                theLFImages.Remove(aItLF);
                continue;
            }
            //
            if (bHasReallyInvalid || (bAllInvalid && !(bHasValid || bAllValid) && (!bAllInvNeutral || (aNbChecked != 1))))
            {
                theInvFaces.Append(aFIm);
                if (theMFHoles.Contains(aFIm))
                {
                    theMFInvInHole.Add(aFIm);
                }
                aItLF.Next();
                continue;
            }
            //
            if (theMFHoles.Contains(aFIm))
            {
                // remove edges from neutral
                TopExp.MapShapes(aFIm, TopAbs_EDGE, aMENRem);
                // remove face
                theLFImages.Remove(aItLF);
                continue;
            }
            //
            if (bIsInvalidByInverted && !(bHasValid || bAllValid))
            {
                // The face contains only the inverted and locally invalid edges
                theInvertedFaces.Append(aFIm);
            }

            if (!bAllInvNeutral)
            {
                aLFPT.Append(aFIm);
            }
            else
            {
                // remove edges from neutral
                TopExp.MapShapes(aFIm, TopAbs_EDGE, aMENRem);
            }
            aItLF.Next();
        }
        //
        if (aLFPT.IsEmpty() || aMENRem.IsEmpty())
        {
            return;
        }

        // check the splits once more
        aItLF.Initialize(aLFPT);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
            //
            // valid edges for this split
            theDMFMVE.Find(aFIm, out NCollection_Map<TopoDS_Shape>? pMVE);
            //
            bHasValid = false;
            bAllValid = true;
            bAllInvalid = true;
            //
            TopoDS_Wire aWIm = BRepTools.OuterWire(aFIm);
            TopExp_Explorer aExp = new(aWIm, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aEIm = aExp.Current();
                //
                bValid = myValidEdges.Contains(aEIm);
                bInvalid = myInvalidEdges.Contains(aEIm);
                bNeutral = theMENeutral.Contains(aEIm) && !aMENRem.Contains(aEIm);
                bValidLoc = pMVE is not null && pMVE.Contains(aEIm);
                //
                if (!bInvalid && bTreatInvertedAsInvalid)
                {
                    bInvalid = myInvertedEdges.Contains(aEIm);
                }
                //
                if (bValidLoc && bNeutral)
                {
                    bHasValid = true;
                }
                //
                bAllValid = bAllValid && bValidLoc;
                bAllInvalid = bAllInvalid && bInvalid;
            }
            //
            if (bAllInvalid && !bHasValid && !bAllValid)
            {
                theInvFaces.Append(aFIm);
            }
        }
    }

    //=======================================================================
    // function : FindFacesInsideHoleWires
    // purpose  : Find faces inside holes wires from the original face
    //=======================================================================
    private void FindFacesInsideHoleWires(in TopoDS_Face theFOrigin, in TopoDS_Face theFOffset,
                                          NCollection_List<TopoDS_Shape> theLFImages,
                                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEOrLEIm,
                                          NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEFMap,
                                          NCollection_Map<TopoDS_Shape> theMFHoles)
    {
        if (theLFImages.IsEmpty())
        {
            return;
        }
        //
        // find all hole wires in the original face
        NCollection_List<TopoDS_Shape> aLHoleWires = new();
        TopoDS_Wire anOuterWire = BRepTools.OuterWire(theFOrigin);
        TopExp_Explorer aExpW = new(theFOrigin, TopAbs_WIRE);
        for (; aExpW.More(); aExpW.Next())
        {
            TopoDS_Wire aHoleWire = TopoDS.Wire(aExpW.Current());
            if (!aHoleWire.IsSame(anOuterWire) && aHoleWire.Orientation() != TopAbs_INTERNAL)
            {
                aLHoleWires.Append(aHoleWire);
            }
        }
        //
        if (aLHoleWires.IsEmpty())
        {
            // no holes in the face
            return;
        }
        //
        if (!myFNewHoles.Find(theFOrigin, out NCollection_List<TopoDS_Shape> pLFNewHoles))
        {
            pLFNewHoles = new NCollection_List<TopoDS_Shape>();
            myFNewHoles.Bind(theFOrigin, pLFNewHoles);
        }
        if (pLFNewHoles.IsEmpty())
        {
            //
            // find the faces representing holes in the images of the faces:
            // 1. for each original hole wire try to build its image
            // 2. build the new planar face from the images
            //
            // map vertices and edges of the splits
            NCollection_IndexedMap<TopoDS_Shape> aMESplits = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape>.Iterator aItLF0 = new(theLFImages);
            for (; aItLF0.More(); aItLF0.Next())
            {
                TopExp.MapShapes(aItLF0.Value(), TopAbs_EDGE, aMESplits);
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLW = new(aLHoleWires);
            for (; aItLW.More(); aItLW.Next())
            {
                TopoDS_Wire aHoleWire = TopoDS.Wire(aItLW.Value());
                // find images of all edges of the original wire
                NCollection_IndexedMap<TopoDS_Shape> aMEImWire = new(TopTools_ShapeMapHasher.Instance);
                TopoDS_Iterator aItE = new(aHoleWire);
                for (; aItE.More(); aItE.Next())
                {
                    TopoDS_Shape aEOr = aItE.Value();
                    if (!theDMEOrLEIm.Find(aEOr, out NCollection_List<TopoDS_Shape> pLEIm) || pLEIm.IsEmpty())
                    {
                        continue;
                    }
                    NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
                    for (; aItLEIm.More(); aItLEIm.Next())
                    {
                        TopoDS_Shape aEIm = aItLEIm.Value();
                        if (aMESplits.Contains(aEIm))
                        {
                            aMEImWire.Add(aEIm);
                        }
                    }
                }
                //
                if (aMEImWire.IsEmpty())
                {
                    continue;
                }
                //
                // build new planar face using these edges
                NCollection_List<TopoDS_Shape> aLE = new();
                int i, aNbE0 = aMEImWire.Extent();
                for (i = 1; i <= aNbE0; ++i)
                {
                    aLE.Append(aMEImWire[i].Oriented(TopAbs_FORWARD));
                    aLE.Append(aMEImWire[i].Oriented(TopAbs_REVERSED));
                }
                //
                BOPAlgo_BuilderFace aBF = new();
                aBF.SetFace(TopoDS.Face(theFOffset.Oriented(TopAbs_FORWARD)));
                aBF.SetShapes(aLE);
                aBF.Perform();
                //
                NCollection_List<TopoDS_Shape> aLFNew = aBF.Areas();
                if (aLFNew.IsEmpty())
                {
                    continue;
                }
                //
                // check if outer edges in the new faces are not inverted
                // because the inverted edges mean that the hole has been
                // filled during offset and there will be no faces to remove
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEFNew = new(TopTools_ShapeMapHasher.Instance);
                NCollection_List<TopoDS_Shape>.Iterator aItLFNew = new(aLFNew);
                for (; aItLFNew.More(); aItLFNew.Next())
                {
                    TopExp.MapShapesAndAncestors(aItLFNew.Value(), TopAbs_EDGE, TopAbs_FACE, aDMEFNew);
                }
                //
                aNbE0 = aDMEFNew.Extent();
                for (i = 1; i <= aNbE0; ++i)
                {
                    if (aDMEFNew[i].Extent() == 1)
                    {
                        TopoDS_Shape aE = aDMEFNew.FindKey(i);
                        if (myInvertedEdges.Contains(aE))
                        {
                            break;
                        }
                    }
                }
                //
                if (i <= aNbE0)
                {
                    continue;
                }
                //
                aItLFNew.Initialize(aLFNew);
                for (; aItLFNew.More(); aItLFNew.Next())
                {
                    pLFNewHoles.Append(aItLFNew.Value());
                }
            }
        }

        // Build Edge-Face map for splits of current offset face
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEFSplitsMap = new(TopTools_ShapeMapHasher.Instance);
        // Build Edge-Face map for holes
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEFHolesMap = new(TopTools_ShapeMapHasher.Instance);

        // among the splits of the offset face find those that are
        // located inside the hole faces
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(theLFImages);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
            TopExp.MapShapesAndAncestors(aFIm, TopAbs_EDGE, TopAbs_FACE, anEFSplitsMap);
            // get the point inside the face and classify it relatively hole faces
            Point3D aP3D = new();
            Point2D aP2D = new();
            int iErr = BOPTools_AlgoTools3D.PointInFace(aFIm, ref aP3D, ref aP2D, myContext);
            if (iErr != 0)
            {
                continue;
            }
            //
            double aTol = BRep_Tool.Tolerance(aFIm);
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLFNew = new(pLFNewHoles);
            for (; aItLFNew.More(); aItLFNew.Next())
            {
                TopoDS_Face aFNew = TopoDS.Face(aItLFNew.Value());
                if (myContext.IsValidPointForFace(aP3D, aFNew, aTol))
                {
                    // the face is classified as IN
                    theMFHoles.Add(aFIm);
                    TopExp.MapShapesAndAncestors(aFIm, TopAbs_EDGE, TopAbs_FACE, anEFHolesMap);
                    break;
                }
            }
        }

        // Out of all found holes find those which cannot be removed
        // by checking their connectivity to splits of other offset faces.
        // These are the faces, which will create uncovered holes if removed.
        int aNbE = anEFHolesMap.Extent();
        for (int i = 1; i <= aNbE; ++i)
        {
            TopoDS_Shape anEdge = anEFHolesMap.FindKey(i);
            NCollection_List<TopoDS_Shape> aLFHoles = anEFHolesMap[i];
            // Check if the edge is outer for holes
            if (aLFHoles.Extent() != 1)
            {
                continue;
            }

            TopoDS_Shape aFHole = aLFHoles.First();
            if (!theMFHoles.Contains(aFHole))
            {
                // Already removed
                continue;
            }

            // Check if the edge is not outer for splits
            NCollection_List<TopoDS_Shape> aLSplits = anEFSplitsMap.FindFromKey(anEdge);
            if (aLSplits.Extent() == 1)
            {
                continue;
            }

            // Check if edge is only connected to splits of the current offset face
            NCollection_List<TopoDS_Shape> aLFAll = theEFMap.FindFromKey(anEdge);
            if (aLFAll.Extent() == 2)
            {
                // Avoid removal of the hole from the splits
                theMFHoles.Remove(aFHole);
            }
        }
    }

    //=======================================================================
    // function : CheckInverted
    // purpose  : Checks if the edge has been inverted
    //=======================================================================
    private bool CheckInverted(in TopoDS_Edge theEIm, in TopoDS_Face theFOr,
                               NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVE,
                               NCollection_IndexedMap<TopoDS_Shape> theMEdges)
    {
        // It is necessary to compare the direction from first vertex
        // to the last vertex on the original edge with the
        // same direction on the new edge. If the directions
        // will be different - the edge has been inverted.
        //
        TopoDS_Vertex aVO1 = new(), aVO2 = new(); // vertices on the original edge
        //
        int i;
        // find vertices of the offset shape
        TopExp.Vertices(theEIm, out TopoDS_Vertex aVI1, out TopoDS_Vertex aVI2); // vertices on the offset edge
        //
        // find images
        NCollection_List<TopoDS_Shape> aLEImages = new();
        if (myOEOrigins.IsBound(theEIm))
        {
            TopoDS_Wire anImages = new();
            new BRep_Builder().MakeWire(ref anImages);
            //
            NCollection_Map<TopoDS_Shape> aMImFence = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape> aLOffsetOr = myOEOrigins.Find(theEIm);
            NCollection_List<TopoDS_Shape>.Iterator aItOffset = new(aLOffsetOr);
            for (; aItOffset.More(); aItOffset.Next())
            {
                TopoDS_Shape aEOffsetOr = aItOffset.Value();
                NCollection_List<TopoDS_Shape> aLImages = myOEImages.Find(aEOffsetOr);
                //
                NCollection_List<TopoDS_Shape>.Iterator aItImages = new(aLImages);
                for (; aItImages.More(); aItImages.Next())
                {
                    TopoDS_Edge anIm = TopoDS.Edge(aItImages.Value());
                    if (theMEdges.Contains(anIm) && aMImFence.Add(anIm))
                    {
                        new BRep_Builder().Add(anImages, anIm);
                        aLEImages.Append(anIm);
                    }
                }
            }
            //
            // find alone vertices
            TopoDS_Vertex aVW1, aVW2;
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMImVE = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(anImages, TopAbs_VERTEX, TopAbs_EDGE, aDMImVE);
            //
            NCollection_List<TopoDS_Shape> aLVAlone = new();
            int aNb = aDMImVE.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                NCollection_List<TopoDS_Shape> aLImE = aDMImVE[i];
                if (aLImE.Extent() == 1)
                {
                    aLVAlone.Append(aDMImVE.FindKey(i));
                }
            }
            //
            if (aLVAlone.Extent() > 1)
            {
                aVW1 = TopoDS.Vertex(aLVAlone.First());
                aVW2 = TopoDS.Vertex(aLVAlone.Last());
                //
                // check distances
                Point3D aPI1_ = BRep_Tool.Pnt(aVI1);
                Point3D aPW1 = BRep_Tool.Pnt(aVW1);
                Point3D aPW2 = BRep_Tool.Pnt(aVW2);
                //
                double aDist1 = aPI1_.SquareDistance(aPW1);
                double aDist2 = aPI1_.SquareDistance(aPW2);
                //
                if (aDist1 < aDist2)
                {
                    aVI1 = aVW1;
                    aVI2 = aVW2;
                }
                else
                {
                    aVI1 = aVW2;
                    aVI2 = aVW1;
                }
            }
        }
        else
        {
            aLEImages.Append(theEIm);
        }
        //
        // Find edges connected to these vertices
        NCollection_List<TopoDS_Shape> aLIE1 = theDMVE.FindFromKey(aVI1);
        NCollection_List<TopoDS_Shape> aLIE2 = theDMVE.FindFromKey(aVI2);
        //
        // Find vertices on the original face corresponding to vertices on the offset edge
        //
        // find original edges for both lists
        NCollection_List<TopoDS_Shape> aLOE1 = new(), aLOE2 = new();
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<TopoDS_Shape> aLIE = i == 0 ? aLIE1 : aLIE2;
            NCollection_List<TopoDS_Shape> aLOE = i == 0 ? aLOE1 : aLOE2;
            //
            NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLIE = new(aLIE);
            for (; aItLIE.More(); aItLIE.Next())
            {
                TopoDS_Shape aEI = aItLIE.Value();
                if (myEdgesOrigins!.IsBound(aEI))
                {
                    NCollection_List<TopoDS_Shape> aLEOrigins = myEdgesOrigins.Find(aEI);
                    //
                    NCollection_List<TopoDS_Shape>.Iterator aItLOE = new(aLEOrigins);
                    for (; aItLOE.More(); aItLOE.Next())
                    {
                        TopoDS_Shape aEO = aItLOE.Value();
                        if (aEO.ShapeType() == TopAbs_EDGE && aMFence.Add(aEO))
                        {
                            TopoDS_Shape aEOin = new();
                            if (FindShape(aEO, theFOr, null, ref aEOin))
                            {
                                AppendToList(aLOE, aEO);
                            }
                        }
                    }
                }
            }
        }
        //
        if (aLOE1.Extent() < 2 || aLOE2.Extent() < 2)
        {
            return false;
        }
        //
        // find vertices common for the max number of edges in the lists
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<TopoDS_Shape> aLOE = i == 0 ? aLOE1 : aLOE2;
            ref TopoDS_Vertex aVO = ref (i == 0 ? ref aVO1 : ref aVO2);

            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVELoc = new(TopTools_ShapeMapHasher.Instance);
            for (NCollection_List<TopoDS_Shape>.Iterator itLOE = new(aLOE); itLOE.More(); itLOE.Next())
            {
                TopExp.MapShapesAndAncestors(itLOE.Value(), TopAbs_VERTEX, TopAbs_EDGE, aDMVELoc);
            }

            int aNbEMax = 0;
            for (int j = 1; j <= aDMVELoc.Extent(); ++j)
            {
                int aNbE = aDMVELoc[j].Extent();
                if (aNbE > 1 && aNbE > aNbEMax)
                {
                    aVO = TopoDS.Vertex(aDMVELoc.FindKey(j));
                    aNbEMax = aNbE;
                }
            }
            if (aVO.IsNull())
            {
                return false;
            }
        }

        if (aVO1.IsSame(aVO2))
        {
            return false;
        }
        //
        // check positions of the offset and original vertices
        Point3D aPI1 = BRep_Tool.Pnt(aVI1);
        Point3D aPI2 = BRep_Tool.Pnt(aVI2);
        Point3D aPO1 = BRep_Tool.Pnt(aVO1);
        Point3D aPO2 = BRep_Tool.Pnt(aVO2);
        //
        Vector3D aVI = new(aPI1, aPI2);
        Vector3D aVOv = new(aPO1, aPO2);
        //
        double anAngle = aVI.Angle(aVOv);
        bool bInverted = Math.Abs(anAngle - M_PI) < 1.0e-4;
        if (bInverted)
        {
            NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(aLEImages);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                TopoDS_Shape aEInvr = aItLEIm.Value();
                myInvertedEdges.Add(aEInvr);
            }
        }
        return bInverted;
    }

    //=======================================================================
    // function : GetVerticesOnEdges
    // purpose  : Get vertices from the given shape belonging to the given edges
    //=======================================================================
    private static void GetVerticesOnEdges(in TopoDS_Shape theCB, NCollection_IndexedMap<TopoDS_Shape> theEdges,
                                           NCollection_Map<TopoDS_Shape> theVerticesOnEdges,
                                           NCollection_Map<TopoDS_Shape> theAllVertices)
    {
        TopExp_Explorer aExp = new(theCB, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aE = aExp.Current();
            bool isOnGivenEdges = theEdges.Contains(aE);
            for (TopoDS_Iterator aItV = new(aE); aItV.More(); aItV.Next())
            {
                theAllVertices.Add(aItV.Value());
                if (isOnGivenEdges)
                {
                    theVerticesOnEdges.Add(aItV.Value());
                }
            }
        }
    }

    //=======================================================================
    // function : CheckInvertedBlock
    // purpose  : Checks if it is possible to remove the block containing inverted edges
    //=======================================================================
    private bool CheckInvertedBlock(in TopoDS_Shape theCB, NCollection_List<TopoDS_Shape> theLCBF,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMCBVInverted,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMCBVAll)
    {
        // For possible removal of the block:
        // 1. There should be more than just one face in the block
        if (theCB.NbChildren() < 2)
        {
            return false;
        }
        //
        // 2. The block should at least contain two connected inverted edges with
        //    different origins (not just two images/splits of the same edge)
        NCollection_Map<TopoDS_Shape> aMECBInv = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Compound aCECBInv = new();
        new BRep_Builder().MakeCompound(ref aCECBInv);
        //
        TopExp_Explorer aExp = new(theCB, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aE = aExp.Current();
            if (myInvertedEdges.Contains(aE))
            {
                if (aMECBInv.Add(aE))
                {
                    new BRep_Builder().Add(aCECBInv, aE);
                }
            }
        }
        //
        if (aMECBInv.Extent() < 2)
        {
            return false;
        }
        //
        // check that the edges are connected and different
        NCollection_List<TopoDS_Shape> aLCBE = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCECBInv, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
        //
        NCollection_List<TopoDS_Shape>.Iterator aItLCBE = new(aLCBE);
        for (; aItLCBE.More(); aItLCBE.Next())
        {
            TopoDS_Shape aCBE = aItLCBE.Value();
            // count the unique edges in the block
            int aNbUnique = 0;
            NCollection_Map<TopoDS_Shape> aMEOrigins = new(TopTools_ShapeMapHasher.Instance);
            TopoDS_Iterator aItE = new(aCBE);
            for (; aItE.More(); aItE.Next())
            {
                TopoDS_Shape aE = aItE.Value();
                if (!myOEOrigins.Find(aE, out NCollection_List<TopoDS_Shape> pLEOr))
                {
                    aMEOrigins.Add(aE);
                    ++aNbUnique;
                    continue;
                }
                NCollection_List<TopoDS_Shape>.Iterator aItLEOr = new(pLEOr);
                for (; aItLEOr.More(); aItLEOr.Next())
                {
                    TopoDS_Shape aEOr = aItLEOr.Value();
                    if (aMEOrigins.Add(aEOr))
                    {
                        ++aNbUnique;
                    }
                }
            }
            //
            if (aNbUnique >= 2)
            {
                break;
            }
        }
        //
        if (!aItLCBE.More())
        {
            return false;
        }
        //
        // 3. the block should not contain inverted edges which vertices
        //    are contained in other blocks
        //
        // collect vertices from inverted edges and compare them with
        // vertices from other blocks
        theDMCBVInverted.Find(theCB, out NCollection_Map<TopoDS_Shape>? pMVInverted);
        theDMCBVAll.Find(theCB, out NCollection_Map<TopoDS_Shape>? pMVAll);
        if (pMVInverted is null)
        {
            pMVInverted = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            theDMCBVInverted.Bind(theCB, pMVInverted);
            pMVAll = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            theDMCBVAll.Bind(theCB, pMVAll);
            //
            GetVerticesOnEdges(theCB, myInvertedEdges, pMVInverted, pMVAll);
        }
        //
        NCollection_List<TopoDS_Shape>.Iterator aItLCB1 = new(theLCBF);
        for (; aItLCB1.More(); aItLCB1.Next())
        {
            TopoDS_Shape aCB1 = aItLCB1.Value();
            if (aCB1.IsSame(theCB))
            {
                continue;
            }
            //
            // collect vertices from inverted edges
            theDMCBVInverted.Find(aCB1, out NCollection_Map<TopoDS_Shape>? pMVInverted1);
            theDMCBVAll.Find(aCB1, out NCollection_Map<TopoDS_Shape>? pMVAll1);
            if (pMVInverted1 is null)
            {
                pMVInverted1 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                theDMCBVInverted.Bind(aCB1, pMVInverted1);
                pMVAll1 = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
                theDMCBVAll.Bind(aCB1, pMVAll1);
                //
                GetVerticesOnEdges(aCB1, myInvertedEdges, pMVInverted1, pMVAll1);
            }
            //
            if (NCollection_MapAlgo.HasIntersection(pMVInverted, pMVAll1!))
            {
                return false;
            }
        }
        //
        return true;
    }

    //=======================================================================
    // function : RemoveInvalidSplitsByInvertedEdges
    // purpose  : Looking for the invalid faces containing inverted edges
    //           that can be safely removed
    //=======================================================================
    private void RemoveInvalidSplitsByInvertedEdges(NCollection_IndexedMap<TopoDS_Shape> theMERemoved)
    {
        if (myInvertedEdges.IsEmpty())
        {
            return;
        }
        //
        // check the faces on regularity, i.e. the splits of the same face
        // should not be connected only by vertex. Such irregular splits
        // will have to be rebuilt and cannot be removed.
        //
        BRep_Builder aBB = new();
        NCollection_IndexedMap<TopoDS_Shape> aMEAvoid = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVF = new(TopTools_ShapeMapHasher.Instance);
        int aNb = myOFImages.Extent(), i;
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages[i];
            //
            TopoDS_Compound aCFIm = new();
            aBB.MakeCompound(ref aCFIm);
            //
            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLFIm);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aF = aIt.Value();
                aBB.Add(aCFIm, aF);
                //
                // make a map to use only outer edges
                TopExp_Explorer aExp = new(aF, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    //
                    if (!aDMEF.Find(aE, out NCollection_List<TopoDS_Shape> pLF))
                    {
                        pLF = new NCollection_List<TopoDS_Shape>();
                        aDMEF.Bind(aE, pLF);
                    }
                    else
                    {
                        // internal edges should not be used
                        aMEAvoid.Add(aE);
                    }
                    AppendToList(pLF, aF);
                }
                //
                // fill connection map of the vertices of inverted edges to faces
                aExp.Init(aF, TopAbs_VERTEX);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aV = aExp.Current();
                    //
                    if (!aDMVF.Find(aV, out NCollection_List<TopoDS_Shape> pLF))
                    {
                        pLF = new NCollection_List<TopoDS_Shape>();
                        aDMVF.Bind(aV, pLF);
                    }
                    AppendToList(pLF, aF);
                }
            }
            //
            // for the splits to be regular they should form only one block
            NCollection_List<TopoDS_Shape> aLCBF = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCFIm, TopAbs_EDGE, TopAbs_FACE, aLCBF);
            if (aLCBF.Extent() == 1)
            {
                continue;
            }
            //
            // check if the inverted edges create the irregularity
            NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> aDMCBVInverted = new(TopTools_ShapeMapHasher.Instance),
                                                                             aDMCBVAll = new(TopTools_ShapeMapHasher.Instance);
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCBF);
            for (; aItLCB.More(); aItLCB.Next())
            {
                TopoDS_Shape aCB = aItLCB.Value();
                //
                // check if it is possible to remove the block
                if (!CheckInvertedBlock(aCB, aLCBF, aDMCBVInverted, aDMCBVAll))
                {
                    // non of the edges in this block should be removed
                    TopExp.MapShapes(aCB, TopAbs_EDGE, aMEAvoid);
                    continue;
                }
            }
        }
        //
        // all edges not included in aMEAvoid can be removed
        NCollection_Map<TopoDS_Shape> aMERem = new(TopTools_ShapeMapHasher.Instance);
        for (int iInverted = 1; iInverted <= myInvertedEdges.Extent(); ++iInverted)
        {
            TopoDS_Shape aE = myInvertedEdges[iInverted];
            if (!aMEAvoid.Contains(aE))
            {
                TopoDS_Iterator aIt = new(aE);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aV = aIt.Value();
                    if (aDMVF.Find(aV, out NCollection_List<TopoDS_Shape> pLF) && (pLF.Extent() > 3))
                    {
                        aMERem.Add(aE);
                        break;
                    }
                }
            }
        }
        //
        if (aMERem.IsEmpty())
        {
            return;
        }
        //
        // all invalid faces containing these edges can be removed
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aInvFaces = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aMFRem = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMFToUpdate = new(TopTools_ShapeMapHasher.Instance);
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            NCollection_List<TopoDS_Shape> aLFIm = myInvalidFaces[i];
            //
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLFIm);
            for (; aIt.More();)
            {
                TopoDS_Shape aFIm = aIt.Value();
                //
                // to be removed the face should have at least two not connected
                // inverted edges
                TopoDS_Compound aCEInv = new();
                aBB.MakeCompound(ref aCEInv);
                TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    if (aMERem.Contains(aE))
                    {
                        aBB.Add(aCEInv, aE);
                    }
                }
                //
                // check connectivity
                NCollection_List<TopoDS_Shape> aLCBE = new();
                BOPTools_AlgoTools.MakeConnexityBlocks(aCEInv, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
                //
                if (aLCBE.Extent() >= 2)
                {
                    aMFToUpdate.Add(aF);
                    aMFRem.Add(aFIm);
                    aLFIm.Remove(aIt);
                }
                else
                {
                    aIt.Next();
                }
            }
            //
            if (aLFIm.Extent() != 0)
            {
                aInvFaces.Add(aF, new NCollection_List<TopoDS_Shape>(aLFIm));
            }
        }
        //
        if (aMFRem.IsEmpty())
        {
            return;
        }
        //
        myInvalidFaces = aInvFaces;
        // remove from splits
        aNb = aMFToUpdate.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = aMFToUpdate[i];
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages.ChangeFromKey(aF);
            //
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLFIm);
            for (; aIt.More();)
            {
                TopoDS_Shape aFIm = aIt.Value();
                if (aMFRem.Contains(aFIm))
                {
                    TopExp.MapShapes(aFIm, TopAbs_EDGE, theMERemoved);
                    aLFIm.Remove(aIt);
                }
                else
                {
                    aIt.Next();
                }
            }
        }
    }

    //=======================================================================
    // function : RemoveInvalidSplitsFromValid
    // purpose  : Removing invalid splits of faces from valid
    //=======================================================================
    private void RemoveInvalidSplitsFromValid(NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theDMFMVIE)
    {
        // Decide whether to remove the found invalid faces or not.
        // The procedure is the following:
        // 1. Make connexity blocks from invalid faces;
        // 2. Find free edges in this blocks;
        // 3. If all free edges are valid for the faces - remove block.
        //
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance), aMFToRem = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Compound aCFInv = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aCFInv);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        //
        // make compound of invalid faces
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDMIFOF = new(TopTools_ShapeMapHasher.Instance);
        int i, aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            // artificially invalid faces should not be removed
            if (myArtInvalidFaces.IsBound(aF))
            {
                continue;
            }
            NCollection_List<TopoDS_Shape> aLFInv = myInvalidFaces[i];
            aItLF.Initialize(aLFInv);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aFIm = aItLF.Value();
                if (aMFence.Add(aFIm))
                {
                    aBB.Add(aCFInv, aFIm);
                    aDMIFOF.Bind(aFIm, aF);
                }
            }
        }
        //
        // make connexity blocks
        NCollection_List<TopoDS_Shape> aLCBInv = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFInv, TopAbs_EDGE, TopAbs_FACE, aLCBInv);
        //
        // analyze each block
        aItLF.Initialize(aLCBInv);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Shape aCB = aItLF.Value();
            //
            // if connexity block contains only one face - it should be removed;
            TopExp_Explorer aExp = new(aCB, TopAbs_FACE);
            aExp.Next();
            if (aExp.More())
            {
                // check if there are valid images left
                aExp.Init(aCB, TopAbs_FACE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aFIm = aExp.Current();
                    TopoDS_Shape aF = aDMIFOF.Find(aFIm);
                    //
                    NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(aF);
                    NCollection_List<TopoDS_Shape> aLFInv = myInvalidFaces.FindFromKey(aF);
                    //
                    if (aLFIm.Extent() == aLFInv.Extent())
                    {
                        break;
                    }
                }
            }
            //
            if (!aExp.More())
            {
                aExp.Init(aCB, TopAbs_FACE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aF = aExp.Current();
                    aMFToRem.Add(aF);
                }
                continue;
            }
            //
            // remove faces connected by inverted edges
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(aCB, TopAbs_EDGE, TopAbs_FACE, aDMEF);
            //
            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMFF = new(TopTools_ShapeMapHasher.Instance);
            aExp.Init(aCB, TopAbs_FACE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aFCB = aExp.Current();
                TopoDS_Shape aF = aDMIFOF.Find(aFCB);
                if (!aDMFF.Find(aF, out NCollection_List<TopoDS_Shape> pList))
                {
                    pList = new NCollection_List<TopoDS_Shape>();
                    aDMFF.Bind(aF, pList);
                }
                pList.Append(aFCB);
            }

            // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itM in aDMFF)
            {
                TopoDS_Shape aF = itM.Key;
                theDMFMVIE.Find(aF, out NCollection_Map<TopoDS_Shape>? pValidInverted);

                // either remove all of these faces or none.
                NCollection_List<TopoDS_Shape> aLFCB = itM.Value;
                NCollection_List<TopoDS_Shape>.Iterator itL = new(aLFCB);
                for (; itL.More(); itL.Next())
                {
                    TopoDS_Shape aFCB = itL.Value();
                    TopExp_Explorer aExpE = new(aFCB, TopAbs_EDGE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        TopoDS_Shape aECB = aExpE.Current();
                        if (pValidInverted is not null && pValidInverted.Contains(aECB))
                        {
                            break;
                        }
                        if (aDMEF.FindFromKey(aECB).Extent() > 1)
                        {
                            if (!myInvertedEdges.Contains(aECB))
                            {
                                break;
                            }
                        }
                    }
                    if (!aExpE.More())
                    {
                        // if one removed - remove all
                        break;
                    }
                }
                if (itL.More())
                {
                    for (itL.Initialize(aLFCB); itL.More(); itL.Next())
                    {
                        aMFToRem.Add(itL.Value());
                    }
                }
            }
        }
        //
        if (aMFToRem.Extent() != 0)
        {
            // remove invalid faces from images
            aNb = myInvalidFaces.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                TopoDS_Shape aF = myInvalidFaces.FindKey(i);
                NCollection_List<TopoDS_Shape> aLFImages = myOFImages.ChangeFromKey(aF);
                aItLF.Initialize(aLFImages);
                for (; aItLF.More();)
                {
                    TopoDS_Shape aFIm = aItLF.Value();
                    if (aMFToRem.Contains(aFIm))
                    {
                        aLFImages.Remove(aItLF);
                    }
                    else
                    {
                        aItLF.Next();
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void buildPairs(NCollection_IndexedMap<TopoDS_Shape> theSMap,
                                   NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> theIntPairs)
    {
        int aNbS = theSMap.Extent();
        if (aNbS < 2)
        {
            return;
        }
        for (int it1 = 1; it1 <= aNbS; ++it1)
        {
            TopoDS_Shape aS = theSMap[it1];
            if (!theIntPairs.IsBound(aS))
            {
                theIntPairs.Bind(aS, new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance));
            }
        }

        for (int it1 = 1; it1 <= aNbS; ++it1)
        {
            TopoDS_Shape aS1 = theSMap[it1];
            NCollection_Map<TopoDS_Shape> aMap1 = theIntPairs[aS1];
            for (int it2 = it1 + 1; it2 <= aNbS; ++it2)
            {
                TopoDS_Shape aS2 = theSMap[it2];
                aMap1.Add(aS2);
                theIntPairs[aS2].Add(aS1);
            }
        }
    }

    //=================================================================================================

    private static void buildIntersectionPairs(
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOFImages,
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myInvalidFaces, BOPAlgo_Builder theBuilder,
        NCollection_Map<TopoDS_Shape> theMFRemoved, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theFOrigins,
        NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>>> theIntPairs)
    {
        TopAbs_ShapeEnum aCType = TopAbs_VERTEX;
        // Build connection map from vertices to faces
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theBuilder.InternalShape(), aCType, TopAbs_FACE, aDMVF);

        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anImages = theBuilder.Images();
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anOrigins = theBuilder.Origins();

        // Find all faces connected to the not removed faces and build intersection pairs among them.
        // For removed faces intersect only those connected to each other.

        for (int iF = 1; iF <= myInvalidFaces.Extent(); ++iF)
        {
            TopoDS_Shape aFInv = myInvalidFaces.FindKey(iF);

            TopoDS_Compound aCF = new(), aCFRem = new();
            new BRep_Builder().MakeCompound(ref aCF);
            new BRep_Builder().MakeCompound(ref aCFRem);

            for (int iC = 0; iC < 2; ++iC)
            {
                NCollection_List<TopoDS_Shape> aLF = iC == 0 ? myInvalidFaces[iF] : myOFImages.FindFromKey(aFInv);

                for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLF); it.More(); it.Next())
                {
                    NCollection_List<TopoDS_Shape> aLFIm = new();
                    TakeModified(it.Value(), anImages, aLFIm);

                    for (NCollection_List<TopoDS_Shape>.Iterator itIm = new(aLFIm); itIm.More(); itIm.Next())
                    {
                        TopoDS_Shape aFIm = itIm.Value();

                        if (theMFRemoved.Contains(aFIm))
                        {
                            new BRep_Builder().Add(aCFRem, aFIm);
                        }
                        else
                        {
                            new BRep_Builder().Add(aCF, aFIm);
                        }
                    }
                }
            }

            NCollection_List<TopoDS_Shape> aLCB = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCF, TopAbs_EDGE, TopAbs_FACE, aLCB);

            if (aLCB.IsEmpty())
            {
                continue;
            }

            NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> pFInterMap = new(TopTools_ShapeMapHasher.Instance);
            theIntPairs.Bind(aFInv, pFInterMap);

            // build pairs for not removed faces
            for (NCollection_List<TopoDS_Shape>.Iterator itCB = new(aLCB); itCB.More(); itCB.Next())
            {
                TopoDS_Shape aCB = itCB.Value();

                NCollection_IndexedMap<TopoDS_Shape> aMFInter = new(TopTools_ShapeMapHasher.Instance);
                for (TopExp_Explorer exp = new(aCB, aCType); exp.More(); exp.Next())
                {
                    TopoDS_Shape aCS = exp.Current();
                    if (!aDMVF.FindFromKey(aCS, out NCollection_List<TopoDS_Shape> pLFV))
                    {
                        continue;
                    }

                    for (NCollection_List<TopoDS_Shape>.Iterator itFV = new(pLFV); itFV.More(); itFV.Next())
                    {
                        TopoDS_Shape aFConnected = itFV.Value();

                        NCollection_List<TopoDS_Shape> aLFOr = new();
                        TakeModified(aFConnected, anOrigins, aLFOr);
                        for (NCollection_List<TopoDS_Shape>.Iterator itOr = new(aLFOr); itOr.More(); itOr.Next())
                        {
                            if (theFOrigins.Find(itOr.Value(), out TopoDS_Shape pFOr))
                            {
                                aMFInter.Add(pFOr);
                            }
                        }
                    }
                }

                // build intersection pairs
                buildPairs(aMFInter, pFInterMap);
            }

            aLCB.Clear();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCFRem, TopAbs_EDGE, TopAbs_FACE, aLCB);

            if (aLCB.IsEmpty())
            {
                continue;
            }

            for (NCollection_List<TopoDS_Shape>.Iterator itCB = new(aLCB); itCB.More(); itCB.Next())
            {
                TopoDS_Shape aCB = itCB.Value();

                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance);
                for (TopExp_Explorer exp = new(aCB, aCType); exp.More(); exp.Next())
                {
                    TopoDS_Shape aCS = exp.Current();
                    if (!aDMVF.FindFromKey(aCS, out NCollection_List<TopoDS_Shape> pLFV))
                    {
                        continue;
                    }

                    for (NCollection_List<TopoDS_Shape>.Iterator itFV = new(pLFV); itFV.More(); itFV.Next())
                    {
                        TopoDS_Shape aFConnected = itFV.Value();
                        TopExp.MapShapesAndAncestors(aFConnected, TopAbs_EDGE, TopAbs_FACE, aDMEF);
                    }
                }

                for (int iE = 1; iE <= aDMEF.Extent(); ++iE)
                {
                    NCollection_List<TopoDS_Shape> aLFConnected = aDMEF[iE];
                    if (aLFConnected.Extent() < 2)
                    {
                        continue;
                    }

                    NCollection_IndexedMap<TopoDS_Shape> aMFInter = new(TopTools_ShapeMapHasher.Instance);
                    for (NCollection_List<TopoDS_Shape>.Iterator itLF = new(aLFConnected); itLF.More(); itLF.Next())
                    {
                        TopoDS_Shape aFConnected = itLF.Value();

                        NCollection_List<TopoDS_Shape> aLFOr = new();
                        TakeModified(aFConnected, anOrigins, aLFOr);
                        for (NCollection_List<TopoDS_Shape>.Iterator itOr = new(aLFOr); itOr.More(); itOr.Next())
                        {
                            if (theFOrigins.Find(itOr.Value(), out TopoDS_Shape pFOr))
                            {
                                aMFInter.Add(pFOr);
                            }
                        }
                    }

                    buildPairs(aMFInter, pFInterMap);
                }
            }
        }
    }

    //=======================================================================
    // function : RemoveInsideFaces
    // purpose  : Looking for the inside faces that can be safely removed
    //=======================================================================
    private void RemoveInsideFaces(NCollection_List<TopoDS_Shape> theInvertedFaces,
                                   NCollection_IndexedMap<TopoDS_Shape> theMFToCheckInt,
                                   NCollection_IndexedMap<TopoDS_Shape> theMFInvInHole, in TopoDS_Shape theFHoles,
                                   NCollection_IndexedMap<TopoDS_Shape> theMERemoved,
                                   NCollection_IndexedMap<TopoDS_Shape> theMEInside, ProgressRange theRange)
    {
        NCollection_List<TopoDS_Shape> aLS = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMFInv = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDMFImF = new(TopTools_ShapeMapHasher.Instance);
        //
        using ProgressScope aPS = new(theRange, "Looking for inside faces", 10);
        int i, aNb = myOFImages.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Shape aF = myOFImages.FindKey(i);
            // to avoid intersection of the splits of the same
            // offset faces among themselves make compound of the
            // splits and use it as one argument
            TopoDS_Compound aCFImi = new();
            new BRep_Builder().MakeCompound(ref aCFImi);
            //
            for (int j = 0; j < 2; ++j)
            {
                NCollection_List<TopoDS_Shape>? pLFSp = j == 0
                    ? (myInvalidFaces.FindFromKey(aF, out NCollection_List<TopoDS_Shape> aLInv) ? aLInv : null)
                    : myOFImages[i];
                if (pLFSp is null)
                {
                    continue;
                }
                //
                aItLF.Initialize(pLFSp);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFIm = aItLF.Value();
                    if (aMFence.Add(aFIm))
                    {
                        new BRep_Builder().Add(aCFImi, aFIm);
                        aDMFImF.Bind(aFIm, aF);
                        if (j == 0)
                        {
                            aMFInv.Add(aFIm);
                        }
                    }
                }
            }
            //
            aLS.Append(aCFImi);
        }
        //
        // to make the solids more complete add for intersection also the faces
        // consisting only of invalid edges and not included into splits
        aNb = theMFToCheckInt.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aFSp = theMFToCheckInt[i];
            if (aMFence.Add(aFSp))
            {
                aLS.Append(aFSp);
            }
        }
        //
        BOPAlgo_MakerVolume aMV = new();
        aMV.SetArguments(aLS);
        aMV.SetIntersect(true);
        aMV.Perform(aPS.Next(9));
        if (aMV.HasErrors())
        {
            return;
        }

        //
        // get shapes connection for using in the rebuilding process
        // for the cases in which some of the intersection left undetected
        ShapesConnections(aDMFImF, aMV);
        //
        // find faces to remove
        TopoDS_Shape aSols = aMV.InternalShape();
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMFS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aSols, TopAbs_FACE, TopAbs_SOLID, aDMFS);
        //
        aNb = aDMFS.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        // To use the created solids for classifications, firstly, it is necessary
        // to check them on validity - the created solids should be complete,
        // i.e. all faces should be included.
        //
        NCollection_Map<TopoDS_Shape> aMFToRem = new(TopTools_ShapeMapHasher.Instance);
        // Check completeness
        if (aMV.HasDeleted())
        {
            NCollection_IndexedMap<TopoDS_Shape> aMEHoles = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(theFHoles, TopAbs_EDGE, aMEHoles);

            // Map edges of the solids to check the connectivity
            // of the removed invalid splits
            NCollection_IndexedMap<TopoDS_Shape> aMESols = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aSols, TopAbs_EDGE, aMESols);

            // perform additional check on faces
            aNb = myOFImages.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                if (!aPS.More())
                {
                    return;
                }
                NCollection_List<TopoDS_Shape> aLFIm = myOFImages[i];
                if (aLFIm.IsEmpty())
                {
                    continue;
                }

                TopoDS_Shape aF = myOFImages.FindKey(i);
                bool bInvalid = myInvalidFaces.Contains(aF);
                // For invalid faces it is allowed to be at least connected
                // to the solids, otherwise the solids are considered as broken
                bool bConnected = false;

                bool bFaceKept = false;
                aItLF.Initialize(aLFIm);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFIm = aItLF.Value();
                    if (!aMV.IsDeleted(aFIm))
                    {
                        bFaceKept = true;
                        continue;
                    }
                    //
                    TopExp_Explorer aExpE = new(aFIm, TopAbs_EDGE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        if (aMEHoles.Contains(aExpE.Current()))
                        {
                            bFaceKept = true;
                            aMFToRem.Add(aFIm);
                            break;
                        }
                        if (!bFaceKept && bInvalid && !bConnected)
                        {
                            bConnected = aMESols.Contains(aExpE.Current());
                        }
                    }
                }
                //
                if (!bFaceKept && !bConnected)
                {
                    return;
                }
            }
        }
        //
        NCollection_IndexedMap<TopoDS_Shape> aMEBoundary = new(TopTools_ShapeMapHasher.Instance);
        aNb = aDMFS.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aFIm = aDMFS.FindKey(i);
            NCollection_List<TopoDS_Shape> aLSol = aDMFS[i];
            if (aLSol.Extent() > 1)
            {
                aMFToRem.Add(aFIm);
            }
            else if (aFIm.Orientation() != TopAbs_INTERNAL)
            {
                TopExp.MapShapes(aFIm, TopAbs_EDGE, aMEBoundary);
            }
        }

        // Tool for getting the splits of faces
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVIms = aMV.Images();

        // update invalid faces with images
        aNb = aMFInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aFInv = aMFInv[i];
            TakeModified(aFInv, aMVIms, aMFInv);
        }

        // Take into account the faces invalid by inverted edges
        for (NCollection_List<TopoDS_Shape>.Iterator itLF = new(theInvertedFaces); itLF.More(); itLF.Next())
        {
            TakeModified(itLF.Value(), aMVIms, aMFInv);
        }

        // check if the invalid faces inside the holes are really invalid:
        // check its normal direction - if it has changed relatively the
        // original face the offset face is invalid and should be kept for rebuilding
        int aNbFH = theMFInvInHole.Extent();
        for (i = 1; i <= aNbFH; ++i)
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Shape aFInv = theMFInvInHole[i];
            NCollection_List<TopoDS_Shape> aLFInvIm = ModifiedOf(aMV, aFInv);
            if (aLFInvIm.IsEmpty())
            {
                aLFInvIm.Append(aFInv);
            }
            //
            if (!aDMFImF.Find(aFInv, out TopoDS_Shape pFOffset))
            {
                continue;
            }
            NCollection_List<TopoDS_Shape>.Iterator aItLFInv = new(aLFInvIm);
            for (; aItLFInv.More(); aItLFInv.Next())
            {
                TopoDS_Shape aFInvIm = aItLFInv.Value();
                if (!aDMFS.FindFromKey(aFInvIm, out NCollection_List<TopoDS_Shape> pLSols) || pLSols.Extent() != 1)
                {
                    continue;
                }
                //
                TopoDS_Shape aFSol = pLSols.First();
                //
                TopoDS_Shape aFx = new();
                if (!FindShape(aFInvIm, aFSol, null, ref aFx))
                {
                    continue;
                }
                //
                if (BRepOffset_Tool.CheckPlanesNormals(TopoDS.Face(aFx), TopoDS.Face(pFOffset)))
                {
                    // the normal direction has not changed, thus the face can be removed
                    aMFToRem.Add(aFInvIm);
                }
            }
        }
        //
        TopoDS_Compound aSolids = new();
        new BRep_Builder().MakeCompound(ref aSolids);
        NCollection_Map<TopoDS_Shape> aMFKeep = new(TopTools_ShapeMapHasher.Instance);
        //
        TopExp_Explorer aExpS = new(aSols, TopAbs_SOLID);
        for (; aExpS.More(); aExpS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Shape aSol = aExpS.Current();
            //
            bool bAllInv = true, bAllRemoved = true;

            for (TopExp_Explorer aExpF = new(aSol, TopAbs_FACE); aExpF.More(); aExpF.Next())
            {
                TopoDS_Shape aFS = aExpF.Current();
                //
                if (aFS.Orientation() == TopAbs_INTERNAL)
                {
                    aMFToRem.Add(aFS);
                    continue;
                }

                if (aMFToRem.Contains(aFS))
                {
                    continue;
                }

                bAllRemoved = false;
                bAllInv &= aMFInv.Contains(aFS);
            }
            //
            if (bAllInv && !bAllRemoved)
            {
                // remove invalid faces but keep those that have already been marked for removal
                TopExp_Explorer aExpF = new(aSol, TopAbs_FACE);
                for (; aExpF.More(); aExpF.Next())
                {
                    TopoDS_Shape aFS = aExpF.Current();
                    //
                    if (aMFToRem.Contains(aFS))
                    {
                        if (!aMFKeep.Add(aFS))
                        {
                            aMFKeep.Remove(aFS);
                        }
                    }
                    else
                    {
                        aMFToRem.Add(aFS);
                    }
                }
            }
            else
            {
                new BRep_Builder().Add(aSolids, aSol);
                mySolids = aSolids;
            }
        }
        //
        foreach (TopoDS_Shape aItM in aMFKeep)
        {
            aMFToRem.Remove(aItM);
        }

        // Remove the invalid hanging parts external to the solids
        RemoveHangingParts(aMV, aDMFImF, aMFInv, aMFToRem);

        // Remove newly found internal and hanging faces
        RemoveValidSplits(aMFToRem, aMV, theMERemoved);
        RemoveInvalidSplits(aMFToRem, aMV, theMERemoved);
        //
        // Get inside faces from the removed ones comparing them with boundary edges
        theMEInside.Clear();
        aNb = theMERemoved.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aE = theMERemoved[i];
            if (!aMEBoundary.Contains(aE))
            {
                theMEInside.Add(aE);
            }
        }

        // build all possible intersection pairs basing on the intersection results
        // taking into account removed faces.
        if (aMFToRem.Extent() != 0)
        {
            buildIntersectionPairs(myOFImages, myInvalidFaces, aMV, aMFToRem, aDMFImF, myIntersectionPairs);
        }
    }

    //=======================================================================
    // function : ShapesConnections
    // purpose  : Looking for the connections between faces not to miss
    //           some necessary intersection
    //=======================================================================
    private void ShapesConnections(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theDMFOr, BOPAlgo_Builder theBuilder)
    {
        // Make connexity blocks from invalid edges to use the whole block
        // to which the edge is connected instead of the single edge.
        TopoDS_Compound aCEInv = new();
        new BRep_Builder().MakeCompound(ref aCEInv);
        for (int i = 1; i <= myInvalidEdges.Extent(); ++i)
        {
            AddToContainer(myInvalidEdges[i], aCEInv);
        }

        NCollection_List<TopoDS_Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCEInv, TopAbs_VERTEX, TopAbs_EDGE, aLCB);

        // Binding from the edge to the block
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aECBMap = new(TopTools_ShapeMapHasher.Instance);
        for (NCollection_List<TopoDS_Shape>.Iterator itCB = new(aLCB); itCB.More(); itCB.Next())
        {
            for (TopoDS_Iterator itE = new(itCB.Value()); itE.More(); itE.Next())
            {
                aECBMap.Bind(itE.Value(), itCB.Value());
            }
        }

        // update invalid edges with images and keep connection to original edge
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEOr = new(TopTools_ShapeMapHasher.Instance);
        int aNb = myInvalidEdges.Extent();
        for (int i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aEInv = myInvalidEdges[i];
            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(theBuilder, aEInv);
            if (aLEIm.IsEmpty())
            {
                NCollection_List<TopoDS_Shape> aL = new();
                aL.Append(aEInv);
                aDMEOr.Bind(aEInv, aL);
                continue;
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(aLEIm);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                TopoDS_Shape aEIm = aItLEIm.Value();
                AddToContainer(aEIm, aEInv, aDMEOr);
            }
        }
        //
        // get shapes connections for using in the rebuilding process
        BOPDS_DS pDS = theBuilder.PDS()!;
        // analyze all Face/Face intersections
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = pDS.InterfFF();
        int iInt, aNbFF = aFFs.Length();
        for (iInt = 0; iInt < aNbFF; ++iInt)
        {
            BOPDS_InterfFF aFF = aFFs[iInt];
            NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.Curves();
            int aNbC = aVNC.Length();
            if (aNbC == 0)
            {
                continue;
            }
            //
            TopoDS_Shape aFIm1 = pDS.Shape(aFF.Index1());
            TopoDS_Shape aFIm2 = pDS.Shape(aFF.Index2());
            //
            if (!theDMFOr.Find(aFIm1, out TopoDS_Shape pF1) || !theDMFOr.Find(aFIm2, out TopoDS_Shape pF2))
            {
                continue;
            }
            //
            if (pF1.IsSame(pF2))
            {
                continue;
            }
            //
            bool bInv1 = myInvalidFaces.Contains(pF1);
            bool bInv2 = myInvalidFaces.Contains(pF2);
            //
            if (!bInv1 && !bInv2)
            {
                continue;
            }
            //
            // check if it is real Face/Face intersection
            NCollection_Map<TopoDS_Shape> aMEInt = new(TopTools_ShapeMapHasher.Instance);
            for (int iC = 0; iC < aNbC; ++iC)
            {
                BOPDS_Curve aNC = aVNC[iC];
                NCollection_List<BOPDS_PaveBlock> aLPB = aNC.PaveBlocks();
                NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
                for (; aItLPB.More(); aItLPB.Next())
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    if (aPB.HasEdge(out int nEInt))
                    {
                        TopoDS_Shape aEInt = pDS.Shape(nEInt);
                        aMEInt.Add(aEInt);
                    }
                }
            }
            //
            if (aMEInt.IsEmpty())
            {
                continue;
            }
            //
            // check if invalid edges of the face are in the same splits with intersection edges
            for (int i = 0; i < 2; ++i)
            {
                if ((i == 0 && !bInv1) || (i != 0 && !bInv2))
                {
                    continue;
                }
                //
                TopoDS_Shape aF = i == 0 ? pF1 : pF2;
                TopoDS_Shape aFOp = i == 0 ? pF2 : pF1;
                TopoDS_Shape aFIm = i == 0 ? aFIm1 : aFIm2;
                //
                bool bFound = false;
                //
                NCollection_List<TopoDS_Shape> aLFIm = ModifiedOf(theBuilder, aFIm);
                if (aLFIm.IsEmpty())
                {
                    aLFIm.Append(aFIm);
                }
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    TopoDS_Shape aFImIm = aItLFIm.Value();
                    //
                    bool bInv = false, bInt = false;
                    TopExp_Explorer aExpE = new(aFImIm, TopAbs_EDGE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        TopoDS_Shape aE = aExpE.Current();
                        if (!bInv)
                        {
                            bInv = aDMEOr.IsBound(aE);
                        }
                        if (!bInt)
                        {
                            bInt = aMEInt.Contains(aE);
                        }
                        if (bInv && bInt)
                        {
                            break;
                        }
                    }
                    //
                    if (!bInt || !bInv)
                    {
                        continue;
                    }
                    //
                    bFound = true;
                    //
                    // append opposite face to all invalid edges in the split
                    aExpE.Init(aFImIm, TopAbs_EDGE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        TopoDS_Shape aE = aExpE.Current();
                        if (!aDMEOr.Find(aE, out NCollection_List<TopoDS_Shape> pLEOr))
                        {
                            continue;
                        }
                        //
                        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(pLEOr);
                        for (; aItLE.More(); aItLE.Next())
                        {
                            TopoDS_Shape aEOr = aItLE.Value();
                            if (!mySSInterfs.Find(aEOr, out NCollection_List<TopoDS_Shape> pLFE))
                            {
                                pLFE = new NCollection_List<TopoDS_Shape>();
                                mySSInterfs.Bind(aEOr, pLFE);
                            }
                            AppendToList(pLFE, aFOp);
                        }
                    }
                }
                if (bFound)
                {
                    // save connection between offset faces
                    if (!mySSInterfs.Find(aF, out NCollection_List<TopoDS_Shape> pLF))
                    {
                        pLF = new NCollection_List<TopoDS_Shape>();
                        mySSInterfs.Bind(aF, pLF);
                    }
                    AppendToList(pLF, aFOp);
                }
            }

            // Treatment for the artificial case - check if one of the faces is artificially invalid
            for (int iF = 0; iF < 2; ++iF)
            {
                TopoDS_Shape aFArt = iF == 0 ? pF1 : pF2;
                TopoDS_Shape aFOpposite = iF == 0 ? pF2 : pF1;

                if (!myArtInvalidFaces.IsBound(aFArt))
                {
                    continue;
                }

                if (myInvalidFaces.Contains(aFOpposite) && !myArtInvalidFaces.IsBound(aFOpposite))
                {
                    continue;
                }

                // Collect own invalid edges of the face and the invalid edges connected to those
                // own invalid edges to be avoided in the check for intersection.
                NCollection_IndexedMap<TopoDS_Shape> aMEAvoid = new(TopTools_ShapeMapHasher.Instance);
                if (myArtInvalidFaces.Find(aFOpposite, out NCollection_IndexedMap<TopoDS_Shape> pFEInv))
                {
                    for (int iE = 1; iE <= pFEInv.Extent(); ++iE)
                    {
                        if (aECBMap.Find(pFEInv.FindKey(iE), out TopoDS_Shape pCB))
                        {
                            TopExp.MapShapes(pCB, TopAbs_EDGE, aMEAvoid);
                        }
                    }
                }
                else if (myOFImages.FindFromKey(aFOpposite, out NCollection_List<TopoDS_Shape> pLFIm))
                {
                    for (NCollection_List<TopoDS_Shape>.Iterator itLFIm = new(pLFIm); itLFIm.More(); itLFIm.Next())
                    {
                        for (TopExp_Explorer expE = new(itLFIm.Value(), TopAbs_EDGE); expE.More(); expE.Next())
                        {
                            if (aECBMap.Find(expE.Current(), out TopoDS_Shape pCB))
                            {
                                TopExp.MapShapes(pCB, TopAbs_EDGE, aMEAvoid);
                            }
                        }
                    }
                }

                TopoDS_Shape aFArtIm = iF == 0 ? aFIm1 : aFIm2;
                TopoDS_Shape aFOppositeIm = iF == 0 ? aFIm2 : aFIm1;

                // Check if there are any intersections between edges of artificially
                // invalid face and opposite face
                int nFOp = pDS.Index(aFOppositeIm);

                for (TopExp_Explorer expE = new(aFArtIm, TopAbs_EDGE); expE.More(); expE.Next())
                {
                    TopoDS_Shape aE = expE.Current();
                    if (!myInvalidEdges.Contains(aE) || myInvertedEdges.Contains(aE) || aMEAvoid.Contains(aE))
                    {
                        continue;
                    }
                    int nE = pDS.Index(aE);
                    if (nE < 0)
                    {
                        continue;
                    }

                    if (!pDS.HasInterf(nE, nFOp))
                    {
                        continue;
                    }

                    NCollection_List<TopoDS_Shape> aLV = new();
                    NCollection_DynamicArray<BOPDS_InterfEF> aEFs = pDS.InterfEF();
                    for (int iEF = 0; iEF < aEFs.Length(); ++iEF)
                    {
                        BOPDS_InterfEF aEF = aEFs[iEF];
                        if (aEF.Contains(nE) && aEF.Contains(nFOp))
                        {
                            if (aEF.CommonPart().Type() == TopAbs_VERTEX)
                            {
                                aLV.Append(pDS.Shape(aEF.IndexNew()));
                            }
                        }
                    }

                    if (aLV.IsEmpty())
                    {
                        continue;
                    }

                    // Make sure that there is an opposite intersection exists, i.e. some of the edges
                    // of the opposite face intersect the artificially invalid face.
                    int nFArt = pDS.Index(aFArtIm);
                    TopExp_Explorer expEOp = new(aFOppositeIm, TopAbs_EDGE);
                    for (; expEOp.More(); expEOp.Next())
                    {
                        TopoDS_Shape aEOp = expEOp.Current();
                        int nEOp = pDS.Index(aEOp);
                        if (pDS.HasInterf(nEOp, nFArt))
                        {
                            break;
                        }
                    }
                    if (!expEOp.More())
                    {
                        continue;
                    }

                    // Intersection is present - add connection between offset faces.
                    AddToContainer(aFArt, aFOpposite, mySSInterfsArt);

                    // Add connection between edge and opposite face
                    AddToContainer(aE, aFOpposite, mySSInterfsArt);

                    // Along with the opposite face, save the intersection vertices to
                    // be used for trimming the intersection edge in the rebuilding process
                    for (NCollection_List<TopoDS_Shape>.Iterator itLV = new(aLV); itLV.More(); itLV.Next())
                    {
                        // Add connection to intersection vertex
                        AddToContainer(aE, itLV.Value(), mySSInterfsArt);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : RemoveHangingParts
    // purpose  : Remove isolated invalid hanging parts
    //=======================================================================
    private void RemoveHangingParts(BOPAlgo_MakerVolume theMV, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theDMFImF,
                                    NCollection_IndexedMap<TopoDS_Shape> theMFInv, NCollection_Map<TopoDS_Shape> theMFToRem)
    {
        // Map the faces of the result solids to filter them from avoided faces
        NCollection_IndexedMap<TopoDS_Shape> aMFS = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theMV.InternalShape(), TopAbs_FACE, aMFS);

        BRep_Builder aBB = new();
        // Build compound of all faces not included into solids
        TopoDS_Compound aCFHangs = new();
        aBB.MakeCompound(ref aCFHangs);

        // Tool for getting the splits of faces
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVIms = theMV.Images();

        NCollection_List<TopoDS_Shape>.Iterator aItLArgs = new(theMV.Arguments());
        for (; aItLArgs.More(); aItLArgs.Next())
        {
            TopExp_Explorer anExpF0 = new(aItLArgs.Value(), TopAbs_FACE);
            for (; anExpF0.More(); anExpF0.Next())
            {
                TopoDS_Shape aF = anExpF0.Current();
                TakeModified(aF, aMVIms, aCFHangs, aMFS);
            }
        }

        // Make connexity blocks of all hanging parts and check that they are isolated
        NCollection_List<TopoDS_Shape> aLCBHangs = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFHangs, TopAbs_EDGE, TopAbs_FACE, aLCBHangs);
        if (aLCBHangs.IsEmpty())
        {
            return;
        }

        // To be removed, the block should contain invalid splits of offset faces and should
        // meet one of the following conditions:
        // 1. The block should not be connected to any invalid parts (Faces or Edges)
        //    contained in solids;
        // 2. The block should be isolated from other faces, i.e. it should consist of
        //    the splits of the single offset face.

        // Map the edges and vertices of the result solids to check connectivity
        // of the hanging blocks to invalid parts contained in solids
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEF = new(TopTools_ShapeMapHasher.Instance),
                                                                                 aDMVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theMV.InternalShape(), TopAbs_EDGE, TopAbs_FACE, aDMEF);
        TopExp.MapShapesAndAncestors(theMV.InternalShape(), TopAbs_VERTEX, TopAbs_EDGE, aDMVE);

        // Update invalid edges with intersection results
        NCollection_Map<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
        int i, aNbE = myInvalidEdges.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            TakeModified(myInvalidEdges[i], aMVIms, aMEInv);
        }

        // Update inverted edges with intersection results
        NCollection_Map<TopoDS_Shape> aMEInverted = new(TopTools_ShapeMapHasher.Instance);
        for (int iInv = 1; iInv <= myInvertedEdges.Extent(); ++iInv)
        {
            TakeModified(myInvertedEdges[iInv], aMVIms, aMEInverted);
        }

        // Tool for getting the origins of the splits
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVOrs = theMV.Origins();

        // Find hanging blocks to remove
        NCollection_List<TopoDS_Shape> aBlocksToRemove = new();

        NCollection_List<TopoDS_Shape>.Iterator aItLCBH = new(aLCBHangs);
        for (; aItLCBH.More(); aItLCBH.Next())
        {
            TopoDS_Shape aCBH = aItLCBH.Value();

            // Remove the block containing the inverted edges
            bool bHasInverted = false;
            TopExp_Explorer anExpE = new(aCBH, TopAbs_EDGE);
            for (; anExpE.More() && !bHasInverted; anExpE.Next())
            {
                TopoDS_Shape aE = anExpE.Current();
                bHasInverted = !aDMEF.Contains(aE) && aMEInverted.Contains(aE);
            }

            if (bHasInverted)
            {
                aBlocksToRemove.Append(aCBH);
                continue;
            }

            // Check the block to contain invalid split
            bool bHasInvalidFace = false;
            // Check connectivity to invalid parts
            bool bIsConnected = false;
            NCollection_IndexedMap<TopoDS_Shape> aBlockME = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aCBH, TopAbs_EDGE, aBlockME);
            // Map to collect all original faces
            NCollection_Map<TopoDS_Shape> aMOffsetF = new(TopTools_ShapeMapHasher.Instance);

            TopExp_Explorer anExpF = new(aCBH, TopAbs_FACE);
            for (; anExpF.More(); anExpF.Next())
            {
                TopoDS_Shape aF = anExpF.Current();
                // Check block to contain invalid face
                if (!bHasInvalidFace)
                {
                    bHasInvalidFace = theMFInv.Contains(aF);
                }

                // Check block for connectivity to invalid parts
                if (!bIsConnected)
                {
                    // check edges
                    anExpE.Init(aF, TopAbs_EDGE);
                    for (; anExpE.More() && !bIsConnected; anExpE.Next())
                    {
                        TopoDS_Shape aE = anExpE.Current();
                        if (aDMEF.FindFromKey(aE, out NCollection_List<TopoDS_Shape> pLF))
                        {
                            NCollection_List<TopoDS_Shape>.Iterator aItLF = new(pLF);
                            for (; aItLF.More() && !bIsConnected; aItLF.Next())
                            {
                                bIsConnected = theMFInv.Contains(aItLF.Value());
                            }
                        }
                    }
                    // check vertices
                    if (!bIsConnected)
                    {
                        TopExp_Explorer anExpV = new(aF, TopAbs_VERTEX);
                        for (; anExpV.More() && !bIsConnected; anExpV.Next())
                        {
                            TopoDS_Shape aV = anExpV.Current();
                            if (aDMVE.FindFromKey(aV, out NCollection_List<TopoDS_Shape> pLE))
                            {
                                NCollection_List<TopoDS_Shape>.Iterator aItLE = new(pLE);
                                for (; aItLE.More() && !bIsConnected; aItLE.Next())
                                {
                                    bIsConnected = !aBlockME.Contains(aItLE.Value()) && aMEInv.Contains(aItLE.Value());
                                }
                            }
                        }
                    }
                }

                // Check block to be isolated
                if (aMVOrs.Find(aF, out NCollection_List<TopoDS_Shape> pLFOr))
                {
                    NCollection_List<TopoDS_Shape>.Iterator aItLFOr = new(pLFOr);
                    for (; aItLFOr.More(); aItLFOr.Next())
                    {
                        if (theDMFImF.Find(aItLFOr.Value(), out TopoDS_Shape pFOffset))
                        {
                            aMOffsetF.Add(pFOffset);
                        }
                    }
                }
                else
                {
                    if (theDMFImF.Find(aF, out TopoDS_Shape pFOffset))
                    {
                        aMOffsetF.Add(pFOffset);
                    }
                }
            }

            bool bRemove = bHasInvalidFace && (!bIsConnected || aMOffsetF.Extent() == 1);

            if (bRemove)
            {
                aBlocksToRemove.Append(aCBH);
            }
        }

        // remove the invalidated blocks
        aItLCBH.Initialize(aBlocksToRemove);
        for (; aItLCBH.More(); aItLCBH.Next())
        {
            TopoDS_Shape aCBH = aItLCBH.Value();
            TopExp_Explorer anExpF = new(aCBH, TopAbs_FACE);
            for (; anExpF.More(); anExpF.Next())
            {
                theMFToRem.Add(anExpF.Current());
            }
        }
    }

    //=======================================================================
    // function : RemoveValidSplits
    // purpose  : Removing valid splits according to results of intersection
    //=======================================================================
    private void RemoveValidSplits(NCollection_Map<TopoDS_Shape> theSpRem, BOPAlgo_Builder theGF,
                                   NCollection_IndexedMap<TopoDS_Shape> theMERemoved)
    {
        int i, aNb = myOFImages.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<TopoDS_Shape> aLSIm = myOFImages[i];
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLSIm);
            for (; aIt.More();)
            {
                TopoDS_Shape aSIm = aIt.Value();
                if (theSpRem.Contains(aSIm))
                {
                    TopExp.MapShapes(aSIm, TopAbs_EDGE, theMERemoved);
                    aLSIm.Remove(aIt);
                    continue;
                }
                //
                // check if all its images are have to be removed
                NCollection_List<TopoDS_Shape> aLSImIm = ModifiedOf(theGF, aSIm);
                if (aLSImIm.Extent() != 0)
                {
                    bool bAllRem = true;
                    NCollection_List<TopoDS_Shape>.Iterator aIt1 = new(aLSImIm);
                    for (; aIt1.More(); aIt1.Next())
                    {
                        TopoDS_Shape aSImIm = aIt1.Value();
                        if (theSpRem.Contains(aSImIm))
                        {
                            TopExp.MapShapes(aSImIm, TopAbs_EDGE, theMERemoved);
                        }
                        else
                        {
                            bAllRem = false;
                        }
                    }
                    //
                    if (bAllRem)
                    {
                        TopExp.MapShapes(aSIm, TopAbs_EDGE, theMERemoved);
                        aLSIm.Remove(aIt);
                        continue;
                    }
                }
                aIt.Next();
            }
        }
    }

    //=======================================================================
    // function : RemoveInvalidSplits
    // purpose  : Removing invalid splits according to the results of intersection
    //=======================================================================
    private void RemoveInvalidSplits(NCollection_Map<TopoDS_Shape> theSpRem, BOPAlgo_Builder theGF,
                                     NCollection_IndexedMap<TopoDS_Shape> theMERemoved)
    {
        int i, aNb = myInvalidFaces.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aS = myInvalidFaces.FindKey(i);
            bool bArt = myArtInvalidFaces.IsBound(aS);
            //
            NCollection_List<TopoDS_Shape> aLSIm = myInvalidFaces[i];
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLSIm);
            for (; aIt.More();)
            {
                TopoDS_Shape aSIm = aIt.Value();
                if (theSpRem.Contains(aSIm))
                {
                    TopExp.MapShapes(aSIm, TopAbs_EDGE, theMERemoved);
                    aLSIm.Remove(aIt);
                    continue;
                }
                //
                // check if all its images are have to be removed
                NCollection_List<TopoDS_Shape> aLSImIm = ModifiedOf(theGF, aSIm);
                if (aLSImIm.IsEmpty())
                {
                    aIt.Next();
                    continue;
                }
                //
                bool bAllRem = true;
                NCollection_IndexedMap<TopoDS_Shape> aMERemoved = new(TopTools_ShapeMapHasher.Instance);
                NCollection_List<TopoDS_Shape>.Iterator aIt1 = new(aLSImIm);
                for (; aIt1.More(); aIt1.Next())
                {
                    TopoDS_Shape aSImIm = aIt1.Value();
                    if (theSpRem.Contains(aSImIm))
                    {
                        TopExp.MapShapes(aSImIm, TopAbs_EDGE, aMERemoved);
                    }
                    else
                    {
                        bAllRem = false;
                    }
                }
                //
                if (bAllRem)
                {
                    aLSIm.Remove(aIt);
                    continue;
                }
                //
                if (bArt)
                {
                    aIt.Next();
                    continue;
                }
                //
                // remove the face from invalid if all invalid edges of this face
                // have been marked for removal
                TopExp_Explorer aExpE = new(aSIm, TopAbs_EDGE);
                for (; aExpE.More(); aExpE.Next())
                {
                    TopoDS_Shape aEInv = aExpE.Current();
                    if (myInvalidEdges.Contains(aEInv) && !aMERemoved.Contains(aEInv))
                    {
                        break;
                    }
                }
                if (!aExpE.More())
                {
                    TopExp.MapShapes(aSIm, TopAbs_EDGE, theMERemoved);
                    aLSIm.Remove(aIt);
                }
                else
                {
                    aIt.Next();
                }
            }
        }
    }

    //=======================================================================
    // function : FilterEdgesImages
    // purpose  : Updating the maps of images and origins of the offset edges
    //=======================================================================
    private void FilterEdgesImages(in TopoDS_Shape theS)
    {
        // map edges
        NCollection_IndexedMap<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, TopAbs_EDGE, aME);
        //
        myOEOrigins.Clear();
        // OCCT.NET: OCCT iterates the map by its edges' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aItDM in myOEImages)
        {
            TopoDS_Shape aE = aItDM.Key;
            NCollection_List<TopoDS_Shape> aLEIm = aItDM.Value;
            //
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLEIm);
            for (; aIt.More();)
            {
                TopoDS_Shape aEIm = aIt.Value();
                // filter images
                if (!aME.Contains(aEIm))
                {
                    // remove the image
                    // edges with no images left should be kept in the map
                    // to avoid their usage when building the splits of faces
                    aLEIm.Remove(aIt);
                    continue;
                }
                //
                // save origins
                if (myOEOrigins.IsBound(aEIm))
                {
                    AppendToList(myOEOrigins.ChangeFind(aEIm), aE);
                }
                else
                {
                    NCollection_List<TopoDS_Shape> aLOr = new();
                    aLOr.Append(aE);
                    myOEOrigins.Bind(aEIm, aLOr);
                }
                //
                aIt.Next();
            }
        }
    }

    // OCCT's copy of an IndexedDataMap of lists (a value copy: the lists are copied too).
    private static NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> CopyOf(
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aCopy = new(TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= theMap.Extent(); ++i)
        {
            aCopy.Add(theMap.FindKey(i), new NCollection_List<TopoDS_Shape>(theMap[i]));
        }
        return aCopy;
    }

    //=======================================================================
    // function : FilterInvalidFaces
    // purpose  : Filtering of the invalid faces
    //=======================================================================
    private void FilterInvalidFaces(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEF,
                                    NCollection_IndexedMap<TopoDS_Shape> theMERemoved)
    {
        //
        // filter invalid faces, considering faces having only valid
        // images left with non-free edges as valid
        // do not remove invalid faces if it creates free edges
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aReallyInvFaces = new(TopTools_ShapeMapHasher.Instance);
        // Edge-Face connexity map of all splits, both invalid and valid
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEFAll = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        //
        int aNb = myInvalidFaces.Extent();
        for (int i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            NCollection_List<TopoDS_Shape> aLFInv = myInvalidFaces[i];
            //
            if (myArtInvalidFaces.IsBound(aF))
            {
                if (aLFInv.IsEmpty())
                {
                    myArtInvalidFaces.UnBind(aF);
                }
                else
                {
                    aReallyInvFaces.Add(aF, new NCollection_List<TopoDS_Shape>(aLFInv));
                }
                continue;
            }
            //
            if (aLFInv.IsEmpty())
            {
                continue;
            }
            //
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages.ChangeFromKey(aF);
            bool bInvalid = aLFIm.IsEmpty();
            //
            if (!bInvalid)
            {
                // check two lists on common splits
                aItLF.Initialize(aLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFInv = aItLF.Value();
                    //
                    NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
                    for (; aItLFIm.More(); aItLFIm.Next())
                    {
                        TopoDS_Shape aFIm = aItLFIm.Value();
                        //
                        if (aFInv.IsSame(aFIm))
                        {
                            break;
                        }
                    }
                    //
                    if (aItLFIm.More())
                    {
                        break;
                    }
                }
                //
                bInvalid = aItLF.More();
            }
            //
            if (!bInvalid)
            {
                // check for free edges
                for (int j = 0; !bInvalid && j < 2; ++j)
                {
                    NCollection_List<TopoDS_Shape> aLI = j == 0 ? aLFIm : aLFInv;
                    aItLF.Initialize(aLI);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        TopoDS_Shape aFIm = aItLF.Value();
                        //
                        TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
                        for (; aExp.More(); aExp.Next())
                        {
                            TopoDS_Shape aE = aExp.Current();
                            if (!theMERemoved.Contains(aE))
                            {
                                if (theDMEF.FindFromKey(aE, out NCollection_List<TopoDS_Shape> pLEF) && pLEF.Extent() == 1)
                                {
                                    break;
                                }
                            }
                        }
                        //
                        if (aExp.More())
                        {
                            break;
                        }
                    }
                    bInvalid = aItLF.More();
                }
            }
            if (bInvalid)
            {
                if (aDMEFAll.IsEmpty())
                {
                    aDMEFAll = CopyOf(theDMEF);
                    for (int iF = 1; iF <= aNb; ++iF)
                    {
                        for (NCollection_List<TopoDS_Shape>.Iterator itLFInv = new(myInvalidFaces[iF]); itLFInv.More(); itLFInv.Next())
                        {
                            TopExp.MapShapesAndAncestors(itLFInv.Value(), TopAbs_EDGE, TopAbs_FACE, aDMEFAll);
                        }
                    }
                }

                NCollection_Map<TopoDS_Shape> aLocalSplits = new(TopTools_ShapeMapHasher.Instance);
                for (int j = 0; j < 2; ++j)
                {
                    for (aItLF.Initialize((j == 0 ? aLFIm : aLFInv)); aItLF.More(); aItLF.Next())
                    {
                        aLocalSplits.Add(aItLF.Value());
                    }
                }

                // Check if all invalid edges are located inside the split and do not touch
                // any other faces both invalid and valid
                aItLF.Initialize(aLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFIm = aItLF.Value();
                    TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
                    for (; aExp.More(); aExp.Next())
                    {
                        TopoDS_Shape aE = aExp.Current();
                        if (myInvalidEdges.Contains(aE) && !theMERemoved.Contains(aE))
                        {
                            NCollection_List<TopoDS_Shape> aLF = aDMEFAll.FindFromKey(aE);
                            NCollection_List<TopoDS_Shape>.Iterator itLF = new(aLF);
                            for (; itLF.More(); itLF.Next())
                            {
                                if (!aLocalSplits.Contains(itLF.Value()))
                                {
                                    break;
                                }
                            }
                            if (itLF.More())
                            {
                                break;
                            }
                        }
                    }
                    if (aExp.More())
                    {
                        break;
                    }
                }
                bInvalid = aItLF.More();
                if (!bInvalid)
                {
                    aItLF.Initialize(aLFInv);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        AppendToList(aLFIm, aItLF.Value());
                    }
                }
            }
            //
            if (bInvalid)
            {
                aReallyInvFaces.Add(aF, new NCollection_List<TopoDS_Shape>(aLFInv));
            }
        }
        //
        myInvalidFaces = aReallyInvFaces;
    }

    //=======================================================================
    // function : CheckEdgesCreatedByVertex
    // purpose  : Checks additionally the unchecked edges originated from vertices
    //=======================================================================
    private void CheckEdgesCreatedByVertex()
    {
        // Mark the unchecked edges contained in invalid faces as invalid
        int aNbF = myInvalidFaces.Extent();
        for (int i = 1; i <= aNbF; ++i)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            if (myArtInvalidFaces.IsBound(aF))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> aLFIm = myInvalidFaces[i];
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLFIm); it.More(); it.Next())
            {
                TopoDS_Shape aFIm = it.Value();
                for (TopExp_Explorer expE = new(aFIm, TopAbs_EDGE); expE.More(); expE.Next())
                {
                    TopoDS_Shape aE = expE.Current();
                    if (myInvalidEdges.Contains(aE) || myValidEdges.Contains(aE))
                    {
                        continue;
                    }

                    // check if this edges is not created from vertex and mark it as invalid
                    if (!myEdgesOrigins!.Find(aE, out NCollection_List<TopoDS_Shape> pLEOr))
                    {
                        continue;
                    }
                    NCollection_List<TopoDS_Shape>.Iterator itLEO = new(pLEOr);
                    for (; itLEO.More(); itLEO.Next())
                    {
                        if (itLEO.Value().ShapeType() != TopAbs_VERTEX)
                        {
                            break;
                        }
                    }
                    if (!itLEO.More())
                    {
                        myInvalidEdges.Add(aE);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FilterInvalidEdges
    // purpose  : Filtering the invalid edges according to currently invalid faces
    //=======================================================================
    private void FilterInvalidEdges(NCollection_DataMap<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> theDMFMIE,
                                    NCollection_IndexedMap<TopoDS_Shape> theMERemoved,
                                    NCollection_IndexedMap<TopoDS_Shape> theMEInside,
                                    NCollection_Map<TopoDS_Shape> theMEUseInRebuild)
    {
        TopoDS_Compound aCEInv = new();
        NCollection_IndexedMap<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aCEInv);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        //
        int i, aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<TopoDS_Shape> aLFInv = myInvalidFaces[i];
            aItLF.Initialize(aLFInv);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aFIm = aItLF.Value();
                TopExp.MapShapes(aFIm, TopAbs_EDGE, aMEInv);
                //
                TopExp_Explorer aExpE = new(aFIm, TopAbs_EDGE);
                for (; aExpE.More(); aExpE.Next())
                {
                    TopoDS_Shape aE = aExpE.Current();
                    if (myInvalidEdges.Contains(aE))
                    {
                        aBB.Add(aCEInv, aE);
                    }
                }
            }
        }
        //
        // remove edges which have been marked for removal
        NCollection_IndexedMap<TopoDS_Shape> aMEInvToAvoid = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aLCBE = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCEInv, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
        //
        NCollection_List<TopoDS_Shape>.Iterator aItLCBE = new(aLCBE);
        for (; aItLCBE.More(); aItLCBE.Next())
        {
            TopoDS_Shape aCBE = aItLCBE.Value();
            TopExp_Explorer aExpCB = new(aCBE, TopAbs_EDGE);
            for (; aExpCB.More(); aExpCB.Next())
            {
                TopoDS_Shape aE = aExpCB.Current();
                if (!theMERemoved.Contains(aE))
                {
                    break;
                }
            }
            //
            if (!aExpCB.More())
            {
                TopExp.MapShapes(aCBE, TopAbs_EDGE, aMEInvToAvoid);
            }
        }
        //
        NCollection_IndexedMap<TopoDS_Shape> aReallyInvEdges = new(TopTools_ShapeMapHasher.Instance);
        //
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            if (myArtInvalidFaces.IsBound(aF))
            {
                if (theDMFMIE.Find(aF, out NCollection_IndexedMap<TopoDS_Shape> aMIE))
                {
                    int aNbIE = aMIE.Extent();
                    for (int iE = 1; iE <= aNbIE; ++iE)
                    {
                        TopoDS_Shape aE = aMIE.FindKey(iE);
                        if (aMEInv.Contains(aE) && !aMEInvToAvoid.Contains(aE))
                        {
                            aReallyInvEdges.Add(aE);
                        }
                    }
                }
            }
            else
            {
                NCollection_List<TopoDS_Shape> aLFInv = myInvalidFaces[i];
                aItLF.Initialize(aLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFIm = aItLF.Value();
                    TopExp_Explorer aExpE = new(aFIm, TopAbs_EDGE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        TopoDS_Shape aE = aExpE.Current();
                        if (myInvalidEdges.Contains(aE) && !aMEInvToAvoid.Contains(aE))
                        {
                            aReallyInvEdges.Add(aE);
                        }
                    }
                }
            }
        }

        myInvalidEdges = aReallyInvEdges;

        // Check if any of the currently invalid edges may be used for
        // rebuilding splits of invalid faces.
        // For that the edge should be inside and not connected to invalid
        // boundary edges of the same origin.

        aNb = myInvalidEdges.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aE = myInvalidEdges[i];
            if (!theMEInside.Contains(aE) || !myValidEdges.Contains(aE))
            {
                continue;
            }

            if (!myOEOrigins.Find(aE, out NCollection_List<TopoDS_Shape> pEOrigins))
            {
                continue;
            }

            bool bHasInvOutside = false;
            for (NCollection_List<TopoDS_Shape>.Iterator itEOr = new(pEOrigins); !bHasInvOutside && itEOr.More(); itEOr.Next())
            {
                if (myOEImages.Find(itEOr.Value(), out NCollection_List<TopoDS_Shape> pEIms))
                {
                    for (NCollection_List<TopoDS_Shape>.Iterator itEIms = new(pEIms); !bHasInvOutside && itEIms.More(); itEIms.Next())
                    {
                        bHasInvOutside = myInvalidEdges.Contains(itEIms.Value()) && !theMEInside.Contains(itEIms.Value());
                    }
                }
            }
            if (!bHasInvOutside)
            {
                theMEUseInRebuild.Add(aE);
            }
        }
    }

    //=======================================================================
    // function : FindFacesToRebuild
    // purpose  : Looking for the faces that have to be rebuilt:
    //           1. Faces close to invalidity
    //           2. Faces containing some invalid parts
    //=======================================================================
    private void FindFacesToRebuild()
    {
        int i, aNb = myOFImages.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        bool bRebuild;
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        NCollection_List<TopoDS_Shape> aLEValid = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance), aMEReb = new(TopTools_ShapeMapHasher.Instance),
                                      aMFReb = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer aExp = new();
        //
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMFLV = new(TopTools_ShapeMapHasher.Instance);
        // get edges from invalid faces
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; i++)
        {
            TopoDS_Shape aF = myInvalidFaces.FindKey(i);
            aMFence.Clear();
            NCollection_List<TopoDS_Shape> aLVAvoid = new();
            NCollection_List<TopoDS_Shape> aLFIm = myInvalidFaces[i];
            aItLF.Initialize(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aFIm = aItLF.Value();
                aExp.Init(aFIm, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    aMEReb.Add(aE);
                    if (myInvalidEdges.Contains(aE))
                    {
                        TopExp_Explorer aExpV = new(aE, TopAbs_VERTEX);
                        for (; aExpV.More(); aExpV.Next())
                        {
                            TopoDS_Shape aV = aExpV.Current();
                            if (aMFence.Add(aV))
                            {
                                aLVAvoid.Append(aV);
                                aMEReb.Add(aV);
                            }
                        }
                    }
                }
            }
            //
            if (aLVAvoid.Extent() != 0)
            {
                aDMFLV.Bind(aF, aLVAvoid);
            }
            //
            NCollection_List<TopoDS_Shape>? pLF = null;
            if (!myArtInvalidFaces.IsBound(aF))
            {
                mySSInterfs.Find(aF, out pLF);
            }
            else
            {
                mySSInterfsArt.Find(aF, out pLF);
            }
            if (pLF is not null)
            {
                NCollection_List<TopoDS_Shape>.Iterator aItLFE = new(pLF);
                for (; aItLFE.More(); aItLFE.Next())
                {
                    TopoDS_Shape aFE = aItLFE.Value();
                    aMFReb.Add(aFE);
                }
            }
        }
        //
        // get face to rebuild
        aNb = myOFImages.Extent();
        for (i = 1; i <= aNb; i++)
        {
            TopoDS_Shape aF = myOFImages.FindKey(i);
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages[i];
            NCollection_Map<TopoDS_Shape> aMVAvoid = new(TopTools_ShapeMapHasher.Instance);
            if (aDMFLV.IsBound(aF))
            {
                NCollection_List<TopoDS_Shape> aLVAvoid = aDMFLV.Find(aF);
                NCollection_List<TopoDS_Shape>.Iterator aItLV = new(aLVAvoid);
                for (; aItLV.More(); aItLV.Next())
                {
                    TopoDS_Shape aV = aItLV.Value();
                    aMVAvoid.Add(aV);
                }
            }
            //
            bRebuild = aMFReb.Contains(aF);
            aLEValid.Clear();
            aMFence.Clear();
            //
            aItLF.Initialize(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aFIm = aItLF.Value();
                aExp.Init(aFIm, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Edge anEIm = TopoDS.Edge(aExp.Current());
                    if (!myInvalidEdges.Contains(anEIm))
                    {
                        if (aMFence.Add(anEIm))
                        {
                            aLEValid.Append(anEIm);
                        }
                    }
                    //
                    if (!bRebuild)
                    {
                        bRebuild = aMEReb.Contains(anEIm);
                    }
                    //
                    if (!bRebuild)
                    {
                        // check vertices
                        TopExp_Explorer aExpV = new(anEIm, TopAbs_VERTEX);
                        for (; aExpV.More() && !bRebuild; aExpV.Next())
                        {
                            TopoDS_Shape aV = aExpV.Current();
                            if (!aMVAvoid.Contains(aV))
                            {
                                bRebuild = aMEReb.Contains(aV);
                            }
                        }
                    }
                }
            }
            //
            if (!bRebuild)
            {
                bRebuild = aLFIm.Extent() != 0 && myInvalidFaces.Contains(aF);
                if (bRebuild)
                {
                    myFSelfRebAvoid.Add(aF);
                }
            }
            //
            if (bRebuild)
            {
                myFacesToRebuild.Add(aF, new NCollection_List<TopoDS_Shape>(aLEValid));
            }
        }
    }

    //=======================================================================
    // function : mapShapes
    // purpose  : Collect theVecShapes into theMap with theType set
    //=======================================================================
    private static void mapShapes(NCollection_List<TopoDS_Shape> theVecShapes, TopAbs_ShapeEnum theType,
                                  NCollection_Map<TopoDS_Shape> theMap)
    {
        foreach (TopoDS_Shape aShape in theVecShapes)
        {
            for (TopExp_Explorer anExp = new(aShape, theType); anExp.More(); anExp.Next())
            {
                theMap.Add(anExp.Current());
            }
        }
    }

    //=======================================================================
    // function : IntersectFaces
    // purpose  : Intersection of the faces that should be rebuild to resolve all invalidities
    //=======================================================================
    private void IntersectFaces(NCollection_Map<TopoDS_Shape> theVertsToAvoid, ProgressRange theRange)
    {
        int aNbFR = myFacesToRebuild.Extent();
        if (aNbFR == 0)
        {
            return;
        }

        using ProgressScope aPSOuter = new(theRange, "Rebuilding invalid faces", 10);
        //
        int i, j, k, aNbInv;
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(), aItLE = new();
        //
        // get vertices from invalid edges
        NCollection_Map<TopoDS_Shape> aMVInv = new(TopTools_ShapeMapHasher.Instance), aMVInvAll = new(TopTools_ShapeMapHasher.Instance);
        aNbInv = myInvalidEdges.Extent();
        for (i = 1; i <= aNbInv; ++i)
        {
            TopoDS_Shape aEInv = myInvalidEdges[i];
            bool bValid = myValidEdges.Contains(aEInv);
            for (TopExp_Explorer aExp = new(aEInv, TopAbs_VERTEX); aExp.More(); aExp.Next())
            {
                TopoDS_Shape aV = aExp.Current();
                aMVInvAll.Add(aV);
                if (!bValid)
                {
                    aMVInv.Add(aV);
                }
            }
        }
        //
        bool bLookVertToAvoid = (aMVInv.Extent() > 0);
        //
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMSF = new(TopTools_ShapeMapHasher.Instance),
                                                                          aMDone = new(TopTools_ShapeMapHasher.Instance),
                                                                          aMEInfETrim = new(TopTools_ShapeMapHasher.Instance),
                                                                          aDMVEFull = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aFLE = new(TopTools_ShapeMapHasher.Instance),
                                                                                 aDMEFInv = new(TopTools_ShapeMapHasher.Instance);
        //
        // Add all faces to rebuild to outgoing map <aFLE>,
        // plus link edges and vertices to the faces to
        // define intersection faces
        PrepareFacesForIntersection(bLookVertToAvoid, aFLE, aMDone, aDMSF, aMEInfETrim, aDMVEFull, aDMEFInv);

        // Find vertices to avoid while trimming the edges.
        // These vertices are taken from the invalid edges common between
        // splits of different invalid, but not artificially, faces.
        // Additional condition for these vertices is that all
        // edges adjacent to this vertex must be either invalid
        // or contained in invalid faces
        NCollection_Map<TopoDS_Shape> aMVRInv = new(theVertsToAvoid);
        FindVerticesToAvoid(aDMEFInv, aDMVEFull, aMVRInv);
        //
        aPSOuter.Next();
        if (!aPSOuter.More())
        {
            return;
        }

        // The faces should be intersected selectively -
        // intersect only faces neighboring to the same invalid face
        // and connected to its invalid edges;
        // when dealing with artificially invalid faces for intersection to be
        // complete we need to use not only invalid edges, but also the
        // edges connected to invalid ones

        // find blocks of artificially invalid faces
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDMFImF = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Compound aCFArt = new();
        new BRep_Builder().MakeCompound(ref aCFArt);
        // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<TopoDS_Shape, NCollection_IndexedMap<TopoDS_Shape>> aItM in myArtInvalidFaces)
        {
            TopoDS_Shape aF = aItM.Key;
            NCollection_List<TopoDS_Shape> aLFInv0 = myInvalidFaces.FindFromKey(aF);
            aItLF.Initialize(aLFInv0);
            for (; aItLF.More(); aItLF.Next())
            {
                new BRep_Builder().Add(aCFArt, aItLF.Value());
                aDMFImF.Bind(aItLF.Value(), aF);
            }
        }
        //
        // make connexity blocks
        NCollection_List<TopoDS_Shape> aLCBArt = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFArt, TopAbs_VERTEX, TopAbs_FACE, aLCBArt);
        //
        // alone edges
        NCollection_Map<TopoDS_Shape> aMEAlone = new(TopTools_ShapeMapHasher.Instance), aMEInvOnArt = new(TopTools_ShapeMapHasher.Instance);
        //
        using ProgressScope aPSArt = new(aPSOuter.Next(), null, aLCBArt.Extent());
        NCollection_List<TopoDS_Shape>.Iterator aItLCBArt = new(aLCBArt);
        for (; aItLCBArt.More(); aItLCBArt.Next(), aPSArt.Next())
        {
            if (!aPSArt.More())
            {
                return;
            }
            TopoDS_Shape aCB = aItLCBArt.Value();
            //
            // check if aCB contains splits of only one offset face
            NCollection_Map<TopoDS_Shape> aMFArt = new(TopTools_ShapeMapHasher.Instance);
            TopExp_Explorer aExpF = new(aCB, TopAbs_FACE);
            for (; aExpF.More(); aExpF.Next())
            {
                aMFArt.Add(aDMFImF.Find(aExpF.Current()));
            }
            //
            bool bAlone = (aMFArt.Extent() == 1);
            //
            // vertices on invalid edges
            NCollection_Map<TopoDS_Shape> aMVEInv = new(TopTools_ShapeMapHasher.Instance);
            NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
            // edges that should not be marked as alone - edges having same origins as invalid ones
            NCollection_Map<TopoDS_Shape> aMEAvoid = new(TopTools_ShapeMapHasher.Instance);
            // map to find alone edges by looking for free vertices
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVEVal = new(TopTools_ShapeMapHasher.Instance);
            //
            TopExp_Explorer aExpE = new(aCB, TopAbs_EDGE);
            for (; aExpE.More(); aExpE.Next())
            {
                TopoDS_Shape aE = aExpE.Current();
                if (myInvalidEdges.Contains(aE))
                {
                    aMEInvOnArt.Add(aE);
                    for (TopoDS_Iterator aItV = new(aE); aItV.More(); aItV.Next())
                    {
                        aMVEInv.Add(aItV.Value());
                    }
                    //
                    if (bAlone)
                    {
                        if (myOEOrigins.Find(aE, out NCollection_List<TopoDS_Shape> pLEOr0))
                        {
                            NCollection_List<TopoDS_Shape>.Iterator aItLEOr0 = new(pLEOr0);
                            for (; aItLEOr0.More(); aItLEOr0.Next())
                            {
                                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(myOEImages.Find(aItLEOr0.Value()));
                                for (; aItLEIm.More(); aItLEIm.Next())
                                {
                                    aMEAvoid.Add(aItLEIm.Value());
                                }
                            }
                        }
                    }
                    continue;
                }
                //
                if (aMFence.Add(aE))
                {
                    TopExp.MapShapesAndAncestors(aE, TopAbs_VERTEX, TopAbs_EDGE, aDMVEVal);
                }
            }
            //
            // find edges with free vertices
            int aNbV = aDMVEVal.Extent();
            for (i = 1; i <= aNbV; ++i)
            {
                TopoDS_Shape aV = aDMVEVal.FindKey(i);
                if (!aMVEInv.Contains(aV))
                {
                    continue;
                }
                //
                NCollection_List<TopoDS_Shape> aLEV = aDMVEVal[i];
                if (aLEV.Extent() > 1)
                {
                    continue;
                }
                //
                TopoDS_Shape aE = aLEV.First();
                if (aMEAvoid.Contains(aE))
                {
                    continue;
                }
                //
                aMEAlone.Add(aE);
                //
                // if this alone edge adds nothing to the intersection list
                // it means that the origin of this edge has been split and we need to
                // add the neighboring images of the same origins
                if (aDMSF.Find(aE).Extent() > 1)
                {
                    continue;
                }
                //
                // check also its vertices
                TopoDS_Iterator aItE = new(aE);
                for (; aItE.More(); aItE.Next())
                {
                    TopoDS_Shape aVE = aItE.Value();
                    if (aDMSF.Find(aVE).Extent() > 2)
                    {
                        break;
                    }
                }
                //
                if (aItE.More())
                {
                    continue;
                }
                //
                // the edge is useless - look for other images
                if (!myOEOrigins.Find(aE, out NCollection_List<TopoDS_Shape> pLEOr))
                {
                    continue;
                }
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLEOr = new(pLEOr);
                for (; aItLEOr.More(); aItLEOr.Next())
                {
                    TopoDS_Shape aEOr = aItLEOr.Value();
                    //
                    NCollection_List<TopoDS_Shape> aLEIm = myOEImages.Find(aEOr);
                    NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(aLEIm);
                    for (; aItLEIm.More(); aItLEIm.Next())
                    {
                        TopoDS_Shape aEIm = aItLEIm.Value();
                        //
                        if (aMFence.Contains(aEIm))
                        {
                            aMEAlone.Add(aEIm);
                        }
                    }
                }
            }
        }
        //
        // Get all invalidities from all faces to be used for avoiding
        // repeated usage of the common edges
        NCollection_Map<TopoDS_Shape> aMAllInvs = new(TopTools_ShapeMapHasher.Instance);
        aNbInv = myInvalidFaces.Extent();
        for (k = 1; k <= aNbInv; ++k)
        {
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(myInvalidFaces[k]);
            for (; aIt.More(); aIt.Next())
            {
                TopExp_Explorer aExp = new(aIt.Value(), TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    if (myInvalidEdges.Contains(aE) || aMEAlone.Contains(aE))
                    {
                        aMAllInvs.Add(aE);
                        TopoDS_Iterator aItV = new(aE);
                        for (; aItV.More(); aItV.Next())
                        {
                            aMAllInvs.Add(aItV.Value());
                        }
                    }
                }
            }
        }
        //
        // Bounding vertices of not trimmed edges
        NCollection_Map<TopoDS_Shape> aMVBounds = new(TopTools_ShapeMapHasher.Instance);
        //
        NCollection_Map<TopoDS_Shape> aMECheckExt = new(TopTools_ShapeMapHasher.Instance);
        // Save connections between not trimmed edge and its trimmed parts
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEETrim = new(TopTools_ShapeMapHasher.Instance);
        // Splits of the new edges
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aEImages = new(TopTools_ShapeMapHasher.Instance);
        BRep_Builder aBB = new();

        // Keep connection between blocks of invalid edges to the lists of
        // found edges to be intersected for its treatment
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMOENEdges = new(TopTools_ShapeMapHasher.Instance);

        aNbInv = myInvalidFaces.Extent();
        using ProgressScope aPSInter = new(aPSOuter.Next(5), null, aNbInv);
        for (k = 1; k <= aNbInv; ++k)
        {
            if (!aPSInter.More())
            {
                return;
            }
            TopoDS_Shape aFInv = myInvalidFaces.FindKey(k);
            bool bSelfRebAvoid = myFSelfRebAvoid.Contains(aFInv);
            NCollection_List<TopoDS_Shape> aLFInv = myInvalidFaces[k];
            //
            NCollection_List<TopoDS_Shape> aLCB = new();
            if (aLFInv.Extent() > 1)
            {
                // make compound of invalid faces
                TopoDS_Compound aCFInv = new();
                aBB.MakeCompound(ref aCFInv);
                //
                NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLFInv);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aFIm = aIt.Value();
                    aBB.Add(aCFInv, aFIm);
                }
                //
                // make connexity blocks
                BOPTools_AlgoTools.MakeConnexityBlocks(aCFInv, TopAbs_EDGE, TopAbs_FACE, aLCB);
            }
            else
            {
                aLCB.Assign(aLFInv);
            }
            //
            using ProgressScope aPSCB = new(aPSInter.Next(), null, aLCB.Extent());
            bool bArtificial = myArtInvalidFaces.IsBound(aFInv);
            NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCB);
            for (; aItLCB.More(); aItLCB.Next())
            {
                if (!aPSCB.More())
                {
                    return;
                }
                TopoDS_Shape aCBInv = aItLCB.Value();
                //
                NCollection_Map<TopoDS_Shape> aMEFence = new(TopTools_ShapeMapHasher.Instance);
                //
                TopoDS_Compound aCBE = new();
                aBB.MakeCompound(ref aCBE);
                //
                // remember inside edges and vertices to further check
                NCollection_Map<TopoDS_Shape> anInsideEdges = new(TopTools_ShapeMapHasher.Instance);
                NCollection_Map<TopoDS_Shape> anInsideVertices = new(TopTools_ShapeMapHasher.Instance);
                TopExp_Explorer aExp = new(aCBInv, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    if (myInvalidEdges.Contains(aE) || (bArtificial && aMEAlone.Contains(aE)))
                    {
                        if (aMEFence.Add(aE))
                        {
                            aBB.Add(aCBE, aE);
                            if (!myEdgesToAvoid.Contains(aE) && myInvalidEdges.Contains(aE))
                            {
                                anInsideEdges.Add(aE);
                                TopoDS_Iterator anIt = new(aE);
                                for (; anIt.More(); anIt.Next())
                                {
                                    anInsideVertices.Add(anIt.Value());
                                }
                            }
                        }
                    }
                }
                //
                // make connexity blocks of edges
                NCollection_List<TopoDS_Shape> aLCBE = new();
                BOPTools_AlgoTools.MakeConnexityBlocks(aCBE, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
                //
                using ProgressScope aPSCBE = new(aPSCB.Next(), null, aLCBE.Extent());
                NCollection_List<TopoDS_Shape>.Iterator aItLCBE = new(aLCBE);
                for (; aItLCBE.More(); aItLCBE.Next())
                {
                    if (!aPSCBE.More())
                    {
                        return;
                    }
                    TopoDS_Shape aCBELoc = aItLCBE.Value();
                    //
                    // map of edges and vertices of processing invalidity
                    NCollection_IndexedMap<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance);
                    // map of vertices to trim the new edges
                    TopExp.MapShapes(aCBELoc, TopAbs_EDGE, aME);
                    NCollection_IndexedMap<TopoDS_Shape> aMECV = new(aME);
                    TopExp.MapShapes(aCBELoc, TopAbs_VERTEX, aME);
                    NCollection_IndexedMap<TopoDS_Shape> aMFInt = new(TopTools_ShapeMapHasher.Instance);
                    // additional faces for intersection
                    NCollection_IndexedMap<TopoDS_Shape> aMFIntExt = new(TopTools_ShapeMapHasher.Instance);
                    // splits of faces for intersection
                    NCollection_List<TopoDS_Shape> aLFInt = new();
                    // faces to avoid intersection
                    NCollection_IndexedMap<TopoDS_Shape> aMFAvoid = new(TopTools_ShapeMapHasher.Instance);
                    //
                    FindFacesForIntersection(aFInv, aME, aDMSF, aMVInvAll, bArtificial, aMFAvoid, aMFInt, aMFIntExt, aLFInt);
                    if (aMFInt.Extent() < 3)
                    {
                        // nothing to intersect
                        aPSCBE.Next();
                        continue;
                    }
                    //
                    myIntersectionPairs.Find(aFInv, out NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>>? pMFInter);
                    // intersect the faces, but do not intersect the invalid ones
                    // among each other (except for the artificially invalid faces)
                    NCollection_IndexedMap<TopoDS_Shape> aMEToInt = new(TopTools_ShapeMapHasher.Instance);
                    int aNb = aMFInt.Extent();
                    using ProgressScope aPSIntPair = new(aPSCBE.Next(), null, aNb);
                    for (i = 1; i <= aNb; ++i, aPSIntPair.Next())
                    {
                        if (!aPSIntPair.More())
                        {
                            return;
                        }
                        TopoDS_Face aFi = TopoDS.Face(aMFInt[i]);
                        if (bSelfRebAvoid && aFi.IsSame(aFInv))
                        {
                            continue;
                        }
                        //
                        if (!myOFImages.FindFromKey(aFi, out NCollection_List<TopoDS_Shape> aLFImi))
                        {
                            continue;
                        }
                        //
                        if (!aFLE.FindFromKey(aFi, out NCollection_List<TopoDS_Shape> aLFEi))
                        {
                            continue;
                        }
                        //
                        NCollection_List<TopoDS_Shape> aLFDone = aMDone.ChangeFind(aFi);
                        //
                        NCollection_Map<TopoDS_Shape>? pInterFi = null;
                        if (pMFInter is not null)
                        {
                            pMFInter.Find(aFi, out pInterFi);
                        }
                        if (pMFInter is not null && pInterFi is null)
                        {
                            continue;
                        }

                        // create map of edges and vertices for aLFImi
                        NCollection_Map<TopoDS_Shape> aMEVIm = new(TopTools_ShapeMapHasher.Instance);
                        mapShapes(aLFImi, TopAbs_EDGE, aMEVIm);
                        mapShapes(aLFImi, TopAbs_VERTEX, aMEVIm);

                        bool isIContainsE = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideEdges);
                        bool isIContainsV = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideVertices);

                        for (j = i + 1; j <= aNb; ++j)
                        {
                            TopoDS_Face aFj = TopoDS.Face(aMFInt[j]);
                            if (bSelfRebAvoid && aFj.IsSame(aFInv))
                            {
                                continue;
                            }
                            //
                            if (pInterFi is not null && !pInterFi.Contains(aFj))
                            {
                                continue;
                            }

                            if (!myOFImages.FindFromKey(aFj, out NCollection_List<TopoDS_Shape> aLFImj))
                            {
                                continue;
                            }
                            //
                            if (!aFLE.FindFromKey(aFj, out NCollection_List<TopoDS_Shape> aLFEj))
                            {
                                continue;
                            }

                            // create map of edges and vertices for aLFImi
                            aMEVIm.Clear();
                            mapShapes(aLFImj, TopAbs_EDGE, aMEVIm);
                            mapShapes(aLFImj, TopAbs_VERTEX, aMEVIm);
                            // check images of both faces contain anInsideEdges and anInsideVertices
                            // not process if false and true
                            bool isJContainsE = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideEdges);
                            bool isJContainsV = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideVertices);

                            // Check if one face is connected to inside edge then
                            // the other must be also connected
                            if ((isIContainsE && !isJContainsV) || (isJContainsE && !isIContainsV))
                            {
                                NCollection_List<TopoDS_Shape> aLVC = new();
                                // it is necessary to process the images if they already have
                                // common vertices
                                FindCommonParts(aLFImi, aLFImj, aLVC, TopAbs_VERTEX);

                                if (aLVC.IsEmpty())
                                {
                                    continue;
                                }
                            }
                            //
                            // if there are some common edges between faces
                            // we should use these edges and do not intersect again.
                            NCollection_List<TopoDS_Shape> aLEC = new();
                            FindCommonParts(aLFImi, aLFImj, aLEC);
                            //
                            if (aLEC.Extent() != 0)
                            {
                                // no need to intersect if we have common edges between faces
                                bool bForceUse = aMFIntExt.Contains(aFi) || aMFIntExt.Contains(aFj);
                                ProcessCommonEdges(aLEC, aME, aMEInfETrim, aMAllInvs, bForceUse, aMECV, aMECheckExt,
                                                   aDMEETrim, aLFEi, aLFEj, aMEToInt);

                                // Add common vertices not belonging to the common edges for trimming the intersection
                                // edges
                                NCollection_IndexedMap<TopoDS_Shape> aMVOnCE = new(TopTools_ShapeMapHasher.Instance);
                                for (NCollection_List<TopoDS_Shape>.Iterator itE = new(aLEC); itE.More(); itE.Next())
                                {
                                    TopExp.MapShapes(itE.Value(), TopAbs_VERTEX, aMVOnCE);
                                }

                                NCollection_List<TopoDS_Shape> aLEV = new();
                                FindCommonParts(aLFImi, aLFImj, aLEV, TopAbs_VERTEX);
                                for (NCollection_List<TopoDS_Shape>.Iterator itV = new(aLEV); itV.More(); itV.Next())
                                {
                                    if (!aMVOnCE.Contains(itV.Value()))
                                    {
                                        aMECV.Add(itV.Value());
                                    }
                                }
                                continue;
                            }
                            //
                            // check if both these faces are invalid and sharing edges
                            if (myInvalidFaces.Contains(aFi) && myInvalidFaces.Contains(aFj)
                                && !myArtInvalidFaces.IsBound(aFi) && !myArtInvalidFaces.IsBound(aFj))
                            {
                                continue;
                            }
                            //
                            // check if these two faces have already been treated
                            aItLE.Initialize(aLFDone);
                            for (; aItLE.More(); aItLE.Next())
                            {
                                TopoDS_Shape aF = aItLE.Value();
                                if (aF.IsSame(aFj))
                                {
                                    break;
                                }
                            }
                            //
                            if (aItLE.More())
                            {
                                // use intersection line obtained on the previous steps
                                // plus, find new origins for these lines
                                UpdateIntersectedFaces(aFInv, aFi, aFj, aLFInv, aLFImi, aLFImj, aLFEi, aLFEj, aMEToInt);
                                continue;
                            }
                            //
                            if (aMFAvoid.Contains(aFi) || aMFAvoid.Contains(aFj))
                            {
                                continue;
                            }
                            //
                            aLFDone.Append(aFj);
                            aMDone.ChangeFind(aFj).Append(aFi);
                            //
                            IntersectFaces(aFInv, aFi, aFj, aLFInv, aLFImi, aLFImj, aLFEi, aLFEj, aMECV, aMEToInt);
                        }
                    }
                    //
                    // intersect and trim edges for this chain
                    IntersectAndTrimEdges(aMFInt, aMEToInt, aDMEETrim, aME, aMECV, aMVInv, aMVRInv, aMECheckExt,
                                          bArtificial ? mySSInterfsArt : null, aMVBounds, aEImages);
                    //
                    int iE, aNbEToInt = aMEToInt.Extent();
                    for (iE = 1; iE <= aNbEToInt; ++iE)
                    {
                        TopoDS_Shape aEInt = aMEToInt[iE];
                        TopExp_Explorer anExpE = new(aCBELoc, TopAbs_EDGE);
                        for (; anExpE.More(); anExpE.Next())
                        {
                            TopoDS_Shape aE = anExpE.Current();
                            if (!aDMOENEdges.FindFromKey(aE, out NCollection_List<TopoDS_Shape> pLEToInt))
                            {
                                pLEToInt = aDMOENEdges[aDMOENEdges.Add(aE, new NCollection_List<TopoDS_Shape>())];
                            }
                            AppendToList(pLEToInt, aEInt);
                        }
                    }
                }
            }
        }
        //
        // filter the obtained edges
        UpdateValidEdges(aFLE, aDMOENEdges, aMVBounds, aMEInvOnArt, aMECheckExt, theVertsToAvoid, aEImages, aDMEETrim,
                         aPSOuter.Next(3));
    }

    //=======================================================================
    // function : PrepareFacesForIntersection
    // purpose  : Preparation of the maps for analyzing intersections of the faces
    //=======================================================================
    private void PrepareFacesForIntersection(bool theLookVertToAvoid,
                                             NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theFLE,
                                             NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMDone,
                                             NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMSF,
                                             NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMEInfETrim,
                                             NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVEFull,
                                             NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEFInv)
    {
        int i, aNb = myFacesToRebuild.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = myFacesToRebuild.FindKey(i);
            //
            // OCCT binds copies of the same empty list.
            theFLE.Add(aF, new NCollection_List<TopoDS_Shape>());
            theMDone.Bind(aF, new NCollection_List<TopoDS_Shape>());
            //
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(aF);
            NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aFIm = aItLF.Value();
                TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    // save connection to untrimmed face
                    if (!theDMSF.Find(aE, out NCollection_List<TopoDS_Shape> pLF))
                    {
                        pLF = new NCollection_List<TopoDS_Shape>();
                        theDMSF.Bind(aE, pLF);
                    }
                    AppendToList(pLF, aF);
                    //
                    // save connection to untrimmed edge
                    TopoDS_Shape aEInf = myETrimEInf!.Find(aE);
                    if (!theMEInfETrim.Find(aEInf, out NCollection_List<TopoDS_Shape> pLETrim))
                    {
                        pLETrim = new NCollection_List<TopoDS_Shape>();
                        theMEInfETrim.Bind(aEInf, pLETrim);
                    }
                    AppendToList(pLETrim, aE);
                    //
                    TopExp_Explorer aExpV = new(aE, TopAbs_VERTEX);
                    for (; aExpV.More(); aExpV.Next())
                    {
                        TopoDS_Shape aV = aExpV.Current();
                        // save connection to face
                        if (!theDMSF.Find(aV, out NCollection_List<TopoDS_Shape> pLFV))
                        {
                            pLFV = new NCollection_List<TopoDS_Shape>();
                            theDMSF.Bind(aV, pLFV);
                        }
                        AppendToList(pLFV, aF);
                        //
                        if (theLookVertToAvoid)
                        {
                            // save connection to edges
                            if (!theDMVEFull.Find(aV, out NCollection_List<TopoDS_Shape> pLEV))
                            {
                                pLEV = new NCollection_List<TopoDS_Shape>();
                                theDMVEFull.Bind(aV, pLEV);
                            }
                            AppendToList(pLEV, aE);
                        }
                    }
                }
            }
            //
            if (theLookVertToAvoid)
            {
                // get edges of invalid faces (from invalid splits only)
                if (!myInvalidFaces.FindFromKey(aF, out NCollection_List<TopoDS_Shape> pLFInv) || myArtInvalidFaces.IsBound(aF))
                {
                    continue;
                }
                //
                aItLF.Initialize(pLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFInv = aItLF.Value();
                    TopExp_Explorer aExp = new(aFInv, TopAbs_EDGE);
                    for (; aExp.More(); aExp.Next())
                    {
                        TopoDS_Shape aE = aExp.Current();
                        if (!theDMEFInv.FindFromKey(aE, out NCollection_List<TopoDS_Shape> pLF))
                        {
                            pLF = theDMEFInv[theDMEFInv.Add(aE, new NCollection_List<TopoDS_Shape>())];
                        }
                        AppendToList(pLF, aF);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FindVerticesToAvoid
    // purpose  : Looking for the invalid vertices
    //=======================================================================
    private void FindVerticesToAvoid(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEFInv,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMVEFull,
                                     NCollection_Map<TopoDS_Shape> theMVRInv)
    {
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        int i, aNb = theDMEFInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<TopoDS_Shape> aLFInv = theDMEFInv[i];
            if (aLFInv.Extent() == 1)
            {
                continue;
            }
            //
            TopoDS_Shape aE = theDMEFInv.FindKey(i);
            if (!myInvalidEdges.Contains(aE) || myValidEdges.Contains(aE))
            {
                continue;
            }

            if (!aMFence.Add(aE))
            {
                continue;
            }

            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVEEdges = new(TopTools_ShapeMapHasher.Instance);
            // Do not check the splitting vertices, but check only the ending ones
            if (myOEOrigins.Find(aE, out NCollection_List<TopoDS_Shape> pLEOr))
            {
                NCollection_List<TopoDS_Shape>.Iterator aItLEOr = new(pLEOr);
                for (; aItLEOr.More(); aItLEOr.Next())
                {
                    NCollection_List<TopoDS_Shape> aLEIm = myOEImages.Find(aItLEOr.Value());
                    NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(aLEIm);
                    for (; aItLEIm.More(); aItLEIm.Next())
                    {
                        aMFence.Add(aItLEIm.Value());
                        TopExp.MapShapesAndAncestors(aItLEIm.Value(), TopAbs_VERTEX, TopAbs_EDGE, aMVEEdges);
                    }
                }
            }
            else
            {
                TopExp.MapShapesAndAncestors(aE, TopAbs_VERTEX, TopAbs_EDGE, aMVEEdges);
            }

            int j, aNbV = aMVEEdges.Extent();
            for (j = 1; j <= aNbV; ++j)
            {
                if (aMVEEdges[j].Extent() != 1)
                {
                    continue;
                }

                TopoDS_Shape aV = aMVEEdges.FindKey(j);
                if (!aMFence.Add(aV))
                {
                    continue;
                }
                if (!theDMVEFull.Find(aV, out NCollection_List<TopoDS_Shape> pLE))
                {
                    // isolated vertex
                    theMVRInv.Add(aV);
                    continue;
                }
                //
                // If all edges sharing the vertex are either invalid or
                // the vertex is connected to at least two inverted edges
                // mark the vertex to be avoided in the new splits
                int iNbEInverted = 0;
                bool bAllEdgesInv = true;
                NCollection_List<TopoDS_Shape>.Iterator aItLE = new(pLE);
                for (; aItLE.More(); aItLE.Next())
                {
                    TopoDS_Shape aEV = aItLE.Value();

                    if (myInvertedEdges.Contains(aEV))
                    {
                        ++iNbEInverted;
                    }

                    if (bAllEdgesInv)
                    {
                        bAllEdgesInv = myInvalidEdges.Contains(aEV);
                    }
                }

                if (iNbEInverted > 1 || bAllEdgesInv)
                {
                    theMVRInv.Add(aV);
                }
            }
        }
    }

    //=======================================================================
    // function : FindFacesForIntersection
    // purpose  : Looking for the faces around each invalidity for intersection
    //=======================================================================
    private void FindFacesForIntersection(in TopoDS_Shape theFInv, NCollection_IndexedMap<TopoDS_Shape> theME,
                                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMSF,
                                          NCollection_Map<TopoDS_Shape> theMVInvAll, bool theArtCase,
                                          NCollection_IndexedMap<TopoDS_Shape> theMFAvoid,
                                          NCollection_IndexedMap<TopoDS_Shape> theMFInt,
                                          NCollection_IndexedMap<TopoDS_Shape> theMFIntExt,
                                          NCollection_List<TopoDS_Shape> theLFImInt)
    {
        int i, aNbE = theME.Extent();
        //
        NCollection_IndexedMap<TopoDS_Shape> aMShapes = new(TopTools_ShapeMapHasher.Instance);
        //
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Shape aS = theME[i];
            if (!theDMSF.IsBound(aS))
            {
                continue;
            }
            //
            // in artificial case we intersect the faces which are close to invalidity
            bool bAvoid = theArtCase ? ((aS.ShapeType() == TopAbs_VERTEX) && !theMVInvAll.Contains(aS)) : false;
            //
            NCollection_List<TopoDS_Shape> aLF0 = theDMSF.Find(aS);
            NCollection_List<TopoDS_Shape>.Iterator aItLF0 = new(aLF0);
            for (; aItLF0.More(); aItLF0.Next())
            {
                TopoDS_Shape aF = aItLF0.Value();
                if (theMFInt.Contains(aF))
                {
                    continue;
                }
                //
                if (bAvoid && myArtInvalidFaces.IsBound(aF))
                {
                    theMFAvoid.Add(aF);
                }
                //
                theMFInt.Add(aF);
                //
                bool bUse = !aF.IsSame(theFInv);
                //
                NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(aF);
                NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    TopoDS_Shape aFIm = aItLFIm.Value();
                    theLFImInt.Append(aFIm);
                    if (bUse)
                    {
                        TopExp.MapShapes(aFIm, TopAbs_EDGE, aMShapes);
                    }
                }
            }
        }
        //
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aSSInterfsMap = theArtCase ? mySSInterfsArt : mySSInterfs;
        if (!aSSInterfsMap.Find(theFInv, out NCollection_List<TopoDS_Shape> pLFInv))
        {
            return;
        }
        //
        NCollection_Map<TopoDS_Shape> aMF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(pLFInv);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Shape aF = aItLF.Value();
            aMF.Add(aF);
        }
        //
        // the faces should be unique in each place
        TopoDS_Compound aCF = new();
        new BRep_Builder().MakeCompound(ref aCF);
        //
        NCollection_IndexedMap<TopoDS_Shape> aMFToAdd = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDMFOr = new(TopTools_ShapeMapHasher.Instance);
        //
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Shape aS = theME[i];
            if (!aSSInterfsMap.Find(aS, out NCollection_List<TopoDS_Shape> pLF))
            {
                continue;
            }
            //
            aItLF.Initialize(pLF);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aF = aItLF.Value();
                if (theMFInt.Contains(aF) || aMFToAdd.Contains(aF) || !aMF.Contains(aF))
                {
                    continue;
                }
                //
                // check if the face has some connection to already added for intersection faces
                NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(aF);
                if (!theArtCase)
                {
                    NCollection_List<TopoDS_Shape>.Iterator aItLFIm0 = new(aLFIm);
                    for (; aItLFIm0.More(); aItLFIm0.Next())
                    {
                        TopoDS_Shape aFIm = aItLFIm0.Value();
                        TopExp_Explorer aExp = new(aFIm, TopAbs_EDGE);
                        for (; aExp.More(); aExp.Next())
                        {
                            if (aMShapes.Contains(aExp.Current()))
                            {
                                break;
                            }
                        }
                        if (aExp.More())
                        {
                            break;
                        }
                    }
                    if (!aItLFIm0.More())
                    {
                        continue;
                    }
                }
                //
                aMFToAdd.Add(aF);
                NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    TopoDS_Shape aFIm = aItLFIm.Value();
                    aDMFOr.Bind(aFIm, aF);
                    new BRep_Builder().Add(aCF, aFIm);
                }
            }
        }
        //
        if (aMFToAdd.IsEmpty())
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCF, TopAbs_EDGE, TopAbs_FACE, aLCB);
        //
        if ((aLCB.Extent() == 1) && (aMFToAdd.Extent() > 1))
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCB);
        for (; aItLCB.More(); aItLCB.Next())
        {
            TopoDS_Shape aCB = aItLCB.Value();
            aMFToAdd.Clear();
            TopExp_Explorer aExpF = new(aCB, TopAbs_FACE);
            for (; aExpF.More(); aExpF.Next())
            {
                TopoDS_Shape aFIm = aExpF.Current();
                aMFToAdd.Add(aDMFOr.Find(aFIm));
            }
            //
            if (aMFToAdd.Extent() == 1)
            {
                TopoDS_Shape aF = aMFToAdd[1];
                //
                theMFInt.Add(aF);
                theMFIntExt.Add(aF);
                //
                NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(aF);
                NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    TopoDS_Shape aFIm = aItLFIm.Value();
                    theLFImInt.Append(aFIm);
                }
            }
        }
    }

    //=======================================================================
    // function : ProcessCommonEdges
    // purpose  : Analyzing the common edges between splits of offset faces
    //=======================================================================
    private void ProcessCommonEdges(NCollection_List<TopoDS_Shape> theLEC, NCollection_IndexedMap<TopoDS_Shape> theME,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMEInfETrim,
                                    NCollection_Map<TopoDS_Shape> theAllInvs, bool theForceUse,
                                    NCollection_IndexedMap<TopoDS_Shape> theMECV, NCollection_Map<TopoDS_Shape> theMECheckExt,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEETrim,
                                    NCollection_List<TopoDS_Shape> theLFEi, NCollection_List<TopoDS_Shape> theLFEj,
                                    NCollection_IndexedMap<TopoDS_Shape> theMEToInt)
    {
        NCollection_List<TopoDS_Shape> aLEC = new();
        // process common edges
        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(theLEC);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aEC = aItLE.Value();
            //
            // check first if common edges are valid
            if (myInvalidEdges.Contains(aEC) && !myValidEdges.Contains(aEC))
            {
                continue;
            }
            //
            // common edge should have connection to current invalidity
            if (theME.Contains(aEC))
            {
                aLEC.Append(aEC);
                continue;
            }
            //
            TopoDS_Iterator aItV = new(aEC);
            for (; aItV.More(); aItV.Next())
            {
                TopoDS_Shape aVE = aItV.Value();
                if (theME.Contains(aVE))
                {
                    aLEC.Append(aEC);
                    break;
                }
            }
        }
        //
        bool bUseOnlyInf = aLEC.IsEmpty();
        if (bUseOnlyInf)
        {
            if (theForceUse)
            {
                aLEC.Assign(theLEC);
            }
            else
            {
                aItLE.Initialize(theLEC);
                for (; aItLE.More(); aItLE.Next())
                {
                    TopoDS_Shape aEC = aItLE.Value();
                    // check if all images of the origin of this edge
                    // are not connected to any invalidity
                    TopoDS_Shape aEInt0 = myETrimEInf!.Find(aEC);
                    NCollection_List<TopoDS_Shape> aLVE = theMEInfETrim.Find(aEInt0);
                    NCollection_List<TopoDS_Shape>.Iterator aItLVE = new(aLVE);
                    for (; aItLVE.More(); aItLVE.Next())
                    {
                        TopoDS_Shape aECx = aItLVE.Value();
                        if (theAllInvs.Contains(aECx) || myInvalidEdges.Contains(aECx))
                        {
                            return;
                        }
                        //
                        TopoDS_Iterator aItV = new(aECx);
                        for (; aItV.More(); aItV.Next())
                        {
                            if (theAllInvs.Contains(aItV.Value()))
                            {
                                return;
                            }
                        }
                    }
                    // use only one element
                    if (aLEC.IsEmpty())
                    {
                        aLEC.Append(aEC);
                    }
                }
            }
        }
        //
        aItLE.Initialize(aLEC);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aEC = aItLE.Value();
            //
            TopoDS_Shape aEInt = myETrimEInf!.Find(aEC);
            if (!bUseOnlyInf)
            {
                // find the edges of the same original edge
                // and take their vertices as well
                NCollection_List<TopoDS_Shape> aLVE = theMEInfETrim.Find(aEInt);
                NCollection_List<TopoDS_Shape>.Iterator aItLVE = new(aLVE);
                for (; aItLVE.More(); aItLVE.Next())
                {
                    TopoDS_Shape aECx = aItLVE.Value();
                    //
                    if (!myOEOrigins.Find(aECx, out NCollection_List<TopoDS_Shape> pLEOr) || (pLEOr.Extent() == 1))
                    {
                        TopExp.MapShapes(aECx, TopAbs_VERTEX, theMECV);
                    }
                }
                //
                // bind unlimited edge to its trimmed part in face to update maps of
                // images and origins in the future
                if (!theDMEETrim.Find(aEInt, out NCollection_List<TopoDS_Shape> pLTAdded))
                {
                    pLTAdded = new NCollection_List<TopoDS_Shape>();
                    theDMEETrim.Bind(aEInt, pLTAdded);
                }
                AppendToList(pLTAdded, aEC);
            }
            else if (!theForceUse)
            {
                theMECheckExt.Add(aEInt);
            }
            //
            AppendToList(theLFEi, aEInt);
            AppendToList(theLFEj, aEInt);
            theMEToInt.Add(aEInt);
        }
    }

    //=================================================================================================

    private static void FindOrigins(NCollection_List<TopoDS_Shape> theLFIm1, NCollection_List<TopoDS_Shape> theLFIm2,
                                    NCollection_IndexedMap<TopoDS_Shape> theME,
                                    NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theOrigins,
                                    NCollection_List<TopoDS_Shape> theLEOr)
    {
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<TopoDS_Shape> aLF = i == 0 ? theLFIm1 : theLFIm2;
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLF);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aF = aIt.Value();
                //
                TopExp_Explorer aExp = new(aF, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    //
                    if (theME.Contains(aE) && theOrigins.IsBound(aE))
                    {
                        NCollection_List<TopoDS_Shape> aLEOr = theOrigins.Find(aE);
                        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLEOr);
                        for (; aItLE.More(); aItLE.Next())
                        {
                            TopoDS_Shape aEOr = aItLE.Value();
                            //
                            if (aMFence.Add(aEOr) && (aEOr.ShapeType() == TopAbs_EDGE))
                            {
                                theLEOr.Append(aEOr);
                            }
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : UpdateIntersectedFaces
    // purpose  : Updating the already interfered faces
    //=======================================================================
    private void UpdateIntersectedFaces(in TopoDS_Shape theFInv, in TopoDS_Shape theFi, in TopoDS_Shape theFj,
                                        NCollection_List<TopoDS_Shape> theLFInv, NCollection_List<TopoDS_Shape> theLFImi,
                                        NCollection_List<TopoDS_Shape> theLFImj, NCollection_List<TopoDS_Shape> theLFEi,
                                        NCollection_List<TopoDS_Shape> theLFEj, NCollection_IndexedMap<TopoDS_Shape> theMEToInt)
    {
        // Find common edges in these two lists
        NCollection_Map<TopoDS_Shape> aMEi = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(theLFEi);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aE = aItLE.Value();
            aMEi.Add(aE);
        }
        //
        // find origins
        NCollection_IndexedMap<TopoDS_Shape> aMEToFindOrigins = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aLEToFindOrigins = new();
        if (!theFi.IsSame(theFInv))
        {
            FindCommonParts(theLFImi, theLFInv, aLEToFindOrigins);
        }
        if (!theFj.IsSame(theFInv))
        {
            FindCommonParts(theLFImj, theLFInv, aLEToFindOrigins);
        }
        //
        NCollection_List<TopoDS_Shape> aLEOrInit = new();
        aItLE.Initialize(aLEToFindOrigins);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aEC = aItLE.Value();
            aMEToFindOrigins.Add(aEC);
        }
        //
        FindOrigins(theLFImi, theLFImj, aMEToFindOrigins, myEdgesOrigins!, aLEOrInit);
        //
        aItLE.Initialize(theLFEj);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aE = aItLE.Value();
            if (aMEi.Contains(aE))
            {
                theMEToInt.Add(aE);
                if (aLEOrInit.Extent() != 0)
                {
                    if (myEdgesOrigins!.IsBound(aE))
                    {
                        NCollection_List<TopoDS_Shape> aLEOr = myEdgesOrigins.ChangeFind(aE);
                        NCollection_List<TopoDS_Shape>.Iterator aItLEOr = new(aLEOrInit);
                        for (; aItLEOr.More(); aItLEOr.Next())
                        {
                            TopoDS_Shape aEOr = aItLEOr.Value();
                            AppendToList(aLEOr, aEOr);
                        }
                    }
                    else
                    {
                        myEdgesOrigins.Bind(aE, new NCollection_List<TopoDS_Shape>(aLEOrInit));
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IntersectFaces
    // purpose  : Intersection of the pair of faces
    //=======================================================================
    private void IntersectFaces(in TopoDS_Shape theFInv, in TopoDS_Shape theFi, in TopoDS_Shape theFj,
                                NCollection_List<TopoDS_Shape> theLFInv, NCollection_List<TopoDS_Shape> theLFImi,
                                NCollection_List<TopoDS_Shape> theLFImj, NCollection_List<TopoDS_Shape> theLFEi,
                                NCollection_List<TopoDS_Shape> theLFEj, NCollection_IndexedMap<TopoDS_Shape> theMECV,
                                NCollection_IndexedMap<TopoDS_Shape> theMEToInt)
    {
        // intersect faces
        TopAbs_State aSide = TopAbs_OUT;
        NCollection_List<TopoDS_Shape> aLInt1 = new(), aLInt2 = new();
        TopoDS_Edge aNullEdge = new();
        TopoDS_Face aNullFace = new();
        BRepOffset_Tool.Inter3D(TopoDS.Face(theFi), TopoDS.Face(theFj), aLInt1, aLInt2, aSide, aNullEdge, aNullFace,
                                aNullFace);
        //
        if (aLInt1.IsEmpty())
        {
            return;
        }
        //
        // find common vertices for trimming edges
        NCollection_List<TopoDS_Shape> aLCV = new();
        NCollection_List<TopoDS_Shape>.Iterator aItLE = new();
        FindCommonParts(theLFImi, theLFImj, aLCV, TopAbs_VERTEX);
        if (aLCV.Extent() > 1)
        {
            aItLE.Initialize(aLCV);
            for (; aItLE.More(); aItLE.Next())
            {
                TopoDS_Shape aCV = aItLE.Value();
                theMECV.Add(aCV);
            }
        }
        //
        // find origins
        NCollection_IndexedMap<TopoDS_Shape> aMEToFindOrigins = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aLEToFindOrigins = new();
        if (!theFi.IsSame(theFInv))
        {
            FindCommonParts(theLFImi, theLFInv, aLEToFindOrigins);
        }
        if (!theFj.IsSame(theFInv))
        {
            FindCommonParts(theLFImj, theLFInv, aLEToFindOrigins);
        }
        NCollection_List<TopoDS_Shape> aLEOrInit = new();
        aItLE.Initialize(aLEToFindOrigins);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aEC = aItLE.Value();
            aMEToFindOrigins.Add(aEC);
        }
        //
        FindOrigins(theLFImi, theLFImj, aMEToFindOrigins, myEdgesOrigins!, aLEOrInit);
        //
        aItLE.Initialize(aLInt1);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aEInt = aItLE.Value();
            theLFEi.Append(aEInt);
            theLFEj.Append(aEInt);
            //
            if (aLEOrInit.Extent() != 0)
            {
                myEdgesOrigins!.Bind(aEInt, new NCollection_List<TopoDS_Shape>(aLEOrInit));
            }
            //
            theMEToInt.Add(aEInt);
        }
    }

    //=======================================================================
    // function : IntersectAndTrimEdges
    // purpose  : Intersection of the new intersection edges among themselves
    //=======================================================================
    private void IntersectAndTrimEdges(NCollection_IndexedMap<TopoDS_Shape> theMFInt, NCollection_IndexedMap<TopoDS_Shape> theMEInt,
                                       NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEETrim,
                                       NCollection_IndexedMap<TopoDS_Shape> theMSInv, NCollection_IndexedMap<TopoDS_Shape> theMVE,
                                       NCollection_Map<TopoDS_Shape> theVertsToAvoid,
                                       NCollection_Map<TopoDS_Shape> theNewVertsToAvoid,
                                       NCollection_Map<TopoDS_Shape> theMECheckExt,
                                       NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>? theSSInterfs,
                                       NCollection_Map<TopoDS_Shape> theMVBounds,
                                       NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages)
    {
        int i, aNb = theMEInt.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape> aLArgs = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(), aIt1 = new();
        TopExp_Explorer aExp = new();
        //
        // get vertices from the splits of intersected faces.
        // vertices are taken from the edges close to invalidity
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVE = new(TopTools_ShapeMapHasher.Instance);
        aNb = theMFInt.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aF = theMFInt[i];
            NCollection_List<TopoDS_Shape> aLE = myFacesToRebuild.FindFromKey(aF);
            //
            aIt.Initialize(aLE);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aE = aIt.Value();
                TopExp.MapShapesAndAncestors(aE, TopAbs_VERTEX, TopAbs_EDGE, aDMVE);
                //
                aExp.Init(aE, TopAbs_VERTEX);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aV1 = aExp.Current();
                    if (!theVertsToAvoid.Contains(aV1) && theMVE.Contains(aV1) && aMFence.Add(aV1))
                    {
                        aLArgs.Append(aV1);
                    }
                }
            }
        }
        //
        aNb = theMSInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aS = theMSInv[i];
            // edge case
            if (theSSInterfs is not null)
            {
                if (theSSInterfs.Find(aS, out NCollection_List<TopoDS_Shape> pLV))
                {
                    // Add vertices from intersection info to trim section edges of artificial faces
                    for (NCollection_List<TopoDS_Shape>.Iterator itLV = new(pLV); itLV.More(); itLV.Next())
                    {
                        if (itLV.Value().ShapeType() == TopAbs_VERTEX)
                        {
                            aLArgs.Append(itLV.Value());
                        }
                    }
                }
            }

            // vertex case
            if (aDMVE.FindFromKey(aS, out NCollection_List<TopoDS_Shape> pLVE))
            {
                aIt.Initialize(pLVE);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aE = aIt.Value();
                    //
                    aExp.Init(aE, TopAbs_VERTEX);
                    for (; aExp.More(); aExp.Next())
                    {
                        TopoDS_Shape aV1 = aExp.Current();
                        if (!theVertsToAvoid.Contains(aV1) && aMFence.Add(aV1))
                        {
                            aLArgs.Append(aV1);
                        }
                    }
                }
            }
        }
        //
        // bounding vertices of untrimmed edges
        NCollection_List<TopoDS_Shape> aLVBounds = new();
        // new intersection edges
        NCollection_List<TopoDS_Shape> aLENew = new();
        // get edges to intersect
        NCollection_List<TopoDS_Shape> aLEInt = new();
        // Common intersection edges. Should be intersected separately
        NCollection_List<TopoDS_Shape> aLCE = new();
        //
        aNb = theMEInt.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aE = theMEInt[i];
            if (theMECheckExt.Contains(aE))
            {
                // avoid trimming of the intersection edges by additional common edges
                aLCE.Append(aE);
                continue;
            }
            //
            if (!theDMEETrim.IsBound(aE))
            {
                aLENew.Append(aE);
            }
            //
            aLEInt.Append(aE);
            aLArgs.Append(aE);
            //
            aExp.Init(aE, TopAbs_VERTEX);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aV = aExp.Current();
                aLVBounds.Append(aV);
            }
        }
        //
        // Intersect Edges
        BOPAlgo_Builder aGF = new();
        aGF.SetArguments(aLArgs);
        aGF.Perform();
        if (aGF.HasErrors())
        {
            return;
        }
        //
        // update vertices to avoid with SD vertices
        aIt.Initialize(aLVBounds);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aV = aIt.Value();
            NCollection_List<TopoDS_Shape> aLVIm = ModifiedOf(aGF, aV);
            if (aLVIm.IsEmpty())
            {
                theMVBounds.Add(aV);
            }
            else
            {
                TopoDS_Shape aVIm = aLVIm.First();
                theMVBounds.Add(aVIm);
            }
        }
        //
        // find invalid splits of edges
        NCollection_Map<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
        GetInvalidEdges(theNewVertsToAvoid, theMVBounds, aGF, aMEInv);
        //
        BRep_Builder aBB = new();
        // get valid splits to intersect with the commons
        TopoDS_Compound aCEIm = new();
        aBB.MakeCompound(ref aCEIm);
        //
        // remove the splits containing vertices from invalid edges
        aIt.Initialize(aLEInt);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            //
            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(aGF, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }
            //
            aIt1.Initialize(aLEIm);
            for (; aIt1.More();)
            {
                TopoDS_Shape aEIm = aIt1.Value();
                //
                if (aMEInv.Contains(aEIm))
                {
                    aLEIm.Remove(aIt1);
                }
                else
                {
                    aBB.Add(aCEIm, aEIm);
                    aIt1.Next();
                }
            }
            //
            if (aLEIm.Extent() != 0)
            {
                if (!theEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
                {
                    pLEIm = new NCollection_List<TopoDS_Shape>();
                    theEImages.Bind(aE, pLEIm);
                }
                pLEIm.Append(aLEIm);
            }
        }
        //
        if (aLCE.Extent() == 0)
        {
            return;
        }
        //
        // trim common edges by other intersection edges
        BOPAlgo_Builder aGFCE = new();
        aGFCE.SetArguments(aLCE);
        aGFCE.AddArgument(aCEIm);
        aGFCE.Perform();
        //
        if (aGFCE.HasErrors())
        {
            return;
        }
        //
        BOPDS_DS pDS = aGFCE.PDS()!;
        NCollection_List<TopoDS_Shape>.Iterator aItLCE = new(aLCE);
        for (; aItLCE.More(); aItLCE.Next())
        {
            TopoDS_Shape aE = aItLCE.Value();
            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(aGFCE, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }
            //
            // check if it's not coincide with some intersection edge
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(pDS.PaveBlocks(pDS.Index(aE)));
            for (; aItLPB.More(); aItLPB.Next())
            {
                if (pDS.IsCommonBlock(aItLPB.Value()))
                {
                    // find with what it is a common
                    NCollection_List<BOPDS_PaveBlock> aLPBC = pDS.CommonBlock(aItLPB.Value())!.PaveBlocks();
                    NCollection_List<BOPDS_PaveBlock>.Iterator aItLPBC = new(aLPBC);
                    for (; aItLPBC.More(); aItLPBC.Next())
                    {
                        TopoDS_Shape aEC = pDS.Shape(aItLPBC.Value().OriginalEdge());
                        if (!theMECheckExt.Contains(aEC))
                        {
                            break;
                        }
                    }
                    if (aItLPBC.More())
                    {
                        break;
                    }
                }
            }
            if (aItLPB.More())
            {
                // avoid creation of unnecessary splits from commons which
                // coincide with intersection edges
                continue;
            }
            //
            // save the images
            if (!theEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
            {
                pLEIm = new NCollection_List<TopoDS_Shape>();
                theEImages.Bind(aE, pLEIm);
            }
            pLEIm.Append(aLEIm);
            //
            // save bounding vertices
            for (TopoDS_Iterator aItV = new(aE); aItV.More(); aItV.Next())
            {
                TopoDS_Shape aV = aItV.Value();
                NCollection_List<TopoDS_Shape> aLVIm = ModifiedOf(aGFCE, aV);
                theMVBounds.Add(aLVIm.IsEmpty() ? aV : aLVIm.First());
            }
        }
    }

    //=======================================================================
    // function : GetInvalidEdges
    // purpose  : Looking for the invalid edges by intersecting with invalid vertices
    //=======================================================================
    private void GetInvalidEdges(NCollection_Map<TopoDS_Shape> theVertsToAvoid, NCollection_Map<TopoDS_Shape> theMVBounds,
                                 BOPAlgo_Builder theGF, NCollection_Map<TopoDS_Shape> theMEInv)
    {
        if (theVertsToAvoid.IsEmpty())
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        // get vertices created with intersection edges
        TopoDS_Shape aRes = theGF.InternalShape();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aRes, TopAbs_VERTEX, TopAbs_EDGE, aDMVE);
        //
        BOPDS_DS pDS = theGF.PDS()!;
        //
        // find invalid splits of edges
        // check if the vertex is invalid:
        // a. it may be the vertex SD with the vertices to avoid
        // b. or it may be the vertex which is created by the intersection
        //    of only existing edges, i.e. no new intersection edges goes
        //    through this vertex
        //
        NCollection_Map<TopoDS_Shape> aMVInv = new(TopTools_ShapeMapHasher.Instance);
        int i, aNb = aDMVE.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aDMVE.FindKey(i));
            if (theMVBounds.Contains(aV))
            {
                continue;
            }
            //
            int nV = pDS.Index(aV);
            if ((nV >= 0) && !pDS.IsNewShape(nV))
            {
                continue;
            }
            //
            // OCCT.NET: OCCT iterates the map by its vertices' addresses; the port in insertion order (decision 1032).
            bool bFound = false;
            foreach (TopoDS_Shape aItM in theVertsToAvoid)
            {
                TopoDS_Vertex aVInv = TopoDS.Vertex(aItM);
                int iFlag = BOPTools_AlgoTools.ComputeVV(aV, aVInv);
                if (iFlag == 0)
                {
                    aMVInv.Add(aV);
                    bFound = true;
                    break;
                }
            }
            //
            if (bFound)
            {
                NCollection_List<TopoDS_Shape> aLVE = aDMVE.FindFromKey(aV);
                aIt.Initialize(aLVE);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aE = aIt.Value();
                    theMEInv.Add(aE);
                }
            }
        }
    }

    //=======================================================================
    // function : UpdateValidEdges
    // purpose  : Making the new splits and updating the maps
    //=======================================================================
    private void UpdateValidEdges(NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theFLE,
                                  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theOENEdges,
                                  NCollection_Map<TopoDS_Shape> theMVBounds, NCollection_Map<TopoDS_Shape> theMEInvOnArt,
                                  NCollection_Map<TopoDS_Shape> theMECheckExt, NCollection_Map<TopoDS_Shape> theVertsToAvoid,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages,
                                  NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEETrim,
                                  ProgressRange theRange)
    {
        using ProgressScope aPSOuter = new(theRange, "Updating edges", 10);
        // update images and origins of edges, plus update AsDes
        //
        // new edges
        NCollection_List<TopoDS_Shape> aLE = new();
        // back connection from edges to faces
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMELF = new(TopTools_ShapeMapHasher.Instance);
        //
        NCollection_Map<TopoDS_Shape> aMETmp = new(TopTools_ShapeMapHasher.Instance);
        int i, aNb = theFLE.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Face aF = TopoDS.Face(theFLE.FindKey(i));
            //
            NCollection_List<TopoDS_Shape> aLEInt = theFLE[i];
            NCollection_List<TopoDS_Shape>.Iterator aItLE0 = new(aLEInt);
            for (; aItLE0.More(); aItLE0.Next())
            {
                TopoDS_Shape aE = aItLE0.Value();
                if ((theMECheckExt.Contains(aE) || aMETmp.Contains(aE)) && !theEImages.IsBound(aE))
                {
                    theMECheckExt.Remove(aE);
                    aMETmp.Add(aE);
                    continue;
                }
                if (!aMELF.Find(aE, out NCollection_List<TopoDS_Shape> pLF))
                {
                    pLF = new NCollection_List<TopoDS_Shape>();
                    aMELF.Bind(aE, pLF);
                    aLE.Append(aE);
                }
                pLF.Append(aF);
            }
        }
        //
        if (aLE.IsEmpty())
        {
            return;
        }
        //
        // bounding edges, that are going to be replaced
        NCollection_Map<TopoDS_Shape> aMEB = new(TopTools_ShapeMapHasher.Instance);
        //
        // new intersection edges
        NCollection_Map<TopoDS_Shape> aMENew = new(TopTools_ShapeMapHasher.Instance);
        // map of old vertices
        NCollection_Map<TopoDS_Shape> aMVOld = new(TopTools_ShapeMapHasher.Instance);
        // back connection to untrimmed edges
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMEOr = new(TopTools_ShapeMapHasher.Instance);
        //
        // trim the new intersection edges
        TrimNewIntersectionEdges(aLE, theEETrim, theMVBounds, theMECheckExt, theEImages, aMEB, aMVOld, aMENew, aDMEOr, aMELF);
        //
        if (theEImages.IsEmpty())
        {
            // No new splits is preserved
            // update intersection edges and exit
            UpdateNewIntersectionEdges(aLE, aMELF, theEImages, theEETrim);
            return;
        }

        aPSOuter.Next();
        if (!aPSOuter.More())
        {
            return;
        }

        BRep_Builder aBB = new();

        // Make connexity blocks of the invalid edges
        // and treat each block separately

        // Compound of all invalid edges to make the blocks
        TopoDS_Compound aCEAll = new();
        aBB.MakeCompound(ref aCEAll);

        int aNbE = theOENEdges.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            aBB.Add(aCEAll, theOENEdges.FindKey(i));
        }

        // Separate the edges into blocks
        NCollection_List<TopoDS_Shape> aLBlocks = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCEAll, TopAbs_VERTEX, TopAbs_EDGE, aLBlocks);

        // Perform intersection of the new splits for each block

        // Intersected splits
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMBlocksSp = new(TopTools_ShapeMapHasher.Instance);

        using ProgressScope aPSB = new(aPSOuter.Next(), null, aLBlocks.Extent());
        NCollection_List<TopoDS_Shape>.Iterator aItLB = new(aLBlocks);
        for (; aItLB.More(); aItLB.Next(), aPSB.Next())
        {
            if (!aPSB.More())
            {
                return;
            }
            TopoDS_Shape aBlock = aItLB.Value();

            // Get the list of new edges for the block
            NCollection_List<TopoDS_Shape> aBlockLENew = new();
            {
                // Fence map
                NCollection_Map<TopoDS_Shape> aMEFence = new(TopTools_ShapeMapHasher.Instance);
                TopExp_Explorer anExpE = new(aBlock, TopAbs_EDGE);
                for (; anExpE.More(); anExpE.Next())
                {
                    TopoDS_Shape aE = anExpE.Current();
                    NCollection_List<TopoDS_Shape> aLEInt = theOENEdges.FindFromKey(aE);
                    NCollection_List<TopoDS_Shape>.Iterator aItLEInt = new(aLEInt);
                    for (; aItLEInt.More(); aItLEInt.Next())
                    {
                        if (aMEFence.Add(aItLEInt.Value()))
                        {
                            aBlockLENew.Append(aItLEInt.Value());
                        }
                    }
                }
            }

            if (aBlockLENew.IsEmpty())
            {
                continue;
            }

            // Get the splits of new edges to intersect
            NCollection_List<TopoDS_Shape> aLSplits = new();

            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aBlockLENew);
            for (; aItLE.More(); aItLE.Next())
            {
                TopoDS_Shape aE = aItLE.Value();
                if (!theEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm) || pLEIm.IsEmpty())
                {
                    continue;
                }

                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    aLSplits.Append(aItLEIm.Value());
                }
            }

            if (aLSplits.IsEmpty())
            {
                continue;
            }

            TopoDS_Shape aCE = new();
            if (aLSplits.Extent() > 1)
            {
                // Intersect the new splits among themselves to avoid self-intersections
                IntersectEdges(aLSplits, aBlockLENew, theMVBounds, theVertsToAvoid, aMENew, theMECheckExt, theEImages,
                               aDMEOr, aMELF, ref aCE);
            }
            else
            {
                aCE = aLSplits.First();
            }

            aMBlocksSp.Add(aCE, aBlockLENew);
        }

        // Perform filtering of the edges in two steps:
        // - Check each block separately using localized bounds
        //   taken only from the splits of faces of the current block;
        // - Intersect all splits together and filter the splits by all bounds.

        // FIRST STAGE - separate treatment of the blocks

        // Valid splits to be preserved on the first stage
        NCollection_Map<TopoDS_Shape> aMEVal = new(TopTools_ShapeMapHasher.Instance);

        // Blocks of valid edges on the first stage
        NCollection_List<TopoDS_Shape> aLValBlocks = new();

        int aNbB = aMBlocksSp.Extent();
        using ProgressScope aPSBSp = new(aPSOuter.Next(), null, aNbB);
        for (i = 1; i <= aNbB; ++i, aPSBSp.Next())
        {
            if (!aPSBSp.More())
            {
                return;
            }
            TopoDS_Shape aCE = aMBlocksSp.FindKey(i);
            NCollection_List<TopoDS_Shape> aBlockLENew = aMBlocksSp[i];

            // Get all participating faces to get the bounds
            NCollection_List<TopoDS_Shape> aLFaces0 = new();
            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aBlockLENew);
            for (; aItLE.More(); aItLE.Next())
            {
                if (!aMELF.Find(aItLE.Value(), out NCollection_List<TopoDS_Shape> pLF))
                {
                    continue;
                }
                NCollection_List<TopoDS_Shape>.Iterator aItLF = new(pLF);
                for (; aItLF.More(); aItLF.Next())
                {
                    AppendToList(aLFaces0, aItLF.Value());
                }
            }

            // Localized bounds of the splits of the offset faces
            // to filter the new splits of the current block
            TopoDS_Shape aFilterBounds0 = new();
            GetBounds(aLFaces0, aMEB, ref aFilterBounds0);

            // Filter the splits by bounds
            NCollection_Map<TopoDS_Shape> aMEInvLoc = new(TopTools_ShapeMapHasher.Instance);
            GetInvalidEdgesByBounds(aCE, aFilterBounds0, aMVOld, aMENew, aDMEOr, aMELF, theEImages, theMECheckExt, theMEInvOnArt,
                                    theVertsToAvoid, aMEInvLoc);

            // Keep only valid edges of the block
            TopoDS_Compound aCEVal = new();
            aBB.MakeCompound(ref aCEVal);

            bool bKept = false;

            TopExp_Explorer anExpE = new(aCE, TopAbs_EDGE);
            for (; anExpE.More(); anExpE.Next())
            {
                TopoDS_Shape aESp = anExpE.Current();
                if (!aMEInvLoc.Contains(aESp) && aMEVal.Add(aESp))
                {
                    aBB.Add(aCEVal, aESp);
                    bKept = true;
                }
            }

            if (bKept)
            {
                aLValBlocks.Append(aCEVal);
            }
        }

        // Filter the images of edges after the first filtering stage
        TopoDS_Shape aSplits1 = new();
        FilterSplits(aLE, aMEVal, false, theEImages, ref aSplits1);

        if (aLValBlocks.IsEmpty())
        {
            // update intersection edges
            UpdateNewIntersectionEdges(aLE, aMELF, theEImages, theEETrim);
            return;
        }

        aPSOuter.Next();
        if (!aPSOuter.More())
        {
            return;
        }

        // SECOND STAGE - Filter the remaining splits together

        // Add for intersection already removed new edges using them
        // as markers for other invalid edges
        aNbB = aMBlocksSp.Extent();
        for (i = 1; i <= aNbB; ++i)
        {
            TopoDS_Shape aCE = aMBlocksSp.FindKey(i);
            for (TopExp_Explorer anExp = new(aCE, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                TopoDS_Shape aEIm = anExp.Current();
                if (aMENew.Contains(aEIm) && !aMEVal.Contains(aEIm))
                {
                    aLValBlocks.Append(aEIm);
                }
            }
        }

        if (aLValBlocks.Extent() > 1)
        {
            // intersect the new splits among themselves to avoid self-intersections
            IntersectEdges(aLValBlocks, aLE, theMVBounds, theVertsToAvoid, aMENew, theMECheckExt, theEImages, aDMEOr, aMELF,
                           ref aSplits1);
        }
        else
        {
            aSplits1 = aLValBlocks.First();
        }

        aPSOuter.Next();
        if (!aPSOuter.More())
        {
            return;
        }

        // Get all faces to get the bounds from their splits
        NCollection_List<TopoDS_Shape> aLFaces = new();
        for (i = 1; i <= myOFImages.Extent(); ++i)
        {
            aLFaces.Append(myOFImages.FindKey(i));
        }

        // Bounds of the splits of the offset faces to filter the new splits
        TopoDS_Shape aFilterBounds = new();
        GetBounds(aLFaces, aMEB, ref aFilterBounds);

        // Filter the splits by intersection with bounds
        NCollection_Map<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
        GetInvalidEdgesByBounds(aSplits1, aFilterBounds, aMVOld, aMENew, aDMEOr, aMELF, theEImages, theMECheckExt, theMEInvOnArt,
                                theVertsToAvoid, aMEInv);

        // Filter the images of edges after the second filtering stage
        // and combine all valid edges into a single compound
        TopoDS_Shape aSplits = new();
        FilterSplits(aLE, aMEInv, true, theEImages, ref aSplits);

        aPSOuter.Next();
        if (!aPSOuter.More())
        {
            return;
        }

        // get bounds to update
        // we need to update the edges of all the affected faces
        NCollection_List<TopoDS_Shape> aLF = new();
        // prepare the vertices from new splits of edges
        NCollection_IndexedMap<TopoDS_Shape> aMVSp = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aSplits, TopAbs_VERTEX, aMVSp);
        //
        int aNbF = myOFImages.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Shape aF = myOFImages.FindKey(i);
            if (theFLE.Contains(aF))
            {
                aLF.Append(aF);
                continue;
            }
            //
            // check the splits of faces to have vertices from splits
            NCollection_List<TopoDS_Shape> aLFIm = myOFImages[i];
            NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(aLFIm);
            for (; aItLFIm.More(); aItLFIm.Next())
            {
                TopoDS_Shape aFIm = aItLFIm.Value();
                //
                TopExp_Explorer aExpV = new(aFIm, TopAbs_VERTEX);
                for (; aExpV.More(); aExpV.Next())
                {
                    TopoDS_Shape aV = aExpV.Current();
                    if (aMVSp.Contains(aV))
                    {
                        break;
                    }
                }
                //
                if (aExpV.More())
                {
                    break;
                }
            }
            //
            if (aItLFIm.More())
            {
                aLF.Append(aF);
            }
        }
        //
        // get bounds from splits of faces of aLF
        TopoDS_Shape aBounds = new();
        NCollection_List<TopoDS_Shape> aLAValid = new(), aLABounds = new();
        GetBoundsToUpdate(aLF, aMEB, aLABounds, aLAValid, ref aBounds);
        //
        // Intersect valid splits with bounds and update both
        BOPAlgo_Builder aGF = new();
        aGF.AddArgument(aBounds);
        aGF.AddArgument(aSplits);
        aGF.Perform(aPSOuter.Next(3));
        //
        // update splits
        UpdateImages(aLE, theEImages, aGF, myModifiedEdges);
        //
        // update new intersection edges
        UpdateNewIntersectionEdges(aLE, aMELF, theEImages, theEETrim);
        //
        // update bounds
        UpdateImages(aLAValid, myOEImages, aGF, myModifiedEdges);
        UpdateOrigins(aLABounds, myOEOrigins, aGF);
        UpdateOrigins(aLABounds, myEdgesOrigins!, aGF);
        UpdateIntersectedEdges(aLABounds, aGF);
        //
        // update the EdgesToAvoid with the splits
        NCollection_IndexedMap<TopoDS_Shape> aNewEdges = new(TopTools_ShapeMapHasher.Instance);
        if (aGF.Images().Find(aSplits, out NCollection_List<TopoDS_Shape> pSplitsIm))
        {
            NCollection_List<TopoDS_Shape>.Iterator aItSpIm = new(pSplitsIm);
            for (; aItSpIm.More(); aItSpIm.Next())
            {
                TopExp.MapShapes(aItSpIm.Value(), TopAbs_EDGE, aNewEdges);
            }
        }

        TopoDS_Compound anInsideEdges = new();
        new BRep_Builder().MakeCompound(ref anInsideEdges);
        for (int iE = 1; iE <= myInsideEdges.Extent(); ++iE)
        {
            new BRep_Builder().Add(anInsideEdges, myInsideEdges[iE]);
        }

        //
        // Rebuild the map of edges to avoid, using the intersection results
        NCollection_IndexedMap<TopoDS_Shape> aMEAvoid = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Compound aCEAvoid = new();
        new BRep_Builder().MakeCompound(ref aCEAvoid);
        // GF's data structure
        BOPDS_DS pDS = aGF.PDS()!;

        aNbE = myEdgesToAvoid.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Shape aE = myEdgesToAvoid[i];
            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(aGF, aE);

            // Only untouched and fully coinciding edges should be kept in the avoid map
            bool bKeep = aLEIm.IsEmpty();
            if (aLEIm.Extent() == 1 && aE.IsSame(aLEIm.First()))
            {
                NCollection_List<BOPDS_PaveBlock> aLPB = pDS.PaveBlocks(pDS.Index(aE));
                if (aLPB.Extent() == 1)
                {
                    BOPDS_PaveBlock aPB = aLPB.First();
                    BOPDS_CommonBlock? aCB = pDS.CommonBlock(aPB);
                    if (aCB is not null)
                    {
                        NCollection_List<BOPDS_PaveBlock> aLPBCB = aCB.PaveBlocks();
                        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPBCB);
                        for (; aItLPB.More(); aItLPB.Next())
                        {
                            if (pDS.PaveBlocks(aItLPB.Value().OriginalEdge()).Extent() > 1)
                            {
                                break;
                            }
                        }
                        bKeep = !aItLPB.More();
                    }
                }
            }

            if (bKeep)
            {
                // keep the original edge
                AddToContainer(aE, aMEAvoid);
                continue;
            }

            NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(aLEIm);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                TopoDS_Shape aEIm = aItLEIm.Value();
                if (!aNewEdges.Contains(aEIm))
                {
                    AddToContainer(aEIm, aCEAvoid);
                }
            }
        }

        bool isCut = false;
        if (aCEAvoid.NbChildren() > 0)
        {
            // Perform intersection with the small subset of the edges to make
            // it possible to use the inside edges for building new splits.
            BOPAlgo_BOP aBOP = new();
            aBOP.AddArgument(aCEAvoid);
            aBOP.AddTool(anInsideEdges);
            aBOP.SetOperation(BOPAlgo_Operation.BOPAlgo_CUT);
            aBOP.Perform();
            isCut = !aBOP.HasErrors();

            if (isCut)
            {
                for (TopoDS_Iterator itCE = new(aCEAvoid); itCE.More(); itCE.Next())
                {
                    if (!aBOP.IsDeleted(itCE.Value()))
                    {
                        aMEAvoid.Add(itCE.Value());
                    }
                }
            }
        }

        if (!isCut)
        {
            TopExp.MapShapes(aCEAvoid, TopAbs_EDGE, aMEAvoid);
        }

        myEdgesToAvoid = aMEAvoid;
    }

    //=================================================================================================

    //! Trims intersection edges
    private void TrimNewIntersectionEdges(NCollection_List<TopoDS_Shape> theLE,
                                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEETrim,
                                          NCollection_Map<TopoDS_Shape> theMVBounds, NCollection_Map<TopoDS_Shape> theMECheckExt,
                                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages,
                                          NCollection_Map<TopoDS_Shape> theMEB, NCollection_Map<TopoDS_Shape> theMVOld,
                                          NCollection_Map<TopoDS_Shape> theMENew,
                                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEOr,
                                          NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMELF)
    {
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(), aIt1 = new();
        aIt.Initialize(theLE);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            //
            bool bCheckExt = theMECheckExt.Remove(aE);
            //
            bool bOld = theEETrim.IsBound(aE);
            if (bOld)
            {
                NCollection_List<TopoDS_Shape> aLET = theEETrim.Find(aE);
                aIt1.Initialize(aLET);
                for (; aIt1.More(); aIt1.Next())
                {
                    TopoDS_Shape aET = aIt1.Value();
                    theMEB.Add(aET);
                    TopExp_Explorer aExpV = new(aET, TopAbs_VERTEX);
                    for (; aExpV.More(); aExpV.Next())
                    {
                        TopoDS_Shape aV = aExpV.Current();
                        theMVOld.Add(aV);
                    }
                }
            }
            //
            if (!theEImages.IsBound(aE))
            {
                continue;
            }
            //
            NCollection_List<TopoDS_Shape> aLEIm = theEImages.ChangeFind(aE);
            if (aLEIm.IsEmpty())
            {
                theEImages.UnBind(aE);
                continue;
            }
            //
            TopoDS_Shape aCEIm = new();
            NCollection_Map<TopoDS_Shape> aMEVBounds = new(TopTools_ShapeMapHasher.Instance);
            //
            if (aLEIm.Extent() > 1)
            {
                NCollection_IndexedMap<TopoDS_Shape> aMV = new(TopTools_ShapeMapHasher.Instance);
                // fuse these parts
                BOPAlgo_Builder aGFE = new();
                NCollection_List<TopoDS_Shape>.Iterator aItLEIm0 = new(aLEIm);
                for (; aItLEIm0.More(); aItLEIm0.Next())
                {
                    TopoDS_Shape aEIm = aItLEIm0.Value();
                    aGFE.AddArgument(aEIm);
                    TopExp.MapShapes(aEIm, TopAbs_VERTEX, aMV);
                }
                //
                // add two bounding vertices of this edge to the operation
                TopExp.Vertices(TopoDS.Edge(aE), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                //
                aGFE.AddArgument(aV1);
                aGFE.AddArgument(aV2);
                aMV.Add(aV1);
                aMV.Add(aV2);
                //
                aGFE.Perform();
                if (!aGFE.HasErrors())
                {
                    // get images of bounding vertices to remove splits containing them
                    // in case some of the bounding edges has been interfered
                    // during operation it is necessary to update their images as well
                    int iV, aNbV = aMV.Extent();
                    for (iV = 1; iV <= aNbV; ++iV)
                    {
                        TopoDS_Shape aV = aMV[iV];
                        if (theMVBounds.Contains(aV) || aV.IsSame(aV1) || aV.IsSame(aV2))
                        {
                            NCollection_List<TopoDS_Shape> aLVIm = ModifiedOf(aGFE, aV);
                            aMEVBounds.Add(aLVIm.IsEmpty() ? aV : aLVIm.First());
                        }
                    }
                    //
                    aCEIm = aGFE.InternalShape();
                }
            }
            else
            {
                aCEIm = aLEIm.First();
            }
            //
            aLEIm.Clear();
            //
            TopExp_Explorer aExp = new(aCEIm, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aEIm = aExp.Current();
                //
                // check the split not to contain bounding vertices
                TopoDS_Iterator aItV = new(aEIm);
                for (; aItV.More(); aItV.Next())
                {
                    TopoDS_Shape aV = aItV.Value();
                    if (aMEVBounds.Contains(aV) || theMVBounds.Contains(aV))
                    {
                        break;
                    }
                }
                //
                if (!aItV.More())
                {
                    aLEIm.Append(aEIm);
                    //
                    // OCCT's Bound replaces any list already bound.
                    NCollection_List<TopoDS_Shape> aLOr = new();
                    aLOr.Append(aE);
                    theDMEOr.Bind(aEIm, aLOr);
                }
            }
            //
            if (aLEIm.IsEmpty())
            {
                theEImages.UnBind(aE);
            }
            else
            {
                NCollection_List<TopoDS_Shape> aLFE = theMELF.Find(aE);
                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(aLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    TopoDS_Shape aEIm = aItLEIm.Value();
                    if (!theMELF.Find(aEIm, out NCollection_List<TopoDS_Shape> pLFEIm))
                    {
                        pLFEIm = new NCollection_List<TopoDS_Shape>();
                        theMELF.Bind(aEIm, pLFEIm);
                    }
                    NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLFE);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        AppendToList(pLFEIm, aItLF.Value());
                    }
                    //
                    if (bCheckExt)
                    {
                        theMECheckExt.Add(aEIm);
                    }
                    else if (!bOld)
                    {
                        theMENew.Add(aEIm);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IntersectEdges
    // purpose  : Intersecting the trimmed edges to avoid self-intersections
    //=======================================================================
    private void IntersectEdges(NCollection_List<TopoDS_Shape> theLA, NCollection_List<TopoDS_Shape> theLE,
                                NCollection_Map<TopoDS_Shape> theMVBounds, NCollection_Map<TopoDS_Shape> theVertsToAvoid,
                                NCollection_Map<TopoDS_Shape> theMENew, NCollection_Map<TopoDS_Shape> theMECheckExt,
                                NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages,
                                NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEOr,
                                NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMELF,
                                ref TopoDS_Shape theSplits)
    {
        BOPAlgo_Builder aGFA = new();
        aGFA.SetArguments(theLA);
        aGFA.Perform();
        if (aGFA.HasErrors())
        {
            // just copy input to the result
            TopoDS_Compound aSp0 = new();
            BRep_Builder aBB0 = new();
            aBB0.MakeCompound(ref aSp0);
            NCollection_List<TopoDS_Shape>.Iterator anIt = new(theLA);
            for (; anIt.More(); anIt.Next())
            {
                TopoDS_Shape aE = anIt.Value();
                aBB0.Add(aSp0, aE);
            }
            theSplits = aSp0;
            return;
        }
        //
        UpdateImages(theLE, theEImages, aGFA, myModifiedEdges);
        //
        // compound of valid splits
        theSplits = aGFA.InternalShape();
        //
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(), aIt1 = new();

        // prepare list of edges to update
        NCollection_List<TopoDS_Shape> aLEInput = new();
        for (aIt.Initialize(theLA); aIt.More(); aIt.Next())
        {
            TopExp_Explorer anExpE = new(aIt.Value(), TopAbs_EDGE);
            for (; anExpE.More(); anExpE.Next())
            {
                aLEInput.Append(anExpE.Current());
            }
        }

        // update new edges
        aIt.Initialize(aLEInput);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            if (!theMENew.Contains(aE))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(aGFA, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }

            theMENew.Remove(aE);
            aIt1.Initialize(aLEIm);
            for (; aIt1.More(); aIt1.Next())
            {
                theMENew.Add(aIt1.Value());
            }
        }
        //
        // update edges after intersection for extended checking
        aIt.Initialize(aLEInput);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            NCollection_List<TopoDS_Shape> aLEIm = ModifiedOf(aGFA, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }
            //
            if (theMECheckExt.Contains(aE))
            {
                aIt1.Initialize(aLEIm);
                for (; aIt1.More(); aIt1.Next())
                {
                    theMECheckExt.Add(aIt1.Value());
                }
                theMECheckExt.Remove(aE);
            }
            //
            NCollection_List<TopoDS_Shape> aLFE = theMELF.Find(aE);
            aIt1.Initialize(aLEIm);
            for (; aIt1.More(); aIt1.Next())
            {
                TopoDS_Shape aEIm = aIt1.Value();
                if (!theMELF.Find(aEIm, out NCollection_List<TopoDS_Shape> pLFEIm))
                {
                    pLFEIm = new NCollection_List<TopoDS_Shape>();
                    theMELF.Bind(aEIm, pLFEIm);
                }
                NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLFE);
                for (; aItLF.More(); aItLF.Next())
                {
                    AppendToList(pLFEIm, aItLF.Value());
                }
            }
        }
        //
        NCollection_Map<TopoDS_Shape> aMEInv = new(TopTools_ShapeMapHasher.Instance);
        GetInvalidEdges(theVertsToAvoid, theMVBounds, aGFA, aMEInv);
        if (aMEInv.Extent() != 0)
        {
            // update shape
            TopoDS_Compound aSp = new();
            BRep_Builder aBB = new();
            aBB.MakeCompound(ref aSp);
            TopExp_Explorer aExp = new(theSplits, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aE = aExp.Current();
                if (!aMEInv.Contains(aE))
                {
                    aBB.Add(aSp, aE);
                }
            }
            theSplits = aSp;
        }
        //
        // update origins
        UpdateOrigins(aLEInput, theDMEOr, aGFA);
    }

    //=======================================================================
    // function : GetBounds
    // purpose  : Getting edges from the splits of offset faces
    //=======================================================================
    private void GetBounds(NCollection_List<TopoDS_Shape> theLFaces, NCollection_Map<TopoDS_Shape> theMEB,
                           ref TopoDS_Shape theBounds)
    {
        BRep_Builder aBB = new();
        // Make compound of edges contained in the splits of faces
        TopoDS_Compound aBounds = new();
        aBB.MakeCompound(ref aBounds);
        // Fence map
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);

        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(theLFaces);
        for (; aItLF.More(); aItLF.Next())
        {
            if (!myOFImages.FindFromKey(aItLF.Value(), out NCollection_List<TopoDS_Shape> pLFIm))
            {
                continue;
            }
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(pLFIm);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aFIm = aIt.Value();
                //
                TopExp_Explorer aExpE = new(aFIm, TopAbs_EDGE);
                for (; aExpE.More(); aExpE.Next())
                {
                    TopoDS_Shape aEIm = aExpE.Current();
                    if (!theMEB.Contains(aEIm) && aMFence.Add(aEIm))
                    {
                        aBB.Add(aBounds, aEIm);
                    }
                }
            }
        }
        theBounds = aBounds;
    }

    //=======================================================================
    // function : GetBoundsToUpdate
    // purpose  : Get bounding edges that should be updated
    //=======================================================================
    private void GetBoundsToUpdate(NCollection_List<TopoDS_Shape> theLF, NCollection_Map<TopoDS_Shape> theMEB,
                                   NCollection_List<TopoDS_Shape> theLABounds, NCollection_List<TopoDS_Shape> theLAValid,
                                   ref TopoDS_Shape theBounds)
    {
        // get all edges
        TopoDS_Compound aBounds = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aBounds);
        //
        NCollection_Map<TopoDS_Shape> aMAValid = new(TopTools_ShapeMapHasher.Instance), aMFence = new(TopTools_ShapeMapHasher.Instance);
        //
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(theLF);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Shape aF = aItLF.Value();
            //
            NCollection_IndexedMap<TopoDS_Shape> aMDE = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape> aLFDes = myAsDes!.Descendant(aF);
            NCollection_List<TopoDS_Shape>.Iterator aItLFDes = new(aLFDes);
            for (; aItLFDes.More(); aItLFDes.Next())
            {
                TopoDS_Shape aED = aItLFDes.Value();
                if (!myOEImages.Find(aED, out NCollection_List<TopoDS_Shape> pLEDIm))
                {
                    aMDE.Add(aED);
                    continue;
                }
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLEDIm = new(pLEDIm);
                for (; aItLEDIm.More(); aItLEDIm.Next())
                {
                    TopoDS_Shape aEDIm = aItLEDIm.Value();
                    aMDE.Add(aEDIm);
                }
            }
            //
            int j, aNbE = aMDE.Extent();
            for (j = 1; j <= aNbE; ++j)
            {
                TopoDS_Edge aEIm = TopoDS.Edge(aMDE[j]);
                //
                if (!theMEB.Contains(aEIm) && aMFence.Add(aEIm))
                {
                    aBB.Add(aBounds, aEIm);
                    theLABounds.Append(aEIm);
                }
                //
                if (myOEOrigins.Find(aEIm, out NCollection_List<TopoDS_Shape> pLO))
                {
                    NCollection_List<TopoDS_Shape>.Iterator aItLO = new(pLO);
                    for (; aItLO.More(); aItLO.Next())
                    {
                        TopoDS_Shape aEO = aItLO.Value();
                        //
                        if (aMAValid.Add(aEO))
                        {
                            theLAValid.Append(aEO);
                        }
                    }
                }
                else
                {
                    if (aMAValid.Add(aEIm))
                    {
                        theLAValid.Append(aEIm);
                    }
                }
            }
        }
        theBounds = aBounds;
    }

    //=======================================================================
    // function : GetInvalidEdgesByBounds
    // purpose  : Filter new splits by intersection with bounds
    //=======================================================================
    private void GetInvalidEdgesByBounds(in TopoDS_Shape theSplits, in TopoDS_Shape theBounds,
                                         NCollection_Map<TopoDS_Shape> theMVOld, NCollection_Map<TopoDS_Shape> theMENew,
                                         NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theDMEOr,
                                         NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMELF,
                                         NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages,
                                         NCollection_Map<TopoDS_Shape> theMECheckExt, NCollection_Map<TopoDS_Shape> theMEInvOnArt,
                                         NCollection_Map<TopoDS_Shape> theVertsToAvoid, NCollection_Map<TopoDS_Shape> theMEInv)
    {
        // map splits to check the vertices of edges
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theSplits, TopAbs_VERTEX, TopAbs_EDGE, aDMVE);
        //
        BOPAlgo_Section aSec = new();
        aSec.AddArgument(theSplits);
        aSec.AddArgument(theBounds);
        //
        aSec.Perform();
        //
        // invalid vertices
        NCollection_IndexedMap<TopoDS_Shape> aMVInv = new(TopTools_ShapeMapHasher.Instance);
        // vertices to check additionally by classification relatively to solid
        NCollection_Map<TopoDS_Shape> aMVCheckAdd = new(TopTools_ShapeMapHasher.Instance);
        // collect parts for removal
        BOPDS_DS pDS = aSec.PDS()!;
        //
        // check edge/edge intersections
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = pDS.InterfEE();
        int i, aNb = aEEs.Length();
        for (i = 0; i < aNb; ++i)
        {
            BOPDS_InterfEE aEE = aEEs[i];
            //
            TopoDS_Shape aE1 = pDS.Shape(aEE.Index1());
            TopoDS_Shape aE2 = pDS.Shape(aEE.Index2());
            //
            if (!aEE.HasIndexNew())
            {
                if (theMECheckExt.Contains(aE1) && (aEE.CommonPart().Type() == TopAbs_EDGE))
                {
                    theMEInv.Add(aE1);
                }
                continue;
            }
            //
            if (myInvalidEdges.Contains(aE2))
            {
                theMEInv.Add(aE1);
            }
            //
            if (theMEInvOnArt.Contains(aE2))
            {
                // avoid checking of the vertices of the split edge intersected by
                // the invalid edge from artificial face
                TopExp.Vertices(TopoDS.Edge(aE2), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                if (aDMVE.Contains(aV1) && aDMVE.Contains(aV2))
                {
                    continue;
                }
            }
            //
            // add vertices of all images of the edge from splits for checking
            NCollection_List<TopoDS_Shape> aLEOr = theDMEOr.Find(aE1);
            NCollection_List<TopoDS_Shape>.Iterator aItLEOr = new(aLEOr);
            for (; aItLEOr.More(); aItLEOr.Next())
            {
                TopoDS_Shape aEOr = aItLEOr.Value();
                //
                if (!theEImages.Find(aEOr, out NCollection_List<TopoDS_Shape> pLEIm))
                {
                    continue;
                }
                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    TopoDS_Shape aEIm = aItLEIm.Value();
                    //
                    TopoDS_Iterator aItV = new(aEIm);
                    for (; aItV.More(); aItV.Next())
                    {
                        TopoDS_Shape aV = aItV.Value();
                        if (!theMVOld.Contains(aV))
                        {
                            aMVInv.Add(aV);
                            aMVCheckAdd.Add(aV);
                        }
                    }
                }
            }
        }
        //
        // to avoid unnecessary filling of parts due to extra trim of the edges
        // process Edge/Edge interferences of type EDGE, i.e. common blocks and check
        // not the bounding vertices of the edges, but check the edge itself
        // to be lying on some face
        //
        // all common blocks are contained in the result of SECTION operation
        // between sets of edges
        TopoDS_Shape aSecR = aSec.InternalShape();
        //
        NCollection_IndexedMap<TopoDS_Shape> aMSSec = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aSecR, aMSSec);
        //
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anIm = aSec.Images();
        for (TopExp_Explorer aExp = new(theSplits, TopAbs_EDGE); aExp.More(); aExp.Next())
        {
            TopoDS_Shape aE = aExp.Current();
            if (aSec.IsDeleted(aE))
            {
                // no common blocks for this edge
                continue;
            }
            //
            if (!anIm.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
            {
                // no splits, i.e. completely coincides with some edge from boundary
                continue;
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                TopoDS_Shape aEIm = aItLEIm.Value();
                if (!aMSSec.Contains(aEIm))
                {
                    // the edge included in section only partially.
                    // the part not included in section may be excessive
                    //
                    // check vertices of this edge - if one of them is new
                    // the edge might be removed
                    TopExp.Vertices(TopoDS.Edge(aEIm), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                    if (!theMVOld.Contains(aV1) || !theMVOld.Contains(aV2))
                    {
                        // add this edge for checking by making new vertex in the middle of the edge
                        TopoDS_Vertex aV = new();
                        Geom_Curve aC = BRep_Tool.Curve(TopoDS.Edge(aEIm), out double f, out double l)!;
                        new BRep_Builder().MakeVertex(ref aV, aC.Value((f + l) * 0.5), global::Kerf.Engine.Foundation.Precision.Confusion());
                        // and adding this vertex for checking
                        aDMVE.ChangeFromIndex(aDMVE.Add(aV, new NCollection_List<TopoDS_Shape>())).Append(aE);
                        aMVInv.Add(aV);
                        break;
                    }
                }
            }
        }
        //
        // Add for check also the edges created from common between splits
        // of offset faces edges not connected to any invalidity.
        // These edges may also accidentally fill some part.
        // OCCT.NET: OCCT iterates the map by its edges' addresses; the port in insertion order (decision 1032).
        foreach (TopoDS_Shape aE in theMECheckExt)
        {
            //
            // make new vertex in the middle of the edge
            TopoDS_Vertex aV = new();
            Geom_Curve aC = BRep_Tool.Curve(TopoDS.Edge(aE), out double f, out double l)!;
            new BRep_Builder().MakeVertex(ref aV, aC.Value((f + l) * 0.5), global::Kerf.Engine.Foundation.Precision.Confusion());
            // add this vertex for checking
            aDMVE.ChangeFromIndex(aDMVE.Add(aV, new NCollection_List<TopoDS_Shape>())).Append(aE);
            aMVInv.Add(aV);
        }
        //
        // add for check also the vertices connected only to new or old edges
        aNb = aDMVE.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aV = aDMVE.FindKey(i);
            if (theMVOld.Contains(aV))
            {
                continue;
            }
            //
            bool bNew = false, bOld = false;
            NCollection_List<TopoDS_Shape> aLEx = aDMVE[i];
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(aLEx);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aE = aIt.Value();
                if (theMECheckExt.Contains(aE))
                {
                    continue;
                }
                //
                if (theMENew.Contains(aE))
                {
                    bNew = true;
                }
                else
                {
                    bOld = true;
                }
                //
                if (bNew && bOld)
                {
                    break;
                }
            }
            //
            if (!bNew || !bOld)
            {
                aMVInv.Add(aV);
                aMVCheckAdd.Remove(aV);
            }
        }
        //
        // perform the checking of the vertices
        int iv, aNbIV = aMVInv.Extent();
        for (iv = 1; iv <= aNbIV; ++iv)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aMVInv[iv]);
            if (theMVOld.Contains(aV))
            {
                continue;
            }
            //
            if (!aDMVE.FindFromKey(aV, out NCollection_List<TopoDS_Shape> pLEInv))
            {
                continue;
            }
            // find faces by the edges to check the vertex
            NCollection_IndexedMap<TopoDS_Shape> aMF = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(pLEInv);
            for (; aItLE.More(); aItLE.Next())
            {
                TopoDS_Shape aE = aItLE.Value();
                NCollection_List<TopoDS_Shape> aLF = theMELF.Find(aE);
                NCollection_List<TopoDS_Shape>.Iterator aItLF0 = new(aLF);
                for (; aItLF0.More(); aItLF0.Next())
                {
                    aMF.Add(aItLF0.Value());
                }
            }
            //
            // check the vertex to belong to some split of the faces
            bool bInvalid = true;
            //
            int aNbF = aMF.Extent();
            for (i = 1; i <= aNbF && bInvalid; ++i)
            {
                TopoDS_Face aF = TopoDS.Face(aMF[i]);
                NCollection_List<TopoDS_Shape> aLFIm = myOFImages.FindFromKey(aF);
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLFIm);
                for (; aItLF.More() && bInvalid; aItLF.Next())
                {
                    TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
                    TopExp_Explorer aExp = new(aFIm, TopAbs_VERTEX);
                    for (; aExp.More() && bInvalid; aExp.Next())
                    {
                        TopoDS_Shape aVF = aExp.Current();
                        bInvalid = !aVF.IsSame(aV);
                    }
                }
                //
                if (bInvalid)
                {
                    double U = 0.0, V = 0.0, aTol = 0.0;
                    int iStatus = myContext.ComputeVF(aV, aF, ref U, ref V, ref aTol);
                    if (iStatus == 0)
                    {
                        // classify the point relatively faces
                        Point2D aP2d = new(U, V);
                        aItLF.Initialize(aLFIm);
                        for (; aItLF.More() && bInvalid; aItLF.Next())
                        {
                            TopoDS_Face aFIm = TopoDS.Face(aItLF.Value());
                            bInvalid = !myContext.IsPointInOnFace(aFIm, aP2d);
                        }
                    }
                }
            }
            //
            if (bInvalid && aMVCheckAdd.Contains(aV))
            {
                // the vertex is invalid for all faces
                // check the same vertex for the solids
                Point3D aP = BRep_Tool.Pnt(aV);
                double aTolV = BRep_Tool.Tolerance(aV);
                //
                TopExp_Explorer aExpS = new(mySolids, TopAbs_SOLID);
                for (; aExpS.More() && bInvalid; aExpS.Next())
                {
                    TopoDS_Solid aSol = TopoDS.Solid(aExpS.Current());
                    BRepClass3d_SolidClassifier aSC = myContext.SolidClassifier(aSol);
                    aSC.Perform(aP, aTolV);
                    bInvalid = (aSC.State() == TopAbs_OUT);
                }
            }
            //
            if (bInvalid)
            {
                theVertsToAvoid.Add(aV);
                aItLE.Initialize(pLEInv);
                for (; aItLE.More(); aItLE.Next())
                {
                    theMEInv.Add(aItLE.Value());
                }
            }
        }
    }

    //=======================================================================
    // function : FilterSplits
    // purpose  : Filter the images of edges from the invalid edges
    //=======================================================================
    private void FilterSplits(NCollection_List<TopoDS_Shape> theLE, NCollection_Map<TopoDS_Shape> theMEFilter, bool theIsInv,
                              NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages,
                              ref TopoDS_Shape theSplits)
    {
        TopoDS_Compound aSplits = new();
        new BRep_Builder().MakeCompound(ref aSplits);
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);

        NCollection_List<TopoDS_Shape>.Iterator aItLE = new(theLE);
        for (; aItLE.More(); aItLE.Next())
        {
            TopoDS_Shape aE = aItLE.Value();
            if (!theEImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
            for (; aItLEIm.More();)
            {
                TopoDS_Shape aEIm = aItLEIm.Value();
                if (theMEFilter.Contains(aEIm) == theIsInv)
                {
                    pLEIm.Remove(aItLEIm);
                    continue;
                }

                if (aMFence.Add(aEIm))
                {
                    new BRep_Builder().Add(aSplits, aEIm);
                }
                aItLEIm.Next();
            }

            if (pLEIm.IsEmpty())
            {
                theEImages.UnBind(aE);
            }
        }
        theSplits = aSplits;
    }

    //=======================================================================
    // function : UpdateNewIntersectionEdges
    // purpose  : Updating the maps of images and origins of the offset edges
    //=======================================================================
    private void UpdateNewIntersectionEdges(NCollection_List<TopoDS_Shape> theLE,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMELF,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEImages,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEETrim)
    {
        NCollection_List<TopoDS_Shape> aLEImEmpty = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(), aIt1 = new();
        // update global maps of images and origins with new splits
        aIt.Initialize(theLE);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            //
            if (!theEImages.IsBound(aE))
            {
                if (!theEETrim.Find(aE, out NCollection_List<TopoDS_Shape> pLET))
                {
                    continue;
                }
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLET = new(pLET);
                for (; aItLET.More();)
                {
                    TopoDS_Shape aET = aItLET.Value();
                    if (!myInvalidEdges.Contains(aET) && !myInvertedEdges.Contains(aET))
                    {
                        pLET.Remove(aItLET);
                    }
                    else
                    {
                        aItLET.Next();
                    }
                }
                //
                if (pLET.IsEmpty())
                {
                    continue;
                }
            }
            // new images
            NCollection_List<TopoDS_Shape> aLENew = theEImages.IsBound(aE) ? theEImages.Find(aE) : aLEImEmpty;
            //
            // save connection to untrimmed edge for the next steps
            aIt1.Initialize(aLENew);
            for (; aIt1.More(); aIt1.Next())
            {
                TopoDS_Shape aET = aIt1.Value();
                myETrimEInf!.Bind(aET, aE);
                myModifiedEdges.Add(aET);
            }
            //
            // check if it is existing edge
            if (!theEETrim.IsBound(aE))
            {
                NCollection_List<TopoDS_Shape> aLF = theMELF.Find(aE);
                // the edge is new
                // add this edge to AsDes
                aIt1.Initialize(aLF);
                for (; aIt1.More(); aIt1.Next())
                {
                    TopoDS_Shape aF = aIt1.Value();
                    myAsDes!.Add(aF, aE);
                }
                //
                // add aE to the images
                myOEImages.Bind(aE, new NCollection_List<TopoDS_Shape>(aLENew));
                myModifiedEdges.Add(aE);
                //
                // add to origins
                NCollection_List<TopoDS_Shape>.Iterator aItNew = new(aLENew);
                for (; aItNew.More(); aItNew.Next())
                {
                    TopoDS_Shape aENew = aItNew.Value();
                    if (myOEOrigins.IsBound(aENew))
                    {
                        NCollection_List<TopoDS_Shape> aEOrigins = myOEOrigins.ChangeFind(aENew);
                        AppendToList(aEOrigins, aE);
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape> aEOrigins = new();
                        aEOrigins.Append(aE);
                        myOEOrigins.Bind(aENew, aEOrigins);
                    }
                }
                //
                // update connection to initial origins
                if (myEdgesOrigins!.IsBound(aE))
                {
                    NCollection_List<TopoDS_Shape> aLEOrInit = myEdgesOrigins.Find(aE);
                    aIt1.Initialize(aLENew);
                    for (; aIt1.More(); aIt1.Next())
                    {
                        TopoDS_Shape aENew = aIt1.Value();
                        if (myEdgesOrigins.IsBound(aENew))
                        {
                            NCollection_List<TopoDS_Shape> aLENewOr = myEdgesOrigins.ChangeFind(aENew);
                            NCollection_List<TopoDS_Shape>.Iterator aItOrInit = new(aLEOrInit);
                            for (; aItOrInit.More(); aItOrInit.Next())
                            {
                                TopoDS_Shape aEOr = aItOrInit.Value();
                                AppendToList(aLENewOr, aEOr);
                            }
                        }
                        else
                        {
                            myEdgesOrigins.Bind(aENew, new NCollection_List<TopoDS_Shape>(aLEOrInit));
                        }
                    }
                }
                //
                continue;
            }
            //
            // old images
            NCollection_List<TopoDS_Shape> aLEOld = theEETrim.Find(aE);
            //
            // list of initial origins
            NCollection_List<TopoDS_Shape> anInitOrigins = new();
            //
            // it is necessary to replace the old edges with new ones
            aIt1.Initialize(aLEOld);
            for (; aIt1.More(); aIt1.Next())
            {
                TopoDS_Shape aEOld = aIt1.Value();
                //
                if (myOEOrigins.IsBound(aEOld))
                {
                    // get its origins
                    NCollection_List<TopoDS_Shape> aEOrigins = myOEOrigins.Find(aEOld);
                    //
                    NCollection_List<TopoDS_Shape>.Iterator aItOr = new(aEOrigins);
                    for (; aItOr.More(); aItOr.Next())
                    {
                        TopoDS_Shape aEOr = aItOr.Value();
                        //
                        myModifiedEdges.Add(aEOr);
                        //
                        NCollection_List<TopoDS_Shape> aEImages = myOEImages.ChangeFind(aEOr);
                        //
                        // remove old edge from images
                        NCollection_List<TopoDS_Shape>.Iterator aItIm = new(aEImages);
                        for (; aItIm.More();)
                        {
                            TopoDS_Shape aEIm = aItIm.Value();
                            if (aEIm.IsSame(aEOld))
                            {
                                aEImages.Remove(aItIm);
                            }
                            else
                            {
                                aItIm.Next();
                            }
                        }
                        //
                        // add new images
                        NCollection_List<TopoDS_Shape>.Iterator aItNew = new(aLENew);
                        for (; aItNew.More(); aItNew.Next())
                        {
                            TopoDS_Shape aENew = aItNew.Value();
                            AppendToList(aEImages, aENew);
                            if (myOEOrigins.IsBound(aENew))
                            {
                                NCollection_List<TopoDS_Shape> aENewOrigins = myOEOrigins.ChangeFind(aENew);
                                AppendToList(aENewOrigins, aEOr);
                            }
                            else
                            {
                                NCollection_List<TopoDS_Shape> aENewOrigins = new();
                                aENewOrigins.Append(aEOr);
                                myOEOrigins.Bind(aENew, aENewOrigins);
                            }
                        }
                    }
                }
                else
                {
                    // add to images
                    myOEImages.Bind(aEOld, new NCollection_List<TopoDS_Shape>(aLENew));
                    //
                    myModifiedEdges.Add(aEOld);
                    //
                    // add to origins
                    NCollection_List<TopoDS_Shape>.Iterator aItNew = new(aLENew);
                    for (; aItNew.More(); aItNew.Next())
                    {
                        TopoDS_Shape aENew = aItNew.Value();
                        if (myOEOrigins.IsBound(aENew))
                        {
                            NCollection_List<TopoDS_Shape> aEOrigins = myOEOrigins.ChangeFind(aENew);
                            AppendToList(aEOrigins, aEOld);
                        }
                        else
                        {
                            NCollection_List<TopoDS_Shape> aEOrigins = new();
                            aEOrigins.Append(aEOld);
                            myOEOrigins.Bind(aENew, aEOrigins);
                        }
                    }
                }
                //
                // update connection to initial shape
                if (myEdgesOrigins!.IsBound(aEOld))
                {
                    NCollection_List<TopoDS_Shape> aLEOrInit = myEdgesOrigins.Find(aEOld);
                    NCollection_List<TopoDS_Shape>.Iterator aItEOrInit = new(aLEOrInit);
                    for (; aItEOrInit.More(); aItEOrInit.Next())
                    {
                        TopoDS_Shape aEOrInit = aItEOrInit.Value();
                        AppendToList(anInitOrigins, aEOrInit);
                    }
                }
            }
            //
            if (anInitOrigins.Extent() != 0)
            {
                NCollection_List<TopoDS_Shape>.Iterator aItNew = new(aLENew);
                for (; aItNew.More(); aItNew.Next())
                {
                    TopoDS_Shape aENew = aItNew.Value();
                    if (myEdgesOrigins!.IsBound(aENew))
                    {
                        NCollection_List<TopoDS_Shape> aLENewOr = myEdgesOrigins.ChangeFind(aENew);
                        NCollection_List<TopoDS_Shape>.Iterator aItOrInit = new(anInitOrigins);
                        for (; aItOrInit.More(); aItOrInit.Next())
                        {
                            TopoDS_Shape aEOr = aItOrInit.Value();
                            AppendToList(aLENewOr, aEOr);
                        }
                    }
                    else
                    {
                        myEdgesOrigins.Bind(aENew, new NCollection_List<TopoDS_Shape>(anInitOrigins));
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FillGaps
    // purpose  : Fill possible gaps (holes) in the splits of the offset faces
    //=======================================================================
    private void FillGaps(ProgressRange theRange)
    {
        int aNbF = myOFImages.Extent();
        if (aNbF == 0)
        {
            return;
        }

        using ProgressScope aPS = new(theRange, "Filling gaps", 2 * aNbF);

        // Check the splits of offset faces on the free edges and fill the gaps (holes)
        // in created splits, otherwise the closed volume will not be possible to create.

        // Map the splits of faces to find free edges
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEFMap = new(TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= aNbF; ++i, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }

            NCollection_List<TopoDS_Shape>.Iterator itLF = new(myOFImages[i]);
            for (; itLF.More(); itLF.Next())
            {
                TopExp.MapShapesAndAncestors(itLF.Value(), TopAbs_EDGE, TopAbs_FACE, anEFMap);
            }
        }

        // Analyze images of each offset face on the presence of free edges
        // and try to fill the holes
        for (int i = 1; i <= aNbF; ++i, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }

            NCollection_List<TopoDS_Shape> aLFImages = myOFImages[i];
            if (aLFImages.IsEmpty())
            {
                continue;
            }

            // Collect all edges from the splits
            TopoDS_Compound anEdges = new();
            new BRep_Builder().MakeCompound(ref anEdges);

            // Collect all free edges into a map with reverted orientation
            // (OCCT's default hasher: the orientation counts).
            NCollection_Map<TopoDS_Shape> aFreeEdgesMap = new(EqualityComparer<TopoDS_Shape>.Default);
            NCollection_List<TopoDS_Shape>.Iterator itLF = new(aLFImages);
            for (; itLF.More(); itLF.Next())
            {
                TopoDS_Shape aFIm = itLF.Value();
                TopExp_Explorer anExpE = new(aFIm, TopAbs_EDGE);
                for (; anExpE.More(); anExpE.Next())
                {
                    TopoDS_Shape aE = anExpE.Current();
                    if (aE.Orientation() != TopAbs_FORWARD && aE.Orientation() != TopAbs_REVERSED)
                    {
                        // Skip internals
                        continue;
                    }

                    NCollection_List<TopoDS_Shape> aLF = anEFMap.FindFromKey(aE);
                    if (aLF.Extent() == 1)
                    {
                        aFreeEdgesMap.Add(aE.Reversed());
                    }

                    new BRep_Builder().Add(anEdges, aE);
                }
            }

            if (aFreeEdgesMap.IsEmpty())
            {
                // No free edges
                continue;
            }

            // Free edges are found - fill the gaps by creating new splits
            // of the face using these free edges
            TopoDS_Shape aF = myOFImages.FindKey(i);

            // Build new splits using all kept edges and among new splits
            // find those containing free edges
            NCollection_List<TopoDS_Shape> aLFNew = new();
            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDummy = new(TopTools_ShapeMapHasher.Instance);

            BuildSplitsOfFace(TopoDS.Face(aF), anEdges, aDummy, aLFNew);

            // Find faces filling holes
            itLF.Initialize(aLFNew);
            for (; itLF.More(); itLF.Next())
            {
                TopoDS_Shape aFNew = itLF.Value();
                TopExp_Explorer anExpE = new(aFNew, TopAbs_EDGE);
                for (; anExpE.More(); anExpE.Next())
                {
                    TopoDS_Shape aE = anExpE.Current();
                    if (aFreeEdgesMap.Contains(aE))
                    {
                        // Add face to splits
                        aLFImages.Append(aFNew);
                        break;
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : FillHistory
    // purpose  : Saving obtained results in history tools
    //=======================================================================
    private void FillHistory()
    {
        int aNbF = myOFImages.Extent();
        if (aNbF == 0)
        {
            return;
        }

        // OFFSET_DEBUG: the compound of the preliminary result is left out.

        // Map of kept edges
        NCollection_IndexedMap<TopoDS_Shape> anEdgesMap = new(TopTools_ShapeMapHasher.Instance);

        // Fill history for faces
        for (int i = 1; i <= aNbF; ++i)
        {
            NCollection_List<TopoDS_Shape> aLFImages = myOFImages[i];
            if (aLFImages.IsEmpty())
            {
                continue;
            }

            // Add the splits to history map
            TopoDS_Shape aF = myOFImages.FindKey(i);
            if (myImage.HasImage(aF))
            {
                myImage.Add(aF, aLFImages);
            }
            else
            {
                myImage.Bind(aF, aLFImages);
            }

            // Collect edges from splits
            NCollection_List<TopoDS_Shape>.Iterator itLF = new(aLFImages);
            for (; itLF.More(); itLF.Next())
            {
                TopoDS_Shape aFIm = itLF.Value();
                TopExp.MapShapes(aFIm, TopAbs_EDGE, anEdgesMap);
            }
        }

        // Fill history for edges (iteration by the map is safe because the
        // order is not important here)
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aItEIm in myOEImages)
        {
            TopoDS_Shape aE = aItEIm.Key;
            NCollection_List<TopoDS_Shape> aLEIm = aItEIm.Value;

            bool bHasImage = myImage.HasImage(aE);
            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLEIm);
            for (; aItLE.More(); aItLE.Next())
            {
                TopoDS_Shape aEIm = aItLE.Value();
                if (anEdgesMap.Contains(aEIm))
                {
                    if (bHasImage)
                    {
                        myImage.Add(aE, aEIm);
                    }
                    else
                    {
                        myImage.Bind(aE, aEIm);
                        bHasImage = true;
                    }
                }
            }
        }
    }
}

public sealed partial class BRepOffset_MakeOffset
{
    //=======================================================================
    // function : BuildSplitsOfTrimmedFaces
    // purpose  : Building splits of already trimmed faces
    //=======================================================================

    /// <summary>
    /// Builds the splits of the already trimmed offset faces, for the Complete mode with the Intersection join.
    /// </summary>
    private void BuildSplitsOfTrimmedFaces(NCollection_List<TopoDS_Shape> theLF, BRepAlgo_AsDes theAsDes,
                                           BRepAlgo_Image theImage, ProgressRange theRange)
    {
        BRepOffset_BuildOffsetFaces aBFTool = new(theImage);
        aBFTool.SetFaces(theLF);
        aBFTool.SetAsDesInfo(theAsDes);
        aBFTool.BuildSplitsOfTrimmedFaces(theRange);
    }

    //=======================================================================
    // function : BuildSplitsOfExtendedFaces
    // purpose  : Building splits of not-trimmed offset faces.
    //           For the cases in which invalidity will be found,
    //           these invalidities will be rebuilt.
    //=======================================================================

    /// <summary>
    /// Builds the splits of the extended (not trimmed) offset faces, for the Complete mode with the Intersection join
    /// (an alternative to BRepOffset_MakeLoops::Build, limited to planar cases); invalidities found are rebuilt.
    /// </summary>
    private void BuildSplitsOfExtendedFaces(NCollection_List<TopoDS_Shape> theLF, BRepOffset_Analyse theAnalyse,
                                            BRepAlgo_AsDes theAsDes,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgesOrigins,
                                            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theFacesOrigins,
                                            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theETrimEInf,
                                            BRepAlgo_Image theImage, ProgressRange theRange)
    {
        BRepOffset_BuildOffsetFaces aBFTool = new(theImage);
        aBFTool.SetFaces(theLF);
        aBFTool.SetAsDesInfo(theAsDes);
        aBFTool.SetAnalysis(theAnalyse);
        aBFTool.SetEdgesOrigins(theEdgesOrigins);
        aBFTool.SetFacesOrigins(theFacesOrigins);
        aBFTool.SetInfEdges(theETrimEInf);
        aBFTool.BuildSplitsOfExtendedFaces(theRange);
    }
}
