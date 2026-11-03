// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// This is the implementation of the extension of the 3D offset algorithm
// to work in mode Complete and Join Type Intersection.
// Currently only the Planar cases are supported.

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private static bool AddToContainer(in Shape theS, object theContainer)
    {
        switch (theContainer)
        {
            case NCollection_List<Shape> aList:
                aList.Append(theS);
                return true;
            case NCollection_Map<Shape> aMap:
                return aMap.Add(theS);
            case NCollection_IndexedMap<Shape> anIMap:
            {
                int aNb = anIMap.Extent();
                int anInd = anIMap.Add(theS);
                return anInd > aNb;
            }
            case Shape aSOut:
                new BrepBuilder().Add(aSOut, theS);
                return true;
            case Compound aCOut:
                new BrepBuilder().Add(aCOut, theS);
                return true;
            default:
                throw new ProgramErrorException("BRepOffset_BuildOffsetFaces::AddToContainer");
        }
    }

    private static void AddToContainer(in Shape theKey, in Shape theValue,
                                       NCollection_DataMap<Shape, NCollection_List<Shape>> theMap)
    {
        if (theMap.Find(theKey, out NCollection_List<Shape> pList))
        {
            pList.Append(theValue);
        }
        else
        {
            NCollection_List<Shape> aList = new();
            aList.Append(theValue);
            theMap.Bind(theKey, aList);
        }
    }

    //=======================================================================
    // function : TakeModified
    // purpose  : Check if the shape has images in the given images map.
    //           Puts in the output map either the images or the shape itself.
    //=======================================================================
    private static bool TakeModified(in Shape theS,
                                     NCollection_DataMap<Shape, NCollection_List<Shape>> theImages,
                                     object theContainer, object? theMFence)
    {
        if (theImages.Find(theS, out NCollection_List<Shape> pLSIm))
        {
            NCollection_List<Shape>.Iterator itLSIm = new(pLSIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                Shape aSIm = itLSIm.Value();
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

    private static bool TakeModified(in Shape theS,
                                     NCollection_DataMap<Shape, NCollection_List<Shape>> theImages,
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

    private static bool hasData(NCollection_List<Shape>? theData) => theData is not null && !theData.IsEmpty();

    //=======================================================================
    // function : AppendToList
    // purpose  : Add to a list only unique elements
    //=======================================================================
    private static void AppendToList(NCollection_List<Shape> theList, in Shape theShape)
    {
        for (NCollection_List<Shape>.Iterator it = new(theList); it.More(); it.Next())
        {
            Shape aS = it.Value();
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
    private static bool ProcessMicroEdge(in Edge theEdge, IntTools_Context theCtx)
    {
        Exploration.Vertices(theEdge, out Vertex aV1, out Vertex aV2);
        if (aV1.IsNull || aV2.IsNull)
        {
            return false;
        }

        bool bMicro = BOPTools_AlgoTools.IsMicroEdge(theEdge, theCtx);
        if (bMicro && new EdgeCurveAdaptor(theEdge).GetType() == Line)
        {
            double aLen = Tool.Pnt(aV1).Distance(Tool.Pnt(aV2));
            new BrepBuilder().UpdateVertex(aV1, aLen / 2.0);
            new BrepBuilder().UpdateVertex(aV2, aLen / 2.0);
        }

        return bMicro;
    }

    // OCCT's BOPAlgo_Builder::Modified (a const reference to the history's list), as a list of its own.
    private static NCollection_List<Shape> ModifiedOf(BOPAlgo_BuilderShape theGF, in Shape theS) =>
        new(theGF.Modified(theS));

    //=================================================================================================

    private static void UpdateOrigins(NCollection_List<Shape> theLA,
                                      NCollection_DataMap<Shape, NCollection_List<Shape>> theOrigins,
                                      BOPAlgo_Builder theGF)
    {
        for (NCollection_List<Shape>.Iterator aItA = new(theLA); aItA.More(); aItA.Next())
        {
            Shape aS = aItA.Value();

            NCollection_List<Shape> aLSIm = ModifiedOf(theGF, aS);
            if (aLSIm.IsEmpty())
            {
                continue;
            }

            NCollection_List<Shape> aLSEmpt = new();
            if (!theOrigins.Find(aS, out NCollection_List<Shape> pLS))
            {
                pLS = aLSEmpt;
                pLS.Append(aS);
            }

            for (NCollection_List<Shape>.Iterator aIt = new(aLSIm); aIt.More(); aIt.Next())
            {
                Shape aSIm = aIt.Value();
                if (theOrigins.Find(aSIm, out NCollection_List<Shape> pLSOr))
                {
                    // merge two lists
                    for (NCollection_List<Shape>.Iterator aIt1 = new(pLS); aIt1.More(); aIt1.Next())
                    {
                        AppendToList(pLSOr, aIt1.Value());
                    }
                }
                else
                {
                    theOrigins.Bind(aSIm, new NCollection_List<Shape>(pLS));
                }
            }
        }
    }

    //=================================================================================================

    private static void UpdateImages(NCollection_List<Shape> theLA,
                                     NCollection_DataMap<Shape, NCollection_List<Shape>> theImages,
                                     BOPAlgo_Builder theGF, NCollection_Map<Shape> theModified)
    {
        for (NCollection_List<Shape>.Iterator aIt = new(theLA); aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();

            if (!theImages.Find(aS, out NCollection_List<Shape> pLSIm))
            {
                NCollection_List<Shape> aLSIm = ModifiedOf(theGF, aS);
                if (aLSIm.Extent() != 0)
                {
                    theImages.Bind(aS, aLSIm);
                    theModified.Add(aS);
                }
                continue;
            }

            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            NCollection_List<Shape> aLSImNew = new();

            bool bModified = false;

            // check modifications of the images
            for (NCollection_List<Shape>.Iterator aIt1 = new(pLSIm); aIt1.More(); aIt1.Next())
            {
                Shape aSIm = aIt1.Value();
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
    private static void FindCommonParts(NCollection_List<Shape> theLS1, NCollection_List<Shape> theLS2,
                                        NCollection_List<Shape> theLSC, ShapeType theType = ShapeType.Edge)
    {
        // map shapes in the first list
        NCollection_IndexedMap<Shape> aMS1 = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator aIt = new(theLS1); aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            Exploration.MapShapes(aS, theType, aMS1);
        }
        if (aMS1.IsEmpty())
        {
            return;
        }

        // check for such shapes in the other list
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator aIt = new(theLS2); aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();

            Explorer aExp = new(aS, theType);
            for (; aExp.More(); aExp.Next())
            {
                Shape aST = aExp.Current();
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
    private static int NbPoints(in Edge theEdge)
    {
        EdgeCurveAdaptor aBAC = new(theEdge);
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
    private static bool FindShape(in Shape theSWhat, in Shape theSWhere, BRepOffset_Analyse? theAnalyse,
                                  ref Shape theRes)
    {
        bool bFound = false;
        ShapeType aType = theSWhat.ShapeType();
        Explorer aExp = new(theSWhere, aType);
        for (; aExp.More(); aExp.Next())
        {
            Shape aS = aExp.Current();
            if (aS.IsSame(theSWhat))
            {
                theRes = aS;
                bFound = true;
                break;
            }
        }

        if (!bFound && theAnalyse is not null)
        {
            NCollection_List<Shape>? pLD = theAnalyse.Descendants(theSWhere);
            if (pLD is not null)
            {
                for (NCollection_List<Shape>.Iterator it = new(pLD); it.More(); it.Next())
                {
                    Shape aS = it.Value();
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
    private static void BuildSplitsOfTrimmedFace(in Face theFace, in Shape theEdges,
                                                 NCollection_List<Shape> theLFImages,
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
        for (Explorer anExp = new(aSplitter.InternalShape(), ShapeType.Face); anExp.More(); anExp.Next())
        {
            theLFImages.Append(anExp.Current());
        }
    }

    //=======================================================================
    // function : BuildSplitsOfFace
    // purpose  : Building the splits of offset face
    //=======================================================================
    private static void BuildSplitsOfFace(in Face theFace, in Shape theEdges,
                                          NCollection_DataMap<Shape, Shape> theFacesOrigins,
                                          NCollection_List<Shape> theLFImages)
    {
        theLFImages.Clear();
        //
        // take edges to split the face
        NCollection_List<Shape> aLE = new();
        Explorer aExp = new(theEdges, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aE = ShapeCasts.Edge(aExp.Current());
            aE.Orientation(Forward);
            aLE.Append(aE);
            aE.Orientation(Reversed);
            aLE.Append(aE);
        }
        //
        Face aFF = theFace;
        Orientation anOr = theFace.Orientation();
        aFF.Orientation(Forward);
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
        NCollection_List<Shape> aLFSp = aBF.Areas();
        NCollection_List<Shape>.Iterator aItLF = new(aLFSp);
        for (; aItLF.More(); aItLF.Next())
        {
            ref Shape aFSp = ref aItLF.ChangeValue();
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
    private static Vector3D GetAverageTangent(in Shape theS, int theNbP)
    {
        Vector3D aVA = new();
        Explorer aExp = new(theS, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aE = ShapeCasts.Edge(aExp.Current());
            //
            Curve3D aC = Tool.Curve(aE, out double aT1, out double aT2)!;
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
            if (aE.Orientation() == Reversed)
            {
                aVSum.Reverse();
            }
            //
            aVA += aVSum;
        }
        return aVA;
    }

    // Input data
    private NCollection_List<Shape>? myFaces; //!< Input faces which have to be split
    private BRepAlgo_AsDes? myAsDes; //!< Ascendants/descendants of the edges faces
    private BRepOffset_Analyse? myAnalyzer; //!< Analyzer of the input parameters

    //!< Origins of the offset edges (binding between offset edge and original edge)
    private NCollection_DataMap<Shape, NCollection_List<Shape>>? myEdgesOrigins;
    //!< Origins of the offset faces (binding between offset face and original face)
    private NCollection_DataMap<Shape, Shape>? myFacesOrigins;
    //!< Binding between trimmed and infinite edge
    private NCollection_DataMap<Shape, Shape>? myETrimEInf;

    // Intermediate data
    //!< Images of the offset edges
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myOEImages = new(ShapeHasher.Instance);
    //!< Origins of the splits of offset edges
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myOEOrigins = new(ShapeHasher.Instance);
    //!< Images of the offset edges
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myOFImages = new(ShapeHasher.Instance);

    //!< Edges considered invalid (orientation is changed) in some split of face
    private NCollection_IndexedMap<Shape> myInvalidEdges = new(ShapeHasher.Instance);
    //!< Edges considered valid (orientation is kept) in some split of face
    private readonly NCollection_IndexedMap<Shape> myValidEdges = new(ShapeHasher.Instance);
    //!< Edges considered inverted (vertices swapped) in some split of face
    private readonly NCollection_IndexedMap<Shape> myInvertedEdges = new(ShapeHasher.Instance);
    //!< Splits of edges to be avoided when building splits of faces
    private NCollection_IndexedMap<Shape> myEdgesToAvoid = new(ShapeHasher.Instance);
    //!< Edges marked invalid on the current step and to be avoided on the next step
    private readonly NCollection_Map<Shape> myLastInvEdges = new(ShapeHasher.Instance);
    //!< Edges to be used for building splits
    private readonly NCollection_Map<Shape> myModifiedEdges = new(ShapeHasher.Instance);
    //!< Edges located fully inside solids built from the splits of offset faces
    private readonly NCollection_IndexedMap<Shape> myInsideEdges = new(ShapeHasher.Instance);

    //!< Invalid faces - splits of offset faces consisting of invalid edges
    private NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myInvalidFaces = new(ShapeHasher.Instance);
    //!< Artificially invalid faces - valid faces intentionally marked invalid to be rebuilt on the future steps in the
    //!  situations when invalid edges are present, but invalid faces not
    private readonly NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> myArtInvalidFaces = new(ShapeHasher.Instance);
    //!< Count number of the same face being marked invalid to avoid infinite rebuilding of the same face
    private readonly NCollection_DataMap<Shape, int> myAlreadyInvFaces = new(ShapeHasher.Instance);
    //!< Images of the hole faces of the original face
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myFNewHoles = new(ShapeHasher.Instance);

    //!< Intersection information, used to collect intersection pairs during rebuild
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> mySSInterfs = new(ShapeHasher.Instance);
    //!< Intersection information, used to collect intersection pairs during rebuild
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> mySSInterfsArt = new(ShapeHasher.Instance);
    //!< All possible intersection pairs, used to avoid some of the intersections
    private readonly NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_Map<Shape>>> myIntersectionPairs = new(ShapeHasher.Instance);

    //!< Faces that have to be rebuilt (invalid and close to invalid faces)
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myFacesToRebuild = new(ShapeHasher.Instance);
    //!< Faces that have to be avoided when rebuilding splits of the same offset face
    private readonly NCollection_Map<Shape> myFSelfRebAvoid = new(ShapeHasher.Instance);

    private Shape mySolids = new(); //!< Solids built from the splits of faces

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
    public void SetFaces(NCollection_List<Shape> theFaces) => myFaces = theFaces;

    //! Sets ascendants/descendants information
    public void SetAsDesInfo(BRepAlgo_AsDes theAsDes) => myAsDes = theAsDes;

    //! Sets the analysis info of the input shape
    public void SetAnalysis(BRepOffset_Analyse theAnalyse) => myAnalyzer = theAnalyse;

    //! Sets origins of the offset edges (from original shape)
    public void SetEdgesOrigins(NCollection_DataMap<Shape, NCollection_List<Shape>> theEdgesOrigins) =>
        myEdgesOrigins = theEdgesOrigins;

    //! Sets origins of the offset faces (from original shape)
    public void SetFacesOrigins(NCollection_DataMap<Shape, Shape> theFacesOrigins) =>
        myFacesOrigins = theFacesOrigins;

    //! Sets infinite (extended) edges for the trimmed ones
    public void SetInfEdges(NCollection_DataMap<Shape, Shape> theETrimEInf) => myETrimEInf = theETrimEInf;

    //=================================================================================================

    //! Build splits of already trimmed faces
    public void BuildSplitsOfTrimmedFaces(ProgressRange theRange)
    {
        if (!hasData(myFaces))
        {
            return;
        }

        NCollection_DataMap<Shape, NCollection_List<Shape>> anEdgesOrigins = new(ShapeHasher.Instance);
        if (myEdgesOrigins is null)
        {
            myEdgesOrigins = anEdgesOrigins;
        }

        using ProgressScope aPS = new(theRange, "Building splits of trimmed faces", 5);

        // Fuse all edges
        IntersectTrimmedEdges(aPS.Next(1));

        using ProgressScope aPSLoop = new(aPS.Next(4), null, myFaces!.Extent());
        for (NCollection_List<Shape>.Iterator aItLF = new(myFaces); aItLF.More(); aItLF.Next())
        {
            if (!aPSLoop.More())
            {
                return;
            }
            Face aF = ShapeCasts.Face(aItLF.Value());

            Shape aCE = new();
            bool bFound = GetEdges(aF, ref aCE);

            // split the face by the edges
            if (!bFound)
            {
                if (!myImage.HasImage(aF))
                {
                    myOFImages[myOFImages.Add(aF, new NCollection_List<Shape>())].Append(aF);
                }
                continue;
            }

            NCollection_List<Shape> aLFImages = new();
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
        NCollection_Map<Shape> aVertsToAvoid = new(ShapeHasher.Instance);

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
    private void UpdateIntersectedEdges(NCollection_List<Shape> theLA, BOPAlgo_Builder theGF)
    {
        for (NCollection_List<Shape>.Iterator aItA = new(theLA); aItA.More(); aItA.Next())
        {
            Shape aS = aItA.Value();
            if (!myETrimEInf!.Find(aS, out Shape pEInf))
            {
                continue;
            }

            NCollection_List<Shape> aLSIm = ModifiedOf(theGF, aS);
            if (aLSIm.IsEmpty())
            {
                continue;
            }

            for (NCollection_List<Shape>.Iterator aIt = new(aLSIm); aIt.More(); aIt.Next())
            {
                Shape aEIm = aIt.Value();
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
        NCollection_List<Shape> aLS = new();
        //
        using ProgressScope aPS = new(theRange, null, 2);
        NCollection_List<Shape>.Iterator aItLF = new(myFaces!);
        for (; aItLF.More(); aItLF.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Face aF = ShapeCasts.Face(aItLF.Value());
            //
            NCollection_List<Shape> aLE = myAsDes!.Descendant(aF);
            NCollection_List<Shape>.Iterator aItLE = new(aLE);
            for (; aItLE.More(); aItLE.Next())
            {
                Edge aE = ShapeCasts.Edge(aItLE.Value());
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
        NCollection_List<Shape> aLA = new();
        // fill map with edges images
        using ProgressScope aPSLoop = new(aPS.Next(), null, aLS.Extent());
        for (NCollection_List<Shape>.Iterator aIt = new(aLS); aIt.More(); aIt.Next(), aPSLoop.Next())
        {
            if (!aPSLoop.More())
            {
                return;
            }
            Shape aE = aIt.Value();
            NCollection_List<Shape> aLEIm = ModifiedOf(aGFE, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }
            //
            aLA.Append(aE);
            // save images
            myOEImages.Bind(aE, new NCollection_List<Shape>(aLEIm));
            // save origins
            NCollection_List<Shape>.Iterator aItLE = new(aLEIm);
            for (; aItLE.More(); aItLE.Next())
            {
                Shape aEIm = aItLE.Value();
                if (myOEOrigins.Find(aEIm, out NCollection_List<Shape> pLEOr))
                {
                    AppendToList(pLEOr, aE);
                }
                else
                {
                    NCollection_List<Shape> aL = new();
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
    private static bool checkConnectionsOfFace(in Shape theFace, NCollection_List<Shape> theLF)
    {
        NCollection_IndexedMap<Shape> aShapeVert = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator aFImIterator = new(theLF); aFImIterator.More(); aFImIterator.Next())
        {
            Shape aShape = aFImIterator.Value();
            if (aShape.IsSame(theFace))
            {
                continue;
            }
            Exploration.MapShapes(aShape, ShapeType.Vertex, aShapeVert);
        }
        int aNbConnections = 0;
        NCollection_IndexedMap<Shape> aFaceVertices = new(ShapeHasher.Instance);
        Exploration.MapShapes(theFace, ShapeType.Vertex, aFaceVertices);
        for (int iV = 1; iV <= aFaceVertices.Extent(); ++iV)
        {
            Shape aVert = aFaceVertices[iV];
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
        BrepBuilder aBB = new();
        int i, aNb;
        //
        // processed faces
        NCollection_List<Shape> aLFDone = new();
        // extended face - map of neutral edges, i.e. in one split - valid and in other - invalid
        NCollection_DataMap<Shape, NCollection_Map<Shape>> aDMFMNE = new(ShapeHasher.Instance);
        // map of valid edges for each face
        NCollection_DataMap<Shape, NCollection_Map<Shape>> aDMFMVE = new(ShapeHasher.Instance);
        // map of invalid edges for each face
        NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> aDMFMIE = new(ShapeHasher.Instance);
        // map of valid inverted edges for the face
        NCollection_DataMap<Shape, NCollection_Map<Shape>> aDMFMVIE = new(ShapeHasher.Instance);
        // map of splits to check for internals
        NCollection_IndexedMap<Shape> aMFToCheckInt = new(ShapeHasher.Instance);
        // map of edges created from vertex and marked as invalid
        NCollection_Map<Shape> aMEdgeInvalidByVertex = new(ShapeHasher.Instance);
        // map of edges created from vertex and marked as valid
        NCollection_Map<Shape> aMEdgeValidByVertex = new(ShapeHasher.Instance);
        // connection map from old edges to new ones
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMEOrLEIm = new(ShapeHasher.Instance);
        //
        // Outer range
        using ProgressScope aPSOuter = new(theRange, null, 10.0);
        // build splits of faces
        using ProgressScope aPSBF = new(aPSOuter.Next(3.0), "Building faces", 2 * myFaces!.Extent());
        NCollection_List<Shape>.Iterator aItLF = new(myFaces);
        for (; aItLF.More(); aItLF.Next(), aPSBF.Next())
        {
            if (!aPSBF.More())
            {
                return;
            }
            Face aF = ShapeCasts.Face(aItLF.Value());
            //
            NCollection_List<Shape>? pLFIm = myOFImages.FindFromKey(aF, out NCollection_List<Shape> aFound) ? aFound : null;
            if (pLFIm is not null && pLFIm.IsEmpty())
            {
                continue;
            }
            // get edges by which the face should be split
            Shape aCE = new();
            NCollection_IndexedMap<Shape> aMapEInv = new(ShapeHasher.Instance);
            bool bFound = GetEdges(aF, ref aCE, aMapEInv);
            if (!bFound)
            {
                continue;
            }
            //
            // OFFSET_DEBUG: the self-intersection check of the edges is left out.
            // build splits
            NCollection_List<Shape> aLFImages = new();
            BuildSplitsOfFace(aF, aCE, myFacesOrigins!, aLFImages);
            //
            if (aMapEInv.Extent() != 0)
            {
                // check if all possible faces are built
                NCollection_Map<Shape> aMENInv = new(ShapeHasher.Instance);
                bool bArtificialCase = aLFImages.IsEmpty() || CheckIfArtificial(aF, aLFImages, aCE, aMapEInv, aMENInv);
                //
                // try to build splits using invalid edges
                Compound aCE1 = new();
                aBB.MakeCompound(ref aCE1);
                aBB.Add(aCE1, aCE);
                for (i = 1; i <= aMapEInv.Extent(); ++i)
                {
                    aBB.Add(aCE1, aMapEInv[i]);
                }
                //
                NCollection_List<Shape> aLFImages1 = new();
                BuildSplitsOfFace(aF, aCE1, myFacesOrigins!, aLFImages1);
                //
                // check if the rebuilding has added some new faces to the splits
                for (NCollection_List<Shape>.Iterator aItLFIm = new(aLFImages1); aItLFIm.More();)
                {
                    bool bAllInv = true;
                    // Additional check for artificial case
                    // if current image face consist only of edges from aMapEInv and aMENInv
                    // then recheck current face for the further processing
                    bool aToReCheckFace = bArtificialCase;
                    Shape aFIm = aItLFIm.Value();
                    Explorer aExpE = new(aFIm, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape aE = aExpE.Current();
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
                    NCollection_IndexedMap<Shape> aMEInv = new(ShapeHasher.Instance);
                    //
                    // OCCT: "*pLFIm = aLFImages" dereferences a null pointer when the face has no splits yet
                    // (an access violation natively, a NullReferenceException here; occt-issues.md).
                    pLFIm!.Assign(aLFImages);
                    NCollection_List<Shape>.Iterator aItLFIm = new(aLFImages);
                    for (; aItLFIm.More(); aItLFIm.Next())
                    {
                        Shape aFIm = aItLFIm.Value();
                        Explorer aExpE = new(aFIm, ShapeType.Edge);
                        for (; aExpE.More(); aExpE.Next())
                        {
                            Shape aE = aExpE.Current();
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
                    myArtInvalidFaces.Bind(aF, new NCollection_IndexedMap<Shape>(aMEInv));
                    aDMFMIE.Bind(aF, new NCollection_IndexedMap<Shape>(aMEInv));
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
                pLFIm = myOFImages[myOFImages.Add(aF, new NCollection_List<Shape>())];
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
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFMap = new(ShapeHasher.Instance);
        int aNbF = myOFImages.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            NCollection_List<Shape>.Iterator itLFIm = new(myOFImages[i]);
            for (; itLFIm.More(); itLFIm.Next())
            {
                Exploration.MapShapesAndAncestors(itLFIm.Value(), ShapeType.Edge, ShapeType.Face, anEFMap);
            }
        }

        NCollection_Map<Shape> anEmptyMap = new(ShapeHasher.Instance);
        // invalid faces inside the holes
        NCollection_IndexedMap<Shape> aMFInvInHole = new(ShapeHasher.Instance);
        // all hole faces
        Compound aFHoles = new();
        aBB.MakeCompound(ref aFHoles);
        // Find the faces containing only the inverted edges and the invalid ones
        NCollection_List<Shape> anInvertedFaces = new();

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
            Face aF = ShapeCasts.Face(aItLF.Value());
            NCollection_List<Shape> aLFImages = myOFImages.ChangeFromKey(aF);
            //
            NCollection_List<Shape> aLFInv = new();
            bool bArtificialCase = myArtInvalidFaces.IsBound(aF);
            if (bArtificialCase)
            {
                aLFInv.Assign(aLFImages);
            }
            else
            {
                // neutral edges
                if (!aDMFMNE.Find(aF, out NCollection_Map<Shape> pMNE))
                {
                    pMNE = anEmptyMap;
                }
                // find faces inside holes wires
                NCollection_Map<Shape> aMFHoles = new(ShapeHasher.Instance);
                Face aFOr = ShapeCasts.Face(myFacesOrigins!.Find(aF));
                FindFacesInsideHoleWires(aFOr, aF, aLFImages, aDMEOrLEIm, anEFMap, aMFHoles);
                //
                foreach (Shape aItMH in aMFHoles)
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
        NCollection_IndexedMap<Shape> aMERemoved = new(ShapeHasher.Instance);
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
        Compound aCFIm = new();
        aBB.MakeCompound(ref aCFIm);
        //
        aNb = myOFImages.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<Shape> aLFIm = myOFImages[i];
            aItLF.Initialize(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape aFIm = aItLF.Value();
                aBB.Add(aCFIm, aFIm);
            }
        }
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aCFIm, ShapeType.Edge, ShapeType.Face, aDMEF);
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
        NCollection_Map<Shape> aMEUseInRebuild = new(ShapeHasher.Instance);
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
            Shape aE = myInvalidEdges[i];
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
            Shape aF = myInvalidFaces.FindKey(i);
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
    private bool GetEdges(in Face theFace, ref Shape theEdges,
                          NCollection_IndexedMap<Shape>? theInvMap = null)
    {
        // get boundary edges
        NCollection_Map<Shape> aMFBounds = new(ShapeHasher.Instance);
        Explorer aExp = new(theFace, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Shape aE = aExp.Current();
            if (myOEImages.Find(aE, out NCollection_List<Shape> pLEIm))
            {
                for (NCollection_List<Shape>.Iterator aItLE0 = new(pLEIm); aItLE0.More(); aItLE0.Next())
                {
                    aMFBounds.Add(aItLE0.Value());
                }
            }
            else
            {
                aMFBounds.Add(aE);
            }
        }

        BrepBuilder aBB = new();
        bool bFound = false, bUpdate = false;
        // the resulting edges
        Compound anEdges = new();
        aBB.MakeCompound(ref anEdges);
        // Fence map
        NCollection_Map<Shape> aMEFence = new(ShapeHasher.Instance);
        // the edges by which the offset face should be split
        NCollection_List<Shape> aLE = myAsDes!.Descendant(theFace);
        NCollection_List<Shape>.Iterator aItLE = new(aLE);
        for (; aItLE.More(); aItLE.Next())
        {
            Edge aE = ShapeCasts.Edge(aItLE.Value());
            //
            if (!bUpdate)
            {
                bUpdate = myModifiedEdges.Contains(aE);
            }
            //
            if (myOEImages.Find(aE, out NCollection_List<Shape> pLEIm))
            {
                NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    Edge aEIm = ShapeCasts.Edge(aItLEIm.Value());
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
    private bool CheckIfArtificial(in Shape theF, NCollection_List<Shape> theLFImages, in Shape theCE,
                                   NCollection_IndexedMap<Shape> theMapEInv, NCollection_Map<Shape> theMENInv)
    {
        // all boundary edges should be used
        NCollection_IndexedMap<Shape> aMEUsed = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLFIm = new(theLFImages);
        for (; aItLFIm.More(); aItLFIm.Next())
        {
            Shape aFIm = aItLFIm.Value();
            Exploration.MapShapes(aFIm, ShapeType.Edge, aMEUsed);
            Exploration.MapShapes(aFIm, ShapeType.Vertex, aMEUsed);
        }
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theCE, ShapeType.Vertex, ShapeType.Edge, aMVE);
        //
        int i, aNb = theMapEInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aEInv = theMapEInv[i];
            Explorer aExpV = new(aEInv, ShapeType.Vertex);
            for (; aExpV.More(); aExpV.Next())
            {
                Shape aVEInv = aExpV.Current();
                if (aMVE.FindFromKey(aVEInv, out NCollection_List<Shape> pLENInv))
                {
                    NCollection_List<Shape>.Iterator aItLEInv = new(pLENInv);
                    for (; aItLEInv.More(); aItLEInv.Next())
                    {
                        Shape aENInv = aItLEInv.Value();
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
        NCollection_IndexedMap<Shape> aMEFound = new(ShapeHasher.Instance);
        Exploration.MapShapes(theCE, ShapeType.Edge, aMEFound);
        //
        NCollection_List<Shape> aLE = myAsDes!.Descendant(theF);
        NCollection_List<Shape>.Iterator aItLE = new(aLE);
        for (; aItLE.More(); aItLE.Next())
        {
            Edge aE = ShapeCasts.Edge(aItLE.Value());
            //
            if (myOEImages.Find(aE, out NCollection_List<Shape> pLEIm))
            {
                bool bChecked = false;
                NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    Edge aEIm = ShapeCasts.Edge(aItLEIm.Value());
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
    private void FindInvalidEdges(in Face theF, NCollection_List<Shape> theLFImages,
                                  NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMFMVE,
                                  NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMFMNE,
                                  NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> theDMFMIE,
                                  NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMFMVIE,
                                  NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEOrLEIm,
                                  NCollection_Map<Shape> theEdgesInvalidByVertex,
                                  NCollection_Map<Shape> theEdgesValidByVertex, ProgressRange theRange)
    {
        // Edge is considered as invalid in the following cases:
        // 1. Its orientation on the face has changed comparing to the originals edge and face;
        // 2. The vertices of the edge have changed places comparing to the originals edge and face.
        //
        // The edges created from vertices, i.e. as intersection between two faces connected only
        // by VERTEX, will also be checked on validity. For these edges the correct orientation will
        // be defined by the edges on the original face adjacent to the connection vertex

        // original face
        Face aFOr = ShapeCasts.Face(myFacesOrigins!.Find(theF));
        // invalid edges
        NCollection_IndexedMap<Shape> aMEInv = new(ShapeHasher.Instance);
        // valid edges
        NCollection_Map<Shape> aMEVal = new(ShapeHasher.Instance);
        // internal edges
        NCollection_Map<Shape> aMEInt = new(ShapeHasher.Instance);
        //
        // maps for checking the inverted edges
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVE = new(ShapeHasher.Instance),
                                                                                 aDMEF = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMEdges = new(ShapeHasher.Instance);
        // back map from the original shapes to their offset images
        NCollection_DataMap<Shape, NCollection_List<Shape>> anImages = new(ShapeHasher.Instance);
        //
        using ProgressScope aPS = new(theRange, "Checking validity of edges", 2 * theLFImages.Extent());
        NCollection_List<Shape>.Iterator aItLF = new(theLFImages);
        for (; aItLF.More(); aItLF.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Face aFIm = ShapeCasts.Face(aItLF.Value());
            //
            Explorer aExp = new(aFIm, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Shape aE = aExp.Current();
                // keep all edges
                aMEdges.Add(aE);
                //
                // keep connection from edges to faces
                if (!aDMEF.FindFromKey(aE, out NCollection_List<Shape> pLF))
                {
                    pLF = aDMEF[aDMEF.Add(aE, new NCollection_List<Shape>())];
                }
                AppendToList(pLF, aFIm);
                //
                // keep connection from vertices to edges
                Iterator aItV = new(aE);
                for (; aItV.More(); aItV.Next())
                {
                    Shape aV = aItV.Value();
                    //
                    if (!aDMVE.FindFromKey(aV, out NCollection_List<Shape> pLE))
                    {
                        pLE = aDMVE[aDMVE.Add(aV, new NCollection_List<Shape>())];
                    }
                    AppendToList(pLE, aE);
                }

                // back map from original edges to their offset images
                if (!myEdgesOrigins!.Find(aE, out NCollection_List<Shape> pLOr))
                {
                    continue;
                }
                for (NCollection_List<Shape>.Iterator itOr = new(pLOr); itOr.More(); itOr.Next())
                {
                    Shape aSOr = itOr.Value();
                    Shape aSInF = new();
                    if (!FindShape(aSOr, aFOr, myAnalyzer, ref aSInF))
                    {
                        continue;
                    }
                    if (!anImages.Find(aSInF, out NCollection_List<Shape> pImages))
                    {
                        pImages = new NCollection_List<Shape>();
                        anImages.Bind(aSInF, pImages);
                    }
                    AppendToList(pImages, aE);
                }
            }
        }
        //
        // the map will be used to find the edges on the original face
        // adjacent to the same vertex. It will be filled at first necessity;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVEFOr = new(ShapeHasher.Instance);
        //
        aItLF.Initialize(theLFImages);
        for (; aItLF.More(); aItLF.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Face aFIm = ShapeCasts.Face(aItLF.Value());
            //
            // valid edges for this split
            NCollection_Map<Shape> aMVE = new(ShapeHasher.Instance);
            // invalid edges for this split
            NCollection_IndexedMap<Shape> aMIE = new(ShapeHasher.Instance);
            //
            Explorer aExp = new(aFIm, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Edge aEIm = ShapeCasts.Edge(aExp.Current());
                //
                if (aEIm.Orientation() == Internal)
                {
                    aMEInt.Add(aEIm);
                    continue;
                }
                //
                if (!myEdgesOrigins!.Find(aEIm, out NCollection_List<Shape> pLEOr) || pLEOr.IsEmpty())
                {
                    continue;
                }
                //
                int aNbVOr = 0;
                NCollection_List<Shape>.Iterator aItLEO = new(pLEOr);
                for (; aItLEO.More(); aItLEO.Next())
                {
                    if (aItLEO.Value().ShapeType() == ShapeType.Vertex)
                    {
                        ++aNbVOr;
                    }
                }
                if (aNbVOr > 1 && (pLEOr.Extent() - aNbVOr) > 1)
                {
                    continue;
                }
                //
                NCollection_Map<Shape> aME = new(ShapeHasher.Instance), aMV = new(ShapeHasher.Instance),
                                              aMF = new(ShapeHasher.Instance);
                bool bInvalid = false, bChecked = false;
                int aNbP = NbPoints(aEIm), aNbInv = 0;
                bool bUseVertex = aNbVOr == 0 ? false
                                              : (aNbVOr == 1 && aDMEF.FindFromKey(aEIm).Extent() == 1
                                                 && !myOEOrigins.IsBound(aEIm));
                //
                aItLEO.Initialize(pLEOr);
                for (; aItLEO.More(); aItLEO.Next())
                {
                    Shape aSOr = aItLEO.Value();
                    bool bVertex = (aSOr.ShapeType() == ShapeType.Vertex);
                    //
                    Shape aEOrF = new();
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
                            Exploration.MapShapesAndAncestors(aFOr, ShapeType.Vertex, ShapeType.Edge, aDMVEFOr);
                        }
                        //
                        if (aDMVEFOr.FindFromKey(aSOr, out NCollection_List<Shape> pLEFOr))
                        {
                            Compound aCEOr = new();
                            new BrepBuilder().MakeCompound(ref aCEOr);
                            // Avoid classification of edges originated from vertices
                            // located between tangent edges
                            bool bAllTgt = true;
                            NCollection_List<Shape>.Iterator aItLEFOr = new(pLEFOr);
                            Vector3D aVRef = GetAverageTangent(aItLEFOr.Value(), aNbP);
                            for (; aItLEFOr.More(); aItLEFOr.Next())
                            {
                                Shape aEOr = aItLEFOr.Value();
                                new BrepBuilder().Add(aCEOr, aEOr);

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
                        if (!theDMEOrLEIm.Find(aSOr, out NCollection_List<Shape> pLEIm))
                        {
                            pLEIm = new NCollection_List<Shape>();
                            theDMEOrLEIm.Bind(aSOr, pLEIm);
                        }
                        AppendToList(pLEIm, aEIm);
                    }
                    //
                    if (aEOrF.IsNull)
                    {
                        // the edge has not been found
                        continue;
                    }

                    if (bVertex)
                    {
                        NCollection_Map<Shape> aMVTotal = new(ShapeHasher.Instance);
                        int aNbChecked = 0;
                        // Just check if the original edges sharing the vertex do not share it any more.
                        for (Iterator it = new(aEOrF); it.More(); it.Next())
                        {
                            Shape aEOr = it.Value();
                            if (!anImages.Find(aEOr, out NCollection_List<Shape> aLIm))
                            {
                                continue;
                            }
                            ++aNbChecked;
                            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVLoc = new(ShapeHasher.Instance);
                            for (NCollection_List<Shape>.Iterator itLIM = new(aLIm); itLIM.More(); itLIM.Next())
                            {
                                Exploration.MapShapesAndAncestors(itLIM.Value(), ShapeType.Vertex, ShapeType.Edge, aMVLoc);
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
                        Explorer aExpE = new(aEOrF, ShapeType.Vertex);
                        for (; aExpE.More(); aExpE.Next())
                        {
                            Shape aV = aExpE.Current();
                            aMV.Add(aV);
                        }
                        if (myAnalyzer is not null)
                        {
                            for (NCollection_List<Shape>.Iterator itFA = new(myAnalyzer.Ancestors(aEOrF)); itFA.More(); itFA.Next())
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
                            Iterator itV = new(aEIm);
                            for (; itV.More(); itV.Next())
                            {
                                NCollection_List<Shape>.Iterator itE = new(aDMVE.FindFromKey(itV.Value()));
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
        NCollection_Map<Shape> aMVIE = new(ShapeHasher.Instance);
        // fill neutral edges
        NCollection_Map<Shape> aMNE = new(ShapeHasher.Instance);
        //
        int i, aNbEInv = aMEInv.Extent();
        for (i = 1; i <= aNbEInv; ++i)
        {
            Shape aEIm = aMEInv[i];
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
            if (!myOEOrigins.Find(aEIm, out NCollection_List<Shape> pLOEOr))
            {
                continue;
            }
            //
            NCollection_List<Shape>.Iterator aItLOEOr = new(pLOEOr);
            for (; aItLOEOr.More(); aItLOEOr.Next())
            {
                Shape aOEOr = aItLOEOr.Value();
                NCollection_List<Shape> aLEIm1 = myOEImages.Find(aOEOr);
                //
                NCollection_List<Shape>.Iterator aItLEIm1 = new(aLEIm1);
                for (; aItLEIm1.More(); aItLEIm1.Next())
                {
                    Shape aEIm1 = aItLEIm1.Value();
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
    private static void addAsNeutral(in Shape theE, in Shape theFInv, in Shape theFVal,
                                     NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> theLocInvEdges,
                                     NCollection_DataMap<Shape, NCollection_Map<Shape>> theLocValidEdges)
    {
        if (!theLocInvEdges.Find(theFInv, out NCollection_IndexedMap<Shape> pMEInv))
        {
            pMEInv = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            theLocInvEdges.Bind(theFInv, pMEInv);
        }
        pMEInv.Add(theE);

        if (!theLocValidEdges.Find(theFVal, out NCollection_Map<Shape> pMEVal))
        {
            pMEVal = new NCollection_Map<Shape>(ShapeHasher.Instance);
            theLocValidEdges.Bind(theFVal, pMEVal);
        }
        pMEVal.Add(theE);
    }

    //=======================================================================
    // function : FindInvalidEdges
    // purpose  : Additional method to look for invalid edges
    //=======================================================================
    private void FindInvalidEdges(NCollection_List<Shape> theLFOffset,
                                  NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> theLocInvEdges,
                                  NCollection_DataMap<Shape, NCollection_Map<Shape>> theLocValidEdges,
                                  NCollection_DataMap<Shape, NCollection_Map<Shape>> theNeutralEdges)
    {
        // 1. Find edges unclassified in faces
        // 2. Find SD faces in which the same edge is classified
        // 3. Check if the edge is neutral in face in which it wasn't classified

        NCollection_IndexedDataMap<Shape, NCollection_Map<Shape>> aMEUnclassified = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> aFSplitFOffset = new(ShapeHasher.Instance);

        // Avoid artificial faces
        NCollection_Map<Shape> aNewFaces = new(ShapeHasher.Instance);
        if (myAnalyzer is not null)
        {
            NCollection_Map<Shape> aMapNewTmp = new(ShapeHasher.Instance);
            for (NCollection_List<Shape>.Iterator it = new(myAnalyzer.NewFaces()); it.More(); it.Next())
            {
                aMapNewTmp.Add(it.Value());
            }

            for (NCollection_List<Shape>.Iterator it = new(theLFOffset); it.More(); it.Next())
            {
                Shape aFOffset = it.Value();
                Shape aFOrigin = myFacesOrigins!.Find(aFOffset);
                if (aMapNewTmp.Contains(aFOrigin))
                {
                    aNewFaces.Add(aFOffset);
                }
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFMap = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator itLFO = new(theLFOffset); itLFO.More(); itLFO.Next())
        {
            Shape aF = itLFO.Value();
            if (aNewFaces.Contains(aF))
            {
                continue;
            }

            NCollection_List<Shape> aLFImages = myOFImages.FindFromKey(aF);
            for (NCollection_List<Shape>.Iterator itLF = new(aLFImages); itLF.More(); itLF.Next())
            {
                Shape aFIm = itLF.Value();

                Exploration.MapShapesAndAncestors(aFIm, ShapeType.Edge, ShapeType.Face, anEFMap);

                theLocInvEdges.Find(aFIm, out NCollection_IndexedMap<Shape>? pMEInvalid);
                theLocValidEdges.Find(aFIm, out NCollection_Map<Shape>? pMEValid);

                for (Explorer expE = new(aFIm, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Shape aE = expE.Current();
                    if (myInvalidEdges.Contains(aE) != myValidEdges.Contains(aE))
                    {
                        // edge is classified in some face

                        if ((pMEInvalid is null || !pMEInvalid.Contains(aE)) && (pMEValid is null || !pMEValid.Contains(aE)))
                        {
                            // but not in the current one
                            if (!aMEUnclassified.FindFromKey(aE, out NCollection_Map<Shape> pMap))
                            {
                                pMap = aMEUnclassified[aMEUnclassified.Add(aE, new NCollection_Map<Shape>(ShapeHasher.Instance))];
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
            Shape aE = aMEUnclassified.FindKey(iE);
            NCollection_Map<Shape> aMFUnclassified = aMEUnclassified[iE];

            NCollection_List<Shape> aLF = anEFMap.FindFromKey(aE);

            for (NCollection_List<Shape>.Iterator itLF = new(aLF); itLF.More(); itLF.Next())
            {
                Shape aFClassified = itLF.Value();
                if (aMFUnclassified.Contains(aFClassified))
                {
                    continue;
                }

                BOPTools_Set anEdgeSetClass = new();
                anEdgeSetClass.Add(aFClassified, ShapeType.Edge);

                Shape aEClassified = new();
                FindShape(aE, aFClassified, null, ref aEClassified);
                Orientation anOriClass = aEClassified.Orientation();

                BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(ShapeCasts.Edge(aEClassified), ShapeCasts.Face(aFClassified),
                                                           out Direction3D aDNClass);

                theLocInvEdges.Find(aFClassified, out NCollection_IndexedMap<Shape>? pMEInvalid);
                bool isInvalid = pMEInvalid is not null && pMEInvalid.Contains(aE);

                // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
                foreach (Shape aFUnclassified in aMFUnclassified)
                {
                    BOPTools_Set anEdgeSetUnclass = new();
                    anEdgeSetUnclass.Add(aFUnclassified, ShapeType.Edge);

                    if (anEdgeSetClass.IsEqual(anEdgeSetUnclass))
                    {
                        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(ShapeCasts.Edge(aE), ShapeCasts.Face(aFUnclassified),
                                                                   out Direction3D aDNUnclass);

                        bool isSameOri = aDNClass.IsEqual(aDNUnclass, global::Kerf.Engine.Foundation.Precision.Angular());

                        // Among other splits of the same face find those where the edge is contained with
                        // different orientation
                        Shape aFOffset = aFSplitFOffset.Find(aFUnclassified);
                        NCollection_List<Shape> aLFSplits = myOFImages.FindFromKey(aFOffset);
                        NCollection_List<Shape>.Iterator itLFSp = new(aLFSplits);
                        for (; itLFSp.More(); itLFSp.Next())
                        {
                            Shape aFSp = itLFSp.Value();

                            if (!aFSp.IsSame(aFUnclassified) && aMFUnclassified.Contains(aFSp))
                            {
                                Shape aEUnclassified = new();
                                FindShape(aE, aFSp, null, ref aEUnclassified);

                                Orientation anOriUnclass = aEUnclassified.Orientation();
                                if (!isSameOri)
                                {
                                    anOriUnclass = global::Kerf.Engine.Topology.TopAbs.Reverse(anOriUnclass);
                                }

                                if (anOriClass != anOriUnclass)
                                {
                                    // make the edge neutral for the face
                                    if (!theNeutralEdges.Find(aFOffset, out NCollection_Map<Shape> pMENeutral))
                                    {
                                        pMENeutral = new NCollection_Map<Shape>(ShapeHasher.Instance);
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
    private void MakeInvertedEdgesInvalid(NCollection_List<Shape> theLFOffset)
    {
        if (myInvalidEdges.IsEmpty() || myInvertedEdges.IsEmpty())
        {
            return;
        }

        // Map all invalid edges
        Compound aCBEInv = new();
        new BrepBuilder().MakeCompound(ref aCBEInv);
        for (int i = 1; i <= myInvalidEdges.Extent(); ++i)
        {
            new BrepBuilder().Add(aCBEInv, myInvalidEdges[i]);
        }

        // Make loops of invalid edges
        NCollection_List<Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCBEInv, ShapeType.Vertex, ShapeType.Edge, aLCB);

        // Analyze each loop on closeness and use only closed ones
        NCollection_DataMap<Shape, Shape> aDMVCB = new(ShapeHasher.Instance);

        for (NCollection_List<Shape>.Iterator itLCB = new(aLCB); itLCB.More(); itLCB.Next())
        {
            Shape aCB = itLCB.Value();

            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVE = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aCB, ShapeType.Vertex, ShapeType.Edge, aDMVE);
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
        for (NCollection_List<Shape>.Iterator itLF = new(theLFOffset); itLF.More(); itLF.Next())
        {
            NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(itLF.Value());
            for (NCollection_List<Shape>.Iterator itLFIm = new(aLFIm); itLFIm.More(); itLFIm.Next())
            {
                for (Explorer expE = new(itLFIm.Value(), ShapeType.Edge); expE.More(); expE.Next())
                {
                    Edge aE = ShapeCasts.Edge(expE.Current());
                    if (!myInvalidEdges.Contains(aE) && myInvertedEdges.Contains(aE))
                    {
                        bool bCB1 = aDMVCB.Find(Exploration.FirstVertex(aE), out Shape pCB1);
                        bool bCB2 = aDMVCB.Find(Exploration.LastVertex(aE), out Shape pCB2);
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
    private void FindInvalidFaces(NCollection_List<Shape> theLFImages,
                                  NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMFMVE,
                                  NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> theDMFMIE,
                                  NCollection_Map<Shape> theMENeutral,
                                  NCollection_Map<Shape> theEdgesInvalidByVertex,
                                  NCollection_Map<Shape> theEdgesValidByVertex,
                                  NCollection_Map<Shape> theMFHoles,
                                  NCollection_IndexedMap<Shape> theMFInvInHole,
                                  NCollection_List<Shape> theInvFaces,
                                  NCollection_List<Shape> theInvertedFaces)
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
        NCollection_IndexedMap<Shape> aMENRem = new(ShapeHasher.Instance);
        //
        // faces for post treat
        NCollection_List<Shape> aLFPT = new();
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEF = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLF = new(theLFImages);
        for (; aItLF.More(); aItLF.Next())
        {
            Face aFIm = ShapeCasts.Face(aItLF.Value());
            Exploration.MapShapesAndAncestors(aFIm, ShapeType.Edge, ShapeType.Face, aDMEF);
        }

        aItLF.Initialize(theLFImages);
        for (; aItLF.More();)
        {
            Face aFIm = ShapeCasts.Face(aItLF.Value());
            //
            // valid edges for this split
            theDMFMVE.Find(aFIm, out NCollection_Map<Shape>? pMVE);
            // invalid edges for this split
            theDMFMIE.Find(aFIm, out NCollection_IndexedMap<Shape>? pMIE);
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
            Wire aWIm = BrepTools.OuterWire(aFIm);
            Explorer aExp = new(aWIm, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Shape aEIm = aExp.Current();
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
                Exploration.MapShapes(aFIm, ShapeType.Edge, aMENRem);
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
                Exploration.MapShapes(aFIm, ShapeType.Edge, aMENRem);
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
                Exploration.MapShapes(aFIm, ShapeType.Edge, aMENRem);
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
            Face aFIm = ShapeCasts.Face(aItLF.Value());
            //
            // valid edges for this split
            theDMFMVE.Find(aFIm, out NCollection_Map<Shape>? pMVE);
            //
            bHasValid = false;
            bAllValid = true;
            bAllInvalid = true;
            //
            Wire aWIm = BrepTools.OuterWire(aFIm);
            Explorer aExp = new(aWIm, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Shape aEIm = aExp.Current();
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
    private void FindFacesInsideHoleWires(in Face theFOrigin, in Face theFOffset,
                                          NCollection_List<Shape> theLFImages,
                                          NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEOrLEIm,
                                          NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theEFMap,
                                          NCollection_Map<Shape> theMFHoles)
    {
        if (theLFImages.IsEmpty())
        {
            return;
        }
        //
        // find all hole wires in the original face
        NCollection_List<Shape> aLHoleWires = new();
        Wire anOuterWire = BrepTools.OuterWire(theFOrigin);
        Explorer aExpW = new(theFOrigin, ShapeType.Wire);
        for (; aExpW.More(); aExpW.Next())
        {
            Wire aHoleWire = ShapeCasts.Wire(aExpW.Current());
            if (!aHoleWire.IsSame(anOuterWire) && aHoleWire.Orientation() != Internal)
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
        if (!myFNewHoles.Find(theFOrigin, out NCollection_List<Shape> pLFNewHoles))
        {
            pLFNewHoles = new NCollection_List<Shape>();
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
            NCollection_IndexedMap<Shape> aMESplits = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator aItLF0 = new(theLFImages);
            for (; aItLF0.More(); aItLF0.Next())
            {
                Exploration.MapShapes(aItLF0.Value(), ShapeType.Edge, aMESplits);
            }
            //
            NCollection_List<Shape>.Iterator aItLW = new(aLHoleWires);
            for (; aItLW.More(); aItLW.Next())
            {
                Wire aHoleWire = ShapeCasts.Wire(aItLW.Value());
                // find images of all edges of the original wire
                NCollection_IndexedMap<Shape> aMEImWire = new(ShapeHasher.Instance);
                Iterator aItE = new(aHoleWire);
                for (; aItE.More(); aItE.Next())
                {
                    Shape aEOr = aItE.Value();
                    if (!theDMEOrLEIm.Find(aEOr, out NCollection_List<Shape> pLEIm) || pLEIm.IsEmpty())
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
                    for (; aItLEIm.More(); aItLEIm.Next())
                    {
                        Shape aEIm = aItLEIm.Value();
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
                NCollection_List<Shape> aLE = new();
                int i, aNbE0 = aMEImWire.Extent();
                for (i = 1; i <= aNbE0; ++i)
                {
                    aLE.Append(aMEImWire[i].Oriented(Forward));
                    aLE.Append(aMEImWire[i].Oriented(Reversed));
                }
                //
                BOPAlgo_BuilderFace aBF = new();
                aBF.SetFace(ShapeCasts.Face(theFOffset.Oriented(Forward)));
                aBF.SetShapes(aLE);
                aBF.Perform();
                //
                NCollection_List<Shape> aLFNew = aBF.Areas();
                if (aLFNew.IsEmpty())
                {
                    continue;
                }
                //
                // check if outer edges in the new faces are not inverted
                // because the inverted edges mean that the hole has been
                // filled during offset and there will be no faces to remove
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEFNew = new(ShapeHasher.Instance);
                NCollection_List<Shape>.Iterator aItLFNew = new(aLFNew);
                for (; aItLFNew.More(); aItLFNew.Next())
                {
                    Exploration.MapShapesAndAncestors(aItLFNew.Value(), ShapeType.Edge, ShapeType.Face, aDMEFNew);
                }
                //
                aNbE0 = aDMEFNew.Extent();
                for (i = 1; i <= aNbE0; ++i)
                {
                    if (aDMEFNew[i].Extent() == 1)
                    {
                        Shape aE = aDMEFNew.FindKey(i);
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
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFSplitsMap = new(ShapeHasher.Instance);
        // Build Edge-Face map for holes
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFHolesMap = new(ShapeHasher.Instance);

        // among the splits of the offset face find those that are
        // located inside the hole faces
        NCollection_List<Shape>.Iterator aItLF = new(theLFImages);
        for (; aItLF.More(); aItLF.Next())
        {
            Face aFIm = ShapeCasts.Face(aItLF.Value());
            Exploration.MapShapesAndAncestors(aFIm, ShapeType.Edge, ShapeType.Face, anEFSplitsMap);
            // get the point inside the face and classify it relatively hole faces
            Point3D aP3D = new();
            Point2D aP2D = new();
            int iErr = BOPTools_AlgoTools3D.PointInFace(aFIm, ref aP3D, ref aP2D, myContext);
            if (iErr != 0)
            {
                continue;
            }
            //
            double aTol = Tool.Tolerance(aFIm);
            //
            NCollection_List<Shape>.Iterator aItLFNew = new(pLFNewHoles);
            for (; aItLFNew.More(); aItLFNew.Next())
            {
                Face aFNew = ShapeCasts.Face(aItLFNew.Value());
                if (myContext.IsValidPointForFace(aP3D, aFNew, aTol))
                {
                    // the face is classified as IN
                    theMFHoles.Add(aFIm);
                    Exploration.MapShapesAndAncestors(aFIm, ShapeType.Edge, ShapeType.Face, anEFHolesMap);
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
            Shape anEdge = anEFHolesMap.FindKey(i);
            NCollection_List<Shape> aLFHoles = anEFHolesMap[i];
            // Check if the edge is outer for holes
            if (aLFHoles.Extent() != 1)
            {
                continue;
            }

            Shape aFHole = aLFHoles.First();
            if (!theMFHoles.Contains(aFHole))
            {
                // Already removed
                continue;
            }

            // Check if the edge is not outer for splits
            NCollection_List<Shape> aLSplits = anEFSplitsMap.FindFromKey(anEdge);
            if (aLSplits.Extent() == 1)
            {
                continue;
            }

            // Check if edge is only connected to splits of the current offset face
            NCollection_List<Shape> aLFAll = theEFMap.FindFromKey(anEdge);
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
    private bool CheckInverted(in Edge theEIm, in Face theFOr,
                               NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theDMVE,
                               NCollection_IndexedMap<Shape> theMEdges)
    {
        // It is necessary to compare the direction from first vertex
        // to the last vertex on the original edge with the
        // same direction on the new edge. If the directions
        // will be different - the edge has been inverted.
        //
        Vertex aVO1 = new(), aVO2 = new(); // vertices on the original edge
        //
        int i;
        // find vertices of the offset shape
        Exploration.Vertices(theEIm, out Vertex aVI1, out Vertex aVI2); // vertices on the offset edge
        //
        // find images
        NCollection_List<Shape> aLEImages = new();
        if (myOEOrigins.IsBound(theEIm))
        {
            Wire anImages = new();
            new BrepBuilder().MakeWire(ref anImages);
            //
            NCollection_Map<Shape> aMImFence = new(ShapeHasher.Instance);
            NCollection_List<Shape> aLOffsetOr = myOEOrigins.Find(theEIm);
            NCollection_List<Shape>.Iterator aItOffset = new(aLOffsetOr);
            for (; aItOffset.More(); aItOffset.Next())
            {
                Shape aEOffsetOr = aItOffset.Value();
                NCollection_List<Shape> aLImages = myOEImages.Find(aEOffsetOr);
                //
                NCollection_List<Shape>.Iterator aItImages = new(aLImages);
                for (; aItImages.More(); aItImages.Next())
                {
                    Edge anIm = ShapeCasts.Edge(aItImages.Value());
                    if (theMEdges.Contains(anIm) && aMImFence.Add(anIm))
                    {
                        new BrepBuilder().Add(anImages, anIm);
                        aLEImages.Append(anIm);
                    }
                }
            }
            //
            // find alone vertices
            Vertex aVW1, aVW2;
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMImVE = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(anImages, ShapeType.Vertex, ShapeType.Edge, aDMImVE);
            //
            NCollection_List<Shape> aLVAlone = new();
            int aNb = aDMImVE.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                NCollection_List<Shape> aLImE = aDMImVE[i];
                if (aLImE.Extent() == 1)
                {
                    aLVAlone.Append(aDMImVE.FindKey(i));
                }
            }
            //
            if (aLVAlone.Extent() > 1)
            {
                aVW1 = ShapeCasts.Vertex(aLVAlone.First());
                aVW2 = ShapeCasts.Vertex(aLVAlone.Last());
                //
                // check distances
                Point3D aPI1_ = Tool.Pnt(aVI1);
                Point3D aPW1 = Tool.Pnt(aVW1);
                Point3D aPW2 = Tool.Pnt(aVW2);
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
        NCollection_List<Shape> aLIE1 = theDMVE.FindFromKey(aVI1);
        NCollection_List<Shape> aLIE2 = theDMVE.FindFromKey(aVI2);
        //
        // Find vertices on the original face corresponding to vertices on the offset edge
        //
        // find original edges for both lists
        NCollection_List<Shape> aLOE1 = new(), aLOE2 = new();
        for (i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aLIE = i == 0 ? aLIE1 : aLIE2;
            NCollection_List<Shape> aLOE = i == 0 ? aLOE1 : aLOE2;
            //
            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            //
            NCollection_List<Shape>.Iterator aItLIE = new(aLIE);
            for (; aItLIE.More(); aItLIE.Next())
            {
                Shape aEI = aItLIE.Value();
                if (myEdgesOrigins!.IsBound(aEI))
                {
                    NCollection_List<Shape> aLEOrigins = myEdgesOrigins.Find(aEI);
                    //
                    NCollection_List<Shape>.Iterator aItLOE = new(aLEOrigins);
                    for (; aItLOE.More(); aItLOE.Next())
                    {
                        Shape aEO = aItLOE.Value();
                        if (aEO.ShapeType() == ShapeType.Edge && aMFence.Add(aEO))
                        {
                            Shape aEOin = new();
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
            NCollection_List<Shape> aLOE = i == 0 ? aLOE1 : aLOE2;
            ref Vertex aVO = ref (i == 0 ? ref aVO1 : ref aVO2);

            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVELoc = new(ShapeHasher.Instance);
            for (NCollection_List<Shape>.Iterator itLOE = new(aLOE); itLOE.More(); itLOE.Next())
            {
                Exploration.MapShapesAndAncestors(itLOE.Value(), ShapeType.Vertex, ShapeType.Edge, aDMVELoc);
            }

            int aNbEMax = 0;
            for (int j = 1; j <= aDMVELoc.Extent(); ++j)
            {
                int aNbE = aDMVELoc[j].Extent();
                if (aNbE > 1 && aNbE > aNbEMax)
                {
                    aVO = ShapeCasts.Vertex(aDMVELoc.FindKey(j));
                    aNbEMax = aNbE;
                }
            }
            if (aVO.IsNull)
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
        Point3D aPI1 = Tool.Pnt(aVI1);
        Point3D aPI2 = Tool.Pnt(aVI2);
        Point3D aPO1 = Tool.Pnt(aVO1);
        Point3D aPO2 = Tool.Pnt(aVO2);
        //
        Vector3D aVI = new(aPI1, aPI2);
        Vector3D aVOv = new(aPO1, aPO2);
        //
        double anAngle = aVI.Angle(aVOv);
        bool bInverted = Math.Abs(anAngle - M_PI) < 1.0e-4;
        if (bInverted)
        {
            NCollection_List<Shape>.Iterator aItLEIm = new(aLEImages);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                Shape aEInvr = aItLEIm.Value();
                myInvertedEdges.Add(aEInvr);
            }
        }
        return bInverted;
    }

    //=======================================================================
    // function : GetVerticesOnEdges
    // purpose  : Get vertices from the given shape belonging to the given edges
    //=======================================================================
    private static void GetVerticesOnEdges(in Shape theCB, NCollection_IndexedMap<Shape> theEdges,
                                           NCollection_Map<Shape> theVerticesOnEdges,
                                           NCollection_Map<Shape> theAllVertices)
    {
        Explorer aExp = new(theCB, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Shape aE = aExp.Current();
            bool isOnGivenEdges = theEdges.Contains(aE);
            for (Iterator aItV = new(aE); aItV.More(); aItV.Next())
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
    private bool CheckInvertedBlock(in Shape theCB, NCollection_List<Shape> theLCBF,
                                    NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMCBVInverted,
                                    NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMCBVAll)
    {
        // For possible removal of the block:
        // 1. There should be more than just one face in the block
        if (theCB.ChildCount < 2)
        {
            return false;
        }
        //
        // 2. The block should at least contain two connected inverted edges with
        //    different origins (not just two images/splits of the same edge)
        NCollection_Map<Shape> aMECBInv = new(ShapeHasher.Instance);
        Compound aCECBInv = new();
        new BrepBuilder().MakeCompound(ref aCECBInv);
        //
        Explorer aExp = new(theCB, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Shape aE = aExp.Current();
            if (myInvertedEdges.Contains(aE))
            {
                if (aMECBInv.Add(aE))
                {
                    new BrepBuilder().Add(aCECBInv, aE);
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
        NCollection_List<Shape> aLCBE = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCECBInv, ShapeType.Vertex, ShapeType.Edge, aLCBE);
        //
        NCollection_List<Shape>.Iterator aItLCBE = new(aLCBE);
        for (; aItLCBE.More(); aItLCBE.Next())
        {
            Shape aCBE = aItLCBE.Value();
            // count the unique edges in the block
            int aNbUnique = 0;
            NCollection_Map<Shape> aMEOrigins = new(ShapeHasher.Instance);
            Iterator aItE = new(aCBE);
            for (; aItE.More(); aItE.Next())
            {
                Shape aE = aItE.Value();
                if (!myOEOrigins.Find(aE, out NCollection_List<Shape> pLEOr))
                {
                    aMEOrigins.Add(aE);
                    ++aNbUnique;
                    continue;
                }
                NCollection_List<Shape>.Iterator aItLEOr = new(pLEOr);
                for (; aItLEOr.More(); aItLEOr.Next())
                {
                    Shape aEOr = aItLEOr.Value();
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
        theDMCBVInverted.Find(theCB, out NCollection_Map<Shape>? pMVInverted);
        theDMCBVAll.Find(theCB, out NCollection_Map<Shape>? pMVAll);
        if (pMVInverted is null)
        {
            pMVInverted = new NCollection_Map<Shape>(ShapeHasher.Instance);
            theDMCBVInverted.Bind(theCB, pMVInverted);
            pMVAll = new NCollection_Map<Shape>(ShapeHasher.Instance);
            theDMCBVAll.Bind(theCB, pMVAll);
            //
            GetVerticesOnEdges(theCB, myInvertedEdges, pMVInverted, pMVAll);
        }
        //
        NCollection_List<Shape>.Iterator aItLCB1 = new(theLCBF);
        for (; aItLCB1.More(); aItLCB1.Next())
        {
            Shape aCB1 = aItLCB1.Value();
            if (aCB1.IsSame(theCB))
            {
                continue;
            }
            //
            // collect vertices from inverted edges
            theDMCBVInverted.Find(aCB1, out NCollection_Map<Shape>? pMVInverted1);
            theDMCBVAll.Find(aCB1, out NCollection_Map<Shape>? pMVAll1);
            if (pMVInverted1 is null)
            {
                pMVInverted1 = new NCollection_Map<Shape>(ShapeHasher.Instance);
                theDMCBVInverted.Bind(aCB1, pMVInverted1);
                pMVAll1 = new NCollection_Map<Shape>(ShapeHasher.Instance);
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
    private void RemoveInvalidSplitsByInvertedEdges(NCollection_IndexedMap<Shape> theMERemoved)
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
        BrepBuilder aBB = new();
        NCollection_IndexedMap<Shape> aMEAvoid = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMVF = new(ShapeHasher.Instance);
        int aNb = myOFImages.Extent(), i;
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<Shape> aLFIm = myOFImages[i];
            //
            Compound aCFIm = new();
            aBB.MakeCompound(ref aCFIm);
            //
            NCollection_DataMap<Shape, NCollection_List<Shape>> aDMEF = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator aIt = new(aLFIm);
            for (; aIt.More(); aIt.Next())
            {
                Shape aF = aIt.Value();
                aBB.Add(aCFIm, aF);
                //
                // make a map to use only outer edges
                Explorer aExp = new(aF, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    //
                    if (!aDMEF.Find(aE, out NCollection_List<Shape> pLF))
                    {
                        pLF = new NCollection_List<Shape>();
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
                aExp.Init(aF, ShapeType.Vertex);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aV = aExp.Current();
                    //
                    if (!aDMVF.Find(aV, out NCollection_List<Shape> pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        aDMVF.Bind(aV, pLF);
                    }
                    AppendToList(pLF, aF);
                }
            }
            //
            // for the splits to be regular they should form only one block
            NCollection_List<Shape> aLCBF = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCFIm, ShapeType.Edge, ShapeType.Face, aLCBF);
            if (aLCBF.Extent() == 1)
            {
                continue;
            }
            //
            // check if the inverted edges create the irregularity
            NCollection_DataMap<Shape, NCollection_Map<Shape>> aDMCBVInverted = new(ShapeHasher.Instance),
                                                                             aDMCBVAll = new(ShapeHasher.Instance);
            //
            NCollection_List<Shape>.Iterator aItLCB = new(aLCBF);
            for (; aItLCB.More(); aItLCB.Next())
            {
                Shape aCB = aItLCB.Value();
                //
                // check if it is possible to remove the block
                if (!CheckInvertedBlock(aCB, aLCBF, aDMCBVInverted, aDMCBVAll))
                {
                    // non of the edges in this block should be removed
                    Exploration.MapShapes(aCB, ShapeType.Edge, aMEAvoid);
                    continue;
                }
            }
        }
        //
        // all edges not included in aMEAvoid can be removed
        NCollection_Map<Shape> aMERem = new(ShapeHasher.Instance);
        for (int iInverted = 1; iInverted <= myInvertedEdges.Extent(); ++iInverted)
        {
            Shape aE = myInvertedEdges[iInverted];
            if (!aMEAvoid.Contains(aE))
            {
                Iterator aIt = new(aE);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aV = aIt.Value();
                    if (aDMVF.Find(aV, out NCollection_List<Shape> pLF) && (pLF.Extent() > 3))
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
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aInvFaces = new(ShapeHasher.Instance);
        NCollection_Map<Shape> aMFRem = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMFToUpdate = new(ShapeHasher.Instance);
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aF = myInvalidFaces.FindKey(i);
            NCollection_List<Shape> aLFIm = myInvalidFaces[i];
            //
            NCollection_List<Shape>.Iterator aIt = new(aLFIm);
            for (; aIt.More();)
            {
                Shape aFIm = aIt.Value();
                //
                // to be removed the face should have at least two not connected
                // inverted edges
                Compound aCEInv = new();
                aBB.MakeCompound(ref aCEInv);
                Explorer aExp = new(aFIm, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    if (aMERem.Contains(aE))
                    {
                        aBB.Add(aCEInv, aE);
                    }
                }
                //
                // check connectivity
                NCollection_List<Shape> aLCBE = new();
                BOPTools_AlgoTools.MakeConnexityBlocks(aCEInv, ShapeType.Vertex, ShapeType.Edge, aLCBE);
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
                aInvFaces.Add(aF, new NCollection_List<Shape>(aLFIm));
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
            Shape aF = aMFToUpdate[i];
            NCollection_List<Shape> aLFIm = myOFImages.ChangeFromKey(aF);
            //
            NCollection_List<Shape>.Iterator aIt = new(aLFIm);
            for (; aIt.More();)
            {
                Shape aFIm = aIt.Value();
                if (aMFRem.Contains(aFIm))
                {
                    Exploration.MapShapes(aFIm, ShapeType.Edge, theMERemoved);
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
    private void RemoveInvalidSplitsFromValid(NCollection_DataMap<Shape, NCollection_Map<Shape>> theDMFMVIE)
    {
        // Decide whether to remove the found invalid faces or not.
        // The procedure is the following:
        // 1. Make connexity blocks from invalid faces;
        // 2. Find free edges in this blocks;
        // 3. If all free edges are valid for the faces - remove block.
        //
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance), aMFToRem = new(ShapeHasher.Instance);
        Compound aCFInv = new();
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aCFInv);
        NCollection_List<Shape>.Iterator aItLF = new();
        //
        // make compound of invalid faces
        NCollection_DataMap<Shape, Shape> aDMIFOF = new(ShapeHasher.Instance);
        int i, aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aF = myInvalidFaces.FindKey(i);
            // artificially invalid faces should not be removed
            if (myArtInvalidFaces.IsBound(aF))
            {
                continue;
            }
            NCollection_List<Shape> aLFInv = myInvalidFaces[i];
            aItLF.Initialize(aLFInv);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape aFIm = aItLF.Value();
                if (aMFence.Add(aFIm))
                {
                    aBB.Add(aCFInv, aFIm);
                    aDMIFOF.Bind(aFIm, aF);
                }
            }
        }
        //
        // make connexity blocks
        NCollection_List<Shape> aLCBInv = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFInv, ShapeType.Edge, ShapeType.Face, aLCBInv);
        //
        // analyze each block
        aItLF.Initialize(aLCBInv);
        for (; aItLF.More(); aItLF.Next())
        {
            Shape aCB = aItLF.Value();
            //
            // if connexity block contains only one face - it should be removed;
            Explorer aExp = new(aCB, ShapeType.Face);
            aExp.Next();
            if (aExp.More())
            {
                // check if there are valid images left
                aExp.Init(aCB, ShapeType.Face);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aFIm = aExp.Current();
                    Shape aF = aDMIFOF.Find(aFIm);
                    //
                    NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(aF);
                    NCollection_List<Shape> aLFInv = myInvalidFaces.FindFromKey(aF);
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
                aExp.Init(aCB, ShapeType.Face);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aF = aExp.Current();
                    aMFToRem.Add(aF);
                }
                continue;
            }
            //
            // remove faces connected by inverted edges
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEF = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aCB, ShapeType.Edge, ShapeType.Face, aDMEF);
            //
            NCollection_DataMap<Shape, NCollection_List<Shape>> aDMFF = new(ShapeHasher.Instance);
            aExp.Init(aCB, ShapeType.Face);
            for (; aExp.More(); aExp.Next())
            {
                Shape aFCB = aExp.Current();
                Shape aF = aDMIFOF.Find(aFCB);
                if (!aDMFF.Find(aF, out NCollection_List<Shape> pList))
                {
                    pList = new NCollection_List<Shape>();
                    aDMFF.Bind(aF, pList);
                }
                pList.Append(aFCB);
            }

            // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itM in aDMFF)
            {
                Shape aF = itM.Key;
                theDMFMVIE.Find(aF, out NCollection_Map<Shape>? pValidInverted);

                // either remove all of these faces or none.
                NCollection_List<Shape> aLFCB = itM.Value;
                NCollection_List<Shape>.Iterator itL = new(aLFCB);
                for (; itL.More(); itL.Next())
                {
                    Shape aFCB = itL.Value();
                    Explorer aExpE = new(aFCB, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape aECB = aExpE.Current();
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
                Shape aF = myInvalidFaces.FindKey(i);
                NCollection_List<Shape> aLFImages = myOFImages.ChangeFromKey(aF);
                aItLF.Initialize(aLFImages);
                for (; aItLF.More();)
                {
                    Shape aFIm = aItLF.Value();
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

    private static void buildPairs(NCollection_IndexedMap<Shape> theSMap,
                                   NCollection_DataMap<Shape, NCollection_Map<Shape>> theIntPairs)
    {
        int aNbS = theSMap.Extent();
        if (aNbS < 2)
        {
            return;
        }
        for (int it1 = 1; it1 <= aNbS; ++it1)
        {
            Shape aS = theSMap[it1];
            if (!theIntPairs.IsBound(aS))
            {
                theIntPairs.Bind(aS, new NCollection_Map<Shape>(ShapeHasher.Instance));
            }
        }

        for (int it1 = 1; it1 <= aNbS; ++it1)
        {
            Shape aS1 = theSMap[it1];
            NCollection_Map<Shape> aMap1 = theIntPairs[aS1];
            for (int it2 = it1 + 1; it2 <= aNbS; ++it2)
            {
                Shape aS2 = theSMap[it2];
                aMap1.Add(aS2);
                theIntPairs[aS2].Add(aS1);
            }
        }
    }

    //=================================================================================================

    private static void buildIntersectionPairs(
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myOFImages,
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myInvalidFaces, BOPAlgo_Builder theBuilder,
        NCollection_Map<Shape> theMFRemoved, NCollection_DataMap<Shape, Shape> theFOrigins,
        NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_Map<Shape>>> theIntPairs)
    {
        ShapeType aCType = ShapeType.Vertex;
        // Build connection map from vertices to faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theBuilder.InternalShape(), aCType, ShapeType.Face, aDMVF);

        NCollection_DataMap<Shape, NCollection_List<Shape>> anImages = theBuilder.Images();
        NCollection_DataMap<Shape, NCollection_List<Shape>> anOrigins = theBuilder.Origins();

        // Find all faces connected to the not removed faces and build intersection pairs among them.
        // For removed faces intersect only those connected to each other.

        for (int iF = 1; iF <= myInvalidFaces.Extent(); ++iF)
        {
            Shape aFInv = myInvalidFaces.FindKey(iF);

            Compound aCF = new(), aCFRem = new();
            new BrepBuilder().MakeCompound(ref aCF);
            new BrepBuilder().MakeCompound(ref aCFRem);

            for (int iC = 0; iC < 2; ++iC)
            {
                NCollection_List<Shape> aLF = iC == 0 ? myInvalidFaces[iF] : myOFImages.FindFromKey(aFInv);

                for (NCollection_List<Shape>.Iterator it = new(aLF); it.More(); it.Next())
                {
                    NCollection_List<Shape> aLFIm = new();
                    TakeModified(it.Value(), anImages, aLFIm);

                    for (NCollection_List<Shape>.Iterator itIm = new(aLFIm); itIm.More(); itIm.Next())
                    {
                        Shape aFIm = itIm.Value();

                        if (theMFRemoved.Contains(aFIm))
                        {
                            new BrepBuilder().Add(aCFRem, aFIm);
                        }
                        else
                        {
                            new BrepBuilder().Add(aCF, aFIm);
                        }
                    }
                }
            }

            NCollection_List<Shape> aLCB = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCF, ShapeType.Edge, ShapeType.Face, aLCB);

            if (aLCB.IsEmpty())
            {
                continue;
            }

            NCollection_DataMap<Shape, NCollection_Map<Shape>> pFInterMap = new(ShapeHasher.Instance);
            theIntPairs.Bind(aFInv, pFInterMap);

            // build pairs for not removed faces
            for (NCollection_List<Shape>.Iterator itCB = new(aLCB); itCB.More(); itCB.Next())
            {
                Shape aCB = itCB.Value();

                NCollection_IndexedMap<Shape> aMFInter = new(ShapeHasher.Instance);
                for (Explorer exp = new(aCB, aCType); exp.More(); exp.Next())
                {
                    Shape aCS = exp.Current();
                    if (!aDMVF.FindFromKey(aCS, out NCollection_List<Shape> pLFV))
                    {
                        continue;
                    }

                    for (NCollection_List<Shape>.Iterator itFV = new(pLFV); itFV.More(); itFV.Next())
                    {
                        Shape aFConnected = itFV.Value();

                        NCollection_List<Shape> aLFOr = new();
                        TakeModified(aFConnected, anOrigins, aLFOr);
                        for (NCollection_List<Shape>.Iterator itOr = new(aLFOr); itOr.More(); itOr.Next())
                        {
                            if (theFOrigins.Find(itOr.Value(), out Shape pFOr))
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
            BOPTools_AlgoTools.MakeConnexityBlocks(aCFRem, ShapeType.Edge, ShapeType.Face, aLCB);

            if (aLCB.IsEmpty())
            {
                continue;
            }

            for (NCollection_List<Shape>.Iterator itCB = new(aLCB); itCB.More(); itCB.Next())
            {
                Shape aCB = itCB.Value();

                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEF = new(ShapeHasher.Instance);
                for (Explorer exp = new(aCB, aCType); exp.More(); exp.Next())
                {
                    Shape aCS = exp.Current();
                    if (!aDMVF.FindFromKey(aCS, out NCollection_List<Shape> pLFV))
                    {
                        continue;
                    }

                    for (NCollection_List<Shape>.Iterator itFV = new(pLFV); itFV.More(); itFV.Next())
                    {
                        Shape aFConnected = itFV.Value();
                        Exploration.MapShapesAndAncestors(aFConnected, ShapeType.Edge, ShapeType.Face, aDMEF);
                    }
                }

                for (int iE = 1; iE <= aDMEF.Extent(); ++iE)
                {
                    NCollection_List<Shape> aLFConnected = aDMEF[iE];
                    if (aLFConnected.Extent() < 2)
                    {
                        continue;
                    }

                    NCollection_IndexedMap<Shape> aMFInter = new(ShapeHasher.Instance);
                    for (NCollection_List<Shape>.Iterator itLF = new(aLFConnected); itLF.More(); itLF.Next())
                    {
                        Shape aFConnected = itLF.Value();

                        NCollection_List<Shape> aLFOr = new();
                        TakeModified(aFConnected, anOrigins, aLFOr);
                        for (NCollection_List<Shape>.Iterator itOr = new(aLFOr); itOr.More(); itOr.Next())
                        {
                            if (theFOrigins.Find(itOr.Value(), out Shape pFOr))
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
    private void RemoveInsideFaces(NCollection_List<Shape> theInvertedFaces,
                                   NCollection_IndexedMap<Shape> theMFToCheckInt,
                                   NCollection_IndexedMap<Shape> theMFInvInHole, in Shape theFHoles,
                                   NCollection_IndexedMap<Shape> theMERemoved,
                                   NCollection_IndexedMap<Shape> theMEInside, ProgressRange theRange)
    {
        NCollection_List<Shape> aLS = new();
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMFInv = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLF = new();
        NCollection_DataMap<Shape, Shape> aDMFImF = new(ShapeHasher.Instance);
        //
        using ProgressScope aPS = new(theRange, "Looking for inside faces", 10);
        int i, aNb = myOFImages.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            if (!aPS.More())
            {
                return;
            }
            Shape aF = myOFImages.FindKey(i);
            // to avoid intersection of the splits of the same
            // offset faces among themselves make compound of the
            // splits and use it as one argument
            Compound aCFImi = new();
            new BrepBuilder().MakeCompound(ref aCFImi);
            //
            for (int j = 0; j < 2; ++j)
            {
                NCollection_List<Shape>? pLFSp = j == 0
                    ? (myInvalidFaces.FindFromKey(aF, out NCollection_List<Shape> aLInv) ? aLInv : null)
                    : myOFImages[i];
                if (pLFSp is null)
                {
                    continue;
                }
                //
                aItLF.Initialize(pLFSp);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape aFIm = aItLF.Value();
                    if (aMFence.Add(aFIm))
                    {
                        new BrepBuilder().Add(aCFImi, aFIm);
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
            Shape aFSp = theMFToCheckInt[i];
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
        Shape aSols = aMV.InternalShape();
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMFS = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aSols, ShapeType.Face, ShapeType.Solid, aDMFS);
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
        NCollection_Map<Shape> aMFToRem = new(ShapeHasher.Instance);
        // Check completeness
        if (aMV.HasDeleted())
        {
            NCollection_IndexedMap<Shape> aMEHoles = new(ShapeHasher.Instance);
            Exploration.MapShapes(theFHoles, ShapeType.Edge, aMEHoles);

            // Map edges of the solids to check the connectivity
            // of the removed invalid splits
            NCollection_IndexedMap<Shape> aMESols = new(ShapeHasher.Instance);
            Exploration.MapShapes(aSols, ShapeType.Edge, aMESols);

            // perform additional check on faces
            aNb = myOFImages.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                if (!aPS.More())
                {
                    return;
                }
                NCollection_List<Shape> aLFIm = myOFImages[i];
                if (aLFIm.IsEmpty())
                {
                    continue;
                }

                Shape aF = myOFImages.FindKey(i);
                bool bInvalid = myInvalidFaces.Contains(aF);
                // For invalid faces it is allowed to be at least connected
                // to the solids, otherwise the solids are considered as broken
                bool bConnected = false;

                bool bFaceKept = false;
                aItLF.Initialize(aLFIm);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape aFIm = aItLF.Value();
                    if (!aMV.IsDeleted(aFIm))
                    {
                        bFaceKept = true;
                        continue;
                    }
                    //
                    Explorer aExpE = new(aFIm, ShapeType.Edge);
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
        NCollection_IndexedMap<Shape> aMEBoundary = new(ShapeHasher.Instance);
        aNb = aDMFS.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aFIm = aDMFS.FindKey(i);
            NCollection_List<Shape> aLSol = aDMFS[i];
            if (aLSol.Extent() > 1)
            {
                aMFToRem.Add(aFIm);
            }
            else if (aFIm.Orientation() != Internal)
            {
                Exploration.MapShapes(aFIm, ShapeType.Edge, aMEBoundary);
            }
        }

        // Tool for getting the splits of faces
        NCollection_DataMap<Shape, NCollection_List<Shape>> aMVIms = aMV.Images();

        // update invalid faces with images
        aNb = aMFInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aFInv = aMFInv[i];
            TakeModified(aFInv, aMVIms, aMFInv);
        }

        // Take into account the faces invalid by inverted edges
        for (NCollection_List<Shape>.Iterator itLF = new(theInvertedFaces); itLF.More(); itLF.Next())
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
            Shape aFInv = theMFInvInHole[i];
            NCollection_List<Shape> aLFInvIm = ModifiedOf(aMV, aFInv);
            if (aLFInvIm.IsEmpty())
            {
                aLFInvIm.Append(aFInv);
            }
            //
            if (!aDMFImF.Find(aFInv, out Shape pFOffset))
            {
                continue;
            }
            NCollection_List<Shape>.Iterator aItLFInv = new(aLFInvIm);
            for (; aItLFInv.More(); aItLFInv.Next())
            {
                Shape aFInvIm = aItLFInv.Value();
                if (!aDMFS.FindFromKey(aFInvIm, out NCollection_List<Shape> pLSols) || pLSols.Extent() != 1)
                {
                    continue;
                }
                //
                Shape aFSol = pLSols.First();
                //
                Shape aFx = new();
                if (!FindShape(aFInvIm, aFSol, null, ref aFx))
                {
                    continue;
                }
                //
                if (BRepOffset_Tool.CheckPlanesNormals(ShapeCasts.Face(aFx), ShapeCasts.Face(pFOffset)))
                {
                    // the normal direction has not changed, thus the face can be removed
                    aMFToRem.Add(aFInvIm);
                }
            }
        }
        //
        Compound aSolids = new();
        new BrepBuilder().MakeCompound(ref aSolids);
        NCollection_Map<Shape> aMFKeep = new(ShapeHasher.Instance);
        //
        Explorer aExpS = new(aSols, ShapeType.Solid);
        for (; aExpS.More(); aExpS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Shape aSol = aExpS.Current();
            //
            bool bAllInv = true, bAllRemoved = true;

            for (Explorer aExpF = new(aSol, ShapeType.Face); aExpF.More(); aExpF.Next())
            {
                Shape aFS = aExpF.Current();
                //
                if (aFS.Orientation() == Internal)
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
                Explorer aExpF = new(aSol, ShapeType.Face);
                for (; aExpF.More(); aExpF.Next())
                {
                    Shape aFS = aExpF.Current();
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
                new BrepBuilder().Add(aSolids, aSol);
                mySolids = aSolids;
            }
        }
        //
        foreach (Shape aItM in aMFKeep)
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
            Shape aE = theMERemoved[i];
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
    private void ShapesConnections(NCollection_DataMap<Shape, Shape> theDMFOr, BOPAlgo_Builder theBuilder)
    {
        // Make connexity blocks from invalid edges to use the whole block
        // to which the edge is connected instead of the single edge.
        Compound aCEInv = new();
        new BrepBuilder().MakeCompound(ref aCEInv);
        for (int i = 1; i <= myInvalidEdges.Extent(); ++i)
        {
            AddToContainer(myInvalidEdges[i], aCEInv);
        }

        NCollection_List<Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCEInv, ShapeType.Vertex, ShapeType.Edge, aLCB);

        // Binding from the edge to the block
        NCollection_DataMap<Shape, Shape> aECBMap = new(ShapeHasher.Instance);
        for (NCollection_List<Shape>.Iterator itCB = new(aLCB); itCB.More(); itCB.Next())
        {
            for (Iterator itE = new(itCB.Value()); itE.More(); itE.Next())
            {
                aECBMap.Bind(itE.Value(), itCB.Value());
            }
        }

        // update invalid edges with images and keep connection to original edge
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMEOr = new(ShapeHasher.Instance);
        int aNb = myInvalidEdges.Extent();
        for (int i = 1; i <= aNb; ++i)
        {
            Shape aEInv = myInvalidEdges[i];
            NCollection_List<Shape> aLEIm = ModifiedOf(theBuilder, aEInv);
            if (aLEIm.IsEmpty())
            {
                NCollection_List<Shape> aL = new();
                aL.Append(aEInv);
                aDMEOr.Bind(aEInv, aL);
                continue;
            }
            //
            NCollection_List<Shape>.Iterator aItLEIm = new(aLEIm);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                Shape aEIm = aItLEIm.Value();
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
            Shape aFIm1 = pDS.Shape(aFF.Index1());
            Shape aFIm2 = pDS.Shape(aFF.Index2());
            //
            if (!theDMFOr.Find(aFIm1, out Shape pF1) || !theDMFOr.Find(aFIm2, out Shape pF2))
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
            NCollection_Map<Shape> aMEInt = new(ShapeHasher.Instance);
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
                        Shape aEInt = pDS.Shape(nEInt);
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
                Shape aF = i == 0 ? pF1 : pF2;
                Shape aFOp = i == 0 ? pF2 : pF1;
                Shape aFIm = i == 0 ? aFIm1 : aFIm2;
                //
                bool bFound = false;
                //
                NCollection_List<Shape> aLFIm = ModifiedOf(theBuilder, aFIm);
                if (aLFIm.IsEmpty())
                {
                    aLFIm.Append(aFIm);
                }
                //
                NCollection_List<Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    Shape aFImIm = aItLFIm.Value();
                    //
                    bool bInv = false, bInt = false;
                    Explorer aExpE = new(aFImIm, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape aE = aExpE.Current();
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
                    aExpE.Init(aFImIm, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape aE = aExpE.Current();
                        if (!aDMEOr.Find(aE, out NCollection_List<Shape> pLEOr))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape>.Iterator aItLE = new(pLEOr);
                        for (; aItLE.More(); aItLE.Next())
                        {
                            Shape aEOr = aItLE.Value();
                            if (!mySSInterfs.Find(aEOr, out NCollection_List<Shape> pLFE))
                            {
                                pLFE = new NCollection_List<Shape>();
                                mySSInterfs.Bind(aEOr, pLFE);
                            }
                            AppendToList(pLFE, aFOp);
                        }
                    }
                }
                if (bFound)
                {
                    // save connection between offset faces
                    if (!mySSInterfs.Find(aF, out NCollection_List<Shape> pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        mySSInterfs.Bind(aF, pLF);
                    }
                    AppendToList(pLF, aFOp);
                }
            }

            // Treatment for the artificial case - check if one of the faces is artificially invalid
            for (int iF = 0; iF < 2; ++iF)
            {
                Shape aFArt = iF == 0 ? pF1 : pF2;
                Shape aFOpposite = iF == 0 ? pF2 : pF1;

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
                NCollection_IndexedMap<Shape> aMEAvoid = new(ShapeHasher.Instance);
                if (myArtInvalidFaces.Find(aFOpposite, out NCollection_IndexedMap<Shape> pFEInv))
                {
                    for (int iE = 1; iE <= pFEInv.Extent(); ++iE)
                    {
                        if (aECBMap.Find(pFEInv.FindKey(iE), out Shape pCB))
                        {
                            Exploration.MapShapes(pCB, ShapeType.Edge, aMEAvoid);
                        }
                    }
                }
                else if (myOFImages.FindFromKey(aFOpposite, out NCollection_List<Shape> pLFIm))
                {
                    for (NCollection_List<Shape>.Iterator itLFIm = new(pLFIm); itLFIm.More(); itLFIm.Next())
                    {
                        for (Explorer expE = new(itLFIm.Value(), ShapeType.Edge); expE.More(); expE.Next())
                        {
                            if (aECBMap.Find(expE.Current(), out Shape pCB))
                            {
                                Exploration.MapShapes(pCB, ShapeType.Edge, aMEAvoid);
                            }
                        }
                    }
                }

                Shape aFArtIm = iF == 0 ? aFIm1 : aFIm2;
                Shape aFOppositeIm = iF == 0 ? aFIm2 : aFIm1;

                // Check if there are any intersections between edges of artificially
                // invalid face and opposite face
                int nFOp = pDS.Index(aFOppositeIm);

                for (Explorer expE = new(aFArtIm, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Shape aE = expE.Current();
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

                    NCollection_List<Shape> aLV = new();
                    NCollection_DynamicArray<BOPDS_InterfEF> aEFs = pDS.InterfEF();
                    for (int iEF = 0; iEF < aEFs.Length(); ++iEF)
                    {
                        BOPDS_InterfEF aEF = aEFs[iEF];
                        if (aEF.Contains(nE) && aEF.Contains(nFOp))
                        {
                            if (aEF.CommonPart().Type() == ShapeType.Vertex)
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
                    Explorer expEOp = new(aFOppositeIm, ShapeType.Edge);
                    for (; expEOp.More(); expEOp.Next())
                    {
                        Shape aEOp = expEOp.Current();
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
                    for (NCollection_List<Shape>.Iterator itLV = new(aLV); itLV.More(); itLV.Next())
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
    private void RemoveHangingParts(BOPAlgo_MakerVolume theMV, NCollection_DataMap<Shape, Shape> theDMFImF,
                                    NCollection_IndexedMap<Shape> theMFInv, NCollection_Map<Shape> theMFToRem)
    {
        // Map the faces of the result solids to filter them from avoided faces
        NCollection_IndexedMap<Shape> aMFS = new(ShapeHasher.Instance);
        Exploration.MapShapes(theMV.InternalShape(), ShapeType.Face, aMFS);

        BrepBuilder aBB = new();
        // Build compound of all faces not included into solids
        Compound aCFHangs = new();
        aBB.MakeCompound(ref aCFHangs);

        // Tool for getting the splits of faces
        NCollection_DataMap<Shape, NCollection_List<Shape>> aMVIms = theMV.Images();

        NCollection_List<Shape>.Iterator aItLArgs = new(theMV.Arguments());
        for (; aItLArgs.More(); aItLArgs.Next())
        {
            Explorer anExpF0 = new(aItLArgs.Value(), ShapeType.Face);
            for (; anExpF0.More(); anExpF0.Next())
            {
                Shape aF = anExpF0.Current();
                TakeModified(aF, aMVIms, aCFHangs, aMFS);
            }
        }

        // Make connexity blocks of all hanging parts and check that they are isolated
        NCollection_List<Shape> aLCBHangs = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFHangs, ShapeType.Edge, ShapeType.Face, aLCBHangs);
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
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEF = new(ShapeHasher.Instance),
                                                                                 aDMVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theMV.InternalShape(), ShapeType.Edge, ShapeType.Face, aDMEF);
        Exploration.MapShapesAndAncestors(theMV.InternalShape(), ShapeType.Vertex, ShapeType.Edge, aDMVE);

        // Update invalid edges with intersection results
        NCollection_Map<Shape> aMEInv = new(ShapeHasher.Instance);
        int i, aNbE = myInvalidEdges.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            TakeModified(myInvalidEdges[i], aMVIms, aMEInv);
        }

        // Update inverted edges with intersection results
        NCollection_Map<Shape> aMEInverted = new(ShapeHasher.Instance);
        for (int iInv = 1; iInv <= myInvertedEdges.Extent(); ++iInv)
        {
            TakeModified(myInvertedEdges[iInv], aMVIms, aMEInverted);
        }

        // Tool for getting the origins of the splits
        NCollection_DataMap<Shape, NCollection_List<Shape>> aMVOrs = theMV.Origins();

        // Find hanging blocks to remove
        NCollection_List<Shape> aBlocksToRemove = new();

        NCollection_List<Shape>.Iterator aItLCBH = new(aLCBHangs);
        for (; aItLCBH.More(); aItLCBH.Next())
        {
            Shape aCBH = aItLCBH.Value();

            // Remove the block containing the inverted edges
            bool bHasInverted = false;
            Explorer anExpE = new(aCBH, ShapeType.Edge);
            for (; anExpE.More() && !bHasInverted; anExpE.Next())
            {
                Shape aE = anExpE.Current();
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
            NCollection_IndexedMap<Shape> aBlockME = new(ShapeHasher.Instance);
            Exploration.MapShapes(aCBH, ShapeType.Edge, aBlockME);
            // Map to collect all original faces
            NCollection_Map<Shape> aMOffsetF = new(ShapeHasher.Instance);

            Explorer anExpF = new(aCBH, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                Shape aF = anExpF.Current();
                // Check block to contain invalid face
                if (!bHasInvalidFace)
                {
                    bHasInvalidFace = theMFInv.Contains(aF);
                }

                // Check block for connectivity to invalid parts
                if (!bIsConnected)
                {
                    // check edges
                    anExpE.Init(aF, ShapeType.Edge);
                    for (; anExpE.More() && !bIsConnected; anExpE.Next())
                    {
                        Shape aE = anExpE.Current();
                        if (aDMEF.FindFromKey(aE, out NCollection_List<Shape> pLF))
                        {
                            NCollection_List<Shape>.Iterator aItLF = new(pLF);
                            for (; aItLF.More() && !bIsConnected; aItLF.Next())
                            {
                                bIsConnected = theMFInv.Contains(aItLF.Value());
                            }
                        }
                    }
                    // check vertices
                    if (!bIsConnected)
                    {
                        Explorer anExpV = new(aF, ShapeType.Vertex);
                        for (; anExpV.More() && !bIsConnected; anExpV.Next())
                        {
                            Shape aV = anExpV.Current();
                            if (aDMVE.FindFromKey(aV, out NCollection_List<Shape> pLE))
                            {
                                NCollection_List<Shape>.Iterator aItLE = new(pLE);
                                for (; aItLE.More() && !bIsConnected; aItLE.Next())
                                {
                                    bIsConnected = !aBlockME.Contains(aItLE.Value()) && aMEInv.Contains(aItLE.Value());
                                }
                            }
                        }
                    }
                }

                // Check block to be isolated
                if (aMVOrs.Find(aF, out NCollection_List<Shape> pLFOr))
                {
                    NCollection_List<Shape>.Iterator aItLFOr = new(pLFOr);
                    for (; aItLFOr.More(); aItLFOr.Next())
                    {
                        if (theDMFImF.Find(aItLFOr.Value(), out Shape pFOffset))
                        {
                            aMOffsetF.Add(pFOffset);
                        }
                    }
                }
                else
                {
                    if (theDMFImF.Find(aF, out Shape pFOffset))
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
            Shape aCBH = aItLCBH.Value();
            Explorer anExpF = new(aCBH, ShapeType.Face);
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
    private void RemoveValidSplits(NCollection_Map<Shape> theSpRem, BOPAlgo_Builder theGF,
                                   NCollection_IndexedMap<Shape> theMERemoved)
    {
        int i, aNb = myOFImages.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<Shape> aLSIm = myOFImages[i];
            NCollection_List<Shape>.Iterator aIt = new(aLSIm);
            for (; aIt.More();)
            {
                Shape aSIm = aIt.Value();
                if (theSpRem.Contains(aSIm))
                {
                    Exploration.MapShapes(aSIm, ShapeType.Edge, theMERemoved);
                    aLSIm.Remove(aIt);
                    continue;
                }
                //
                // check if all its images are have to be removed
                NCollection_List<Shape> aLSImIm = ModifiedOf(theGF, aSIm);
                if (aLSImIm.Extent() != 0)
                {
                    bool bAllRem = true;
                    NCollection_List<Shape>.Iterator aIt1 = new(aLSImIm);
                    for (; aIt1.More(); aIt1.Next())
                    {
                        Shape aSImIm = aIt1.Value();
                        if (theSpRem.Contains(aSImIm))
                        {
                            Exploration.MapShapes(aSImIm, ShapeType.Edge, theMERemoved);
                        }
                        else
                        {
                            bAllRem = false;
                        }
                    }
                    //
                    if (bAllRem)
                    {
                        Exploration.MapShapes(aSIm, ShapeType.Edge, theMERemoved);
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
    private void RemoveInvalidSplits(NCollection_Map<Shape> theSpRem, BOPAlgo_Builder theGF,
                                     NCollection_IndexedMap<Shape> theMERemoved)
    {
        int i, aNb = myInvalidFaces.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        for (i = 1; i <= aNb; ++i)
        {
            Shape aS = myInvalidFaces.FindKey(i);
            bool bArt = myArtInvalidFaces.IsBound(aS);
            //
            NCollection_List<Shape> aLSIm = myInvalidFaces[i];
            NCollection_List<Shape>.Iterator aIt = new(aLSIm);
            for (; aIt.More();)
            {
                Shape aSIm = aIt.Value();
                if (theSpRem.Contains(aSIm))
                {
                    Exploration.MapShapes(aSIm, ShapeType.Edge, theMERemoved);
                    aLSIm.Remove(aIt);
                    continue;
                }
                //
                // check if all its images are have to be removed
                NCollection_List<Shape> aLSImIm = ModifiedOf(theGF, aSIm);
                if (aLSImIm.IsEmpty())
                {
                    aIt.Next();
                    continue;
                }
                //
                bool bAllRem = true;
                NCollection_IndexedMap<Shape> aMERemoved = new(ShapeHasher.Instance);
                NCollection_List<Shape>.Iterator aIt1 = new(aLSImIm);
                for (; aIt1.More(); aIt1.Next())
                {
                    Shape aSImIm = aIt1.Value();
                    if (theSpRem.Contains(aSImIm))
                    {
                        Exploration.MapShapes(aSImIm, ShapeType.Edge, aMERemoved);
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
                Explorer aExpE = new(aSIm, ShapeType.Edge);
                for (; aExpE.More(); aExpE.Next())
                {
                    Shape aEInv = aExpE.Current();
                    if (myInvalidEdges.Contains(aEInv) && !aMERemoved.Contains(aEInv))
                    {
                        break;
                    }
                }
                if (!aExpE.More())
                {
                    Exploration.MapShapes(aSIm, ShapeType.Edge, theMERemoved);
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
    private void FilterEdgesImages(in Shape theS)
    {
        // map edges
        NCollection_IndexedMap<Shape> aME = new(ShapeHasher.Instance);
        Exploration.MapShapes(theS, ShapeType.Edge, aME);
        //
        myOEOrigins.Clear();
        // OCCT.NET: OCCT iterates the map by its edges' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> aItDM in myOEImages)
        {
            Shape aE = aItDM.Key;
            NCollection_List<Shape> aLEIm = aItDM.Value;
            //
            NCollection_List<Shape>.Iterator aIt = new(aLEIm);
            for (; aIt.More();)
            {
                Shape aEIm = aIt.Value();
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
                    NCollection_List<Shape> aLOr = new();
                    aLOr.Append(aE);
                    myOEOrigins.Bind(aEIm, aLOr);
                }
                //
                aIt.Next();
            }
        }
    }

    // OCCT's copy of an IndexedDataMap of lists (a value copy: the lists are copied too).
    private static NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> CopyOf(
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMap)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aCopy = new(ShapeHasher.Instance);
        for (int i = 1; i <= theMap.Extent(); ++i)
        {
            aCopy.Add(theMap.FindKey(i), new NCollection_List<Shape>(theMap[i]));
        }
        return aCopy;
    }

    //=======================================================================
    // function : FilterInvalidFaces
    // purpose  : Filtering of the invalid faces
    //=======================================================================
    private void FilterInvalidFaces(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theDMEF,
                                    NCollection_IndexedMap<Shape> theMERemoved)
    {
        //
        // filter invalid faces, considering faces having only valid
        // images left with non-free edges as valid
        // do not remove invalid faces if it creates free edges
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aReallyInvFaces = new(ShapeHasher.Instance);
        // Edge-Face connexity map of all splits, both invalid and valid
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMEFAll = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLF = new();
        //
        int aNb = myInvalidFaces.Extent();
        for (int i = 1; i <= aNb; ++i)
        {
            Shape aF = myInvalidFaces.FindKey(i);
            NCollection_List<Shape> aLFInv = myInvalidFaces[i];
            //
            if (myArtInvalidFaces.IsBound(aF))
            {
                if (aLFInv.IsEmpty())
                {
                    myArtInvalidFaces.UnBind(aF);
                }
                else
                {
                    aReallyInvFaces.Add(aF, new NCollection_List<Shape>(aLFInv));
                }
                continue;
            }
            //
            if (aLFInv.IsEmpty())
            {
                continue;
            }
            //
            NCollection_List<Shape> aLFIm = myOFImages.ChangeFromKey(aF);
            bool bInvalid = aLFIm.IsEmpty();
            //
            if (!bInvalid)
            {
                // check two lists on common splits
                aItLF.Initialize(aLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape aFInv = aItLF.Value();
                    //
                    NCollection_List<Shape>.Iterator aItLFIm = new(aLFIm);
                    for (; aItLFIm.More(); aItLFIm.Next())
                    {
                        Shape aFIm = aItLFIm.Value();
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
                    NCollection_List<Shape> aLI = j == 0 ? aLFIm : aLFInv;
                    aItLF.Initialize(aLI);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        Shape aFIm = aItLF.Value();
                        //
                        Explorer aExp = new(aFIm, ShapeType.Edge);
                        for (; aExp.More(); aExp.Next())
                        {
                            Shape aE = aExp.Current();
                            if (!theMERemoved.Contains(aE))
                            {
                                if (theDMEF.FindFromKey(aE, out NCollection_List<Shape> pLEF) && pLEF.Extent() == 1)
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
                        for (NCollection_List<Shape>.Iterator itLFInv = new(myInvalidFaces[iF]); itLFInv.More(); itLFInv.Next())
                        {
                            Exploration.MapShapesAndAncestors(itLFInv.Value(), ShapeType.Edge, ShapeType.Face, aDMEFAll);
                        }
                    }
                }

                NCollection_Map<Shape> aLocalSplits = new(ShapeHasher.Instance);
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
                    Shape aFIm = aItLF.Value();
                    Explorer aExp = new(aFIm, ShapeType.Edge);
                    for (; aExp.More(); aExp.Next())
                    {
                        Shape aE = aExp.Current();
                        if (myInvalidEdges.Contains(aE) && !theMERemoved.Contains(aE))
                        {
                            NCollection_List<Shape> aLF = aDMEFAll.FindFromKey(aE);
                            NCollection_List<Shape>.Iterator itLF = new(aLF);
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
                aReallyInvFaces.Add(aF, new NCollection_List<Shape>(aLFInv));
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
            Shape aF = myInvalidFaces.FindKey(i);
            if (myArtInvalidFaces.IsBound(aF))
            {
                continue;
            }

            NCollection_List<Shape> aLFIm = myInvalidFaces[i];
            for (NCollection_List<Shape>.Iterator it = new(aLFIm); it.More(); it.Next())
            {
                Shape aFIm = it.Value();
                for (Explorer expE = new(aFIm, ShapeType.Edge); expE.More(); expE.Next())
                {
                    Shape aE = expE.Current();
                    if (myInvalidEdges.Contains(aE) || myValidEdges.Contains(aE))
                    {
                        continue;
                    }

                    // check if this edges is not created from vertex and mark it as invalid
                    if (!myEdgesOrigins!.Find(aE, out NCollection_List<Shape> pLEOr))
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator itLEO = new(pLEOr);
                    for (; itLEO.More(); itLEO.Next())
                    {
                        if (itLEO.Value().ShapeType() != ShapeType.Vertex)
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
    private void FilterInvalidEdges(NCollection_DataMap<Shape, NCollection_IndexedMap<Shape>> theDMFMIE,
                                    NCollection_IndexedMap<Shape> theMERemoved,
                                    NCollection_IndexedMap<Shape> theMEInside,
                                    NCollection_Map<Shape> theMEUseInRebuild)
    {
        Compound aCEInv = new();
        NCollection_IndexedMap<Shape> aMEInv = new(ShapeHasher.Instance);
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aCEInv);
        NCollection_List<Shape>.Iterator aItLF = new();
        //
        int i, aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<Shape> aLFInv = myInvalidFaces[i];
            aItLF.Initialize(aLFInv);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape aFIm = aItLF.Value();
                Exploration.MapShapes(aFIm, ShapeType.Edge, aMEInv);
                //
                Explorer aExpE = new(aFIm, ShapeType.Edge);
                for (; aExpE.More(); aExpE.Next())
                {
                    Shape aE = aExpE.Current();
                    if (myInvalidEdges.Contains(aE))
                    {
                        aBB.Add(aCEInv, aE);
                    }
                }
            }
        }
        //
        // remove edges which have been marked for removal
        NCollection_IndexedMap<Shape> aMEInvToAvoid = new(ShapeHasher.Instance);
        NCollection_List<Shape> aLCBE = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCEInv, ShapeType.Vertex, ShapeType.Edge, aLCBE);
        //
        NCollection_List<Shape>.Iterator aItLCBE = new(aLCBE);
        for (; aItLCBE.More(); aItLCBE.Next())
        {
            Shape aCBE = aItLCBE.Value();
            Explorer aExpCB = new(aCBE, ShapeType.Edge);
            for (; aExpCB.More(); aExpCB.Next())
            {
                Shape aE = aExpCB.Current();
                if (!theMERemoved.Contains(aE))
                {
                    break;
                }
            }
            //
            if (!aExpCB.More())
            {
                Exploration.MapShapes(aCBE, ShapeType.Edge, aMEInvToAvoid);
            }
        }
        //
        NCollection_IndexedMap<Shape> aReallyInvEdges = new(ShapeHasher.Instance);
        //
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aF = myInvalidFaces.FindKey(i);
            if (myArtInvalidFaces.IsBound(aF))
            {
                if (theDMFMIE.Find(aF, out NCollection_IndexedMap<Shape> aMIE))
                {
                    int aNbIE = aMIE.Extent();
                    for (int iE = 1; iE <= aNbIE; ++iE)
                    {
                        Shape aE = aMIE.FindKey(iE);
                        if (aMEInv.Contains(aE) && !aMEInvToAvoid.Contains(aE))
                        {
                            aReallyInvEdges.Add(aE);
                        }
                    }
                }
            }
            else
            {
                NCollection_List<Shape> aLFInv = myInvalidFaces[i];
                aItLF.Initialize(aLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape aFIm = aItLF.Value();
                    Explorer aExpE = new(aFIm, ShapeType.Edge);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        Shape aE = aExpE.Current();
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
            Shape aE = myInvalidEdges[i];
            if (!theMEInside.Contains(aE) || !myValidEdges.Contains(aE))
            {
                continue;
            }

            if (!myOEOrigins.Find(aE, out NCollection_List<Shape> pEOrigins))
            {
                continue;
            }

            bool bHasInvOutside = false;
            for (NCollection_List<Shape>.Iterator itEOr = new(pEOrigins); !bHasInvOutside && itEOr.More(); itEOr.Next())
            {
                if (myOEImages.Find(itEOr.Value(), out NCollection_List<Shape> pEIms))
                {
                    for (NCollection_List<Shape>.Iterator itEIms = new(pEIms); !bHasInvOutside && itEIms.More(); itEIms.Next())
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
        NCollection_List<Shape>.Iterator aItLF = new();
        NCollection_List<Shape> aLEValid = new();
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance), aMEReb = new(ShapeHasher.Instance),
                                      aMFReb = new(ShapeHasher.Instance);
        Explorer aExp = new();
        //
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMFLV = new(ShapeHasher.Instance);
        // get edges from invalid faces
        aNb = myInvalidFaces.Extent();
        for (i = 1; i <= aNb; i++)
        {
            Shape aF = myInvalidFaces.FindKey(i);
            aMFence.Clear();
            NCollection_List<Shape> aLVAvoid = new();
            NCollection_List<Shape> aLFIm = myInvalidFaces[i];
            aItLF.Initialize(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape aFIm = aItLF.Value();
                aExp.Init(aFIm, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    aMEReb.Add(aE);
                    if (myInvalidEdges.Contains(aE))
                    {
                        Explorer aExpV = new(aE, ShapeType.Vertex);
                        for (; aExpV.More(); aExpV.Next())
                        {
                            Shape aV = aExpV.Current();
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
            NCollection_List<Shape>? pLF = null;
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
                NCollection_List<Shape>.Iterator aItLFE = new(pLF);
                for (; aItLFE.More(); aItLFE.Next())
                {
                    Shape aFE = aItLFE.Value();
                    aMFReb.Add(aFE);
                }
            }
        }
        //
        // get face to rebuild
        aNb = myOFImages.Extent();
        for (i = 1; i <= aNb; i++)
        {
            Shape aF = myOFImages.FindKey(i);
            NCollection_List<Shape> aLFIm = myOFImages[i];
            NCollection_Map<Shape> aMVAvoid = new(ShapeHasher.Instance);
            if (aDMFLV.IsBound(aF))
            {
                NCollection_List<Shape> aLVAvoid = aDMFLV.Find(aF);
                NCollection_List<Shape>.Iterator aItLV = new(aLVAvoid);
                for (; aItLV.More(); aItLV.Next())
                {
                    Shape aV = aItLV.Value();
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
                Shape aFIm = aItLF.Value();
                aExp.Init(aFIm, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Edge anEIm = ShapeCasts.Edge(aExp.Current());
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
                        Explorer aExpV = new(anEIm, ShapeType.Vertex);
                        for (; aExpV.More() && !bRebuild; aExpV.Next())
                        {
                            Shape aV = aExpV.Current();
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
                myFacesToRebuild.Add(aF, new NCollection_List<Shape>(aLEValid));
            }
        }
    }

    //=======================================================================
    // function : mapShapes
    // purpose  : Collect theVecShapes into theMap with theType set
    //=======================================================================
    private static void mapShapes(NCollection_List<Shape> theVecShapes, ShapeType theType,
                                  NCollection_Map<Shape> theMap)
    {
        foreach (Shape aShape in theVecShapes)
        {
            for (Explorer anExp = new(aShape, theType); anExp.More(); anExp.Next())
            {
                theMap.Add(anExp.Current());
            }
        }
    }

    //=======================================================================
    // function : IntersectFaces
    // purpose  : Intersection of the faces that should be rebuild to resolve all invalidities
    //=======================================================================
    private void IntersectFaces(NCollection_Map<Shape> theVertsToAvoid, ProgressRange theRange)
    {
        int aNbFR = myFacesToRebuild.Extent();
        if (aNbFR == 0)
        {
            return;
        }

        using ProgressScope aPSOuter = new(theRange, "Rebuilding invalid faces", 10);
        //
        int i, j, k, aNbInv;
        NCollection_List<Shape>.Iterator aItLF = new(), aItLE = new();
        //
        // get vertices from invalid edges
        NCollection_Map<Shape> aMVInv = new(ShapeHasher.Instance), aMVInvAll = new(ShapeHasher.Instance);
        aNbInv = myInvalidEdges.Extent();
        for (i = 1; i <= aNbInv; ++i)
        {
            Shape aEInv = myInvalidEdges[i];
            bool bValid = myValidEdges.Contains(aEInv);
            for (Explorer aExp = new(aEInv, ShapeType.Vertex); aExp.More(); aExp.Next())
            {
                Shape aV = aExp.Current();
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
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMSF = new(ShapeHasher.Instance),
                                                                          aMDone = new(ShapeHasher.Instance),
                                                                          aMEInfETrim = new(ShapeHasher.Instance),
                                                                          aDMVEFull = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aFLE = new(ShapeHasher.Instance),
                                                                                 aDMEFInv = new(ShapeHasher.Instance);
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
        NCollection_Map<Shape> aMVRInv = new(theVertsToAvoid);
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
        NCollection_DataMap<Shape, Shape> aDMFImF = new(ShapeHasher.Instance);
        Compound aCFArt = new();
        new BrepBuilder().MakeCompound(ref aCFArt);
        // OCCT.NET: OCCT iterates the map by its faces' addresses; the port in insertion order (decision 1032).
        foreach (KeyValuePair<Shape, NCollection_IndexedMap<Shape>> aItM in myArtInvalidFaces)
        {
            Shape aF = aItM.Key;
            NCollection_List<Shape> aLFInv0 = myInvalidFaces.FindFromKey(aF);
            aItLF.Initialize(aLFInv0);
            for (; aItLF.More(); aItLF.Next())
            {
                new BrepBuilder().Add(aCFArt, aItLF.Value());
                aDMFImF.Bind(aItLF.Value(), aF);
            }
        }
        //
        // make connexity blocks
        NCollection_List<Shape> aLCBArt = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFArt, ShapeType.Vertex, ShapeType.Face, aLCBArt);
        //
        // alone edges
        NCollection_Map<Shape> aMEAlone = new(ShapeHasher.Instance), aMEInvOnArt = new(ShapeHasher.Instance);
        //
        using ProgressScope aPSArt = new(aPSOuter.Next(), null, aLCBArt.Extent());
        NCollection_List<Shape>.Iterator aItLCBArt = new(aLCBArt);
        for (; aItLCBArt.More(); aItLCBArt.Next(), aPSArt.Next())
        {
            if (!aPSArt.More())
            {
                return;
            }
            Shape aCB = aItLCBArt.Value();
            //
            // check if aCB contains splits of only one offset face
            NCollection_Map<Shape> aMFArt = new(ShapeHasher.Instance);
            Explorer aExpF = new(aCB, ShapeType.Face);
            for (; aExpF.More(); aExpF.Next())
            {
                aMFArt.Add(aDMFImF.Find(aExpF.Current()));
            }
            //
            bool bAlone = (aMFArt.Extent() == 1);
            //
            // vertices on invalid edges
            NCollection_Map<Shape> aMVEInv = new(ShapeHasher.Instance);
            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            // edges that should not be marked as alone - edges having same origins as invalid ones
            NCollection_Map<Shape> aMEAvoid = new(ShapeHasher.Instance);
            // map to find alone edges by looking for free vertices
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVEVal = new(ShapeHasher.Instance);
            //
            Explorer aExpE = new(aCB, ShapeType.Edge);
            for (; aExpE.More(); aExpE.Next())
            {
                Shape aE = aExpE.Current();
                if (myInvalidEdges.Contains(aE))
                {
                    aMEInvOnArt.Add(aE);
                    for (Iterator aItV = new(aE); aItV.More(); aItV.Next())
                    {
                        aMVEInv.Add(aItV.Value());
                    }
                    //
                    if (bAlone)
                    {
                        if (myOEOrigins.Find(aE, out NCollection_List<Shape> pLEOr0))
                        {
                            NCollection_List<Shape>.Iterator aItLEOr0 = new(pLEOr0);
                            for (; aItLEOr0.More(); aItLEOr0.Next())
                            {
                                NCollection_List<Shape>.Iterator aItLEIm = new(myOEImages.Find(aItLEOr0.Value()));
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
                    Exploration.MapShapesAndAncestors(aE, ShapeType.Vertex, ShapeType.Edge, aDMVEVal);
                }
            }
            //
            // find edges with free vertices
            int aNbV = aDMVEVal.Extent();
            for (i = 1; i <= aNbV; ++i)
            {
                Shape aV = aDMVEVal.FindKey(i);
                if (!aMVEInv.Contains(aV))
                {
                    continue;
                }
                //
                NCollection_List<Shape> aLEV = aDMVEVal[i];
                if (aLEV.Extent() > 1)
                {
                    continue;
                }
                //
                Shape aE = aLEV.First();
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
                Iterator aItE = new(aE);
                for (; aItE.More(); aItE.Next())
                {
                    Shape aVE = aItE.Value();
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
                if (!myOEOrigins.Find(aE, out NCollection_List<Shape> pLEOr))
                {
                    continue;
                }
                //
                NCollection_List<Shape>.Iterator aItLEOr = new(pLEOr);
                for (; aItLEOr.More(); aItLEOr.Next())
                {
                    Shape aEOr = aItLEOr.Value();
                    //
                    NCollection_List<Shape> aLEIm = myOEImages.Find(aEOr);
                    NCollection_List<Shape>.Iterator aItLEIm = new(aLEIm);
                    for (; aItLEIm.More(); aItLEIm.Next())
                    {
                        Shape aEIm = aItLEIm.Value();
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
        NCollection_Map<Shape> aMAllInvs = new(ShapeHasher.Instance);
        aNbInv = myInvalidFaces.Extent();
        for (k = 1; k <= aNbInv; ++k)
        {
            NCollection_List<Shape>.Iterator aIt = new(myInvalidFaces[k]);
            for (; aIt.More(); aIt.Next())
            {
                Explorer aExp = new(aIt.Value(), ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    if (myInvalidEdges.Contains(aE) || aMEAlone.Contains(aE))
                    {
                        aMAllInvs.Add(aE);
                        Iterator aItV = new(aE);
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
        NCollection_Map<Shape> aMVBounds = new(ShapeHasher.Instance);
        //
        NCollection_Map<Shape> aMECheckExt = new(ShapeHasher.Instance);
        // Save connections between not trimmed edge and its trimmed parts
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMEETrim = new(ShapeHasher.Instance);
        // Splits of the new edges
        NCollection_DataMap<Shape, NCollection_List<Shape>> aEImages = new(ShapeHasher.Instance);
        BrepBuilder aBB = new();

        // Keep connection between blocks of invalid edges to the lists of
        // found edges to be intersected for its treatment
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMOENEdges = new(ShapeHasher.Instance);

        aNbInv = myInvalidFaces.Extent();
        using ProgressScope aPSInter = new(aPSOuter.Next(5), null, aNbInv);
        for (k = 1; k <= aNbInv; ++k)
        {
            if (!aPSInter.More())
            {
                return;
            }
            Shape aFInv = myInvalidFaces.FindKey(k);
            bool bSelfRebAvoid = myFSelfRebAvoid.Contains(aFInv);
            NCollection_List<Shape> aLFInv = myInvalidFaces[k];
            //
            NCollection_List<Shape> aLCB = new();
            if (aLFInv.Extent() > 1)
            {
                // make compound of invalid faces
                Compound aCFInv = new();
                aBB.MakeCompound(ref aCFInv);
                //
                NCollection_List<Shape>.Iterator aIt = new(aLFInv);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aFIm = aIt.Value();
                    aBB.Add(aCFInv, aFIm);
                }
                //
                // make connexity blocks
                BOPTools_AlgoTools.MakeConnexityBlocks(aCFInv, ShapeType.Edge, ShapeType.Face, aLCB);
            }
            else
            {
                aLCB.Assign(aLFInv);
            }
            //
            using ProgressScope aPSCB = new(aPSInter.Next(), null, aLCB.Extent());
            bool bArtificial = myArtInvalidFaces.IsBound(aFInv);
            NCollection_List<Shape>.Iterator aItLCB = new(aLCB);
            for (; aItLCB.More(); aItLCB.Next())
            {
                if (!aPSCB.More())
                {
                    return;
                }
                Shape aCBInv = aItLCB.Value();
                //
                NCollection_Map<Shape> aMEFence = new(ShapeHasher.Instance);
                //
                Compound aCBE = new();
                aBB.MakeCompound(ref aCBE);
                //
                // remember inside edges and vertices to further check
                NCollection_Map<Shape> anInsideEdges = new(ShapeHasher.Instance);
                NCollection_Map<Shape> anInsideVertices = new(ShapeHasher.Instance);
                Explorer aExp = new(aCBInv, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    if (myInvalidEdges.Contains(aE) || (bArtificial && aMEAlone.Contains(aE)))
                    {
                        if (aMEFence.Add(aE))
                        {
                            aBB.Add(aCBE, aE);
                            if (!myEdgesToAvoid.Contains(aE) && myInvalidEdges.Contains(aE))
                            {
                                anInsideEdges.Add(aE);
                                Iterator anIt = new(aE);
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
                NCollection_List<Shape> aLCBE = new();
                BOPTools_AlgoTools.MakeConnexityBlocks(aCBE, ShapeType.Vertex, ShapeType.Edge, aLCBE);
                //
                using ProgressScope aPSCBE = new(aPSCB.Next(), null, aLCBE.Extent());
                NCollection_List<Shape>.Iterator aItLCBE = new(aLCBE);
                for (; aItLCBE.More(); aItLCBE.Next())
                {
                    if (!aPSCBE.More())
                    {
                        return;
                    }
                    Shape aCBELoc = aItLCBE.Value();
                    //
                    // map of edges and vertices of processing invalidity
                    NCollection_IndexedMap<Shape> aME = new(ShapeHasher.Instance);
                    // map of vertices to trim the new edges
                    Exploration.MapShapes(aCBELoc, ShapeType.Edge, aME);
                    NCollection_IndexedMap<Shape> aMECV = new(aME);
                    Exploration.MapShapes(aCBELoc, ShapeType.Vertex, aME);
                    NCollection_IndexedMap<Shape> aMFInt = new(ShapeHasher.Instance);
                    // additional faces for intersection
                    NCollection_IndexedMap<Shape> aMFIntExt = new(ShapeHasher.Instance);
                    // splits of faces for intersection
                    NCollection_List<Shape> aLFInt = new();
                    // faces to avoid intersection
                    NCollection_IndexedMap<Shape> aMFAvoid = new(ShapeHasher.Instance);
                    //
                    FindFacesForIntersection(aFInv, aME, aDMSF, aMVInvAll, bArtificial, aMFAvoid, aMFInt, aMFIntExt, aLFInt);
                    if (aMFInt.Extent() < 3)
                    {
                        // nothing to intersect
                        aPSCBE.Next();
                        continue;
                    }
                    //
                    myIntersectionPairs.Find(aFInv, out NCollection_DataMap<Shape, NCollection_Map<Shape>>? pMFInter);
                    // intersect the faces, but do not intersect the invalid ones
                    // among each other (except for the artificially invalid faces)
                    NCollection_IndexedMap<Shape> aMEToInt = new(ShapeHasher.Instance);
                    int aNb = aMFInt.Extent();
                    using ProgressScope aPSIntPair = new(aPSCBE.Next(), null, aNb);
                    for (i = 1; i <= aNb; ++i, aPSIntPair.Next())
                    {
                        if (!aPSIntPair.More())
                        {
                            return;
                        }
                        Face aFi = ShapeCasts.Face(aMFInt[i]);
                        if (bSelfRebAvoid && aFi.IsSame(aFInv))
                        {
                            continue;
                        }
                        //
                        if (!myOFImages.FindFromKey(aFi, out NCollection_List<Shape> aLFImi))
                        {
                            continue;
                        }
                        //
                        if (!aFLE.FindFromKey(aFi, out NCollection_List<Shape> aLFEi))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape> aLFDone = aMDone.ChangeFind(aFi);
                        //
                        NCollection_Map<Shape>? pInterFi = null;
                        if (pMFInter is not null)
                        {
                            pMFInter.Find(aFi, out pInterFi);
                        }
                        if (pMFInter is not null && pInterFi is null)
                        {
                            continue;
                        }

                        // create map of edges and vertices for aLFImi
                        NCollection_Map<Shape> aMEVIm = new(ShapeHasher.Instance);
                        mapShapes(aLFImi, ShapeType.Edge, aMEVIm);
                        mapShapes(aLFImi, ShapeType.Vertex, aMEVIm);

                        bool isIContainsE = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideEdges);
                        bool isIContainsV = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideVertices);

                        for (j = i + 1; j <= aNb; ++j)
                        {
                            Face aFj = ShapeCasts.Face(aMFInt[j]);
                            if (bSelfRebAvoid && aFj.IsSame(aFInv))
                            {
                                continue;
                            }
                            //
                            if (pInterFi is not null && !pInterFi.Contains(aFj))
                            {
                                continue;
                            }

                            if (!myOFImages.FindFromKey(aFj, out NCollection_List<Shape> aLFImj))
                            {
                                continue;
                            }
                            //
                            if (!aFLE.FindFromKey(aFj, out NCollection_List<Shape> aLFEj))
                            {
                                continue;
                            }

                            // create map of edges and vertices for aLFImi
                            aMEVIm.Clear();
                            mapShapes(aLFImj, ShapeType.Edge, aMEVIm);
                            mapShapes(aLFImj, ShapeType.Vertex, aMEVIm);
                            // check images of both faces contain anInsideEdges and anInsideVertices
                            // not process if false and true
                            bool isJContainsE = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideEdges);
                            bool isJContainsV = NCollection_MapAlgo.HasIntersection(aMEVIm, anInsideVertices);

                            // Check if one face is connected to inside edge then
                            // the other must be also connected
                            if ((isIContainsE && !isJContainsV) || (isJContainsE && !isIContainsV))
                            {
                                NCollection_List<Shape> aLVC = new();
                                // it is necessary to process the images if they already have
                                // common vertices
                                FindCommonParts(aLFImi, aLFImj, aLVC, ShapeType.Vertex);

                                if (aLVC.IsEmpty())
                                {
                                    continue;
                                }
                            }
                            //
                            // if there are some common edges between faces
                            // we should use these edges and do not intersect again.
                            NCollection_List<Shape> aLEC = new();
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
                                NCollection_IndexedMap<Shape> aMVOnCE = new(ShapeHasher.Instance);
                                for (NCollection_List<Shape>.Iterator itE = new(aLEC); itE.More(); itE.Next())
                                {
                                    Exploration.MapShapes(itE.Value(), ShapeType.Vertex, aMVOnCE);
                                }

                                NCollection_List<Shape> aLEV = new();
                                FindCommonParts(aLFImi, aLFImj, aLEV, ShapeType.Vertex);
                                for (NCollection_List<Shape>.Iterator itV = new(aLEV); itV.More(); itV.Next())
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
                                Shape aF = aItLE.Value();
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
                        Shape aEInt = aMEToInt[iE];
                        Explorer anExpE = new(aCBELoc, ShapeType.Edge);
                        for (; anExpE.More(); anExpE.Next())
                        {
                            Shape aE = anExpE.Current();
                            if (!aDMOENEdges.FindFromKey(aE, out NCollection_List<Shape> pLEToInt))
                            {
                                pLEToInt = aDMOENEdges[aDMOENEdges.Add(aE, new NCollection_List<Shape>())];
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
                                             NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theFLE,
                                             NCollection_DataMap<Shape, NCollection_List<Shape>> theMDone,
                                             NCollection_DataMap<Shape, NCollection_List<Shape>> theDMSF,
                                             NCollection_DataMap<Shape, NCollection_List<Shape>> theMEInfETrim,
                                             NCollection_DataMap<Shape, NCollection_List<Shape>> theDMVEFull,
                                             NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theDMEFInv)
    {
        int i, aNb = myFacesToRebuild.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aF = myFacesToRebuild.FindKey(i);
            //
            // OCCT binds copies of the same empty list.
            theFLE.Add(aF, new NCollection_List<Shape>());
            theMDone.Bind(aF, new NCollection_List<Shape>());
            //
            NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(aF);
            NCollection_List<Shape>.Iterator aItLF = new(aLFIm);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape aFIm = aItLF.Value();
                Explorer aExp = new(aFIm, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    // save connection to untrimmed face
                    if (!theDMSF.Find(aE, out NCollection_List<Shape> pLF))
                    {
                        pLF = new NCollection_List<Shape>();
                        theDMSF.Bind(aE, pLF);
                    }
                    AppendToList(pLF, aF);
                    //
                    // save connection to untrimmed edge
                    Shape aEInf = myETrimEInf!.Find(aE);
                    if (!theMEInfETrim.Find(aEInf, out NCollection_List<Shape> pLETrim))
                    {
                        pLETrim = new NCollection_List<Shape>();
                        theMEInfETrim.Bind(aEInf, pLETrim);
                    }
                    AppendToList(pLETrim, aE);
                    //
                    Explorer aExpV = new(aE, ShapeType.Vertex);
                    for (; aExpV.More(); aExpV.Next())
                    {
                        Shape aV = aExpV.Current();
                        // save connection to face
                        if (!theDMSF.Find(aV, out NCollection_List<Shape> pLFV))
                        {
                            pLFV = new NCollection_List<Shape>();
                            theDMSF.Bind(aV, pLFV);
                        }
                        AppendToList(pLFV, aF);
                        //
                        if (theLookVertToAvoid)
                        {
                            // save connection to edges
                            if (!theDMVEFull.Find(aV, out NCollection_List<Shape> pLEV))
                            {
                                pLEV = new NCollection_List<Shape>();
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
                if (!myInvalidFaces.FindFromKey(aF, out NCollection_List<Shape> pLFInv) || myArtInvalidFaces.IsBound(aF))
                {
                    continue;
                }
                //
                aItLF.Initialize(pLFInv);
                for (; aItLF.More(); aItLF.Next())
                {
                    Shape aFInv = aItLF.Value();
                    Explorer aExp = new(aFInv, ShapeType.Edge);
                    for (; aExp.More(); aExp.Next())
                    {
                        Shape aE = aExp.Current();
                        if (!theDMEFInv.FindFromKey(aE, out NCollection_List<Shape> pLF))
                        {
                            pLF = theDMEFInv[theDMEFInv.Add(aE, new NCollection_List<Shape>())];
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
    private void FindVerticesToAvoid(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theDMEFInv,
                                     NCollection_DataMap<Shape, NCollection_List<Shape>> theDMVEFull,
                                     NCollection_Map<Shape> theMVRInv)
    {
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        int i, aNb = theDMEFInv.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            NCollection_List<Shape> aLFInv = theDMEFInv[i];
            if (aLFInv.Extent() == 1)
            {
                continue;
            }
            //
            Shape aE = theDMEFInv.FindKey(i);
            if (!myInvalidEdges.Contains(aE) || myValidEdges.Contains(aE))
            {
                continue;
            }

            if (!aMFence.Add(aE))
            {
                continue;
            }

            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMVEEdges = new(ShapeHasher.Instance);
            // Do not check the splitting vertices, but check only the ending ones
            if (myOEOrigins.Find(aE, out NCollection_List<Shape> pLEOr))
            {
                NCollection_List<Shape>.Iterator aItLEOr = new(pLEOr);
                for (; aItLEOr.More(); aItLEOr.Next())
                {
                    NCollection_List<Shape> aLEIm = myOEImages.Find(aItLEOr.Value());
                    NCollection_List<Shape>.Iterator aItLEIm = new(aLEIm);
                    for (; aItLEIm.More(); aItLEIm.Next())
                    {
                        aMFence.Add(aItLEIm.Value());
                        Exploration.MapShapesAndAncestors(aItLEIm.Value(), ShapeType.Vertex, ShapeType.Edge, aMVEEdges);
                    }
                }
            }
            else
            {
                Exploration.MapShapesAndAncestors(aE, ShapeType.Vertex, ShapeType.Edge, aMVEEdges);
            }

            int j, aNbV = aMVEEdges.Extent();
            for (j = 1; j <= aNbV; ++j)
            {
                if (aMVEEdges[j].Extent() != 1)
                {
                    continue;
                }

                Shape aV = aMVEEdges.FindKey(j);
                if (!aMFence.Add(aV))
                {
                    continue;
                }
                if (!theDMVEFull.Find(aV, out NCollection_List<Shape> pLE))
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
                NCollection_List<Shape>.Iterator aItLE = new(pLE);
                for (; aItLE.More(); aItLE.Next())
                {
                    Shape aEV = aItLE.Value();

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
    private void FindFacesForIntersection(in Shape theFInv, NCollection_IndexedMap<Shape> theME,
                                          NCollection_DataMap<Shape, NCollection_List<Shape>> theDMSF,
                                          NCollection_Map<Shape> theMVInvAll, bool theArtCase,
                                          NCollection_IndexedMap<Shape> theMFAvoid,
                                          NCollection_IndexedMap<Shape> theMFInt,
                                          NCollection_IndexedMap<Shape> theMFIntExt,
                                          NCollection_List<Shape> theLFImInt)
    {
        int i, aNbE = theME.Extent();
        //
        NCollection_IndexedMap<Shape> aMShapes = new(ShapeHasher.Instance);
        //
        for (i = 1; i <= aNbE; ++i)
        {
            Shape aS = theME[i];
            if (!theDMSF.IsBound(aS))
            {
                continue;
            }
            //
            // in artificial case we intersect the faces which are close to invalidity
            bool bAvoid = theArtCase ? ((aS.ShapeType() == ShapeType.Vertex) && !theMVInvAll.Contains(aS)) : false;
            //
            NCollection_List<Shape> aLF0 = theDMSF.Find(aS);
            NCollection_List<Shape>.Iterator aItLF0 = new(aLF0);
            for (; aItLF0.More(); aItLF0.Next())
            {
                Shape aF = aItLF0.Value();
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
                NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(aF);
                NCollection_List<Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    Shape aFIm = aItLFIm.Value();
                    theLFImInt.Append(aFIm);
                    if (bUse)
                    {
                        Exploration.MapShapes(aFIm, ShapeType.Edge, aMShapes);
                    }
                }
            }
        }
        //
        NCollection_DataMap<Shape, NCollection_List<Shape>> aSSInterfsMap = theArtCase ? mySSInterfsArt : mySSInterfs;
        if (!aSSInterfsMap.Find(theFInv, out NCollection_List<Shape> pLFInv))
        {
            return;
        }
        //
        NCollection_Map<Shape> aMF = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLF = new(pLFInv);
        for (; aItLF.More(); aItLF.Next())
        {
            Shape aF = aItLF.Value();
            aMF.Add(aF);
        }
        //
        // the faces should be unique in each place
        Compound aCF = new();
        new BrepBuilder().MakeCompound(ref aCF);
        //
        NCollection_IndexedMap<Shape> aMFToAdd = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> aDMFOr = new(ShapeHasher.Instance);
        //
        for (i = 1; i <= aNbE; ++i)
        {
            Shape aS = theME[i];
            if (!aSSInterfsMap.Find(aS, out NCollection_List<Shape> pLF))
            {
                continue;
            }
            //
            aItLF.Initialize(pLF);
            for (; aItLF.More(); aItLF.Next())
            {
                Shape aF = aItLF.Value();
                if (theMFInt.Contains(aF) || aMFToAdd.Contains(aF) || !aMF.Contains(aF))
                {
                    continue;
                }
                //
                // check if the face has some connection to already added for intersection faces
                NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(aF);
                if (!theArtCase)
                {
                    NCollection_List<Shape>.Iterator aItLFIm0 = new(aLFIm);
                    for (; aItLFIm0.More(); aItLFIm0.Next())
                    {
                        Shape aFIm = aItLFIm0.Value();
                        Explorer aExp = new(aFIm, ShapeType.Edge);
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
                NCollection_List<Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    Shape aFIm = aItLFIm.Value();
                    aDMFOr.Bind(aFIm, aF);
                    new BrepBuilder().Add(aCF, aFIm);
                }
            }
        }
        //
        if (aMFToAdd.IsEmpty())
        {
            return;
        }
        //
        NCollection_List<Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCF, ShapeType.Edge, ShapeType.Face, aLCB);
        //
        if ((aLCB.Extent() == 1) && (aMFToAdd.Extent() > 1))
        {
            return;
        }
        //
        NCollection_List<Shape>.Iterator aItLCB = new(aLCB);
        for (; aItLCB.More(); aItLCB.Next())
        {
            Shape aCB = aItLCB.Value();
            aMFToAdd.Clear();
            Explorer aExpF = new(aCB, ShapeType.Face);
            for (; aExpF.More(); aExpF.Next())
            {
                Shape aFIm = aExpF.Current();
                aMFToAdd.Add(aDMFOr.Find(aFIm));
            }
            //
            if (aMFToAdd.Extent() == 1)
            {
                Shape aF = aMFToAdd[1];
                //
                theMFInt.Add(aF);
                theMFIntExt.Add(aF);
                //
                NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(aF);
                NCollection_List<Shape>.Iterator aItLFIm = new(aLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    Shape aFIm = aItLFIm.Value();
                    theLFImInt.Append(aFIm);
                }
            }
        }
    }

    //=======================================================================
    // function : ProcessCommonEdges
    // purpose  : Analyzing the common edges between splits of offset faces
    //=======================================================================
    private void ProcessCommonEdges(NCollection_List<Shape> theLEC, NCollection_IndexedMap<Shape> theME,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> theMEInfETrim,
                                    NCollection_Map<Shape> theAllInvs, bool theForceUse,
                                    NCollection_IndexedMap<Shape> theMECV, NCollection_Map<Shape> theMECheckExt,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEETrim,
                                    NCollection_List<Shape> theLFEi, NCollection_List<Shape> theLFEj,
                                    NCollection_IndexedMap<Shape> theMEToInt)
    {
        NCollection_List<Shape> aLEC = new();
        // process common edges
        NCollection_List<Shape>.Iterator aItLE = new(theLEC);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aEC = aItLE.Value();
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
            Iterator aItV = new(aEC);
            for (; aItV.More(); aItV.Next())
            {
                Shape aVE = aItV.Value();
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
                    Shape aEC = aItLE.Value();
                    // check if all images of the origin of this edge
                    // are not connected to any invalidity
                    Shape aEInt0 = myETrimEInf!.Find(aEC);
                    NCollection_List<Shape> aLVE = theMEInfETrim.Find(aEInt0);
                    NCollection_List<Shape>.Iterator aItLVE = new(aLVE);
                    for (; aItLVE.More(); aItLVE.Next())
                    {
                        Shape aECx = aItLVE.Value();
                        if (theAllInvs.Contains(aECx) || myInvalidEdges.Contains(aECx))
                        {
                            return;
                        }
                        //
                        Iterator aItV = new(aECx);
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
            Shape aEC = aItLE.Value();
            //
            Shape aEInt = myETrimEInf!.Find(aEC);
            if (!bUseOnlyInf)
            {
                // find the edges of the same original edge
                // and take their vertices as well
                NCollection_List<Shape> aLVE = theMEInfETrim.Find(aEInt);
                NCollection_List<Shape>.Iterator aItLVE = new(aLVE);
                for (; aItLVE.More(); aItLVE.Next())
                {
                    Shape aECx = aItLVE.Value();
                    //
                    if (!myOEOrigins.Find(aECx, out NCollection_List<Shape> pLEOr) || (pLEOr.Extent() == 1))
                    {
                        Exploration.MapShapes(aECx, ShapeType.Vertex, theMECV);
                    }
                }
                //
                // bind unlimited edge to its trimmed part in face to update maps of
                // images and origins in the future
                if (!theDMEETrim.Find(aEInt, out NCollection_List<Shape> pLTAdded))
                {
                    pLTAdded = new NCollection_List<Shape>();
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

    private static void FindOrigins(NCollection_List<Shape> theLFIm1, NCollection_List<Shape> theLFIm2,
                                    NCollection_IndexedMap<Shape> theME,
                                    NCollection_DataMap<Shape, NCollection_List<Shape>> theOrigins,
                                    NCollection_List<Shape> theLEOr)
    {
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aLF = i == 0 ? theLFIm1 : theLFIm2;
            NCollection_List<Shape>.Iterator aIt = new(aLF);
            for (; aIt.More(); aIt.Next())
            {
                Shape aF = aIt.Value();
                //
                Explorer aExp = new(aF, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    //
                    if (theME.Contains(aE) && theOrigins.IsBound(aE))
                    {
                        NCollection_List<Shape> aLEOr = theOrigins.Find(aE);
                        NCollection_List<Shape>.Iterator aItLE = new(aLEOr);
                        for (; aItLE.More(); aItLE.Next())
                        {
                            Shape aEOr = aItLE.Value();
                            //
                            if (aMFence.Add(aEOr) && (aEOr.ShapeType() == ShapeType.Edge))
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
    private void UpdateIntersectedFaces(in Shape theFInv, in Shape theFi, in Shape theFj,
                                        NCollection_List<Shape> theLFInv, NCollection_List<Shape> theLFImi,
                                        NCollection_List<Shape> theLFImj, NCollection_List<Shape> theLFEi,
                                        NCollection_List<Shape> theLFEj, NCollection_IndexedMap<Shape> theMEToInt)
    {
        // Find common edges in these two lists
        NCollection_Map<Shape> aMEi = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItLE = new(theLFEi);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aE = aItLE.Value();
            aMEi.Add(aE);
        }
        //
        // find origins
        NCollection_IndexedMap<Shape> aMEToFindOrigins = new(ShapeHasher.Instance);
        NCollection_List<Shape> aLEToFindOrigins = new();
        if (!theFi.IsSame(theFInv))
        {
            FindCommonParts(theLFImi, theLFInv, aLEToFindOrigins);
        }
        if (!theFj.IsSame(theFInv))
        {
            FindCommonParts(theLFImj, theLFInv, aLEToFindOrigins);
        }
        //
        NCollection_List<Shape> aLEOrInit = new();
        aItLE.Initialize(aLEToFindOrigins);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aEC = aItLE.Value();
            aMEToFindOrigins.Add(aEC);
        }
        //
        FindOrigins(theLFImi, theLFImj, aMEToFindOrigins, myEdgesOrigins!, aLEOrInit);
        //
        aItLE.Initialize(theLFEj);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aE = aItLE.Value();
            if (aMEi.Contains(aE))
            {
                theMEToInt.Add(aE);
                if (aLEOrInit.Extent() != 0)
                {
                    if (myEdgesOrigins!.IsBound(aE))
                    {
                        NCollection_List<Shape> aLEOr = myEdgesOrigins.ChangeFind(aE);
                        NCollection_List<Shape>.Iterator aItLEOr = new(aLEOrInit);
                        for (; aItLEOr.More(); aItLEOr.Next())
                        {
                            Shape aEOr = aItLEOr.Value();
                            AppendToList(aLEOr, aEOr);
                        }
                    }
                    else
                    {
                        myEdgesOrigins.Bind(aE, new NCollection_List<Shape>(aLEOrInit));
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IntersectFaces
    // purpose  : Intersection of the pair of faces
    //=======================================================================
    private void IntersectFaces(in Shape theFInv, in Shape theFi, in Shape theFj,
                                NCollection_List<Shape> theLFInv, NCollection_List<Shape> theLFImi,
                                NCollection_List<Shape> theLFImj, NCollection_List<Shape> theLFEi,
                                NCollection_List<Shape> theLFEj, NCollection_IndexedMap<Shape> theMECV,
                                NCollection_IndexedMap<Shape> theMEToInt)
    {
        // intersect faces
        Containment aSide = Out;
        NCollection_List<Shape> aLInt1 = new(), aLInt2 = new();
        Edge aNullEdge = new();
        Face aNullFace = new();
        BRepOffset_Tool.Inter3D(ShapeCasts.Face(theFi), ShapeCasts.Face(theFj), aLInt1, aLInt2, aSide, aNullEdge, aNullFace,
                                aNullFace);
        //
        if (aLInt1.IsEmpty())
        {
            return;
        }
        //
        // find common vertices for trimming edges
        NCollection_List<Shape> aLCV = new();
        NCollection_List<Shape>.Iterator aItLE = new();
        FindCommonParts(theLFImi, theLFImj, aLCV, ShapeType.Vertex);
        if (aLCV.Extent() > 1)
        {
            aItLE.Initialize(aLCV);
            for (; aItLE.More(); aItLE.Next())
            {
                Shape aCV = aItLE.Value();
                theMECV.Add(aCV);
            }
        }
        //
        // find origins
        NCollection_IndexedMap<Shape> aMEToFindOrigins = new(ShapeHasher.Instance);
        NCollection_List<Shape> aLEToFindOrigins = new();
        if (!theFi.IsSame(theFInv))
        {
            FindCommonParts(theLFImi, theLFInv, aLEToFindOrigins);
        }
        if (!theFj.IsSame(theFInv))
        {
            FindCommonParts(theLFImj, theLFInv, aLEToFindOrigins);
        }
        NCollection_List<Shape> aLEOrInit = new();
        aItLE.Initialize(aLEToFindOrigins);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aEC = aItLE.Value();
            aMEToFindOrigins.Add(aEC);
        }
        //
        FindOrigins(theLFImi, theLFImj, aMEToFindOrigins, myEdgesOrigins!, aLEOrInit);
        //
        aItLE.Initialize(aLInt1);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aEInt = aItLE.Value();
            theLFEi.Append(aEInt);
            theLFEj.Append(aEInt);
            //
            if (aLEOrInit.Extent() != 0)
            {
                myEdgesOrigins!.Bind(aEInt, new NCollection_List<Shape>(aLEOrInit));
            }
            //
            theMEToInt.Add(aEInt);
        }
    }

    //=======================================================================
    // function : IntersectAndTrimEdges
    // purpose  : Intersection of the new intersection edges among themselves
    //=======================================================================
    private void IntersectAndTrimEdges(NCollection_IndexedMap<Shape> theMFInt, NCollection_IndexedMap<Shape> theMEInt,
                                       NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEETrim,
                                       NCollection_IndexedMap<Shape> theMSInv, NCollection_IndexedMap<Shape> theMVE,
                                       NCollection_Map<Shape> theVertsToAvoid,
                                       NCollection_Map<Shape> theNewVertsToAvoid,
                                       NCollection_Map<Shape> theMECheckExt,
                                       NCollection_DataMap<Shape, NCollection_List<Shape>>? theSSInterfs,
                                       NCollection_Map<Shape> theMVBounds,
                                       NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages)
    {
        int i, aNb = theMEInt.Extent();
        if (aNb == 0)
        {
            return;
        }
        //
        NCollection_List<Shape> aLArgs = new();
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aIt = new(), aIt1 = new();
        Explorer aExp = new();
        //
        // get vertices from the splits of intersected faces.
        // vertices are taken from the edges close to invalidity
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVE = new(ShapeHasher.Instance);
        aNb = theMFInt.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aF = theMFInt[i];
            NCollection_List<Shape> aLE = myFacesToRebuild.FindFromKey(aF);
            //
            aIt.Initialize(aLE);
            for (; aIt.More(); aIt.Next())
            {
                Shape aE = aIt.Value();
                Exploration.MapShapesAndAncestors(aE, ShapeType.Vertex, ShapeType.Edge, aDMVE);
                //
                aExp.Init(aE, ShapeType.Vertex);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aV1 = aExp.Current();
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
            Shape aS = theMSInv[i];
            // edge case
            if (theSSInterfs is not null)
            {
                if (theSSInterfs.Find(aS, out NCollection_List<Shape> pLV))
                {
                    // Add vertices from intersection info to trim section edges of artificial faces
                    for (NCollection_List<Shape>.Iterator itLV = new(pLV); itLV.More(); itLV.Next())
                    {
                        if (itLV.Value().ShapeType() == ShapeType.Vertex)
                        {
                            aLArgs.Append(itLV.Value());
                        }
                    }
                }
            }

            // vertex case
            if (aDMVE.FindFromKey(aS, out NCollection_List<Shape> pLVE))
            {
                aIt.Initialize(pLVE);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aE = aIt.Value();
                    //
                    aExp.Init(aE, ShapeType.Vertex);
                    for (; aExp.More(); aExp.Next())
                    {
                        Shape aV1 = aExp.Current();
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
        NCollection_List<Shape> aLVBounds = new();
        // new intersection edges
        NCollection_List<Shape> aLENew = new();
        // get edges to intersect
        NCollection_List<Shape> aLEInt = new();
        // Common intersection edges. Should be intersected separately
        NCollection_List<Shape> aLCE = new();
        //
        aNb = theMEInt.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aE = theMEInt[i];
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
            aExp.Init(aE, ShapeType.Vertex);
            for (; aExp.More(); aExp.Next())
            {
                Shape aV = aExp.Current();
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
            Shape aV = aIt.Value();
            NCollection_List<Shape> aLVIm = ModifiedOf(aGF, aV);
            if (aLVIm.IsEmpty())
            {
                theMVBounds.Add(aV);
            }
            else
            {
                Shape aVIm = aLVIm.First();
                theMVBounds.Add(aVIm);
            }
        }
        //
        // find invalid splits of edges
        NCollection_Map<Shape> aMEInv = new(ShapeHasher.Instance);
        GetInvalidEdges(theNewVertsToAvoid, theMVBounds, aGF, aMEInv);
        //
        BrepBuilder aBB = new();
        // get valid splits to intersect with the commons
        Compound aCEIm = new();
        aBB.MakeCompound(ref aCEIm);
        //
        // remove the splits containing vertices from invalid edges
        aIt.Initialize(aLEInt);
        for (; aIt.More(); aIt.Next())
        {
            Shape aE = aIt.Value();
            //
            NCollection_List<Shape> aLEIm = ModifiedOf(aGF, aE);
            if (aLEIm.IsEmpty())
            {
                continue;
            }
            //
            aIt1.Initialize(aLEIm);
            for (; aIt1.More();)
            {
                Shape aEIm = aIt1.Value();
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
                if (!theEImages.Find(aE, out NCollection_List<Shape> pLEIm))
                {
                    pLEIm = new NCollection_List<Shape>();
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
        NCollection_List<Shape>.Iterator aItLCE = new(aLCE);
        for (; aItLCE.More(); aItLCE.Next())
        {
            Shape aE = aItLCE.Value();
            NCollection_List<Shape> aLEIm = ModifiedOf(aGFCE, aE);
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
                        Shape aEC = pDS.Shape(aItLPBC.Value().OriginalEdge());
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
            if (!theEImages.Find(aE, out NCollection_List<Shape> pLEIm))
            {
                pLEIm = new NCollection_List<Shape>();
                theEImages.Bind(aE, pLEIm);
            }
            pLEIm.Append(aLEIm);
            //
            // save bounding vertices
            for (Iterator aItV = new(aE); aItV.More(); aItV.Next())
            {
                Shape aV = aItV.Value();
                NCollection_List<Shape> aLVIm = ModifiedOf(aGFCE, aV);
                theMVBounds.Add(aLVIm.IsEmpty() ? aV : aLVIm.First());
            }
        }
    }

    //=======================================================================
    // function : GetInvalidEdges
    // purpose  : Looking for the invalid edges by intersecting with invalid vertices
    //=======================================================================
    private void GetInvalidEdges(NCollection_Map<Shape> theVertsToAvoid, NCollection_Map<Shape> theMVBounds,
                                 BOPAlgo_Builder theGF, NCollection_Map<Shape> theMEInv)
    {
        if (theVertsToAvoid.IsEmpty())
        {
            return;
        }
        //
        NCollection_List<Shape>.Iterator aIt = new();
        // get vertices created with intersection edges
        Shape aRes = theGF.InternalShape();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aRes, ShapeType.Vertex, ShapeType.Edge, aDMVE);
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
        NCollection_Map<Shape> aMVInv = new(ShapeHasher.Instance);
        int i, aNb = aDMVE.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Vertex aV = ShapeCasts.Vertex(aDMVE.FindKey(i));
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
            foreach (Shape aItM in theVertsToAvoid)
            {
                Vertex aVInv = ShapeCasts.Vertex(aItM);
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
                NCollection_List<Shape> aLVE = aDMVE.FindFromKey(aV);
                aIt.Initialize(aLVE);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aE = aIt.Value();
                    theMEInv.Add(aE);
                }
            }
        }
    }

    //=======================================================================
    // function : UpdateValidEdges
    // purpose  : Making the new splits and updating the maps
    //=======================================================================
    private void UpdateValidEdges(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theFLE,
                                  NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theOENEdges,
                                  NCollection_Map<Shape> theMVBounds, NCollection_Map<Shape> theMEInvOnArt,
                                  NCollection_Map<Shape> theMECheckExt, NCollection_Map<Shape> theVertsToAvoid,
                                  NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages,
                                  NCollection_DataMap<Shape, NCollection_List<Shape>> theEETrim,
                                  ProgressRange theRange)
    {
        using ProgressScope aPSOuter = new(theRange, "Updating edges", 10);
        // update images and origins of edges, plus update AsDes
        //
        // new edges
        NCollection_List<Shape> aLE = new();
        // back connection from edges to faces
        NCollection_DataMap<Shape, NCollection_List<Shape>> aMELF = new(ShapeHasher.Instance);
        //
        NCollection_Map<Shape> aMETmp = new(ShapeHasher.Instance);
        int i, aNb = theFLE.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Face aF = ShapeCasts.Face(theFLE.FindKey(i));
            //
            NCollection_List<Shape> aLEInt = theFLE[i];
            NCollection_List<Shape>.Iterator aItLE0 = new(aLEInt);
            for (; aItLE0.More(); aItLE0.Next())
            {
                Shape aE = aItLE0.Value();
                if ((theMECheckExt.Contains(aE) || aMETmp.Contains(aE)) && !theEImages.IsBound(aE))
                {
                    theMECheckExt.Remove(aE);
                    aMETmp.Add(aE);
                    continue;
                }
                if (!aMELF.Find(aE, out NCollection_List<Shape> pLF))
                {
                    pLF = new NCollection_List<Shape>();
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
        NCollection_Map<Shape> aMEB = new(ShapeHasher.Instance);
        //
        // new intersection edges
        NCollection_Map<Shape> aMENew = new(ShapeHasher.Instance);
        // map of old vertices
        NCollection_Map<Shape> aMVOld = new(ShapeHasher.Instance);
        // back connection to untrimmed edges
        NCollection_DataMap<Shape, NCollection_List<Shape>> aDMEOr = new(ShapeHasher.Instance);
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

        BrepBuilder aBB = new();

        // Make connexity blocks of the invalid edges
        // and treat each block separately

        // Compound of all invalid edges to make the blocks
        Compound aCEAll = new();
        aBB.MakeCompound(ref aCEAll);

        int aNbE = theOENEdges.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            aBB.Add(aCEAll, theOENEdges.FindKey(i));
        }

        // Separate the edges into blocks
        NCollection_List<Shape> aLBlocks = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCEAll, ShapeType.Vertex, ShapeType.Edge, aLBlocks);

        // Perform intersection of the new splits for each block

        // Intersected splits
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMBlocksSp = new(ShapeHasher.Instance);

        using ProgressScope aPSB = new(aPSOuter.Next(), null, aLBlocks.Extent());
        NCollection_List<Shape>.Iterator aItLB = new(aLBlocks);
        for (; aItLB.More(); aItLB.Next(), aPSB.Next())
        {
            if (!aPSB.More())
            {
                return;
            }
            Shape aBlock = aItLB.Value();

            // Get the list of new edges for the block
            NCollection_List<Shape> aBlockLENew = new();
            {
                // Fence map
                NCollection_Map<Shape> aMEFence = new(ShapeHasher.Instance);
                Explorer anExpE = new(aBlock, ShapeType.Edge);
                for (; anExpE.More(); anExpE.Next())
                {
                    Shape aE = anExpE.Current();
                    NCollection_List<Shape> aLEInt = theOENEdges.FindFromKey(aE);
                    NCollection_List<Shape>.Iterator aItLEInt = new(aLEInt);
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
            NCollection_List<Shape> aLSplits = new();

            NCollection_List<Shape>.Iterator aItLE = new(aBlockLENew);
            for (; aItLE.More(); aItLE.Next())
            {
                Shape aE = aItLE.Value();
                if (!theEImages.Find(aE, out NCollection_List<Shape> pLEIm) || pLEIm.IsEmpty())
                {
                    continue;
                }

                NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    aLSplits.Append(aItLEIm.Value());
                }
            }

            if (aLSplits.IsEmpty())
            {
                continue;
            }

            Shape aCE = new();
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
        NCollection_Map<Shape> aMEVal = new(ShapeHasher.Instance);

        // Blocks of valid edges on the first stage
        NCollection_List<Shape> aLValBlocks = new();

        int aNbB = aMBlocksSp.Extent();
        using ProgressScope aPSBSp = new(aPSOuter.Next(), null, aNbB);
        for (i = 1; i <= aNbB; ++i, aPSBSp.Next())
        {
            if (!aPSBSp.More())
            {
                return;
            }
            Shape aCE = aMBlocksSp.FindKey(i);
            NCollection_List<Shape> aBlockLENew = aMBlocksSp[i];

            // Get all participating faces to get the bounds
            NCollection_List<Shape> aLFaces0 = new();
            NCollection_List<Shape>.Iterator aItLE = new(aBlockLENew);
            for (; aItLE.More(); aItLE.Next())
            {
                if (!aMELF.Find(aItLE.Value(), out NCollection_List<Shape> pLF))
                {
                    continue;
                }
                NCollection_List<Shape>.Iterator aItLF = new(pLF);
                for (; aItLF.More(); aItLF.Next())
                {
                    AppendToList(aLFaces0, aItLF.Value());
                }
            }

            // Localized bounds of the splits of the offset faces
            // to filter the new splits of the current block
            Shape aFilterBounds0 = new();
            GetBounds(aLFaces0, aMEB, ref aFilterBounds0);

            // Filter the splits by bounds
            NCollection_Map<Shape> aMEInvLoc = new(ShapeHasher.Instance);
            GetInvalidEdgesByBounds(aCE, aFilterBounds0, aMVOld, aMENew, aDMEOr, aMELF, theEImages, theMECheckExt, theMEInvOnArt,
                                    theVertsToAvoid, aMEInvLoc);

            // Keep only valid edges of the block
            Compound aCEVal = new();
            aBB.MakeCompound(ref aCEVal);

            bool bKept = false;

            Explorer anExpE = new(aCE, ShapeType.Edge);
            for (; anExpE.More(); anExpE.Next())
            {
                Shape aESp = anExpE.Current();
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
        Shape aSplits1 = new();
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
            Shape aCE = aMBlocksSp.FindKey(i);
            for (Explorer anExp = new(aCE, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                Shape aEIm = anExp.Current();
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
        NCollection_List<Shape> aLFaces = new();
        for (i = 1; i <= myOFImages.Extent(); ++i)
        {
            aLFaces.Append(myOFImages.FindKey(i));
        }

        // Bounds of the splits of the offset faces to filter the new splits
        Shape aFilterBounds = new();
        GetBounds(aLFaces, aMEB, ref aFilterBounds);

        // Filter the splits by intersection with bounds
        NCollection_Map<Shape> aMEInv = new(ShapeHasher.Instance);
        GetInvalidEdgesByBounds(aSplits1, aFilterBounds, aMVOld, aMENew, aDMEOr, aMELF, theEImages, theMECheckExt, theMEInvOnArt,
                                theVertsToAvoid, aMEInv);

        // Filter the images of edges after the second filtering stage
        // and combine all valid edges into a single compound
        Shape aSplits = new();
        FilterSplits(aLE, aMEInv, true, theEImages, ref aSplits);

        aPSOuter.Next();
        if (!aPSOuter.More())
        {
            return;
        }

        // get bounds to update
        // we need to update the edges of all the affected faces
        NCollection_List<Shape> aLF = new();
        // prepare the vertices from new splits of edges
        NCollection_IndexedMap<Shape> aMVSp = new(ShapeHasher.Instance);
        Exploration.MapShapes(aSplits, ShapeType.Vertex, aMVSp);
        //
        int aNbF = myOFImages.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            Shape aF = myOFImages.FindKey(i);
            if (theFLE.Contains(aF))
            {
                aLF.Append(aF);
                continue;
            }
            //
            // check the splits of faces to have vertices from splits
            NCollection_List<Shape> aLFIm = myOFImages[i];
            NCollection_List<Shape>.Iterator aItLFIm = new(aLFIm);
            for (; aItLFIm.More(); aItLFIm.Next())
            {
                Shape aFIm = aItLFIm.Value();
                //
                Explorer aExpV = new(aFIm, ShapeType.Vertex);
                for (; aExpV.More(); aExpV.Next())
                {
                    Shape aV = aExpV.Current();
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
        Shape aBounds = new();
        NCollection_List<Shape> aLAValid = new(), aLABounds = new();
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
        NCollection_IndexedMap<Shape> aNewEdges = new(ShapeHasher.Instance);
        if (aGF.Images().Find(aSplits, out NCollection_List<Shape> pSplitsIm))
        {
            NCollection_List<Shape>.Iterator aItSpIm = new(pSplitsIm);
            for (; aItSpIm.More(); aItSpIm.Next())
            {
                Exploration.MapShapes(aItSpIm.Value(), ShapeType.Edge, aNewEdges);
            }
        }

        Compound anInsideEdges = new();
        new BrepBuilder().MakeCompound(ref anInsideEdges);
        for (int iE = 1; iE <= myInsideEdges.Extent(); ++iE)
        {
            new BrepBuilder().Add(anInsideEdges, myInsideEdges[iE]);
        }

        //
        // Rebuild the map of edges to avoid, using the intersection results
        NCollection_IndexedMap<Shape> aMEAvoid = new(ShapeHasher.Instance);
        Compound aCEAvoid = new();
        new BrepBuilder().MakeCompound(ref aCEAvoid);
        // GF's data structure
        BOPDS_DS pDS = aGF.PDS()!;

        aNbE = myEdgesToAvoid.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            Shape aE = myEdgesToAvoid[i];
            NCollection_List<Shape> aLEIm = ModifiedOf(aGF, aE);

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

            NCollection_List<Shape>.Iterator aItLEIm = new(aLEIm);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                Shape aEIm = aItLEIm.Value();
                if (!aNewEdges.Contains(aEIm))
                {
                    AddToContainer(aEIm, aCEAvoid);
                }
            }
        }

        bool isCut = false;
        if (aCEAvoid.ChildCount > 0)
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
                for (Iterator itCE = new(aCEAvoid); itCE.More(); itCE.Next())
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
            Exploration.MapShapes(aCEAvoid, ShapeType.Edge, aMEAvoid);
        }

        myEdgesToAvoid = aMEAvoid;
    }

    //=================================================================================================

    //! Trims intersection edges
    private void TrimNewIntersectionEdges(NCollection_List<Shape> theLE,
                                          NCollection_DataMap<Shape, NCollection_List<Shape>> theEETrim,
                                          NCollection_Map<Shape> theMVBounds, NCollection_Map<Shape> theMECheckExt,
                                          NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages,
                                          NCollection_Map<Shape> theMEB, NCollection_Map<Shape> theMVOld,
                                          NCollection_Map<Shape> theMENew,
                                          NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEOr,
                                          NCollection_DataMap<Shape, NCollection_List<Shape>> theMELF)
    {
        NCollection_List<Shape>.Iterator aIt = new(), aIt1 = new();
        aIt.Initialize(theLE);
        for (; aIt.More(); aIt.Next())
        {
            Shape aE = aIt.Value();
            //
            bool bCheckExt = theMECheckExt.Remove(aE);
            //
            bool bOld = theEETrim.IsBound(aE);
            if (bOld)
            {
                NCollection_List<Shape> aLET = theEETrim.Find(aE);
                aIt1.Initialize(aLET);
                for (; aIt1.More(); aIt1.Next())
                {
                    Shape aET = aIt1.Value();
                    theMEB.Add(aET);
                    Explorer aExpV = new(aET, ShapeType.Vertex);
                    for (; aExpV.More(); aExpV.Next())
                    {
                        Shape aV = aExpV.Current();
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
            NCollection_List<Shape> aLEIm = theEImages.ChangeFind(aE);
            if (aLEIm.IsEmpty())
            {
                theEImages.UnBind(aE);
                continue;
            }
            //
            Shape aCEIm = new();
            NCollection_Map<Shape> aMEVBounds = new(ShapeHasher.Instance);
            //
            if (aLEIm.Extent() > 1)
            {
                NCollection_IndexedMap<Shape> aMV = new(ShapeHasher.Instance);
                // fuse these parts
                BOPAlgo_Builder aGFE = new();
                NCollection_List<Shape>.Iterator aItLEIm0 = new(aLEIm);
                for (; aItLEIm0.More(); aItLEIm0.Next())
                {
                    Shape aEIm = aItLEIm0.Value();
                    aGFE.AddArgument(aEIm);
                    Exploration.MapShapes(aEIm, ShapeType.Vertex, aMV);
                }
                //
                // add two bounding vertices of this edge to the operation
                Exploration.Vertices(ShapeCasts.Edge(aE), out Vertex aV1, out Vertex aV2);
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
                        Shape aV = aMV[iV];
                        if (theMVBounds.Contains(aV) || aV.IsSame(aV1) || aV.IsSame(aV2))
                        {
                            NCollection_List<Shape> aLVIm = ModifiedOf(aGFE, aV);
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
            Explorer aExp = new(aCEIm, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Shape aEIm = aExp.Current();
                //
                // check the split not to contain bounding vertices
                Iterator aItV = new(aEIm);
                for (; aItV.More(); aItV.Next())
                {
                    Shape aV = aItV.Value();
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
                    NCollection_List<Shape> aLOr = new();
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
                NCollection_List<Shape> aLFE = theMELF.Find(aE);
                NCollection_List<Shape>.Iterator aItLEIm = new(aLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    Shape aEIm = aItLEIm.Value();
                    if (!theMELF.Find(aEIm, out NCollection_List<Shape> pLFEIm))
                    {
                        pLFEIm = new NCollection_List<Shape>();
                        theMELF.Bind(aEIm, pLFEIm);
                    }
                    NCollection_List<Shape>.Iterator aItLF = new(aLFE);
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
    private void IntersectEdges(NCollection_List<Shape> theLA, NCollection_List<Shape> theLE,
                                NCollection_Map<Shape> theMVBounds, NCollection_Map<Shape> theVertsToAvoid,
                                NCollection_Map<Shape> theMENew, NCollection_Map<Shape> theMECheckExt,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEOr,
                                NCollection_DataMap<Shape, NCollection_List<Shape>> theMELF,
                                ref Shape theSplits)
    {
        BOPAlgo_Builder aGFA = new();
        aGFA.SetArguments(theLA);
        aGFA.Perform();
        if (aGFA.HasErrors())
        {
            // just copy input to the result
            Compound aSp0 = new();
            BrepBuilder aBB0 = new();
            aBB0.MakeCompound(ref aSp0);
            NCollection_List<Shape>.Iterator anIt = new(theLA);
            for (; anIt.More(); anIt.Next())
            {
                Shape aE = anIt.Value();
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
        NCollection_List<Shape>.Iterator aIt = new(), aIt1 = new();

        // prepare list of edges to update
        NCollection_List<Shape> aLEInput = new();
        for (aIt.Initialize(theLA); aIt.More(); aIt.Next())
        {
            Explorer anExpE = new(aIt.Value(), ShapeType.Edge);
            for (; anExpE.More(); anExpE.Next())
            {
                aLEInput.Append(anExpE.Current());
            }
        }

        // update new edges
        aIt.Initialize(aLEInput);
        for (; aIt.More(); aIt.Next())
        {
            Shape aE = aIt.Value();
            if (!theMENew.Contains(aE))
            {
                continue;
            }

            NCollection_List<Shape> aLEIm = ModifiedOf(aGFA, aE);
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
            Shape aE = aIt.Value();
            NCollection_List<Shape> aLEIm = ModifiedOf(aGFA, aE);
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
            NCollection_List<Shape> aLFE = theMELF.Find(aE);
            aIt1.Initialize(aLEIm);
            for (; aIt1.More(); aIt1.Next())
            {
                Shape aEIm = aIt1.Value();
                if (!theMELF.Find(aEIm, out NCollection_List<Shape> pLFEIm))
                {
                    pLFEIm = new NCollection_List<Shape>();
                    theMELF.Bind(aEIm, pLFEIm);
                }
                NCollection_List<Shape>.Iterator aItLF = new(aLFE);
                for (; aItLF.More(); aItLF.Next())
                {
                    AppendToList(pLFEIm, aItLF.Value());
                }
            }
        }
        //
        NCollection_Map<Shape> aMEInv = new(ShapeHasher.Instance);
        GetInvalidEdges(theVertsToAvoid, theMVBounds, aGFA, aMEInv);
        if (aMEInv.Extent() != 0)
        {
            // update shape
            Compound aSp = new();
            BrepBuilder aBB = new();
            aBB.MakeCompound(ref aSp);
            Explorer aExp = new(theSplits, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Shape aE = aExp.Current();
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
    private void GetBounds(NCollection_List<Shape> theLFaces, NCollection_Map<Shape> theMEB,
                           ref Shape theBounds)
    {
        BrepBuilder aBB = new();
        // Make compound of edges contained in the splits of faces
        Compound aBounds = new();
        aBB.MakeCompound(ref aBounds);
        // Fence map
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator aItLF = new(theLFaces);
        for (; aItLF.More(); aItLF.Next())
        {
            if (!myOFImages.FindFromKey(aItLF.Value(), out NCollection_List<Shape> pLFIm))
            {
                continue;
            }
            NCollection_List<Shape>.Iterator aIt = new(pLFIm);
            for (; aIt.More(); aIt.Next())
            {
                Shape aFIm = aIt.Value();
                //
                Explorer aExpE = new(aFIm, ShapeType.Edge);
                for (; aExpE.More(); aExpE.Next())
                {
                    Shape aEIm = aExpE.Current();
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
    private void GetBoundsToUpdate(NCollection_List<Shape> theLF, NCollection_Map<Shape> theMEB,
                                   NCollection_List<Shape> theLABounds, NCollection_List<Shape> theLAValid,
                                   ref Shape theBounds)
    {
        // get all edges
        Compound aBounds = new();
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aBounds);
        //
        NCollection_Map<Shape> aMAValid = new(ShapeHasher.Instance), aMFence = new(ShapeHasher.Instance);
        //
        NCollection_List<Shape>.Iterator aItLF = new(theLF);
        for (; aItLF.More(); aItLF.Next())
        {
            Shape aF = aItLF.Value();
            //
            NCollection_IndexedMap<Shape> aMDE = new(ShapeHasher.Instance);
            NCollection_List<Shape> aLFDes = myAsDes!.Descendant(aF);
            NCollection_List<Shape>.Iterator aItLFDes = new(aLFDes);
            for (; aItLFDes.More(); aItLFDes.Next())
            {
                Shape aED = aItLFDes.Value();
                if (!myOEImages.Find(aED, out NCollection_List<Shape> pLEDIm))
                {
                    aMDE.Add(aED);
                    continue;
                }
                //
                NCollection_List<Shape>.Iterator aItLEDIm = new(pLEDIm);
                for (; aItLEDIm.More(); aItLEDIm.Next())
                {
                    Shape aEDIm = aItLEDIm.Value();
                    aMDE.Add(aEDIm);
                }
            }
            //
            int j, aNbE = aMDE.Extent();
            for (j = 1; j <= aNbE; ++j)
            {
                Edge aEIm = ShapeCasts.Edge(aMDE[j]);
                //
                if (!theMEB.Contains(aEIm) && aMFence.Add(aEIm))
                {
                    aBB.Add(aBounds, aEIm);
                    theLABounds.Append(aEIm);
                }
                //
                if (myOEOrigins.Find(aEIm, out NCollection_List<Shape> pLO))
                {
                    NCollection_List<Shape>.Iterator aItLO = new(pLO);
                    for (; aItLO.More(); aItLO.Next())
                    {
                        Shape aEO = aItLO.Value();
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
    private void GetInvalidEdgesByBounds(in Shape theSplits, in Shape theBounds,
                                         NCollection_Map<Shape> theMVOld, NCollection_Map<Shape> theMENew,
                                         NCollection_DataMap<Shape, NCollection_List<Shape>> theDMEOr,
                                         NCollection_DataMap<Shape, NCollection_List<Shape>> theMELF,
                                         NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages,
                                         NCollection_Map<Shape> theMECheckExt, NCollection_Map<Shape> theMEInvOnArt,
                                         NCollection_Map<Shape> theVertsToAvoid, NCollection_Map<Shape> theMEInv)
    {
        // map splits to check the vertices of edges
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theSplits, ShapeType.Vertex, ShapeType.Edge, aDMVE);
        //
        BOPAlgo_Section aSec = new();
        aSec.AddArgument(theSplits);
        aSec.AddArgument(theBounds);
        //
        aSec.Perform();
        //
        // invalid vertices
        NCollection_IndexedMap<Shape> aMVInv = new(ShapeHasher.Instance);
        // vertices to check additionally by classification relatively to solid
        NCollection_Map<Shape> aMVCheckAdd = new(ShapeHasher.Instance);
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
            Shape aE1 = pDS.Shape(aEE.Index1());
            Shape aE2 = pDS.Shape(aEE.Index2());
            //
            if (!aEE.HasIndexNew())
            {
                if (theMECheckExt.Contains(aE1) && (aEE.CommonPart().Type() == ShapeType.Edge))
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
                Exploration.Vertices(ShapeCasts.Edge(aE2), out Vertex aV1, out Vertex aV2);
                if (aDMVE.Contains(aV1) && aDMVE.Contains(aV2))
                {
                    continue;
                }
            }
            //
            // add vertices of all images of the edge from splits for checking
            NCollection_List<Shape> aLEOr = theDMEOr.Find(aE1);
            NCollection_List<Shape>.Iterator aItLEOr = new(aLEOr);
            for (; aItLEOr.More(); aItLEOr.Next())
            {
                Shape aEOr = aItLEOr.Value();
                //
                if (!theEImages.Find(aEOr, out NCollection_List<Shape> pLEIm))
                {
                    continue;
                }
                NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    Shape aEIm = aItLEIm.Value();
                    //
                    Iterator aItV = new(aEIm);
                    for (; aItV.More(); aItV.Next())
                    {
                        Shape aV = aItV.Value();
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
        Shape aSecR = aSec.InternalShape();
        //
        NCollection_IndexedMap<Shape> aMSSec = new(ShapeHasher.Instance);
        Exploration.MapShapes(aSecR, aMSSec);
        //
        NCollection_DataMap<Shape, NCollection_List<Shape>> anIm = aSec.Images();
        for (Explorer aExp = new(theSplits, ShapeType.Edge); aExp.More(); aExp.Next())
        {
            Shape aE = aExp.Current();
            if (aSec.IsDeleted(aE))
            {
                // no common blocks for this edge
                continue;
            }
            //
            if (!anIm.Find(aE, out NCollection_List<Shape> pLEIm))
            {
                // no splits, i.e. completely coincides with some edge from boundary
                continue;
            }
            //
            NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
            for (; aItLEIm.More(); aItLEIm.Next())
            {
                Shape aEIm = aItLEIm.Value();
                if (!aMSSec.Contains(aEIm))
                {
                    // the edge included in section only partially.
                    // the part not included in section may be excessive
                    //
                    // check vertices of this edge - if one of them is new
                    // the edge might be removed
                    Exploration.Vertices(ShapeCasts.Edge(aEIm), out Vertex aV1, out Vertex aV2);
                    if (!theMVOld.Contains(aV1) || !theMVOld.Contains(aV2))
                    {
                        // add this edge for checking by making new vertex in the middle of the edge
                        Vertex aV = new();
                        Curve3D aC = Tool.Curve(ShapeCasts.Edge(aEIm), out double f, out double l)!;
                        new BrepBuilder().MakeVertex(ref aV, aC.Value((f + l) * 0.5), global::Kerf.Engine.Foundation.Precision.Confusion());
                        // and adding this vertex for checking
                        aDMVE.ChangeFromIndex(aDMVE.Add(aV, new NCollection_List<Shape>())).Append(aE);
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
        foreach (Shape aE in theMECheckExt)
        {
            //
            // make new vertex in the middle of the edge
            Vertex aV = new();
            Curve3D aC = Tool.Curve(ShapeCasts.Edge(aE), out double f, out double l)!;
            new BrepBuilder().MakeVertex(ref aV, aC.Value((f + l) * 0.5), global::Kerf.Engine.Foundation.Precision.Confusion());
            // add this vertex for checking
            aDMVE.ChangeFromIndex(aDMVE.Add(aV, new NCollection_List<Shape>())).Append(aE);
            aMVInv.Add(aV);
        }
        //
        // add for check also the vertices connected only to new or old edges
        aNb = aDMVE.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aV = aDMVE.FindKey(i);
            if (theMVOld.Contains(aV))
            {
                continue;
            }
            //
            bool bNew = false, bOld = false;
            NCollection_List<Shape> aLEx = aDMVE[i];
            NCollection_List<Shape>.Iterator aIt = new(aLEx);
            for (; aIt.More(); aIt.Next())
            {
                Shape aE = aIt.Value();
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
            Vertex aV = ShapeCasts.Vertex(aMVInv[iv]);
            if (theMVOld.Contains(aV))
            {
                continue;
            }
            //
            if (!aDMVE.FindFromKey(aV, out NCollection_List<Shape> pLEInv))
            {
                continue;
            }
            // find faces by the edges to check the vertex
            NCollection_IndexedMap<Shape> aMF = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator aItLE = new(pLEInv);
            for (; aItLE.More(); aItLE.Next())
            {
                Shape aE = aItLE.Value();
                NCollection_List<Shape> aLF = theMELF.Find(aE);
                NCollection_List<Shape>.Iterator aItLF0 = new(aLF);
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
                Face aF = ShapeCasts.Face(aMF[i]);
                NCollection_List<Shape> aLFIm = myOFImages.FindFromKey(aF);
                //
                NCollection_List<Shape>.Iterator aItLF = new(aLFIm);
                for (; aItLF.More() && bInvalid; aItLF.Next())
                {
                    Face aFIm = ShapeCasts.Face(aItLF.Value());
                    Explorer aExp = new(aFIm, ShapeType.Vertex);
                    for (; aExp.More() && bInvalid; aExp.Next())
                    {
                        Shape aVF = aExp.Current();
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
                            Face aFIm = ShapeCasts.Face(aItLF.Value());
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
                Point3D aP = Tool.Pnt(aV);
                double aTolV = Tool.Tolerance(aV);
                //
                Explorer aExpS = new(mySolids, ShapeType.Solid);
                for (; aExpS.More() && bInvalid; aExpS.Next())
                {
                    Solid aSol = ShapeCasts.Solid(aExpS.Current());
                    BRepClass3d_SolidClassifier aSC = myContext.SolidClassifier(aSol);
                    aSC.Perform(aP, aTolV);
                    bInvalid = (aSC.State() == Out);
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
    private void FilterSplits(NCollection_List<Shape> theLE, NCollection_Map<Shape> theMEFilter, bool theIsInv,
                              NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages,
                              ref Shape theSplits)
    {
        Compound aSplits = new();
        new BrepBuilder().MakeCompound(ref aSplits);
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator aItLE = new(theLE);
        for (; aItLE.More(); aItLE.Next())
        {
            Shape aE = aItLE.Value();
            if (!theEImages.Find(aE, out NCollection_List<Shape> pLEIm))
            {
                continue;
            }

            NCollection_List<Shape>.Iterator aItLEIm = new(pLEIm);
            for (; aItLEIm.More();)
            {
                Shape aEIm = aItLEIm.Value();
                if (theMEFilter.Contains(aEIm) == theIsInv)
                {
                    pLEIm.Remove(aItLEIm);
                    continue;
                }

                if (aMFence.Add(aEIm))
                {
                    new BrepBuilder().Add(aSplits, aEIm);
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
    private void UpdateNewIntersectionEdges(NCollection_List<Shape> theLE,
                                            NCollection_DataMap<Shape, NCollection_List<Shape>> theMELF,
                                            NCollection_DataMap<Shape, NCollection_List<Shape>> theEImages,
                                            NCollection_DataMap<Shape, NCollection_List<Shape>> theEETrim)
    {
        NCollection_List<Shape> aLEImEmpty = new();
        NCollection_List<Shape>.Iterator aIt = new(), aIt1 = new();
        // update global maps of images and origins with new splits
        aIt.Initialize(theLE);
        for (; aIt.More(); aIt.Next())
        {
            Shape aE = aIt.Value();
            //
            if (!theEImages.IsBound(aE))
            {
                if (!theEETrim.Find(aE, out NCollection_List<Shape> pLET))
                {
                    continue;
                }
                //
                NCollection_List<Shape>.Iterator aItLET = new(pLET);
                for (; aItLET.More();)
                {
                    Shape aET = aItLET.Value();
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
            NCollection_List<Shape> aLENew = theEImages.IsBound(aE) ? theEImages.Find(aE) : aLEImEmpty;
            //
            // save connection to untrimmed edge for the next steps
            aIt1.Initialize(aLENew);
            for (; aIt1.More(); aIt1.Next())
            {
                Shape aET = aIt1.Value();
                myETrimEInf!.Bind(aET, aE);
                myModifiedEdges.Add(aET);
            }
            //
            // check if it is existing edge
            if (!theEETrim.IsBound(aE))
            {
                NCollection_List<Shape> aLF = theMELF.Find(aE);
                // the edge is new
                // add this edge to AsDes
                aIt1.Initialize(aLF);
                for (; aIt1.More(); aIt1.Next())
                {
                    Shape aF = aIt1.Value();
                    myAsDes!.Add(aF, aE);
                }
                //
                // add aE to the images
                myOEImages.Bind(aE, new NCollection_List<Shape>(aLENew));
                myModifiedEdges.Add(aE);
                //
                // add to origins
                NCollection_List<Shape>.Iterator aItNew = new(aLENew);
                for (; aItNew.More(); aItNew.Next())
                {
                    Shape aENew = aItNew.Value();
                    if (myOEOrigins.IsBound(aENew))
                    {
                        NCollection_List<Shape> aEOrigins = myOEOrigins.ChangeFind(aENew);
                        AppendToList(aEOrigins, aE);
                    }
                    else
                    {
                        NCollection_List<Shape> aEOrigins = new();
                        aEOrigins.Append(aE);
                        myOEOrigins.Bind(aENew, aEOrigins);
                    }
                }
                //
                // update connection to initial origins
                if (myEdgesOrigins!.IsBound(aE))
                {
                    NCollection_List<Shape> aLEOrInit = myEdgesOrigins.Find(aE);
                    aIt1.Initialize(aLENew);
                    for (; aIt1.More(); aIt1.Next())
                    {
                        Shape aENew = aIt1.Value();
                        if (myEdgesOrigins.IsBound(aENew))
                        {
                            NCollection_List<Shape> aLENewOr = myEdgesOrigins.ChangeFind(aENew);
                            NCollection_List<Shape>.Iterator aItOrInit = new(aLEOrInit);
                            for (; aItOrInit.More(); aItOrInit.Next())
                            {
                                Shape aEOr = aItOrInit.Value();
                                AppendToList(aLENewOr, aEOr);
                            }
                        }
                        else
                        {
                            myEdgesOrigins.Bind(aENew, new NCollection_List<Shape>(aLEOrInit));
                        }
                    }
                }
                //
                continue;
            }
            //
            // old images
            NCollection_List<Shape> aLEOld = theEETrim.Find(aE);
            //
            // list of initial origins
            NCollection_List<Shape> anInitOrigins = new();
            //
            // it is necessary to replace the old edges with new ones
            aIt1.Initialize(aLEOld);
            for (; aIt1.More(); aIt1.Next())
            {
                Shape aEOld = aIt1.Value();
                //
                if (myOEOrigins.IsBound(aEOld))
                {
                    // get its origins
                    NCollection_List<Shape> aEOrigins = myOEOrigins.Find(aEOld);
                    //
                    NCollection_List<Shape>.Iterator aItOr = new(aEOrigins);
                    for (; aItOr.More(); aItOr.Next())
                    {
                        Shape aEOr = aItOr.Value();
                        //
                        myModifiedEdges.Add(aEOr);
                        //
                        NCollection_List<Shape> aEImages = myOEImages.ChangeFind(aEOr);
                        //
                        // remove old edge from images
                        NCollection_List<Shape>.Iterator aItIm = new(aEImages);
                        for (; aItIm.More();)
                        {
                            Shape aEIm = aItIm.Value();
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
                        NCollection_List<Shape>.Iterator aItNew = new(aLENew);
                        for (; aItNew.More(); aItNew.Next())
                        {
                            Shape aENew = aItNew.Value();
                            AppendToList(aEImages, aENew);
                            if (myOEOrigins.IsBound(aENew))
                            {
                                NCollection_List<Shape> aENewOrigins = myOEOrigins.ChangeFind(aENew);
                                AppendToList(aENewOrigins, aEOr);
                            }
                            else
                            {
                                NCollection_List<Shape> aENewOrigins = new();
                                aENewOrigins.Append(aEOr);
                                myOEOrigins.Bind(aENew, aENewOrigins);
                            }
                        }
                    }
                }
                else
                {
                    // add to images
                    myOEImages.Bind(aEOld, new NCollection_List<Shape>(aLENew));
                    //
                    myModifiedEdges.Add(aEOld);
                    //
                    // add to origins
                    NCollection_List<Shape>.Iterator aItNew = new(aLENew);
                    for (; aItNew.More(); aItNew.Next())
                    {
                        Shape aENew = aItNew.Value();
                        if (myOEOrigins.IsBound(aENew))
                        {
                            NCollection_List<Shape> aEOrigins = myOEOrigins.ChangeFind(aENew);
                            AppendToList(aEOrigins, aEOld);
                        }
                        else
                        {
                            NCollection_List<Shape> aEOrigins = new();
                            aEOrigins.Append(aEOld);
                            myOEOrigins.Bind(aENew, aEOrigins);
                        }
                    }
                }
                //
                // update connection to initial shape
                if (myEdgesOrigins!.IsBound(aEOld))
                {
                    NCollection_List<Shape> aLEOrInit = myEdgesOrigins.Find(aEOld);
                    NCollection_List<Shape>.Iterator aItEOrInit = new(aLEOrInit);
                    for (; aItEOrInit.More(); aItEOrInit.Next())
                    {
                        Shape aEOrInit = aItEOrInit.Value();
                        AppendToList(anInitOrigins, aEOrInit);
                    }
                }
            }
            //
            if (anInitOrigins.Extent() != 0)
            {
                NCollection_List<Shape>.Iterator aItNew = new(aLENew);
                for (; aItNew.More(); aItNew.Next())
                {
                    Shape aENew = aItNew.Value();
                    if (myEdgesOrigins!.IsBound(aENew))
                    {
                        NCollection_List<Shape> aLENewOr = myEdgesOrigins.ChangeFind(aENew);
                        NCollection_List<Shape>.Iterator aItOrInit = new(anInitOrigins);
                        for (; aItOrInit.More(); aItOrInit.Next())
                        {
                            Shape aEOr = aItOrInit.Value();
                            AppendToList(aLENewOr, aEOr);
                        }
                    }
                    else
                    {
                        myEdgesOrigins.Bind(aENew, new NCollection_List<Shape>(anInitOrigins));
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
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFMap = new(ShapeHasher.Instance);
        for (int i = 1; i <= aNbF; ++i, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }

            NCollection_List<Shape>.Iterator itLF = new(myOFImages[i]);
            for (; itLF.More(); itLF.Next())
            {
                Exploration.MapShapesAndAncestors(itLF.Value(), ShapeType.Edge, ShapeType.Face, anEFMap);
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

            NCollection_List<Shape> aLFImages = myOFImages[i];
            if (aLFImages.IsEmpty())
            {
                continue;
            }

            // Collect all edges from the splits
            Compound anEdges = new();
            new BrepBuilder().MakeCompound(ref anEdges);

            // Collect all free edges into a map with reverted orientation
            // (OCCT's default hasher: the orientation counts).
            NCollection_Map<Shape> aFreeEdgesMap = new(EqualityComparer<Shape>.Default);
            NCollection_List<Shape>.Iterator itLF = new(aLFImages);
            for (; itLF.More(); itLF.Next())
            {
                Shape aFIm = itLF.Value();
                Explorer anExpE = new(aFIm, ShapeType.Edge);
                for (; anExpE.More(); anExpE.Next())
                {
                    Shape aE = anExpE.Current();
                    if (aE.Orientation() != Forward && aE.Orientation() != Reversed)
                    {
                        // Skip internals
                        continue;
                    }

                    NCollection_List<Shape> aLF = anEFMap.FindFromKey(aE);
                    if (aLF.Extent() == 1)
                    {
                        aFreeEdgesMap.Add(aE.Reversed());
                    }

                    new BrepBuilder().Add(anEdges, aE);
                }
            }

            if (aFreeEdgesMap.IsEmpty())
            {
                // No free edges
                continue;
            }

            // Free edges are found - fill the gaps by creating new splits
            // of the face using these free edges
            Shape aF = myOFImages.FindKey(i);

            // Build new splits using all kept edges and among new splits
            // find those containing free edges
            NCollection_List<Shape> aLFNew = new();
            NCollection_DataMap<Shape, Shape> aDummy = new(ShapeHasher.Instance);

            BuildSplitsOfFace(ShapeCasts.Face(aF), anEdges, aDummy, aLFNew);

            // Find faces filling holes
            itLF.Initialize(aLFNew);
            for (; itLF.More(); itLF.Next())
            {
                Shape aFNew = itLF.Value();
                Explorer anExpE = new(aFNew, ShapeType.Edge);
                for (; anExpE.More(); anExpE.Next())
                {
                    Shape aE = anExpE.Current();
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
        NCollection_IndexedMap<Shape> anEdgesMap = new(ShapeHasher.Instance);

        // Fill history for faces
        for (int i = 1; i <= aNbF; ++i)
        {
            NCollection_List<Shape> aLFImages = myOFImages[i];
            if (aLFImages.IsEmpty())
            {
                continue;
            }

            // Add the splits to history map
            Shape aF = myOFImages.FindKey(i);
            if (myImage.HasImage(aF))
            {
                myImage.Add(aF, aLFImages);
            }
            else
            {
                myImage.Bind(aF, aLFImages);
            }

            // Collect edges from splits
            NCollection_List<Shape>.Iterator itLF = new(aLFImages);
            for (; itLF.More(); itLF.Next())
            {
                Shape aFIm = itLF.Value();
                Exploration.MapShapes(aFIm, ShapeType.Edge, anEdgesMap);
            }
        }

        // Fill history for edges (iteration by the map is safe because the
        // order is not important here)
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> aItEIm in myOEImages)
        {
            Shape aE = aItEIm.Key;
            NCollection_List<Shape> aLEIm = aItEIm.Value;

            bool bHasImage = myImage.HasImage(aE);
            NCollection_List<Shape>.Iterator aItLE = new(aLEIm);
            for (; aItLE.More(); aItLE.Next())
            {
                Shape aEIm = aItLE.Value();
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
    private void BuildSplitsOfTrimmedFaces(NCollection_List<Shape> theLF, BRepAlgo_AsDes theAsDes,
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
    private void BuildSplitsOfExtendedFaces(NCollection_List<Shape> theLF, BRepOffset_Analyse theAnalyse,
                                            BRepAlgo_AsDes theAsDes,
                                            NCollection_DataMap<Shape, NCollection_List<Shape>> theEdgesOrigins,
                                            NCollection_DataMap<Shape, Shape> theFacesOrigins,
                                            NCollection_DataMap<Shape, Shape> theETrimEInf,
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
