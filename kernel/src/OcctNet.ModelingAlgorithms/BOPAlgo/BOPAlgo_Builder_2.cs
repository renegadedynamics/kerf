// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The same-domain check of a pair of faces, in the builder's parallel loop.</summary>
internal sealed class BOPAlgo_PairOfShapeBoolean : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private bool myFlag;
    private TopoDS_Shape myShape1 = new();
    private TopoDS_Shape myShape2 = new();
    private IntTools_Context? myContext;

    public BOPAlgo_PairOfShapeBoolean()
    {
        myFlag = false;
    }

    //
    public ref TopoDS_Shape Shape1() => ref myShape1;

    //
    public ref TopoDS_Shape Shape2() => ref myShape2;

    //
    public ref bool Flag() => ref myFlag;

    //
    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    //
    public IntTools_Context? Context() => myContext;

    //
    public override void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (UserBreak(aPS))
        {
            return;
        }

        TopoDS_Face aFj = TopoDS.Face(myShape1);
        TopoDS_Face aFk = TopoDS.Face(myShape2);
        myFlag = BOPTools_AlgoTools.AreFacesSameDomain(aFj, aFk, myContext!, myFuzzyValue);
    }
}

/// <summary>Extends BOPAlgo_BuilderFace with its own progress range, for the builder's parallel loop.</summary>
internal sealed class BOPAlgo_SplitFace : BOPAlgo_BuilderFace, BOPTools_Parallel.ISolver
{
    private Message_ProgressRange myRange;

    //! Sets progress range
    public void SetProgressRange(Message_ProgressRange theRange) => myRange = theRange;

    // New perform method, using own progress range
    public void Perform()
    {
        using var aPS = new Message_ProgressScope(myRange, null, 1);
        if (!aPS.More())
        {
            return;
        }
        base.Perform(aPS.Next());
    }

    //! Disables the range enabled method (private in OCCT; C# can't narrow an override)
    public override void Perform(Message_ProgressRange theRange = default)
    {
    }
}

/// <summary>The classification of a vertex relatively a face, in the builder's parallel loop.</summary>
internal sealed class BOPAlgo_VFI : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private bool myIsInternal;
    private TopoDS_Vertex myV = new();
    private TopoDS_Face myF = new();
    private IntTools_Context? myContext;

    public BOPAlgo_VFI()
    {
        myIsInternal = false;
    }

    //
    public void SetVertex(in TopoDS_Vertex aV) => myV = aV;

    //
    public ref TopoDS_Vertex Vertex() => ref myV;

    //
    public void SetFace(in TopoDS_Face aF) => myF = aF;

    //
    public ref TopoDS_Face Face() => ref myF;

    //
    public bool IsInternal() => myIsInternal;

    //
    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    //
    public IntTools_Context? Context() => myContext;

    //
    public override void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (UserBreak(aPS))
        {
            return;
        }

        double aT1 = 0, aT2 = 0, dummy = 0;
        //
        int iFlag = myContext!.ComputeVF(myV, myF, ref aT1, ref aT2, ref dummy, myFuzzyValue);
        myIsInternal = iFlag == 0;
    }
}

public partial class BOPAlgo_Builder
{
    /// <summary>Fills the images of the faces: the split faces, the same-domain faces and the internal vertices.</summary>
    protected void FillImagesFaces(Message_ProgressRange theRange)
    {
        using var aPS = new Message_ProgressScope(theRange, "Filling splits of faces", 10);
        BuildSplitFaces(aPS.Next(9));
        if (HasErrors())
        {
            return;
        }
        FillSameDomainFaces(aPS.Next(0.5));
        if (HasErrors())
        {
            return;
        }
        FillInternalVertices(aPS.Next(0.5));
    }

    /// <summary>Builds the split faces of the faces having IN, ON or section pave blocks, or alone vertices.</summary>
    protected virtual void BuildSplitFaces(Message_ProgressRange theRange)
    {
        bool bHasFaceInfo, bIsClosed, bIsDegenerated, bToReverse;
        int i, j, k, aNbS, aNbPBIn, aNbPBOn, aNbPBSc, aNbAV, nSp;
        TopoDS_Face aFF;
        TopoDS_Edge aSp, aEE;
        TopAbs_Orientation anOriF, anOriE;
        TopExp_Explorer aExp = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        NCollection_List<int> aLIAV = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DynamicArray<BOPAlgo_SplitFace> aVBF = new();
        //
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~scope f
        NCollection_List<TopoDS_Shape> aLE = new();
        // (OCCT's unused map aMDE is left out)
        //
        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);
        // Build temporary map of faces images to avoid rebuilding
        // of the faces without any IN or section edges
        NCollection_IndexedDataMap<int, NCollection_List<TopoDS_Shape>> aFacesIm = new();
        //
        aNbS = myDS!.NbSourceShapes();
        //
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            if (UserBreak(aPSOuter))
            {
                return;
            }
            //
            TopoDS_Face aF = TopoDS.Face(aSI.Shape());
            bool isUClosed = false, isVClosed = false, isChecked = false;
            //
            bHasFaceInfo = myDS.HasFaceInfo(i);
            if (!bHasFaceInfo)
            {
                continue;
            }
            //
            BOPDS_FaceInfo aFI = myDS.FaceInfo(i);
            //
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBIn = aFI.PaveBlocksIn();
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBOn = aFI.PaveBlocksOn();
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBSc = aFI.PaveBlocksSc();
            aLIAV.Clear();
            myDS.AloneVertices(i, aLIAV);

            aNbPBIn = aMPBIn.Extent();
            aNbPBOn = aMPBOn.Extent();
            aNbPBSc = aMPBSc.Extent();
            aNbAV = aLIAV.Extent();
            if (aNbPBIn == 0 && aNbPBOn == 0 && aNbPBSc == 0 && aNbAV == 0)
            { // not compete
                continue;
            }

            if (aNbPBIn == 0 && aNbPBSc == 0)
            {
                // If there are any alone vertices to be put in the face,
                // the new face has to be created even if the wires of the
                // face have not been modified.

                // It is also necessary to check if the face contains any internal edges,
                // as such edges may split the face on parts and it is better
                // to send the face be treated by the BuilderFace algorithm.
                // In case of alone vertices the check for internals will be performed
                // in the BuildDraftFace method.
                bool hasInternals = false;
                if (aNbAV == 0)
                {
                    // Check if any wires of the face have been modified.
                    // If no modified and internal wires present in the face
                    // there is no need to create the new face.
                    bool hasModified = false;

                    TopoDS_Iterator aItW = new(aF);
                    for (; aItW.More(); aItW.Next())
                    {
                        TopoDS_Iterator itE = new(aItW.Value());
                        hasInternals = itE.More() && (itE.Value().Orientation() == TopAbs_INTERNAL);
                        if (hasInternals)
                        {
                            break;
                        }

                        hasModified |= myImages.IsBound(aItW.Value());
                    }

                    if (!hasInternals && !hasModified)
                    {
                        continue;
                    }
                }

                if (!hasInternals)
                {
                    // No internal parts for the face, so just build the draft face
                    // and keep it to pass directly into result.
                    // If the original face has any internal edges or multi-connected vertices,
                    // the draft face will be null, as such sub-shapes may split the face on parts
                    // (as in the case "bugs modalg_5 bug25245_1").
                    // The BuilderFace algorithm will be called in this case.
                    TopoDS_Face aFD = BuildDraftFace(aF, myImages, myContext!, myReport);
                    if (!aFD.IsNull())
                    {
                        aFacesIm[aFacesIm.Add(i, new NCollection_List<TopoDS_Shape>())].Append(aFD);
                        continue;
                    }
                }
            }

            aMFence.Clear();
            //
            anOriF = aF.Orientation();
            aFF = aF;
            aFF.Orientation(TopAbs_FORWARD);
            //
            // 1. Fill the edges set for the face aFF -> LE
            aLE.Clear();

            // 1.1 Bounding edges
            aExp.Init(aFF, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                anOriE = aE.Orientation();
                //
                if (!myImages.IsBound(aE))
                {
                    if (anOriE == TopAbs_INTERNAL)
                    {
                        aEE = aE;
                        aEE.Orientation(TopAbs_FORWARD);
                        aLE.Append(aEE);
                        aEE.Orientation(TopAbs_REVERSED);
                        aLE.Append(aEE);
                    }
                    else
                    {
                        aLE.Append(aE);
                    }

                    continue;
                }

                if (!isChecked)
                {
                    Geom_Surface aSurf = BRep_Tool.Surface(aF)!;
                    GeomLib.IsClosed(aSurf, BRep_Tool.Tolerance(aE), out isUClosed, out isVClosed);

                    isChecked = true;
                }

                bIsClosed = false;

                if ((isUClosed || isVClosed) && BRep_Tool.IsClosed(aE, aF))
                {
                    BOPTools_AlgoTools2D.IsEdgeIsoline(aE, aF, out bool isUIso, out bool isVIso);

                    bIsClosed = (isUClosed && isUIso) || (isVClosed && isVIso);
                }

                bIsDegenerated = BRep_Tool.Degenerated(aE);

                NCollection_List<TopoDS_Shape> aLIE = myImages.Find(aE);
                aIt.Initialize(aLIE);
                for (; aIt.More(); aIt.Next())
                {
                    aSp = TopoDS.Edge(aIt.Value());
                    if (bIsDegenerated)
                    {
                        aSp.Orientation(anOriE);
                        aLE.Append(aSp);
                        continue;
                    }
                    //
                    if (anOriE == TopAbs_INTERNAL)
                    {
                        aSp.Orientation(TopAbs_FORWARD);
                        aLE.Append(aSp);
                        aSp.Orientation(TopAbs_REVERSED);
                        aLE.Append(aSp);
                        continue;
                    }
                    //
                    if (bIsClosed)
                    {
                        if (aMFence.Add(aSp))
                        {
                            if (!BRep_Tool.IsClosed(aSp, aF))
                            {
                                if (!BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSp, aF))
                                {
                                    // try different approach
                                    if (!BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aE, aSp, aF))
                                    {
                                        TopoDS_Compound aWS = new();
                                        new BRep_Builder().MakeCompound(ref aWS);
                                        new BRep_Builder().Add(aWS, aF);
                                        new BRep_Builder().Add(aWS, aSp);
                                        AddWarning(new BOPAlgo_AlertUnableToMakeClosedEdgeOnFace(aWS));
                                    }
                                }
                            }
                            //
                            aSp.Orientation(TopAbs_FORWARD);
                            aLE.Append(aSp);
                            aSp.Orientation(TopAbs_REVERSED);
                            aLE.Append(aSp);
                        } // if (aMFence.Add(aSp))
                        continue;
                    } // if (bIsClosed){
                    //
                    aSp.Orientation(anOriE);
                    bToReverse = BOPTools_AlgoTools.IsSplitToReverseWithWarn(aSp, aE, myContext, myReport);
                    if (bToReverse)
                    {
                        aSp.Reverse();
                    }
                    aLE.Append(aSp);
                } // for (; aIt.More(); aIt.Next()) {
            } // for (; aExp.More(); aExp.Next()) {
            //
            //
            // 1.2 In edges
            for (j = 1; j <= aNbPBIn; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBIn[j];
                nSp = aPB.Edge();
                if (nSp < 0)
                {
                    // Standard_ASSERT: "Face information is not up to date" (a debug report in OCCT)
                    continue;
                }
                aSp = TopoDS.Edge(myDS.Shape(nSp));
                //
                aSp.Orientation(TopAbs_FORWARD);
                aLE.Append(aSp);
                aSp.Orientation(TopAbs_REVERSED);
                aLE.Append(aSp);
            }
            //
            //
            // 1.3 Section edges
            for (j = 1; j <= aNbPBSc; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBSc[j];
                nSp = aPB.Edge();
                aSp = TopoDS.Edge(myDS.Shape(nSp));
                //
                aSp.Orientation(TopAbs_FORWARD);
                aLE.Append(aSp);
                aSp.Orientation(TopAbs_REVERSED);
                aLE.Append(aSp);
            }
            //
            if (!myPaveFiller!.NonDestructive())
            {
                // speed up for planar faces
                BRepLib.BuildPCurveForEdgesOnPlane(aLE, aFF);
            }
            // 3 Build split faces
            BOPAlgo_SplitFace aBF = aVBF.Append(new BOPAlgo_SplitFace());
            aBF.SetFace(aF);
            aBF.SetShapes(aLE);
            aBF.SetRunParallel(myRunParallel);
            //
        } // for (i=0; i<aNbS; ++i) {

        // close preparation task
        aPSOuter.Next();
        //
        int aNbBF = aVBF.Length();
        // Set progress range for each task to be run in parallel
        using var aPSParallel = new Message_ProgressScope(aPSOuter.Next(9), "Splitting faces", aNbBF);
        for (int iF = 0; iF < aNbBF; iF++)
        {
            BOPAlgo_SplitFace aBF = aVBF.ChangeValue(iF);
            aBF.SetProgressRange(aPSParallel.Next());
        }
        //===================================================
        BOPTools_Parallel.Perform(myRunParallel, aVBF);
        //===================================================
        if (UserBreak(aPSOuter))
        {
            return;
        }
        for (k = 0; k < aNbBF; ++k)
        {
            BOPAlgo_BuilderFace aBF = aVBF[k];
            aFacesIm.Add(myDS.Index(aBF.Face()), new NCollection_List<TopoDS_Shape>(aBF.Areas()));
            myReport.Merge(aBF.GetReport());
        }

        aNbBF = aFacesIm.Extent();
        for (k = 1; k <= aNbBF; ++k)
        {
            TopoDS_Face aF = TopoDS.Face(myDS.Shape(aFacesIm.FindKey(k)));
            anOriF = aF.Orientation();
            NCollection_List<TopoDS_Shape> aLFR = aFacesIm[k];
            //
            NCollection_List<TopoDS_Shape> pLFIm = myImages.Bound(aF, new NCollection_List<TopoDS_Shape>());
            aIt.Initialize(aLFR);
            for (; aIt.More(); aIt.Next())
            {
                ref TopoDS_Shape aFR = ref aIt.ChangeValue();
                if (anOriF == TopAbs_REVERSED)
                {
                    aFR.Orientation(TopAbs_REVERSED);
                }
                pLFIm.Append(aFR);
            }
        }
        //
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~scope t
    }

    // Adds the set of the edges of the shape to the map, keeping the connection to the shape.
    private static void AddEdgeSet(in TopoDS_Shape theS, NCollection_IndexedDataMap<BOPTools_Set, NCollection_List<TopoDS_Shape>> theMap)
    {
        // Make set
        BOPTools_Set aSE = new();
        aSE.Add(theS, TopAbs_EDGE);
        // Add set to the map, keeping connection to the shape
        int anIndex = theMap.FindIndex(aSE);
        NCollection_List<TopoDS_Shape> pLF = anIndex > 0
                                                 ? theMap[anIndex]
                                                 : theMap[theMap.Add(aSE, new NCollection_List<TopoDS_Shape>())];
        pLF.Append(theS);
    }

    /// <summary>Finds the same-domain faces among the faces having interfered, and makes one represent each group.</summary>
    protected void FillSameDomainFaces(Message_ProgressRange theRange)
    {
        // It is necessary to analyze all Face/Face intersections
        // and find all faces with equal sets of edges
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS!.InterfFF();
        int aNbFFs = aFFs.Length();
        if (aNbFFs == 0)
        {
            return;
        }

        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);

        // Two distinct faces of one solid cannot be Same-Domain: that would
        // imply a zero-thickness interior in a single operand.
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aFaceToParent = new(TopTools_ShapeMapHasher.Instance);
        {
            int aNbSrc = myDS.NbSourceShapes();
            for (int iSrc = 0; iSrc < aNbSrc; ++iSrc)
            {
                BOPDS_ShapeInfo aSI = myDS.ShapeInfo(iSrc);
                if (aSI.ShapeType() != TopAbs_SOLID)
                {
                    continue;
                }
                TopoDS_Shape aSolid = aSI.Shape();
                for (TopExp_Explorer anExpF = new(aSolid, TopAbs_FACE); anExpF.More(); anExpF.Next())
                {
                    TopoDS_Shape aF = anExpF.Current();
                    if (!aFaceToParent.IsBound(aF))
                    {
                        aFaceToParent.Bind(aF, aSolid);
                    }
                }
            }
            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aPropagation = new(TopTools_ShapeMapHasher.Instance);
            // OCCT iterates the images map, a shape-keyed map, in address order (the port: insertion order); a split
            // face belongs to one face's images, so the order isn't observable here
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItIm in myImages)
            {
                if (!aFaceToParent.Find(anItIm.Key, out TopoDS_Shape pParent))
                {
                    continue;
                }
                for (NCollection_List<TopoDS_Shape>.Iterator anItPiece = new(anItIm.Value); anItPiece.More(); anItPiece.Next())
                {
                    if (!aFaceToParent.IsBound(anItPiece.Value()))
                    {
                        aPropagation.Bind(anItPiece.Value(), pParent);
                    }
                }
            }
            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItProp in aPropagation)
            {
                aFaceToParent.Bind(anItProp.Key, anItProp.Value);
            }
        }

        // Vector to store the indices of faces for future sorting
        // for making the SD face for the group from the face with
        // smallest index in Data structure
        NCollection_DynamicArray<int> aFIVec = new(256);
        // Fence map to avoid repeated checks of the same face.
        NCollection_IntMap aMFence = new(1);

        // Fill the vector with indices of faces
        Span<int> nF2 = stackalloc int[2]; // OCCT's nF[2], renamed: C# can't redeclare nF below
        for (int i = 0; i < aNbFFs; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPDS_InterfFF aFF = aFFs[i];
            // get indices
            aFF.Indices(out nF2[0], out nF2[1]);
            // store indices to the vector
            for (int j = 0; j < 2; ++j)
            {
                if (!myDS.HasFaceInfo(nF2[j]))
                {
                    continue;
                }

                if (!aMFence.Add(nF2[j]))
                {
                    continue;
                }

                aFIVec.Append(nF2[j]);
            }
        }

        // Sort the indices (distinct, so any sort gives std::sort's order)
        int[] aFIArr = [.. aFIVec];
        Array.Sort(aFIArr);
        for (int i = 0; i < aFIArr.Length; ++i)
        {
            aFIVec[i] = aFIArr[i];
        }

        // Data map of set of edges with all faces having this set
        NCollection_IndexedDataMap<BOPTools_Set, NCollection_List<TopoDS_Shape>> anESetFaces = new();
        // Map of planar bounded faces. If such faces have the same Edge set
        // they are considered Same domain, without additional check.
        NCollection_Map<TopoDS_Shape> aMFPlanar = new(TopTools_ShapeMapHasher.Instance);

        int aNbF = aFIVec.Length();
        for (int i = 0; i < aNbF; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            int nF = aFIVec[i];
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(nF);
            TopoDS_Shape aF = aSI.Shape();

            bool bCheckPlanar = false;
            {
                // At this stage, context should contain adaptor for all intersected faces,
                // so getting a type of the underlying surface should be done at no cost.
                if (myContext!.SurfaceAdaptor(TopoDS.Face(aF)).GetType() == GeomAbs_Plane)
                {
                    // Check bounding box of the face - it should not be open in any side
                    Bnd_Box aBox = aSI.Box();
                    bCheckPlanar = !(aBox.IsOpenXmin() || aBox.IsOpenXmax() || aBox.IsOpenYmin()
                                     || aBox.IsOpenYmax() || aBox.IsOpenZmin() || aBox.IsOpenZmax());
                }
            }

            if (myImages.Find(aF, out NCollection_List<TopoDS_Shape> pLFSp))
            {
                NCollection_List<TopoDS_Shape>.Iterator aItLF = new(pLFSp);
                for (; aItLF.More(); aItLF.Next())
                {
                    AddEdgeSet(aItLF.Value(), anESetFaces);
                    if (bCheckPlanar)
                    {
                        aMFPlanar.Add(aItLF.Value());
                    }
                }
            }
            else
            {
                AddEdgeSet(aF, anESetFaces);
                if (bCheckPlanar)
                {
                    aMFPlanar.Add(aF);
                }
            }
        }

        // Store pairs of faces with equal set of edges to check if they are really Same Domain
        NCollection_DynamicArray<BOPAlgo_PairOfShapeBoolean> aVPSB = new();

        // Back and forth map of SD faces to make the blocks
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMSLS = new(TopTools_ShapeMapHasher.Instance);

        int aNbSets = anESetFaces.Extent();
        for (int i = 1; i <= aNbSets; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            NCollection_List<TopoDS_Shape> aLF = anESetFaces[i];
            if (aLF.Extent() < 2)
            {
                continue;
            }

            // All possible pairs from <aLF> should be checked
            NCollection_List<TopoDS_Shape>.Iterator aIt1 = new(aLF);
            for (; aIt1.More(); aIt1.Next())
            {
                TopoDS_Shape aF1 = aIt1.Value();
                bool bCheckPlanar = aMFPlanar.Contains(aF1);
                bool hasParent1 = aFaceToParent.Find(aF1, out TopoDS_Shape pParent1);

                NCollection_List<TopoDS_Shape>.Iterator aIt2 = new();
                aIt2.Reset(aLF, aIt1.Index); // aIt2 = aIt1
                for (aIt2.Next(); aIt2.More(); aIt2.Next())
                {
                    TopoDS_Shape aF2 = aIt2.Value();
                    bool hasParent2 = aFaceToParent.Find(aF2, out TopoDS_Shape pParent2);
                    if (hasParent1 && hasParent2 && pParent1.IsSame(pParent2))
                    {
                        continue;
                    }
                    if (bCheckPlanar && aMFPlanar.Contains(aF2))
                    {
                        // Consider planar bounded faces as Same Domain without additional check
                        BOPAlgo_Tools.FillMap(aF1, aF2, aDMSLS);
                        continue;
                    }
                    // Add pair for analysis
                    BOPAlgo_PairOfShapeBoolean aPSB = aVPSB.Append(new BOPAlgo_PairOfShapeBoolean());
                    aPSB.Shape1() = aF1;
                    aPSB.Shape2() = aF2;
                    aPSB.SetFuzzyValue(myFuzzyValue);
                }
            }
        }

        aPSOuter.Next();

        // Set progress range for each task to be run in parallel
        using (var aPSParallel = new Message_ProgressScope(aPSOuter.Next(6), "Checking SD faces", aVPSB.Length()))
        {
            for (int iPSB = 0; iPSB < aVPSB.Length(); ++iPSB)
            {
                aVPSB.ChangeValue(iPSB).SetProgressRange(aPSParallel.Next());
            }
            //================================================================
            // Perform analysis
            BOPTools_Parallel.Perform(myRunParallel, aVPSB, myContext!);
            //================================================================
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }

        NCollection_List<NCollection_List<TopoDS_Shape>> aMBlocks = new();
        // Fill map with SD faces to make the blocks
        int aNbPairs = aVPSB.Length();
        for (int i = 0; i < aNbPairs; ++i)
        {
            BOPAlgo_PairOfShapeBoolean aPSB = aVPSB[i];
            if (aPSB.Flag())
            {
                BOPAlgo_Tools.FillMap(aPSB.Shape1(), aPSB.Shape2(), aDMSLS);
            }
        }
        aVPSB.Clear();

        // Make blocks of SD faces using the back and forth map
        BOPAlgo_Tools.MakeBlocks(aDMSLS, aMBlocks, TopTools_ShapeMapHasher.Instance);

        using var aPS = new Message_ProgressScope(aPSOuter.Next(3), "Filling same domain faces map", aMBlocks.Extent());
        // Fill same domain faces map
        NCollection_List<NCollection_List<TopoDS_Shape>>.Iterator aItB = new(aMBlocks);
        for (; aItB.More(); aItB.Next(), aPS.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            NCollection_List<TopoDS_Shape> aLSD = aItB.Value();
            // If the group contains some original faces, the one with minimal
            // index in the DS will be chosen as the SD for the whole group.
            // If there are no original faces in the group, the first face from
            // the group will be used as the SD face.
            // Such SD face will be representative of the whole group in the result.
            TopoDS_Shape? pFSD = null;
            int nFMin = IntegerLast();
            NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLSD);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aF = aItLF.Value();
                // Check the index of the face in DS
                int nF = myDS.Index(aF);
                if (nF >= 0)
                {
                    // The fact that the face is found in the DS, means that
                    // the face has not been change, and thus it is original one.
                    //
                    // Such face does not have any splits, but have an SD face.
                    // Consider it being split.
                    myImages.Bound(aF, new NCollection_List<TopoDS_Shape>()).Append(aF);

                    // For the SD face chose the one with minimal index
                    if (nF < nFMin)
                    {
                        nFMin = nF;
                        pFSD = aF;
                    }
                }
            }

            if (pFSD is null)
            {
                // No original faces in the group, take the first one
                pFSD = aLSD.First();
            }

            // Save all SD connections
            aItLF.Initialize(aLSD);
            for (; aItLF.More(); aItLF.Next())
            {
                TopoDS_Shape aF = aItLF.Value();
                myShapesSD.Bind(aF, pFSD.Value);
            }
        }

        // Update the map of images with SD faces and
        // fill the map of origins.
        int aNbS = myDS.NbSourceShapes();
        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }

            TopoDS_Shape aF = aSI.Shape();
            if (!myImages.Find(aF, out NCollection_List<TopoDS_Shape> pLFIm))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(pLFIm);
            for (; aItLFIm.More(); aItLFIm.Next())
            {
                ref TopoDS_Shape aFIm = ref aItLFIm.ChangeValue();
                if (myShapesSD.Find(aFIm, out TopoDS_Shape pFSD))
                {
                    // Update image with SD face
                    aFIm = pFSD;
                }

                // Fill the map of origins
                if (!myOrigins.Find(aFIm, out NCollection_List<TopoDS_Shape> pLFOr))
                {
                    pLFOr = myOrigins.Bound(aFIm, new NCollection_List<TopoDS_Shape>());
                }
                pLFOr.Append(aF);
            }
        }

        aMBlocks.Clear();
        aDMSLS.Clear();
    }

    /// <summary>Adds the alone vertices of the faces into their splits as internal vertices.</summary>
    protected void FillInternalVertices(Message_ProgressRange theRange)
    {
        using var aPSOuter = new Message_ProgressScope(theRange, null, 1);

        // Vector of pairs of Vertex/Face for classification of the vertices
        // relatively faces, and adding them as internal into the faces
        NCollection_DynamicArray<BOPAlgo_VFI> aVVFI = new();

        int aNbS = myDS!.NbSourceShapes();
        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }

            if (UserBreak(aPSOuter))
            {
                return;
            }

            TopoDS_Shape aF = aSI.Shape();
            if (!myImages.Find(aF, out NCollection_List<TopoDS_Shape> pLFIm))
            {
                continue;
            }

            // Find vertices to add as internal into the splits
            NCollection_List<int> aLIAV = new();
            myDS.AloneVertices(i, aLIAV);

            // Add vertices and faces for classification
            NCollection_List<int>.Iterator aItLV = new(aLIAV);
            for (; aItLV.More(); aItLV.Next())
            {
                TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(aItLV.Value()));
                aV.Orientation(TopAbs_INTERNAL);

                NCollection_List<TopoDS_Shape>.Iterator aItLFIm = new(pLFIm);
                for (; aItLFIm.More(); aItLFIm.Next())
                {
                    TopoDS_Face aFIm = TopoDS.Face(aItLFIm.Value());
                    // Make the pair
                    BOPAlgo_VFI aVFI = aVVFI.Append(new BOPAlgo_VFI());
                    aVFI.SetVertex(aV);
                    aVFI.SetFace(aFIm);
                    aVFI.SetFuzzyValue(myFuzzyValue);
                }
            }
        }

        // Set progress range for each task to be run in parallel
        using (var aPSParallel = new Message_ProgressScope(aPSOuter.Next(), "Looking for internal shapes", aVVFI.Length()))
        {
            for (int iVFI = 0; iVFI < aVVFI.Length(); ++iVFI)
            {
                aVVFI.ChangeValue(iVFI).SetProgressRange(aPSParallel.Next());
            }
            // Perform classification
            //================================================================
            BOPTools_Parallel.Perform(myRunParallel, aVVFI, myContext!);
            //================================================================
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }

        int aNbVFI = aVVFI.Length();
        for (int i = 0; i < aNbVFI; ++i)
        {
            BOPAlgo_VFI aVFI = aVVFI[i];
            if (aVFI.IsInternal())
            {
                TopoDS_Vertex aV = aVFI.Vertex();
                TopoDS_Face aF = aVFI.Face();
                // OCCT adds the vertex into the split face, which may be an argument's face kept as the
                // same-domain face of a group (a frozen argument face then throws, unit 6F)
                new BRep_Builder().Add(aF, aV);
            }
        }
    }

    // Checks if the edge has multi-connected vertices.
    private static bool HasMultiConnected(in TopoDS_Edge theEdge, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        TopoDS_Iterator itV = new(theEdge);
        for (; itV.More(); itV.Next())
        {
            TopoDS_Shape aV = itV.Value();
            if (!theMap.Find(aV, out NCollection_List<TopoDS_Shape> pList))
            {
                pList = theMap.Bound(aV, new NCollection_List<TopoDS_Shape>());
                pList.Append(theEdge);
            }
            else
            {
                // The list is expected to be 1-2 elements long,
                // thus using "Contains" is safe.
                if (!pList.Contains(theEdge))
                {
                    pList.Append(theEdge);
                }

                if (pList.Extent() > 2)
                {
                    return true;
                }
            }
        }
        return false;
    }

    // Builds the draft face, updating the bounding edges according to the information stored into the images map.
    private static TopoDS_Face BuildDraftFace(in TopoDS_Face theFace,
                                             NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages,
                                             IntTools_Context theCtx,
                                             Message_Report theReport)
    {
        BRep_Builder aBB = new();
        // Take the information from the original face
        Geom_Surface? aS = BRep_Tool.Surface(theFace, out TopLoc_Location aLoc);
        double aTol = BRep_Tool.Tolerance(theFace);
        // Make the new face, without any wires
        TopoDS_Face aDraftFace = new();
        aBB.MakeFace(ref aDraftFace, aS, aLoc, aTol);

        // Check if the thin face can be split by a vertex - in this case
        // this vertex will be contained in more than two edges. Thus, count
        // the vertices appearance, and if the multi-connexity is met return
        // the null face to use the BuilderFace algorithm for checking the
        // possibility of split.
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVerticesCounter = new(TopTools_ShapeMapHasher.Instance);

        // Check that the edges of the initial face have not been unified during intersection.
        // Otherwise, it will be necessary to check validity of the new wires.
        NCollection_Map<TopoDS_Shape> aMEdges = new(TopTools_ShapeMapHasher.Instance);

        // Update wires of the original face and add them to draft face
        TopoDS_Iterator aItW = new(theFace.Oriented(TopAbs_FORWARD));
        for (; aItW.More(); aItW.Next())
        {
            TopoDS_Shape aW = aItW.Value();
            if (aW.ShapeType() != TopAbs_WIRE)
            {
                continue;
            }

            // Rebuild wire using images of edges
            TopoDS_Iterator aItE = new(aW.Oriented(TopAbs_FORWARD));
            if (!aItE.More())
            {
                continue;
            }

            TopoDS_Wire aNewWire = new();
            aBB.MakeWire(ref aNewWire);

            for (; aItE.More(); aItE.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aItE.Value());

                TopAbs_Orientation anOriE = aE.Orientation();
                if (anOriE == TopAbs_INTERNAL)
                {
                    // The internal edges could split the original face on halves.
                    // Thus, use the BuilderFace algorithm to build the new face.
                    return new TopoDS_Face();
                }

                // Check if the original edge is degenerated
                bool bIsDegenerated = BRep_Tool.Degenerated(aE);
                // Check if the original edge is closed on the face
                bool bIsClosed = BRep_Tool.IsClosed(aE, theFace);

                // Check for the splits of the edge
                if (!theImages.Find(aE, out NCollection_List<TopoDS_Shape> pLEIm))
                {
                    // Check if the edge has multi-connected vertices
                    if (!bIsDegenerated && HasMultiConnected(aE, aVerticesCounter))
                    {
                        return new TopoDS_Face();
                    }

                    // Check edges unification
                    if (!bIsClosed && !aMEdges.Add(aE))
                    {
                        return new TopoDS_Face();
                    }

                    aBB.Add(aNewWire, aE);
                    continue;
                }

                NCollection_List<TopoDS_Shape>.Iterator aItLEIm = new(pLEIm);
                for (; aItLEIm.More(); aItLEIm.Next())
                {
                    // OCCT changes the orientation of the split in the images map itself (ChangeValue through the
                    // map's const reference); kept
                    ref TopoDS_Shape aSp = ref aItLEIm.ChangeValue();

                    // Check if the split has multi-connected vertices
                    if (!bIsDegenerated && HasMultiConnected(TopoDS.Edge(aSp), aVerticesCounter))
                    {
                        return new TopoDS_Face();
                    }

                    // Check edges unification
                    if (!bIsClosed && !aMEdges.Add(aSp))
                    {
                        return new TopoDS_Face();
                    }

                    aSp.Orientation(anOriE);
                    if (bIsDegenerated)
                    {
                        aBB.Add(aNewWire, aSp);
                        continue;
                    }

                    // If the original edge is closed on the face check closeness
                    // of the split edge and if it is not closed make the second PCurve
                    if (bIsClosed && !BRep_Tool.IsClosed(TopoDS.Edge(aSp), theFace))
                    {
                        BOPTools_AlgoTools3D.DoSplitSEAMOnFace(TopoDS.Edge(aSp), theFace);
                    }

                    // Check if the split should be reversed
                    if (BOPTools_AlgoTools.IsSplitToReverseWithWarn(aSp, aE, theCtx, theReport))
                    {
                        aSp.Reverse();
                    }

                    aBB.Add(aNewWire, aSp);
                }
            }

            aNewWire.Orientation(aW.Orientation());
            aNewWire.Closed(BRep_Tool.IsClosed(aNewWire));
            aBB.Add(aDraftFace, aNewWire);
        }

        if (theFace.Orientation() == TopAbs_REVERSED)
        {
            aDraftFace.Reverse();
        }

        return aDraftFace;
    }
}
