// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ShellSplitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ShellSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Builds the loops (shells) from a set of faces.</summary>
/// <remarks>
/// The irregular connexity blocks are split in parallel (BOPAlgo_CBK solvers, each changing its own block's loops)
/// when <see cref="BOPAlgo_Options.SetRunParallel"/> is set. The allocator constructor goes.
/// </remarks>
public class BOPAlgo_ShellSplitter : BOPAlgo_Algo
{
    protected NCollection_List<TopoDS_Shape> myStartShapes = new();
    protected NCollection_List<TopoDS_Shape> myShells = new();
    protected NCollection_List<BOPTools_ConnexityBlock> myLCB = new();

    public BOPAlgo_ShellSplitter()
    {
    }

    /// <summary>Adds a face to process.</summary>
    public void AddStartElement(in TopoDS_Shape aE) => myStartShapes.Append(aE);

    /// <summary>The faces to process.</summary>
    public NCollection_List<TopoDS_Shape> StartElements() => myStartShapes;

    /// <summary>The shells built.</summary>
    public NCollection_List<TopoDS_Shape> Shells() => myShells;

    /// <summary>Builds the shells.</summary>
    public override void Perform(Message_ProgressRange theRange = default)
    {
        GetReport().Clear();
        using var aPS = new Message_ProgressScope(theRange, "Building shells", 1);
        //
        BOPTools_AlgoTools.MakeConnexityBlocks(myStartShapes, TopAbs_EDGE, TopAbs_FACE, myLCB);
        if (UserBreak(aPS))
        {
            return;
        }

        MakeShells(aPS.Next());
    }

    /// <summary>Splits the (irregular) connexity block into shells, put into its loops.</summary>
    public static void SplitBlock(BOPTools_ConnexityBlock aCB)
    {
        int aNbLF, aNbOff, aNbFP;
        int i;
        TopAbs_Orientation anOr;
        TopoDS_Edge aEL = new();
        BRep_Builder aBB = new();
        TopoDS_Iterator aItS = new();
        TopExp_Explorer aExp = new();
        NCollection_List<TopoDS_Shape>.Iterator aItF = new();
        BOPTools_CoupleOfShape aCSOff = new();
        NCollection_Map<TopoDS_Shape> AddedFacesMap = new(EqualityComparer<TopoDS_Shape>.Default);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aEFMap = new(TopTools_ShapeMapHasher.Instance),
                                                                                  aMEFP = new(TopTools_ShapeMapHasher.Instance);
        IntTools_Context aContext;
        //
        aContext = new IntTools_Context();
        //
        NCollection_List<TopoDS_Shape> myShapes = aCB.Shapes();
        //
        NCollection_List<TopoDS_Shape> myLoops = aCB.ChangeLoops();
        myLoops.Clear();
        //
        // Copy faces into the map, for recursive search of free bounds
        NCollection_Map<TopoDS_Shape> aMFaces = new(EqualityComparer<TopoDS_Shape>.Default);
        aItF.Initialize(myShapes);
        for (; aItF.More(); aItF.Next())
        {
            aMFaces.Add(aItF.Value());
        }
        //
        // remove the faces with free edges from processing
        for (;;)
        {
            // map the shapes
            aEFMap.Clear();
            aItF.Initialize(myShapes);
            for (; aItF.More(); aItF.Next())
            {
                TopoDS_Shape aF = aItF.Value();
                if (aMFaces.Contains(aF))
                {
                    TopExp.MapShapesAndAncestors(aF, TopAbs_EDGE, TopAbs_FACE, aEFMap);
                }
            }
            //
            int aNbBegin = aMFaces.Extent();
            // check the free edges
            int aNbE = aEFMap.Extent();
            for (i = 1; i <= aNbE; ++i)
            {
                TopoDS_Edge aE = TopoDS.Edge(aEFMap.FindKey(i));
                if (!BRep_Tool.Degenerated(aE) && aE.Orientation() != TopAbs_INTERNAL)
                {
                    NCollection_List<TopoDS_Shape> aLF = aEFMap[i];
                    if (aLF.Extent() == 1)
                    {
                        // remove the face
                        aMFaces.Remove(aLF.First());
                    }
                }
            }
            //
            // check if any faces have been removed
            int aNbEnd = aMFaces.Extent();
            if ((aNbEnd == aNbBegin) || (aNbEnd == 0))
            {
                break;
            }
        }
        //
        if (aMFaces.IsEmpty())
        {
            return;
        }
        //
        // use only connected faces
        NCollection_List<TopoDS_Shape> aLFConnected = new();
        // Boundary faces
        NCollection_Map<TopoDS_Shape> aBoundaryFaces = new(TopTools_ShapeMapHasher.Instance);
        aItF.Initialize(myShapes);
        for (; aItF.More(); aItF.Next())
        {
            TopoDS_Shape aF = aItF.Value();
            if (aMFaces.Contains(aF))
            {
                aLFConnected.Append(aF);
                if (!aBoundaryFaces.Add(aF))
                {
                    aBoundaryFaces.Remove(aF);
                }
            }
        }
        //
        int aNbShapes = aLFConnected.Extent();
        bool bAllFacesTaken = false;
        //
        // Build the shells
        aItF.Initialize(aLFConnected);
        for (i = 1; aItF.More() && !bAllFacesTaken; aItF.Next(), ++i)
        {
            TopoDS_Shape aFF = aItF.Value();
            if (!AddedFacesMap.Add(aFF))
            {
                continue;
            }
            //
            // make a new shell
            TopoDS_Shell aShell = new();
            aBB.MakeShell(ref aShell);
            aBB.Add(aShell, aFF);

            aMEFP.Clear();
            TopExp.MapShapesAndAncestors(aShell, TopAbs_EDGE, TopAbs_FACE, aMEFP);
            //
            // loop on faces added to Shell;
            // add their neighbor faces to Shell and so on
            aItS.Initialize(aShell);
            for (; aItS.More(); aItS.Next())
            {
                TopoDS_Face aF = TopoDS.Face(aItS.Value());
                bool isBoundary = aBoundaryFaces.Contains(aF);
                //
                // loop on edges of aF; find a good neighbor face of aF by aE
                aExp.Init(aF, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                    //
                    // proceed only free edges in this shell
                    if (aMEFP.Contains(aE))
                    {
                        NCollection_List<TopoDS_Shape> aLFP = aMEFP.FindFromKey(aE);
                        aNbFP = aLFP.Extent();
                        if (aNbFP > 1)
                        {
                            continue;
                        }
                    }
                    // avoid processing of internal edges
                    anOr = aE.Orientation();
                    if (anOr == TopAbs_INTERNAL)
                    {
                        continue;
                    }
                    // avoid processing of degenerated edges
                    if (BRep_Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    //
                    // candidate faces list
                    NCollection_List<TopoDS_Shape> aLF = aEFMap.FindFromKey(aE);
                    aNbLF = aLF.Extent();
                    if (aNbLF == 0)
                    {
                        continue;
                    }
                    //
                    // prepare for selecting the next face
                    // take only not-processed faces as a candidates
                    NCollection_List<BOPTools_CoupleOfShape> aLCSOff = new();
                    //
                    int aNbWaysInside = 0;
                    TopoDS_Face aSelF = new();
                    NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLF);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        TopoDS_Face aFL = TopoDS.Face(aItLF.Value());
                        if (aF.IsSame(aFL) || AddedFacesMap.Contains(aFL))
                        {
                            continue;
                        }
                        //
                        // find current edge in the face
                        if (!BOPTools_AlgoTools.GetEdgeOff(aE, aFL, ref aEL))
                        {
                            continue;
                        }
                        //
                        if (isBoundary && !aBoundaryFaces.Contains(aFL))
                        {
                            ++aNbWaysInside;
                            aSelF = aFL;
                        }
                        aCSOff.SetShape1(aEL);
                        aCSOff.SetShape2(aFL);
                        aLCSOff.Append(aCSOff);
                    } // for (; aItLF.More(); aItLF.Next()) {
                    //
                    aNbOff = aLCSOff.Extent();
                    if (aNbOff == 0)
                    {
                        continue;
                    }
                    //
                    // among all the adjacent faces chose one with the minimal
                    // angle to the current one
                    if (!isBoundary || aNbWaysInside != 1)
                    {
                        if (aNbOff == 1)
                        {
                            aSelF = TopoDS.Face(aLCSOff.First().Shape2());
                        }
                        else if (aNbOff > 1)
                        {
                            BOPTools_AlgoTools.GetFaceOff(aE, aF, aLCSOff, ref aSelF, aContext);
                        }
                    }
                    //
                    if (!aSelF.IsNull() && AddedFacesMap.Add(aSelF))
                    {
                        aBB.Add(aShell, aSelF);
                        TopExp.MapShapesAndAncestors(aSelF, TopAbs_EDGE, TopAbs_FACE, aMEFP);
                    }
                } // for (; aExp.More(); aExp.Next()) {
            } // for (; aItS.More(); aItS.Next()) {
            //
            // split the shell on multi-connected edges
            NCollection_List<TopoDS_Shape> aLShSp = new();
            RefineShell(aShell, aMEFP, aLShSp);
            //
            // collect the not closed shells for further processing
            NCollection_List<TopoDS_Shape> aLShNC = new();
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLShSp = new(aLShSp);
            for (; aItLShSp.More(); aItLShSp.Next())
            {
                TopoDS_Shell aShSp = TopoDS.Shell(aItLShSp.Value());
                //
                if (BRep_Tool.IsClosed(aShSp))
                {
                    aShSp.Closed(true);
                    myLoops.Append(aShSp);
                }
                else
                {
                    aLShNC.Append(aShSp);
                }
            }
            //
            bAllFacesTaken = (AddedFacesMap.Extent() == aNbShapes);
            if (bAllFacesTaken)
            {
                break;
            }
            //
            if (aLShSp.Extent() == 1)
            {
                // not further processing of not closed shells is needed,
                // as it will not bring any new results
                continue;
            }
            //

            // remove th faces of not closed shells from the map of processed faces
            // and try to rebuild the shells using all not processed faces,
            // because faces of one shell might be needed for building the other
            NCollection_List<TopoDS_Shape>.Iterator aItLShNC = new(aLShNC);
            for (; aItLShNC.More(); aItLShNC.Next())
            {
                TopoDS_Iterator aItNC = new(aItLShNC.Value());
                for (; aItNC.More(); aItNC.Next())
                {
                    AddedFacesMap.Remove(aItNC.Value());
                }
            }
        } // for (; aItF.More(); aItF.Next()) {
    }

    // The sub-shape of theShape that is the same as theShapeToFind (OCCT's file-level FindShape).
    private static TopoDS_Shape FindShape(in TopoDS_Shape theShapeToFind, in TopoDS_Shape theShape)
    {
        TopoDS_Shape aRes = new();
        TopExp_Explorer anExp = new(theShape, theShapeToFind.ShapeType());
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Shape aShape = anExp.Current();
            if (aShape.IsSame(theShapeToFind))
            {
                aRes = aShape;
                break;
            }
        }
        return aRes;
    }

    // Splits the shell on its branch edges (OCCT's file-static RefineShell).
    private static void RefineShell(in TopoDS_Shell theShell,
                                    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMEF,
                                    NCollection_List<TopoDS_Shape> theLShSp)
    {
        TopoDS_Iterator aIt = new(theShell);
        if (!aIt.More())
        {
            return;
        }
        //
        // Find edges with more than 2 adjacent faces - branch edges -
        // edges on which the input shell should be split
        NCollection_Map<TopoDS_Shape> aMEStop = new(TopTools_ShapeMapHasher.Instance);
        //
        int i, aNbMEF = theMEF.Extent();
        for (i = 1; i <= aNbMEF; ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(theMEF.FindKey(i));
            NCollection_List<TopoDS_Shape> aLF = theMEF[i];
            if (aLF.Extent() > 2)
            {
                aMEStop.Add(aE);
                continue;
            }

            if (aLF.Extent() == 2)
            {
                TopoDS_Face aF1 = TopoDS.Face(aLF.First());
                TopoDS_Face aF2 = TopoDS.Face(aLF.Last());

                TopoDS_Shape aE1 = FindShape(aE, aF1);
                TopoDS_Shape aE2 = FindShape(aE, aF2);

                if (aE1.Orientation() == aE2.Orientation())
                {
                    aMEStop.Add(aE);
                    continue;
                }
            }
            //
            // check for internal edges - count faces, in which the edge
            // is internal, twice
            int aNbF = 0;
            NCollection_List<TopoDS_Shape>.Iterator aItLF0 = new(aLF);
            for (; aItLF0.More() && aNbF <= 2; aItLF0.Next())
            {
                TopoDS_Face aF = TopoDS.Face(aItLF0.Value());
                ++aNbF;
                TopExp_Explorer aExp0 = new(aF, TopAbs_EDGE);
                for (; aExp0.More(); aExp0.Next())
                {
                    TopoDS_Shape aEF = aExp0.Current();
                    if (aEF.IsSame(aE))
                    {
                        if (aEF.Orientation() == TopAbs_INTERNAL)
                        {
                            ++aNbF;
                        }
                        break;
                    }
                }
            }
            //
            if (aNbF > 2)
            {
                aMEStop.Add(aE);
            }
        }
        //
        if (aMEStop.IsEmpty())
        {
            theLShSp.Append(theShell);
            return;
        }
        //
        TopoDS_Builder aBB = new();
        TopExp_Explorer aExp = new();
        NCollection_IndexedMap<TopoDS_Shape> aMFB = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aMFProcessed = new(EqualityComparer<TopoDS_Shape>.Default);
        NCollection_List<TopoDS_Shape> aLFP = new(), aLFP1 = new();
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(), aItLFP = new();
        //
        // The first Face
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aF1 = aIt.Value();
            if (!aMFProcessed.Add(aF1))
            {
                continue;
            }
            //
            aMFB.Clear();
            aLFP.Clear();
            //
            aMFB.Add(aF1);
            aLFP.Append(aF1);
            //
            // Trying to reach the branch point
            for (;;)
            {
                aItLFP.Initialize(aLFP);
                for (; aItLFP.More(); aItLFP.Next())
                {
                    TopoDS_Shape aFP = aItLFP.Value();
                    //
                    aExp.Init(aFP, TopAbs_EDGE);
                    for (; aExp.More(); aExp.Next())
                    {
                        TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                        if (aMEStop.Contains(aE))
                        {
                            continue;
                        }
                        //
                        if (aE.Orientation() == TopAbs_INTERNAL)
                        {
                            continue;
                        }
                        //
                        if (BRep_Tool.Degenerated(aE))
                        {
                            continue;
                        }
                        //
                        NCollection_List<TopoDS_Shape> aLF = theMEF.FindFromKey(aE);
                        //
                        aItLF.Initialize(aLF);
                        for (; aItLF.More(); aItLF.Next())
                        {
                            TopoDS_Shape aFP1 = aItLF.Value();
                            if (aFP1.IsSame(aFP))
                            {
                                continue;
                            }
                            if (aMFB.Contains(aFP1))
                            {
                                continue;
                            }
                            //
                            if (aMFProcessed.Add(aFP1))
                            {
                                aMFB.Add(aFP1);
                                aLFP1.Append(aFP1);
                            }
                        } // for (; aItLF.More(); aItLF.Next()) {
                    } // for (; aExp.More(); aExp.Next()) {
                } // for (; aItLFP.More(); aItLFP.Next()) {
                //
                //
                if (aLFP1.IsEmpty())
                {
                    break;
                }
                //
                aLFP.Clear();
                aLFP.Append(aLFP1);
            } // for (;;) {
            //
            int aNbMFB = aMFB.Extent();
            if (aNbMFB != 0)
            {
                TopoDS_Shell aShSp = new();
                aBB.MakeShell(ref aShSp);
                //
                for (i = 1; i <= aNbMFB; ++i)
                {
                    TopoDS_Shape aFB = aMFB[i];
                    aBB.Add(aShSp, aFB);
                }
                theLShSp.Append(aShSp);
            }
        } // for (; aIt.More(); aIt.Next()) {
    }

    /// <summary>Makes a shell of each regular block, and splits the others (in parallel).</summary>
    protected void MakeShells(Message_ProgressRange theRange)
    {
        bool bIsRegular;
        int aNbVCBK, k;
        NCollection_List<BOPTools_ConnexityBlock>.Iterator aItCB = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        NCollection_DynamicArray<BOPAlgo_CBK> aVCBK = new();
        //
        using var aPSOuter = new Message_ProgressScope(theRange, null, 1);
        myShells.Clear();
        //
        aItCB.Initialize(myLCB);
        for (; aItCB.More(); aItCB.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPTools_ConnexityBlock aCB = aItCB.Value();
            bIsRegular = aCB.IsRegular();
            if (bIsRegular)
            {
                TopoDS_Shell aShell = new();
                //
                NCollection_List<TopoDS_Shape> aLF = aCB.Shapes();
                MakeShell(aLF, ref aShell);
                aShell.Closed(true);
                myShells.Append(aShell);
            }
            else
            {
                BOPAlgo_CBK aCBK = aVCBK.Append(new BOPAlgo_CBK());
                aCBK.SetConnexityBlock(aCB);
            }
        }
        //
        aNbVCBK = aVCBK.Length();
        using var aPSParallel = new Message_ProgressScope(aPSOuter.Next(), null, aNbVCBK);
        for (int iS = 0; iS < aNbVCBK; ++iS)
        {
            aVCBK.ChangeValue(iS).SetProgressRange(aPSParallel.Next());
        }
        //===================================================
        BOPTools_Parallel.Perform(myRunParallel, aVCBK);
        //===================================================
        for (k = 0; k < aNbVCBK; ++k)
        {
            BOPAlgo_CBK aCBK = aVCBK[k];
            BOPTools_ConnexityBlock aCB = aCBK.ConnexityBlock();
            NCollection_List<TopoDS_Shape> aLS = aCB.Loops();
            aIt.Initialize(aLS);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aShell = aIt.Value();
                aShell.Closed(true);
                myShells.Append(aShell);
            }
        }
    }

    // Makes a shell of the faces and orients them (OCCT's file-static MakeShell).
    private static void MakeShell(NCollection_List<TopoDS_Shape> aLS, ref TopoDS_Shell aShell)
    {
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        aBB.MakeShell(ref aShell);
        //
        aIt.Initialize(aLS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aF = aIt.Value();
            aBB.Add(aShell, aF);
        }
        //
        TopoDS_Shape aSh = aShell;
        BOPTools_AlgoTools.OrientFacesOnShell(ref aSh);
        aShell = TopoDS.Shell(aSh);
    }
}

/// <summary>The solver splitting one irregular connexity block (OCCT's file-local BOPAlgo_CBK).</summary>
/// <remarks>It refers to the splitter's own block, whose loops <see cref="BOPAlgo_ShellSplitter.SplitBlock"/> fills.</remarks>
internal sealed class BOPAlgo_CBK : BOPTools_Parallel.ISolver
{
    private BOPTools_ConnexityBlock? myPCB;
    private Message_ProgressRange myProgressRange;

    public BOPAlgo_CBK()
    {
        myPCB = null;
    }

    //
    public void SetConnexityBlock(BOPTools_ConnexityBlock aCB) => myPCB = aCB;

    //
    public BOPTools_ConnexityBlock ConnexityBlock() => myPCB!;

    //
    public void SetProgressRange(Message_ProgressRange theRange) => myProgressRange = theRange;

    //
    public void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (!aPS.More())
        {
            return;
        }
        BOPAlgo_ShellSplitter.SplitBlock(myPCB!);
    }
}
