// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ShellSplitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ShellSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Builds the loops (shells) from a set of faces.</summary>
/// <remarks>
/// The irregular connexity blocks are split in parallel (BOPAlgo_CBK solvers, each changing its own block's loops)
/// when <see cref="BOPAlgo_Options.SetRunParallel"/> is set. The allocator constructor goes.
/// </remarks>
public class BOPAlgo_ShellSplitter : BOPAlgo_Algo
{
    protected NCollection_List<Shape> myStartShapes = new();
    protected NCollection_List<Shape> myShells = new();
    protected NCollection_List<BOPTools_ConnexityBlock> myLCB = new();

    public BOPAlgo_ShellSplitter()
    {
    }

    /// <summary>Adds a face to process.</summary>
    public void AddStartElement(in Shape aE) => myStartShapes.Append(aE);

    /// <summary>The faces to process.</summary>
    public NCollection_List<Shape> StartElements() => myStartShapes;

    /// <summary>The shells built.</summary>
    public NCollection_List<Shape> Shells() => myShells;

    /// <summary>Builds the shells.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        GetReport().Clear();
        using var aPS = new ProgressScope(theRange, "Building shells", 1);
        //
        BOPTools_AlgoTools.MakeConnexityBlocks(myStartShapes, ShapeType.Edge, ShapeType.Face, myLCB);
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
        Orientation anOr;
        Edge aEL = new();
        BrepBuilder aBB = new();
        Iterator aItS = new();
        Explorer aExp = new();
        NCollection_List<Shape>.Iterator aItF = new();
        BOPTools_CoupleOfShape aCSOff = new();
        NCollection_Map<Shape> AddedFacesMap = new(EqualityComparer<Shape>.Default);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aEFMap = new(ShapeHasher.Instance),
                                                                                  aMEFP = new(ShapeHasher.Instance);
        IntTools_Context aContext;
        //
        aContext = new IntTools_Context();
        //
        NCollection_List<Shape> myShapes = aCB.Shapes();
        //
        NCollection_List<Shape> myLoops = aCB.ChangeLoops();
        myLoops.Clear();
        //
        // Copy faces into the map, for recursive search of free bounds
        NCollection_Map<Shape> aMFaces = new(EqualityComparer<Shape>.Default);
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
                Shape aF = aItF.Value();
                if (aMFaces.Contains(aF))
                {
                    Exploration.MapShapesAndAncestors(aF, ShapeType.Edge, ShapeType.Face, aEFMap);
                }
            }
            //
            int aNbBegin = aMFaces.Extent();
            // check the free edges
            int aNbE = aEFMap.Extent();
            for (i = 1; i <= aNbE; ++i)
            {
                Edge aE = ShapeCasts.Edge(aEFMap.FindKey(i));
                if (!Tool.Degenerated(aE) && aE.Orientation() != Internal)
                {
                    NCollection_List<Shape> aLF = aEFMap[i];
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
        NCollection_List<Shape> aLFConnected = new();
        // Boundary faces
        NCollection_Map<Shape> aBoundaryFaces = new(ShapeHasher.Instance);
        aItF.Initialize(myShapes);
        for (; aItF.More(); aItF.Next())
        {
            Shape aF = aItF.Value();
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
            Shape aFF = aItF.Value();
            if (!AddedFacesMap.Add(aFF))
            {
                continue;
            }
            //
            // make a new shell
            Shell aShell = new();
            aBB.MakeShell(ref aShell);
            aBB.Add(aShell, aFF);

            aMEFP.Clear();
            Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, aMEFP);
            //
            // loop on faces added to Shell;
            // add their neighbor faces to Shell and so on
            aItS.Initialize(aShell);
            for (; aItS.More(); aItS.Next())
            {
                Face aF = ShapeCasts.Face(aItS.Value());
                bool isBoundary = aBoundaryFaces.Contains(aF);
                //
                // loop on edges of aF; find a good neighbor face of aF by aE
                aExp.Init(aF, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Edge aE = ShapeCasts.Edge(aExp.Current());
                    //
                    // proceed only free edges in this shell
                    if (aMEFP.Contains(aE))
                    {
                        NCollection_List<Shape> aLFP = aMEFP.FindFromKey(aE);
                        aNbFP = aLFP.Extent();
                        if (aNbFP > 1)
                        {
                            continue;
                        }
                    }
                    // avoid processing of internal edges
                    anOr = aE.Orientation();
                    if (anOr == Internal)
                    {
                        continue;
                    }
                    // avoid processing of degenerated edges
                    if (Tool.Degenerated(aE))
                    {
                        continue;
                    }
                    //
                    // candidate faces list
                    NCollection_List<Shape> aLF = aEFMap.FindFromKey(aE);
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
                    Face aSelF = new();
                    NCollection_List<Shape>.Iterator aItLF = new(aLF);
                    for (; aItLF.More(); aItLF.Next())
                    {
                        Face aFL = ShapeCasts.Face(aItLF.Value());
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
                            aSelF = ShapeCasts.Face(aLCSOff.First().Shape2());
                        }
                        else if (aNbOff > 1)
                        {
                            BOPTools_AlgoTools.GetFaceOff(aE, aF, aLCSOff, ref aSelF, aContext);
                        }
                    }
                    //
                    if (!aSelF.IsNull && AddedFacesMap.Add(aSelF))
                    {
                        aBB.Add(aShell, aSelF);
                        Exploration.MapShapesAndAncestors(aSelF, ShapeType.Edge, ShapeType.Face, aMEFP);
                    }
                } // for (; aExp.More(); aExp.Next()) {
            } // for (; aItS.More(); aItS.Next()) {
            //
            // split the shell on multi-connected edges
            NCollection_List<Shape> aLShSp = new();
            RefineShell(aShell, aMEFP, aLShSp);
            //
            // collect the not closed shells for further processing
            NCollection_List<Shape> aLShNC = new();
            //
            NCollection_List<Shape>.Iterator aItLShSp = new(aLShSp);
            for (; aItLShSp.More(); aItLShSp.Next())
            {
                Shell aShSp = ShapeCasts.Shell(aItLShSp.Value());
                //
                if (Tool.IsClosed(aShSp))
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
            NCollection_List<Shape>.Iterator aItLShNC = new(aLShNC);
            for (; aItLShNC.More(); aItLShNC.Next())
            {
                Iterator aItNC = new(aItLShNC.Value());
                for (; aItNC.More(); aItNC.Next())
                {
                    AddedFacesMap.Remove(aItNC.Value());
                }
            }
        } // for (; aItF.More(); aItF.Next()) {
    }

    // The sub-shape of theShape that is the same as theShapeToFind (OCCT's file-level FindShape).
    private static Shape FindShape(in Shape theShapeToFind, in Shape theShape)
    {
        Shape aRes = new();
        Explorer anExp = new(theShape, theShapeToFind.ShapeType());
        for (; anExp.More(); anExp.Next())
        {
            Shape aShape = anExp.Current();
            if (aShape.IsSame(theShapeToFind))
            {
                aRes = aShape;
                break;
            }
        }
        return aRes;
    }

    // Splits the shell on its branch edges (OCCT's file-static RefineShell).
    private static void RefineShell(in Shell theShell,
                                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMEF,
                                    NCollection_List<Shape> theLShSp)
    {
        Iterator aIt = new(theShell);
        if (!aIt.More())
        {
            return;
        }
        //
        // Find edges with more than 2 adjacent faces - branch edges -
        // edges on which the input shell should be split
        NCollection_Map<Shape> aMEStop = new(ShapeHasher.Instance);
        //
        int i, aNbMEF = theMEF.Extent();
        for (i = 1; i <= aNbMEF; ++i)
        {
            Edge aE = ShapeCasts.Edge(theMEF.FindKey(i));
            NCollection_List<Shape> aLF = theMEF[i];
            if (aLF.Extent() > 2)
            {
                aMEStop.Add(aE);
                continue;
            }

            if (aLF.Extent() == 2)
            {
                Face aF1 = ShapeCasts.Face(aLF.First());
                Face aF2 = ShapeCasts.Face(aLF.Last());

                Shape aE1 = FindShape(aE, aF1);
                Shape aE2 = FindShape(aE, aF2);

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
            NCollection_List<Shape>.Iterator aItLF0 = new(aLF);
            for (; aItLF0.More() && aNbF <= 2; aItLF0.Next())
            {
                Face aF = ShapeCasts.Face(aItLF0.Value());
                ++aNbF;
                Explorer aExp0 = new(aF, ShapeType.Edge);
                for (; aExp0.More(); aExp0.Next())
                {
                    Shape aEF = aExp0.Current();
                    if (aEF.IsSame(aE))
                    {
                        if (aEF.Orientation() == Internal)
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
        Builder aBB = new();
        Explorer aExp = new();
        NCollection_IndexedMap<Shape> aMFB = new(ShapeHasher.Instance);
        NCollection_Map<Shape> aMFProcessed = new(EqualityComparer<Shape>.Default);
        NCollection_List<Shape> aLFP = new(), aLFP1 = new();
        NCollection_List<Shape>.Iterator aItLF = new(), aItLFP = new();
        //
        // The first Face
        for (; aIt.More(); aIt.Next())
        {
            Shape aF1 = aIt.Value();
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
                    Shape aFP = aItLFP.Value();
                    //
                    aExp.Init(aFP, ShapeType.Edge);
                    for (; aExp.More(); aExp.Next())
                    {
                        Edge aE = ShapeCasts.Edge(aExp.Current());
                        if (aMEStop.Contains(aE))
                        {
                            continue;
                        }
                        //
                        if (aE.Orientation() == Internal)
                        {
                            continue;
                        }
                        //
                        if (Tool.Degenerated(aE))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape> aLF = theMEF.FindFromKey(aE);
                        //
                        aItLF.Initialize(aLF);
                        for (; aItLF.More(); aItLF.Next())
                        {
                            Shape aFP1 = aItLF.Value();
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
                Shell aShSp = new();
                aBB.MakeShell(ref aShSp);
                //
                for (i = 1; i <= aNbMFB; ++i)
                {
                    Shape aFB = aMFB[i];
                    aBB.Add(aShSp, aFB);
                }
                theLShSp.Append(aShSp);
            }
        } // for (; aIt.More(); aIt.Next()) {
    }

    /// <summary>Makes a shell of each regular block, and splits the others (in parallel).</summary>
    protected void MakeShells(ProgressRange theRange)
    {
        bool bIsRegular;
        int aNbVCBK, k;
        NCollection_List<BOPTools_ConnexityBlock>.Iterator aItCB = new();
        NCollection_List<Shape>.Iterator aIt = new();
        NCollection_DynamicArray<BOPAlgo_CBK> aVCBK = new();
        //
        using var aPSOuter = new ProgressScope(theRange, null, 1);
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
                Shell aShell = new();
                //
                NCollection_List<Shape> aLF = aCB.Shapes();
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
        using var aPSParallel = new ProgressScope(aPSOuter.Next(), null, aNbVCBK);
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
            NCollection_List<Shape> aLS = aCB.Loops();
            aIt.Initialize(aLS);
            for (; aIt.More(); aIt.Next())
            {
                Shape aShell = aIt.Value();
                aShell.Closed(true);
                myShells.Append(aShell);
            }
        }
    }

    // Makes a shell of the faces and orients them (OCCT's file-static MakeShell).
    private static void MakeShell(NCollection_List<Shape> aLS, ref Shell aShell)
    {
        BrepBuilder aBB = new();
        NCollection_List<Shape>.Iterator aIt = new();
        //
        aBB.MakeShell(ref aShell);
        //
        aIt.Initialize(aLS);
        for (; aIt.More(); aIt.Next())
        {
            Shape aF = aIt.Value();
            aBB.Add(aShell, aF);
        }
        //
        Shape aSh = aShell;
        BOPTools_AlgoTools.OrientFacesOnShell(ref aSh);
        aShell = ShapeCasts.Shell(aSh);
    }
}

/// <summary>The solver splitting one irregular connexity block (OCCT's file-local BOPAlgo_CBK).</summary>
/// <remarks>It refers to the splitter's own block, whose loops <see cref="BOPAlgo_ShellSplitter.SplitBlock"/> fills.</remarks>
internal sealed class BOPAlgo_CBK : BOPTools_Parallel.ISolver
{
    private BOPTools_ConnexityBlock? myPCB;
    private ProgressRange myProgressRange;

    public BOPAlgo_CBK()
    {
        myPCB = null;
    }

    //
    public void SetConnexityBlock(BOPTools_ConnexityBlock aCB) => myPCB = aCB;

    //
    public BOPTools_ConnexityBlock ConnexityBlock() => myPCB!;

    //
    public void SetProgressRange(ProgressRange theRange) => myProgressRange = theRange;

    //
    public void Perform()
    {
        using var aPS = new ProgressScope(myProgressRange, null, 1);
        if (!aPS.More())
        {
            return;
        }
        BOPAlgo_ShellSplitter.SplitBlock(myPCB!);
    }
}
