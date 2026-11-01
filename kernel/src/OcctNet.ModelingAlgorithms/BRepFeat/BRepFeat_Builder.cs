// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_Operation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Provides a basic tool to implement features topological operations: a Boolean operation of the basis shape with a
/// tool, in two steps. First the tool's parts are computed (<see cref="PartsOfTool"/>); the caller keeps some of them
/// (<see cref="KeepParts"/>), and <see cref="PerformResult"/> builds the result from the kept parts only, removing the
/// others' faces from the result.
/// </summary>
/// <remarks>
/// A BOPAlgo_BOP: its arguments are read as the Boolean operations read theirs, never written (decision 274).
/// </remarks>
public class BRepFeat_Builder : BOPAlgo_BOP
{
    protected NCollection_Map<TopoDS_Shape> myShapes = new(TopTools_ShapeMapHasher.Instance);
    protected NCollection_Map<TopoDS_Shape> myRemoved = new(TopTools_ShapeMapHasher.Instance);
    protected int myFuse;

    public BRepFeat_Builder()
    {
        Clear();
    }

    /// <summary>Clears internal fields and arguments.</summary>
    public override void Clear()
    {
        myShapes?.Clear();
        myRemoved?.Clear();
        base.Clear();
    }

    /// <summary>Initializes the object of local boolean operation.</summary>
    public void Init(in TopoDS_Shape theShape)
    {
        Clear();
        //
        AddArgument(theShape);
    }

    /// <summary>Initializes the arguments of local boolean operation.</summary>
    public void Init(in TopoDS_Shape theShape, in TopoDS_Shape theTool)
    {
        Clear();
        //
        AddArgument(theShape);
        AddTool(theTool);
    }

    /// <summary>Sets the operation of local boolean operation: a cut if <paramref name="theFuse"/> is 0, else a fuse.</summary>
    public void SetOperation(int theFuse)
    {
        myFuse = theFuse;
        myOperation = myFuse != 0 ? BOPAlgo_FUSE : BOPAlgo_CUT;
    }

    /// <summary>
    /// Sets the operation of local boolean operation. When <paramref name="theFlag"/> is true no selection of the tool's
    /// parts is needed (no second part): the operation is then COMMON if <paramref name="theFuse"/> is 0, else CUT21.
    /// When false, as <see cref="SetOperation(int)"/>.
    /// </summary>
    public void SetOperation(int theFuse, bool theFlag)
    {
        myFuse = theFuse;
        if (!theFlag)
        {
            myOperation = myFuse != 0 ? BOPAlgo_FUSE : BOPAlgo_CUT;
        }
        else
        {
            myOperation = myFuse != 0 ? BOPAlgo_CUT21 : BOPAlgo_COMMON;
        }
    }

    /// <summary>Collects parts of the tool (the solids of the first step's result; not frozen).</summary>
    public void PartsOfTool(NCollection_List<TopoDS_Shape> aLT)
    {
        TopExp_Explorer aExp = new();
        //
        aLT.Clear();
        aExp.Init(myShape, TopAbs_SOLID);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aS = aExp.Current();
            aLT.Append(aS);
        }
    }

    /// <summary>Initializes parts of the tool for the second step: collects the shapes and their sub-shapes.</summary>
    public void KeepParts(NCollection_List<TopoDS_Shape> theIm)
    {
        NCollection_List<TopoDS_Shape>.Iterator aItT = new();
        aItT.Initialize(theIm);
        for (; aItT.More(); aItT.Next())
        {
            TopoDS_Shape aTIm = aItT.Value();
            KeepPart(aTIm);
        }
    }

    /// <summary>Adds the shape and all its sub-shapes into the kept shapes.</summary>
    public void KeepPart(in TopoDS_Shape thePart)
    {
        TopExp.MapShapes(thePart, myShapes);
    }

    /// <summary>Prepares builder of local operation.</summary>
    protected override void Prepare()
    {
        GetReport().Clear();
        //
        BRep_Builder aBB = new();
        TopoDS_Compound aC = new();
        aBB.MakeCompound(ref aC);
        myShape = aC;
        //
        FillRemoved();
    }

    /// <summary>Collects the removed parts of the tool.</summary>
    public void FillRemoved()
    {
        TopExp_Explorer aExp = new();
        //
        TopoDS_Shape aArgs0 = myArguments.First();
        TopoDS_Shape aArgs1 = myTools.First();
        //
        aExp.Init(aArgs0, TopAbs_SOLID);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aS = aExp.Current();
            myImages.UnBind(aS);
        }
        //
        if (!myImages.IsBound(aArgs1))
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape>.Iterator aItIm = new();
        //
        NCollection_List<TopoDS_Shape> aLS = myImages.ChangeFind(aArgs1);
        aItIm.Initialize(aLS);
        for (; aItIm.More(); aItIm.Next())
        {
            TopoDS_Shape aS = aItIm.Value();
            FillRemoved(aS, myRemoved);
        }
    }

    /// <summary>Main function to build the result of the local operation required.</summary>
    public void PerformResult(Message_ProgressRange theRange = default)
    {
        myOperation = myFuse != 0 ? BOPAlgo_FUSE : BOPAlgo_CUT;
        if (myShapes.IsEmpty())
        {
            BuildShape(theRange);
            return;
        }

        double aWhole = 100.0;
        using var aPS = new Message_ProgressScope(theRange, "BRepFeat_Builder", aWhole);
        double aBSPart = 15;
        aWhole -= aBSPart;

        // Compute PI steps
        const int aSize = 4;
        NCollection_Array1<double> aSteps = new(0, aSize - 1);
        {
            for (int i = 0; i < aSize; ++i)
            {
                aSteps[i] = 0.0;
            }

            NbShapes aNbShapes = getNbShapes();
            double aTreatFaces = 5 * aNbShapes.NbFaces();
            double aTreatShells = aNbShapes.NbShells();
            double aTreatSolids = 20 * aNbShapes.NbSolids();
            double aTreatCompounds = aNbShapes.NbCompounds();

            double aSum = aTreatFaces + aTreatShells + aTreatSolids + aTreatCompounds;
            if (aSum > 0)
            {
                aSteps[0] = aTreatFaces * aWhole / aSum;
                aSteps[1] = aTreatShells * aWhole / aSum;
                aSteps[2] = aTreatSolids * aWhole / aSum;
                aSteps[3] = aTreatCompounds * aWhole / aSum;
            }
        }
        //
        Prepare();
        //
        RebuildFaces();
        aPS.Next(aSteps[0]);
        //
        FillImagesContainers(TopAbs_SHELL, aPS.Next(aSteps[1]));
        if (HasErrors())
        {
            return;
        }
        //
        FillImagesSolids(aPS.Next(aSteps[2]));
        if (HasErrors())
        {
            return;
        }
        //
        CheckSolidImages();
        //
        BuildResult(TopAbs_SOLID);
        if (HasErrors())
        {
            return;
        }
        //
        FillImagesCompounds(aPS.Next(aSteps[3]));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_COMPOUND);
        if (HasErrors())
        {
            return;
        }
        //
        BuildShape(aPS.Next(aBSPart));
    }

    /// <summary>Rebuilds faces in accordance with the kept parts of the tool.</summary>
    public void RebuildFaces()
    {
        int aNbS, i, iRank, nSp, j;
        bool bIsClosed, bIsDegenerated, bToReverse, bRem, bIm, bFlagSD, bVInShapes;
        TopAbs_Orientation anOriF, anOriE;
        TopoDS_Face aFF;
        TopoDS_Edge aSp;
        TopoDS_Shape aSx;
        TopExp_Explorer aExp = new(), aExpE = new();
        NCollection_Map<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance), aMESplit = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItIm = new();
        // OCCT's NCollection_Map<BOPTools_Set>, of which Added() returns the key in the map: an indexed map gives the
        // stored key back from its index, and the map is never iterated (as BOPAlgo_Builder_3)
        NCollection_IndexedMap<BOPTools_Set> aMST = new();
        NCollection_List<TopoDS_Shape> aLE = new();
        //
        foreach (TopoDS_Shape aS in myShapes)
        {
            if (aS.ShapeType() == TopAbs_FACE)
            {
                BOPTools_Set aST = new();
                aST.Add(aS, TopAbs_EDGE);
                aMST.Add(aST);
            }
        }
        //
        aNbS = myDS!.NbSourceShapes();
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            //
            iRank = myDS.Rank(i);
            if (iRank == 1)
            {
                TopoDS_Shape aS = aSI.Shape();
                //
                if (myImages.IsBound(aS))
                {
                    NCollection_List<TopoDS_Shape> aLIm = myImages.ChangeFind(aS);
                    aItIm.Initialize(aLIm);
                    for (; aItIm.More();)
                    {
                        TopoDS_Shape aSIm = aItIm.Value();
                        if (!myShapes.Contains(aSIm))
                        {
                            aLIm.Remove(aItIm);
                            continue;
                        }
                        aItIm.Next();
                    }
                }
                continue;
            }
            //
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            //
            BOPDS_FaceInfo aFI = myDS.FaceInfo(i);
            TopoDS_Face aF = TopoDS.Face(aSI.Shape());
            //
            if (!myImages.IsBound(aF))
            {
                continue;
            }
            //
            anOriF = aF.Orientation();
            aFF = aF;
            aFF.Orientation(TopAbs_FORWARD);

            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBIn = aFI.PaveBlocksIn();
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBSc = aFI.PaveBlocksSc();

            aLE.Clear();

            // bounding edges
            aExp.Init(aFF, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                anOriE = aE.Orientation();
                bIsDegenerated = BRep_Tool.Degenerated(aE);
                bIsClosed = BRep_Tool.IsClosed(aE, aF);
                if (myImages.IsBound(aE))
                {
                    NCollection_List<TopoDS_Shape> aLEIm = myImages.ChangeFind(aE);
                    //
                    bRem = false;
                    bIm = false;
                    aME.Clear();
                    NCollection_List<TopoDS_Shape> aLEImNew = new();
                    //
                    aItIm.Initialize(aLEIm);
                    for (; aItIm.More(); aItIm.Next())
                    {
                        TopoDS_Shape aS = aItIm.Value();

                        bVInShapes = false;
                        if (myShapes.Contains(aS))
                        {
                            bVInShapes = true;
                        }
                        else
                        {
                            aExpE.Init(aS, TopAbs_VERTEX);
                            for (; aExpE.More(); aExpE.Next())
                            {
                                TopoDS_Shape aV = aExpE.Current();
                                if (myShapes.Contains(aV))
                                {
                                    bVInShapes = true;
                                    break;
                                }
                            }
                        }
                        //
                        if (bVInShapes)
                        {
                            bIm = true;
                            aLEImNew.Append(aS);
                        }
                        else
                        {
                            bRem = true;
                            aME.Add(aS);
                        }
                    }
                    //
                    if (!bIm)
                    {
                        aLE.Append(aE);
                        continue;
                    }
                    //
                    if (bRem && bIm)
                    {
                        if (aLEIm.Extent() == 2)
                        {
                            aLE.Append(aE);
                            continue;
                        }
                        if (aMESplit.Add(aE))
                        {
                            RebuildEdge(aE, aFF, aME, aLEImNew);
                            aLEIm.Assign(aLEImNew);
                            if (aLEIm.Extent() == 1)
                            {
                                aLE.Append(aE);
                                continue;
                            }
                        }
                    }
                    //
                    aItIm.Initialize(aLEIm);
                    for (; aItIm.More(); aItIm.Next())
                    {
                        aSp = TopoDS.Edge(aItIm.Value());

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
                            if (!BRep_Tool.IsClosed(aSp, aFF))
                            {
                                BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSp, aFF);
                            }
                            //
                            aSp.Orientation(TopAbs_FORWARD);
                            aLE.Append(aSp);
                            aSp.Orientation(TopAbs_REVERSED);
                            aLE.Append(aSp);
                            continue;
                        } // if (bIsClosed){
                        //
                        aSp.Orientation(anOriE);
                        bToReverse = BOPTools_AlgoTools.IsSplitToReverse(aSp, aE, myContext);
                        if (bToReverse)
                        {
                            aSp.Reverse();
                        }
                        aLE.Append(aSp);
                    }
                }
                else
                {
                    aLE.Append(aE);
                }
            }

            int aNbPBIn, aNbPBSc;
            aNbPBIn = aMPBIn.Extent();
            aNbPBSc = aMPBSc.Extent();
            //
            // in edges
            for (j = 1; j <= aNbPBIn; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBIn.FindKey(j);
                nSp = aPB.Edge();
                aSp = TopoDS.Edge(myDS.Shape(nSp));
                if (myRemoved.Contains(aSp))
                {
                    continue;
                }
                //
                aSp.Orientation(TopAbs_FORWARD);
                aLE.Append(aSp);
                aSp.Orientation(TopAbs_REVERSED);
                aLE.Append(aSp);
            }
            // section edges
            for (j = 1; j <= aNbPBSc; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBSc.FindKey(j);
                nSp = aPB.Edge();
                aSp = TopoDS.Edge(myDS.Shape(nSp));
                if (myRemoved.Contains(aSp))
                {
                    continue;
                }
                //
                aSp.Orientation(TopAbs_FORWARD);
                aLE.Append(aSp);
                aSp.Orientation(TopAbs_REVERSED);
                aLE.Append(aSp);
            }

            // build new faces
            BOPAlgo_BuilderFace aBF = new();
            aBF.SetFace(aFF);
            aBF.SetShapes(aLE);

            aBF.Perform();

            NCollection_List<TopoDS_Shape> aLFIm = myImages.ChangeFind(aF);
            aLFIm.Clear();

            NCollection_List<TopoDS_Shape> aLFR = aBF.Areas();
            aItIm.Initialize(aLFR);
            for (; aItIm.More(); aItIm.Next())
            {
                TopoDS_Shape aFR = aItIm.Value();
                //
                BOPTools_Set aST = new();
                aST.Add(aFR, TopAbs_EDGE);
                bFlagSD = aMST.Contains(aST);
                //
                BOPTools_Set aSTx = aMST.FindKey(aMST.Add(aST));
                aSx = aSTx.Shape();
                aSx.Orientation(anOriF);
                aLFIm.Append(aSx);
                //
                if (!myOrigins.Find(aSx, out NCollection_List<TopoDS_Shape> pLOr))
                {
                    pLOr = myOrigins.Bound(aSx, new NCollection_List<TopoDS_Shape>());
                }
                pLOr.Append(aF);
                //
                if (bFlagSD)
                {
                    myShapesSD.Bind(aFR, aSx);
                }
            }
            //
            if (aLFIm.Extent() == 0)
            {
                myImages.UnBind(aF);
            }
        }
    }

    /// <summary>Rebuilds edges in accordance with the kept parts of the tool.</summary>
    public void RebuildEdge(in TopoDS_Shape theE, in TopoDS_Face theF, NCollection_Map<TopoDS_Shape> aME,
                            NCollection_List<TopoDS_Shape> aLIm)
    {
        int nE, nSp, nV1, nV2, nE1, nV, nVx, nVSD = 0;
        int nV11, nV21;
        bool bOld;
        double aT11, aT21;
        double aT1, aT2;
        TopoDS_Edge aE;
        BOPDS_ShapeInfo aSI = new();
        TopoDS_Vertex aV1, aV2;
        BOPDS_PaveBlock aPBNew;
        NCollection_Map<int> aMI = new(), aMAdd = new(), aMV = new(), aMVOr = new();
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        NCollection_List<int>.Iterator aItLI = new();
        // OCCT iterates this address-keyed map to find the one block with the same bounds: the order is not observable
        NCollection_Map<BOPDS_PaveBlock> aMPB = new(ReferenceEqualityComparer.Instance);
        //
        aSI.SetShapeType(TopAbs_EDGE);

        // 1. collect origin vertices to aMV map.
        nE = myDS!.Index(theE);
        BOPDS_ShapeInfo aSIE = myDS.ShapeInfo(nE);
        NCollection_List<int> aLS = aSIE.SubShapes();
        aItLI.Initialize(aLS);
        for (; aItLI.More(); aItLI.Next())
        {
            nV = aItLI.Value();
            nVx = nV;
            if (myDS.HasShapeSD(nV, ref nVSD))
            {
                nVx = nVSD;
            }
            aMV.Add(nVx);
            aMVOr.Add(nVx);
        }
        //
        // 2. collect vertices that should be removed to aMI map.
        aPBNew = new BOPDS_PaveBlock();
        NCollection_List<BOPDS_Pave> aLPExt = aPBNew.ChangeExtPaves();
        NCollection_List<BOPDS_PaveBlock> aLPB = myDS.ChangePaveBlocks(nE);
        //
        for (aItPB.Initialize(aLPB); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            nE1 = aPB.Edge();
            TopoDS_Shape aE1 = myDS.Shape(nE1);
            //
            if (aME.Contains(aE1))
            {
                aPB.Indices(out nV1, out nV2);
                aMI.Add(nV1);
                aMI.Add(nV2);
            }
            else
            {
                aMPB.Add(aPB);
            }
        }
        // 3. collect vertices that split the source shape.
        for (aItPB.Initialize(aLPB); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            aPB.Indices(out nV1, out nV2);
            //
            if (!aMI.Contains(nV1))
            {
                aMV.Add(nV1);
            }
            if (!aMI.Contains(nV2))
            {
                aMV.Add(nV2);
            }
        }
        // 4. collect ext paves.
        for (aItPB.Initialize(aLPB); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            aPB.Indices(out nV1, out nV2);
            //
            if (aMV.Contains(nV1))
            {
                if (aMAdd.Add(nV1) || aMVOr.Contains(nV1))
                {
                    aLPExt.Append(aPB.Pave1());
                }
            }
            //
            if (aMV.Contains(nV2))
            {
                if (aMAdd.Add(nV2) || aMVOr.Contains(nV2))
                {
                    aLPExt.Append(aPB.Pave2());
                }
            }
        }

        aE = TopoDS.Edge(theE);
        aE.Orientation(TopAbs_FORWARD);
        //
        aLIm.Clear();
        //
        // 5. split edge by new set of vertices.
        aLPB.Clear();
        aPBNew.SetOriginalEdge(nE);
        aPBNew.Update(aLPB, false);
        //
        for (aItPB.Initialize(aLPB); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            BOPDS_Pave aPave1 = aPB.Pave1();
            aPave1.Contents(out nV1, out aT1);
            //
            BOPDS_Pave aPave2 = aPB.Pave2();
            aPave2.Contents(out nV2, out aT2);
            //
            // check if it is the old pave block.
            bOld = false;
            foreach (BOPDS_PaveBlock aPB1 in aMPB)
            {
                aPB1.Indices(out nV11, out nV21);
                aPB1.Range(out aT11, out aT21);
                if (nV1 == nV11 && nV2 == nV21 && aT1 == aT11 && aT2 == aT21)
                {
                    TopoDS_Shape aEIm = myDS.Shape(aPB1.Edge());
                    aLIm.Append(aEIm);
                    bOld = true;
                    break;
                }
            }
            if (bOld)
            {
                continue;
            }
            //
            aV1 = TopoDS.Vertex(myDS.Shape(nV1));
            aV1.Orientation(TopAbs_FORWARD);
            //
            aV2 = TopoDS.Vertex(myDS.Shape(nV2));
            aV2.Orientation(TopAbs_REVERSED);
            //
            BOPTools_AlgoTools.MakeSplitEdge(aE, aV1, aT1, aV2, aT2, out TopoDS_Edge aSp);
            BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aSp, theF, myContext);
            //
            aSI.SetShape(aSp);
            //
            ref Bnd_Box aBox = ref aSI.ChangeBox();
            BRepBndLib.Add(aSp, ref aBox);
            //
            nSp = myDS.Append(aSI);
            //
            aPB.SetEdge(nSp);
            aLIm.Append(aSp);
        }
    }

    /// <summary>Collects the images of the object, that contains in the images of the tool.</summary>
    public void CheckSolidImages()
    {
        NCollection_IndexedMap<BOPTools_Set> aMST = new(); // OCCT's Map with Added(): see RebuildFaces
        NCollection_List<TopoDS_Shape> aLSImNew = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        TopExp_Explorer aExp = new();
        bool bFlagSD;
        //
        TopoDS_Shape aArgs0 = myArguments.First();
        TopoDS_Shape aArgs1 = myTools.First();
        //
        NCollection_List<TopoDS_Shape> aLSIm = myImages.Find(aArgs1);
        aIt.Initialize(aLSIm);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSolIm = aIt.Value();
            //
            BOPTools_Set aST = new();
            aST.Add(aSolIm, TopAbs_FACE);
            aMST.Add(aST);
        }
        //
        aExp.Init(aArgs0, TopAbs_SOLID);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aSolid = aExp.Current();
            if (myImages.IsBound(aSolid))
            {
                NCollection_List<TopoDS_Shape> aLSImSol = myImages.ChangeFind(aSolid);
                aIt.Initialize(aLSImSol);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aSolIm = aIt.Value();
                    //
                    BOPTools_Set aST = new();
                    aST.Add(aSolIm, TopAbs_FACE);
                    bFlagSD = aMST.Contains(aST);
                    //
                    BOPTools_Set aSTx = aMST.FindKey(aMST.Add(aST));
                    TopoDS_Shape aSx = aSTx.Shape();
                    aLSImNew.Append(aSx);
                    //
                    if (bFlagSD)
                    {
                        myShapesSD.Bind(aSolIm, aSx);
                    }
                }
                aLSImSol.Assign(aLSImNew);
            }
        }
    }

    /// <summary>Adds the shape <paramref name="S"/> and its sub-shapes not kept into the map <paramref name="M"/>.</summary>
    public void FillRemoved(in TopoDS_Shape S, NCollection_Map<TopoDS_Shape> M)
    {
        if (myShapes.Contains(S))
        {
            return;
        }
        //
        M.Add(S);
        TopoDS_Iterator It = new(S);
        while (It.More())
        {
            FillRemoved(It.Value(), M);
            It.Next();
        }
    }

    /// <summary>Redefined to avoid the usage of removed faces.</summary>
    protected override void FillIn3DParts(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theDraftSolids,
                                          Message_ProgressRange theRange)
    {
        GetReport().Clear();

        base.FillIn3DParts(theDraftSolids, theRange);

        // Clear the IN parts of the solids from the removed faces
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itM in myInParts)
        {
            NCollection_List<TopoDS_Shape> aList = itM.Value;
            NCollection_List<TopoDS_Shape>.Iterator itL = new(aList);
            for (; itL.More();)
            {
                if (myRemoved.Contains(itL.Value()))
                {
                    aList.Remove(itL);
                }
                else
                {
                    itL.Next();
                }
            }
        }
    }

    /// <summary>Avoid the check for open solids and always use the splits of solids for building the result shape.</summary>
    protected override bool CheckArgsForOpenSolid() => false;
}
