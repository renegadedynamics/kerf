// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Occt.BOPAlgo_Operation;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    protected NCollection_Map<Shape> myShapes = new(ShapeHasher.Instance);
    protected NCollection_Map<Shape> myRemoved = new(ShapeHasher.Instance);
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
    public void Init(in Shape theShape)
    {
        Clear();
        //
        AddArgument(theShape);
    }

    /// <summary>Initializes the arguments of local boolean operation.</summary>
    public void Init(in Shape theShape, in Shape theTool)
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
    public void PartsOfTool(NCollection_List<Shape> aLT)
    {
        Explorer aExp = new();
        //
        aLT.Clear();
        aExp.Init(myShape, ShapeType.Solid);
        for (; aExp.More(); aExp.Next())
        {
            Shape aS = aExp.Current();
            aLT.Append(aS);
        }
    }

    /// <summary>Initializes parts of the tool for the second step: collects the shapes and their sub-shapes.</summary>
    public void KeepParts(NCollection_List<Shape> theIm)
    {
        NCollection_List<Shape>.Iterator aItT = new();
        aItT.Initialize(theIm);
        for (; aItT.More(); aItT.Next())
        {
            Shape aTIm = aItT.Value();
            KeepPart(aTIm);
        }
    }

    /// <summary>Adds the shape and all its sub-shapes into the kept shapes.</summary>
    public void KeepPart(in Shape thePart)
    {
        Exploration.MapShapes(thePart, myShapes);
    }

    /// <summary>Prepares builder of local operation.</summary>
    protected override void Prepare()
    {
        GetReport().Clear();
        //
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        myShape = aC;
        //
        FillRemoved();
    }

    /// <summary>Collects the removed parts of the tool.</summary>
    public void FillRemoved()
    {
        Explorer aExp = new();
        //
        Shape aArgs0 = myArguments.First();
        Shape aArgs1 = myTools.First();
        //
        aExp.Init(aArgs0, ShapeType.Solid);
        for (; aExp.More(); aExp.Next())
        {
            Shape aS = aExp.Current();
            myImages.UnBind(aS);
        }
        //
        if (!myImages.IsBound(aArgs1))
        {
            return;
        }
        //
        NCollection_List<Shape>.Iterator aItIm = new();
        //
        NCollection_List<Shape> aLS = myImages.ChangeFind(aArgs1);
        aItIm.Initialize(aLS);
        for (; aItIm.More(); aItIm.Next())
        {
            Shape aS = aItIm.Value();
            FillRemoved(aS, myRemoved);
        }
    }

    /// <summary>Main function to build the result of the local operation required.</summary>
    public void PerformResult(ProgressRange theRange = default)
    {
        myOperation = myFuse != 0 ? BOPAlgo_FUSE : BOPAlgo_CUT;
        if (myShapes.IsEmpty())
        {
            BuildShape(theRange);
            return;
        }

        double aWhole = 100.0;
        using var aPS = new ProgressScope(theRange, "BRepFeat_Builder", aWhole);
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
        FillImagesContainers(ShapeType.Shell, aPS.Next(aSteps[1]));
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
        BuildResult(ShapeType.Solid);
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
        BuildResult(ShapeType.Compound);
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
        Orientation anOriF, anOriE;
        Face aFF;
        Edge aSp;
        Shape aSx;
        Explorer aExp = new(), aExpE = new();
        NCollection_Map<Shape> aME = new(ShapeHasher.Instance), aMESplit = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator aItIm = new();
        // OCCT's NCollection_Map<BOPTools_Set>, of which Added() returns the key in the map: an indexed map gives the
        // stored key back from its index, and the map is never iterated (as BOPAlgo_Builder_3)
        NCollection_IndexedMap<BOPTools_Set> aMST = new();
        NCollection_List<Shape> aLE = new();
        //
        foreach (Shape aS in myShapes)
        {
            if (aS.ShapeType() == ShapeType.Face)
            {
                BOPTools_Set aST = new();
                aST.Add(aS, ShapeType.Edge);
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
                Shape aS = aSI.Shape();
                //
                if (myImages.IsBound(aS))
                {
                    NCollection_List<Shape> aLIm = myImages.ChangeFind(aS);
                    aItIm.Initialize(aLIm);
                    for (; aItIm.More();)
                    {
                        Shape aSIm = aItIm.Value();
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
            if (aSI.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            //
            BOPDS_FaceInfo aFI = myDS.FaceInfo(i);
            Face aF = ShapeCasts.Face(aSI.Shape());
            //
            if (!myImages.IsBound(aF))
            {
                continue;
            }
            //
            anOriF = aF.Orientation();
            aFF = aF;
            aFF.Orientation(Forward);

            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBIn = aFI.PaveBlocksIn();
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBSc = aFI.PaveBlocksSc();

            aLE.Clear();

            // bounding edges
            aExp.Init(aFF, ShapeType.Edge);
            for (; aExp.More(); aExp.Next())
            {
                Edge aE = ShapeCasts.Edge(aExp.Current());
                anOriE = aE.Orientation();
                bIsDegenerated = Tool.Degenerated(aE);
                bIsClosed = Tool.IsClosed(aE, aF);
                if (myImages.IsBound(aE))
                {
                    NCollection_List<Shape> aLEIm = myImages.ChangeFind(aE);
                    //
                    bRem = false;
                    bIm = false;
                    aME.Clear();
                    NCollection_List<Shape> aLEImNew = new();
                    //
                    aItIm.Initialize(aLEIm);
                    for (; aItIm.More(); aItIm.Next())
                    {
                        Shape aS = aItIm.Value();

                        bVInShapes = false;
                        if (myShapes.Contains(aS))
                        {
                            bVInShapes = true;
                        }
                        else
                        {
                            aExpE.Init(aS, ShapeType.Vertex);
                            for (; aExpE.More(); aExpE.Next())
                            {
                                Shape aV = aExpE.Current();
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
                        aSp = ShapeCasts.Edge(aItIm.Value());

                        if (bIsDegenerated)
                        {
                            aSp.Orientation(anOriE);
                            aLE.Append(aSp);
                            continue;
                        }
                        //
                        if (anOriE == Internal)
                        {
                            aSp.Orientation(Forward);
                            aLE.Append(aSp);
                            aSp.Orientation(Reversed);
                            aLE.Append(aSp);
                            continue;
                        }
                        //
                        if (bIsClosed)
                        {
                            if (!Tool.IsClosed(aSp, aFF))
                            {
                                BOPTools_AlgoTools3D.DoSplitSEAMOnFace(aSp, aFF);
                            }
                            //
                            aSp.Orientation(Forward);
                            aLE.Append(aSp);
                            aSp.Orientation(Reversed);
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
                aSp = ShapeCasts.Edge(myDS.Shape(nSp));
                if (myRemoved.Contains(aSp))
                {
                    continue;
                }
                //
                aSp.Orientation(Forward);
                aLE.Append(aSp);
                aSp.Orientation(Reversed);
                aLE.Append(aSp);
            }
            // section edges
            for (j = 1; j <= aNbPBSc; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBSc.FindKey(j);
                nSp = aPB.Edge();
                aSp = ShapeCasts.Edge(myDS.Shape(nSp));
                if (myRemoved.Contains(aSp))
                {
                    continue;
                }
                //
                aSp.Orientation(Forward);
                aLE.Append(aSp);
                aSp.Orientation(Reversed);
                aLE.Append(aSp);
            }

            // build new faces
            BOPAlgo_BuilderFace aBF = new();
            aBF.SetFace(aFF);
            aBF.SetShapes(aLE);

            aBF.Perform();

            NCollection_List<Shape> aLFIm = myImages.ChangeFind(aF);
            aLFIm.Clear();

            NCollection_List<Shape> aLFR = aBF.Areas();
            aItIm.Initialize(aLFR);
            for (; aItIm.More(); aItIm.Next())
            {
                Shape aFR = aItIm.Value();
                //
                BOPTools_Set aST = new();
                aST.Add(aFR, ShapeType.Edge);
                bFlagSD = aMST.Contains(aST);
                //
                BOPTools_Set aSTx = aMST.FindKey(aMST.Add(aST));
                aSx = aSTx.Shape();
                aSx.Orientation(anOriF);
                aLFIm.Append(aSx);
                //
                if (!myOrigins.Find(aSx, out NCollection_List<Shape> pLOr))
                {
                    pLOr = myOrigins.Bound(aSx, new NCollection_List<Shape>());
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
    public void RebuildEdge(in Shape theE, in Face theF, NCollection_Map<Shape> aME,
                            NCollection_List<Shape> aLIm)
    {
        int nE, nSp, nV1, nV2, nE1, nV, nVx, nVSD = 0;
        int nV11, nV21;
        bool bOld;
        double aT11, aT21;
        double aT1, aT2;
        Edge aE;
        BOPDS_ShapeInfo aSI = new();
        Vertex aV1, aV2;
        BOPDS_PaveBlock aPBNew;
        NCollection_Map<int> aMI = new(), aMAdd = new(), aMV = new(), aMVOr = new();
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        NCollection_List<int>.Iterator aItLI = new();
        // OCCT iterates this address-keyed map to find the one block with the same bounds: the order is not observable
        NCollection_Map<BOPDS_PaveBlock> aMPB = new(ReferenceEqualityComparer.Instance);
        //
        aSI.SetShapeType(ShapeType.Edge);

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
            Shape aE1 = myDS.Shape(nE1);
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

        aE = ShapeCasts.Edge(theE);
        aE.Orientation(Forward);
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
                    Shape aEIm = myDS.Shape(aPB1.Edge());
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
            aV1 = ShapeCasts.Vertex(myDS.Shape(nV1));
            aV1.Orientation(Forward);
            //
            aV2 = ShapeCasts.Vertex(myDS.Shape(nV2));
            aV2.Orientation(Reversed);
            //
            BOPTools_AlgoTools.MakeSplitEdge(aE, aV1, aT1, aV2, aT2, out Edge aSp);
            BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aSp, theF, myContext);
            //
            aSI.SetShape(aSp);
            //
            ref Box aBox = ref aSI.ChangeBox();
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
        NCollection_List<Shape> aLSImNew = new();
        NCollection_List<Shape>.Iterator aIt = new();
        Explorer aExp = new();
        bool bFlagSD;
        //
        Shape aArgs0 = myArguments.First();
        Shape aArgs1 = myTools.First();
        //
        NCollection_List<Shape> aLSIm = myImages.Find(aArgs1);
        aIt.Initialize(aLSIm);
        for (; aIt.More(); aIt.Next())
        {
            Shape aSolIm = aIt.Value();
            //
            BOPTools_Set aST = new();
            aST.Add(aSolIm, ShapeType.Face);
            aMST.Add(aST);
        }
        //
        aExp.Init(aArgs0, ShapeType.Solid);
        for (; aExp.More(); aExp.Next())
        {
            Shape aSolid = aExp.Current();
            if (myImages.IsBound(aSolid))
            {
                NCollection_List<Shape> aLSImSol = myImages.ChangeFind(aSolid);
                aIt.Initialize(aLSImSol);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aSolIm = aIt.Value();
                    //
                    BOPTools_Set aST = new();
                    aST.Add(aSolIm, ShapeType.Face);
                    bFlagSD = aMST.Contains(aST);
                    //
                    BOPTools_Set aSTx = aMST.FindKey(aMST.Add(aST));
                    Shape aSx = aSTx.Shape();
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
    public void FillRemoved(in Shape S, NCollection_Map<Shape> M)
    {
        if (myShapes.Contains(S))
        {
            return;
        }
        //
        M.Add(S);
        Iterator It = new(S);
        while (It.More())
        {
            FillRemoved(It.Value(), M);
            It.Next();
        }
    }

    /// <summary>Redefined to avoid the usage of removed faces.</summary>
    protected override void FillIn3DParts(NCollection_DataMap<Shape, Shape> theDraftSolids,
                                          ProgressRange theRange)
    {
        GetReport().Clear();

        base.FillIn3DParts(theDraftSolids, theRange);

        // Clear the IN parts of the solids from the removed faces
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itM in myInParts)
        {
            NCollection_List<Shape> aList = itM.Value;
            NCollection_List<Shape>.Iterator itL = new(aList);
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
