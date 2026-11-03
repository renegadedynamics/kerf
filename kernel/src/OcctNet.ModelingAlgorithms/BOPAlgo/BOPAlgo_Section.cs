// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Section.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The section of the arguments: the edges and vertices where they meet.</summary>
/// <remarks>
/// The result is a compound of the section edges and vertices: the intersections of faces, the edges shared with
/// faces, the vertices found in faces, and the parts of the arguments' boundaries shared by several arguments.
/// </remarks>
public class BOPAlgo_Section : BOPAlgo_Builder
{
    public BOPAlgo_Section()
    {
        Clear();
    }

    protected override void CheckData()
    {
        int aNbArgs;
        //
        aNbArgs = myArguments.Extent();
        if (aNbArgs == 0)
        {
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>The steps of the operation (OCCT's nested enum).</summary>
    protected new enum BOPAlgo_PIOperation
    {
        PIOperation_TreatVertices = 0,
        PIOperation_TreatEdges,
        PIOperation_BuildSection,
        PIOperation_FillHistory,
        PIOperation_PostTreat,
        PIOperation_Last,
    }

    protected override void fillPIConstants(double theWhole, BOPAlgo_PISteps theSteps)
    {
        // Fill in the constants:
        if (myFillHistory)
        {
            // for FillHistroty, which takes about 10% of the whole operation
            theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory, 10.0 * theWhole / 100.0);
        }

        // and for PostTreat, which takes about 5% of the whole operation
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat, 5.0 * theWhole / 100.0);
    }

    protected override void fillPISteps(BOPAlgo_PISteps theSteps)
    {
        // Compute the rest of the operations - all depend on the number of sub-shapes of certain type
        NbShapes aNbShapes = getNbShapes();
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices, aNbShapes.NbVertices());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges, aNbShapes.NbEdges());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSection, aNbShapes.NbEdges() + aNbShapes.NbFaces());
    }

    protected override void PerformInternal1(BOPAlgo_PaveFiller theFiller, ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, "Building result of SECTION operation", 100);
        myPaveFiller = theFiller;
        myDS = myPaveFiller.PDS();
        myContext = myPaveFiller.Context();
        //
        // 1. CheckData
        CheckData();
        if (HasErrors())
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors())
        {
            return;
        }
        //
        BOPAlgo_PISteps aSteps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        analyzeProgress(100.0, aSteps);
        // 3. Fill Images
        // 3.1 Vertices
        FillImagesVertices(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_VERTEX);
        if (HasErrors())
        {
            return;
        }
        // 3.2 Edges
        FillImagesEdges(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_EDGE);
        if (HasErrors())
        {
            return;
        }
        // 4. Section
        BuildSection(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSection)));
        if (HasErrors())
        {
            return;
        }
        // 5.History
        PrepareHistory(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors())
        {
            return;
        }
        // 6. Post-treatment
        PostTreat(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    /// <summary>Builds the section: the section edges and vertices, and the arguments' shared boundaries.</summary>
    protected virtual void BuildSection(ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, "Building the result of Section operation", 1);
        int i, aNbMS, aNbLE;
        int j, nE, nV, aNb, aNbF, aNbPBSc;
        TopoDS_Shape aRC = new(), aRC1 = new();
        BRep_Builder aBB = new();
        TopExp_Explorer aExp = new();
        NCollection_List<TopoDS_Shape> aLSA = new(), aLS = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(), aItIm = new(), aItLS = new();
        NCollection_IndexedDataMap<TopoDS_Shape, int> aMSI = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMS = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IntMap.Iterator aItMI = new();
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        //
        GetReport().Clear();
        //
        BOPTools_AlgoTools.MakeContainer(TopAbs_COMPOUND, ref aRC1);
        //
        // 1. aRC1
        aNb = myDS!.NbSourceShapes();
        for (i = 0; i < aNb; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            if (UserBreak(aPS))
            {
                return;
            }
            //
            BOPDS_FaceInfo aFI = myDS.FaceInfo(i);
            //
            // 1.1 Vertices that are section vertices (in OCCT's integer-map order, decision 784)
            NCollection_IntMap aMVSc = aFI.VerticesSc();
            aItMI.Initialize(aMVSc);
            for (; aItMI.More(); aItMI.Next())
            {
                nV = aItMI.Key();
                TopoDS_Shape aV = myDS.Shape(nV);
                aBB.Add(aRC1, aV);
            }
            //
            // 1.2 Vertices that are in a face
            NCollection_IntMap aMI = aFI.VerticesIn();
            aItMI.Initialize(aMI);
            for (; aItMI.More(); aItMI.Next())
            {
                nV = aItMI.Key();
                if (nV < 0)
                {
                    continue;
                }
                if (myDS.IsNewShape(nV) || myDS.HasInterf(nV))
                {
                    TopoDS_Shape aV = myDS.Shape(nV);
                    aBB.Add(aRC1, aV);
                }
            }
            //
            // 1.3 Section edges
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBSc = aFI.PaveBlocksSc();
            //
            aNbPBSc = aMPBSc.Extent();
            for (j = 1; j <= aNbPBSc; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBSc[j];
                nE = aPB.Edge();
                TopoDS_Shape aE = myDS.Shape(nE);
                aBB.Add(aRC1, aE);
            }
        }
        //
        // 2. Common blocks between an edge and a face
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS.PaveBlocksPool();
        //
        aNb = aPBP.Length();
        for (i = 0; i < aNb; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP.Value(i);
            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                BOPDS_PaveBlock aPB = aItPB.Value();
                BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);
                if (aCB is not null)
                {
                    NCollection_List<int> aLF = aCB.Faces();
                    aNbF = aLF.Extent();
                    if (aNbF != 0)
                    {
                        BOPDS_PaveBlock aPBR = aCB.PaveBlock1();
                        nE = aPBR.Edge();
                        TopoDS_Shape aE = myDS.Shape(nE);
                        aBB.Add(aRC1, aE);
                    }
                }
            }
        }
        //
        aIt.Initialize(myArguments);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSA = aIt.Value();
            if (aMFence.Add(aSA))
            {
                aLSA.Append(aSA);
            }
        }
        //
        aMFence.Clear();
        //
        // 3. Treatment boundaries of arguments
        //
        // 3.1 Set to treat => aLS
        aIt.Initialize(aLSA);
        for (; aIt.More(); aIt.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            TopoDS_Shape aSA = aIt.Value();
            //
            aLS.Clear();
            aMS.Clear();
            aMFence.Clear();
            //
            aExp.Init(aSA, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aE = aExp.Current();
                if (aMFence.Add(aE))
                {
                    aLS.Append(aE);
                }
            }
            aExp.Init(aSA, TopAbs_VERTEX);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aE = aExp.Current();
                if (aMFence.Add(aE))
                {
                    aLS.Append(aE);
                }
            }
            //
            // 3.2 aMSI
            aItLS.Initialize(aLS);
            for (; aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aS = aItLS.Value();
                //
                if (myImages.IsBound(aS))
                {
                    NCollection_List<TopoDS_Shape> aLSIm = myImages.Find(aS);
                    aItIm.Initialize(aLSIm);
                    for (; aItIm.More(); aItIm.Next())
                    {
                        TopoDS_Shape aSIm = aItIm.Value();
                        TopExp.MapShapes(aSIm, TopAbs_VERTEX, aMS);
                        TopExp.MapShapes(aSIm, TopAbs_EDGE, aMS);
                    }
                }
                else
                {
                    TopExp.MapShapes(aS, TopAbs_VERTEX, aMS);
                    TopExp.MapShapes(aS, TopAbs_EDGE, aMS);
                }
            }
            //
            aNbMS = aMS.Extent();
            for (i = 1; i <= aNbMS; ++i)
            {
                TopoDS_Shape aS = aMS[i];
                if (aMSI.Contains(aS))
                {
                    ref int iCnt = ref aMSI.ChangeFromKey(aS);
                    ++iCnt;
                }
                else
                {
                    aMSI.Add(aS, 1);
                }
            }
        }
        //
        aMS.Clear();
        aMFence.Clear();
        //
        // 4. Build the result
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVE = new(TopTools_ShapeMapHasher.Instance);
        //
        TopExp.MapShapesAndAncestors(aRC1, TopAbs_VERTEX, TopAbs_EDGE, aMVE);
        //
        aNbMS = aMSI.Extent();
        for (i = 1; i <= aNbMS; ++i)
        {
            TopoDS_Shape aV = aMSI.FindKey(i);
            int iCnt = aMSI.FindFromIndex(i);
            if (iCnt > 1)
            {
                TopExp.MapShapesAndAncestors(aV, TopAbs_VERTEX, TopAbs_EDGE, aMVE);
            }
        }
        //
        BOPTools_AlgoTools.MakeContainer(TopAbs_COMPOUND, ref aRC);
        //
        aNbMS = aMVE.Extent();
        for (i = 1; i <= aNbMS; ++i)
        {
            if (UserBreak(aPS))
            {
                return;
            }
            TopoDS_Shape aV = aMVE.FindKey(i);
            NCollection_List<TopoDS_Shape> aLE = aMVE.FindFromIndex(i);
            aNbLE = aLE.Extent();
            if (aNbLE == 0)
            {
                // alone vertices
                if (aMFence.Add(aV))
                {
                    aBB.Add(aRC, aV);
                }
            }
            else
            {
                // edges
                aIt.Initialize(aLE);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aE = aIt.Value();
                    if (aMFence.Add(aE))
                    {
                        aBB.Add(aRC, aE);
                    }
                }
            }
        }
        //
        myShape = aRC;
    }
}
