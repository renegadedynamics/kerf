// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The shrunk range of a pave block, as a parallel solver of the pave filler.</summary>
internal sealed class BOPAlgo_ShrunkRange : IntTools_ShrunkRange, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private BOPDS_PaveBlock? myPB;

    public void SetPaveBlock(BOPDS_PaveBlock aPB) => myPB = aPB;

    public BOPDS_PaveBlock PaveBlock() => myPB!;

    void BOPTools_Parallel.IContextSolver<IntTools_Context>.SetContext(IntTools_Context theContext) => SetContext(theContext);
}

public partial class BOPAlgo_PaveFiller
{
    protected void FillShrunkData(ShapeType aType1, ShapeType aType2)
    {
        myIterator!.Initialize(aType1, aType2);
        int iSize = myIterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        int i, nE, nV1, nV2, aNbVSD, k;
        double aT1, aT2;
        Span<int> nS = stackalloc int[2];
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new();
        NCollection_IntMap aMI = new();
        NCollection_DynamicArray<BOPAlgo_ShrunkRange> aVSD = new();
        Span<ShapeType> aType = [aType1, aType2];
        //
        for (; myIterator.More(); myIterator.Next())
        {
            myIterator.Value(out nS[0], out nS[1]);
            //
            for (i = 0; i < 2; ++i)
            {
                nE = nS[i];
                if (aType[i] != ShapeType.Edge || !aMI.Add(nE))
                {
                    continue;
                }
                //
                BOPDS_ShapeInfo aSIE = myDS!.ShapeInfo(nE);
                if (aSIE.HasFlag())
                {
                    continue;
                }
                //
                NCollection_List<BOPDS_PaveBlock> aLPB = myDS.ChangePaveBlocks(nE);
                aItLPB.Initialize(aLPB);
                for (; aItLPB.More(); aItLPB.Next())
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    if (aPB.HasShrunkData() && myDS.IsValidShrunkData(aPB))
                    {
                        continue;
                    }
                    //
                    // FillShrunkData(aPB);
                    aPB.Indices(out nV1, out nV2);
                    aPB.Range(out aT1, out aT2);
                    //
                    Edge aE = ShapeCasts.Edge(aSIE.Shape());
                    //
                    Vertex aV1 = ShapeCasts.Vertex(myDS.Shape(nV1));
                    //
                    Vertex aV2 = ShapeCasts.Vertex(myDS.Shape(nV2));
                    //
                    BOPAlgo_ShrunkRange aSD = aVSD.Append(new BOPAlgo_ShrunkRange());
                    //
                    aSD.SetPaveBlock(aPB);
                    aSD.SetData(aE, aT1, aT2, aV1, aV2);
                }
            }
        }
        //
        aNbVSD = aVSD.Length();
        //=============================================================
        BOPTools_Parallel.Perform(myRunParallel, aVSD, myContext!);
        //=============================================================
        //
        for (k = 0; k < aNbVSD; ++k)
        {
            BOPAlgo_ShrunkRange aSD = aVSD[k];
            AnalyzeShrunkData(aSD.PaveBlock(), aSD);
        }
    }
}
