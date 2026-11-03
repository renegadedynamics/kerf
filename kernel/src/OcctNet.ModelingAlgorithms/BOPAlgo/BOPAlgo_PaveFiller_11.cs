// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_11.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    /// <summary>
    /// Checks for the self-interferences acquired by the arguments during the operation: shared vertices, common
    /// blocks and section shapes connecting several sub-shapes of one argument
    /// (<see cref="BOPAlgo_AlertAcquiredSelfIntersection"/> warnings).
    /// </summary>
    protected void CheckSelfInterference()
    {
        if (myArguments.Extent() == 1)
        {
            // Self-interference mode
            return;
        }
        //
        BrepBuilder aBB = new();
        Span<int> nVs = stackalloc int[2];
        //
        int i, aNbR = myDS!.NbRanges();
        for (i = 0; i < aNbR; ++i)
        {
            BOPDS_IndexRange aR = myDS.Range(i);
            //
            // Map of connections of interfering shapes
            NCollection_IndexedDataMap<Shape, NCollection_IndexedMap<Shape>> aMCSI =
                new(ShapeHasher.Instance);
            NCollection_Map<BOPDS_CommonBlock> aMCBFence = new();
            //
            int j = aR.First(), aRLast = aR.Last();
            for (; j <= aRLast; ++j)
            {
                BOPDS_ShapeInfo aSI = myDS.ShapeInfo(j);
                if (!aSI.HasReference())
                {
                    // No pave blocks and no face info
                    continue;
                }
                //
                Shape aS = aSI.Shape();
                //
                if (aSI.ShapeType() == ShapeType.Edge)
                {
                    if (aSI.HasFlag())
                    {
                        continue;
                    }
                    //
                    // Analyze the shared vertices and common blocks
                    //
                    NCollection_IntMap aMSubS = new();
                    NCollection_List<int>.Iterator aItLI = new(aSI.SubShapes());
                    for (; aItLI.More(); aItLI.Next())
                    {
                        int nV = aItLI.Value();
                        myDS.HasShapeSD(nV, ref nV);
                        aMSubS.Add(nV);
                    }
                    //
                    NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(j);
                    bool bAnalyzeV = aLPB.Extent() > 1;
                    //
                    NCollection_List<BOPDS_PaveBlock>.Iterator aIt = new(aLPB);
                    for (; aIt.More(); aIt.Next())
                    {
                        BOPDS_PaveBlock aPB = aIt.Value();
                        //
                        // Check the vertices
                        if (bAnalyzeV)
                        {
                            aPB.Indices(out nVs[0], out nVs[1]);
                            for (int k = 0; k < 2; ++k)
                            {
                                if (!aR.Contains(nVs[k]) && !aMSubS.Contains(nVs[k]))
                                {
                                    // Add connection
                                    Shape aV = myDS.Shape(nVs[k]);
                                    AddConnection(aMCSI, aV, aS);
                                }
                            }
                        }
                        //
                        // Check common blocks
                        if (myDS.IsCommonBlock(aPB))
                        {
                            BOPDS_CommonBlock aCB = myDS.CommonBlock(aPB)!;
                            if (aMCBFence.Add(aCB))
                            {
                                NCollection_List<BOPDS_PaveBlock> aLPBCB = aCB.PaveBlocks();
                                //
                                NCollection_List<int> aLE = new();
                                NCollection_List<BOPDS_PaveBlock>.Iterator aItCB = new(aLPBCB);
                                for (; aItCB.More(); aItCB.Next())
                                {
                                    BOPDS_PaveBlock aPBCB = aItCB.Value();
                                    int nEOr = aPBCB.OriginalEdge();
                                    if (aR.Contains(nEOr))
                                    {
                                        aLE.Append(nEOr);
                                    }
                                }
                                //
                                if (aLE.Extent() > 1)
                                {
                                    // Add the acquired self-interference warning:
                                    // The same common block contains several edges from one argument
                                    Compound aWC = new();
                                    aBB.MakeCompound(ref aWC);
                                    //
                                    NCollection_List<int>.Iterator aItLE = new(aLE);
                                    for (; aItLE.More(); aItLE.Next())
                                    {
                                        Shape aE1 = myDS.Shape(aItLE.Value());
                                        aBB.Add(aWC, aE1);
                                    }
                                    //
                                    AddWarning(new BOPAlgo_AlertAcquiredSelfIntersection(aWC));
                                }
                            }
                        }
                    }
                }
                else if (aSI.ShapeType() == ShapeType.Face)
                {
                    // Analyze IN and Section vertices and edges of the faces
                    BOPDS_FaceInfo aFI = myDS.FaceInfo(j);
                    //
                    for (int k = 0; k < 2; ++k)
                    {
                        NCollection_IntMap aMVF = k == 0 ? aFI.VerticesIn() : aFI.VerticesSc();
                        foreach (int aVIndex in aMVF)
                        {
                            Shape aV = myDS.Shape(aVIndex);
                            // add connection
                            AddConnection(aMCSI, aV, aS);
                        }
                    }
                    //
                    for (int k = 0; k < 2; ++k)
                    {
                        NCollection_IndexedMap<BOPDS_PaveBlock> aMPBF = k == 0 ? aFI.PaveBlocksIn() : aFI.PaveBlocksSc();
                        int iPB, aNbPB = aMPBF.Extent();
                        for (iPB = 1; iPB <= aNbPB; ++iPB)
                        {
                            BOPDS_PaveBlock aPB = aMPBF[iPB];
                            // OCCT's Standard_ASSERT(aPB->HasEdge(), ..., continue): in release builds it only continues.
                            if (!aPB.HasEdge())
                            {
                                continue;
                            }
                            Shape aE = myDS.Shape(aPB.Edge());
                            // add connection
                            AddConnection(aMCSI, aE, aS);
                        }
                    }
                }
            }
            //
            // Analyze connections
            int aNbC = aMCSI.Extent();
            for (j = 1; j <= aNbC; ++j)
            {
                NCollection_IndexedMap<Shape> aMCS = aMCSI[j];
                if (aMCS.Extent() > 1)
                {
                    // Add acquired self-interference warning:
                    // Several faces from one argument contain the same vertex or edge
                    Compound aWC = new();
                    aBB.MakeCompound(ref aWC);
                    //
                    int iS, aNbS = aMCS.Extent();
                    for (iS = 1; iS <= aNbS; ++iS)
                    {
                        Shape aSx = aMCS[iS];
                        aBB.Add(aWC, aSx);
                    }
                    AddWarning(new BOPAlgo_AlertAcquiredSelfIntersection(aWC));
                }
            }
        }
    }

    // OCCT's ChangeSeek-or-Add of the connection map, written once.
    private static void AddConnection(NCollection_IndexedDataMap<Shape, NCollection_IndexedMap<Shape>> theMCSI,
                                      in Shape theKey, in Shape theShape)
    {
        int anIndex = theMCSI.FindIndex(theKey);
        NCollection_IndexedMap<Shape> pMSOr = anIndex > 0
            ? theMCSI[anIndex]
            : theMCSI[theMCSI.Add(theKey, new NCollection_IndexedMap<Shape>(ShapeHasher.Instance))];
        pMSOr.Add(theShape);
    }
}
