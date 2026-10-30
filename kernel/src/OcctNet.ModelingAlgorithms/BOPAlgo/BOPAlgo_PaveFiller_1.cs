// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    /// <summary>The vertex/vertex interferences: the chains of coinciding vertices become same-domain vertices.</summary>
    protected virtual void PerformVV(Message_ProgressRange theRange)
    {
        int n1, n2, iFlag, aSize;
        //
        myIterator!.Initialize(TopAbs_VERTEX, TopAbs_VERTEX);
        aSize = myIterator.ExpectedLength();
        using var aPS = new Message_ProgressScope(theRange, null, 2.0);
        if (aSize == 0)
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = myDS!.InterfVV();
        //
        //-----------------------------------------------------scope f
        NCollection_IndexedDataMap<int, NCollection_List<int>> aMILI = new();
        NCollection_List<NCollection_List<int>> aMBlocks = new();
        //
        // 1. Map V/LV
        // Split progress range on intersection stage and making blocks. Display only intersection stage.
        using (var aPSLoop = new Message_ProgressScope(aPS.Next(1.0), "Performing Vertex-Vertex intersection", aSize))
        {
            for (; myIterator.More(); myIterator.Next(), aPSLoop.Next())
            {
                if (UserBreak(aPS))
                {
                    return;
                }
                myIterator.Value(out n1, out n2);
                //
                if (myDS.HasInterf(n1, n2))
                {
                    BOPAlgo_Tools.FillMap(n1, n2, aMILI);
                    continue;
                }

                // Check for SD vertices
                int n1SD = n1;
                myDS.HasShapeSD(n1, ref n1SD);

                int n2SD = n2;
                myDS.HasShapeSD(n2, ref n2SD);

                TopoDS_Vertex aV1 = TopoDS.Vertex(myDS.Shape(n1SD));
                TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(n2SD));

                iFlag = BOPTools_AlgoTools.ComputeVV(aV1, aV2, myFuzzyValue);
                if (iFlag == 0)
                {
                    BOPAlgo_Tools.FillMap(n1, n2, aMILI);
                }
            }
        }
        //
        // 2. Make blocks
        BOPAlgo_Tools.MakeBlocks(aMILI, aMBlocks);
        //
        // 3. Make vertices
        NCollection_List<NCollection_List<int>>.Iterator aItB = new(aMBlocks);
        for (; aItB.More(); aItB.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            NCollection_List<int> aLI = aItB.Value();
            MakeSDVertices(aLI);
        }
        //
        // In OCCT's bucket order (decision 6901): it decides the order in which InitPaveBlocksForVertex adds the edges'
        // lists to the pave-block pool, and so the order of MakeSplitEdges' new edges.
        NCollection_IntDataMap<int> aDMII = myDS.ShapesSD();
        foreach (KeyValuePair<int, int> aItDMII in aDMII)
        {
            if (UserBreak(aPS))
            {
                return;
            }
            n1 = aItDMII.Key;
            myDS.InitPaveBlocksForVertex(n1);
        }
        //
        //-----------------------------------------------------scope t
        aMBlocks.Clear();
        aMILI.Clear();
    }

    /// <summary>
    /// Makes one vertex of the given coinciding vertices (or updates their existing same-domain vertex), binds it as
    /// their same-domain vertex and adds the vertex/vertex interferences. Returns its index.
    /// </summary>
    protected int MakeSDVertices(NCollection_List<int> theVertIndices, bool theAddInterfs = true)
    {
        TopoDS_Vertex aVSD = new(), aVn = new();
        int nSD = -1;
        NCollection_List<int>.Iterator aItLI = new(theVertIndices);
        NCollection_List<TopoDS_Shape> aLV = new();
        for (; aItLI.More(); aItLI.Next())
        {
            int nX = aItLI.Value(), nSD1 = 0;
            if (myDS!.HasShapeSD(nX, ref nSD1))
            {
                TopoDS_Shape aVSD1 = myDS.Shape(nSD1);
                if (nSD == -1)
                {
                    aVSD = TopoDS.Vertex(aVSD1);
                    nSD = nSD1;
                }
                else
                {
                    aLV.Append(aVSD1);
                }
            }
            TopoDS_Shape aV = myDS.Shape(nX);
            aLV.Append(aV);
        }
        BOPTools_AlgoTools.MakeVertex(aLV, ref aVn);
        int nV;
        if (nSD != -1)
        {
            // update old SD vertex with new value (a vertex the filler made: never an argument's)
            BRep_TVertex aTVertex = (BRep_TVertex)aVSD.TShape()!;
            aTVertex.Pnt(BRep_Tool.Pnt(aVn));
            aTVertex.Tolerance(BRep_Tool.Tolerance(aVn));
            aVn = aVSD;
            nV = nSD;
        }
        else
        {
            // Append new vertex to the DS
            BOPDS_ShapeInfo aSIn = new();
            aSIn.SetShapeType(TopAbs_VERTEX);
            aSIn.SetShape(aVn);
            nV = myDS!.Append(aSIn);
        }
        BOPDS_ShapeInfo aSIDS = myDS!.ChangeShapeInfo(nV);
        ref Bnd_Box aBox = ref aSIDS.ChangeBox();
        aBox.Add(BRep_Tool.Pnt(aVn));
        aBox.SetGap(BRep_Tool.Tolerance(aVn) + Precision.Confusion());
        //
        // Fill ShapesSD
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = myDS.InterfVV();
        //
        aItLI.Initialize(theVertIndices);
        for (; aItLI.More(); aItLI.Next())
        {
            int n1 = aItLI.Value();
            myDS.AddShapeSD(n1, nV);
            //
            int iR1 = myDS.Rank(n1);
            TopoDS_Shape aV1 = myDS.Shape(n1);
            //
            NCollection_List<int>.Iterator aItLI2 = new();
            aItLI2.Reset(theVertIndices, aItLI.Index);
            aItLI2.Next();
            for (; aItLI2.More(); aItLI2.Next())
            {
                int n2 = aItLI2.Value();
                //
                if (iR1 >= 0 && iR1 == myDS.Rank(n2))
                {
                    // add warning status
                    TopoDS_Shape aV2 = myDS.Shape(n2);
                    //
                    TopoDS_Compound aWC = new();
                    new BRep_Builder().MakeCompound(ref aWC);
                    new BRep_Builder().Add(aWC, aV1);
                    new BRep_Builder().Add(aWC, aV2);
                    //
                    AddWarning(new BOPAlgo_AlertSelfInterferingShape(aWC));
                }
                //
                if (theAddInterfs)
                {
                    if (myDS.AddInterf(n1, n2))
                    {
                        BOPDS_InterfVV aVV = aVVs.Append(new BOPDS_InterfVV());
                        aVV.SetIndices(n1, n2);
                        aVV.SetIndexNew(nV);
                    }
                }
            }
        }
        return nV;
    }
}
