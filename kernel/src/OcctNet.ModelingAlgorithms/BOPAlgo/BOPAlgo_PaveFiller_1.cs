// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    /// <summary>The vertex/vertex interferences: the chains of coinciding vertices become same-domain vertices.</summary>
    protected virtual void PerformVV(ProgressRange theRange)
    {
        int n1, n2, iFlag, aSize;
        //
        myIterator!.Initialize(ShapeType.Vertex, ShapeType.Vertex);
        aSize = myIterator.ExpectedLength();
        using var aPS = new ProgressScope(theRange, null, 2.0);
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
        using (var aPSLoop = new ProgressScope(aPS.Next(1.0), "Performing Vertex-Vertex intersection", aSize))
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

                Vertex aV1 = ShapeCasts.Vertex(myDS.Shape(n1SD));
                Vertex aV2 = ShapeCasts.Vertex(myDS.Shape(n2SD));

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
        // In OCCT's bucket order (decision 784): it decides the order in which InitPaveBlocksForVertex adds the edges'
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
        Vertex aVSD = new(), aVn = new();
        int nSD = -1;
        NCollection_List<int>.Iterator aItLI = new(theVertIndices);
        NCollection_List<Shape> aLV = new();
        for (; aItLI.More(); aItLI.Next())
        {
            int nX = aItLI.Value(), nSD1 = 0;
            if (myDS!.HasShapeSD(nX, ref nSD1))
            {
                Shape aVSD1 = myDS.Shape(nSD1);
                if (nSD == -1)
                {
                    aVSD = ShapeCasts.Vertex(aVSD1);
                    nSD = nSD1;
                }
                else
                {
                    aLV.Append(aVSD1);
                }
            }
            Shape aV = myDS.Shape(nX);
            aLV.Append(aV);
        }
        BOPTools_AlgoTools.MakeVertex(aLV, ref aVn);
        int nV;
        if (nSD != -1)
        {
            // update old SD vertex with new value (a vertex the filler made: never an argument's)
            BrepTVertex aTVertex = (BrepTVertex)aVSD.TShape()!;
            aTVertex.Pnt(Tool.Pnt(aVn));
            aTVertex.Tolerance(Tool.Tolerance(aVn));
            aVn = aVSD;
            nV = nSD;
        }
        else
        {
            // Append new vertex to the DS
            BOPDS_ShapeInfo aSIn = new();
            aSIn.SetShapeType(ShapeType.Vertex);
            aSIn.SetShape(aVn);
            nV = myDS!.Append(aSIn);
        }
        BOPDS_ShapeInfo aSIDS = myDS!.ChangeShapeInfo(nV);
        ref Box aBox = ref aSIDS.ChangeBox();
        aBox.Add(Tool.Pnt(aVn));
        aBox.SetGap(Tool.Tolerance(aVn) + global::Kerf.Engine.Foundation.Precision.Confusion());
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
            Shape aV1 = myDS.Shape(n1);
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
                    Shape aV2 = myDS.Shape(n2);
                    //
                    Compound aWC = new();
                    new BrepBuilder().MakeCompound(ref aWC);
                    new BrepBuilder().Add(aWC, aV1);
                    new BrepBuilder().Add(aWC, aV2);
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
