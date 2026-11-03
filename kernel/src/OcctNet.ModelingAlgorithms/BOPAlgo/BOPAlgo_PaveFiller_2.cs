// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The vertex/edge intersection solver of the pave filler's parallel loop.</summary>
internal sealed class BOPAlgo_VertexEdge : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private int myIV;
    private int myIE;
    private int myFlag;
    private double myT;
    private double myTolVNew;
    private Vertex myV = new();
    private Edge myE = new();
    private IntTools_Context? myContext;
    private BOPDS_PaveBlock? myPB;

    public BOPAlgo_VertexEdge()
    {
        myIV = -1;
        myIE = -1;
        myFlag = -1;
        myT = -1.0;
        myTolVNew = -1.0;
    }

    //
    public void SetIndices(int nV, int nE)
    {
        myIV = nV;
        myIE = nE;
    }

    //
    public void Indices(out int nV, out int nE)
    {
        nV = myIV;
        nE = myIE;
    }

    //
    public void SetVertex(in Vertex aV) => myV = aV;

    //
    public void SetEdge(in Edge aE) => myE = aE;

    //
    public Vertex Vertex() => myV;

    //
    public Edge Edge() => myE;

    //
    public int Flag() => myFlag;

    //
    public double Parameter() => myT;

    //
    public double VertexNewTolerance() => myTolVNew;

    //
    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    //
    public IntTools_Context? Context() => myContext;

    //
    public void SetPaveBlock(BOPDS_PaveBlock thePB) => myPB = thePB;

    //
    public BOPDS_PaveBlock? PaveBlock() => myPB;

    //
    public override void Perform()
    {
        using var aPS = new ProgressScope(myProgressRange, null, 1);
        if (UserBreak(aPS))
        {
            return;
        }
        try
        {
            myFlag = myContext!.ComputeVE(myV, myE, ref myT, ref myTolVNew, myFuzzyValue);
        }
        catch (FailureException)
        {
            AddError(new BOPAlgo_AlertIntersectionFailed());
        }
    }
}

public partial class BOPAlgo_PaveFiller
{
    /// <summary>The vertex/edge interferences.</summary>
    protected virtual void PerformVE(ProgressRange theRange)
    {
        FillShrunkData(ShapeType.Vertex, ShapeType.Edge);
        //
        myIterator!.Initialize(ShapeType.Vertex, ShapeType.Edge);
        using var aPS = new ProgressScope(theRange, null, 1);

        int iSize = myIterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        // Prepare pairs for intersection
        NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> aMVEPairs = new();
        for (; myIterator.More(); myIterator.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            myIterator.Value(out int nV, out int nE);
            //
            BOPDS_ShapeInfo aSIE = myDS!.ShapeInfo(nE);
            if (aSIE.HasSubShape(nV))
            {
                continue;
            }
            //
            if (aSIE.HasFlag())
            {
                continue;
            }
            //
            if (myDS.HasInterf(nV, nE))
            {
                continue;
            }
            //
            if (myDS.HasInterfShapeSubShapes(nV, nE))
            {
                continue;
            }
            //
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(nE);
            if (aLPB.IsEmpty())
            {
                continue;
            }
            //
            BOPDS_PaveBlock aPB = aLPB.First();
            if (!aPB.IsSplittable())
            {
                // this is a micro edge, ignore it
                continue;
            }
            //
            int anIndex = aMVEPairs.FindIndex(aPB);
            NCollection_List<int> pLV = anIndex > 0 ? aMVEPairs[anIndex] : aMVEPairs[aMVEPairs.Add(aPB, new NCollection_List<int>())];
            pLV.Append(nV);
        }
        //
        IntersectVE(aMVEPairs, aPS.Next());
    }

    /// <summary>
    /// Intersects the vertices with the pave blocks of <paramref name="theVEPairs"/>, adds the extra paves and splits
    /// the pave blocks; with <paramref name="theAddInterfs"/> the vertex/edge interferences are added to the DS.
    /// </summary>
    protected void IntersectVE(NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> theVEPairs,
                               ProgressRange theRange, bool theAddInterfs = true)
    {
        int i, aNbVE = theVEPairs.Extent();
        if (aNbVE == 0)
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfVE> aVEs = myDS!.InterfVE();
        //
        // Prepare for intersection.
        NCollection_DynamicArray<BOPAlgo_VertexEdge> aVVE = new();
        // Map to collect all SD connections to add interferences
        // for all vertices having the same SD vertex.
        // It will also be used as a Fence map to avoid repeated
        // intersection of the same SD vertex with edge
        NCollection_DataMap<BOPDS_Pair, NCollection_List<int>> aDMVSD = new();
        //
        using var aPSOuter = new ProgressScope(theRange, null, 10);
        for (i = 1; i <= aNbVE; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPDS_PaveBlock aPB = theVEPairs.FindKey(i);
            int nE = aPB.OriginalEdge();
            //
            NCollection_IntMap aMVPB = new();
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(nE);
            for (NCollection_List<BOPDS_PaveBlock>.Iterator itPB = new(aLPB); itPB.More(); itPB.Next())
            {
                aMVPB.Add(itPB.Value().Pave1().Index());
                aMVPB.Add(itPB.Value().Pave2().Index());
            }

            NCollection_List<int> aLV = theVEPairs[i];
            NCollection_List<int>.Iterator aItLV = new(aLV);
            for (; aItLV.More(); aItLV.Next())
            {
                int nV = aItLV.Value();
                //
                int nVSD = nV;
                myDS.HasShapeSD(nV, ref nVSD);
                //
                if (aMVPB.Contains(nVSD))
                {
                    continue;
                }

                BOPDS_Pair aPair = new(nVSD, nE);
                if (aDMVSD.Find(aPair, out NCollection_List<int> pLI0))
                {
                    // Already added
                    pLI0.Append(nV);
                    continue;
                }
                // New pair
                NCollection_List<int> pLI = aDMVSD.Bound(aPair, new NCollection_List<int>());
                pLI.Append(nV);
                //
                Vertex aV = ShapeCasts.Vertex(myDS.Shape(nVSD));
                Edge aE = ShapeCasts.Edge(myDS.Shape(nE));
                //
                BOPAlgo_VertexEdge aVESolver = aVVE.Append(new BOPAlgo_VertexEdge());
                aVESolver.SetIndices(nVSD, nE);
                aVESolver.SetVertex(aV);
                aVESolver.SetEdge(aE);
                aVESolver.SetPaveBlock(aPB);
                aVESolver.SetFuzzyValue(myFuzzyValue);
            }
        }
        //
        aNbVE = aVVE.Length();

        using (var aPS = new ProgressScope(aPSOuter.Next(9), "Performing Vertex-Edge intersection", aNbVE))
        {
            for (i = 0; i < aNbVE; i++)
            {
                BOPAlgo_VertexEdge aVESolver = aVVE.ChangeValue(i);
                aVESolver.SetProgressRange(aPS.Next());
            }
            // Perform intersection
            //=============================================================
            BOPTools_Parallel.Perform(myRunParallel, aVVE, myContext!);
            //=============================================================
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        // Keep the modified edges for further update
        NCollection_IntMap aMEdges = new();
        //
        // Analyze intersections
        for (i = 0; i < aNbVE; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_VertexEdge aVESolver = aVVE[i];
            if (aVESolver.Flag() != 0)
            {
                if (aVESolver.HasErrors())
                {
                    // Warn about failed intersection of sub-shapes
                    AddIntersectionFailedWarning(aVESolver.Vertex(), aVESolver.Edge());
                }
                continue;
            }
            //
            aVESolver.Indices(out int nV, out int nE);
            // Parameter of vertex on edge
            double aT = aVESolver.Parameter();
            // 1. Update vertex V/E if necessary
            double aTolVNew = aVESolver.VertexNewTolerance();
            int nVx = UpdateVertex(nV, aTolVNew);
            // 2. Create new pave and add it as extra pave to pave block
            //    for further splitting of the edge
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(nE);
            // Find the appropriate one
            BOPDS_PaveBlock? aPB = null;
            NCollection_List<BOPDS_PaveBlock>.Iterator itPB = new(aLPB);
            for (; itPB.More(); itPB.Next())
            {
                aPB = itPB.Value();
                aPB.Range(out double aT1, out double aT2);
                if (aT > aT1 && aT < aT2)
                {
                    break;
                }
            }
            if (!itPB.More())
            {
                continue;
            }

            BOPDS_Pave aPave = new();
            aPave.SetIndex(nVx);
            aPave.SetParameter(aT);
            aPB!.AppendExtPave(aPave);
            aMEdges.Add(nE);
            //
            if (theAddInterfs)
            {
                // Add interferences into DS
                BOPDS_Pair aPair = new(nV, nE);
                NCollection_List<int> aLI = aDMVSD.Find(aPair);
                NCollection_List<int>.Iterator aItLI = new(aLI);
                for (; aItLI.More(); aItLI.Next())
                {
                    int nVOld = aItLI.Value();
                    // 3. Create interference V/E
                    BOPDS_InterfVE aVE = aVEs.Append(new BOPDS_InterfVE());
                    aVE.SetIndices(nVOld, nE);
                    aVE.SetParameter(aT);
                    // 2. Add a pair in the whole table of interferences
                    myDS.AddInterf(nVOld, nE);
                    // 4. Set index of new vertex in the interference
                    if (myDS.IsNewShape(nVx))
                    {
                        aVE.SetIndexNew(nVx);
                    }
                }
            }
        }
        //
        // Split pave blocks of the intersected edges with the extra paves.
        // At the same time compute shrunk data for the new pave blocks
        // and in case there is no valid range for the pave block,
        // the vertices of this pave block should be unified.
        SplitPaveBlocks(aMEdges, theAddInterfs);
    }

    // Make new Common Block from the given list of Pave Blocks
    private static void MakeNewCommonBlock(NCollection_List<BOPDS_PaveBlock> theLPB, NCollection_List<int> theLFaces, BOPDS_DS theDS)
    {
        // Make Common Block from the pave blocks in the list
        BOPDS_CommonBlock aCBNew = new();
        aCBNew.SetPaveBlocks(theLPB);
        aCBNew.SetFaces(theLFaces);
        //
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(theLPB);
        for (; aItLPB.More(); aItLPB.Next())
        {
            theDS.SetCommonBlock(aItLPB.Value(), aCBNew);
        }
    }

    /// <summary>
    /// Splits the pave blocks of the edges by their extra paves; the vertices of a new pave block without a valid range
    /// are unified.
    /// </summary>
    protected void SplitPaveBlocks(NCollection_IntMap theMEdges, bool theAddInterfs)
    {
        // Fence map to avoid unification of the same vertices twice
        NCollection_Map<BOPDS_Pair> aMPairs = new();
        // Map to treat the Common Blocks
        NCollection_IndexedDataMap<BOPDS_CommonBlock, NCollection_List<BOPDS_PaveBlock>> aMCBNewPB = new();
        //
        // Map of vertices to init the pave blocks for them
        NCollection_IntMap aMVerticesToInitPB = new();

        // OCCT iterates the edges in the int map's bucket order (decision 784): it decides the order of the new
        // same-domain vertices.
        foreach (int nE in theMEdges)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS!.ChangePaveBlocks(nE);
            //
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More();)
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                //
                if (!aPB.IsToUpdate())
                {
                    aItLPB.Next();
                    continue;
                }
                //
                BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);
                //
                // Compute new pave blocks
                NCollection_List<BOPDS_PaveBlock> aLPBN = new();
                aPB.Update(aLPBN);
                //
                // Make sure that each new pave block has a valid range,
                // otherwise unify the vertices of the pave block
                NCollection_List<BOPDS_PaveBlock>.Iterator aItLPBN = new(aLPBN);
                for (; aItLPBN.More(); aItLPBN.Next())
                {
                    BOPDS_PaveBlock aPBN = aItLPBN.Value();
                    myDS.UpdatePaveBlockWithSDVertices(aPBN);
                    FillShrunkData(aPBN);
                    //
                    bool bHasValidRange = aPBN.HasShrunkData();
                    // Take into account that the edge could have really small valid range,
                    // so that the Pave Block cannot be further split. In this case, check if
                    // the vertices of the Pave Block do not interfere. And if they are, unify them.
                    bool bCheckDist = (bHasValidRange && !aPBN.IsSplittable());
                    if (!bHasValidRange || bCheckDist)
                    {
                        aPBN.Indices(out int nV1, out int nV2);
                        if (nV1 == nV2)
                        {
                            // Same vertices -> no valid range, no need to unify vertices
                            continue;
                        }

                        // Decide whether to unify vertices or not
                        if (bCheckDist)
                        {
                            Vertex aV1 = ShapeCasts.Vertex(myDS.Shape(nV1));
                            Vertex aV2 = ShapeCasts.Vertex(myDS.Shape(nV2));
                            if (BOPTools_AlgoTools.ComputeVV(aV1, aV2, myFuzzyValue) == 0)
                            {
                                // vertices are interfering -> no valid range, unify vertices
                                bHasValidRange = false;
                            }
                        }

                        if (!bHasValidRange)
                        {
                            BOPDS_Pair aPair = new();
                            aPair.SetIndices(nV1, nV2);
                            if (aMPairs.Add(aPair))
                            {
                                NCollection_List<int> aLV = new();
                                aLV.Append(nV1);
                                aLV.Append(nV2);
                                MakeSDVertices(aLV, theAddInterfs);

                                // Save vertices to init pave blocks
                                aMVerticesToInitPB.Add(nV1);
                                aMVerticesToInitPB.Add(nV2);
                            }
                            continue;
                        }
                    }
                    //
                    // Update the list with new pave block
                    aLPB.Append(aPBN);
                    // Treat the common block
                    if (aCB is not null)
                    {
                        // Store the new pave block to make new common block
                        int anIndex = aMCBNewPB.FindIndex(aCB);
                        NCollection_List<BOPDS_PaveBlock> pLPBCB = anIndex > 0
                            ? aMCBNewPB[anIndex]
                            : aMCBNewPB[aMCBNewPB.Add(aCB, new NCollection_List<BOPDS_PaveBlock>())];
                        pLPBCB.Append(aPBN);
                    }
                }
                // Remove old pave block
                aLPB.Remove(aItLPB);
            }
        }
        //
        // Make Common Blocks
        int i, aNbCB = aMCBNewPB.Extent();
        for (i = 1; i <= aNbCB; ++i)
        {
            BOPDS_CommonBlock aCB = aMCBNewPB.FindKey(i);
            NCollection_List<BOPDS_PaveBlock> aLPBN = aMCBNewPB[i];
            //
            // For each group of pave blocks with the same vertices make new common block
            NCollection_IndexedDataMap<BOPDS_Pair, NCollection_List<BOPDS_PaveBlock>> aMInds = new();
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPBN);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                //
                BOPDS_Pair aPair = new();
                aPair.SetIndices(aPB.Pave1().Index(), aPB.Pave2().Index());
                //
                int anIndex = aMInds.FindIndex(aPair);
                NCollection_List<BOPDS_PaveBlock> pLPBx = anIndex > 0
                    ? aMInds[anIndex]
                    : aMInds[aMInds.Add(aPair, new NCollection_List<BOPDS_PaveBlock>())];
                pLPBx.Append(aPB);
            }
            //
            aCB.PaveBlock1().Indices(out int nV1, out int nV2);
            bool bIsClosed = (nV1 == nV2);
            //
            int j, aNbPairs = aMInds.Extent();
            for (j = 1; j <= aNbPairs; ++j)
            {
                NCollection_List<BOPDS_PaveBlock> aLPB = aMInds[j];
                //
                if (!bIsClosed)
                {
                    // Make Common Block from the pave blocks in the list
                    MakeNewCommonBlock(aLPB, aCB.Faces(), myDS!);
                    continue;
                }
                //
                // Find coinciding pave blocks
                while (aLPB.Extent() != 0)
                {
                    // Pave blocks forming the common block
                    NCollection_List<BOPDS_PaveBlock> aLPBCB = new();
                    // Point in the middle of the first pave block in the common block
                    Point3D aPMFirst = new(0.0, 0.0, 0.0);
                    // Tolerance of the first edge in the common block
                    double aTolEFirst = 0.0;
                    //
                    aItLPB.Initialize(aLPB);
                    for (; aItLPB.More();)
                    {
                        BOPDS_PaveBlock aPB = aItLPB.Value();
                        if (aLPBCB.IsEmpty())
                        {
                            aLPBCB.Append(aPB);
                            Edge aEFirst = ShapeCasts.Edge(myDS!.Shape(aPB.OriginalEdge()));
                            aTolEFirst = Tool.MaxTolerance(aEFirst, ShapeType.Vertex);
                            //
                            double aTmFirst = (aPB.Pave1().Parameter() + aPB.Pave2().Parameter()) / 2.0;
                            BOPTools_AlgoTools.PointOnEdge(aEFirst, aTmFirst, out aPMFirst);
                            //
                            aLPB.Remove(aItLPB);
                            continue;
                        }
                        //
                        // Check pave blocks for coincidence
                        Edge aE = ShapeCasts.Edge(myDS!.Shape(aPB.OriginalEdge()));
                        double aTolE = Tool.MaxTolerance(aE, ShapeType.Vertex);
                        //
                        double aTOut = 0.0, aDist = 0.0; // OCCT: uninitialised, read only on success
                        int iErr = myContext!.ComputePE(aPMFirst, aTolEFirst + aTolE + myFuzzyValue, aE, ref aTOut, ref aDist);
                        if (iErr == 0 && ((aTOut > aPB.Pave1().Parameter()) && (aTOut < aPB.Pave2().Parameter())))
                        {
                            aLPBCB.Append(aPB);
                            aLPB.Remove(aItLPB);
                            continue;
                        }
                        aItLPB.Next();
                    }
                    //
                    // Make Common Block from the pave blocks in the list
                    MakeNewCommonBlock(aLPBCB, aCB.Faces(), myDS!);
                }
            }
        }

        // Init pave blocks for vertices which have acquired SD vertex
        foreach (int nV in aMVerticesToInitPB)
        {
            myDS!.InitPaveBlocksForVertex(nV);
        }
    }

    /// <summary>Adds the warning <see cref="BOPAlgo_AlertIntersectionOfPairOfShapesFailed"/> for the two shapes.</summary>
    protected void AddIntersectionFailedWarning(in Shape theS1, in Shape theS2)
    {
        // Create the warn shape
        Compound aWC = new();
        new BrepBuilder().MakeCompound(ref aWC);
        new BrepBuilder().Add(aWC, theS1);
        new BrepBuilder().Add(aWC, theS2);
        // Add the warning
        AddWarning(new BOPAlgo_AlertIntersectionOfPairOfShapesFailed(aWC));
    }
}
