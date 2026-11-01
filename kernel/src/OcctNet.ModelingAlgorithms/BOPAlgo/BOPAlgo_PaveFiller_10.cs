// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_10.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    /// <summary>
    /// Turns the non-destructive mode on if an argument is locked. Frozen shapes are locked, so every operation on
    /// a published shape is non-destructive (decision 577); so is one on an argument holding frozen sub-shapes
    /// (decision 9505).
    /// </summary>
    protected void SetNonDestructive()
    {
        if (!myIsPrimary || myNonDestructive)
        {
            return;
        }
        //
        bool bFlag;
        NCollection_List<TopoDS_Shape>.Iterator aItLS = new();
        //
        bFlag = false;
        aItLS.Initialize(myArguments);
        for (; aItLS.More() && (!bFlag); aItLS.Next())
        {
            TopoDS_Shape aS = aItLS.Value();
            // OCCT.NET: an unfrozen argument may hold frozen sub-shapes (ported code's intermediate shapes sharing a
            // caller's unchanged ones, decision 1013); a destructive operation would throw writing into them, so they
            // count as locked too (decision 9505)
            bFlag = aS.Locked() || BRepTools.HoldsFrozen(aS);
        }
        myNonDestructive = bFlag;
    }

    /// <summary>Increases the tolerance of the edge and of its vertices (not of an argument edge when non-destructive).</summary>
    protected void UpdateEdgeTolerance(int nE, double theTol)
    {
        BOPDS_ShapeInfo aSIE = myDS!.ChangeShapeInfo(nE);
        NCollection_List<int> aLI = aSIE.SubShapes();

        // For the safe input mode avoid modifying the input shapes
        if (myNonDestructive)
        {
            if (!myDS.IsNewShape(nE))
            {
                return;
            }

            NCollection_List<int>.Iterator itLI0 = new(aLI);
            for (; itLI0.More(); itLI0.Next())
            {
                int nV = itLI0.Value(), nVSD = 0;
                if (!myDS.IsNewShape(nV) && !myDS.HasShapeSD(nV, ref nVSD))
                {
                    return;
                }
            }
        }

        // Update edge
        TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
        new BRep_Builder().UpdateEdge(aE, theTol);
        ref Bnd_Box aBoxE = ref aSIE.ChangeBox();
        BRepBndLib.Add(aE, ref aBoxE);
        aBoxE.SetGap(aBoxE.GetGap() + Precision.Confusion());

        // Update vertices
        NCollection_List<int>.Iterator itLI = new(aLI);
        for (; itLI.More(); itLI.Next())
        {
            int nV = itLI.Value();
            UpdateVertex(nV, theTol);
        }
    }

    /// <summary>
    /// Increases the tolerance of the vertex. In non-destructive mode an argument vertex without a same-domain one gets
    /// a new vertex instead, bound as its same-domain vertex. Returns the index of the vertex updated.
    /// </summary>
    protected int UpdateVertex(int nV, double aTolNew)
    {
        int nVNew;
        double aTolV;
        BRep_Builder aBB = new();

        nVNew = nV;
        if (myDS!.IsNewShape(nVNew) || myDS.HasShapeSD(nV, ref nVNew) || !myNonDestructive)
        {
            // nV is a new vertex, it has SD or non-destructive mode is not in force
            TopoDS_Vertex aVSD = TopoDS.Vertex(myDS.Shape(nVNew));
            aTolV = BRep_Tool.Tolerance(aVSD);
            if (aTolV < aTolNew)
            {
                aBB.UpdateVertex(aVSD, aTolNew);
                BOPDS_ShapeInfo aSIV = myDS.ChangeShapeInfo(nVNew);
                ref Bnd_Box aBoxV = ref aSIV.ChangeBox();
                BRepBndLib.Add(aVSD, ref aBoxV);
                aBoxV.SetGap(aBoxV.GetGap() + Precision.Confusion());
                myIncreasedSS.Add(nV);
            }
            return nVNew;
        }
        //
        // nV is old vertex
        TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
        aTolV = BRep_Tool.Tolerance(aV);
        //
        // create new vertex
        TopoDS_Vertex aVNew = new();
        gp_Pnt aPV = BRep_Tool.Pnt(aV);
        aBB.MakeVertex(ref aVNew, aPV, Max(aTolV, aTolNew));
        //
        // append new vertex to DS
        BOPDS_ShapeInfo aSIV0 = new();
        aSIV0.SetShapeType(TopAbs_VERTEX);
        aSIV0.SetShape(aVNew);
        nVNew = myDS.Append(aSIV0);
        //
        // bounding box for the new vertex
        BOPDS_ShapeInfo aSIDS = myDS.ChangeShapeInfo(nVNew);
        ref Bnd_Box aBoxDS = ref aSIDS.ChangeBox();
        BRepBndLib.Add(aVNew, ref aBoxDS);
        aBoxDS.SetGap(aBoxDS.GetGap() + Precision.Confusion());
        //
        // add vertex to SD map
        myDS.AddShapeSD(nV, nVNew);

        // Add new vertex to map of vertices to avoid further extension
        myVertsToAvoidExtension.Add(nVNew);

        if (aTolV < aTolNew)
        {
            myIncreasedSS.Add(nV);
        }

        return nVNew;
    }

    /// <summary>Replaces the vertices of the pave blocks by their same-domain vertices.</summary>
    protected void UpdatePaveBlocksWithSDVertices() => myDS!.UpdatePaveBlocksWithSDVertices();

    /// <summary>Replaces the vertices of the common blocks by their same-domain vertices.</summary>
    protected void UpdateCommonBlocksWithSDVertices()
    {
        if (!myNonDestructive)
        {
            UpdatePaveBlocksWithSDVertices();
            return;
        }
        int aNbPBP;
        //
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        aNbPBP = aPBP.Length();
        if (aNbPBP == 0)
        {
            return;
        }
        //
        int i, nV1, nV2;
        double aTolV;
        NCollection_Map<BOPDS_CommonBlock> aMCB = new();
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        BOPDS_PaveBlock aPB;
        //
        aTolV = Precision.Confusion();
        //
        for (i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                aPB = aItPB.Value();
                BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);
                if (aCB is null)
                {
                    continue;
                }
                //
                if (aMCB.Add(aCB))
                {
                    aPB.Indices(out nV1, out nV2);
                    UpdateVertex(nV1, aTolV);
                    UpdateVertex(nV2, aTolV);
                    myDS.UpdateCommonBlockWithSDVertices(aCB);
                }
            }
        }
        UpdatePaveBlocksWithSDVertices();
    }

    private static void UpdateIntfsWithSDVertices<InterfType>(BOPDS_DS theDS, NCollection_DynamicArray<InterfType> theInterfs)
        where InterfType : BOPDS_Interf
    {
        for (int i = 0; i < theInterfs.Length(); i++)
        {
            InterfType anIntf = theInterfs[i];
            if (anIntf.HasIndexNew(out int anInd))
            {
                int anIndSD = 0;
                if (theDS.HasShapeSD(anInd, ref anIndSD))
                {
                    anIntf.SetIndexNew(anIndSD);
                }
            }
        }
    }

    /// <summary>Replaces the new vertices of the interferences by their same-domain vertices.</summary>
    protected void UpdateInterfsWithSDVertices()
    {
        UpdateIntfsWithSDVertices(myDS!, myDS!.InterfVV());
        UpdateIntfsWithSDVertices(myDS, myDS.InterfVE());
        UpdateIntfsWithSDVertices(myDS, myDS.InterfVF());
        UpdateIntfsWithSDVertices(myDS, myDS.InterfEE());
        UpdateIntfsWithSDVertices(myDS, myDS.InterfEF());
    }
}
