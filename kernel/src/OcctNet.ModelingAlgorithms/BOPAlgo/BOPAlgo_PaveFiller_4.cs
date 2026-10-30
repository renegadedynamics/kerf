// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_GlueEnum;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The vertex/face intersection solver of the pave filler's parallel loop.</summary>
internal sealed class BOPAlgo_VertexFace : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private int myIV;
    private int myIF;
    private int myFlag;
    private double myT1;
    private double myT2;
    private double myTolVNew;
    private TopoDS_Vertex myV = new();
    private TopoDS_Face myF = new();
    private IntTools_Context? myContext;

    public BOPAlgo_VertexFace()
    {
        myIV = -1;
        myIF = -1;
        myFlag = -1;
        myT1 = -1.0;
        myT2 = -1.0;
        myTolVNew = -1.0;
    }

    //
    public void SetIndices(int nV, int nF)
    {
        myIV = nV;
        myIF = nF;
    }

    //
    public void Indices(out int nV, out int nF)
    {
        nV = myIV;
        nF = myIF;
    }

    //
    public void SetVertex(in TopoDS_Vertex aV) => myV = aV;

    //
    public TopoDS_Vertex Vertex() => myV;

    //
    public void SetFace(in TopoDS_Face aF) => myF = aF;

    //
    public TopoDS_Face Face() => myF;

    //
    public int Flag() => myFlag;

    //
    public void Parameters(out double aT1, out double aT2)
    {
        aT1 = myT1;
        aT2 = myT2;
    }

    //
    public double VertexNewTolerance() => myTolVNew;

    //
    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    //
    public IntTools_Context? Context() => myContext;

    //
    public override void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (UserBreak(aPS))
        {
            return;
        }
        try
        {
            myFlag = myContext!.ComputeVF(myV, myF, ref myT1, ref myT2, ref myTolVNew, myFuzzyValue);
        }
        catch (Standard_Failure)
        {
            AddError(new BOPAlgo_AlertIntersectionFailed());
        }
    }
}

public partial class BOPAlgo_PaveFiller
{
    /// <summary>The vertex/face interferences.</summary>
    protected virtual void PerformVF(Message_ProgressRange theRange)
    {
        myIterator!.Initialize(TopAbs_VERTEX, TopAbs_FACE);
        int iSize = myIterator.ExpectedLength();
        //
        int nV, nF;
        //
        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);
        if (myGlue == BOPAlgo_GlueFull)
        {
            // there is no need to intersect vertices with faces in this mode
            // just initialize FaceInfo for all faces
            for (; myIterator.More(); myIterator.Next())
            {
                myIterator.Value(out nV, out nF);
                if (!myDS!.IsSubShape(nV, nF))
                {
                    myDS.ChangeFaceInfo(nF);
                }
            }
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfVF> aVFs = myDS!.InterfVF();
        if (iSize == 0)
        {
            //
            TreatVerticesEE();
            return;
        }
        //
        int nVSD = 0, iFlag, nVx, aNbVF, k;
        NCollection_DynamicArray<BOPAlgo_VertexFace> aVVF = new();
        //
        // Avoid repeated intersection of the same vertex with face in case
        // the group of vertices formed a single SD vertex
        NCollection_DataMap<BOPDS_Pair, NCollection_IntMap> aMVFPairs = new();
        for (; myIterator.More(); myIterator.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            myIterator.Value(out nV, out nF);
            //
            if (myDS.IsSubShape(nV, nF))
            {
                continue;
            }
            //
            if (myDS.HasInterf(nV, nF))
            {
                continue;
            }
            //
            myDS.ChangeFaceInfo(nF);
            if (myDS.HasInterfShapeSubShapes(nV, nF))
            {
                continue;
            }
            //
            nVx = nV;
            if (myDS.HasShapeSD(nV, ref nVSD))
            {
                nVx = nVSD;
            }
            //
            BOPDS_Pair aVFPair = new(nVx, nF);
            if (aMVFPairs.Find(aVFPair, out NCollection_IntMap pMV0))
            {
                pMV0.Add(nV);
                continue;
            }

            NCollection_IntMap pMV = aMVFPairs.Bound(aVFPair, new NCollection_IntMap());
            pMV.Add(nV);

            TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nVx));
            TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));
            //
            BOPAlgo_VertexFace aVertexFace = aVVF.Append(new BOPAlgo_VertexFace());
            //
            aVertexFace.SetIndices(nVx, nF);
            aVertexFace.SetVertex(aV);
            aVertexFace.SetFace(aF);
            aVertexFace.SetFuzzyValue(myFuzzyValue);

        } // for (; myIterator->More(); myIterator->Next()) {
        //
        aNbVF = aVVF.Length();
        using (var aPS = new Message_ProgressScope(aPSOuter.Next(9), "Performing Vertex-Face intersection", aNbVF))
        {
            for (k = 0; k < aNbVF; k++)
            {
                BOPAlgo_VertexFace aVertexFace = aVVF.ChangeValue(k);
                aVertexFace.SetProgressRange(aPS.Next());
            }
            //================================================================
            BOPTools_Parallel.Perform(myRunParallel, aVVF, myContext!);
            //================================================================
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        for (k = 0; k < aNbVF; ++k)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_VertexFace aVertexFace = aVVF[k];
            //
            iFlag = aVertexFace.Flag();
            if (iFlag != 0)
            {
                if (aVertexFace.HasErrors())
                {
                    // Warn about failed intersection of sub-shapes
                    AddIntersectionFailedWarning(aVertexFace.Vertex(), aVertexFace.Face());
                }
                continue;
            }
            //
            aVertexFace.Indices(out nVx, out nF);
            aVertexFace.Parameters(out double aT1, out double aT2);
            double aTolVNew = aVertexFace.VertexNewTolerance();

            BOPDS_Pair aVFPair = new(nVx, nF);
            NCollection_IntMap aMV = aMVFPairs.Find(aVFPair);
            // OCCT iterates the vertices in the int map's bucket order (decision 784): it decides the order of the
            // interferences and which nVx is added to VerticesIn below (the last one).
            foreach (int nVi in aMV)
            {
                nV = nVi;
                // 1
                BOPDS_InterfVF aVF = aVFs.Append(new BOPDS_InterfVF());
                aVF.SetIndices(nV, nF);
                aVF.SetUV(aT1, aT2);
                // 2
                myDS.AddInterf(nV, nF);
                //
                // 3 update vertex V/F if necessary
                nVx = UpdateVertex(nV, aTolVNew);
                //
                // 4
                if (myDS.IsNewShape(nVx))
                {
                    aVF.SetIndexNew(nVx);
                }
            }
            // 5 update FaceInfo
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            NCollection_IntMap aMVIn = aFI.ChangeVerticesIn();
            aMVIn.Add(nVx);
        } // for (k=0; k < aNbVF; ++k) {
        //
        TreatVerticesEE();
    }

    /// <summary>Intersects the new vertices of the edge/edge interferences with the faces.</summary>
    protected void TreatVerticesEE()
    {
        int i, aNbS, aNbEEs, nF, nV, iFlag;
        double aT1 = 0.0, aT2 = 0.0, dummy = 0.0; // OCCT: uninitialised, read only on success
        //
        NCollection_List<int> aLIV = new(), aLIF = new();
        NCollection_IntMap aMI = new(100);
        //
        aNbS = myDS!.NbSourceShapes();
        //
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        aNbEEs = aEEs.Length();
        for (i = 0; i < aNbEEs; ++i)
        {
            BOPDS_InterfEE aEE = aEEs[i];
            if (aEE.HasIndexNew())
            {
                nV = aEE.IndexNew();
                if (aMI.Add(nV))
                {
                    aLIV.Append(nV);
                }
            }
        }
        if (aLIV.Extent() == 0)
        {
            return;
        }
        //
        aNbS = myDS.NbSourceShapes();
        for (nF = 0; nF < aNbS; ++nF)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(nF);
            if (aSI.ShapeType() == TopAbs_FACE)
            {
                aLIF.Append(nF);
            }
        }
        if (aLIF.Extent() == 0)
        {
            return;
        }
        //-------------------------------------------------------------
        NCollection_DynamicArray<BOPDS_InterfVF> aVFs = myDS.InterfVF();
        //
        BOPDS_SubIterator aIt = new();
        //
        aIt.SetDS(myDS);
        aIt.SetSubSet1(aLIF);
        aIt.SetSubSet2(aLIV);
        aIt.Prepare();
        aIt.Initialize();
        for (; aIt.More(); aIt.Next())
        {
            aIt.Value(out nV, out nF);
            //
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            NCollection_IntMap aMVOn = aFI.VerticesOn();
            //
            if (!aMVOn.Contains(nV))
            {
                TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV));
                TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));
                iFlag = myContext!.ComputeVF(aV, aF, ref aT1, ref aT2, ref dummy, myFuzzyValue);
                if (iFlag == 0)
                {
                    // 1
                    BOPDS_InterfVF aVF = aVFs.Append(new BOPDS_InterfVF());
                    i = aVFs.Length() - 1;
                    aVF.SetIndices(nV, nF);
                    aVF.SetUV(aT1, aT2);
                    // 2
                    myDS.AddInterf(nV, nF);
                    //
                    NCollection_IntMap aMVIn = aFI.ChangeVerticesIn();
                    aMVIn.Add(nV);
                }
            }
        }
    }
}
