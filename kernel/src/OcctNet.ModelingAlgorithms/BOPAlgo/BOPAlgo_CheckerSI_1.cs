// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckerSI_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>A vertex classified against a solid (OCCT's file-local BOPAlgo_VertexSolid).</summary>
internal sealed class BOPAlgo_VertexSolid : BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private int myIV;
    private int myIZ;
    private Containment myState;
    private Vertex myV = new();
    private Solid myZ = new();
    private IntTools_Context? myContext;
    private ProgressRange myProgressRange;

    public BOPAlgo_VertexSolid()
    {
        myIV = -1;
        myIZ = -1;
        myState = Unknown;
    }

    //
    public void SetIndices(int nV, int nZ)
    {
        myIV = nV;
        myIZ = nZ;
    }

    //
    public void Indices(out int nV, out int nZ)
    {
        nV = myIV;
        nZ = myIZ;
    }

    //
    public void SetVertex(in Vertex aV) => myV = aV;

    //
    public Vertex Vertex() => myV;

    //
    public void SetSolid(in Solid aZ) => myZ = aZ;

    //
    public Solid Solid() => myZ;

    //
    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    //
    public IntTools_Context? Context() => myContext;

    //
    public Containment State() => myState;

    //
    public void SetProgressRange(ProgressRange theRange) => myProgressRange = theRange;

    //
    public void Perform()
    {
        using var aPS = new ProgressScope(myProgressRange, null, 1);
        if (!aPS.More())
        {
            return;
        }

        double aTol;
        Point3D aPV;
        //
        BRepClass3d_SolidClassifier aSC = myContext!.SolidClassifier(myZ);
        //
        aPV = Tool.Pnt(myV);
        aTol = Tool.Tolerance(myV);
        //
        aSC.Perform(aPV, aTol);
        //
        myState = aSC.State();
    }
}

/// <summary>An edge or face tested against a solid (OCCT's file-local BOPAlgo_ShapeSolid).</summary>
internal class BOPAlgo_ShapeSolid : BOPTools_Parallel.ISolver
{
    protected int myIE;
    protected int myIZ;
    protected bool myHasInterf;
    protected BOPDS_DS? myDS;
    protected ProgressRange myProgressRange;

    public BOPAlgo_ShapeSolid()
    {
        myIE = -1;
        myIZ = -1;
        myHasInterf = false;
        myDS = null;
    }

    //
    public void SetIndices(int nE, int nZ)
    {
        myIE = nE;
        myIZ = nZ;
    }

    //
    public void Indices(out int nE, out int nZ)
    {
        nE = myIE;
        nZ = myIZ;
    }

    //
    public void SetDS(BOPDS_DS pDS) => myDS = pDS;

    //
    public bool HasInterf() => myHasInterf;

    //
    public void SetProgressRange(ProgressRange theRange) => myProgressRange = theRange;

    //
    public virtual void Perform()
    {
        using var aPS = new ProgressScope(myProgressRange, null, 1);
        if (!aPS.More())
        {
            return;
        }

        bool bHasInterf;
        //
        myHasInterf = false;
        //
        bHasInterf = myDS!.HasInterfShapeSubShapes(myIE, myIZ);
        if (!bHasInterf)
        {
            myHasInterf = myDS.HasInterfShapeSubShapes(myIZ, myIE);
        }
    }
}

/// <summary>Two solids tested against each other (OCCT's file-local BOPAlgo_SolidSolid).</summary>
internal sealed class BOPAlgo_SolidSolid : BOPAlgo_ShapeSolid
{
    //
    public override void Perform()
    {
        using var aPS = new ProgressScope(myProgressRange, null, 1);
        if (!aPS.More())
        {
            return;
        }

        bool bFlag;
        //
        bFlag = false;
        myHasInterf = false;
        //
        myHasInterf = myDS!.HasInterfShapeSubShapes(myIZ, myIE, bFlag);
        if (!myHasInterf)
        {
            myHasInterf = myDS.HasInterfShapeSubShapes(myIE, myIZ, bFlag);
        }
    }
}

public partial class BOPAlgo_CheckerSI
{
    /// <summary>Vertex/solid intersection: a vertex classified IN a solid that doesn't share it.</summary>
    protected virtual void PerformVZ(ProgressRange theRange)
    {
        using var aPSOuter = new ProgressScope(theRange, null, 1);

        int iSize, nV, nZ, k, aNbVVS;
        Containment aState;
        NCollection_Map<BOPDS_Pair> aMPK = new();
        //
        myIterator!.Initialize(ShapeType.Vertex, ShapeType.Solid);
        iSize = myIterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfVZ> aVZs = myDS!.InterfVZ();
        //
        List<BOPAlgo_VertexSolid> aVVS = [];
        //
        for (; myIterator.More(); myIterator.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            myIterator.Value(out nV, out nZ);
            //
            if (myDS.HasInterfShapeSubShapes(nV, nZ))
            {
                continue;
            }
            //
            int nVSD = nV;
            myDS.HasShapeSD(nV, ref nVSD);
            //
            BOPDS_Pair aPK = new();
            aPK.SetIndices(nVSD, nZ);
            if (!aMPK.Add(aPK))
            {
                continue;
            }
            //
            Vertex aV = ShapeCasts.Vertex(myDS.Shape(nVSD));
            Solid aZ = ShapeCasts.Solid(myDS.Shape(nZ));
            //
            BOPAlgo_VertexSolid aVertexSolid = new();
            aVVS.Add(aVertexSolid);
            aVertexSolid.SetIndices(nV, nZ);
            aVertexSolid.SetVertex(aV);
            aVertexSolid.SetSolid(aZ);
        }
        //
        aNbVVS = aVVS.Count;

        using var aPSParallel = new ProgressScope(aPSOuter.Next(), "Performing Vertex-Solid intersection", aNbVVS);
        for (int iVS = 0; iVS < aNbVVS; ++iVS)
        {
            aVVS[iVS].SetProgressRange(aPSParallel.Next());
        }
        //=============================================================
        BOPTools_Parallel.Perform(myRunParallel, aVVS, myContext!);
        //=============================================================
        if (UserBreak(aPSOuter))
        {
            return;
        }
        for (k = 0; k < aNbVVS; ++k)
        {
            BOPAlgo_VertexSolid aVertexSolid = aVVS[k];
            aState = aVertexSolid.State();
            if (aState == In)
            {
                aVertexSolid.Indices(out nV, out nZ);
                //
                BOPDS_InterfVZ aVZ = aVZs.Append(new BOPDS_InterfVZ());
                aVZ.SetIndices(nV, nZ);
                //
                myDS.AddInterf(nV, nZ);
            }
        }
    }

    /// <summary>Edge/solid intersection.</summary>
    protected virtual void PerformEZ(ProgressRange theRange)
    {
        PerformSZ(ShapeType.Edge, theRange);
    }

    /// <summary>Face/solid intersection.</summary>
    protected virtual void PerformFZ(ProgressRange theRange)
    {
        PerformSZ(ShapeType.Face, theRange);
    }

    /// <summary>Solid/solid intersection.</summary>
    protected virtual void PerformZZ(ProgressRange theRange)
    {
        using var aPSOuter = new ProgressScope(theRange, null, 1);

        bool bHasInterf;
        int iSize, nZ1, nZ, k, aNbSolidSolid;
        //
        myIterator!.Initialize(ShapeType.Solid, ShapeType.Solid);
        iSize = myIterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        List<BOPAlgo_SolidSolid> aVSolidSolid = [];
        //
        for (; myIterator.More(); myIterator.Next())
        {
            myIterator.Value(out nZ1, out nZ);
            //
            BOPAlgo_SolidSolid aSolidSolid = new();
            aVSolidSolid.Add(aSolidSolid);
            aSolidSolid.SetIndices(nZ1, nZ);
            aSolidSolid.SetDS(myDS!);
        }
        //
        aNbSolidSolid = aVSolidSolid.Count;

        using var aPSParallel = new ProgressScope(aPSOuter.Next(), "Performing Solid-Solid intersection", aNbSolidSolid);
        for (int iSS = 0; iSS < aNbSolidSolid; ++iSS)
        {
            aVSolidSolid[iSS].SetProgressRange(aPSParallel.Next());
        }
        //======================================================
        BOPTools_Parallel.Perform(myRunParallel, aVSolidSolid);
        //======================================================
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfZZ> aZZs = myDS!.InterfZZ();
        //
        for (k = 0; k < aNbSolidSolid; ++k)
        {
            BOPAlgo_SolidSolid aSolidSolid = aVSolidSolid[k];
            bHasInterf = aSolidSolid.HasInterf();
            if (bHasInterf)
            {
                aSolidSolid.Indices(out nZ1, out nZ);
                //
                BOPDS_InterfZZ aZZ = aZZs.Append(new BOPDS_InterfZZ());
                aZZ.SetIndices(nZ1, nZ);
                //
                myDS.AddInterf(nZ1, nZ);
            }
        }
    }

    /// <summary>Edge/solid or face/solid intersection: the shape and the solid share interfering sub-shapes.</summary>
    protected virtual void PerformSZ(ShapeType theTS, ProgressRange theRange)
    {
        using var aPSOuter = new ProgressScope(theRange, null, 1);

        bool bHasInterf;
        int iSize, nS, nZ, k, aNbShapeSolid;
        //
        myIterator!.Initialize(theTS, ShapeType.Solid);
        iSize = myIterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        List<BOPAlgo_ShapeSolid> aVShapeSolid = [];
        //
        for (; myIterator.More(); myIterator.Next())
        {
            myIterator.Value(out nS, out nZ);
            //
            BOPAlgo_ShapeSolid aShapeSolid = new();
            aVShapeSolid.Add(aShapeSolid);
            aShapeSolid.SetIndices(nS, nZ);
            aShapeSolid.SetDS(myDS!);
        }
        //
        aNbShapeSolid = aVShapeSolid.Count;

        using var aPSParallel = new ProgressScope(aPSOuter.Next(),
                                                          theTS == ShapeType.Edge ? "Performing Edge-Solid intersection"
                                                                               : "Performing Face-Solid intersection",
                                                          aNbShapeSolid);
        for (int iSS = 0; iSS < aNbShapeSolid; ++iSS)
        {
            aVShapeSolid[iSS].SetProgressRange(aPSParallel.Next());
        }
        //======================================================
        BOPTools_Parallel.Perform(myRunParallel, aVShapeSolid);
        //======================================================
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfEZ> aEZs = myDS!.InterfEZ();
        NCollection_DynamicArray<BOPDS_InterfFZ> aFZs = myDS.InterfFZ();
        //
        for (k = 0; k < aNbShapeSolid; ++k)
        {
            BOPAlgo_ShapeSolid aShapeSolid = aVShapeSolid[k];
            bHasInterf = aShapeSolid.HasInterf();
            if (bHasInterf)
            {
                aShapeSolid.Indices(out nS, out nZ);
                //
                if (theTS == ShapeType.Edge)
                {
                    BOPDS_InterfEZ aEZ = aEZs.Append(new BOPDS_InterfEZ());
                    aEZ.SetIndices(nS, nZ);
                }
                else
                { // if (aTS==TopAbs_FACE)
                    BOPDS_InterfFZ aFZ = aFZs.Append(new BOPDS_InterfFZ());
                    aFZ.SetIndices(nS, nZ);
                }
                //
                myDS.AddInterf(nS, nZ);
            }
        }
    }
}
