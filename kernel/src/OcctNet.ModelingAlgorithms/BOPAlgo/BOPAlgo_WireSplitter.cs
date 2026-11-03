// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Builds the loops (wires) from the edges of a BOPAlgo_WireEdgeSet on its face: the connexity blocks that are
/// regular make one wire each, the others are split by following the smallest clockwise angle at each vertex.
/// </summary>
public partial class BOPAlgo_WireSplitter : BOPAlgo_Algo
{
    protected BOPAlgo_WireEdgeSet? myWES;
    protected NCollection_List<BOPTools_ConnexityBlock> myLCB = new();
    protected IntTools_Context? myContext;

    public BOPAlgo_WireSplitter()
    {
        myWES = null;
    }

    /// <summary>Sets the edge set (held by reference, as OCCT's pointer: the wires are added to it).</summary>
    public void SetWES(BOPAlgo_WireEdgeSet theWES) => myWES = theWES;

    public BOPAlgo_WireEdgeSet WES() => myWES!;

    public void SetContext(IntTools_Context? theContext) => myContext = theContext;

    public IntTools_Context? Context() => myContext;

    protected override void CheckData()
    {
        if (myWES is null)
        {
            AddError(new BOPAlgo_AlertNullInputShapes());
            return;
        }
    }

    public override void Perform(ProgressRange theRange = default)
    {
        GetReport().Clear();
        using var aPS = new ProgressScope(theRange, "Building wires", 1);
        //
        CheckData();
        if (HasErrors())
        {
            return;
        }
        //
        // create a context
        if (myContext is null)
        {
            myContext = new IntTools_Context();
        }
        //
        BOPTools_AlgoTools.MakeConnexityBlocks(myWES!.StartElements(), ShapeType.Vertex, ShapeType.Edge, myLCB);
        if (UserBreak(aPS))
        {
            return;
        }

        MakeWires(aPS.Next());
    }

    /// <summary>Makes a wire of the edges (the .lxx's inline function); the wire is closed if its edges are.</summary>
    public static void MakeWire(NCollection_List<Shape> aLE, out Wire aWire)
    {
        BrepBuilder aBB = new();
        aWire = new Wire();
        aBB.MakeWire(ref aWire);
        NCollection_List<Shape>.Iterator aIt = new(aLE);
        for (; aIt.More(); aIt.Next())
        {
            aBB.Add(aWire, aIt.Value());
        }
        aWire.Closed(Tool.IsClosed(aWire));
    }

    protected void MakeWires(ProgressRange theRange)
    {
        bool bIsRegular;
        int aNbVCB, k;
        Wire aW;
        NCollection_List<BOPTools_ConnexityBlock>.Iterator aItCB = new(myLCB);
        NCollection_List<Shape>.Iterator aIt;
        NCollection_DynamicArray<BOPAlgo_WS_ConnexityBlock> aVCB = new();
        //
        using var aPSOuter = new ProgressScope(theRange, null, 1);
        //
        Face aF = myWES!.Face();
        //
        aItCB.Initialize(myLCB);
        for (; aItCB.More(); aItCB.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }

            BOPTools_ConnexityBlock aCB = aItCB.Value();
            bIsRegular = aCB.IsRegular();
            if (bIsRegular)
            {
                NCollection_List<Shape> aLE = aCB.ChangeShapes();
                MakeWire(aLE, out aW);
                myWES.AddShape(aW);
            }
            else
            {
                BOPAlgo_WS_ConnexityBlock aWSCB = new();
                aVCB.Append(aWSCB);
                aWSCB.SetFace(aF);
                aWSCB.SetConnexityBlock(aCB);
            }
        }
        aNbVCB = aVCB.Length();
        using var aPSParallel = new ProgressScope(aPSOuter.Next(), null, aNbVCB);
        for (int iW = 0; iW < aNbVCB; ++iW)
        {
            aVCB.Value(iW).SetProgressRange(aPSParallel.Next());
        }
        //===================================================
        BOPTools_Parallel.Perform(myRunParallel, aVCB, myContext!);
        //===================================================
        for (k = 0; k < aNbVCB; ++k)
        {
            BOPAlgo_WS_ConnexityBlock aCB = aVCB.Value(k);
            NCollection_List<Shape> aLW = aCB.ConnexityBlock().Loops();
            aIt = new NCollection_List<Shape>.Iterator(aLW);
            for (; aIt.More(); aIt.Next())
            {
                Shape aWx = aIt.Value();
                myWES.AddShape(aWx);
            }
        }
    }
}

/// <summary>An edge at a vertex in the wire splitter: incoming or outgoing, its angle, whether it has been passed.</summary>
/// <remarks>A working object held in the splitter's lists and changed in place (OCCT's code keeps pointers to them).</remarks>
internal sealed class BOPAlgo_EdgeInfo
{
    private Edge myEdge;
    private bool myPassed;
    private bool myInFlag;
    private bool myIsInside;
    private double myAngle;

    public BOPAlgo_EdgeInfo()
    {
        myEdge = new Edge();
        myPassed = false;
        myInFlag = false;
        myIsInside = false;
        myAngle = -1.0;
    }

    public void SetEdge(in Edge theE) => myEdge = theE;

    public Edge Edge() => myEdge;

    public void SetPassed(bool theFlag) => myPassed = theFlag;

    public bool Passed() => myPassed;

    public void SetInFlag(bool theFlag) => myInFlag = theFlag;

    public bool IsIn() => myInFlag;

    public void SetAngle(double theAngle) => myAngle = theAngle;

    public double Angle() => myAngle;

    public bool IsInside() => myIsInside;

    public void SetIsInside(bool theIsInside) => myIsInside = theIsInside;
}

/// <summary>The solver splitting one irregular connexity block (OCCT's file-local <c>BOPAlgo_WS_ConnexityBlock</c>).</summary>
internal sealed class BOPAlgo_WS_ConnexityBlock : BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private Face myFace = new();
    private BOPTools_ConnexityBlock myCB = new();
    private IntTools_Context? myContext;
    private ProgressRange myRange;

    public void SetFace(in Face theF) => myFace = theF;

    public Face Face() => myFace;

    // OCCT copies the block by value
    public void SetConnexityBlock(BOPTools_ConnexityBlock theCB) => myCB = new BOPTools_ConnexityBlock(theCB);

    public BOPTools_ConnexityBlock ConnexityBlock() => myCB;

    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    public IntTools_Context? Context() => myContext;

    public void SetProgressRange(ProgressRange theRange) => myRange = theRange;

    public void Perform()
    {
        using var aPS = new ProgressScope(myRange, null, 1);
        if (!aPS.More())
        {
            return;
        }
        BOPAlgo_WireSplitter.SplitBlock(myFace, myCB, myContext!);
    }
}
