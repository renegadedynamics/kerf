// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PArgumentAnalyzer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.BOPAlgo_CheckStatus;
using static Occt.BOPAlgo_Operation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Checks one or two shapes for a Boolean operation: their types for the operation, self-interference, small edges,
/// faces that can't be rebuilt, vertices and edges of the two shapes too close to merge, C0 geometry and curves off
/// their surfaces. Each test is switched on by its mode (all off by default); the faults found are
/// <see cref="GetCheckResult"/>.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: the modes and the operation are OCCT's modifiable references (<c>ref</c> returns).
/// The shapes are only read, so frozen shapes are checked as they are (the self-interference test runs a
/// non-destructive <see cref="BOPAlgo_CheckerSI"/>). <see cref="GetCheckResult"/> publishes the results, whose shapes
/// are frozen as they are read (<see cref="BOPAlgo_CheckResult"/>); ported code reads
/// <see cref="InternalGetCheckResult"/>. The tangency test is not implemented in OCCT. OCCT's pointer typedef
/// <c>BOPAlgo_PArgumentAnalyzer</c> is the class reference.
/// </remarks>
public class BOPAlgo_ArgumentAnalyzer : BOPAlgo_Algo
{
    private TopoDS_Shape myShape1 = new();
    private TopoDS_Shape myShape2 = new();
    private bool myStopOnFirst;
    private BOPAlgo_Operation myOperation;
    private bool myArgumentTypeMode;
    private bool mySelfInterMode;
    private bool mySmallEdgeMode;
    private bool myRebuildFaceMode;
    private bool myTangentMode;
    private bool myMergeVertexMode;
    private bool myMergeEdgeMode;
    private bool myContinuityMode;
    private bool myCurveOnSurfaceMode;
    private bool myEmpty1;
    private bool myEmpty2;
    private readonly NCollection_List<BOPAlgo_CheckResult> myResult = new();

    /// <summary>An analyzer with every test off and the operation unknown.</summary>
    public BOPAlgo_ArgumentAnalyzer()
    {
        myStopOnFirst = false;
        myOperation = BOPAlgo_UNKNOWN;
        myArgumentTypeMode = false;
        mySelfInterMode = false;
        mySmallEdgeMode = false;
        myRebuildFaceMode = false;
        myTangentMode = false;
        myMergeVertexMode = false;
        myMergeEdgeMode = false;
        myContinuityMode = false;
        myCurveOnSurfaceMode = false;
        myEmpty1 = false;
        myEmpty2 = false;
    }

    /// <summary>Sets the object shape.</summary>
    public void SetShape1(in TopoDS_Shape TheShape) => myShape1 = TheShape;

    /// <summary>Sets the tool shape.</summary>
    public void SetShape2(in TopoDS_Shape TheShape) => myShape2 = TheShape;

    /// <summary>The object shape.</summary>
    public TopoDS_Shape GetShape1() => myShape1;

    /// <summary>The tool shape.</summary>
    public TopoDS_Shape GetShape2() => myShape2;

    /// <summary>The operation the shapes are checked for (modifiable).</summary>
    public ref BOPAlgo_Operation OperationType() => ref myOperation;

    /// <summary>Whether the analysis stops at the first fault (modifiable).</summary>
    public ref bool StopOnFirstFaulty() => ref myStopOnFirst;

    /// <summary>The mode checking the types of the shapes (modifiable).</summary>
    public ref bool ArgumentTypeMode() => ref myArgumentTypeMode;

    /// <summary>The mode checking self-interference of the shapes (modifiable).</summary>
    public ref bool SelfInterMode() => ref mySelfInterMode;

    /// <summary>The mode checking small edges (modifiable).</summary>
    public ref bool SmallEdgeMode() => ref mySmallEdgeMode;

    /// <summary>The mode checking whether faces can be split or rebuilt (modifiable).</summary>
    public ref bool RebuildFaceMode() => ref myRebuildFaceMode;

    /// <summary>The mode checking tangency between sub-shapes (modifiable; not implemented in OCCT).</summary>
    public ref bool TangentMode() => ref myTangentMode;

    /// <summary>The mode checking problems of merging vertices (modifiable).</summary>
    public ref bool MergeVertexMode() => ref myMergeVertexMode;

    /// <summary>The mode checking problems of merging edges (modifiable).</summary>
    public ref bool MergeEdgeMode() => ref myMergeEdgeMode;

    /// <summary>The mode checking the continuity of the shapes (modifiable).</summary>
    public ref bool ContinuityMode() => ref myContinuityMode;

    /// <summary>The mode checking invalid curves on surfaces (modifiable).</summary>
    public ref bool CurveOnSurfaceMode() => ref myCurveOnSurfaceMode;

    /// <summary>Prepares the data.</summary>
    protected void Prepare()
    {
        bool isS1 = myShape1.IsNull(), isS2 = myShape2.IsNull();
        if (!isS1)
        {
            myEmpty1 = BOPTools_AlgoTools3D.IsEmptyShape(myShape1);
        }
        if (!isS2)
        {
            myEmpty2 = BOPTools_AlgoTools3D.IsEmptyShape(myShape2);
        }
    }

    /// <summary>Performs the analysis; an exception ends it with a result of unknown status.</summary>
    public override void Perform(Message_ProgressRange theRange = default)
    {
        using var aPS = new Message_ProgressScope(theRange, "Analyze shapes", 10);
        try
        {
            myResult.Clear();
            if (UserBreak(aPS))
            {
                return;
            }
            // 1. Prepare
            Prepare();
            //
            // 2. Test types
            if (myArgumentTypeMode)
            {
                TestTypes();
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 3. Test self-interference
            if (mySelfInterMode)
            {
                TestSelfInterferences(aPS.Next(8));
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 4. Test small edges
            if (mySmallEdgeMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestSmallEdge();
                }
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 5. Test possibility to rebuild faces
            if (myRebuildFaceMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestRebuildFace();
                }
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 6. Test tangent
            if (myTangentMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestTangent();
                    if (UserBreak(aPS))
                    {
                        return;
                    }
                }
            }
            //
            // 7. Test merge vertices
            if (myMergeVertexMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestMergeVertex();
                }
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 8. Test merge edges
            if (myMergeEdgeMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestMergeEdge();
                }
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 9. Test shapes continuity
            if (myContinuityMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestContinuity();
                }
                if (UserBreak(aPS))
                {
                    return;
                }
            }
            //
            // 10. Test validity of the curves on the surfaces
            if (myCurveOnSurfaceMode)
            {
                if (myResult.IsEmpty() || !myStopOnFirst)
                {
                    TestCurveOnSurface();
                }
            }
        }
        catch (Standard_Failure)
        {
            BOPAlgo_CheckResult aResult = new();
            aResult.SetCheckStatus(BOPAlgo_CheckUnknown);
            myResult.Append(aResult);
        }
    }

    /// <summary>True if a fault was found.</summary>
    public bool HasFaulty() => !myResult.IsEmpty();

    /// <summary>The faults found (their shapes frozen as they are read).</summary>
    public IBentoList<BOPAlgo_CheckResult> GetCheckResult() => PackedList.Own([.. myResult]);

    /// <summary>OCCT's <c>GetCheckResult</c>: the analyzer's own list, for ported code.</summary>
    internal NCollection_List<BOPAlgo_CheckResult> InternalGetCheckResult() => myResult;

    /// <summary>Checks the types of the shapes, and for two shapes their dimensions for the operation.</summary>
    protected void TestTypes()
    {
        bool isS1 = myShape1.IsNull(), isS2 = myShape2.IsNull();

        if (isS1 && isS2)
        {
            BOPAlgo_CheckResult aResult = new();
            aResult.SetCheckStatus(BOPAlgo_BadType);
            myResult.Append(aResult);
            return;
        }

        // single shape check
        if ((isS1 && !isS2) || (!isS1 && isS2))
        {
            bool bIsEmpty = isS1 ? myEmpty2 : myEmpty1;

            if (bIsEmpty || myOperation != BOPAlgo_UNKNOWN)
            {
                TopoDS_Shape aS = isS1 ? myShape2 : myShape1;
                BOPAlgo_CheckResult aResult = new();
                aResult.SetShape1(aS);
                aResult.SetCheckStatus(BOPAlgo_BadType);
                myResult.Append(aResult);
                return;
            }
        }
        // two shapes check (begin)
        else
        {
            if (myEmpty1 || myEmpty2)
            {
                BOPAlgo_CheckResult aResult = new();
                if (myEmpty1 && myEmpty2)
                {
                    aResult.SetShape1(myShape1);
                    aResult.SetShape2(myShape2);
                }
                else
                {
                    TopoDS_Shape aS = myEmpty1 ? myShape1 : myShape2;
                    if (myEmpty1)
                    {
                        aResult.SetShape1(aS);
                    }
                    else
                    {
                        aResult.SetShape2(aS);
                    }
                }
                aResult.SetCheckStatus(BOPAlgo_BadType);
                myResult.Append(aResult);
                return;
            }
            //
            if (myOperation != BOPAlgo_UNKNOWN && myOperation != BOPAlgo_COMMON)
            {
                Span<int> iDimMin = stackalloc int[2], iDimMax = stackalloc int[2];
                BOPTools_AlgoTools.Dimensions(myShape1, out iDimMin[0], out iDimMax[0]);
                BOPTools_AlgoTools.Dimensions(myShape2, out iDimMin[1], out iDimMax[1]);

                bool bBadTypes =
                    ((myOperation == BOPAlgo_FUSE)
                     && (iDimMin[0] != iDimMax[0] || iDimMin[1] != iDimMax[1] || iDimMin[0] != iDimMin[1]))
                    || ((myOperation == BOPAlgo_CUT) && (iDimMax[0] > iDimMin[1]))
                    || ((myOperation == BOPAlgo_CUT21) && (iDimMin[0] < iDimMax[1]));

                if (bBadTypes)
                {
                    BOPAlgo_CheckResult aResult = new();
                    aResult.SetShape1(myShape1);
                    aResult.SetShape2(myShape2);
                    aResult.SetCheckStatus(BOPAlgo_BadType);
                    myResult.Append(aResult);
                }
            }
        }
    }

    /// <summary>Checks each shape on self-interference with <see cref="BOPAlgo_CheckerSI"/>.</summary>
    protected void TestSelfInterferences(Message_ProgressRange theRange)
    {
        using var aPS = new Message_ProgressScope(theRange, null, (!myShape1.IsNull() && !myShape2.IsNull() ? 2 : 1));
        int ii;
        //
        for (ii = 0; ii < 2; ii++)
        {
            TopoDS_Shape aS = (ii == 0) ? myShape1 : myShape2;
            if (aS.IsNull())
            {
                continue;
            }
            //
            bool bIsEmpty = (ii == 0) ? myEmpty1 : myEmpty2;
            if (bIsEmpty)
            {
                continue;
            }
            //
            int n1, n2;
            BOPDS_MapOfPair.Iterator aItMPK = new();
            NCollection_List<TopoDS_Shape> anArgs = new();
            BOPAlgo_CheckerSI aChecker = new();
            //
            anArgs.Append(aS);
            aChecker.SetArguments(anArgs);
            aChecker.SetNonDestructive(true);
            aChecker.SetRunParallel(myRunParallel);
            aChecker.SetFuzzyValue(myFuzzyValue);
            //
            aChecker.Perform(aPS.Next());
            bool hasError = aChecker.HasErrors();
            //
            BOPDS_DS aDS = aChecker.PDS()!;
            BOPDS_MapOfPair aMPK = aDS.Interferences();
            //
            aItMPK.Initialize(aMPK);
            for (; aItMPK.More(); aItMPK.Next())
            {
                BOPDS_Pair aPK = aItMPK.Value();
                aPK.Indices(out n1, out n2);
                if (aDS.IsNewShape(n1) || aDS.IsNewShape(n2))
                {
                    continue;
                }
                //
                // OCCT.NET: the argument's own sub-shapes (the checker may have worked on a copy, decision 6971)
                TopoDS_Shape aS1 = aChecker.Original(aDS.Shape(n1));
                TopoDS_Shape aS2 = aChecker.Original(aDS.Shape(n2));
                //
                BOPAlgo_CheckResult aResult = new();
                if (ii == 0)
                {
                    aResult.SetShape1(myShape1);
                    aResult.AddFaultyShape1(aS1);
                    if (!aS1.IsSame(aS2))
                    {
                        aResult.AddFaultyShape1(aS2);
                    }
                }
                else
                {
                    aResult.SetShape2(myShape2);
                    aResult.AddFaultyShape2(aS1);
                    if (!aS1.IsSame(aS2))
                    {
                        aResult.AddFaultyShape2(aS2);
                    }
                }
                aResult.SetCheckStatus(BOPAlgo_SelfIntersect);
                myResult.Append(aResult);
            }
            //
            if (hasError)
            {
                BOPAlgo_CheckResult aResult = new();
                if (ii == 0)
                {
                    aResult.SetShape1(myShape1);
                    aResult.AddFaultyShape1(myShape1);
                }
                else
                {
                    aResult.SetShape2(myShape2);
                    aResult.AddFaultyShape2(myShape2);
                }
                aResult.SetCheckStatus(BOPAlgo_OperationAborted);
                myResult.Append(aResult);
            }
        } // for(ii = 0; ii < 2; ii++) {
    }

    /// <summary>Checks the shapes for micro edges (for a section, only those touching the other shape).</summary>
    protected void TestSmallEdge()
    {
        int i = 0;
        BRepExtrema_DistShapeShape aDist = new();
        IntTools_Context aCtx;
        //
        aCtx = new IntTools_Context();

        for (i = 0; i < 2; i++)
        {
            TopoDS_Shape aS = (i == 0) ? myShape1 : myShape2;

            if (aS.IsNull())
            {
                continue;
            }

            TopExp_Explorer anExp = new(aS, TopAbs_EDGE);

            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
                if (BRep_Tool.Degenerated(anEdge))
                {
                    continue;
                }

                if (BOPTools_AlgoTools.IsMicroEdge(anEdge, aCtx))
                {
                    bool bKeepResult = true;

                    if (myOperation == BOPAlgo_SECTION)
                    {
                        TopoDS_Shape anOtherS = (i == 0) ? myShape2 : myShape1;

                        if (!anOtherS.IsNull())
                        {
                            aDist.LoadS2(anOtherS);

                            bool bVertexIsOnShape = false;
                            int ii = 0;
                            TopExp_Explorer anExpV = new(anEdge, TopAbs_VERTEX);

                            for (; anExpV.More(); anExpV.Next())
                            {
                                TopoDS_Shape aV = anExpV.Current();

                                aDist.LoadS1(aV);
                                aDist.Perform();

                                if (aDist.IsDone())
                                {
                                    for (ii = 1; ii <= aDist.NbSolution(); ii++)
                                    {
                                        double aTolerance = BRep_Tool.Tolerance(TopoDS.Vertex(aV));
                                        TopoDS_Shape aSupportShape = aDist.SupportOnShape2(ii);

                                        switch (aSupportShape.ShapeType())
                                        {
                                            case TopAbs_VERTEX:
                                            {
                                                aTolerance += BRep_Tool.Tolerance(TopoDS.Vertex(aSupportShape));
                                                break;
                                            }
                                            case TopAbs_EDGE:
                                            {
                                                aTolerance += BRep_Tool.Tolerance(TopoDS.Edge(aSupportShape));
                                                break;
                                            }
                                            case TopAbs_FACE:
                                            {
                                                aTolerance += BRep_Tool.Tolerance(TopoDS.Face(aSupportShape));
                                                break;
                                            }
                                            default:
                                                break;
                                        }

                                        if (aDist.Value() < aTolerance)
                                        {
                                            bVertexIsOnShape = true;
                                            break;
                                        }
                                    }
                                }
                            }

                            if (!bVertexIsOnShape)
                            {
                                bKeepResult = false;
                            }
                        }
                    }

                    if (bKeepResult)
                    {
                        BOPAlgo_CheckResult aResult = new();

                        if (i == 0)
                        {
                            aResult.SetShape1(myShape1);
                            aResult.AddFaultyShape1(anEdge);
                        }
                        else
                        {
                            aResult.SetShape2(myShape2);
                            aResult.AddFaultyShape2(anEdge);
                        }

                        aResult.SetCheckStatus(BOPAlgo_TooSmallEdge);
                        myResult.Append(aResult);

                        if (myStopOnFirst)
                        {
                            return;
                        }
                    }
                }
            }
        }
    }

    /// <summary>Checks that each face can be rebuilt from its own edges by <see cref="BOPAlgo_BuilderFace"/>.</summary>
    protected void TestRebuildFace()
    {
        if ((myOperation == BOPAlgo_SECTION) || (myOperation == BOPAlgo_UNKNOWN))
        {
            return;
        }
        int i = 0;

        for (i = 0; i < 2; i++)
        {
            TopoDS_Shape aS = (i == 0) ? myShape1 : myShape2;

            if (aS.IsNull())
            {
                continue;
            }

            TopExp_Explorer anExp = new(aS, TopAbs_FACE);
            NCollection_List<TopoDS_Shape> aLS = new();

            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Face aFace = TopoDS.Face(anExp.Current());

                TopoDS_Face aFF = aFace;
                aFF.Orientation(TopAbs_FORWARD);
                TopExp_Explorer anExpE = new(aFF, TopAbs_EDGE);
                int nbstartedges = 0;
                aLS.Clear();
                //
                for (; anExpE.More(); anExpE.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(anExpE.Current());
                    TopAbs_Orientation anOriE = aE.Orientation();
                    //
                    if (anOriE == TopAbs_INTERNAL)
                    {
                        TopoDS_Edge aEE = aE;
                        aEE.Orientation(TopAbs_FORWARD);
                        aLS.Append(aEE);
                        aEE.Orientation(TopAbs_REVERSED);
                        aLS.Append(aEE);
                    }
                    else
                    {
                        aLS.Append(aE);
                    }
                    nbstartedges++;
                }
                BOPAlgo_BuilderFace aBF = new();
                aBF.SetFace(aFace);
                aBF.SetShapes(aLS);
                aBF.Perform();
                NCollection_List<TopoDS_Shape> aLF = aBF.Areas();
                bool bBadFace = false;

                if (aLF.Extent() != 1)
                {
                    bBadFace = true;
                }
                else
                {
                    int nbedgeused = 0;
                    anExpE.Init(aLF.First(), TopAbs_EDGE);

                    for (; anExpE.More(); anExpE.Next(), nbedgeused++)
                    {
                    }

                    if (nbstartedges != nbedgeused)
                    {
                        bBadFace = true;
                    }
                }

                if (bBadFace)
                {
                    BOPAlgo_CheckResult aResult = new();

                    if (i == 0)
                    {
                        aResult.SetShape1(myShape1);
                        aResult.AddFaultyShape1(aFace);
                    }
                    else
                    {
                        aResult.SetShape2(myShape2);
                        aResult.AddFaultyShape2(aFace);
                    }

                    aResult.SetCheckStatus(BOPAlgo_NonRecoverableFace);
                    myResult.Append(aResult);

                    if (myStopOnFirst)
                    {
                        return;
                    }
                }
            }
        }
    }

    /// <summary>Not implemented (as in OCCT).</summary>
    protected void TestTangent()
    {
        // not implemented
    }

    /// <summary>Finds sub-shapes of one shape that would merge with several of the other's.</summary>
    protected void TestMergeSubShapes(TopAbs_ShapeEnum theType)
    {
        if (myShape1.IsNull() || myShape2.IsNull())
        {
            return;
        }

        if (myEmpty1 || myEmpty2)
        {
            return;
        }

        BOPAlgo_CheckStatus aStatus = BOPAlgo_CheckUnknown;

        switch (theType)
        {
            case TopAbs_VERTEX:
            {
                aStatus = BOPAlgo_IncompatibilityOfVertex;
                break;
            }
            case TopAbs_EDGE:
            {
                aStatus = BOPAlgo_IncompatibilityOfEdge;
                break;
            }
            case TopAbs_FACE:
            {
                aStatus = BOPAlgo_IncompatibilityOfFace;
                break;
            }
            default:
                return;
        }
        TopExp_Explorer anExp1 = new(myShape1, theType);
        TopExp_Explorer anExp2 = new(myShape2, theType);
        NCollection_Sequence<TopoDS_Shape> aSeq1 = new(), aSeq2 = new();
        NCollection_Map<TopoDS_Shape> aMap1 = new(TopTools_ShapeMapHasher.Instance), aMap2 = new(TopTools_ShapeMapHasher.Instance);

        for (; anExp1.More(); anExp1.Next())
        {
            TopoDS_Shape aS1 = anExp1.Current();

            if (aMap1.Contains(aS1))
            {
                continue;
            }
            aSeq1.Append(aS1);
            aMap1.Add(aS1);
        }

        for (; anExp2.More(); anExp2.Next())
        {
            TopoDS_Shape aS2 = anExp2.Current();
            if (aMap2.Contains(aS2))
            {
                continue;
            }
            aSeq2.Append(aS2);
            aMap2.Add(aS2);
        }

        // OCCT.NET: an empty side makes OCCT's Array2(1, 0, ...) an empty array; NCollection_Array2 needs a size
        bool[,] anArrayOfFlag = new bool[aSeq1.Length() + 1, aSeq2.Length() + 1];
        int i = 0, j = 0;
        for (i = 1; i <= aSeq1.Length(); i++)
        {
            for (j = 1; j <= aSeq2.Length(); j++)
            {
                anArrayOfFlag[i, j] = false;
            }
        }

        for (i = 1; i <= aSeq1.Length(); i++)
        {
            TopoDS_Shape aS1 = aSeq1.Value(i);
            NCollection_List<TopoDS_Shape> aListOfS2 = new();
            int nbs = 0;

            for (j = 1; j <= aSeq2.Length(); j++)
            {
                TopoDS_Shape aS2 = aSeq2.Value(j);
                bool bIsEqual = false;

                if (theType == TopAbs_VERTEX)
                {
                    TopoDS_Vertex aV1 = TopoDS.Vertex(aS1);
                    TopoDS_Vertex aV2 = TopoDS.Vertex(aS2);
                    gp_Pnt aP1 = BRep_Tool.Pnt(aV1);
                    gp_Pnt aP2 = BRep_Tool.Pnt(aV2);
                    double aDist = aP1.Distance(aP2);

                    if (aDist <= (BRep_Tool.Tolerance(aV1) + BRep_Tool.Tolerance(aV2)))
                    {
                        bIsEqual = true;
                    }
                }
                else if (theType == TopAbs_EDGE)
                {
                    TopoDS_Edge aE1 = TopoDS.Edge(aS1);
                    TopoDS_Edge aE2 = TopoDS.Edge(aS2);
                    //
                    IntTools_EdgeEdge aEE = new(aE1, aE2);
                    //
                    aEE.Perform();
                    if (aEE.IsDone())
                    {
                        NCollection_Sequence<IntTools_CommonPrt> aCPrts = aEE.CommonParts();
                        int ii = 0;

                        for (ii = 1; ii <= aCPrts.Length(); ii++)
                        {
                            IntTools_CommonPrt aCPart = aCPrts.Value(ii);

                            if (aCPart.Type() == TopAbs_EDGE)
                            {
                                bIsEqual = true;
                            }
                        }
                    }
                }
                else if (theType == TopAbs_FACE)
                {
                    // not yet implemented!
                }

                if (bIsEqual)
                {
                    anArrayOfFlag[i, j] = true;
                    aListOfS2.Append(aS2);
                    nbs++;
                }
            }

            if (nbs > 1)
            {
                BOPAlgo_CheckResult aResult = new();

                aResult.SetShape1(myShape1);
                aResult.SetShape2(myShape2);
                aResult.AddFaultyShape1(aS1);
                NCollection_List<TopoDS_Shape>.Iterator anIt = new(aListOfS2);

                for (; anIt.More(); anIt.Next())
                {
                    aResult.AddFaultyShape2(anIt.Value());
                }

                aResult.SetCheckStatus(aStatus);
                myResult.Append(aResult);

                if (myStopOnFirst)
                {
                    return;
                }
            }
        }

        for (i = 1; i <= aSeq2.Length(); i++)
        {
            TopoDS_Shape aS2 = aSeq2.Value(i);
            NCollection_List<TopoDS_Shape> aListOfS1 = new();
            int nbs = 0;

            for (j = 1; j <= aSeq1.Length(); j++)
            {
                TopoDS_Shape aS1 = aSeq1.Value(j);

                if (anArrayOfFlag[j, i])
                {
                    aListOfS1.Append(aS1);
                    nbs++;
                }
            }

            if (nbs > 1)
            {
                BOPAlgo_CheckResult aResult = new();

                aResult.SetShape1(myShape1);
                aResult.SetShape2(myShape2);
                NCollection_List<TopoDS_Shape>.Iterator anIt = new(aListOfS1);

                for (; anIt.More(); anIt.Next())
                {
                    aResult.AddFaultyShape1(anIt.Value());
                }
                aResult.AddFaultyShape2(aS2);

                aResult.SetCheckStatus(aStatus);
                myResult.Append(aResult);

                if (myStopOnFirst)
                {
                    return;
                }
            }
        }
    }

    /// <summary>Checks the vertices of the two shapes for merging problems.</summary>
    protected void TestMergeVertex()
    {
        TestMergeSubShapes(TopAbs_VERTEX);
    }

    /// <summary>Checks the edges of the two shapes for merging problems.</summary>
    protected void TestMergeEdge()
    {
        TestMergeSubShapes(TopAbs_EDGE);
    }

    /// <summary>Lists the edges and faces whose geometry is only C0.</summary>
    protected void TestContinuity()
    {
        int i, j, aNbS;
        TopExp_Explorer aExp = new();
        //
        for (i = 0; i < 2; ++i)
        {
            TopoDS_Shape aS = i == 0 ? myShape1 : myShape2;
            if (aS.IsNull())
            {
                continue;
            }
            //
            NCollection_IndexedMap<TopoDS_Shape> aMS = new(TopTools_ShapeMapHasher.Instance);
            // Edges
            aExp.Init(aS, TopAbs_EDGE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                if (BRep_Tool.Degenerated(aE))
                {
                    continue;
                }
                Geom_Curve? aC = BRep_Tool.Curve(aE, out _, out _);
                if (aC!.Continuity() == GeomAbs_Shape.GeomAbs_C0)
                {
                    aMS.Add(aE);
                }
            }
            // Faces
            aExp.Init(aS, TopAbs_FACE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Face aF = TopoDS.Face(aExp.Current());
                Geom_Surface aSurf = BRep_Tool.Surface(aF)!;
                if (aSurf.Continuity() == GeomAbs_Shape.GeomAbs_C0)
                {
                    aMS.Add(aF);
                }
            }
            //
            // add shapes with continuity C0 to result
            aNbS = aMS.Extent();
            for (j = 1; j <= aNbS; ++j)
            {
                TopoDS_Shape aFS = aMS.FindKey(j);
                BOPAlgo_CheckResult aResult = new();
                if (i == 0)
                {
                    aResult.SetShape1(myShape1);
                    aResult.AddFaultyShape1(aFS);
                }
                else
                {
                    aResult.SetShape2(myShape2);
                    aResult.AddFaultyShape2(aFS);
                }
                aResult.SetCheckStatus(BOPAlgo_GeomAbs_C0);
                myResult.Append(aResult);
            }
        }
    }

    /// <summary>Lists the edges whose curves are farther from their faces' surfaces than their tolerance.</summary>
    protected void TestCurveOnSurface()
    {
        int i;
        double aT, aD, aTolE;
        TopExp_Explorer aExpF = new(), aExpE = new();
        //
        for (i = 0; i < 2; i++)
        {
            TopoDS_Shape aS = (i == 0) ? myShape1 : myShape2;
            if (aS.IsNull())
            {
                continue;
            }
            //
            aExpF.Init(aS, TopAbs_FACE);
            for (; aExpF.More(); aExpF.Next())
            {
                TopoDS_Face aF = TopoDS.Face(aExpF.Current());
                //
                aExpE.Init(aF, TopAbs_EDGE);
                for (; aExpE.More(); aExpE.Next())
                {
                    TopoDS_Edge aE = TopoDS.Edge(aExpE.Current());
                    //
                    if (BOPTools_AlgoTools.ComputeTolerance(aF, aE, out aD, out aT))
                    {
                        aTolE = BRep_Tool.Tolerance(aE);
                        if (aD > aTolE)
                        {
                            BOPAlgo_CheckResult aResult = new();
                            aResult.SetCheckStatus(BOPAlgo_InvalidCurveOnSurface);
                            if (i == 0)
                            {
                                aResult.SetShape1(myShape1);
                                aResult.AddFaultyShape1(aE);
                                aResult.AddFaultyShape1(aF);
                                aResult.SetMaxDistance1(aD);
                                aResult.SetMaxParameter1(aT);
                            }
                            else
                            {
                                aResult.SetShape2(myShape2);
                                aResult.AddFaultyShape2(aE);
                                aResult.AddFaultyShape2(aF);
                                aResult.SetMaxDistance2(aD);
                                aResult.SetMaxParameter2(aT);
                            }
                            myResult.Append(aResult);
                        }
                    }
                }
            }
        }
    }
}
