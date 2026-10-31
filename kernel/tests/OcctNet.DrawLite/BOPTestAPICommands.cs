// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// BOPTest_APICommands.cxx: <c>bapibuild</c>, <c>bapibop</c> and <c>bapisplit</c>, the session's objects and tools
/// through BRepAlgoAPI, with the result simplified as <c>bsimplify</c> asks (unit 6L).
/// </summary>
/// <remarks>Draw-lite has no help texts, so PrintHelp's usage errors return 1 with an empty result.</remarks>
internal static class BOPTestAPICommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bapibuild", BApiBuild);
        DrawCxx.Add(theInterpreter, "bapibop", BApiBop);
        DrawCxx.Add(theInterpreter, "bapisplit", BApiSplit);
    }

    /// <summary>The warnings, then the errors and false if any (the commands' common ending before the result).</summary>
    private static bool DumpReport(DrawResult di, BRepAlgoAPI_Algo theAlgo)
    {
        if (theAlgo.HasWarnings())
        {
            StringWriter aSStream = new();
            theAlgo.DumpWarnings(aSStream);
            di.Append(aSStream.ToString());
        }
        if (theAlgo.HasErrors())
        {
            StringWriter aSStream = new();
            theAlgo.DumpErrors(aSStream);
            di.Append(aSStream.ToString());
            return false;
        }
        return true;
    }

    private static int BApiBop(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            return 1;
        }

        BOPAlgo_Operation anOp = BOPTestPartitionCommands.GetOperationType(a[2]);
        if (anOp == BOPAlgo_Operation.BOPAlgo_UNKNOWN)
        {
            di.Append("Invalid operation type\n");
            return 0;
        }
        BRepAlgoAPI_BooleanOperation pBuilder = anOp switch
        {
            BOPAlgo_Operation.BOPAlgo_COMMON => new BRepAlgoAPI_Common(),
            BOPAlgo_Operation.BOPAlgo_FUSE => new BRepAlgoAPI_Fuse(),
            BOPAlgo_Operation.BOPAlgo_SECTION => new BRepAlgoAPI_Section(),
            _ => new BRepAlgoAPI_Cut(),
        };
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        NCollection_List<TopoDS_Shape> aLS = aSession.Shapes;
        NCollection_List<TopoDS_Shape> aLT = aSession.Tools;
        if (anOp != BOPAlgo_Operation.BOPAlgo_CUT21)
        {
            pBuilder.SetArguments(aLS);
            pBuilder.SetTools(aLT);
        }
        else
        {
            pBuilder.SetArguments(aLT);
            pBuilder.SetTools(aLS);
        }
        pBuilder.SetRunParallel(aSession.RunParallel);
        pBuilder.SetFuzzyValue(aSession.FuzzyValue);
        pBuilder.SetNonDestructive(aSession.NonDestructive);
        pBuilder.SetGlue(aSession.Glue);
        pBuilder.SetCheckInverted(aSession.CheckInverted);
        pBuilder.SetUseOBB(aSession.UseOBB);
        pBuilder.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        pBuilder.Build();
        pBuilder.SimplifyResult(aSession.UnifyEdges, aSession.UnifyFaces, aSession.Angular);

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, pBuilder.History());
        }

        if (!DumpReport(di, pBuilder))
        {
            return 0;
        }
        TopoDS_Shape aR = pBuilder.Shape();
        if (aR.IsNull())
        {
            di.Append("Result is a null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BApiBuild(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        BRepAlgoAPI_BuilderAlgo aBuilder = new();
        NCollection_List<TopoDS_Shape> aLS = new(aSession.Shapes);
        foreach (TopoDS_Shape aS in aSession.Tools)
        {
            aLS.Append(aS);
        }
        aBuilder.SetArguments(aLS);
        aBuilder.SetRunParallel(aSession.RunParallel);
        aBuilder.SetFuzzyValue(aSession.FuzzyValue);
        aBuilder.SetNonDestructive(aSession.NonDestructive);
        aBuilder.SetGlue(aSession.Glue);
        aBuilder.SetCheckInverted(aSession.CheckInverted);
        aBuilder.SetUseOBB(aSession.UseOBB);
        aBuilder.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        aBuilder.Build();
        aBuilder.SimplifyResult(aSession.UnifyEdges, aSession.UnifyFaces, aSession.Angular);

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aBuilder.History());
        }

        if (!DumpReport(di, aBuilder))
        {
            return 0;
        }
        TopoDS_Shape aR = aBuilder.Shape();
        if (aR.IsNull())
        {
            di.Append("Result is a null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BApiSplit(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        BRepAlgoAPI_Splitter aSplitter = new();
        aSplitter.SetArguments(aSession.Shapes);
        aSplitter.SetTools(aSession.Tools);
        aSplitter.SetRunParallel(aSession.RunParallel);
        aSplitter.SetFuzzyValue(aSession.FuzzyValue);
        aSplitter.SetNonDestructive(aSession.NonDestructive);
        aSplitter.SetGlue(aSession.Glue);
        aSplitter.SetCheckInverted(aSession.CheckInverted);
        aSplitter.SetUseOBB(aSession.UseOBB);
        aSplitter.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        aSplitter.Build();
        aSplitter.SimplifyResult(aSession.UnifyEdges, aSession.UnifyFaces, aSession.Angular);

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aSplitter.History());
        }

        if (!DumpReport(di, aSplitter))
        {
            return 0;
        }
        TopoDS_Shape aR = aSplitter.Shape();
        if (aR.IsNull())
        {
            di.Append("Result is a null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }
}
