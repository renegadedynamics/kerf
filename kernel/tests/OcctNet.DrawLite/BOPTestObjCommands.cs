// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The session commands of BOPTest that the builders of unit 6G need: BOPTest_ObjCommands.cxx (<c>baddobjects</c>,
/// <c>bclearobjects</c>, <c>baddtools</c>, <c>bcleartools</c>, <c>baddcompound</c>, <c>baddctools</c>,
/// <c>bclear</c>), <c>bfillds</c> of BOPTest_PartitionCommands.cxx and BOPTest_OptionCommands.cxx but
/// <c>bsimplify</c> (BRepAlgoAPI's SimplifyResult, unit 6P).
/// </summary>
/// <remarks>
/// The session is the interpreter's (<see cref="BOPTestObjects"/>). Draw-lite has no help texts, so PrintHelp's
/// usage errors return 1 with an empty result. Unit 6L adds the rest of BOPTest (bbuild, bsplit, bbop, buildbop, the
/// API and debug commands) over the same session.
/// </remarks>
internal static class BOPTestObjCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "baddobjects", BAddObjects);
        DrawCxx.Add(theInterpreter, "bclearobjects", BClearObjects);
        DrawCxx.Add(theInterpreter, "baddtools", BAddTools);
        DrawCxx.Add(theInterpreter, "bcleartools", BClearTools);
        DrawCxx.Add(theInterpreter, "baddcompound", BAddCompound);
        DrawCxx.Add(theInterpreter, "baddctools", BAddCTools);
        DrawCxx.Add(theInterpreter, "bclear", BClear);
        DrawCxx.Add(theInterpreter, "bfillds", BFillDS);
        DrawCxx.Add(theInterpreter, "boptions", BOptions);
        DrawCxx.Add(theInterpreter, "brunparallel", BRunParallel);
        DrawCxx.Add(theInterpreter, "bnondestructive", BNonDestructive);
        DrawCxx.Add(theInterpreter, "bfuzzyvalue", BFuzzyValue);
        DrawCxx.Add(theInterpreter, "bglue", BGlue);
        DrawCxx.Add(theInterpreter, "bdrawwarnshapes", BDrawWarnShapes);
        DrawCxx.Add(theInterpreter, "bcheckinverted", BCheckInverted);
        DrawCxx.Add(theInterpreter, "buseobb", BUseOBB);
    }

    private static int BAddCompound(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        TopoDS_Shape aS = theDI.GetShape(a[1]);
        NCollection_List<TopoDS_Shape> aLS = BOPTestObjects.Of(theDI).Shapes;
        for (TopoDS_Iterator aIt = new(aS); aIt.More(); aIt.Next())
        {
            aLS.Append(aIt.Value());
        }
        return 0;
    }

    private static int BAddCTools(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        TopoDS_Shape aS = theDI.GetShape(a[1]);
        NCollection_List<TopoDS_Shape> aLT = BOPTestObjects.Of(theDI).Tools;
        for (TopoDS_Iterator aIt = new(aS); aIt.More(); aIt.Next())
        {
            aLT.Append(aIt.Value());
        }
        return 0;
    }

    private static int BAddObjects(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        NCollection_List<TopoDS_Shape> aLS = BOPTestObjects.Of(theDI).Shapes;
        for (int i = 1; i < a.Count; ++i)
        {
            aLS.Append(theDI.GetShape(a[i]));
        }
        return 0;
    }

    private static int BClearObjects(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 1)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).Shapes.Clear();
        return 0;
    }

    private static int BAddTools(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        NCollection_List<TopoDS_Shape> aLS = BOPTestObjects.Of(theDI).Tools;
        for (int i = 1; i < a.Count; ++i)
        {
            aLS.Append(theDI.GetShape(a[i]));
        }
        return 0;
    }

    private static int BClearTools(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 1)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).Tools.Clear();
        return 0;
    }

    private static int BClear(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 1)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).Clear();
        return 0;
    }

    /// <summary><c>bfillds [-t]</c>: intersects the objects and tools with the session's pave filler.</summary>
    private static int BFillDS(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n > 2)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        NCollection_List<TopoDS_Shape> aLS = aSession.Shapes;
        if (aLS.Extent() == 0)
        {
            di.Append("No objects to process\n");
            return 0;
        }
        //
        bool bShowTime = false;
        if (n == 2)
        {
            if (a[1] == "-t")
            {
                bShowTime = true;
            }
            else
            {
                di.Append("Warning: invalid key\n");
            }
        }
        //
        NCollection_List<TopoDS_Shape> aLC = new();
        foreach (TopoDS_Shape aS in aLS)
        {
            aLC.Append(aS);
        }
        foreach (TopoDS_Shape aS in aSession.Tools)
        {
            aLC.Append(aS);
        }
        //
        BOPAlgo_PaveFiller aPF = aSession.PaveFiller();
        aPF.SetArguments(aLC);
        aPF.SetRunParallel(aSession.RunParallel);
        aPF.SetNonDestructive(aSession.NonDestructive);
        aPF.SetFuzzyValue(aSession.FuzzyValue);
        aPF.SetGlue(aSession.Glue);
        aPF.SetUseOBB(aSession.UseOBB);
        //
        Stopwatch aTimer = Stopwatch.StartNew();
        aPF.Perform();
        BOPTestObjects.ReportAlerts(theDI, di, aPF.GetReport());
        if (aPF.HasErrors())
        {
            return 0;
        }
        //
        aTimer.Stop();
        if (bShowTime)
        {
            di.Append($"  Tps: {aTimer.Elapsed.TotalSeconds,7:F2}\n");
        }
        return 0;
    }

    private static int BOptions(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n > 2)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        if (n == 2)
        {
            if (a[1] != "-default")
            {
                return 1;
            }
            // Set all options to default values
            aSession.SetDefaultOptions();
            return 0;
        }
        //
        BOPAlgo_GlueEnum aGlue = aSession.Glue;
        static string YesNo(bool theFlag) => theFlag ? "Yes" : "No";
        di.Append($" RunParallel: {YesNo(aSession.RunParallel)} \t\t(use \"brunparallel\" command to change)\n");
        di.Append($" NonDestructive: {YesNo(aSession.NonDestructive)} \t\t(use \"bnondestructive\" command to change)\n");
        di.Append($" FuzzyValue: {DrawFormat.AsciiString(aSession.FuzzyValue)} \t\t(use \"bfuzzyvalue\" command to change)\n");
        di.Append($" GlueOption: {(aGlue == BOPAlgo_GlueEnum.BOPAlgo_GlueOff ? "Off" : aGlue == BOPAlgo_GlueEnum.BOPAlgo_GlueFull ? "Full" : "Shift")} \t\t(use \"bglue\" command to change)\n");
        di.Append($" Draw Warning Shapes: {YesNo(aSession.DrawWarnShapes)} \t(use \"bdrawwarnshapes\" command to change)\n");
        di.Append($" Check for invert solids: {YesNo(aSession.CheckInverted)} \t(use \"bcheckinverted\" command to change)\n");
        di.Append($" Use OBB: {YesNo(aSession.UseOBB)} \t\t\t(use \"buseobb\" command to change)\n");
        di.Append($" Unify Edges: {YesNo(aSession.UnifyEdges)} \t\t(use \"bsimplify -e\" command to change)\n");
        di.Append($" Unify Faces: {YesNo(aSession.UnifyFaces)} \t\t(use \"bsimplify -f\" command to change)\n");
        di.Append($" Angular: {DrawFormat.AsciiString(aSession.Angular)} \t\t(use \"bsimplify -a\" command to change)\n");
        return 0;
    }

    private static int BFuzzyValue(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).FuzzyValue = theDI.Atof(a[1]);
        return 0;
    }

    private static int BRunParallel(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).RunParallel = theDI.Atoi(a[1]) != 0;
        return 0;
    }

    private static int BNonDestructive(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).NonDestructive = theDI.Atoi(a[1]) != 0;
        return 0;
    }

    private static int BGlue(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        int iGlue = theDI.Atoi(a[1]);
        if (iGlue < 0 || iGlue > 2)
        {
            di.Append("Wrong value.\n");
            return 1;
        }
        BOPTestObjects.Of(theDI).Glue = (BOPAlgo_GlueEnum)iGlue;
        return 0;
    }

    private static int BDrawWarnShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).DrawWarnShapes = theDI.Atoi(a[1]) != 0;
        return 0;
    }

    private static int BCheckInverted(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).CheckInverted = theDI.Atoi(a[1]) != 0;
        return 0;
    }

    private static int BUseOBB(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }
        BOPTestObjects.Of(theDI).UseOBB = theDI.Atoi(a[1]) != 0;
        return 0;
    }
}
