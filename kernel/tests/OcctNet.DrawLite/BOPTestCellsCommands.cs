// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of BOPTest_CellsCommands.cxx (unit 6G): <c>bcbuild</c>, <c>bcaddall</c>, <c>bcremoveall</c>,
/// <c>bcadd</c>, <c>bcremove</c>, <c>bcremoveint</c>, <c>bcmakecontainers</c>, over the session's cells builder
/// (<see cref="BOPTestObjects.CellsBuilder"/>) and the pave filler of <c>bfillds</c>.
/// </summary>
internal static class BOPTestCellsCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bcbuild", BCBuild);
        DrawCxx.Add(theInterpreter, "bcaddall", BCAddAll);
        DrawCxx.Add(theInterpreter, "bcremoveall", BCRemoveAll);
        DrawCxx.Add(theInterpreter, "bcadd", BCAdd);
        DrawCxx.Add(theInterpreter, "bcremove", BCRemove);
        DrawCxx.Add(theInterpreter, "bcremoveint", BCRemoveInt);
        DrawCxx.Add(theInterpreter, "bcmakecontainers", BCMakeContainers);
    }

    // Updates the history of the Cells Builder in the session
    private static void StoreHistory(DrawInterpreter theDI, CellsBuilder theBuilder)
    {
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, theBuilder.Arguments(), new BOPTestBuilderHistory(theBuilder));
        }
    }

    private static int BCBuild(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Cells builder. Use: bcbuild r\n");
            return 1;
        }
        //
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        if (aSession.PDS() is null)
        {
            di.Append(" prepare PaveFiller first\n");
            return 1;
        }
        //
        PaveFiller aPF = aSession.PaveFiller();
        //
        CellsBuilder aCBuilder = aSession.CellsBuilder();
        aCBuilder.Clear();
        //
        foreach (Shape aS in aSession.Shapes)
        {
            aCBuilder.AddArgument(aS);
        }
        foreach (Shape aS in aSession.Tools)
        {
            aCBuilder.AddArgument(aS);
        }
        //
        // set the options to the algorithm
        aCBuilder.SetRunParallel(aSession.RunParallel);
        aCBuilder.SetFuzzyValue(aSession.FuzzyValue);
        aCBuilder.SetNonDestructive(aSession.NonDestructive);
        aCBuilder.SetGlue(aSession.Glue);
        aCBuilder.SetCheckInverted(aSession.CheckInverted);
        aCBuilder.SetUseOBB(aSession.UseOBB);
        aCBuilder.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        //
        aCBuilder.PerformWithFiller(aPF);
        BOPTestObjects.ReportAlerts(theDI, di, aCBuilder.GetReport());
        // Store the history of the Cells Builder into the session
        StoreHistory(theDI, aCBuilder);
        if (aCBuilder.HasErrors)
        {
            return 0;
        }
        //
        aSession.SetBuilder(aCBuilder);
        //
        Shape aR = aCBuilder.GetAllParts();
        if (aR.IsNull)
        {
            di.Append("no parts were built\n");
            return 0;
        }
        //
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BCAddAll(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2 || n > 5)
        {
            di.Append("Add all parts to result. Use: bcaddall r [-m material [-u]]\n");
            return 1;
        }
        //
        int iMaterial = 0;
        bool bUpdate = false;
        //
        if (n > 3)
        {
            if (a[2] == "-m")
            {
                iMaterial = theDI.Atoi(a[3]);
            }
            //
            if (n == 5)
            {
                bUpdate = a[4] == "-u";
            }
        }
        //
        CellsBuilder aCBuilder = BOPTestObjects.Of(theDI).CellsBuilder();
        //
        aCBuilder.ClearWarnings();
        aCBuilder.AddAllToResult(iMaterial, bUpdate);
        BOPTestObjects.ReportAlerts(theDI, di, aCBuilder.GetReport());
        //
        Shape aR = aCBuilder.Shape();
        StoreHistory(theDI, aCBuilder);
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BCRemoveAll(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 1)
        {
            di.Append("Remove all parts from result. Use: bcremoveall\n");
            return 1;
        }
        //
        CellsBuilder aCBuilder = BOPTestObjects.Of(theDI).CellsBuilder();
        //
        aCBuilder.RemoveAllFromResult();
        StoreHistory(theDI, aCBuilder);
        return 0;
    }

    private static int BCAdd(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            di.Append("Add parts to result. Use: bcadd r s1 (0,1) s2 (0,1) ... [-m material [-u]]\n");
            return 1;
        }
        //
        NCollection_List<Shape> aLSToTake = new(), aLSToAvoid = new();
        int i, iMaterial, iTake, n1;
        bool bUpdate;
        //
        iMaterial = 0;
        bUpdate = false;
        n1 = n;
        //
        if (a[n - 3] == "-m")
        {
            iMaterial = theDI.Atoi(a[n - 2]);
            bUpdate = a[n - 1] == "-u";
            n1 = n - 3;
        }
        else if (a[n - 2] == "-m")
        {
            iMaterial = theDI.Atoi(a[n - 1]);
            n1 = n - 2;
        }
        //
        for (i = 2; i < n1; i += 2)
        {
            Shape aS = theDI.GetShape(a[i]);
            if (aS.IsNull)
            {
                di.Append(a[i]).Append(" is a null shape\n");
                continue;
            }
            // OCCT reads a[i + 1] past the end for an odd count (the terminating null): Atoi of it is 0
            iTake = i + 1 < n ? theDI.Atoi(a[i + 1]) : 0;
            //
            if (iTake != 0)
            {
                aLSToTake.Append(aS);
            }
            else
            {
                aLSToAvoid.Append(aS);
            }
        }
        //
        if (aLSToTake.IsEmpty())
        {
            di.Append("No shapes from which to add the parts\n");
            return 1;
        }
        //
        CellsBuilder aCBuilder = BOPTestObjects.Of(theDI).CellsBuilder();
        //
        aCBuilder.ClearWarnings();
        aCBuilder.AddToResult(aLSToTake, aLSToAvoid, iMaterial, bUpdate);
        BOPTestObjects.ReportAlerts(theDI, di, aCBuilder.GetReport());
        //
        Shape aR = aCBuilder.Shape();
        StoreHistory(theDI, aCBuilder);
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BCRemove(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4 || ((n % 2) != 0))
        {
            di.Append("Remove parts from result. Use: bcremove r s1 (0,1) s2 (0,1) ...\n");
            return 1;
        }
        //
        NCollection_List<Shape> aLSToTake = new(), aLSToAvoid = new();
        for (int i = 2; i < n; i += 2)
        {
            Shape aS = theDI.GetShape(a[i]);
            if (aS.IsNull)
            {
                di.Append(a[i]).Append(" is a null shape\n");
                return 1;
            }
            int iTake = theDI.Atoi(a[i + 1]);
            //
            if (iTake != 0)
            {
                aLSToTake.Append(aS);
            }
            else
            {
                aLSToAvoid.Append(aS);
            }
        }
        //
        if (aLSToTake.IsEmpty())
        {
            di.Append("No shapes from which to remove the parts\n");
            return 1;
        }
        //
        CellsBuilder aCBuilder = BOPTestObjects.Of(theDI).CellsBuilder();
        aCBuilder.RemoveFromResult(aLSToTake, aLSToAvoid);
        //
        Shape aR = aCBuilder.Shape();
        StoreHistory(theDI, aCBuilder);
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BCRemoveInt(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Remove internal boundaries. Use: bcremoveint r\n");
            return 1;
        }
        //
        CellsBuilder aCBuilder = BOPTestObjects.Of(theDI).CellsBuilder();
        //
        aCBuilder.ClearWarnings();
        aCBuilder.RemoveInternalBoundaries();
        BOPTestObjects.ReportAlerts(theDI, di, aCBuilder.GetReport());
        //
        Shape aR = aCBuilder.Shape();
        StoreHistory(theDI, aCBuilder);
        theDI.SetShape(a[1], aR);
        return 0;
    }

    private static int BCMakeContainers(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            di.Append("Make containers from the parts added to result. Use: bcmakecontainers r\n");
            return 1;
        }
        //
        CellsBuilder aCBuilder = BOPTestObjects.Of(theDI).CellsBuilder();
        aCBuilder.MakeContainers();
        //
        theDI.SetShape(a[1], aCBuilder.Shape());
        return 0;
    }
}
