// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using System.Runtime.CompilerServices;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of BOPTest_MkConnectedCommands.cxx (unit 6G): <c>makeconnected</c>, <c>cmaterialson</c>,
/// <c>cmakeperiodic</c>, <c>crepeatshape</c>, <c>cperiodictwins</c>, <c>cclearrepetitions</c>, over
/// BOPAlgo_MakeConnected.
/// </summary>
/// <remarks>
/// OCCT keeps one tool for the process; Draw-lite keeps one per interpreter. Each command sets the session's history
/// (<c>savehistory</c>) as OCCT's does. Draw-lite has no help texts (PrintHelp: return 1).
/// </remarks>
internal static class BOPTestMkConnectedCommands
{
    private static readonly ConditionalWeakTable<DrawInterpreter, BOPAlgo_MakeConnected> theTools = new();

    private static BOPAlgo_MakeConnected Tool(DrawInterpreter theDI) => theTools.GetValue(theDI, static _ => new BOPAlgo_MakeConnected());

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "makeconnected", MakeConnected);
        DrawCxx.Add(theInterpreter, "cmakeperiodic", MakePeriodic);
        DrawCxx.Add(theInterpreter, "cmaterialson", MaterialsOn);
        DrawCxx.Add(theInterpreter, "crepeatshape", RepeatShape);
        DrawCxx.Add(theInterpreter, "cperiodictwins", GetTwins);
        DrawCxx.Add(theInterpreter, "cclearrepetitions", ClearRepetitions);
    }

    private static int MakeConnected(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 3)
        {
            return 1;
        }

        BOPAlgo_MakeConnected aTool = Tool(theDI);
        aTool.Clear();

        for (int i = 2; i < theArgc; ++i)
        {
            Shape aS = theDI.GetShape(theArgv[i]);
            if (aS.IsNull)
            {
                di.Append("Error: ").Append(theArgv[i]).Append(" is a null shape. Skip it.\n");
                continue;
            }
            aTool.AddArgument(aS);
        }

        aTool.SetRunParallel(BOPTestObjects.Of(theDI).RunParallel);

        aTool.Perform();

        // Print Error/Warning messages
        BOPTestObjects.ReportAlerts(theDI, di, aTool.GetReport());

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aTool.History());

        if (aTool.HasErrors())
        {
            return 0;
        }

        // Draw the result shape
        theDI.SetShape(theArgv[1], aTool.Shape());
        return 0;
    }

    private static int MakePeriodic(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 4)
        {
            return 1;
        }

        BOPAlgo_MakeConnected aTool = Tool(theDI);
        if (aTool.InternalShape().IsNull || aTool.HasErrors())
        {
            di.Append("Make the shapes connected first.\n");
            return 1;
        }

        BOPAlgo_MakePeriodic.PeriodicityParams aParams = new();

        for (int i = 2; i < theArgc;)
        {
            int aDirID = BOPTestPeriodicityCommands.DirectionID(theArgv[i]);
            if (aDirID < 0)
            {
                di.Append(theArgv[i]).Append(" - Invalid key\n");
                return 1;
            }

            string cDirName = theArgv[i][1].ToString();

            double aPeriod = 0;
            if (theArgc > i + 1)
            {
                aPeriod = theDI.Atof(theArgv[++i]);
            }

            if (aPeriod <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                di.Append("Period for ").Append(cDirName).Append(" direction is not set\n");
                return 1;
            }

            aParams.myPeriodic[aDirID] = true;
            aParams.myPeriod[aDirID] = aPeriod;

            ++i;
            if (theArgc > i + 1)
            {
                // Check if trimming is necessary
                if (theArgv[i] == "-trim")
                {
                    if (theArgc == (i + 1))
                    {
                        di.Append("Trim bounds for ").Append(cDirName).Append(" direction are not set\n");
                        return 1;
                    }
                    double aFirst = theDI.Atof(theArgv[++i]);

                    aParams.myIsTrimmed[aDirID] = false;
                    aParams.myPeriodFirst[aDirID] = aFirst;
                    ++i;
                }
            }
        }

        aTool.MakePeriodic(aParams);

        // Print Error/Warning messages
        BOPTestObjects.ReportAlerts(theDI, di, aTool.GetReport());

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aTool.History());

        if (aTool.HasErrors())
        {
            return 0;
        }

        // Draw the result shape
        theDI.SetShape(theArgv[1], aTool.PeriodicShape());
        return 0;
    }

    private static int RepeatShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 4)
        {
            return 1;
        }

        BOPAlgo_MakeConnected aTool = Tool(theDI);
        if (aTool.PeriodicityTool().HasErrors())
        {
            di.Append("The shapes have not been made periodic yet.\n");
            return 1;
        }

        for (int i = 2; i < theArgc; ++i)
        {
            int aDirID = BOPTestPeriodicityCommands.DirectionID(theArgv[i]);
            if (aDirID < 0)
            {
                di.Append(theArgv[i]).Append(" - Invalid key\n");
                return 1;
            }

            string cDirName = theArgv[i][1].ToString();

            int aTimes = 0;
            if (theArgc > i + 1)
            {
                aTimes = theDI.Atoi(theArgv[++i]);
            }

            if (aTimes == 0)
            {
                di.Append("Number of repetitions for ").Append(cDirName).Append(" direction is not set\n");
                return 1;
            }

            aTool.RepeatShape(aDirID, aTimes);
        }

        // Print Error/Warning messages
        BOPTestObjects.ReportAlerts(theDI, di, aTool.GetReport());

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aTool.History());

        if (aTool.HasErrors())
        {
            return 0;
        }

        // Draw the result shape
        theDI.SetShape(theArgv[1], aTool.PeriodicShape());
        return 0;
    }

    private static int MaterialsOn(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        if (theArgv.Count != 4)
        {
            return 1;
        }

        // Get the shape to get materials
        Shape aShape = theDI.GetShape(theArgv[3]);
        if (aShape.IsNull)
        {
            di.Append("Error: ").Append(theArgv[3]).Append(" is a null shape.\n");
            return 1;
        }

        // Get the sign of a shape
        bool bPositive;
        if (theArgv[2] == "+")
        {
            bPositive = true;
        }
        else if (theArgv[2] == "-")
        {
            bPositive = false;
        }
        else
        {
            di.Append(theArgv[2]).Append(" - invalid key.\n");
            return 1;
        }

        BOPAlgo_MakeConnected aTool = Tool(theDI);
        IReadOnlyList<Shape> aLS = [.. bPositive ? aTool.MaterialsOnPositiveSide(aShape) : aTool.MaterialsOnNegativeSide(aShape)];
        if (aLS.Count == 0)
        {
            di.Append("No materials on this side.\n");
        }
        theDI.SetShape(theArgv[1], BOPTestPeriodicityCommands.ListShape(aLS));
        return 0;
    }

    private static int GetTwins(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        if (theArgv.Count != 3)
        {
            return 1;
        }

        // Get the shape to find twins
        Shape aShape = theDI.GetShape(theArgv[2]);
        if (aShape.IsNull)
        {
            di.Append("Error: ").Append(theArgv[2]).Append(" is a null shape.\n");
            return 1;
        }

        IReadOnlyList<Shape> aTwins = [.. Tool(theDI).PeriodicityTool().GetTwins(aShape)];
        if (aTwins.Count == 0)
        {
            di.Append("No twins for the shape.\n");
        }
        theDI.SetShape(theArgv[1], BOPTestPeriodicityCommands.ListShape(aTwins));
        return 0;
    }

    private static int ClearRepetitions(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        BOPAlgo_MakeConnected aTool = Tool(theDI);
        // Clear all previous repetitions
        aTool.ClearRepetitions();

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aTool.History());

        if (theArgv.Count > 1)
        {
            theDI.SetShape(theArgv[1], aTool.PeriodicShape());
        }
        return 0;
    }
}
