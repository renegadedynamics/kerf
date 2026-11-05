// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The commands of BOPTest_PeriodicityCommands.cxx (unit 6G): <c>makeperiodic</c>, <c>periodictwins</c>,
/// <c>repeatshape</c>, <c>clearrepetitions</c>, over BOPAlgo_MakePeriodic.
/// </summary>
/// <remarks>
/// OCCT keeps one periodicity maker for the process; Draw-lite keeps one per interpreter. Each command sets the
/// session's history (<c>savehistory</c>) as OCCT's does. Draw-lite has no help texts (PrintHelp: return 1).
/// </remarks>
internal static class BOPTestPeriodicityCommands
{
    private static readonly ConditionalWeakTable<DrawInterpreter, MakePeriodicAlgorithm> theMakers = new();

    private static MakePeriodicAlgorithm Maker(DrawInterpreter theDI) => theMakers.GetValue(theDI, static _ => new MakePeriodicAlgorithm());

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "makeperiodic", MakePeriodic);
        DrawCxx.Add(theInterpreter, "periodictwins", GetTwins);
        DrawCxx.Add(theInterpreter, "repeatshape", RepeatShape);
        DrawCxx.Add(theInterpreter, "clearrepetitions", ClearRepetitions);
    }

    /// <summary>The direction of a key -x, -y or -z (any case), or -1.</summary>
    internal static int DirectionID(string theKey)
        => theKey.ToLowerInvariant() switch
        {
            "-x" => 0,
            "-y" => 1,
            "-z" => 2,
            _ => -1,
        };

    /// <summary>A list of shapes as Draw sets it: null if empty, the shape if one, else a compound.</summary>
    internal static Shape ListShape(IReadOnlyList<Shape> theList)
    {
        if (theList.Count == 0)
        {
            return new Shape();
        }
        if (theList.Count == 1)
        {
            return theList[0];
        }
        BrepBuilder aBB = new();
        Compound aC = new();
        aBB.MakeCompound(ref aC);
        foreach (Shape aS in theList)
        {
            aBB.Add(aC, aS);
        }
        return aC;
    }

    private static int MakePeriodic(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 5)
        {
            return 1;
        }

        // Get the shape to make periodic
        Shape aShape = theDI.GetShape(theArgv[2]);
        if (aShape.IsNull)
        {
            di.Append("Error: ").Append(theArgv[2]).Append(" is a null shape.\n");
            return 1;
        }

        MakePeriodicAlgorithm aMaker = Maker(theDI);
        aMaker.Clear();
        aMaker.SetShape(aShape);

        for (int i = 3; i < theArgc;)
        {
            // Get periodicity
            int iDir = i;

            int aDirID = DirectionID(theArgv[i]);
            if (aDirID < 0)
            {
                di.Append(theArgv[i]).Append(" - Invalid key\n");
                return 1;
            }

            string cDirName = theArgv[iDir][1].ToString();

            if (theArgc == (i + 1))
            {
                di.Append("Period for ").Append(cDirName).Append(" direction is not set\n");
                return 1;
            }

            double aPeriod = theDI.Atof(theArgv[++i]);

            aMaker.MakePeriodic(aDirID, true, aPeriod);

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

                    aMaker.SetTrimmed(aDirID, false, aFirst);
                    ++i;
                }
            }
        }

        aMaker.SetRunParallel(BOPTestObjects.Of(theDI).RunParallel);

        // Perform operation
        aMaker.Perform();

        // Print Error/Warning messages
        BOPTestObjects.ReportAlerts(theDI, di, aMaker.GetReport());

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aMaker.History());

        if (aMaker.HasErrors)
        {
            return 0;
        }

        // Draw the result shape
        theDI.SetShape(theArgv[1], aMaker.Shape());
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

        IReadOnlyList<Shape> aTwins = [.. Maker(theDI).GetTwins(aShape)];
        if (aTwins.Count == 0)
        {
            di.Append("No twins for the shape.\n");
        }
        theDI.SetShape(theArgv[1], ListShape(aTwins));
        return 0;
    }

    private static int RepeatShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 4)
        {
            return 1;
        }

        MakePeriodicAlgorithm aMaker = Maker(theDI);
        for (int i = 2; i < theArgc; ++i)
        {
            int aDirID = DirectionID(theArgv[i]);
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

            aMaker.InternalRepeatShape(aDirID, aTimes);
        }

        // Print Error/Warning messages
        BOPTestObjects.ReportAlerts(theDI, di, aMaker.GetReport());

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aMaker.History());

        if (aMaker.HasErrors)
        {
            return 0;
        }

        // Draw the result shape
        theDI.SetShape(theArgv[1], aMaker.RepeatedShape());
        return 0;
    }

    private static int ClearRepetitions(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgv)
    {
        MakePeriodicAlgorithm aMaker = Maker(theDI);
        // Clear all previous repetitions
        aMaker.ClearRepetitions();

        // Set the history of the operation in session
        BRepTestHistoryCommands.SetHistory(theDI, aMaker.History());

        if (theArgv.Count > 1)
        {
            theDI.SetShape(theArgv[1], aMaker.Shape());
        }
        return 0;
    }
}
