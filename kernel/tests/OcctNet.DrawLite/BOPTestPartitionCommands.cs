// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using System.Diagnostics;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The builders of BOPTest_PartitionCommands.cxx over the session's pave filler: <c>bbuild</c>, <c>bbop</c>,
/// <c>bsplit</c> and <c>buildbop</c> (<c>bfillds</c> is in <see cref="BOPTestObjCommands"/>, unit 6G), unit 6L.
/// </summary>
/// <remarks>
/// Draw-lite has no help texts, so PrintHelp's usage errors return 1 with an empty result. Draw_ProgressIndicator goes.
/// </remarks>
internal static class BOPTestPartitionCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bbuild", BBuild);
        DrawCxx.Add(theInterpreter, "bbop", BBop);
        DrawCxx.Add(theInterpreter, "bsplit", BSplit);
        DrawCxx.Add(theInterpreter, "buildbop", BuildBop);
    }

    /// <summary>BOPTest::GetOperationType: the operation named by a number 0 to 4 or a word.</summary>
    internal static Operation GetOperationType(string theOp)
    {
        string anOp = theOp.ToLowerInvariant();

        if (IsIntegerValue(anOp, out long iOp))
        {
            // Check if the given value satisfies the enumeration.
            if (iOp >= 0 && iOp <= 4)
            {
                return (Operation)iOp;
            }
            return Operation.Unknown;
        }

        // Check for the meaningful symbolic operation parameter
        return anOp switch
        {
            "common" => Operation.Common,
            "fuse" => Operation.Fuse,
            "cut" => Operation.Cut,
            "tuc" or "cut21" => Operation.CUT21,
            "section" => Operation.Section,
            _ => Operation.Unknown,
        };
    }

    /// <summary>
    /// TCollection_AsciiString::IsIntegerValue and IntegerValue: strtol reads a prefix (leading blanks, a sign,
    /// digits), and the text is an integer when the prefix isn't empty and no '.' follows it.
    /// </summary>
    private static bool IsIntegerValue(string theText, out long theValue)
    {
        int i = 0;
        while (i < theText.Length && char.IsWhiteSpace(theText[i]))
        {
            ++i;
        }
        int aStart = i;
        if (i < theText.Length && (theText[i] == '+' || theText[i] == '-'))
        {
            ++i;
        }
        int aDigits = i;
        while (i < theText.Length && char.IsAsciiDigit(theText[i]))
        {
            ++i;
        }
        theValue = 0;
        if (i == aDigits)
        {
            return false;
        }
        if (!long.TryParse(theText.AsSpan(aStart, i - aStart), System.Globalization.NumberStyles.AllowLeadingSign,
                           System.Globalization.CultureInfo.InvariantCulture, out theValue))
        {
            theValue = theText[aStart] == '-' ? long.MinValue : long.MaxValue;
        }
        return theText.IndexOf('.', i) < 0;
    }

    private static string Tps(Stopwatch theTimer) => $"  Tps: {theTimer.Elapsed.TotalSeconds,7:F2}\n";

    /// <summary><c>bbuild result [-t]</c>: the General Fuse of the objects and tools, on <c>bfillds</c>' filler.</summary>
    private static int BBuild(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2 || n > 3)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        DS? pDS = aSession.PDS();
        if (pDS is null)
        {
            di.Append("Prepare PaveFiller first\n");
            return 0;
        }

        PaveFiller aPF = aSession.PaveFiller();
        aSession.SetBuilderDefault();
        BooleanBuilder aBuilder = aSession.Builder();
        aBuilder.Clear();
        foreach (Shape aS in aSession.Shapes)
        {
            aBuilder.AddArgument(aS);
        }
        foreach (Shape aS in aSession.Tools)
        {
            aBuilder.AddArgument(aS);
        }
        bool bShowTime = false;
        bool bRunParallel = aSession.RunParallel;
        if (n == 3)
        {
            if (a[2] == "-t")
            {
                bShowTime = true;
            }
            else
            {
                di.Append("Warning: invalid key\n");
            }
        }
        aBuilder.SetRunParallel(bRunParallel);
        aBuilder.SetCheckInverted(aSession.CheckInverted);
        aBuilder.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        Stopwatch aTimer = Stopwatch.StartNew();
        aBuilder.PerformWithFiller(aPF);
        BOPTestObjects.ReportAlerts(theDI, di, aBuilder.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aPF.Arguments(), new BOPTestBuilderHistory(aBuilder));
        }

        if (aBuilder.HasErrors)
        {
            return 0;
        }
        aTimer.Stop();
        if (bShowTime)
        {
            di.Append(Tps(aTimer));
        }
        Shape aR = aBuilder.Shape();
        if (aR.IsNull)
        {
            di.Append("Result is a null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary><c>bbop result op [-t]</c>: a Boolean or Section of the objects and tools, on <c>bfillds</c>' filler.</summary>
    private static int BBop(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3 || n > 4)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        DS? pDS = aSession.PDS();
        if (pDS is null)
        {
            di.Append("Prepare PaveFiller first\n");
            return 0;
        }
        Operation anOp = GetOperationType(a[2]);
        if (anOp == Operation.Unknown)
        {
            di.Append("Invalid operation type\n");
            return 0;
        }

        bool bShowTime = false;
        bool bRunParallel = aSession.RunParallel;
        if (n == 4)
        {
            if (a[3] == "-t")
            {
                bShowTime = true;
            }
            else
            {
                di.Append("Warning: invalid key\n");
            }
        }
        PaveFiller aPF = aSession.PaveFiller();
        BooleanBuilder pBuilder;

        if (anOp != Operation.Section)
        {
            pBuilder = aSession.BOP();
        }
        else
        {
            pBuilder = aSession.Section();
        }
        pBuilder.Clear();
        foreach (Shape aS in aSession.Shapes)
        {
            pBuilder.AddArgument(aS);
        }
        if (anOp != Operation.Section)
        {
            Bop pBOP = (Bop)pBuilder;
            foreach (Shape aS in aSession.Tools)
            {
                pBOP.AddTool(aS);
            }
            pBOP.SetOperation(anOp);
        }
        else
        {
            foreach (Shape aS in aSession.Tools)
            {
                pBuilder.AddArgument(aS);
            }
        }
        pBuilder.SetRunParallel(bRunParallel);
        pBuilder.SetCheckInverted(aSession.CheckInverted);
        pBuilder.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        Stopwatch aTimer = Stopwatch.StartNew();
        pBuilder.PerformWithFiller(aPF);
        BOPTestObjects.ReportAlerts(theDI, di, pBuilder.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aPF.Arguments(), new BOPTestBuilderHistory(pBuilder));
        }

        if (pBuilder.HasErrors)
        {
            return 0;
        }
        aTimer.Stop();
        if (bShowTime)
        {
            di.Append(Tps(aTimer));
        }
        Shape aR = pBuilder.Shape();
        if (aR.IsNull)
        {
            di.Append("Result is a null shape\n");
            return 0;
        }
        aSession.SetBuilder(pBuilder);
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary><c>bsplit result [-t]</c>: the objects split by the tools, on <c>bfillds</c>' filler.</summary>
    private static int BSplit(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2 || n > 3)
        {
            return 1;
        }
        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        DS? pDS = aSession.PDS();
        if (pDS is null)
        {
            di.Append("Prepare PaveFiller first\n");
            return 0;
        }
        PaveFiller aPF = aSession.PaveFiller();
        Splitter pSplitter = aSession.Splitter();
        pSplitter.Clear();
        pSplitter.SetArguments(aSession.Shapes);
        pSplitter.SetTools(aSession.Tools);
        pSplitter.SetRunParallel(aSession.RunParallel);
        pSplitter.SetNonDestructive(aSession.NonDestructive);
        pSplitter.SetFuzzyValue(aSession.FuzzyValue);
        pSplitter.SetCheckInverted(aSession.CheckInverted);
        pSplitter.SetToFillHistory(BRepTestHistoryCommands.IsHistoryNeeded(theDI));
        Stopwatch aTimer = Stopwatch.StartNew();
        pSplitter.PerformWithFiller(aPF);
        aTimer.Stop();
        BOPTestObjects.ReportAlerts(theDI, di, pSplitter.GetReport());

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, aPF.Arguments(), new BOPTestBuilderHistory(pSplitter));
        }

        if (pSplitter.HasErrors)
        {
            return 0;
        }
        if (n == 3)
        {
            if (a[2] == "-t")
            {
                di.Append(Tps(aTimer));
            }
            else
            {
                di.Append("Warning: invalid key\n");
            }
        }
        aSession.SetBuilder(pSplitter);
        Shape aR = pSplitter.Shape();
        if (aR.IsNull)
        {
            di.Append(" null shape\n");
            return 0;
        }
        theDI.SetShape(a[1], aR);
        return 0;
    }

    /// <summary>
    /// <c>buildbop result -o s1 [s2 ...] -t s3 [s4 ...] -op operation</c>: a Boolean built by classification from the
    /// splits of the current builder (BOPAlgo_Builder::BuildBOP).
    /// </summary>
    private static int BuildBop(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        BOPTestObjects aSession = BOPTestObjects.Of(theDI);
        DS? pDS = aSession.PDS();
        if (pDS is null)
        {
            di.Append("Error: perform intersection of arguments first");
            return 1;
        }

        BooleanBuilder pBuilder = aSession.Builder();
        if (pBuilder.HasErrors)
        {
            di.Append("Error: there were problems during GF");
            return 0;
        }

        if (pBuilder.Arguments().IsEmpty() || pBuilder.InternalShape().IsNull)
        {
            di.Append("Error: it seems the GF has not been yet performed");
            return 1;
        }

        NCollection_List<Shape> aLObjects = new(), aLTools = new();
        Operation anOp = Operation.Unknown;

        for (int i = 2; i < n; ++i)
        {
            if (a[i] == "-o" || a[i] == "-t")
            {
                if (i == (n - 1))
                {
                    di.Append("Error: shapes are expected after the key ").Append(a[i]);
                    return 1;
                }

                NCollection_List<Shape> aList = a[i] == "-o" ? aLObjects : aLTools;
                int j = i + 1;
                for (; j < n; ++j)
                {
                    if (a[j].StartsWith('-'))
                    {
                        i = j - 1;
                        break;
                    }
                    else
                    {
                        Shape aS = theDI.GetShape(a[j]);
                        if (aS.IsNull)
                        {
                            di.Append("Error: ").Append(a[j]).Append(" is a null shape");
                            return 1;
                        }
                        if (aS.ShapeType() != ShapeType.Solid)
                        {
                            di.Append("Error: ").Append(a[j]).Append(" is not a solid");
                            return 1;
                        }
                        if (pDS.Index(aS) < 0)
                        {
                            di.Append("Error: ").Append(a[j]).Append(" is not an argument of GF");
                            return 1;
                        }
                        aList.Append(aS);
                    }
                }
                if (j == n)
                {
                    break;
                }
            }
            else if (a[i] == "-op")
            {
                if (i == (n - 1))
                {
                    di.Append("Error: operation type is expected after the key ").Append(a[i]);
                    return 1;
                }

                ++i;
                if (string.Equals(a[i], "common", StringComparison.OrdinalIgnoreCase))
                {
                    anOp = Operation.Common;
                }
                else if (string.Equals(a[i], "fuse", StringComparison.OrdinalIgnoreCase))
                {
                    anOp = Operation.Fuse;
                }
                else if (string.Equals(a[i], "cut", StringComparison.OrdinalIgnoreCase))
                {
                    anOp = Operation.Cut;
                }
                else if (string.Equals(a[i], "tuc", StringComparison.OrdinalIgnoreCase))
                {
                    anOp = Operation.CUT21;
                }
                else
                {
                    di.Append("Error: unknown operation type");
                    return 1;
                }
            }
            else
            {
                di.Append("Error: ").Append(a[i]).Append(" invalid key");
                return 1;
            }
        }

        if (anOp == Operation.Unknown)
        {
            di.Append("Error: operation has not been specified");
            return 1;
        }

        bool hasObjects = !aLObjects.IsEmpty();
        bool hasTools = !aLTools.IsEmpty();
        if (!hasObjects && !hasTools)
        {
            di.Append("Error: no shapes are given");
            return 1;
        }

        Report aReport = new();
        pBuilder.BuildBOP(aLObjects, aLTools, anOp, default, aReport);

        BOPTestObjects.ReportAlerts(theDI, di, aReport);

        if (!aReport.GetAlerts(Gravity.Fail).IsEmpty())
        {
            return 0;
        }

        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            BRepTestHistoryCommands.SetHistory(theDI, pDS.Arguments(), new BOPTestBuilderHistory(pBuilder));
        }

        Shape aR = pBuilder.Shape();
        theDI.SetShape(a[1], aR);

        return 0;
    }
}
