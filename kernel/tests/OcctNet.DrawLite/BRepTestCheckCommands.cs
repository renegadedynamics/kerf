// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;
using Occt;
using static Occt.BRepCheck_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's checking commands (BRepTest_CheckCommands.cxx): checkshape (with its contextual and structural dumps),
/// checksection, computetolerance, and validrange (BRepLib::FindValidRange, unit 7W).
/// </summary>
/// <remarks>
/// checkdiff (BRepAlgo::IsValid, TKBool) isn't here: it writes SameParameter and tolerances into the result's
/// edges, so it waits for BRepAlgo and a copy-for-write of the result.
/// </remarks>
internal static class BRepTestCheckCommands
{
    // Number of BRepCheck_Statuses in BRepCheck_Status.hxx file (BRepCheck_NoError is not considered)
    private const int NumberOfStatus = 36;

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "checkshape", CheckShape);
        DrawCxx.Add(theInterpreter, "checksection", CheckSection);
        DrawCxx.Add(theInterpreter, "computetolerance", ComputeTolerance);
        DrawCxx.Add(theInterpreter, "validrange", ValidRange);
    }

    // validrange edge [(out) u1 u2]
    private static int ValidRange(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 2)
        {
            di.Append("usage: validrange edge [(out) u1 u2]");
            return 1;
        }

        TopoDS_Edge aE = TopoDS.Edge(theDI.GetShape(a[1], TopAbs_EDGE, true));
        if (aE.IsNull())
        {
            return 1;
        }

        double u1 = 0.0, u2 = 0.0;
        if (BRepLib.FindValidRange(aE, ref u1, ref u2))
        {
            if (narg > 3)
            {
                theDI.SetNumber(a[2], u1);
                theDI.SetNumber(a[3], u2);
            }
            else
            {
                di.Append(u1).Append(" ").Append(u2);
            }
        }
        else
        {
            di.Append("edge has no valid range");
        }
        return 0;
    }

    /// <summary>
    /// The state of one dump: Draw's file statics <c>theMap</c>, <c>nbfaulty</c> and <c>lfaulty</c>, and the prefix
    /// <c>checkfaultyname</c> ("faulty_", set when the commands are registered).
    /// </summary>
    private sealed class DumpState(DrawInterpreter theDI)
    {
        public readonly DrawInterpreter DI = theDI;

        // OCCT: NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
        public readonly Dictionary<TopoDS_Shape, List<TopoDS_Shape>> theMap = new(TopTools_ShapeMapHasher.Instance);

        public int nbfaulty;

        // OCCT keeps the drawables; their names and shapes are what FindNamed reads
        public readonly List<(string Name, TopoDS_Shape Shape)> lfaulty = [];

        public string checkfaultyname = "faulty_";
    }

    // -------------------------------------------------------------------------------------------------------------
    // checkshape

    private static bool FindNamed(DumpState theState, in TopoDS_Shape S, out string Name)
    {
        foreach ((string aName, TopoDS_Shape aShape) in theState.lfaulty)
        {
            if (aShape.IsSame(S))
            {
                Name = aName;
                return true;
            }
        }

        Name = "";
        return false;
    }

    private static bool Contains(List<TopoDS_Shape> L, in TopoDS_Shape S)
    {
        foreach (TopoDS_Shape aShape in L)
        {
            if (aShape.IsSame(S))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Names a faulty shape faulty_N and binds it (DBRep::Set), as Draw does.</summary>
    private static string NewFaulty(DumpState theState, in TopoDS_Shape S)
    {
        theState.nbfaulty++;
        string aName = theState.checkfaultyname + theState.nbfaulty.ToString(CultureInfo.InvariantCulture);
        theState.DI.SetShape(aName, S);
        theState.lfaulty.Add((aName, S));
        return aName;
    }

    private static void PrintSub(StringBuilder OS, DumpState theState, BRepCheck_Analyzer Ana, in TopoDS_Shape S, TopAbs_ShapeEnum Subtype)
    {
        for (var exp = new TopExp_Explorer(S, Subtype); exp.More(); exp.Next())
        {
            BRepCheck_Result res = Ana.Result(exp.Current())!;
            TopoDS_Shape sub = exp.Current();
            for (res.InitContextIterator(); res.MoreShapeInContext(); res.NextShapeInContext())
            {
                if (res.ContextualShape().IsSame(S) && !Contains(theState.theMap[sub], S))
                {
                    theState.theMap[sub].Add(S);
                    NCollection_List<BRepCheck_Status> aStatuses = res.StatusOnShape();
                    if (aStatuses.First() != BRepCheck_NoError)
                    {
                        if (!FindNamed(theState, sub, out string Name))
                        {
                            Name = NewFaulty(theState, sub);
                        }

                        OS.Append("Shape ").Append(Name).Append(' ');
                        if (!FindNamed(theState, S, out Name))
                        {
                            Name = NewFaulty(theState, S);
                        }

                        OS.Append(" on shape ").Append(Name).Append(" :\n");
                        PrintStatuses(OS, aStatuses, theSkipNoError: false);
                    }

                    break;
                }
            }
        }
    }

    private static void PrintStatuses(StringBuilder OS, NCollection_List<BRepCheck_Status> theStatuses, bool theSkipNoError)
    {
        using var aWriter = new StringWriter(OS, CultureInfo.InvariantCulture);
        foreach (BRepCheck_Status aStatus in theStatuses)
        {
            if (!theSkipNoError || aStatus != BRepCheck_NoError)
            {
                BRepCheck.Print(aStatus, aWriter);
            }
        }
    }

    private static void Print(StringBuilder OS, DumpState theState, BRepCheck_Analyzer Ana, in TopoDS_Shape S)
    {
        for (var iter = new TopoDS_Iterator(S); iter.More(); iter.Next())
        {
            Print(OS, theState, Ana, iter.Value());
        }

        TopAbs_ShapeEnum styp = S.ShapeType();
        if (Ana.Result(S) is { } aResult && !theState.theMap.ContainsKey(S))
        {
            NCollection_List<BRepCheck_Status> aStatuses = aResult.Status();
            if (aStatuses.First() != BRepCheck_NoError)
            {
                if (!FindNamed(theState, S, out string Name))
                {
                    Name = NewFaulty(theState, S);
                }

                OS.Append("On Shape ").Append(Name).Append(" :\n");
                PrintStatuses(OS, aStatuses, theSkipNoError: true);
            }
        }

        theState.theMap.TryAdd(S, []);

        switch (styp)
        {
            case TopAbs_EDGE:
                PrintSub(OS, theState, Ana, S, TopAbs_VERTEX);
                break;
            case TopAbs_WIRE:
                PrintSub(OS, theState, Ana, S, TopAbs_EDGE);
                PrintSub(OS, theState, Ana, S, TopAbs_VERTEX);
                break;
            case TopAbs_FACE:
                PrintSub(OS, theState, Ana, S, TopAbs_WIRE);
                PrintSub(OS, theState, Ana, S, TopAbs_EDGE);
                PrintSub(OS, theState, Ana, S, TopAbs_VERTEX);
                break;
            case TopAbs_SHELL:
                break;
            case TopAbs_SOLID:
                PrintSub(OS, theState, Ana, S, TopAbs_SHELL);
                break;
            default:
                break;
        }
    }

    /// <summary>Contextual (modeling) style of output.</summary>
    private static void ContextualDump(DrawInterpreter theDI, DrawResult theCommands, BRepCheck_Analyzer theAna, in TopoDS_Shape theShape)
    {
        var aState = new DumpState(theDI);
        var aSStream = new StringBuilder();
        Print(aSStream, aState, theAna, theShape);
        theCommands.Append(aSStream.ToString());

        theCommands.Append("\n");
        aState.theMap.Clear();

        if (aState.nbfaulty != 0)
        {
            theCommands.Append("Faulty shapes in variables ").Append(aState.checkfaultyname).Append("1 to ")
                       .Append(aState.checkfaultyname).Append(aState.nbfaulty).Append(" \n");
        }

        theCommands.Append("\n");
    }

    private static void FillProblems(BRepCheck_Status stat, int[] NbProblems)
    {
        int anID = (int)stat;
        if (anID < 1 || anID > NumberOfStatus)
        {
            return;
        }

        NbProblems[anID]++;
    }

    private static int IndexOfSame(List<TopoDS_Shape> sl, in TopoDS_Shape S)
    {
        for (int ii = 0; ii < sl.Count; ii++)
        {
            if (sl[ii].IsSame(S))
            {
                return ii;
            }
        }

        return -1;
    }

    private static void GetProblemSub(DumpState theState, BRepCheck_Analyzer Ana, in TopoDS_Shape Shape, List<TopoDS_Shape> sl, int[] NbProblems,
                                      TopAbs_ShapeEnum Subtype)
    {
        for (var exp = new TopExp_Explorer(Shape, Subtype); exp.More(); exp.Next())
        {
            BRepCheck_Result res = Ana.Result(exp.Current())!;
            TopoDS_Shape sub = exp.Current();
            for (res.InitContextIterator(); res.MoreShapeInContext(); res.NextShapeInContext())
            {
                if (res.ContextualShape().IsSame(Shape) && !Contains(theState.theMap[sub], Shape))
                {
                    theState.theMap[sub].Add(Shape);
                    BRepCheck_Status aFirst = res.StatusOnShape().First();
                    if (aFirst != BRepCheck_NoError)
                    {
                        if (IndexOfSame(sl, sub) < 0)
                        {
                            sl.Add(sub);
                            FillProblems(aFirst, NbProblems);
                        }

                        if (IndexOfSame(sl, Shape) < 0)
                        {
                            sl.Add(Shape);
                            FillProblems(aFirst, NbProblems);
                        }
                    }

                    break;
                }
            }
        }
    }

    private static void GetProblemShapes(DumpState theState, BRepCheck_Analyzer Ana, in TopoDS_Shape Shape, List<TopoDS_Shape> sl, int[] NbProblems)
    {
        for (var iter = new TopoDS_Iterator(Shape); iter.More(); iter.Next())
        {
            GetProblemShapes(theState, Ana, iter.Value(), sl, NbProblems);
        }

        TopAbs_ShapeEnum styp = Shape.ShapeType();
        if (Ana.Result(Shape) is { } aResult && !theState.theMap.ContainsKey(Shape))
        {
            BRepCheck_Status aFirst = aResult.Status().First();
            if (aFirst != BRepCheck_NoError)
            {
                sl.Add(Shape);
                FillProblems(aFirst, NbProblems);
            }
        }

        theState.theMap.TryAdd(Shape, []);

        switch (styp)
        {
            case TopAbs_EDGE:
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, TopAbs_VERTEX);
                break;
            case TopAbs_FACE:
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, TopAbs_WIRE);
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, TopAbs_EDGE);
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, TopAbs_VERTEX);
                break;
            case TopAbs_SHELL:
                break;
            case TopAbs_SOLID:
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, TopAbs_SHELL);
                break;
            default:
                break;
        }
    }

    // The lines of StructuralDump's table, in its order
    private static readonly (BRepCheck_Status Status, string Line)[] s_problemLines =
    [
        (BRepCheck_InvalidPointOnCurve, "  Invalid Point on Curve ................... "),
        (BRepCheck_InvalidPointOnCurveOnSurface, "  Invalid Point on CurveOnSurface .......... "),
        (BRepCheck_InvalidPointOnSurface, "  Invalid Point on Surface ................. "),
        (BRepCheck_No3DCurve, "  No 3D Curve .............................. "),
        (BRepCheck_Multiple3DCurve, "  Multiple 3D Curve ........................ "),
        (BRepCheck_Invalid3DCurve, "  Invalid 3D Curve ......................... "),
        (BRepCheck_NoCurveOnSurface, "  No Curve on Surface ...................... "),
        (BRepCheck_InvalidCurveOnSurface, "  Invalid Curve on Surface ................. "),
        (BRepCheck_InvalidCurveOnClosedSurface, "  Invalid Curve on closed Surface .......... "),
        (BRepCheck_InvalidSameRangeFlag, "  Invalid SameRange Flag ................... "),
        (BRepCheck_InvalidSameParameterFlag, "  Invalid SameParameter Flag ............... "),
        (BRepCheck_InvalidDegeneratedFlag, "  Invalid Degenerated Flag ................. "),
        (BRepCheck_FreeEdge, "  Free Edge ................................ "),
        (BRepCheck_InvalidMultiConnexity, "  Invalid MultiConnexity ................... "),
        (BRepCheck_InvalidRange, "  Invalid Range ............................ "),
        (BRepCheck_EmptyWire, "  Empty Wire ............................... "),
        (BRepCheck_RedundantEdge, "  Redundant Edge ........................... "),
        (BRepCheck_SelfIntersectingWire, "  Self Intersecting Wire ................... "),
        (BRepCheck_NoSurface, "  No Surface ............................... "),
        (BRepCheck_InvalidWire, "  Invalid Wire ............................. "),
        (BRepCheck_RedundantWire, "  Redundant Wire ........................... "),
        (BRepCheck_IntersectingWires, "  Intersecting Wires ....................... "),
        (BRepCheck_InvalidImbricationOfWires, "  Invalid Imbrication of Wires ............. "),
        (BRepCheck_EmptyShell, "  Empty Shell .............................. "),
        (BRepCheck_RedundantFace, "  Redundant Face ........................... "),
        (BRepCheck_UnorientableShape, "  Unorientable Shape ....................... "),
        (BRepCheck_NotClosed, "  Not Closed ............................... "),
        (BRepCheck_NotConnected, "  Not Connected ............................ "),
        (BRepCheck_SubshapeNotInShape, "  Subshape not in Shape .................... "),
        (BRepCheck_BadOrientation, "  Bad Orientation .......................... "),
        (BRepCheck_BadOrientationOfSubshape, "  Bad Orientation of Subshape .............. "),
        (BRepCheck_InvalidToleranceValue, "  Invalid tolerance value................... "),
        (BRepCheck_InvalidPolygonOnTriangulation, "  Invalid polygon on triangulation.......... "),
        (BRepCheck_InvalidImbricationOfShells, "  Invalid Imbrication of Shells............. "),
        (BRepCheck_EnclosedRegion, "  Enclosed Region........................... "),
        (BRepCheck_CheckFail, "  checkshape failure........................ "),
    ];

    /// <summary>Structural (data exchange) style of output.</summary>
    private static void StructuralDump(DrawInterpreter theDI, DrawResult theCommands, BRepCheck_Analyzer theAna, string ShName, string Pref,
                                       in TopoDS_Shape theShape)
    {
        theCommands.Append(" -- The Shape ").Append(ShName).Append(" has problems :\n");
        theCommands.Append("  Check                                    Count\n");
        theCommands.Append(" ------------------------------------------------\n");

        // OCCT: NCollection_HArray1<int>(1, NumberOfStatus), zeroed
        int[] NbProblems = new int[NumberOfStatus + 1];
        var sl = new List<TopoDS_Shape>();
        var aState = new DumpState(theDI);
        GetProblemShapes(aState, theAna, theShape, sl, NbProblems);
        aState.theMap.Clear();

        foreach ((BRepCheck_Status aStatus, string aLine) in s_problemLines)
        {
            int aCount = NbProblems[(int)aStatus];
            if (aCount > 0)
            {
                theCommands.Append(aLine).Append(aCount).Append("\n");
            }
        }

        theCommands.Append(" ------------------------------------------------\n");
        theCommands.Append("*** Shapes with problems : ").Append(sl.Count).Append("\n");

        var aByType = new (TopAbs_ShapeEnum Type, string Label, string Suffix)[]
        {
            (TopAbs_VERTEX, "VERTEX", "_v"), (TopAbs_EDGE, "EDGE", "_e"), (TopAbs_WIRE, "WIRE", "_w"), (TopAbs_FACE, "FACE", "_f"),
            (TopAbs_SHELL, "SHELL", "_s"), (TopAbs_SOLID, "SOLID", "_o"),
        };
        var B = new BRep_Builder();
        foreach ((TopAbs_ShapeEnum aType, string aLabel, string aSuffix) in aByType)
        {
            List<TopoDS_Shape> aShapes = sl.Where(s => s.ShapeType() == aType).ToList();
            if (aShapes.Count == 0)
            {
                continue;
            }

            TopoDS_Compound comp = new();
            B.MakeCompound(ref comp);
            foreach (TopoDS_Shape aShape in aShapes)
            {
                B.Add(comp, aShape);
            }

            // OCCT bug: writes "%s_v" into char aName[20]: a prefix of 18 characters or more overflows it (occt-issues.md)
            string aName = Pref + aSuffix;
            theDI.SetShape(aName, comp);
            int nb = aShapes.Count;
            theCommands.Append(aLabel).Append(nb > 9 ? "\t: " : "\t:  ").Append(nb).Append(" Items -> compound named ").Append(aName).Append("\n");
        }
    }

    private static int CheckShape(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg == 1)
        {
            theCommands.Append("\n");
            theCommands.Append("Usage : checkshape [-top] shape [result] [-short] [-parallel] [-exact]\n");
            theCommands.Append("\n");
            theCommands.Append("Where :\n");
            theCommands.Append("   -top      - check topology only.\n");
            theCommands.Append("   shape     - the name of the shape to test.\n");
            theCommands.Append("   result    - the prefix of the output shape names. If it is used, structural\n");
            theCommands.Append("               output style will be used. Otherwise - contextual one.\n");
            theCommands.Append("   -short    - short description of check.\n");
            theCommands.Append("   -parallel - run check in parallel.\n");
            theCommands.Append("   -exact    - run check using exact method.\n");
            return 0;
        }

        if (narg > 7)
        {
            theCommands.Append("Invalid number of args!!!\n");
            theCommands.Append("No args to have help.\n");
            return 1;
        }

        bool aGeomCtrl = true;
        int aCurInd = 1;
        if (a[1] == "-top")
        {
            aGeomCtrl = false;
            aCurInd++;
        }

        if (aCurInd > narg - 1)
        {
            theCommands.Append("Invalid number of args!!!\n");
            theCommands.Append("No args to have help.\n");
            return 1;
        }

        string aShapeName = a[aCurInd];
        TopoDS_Shape aShape = theDI.GetShape(aShapeName);
        if (aShape.IsNull())
        {
            theCommands.Append(a[aCurInd]).Append(" is not a topological shape!!!\n");
            return 1;
        }

        aCurInd++;

        bool IsShortDump = false;
        bool IsContextDump = true;
        bool IsParallel = false;
        bool IsExact = false;
        string? aPref = null;
        if (aCurInd < narg && !a[aCurInd].StartsWith('-'))
        {
            IsContextDump = false;
            aPref = a[aCurInd];
            aCurInd++;
        }

        for (int anAI = aCurInd; anAI < narg; anAI++)
        {
            string anArg = a[anAI].ToLowerInvariant();
            if (anArg == "-short")
            {
                IsShortDump = true;
            }
            else if (anArg == "-parallel")
            {
                IsParallel = true;
            }
            else if (anArg == "-exact")
            {
                IsExact = true;
            }
            else
            {
                theCommands.Append("Syntax error at '").Append(anArg).Append("'");
                return 1;
            }
        }

        try
        {
            var anAna = new BRepCheck_Analyzer(aShape, aGeomCtrl, IsParallel, IsExact);
            bool isValid = anAna.IsValid();

            if (isValid)
            {
                if (IsContextDump)
                {
                    theCommands.Append("This shape seems to be valid");
                }
                else
                {
                    theCommands.Append(" -- The Shape ").Append(aShapeName).Append(" looks OK");
                }
            }
            else
            {
                if (IsShortDump)
                {
                    theCommands.Append("This shape has faulty shapes");
                }
                else if (IsContextDump)
                {
                    ContextualDump(theDI, theCommands, anAna, aShape);
                }
                else
                {
                    StructuralDump(theDI, theCommands, anAna, aShapeName, aPref!, aShape);
                }
            }
        }
        catch (Exception anException) when (IsCaughtByOcctSignals(anException))
        {
            // OCCT prints what(): the message. The port's messages are its own, not OCCT's
            theCommands.Append("checkshape exception : ");
            theCommands.Append(anException is FailureException aFailure ? aFailure.GetMessageString() : global::Kerf.Engine.Foundation.OcctTypeNames.Of(anException.GetType()));
            theCommands.Append("\n");
            return 1;
        }

        return 0;
    }

    /// <summary>
    /// What OCC_CATCH_SIGNALS and <c>catch (Standard_Failure)</c> catch: OCCT's exceptions and the .NET exceptions that
    /// stand for the signals OCCT turns into failures (as BRepCheck_Analyzer, decision 432).
    /// </summary>
    internal static bool IsCaughtByOcctSignals(Exception theException)
        => theException is FailureException or NullReferenceException or InvalidCastException or IndexOutOfRangeException or DivideByZeroException;

    // -------------------------------------------------------------------------------------------------------------
    // checksection

    private static int CheckSection(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 2)
        {
            di.Append(a[0]).Append(" shape [-r <ref_val>]\n");
            return 1;
        }

        int aCompareValue = -1;
        TopoDS_Shape S = theDI.GetShape(a[1]);

        for (int anAI = 2; anAI < narg; anAI++)
        {
            if (a[anAI] == "-r")
            {
                // OCCT bug: reads a[++anAI] even past the last argument: Draw::Atoi of a null pointer (occt-issues.md)
                aCompareValue = ++anAI < narg ? theDI.Atoi(a[anAI]) : 0;
            }
            else
            {
                di.Append("Error: Wrong option").Append(a[anAI]).Append("\n");
            }
        }

        // OCCT adds a vertex met an odd number of times and removes it when met again; its map's order follows the
        // TShapes' addresses. The port keeps the vertices met an odd number of times in the order they were first met.
        var aParity = new Dictionary<TopoDS_Shape, bool>(TopTools_ShapeMapHasher.Instance);
        var anOrder = new List<TopoDS_Shape>();
        for (var exp = new TopExp_Explorer(S, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            if (aParity.TryGetValue(exp.Current(), out bool anIsIn))
            {
                aParity[exp.Current()] = !anIsIn;
            }
            else
            {
                aParity.Add(exp.Current(), true);
                anOrder.Add(exp.Current());
            }
        }

        List<TopoDS_Shape> theVertices = anOrder.Where(v => aParity[v]).ToList();
        di.Append(" nb alone Vertices : ").Append(theVertices.Count).Append("\n");

        if (aCompareValue >= 0)
        {
            if (theVertices.Count == aCompareValue)
            {
                di.Append("Section is OK\n");
            }
            else
            {
                di.Append("Error: ").Append(aCompareValue).Append(" vertices are expected but ").Append(theVertices.Count).Append(" are found.\n");
            }
        }

        int ipp = 0;
        foreach (TopoDS_Shape aVertex in theVertices)
        {
            ipp++;
            string Name = "alone_" + ipp.ToString(CultureInfo.InvariantCulture);
            theDI.SetShape(Name, aVertex);
            di.Append(Name).Append(" ");
        }

        di.Append("\n");
        return 0;
    }

    // -------------------------------------------------------------------------------------------------------------
    // computetolerance

    private static int ComputeTolerance(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            di.Append("Usage: computetolerance shape\n");
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[1]);
        double tol;
        if (S.ShapeType() == TopAbs_EDGE)
        {
            var bce = new BRepCheck_Edge(TopoDS.Edge(S));
            tol = bce.Tolerance();
            di.Append($"Tolerance de {Address(S)} : {StreamUtilities.FormatReal(tol, 6)}\n");
        }
        else
        {
            var theEdges = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            for (var exp = new TopExp_Explorer(S, TopAbs_EDGE); exp.More(); exp.Next())
            {
                if (theEdges.Add(exp.Current()))
                {
                    var bce = new BRepCheck_Edge(TopoDS.Edge(exp.Current()));
                    tol = bce.Tolerance();
                    di.Append($"Tolerance de {Address(exp.Current())} : {StreamUtilities.FormatReal(tol, 6)}\n");
                }
            }

            di.Append("\n");
        }

        return 0;
    }

    /// <summary>
    /// <c>(void*)&amp;(*S.TShape())</c> as MSVC's stream writes a pointer (16 upper-case hex digits). The port has no
    /// addresses: it writes a number that stays the same for one TShape during the run.
    /// </summary>
    private static string Address(in TopoDS_Shape S)
        => ((long)RuntimeHelpers.GetHashCode(S.TShape()!)).ToString("X16", CultureInfo.InvariantCulture);
}
