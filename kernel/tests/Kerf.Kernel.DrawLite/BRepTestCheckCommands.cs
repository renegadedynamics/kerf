// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;
using Kerf.Engine;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite;

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

        Edge aE = ShapeCasts.Edge(theDI.GetShape(a[1], ShapeType.Edge, true));
        if (aE.IsNull)
        {
            return 1;
        }

        double u1 = 0.0, u2 = 0.0;
        if (BrepLibrary.FindValidRange(aE, ref u1, ref u2))
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
        public readonly Dictionary<Shape, List<Shape>> theMap = new(ShapeHasher.Instance);

        public int nbfaulty;

        // OCCT keeps the drawables; their names and shapes are what FindNamed reads
        public readonly List<(string Name, Shape Shape)> lfaulty = [];

        public string checkfaultyname = "faulty_";
    }

    // -------------------------------------------------------------------------------------------------------------
    // checkshape

    private static bool FindNamed(DumpState theState, in Shape S, out string Name)
    {
        foreach ((string aName, Shape aShape) in theState.lfaulty)
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

    private static bool Contains(List<Shape> L, in Shape S)
    {
        foreach (Shape aShape in L)
        {
            if (aShape.IsSame(S))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Names a faulty shape faulty_N and binds it (DBRep::Set), as Draw does.</summary>
    private static string NewFaulty(DumpState theState, in Shape S)
    {
        theState.nbfaulty++;
        string aName = theState.checkfaultyname + theState.nbfaulty.ToString(CultureInfo.InvariantCulture);
        theState.DI.SetShape(aName, S);
        theState.lfaulty.Add((aName, S));
        return aName;
    }

    private static void PrintSub(StringBuilder OS, DumpState theState, Analyzer Ana, in Shape S, ShapeType Subtype)
    {
        for (var exp = new Explorer(S, Subtype); exp.More(); exp.Next())
        {
            ResultCheck res = Ana.Result(exp.Current())!;
            Shape sub = exp.Current();
            for (res.InitContextIterator(); res.MoreShapeInContext(); res.NextShapeInContext())
            {
                if (res.ContextualShape().IsSame(S) && !Contains(theState.theMap[sub], S))
                {
                    theState.theMap[sub].Add(S);
                    NCollection_List<StatusCheck> aStatuses = res.StatusOnShape();
                    if (aStatuses.First() != StatusCheck.NoError)
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

    private static void PrintStatuses(StringBuilder OS, NCollection_List<StatusCheck> theStatuses, bool theSkipNoError)
    {
        using var aWriter = new StringWriter(OS, CultureInfo.InvariantCulture);
        foreach (StatusCheck aStatus in theStatuses)
        {
            if (!theSkipNoError || aStatus != StatusCheck.NoError)
            {
                BRepCheckCheck.Print(aStatus, aWriter);
            }
        }
    }

    private static void Print(StringBuilder OS, DumpState theState, Analyzer Ana, in Shape S)
    {
        for (var iter = new Iterator(S); iter.More(); iter.Next())
        {
            Print(OS, theState, Ana, iter.Value());
        }

        ShapeType styp = S.Type;
        if (Ana.Result(S) is { } aResult && !theState.theMap.ContainsKey(S))
        {
            NCollection_List<StatusCheck> aStatuses = aResult.Status();
            if (aStatuses.First() != StatusCheck.NoError)
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
            case ShapeType.Edge:
                PrintSub(OS, theState, Ana, S, ShapeType.Vertex);
                break;
            case ShapeType.Wire:
                PrintSub(OS, theState, Ana, S, ShapeType.Edge);
                PrintSub(OS, theState, Ana, S, ShapeType.Vertex);
                break;
            case ShapeType.Face:
                PrintSub(OS, theState, Ana, S, ShapeType.Wire);
                PrintSub(OS, theState, Ana, S, ShapeType.Edge);
                PrintSub(OS, theState, Ana, S, ShapeType.Vertex);
                break;
            case ShapeType.Shell:
                break;
            case ShapeType.Solid:
                PrintSub(OS, theState, Ana, S, ShapeType.Shell);
                break;
            default:
                break;
        }
    }

    /// <summary>Contextual (modeling) style of output.</summary>
    private static void ContextualDump(DrawInterpreter theDI, DrawResult theCommands, Analyzer theAna, in Shape theShape)
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

    private static void FillProblems(StatusCheck stat, int[] NbProblems)
    {
        int anID = (int)stat;
        if (anID < 1 || anID > NumberOfStatus)
        {
            return;
        }

        NbProblems[anID]++;
    }

    private static int IndexOfSame(List<Shape> sl, in Shape S)
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

    private static void GetProblemSub(DumpState theState, Analyzer Ana, in Shape Shape, List<Shape> sl, int[] NbProblems,
                                      ShapeType Subtype)
    {
        for (var exp = new Explorer(Shape, Subtype); exp.More(); exp.Next())
        {
            ResultCheck res = Ana.Result(exp.Current())!;
            Shape sub = exp.Current();
            for (res.InitContextIterator(); res.MoreShapeInContext(); res.NextShapeInContext())
            {
                if (res.ContextualShape().IsSame(Shape) && !Contains(theState.theMap[sub], Shape))
                {
                    theState.theMap[sub].Add(Shape);
                    StatusCheck aFirst = res.StatusOnShape().First();
                    if (aFirst != StatusCheck.NoError)
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

    private static void GetProblemShapes(DumpState theState, Analyzer Ana, in Shape Shape, List<Shape> sl, int[] NbProblems)
    {
        for (var iter = new Iterator(Shape); iter.More(); iter.Next())
        {
            GetProblemShapes(theState, Ana, iter.Value(), sl, NbProblems);
        }

        ShapeType styp = Shape.Type;
        if (Ana.Result(Shape) is { } aResult && !theState.theMap.ContainsKey(Shape))
        {
            StatusCheck aFirst = aResult.Status().First();
            if (aFirst != StatusCheck.NoError)
            {
                sl.Add(Shape);
                FillProblems(aFirst, NbProblems);
            }
        }

        theState.theMap.TryAdd(Shape, []);

        switch (styp)
        {
            case ShapeType.Edge:
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, ShapeType.Vertex);
                break;
            case ShapeType.Face:
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, ShapeType.Wire);
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, ShapeType.Edge);
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, ShapeType.Vertex);
                break;
            case ShapeType.Shell:
                break;
            case ShapeType.Solid:
                GetProblemSub(theState, Ana, Shape, sl, NbProblems, ShapeType.Shell);
                break;
            default:
                break;
        }
    }

    // The lines of StructuralDump's table, in its order
    private static readonly (StatusCheck Status, string Line)[] s_problemLines =
    [
        (StatusCheck.InvalidPointOnCurve, "  Invalid Point on Curve ................... "),
        (StatusCheck.InvalidPointOnCurveOnSurface, "  Invalid Point on CurveOnSurface .......... "),
        (StatusCheck.InvalidPointOnSurface, "  Invalid Point on Surface ................. "),
        (StatusCheck.No3DCurve, "  No 3D Curve .............................. "),
        (StatusCheck.Multiple3DCurve, "  Multiple 3D Curve ........................ "),
        (StatusCheck.Invalid3DCurve, "  Invalid 3D Curve ......................... "),
        (StatusCheck.NoCurveOnSurface, "  No Curve on Surface ...................... "),
        (StatusCheck.InvalidCurveOnSurface, "  Invalid Curve on Surface ................. "),
        (StatusCheck.InvalidCurveOnClosedSurface, "  Invalid Curve on closed Surface .......... "),
        (StatusCheck.InvalidSameRangeFlag, "  Invalid SameRange Flag ................... "),
        (StatusCheck.InvalidSameParameterFlag, "  Invalid SameParameter Flag ............... "),
        (StatusCheck.InvalidDegeneratedFlag, "  Invalid Degenerated Flag ................. "),
        (StatusCheck.FreeEdge, "  Free Edge ................................ "),
        (StatusCheck.InvalidMultiConnexity, "  Invalid MultiConnexity ................... "),
        (StatusCheck.InvalidRange, "  Invalid Range ............................ "),
        (StatusCheck.EmptyWire, "  Empty Wire ............................... "),
        (StatusCheck.RedundantEdge, "  Redundant Edge ........................... "),
        (StatusCheck.SelfIntersectingWire, "  Self Intersecting Wire ................... "),
        (StatusCheck.NoSurface, "  No Surface ............................... "),
        (StatusCheck.InvalidWire, "  Invalid Wire ............................. "),
        (StatusCheck.RedundantWire, "  Redundant Wire ........................... "),
        (StatusCheck.IntersectingWires, "  Intersecting Wires ....................... "),
        (StatusCheck.InvalidImbricationOfWires, "  Invalid Imbrication of Wires ............. "),
        (StatusCheck.EmptyShell, "  Empty Shell .............................. "),
        (StatusCheck.RedundantFace, "  Redundant Face ........................... "),
        (StatusCheck.UnorientableShape, "  Unorientable Shape ....................... "),
        (StatusCheck.NotClosed, "  Not Closed ............................... "),
        (StatusCheck.NotConnected, "  Not Connected ............................ "),
        (StatusCheck.SubshapeNotInShape, "  Subshape not in Shape .................... "),
        (StatusCheck.BadOrientation, "  Bad Orientation .......................... "),
        (StatusCheck.BadOrientationOfSubshape, "  Bad Orientation of Subshape .............. "),
        (StatusCheck.InvalidToleranceValue, "  Invalid tolerance value................... "),
        (StatusCheck.InvalidPolygonOnTriangulation, "  Invalid polygon on triangulation.......... "),
        (StatusCheck.InvalidImbricationOfShells, "  Invalid Imbrication of Shells............. "),
        (StatusCheck.EnclosedRegion, "  Enclosed Region........................... "),
        (StatusCheck.CheckFail, "  checkshape failure........................ "),
    ];

    /// <summary>Structural (data exchange) style of output.</summary>
    private static void StructuralDump(DrawInterpreter theDI, DrawResult theCommands, Analyzer theAna, string ShName, string Pref,
                                       in Shape theShape)
    {
        theCommands.Append(" -- The Shape ").Append(ShName).Append(" has problems :\n");
        theCommands.Append("  Check                                    Count\n");
        theCommands.Append(" ------------------------------------------------\n");

        // OCCT: NCollection_HArray1<int>(1, NumberOfStatus), zeroed
        int[] NbProblems = new int[NumberOfStatus + 1];
        var sl = new List<Shape>();
        var aState = new DumpState(theDI);
        GetProblemShapes(aState, theAna, theShape, sl, NbProblems);
        aState.theMap.Clear();

        foreach ((StatusCheck aStatus, string aLine) in s_problemLines)
        {
            int aCount = NbProblems[(int)aStatus];
            if (aCount > 0)
            {
                theCommands.Append(aLine).Append(aCount).Append("\n");
            }
        }

        theCommands.Append(" ------------------------------------------------\n");
        theCommands.Append("*** Shapes with problems : ").Append(sl.Count).Append("\n");

        var aByType = new (ShapeType Type, string Label, string Suffix)[]
        {
            (ShapeType.Vertex, "VERTEX", "_v"), (ShapeType.Edge, "EDGE", "_e"), (ShapeType.Wire, "WIRE", "_w"), (ShapeType.Face, "FACE", "_f"),
            (ShapeType.Shell, "SHELL", "_s"), (ShapeType.Solid, "SOLID", "_o"),
        };
        var B = new BrepBuilder();
        foreach ((ShapeType aType, string aLabel, string aSuffix) in aByType)
        {
            List<Shape> aShapes = sl.Where(s => s.Type == aType).ToList();
            if (aShapes.Count == 0)
            {
                continue;
            }

            Compound comp = new();
            B.MakeCompound(ref comp);
            foreach (Shape aShape in aShapes)
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
        Shape aShape = theDI.GetShape(aShapeName);
        if (aShape.IsNull)
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
            var anAna = new Analyzer(aShape, aGeomCtrl, IsParallel, IsExact);
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
            theCommands.Append(Refusal.Of(anException) is { } aFailure ? aFailure.GetMessageString() : global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anException).GetType()));
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
        => (Refusal.IsFailure(theException) || theException is NullReferenceException or InvalidCastException or IndexOutOfRangeException or DivideByZeroException);

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
        Shape S = theDI.GetShape(a[1]);

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
        var aParity = new Dictionary<Shape, bool>(ShapeHasher.Instance);
        var anOrder = new List<Shape>();
        for (var exp = new Explorer(S, ShapeType.Vertex); exp.More(); exp.Next())
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

        List<Shape> theVertices = anOrder.Where(v => aParity[v]).ToList();
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
        foreach (Shape aVertex in theVertices)
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

        Shape S = theDI.GetShape(a[1]);
        double tol;
        if (S.Type == ShapeType.Edge)
        {
            var bce = new EdgeCheck(ShapeCasts.Edge(S));
            tol = bce.Tolerance();
            di.Append($"Tolerance de {Address(S)} : {StreamUtilities.FormatReal(tol, 6)}\n");
        }
        else
        {
            var theEdges = new HashSet<Shape>(ShapeHasher.Instance);
            for (var exp = new Explorer(S, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (theEdges.Add(exp.Current()))
                {
                    var bce = new EdgeCheck(ShapeCasts.Edge(exp.Current()));
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
    private static string Address(in Shape S)
        => ((long)RuntimeHelpers.GetHashCode(S.TShape()!)).ToString("X16", CultureInfo.InvariantCulture);
}
