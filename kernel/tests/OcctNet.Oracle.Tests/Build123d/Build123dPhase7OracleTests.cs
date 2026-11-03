// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using System.Globalization;
using OcctNet.TestHarness;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Milestone 2 of phase 7 (unit 7X; plan 1.5 and section 5): every healing, offset, draft and feature call
/// build123d.NET's tests make (<see cref="Build123dPhase7Capture"/>), replayed on the port from native's input bytes and
/// compared as unit 6M compared the Booleans: the exception (by OCCT's type) and the status (IsDone and Error, the draft's
/// status and problematic shape, the feature's status); the result as BRep text (the Free column masked), then canonically
/// where native's own result varies from run to run (five more native runs, on operands at other heap addresses), or
/// attributed; the history (<c>Generated</c>, <c>Modified</c>, <c>IsDeleted</c> of every vertex, edge and face of the
/// operands, as sets of the result's sub-shapes; SplitShape's Left and Right); and then build123d.NET's finishing step
/// (Hollow's solid of two shells and its Fix, Offset3D's FORWARD solid, the Offset operation's Fix, the Thicken
/// operation's Clean, ExtrudeTaper's validity check, SplitByPerimeter's sewing of each side) run on native's result in
/// both. The port's operands are frozen (a caller's) and checked unchanged, and its results frozen. Native's replay of
/// each call and its finishing step must give build123d.NET's own result (sub-shape counts, volume and area to nine
/// digits) or its failure, which checks the transcription. The fillets and chamfers (the unit's second part) are compared
/// geometrically: <c>Build123dPhase7OracleTests.Blend.cs</c>.
/// </summary>
public sealed partial class Build123dPhase7OracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly ShapeFixOracleRun myRun = new(output, "b3d7");

    private const double Build123dTolerance = 1e-6; // build123d.NET's Constants.Tolerance

    public void Dispose() => myRun.Dispose();

    /// <summary>Calls whose result is canonically identical only, where native repeats itself, by name, with the cause.</summary>
    private static readonly (string Prefix, string Cause)[] s_attributed = [];

    private sealed class Tally
    {
        public int Calls, Identical, FreeMasked, CheckedMasked, Canonical, Attributed, SameError, NoResult, NullAfterSuccess, Histories, HistoriesSame,
                   HistoriesCanonical, HistoriesBySize, Geometric, NativeInvalid, HistoriesGeometric, SolidSplit;
        public readonly List<string> Differences = [];
        public readonly List<string> AttributedCases = [];
        public readonly List<string> Failures = [];
    }

    private readonly SortedDictionary<string, Tally> myTallies = new(StringComparer.Ordinal);
    private readonly List<string> myTranscription = [];
    private int myTranscribed;

    private Tally TallyOf(string theLabel)
    {
        if (!myTallies.TryGetValue(theLabel, out Tally? aTally))
        {
            myTallies[theLabel] = aTally = new Tally();
        }
        return aTally;
    }

    [Fact]
    public void Build123d_phase7_operations_as_native()
    {
        var aFailures = new List<string>();
        List<P7Call> aCalls = Build123dPhase7Capture.Recorded(aFailures);
        foreach (string aFailure in aFailures)
        {
            output.WriteLine("scenario failed: " + aFailure);
        }
        Assert.Empty(aFailures);
        foreach (P7Call aCall in aCalls)
        {
            CompareCall(aCall);
        }
        Report(aCalls.Count);
    }

    private string Text(N.TopoDS_Shape? theShape) => theShape is null || theShape.IsNull() ? "" : myRun.NativeText(theShape);

    private string Text(Shape? theShape) => theShape is not Shape aS || aS.IsNull ? "" : myRun.PortText(aS);

    // ---------------------------------------------------------------------------------------------------------------
    // One call

    private void CompareCall(P7Call theCall)
    {
        if (theCall.Op is P7Op.Fillet or P7Op.Chamfer or P7Op.MaxFillet)
        {
            CompareBlend(theCall);
            return;
        }
        string aLabel = Build123dPhase7Capture.Label(theCall);
        string aWhat = $"{theCall.Test}: {theCall.Name} [{aLabel}]";
        Tally aTally = TallyOf(aLabel);
        aTally.Calls++;
        string anOperands = Build123dPhase7Capture.OperandsText(theCall.Operands);

        P7Native aN = Build123dPhase7Capture.RunNative(theCall, Build123dPhase7Capture.NativeOperands(anOperands));
        List<Shape> aPOps = Build123dPhase7Capture.PortOperands(anOperands);
        string aPOpsText = string.Concat(aPOps.Select(BRepTextComparison.Write));
        P7Port aP = Build123dPhase7Capture.RunPort(theCall, aPOps);
        if (string.Concat(aPOps.Select(BRepTextComparison.Write)) != aPOpsText)
        {
            aTally.Differences.Add($"{aWhat}: the port changed its frozen operands");
        }
        if (aP.Result is Shape aPR && !aPR.IsNull && !aPR.TShape()!.IsFrozen)
        {
            aTally.Differences.Add($"{aWhat}: the port's result isn't frozen");
        }

        string aNText = Text(aN.Result), aPText = Text(aP.Result);
        CheckTranscription(theCall, aN, aWhat);
        if (!Outcome(aTally, aWhat, aN.Error, aN.Status, aNText, aP.Error, aP.Status, aPText, theCall.Op == P7Op.DPrism,
                     () => Text(Build123dPhase7Capture.RunNative(theCall, Build123dPhase7Capture.NativeOperands(anOperands), false).Result),
                     out Match aMatch))
        {
            return;
        }
        if (aN.Result is null || aN.Result.IsNull())
        {
            aTally.NullAfterSuccess++;
        }
        else
        {
            CompareHistories(aTally, aWhat, aN.History, aNText, aP.History, aPText, aMatch);
        }
        ComparePost(theCall, aN, aWhat);
    }

    /// <summary>
    /// Compares the exceptions, the statuses and the results' texts; true if both sides gave a result (possibly null) and it
    /// counts as the same, <paramref name="isBytes"/> if byte-identical (Free masked).
    /// </summary>
    private bool Outcome(Tally theTally, string theWhat, string? theNErr, string theNStatus, string theNText, string? thePErr, string thePStatus,
                         string thePText, bool isCoarse, Func<string> theNativeAgain, out Match theMatch)
    {
        theMatch = Match.Bytes;
        if (theNErr is not null || thePErr is not null)
        {
            if (theNErr == thePErr)
            {
                theTally.SameError++;
                theTally.Failures.Add($"{theWhat}: {theNErr}");
            }
            else
            {
                theTally.Differences.Add($"{theWhat}: native {theNErr ?? theNStatus}, port {thePErr ?? thePStatus}");
            }
            return false;
        }
        if (theNStatus != thePStatus)
        {
            theTally.Differences.Add($"{theWhat}: status native {theNStatus}, port {thePStatus}");
            return false;
        }
        if (theNText.Length == 0 || thePText.Length == 0)
        {
            if ((theNText.Length == 0) != (thePText.Length == 0))
            {
                theTally.Differences.Add($"{theWhat}: native result {(theNText.Length == 0 ? "null" : "present")}, port {(thePText.Length == 0 ? "null" : "present")}");
                return false;
            }
            theTally.NoResult++;
            if (!theNStatus.StartsWith("valid=", StringComparison.Ordinal))
            {
                theTally.Failures.Add($"{theWhat}: no result, {theNStatus}");
            }
            return true;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(theNText, thePText);
        if (Environment.GetEnvironmentVariable("P7X_DUMP") is string aDump && !aDiff.IsIdenticalMaskingFree)
        {
            Directory.CreateDirectory(aDump);
            string aBase = Path.Combine(aDump, string.Concat(theWhat.Select(ch => char.IsLetterOrDigit(ch) ? ch : '_')));
            File.WriteAllText(aBase + "_native.brep", theNText);
            File.WriteAllText(aBase + "_port.brep", thePText);
        }
        if (aDiff.IsIdentical || aDiff.IsIdenticalMaskingFree)
        {
            if (aDiff.IsIdentical)
            {
                theTally.Identical++;
            }
            else
            {
                theTally.FreeMasked++;
            }
            return true;
        }
        if (ShapeFixFaceOracleTests.MaskFreeAndChecked(theNText) == ShapeFixFaceOracleTests.MaskFreeAndChecked(thePText))
        {
            // native sets the Checked flag of sub-shapes it shares with its input in place (as decision 738 found for
            // BRepTools::Update), which the port's frozen ones keep
            theTally.CheckedMasked++;
            return true;
        }
        theMatch = Match.Canonical;
        // native again, eight times on operands read at other heap addresses (native iterates some maps in address order,
        // decisions 1017 and 1032): its variants
        var aVariants = new List<string> { theNText };
        var aBallast = new List<N.TopoDS_Shape>();
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 997 * (i % 4 + 1) + 13 * i; j++)
            {
                using var aPnt = new N.gp_Pnt(j, i, 0);
                using var aV = new N.BRepBuilderAPI_MakeVertex(aPnt);
                aBallast.Add(aV.Vertex());
            }
            string anAgain = theNativeAgain();
            if (anAgain.Length > 0 && !aVariants.Contains(anAgain))
            {
                aVariants.Add(anAgain);
            }
        }
        GC.KeepAlive(aBallast);
        bool isNativeVarying = aVariants.Count > 1;
        Shape aNP = BooleanOracleTests.ReadPort(theNText), aPP = BooleanOracleTests.ReadPort(thePText);
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPP, new CanonicalOptions { CheckValidity = false });
        bool isSame = aCanon.IsIdentical;
        if (!isSame && isNativeVarying)
        {
            // the port's result is one of native's variants
            isSame = aVariants.Skip(1).Any(v => BRepTextComparison.Compare(v, thePText).IsIdenticalMaskingFree
                                                || CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(v), aPP,
                                                                                  new CanonicalOptions { CheckValidity = false }).IsIdentical);
        }
        if (!isSame && isCoarse && isNativeVarying)
        {
            // decision 1017: native DPrism's text varies run to run; its coarse record decides
            isSame = BRepFeatOracleTests.Coarse(aNP) == BRepFeatOracleTests.Coarse(aPP);
        }
        if (isSame && isNativeVarying)
        {
            if (!aCanon.IsIdentical)
            {
                // the port's result is another of native's variants than the first, whose history describes that first
                // one: the histories compare by size (OffsetABareSolid's Offset3D, decision 1032, under a gate's load)
                theMatch = Match.Geometric;
            }
            theTally.Canonical++;
            return true;
        }
        if (isNativeVarying && aVariants.Any(v => BRepOffsetAPIMakeThickSolidOracleTests.Geometric(
                                                      BooleanOracleTests.ReadPort(v), aPP,
                                                      CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(v), aPP,
                                                                                     new CanonicalOptions { CheckValidity = false })) is null))
        {
            theMatch = Match.Geometric;
            theTally.Attributed++;
            theTally.AttributedCases.Add($"{theWhat}: decision 1032: native varies ({aVariants.Count} variants in nine runs); the port's result has "
                                         + "native's geometry (sub-shape counts, volume, area, centroid, validity)");
            return true;
        }
        string? aCause = s_attributed.FirstOrDefault(a => theWhat.Contains(a.Prefix, StringComparison.Ordinal)).Cause;
        if (isSame && aCause is not null)
        {
            theTally.Attributed++;
            theTally.AttributedCases.Add($"{theWhat}: {aCause}");
            return true;
        }
        theTally.Differences.Add($"{theWhat}: {aDiff}; canonically {aCanon}; native {(isNativeVarying ? "varies" : "repeats itself")}");
        return false;
    }

    // The history with each result index replaced by the canonical key of that sub-shape of the result read back, each list sorted
    private static string Keyed(string theHistory, string theResultText)
    {
        Shape aResult = BooleanOracleTests.ReadPort(theResultText);
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aResult, aMap);
        CanonicalKeys aKeys = CanonicalKeys.Build(aResult);
        return System.Text.RegularExpressions.Regex.Replace(theHistory, @"([GM]|left |right )\[([^\]]*)\]", m =>
            m.Groups[1].Value + "[" + string.Join(' ', m.Groups[2].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                                                         .Select(s => (I: int.Parse(s.TrimEnd('r'), CultureInfo.InvariantCulture), R: s.EndsWith('r')))
                                                         .Select(t => (t.I > 0 ? aKeys.Key(aMap[t.I]) : "out") + (t.R ? "r" : ""))
                                                         .Order(StringComparer.Ordinal)) + "]");
    }

    /// <summary>How a result matched native's: byte for byte, canonically (or as one of native's variants), or geometrically.</summary>
    private enum Match { Bytes, Canonical, Geometric }

    // The history with each list replaced by its size, for results that agree only geometrically
    private static string Sized(string theHistory) =>
        System.Text.RegularExpressions.Regex.Replace(theHistory, @"([GM]|left |right )\[([^\]]*)\]",
                                                     m => m.Groups[1].Value + "[" + m.Groups[2].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries).Length + "]");

    private static void CompareHistories(Tally theTally, string theWhat, string theNHistory, string theNText, string thePHistory, string thePText,
                                         Match theMatch)
    {
        if (theNHistory.Length == 0 && thePHistory.Length == 0)
        {
            return;
        }
        theTally.Histories++;
        if (theMatch == Match.Geometric)
        {
            if (Sized(theNHistory) == Sized(thePHistory))
            {
                theTally.HistoriesBySize++;
            }
            else
            {
                theTally.Differences.Add($"{theWhat}: histories differ in size: {DeterminismHarness.FirstDifference(Sized(theNHistory), Sized(thePHistory))}");
            }
            return;
        }
        if (theMatch == Match.Bytes)
        {
            if (theNHistory == thePHistory)
            {
                theTally.HistoriesSame++;
            }
            else
            {
                theTally.Differences.Add($"{theWhat}: histories differ: {DeterminismHarness.FirstDifference(theNHistory, thePHistory)}");
            }
            return;
        }
        string aNKeyed = Keyed(theNHistory, theNText), aPKeyed = Keyed(thePHistory, thePText);
        if (aNKeyed == aPKeyed)
        {
            theTally.HistoriesCanonical++;
        }
        else
        {
            theTally.Differences.Add($"{theWhat}: histories differ canonically: {DeterminismHarness.FirstDifference(aNKeyed, aPKeyed)}");
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // build123d.NET's finishing step, on native's result in both

    private void ComparePost(P7Call theCall, P7Native theN, string theWhat)
    {
        if (theCall.Post == P7Post.None || (theCall.Post == P7Post.SewSides ? theN.Left is null : theN.Result is null || theN.Result.IsNull()))
        {
            return; // nothing to finish: build123d.NET reports the failure
        }
        string aLabel = $"{Build123dPhase7Capture.Label(theCall)}: then {theCall.Post}";
        N.TopoDS_Shape[] aPostOps = theCall.Post switch
        {
            P7Post.HollowSolid => [theN.Result!, theCall.Operands[0]],
            P7Post.SewSides => [NCompound(theN.Left!), NCompound(theN.Right!)],
            _ => [theN.Result!],
        };
        Tally aTally = TallyOf(aLabel);
        aTally.Calls++;
        string aText = Build123dPhase7Capture.OperandsText(aPostOps);
        (string? aNErr, string aNStatus, N.TopoDS_Shape? aNRes) = NativePost(theCall, Build123dPhase7Capture.NativeOperands(aText));
        List<Shape> aPOps = Build123dPhase7Capture.PortOperands(aText);
        string aPOpsText = string.Concat(aPOps.Select(BRepTextComparison.Write));
        (string? aPErr, string aPStatus, Shape? aPRes) = PortPost(theCall, aPOps);
        if (string.Concat(aPOps.Select(BRepTextComparison.Write)) != aPOpsText)
        {
            aTally.Differences.Add($"{theWhat} {theCall.Post}: the port changed its frozen operands");
        }
        Outcome(aTally, theWhat, aNErr, aNStatus, Text(aNRes), aPErr, aPStatus, Text(aPRes), false,
                () => Text(NativePost(theCall, Build123dPhase7Capture.NativeOperands(aText)).Item3), out _);
    }

    private static N.TopoDS_Compound NCompound(IEnumerable<N.TopoDS_Shape> theShapes)
    {
        using var aB = new N.BRep_Builder();
        var aC = new N.TopoDS_Compound();
        aB.MakeCompound(aC);
        foreach (N.TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }
        return aC;
    }

    private static List<N.TopoDS_Shape> Children(N.TopoDS_Shape theShape)
    {
        var aList = new List<N.TopoDS_Shape>();
        for (var anIt = new N.TopoDS_Iterator(theShape); anIt.More(); anIt.Next())
        {
            aList.Add(anIt.Value());
        }
        GC.KeepAlive(theShape); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
        return aList;
    }

    private static N.TopoDS_Shape NFirst(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType)
    {
        using var anExp = new N.TopExp_Explorer(theShape, theType);
        return anExp.Current();
    }

    // TopoUtil.Fix: ShapeFix_Shape on a private copy (geometry included), only if BRepCheck finds the shape invalid
    private static N.TopoDS_Shape NFix(N.TopoDS_Shape theShape)
    {
        using (var aCheck = new N.BRepCheck_Analyzer(theShape))
        {
            if (aCheck.IsValid())
            {
                return theShape;
            }
        }
        using var aCopier = new N.BRepBuilderAPI_Copy(theShape, true, false);
        using var aFix = new N.ShapeFix_Shape(aCopier.Shape());
        aFix.Perform();
        return aFix.Shape();
    }

    // ThreeD.ForwardSolid, then the reversal of an inside-out result
    private static N.TopoDS_Shape NForward(N.TopoDS_Shape theShape)
    {
        N.TopoDS_Shape aSolid = theShape;
        if (theShape.ShapeType() == N.TopAbs_ShapeEnum.TopAbs_SOLID)
        {
            var aFlags = new List<N.TopAbs_Orientation> { theShape.Orientation() };
            aFlags.AddRange(Children(theShape).Select(s => s.Orientation()));
            if (!aFlags.All(f => f == N.TopAbs_Orientation.TopAbs_FORWARD))
            {
                using var aB = new N.BRep_Builder();
                var aForward = new N.TopoDS_Solid();
                aB.MakeSolid(aForward);
                foreach (N.TopoDS_Shape aShell in Children(theShape))
                {
                    var aNew = new N.TopoDS_Shell();
                    aB.MakeShell(aNew);
                    foreach (N.TopoDS_Shape aFace in Children(aShell))
                    {
                        aB.Add(aNew, aFace);
                    }
                    aB.Add(aForward, aNew);
                }
                aSolid = aForward;
            }
        }
        using var aProps = new N.GProp_GProps();
        N.BRepGProp.VolumeProperties(aSolid, aProps);
        return aProps.Mass() < 0 ? aSolid.Reversed() : aSolid;
    }

    private static (string?, string, N.TopoDS_Shape?) NativePost(P7Call theCall, List<N.TopoDS_Shape> theOps)
    {
        try
        {
            switch (theCall.Post)
            {
                case P7Post.HollowSolid:
                {
                    N.TopoDS_Shell aShell1 = N.TopoDS.Shell(NFirst(theOps[0], N.TopAbs_ShapeEnum.TopAbs_SHELL));
                    N.TopoDS_Shell aShell2 = N.TopoDS.Shell(NFirst(theOps[1], N.TopAbs_ShapeEnum.TopAbs_SHELL));
                    using var aMaker = theCall.Args.Value > 0 ? new N.BRepBuilderAPI_MakeSolid(aShell1, aShell2) : new N.BRepBuilderAPI_MakeSolid(aShell2, aShell1);
                    return (null, "", NFix(aMaker.Shape()));
                }
                case P7Post.Offset3D:
                    return (null, "", NForward(theOps[0]));
                case P7Post.OffsetOperation:
                    return (null, "", NFix(NForward(theOps[0])));
                case P7Post.Clean:
                {
                    using var aU = new N.ShapeUpgrade_UnifySameDomain(theOps[0], true, true, true);
                    aU.AllowInternalEdges(false);
                    aU.Build();
                    return (null, "", aU.Shape());
                }
                case P7Post.Validity:
                {
                    using var aCheck = new N.BRepCheck_Analyzer(theOps[0]);
                    aCheck.SetParallel(true);
                    return (null, $"valid={aCheck.IsValid()}", null);
                }
                default: // SewSides
                {
                    var aSides = new List<N.TopoDS_Shape>();
                    foreach (N.TopoDS_Shape aSide in theOps)
                    {
                        List<N.TopoDS_Shape> aFaces = Children(aSide);
                        if (aFaces.Count < 2)
                        {
                            aSides.AddRange(aFaces);
                            continue;
                        }
                        using var aSewing = new N.BRepBuilderAPI_Sewing(Build123dTolerance);
                        aFaces.ForEach(aSewing.Add);
                        aSewing.Perform();
                        aSides.Add(aSewing.SewedShape());
                    }
                    return (null, "", NCompound(aSides));
                }
            }
        }
        catch (N.OcctException e)
        {
            return (Build123dPhase7Capture.ErrorOf(e), "", null);
        }
    }

    private static Shape PFirst(in Shape theShape, ShapeType theType) => new Explorer(theShape, theType).Current();

    private static List<Shape> PChildren(in Shape theShape)
    {
        var aList = new List<Shape>();
        for (var anIt = new Iterator(theShape); anIt.More(); anIt.Next())
        {
            aList.Add(anIt.Value());
        }
        return aList;
    }

    private static Shape PFix(in Shape theShape)
    {
        if (new BRepCheck_Analyzer(theShape).IsValid())
        {
            return theShape;
        }
        var aFix = new ShapeFix_Shape(new BRepBuilderAPI_Copy(theShape, true, false).Shape());
        aFix.Perform();
        return aFix.Shape();
    }

    private static Shape PForward(in Shape theShape)
    {
        Shape aSolid = theShape;
        if (theShape.ShapeType() == ShapeType.Solid)
        {
            var aFlags = new List<Orientation> { theShape.Orientation() };
            aFlags.AddRange(PChildren(theShape).Select(s => s.Orientation()));
            if (!aFlags.All(f => f == Orientation.Forward))
            {
                var aB = new BrepBuilder();
                var aForward = new Solid();
                aB.MakeSolid(ref aForward);
                foreach (Shape aShell in PChildren(theShape))
                {
                    var aNew = new Shell();
                    aB.MakeShell(ref aNew);
                    foreach (Shape aFace in PChildren(aShell))
                    {
                        aB.Add(aNew, aFace);
                    }
                    aB.Add(aForward, aNew);
                }
                aSolid = aForward;
            }
        }
        var aProps = new Properties();
        BRepGProp.VolumeProperties(aSolid, aProps);
        return aProps.Mass() < 0 ? aSolid.Reversed() : aSolid;
    }

    private static (string?, string, Shape?) PortPost(P7Call theCall, List<Shape> theOps)
    {
        try
        {
            switch (theCall.Post)
            {
                case P7Post.HollowSolid:
                {
                    Shell aShell1 = ShapeCasts.Shell(PFirst(theOps[0], ShapeType.Shell));
                    Shell aShell2 = ShapeCasts.Shell(PFirst(theOps[1], ShapeType.Shell));
                    var aMaker = theCall.Args.Value > 0 ? new BRepBuilderAPI_MakeSolid(aShell1, aShell2) : new BRepBuilderAPI_MakeSolid(aShell2, aShell1);
                    return (null, "", PFix(aMaker.Shape()));
                }
                case P7Post.Offset3D:
                    return (null, "", PForward(theOps[0]));
                case P7Post.OffsetOperation:
                    return (null, "", PFix(PForward(theOps[0])));
                case P7Post.Clean:
                {
                    var aU = new ShapeUpgrade_UnifySameDomain(theOps[0], true, true, true);
                    aU.AllowInternalEdges(false);
                    aU.Build();
                    return (null, "", aU.Shape());
                }
                case P7Post.Validity:
                {
                    var aCheck = new BRepCheck_Analyzer(theOps[0]);
                    aCheck.SetParallel(true);
                    return (null, $"valid={aCheck.IsValid()}", null);
                }
                default:
                {
                    var aSides = new List<Shape>();
                    foreach (Shape aSide in theOps)
                    {
                        List<Shape> aFaces = PChildren(aSide);
                        if (aFaces.Count < 2)
                        {
                            aSides.AddRange(aFaces);
                            continue;
                        }
                        var aSewing = new BRepBuilderAPI_Sewing(Build123dTolerance);
                        aFaces.ForEach(f => aSewing.Add(f));
                        aSewing.Perform();
                        aSides.Add(aSewing.SewedShape());
                    }
                    return (null, "", Builder.MakeCompound(aSides));
                }
            }
        }
        catch (Exception e) when (e is FailureException or NullReferenceException)
        {
            return (Build123dPhase7Capture.ErrorOf(e), "", null);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The transcription: native's replay of the call (and its finishing step) gives build123d.NET's own result

    private void CheckTranscription(P7Call theCall, P7Native theN, string theWhat)
    {
        if (theCall.Op is P7Op.Split || theCall.Post is P7Post.Validity)
        {
            return; // SplitByPerimeter picks the inside by edge lengths, ExtrudeTaper may fall back to a loft: their results aren't the call's
        }
        myTranscribed++;
        N.TopoDS_Shape? aReplay = theN.Result;
        if (aReplay is not null && !aReplay.IsNull() && theCall.Post != P7Post.None)
        {
            N.TopoDS_Shape[] aPostOps = theCall.Post == P7Post.HollowSolid ? [aReplay, theCall.Operands[0]] : [aReplay];
            aReplay = NativePost(theCall, Build123dPhase7Capture.NativeOperands(Build123dPhase7Capture.OperandsText(aPostOps))).Item3;
        }
        bool isReplayFailing = theN.Error is not null || aReplay is null || aReplay.IsNull() || theN.Status.Contains("done=False", StringComparison.Ordinal)
                               || theN.Status.StartsWith("add=False", StringComparison.Ordinal);
        if (theCall.Error is not null || theCall.Result is null)
        {
            if (!isReplayFailing && theCall.Op != P7Op.MakeOffset)
            {
                myTranscription.Add($"{theWhat}: build123d.NET threw {theCall.Error}, the replay succeeded");
            }
            return;
        }
        if (isReplayFailing)
        {
            myTranscription.Add($"{theWhat}: build123d.NET succeeded, the replay failed ({theN.Error ?? theN.Status})");
            return;
        }
        string anExpected = Signature(theCall.Result), aReplayed = Signature(aReplay!);
        if (anExpected != aReplayed)
        {
            myTranscription.Add($"{theWhat}: build123d.NET's {anExpected}, the replay's {aReplayed}");
        }
    }

    // The numbers of solids, faces, edges and vertices, the volume and the area (to nine digits)
    private static string Signature(N.TopoDS_Shape theShape)
    {
        var aCounts = new List<int>();
        foreach (N.TopAbs_ShapeEnum aType in new[] { N.TopAbs_ShapeEnum.TopAbs_SOLID, N.TopAbs_ShapeEnum.TopAbs_FACE, N.TopAbs_ShapeEnum.TopAbs_EDGE,
                                                     N.TopAbs_ShapeEnum.TopAbs_VERTEX })
        {
            using var aMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theShape, aType, aMap);
            aCounts.Add(aMap.Extent());
        }
        using var aVolume = new N.GProp_GProps();
        N.BRepGProp.VolumeProperties(theShape, aVolume);
        using var anArea = new N.GProp_GProps();
        N.BRepGProp.SurfaceProperties(theShape, anArea);
        return string.Create(CultureInfo.InvariantCulture, $"{string.Join("/", aCounts)} volume {aVolume.Mass():G9} area {anArea.Mass():G9}");
    }

    // ---------------------------------------------------------------------------------------------------------------

    private void Report(int theCalls)
    {
        output.WriteLine($"build123d.NET's tests: {theCalls} phase 7 calls");
        int aDifferences = 0;
        foreach ((string aLabel, Tally t) in myTallies)
        {
            output.WriteLine($"{aLabel}: {t.Calls} calls: {t.Identical} byte-identical, {t.FreeMasked} Free-masked, {t.CheckedMasked} Free and "
                             + $"Checked masked, {t.Canonical} canonical (native varies), {t.Geometric} geometrically native's (option C), "
                             + $"{t.NativeInvalid} where native's result is invalid, {t.Attributed} attributed, {t.SameError} the same "
                             + $"exception, {t.NoResult} no result on both sides (or a status only) ({t.NullAfterSuccess} null after success), {t.Differences.Count} "
                             + $"different; histories {t.HistoriesSame} identical, {t.HistoriesCanonical} canonically equal, {t.HistoriesBySize} "
                             + $"equal in size (results equal geometrically), "
                             + $"{t.HistoriesGeometric} equal as sets of sub-shape measures, of {t.Histories}");
            if (t.SolidSplit > 0)
            {
                output.WriteLine($"  attributed: in {t.SolidSplit} results the input solid's own history differs (Modified empty, IsDeleted true; native: "
                                 + "the result's solid, false): the reconstruction records no split of the solid (unit 7R3)");
            }
            foreach (string aF in t.Failures)
            {
                output.WriteLine("  failure on both sides: " + aF);
            }
            foreach (string anA in t.AttributedCases)
            {
                output.WriteLine("  attributed: " + anA);
            }
            foreach (string aD in t.Differences)
            {
                output.WriteLine("  DIFFERENT " + aD);
            }
            aDifferences += t.Differences.Count;
        }
        output.WriteLine($"transcription: {myTranscribed} calls checked against build123d.NET's own results, {myTranscription.Count} differ");
        foreach (string aT in myTranscription)
        {
            output.WriteLine("  transcription: " + aT);
        }
        Assert.Equal(0, aDifferences);
        Assert.Empty(myTranscription);
    }
}
