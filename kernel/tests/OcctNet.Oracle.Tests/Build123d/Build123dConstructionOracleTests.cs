#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Build123d;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using B = Build123d;
using N = Build123d.Occt;
using G = OcctNet.Oracle.Tests.Geom2dGccOracleTests;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Milestone 6's build123d.NET half (unit 6M, plan section 5): the construction calls build123d.NET's own tests make,
/// replayed on the port. The operands are rebuilt with build123d.NET's public API exactly as its tests build them
/// (decision 952); the native calls are those build123d.NET's source makes, with its arguments and options, in its
/// order: <c>BRepOffsetAPI_MakePipeShell</c> (Solid.Sweep, SweepMulti, ExtrudeLinearWithRotation, Face.Sweep,
/// Shell.Sweep), <c>BRepOffsetAPI_ThruSections</c> (MakeLoft and the Loft operation), <c>BRepOffsetAPI_MakeFilling</c>
/// (Face.MakeSurface, MakeSurfacePatch), <c>GeomAPI_PointsToBSplineSurface</c> (MakeSurfaceFromArrayOfPoints),
/// <c>BRepFill::Face</c>/<c>Shell</c> (MakeSurfaceFromCurves), <c>BRepProj_Projection</c> (Project, ProjectToShape), the
/// seven Geom2dGcc solvers (MakeConstrainedArcs and MakeConstrainedLines), <c>BRepFilletAPI_MakeFillet2d</c> (Chamfer2D)
/// and <c>BRepOffsetAPI_MakeOffset</c> (Offset2D).
/// </summary>
/// <remarks>
/// The operands go through BRep text once: native OCCT writes them as a compound, and every run reads them afresh (the
/// port's frozen, as a caller's, and checked unchanged after the run). Results are compared as BRep text, the Free
/// column masked (open question 1), then canonically (decision 581) where native's own result varies from run to run
/// (five more native runs measure that). The solvers are compared token for token through the Geom2dGcc oracle's
/// records (solutions, tangencies, every double bit for bit).
/// </remarks>
public sealed class Build123dConstructionOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly ShapeFixOracleRun myRun = new(output, "b3dc");

    public void Dispose() => myRun.Dispose();

    private const double DegToRad = Math.PI / 180.0;

    // ---------------------------------------------------------------------------------------------------------------
    // The harness

    private sealed class Tally
    {
        public int Cases, Identical, FreeMasked, SameError, Canonical, Attributed;
        public readonly List<string> Failures = [];
        public readonly Dictionary<string, int> Attributions = [];
    }

    private readonly SortedDictionary<string, Tally> myTallies = new(StringComparer.Ordinal);

    private Tally TallyOf(string theAlgo)
    {
        if (!myTallies.TryGetValue(theAlgo, out Tally? aTally))
        {
            myTallies[theAlgo] = aTally = new Tally();
        }
        return aTally;
    }

    /// <summary>The operands as native OCCT writes them: one compound, whose children are read back in order.</summary>
    private string Operands(params B.Shape[] theShapes) => BooleanOracleTests.ArgsText(myRun, [.. theShapes.Select(s => s.Wrapped)]);

    private List<N.TopoDS_Shape> NativeOperands(string theText)
    {
        string aFile = Path.Combine(Path.GetTempPath(), $"b3dc-{Guid.NewGuid():N}.brep");
        File.WriteAllText(aFile, theText);
        try
        {
            N.TopoDS_Shape aC = ShapeFixOracleRun.NativeRead(aFile) ?? throw new InvalidOperationException("native can't read the operands");
            var aList = new List<N.TopoDS_Shape>();
            for (var anIt = new N.TopoDS_Iterator(aC); anIt.More(); anIt.Next())
            {
                aList.Add(anIt.Value());
            }
            GC.KeepAlive(aC); // native TopoDS_Iterator holds no reference to the shape it iterates (decision 1109)
            return aList;
        }
        finally
        {
            File.Delete(aFile);
        }
    }

    private static List<TopoDS_Shape> PortOperands(string theText, bool theFrozen = true)
    {
        TopoDS_Shape aC = BooleanOracleTests.ReadPort(theText);
        if (theFrozen)
        {
            aC.TShape()!.FreezeGraph(); // a caller's shapes
        }
        else
        {
            // a copy the algorithm may write into, as native's operands (the shapes read are frozen)
            aC = BRepTools.ExactCopyForWrite(aC, out _);
        }
        var aList = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(aC); anIt.More(); anIt.Next())
        {
            aList.Add(anIt.Value());
        }
        return aList;
    }

    private (string? Text, string? Error) RunNative(string theOperands, Func<List<N.TopoDS_Shape>, N.TopoDS_Shape> theRun)
    {
        try
        {
            N.TopoDS_Shape aRes = theRun(NativeOperands(theOperands));
            return (myRun.NativeText(aRes), null);
        }
        catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
        {
            return (null, ShapeFixOracleRun.ErrorOf(e));
        }
    }

    /// <summary>
    /// One case: native's result and the port's from the same operands, compared as bytes (Free masked), then
    /// canonically where native varies; <paramref name="theAttribution"/> explains a canonical-only result where native
    /// repeats itself.
    /// </summary>
    private void Compare(string theAlgo, string theName, string theOperands, Func<List<N.TopoDS_Shape>, N.TopoDS_Shape> theNative,
                         Func<List<TopoDS_Shape>, TopoDS_Shape> thePort, string? theAttribution = null)
    {
        Tally aTally = TallyOf(theAlgo);
        aTally.Cases++;
        (string? aNText, string? aNErr) = RunNative(theOperands, theNative);

        List<TopoDS_Shape> aPOps = PortOperands(theOperands);
        string aPOpsText = string.Concat(aPOps.Select(BRepTextComparison.Write));
        string? aPText = null, aPErr = null;
        try
        {
            aPText = myRun.PortText(thePort(aPOps));
        }
        catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
        {
            aPErr = ShapeFixOracleRun.ErrorOf(e);
        }
        if (string.Concat(aPOps.Select(BRepTextComparison.Write)) != aPOpsText)
        {
            aTally.Failures.Add($"{theName}: the port changed its frozen operands");
        }

        if (aNErr is not null || aPErr is not null)
        {
            if (aNErr == aPErr)
            {
                aTally.SameError++;
            }
            else
            {
                aTally.Failures.Add($"{theName}: native {aNErr ?? "done"}, port {aPErr ?? "done"}");
            }
            return;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNText!, aPText!);
        if (aDiff.IsIdentical)
        {
            aTally.Identical++;
            return;
        }
        if (aDiff.IsIdenticalMaskingFree)
        {
            aTally.FreeMasked++;
            return;
        }
        // native again, five times on operands read at other heap addresses (native iterates some maps in address order)
        bool isNativeVarying = false;
        var aBallast = new List<N.TopoDS_Shape>();
        for (int i = 0; i < 5 && !isNativeVarying; i++)
        {
            for (int j = 0; j < 997 * (i % 4 + 1) + 13 * i; j++)
            {
                using var aP = new N.gp_Pnt(j, i, 0);
                using var aV = new N.BRepBuilderAPI_MakeVertex(aP);
                aBallast.Add(aV.Vertex());
            }
            isNativeVarying = RunNative(theOperands, theNative).Text != aNText;
        }
        GC.KeepAlive(aBallast);
        CanonicalComparison aCanon = CanonicalShapeComparer.Compare(BooleanOracleTests.ReadPort(aNText!), BooleanOracleTests.ReadPort(aPText!),
                                                                    new CanonicalOptions { CheckValidity = false });
        if (aCanon.IsIdentical && isNativeVarying)
        {
            aTally.Canonical++;
        }
        else if (aCanon.IsIdentical && DestructiveMatches(aNText!, theOperands, thePort))
        {
            // decision 577: the port's frozen operands keep a Boolean inside the algorithm non-destructive where native's
            // writes into (or keeps) the caller's sub-shapes; on unfrozen operands the port gives native's bytes
            const string aWhy = "decision 577: byte-identical on unfrozen operands (the port's frozen ones make an inner Boolean non-destructive)";
            aTally.Attributed++;
            aTally.Attributions[aWhy] = aTally.Attributions.GetValueOrDefault(aWhy) + 1;
        }
        else if (aCanon.IsIdentical && theAttribution is not null)
        {
            aTally.Attributed++;
            aTally.Attributions[theAttribution] = aTally.Attributions.GetValueOrDefault(theAttribution) + 1;
        }
        else
        {
            aTally.Failures.Add($"{theName}: {aDiff}; canonically {aCanon}; native {(isNativeVarying ? "varies" : "repeats itself")}");
        }
    }

    // The port on unfrozen operands (read afresh, not frozen), as native's are: true if it then gives native's bytes,
    // the Free column masked
    private bool DestructiveMatches(string theNText, string theOperands, Func<List<TopoDS_Shape>, TopoDS_Shape> thePort)
    {
        try
        {
            string aText = myRun.PortText(thePort(PortOperands(theOperands, false)));
            return BRepTextComparison.Compare(theNText, aText).IsIdenticalMaskingFree;
        }
        catch (Exception e) when (ShapeFixOracleRun.ErrorOf(e) is not null)
        {
            return false;
        }
    }

    /// <summary>One solver case: the records of both sides, token for token.</summary>
    private void CompareSolver(string theName, string theOperands, Func<List<N.TopoDS_Shape>, G.Rec> theNative,
                               Func<List<TopoDS_Shape>, G.Rec> thePort, Func<string, bool>? theMasked = null)
    {
        Tally aTally = TallyOf("Geom2dGcc");
        aTally.Cases++;
        G.Rec aN = theNative(NativeOperands(theOperands));
        G.Rec aP = thePort(PortOperands(theOperands));
        List<string> aDiff = G.Differences(aN, aP, theMasked ?? (_ => false), out _, out _);
        if (aDiff.Count == 0)
        {
            aTally.Identical++;
        }
        else
        {
            aTally.Failures.Add($"{theName}: {string.Join("; ", aDiff.Take(6))}");
        }
    }

    private void Report()
    {
        int aCases = 0, aFailures = 0;
        foreach ((string anAlgo, Tally t) in myTallies)
        {
            output.WriteLine($"{anAlgo}: {t.Cases} cases: {t.Identical} byte-identical, {t.FreeMasked} Free-masked, {t.SameError} the same error, "
                             + $"{t.Canonical} canonical (native varies), {t.Attributed} attributed, {t.Failures.Count} failing");
            foreach ((string aWhy, int aCount) in t.Attributions)
            {
                output.WriteLine($"  attributed ({aCount}): {aWhy}");
            }
            foreach (string aF in t.Failures)
            {
                output.WriteLine("  FAIL " + aF);
            }
            aCases += t.Cases;
            aFailures += t.Failures.Count;
        }
        output.WriteLine($"total: {aCases} cases, {aFailures} failing");
        Assert.Equal(0, aFailures);
    }

    private static N.TopoDS_Compound NCompound(params N.TopoDS_Shape[] theShapes)
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

    private static TopoDS_Compound PCompound(params TopoDS_Shape[] theShapes) => TopoDS_Builder.MakeCompound(theShapes);

    // ---------------------------------------------------------------------------------------------------------------
    // Sweeps: BRepOffsetAPI_MakePipeShell

    /// <summary>
    /// The call sequence of one of build123d.NET's sweeps. Operands: the path wire, the profiles, then the auxiliary
    /// spine if any. <paramref name="Mode"/>: 0 none, 1 SetMode(isFrenet), 2 SetMode(gp_Ax2) at <paramref name="Start"/>
    /// along <paramref name="Dir"/> (a Vector binormal), 3 SetMode(auxiliary, curvilinear). The transition is set before
    /// the profiles are added (Solid.Sweep) or after (Face.Sweep, Shell.Sweep), or not at all (null).
    /// </summary>
    private sealed record SweepCall(int Mode, bool Frenet = false, B.Vector? Start = null, B.Vector? Dir = null, bool Curvilinear = false,
                                    BRepBuilderAPI_TransitionMode? Transition = null, bool TransitionAfterAdd = false, bool MakeSolid = false,
                                    bool Rotate = false, bool DefaultAdd = false);

    private void Sweep(string theName, B.Wire thePath, IReadOnlyList<B.Shape> theProfiles, SweepCall theCall, B.Wire? theAux = null)
    {
        B.Shape[] anOps = [thePath, .. theProfiles, .. (theAux is null ? Array.Empty<B.Shape>() : [theAux])];
        int aNbProfiles = theProfiles.Count;
        Compare("MakePipeShell", theName, Operands(anOps), o =>
        {
            using var aB = new N.BRepOffsetAPI_MakePipeShell(N.TopoDS.Wire(o[0]));
            switch (theCall.Mode)
            {
                case 1:
                    aB.SetMode(theCall.Frenet);
                    break;
                case 2:
                {
                    using var anAx = new N.gp_Ax2();
                    using var aP = new N.gp_Pnt(theCall.Start!.Value.X, theCall.Start.Value.Y, theCall.Start.Value.Z);
                    using var aD = new N.gp_Dir(theCall.Dir!.Value.X, theCall.Dir.Value.Y, theCall.Dir.Value.Z);
                    anAx.SetLocation(aP);
                    anAx.SetDirection(aD);
                    aB.SetMode(anAx);
                    break;
                }
                case 3:
                    aB.SetMode(N.TopoDS.Wire(o[^1]), theCall.Curvilinear);
                    break;
            }
            if (theCall.Transition is { } t && !theCall.TransitionAfterAdd)
            {
                aB.SetTransitionMode((N.BRepBuilderAPI_TransitionMode)(int)t);
            }
            for (int i = 1; i <= aNbProfiles; i++)
            {
                if (theCall.DefaultAdd)
                {
                    aB.Add(o[i]);
                }
                else
                {
                    aB.Add(o[i], false, theCall.Rotate);
                }
            }
            if (theCall.Transition is { } t2 && theCall.TransitionAfterAdd)
            {
                aB.SetTransitionMode((N.BRepBuilderAPI_TransitionMode)(int)t2);
            }
            aB.Build();
            if (theCall.MakeSolid)
            {
                aB.MakeSolid();
            }
            return aB.Shape();
        }, o =>
        {
            var aB = new BRepOffsetAPI_MakePipeShell(TopoDS.Wire(o[0]));
            switch (theCall.Mode)
            {
                case 1:
                    aB.SetMode(theCall.Frenet);
                    break;
                case 2:
                {
                    var anAx = new gp_Ax2();
                    anAx.SetLocation(new gp_Pnt(theCall.Start!.Value.X, theCall.Start.Value.Y, theCall.Start.Value.Z));
                    anAx.SetDirection(new gp_Dir(theCall.Dir!.Value.X, theCall.Dir.Value.Y, theCall.Dir.Value.Z));
                    aB.SetMode(anAx);
                    break;
                }
                case 3:
                    aB.SetMode(TopoDS.Wire(o[^1]), theCall.Curvilinear);
                    break;
            }
            if (theCall.Transition is { } t && !theCall.TransitionAfterAdd)
            {
                aB.SetTransitionMode(t);
            }
            for (int i = 1; i <= aNbProfiles; i++)
            {
                aB.Add(o[i], false, !theCall.DefaultAdd && theCall.Rotate);
            }
            if (theCall.Transition is { } t2 && theCall.TransitionAfterAdd)
            {
                aB.SetTransitionMode(t2);
            }
            aB.Build();
            if (theCall.MakeSolid)
            {
                aB.MakeSolid();
            }
            return aB.Shape();
        });
    }

    private static BRepBuilderAPI_TransitionMode Transition(B.Transition theT) => theT switch
    {
        B.Transition.Transformed => BRepBuilderAPI_TransitionMode.BRepBuilderAPI_Transformed,
        B.Transition.Round => BRepBuilderAPI_TransitionMode.BRepBuilderAPI_RoundCorner,
        _ => BRepBuilderAPI_TransitionMode.BRepBuilderAPI_RightCorner,
    };

    /// <summary>Solid.Sweep of a face or wire (with its holes): one MakePipeShell per wire, as SweepCore.</summary>
    private void SolidSweep(string theName, B.Shape theSection, B.Shape thePath, bool theFrenet = false, object? theMode = null,
                            B.Transition theTransition = B.Transition.Transformed)
    {
        List<B.Wire> aWires = theSection is B.Face aF ? [aF.OuterWire(), .. aF.InnerWires()] : [(B.Wire)theSection];
        B.Wire aPath = thePath as B.Wire ?? new B.Wire((B.Edge)thePath);
        for (int i = 0; i < aWires.Count; i++)
        {
            string aName = $"{theName} (wire {i})";
            SweepCall aCall = new(1, theFrenet, Transition: Transition(theTransition), MakeSolid: true);
            switch (theMode)
            {
                case B.Vector v:
                    // build123d.NET's SetSweepMode: a frame at the path's start, along the vector; the profile rotated
                    Sweep(aName, aPath, [aWires[i]], aCall with { Mode = 2, Start = ((B.Shape1D)thePath).StartPoint(), Dir = v, Rotate = true });
                    break;
                case B.Wire w:
                    Sweep(aName, aPath, [aWires[i]], aCall with { Mode = 3, Curvilinear = true }, w);
                    break;
                case B.Edge e:
                    Sweep(aName, aPath, [aWires[i]], aCall with { Mode = 3, Curvilinear = true }, new B.Wire(e));
                    break;
                default:
                    Sweep(aName, aPath, [aWires[i]], aCall);
                    break;
            }
        }
    }

    /// <summary>Solid.SweepMulti of the profiles' outer wires.</summary>
    private void SweepMulti(string theName, IEnumerable<B.Shape> theProfiles, B.Shape thePath, bool theFrenet = false, object? theBinormal = null)
    {
        List<B.Shape> aWires = [.. theProfiles.Select(p => p is B.Face f ? (B.Shape)f.OuterWire() : p)];
        B.Wire aPath = thePath as B.Wire ?? new B.Wire((B.Edge)thePath);
        SweepCall aCall = new(1, theFrenet, MakeSolid: true);
        switch (theBinormal)
        {
            case B.Vector v:
                Sweep(theName, aPath, aWires, aCall with { Mode = 2, Start = ((B.Shape1D)thePath).StartPoint(), Dir = v, Rotate = true });
                break;
            case B.Wire w:
                Sweep(theName, aPath, aWires, aCall with { Mode = 3, Curvilinear = true }, w);
                break;
            default:
                Sweep(theName, aPath, aWires, aCall);
                break;
        }
    }

    /// <summary>Shell.Sweep: the profile's edges as a wire along the path's edges ordered into a wire.</summary>
    private void ShellSweep(string theName, B.Shape theProfile, B.Shape thePath, B.Transition theTransition = B.Transition.Transformed)
    {
        var aProfile = new B.Wire(theProfile.Edges());
        var aPath = new B.Wire(new B.Wire(thePath.Edges()).OrderEdges());
        Sweep(theName, aPath, [aProfile], new SweepCall(0, Transition: Transition(theTransition), TransitionAfterAdd: true));
    }

    /// <summary>The Sweep operation: faces through Solid.Sweep or SweepMulti, curves through Shell.Sweep.</summary>
    private void SweepOperation(string theName, IReadOnlyList<B.Shape> theSections, B.Shape thePath, bool theMultisection = false,
                                bool theFrenet = false, B.Transition theTransition = B.Transition.Transformed, B.Vector? theNormal = null,
                                B.Shape1D? theBinormal = null)
    {
        var aPath = thePath as B.Wire ?? new B.Wire(thePath.Edges());
        var aFaces = theSections.Where(s => s is not (B.Curve or B.Wire or B.Edge)).SelectMany(s => s.Faces()).ToList();
        bool hasEdges = theSections.Any(s => s is B.Curve or B.Wire or B.Edge);
        object? aMode = theBinormal switch
        {
            null when theNormal is { } n => n,
            B.Edge e => new B.Wire(e),
            _ => theBinormal,
        };
        if (aFaces.Count > 0)
        {
            if (theMultisection)
            {
                SweepMulti(theName, aFaces, aPath, theFrenet, aMode);
            }
            else
            {
                for (int i = 0; i < aFaces.Count; i++)
                {
                    SolidSweep($"{theName} face {i}", aFaces[i], aPath, theFrenet, aMode, theTransition);
                }
            }
        }
        if (hasEdges)
        {
            for (int i = 0; i < theSections.Count; i++)
            {
                ShellSweep($"{theName} section {i}", theSections[i], aPath, theTransition);
            }
        }
    }

    [Fact]
    public void Sweeps_of_build123d_tests_as_native()
    {
        // OperationsGenericTests' TestSweep
        {
            var aPath = new B.Wire([B.Edge.MakeLine((0, 0, 0), (0, 0, 1)), B.Edge.MakeLine((0, 0, 1), (0, 0, 2))]);
            SweepOperation("SweepReturnIsCleaned", [B.Face.MakeRect(2, 1)], aPath);
        }
        {
            var aPath = B.Edge.MakeSpline([(0, 0, 0), (5, 3, 5), (10, 0, 10)]);
            B.Shape aSection = new B.Plane((0, 0, 0), zDir: aPath.TangentAt(0)) * new B.Rectangle(2, 1);
            SweepOperation("SweepFixedNormal following", [aSection], aPath);
            SweepOperation("SweepFixedNormal fixed", [aSection], aPath, theNormal: (0, 0, 1));
        }
        SweepOperation("SweepSingleSection", [new B.Rectangle(2, 2)], B.Edge.MakeLine((0, 0, 0), (0, 0, 10)));
        {
            const int aSegmentCount = 6;
            var aHandlePath = new B.Wire(new B.Spline([(-10, 0, 0), (0, 0, 5), (10, 0, 0)], tangents: [(0, 0, 1), (0, 0, -1)],
                                                      tangentScalars: [1.5, 1.5]));
            var aSections = new List<B.Shape>();
            for (int i = 0; i <= aSegmentCount; i++)
            {
                var aPlane = new B.Plane(aHandlePath.PositionAt(i / (double)aSegmentCount), zDir: aHandlePath.TangentAt(i / (double)aSegmentCount));
                B.Shape aSection = i % aSegmentCount == 0 ? new B.Circle(1) : B.Operations.Fillet(new B.Rectangle(1, 2).Vertices(), radius: 0.2);
                aSections.Add(aPlane * aSection);
            }
            SweepOperation("SweepMultiSection", aSections, aHandlePath, theMultisection: true);
        }
        {
            var aPath = B.Edge.MakeSpline([(0, 0, 0), (-12, 8, 10)], tangents: [new B.Vector(0, 0, 1), new B.Vector(-1, 0, 0)]);
            var aBinormal = B.Edge.MakeLine((-5, 5), (-8, 10));
            SweepOperation("SweepBinormal", [new B.Rectangle(4, 6)], aPath, theBinormal: aBinormal);
        }
        {
            var anArc = new B.PolarLine((1, 0), 2, 135);
            var anArcPath = new B.PolarLine(anArc.PositionAt(1), 10, 45);
            SweepOperation("SweepEdge", [anArc], anArcPath);
        }
        {
            var aSpine = B.Wire.MakePolygon([(0, 0), (1, 10), (10, 10)], close: false);
            SweepOperation("SweepEdgeAlongWire", [aSpine.PerpendicularLine(2, 0)], aSpine, theTransition: B.Transition.Right);
        }
        {
            double d = 8.5, h = 4.65, lip = 0.5;
            var aSk = B.Operations.Fillet(new B.Rectangle(d * 3, d).Vertices().GroupBy(B.Axis.X).First, d / 2);
            var aP = B.Operations.Extrude(aSk, amount: 4.65);
            var aTopEdges = aP.Edges().GroupBy(B.Axis.Z)[2].SortBy(B.Axis.X)[0..3].SortBy(B.Axis.Y);
            var aSk2 = B.Plane.ZY.Offset(-d * 3 / 2) * (new B.Pos(h, -d / 2) * new B.Rectangle(2 * lip, 2 * lip));
            SweepOperation("SweepPathFromEdges", [aSk2], new B.Wire(aTopEdges));
        }
        SweepOperation("EdgeSweepKeepsShell", [B.Edge.MakeLine((0, 0), (0, 10))], B.Wire.MakePolygon([(0, 0), (10, 0), (10, 0, 10)], close: false));

        // ShellTests
        {
            var aXSection = new B.Rotation(90) * B.Edge.MakeSpline([(0, -5), (-3, -2), (-2, 0), (-3, 2), (0, 5)]);
            ShellSweep("Shell constructor", aXSection, B.Wire.MakeCircle(5));
            var aPathC1 = B.Edge.MakeThreePointArc((0, 0), (-1, -1), (0, -2));
            var aPathC2 = new B.Curve([aPathC1, B.Edge.MakeThreePointArc((0, 0), (Math.Sqrt(2), -2 + Math.Sqrt(2)), (2, -2))]);
            var aPathW = aPathC2.Wires().Single();
            var aSectionE = B.Edge.MakeCircle(0.5);
            var aSectionC2 = new B.Wire([B.Edge.MakeLine((0, 0), (0.1, 0)), B.Edge.MakeLine((0.1, 0), (0.2, 0.1))]);
            ShellSweep("Shell sweep EW", (aPathW ^ 0) * aSectionE, aPathW);
            ShellSweep("Shell sweep WE", (aPathC1 ^ 0) * aSectionC2, aPathC1);
            ShellSweep("Shell sweep WW", (aPathW ^ 0) * aSectionC2, aPathW);
            ShellSweep("Shell sweep C2C1", (aPathC1 ^ 0) * aSectionC2, aPathC1);
            ShellSweep("Shell sweep C2C2", (aPathC2 ^ 0) * aSectionC2, aPathC2);
        }

        // SolidTests
        foreach ((string aName, B.Shape aSection) in new (string, B.Shape)[] { ("face", B.Face.MakeRect(1, 1)), ("wire", B.Wire.MakeRect(1, 1)) })
        {
            // ExtrudeLinearWithRotation: a straight spine and a helical auxiliary one, Add(wire) with the defaults
            B.Vector aCenter = (0, 0, 0), aNormal = (0, 0, 1);
            double anAngle = 45;
            var aStraight = B.Wire.Combine([B.Edge.MakeLine(aCenter, aCenter + aNormal)])[0];
            double aPitch = 360.0 / anAngle * aNormal.Length;
            var anAux = new B.Wire([B.Edge.MakeHelix(aPitch, aNormal.Length, 1, aCenter, aNormal)]);
            B.Wire anOuter = aSection is B.Face f ? f.OuterWire() : (B.Wire)aSection;
            Sweep($"ExtrudeLinearWithRotation {aName}", aStraight, [anOuter], new SweepCall(3, MakeSolid: true, DefaultAdd: true), anAux);
        }
        {
            var aPath = B.Edge.MakeSpline([(0, 0), (3, 5), (7, -2)]);
            SolidSweep("Solid sweep", B.Wire.MakeCircle(1, new B.Plane(aPath.PositionAt(0), zDir: aPath % 0)), aPath);
        }
        SolidSweep("HollowSweep", (B.Face.MakeRect(1, 1) - B.Face.MakeRect(0.1, 0.1)).Faces()[0], B.Edge.MakeLine((0, 0, 0), (0, 0, 5)));
        {
            static B.Edge Spline(B.Vector start, B.Vector end, B.Vector tangent0, B.Vector tangent1) =>
                B.Edge.MakeSpline([start, end], [tangent0.Normalized(), tangent1.Normalized()], scale: true);
            var f0 = B.Face.MakeRect(1, 1);
            var f1 = new B.Pos(x: 10) * new B.Face(B.Wire.MakeCircle(1));
            SweepMulti("SweepMulti binormal edge", [f0, f1], Spline((0, 0), (10, 0), (0, 0, 1), (0, 0, -1)), true,
                       new B.Wire(B.Edge.MakeLine((0, 1), (10, 1))));
            SweepMulti("SweepMulti binormal vector", [f0, f1], Spline((0, 0), (10, 0), (0, 0, 1), (1, 0, 0)), true, new B.Vector(5, 0, 1));
        }
        {
            var aPath = B.Wire.MakePolygon([(0, 20), (20, 20), (20, 0)], close: false);
            SolidSweep("SweepWarnsWhenInvalid folded", new B.Plane((0, 20, 0), zDir: (1, 0, 0)) * B.Face.MakeRect(10, 10), aPath);
            var anArc = B.Edge.MakeCircle(5, startAngle: 0, endAngle: 180);
            SweepMulti("SweepWarnsWhenInvalid multi", [.. new[] { 0.0, 1.0 }.Select(u => (B.Shape)(new B.Plane(anArc.PositionAt(u), zDir: anArc % u) * B.Face.MakeRect(30, 30)))], anArc);
        }
        {
            var aPath = new B.Wire([B.Edge.MakeHelix(2, 6, 3)]);
            SolidSweep("sweep_helix", B.Wire.MakeCircle(0.2, new B.Plane((3, 0, 0), zDir: (0, 1, 0))), aPath, theFrenet: true);
        }

        // FaceTests.Sweep: one edge along one edge, the transition after the profile
        Sweep("Face.Sweep", new B.Wire([B.Wire.MakeCircle(1).SingleEdge()]), [new B.Wire([B.Edge.MakeLine((1, 0), (2, 0))])],
              new SweepCall(0, Transition: BRepBuilderAPI_TransitionMode.BRepBuilderAPI_Transformed, TransitionAfterAdd: true));

        // ShapeTests.ManifoldInconsistentOrientation
        {
            var aPath = B.Wire.MakePolygon([(0, 20, 0), (20, 20, 0), (20, 0, 0)], close: false);
            var aStart = new B.Plane((0, 20, 0), zDir: (1, 0, 0));
            SolidSweep("ManifoldInconsistentOrientation folded", aStart * B.Face.MakeRect(40, 40), aPath, theTransition: B.Transition.Round);
            SolidSweep("ManifoldInconsistentOrientation unfolded", aStart * B.Face.MakeRect(30, 30), aPath, theTransition: B.Transition.Round);
        }

        // EdgeTests.IsInterior
        {
            var aPath = new B.RegularPolygon(5, 5).SingleFace().OuterWire();
            B.Shape aProfile = aPath.LocationAt(0) * (new B.Circle(0.6) & new B.Rectangle(2, 1));
            SweepOperation("IsInterior", [aProfile], aPath, theTransition: B.Transition.Right);
        }

        // OperationsPartTests.ThickenSweptSurface
        {
            var aWire = (B.Shape1D)(new B.JernArc((0, -2), (-1, 0), 1, -180) + new B.JernArc((0, 0), (1, 0), 2, -90));
            SweepOperation("ThickenSweptSurface", [(aWire ^ 0) * new B.RadiusArc((0, 0), (0, -1), 1)], aWire);
        }
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Lofts: BRepOffsetAPI_ThruSections

    /// <summary>TopoUtil.MakeLoft: ThruSections(filled, ruled), vertices as they are and private copies of the wires.</summary>
    private void Loft(string theName, IReadOnlyList<B.Shape> theSections, bool theFilled, bool theRuled = false)
    {
        Compare("ThruSections", theName, Operands([.. theSections]), o =>
        {
            using var aLoft = new N.BRepOffsetAPI_ThruSections(theFilled, theRuled);
            foreach (N.TopoDS_Shape aS in o)
            {
                if (aS.ShapeType() == N.TopAbs_ShapeEnum.TopAbs_VERTEX)
                {
                    aLoft.AddVertex(N.TopoDS.Vertex(aS));
                }
                else
                {
                    using var aCopy = new N.BRepBuilderAPI_Copy(aS, false, false);
                    aLoft.AddWire(N.TopoDS.Wire(aCopy.Shape()));
                }
            }
            aLoft.Build();
            return aLoft.Shape();
        }, o =>
        {
            var aLoft = new BRepOffsetAPI_ThruSections(theFilled, theRuled);
            foreach (TopoDS_Shape aS in o)
            {
                if (aS.ShapeType() == TopAbs_ShapeEnum.TopAbs_VERTEX)
                {
                    aLoft.AddVertex(TopoDS.Vertex(aS));
                }
                else
                {
                    aLoft.AddWire(TopoDS.Wire(new BRepBuilderAPI_Copy(aS, false, false).Shape()));
                }
            }
            aLoft.Build();
            return aLoft.Shape();
        });
    }

    /// <summary>
    /// The Loft operation: the sections' outer wires (and vertices) lofted, then each matched chain of holes, as
    /// OperationsPart.Loft matches them (the first assignment of least total distance).
    /// </summary>
    private void LoftOperation(string theName, IReadOnlyList<B.Shape> theSections, bool theRuled = false)
    {
        var aLoftSections = new List<B.Shape>();
        var aFaces = new List<B.Face>();
        foreach (B.Shape aS in theSections.Where(s => !s.IsEmpty))
        {
            if (aS is B.Vertex)
            {
                aLoftSections.Add(aS);
                continue;
            }
            foreach (B.Face aF in aS.Faces())
            {
                aLoftSections.Add(aF.OuterWire());
                aFaces.Add(aF);
            }
        }
        Loft(theName, aLoftSections, true, theRuled);
        if (aFaces.Count == 0 || aFaces[0].InnerWires().Count == 0)
        {
            return;
        }
        var aHoles = aFaces[0].InnerWires().Select(w => new List<B.Wire> { w }).ToList();
        foreach (B.Face aF in aFaces.Skip(1))
        {
            var anInner = aF.InnerWires();
            int[]? aBest = null;
            double aBestQuality = double.PositiveInfinity;
            foreach (int[] aPerm in Permutations(aHoles.Count))
            {
                double aQuality = 0;
                for (int j = 0; j < aPerm.Length; j++)
                {
                    aQuality += (aHoles[j][^1].Center() - anInner[aPerm[j]].Center()).Length;
                }
                if (aQuality < aBestQuality)
                {
                    (aBestQuality, aBest) = (aQuality, aPerm);
                }
            }
            for (int j = 0; j < aBest!.Length; j++)
            {
                aHoles[j].Add(anInner[aBest[j]]);
            }
        }
        for (int j = 0; j < aHoles.Count; j++)
        {
            Loft($"{theName} hole {j}", aHoles[j], true, theRuled);
        }
    }

    private static IEnumerable<int[]> Permutations(int theN)
    {
        if (theN == 0)
        {
            yield return [];
            yield break;
        }
        // lexicographic order, as itertools.permutations
        int[] a = [.. Enumerable.Range(0, theN)];
        while (true)
        {
            yield return [.. a];
            int i = theN - 2;
            while (i >= 0 && a[i] >= a[i + 1])
            {
                i--;
            }
            if (i < 0)
            {
                yield break;
            }
            int j = theN - 1;
            while (a[j] <= a[i])
            {
                j--;
            }
            (a[i], a[j]) = (a[j], a[i]);
            Array.Reverse(a, i + 1, theN - i - 1);
        }
    }

    [Fact]
    public void Lofts_of_build123d_tests_as_native()
    {
        // SolidTests, ShellTests, TopologyReferenceTests, ShapeTests, FaceTests
        Loft("Solid.MakeLoft", [B.Wire.MakeRect(2, 2), B.Wire.MakeCircle(1, new B.Plane((0, 0, 1)))], true);
        Loft("Solid.MakeLoft with vertices", [new B.Vertex(0, 0, -1), B.Wire.MakeRect(1, 1.5), new B.Vertex(0, 0, 1)], true, true);
        Loft("Shell.MakeLoft", [B.Wire.MakeCircle(3, new B.Plane((0, 0, 2))), B.Wire.MakeCircle(3)], false);
        Loft("loft", [B.Wire.MakeRect(4, 4), B.Wire.MakeCircle(1, B.Plane.XY.Offset(5))], true);
        Loft("loft_ruled", [B.Wire.MakeRect(2, 2), B.Wire.MakeRect(1, 1, B.Plane.XY.Offset(3))], true, true);
        Loft("loft_to_vertex", [B.Wire.MakeRect(2, 2), new B.Vertex(0, 0, 3)], true);
        {
            var aSpine = B.Edge.MakeLine((0, -2, 0), (0, 2, 0));
            double[] aSizes = [1, 2, 2, 1], aTwists = [10, 0, 0, -10], aPositions = [0, 0.33, 0.66, 1];
            var aSections = Enumerable.Range(0, 4)
                .Select(i => (B.Shape)(new B.Plane(aSpine ^ aPositions[i]) * new B.Rotation(0, 0, aTwists[i]) * B.Wire.MakeRect(aSizes[i], aSizes[i])))
                .ToList();
            Loft("SplitCurvedFacesByPlane", aSections, true);
        }
        Loft("IsPlanar mount", [B.Wire.MakeRect(1 + 16 + 4, 20, new B.Plane((10.5, 0, 0))), new B.Pos(1, 0, 4) * B.Wire.MakeRect(16, 20, new B.Plane((8, 0, 0)))], true);

        // OperationsPartTests' TestLoft and OperationsGenericTests.OffsetFailure
        LoftOperation("LoftReturnIsCleaned", [new B.Face(B.Wire.MakePolygon([(0, 0), (1, 0), (2, 0), (2, 1), (0, 1)])),
                                              new B.Pos(0, 0, 1) * B.Face.MakeRect(2, 1, new B.Plane((1, 0.5, 0)))]);
        {
            const int aSliceCount = 10;
            var aSections = Enumerable.Range(0, aSliceCount + 1).Select(i =>
                (B.Shape)new B.Face(new B.Wire([B.Edge.MakeCircle(10 * Math.Sin(i * Math.PI / aSliceCount) + 5).Moved(new B.Location((0, 0, i * 3)))]))).ToList();
            LoftOperation("SimpleLoft", aSections);
        }
        LoftOperation("LoftVertex", [new B.Rectangle(1, 1), new B.Vertex(0, 0, 3)], true);
        LoftOperation("LoftVertices", [new B.Vertex(0, 0, -3), new B.Rectangle(1, 1), new B.Vertex(0, 0, 3)], true);
        {
            var aLower = B.Face.MakeRect(10, 10) - B.Face.MakeRect(8, 8);
            LoftOperation("LoftWithHole", [aLower, new B.Pos(0, 0, 5) * aLower]);
        }
        {
            var aLower = new B.Rectangle(10, 10) - new B.Pos(-2.5) * new B.Circle(1) - new B.Pos(2.5) * new B.Circle(1.5);
            var anUpper = new B.Pos(0, 0, 5) * (new B.Rectangle(10, 10) - new B.Pos(2.5) * new B.Circle(1.5) - new B.Pos(-2.5) * new B.Circle(1));
            LoftOperation("LoftWithTwoHoles", [aLower, anUpper]);
        }
        LoftOperation("OffsetFailure cup", [new B.Circle(35), new B.Pos(0, 0, 50) * new B.Circle(37.62)]);
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Surfaces: BRepOffsetAPI_MakeFilling, GeomAPI_PointsToBSplineSurface, BRepFill::Face and Shell

    /// <summary>
    /// Face.MakeSurface: a filling of build123d.NET's parameters through C0 edges, built; then the points added and built
    /// again. Both builds' faces are compared (as one compound).
    /// </summary>
    private void MakeSurface(string theName, IReadOnlyList<B.Edge> theEdges, IReadOnlyList<B.Vector> thePoints)
    {
        Compare("MakeFilling", theName, Operands([.. theEdges]), o =>
        {
            using var aF = new N.BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
            foreach (N.TopoDS_Shape anE in o)
            {
                aF.Add(N.TopoDS.Edge(anE), N.GeomAbs_Shape.GeomAbs_C0);
            }
            aF.Build();
            N.TopoDS_Shape aFirst = aF.Shape();
            if (thePoints.Count == 0)
            {
                return aFirst;
            }
            foreach (B.Vector p in thePoints)
            {
                using var aP = new N.gp_Pnt(p.X, p.Y, p.Z);
                aF.Add(aP);
            }
            aF.Build();
            return NCompound(aFirst, aF.Shape());
        }, o =>
        {
            var aF = new BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
            foreach (TopoDS_Shape anE in o)
            {
                aF.Add(TopoDS.Edge(anE), GeomAbs_Shape.GeomAbs_C0);
            }
            aF.Build();
            TopoDS_Shape aFirst = aF.Shape();
            if (thePoints.Count == 0)
            {
                return aFirst;
            }
            foreach (B.Vector p in thePoints)
            {
                aF.Add(new gp_Pnt(p.X, p.Y, p.Z));
            }
            aF.Build();
            return PCompound(aFirst, aF.Shape());
        });
    }

    /// <summary>Face.MakeSurfacePatch: a default filling, edges on support faces (G1), plain edges (C0), then points.</summary>
    private void MakeSurfacePatch(string theName, IReadOnlyList<(B.Edge Edge, B.Face Face)> theOnFaces, IReadOnlyList<B.Edge> theEdges,
                                  IReadOnlyList<B.Vector> thePoints)
    {
        B.Shape[] anOps = [.. theOnFaces.SelectMany(p => new B.Shape[] { p.Edge, p.Face }), .. theEdges];
        int aNbOnFaces = theOnFaces.Count;
        Compare("MakeFilling", theName, Operands(anOps), o =>
        {
            using var aF = new N.BRepOffsetAPI_MakeFilling();
            for (int i = 0; i < aNbOnFaces; i++)
            {
                aF.Add(N.TopoDS.Edge(o[2 * i]), N.TopoDS.Face(o[2 * i + 1]), N.GeomAbs_Shape.GeomAbs_G1);
            }
            foreach (N.TopoDS_Shape anE in o.Skip(2 * aNbOnFaces))
            {
                aF.Add(N.TopoDS.Edge(anE), N.GeomAbs_Shape.GeomAbs_C0);
            }
            foreach (B.Vector p in thePoints)
            {
                using var aP = new N.gp_Pnt(p.X, p.Y, p.Z);
                aF.Add(aP);
            }
            aF.Build();
            return aF.Shape();
        }, o =>
        {
            var aF = new BRepOffsetAPI_MakeFilling();
            for (int i = 0; i < aNbOnFaces; i++)
            {
                aF.Add(TopoDS.Edge(o[2 * i]), TopoDS.Face(o[2 * i + 1]), GeomAbs_Shape.GeomAbs_G1);
            }
            foreach (TopoDS_Shape anE in o.Skip(2 * aNbOnFaces))
            {
                aF.Add(TopoDS.Edge(anE), GeomAbs_Shape.GeomAbs_C0);
            }
            foreach (B.Vector p in thePoints)
            {
                aF.Add(new gp_Pnt(p.X, p.Y, p.Z));
            }
            aF.Build();
            return aF.Shape();
        });
    }

    /// <summary>Face.MakeSurfaceFromArrayOfPoints: the approximation, as a face on the surface (MakeFace, Precision::Confusion).</summary>
    private void SurfaceFromPoints(string theName, B.Vector[][] thePoints, (double, double, double)? theSmoothing = null, int theMinDeg = 1,
                                   int theMaxDeg = 3)
    {
        int aRows = thePoints.Length, aCols = thePoints[0].Length;
        Compare("PointsToBSplineSurface", theName, Operands(), _ =>
        {
            using var anArray = new N.TColgp_Array2OfPnt(1, aRows, 1, aCols);
            for (int i = 0; i < aRows; i++)
            {
                for (int j = 0; j < aCols; j++)
                {
                    using var aP = new N.gp_Pnt(thePoints[i][j].X, thePoints[i][j].Y, thePoints[i][j].Z);
                    anArray.SetValue(i + 1, j + 1, aP);
                }
            }
            using N.GeomAPI_PointsToBSplineSurface aB = theSmoothing is var (w1, w2, w3)
                ? new(anArray, w1, w2, w3, Math.Max(theMaxDeg, 6), N.GeomAbs_Shape.GeomAbs_C2, 1e-2)
                : new(anArray, theMinDeg, theMaxDeg, N.GeomAbs_Shape.GeomAbs_C2, 1e-2);
            if (!aB.IsDone())
            {
                throw new N.OcctException("StdFail_NotDone", "");
            }
            using var aMaker = new N.BRepBuilderAPI_MakeFace(aB.Surface(), N.Precision.Confusion());
            return aMaker.Face();
        }, _ =>
        {
            var anArray = new NCollection_Array2<gp_Pnt>(1, aRows, 1, aCols);
            for (int i = 0; i < aRows; i++)
            {
                for (int j = 0; j < aCols; j++)
                {
                    anArray.SetValue(i + 1, j + 1, new gp_Pnt(thePoints[i][j].X, thePoints[i][j].Y, thePoints[i][j].Z));
                }
            }
            GeomAPI_PointsToBSplineSurface aB = theSmoothing is var (w1, w2, w3)
                ? new(anArray, w1, w2, w3, Math.Max(theMaxDeg, 6), GeomAbs_Shape.GeomAbs_C2, 1e-2)
                : new(anArray, theMinDeg, theMaxDeg, GeomAbs_Shape.GeomAbs_C2, 1e-2);
            if (!aB.IsDone())
            {
                throw new StdFail_NotDone();
            }
            return new BRepBuilderAPI_MakeFace(aB.Surface(), Precision.Confusion()).Face();
        });
    }

    private void RuledFace(string theName, B.Edge theE1, B.Edge theE2) =>
        Compare("BRepFill", theName, Operands(theE1, theE2), o => N.BRepFill.Face(N.TopoDS.Edge(o[0]), N.TopoDS.Edge(o[1])),
                o => BRepFill.Face(TopoDS.Edge(o[0]), TopoDS.Edge(o[1])));

    private void RuledShell(string theName, B.Wire theW1, B.Wire theW2) =>
        Compare("BRepFill", theName, Operands(theW1, theW2), o => N.BRepFill.Shell(N.TopoDS.Wire(o[0]), N.TopoDS.Wire(o[1])),
                o => BRepFill.Shell(TopoDS.Wire(o[0]), TopoDS.Wire(o[1])));

    private static B.Wire NetExterior()
    {
        B.Vector[] aCorners = [.. from x in new[] { -50.5, 50.5 } from y in new[] { -24.5, 24.5 } select new B.Vector(x, y)];
        return new B.Wire(
        [
            B.Edge.MakeLine(aCorners[3], aCorners[1]),
            B.Edge.MakeLine(aCorners[1], aCorners[0]),
            B.Edge.MakeLine(aCorners[0], aCorners[2]),
            B.Edge.MakeThreePointArc(aCorners[2], (aCorners[2] + aCorners[3]) / 2 - new B.Vector(0, 0, 3), aCorners[3]),
        ]);
    }

    [Fact]
    public void Surfaces_of_build123d_tests_as_native()
    {
        // FaceTests.MakeSurface and MakeSurfaceErrorChecking
        var aNet = NetExterior();
        MakeSurface("MakeSurface with a point", aNet.Edges(), [new B.Vector(0, 0, -5)]);
        MakeSurface("MakeSurface", aNet.Edges(), []);
        MakeSurface("MakeSurface circle", [B.Edge.MakeCircle(50)], [(0, 0, -5)]);
        MakeSurface("MakeSurface line", [B.Edge.MakeLine((0, 0), (1, 0))], []);
        MakeSurface("MakeSurface two points", [B.Edge.MakeCircle(50)], [(0, 0, -50), (0, 0, 50)]);

        // FaceTests.MakeSurfacePatch
        {
            var m1 = B.Edge.MakeSpline([(0, 0), (1, 0), (10, 0, -10)]);
            var m2 = B.Edge.MakeSpline([(0, 0), (0, 1), (0, 10, -10)]);
            var m3 = B.Edge.MakeSpline([m1.PositionAt(1), (7, 7, -10), m2.PositionAt(1)]);
            MakeSurfacePatch("MakeSurfacePatch edges", [], [m1, m2, m3], []);
            var f1 = B.Face.Extrude(m1, (0, -1, 0));
            var f2 = B.Face.Extrude(m2, (-1, 0, 0));
            var f3 = B.Face.Extrude(m3, (0, 0, -1));
            MakeSurfacePatch("MakeSurfacePatch faces", [(m1, f1), (m2, f2), (m3, f3)], [], []);
            MakeSurfacePatch("MakeSurfacePatch mixed", [(m1, f1), (m2, f2)], [m3], []);
            // the point is the first patch's middle moved: native build123d.NET's own result decides it
            var aPatch = B.Face.MakeSurfacePatch(edgeConstraints: [m1, m2, m3]);
            MakeSurfacePatch("MakeSurfacePatch point", [], [m1, m2, m3], [aPatch.PositionAt(0.5, 0.5) + new B.Vector(0.5, 0.5)]);
            MakeSurfacePatch("MakeSurfacePatch two lines", [], [B.Edge.MakeLine((0, 0), (1, 0)), B.Edge.MakeLine((0, 0), (0, 1))], []);
        }

        // FaceTests' arrays of points, SurfaceFromArrayOfPointsTests
        B.Vector[][] aWave = [.. Enumerable.Range(0, 11).Select(y => Enumerable.Range(0, 11)
            .Select(x => new B.Vector(x, y, Math.Cos(Math.PI * x / 10) + Math.Sin(Math.PI * y / 10))).ToArray())];
        SurfaceFromPoints("WavePoints", aWave);
        B.Vector[][] aGrid = [.. Enumerable.Range(0, 6).Select(y => Enumerable.Range(0, 6)
            .Select(x => new B.Vector(x, y, Math.Sin(x / 3.0) * Math.Cos(y / 3.0))).ToArray())];
        SurfaceFromPoints("Grid plain", aGrid);
        SurfaceFromPoints("Grid smoothed 1 1 1", aGrid, (1.0, 1.0, 1.0));
        SurfaceFromPoints("Grid smoothed 1 5 10", aGrid, (1.0, 5.0, 10.0));
        SurfaceFromPoints("Grid smoothed 0.1 1 10", aGrid, (0.1, 1.0, 10.0));
        SurfaceFromPoints("Grid smoothed maxDeg 8", aGrid, (1.0, 1.0, 1.0), theMaxDeg: 8);

        // FaceTests.MakeSurfaceFromCurves, EdgeTests.ConvexitySaddle
        RuledFace("MakeSurfaceFromCurves edges", B.Edge.MakeCircle(1, endAngle: 90), B.Edge.MakeCircle(1, new B.Plane((0, 0, 1)), endAngle: 90));
        RuledShell("MakeSurfaceFromCurves wires", B.Wire.MakeCircle(1), B.Wire.MakeCircle(1, new B.Plane((0, 0, 1))));
        RuledFace("ConvexitySaddle", B.Edge.MakeLine((-10, 0, 0), (10, 0, 0)),
                  B.Edge.MakeSpline([.. Enumerable.Range(0, 11).Select(i => -10 + 2 * i).Select(x =>
                      new B.Vector(x, 5 * Math.Cos(Math.PI / 2 * x / 10), 5 * Math.Sin(Math.PI / 2 * x / 10)))]));
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Projections: BRepProj_Projection

    /// <summary>The projection of a wire or edge onto a shape, along a direction (its components as given) or from a point.</summary>
    private void Projection(string theName, B.Shape theWire, B.Shape theTarget, B.Vector? theDir, B.Vector? theCenter = null)
    {
        Compare("BRepProj_Projection", theName, Operands(theWire, theTarget), o =>
        {
            using N.BRepProj_Projection aP = theDir is { } d
                ? new(o[0], o[1], new N.gp_Dir(d.X, d.Y, d.Z))
                : new(o[0], o[1], new N.gp_Pnt(theCenter!.Value.X, theCenter.Value.Y, theCenter.Value.Z));
            return aP.Shape();
        }, o =>
        {
            BRepProj_Projection aP = theDir is { } d
                ? new(o[0], o[1], new gp_Dir(d.X, d.Y, d.Z))
                : new(o[0], o[1], new gp_Pnt(theCenter!.Value.X, theCenter.Value.Y, theCenter.Value.Z));
            return aP.Shape();
        });
    }

    /// <summary>Wire.ProjectToShape (and Edge's, through a one-edge wire): the direction normalised first.</summary>
    private void ProjectToShape(string theName, B.Shape1D theCurve, B.Shape theTarget, B.Vector? theDir, B.Vector? theCenter = null)
    {
        B.Wire aWire = theCurve as B.Wire ?? new B.Wire([(B.Edge)theCurve]);
        if (theDir is { } d)
        {
            Projection(theName, aWire, theTarget, d.Normalized());
        }
        else
        {
            Projection(theName, aWire, theTarget, null, theCenter);
        }
    }

    [Fact]
    public void Projections_of_build123d_tests_as_native()
    {
        // ProjectionTests
        {
            static B.Part Box(double l, double w, double h) => new([B.Solid.MakeBox(l, w, h).Moved(new B.Location((-l / 2, -w / 2, -h / 2)))]);
            var aTarget = Box(10, 10, 4) - new B.Pos(0, 0, 2) * Box(5, 5, 2);
            ProjectToShape("MultipleOutputWires", B.Wire.MakeCircle(3, B.Plane.XY.Offset(10)), aTarget, (0, 0, -1));
        }
        ProjectToShape("ProjectEdge", B.Edge.MakeCircle(1, B.Plane.XY.Offset(-5)), B.Solid.MakeBox(1, 1, 1), (0, 0, 1));
        {
            double aRadiusX = 45, aRadiusY = 30, aWidth = 25, anOpeningAngle = 80;
            var aCenterArc = B.Edge.MakeEllipse(aRadiusX, aRadiusY, startAngle: 270 + anOpeningAngle / 2, endAngle: 270 - anOpeningAngle / 2);
            var aCenterSurface = (B.Face)(-B.Face.Extrude(aCenterArc, (0, 0, 2 * aWidth)).Moved(new B.Location((0, 0, -aWidth), (0, 0, 180))));
            var aTipCenterLoc = aCenterSurface.LocationAt(aCenterArc.PositionAt(1), xDir: (1, 0, 0));
            var aNormal = aTipCenterLoc.ZAxis.Direction;
            var aPlanarTipArc = B.Edge.MakeCircle(aWidth / 2, startAngle: 270, endAngle: 450).Located(aTipCenterLoc);
            ProjectToShape("ProjectCleanWire", aPlanarTipArc, aCenterSurface, -aNormal);
        }

        // FaceTests: the hexagon wrapped round a cylinder (from a centre), the hole projected onto a filling
        {
            double aRadius = 10;
            double aCircumference = 2 * Math.PI * aRadius;
            double aHexDiagonal = 4 * (aCircumference / 10) / 3;
            var aCylinder = B.Solid.MakeCylinder(aRadius, aHexDiagonal * 5);
            double r = aHexDiagonal * 0.4;
            var aHex = B.Wire.MakePolygon(Enumerable.Range(0, 6)
                .Select(i => new B.Vector(r * Math.Cos(i * Math.PI / 3), aHexDiagonal + r * Math.Sin(i * Math.PI / 3))));
            var aHexWire = (B.Plane.XZ.Offset(aRadius) * new B.Face(aHex)).OuterWire();
            ProjectToShape("hexagon onto a cylinder", aHexWire, aCylinder, null, (0, 0, aHexWire.Center().Z));
        }
        {
            var aSurface = B.Face.MakeSurface(NetExterior(), surfacePoints: [new B.Vector(0, 0, -5)]);
            ProjectToShape("MakeSurface hole", B.Wire.MakeCircle(10), aSurface, (0, 0, -1));
        }

        // Shape1DTests.Project and Project2: the direction as given
        {
            var aTarget = B.Face.MakeRect(10, 10, B.Plane.XY.Rotated((0, 45, 0)));
            var aCircle = B.Edge.MakeCircle(1).Located(new B.Location((0, 0, 10)));
            Projection("Shape1D.Project", aCircle, aTarget, (0, 0, -1));
        }
        {
            var aTarget = B.Solid.MakeCylinder(1, 10, new B.Plane((0, 0, -5))).Faces().FilterBy(B.GeomType.Plane, reverse: true)[0];
            var aSquare = B.Wire.MakeRect(1, 1, B.Plane.YZ).Located(new B.Location((10, 0, 0)));
            Projection("Shape1D.Project2", aSquare, aTarget, (-1, 0, 0));
        }
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // 2D chamfers: BRepFilletAPI_MakeFillet2d

    /// <summary>
    /// Wire.Chamfer2D: the wire made a face (build123d.NET's face from wires), each given vertex's two edges in
    /// MapShapesAndAncestors' order (the reference edge first when given), AddChamfer, Build.
    /// </summary>
    private void Chamfer2D(string theName, B.Wire theWire, IReadOnlyList<B.Vertex> theVertices, double theD1, double theD2, B.Edge? theReference = null)
    {
        var aFace = new B.Face(theWire);
        B.Shape[] anOps = [aFace, .. theVertices, .. (theReference is null ? Array.Empty<B.Shape>() : [theReference])];
        int aNbV = theVertices.Count;
        bool hasReference = theReference is not null;
        Compare("MakeFillet2d", theName, Operands(anOps), o =>
        {
            using var aB = new N.BRepFilletAPI_MakeFillet2d(N.TopoDS.Face(o[0]));
            using var aMap = new N.TopTools_IndexedDataMapOfShapeListOfShape();
            N.TopExp.MapShapesAndAncestors(o[0], N.TopAbs_ShapeEnum.TopAbs_VERTEX, N.TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
            for (int i = 1; i <= aNbV; i++)
            {
                using N.TopTools_ListOfShape aList = aMap.FindFromKey(o[i]);
                N.TopoDS_Shape e1 = aList.First(), e2 = aList.Last();
                if (hasReference && e2.IsSame(o[^1]))
                {
                    (e1, e2) = (e2, e1);
                }
                aB.AddChamfer(N.TopoDS.Edge(e1), N.TopoDS.Edge(e2), theD1, theD2);
            }
            aB.Build();
            return aB.Shape();
        }, o =>
        {
            var aB = new BRepFilletAPI_MakeFillet2d(TopoDS.Face(o[0]));
            var aMap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(o[0], TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
            for (int i = 1; i <= aNbV; i++)
            {
                NCollection_List<TopoDS_Shape> aList = aMap.FindFromKey(o[i]);
                TopoDS_Shape e1 = aList.First(), e2 = aList.Last();
                if (hasReference && e2.IsSame(o[^1]))
                {
                    (e1, e2) = (e2, e1);
                }
                aB.AddChamfer(TopoDS.Edge(e1), TopoDS.Edge(e2), theD1, theD2);
            }
            aB.Build();
            return aB.Shape();
        });
    }

    /// <summary>The Chamfer operation on a sketch: each face's wires chamfered at the given vertices on them (Face.EditWires).</summary>
    private void SketchChamfer(string theName, B.Shape theSketch, IEnumerable<B.Vertex> theVertices, double theLength, double? theLength2 = null,
                               double? theAngle = null, B.Edge? theReference = null)
    {
        double? aLength2 = theAngle is { } a && a != 0 ? theLength * Math.Tan(a * DegToRad) : theLength2;
        double aDistance2 = aLength2 ?? theLength;
        var aList = theVertices.ToList();
        int k = 0;
        foreach (B.Face aFace in theSketch.Faces())
        {
            var anOnFace = aFace.Vertices().Where(aList.Contains).ToList();
            if (anOnFace.Count == 0)
            {
                continue;
            }
            foreach (B.Wire aWire in new[] { aFace.OuterWire() }.Concat(aFace.InnerWires()))
            {
                var aWireVertices = aWire.Vertices();
                var anInWire = anOnFace.Where(v => aWireVertices.Any(wv => wv.Wrapped.IsSame(v.Wrapped))).ToList();
                if (anInWire.Count > 0)
                {
                    Chamfer2D($"{theName} {k++}", aWire, anInWire, theLength, aDistance2, theReference);
                }
            }
        }
    }

    [Fact]
    public void Chamfers_2d_of_build123d_tests_as_native()
    {
        SketchChamfer("Chamfer2D", new B.Rectangle(1, 2), new B.Rectangle(1, 2).Vertices(), 0.2);
        {
            var r = new B.Rectangle(1, 2);
            SketchChamfer("Chamfer2D same rectangle", r, r.Vertices(), 0.2);
            var aTen = new B.Rectangle(10, 10);
            SketchChamfer("SketchChamfer", aTen, aTen.Vertices(), 1);
            var aTwo = new B.Pos(-10, 0) * new B.Rectangle(10, 10) + new B.Pos(10, 0) * new B.Rectangle(10, 10);
            SketchChamfer("SketchChamfer two", aTwo, aTwo.Vertices().FilterByPosition(B.Axis.X, minimum: 0, maximum: 20), 1);
            SketchChamfer("SketchChamferAsymLength", aTen, aTen.Vertices(), 1, Math.Sqrt(3));
            SketchChamfer("SketchChamferAsymAngle", aTen, aTen.Vertices(), 1, theAngle: 60);
            var anEdge = aTen.Edges().SortBy(B.Axis.Y)[0];
            SketchChamfer("SketchChamferAsymAngleReference", aTen, [anEdge.Vertices()[0]], 1, theAngle: 60, theReference: anEdge);
            SketchChamfer("SketchChamferAsymLengthReference", aTen, [anEdge.Vertices()[0]], 1, Math.Sqrt(3), theReference: anEdge);
        }
        {
            var aSquare = new B.Rectangle(20, 20) - new B.Rectangle(5, 5);
            SketchChamfer("SketchChamferWithHole", aSquare, aSquare.Vertices().GroupBy(B.Axis.Y).Last, 2);
        }
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // 2D offsets: BRepOffsetAPI_MakeOffset

    /// <summary>
    /// Shape1D.Offset2D's MakeOffset: the plane's normal and the signed distance as build123d.NET settles them (a closed
    /// wire's side by its winding), a one-edge wire split in two halves, then MakeOffset initialised with the join (on a
    /// plane's face for a straight line with a known plane), AddWire, Perform(distance). The end arcs it removes and the
    /// B-spline conversion are build123d.NET's own.
    /// </summary>
    private void Offset2D(string theName, B.Shape1D theCurve, double theDistance, B.Kind theKind = B.Kind.Arc, B.Side theSide = B.Side.Both,
                          B.Plane? thePlane = null)
    {
        var aJoin = theKind switch
        {
            B.Kind.Arc => GeomAbs_JoinType.GeomAbs_Arc,
            B.Kind.Intersection => GeomAbs_JoinType.GeomAbs_Intersection,
            _ => GeomAbs_JoinType.GeomAbs_Tangent,
        };
        var aLine = theCurve as B.Wire ?? new B.Wire([(B.Edge)theCurve]);
        var anEdges = aLine.Edges();
        var aStart = aLine.PositionAt(0);
        var aTangent = aLine.TangentAt(0);
        var aStartAxis = new B.Axis(aStart, aTangent);
        bool isStraight = anEdges.All(e => e.GeomType == B.GeomType.Line && new B.Axis(e.PositionAt(0), e.TangentAt(0)).IsCoaxial(aStartAxis));
        B.Vector[] aReferences = [theCurve.Location.ZAxis.Direction, anEdges[0].Location.ZAxis.Direction, new B.Vector(0, 0, 1)];
        B.Vector? aNormal = null;
        if (thePlane is not null)
        {
            aNormal = thePlane.ZDir;
        }
        else if (isStraight)
        {
            aNormal = aReferences.Cast<B.Vector?>().FirstOrDefault(r => Math.Abs(r!.Value.Dot(aTangent)) < B.Constants.Tolerance);
        }
        else if (aLine.CommonPlane() is { } anOwn)
        {
            aNormal = anOwn.ZDir;
            foreach (B.Vector r in aReferences)
            {
                if (Math.Abs(anOwn.ZDir.Dot(r)) > 1 - B.Constants.Tolerance)
                {
                    aNormal = r;
                    break;
                }
            }
        }
        double aSigned = theDistance;
        if (theSide != B.Side.Both && aLine.IsClosed && aNormal is { } n)
        {
            int aSteps = Math.Max(16, 8 * anEdges.Count);
            var aPoints = Enumerable.Range(0, aSteps).Select(i => aLine.PositionAt(i / (double)aSteps)).ToList();
            double aTwiceArea = 0;
            for (int i = 1; i < aSteps - 1; i++)
            {
                aTwiceArea += (aPoints[i] - aPoints[0]).Cross(aPoints[i + 1] - aPoints[0]).Dot(n);
            }
            bool isOutwards = (aTwiceArea < 0) == (theSide == B.Side.Left);
            aSigned = isOutwards ? Math.Abs(theDistance) : -Math.Abs(theDistance);
        }
        B.Face? aSpine = isStraight && aNormal is { } sn ? B.Face.FromPlane(new B.Plane(aStart, zDir: sn)) : null;
        double aDistance = aSigned;
        B.Wire aWire = anEdges.Count == 1 ? new B.Wire([anEdges[0].Trim(0.0, 0.5), anEdges[0].Trim(0.5, 1.0)]) : aLine;
        B.Shape[] anOps = aSpine is null ? [aWire] : [aWire, aSpine];
        Compare("MakeOffset", theName, Operands(anOps), o =>
        {
            using var aB = new N.BRepOffsetAPI_MakeOffset();
            if (o.Count > 1)
            {
                aB.Init(N.TopoDS.Face(o[1]), (N.GeomAbs_JoinType)(int)aJoin);
            }
            else
            {
                aB.Init((N.GeomAbs_JoinType)(int)aJoin);
            }
            aB.AddWire(N.TopoDS.Wire(o[0]));
            aB.Perform(aDistance);
            return aB.Shape();
        }, o =>
        {
            var aB = new BRepOffsetAPI_MakeOffset();
            if (o.Count > 1)
            {
                aB.Init(TopoDS.Face(o[1]), aJoin);
            }
            else
            {
                aB.Init(aJoin);
            }
            aB.AddWire(TopoDS.Wire(o[0]));
            aB.Perform(aDistance);
            return aB.Shape();
        });
    }

    // build123d's Polyline: lines between the points, combined into a wire
    private static B.Wire Polyline(params B.Vector[] thePoints) =>
        B.Wire.Combine([.. Enumerable.Range(0, thePoints.Length - 1).Select(i => (B.Shape1D)B.Edge.MakeLine(thePoints[i], thePoints[i + 1]))])[0];

    /// <summary>The Offset operation on a sketch's faces: the outer wire by the amount, each inner one by its opposite.</summary>
    private void FaceOffset(string theName, B.Shape theSketch, double theAmount, B.Kind theKind = B.Kind.Arc)
    {
        int k = 0;
        foreach (B.Face aFace in theSketch.Faces())
        {
            Offset2D($"{theName} outer {k}", aFace.OuterWire(), theAmount, theKind);
            int i = 0;
            foreach (B.Wire anInner in aFace.InnerWires())
            {
                Offset2D($"{theName} inner {k}.{i++}", anInner, -theAmount, theKind);
            }
            k++;
        }
    }

    [Fact]
    public void Offsets_2d_of_build123d_tests_as_native()
    {
        // OperationsGenericTests' OffsetTests
        Offset2D("SingleLineOffset", B.Edge.MakeLine((0, 0), (1, 0)), 1);
        Offset2D("SingleLineOffsetStaysInPlane", (B.Shape1D)(B.Plane.XZ * B.Edge.MakeLine((0, 0), (10, 10))), 1);
        {
            var l1 = B.Edge.MakeLine((0, 0), (1, 0));
            var l2 = B.Edge.MakeLine(l1.PositionAt(1), (1, 1));
            Offset2D("LineOffset", new B.Wire([l1, l2]), 1);
        }
        FaceOffset("FaceOffset", new B.Rectangle(1, 1), 1, B.Kind.Intersection);
        FaceOffset("FaceOffsetWithHoles", new B.Rectangle(100, 100) - new B.GridLocations(80, 80, 2, 2) * new B.Circle(5), -5);
        Offset2D("OffsetCurve", B.Edge.MakeBezier((0, 0), (1, 5), (0, 10)), 1);
        FaceOffset("OffsetFaceWithInnerWire", new B.Rectangle(1, 1) - new B.RegularPolygon(0.25, 3), 0.125);
        FaceOffset("OffsetFaceWithMinLength", new B.Circle(0.5), 0.125);
        FaceOffset("OffsetFaceWithMinLengthAndInner", new B.Circle(0.5) - new B.RegularPolygon(0.25, 3), 0.125);

        // Shape1DTests, WireTests and TopologyReferenceTests
        foreach (B.Plane aPlane in new[] { B.Plane.XY, B.Plane.XZ, B.Plane.YZ, B.Plane.XY.Rotated((30, 20, 0)).Offset(5) })
        {
            var aCorner = (B.Wire)(aPlane * Polyline((0, 0), (10, 0), (10, 8)));
            Offset2D("Offset2DSidesOnAnyPlane left", aCorner, 1, theSide: B.Side.Left);
            Offset2D("Offset2DSidesOnAnyPlane right", aCorner, 1, theSide: B.Side.Right);
        }
        foreach (B.Plane aPlane in new[] { B.Plane.XY, B.Plane.XZ })
        {
            foreach (B.Wire aWire in new[] { Polyline((0, 0), (10, 0), (10, 8), (0, 8), (0, 0)), Polyline((0, 0), (0, 8), (10, 8), (10, 0), (0, 0)) })
            {
                var aPlaced = (B.Wire)(aPlane * aWire);
                Offset2D("Offset2DClosedWireSides left", aPlaced, 0.25, theSide: B.Side.Left);
                Offset2D("Offset2DClosedWireSides right", aPlaced, 0.25, theSide: B.Side.Right);
            }
        }
        {
            var anArc = B.Edge.MakeCircle(5, new B.Plane((0, 0)), 0, 90, B.AngularDirection.CounterClockwise);
            Offset2D("Offset2DCollapsedSide right", anArc, 8, theSide: B.Side.Right);
            Offset2D("Offset2DCollapsedSide left", anArc, 8, theSide: B.Side.Left);
        }
        Offset2D("Offset2DStraightLine around", (B.Shape1D)(B.Plane.XZ * B.Edge.MakeLine((0, 0), (10, 10))), 1);
        Offset2D("Offset2DStraightLine left", (B.Shape1D)(B.Plane.XZ * B.Edge.MakeLine((0, 0), (10, 0))), 1, theSide: B.Side.Left);
        {
            var anInSpace = B.Edge.MakeLine((0, 0, 0), (10, 0, 10));
            Offset2D("Offset2DStraightLine given plane", anInSpace, 1, thePlane: B.Plane.XZ);
            Offset2D("Offset2DStraightLine in space", anInSpace, 1, theSide: B.Side.Left);
        }
        Offset2D("WireTests three-point arc", B.Edge.MakeThreePointArc((0, -20), (5, 0), (0, 20)), 10, theSide: B.Side.Right);
        Offset2D("offset_wire", B.Wire.MakeRect(4, 2), 0.5);
        Offset2D("offset_wire_intersection", B.Wire.MakeRect(4, 2), -0.5, B.Kind.Intersection);
        Offset2D("offset_arc", B.Edge.MakeCircle(3, startAngle: 0, endAngle: 120), 0.5);
        Offset2D("offset_arc_left", B.Edge.MakeCircle(3, startAngle: 0, endAngle: 120), 0.5, theSide: B.Side.Left);
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Constrained arcs and lines: the Geom2dGcc solvers

    /// <summary>A target of a constrained arc or line: an edge (with its qualifier) or a point of XY.</summary>
    private sealed record Target(B.Edge? Edge, B.Tangency Tangency, double X = 0, double Y = 0)
    {
        public static implicit operator Target(B.Edge theE) => new(theE, B.Tangency.Unqualified);
        public static implicit operator Target((double X, double Y) theP) => new(null, B.Tangency.Unqualified, theP.X, theP.Y);
        public static implicit operator Target((B.Edge E, B.Tangency T) theT) => new(theT.E, theT.T);
        public static implicit operator Target(B.Axis theA) => new(B.Edge.FromAxis(theA), B.Tangency.Unqualified);
    }

    /// <summary>A native target: the qualified 2D curve of the edge on XY (GeomAPI::To2d) over its range, or the point.</summary>
    private sealed class NArg
    {
        public N.Geom2dGcc_QualifiedCurve? Q;
        public N.Geom2d_CartesianPoint? P;
        public N.Geom2dAdaptor_Curve? Adaptor;
        public double First, Last;
    }

    private sealed class PArg
    {
        public Geom2dGcc_QualifiedCurve? Q;
        public Geom2d_CartesianPoint? P;
        public Geom2dAdaptor_Curve? Adaptor;
        public double First, Last;
    }

    private static readonly N.gp_Pln s_nPlnXY = new(new N.gp_Ax3(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(0, 0, 1)));
    private static readonly gp_Pln s_pPlnXY = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));

    private static List<NArg> NArgs(List<Target> theTargets, List<N.TopoDS_Shape> theOps)
    {
        var anArgs = new List<NArg>();
        int k = 0;
        foreach (Target t in theTargets)
        {
            if (t.Edge is null)
            {
                anArgs.Add(new NArg { P = new N.Geom2d_CartesianPoint(new N.gp_Pnt2d(t.X, t.Y)) });
                continue;
            }
            N.TopoDS_Edge anE = N.TopoDS.Edge(theOps[k++]);
            double f = 0, l = 0, aFirst = 0, aLast = 0;
            N.Geom_Curve aC = N.BRep_Tool.Curve(anE, ref f, ref l)!;
            N.BRep_Tool.Range(anE, ref aFirst, ref aLast);
            N.Geom2d_Curve aH = N.GeomAPI.To2d(aC, s_nPlnXY)!;
            var anA = new N.Geom2dAdaptor_Curve(aH, aFirst, aLast);
            anArgs.Add(new NArg { Q = new N.Geom2dGcc_QualifiedCurve(anA, (N.GccEnt_Position)(int)t.Tangency), Adaptor = anA, First = aFirst, Last = aLast });
        }
        return anArgs;
    }

    private static List<PArg> PArgs(List<Target> theTargets, List<TopoDS_Shape> theOps)
    {
        var anArgs = new List<PArg>();
        int k = 0;
        foreach (Target t in theTargets)
        {
            if (t.Edge is null)
            {
                anArgs.Add(new PArg { P = new Geom2d_CartesianPoint(new gp_Pnt2d(t.X, t.Y)) });
                continue;
            }
            TopoDS_Edge anE = TopoDS.Edge(theOps[k++]);
            Geom_Curve aC = BRep_Tool.Curve(anE, out _, out _)!;
            BRep_Tool.Range(anE, out double aFirst, out double aLast);
            Geom2d_Curve aH = GeomAPI.To2d(aC, s_pPlnXY)!;
            var anA = new Geom2dAdaptor_Curve(aH, aFirst, aLast);
            anArgs.Add(new PArg { Q = new Geom2dGcc_QualifiedCurve(anA, (GccEnt_Position)(int)t.Tangency), Adaptor = anA, First = aFirst, Last = aLast });
        }
        return anArgs;
    }

    /// <summary>The targets with points last (build123d.NET's stable sort), and their edges as operands.</summary>
    private string Sorted(ref List<Target> theTargets)
    {
        theTargets = [.. theTargets.OrderBy(t => t.Edge is null)];
        return Operands([.. theTargets.Where(t => t.Edge is not null).Select(t => (B.Shape)t.Edge!)]);
    }

    private const double Tol = B.Constants.Tolerance;

    private static double Guess(double theFirst, double theLast) => (theLast - theFirst) / 2 + theFirst;

    /// <summary>MakeConstrainedArcs(t1, t2, radius): Geom2dGcc_Circ2d2TanRad.</summary>
    private void TwoTanRad(string theName, Target theT1, Target theT2, double theRadius)
    {
        List<Target> aTs = [theT1, theT2];
        string anOps = Sorted(ref aTs);
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2d2TanRad((a[0].Q is not null, a[1].Q is not null) switch
            {
                (true, true) => new N.Geom2dGcc_Circ2d2TanRad(a[0].Q!, a[1].Q!, theRadius, Tol),
                (true, false) => new N.Geom2dGcc_Circ2d2TanRad(a[0].Q!, a[1].P!, theRadius, Tol),
                _ => new N.Geom2dGcc_Circ2d2TanRad(a[0].P!, a[1].P!, theRadius, Tol),
            }));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2d2TanRad((a[0].Q is not null, a[1].Q is not null) switch
            {
                (true, true) => new Geom2dGcc_Circ2d2TanRad(a[0].Q!.Value, a[1].Q!.Value, theRadius, Tol),
                (true, false) => new Geom2dGcc_Circ2d2TanRad(a[0].Q!.Value, a[1].P!, theRadius, Tol),
                _ => new Geom2dGcc_Circ2d2TanRad(a[0].P!, a[1].P!, theRadius, Tol),
            }));
        });
    }

    /// <summary>MakeConstrainedArcsCenteredOn(t1, t2, on): Geom2dGcc_Circ2d2TanOn, guessed at the middle of each edge.</summary>
    private void TwoTanOn(string theName, Target theT1, Target theT2, Target theOn)
    {
        List<Target> aTs = [theT1, theT2];
        _ = Sorted(ref aTs);
        aTs.Add(theOn);
        string anOps = Operands([.. aTs.Where(t => t.Edge is not null).Select(t => (B.Shape)t.Edge!)]);
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            var g = a.Where(x => x.Q is not null).Select(x => Guess(x.First, x.Last)).ToList();
            var anOn = a[2].Adaptor!;
            return G.RunNative(() => G.NCirc2d2TanOn(g.Count switch
            {
                3 => new N.Geom2dGcc_Circ2d2TanOn(a[0].Q!, a[1].Q!, anOn, Tol, g[0], g[1], g[2]),
                2 => new N.Geom2dGcc_Circ2d2TanOn(a[0].Q!, a[1].P!, anOn, Tol, g[0], g[1]),
                _ => new N.Geom2dGcc_Circ2d2TanOn(a[0].P!, a[1].P!, anOn, Tol),
            }));
        }, o =>
        {
            var a = PArgs(aTs, o);
            var g = a.Where(x => x.Q is not null).Select(x => Guess(x.First, x.Last)).ToList();
            var anOn = a[2].Adaptor!;
            return G.RunPort(() => G.PCirc2d2TanOn(g.Count switch
            {
                3 => new Geom2dGcc_Circ2d2TanOn(a[0].Q!.Value, a[1].Q!.Value, anOn, Tol, g[0], g[1], g[2]),
                2 => new Geom2dGcc_Circ2d2TanOn(a[0].Q!.Value, a[1].P!, anOn, Tol, g[0], g[1]),
                _ => new Geom2dGcc_Circ2d2TanOn(a[0].P!, a[1].P!, anOn, Tol),
            }));
        });
    }

    /// <summary>MakeConstrainedArcs(t1, t2, t3): Geom2dGcc_Circ2d3Tan, guessed at the middle of each edge.</summary>
    private void ThreeTan(string theName, Target theT1, Target theT2, Target theT3)
    {
        List<Target> aTs = [theT1, theT2, theT3];
        string anOps = Sorted(ref aTs);
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2d3Tan(new N.Geom2dGcc_Circ2d3Tan(a[0].Q!, a[1].Q!, a[2].Q!, Tol, Guess(a[0].First, a[0].Last),
                                                                               Guess(a[1].First, a[1].Last), Guess(a[2].First, a[2].Last))));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2d3Tan(new Geom2dGcc_Circ2d3Tan(a[0].Q!.Value, a[1].Q!.Value, a[2].Q!.Value, Tol, Guess(a[0].First, a[0].Last),
                                                                           Guess(a[1].First, a[1].Last), Guess(a[2].First, a[2].Last))));
        });
    }

    /// <summary>MakeConstrainedArcs(edge, center): Geom2dGcc_Circ2dTanCen.</summary>
    private void TanCen(string theName, Target theT, double theX, double theY)
    {
        List<Target> aTs = [theT];
        string anOps = Sorted(ref aTs);
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2dTanCen(new N.Geom2dGcc_Circ2dTanCen(a[0].Q!, new N.Geom2d_CartesianPoint(new N.gp_Pnt2d(theX, theY)), Tol)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2dTanCen(new Geom2dGcc_Circ2dTanCen(a[0].Q!.Value, new Geom2d_CartesianPoint(new gp_Pnt2d(theX, theY)), Tol)));
        });
    }

    /// <summary>MakeConstrainedArcs(t, radius, centerOn): Geom2dGcc_Circ2dTanOnRad, the locus unqualified.</summary>
    private void TanOnRad(string theName, Target theT, double theRadius, Target theOn)
    {
        List<Target> aTs = [theT, theOn with { Tangency = B.Tangency.Unqualified }];
        string anOps = Operands([.. aTs.Where(t => t.Edge is not null).Select(t => (B.Shape)t.Edge!)]);
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2dTanOnRad(a[0].Q is not null
                ? new N.Geom2dGcc_Circ2dTanOnRad(a[0].Q!, a[1].Adaptor!, theRadius, Tol)
                : new N.Geom2dGcc_Circ2dTanOnRad(a[0].P!, a[1].Adaptor!, theRadius, Tol)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2dTanOnRad(a[0].Q is not null
                ? new Geom2dGcc_Circ2dTanOnRad(a[0].Q!.Value, a[1].Adaptor!, theRadius, Tol)
                : new Geom2dGcc_Circ2dTanOnRad(a[0].P!, a[1].Adaptor!, theRadius, Tol)));
        });
    }

    /// <summary>MakeConstrainedLines(t1, t2): Geom2dGcc_Lin2d2Tan, a point as gp_Pnt2d.</summary>
    private void TwoTanLines(string theName, Target theT1, Target theT2)
    {
        List<Target> aTs = [theT1, theT2];
        string anOps = Sorted(ref aTs);
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NLin2d2Tan(a[1].Q is not null
                ? new N.Geom2dGcc_Lin2d2Tan(a[0].Q!, a[1].Q!, Tol)
                : new N.Geom2dGcc_Lin2d2Tan(a[0].Q!, new N.gp_Pnt2d(aTs[1].X, aTs[1].Y), Tol)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PLin2d2Tan(a[1].Q is not null
                ? new Geom2dGcc_Lin2d2Tan(a[0].Q!.Value, a[1].Q!.Value, Tol)
                : new Geom2dGcc_Lin2d2Tan(a[0].Q!.Value, new gp_Pnt2d(aTs[1].X, aTs[1].Y), Tol)));
        });
    }

    /// <summary>MakeConstrainedLines(t, reference, angle): Geom2dGcc_Lin2dTanObl from the reference axis's line on XY.</summary>
    private void TanObl(string theName, Target theT, B.Axis theReference, double theAngle)
    {
        List<Target> aTs = [theT];
        string anOps = Sorted(ref aTs);
        B.Vector p = theReference.Position, d = theReference.Direction;
        CompareSolver(theName, anOps, o =>
        {
            var a = NArgs(aTs, o);
            var aLin = new N.gp_Lin2d(new N.gp_Ax2d(new N.gp_Pnt2d(p.X, p.Y), new N.gp_Dir2d(d.X, d.Y)));
            return G.RunNative(() => G.NLin2dTanObl(new N.Geom2dGcc_Lin2dTanObl(a[0].Q!, aLin, Tol, theAngle)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            var aLin = new gp_Lin2d(new gp_Ax2d(new gp_Pnt2d(p.X, p.Y), new gp_Dir2d(d.X, d.Y)));
            return G.RunPort(() => G.PLin2dTanObl(new Geom2dGcc_Lin2dTanObl(a[0].Q!.Value, aLin, Tol, theAngle)));
        });
    }

    [Fact]
    public void Constrained_arcs_and_lines_of_build123d_tests_as_native()
    {
        var U = B.Tangency.Unqualified;
        // ConstrainedArcsTests: two tangencies and a radius
        {
            var e1 = new B.Line((-2, 0), (2, 0));
            var e2 = new B.Line((0, -2), (0, 2));
            TwoTanRad("Tan2RadArcs1", e1, e2, 0.5);
        }
        TwoTanRad("Tan2RadArcs2", new B.CenterArc((0, 0), 1, 0, 90), new B.Line((1, 0), (2, 0)), 0.5);
        TwoTanRad("Tan2RadArcs3", (0, 0), (0, 0.5), 0.5);
        TwoTanRad("Tan2RadArcs4", (0, 0.5), new B.Line((0, 0), (1, 0)), 0.5);
        TwoTanRad("Tan2RadArcs5", (0, 0), (10, 0), 2);
        {
            double rLeft = 0.75, rRight = 1.0;
            var aLeft = new B.CenterArc((-2, 0), rLeft, 0, 360);
            var aRight = new B.CenterArc((2, 0), rRight, 0, 360);
            TwoTanRad("Eggplant bottom", (aRight, B.Tangency.Outside), (aLeft, B.Tangency.Outside), 6);
            TwoTanRad("Eggplant top", (aRight, B.Tangency.Enclosing), (aLeft, B.Tangency.Enclosing), 8);
        }

        // centred on a curve
        TwoTanOn("Tan2CenterOn1", (new B.PolarLine((0, 0), 4, -20, lengthMode: B.LengthMode.Horizontal), U), (new B.Line((4, -2), (4, 2)), U),
                 new B.Line((3, -2), (3, 2)));
        TwoTanOn("Tan2CenterOn2", (0, 3), (5, 0), new B.Line((0, -5), (0, 5)));
        TwoTanOn("Tan2CenterOn3", new B.Line((-5, 3), (5, 3)), (5, 0), new B.Line((0, -5), (0, 5)));
        TwoTanOn("Tan2CenterOn4", new B.Line((-5, 3), (5, 3)), (5, 0), B.Axis.Y);
        TwoTanOn("Tan2CenterOn5", new B.Line((-5, 3), (5, 3)), new B.Line((-5, 0), (5, 0)), new B.Line((-5, -1), (5, -1)));
        TwoTanOn("Tan2CenterOn6", new B.Line((0, 0), (5, 0)), new B.Line((0, 0), (0, 5)), new B.Line((20, 20), (22, 22)));
        {
            var c1 = new B.Line((-10, 0), (10, 0));
            var c2 = new B.Line((0, -10), (0, 10));
            TwoTanOn("Tan2CenterOnSagittaBoth", c1, c2, new B.Line((-10, 1), (10, 1)));
            TwoTanOn("Tan2CenterOnSagittaLong", c1, c2, new B.Line((3, -10), (3, 10)));
        }
        TwoTanOn("Tan2CenterOnFiltersFirst", new B.Line((0.0, 0.0), (0.01, 0.0)), new B.Line((10.0, -10.0), (10.0, 10.0)), new B.Line((5.0, -10.0), (5.0, 10.0)));
        TwoTanOn("Tan2CenterOnFiltersSecond", (0.0, 2.0), new B.Line((10.0, -0.005), (10.0, 0.005)), new B.Line((-10.0, 2.0), (10.0, 2.0)));
        TwoTanOn("Tan2CenterOnMultiple", new B.Line((-20, 0), (20, 0)), new B.Line((0, -20), (0, 20)), new B.Line((-20, -2), (20, -2)));
        {
            var c1 = new B.CenterArc((0, 20), 20, -90, 90);
            var ln2 = new B.Line(c1.PositionAt(1), (10, 20));
            TwoTanOn("ConstrainedArcs2TanCenterOn", c1, ln2, new B.Line((10, 10), ln2.PositionAt(1)));
        }

        // a centre
        TanCen("TanCenterOn1", new B.PolarLine((0, 0), 4, 60), 2, 1);
        TanCen("TanCenterOn2", B.Axis.X, 2, 1);
        TanCen("TanCenterOn3", new B.CenterArc((0, 0), 1, 180, 5), 2, 0);

        // a radius and a locus
        TanOnRad("TanRadCenterOn1", new B.PolarLine((0, 0), 4, -20, lengthMode: B.LengthMode.Horizontal), 1, new B.Line((3, -2), (3, 2)));
        TanOnRad("TanRadCenterOn2", new B.PolarLine((0, 0), 4, -20, lengthMode: B.LengthMode.Horizontal), 1, B.Axis.X);
        TanOnRad("TanRadCenterOn4", new B.Line((0, 10), (10, 10)), 1, B.Axis.X);

        // three tangencies
        {
            var c5 = new B.PolarLine((0, 0), 4, 60);
            var c6 = new B.PolarLine((0, 0), 4, 40);
            var c7 = new B.CenterArc((0, 0), 4, 0, 90);
            ThreeTan("Tan31", c5, c6, c7);
        }
        ThreeTan("Tan32", new B.Line((0, 0), (0, 1)), new B.Line((0, 0), (1, 0)), new B.Line((0, 0), (0, -1)));
        ThreeTan("Tan33", new B.Line((0, 0), (10, 0)), new B.Line((0, 2), (10, 2)), new B.Line((0, 5), (10, 5)));
        ThreeTan("Tan34", new B.Line((-1, 0), (-1, 2)), new B.Line((1, 0), (1, 2)), new B.Line((-1, 0), (-0.75, 0)));
        {
            var c1 = new B.CenterArc((0, 20), 20, -90, 90);
            var c2 = new B.CenterArc((5, 25), 10, -90, 90);
            ThreeTan("MakeConstrainedArcs3Tan line", c1, c2, new B.Line(c1.PositionAt(1), c2.PositionAt(1)));
            ThreeTan("MakeConstrainedArcs3Tan arc", c1, c2, new B.RadiusArc(c1.PositionAt(1), c2.PositionAt(1), 10));
        }

        // ConstrainedLinesTests
        var aUnit = B.Edge.MakeCircle(1.0, B.Plane.XY);
        TwoTanLines("MakeConstrainedLines0", aUnit, aUnit.Translate((3, 0, 0)));
        TwoTanLines("MakeConstrainedLines1", aUnit, (3, 0));
        TanObl("MakeConstrainedLines3", aUnit, B.Axis.X, 30 * DegToRad);
        TanObl("MakeConstrainedLines4", aUnit, B.Axis.Y, 30 * DegToRad);
        TanObl("MakeConstrainedLines5", (aUnit, B.Tangency.Enclosing), B.Axis.Y, 30 * DegToRad);
        TanObl("MakeConstrainedLines6", (aUnit, B.Tangency.Enclosing), B.Axis.Y, Math.Atan2(1, 1));
        TanObl("InvalidOrientedTangent 1", aUnit, new B.Axis((1, 0), (0, 1)), 0);
        TanObl("InvalidOrientedTangent 2", aUnit.Translate((0, 1 + 1e-7)), B.Axis.X, 0);
        Report();
    }
}
#endif
