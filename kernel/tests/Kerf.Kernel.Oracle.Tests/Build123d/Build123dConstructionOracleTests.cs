// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text.Json;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;
using G = Kerf.Kernel.Oracle.Tests.Geom2dGccOracleTests;

namespace Kerf.Kernel.Oracle.Tests;

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
/// records (solutions, tangencies, every double bit for bit). Since unit 10A each case is recorded with native OCCT's side, and
/// replayed from there; the scenarios that made the cases went with native OCCT in unit 10B.
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

    private static List<Shape> PortOperands(string theText, bool theFrozen = true)
    {
        Shape aC = BooleanOracleTests.ReadPort(theText);
        if (theFrozen)
        {
            aC.TShape()!.FreezeGraph(); // a caller's shapes
        }
        else
        {
            // a copy the algorithm may write into, as native's operands (the shapes read are frozen)
            aC = BrepTools.ExactCopyForWrite(aC, out _);
        }
        var aList = new List<Shape>();
        for (var anIt = new Iterator(aC); anIt.More(); anIt.Next())
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
                         Func<List<Shape>, Shape> thePort, string? theAttribution = null)
    {
        Tally aTally = TallyOf(theAlgo);
        aTally.Cases++;
        (string? aNText, string? aNErr) = RunNative(theOperands, theNative);

        List<Shape> aPOps = PortOperands(theOperands);
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
    private bool DestructiveMatches(string theNText, string theOperands, Func<List<Shape>, Shape> thePort)
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
                               Func<List<Shape>, G.Rec> thePort, Func<string, bool>? theMasked = null)
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

    private static Compound PCompound(params Shape[] theShapes) => Builder.MakeCompound(theShapes);

    // ---------------------------------------------------------------------------------------------------------------
    // Sweeps: BRepOffsetAPI_MakePipeShell

    /// <summary>
    /// The call sequence of one of build123d.NET's sweeps. Operands: the path wire, the profiles, then the auxiliary
    /// spine if any. <paramref name="Mode"/>: 0 none, 1 SetMode(isFrenet), 2 SetMode(gp_Ax2) at <paramref name="Start"/>

    // ---------------------------------------------------------------------------------------------------------------
    // The cases, recorded (unit 10A)
    //
    // The scenarios built their operands with build123d.NET on native OCCT, so they ran only when recording, and went
    // with native OCCT (unit 10B). Each case they made (its kind, its operands as native OCCT writes them, and its plain
    // data) is recorded; the tests run the recorded cases, their native side replayed from the tape.

    /// <summary>A point or vector of build123d.NET's, as plain numbers.</summary>
    internal readonly record struct Vec3(double X, double Y, double Z);

    /// <summary>One case: the run's kind, the case's name, its operands (BRep text), its data (JSON).</summary>
    private sealed record Case(string Kind, string Name, string Operands, string Data);

    private static readonly JsonSerializerOptions s_json = new() { IncludeFields = true };

    private static T Data<T>(Case theCase) => JsonSerializer.Deserialize<T>(theCase.Data, s_json)!;

    private static Case? Recorded() => Tape.Value("case",
        static r => r.Value() is object?[] a ? new Case((string)a[0]!, (string)a[1]!, (string)a[2]!, (string)a[3]!) : null);

    /// <summary>The recorded cases of a test, each run in turn.</summary>
    private void Cases()
    {
        while (Recorded() is Case aCase)
        {
            Run(aCase);
        }
    }

    private sealed record SweepCase(SweepCall Call, int NbProfiles);

    private sealed record LoftCase(bool Filled, bool Ruled);

    private sealed record FillingCase(int NbOnFaces, Vec3[] Points);

    private sealed record PointsCase(Vec3[][] Points, double[]? Smoothing, int MinDeg, int MaxDeg);

    private sealed record ProjectionCase(Vec3? Dir, Vec3? Center);

    private sealed record Chamfer2DCase(int NbVertices, bool HasReference, double D1, double D2);

    private sealed record OffsetCase(JoinKind Join, double Distance);

    private sealed record SolverCase(TargetData[] Targets, double Radius = 0, double X = 0, double Y = 0, Vec3 P = default, Vec3 D = default,
                                     double Angle = 0);

    private void Run(Case theCase)
    {
        string aName = theCase.Name, anOps = theCase.Operands;
        switch (theCase.Kind)
        {
            case "Sweep": RunSweep(aName, anOps, Data<SweepCase>(theCase)); break;
            case "Loft": RunLoft(aName, anOps, Data<LoftCase>(theCase)); break;
            case "MakeSurface": RunMakeSurface(aName, anOps, Data<FillingCase>(theCase)); break;
            case "MakeSurfacePatch": RunMakeSurfacePatch(aName, anOps, Data<FillingCase>(theCase)); break;
            case "SurfaceFromPoints": RunSurfaceFromPoints(aName, anOps, Data<PointsCase>(theCase)); break;
            case "RuledFace": RunRuledFace(aName, anOps); break;
            case "RuledShell": RunRuledShell(aName, anOps); break;
            case "Projection": RunProjection(aName, anOps, Data<ProjectionCase>(theCase)); break;
            case "Chamfer2D": RunChamfer2D(aName, anOps, Data<Chamfer2DCase>(theCase)); break;
            case "Offset2D": RunOffset2D(aName, anOps, Data<OffsetCase>(theCase)); break;
            case "TwoTanRad": RunTwoTanRad(aName, anOps, Data<SolverCase>(theCase)); break;
            case "TwoTanOn": RunTwoTanOn(aName, anOps, Data<SolverCase>(theCase)); break;
            case "ThreeTan": RunThreeTan(aName, anOps, Data<SolverCase>(theCase)); break;
            case "TanCen": RunTanCen(aName, anOps, Data<SolverCase>(theCase)); break;
            case "TanOnRad": RunTanOnRad(aName, anOps, Data<SolverCase>(theCase)); break;
            case "TwoTanLines": RunTwoTanLines(aName, anOps, Data<SolverCase>(theCase)); break;
            case "TanObl": RunTanObl(aName, anOps, Data<SolverCase>(theCase)); break;
            default: throw new InvalidOperationException($"A recorded case of an unknown kind: {theCase.Kind}");
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Sweeps: BRepOffsetAPI_MakePipeShell

    /// <summary>
    /// The call sequence of one of build123d.NET's sweeps. Operands: the path wire, the profiles, then the auxiliary
    /// spine if any. <paramref name="Mode"/>: 0 none, 1 SetMode(isFrenet), 2 SetMode(gp_Ax2) at <paramref name="Start"/>
    /// along <paramref name="Dir"/> (a Vector binormal), 3 SetMode(auxiliary, curvilinear). The transition is set before
    /// the profiles are added (Solid.Sweep) or after (Face.Sweep, Shell.Sweep), or not at all (null).
    /// </summary>
    private sealed record SweepCall(int Mode, bool Frenet = false, Vec3? Start = null, Vec3? Dir = null, bool Curvilinear = false,
                                    TransitionMode? Transition = null, bool TransitionAfterAdd = false, bool MakeSolid = false,
                                    bool Rotate = false, bool DefaultAdd = false);

    private void RunSweep(string theName, string theOperands, SweepCase theCase)
    {
        SweepCall theCall = theCase.Call;
        int aNbProfiles = theCase.NbProfiles;
        Compare("MakePipeShell", theName, theOperands, o =>
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
            var aB = new MakePipeShell(ShapeCasts.Wire(o[0]));
            switch (theCall.Mode)
            {
                case 1:
                    aB.SetMode(theCall.Frenet);
                    break;
                case 2:
                {
                    var anAx = new Frame3D();
                    anAx = anAx.WithLocation(new Point3D(theCall.Start!.Value.X, theCall.Start.Value.Y, theCall.Start.Value.Z));
                    anAx = anAx.WithDirection(new Direction3D(theCall.Dir!.Value.X, theCall.Dir.Value.Y, theCall.Dir.Value.Z));
                    aB.SetMode(anAx);
                    break;
                }
                case 3:
                    aB.SetMode(ShapeCasts.Wire(o[^1]), theCall.Curvilinear);
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

    [Fact]
    public void Sweeps_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Lofts: BRepOffsetAPI_ThruSections

    /// <summary>TopoUtil.MakeLoft: ThruSections(filled, ruled), vertices as they are and private copies of the wires.</summary>
    private void RunLoft(string theName, string theOperands, LoftCase theCase)
    {
        (bool theFilled, bool theRuled) = (theCase.Filled, theCase.Ruled);
        Compare("ThruSections", theName, theOperands, o =>
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
            var aLoft = new ThruSections(theFilled, theRuled);
            foreach (Shape aS in o)
            {
                if (aS.ShapeType() == ShapeType.Vertex)
                {
                    aLoft.AddVertex(ShapeCasts.Vertex(aS));
                }
                else
                {
                    aLoft.AddWire(ShapeCasts.Wire(new Copy(aS, false, false).Shape()));
                }
            }
            aLoft.Build();
            return aLoft.Shape();
        });
    }

    [Fact]
    public void Lofts_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Surfaces: BRepOffsetAPI_MakeFilling, GeomAPI_PointsToBSplineSurface, BRepFill::Face and Shell

    /// <summary>
    /// Face.MakeSurface: a filling of build123d.NET's parameters through C0 edges, built; then the points added and built
    /// again. Both builds' faces are compared (as one compound).
    /// </summary>
    private void RunMakeSurface(string theName, string theOperands, FillingCase theCase)
    {
        Vec3[] thePoints = theCase.Points;
        Compare("MakeFilling", theName, theOperands, o =>
        {
            using var aF = new N.BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
            foreach (N.TopoDS_Shape anE in o)
            {
                aF.Add(N.TopoDS.Edge(anE), N.GeomAbs_Shape.GeomAbs_C0);
            }
            aF.Build();
            N.TopoDS_Shape aFirst = aF.Shape();
            if (thePoints.Length == 0)
            {
                return aFirst;
            }
            foreach (Vec3 p in thePoints)
            {
                using var aP = new N.gp_Pnt(p.X, p.Y, p.Z);
                aF.Add(aP);
            }
            aF.Build();
            return NCompound(aFirst, aF.Shape());
        }, o =>
        {
            var aF = new MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
            foreach (Shape anE in o)
            {
                aF.Add(ShapeCasts.Edge(anE), Continuity.C0);
            }
            aF.Build();
            Shape aFirst = aF.Shape();
            if (thePoints.Length == 0)
            {
                return aFirst;
            }
            foreach (Vec3 p in thePoints)
            {
                aF.Add(new Point3D(p.X, p.Y, p.Z));
            }
            aF.Build();
            return PCompound(aFirst, aF.Shape());
        });
    }

    /// <summary>Face.MakeSurfacePatch: a default filling, edges on support faces (G1), plain edges (C0), then points.</summary>
    private void RunMakeSurfacePatch(string theName, string theOperands, FillingCase theCase)
    {
        int aNbOnFaces = theCase.NbOnFaces;
        Vec3[] thePoints = theCase.Points;
        Compare("MakeFilling", theName, theOperands, o =>
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
            foreach (Vec3 p in thePoints)
            {
                using var aP = new N.gp_Pnt(p.X, p.Y, p.Z);
                aF.Add(aP);
            }
            aF.Build();
            return aF.Shape();
        }, o =>
        {
            var aF = new MakeFilling();
            for (int i = 0; i < aNbOnFaces; i++)
            {
                aF.Add(ShapeCasts.Edge(o[2 * i]), ShapeCasts.Face(o[2 * i + 1]), Continuity.G1);
            }
            foreach (Shape anE in o.Skip(2 * aNbOnFaces))
            {
                aF.Add(ShapeCasts.Edge(anE), Continuity.C0);
            }
            foreach (Vec3 p in thePoints)
            {
                aF.Add(new Point3D(p.X, p.Y, p.Z));
            }
            aF.Build();
            return aF.Shape();
        });
    }

    /// <summary>Face.MakeSurfaceFromArrayOfPoints: the approximation, as a face on the surface (MakeFace, Precision::Confusion).</summary>
    private void RunSurfaceFromPoints(string theName, string theOperands, PointsCase theCase)
    {
        Vec3[][] thePoints = theCase.Points;
        double[]? theSmoothing = theCase.Smoothing;
        (int theMinDeg, int theMaxDeg) = (theCase.MinDeg, theCase.MaxDeg);
        int aRows = thePoints.Length, aCols = thePoints[0].Length;
        Compare("PointsToBSplineSurface", theName, theOperands, _ =>
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
            using N.GeomAPI_PointsToBSplineSurface aB = theSmoothing is [var w1, var w2, var w3]
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
            var anArray = new NCollection_Array2<Point3D>(1, aRows, 1, aCols);
            for (int i = 0; i < aRows; i++)
            {
                for (int j = 0; j < aCols; j++)
                {
                    anArray.SetValue(i + 1, j + 1, new Point3D(thePoints[i][j].X, thePoints[i][j].Y, thePoints[i][j].Z));
                }
            }
            PointsToBSplineSurface aB = theSmoothing is [var w1, var w2, var w3]
                ? new(anArray, w1, w2, w3, Math.Max(theMaxDeg, 6), Continuity.C2, 1e-2)
                : new(anArray, theMinDeg, theMaxDeg, Continuity.C2, 1e-2);
            if (!aB.IsDone)
            {
                throw new global::Kerf.Engine.Foundation.NotDoneException();
            }
            return new MakeFace(aB.Surface(), global::Kerf.Engine.Foundation.Precision.Confusion()).Face();
        });
    }

    private void RunRuledFace(string theName, string theOperands) =>
        Compare("BRepFill", theName, theOperands, o => N.BRepFill.Face(N.TopoDS.Edge(o[0]), N.TopoDS.Edge(o[1])),
                o => Fill.Face(ShapeCasts.Edge(o[0]), ShapeCasts.Edge(o[1])));

    private void RunRuledShell(string theName, string theOperands) =>
        Compare("BRepFill", theName, theOperands, o => N.BRepFill.Shell(N.TopoDS.Wire(o[0]), N.TopoDS.Wire(o[1])),
                o => Fill.Shell(ShapeCasts.Wire(o[0]), ShapeCasts.Wire(o[1])));

    [Fact]
    public void Surfaces_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Projections: BRepProj_Projection

    /// <summary>The projection of a wire or edge onto a shape, along a direction (its components as given) or from a point.</summary>
    private void RunProjection(string theName, string theOperands, ProjectionCase theCase)
    {
        (Vec3? theDir, Vec3? theCenter) = (theCase.Dir, theCase.Center);
        Compare("BRepProj_Projection", theName, theOperands, o =>
        {
            using N.BRepProj_Projection aP = theDir is { } d
                ? new(o[0], o[1], new N.gp_Dir(d.X, d.Y, d.Z))
                : new(o[0], o[1], new N.gp_Pnt(theCenter!.Value.X, theCenter.Value.Y, theCenter.Value.Z));
            return aP.Shape();
        }, o =>
        {
            Projection aP = theDir is { } d
                ? new(o[0], o[1], new Direction3D(d.X, d.Y, d.Z))
                : new(o[0], o[1], new Point3D(theCenter!.Value.X, theCenter.Value.Y, theCenter.Value.Z));
            return aP.Shape();
        });
    }

    [Fact]
    public void Projections_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // 2D chamfers: BRepFilletAPI_MakeFillet2d

    /// <summary>
    /// Wire.Chamfer2D: the wire made a face (build123d.NET's face from wires), each given vertex's two edges in
    /// MapShapesAndAncestors' order (the reference edge first when given), AddChamfer, Build.
    /// </summary>
    private void RunChamfer2D(string theName, string theOperands, Chamfer2DCase theCase)
    {
        int aNbV = theCase.NbVertices;
        bool hasReference = theCase.HasReference;
        (double theD1, double theD2) = (theCase.D1, theCase.D2);
        Compare("MakeFillet2d", theName, theOperands, o =>
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
            var aB = new MakeFillet2D(ShapeCasts.Face(o[0]));
            var aMap = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(o[0], ShapeType.Vertex, ShapeType.Edge, aMap);
            for (int i = 1; i <= aNbV; i++)
            {
                NCollection_List<Shape> aList = aMap.GetValue(o[i]);
                Shape e1 = aList.First(), e2 = aList.Last();
                if (hasReference && e2.IsSame(o[^1]))
                {
                    (e1, e2) = (e2, e1);
                }
                aB.AddChamfer(ShapeCasts.Edge(e1), ShapeCasts.Edge(e2), theD1, theD2);
            }
            aB.Build();
            return aB.Shape();
        });
    }

    [Fact]
    public void Chamfers_2d_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // 2D offsets: BRepOffsetAPI_MakeOffset

    /// <summary>MakeOffset initialised with the join (on a plane's face when given as the second operand), AddWire, Perform(distance).</summary>
    private void RunOffset2D(string theName, string theOperands, OffsetCase theCase)
    {
        JoinKind aJoin = theCase.Join;
        double aDistance = theCase.Distance;
        Compare("MakeOffset", theName, theOperands, o =>
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
            var aB = new MakeWireOffset();
            if (o.Count > 1)
            {
                aB.Init(ShapeCasts.Face(o[1]), aJoin);
            }
            else
            {
                aB.Init(aJoin);
            }
            aB.AddWire(ShapeCasts.Wire(o[0]));
            aB.Perform(aDistance);
            return aB.Shape();
        });
    }

    [Fact]
    public void Offsets_2d_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Constrained arcs and lines: the Geom2dGcc solvers

    /// <summary>A target as recorded: an edge (an operand, in order) with its qualifier (GccEnt_Position), or a point of XY.</summary>
    private sealed record TargetData(bool HasEdge, int Tangency, double X, double Y);

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
        public QualifiedCurve? Q;
        public CartesianPoint2? P;
        public CurveAdaptor2D? Adaptor;
        public double First, Last;
    }

    private static readonly Plane3D s_pPlnXY = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));

    private static List<NArg> NArgs(TargetData[] theTargets, List<N.TopoDS_Shape> theOps)
    {
        // made per test: a native object can't be static, as each test records its own native calls
        using var aPlnXY = new N.gp_Pln(new N.gp_Ax3(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(0, 0, 1)));
        var anArgs = new List<NArg>();
        int k = 0;
        foreach (TargetData t in theTargets)
        {
            if (!t.HasEdge)
            {
                anArgs.Add(new NArg { P = new N.Geom2d_CartesianPoint(new N.gp_Pnt2d(t.X, t.Y)) });
                continue;
            }
            N.TopoDS_Edge anE = N.TopoDS.Edge(theOps[k++]);
            double f = 0, l = 0, aFirst = 0, aLast = 0;
            N.Geom_Curve aC = N.BRep_Tool.Curve(anE, ref f, ref l)!;
            N.BRep_Tool.Range(anE, ref aFirst, ref aLast);
            N.Geom2d_Curve aH = N.GeomAPI.To2d(aC, aPlnXY)!;
            var anA = new N.Geom2dAdaptor_Curve(aH, aFirst, aLast);
            anArgs.Add(new NArg { Q = new N.Geom2dGcc_QualifiedCurve(anA, (N.GccEnt_Position)t.Tangency), Adaptor = anA, First = aFirst, Last = aLast });
        }
        return anArgs;
    }

    private static List<PArg> PArgs(TargetData[] theTargets, List<Shape> theOps)
    {
        var anArgs = new List<PArg>();
        int k = 0;
        foreach (TargetData t in theTargets)
        {
            if (!t.HasEdge)
            {
                anArgs.Add(new PArg { P = new CartesianPoint2(new Point2D(t.X, t.Y)) });
                continue;
            }
            Edge anE = ShapeCasts.Edge(theOps[k++]);
            Curve3D aC = Tool.Curve(anE, out _, out _)!;
            Tool.Range(anE, out double aFirst, out double aLast);
            Curve2D aH = Projections.To2d(aC, s_pPlnXY)!;
            var anA = new CurveAdaptor2D(aH, aFirst, aLast);
            anArgs.Add(new PArg { Q = new QualifiedCurve(anA, (Qualifier)t.Tangency), Adaptor = anA, First = aFirst, Last = aLast });
        }
        return anArgs;
    }

    private const double Tol = 1e-6; // build123d.NET's Constants.Tolerance

    private static double Guess(double theFirst, double theLast) => (theLast - theFirst) / 2 + theFirst;

    /// <summary>MakeConstrainedArcs(t1, t2, radius): Geom2dGcc_Circ2d2TanRad.</summary>
    private void RunTwoTanRad(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        double theRadius = theCase.Radius;
        CompareSolver(theName, theOps, o =>
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
                (true, true) => new Circ2d2TanRad(a[0].Q!.Value, a[1].Q!.Value, theRadius, Tol),
                (true, false) => new Circ2d2TanRad(a[0].Q!.Value, a[1].P!, theRadius, Tol),
                _ => new Circ2d2TanRad(a[0].P!, a[1].P!, theRadius, Tol),
            }));
        });
    }

    /// <summary>MakeConstrainedArcsCenteredOn(t1, t2, on): Geom2dGcc_Circ2d2TanOn, guessed at the middle of each edge.</summary>
    private void RunTwoTanOn(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        CompareSolver(theName, theOps, o =>
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
                3 => new Circ2d2TanOn(a[0].Q!.Value, a[1].Q!.Value, anOn, Tol, g[0], g[1], g[2]),
                2 => new Circ2d2TanOn(a[0].Q!.Value, a[1].P!, anOn, Tol, g[0], g[1]),
                _ => new Circ2d2TanOn(a[0].P!, a[1].P!, anOn, Tol),
            }));
        });
    }

    /// <summary>MakeConstrainedArcs(t1, t2, t3): Geom2dGcc_Circ2d3Tan, guessed at the middle of each edge.</summary>
    private void RunThreeTan(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        CompareSolver(theName, theOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2d3Tan(new N.Geom2dGcc_Circ2d3Tan(a[0].Q!, a[1].Q!, a[2].Q!, Tol, Guess(a[0].First, a[0].Last),
                                                                               Guess(a[1].First, a[1].Last), Guess(a[2].First, a[2].Last))));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2d3Tan(new Circ2d3Tan(a[0].Q!.Value, a[1].Q!.Value, a[2].Q!.Value, Tol, Guess(a[0].First, a[0].Last),
                                                                           Guess(a[1].First, a[1].Last), Guess(a[2].First, a[2].Last))));
        });
    }

    /// <summary>MakeConstrainedArcs(edge, center): Geom2dGcc_Circ2dTanCen.</summary>
    private void RunTanCen(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        (double theX, double theY) = (theCase.X, theCase.Y);
        CompareSolver(theName, theOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2dTanCen(new N.Geom2dGcc_Circ2dTanCen(a[0].Q!, new N.Geom2d_CartesianPoint(new N.gp_Pnt2d(theX, theY)), Tol)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2dTanCen(new Circ2dTanCen(a[0].Q!.Value, new CartesianPoint2(new Point2D(theX, theY)), Tol)));
        });
    }

    /// <summary>MakeConstrainedArcs(t, radius, centerOn): Geom2dGcc_Circ2dTanOnRad, the locus unqualified.</summary>
    private void RunTanOnRad(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        double theRadius = theCase.Radius;
        CompareSolver(theName, theOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NCirc2dTanOnRad(a[0].Q is not null
                ? new N.Geom2dGcc_Circ2dTanOnRad(a[0].Q!, a[1].Adaptor!, theRadius, Tol)
                : new N.Geom2dGcc_Circ2dTanOnRad(a[0].P!, a[1].Adaptor!, theRadius, Tol)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PCirc2dTanOnRad(a[0].Q is not null
                ? new Circ2dTanOnRad(a[0].Q!.Value, a[1].Adaptor!, theRadius, Tol)
                : new Circ2dTanOnRad(a[0].P!, a[1].Adaptor!, theRadius, Tol)));
        });
    }

    /// <summary>MakeConstrainedLines(t1, t2): Geom2dGcc_Lin2d2Tan, a point as gp_Pnt2d.</summary>
    private void RunTwoTanLines(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        CompareSolver(theName, theOps, o =>
        {
            var a = NArgs(aTs, o);
            return G.RunNative(() => G.NLin2d2Tan(a[1].Q is not null
                ? new N.Geom2dGcc_Lin2d2Tan(a[0].Q!, a[1].Q!, Tol)
                : new N.Geom2dGcc_Lin2d2Tan(a[0].Q!, new N.gp_Pnt2d(aTs[1].X, aTs[1].Y), Tol)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            return G.RunPort(() => G.PLin2d2Tan(a[1].Q is not null
                ? new Lin2d2Tan(a[0].Q!.Value, a[1].Q!.Value, Tol)
                : new Lin2d2Tan(a[0].Q!.Value, new Point2D(aTs[1].X, aTs[1].Y), Tol)));
        });
    }

    /// <summary>MakeConstrainedLines(t, reference, angle): Geom2dGcc_Lin2dTanObl from the reference axis's line on XY.</summary>
    private void RunTanObl(string theName, string theOps, SolverCase theCase)
    {
        TargetData[] aTs = theCase.Targets;
        (Vec3 p, Vec3 d, double theAngle) = (theCase.P, theCase.D, theCase.Angle);
        CompareSolver(theName, theOps, o =>
        {
            var a = NArgs(aTs, o);
            var aLin = new N.gp_Lin2d(new N.gp_Ax2d(new N.gp_Pnt2d(p.X, p.Y), new N.gp_Dir2d(d.X, d.Y)));
            return G.RunNative(() => G.NLin2dTanObl(new N.Geom2dGcc_Lin2dTanObl(a[0].Q!, aLin, Tol, theAngle)));
        }, o =>
        {
            var a = PArgs(aTs, o);
            var aLin = new Line2D(new Axis2D(new Point2D(p.X, p.Y), new Direction2D(d.X, d.Y)));
            return G.RunPort(() => G.PLin2dTanObl(new Lin2dTanObl(a[0].Q!.Value, aLin, Tol, theAngle)));
        });
    }

    [Fact]
    public void Constrained_arcs_and_lines_of_build123d_tests_as_native()
    {
        Cases();
        Report();
    }
}
