// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepFilletAPI_MakeFillet and MakeChamfer (unit 7N3, over 7N2's builder and 7R's reconstruction) against native
/// OCCT: every edge of primitives, Boolean results and data/occ's solids, singly and in chains (all the edges, the
/// edges of a face), constant and evolving radii, chamfers by one and two distances and by a distance and an angle,
/// radii too large (the faulty-contour diagnostics), OCCT's GTests' inputs and build123d.NET's MaxFillet probes.
/// </summary>
/// <remarks>
/// Both sides read the input from the same BRep text (the port's frozen, as a caller's). The reconstruction differs
/// from native's HBuilder in the order of the faces (decision 1114), so the results are compared geometrically
/// (decision 992, 7R's comparison): volume and area (1e-6 relative), the counts of solids, shells, faces, edges and
/// vertices, BRepCheck's validity, the optimal bounding box (1e-4 of the diagonal), and the history as sets of
/// (surface type, area): <c>Modified</c> and <c>IsDeleted</c> of every input face, <c>Generated</c> of every input edge.
/// The exception, IsDone and the diagnostics (contours, faulty contours, stripe statuses, computed surfaces, faulty
/// vertices, HasResult) must be native's exactly. The port's input must be unchanged and its result frozen.
/// </remarks>
public sealed class BRepFilletAPIMakeFilletOracleTests(ITestOutputHelper output)
{
    /// <summary>
    /// An edge of a case: its index in TopExp::MapShapes(input, EDGE) and its parameters. A fillet: radius R1, or R1 to
    /// R2 (R2 &gt; 0), or the (parameter, radius) pairs <see cref="Pairs"/>. A chamfer: distance R1, or R1 on the edge's
    /// first face and R2 (Mode 0, R2 &gt; 0), or R1 and the angle R2 in degrees (Mode 1).
    /// </summary>
    internal sealed record CaseEdge(int K, double R1, double R2 = -1, int Mode = 0, double[]? Pairs = null);

    internal sealed record Case(string Name, bool Chamfer, CaseEdge[] Edges);

    internal sealed class Tally
    {
        public int Runs, SameError, NotDone, Done, Equal, NativeInvalid;
        public readonly SortedDictionary<string, int> Errors = new(StringComparer.Ordinal);
        public readonly List<string> Differences = [];
        public readonly List<string> PortInvalid = [];
        public readonly List<string> Attributed = [];
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Cases

    private static double Diagonal(in Shape theShape)
    {
        Box aBox = new();
        BRepBndLib.Add(theShape, ref aBox);
        return Math.Sqrt(aBox.SquareExtent());
    }

    private static NCollection_IndexedMap<Shape> Map(in Shape S, ShapeType T)
    {
        NCollection_IndexedMap<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, T, M);
        return M;
    }

    /// <summary>The edges that are not degenerated and lie between two different faces (filletable).</summary>
    private static List<int> SharpEdges(in Shape S)
    {
        NCollection_IndexedMap<Shape> anEdges = Map(S, ShapeType.Edge);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> EF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(S, ShapeType.Edge, ShapeType.Face, EF);
        List<int> r = [];
        for (int k = 1; k <= anEdges.Extent(); k++)
        {
            Edge E = ShapeCasts.Edge(anEdges[k]);
            if (Tool.Degenerated(E) || !EF.Contains(E))
            {
                continue;
            }
            NCollection_List<Shape> L = EF.FindFromKey(E);
            if (L.Extent() == 2 && !L.First().IsSame(L.Last()))
            {
                r.Add(k);
            }
        }
        return r;
    }

    /// <summary>The cases on a solid: each edge singly, evolving, chamfered; all the edges; each face's edges; too large.</summary>
    internal static IEnumerable<Case> SolidCases(string theName, Shape theShape, bool theFull = true, int theMaxEdges = 1000)
    {
        double d = Diagonal(theShape);
        double r = Math.Round(d * 0.03, 3);
        List<int> aSharp = SharpEdges(theShape);
        List<int> aSingles = aSharp.Take(theMaxEdges).ToList();
        foreach (int k in aSingles)
        {
            yield return new Case($"{theName} e{k} r{r}", false, [new CaseEdge(k, r)]);
            if (theFull)
            {
                yield return new Case($"{theName} e{k} r{r}-{2 * r}", false, [new CaseEdge(k, r, 2 * r)]);
                yield return new Case($"{theName} e{k} pairs", false, [new CaseEdge(k, r, Mode: 1, Pairs: [0.0, r, 0.4, 1.5 * r, 1.0, r])]);
            }
            yield return new Case($"{theName} e{k} chamfer {r}", true, [new CaseEdge(k, r)]);
            if (theFull)
            {
                yield return new Case($"{theName} e{k} chamfer {r} {1.5 * r}", true, [new CaseEdge(k, r, 1.5 * r)]);
                yield return new Case($"{theName} e{k} chamfer {r} 30deg", true, [new CaseEdge(k, r, 30.0, 1)]);
            }
        }
        if (aSharp.Count == 0)
        {
            yield break;
        }
        yield return new Case($"{theName} all r{r}", false, [.. aSharp.Select(k => new CaseEdge(k, r))]);
        yield return new Case($"{theName} all chamfer {r}", true, [.. aSharp.Select(k => new CaseEdge(k, r))]);
        if (theFull)
        {
            // the edges of each of the first faces (a chain round the face)
            NCollection_IndexedMap<Shape> aFaces = Map(theShape, ShapeType.Face);
            NCollection_IndexedMap<Shape> anEdges = Map(theShape, ShapeType.Edge);
            for (int f = 1; f <= Math.Min(3, aFaces.Extent()); f++)
            {
                List<int> aChain = [];
                for (Explorer ex = new(aFaces[f], ShapeType.Edge); ex.More(); ex.Next())
                {
                    int k = anEdges.FindIndex(ex.Current());
                    if (aSharp.Contains(k) && !aChain.Contains(k))
                    {
                        aChain.Add(k);
                    }
                }
                if (aChain.Count > 0)
                {
                    yield return new Case($"{theName} face{f} edges r{r}", false, [.. aChain.Select(k => new CaseEdge(k, r))]);
                }
            }
            // radii too large: the faulty-contour diagnostics
            yield return new Case($"{theName} e{aSharp[0]} too large r{Math.Round(d, 3)}", false, [new CaseEdge(aSharp[0], Math.Round(d, 3))]);
            yield return new Case($"{theName} all too large r{Math.Round(0.3 * d, 3)}", false,
                                  [.. aSharp.Select(k => new CaseEdge(k, Math.Round(0.3 * d, 3)))]);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Tests

    [Fact]
    public void Primitives_and_Boolean_results_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "fillet");
        var aWatch = Stopwatch.StartNew();
        foreach ((string aName, Shape aShape) in Tape.Budget(BRepOffsetAPIMakeThickSolidOracleTests.Solids()))
        {
            foreach (Case aCase in SolidCases(aName, aShape))
            {
                Compare(aRun, aTally, aCase, aShape);
            }
        }
        output.WriteLine($"time {aWatch.Elapsed.TotalSeconds:F0} s");
        Report(aTally, "primitives and Boolean results", KnownPrimitiveDifferences);
    }

    [Fact]
    public void Data_occ_solids_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "fillet");
        string aDir = Path.Combine(ShapeFixOracleRun.Metadata("OcctSourceDir"), "data", "occ");
        var aWatch = Stopwatch.StartNew();
        foreach (string aFile in Tape.Budget(Directory.GetFiles(aDir, "*.brep").Order(StringComparer.Ordinal), isSpread: true))
        {
            if (new FileInfo(aFile).Length > 300_000)
            {
                continue;
            }
            Assert.True(BrepTools.Read(out Shape aShape, aFile));
            if (aShape.IsNull || !new Explorer(aShape, ShapeType.Solid).More())
            {
                continue;
            }
            string aName = Path.GetFileNameWithoutExtension(aFile);
            foreach (Case aCase in SolidCases(aName, aShape, false, 24))
            {
                Compare(aRun, aTally, aCase, aShape);
            }
        }
        output.WriteLine($"time {aWatch.Elapsed.TotalSeconds:F0} s");
        Report(aTally, "data/occ solids up to 300 kB", KnownDataOccDifferences);
    }

    [Fact]
    public void OCCTs_GTests_inputs_as_native()
    {
        // OCC1077: box common sphere, cut by a cylinder, the section edges filleted (BRepFilletAPI_MakeFillet_Test)
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "fillet");
        Shape aCommon = new BRepAlgoAPI_Common(new BRepPrimAPI_MakeBox(new Point3D(-5, -5, -5), 10, 10, 10).Shape(),
                                                      new BRepPrimAPI_MakeSphere(7).Shape()).Shape();
        Shape aCyl1 = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(0, 0, -10), new Direction3D(0, 0, 1)), 3, 20).Shape();
        BRepAlgoAPI_Cut aCut = new(aCommon, aCyl1);
        Shape aCutShape = aCut.Shape();
        NCollection_IndexedMap<Shape> anEdges = Map(aCutShape, ShapeType.Edge);
        CaseEdge[] aSection = [.. aCut.SectionEdges().Select(e => new CaseEdge(anEdges.FindIndex(e), 0.7))];
        Compare(aRun, aTally, new Case("OCC1077 first cut, section edges r0.7", false, aSection), aCutShape);
        foreach (CaseEdge e in aSection)
        {
            Compare(aRun, aTally, new Case($"OCC1077 first cut, e{e.K} r0.7", false, [e]), aCutShape);
        }
        // the next cuts, on the port's filleted and fixed result (as the GTest builds it)
        Shape aStage = aCutShape;
        {
            BRepFilletAPI_MakeFillet aF = new(aCutShape);
            foreach (CaseEdge e in aSection)
            {
                aF.Add(0.7, ShapeCasts.Edge(anEdges[e.K]));
            }
            ShapeFix_Shape aFix = new(aF.Shape());
            aFix.Perform();
            aStage = aFix.Shape();
        }
        Shape[] aCyls =
        [
            new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(-10, 0, 0), new Direction3D(1, 0, 0)), 3, 20).Shape(),
            new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(0, -10, 0), new Direction3D(0, 1, 0)), 3, 20).Shape(),
        ];
        for (int c = 0; c < aCyls.Length; c++)
        {
            BRepAlgoAPI_Cut aCutC = new(aStage, aCyls[c]);
            Shape aCutCShape = aCutC.Shape();
            NCollection_IndexedMap<Shape> anEdgesC = Map(aCutCShape, ShapeType.Edge);
            CaseEdge[] aSectionC = [.. aCutC.SectionEdges().Select(e => new CaseEdge(anEdgesC.FindIndex(e), 0.7))];
            Compare(aRun, aTally, new Case($"OCC1077 cut {c + 2}, section edges r0.7", false, aSectionC), aCutCShape);
            BRepFilletAPI_MakeFillet aF = new(aCutCShape);
            foreach (CaseEdge e in aSectionC)
            {
                aF.Add(0.7, ShapeCasts.Edge(anEdgesC[e.K]));
            }
            aF.Build();
            if (!aF.IsDone())
            {
                break;
            }
            ShapeFix_Shape aFix = new(aF.Shape());
            aFix.Perform();
            aStage = aFix.Shape();
        }
        // OCC426: three revolved rings fused and unified, every edge r1
        Compare(aRun, aTally, new Case("OCC426 all edges r1", false, [.. SharpEdges(OCC426Input()).Select(k => new CaseEdge(k, 1.0))]),
                OCC426Input());
        // OCC570: a box's first wire, variable and constant radii
        Shape aBox = new BRepPrimAPI_MakeBox(100, 100, 100).Shape();
        NCollection_IndexedMap<Shape> aBoxEdges = Map(aBox, ShapeType.Edge);
        List<int> aWire = [];
        for (Explorer ex = new(new Explorer(aBox, ShapeType.Wire).Current(), ShapeType.Edge); ex.More(); ex.Next())
        {
            aWire.Add(aBoxEdges.FindIndex(ex.Current()));
        }
        double[] aVar = [0.0, 5.0, 0.3, 15.0, 0.7, 15.0, 1.0, 5.0];
        Compare(aRun, aTally, new Case("OCC570", false, [new CaseEdge(aWire[0], 0, Mode: 1, Pairs: aVar), new CaseEdge(aWire[1], 5.0),
                                                           new CaseEdge(aWire[2], 0, Mode: 1, Pairs: aVar), new CaseEdge(aWire[3], 5.0)]), aBox);
        Report(aTally, "OCCT's GTests' inputs", KnownGTestDifferences);
    }

    private static Shape OCC426Input()
    {
        Shape Ring(double r1, double r2, double z1, double z2, double a, double angle)
        {
            BRepBuilderAPI_MakePolygon aW = new();
            double c = Math.Cos(a), s = Math.Sin(a);
            aW.Add(new Point3D(r1 * c, r1 * s, z1));
            aW.Add(new Point3D(r2 * c, r2 * s, z1));
            aW.Add(new Point3D(r2 * c, r2 * s, z2));
            aW.Add(new Point3D(r1 * c, r1 * s, z2));
            aW.Add(new Point3D(r1 * c, r1 * s, z1));
            Face aF = new BRepBuilderAPI_MakeFace(aW.Wire(), false).Face();
            return new BRepPrimAPI_MakeRevol(aF, new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), angle).Shape();
        }
        Shape aRs1 = Ring(10, 20, 0, 10, 0, 2 * Math.PI);
        Shape aRs2 = Ring(10, 20, 10, 20, Math.PI / 4, 270.0 * Math.PI / 180.0);
        Shape aRs3 = Ring(10, 20, 20, 30, 0, 2 * Math.PI);
        Shape aFuse = new BRepAlgoAPI_Fuse(new BRepAlgoAPI_Fuse(aRs3, aRs2).Shape(), aRs1).Shape();
        ShapeUpgrade_UnifySameDomain aU = new(aFuse, true, true, true);
        aU.Build();
        return aU.Shape();
    }

    /// <summary>
    /// build123d.NET's <c>Shape3D.MaxFillet</c>: a binary search on the radius, a new MakeFillet per probe, a probe
    /// failing when the build throws or the result is invalid. The probes and the radius found must be native's.
    /// </summary>
    [Fact]
    public void Build123d_MaxFillet_probes_as_native()
    {
        using var aRun = new ShapeFixOracleRun(output, "fillet");
        List<string> aDifferences = [];
        int aProbes = 0, aSearches = 0;
        Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 12).Shape();
        List<(string, Shape, int[])> aSearchCases =
        [
            ("box all edges", aBox, [.. SharpEdges(aBox)]),
            ("box edge 1", aBox, [1]),
            ("box edges 1 2", aBox, [1, 2]),
            ("cylinder circles", aCyl, [.. SharpEdges(aCyl)]),
        ];
        foreach ((string Name, Shape Shape) s in BRepOffsetAPIMakeThickSolidOracleTests.Solids().Where(s => s.Name is "L prism" or "box with hole" or "wedge"))
        {
            aSearchCases.Add((s.Name + " all edges", s.Shape, [.. SharpEdges(s.Shape)]));
        }
        foreach ((string aName, Shape aShape, int[] anEdges) in aSearchCases)
        {
            if (aRun.Read(BRepOffsetAPIMakeThickSolidOracleTests.Native(aRun, aShape)) is not (N.TopoDS_Shape aNIn, Shape aPIn))
            {
                continue;
            }
            aSearches++;
            List<string> aNTrace = [], aPTrace = [];
            double aN = MaxFillet(aNTrace, w => ProbeNative(aNIn, anEdges, w), Diagonal(aPIn));
            double aP = MaxFillet(aPTrace, w => ProbePort(aPIn, anEdges, w), Diagonal(aPIn));
            aProbes += aNTrace.Count;
            output.WriteLine($"{aName}: native {aN:R} [{string.Join(' ', aNTrace)}], port {aP:R}");
            if (!aN.Equals(aP) || !aNTrace.SequenceEqual(aPTrace))
            {
                aDifferences.Add($"{aName}: native {aN:R} [{string.Join(' ', aNTrace)}], port {aP:R} [{string.Join(' ', aPTrace)}]");
            }
        }
        output.WriteLine($"MaxFillet: {aSearches} searches, {aProbes} probes; {aDifferences.Count} differ");
        foreach (string d in aDifferences)
        {
            output.WriteLine("  " + d);
        }
        Assert.All(aDifferences, d => Assert.True(KnownMaxFilletDifferences.Any(k => d.StartsWith(k, StringComparison.Ordinal)), d));
    }

    // Shape3D.MaxFillet: Search(0, 2 * diagonal), tolerance 0.1, 10 iterations (NaN when it gives up).
    private static double MaxFillet(List<string> theTrace, Func<double, bool> theProbe, double theDiagonal)
    {
        double Search(double windowMin, double windowMax, int iteration)
        {
            double windowMid = (windowMin + windowMax) / 2;
            if (iteration == 10)
            {
                return double.NaN;
            }
            bool ok = theProbe(windowMid);
            theTrace.Add((ok ? "+" : "-") + windowMid.ToString("G6", CultureInfo.InvariantCulture));
            if (!ok)
            {
                return Search(windowMin, windowMid, iteration + 1);
            }
            return windowMid - windowMin <= 0.1 ? windowMid : Search(windowMid, windowMax, iteration + 1);
        }
        return Search(0.0, 2 * theDiagonal, 0);
    }

    private static bool ProbeNative(N.TopoDS_Shape theInput, int[] theEdges, double theR)
    {
        try
        {
            using var anEdges = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theInput, N.TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
            using var aB = new N.BRepFilletAPI_MakeFillet(theInput);
            foreach (int k in theEdges)
            {
                aB.Add(theR, N.TopoDS.Edge(anEdges.FindKey(k)));
            }
            using N.TopoDS_Shape aRes = aB.Shape();
            using var anA = new N.BRepCheck_Analyzer(aRes);
            return anA.IsValid();
        }
        catch (N.OcctException)
        {
            return false;
        }
    }

    private static bool ProbePort(in Shape theInput, int[] theEdges, double theR)
    {
        try
        {
            NCollection_IndexedMap<Shape> anEdges = Map(theInput, ShapeType.Edge);
            BRepFilletAPI_MakeFillet aB = new(theInput);
            foreach (int k in theEdges)
            {
                aB.Add(theR, ShapeCasts.Edge(anEdges[k]));
            }
            return new BRepCheck_Analyzer(aB.Shape()).IsValid();
        }
        catch (FailureException)
        {
            return false;
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // One comparison

    /// <summary>A side's run: the exception, the status and diagnostics, the result, the history over the result's faces.</summary>
    internal sealed record Outcome(string? Error, string Status, Shape? Result, Dictionary<string, (bool Deleted, int[] Faces)> History);

    private static string Status(bool theDone, int theContours, IEnumerable<string> theDiagnostics) =>
        $"done={theDone} contours={theContours} " + string.Join(' ', theDiagnostics);

    internal Outcome RunNative(ShapeFixOracleRun theRun, Case theCase, N.TopoDS_Shape theInput)
    {
        using var anEdges = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theInput, N.TopAbs_ShapeEnum.TopAbs_EDGE, anEdges);
        using var aFaces = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theInput, N.TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
        using var anEF = new N.TopTools_IndexedDataMapOfShapeListOfShape();
        N.TopExp.MapShapesAndAncestors(theInput, N.TopAbs_ShapeEnum.TopAbs_EDGE, N.TopAbs_ShapeEnum.TopAbs_FACE, anEF);
        N.BRepFilletAPI_MakeFillet? aF = null;
        N.BRepFilletAPI_MakeChamfer? aC = null;
        try
        {
            N.BRepBuilderAPI_MakeShape aMaker;
            if (theCase.Chamfer)
            {
                aC = new N.BRepFilletAPI_MakeChamfer(theInput);
                aMaker = aC;
                foreach (CaseEdge e in theCase.Edges)
                {
                    N.TopoDS_Edge E = N.TopoDS.Edge(anEdges.FindKey(e.K));
                    using N.TopTools_ListOfShape aL = anEF.FindFromKey(E);
                    N.TopoDS_Face F = N.TopoDS.Face(aL.First());
                    if (e.Mode == 1)
                    {
                        aC.AddDA(e.R1, e.R2 * Math.PI / 180.0, E, F);
                    }
                    else if (e.R2 < 0)
                    {
                        aC.Add(e.R1, E);
                    }
                    else
                    {
                        aC.Add(e.R1, e.R2, E, F);
                    }
                }
            }
            else
            {
                aF = new N.BRepFilletAPI_MakeFillet(theInput);
                aMaker = aF;
                foreach (CaseEdge e in theCase.Edges)
                {
                    N.TopoDS_Edge E = N.TopoDS.Edge(anEdges.FindKey(e.K));
                    if (e.Pairs is double[] p)
                    {
                        using var anArray = new N.TColgp_Array1OfPnt2d(1, p.Length / 2);
                        for (int i = 0; i < p.Length / 2; i++)
                        {
                            anArray.SetValue(i + 1, new N.gp_Pnt2d(p[2 * i], p[2 * i + 1]));
                        }
                        aF.Add(anArray, E);
                    }
                    else if (e.R2 > 0)
                    {
                        aF.Add(e.R1, e.R2, E);
                    }
                    else
                    {
                        aF.Add(e.R1, E);
                    }
                }
            }
            aMaker.Build();
            bool isDone = aMaker.IsDone();
            string aStatus;
            if (aF is not null)
            {
                List<string> aDiag = [$"faulty={aF.NbFaultyContours()}:" + string.Join(',', Enumerable.Range(1, aF.NbFaultyContours()).Select(aF.FaultyContour))];
                aDiag.Add("statuses=" + string.Join(',', Enumerable.Range(1, aF.NbContours()).Select(ic => (int)aF.StripeStatus(ic))));
                aDiag.Add("computed=" + string.Join(',', Enumerable.Range(1, aF.NbContours()).Select(aF.NbComputedSurfaces)));
                aDiag.Add($"faultyvertices={aF.NbFaultyVertices()} hasresult={aF.HasResult()}");
                aStatus = Status(isDone, aF.NbContours(), aDiag);
            }
            else
            {
                aStatus = Status(isDone, aC!.NbContours(), Enumerable.Range(1, aC.NbContours()).Select(ic => $"edges{ic}={aC.NbEdges(ic)}"));
            }
            if (!isDone)
            {
                return new Outcome(null, aStatus, null, []);
            }
            using N.TopoDS_Shape aRes = aMaker.Shape();
            string aText = theRun.NativeText(aRes);
            using var aResFaces = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(aRes, N.TopAbs_ShapeEnum.TopAbs_FACE, aResFaces);
            Dictionary<string, (bool, int[])> aHistory = [];
            for (int i = 1; i <= aFaces.Extent(); i++)
            {
                using N.TopoDS_Shape S = aFaces.FindKey(i);
                using N.TopTools_ListOfShape aL = aMaker.Modified(S);
                aHistory["F" + i] = (aMaker.IsDeleted(S), [.. aL.Select(x => aResFaces.FindIndex(x))]);
            }
            for (int k = 1; k <= anEdges.Extent(); k++)
            {
                using N.TopoDS_Shape S = anEdges.FindKey(k);
                using N.TopTools_ListOfShape aL = aMaker.Generated(S);
                if (aL.Extent() > 0)
                {
                    aHistory["E" + k] = (false, [.. aL.Select(x => aResFaces.FindIndex(x))]);
                }
            }
            // the input's solids (unit 7R3, decision 1161): IsDeleted, and the faces of the solids Modified gives
            using var aSolids = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theInput, N.TopAbs_ShapeEnum.TopAbs_SOLID, aSolids);
            for (int i = 1; i <= aSolids.Extent(); i++)
            {
                using N.TopoDS_Shape S = aSolids.FindKey(i);
                using N.TopTools_ListOfShape aL = aMaker.Modified(S);
                List<int> aSolidFaces = [];
                foreach (N.TopoDS_Shape aSo in aL)
                {
                    using var aSoFaces = new N.TopTools_IndexedMapOfShape();
                    N.TopExp.MapShapes(aSo, N.TopAbs_ShapeEnum.TopAbs_FACE, aSoFaces);
                    for (int j = 1; j <= aSoFaces.Extent(); j++)
                    {
                        using N.TopoDS_Shape aFace = aSoFaces.FindKey(j);
                        aSolidFaces.Add(aResFaces.FindIndex(aFace));
                    }
                }
                GC.KeepAlive(aL); // decision 1109: the list outlives its native iterator
                aHistory["So" + i] = (aMaker.IsDeleted(S), [.. aSolidFaces]);
            }
            return new Outcome(null, aStatus, BooleanOracleTests.ReadPort(aText), aHistory);
        }
        catch (N.OcctException e)
        {
            return new Outcome(ShapeFixOracleRun.ErrorOf(e), "", null, []);
        }
        finally
        {
            aF?.Dispose();
            aC?.Dispose();
        }
    }

    internal static Outcome RunPort(Case theCase, Shape theInput)
    {
        NCollection_IndexedMap<Shape> anEdges = Map(theInput, ShapeType.Edge);
        NCollection_IndexedMap<Shape> aFaces = Map(theInput, ShapeType.Face);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> EF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theInput, ShapeType.Edge, ShapeType.Face, EF);
        try
        {
            BRepFilletAPI_LocalOperation aMaker;
            string aStatus;
            if (theCase.Chamfer)
            {
                BRepFilletAPI_MakeChamfer aC = new(theInput);
                aMaker = aC;
                foreach (CaseEdge e in theCase.Edges)
                {
                    Edge E = ShapeCasts.Edge(anEdges[e.K]);
                    Face F = ShapeCasts.Face(EF.FindFromKey(E).First());
                    if (e.Mode == 1)
                    {
                        aC.AddDA(e.R1, e.R2 * Math.PI / 180.0, E, F);
                    }
                    else if (e.R2 < 0)
                    {
                        aC.Add(e.R1, E);
                    }
                    else
                    {
                        aC.Add(e.R1, e.R2, E, F);
                    }
                }
                aC.Build();
                aStatus = Status(aC.IsDone(), aC.NbContours(), Enumerable.Range(1, aC.NbContours()).Select(ic => $"edges{ic}={aC.NbEdges(ic)}"));
            }
            else
            {
                BRepFilletAPI_MakeFillet aF = new(theInput);
                aMaker = aF;
                foreach (CaseEdge e in theCase.Edges)
                {
                    Edge E = ShapeCasts.Edge(anEdges[e.K]);
                    if (e.Pairs is double[] p)
                    {
                        NCollection_Array1<Point2D> anArray = new(1, p.Length / 2);
                        for (int i = 0; i < p.Length / 2; i++)
                        {
                            anArray.SetValue(i + 1, new Point2D(p[2 * i], p[2 * i + 1]));
                        }
                        aF.Add(anArray, E);
                    }
                    else if (e.R2 > 0)
                    {
                        aF.Add(e.R1, e.R2, E);
                    }
                    else
                    {
                        aF.Add(e.R1, E);
                    }
                }
                aF.Build();
                List<string> aDiag = [$"faulty={aF.NbFaultyContours()}:" + string.Join(',', Enumerable.Range(1, aF.NbFaultyContours()).Select(aF.FaultyContour))];
                aDiag.Add("statuses=" + string.Join(',', Enumerable.Range(1, aF.NbContours()).Select(ic => (int)aF.StripeStatus(ic))));
                aDiag.Add("computed=" + string.Join(',', Enumerable.Range(1, aF.NbContours()).Select(aF.NbComputedSurfaces)));
                aDiag.Add($"faultyvertices={aF.NbFaultyVertices()} hasresult={aF.HasResult()}");
                aStatus = Status(aF.IsDone(), aF.NbContours(), aDiag);
            }
            if (!aMaker.IsDone())
            {
                return new Outcome(null, aStatus, null, []);
            }
            Shape aRes = aMaker.Shape();
            NCollection_IndexedMap<Shape> aResFaces = Map(aRes, ShapeType.Face);
            Dictionary<string, (bool, int[])> aHistory = [];
            for (int i = 1; i <= aFaces.Extent(); i++)
            {
                aHistory["F" + i] = (aMaker.IsDeleted(aFaces[i]), [.. aMaker.Modified(aFaces[i]).Select(x => aResFaces.FindIndex(x))]);
            }
            for (int k = 1; k <= anEdges.Extent(); k++)
            {
                IReadOnlyList<Shape> aL = aMaker.Generated(anEdges[k]);
                if (aL.Count > 0)
                {
                    aHistory["E" + k] = (false, [.. aL.Select(x => aResFaces.FindIndex(x))]);
                }
            }
            NCollection_IndexedMap<Shape> aSolids = Map(theInput, ShapeType.Solid);
            for (int i = 1; i <= aSolids.Extent(); i++)
            {
                aHistory["So" + i] = (aMaker.IsDeleted(aSolids[i]),
                                      [.. aMaker.Modified(aSolids[i]).SelectMany(x => Map(x, ShapeType.Face)).Select(x => aResFaces.FindIndex(x))]);
            }
            return new Outcome(null, aStatus, aRes, aHistory);
        }
        catch (Exception e) when (e is FailureException or NullReferenceException or IndexOutOfRangeException)
        {
            return new Outcome(global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()), "", null, []);
        }
    }

    internal void Compare(ShapeFixOracleRun theRun, Tally theTally, Case theCase, Shape theInput)
    {
        if (theRun.Read(BRepOffsetAPIMakeThickSolidOracleTests.Native(theRun, theInput)) is not (N.TopoDS_Shape aNIn, Shape aPIn))
        {
            return;
        }
        theTally.Runs++;
        string aWhat = theCase.Name;
        string aPInText = theRun.PortText(aPIn);
        Outcome aN = RunNative(theRun, theCase, aNIn);
        Outcome aP = RunPort(theCase, aPIn);
        if (theRun.PortText(aPIn) != aPInText)
        {
            theTally.Differences.Add($"{aWhat}: the port's input changed");
        }
        if (aP.Result is Shape aPR && !aPR.IsNull && !aPR.TShape()!.IsFrozen)
        {
            theTally.Differences.Add($"{aWhat}: the port's result isn't frozen");
        }
        if (aN.Error is not null || aP.Error is not null)
        {
            if (aN.Error == aP.Error)
            {
                theTally.SameError++;
                theTally.Errors[aN.Error!] = theTally.Errors.GetValueOrDefault(aN.Error!) + 1;
            }
            else
            {
                theTally.Differences.Add($"{aWhat}: native {aN.Error ?? aN.Status}, port {aP.Error ?? aP.Status}");
            }
            return;
        }
        if (aN.Status != aP.Status)
        {
            theTally.Differences.Add($"{aWhat}: native {aN.Status} | port {aP.Status}");
            return;
        }
        if (aN.Result is null)
        {
            theTally.NotDone++;
            return;
        }
        theTally.Done++;
        List<string> d = Geometric(aN, aP);
        bool aNValid = new BRepCheck_Analyzer(aN.Result.Value).IsValid();
        bool aPValid = new BRepCheck_Analyzer(aP.Result!.Value).IsValid();
        if (d.Count == 0)
        {
            theTally.Equal++;
        }
        else if (!aNValid)
        {
            theTally.NativeInvalid++;
            theTally.Attributed.Add($"{aWhat}: native's result is invalid {ReconFaults(aN.Result.Value)}; {string.Join("; ", d.Take(3))}");
        }
        else if (!aPValid)
        {
            theTally.PortInvalid.Add($"{aWhat}: {ReconFaults(aP.Result.Value)}; {string.Join("; ", d.Take(Environment.GetEnvironmentVariable("OCCTNET_7N3_DETAIL") is null ? 3 : 100))}");
        }
        else
        {
            theTally.Differences.Add($"{aWhat}: {string.Join("; ", d.Take(4))}");
        }
    }

    private static double Volume(in Shape S)
    {
        Properties p = new();
        BRepGProp.VolumeProperties(S, p);
        return p.Mass();
    }

    private static double Area(in Shape S)
    {
        Properties p = new();
        BRepGProp.SurfaceProperties(S, p);
        return p.Mass();
    }

    private static bool Close(double a, double b, double theRel) =>
        Math.Abs(a - b) <= theRel * Math.Max(1.0, Math.Max(Math.Abs(a), Math.Abs(b)));

    // The faults BRepCheck finds, by sub-shape type.
    internal static string ReconFaults(in Shape S)
    {
        BRepCheck_Analyzer an = new(S);
        SortedSet<string> aFaults = [];
        foreach (ShapeType T in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Wire, ShapeType.Edge, ShapeType.Vertex })
        {
            for (Explorer exp = new(S, T); exp.More(); exp.Next())
            {
                if (an.Result(exp.Current()) is BRepCheck_Result r)
                {
                    foreach (BRepCheck_Status st in r.Status())
                    {
                        if (st != BRepCheck_Status.BRepCheck_NoError)
                        {
                            aFaults.Add(Kerf.Engine.Foundation.OcctTypeNames.Of(T).Replace("TopAbs_", "") + ":" + Kerf.Engine.Foundation.OcctTypeNames.Of(st).Replace("BRepCheck_", ""));
                        }
                    }
                }
            }
        }
        return "{" + string.Join(" ", aFaults) + "}";
    }

    // (surface type, area) of each face of a list, sorted
    private static List<(string Type, double Area)> Keys(in Shape theResult, int[] theFaces)
    {
        NCollection_IndexedMap<Shape> M = Map(theResult, ShapeType.Face);
        return [.. theFaces.Select(i => i > 0
                                       ? (Kerf.Engine.Foundation.OcctTypeNames.Of(new FaceSurfaceAdaptor(ShapeCasts.Face(M[i])).GetType()).Replace("GeomAbs_", ""), Area(M[i]))
                                       : ("out", 0.0))
                           .OrderBy(k => k.Item1, StringComparer.Ordinal).ThenBy(k => k.Item2)];
    }

    private static bool SameKeys(List<(string Type, double Area)> a, List<(string Type, double Area)> b) =>
        a.Count == b.Count && a.Zip(b).All(p => p.First.Type == p.Second.Type && Close(p.First.Area, p.Second.Area, 1.0e-5));

    private static string KeysText(List<(string Type, double Area)> k) =>
        "[" + string.Join(", ", k.Select(x => x.Type + " " + x.Area.ToString("G6", CultureInfo.InvariantCulture))) + "]";

    // What differs geometrically (empty: equal)
    internal static List<string> Geometric(Outcome theN, Outcome theP)
    {
        List<string> d = [];
        Shape aN = theN.Result!.Value, aP = theP.Result!.Value;
        double v1 = Volume(aN), v2 = Volume(aP), a1 = Area(aN), a2 = Area(aP);
        if (!Close(v1, v2, 1.0e-6))
        {
            d.Add($"volume {v1:G9} | {v2:G9}");
        }
        if (!Close(a1, a2, 1.0e-6))
        {
            d.Add($"area {a1:G9} | {a2:G9}");
        }
        foreach (ShapeType T in new[] { ShapeType.Solid, ShapeType.Shell, ShapeType.Face, ShapeType.Edge, ShapeType.Vertex })
        {
            int n1 = Map(aN, T).Extent(), n2 = Map(aP, T).Extent();
            if (n1 != n2)
            {
                d.Add($"{Kerf.Engine.Foundation.OcctTypeNames.Of(T).Replace("TopAbs_", "")} {n1} | {n2}");
            }
        }
        bool ok1 = new BRepCheck_Analyzer(aN).IsValid(), ok2 = new BRepCheck_Analyzer(aP).IsValid();
        if (ok1 != ok2)
        {
            d.Add($"valid {ok1} | {ok2}");
        }
        Box b1 = new(), b2 = new();
        BRepBndLib.AddOptimal(aN, ref b1, false, false);
        BRepBndLib.AddOptimal(aP, ref b2, false, false);
        if (!b1.IsVoid && !b2.IsVoid)
        {
            b1.Get(out double x1, out double y1, out double z1, out double X1, out double Y1, out double Z1);
            b2.Get(out double x2, out double y2, out double z2, out double X2, out double Y2, out double Z2);
            double aDiag = Math.Sqrt(b1.SquareExtent());
            double[] e = [x1 - x2, y1 - y2, z1 - z2, X1 - X2, Y1 - Y2, Z1 - Z2];
            if (e.Any(x => Math.Abs(x) > 1.0e-4 * aDiag))
            {
                d.Add("box");
            }
        }
        foreach (string aKey in theN.History.Keys.Union(theP.History.Keys).Order(StringComparer.Ordinal))
        {
            (bool Deleted, int[] Faces) h1 = theN.History.GetValueOrDefault(aKey, (false, [])),
                                        h2 = theP.History.GetValueOrDefault(aKey, (false, []));
            List<(string, double)> k1 = Keys(aN, h1.Faces), k2 = Keys(aP, h2.Faces);
            if (h1.Deleted != h2.Deleted || !SameKeys(k1, k2))
            {
                d.Add($"history {aKey} {(h1.Deleted ? "deleted " : "")}{KeysText(k1)} | {(h2.Deleted ? "deleted " : "")}{KeysText(k2)}");
            }
        }
        return d;
    }

    internal void Report(Tally theTally, string theWhat, IReadOnlyDictionary<string, string> theKnown)
    {
        output.WriteLine($"{theWhat}: {theTally.Runs} runs: {theTally.Done} done on both sides, {theTally.NotDone} not done on both "
                         + $"(the same diagnostics), {theTally.SameError} the same exception");
        output.WriteLine($"  results: {theTally.Equal} geometrically native's (history as sets included), {theTally.NativeInvalid} "
                         + $"where native's is invalid (attributed), {theTally.PortInvalid.Count} where only the port's is invalid, "
                         + $"{theTally.Differences.Count} other differences");
        output.WriteLine($"  exceptions: {string.Join(", ", theTally.Errors.Select(p => $"{p.Key} {p.Value}"))}");
        foreach (string a in theTally.Attributed)
        {
            output.WriteLine("  native invalid: " + a);
        }
        foreach (string a in theTally.PortInvalid)
        {
            output.WriteLine("  port invalid: " + a);
        }
        foreach (string a in theTally.Differences)
        {
            output.WriteLine("  differs: " + a);
        }
        List<string> anUnknown = [.. theTally.PortInvalid.Concat(theTally.Differences)
                                             .Where(x => !theKnown.Keys.Any(k => x.StartsWith(k + " ", StringComparison.Ordinal)
                                                                                    || x.StartsWith(k + ":", StringComparison.Ordinal)))];
        foreach (string a in anUnknown)
        {
            output.WriteLine("  NOT ATTRIBUTED: " + a);
        }
        Assert.Empty(anUnknown);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The differences, each attributed (docs/decisions.md, unit 7N3)

    // keys: a case's name or the start of its names, before a space
    internal static readonly Dictionary<string, string> KnownPrimitiveDifferences = new(StringComparer.Ordinal)
    {
        // unit 7N3 attributed 33 runs here to the seam band (a face with a seam cut by the fillet kept the band the
        // fillet removes); unit 7R3 fixed it (decision 1155). Left (decision 1156): a radius of 28.9 on the T's
        // junction (cylinders of radius 3 and 2): the end circle of the small cylinder has 11 points whose
        // transitions do not alternate, and native keeps two pieces of zero area of that cylinder; the reconstruction
        // keeps the band round the seam the rim crosses. Not worth a rule.
        ["cylinder T e2 too large"] = "reconstruction (7R3): a radius ten times the cylinder's, transitions that do not alternate on the end circle",
    };

    internal static readonly Dictionary<string, string> KnownDataOccDifferences = new(StringComparer.Ordinal)
    {
    };

    internal static readonly Dictionary<string, string> KnownGTestDifferences = new(StringComparer.Ordinal)
    {
        // OCC1077's third cut and OCC426: the seam band, fixed by unit 7R3 (decision 1155)
    };

    internal static readonly string[] KnownMaxFilletDifferences = [];
}
