// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;
using static Kerf.Offsets.OffsetMode;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The 3D offset (unit 7F) against native OCCT: BRepOffsetAPI_MakeOffsetShape (PerformByJoin), BRepOffsetAPI_MakeThickSolid
/// (MakeThickSolidByJoin) and BRepOffset_MakeOffset (the nine-argument Initialize, then MakeOffsetShape), on plan section
/// 3's cases: primitives, Boolean results and data/occ's shapes; both joins, both signs, with and without intersection;
/// thickening of faces and shells; build123d.NET's Hollow, Offset3D, Thicken, OffsetCore and OffsetTopodsFace calls.
/// </summary>
/// <remarks>
/// Both sides read the input from the same BRep text (the port's frozen, as a caller's). Compared: the exception, IsDone
/// and Error(); the result as BRep text (bytes, then the Free column masked, then canonically, rerunning native where it
/// differs to see whether native varies); <c>Generated</c>, <c>Modified</c> and <c>IsDeleted</c> of every vertex, edge and
/// face of the input, as indices of the result's sub-shapes (or canonical keys where the results agree only canonically).
/// The port's input is checked unchanged and its result frozen.
/// </remarks>
public sealed class BRepOffsetAPIMakeThickSolidOracleTests(ITestOutputHelper output)
{
    internal enum Kind { OffsetShape, ThickSolid, MakeOffset }

    internal sealed record Run(string Name, Kind Kind, double Offset, double Tol = 1.0e-7, JoinKind Join = Arc,
                               bool Inter = false, bool Thickening = false, bool RemoveIntEdges = false, int[]? Closing = null,
                               bool ThreeArgs = false);

    internal sealed class Tally
    {
        public int Runs, Done, NotDone, SameError, Identical, FreeMasked, Canonical, NativeVaries, NullResults, Histories,
                   HistoriesSame, HistoriesCanonical, InputsChanged, NotFrozen, NativeVariant, NativeVariesGeometric, NativeInputsChanged;
        public readonly SortedDictionary<string, int> Errors = new(StringComparer.Ordinal);
        public readonly List<string> Differences = [];
        public readonly List<string> HistoryDifferences = [];
        public readonly List<string> Attributed = [];
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Inputs

    private static Shape Box(double x, double y, double z) => new MakeBox(x, y, z).Shape();

    private static Shape Fuse(in Shape a, in Shape b) => new Fuse(a, b).Shape();

    private static Shape Cut(in Shape a, in Shape b) => new Cut(a, b).Shape();

    private static Shape Unified(in Shape a)
    {
        var aU = new UnifySameDomain(a);
        aU.Build();
        return aU.Shape();
    }

    private static Shape Moved(in Shape theS, double x, double y, double z)
    {
        Transform3D aT = new Transform3D().WithTranslation(new Vector3D(x, y, z));
        return new Transform(theS, aT, true).Shape();
    }

    private static Frame3D Ax(double x, double y, double z) => new(new Point3D(x, y, z), new Direction3D(0, 0, 1));

    private static Shape Prism(double h, params (double X, double Y)[] thePoints)
    {
        var aP = new MakePolygon();
        foreach ((double x, double y) in thePoints)
        {
            aP.Add(new Point3D(x, y, 0));
        }
        aP.Close();
        Face aF = new MakeFace(aP.Wire()).Face();
        return new ApiMakePrism(aF, new Vector3D(0, 0, h)).Shape();
    }

    /// <summary>Solids: primitives and Boolean results.</summary>
    internal static List<(string Name, Shape Shape)> Solids() =>
    [
        ("box 10x20x30", Box(10, 20, 30)),
        ("cube 10", Box(10, 10, 10)),
        ("cylinder r5 h12", new ApiMakeCylinder(5, 12).Shape()),
        ("cone r6 r2 h10", new ApiMakeCone(6, 2, 10).Shape()),
        ("cone r5 r0 h10", new ApiMakeCone(5, 0, 10).Shape()),
        ("sphere r7", new MakeSphere(7).Shape()),
        ("torus 10 3", new MakeTorus(10, 3).Shape()),
        ("L prism", Prism(8, (0, 0), (20, 0), (20, 6), (6, 6), (6, 15), (0, 15))),
        ("hexagon prism", Prism(5, (4, 0), (2, 3.4641016151377544), (-2, 3.4641016151377544), (-4, 0), (-2, -3.4641016151377544),
                                (2, -3.4641016151377544))),
        ("wedge", new MakeWedge(10, 8, 6, 4).Shape()),
        ("L fuse", Unified(Fuse(Box(10, 10, 10), Box(20, 5, 5)))),
        ("stepped fuse", Unified(Fuse(Box(20, 20, 5), Moved(Box(10, 20, 5), 0, 0, 5)))),
        ("box with hole", Cut(Box(20, 20, 8), new ApiMakeCylinder(Ax(10, 10, -1), 4, 10).Shape())),
        ("box with pocket", Cut(Box(20, 20, 10), Moved(Box(10, 10, 10), 5, 5, 4))),
        ("box and cylinder fuse", Fuse(Box(20, 20, 5), new ApiMakeCylinder(Ax(10, 10, 5), 4, 8).Shape())),
        ("cylinder T", Fuse(new ApiMakeCylinder(Ax(0, 0, 0), 3, 20).Shape(),
                            new ApiMakeCylinder(new Frame3D(new Point3D(-10, 0, 10), new Direction3D(1, 0, 0)), 2, 20).Shape())),
        ("box common sphere", new Common(Box(10, 10, 10), new MakeSphere(new Point3D(5, 5, 5), 6.5).Shape()).Shape()),
        ("box minus sphere", Cut(Box(10, 10, 10), new MakeSphere(new Point3D(10, 10, 10), 5).Shape())),
    ];

    /// <summary>Faces and shells to offset and thicken.</summary>
    internal static List<(string Name, Shape Shape)> Sheets()
    {
        Shape aBox = Box(10, 20, 30);
        var aFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aBox, ShapeType.Face, aFaces);
        Shape aCyl = new ApiMakeCylinder(5, 12).Shape();
        var aCylFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aCyl, ShapeType.Face, aCylFaces);
        Shape aSph = new MakeSphere(new Point3D(0, 0, 0), 6, 0.3, 1.2, 2.0).Shape();
        var aSphFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aSph, ShapeType.Face, aSphFaces);
        BrepBuilder aB = new();
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, aFaces[0]);
        aB.Add(aShell, aFaces[2]);
        aB.Add(aShell, aFaces[4]);
        Shell aShell2 = new();
        aB.MakeShell(ref aShell2);
        for (int i = 1; i <= 5; i++)
        {
            aB.Add(aShell2, aFaces[i - 1]);
        }
        // a B-spline face: a lofted patch
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i * 3, j * 3, ((i + j) % 3) * 0.8);
            }
        }
        var aSurf = new PointsToBSplineSurface(aPoles).Surface();
        Face aBsp = new MakeFace(aSurf, 1.0e-7).Face();
        return
        [
            ("planar face", aFaces[0]),
            ("planar face reversed", aFaces[1]),
            ("cylindrical face", aCylFaces[0]),
            ("spherical patch", aSphFaces[0]),
            ("B-spline face", aBsp),
            ("open shell of 3", aShell),
            ("open shell of 5", aShell2),
        ];
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Tests

    [Fact]
    public void Primitives_and_Boolean_results_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "offset");
        foreach ((string aName, Shape aShape) in Tape.Budget(Solids()))
        {
            double aD = Diagonal(aShape) * 0.04;
            foreach (Run aCase in SolidRuns(aName, aD))
            {
                Compare(aRun, aTally, aCase, aShape);
            }
        }
        Report(aTally, "primitives and Boolean results");
    }

    [Fact]
    public void Faces_and_shells_thickened_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "offset");
        foreach ((string aName, Shape aShape) in Sheets())
        {
            double aD = Diagonal(aShape) * 0.04;
            foreach (double aSign in new[] { 1.0, -1.0 })
            {
                double d = aSign * aD;
                // build123d.NET's Thicken, OffsetCore and OffsetTopodsFace
                Compare(aRun, aTally, new Run($"{aName} thicken {d:g4}", Kind.MakeOffset, d, 1.0e-5, Intersection, true, true, true), aShape);
                Compare(aRun, aTally, new Run($"{aName} offset core {d:g4}", Kind.MakeOffset, d, 1.0e-5, Intersection, true, false, true), aShape);
                Compare(aRun, aTally, new Run($"{aName} offset face {d:g4}", Kind.MakeOffset, d, 1.0e-5, ThreeArgs: true), aShape);
                // the arc join, thickened and not
                Compare(aRun, aTally, new Run($"{aName} thicken arc {d:g4}", Kind.MakeOffset, d, 1.0e-5, Arc, false, true), aShape);
                Compare(aRun, aTally, new Run($"{aName} offset shape arc {d:g4}", Kind.OffsetShape, d, 1.0e-5, Arc), aShape);
                Compare(aRun, aTally, new Run($"{aName} offset shape inter {d:g4}", Kind.OffsetShape, d, 1.0e-5, Intersection), aShape);
            }
        }
        Report(aTally, "faces and shells");
    }

    [Fact]
    public void Data_occ_shapes_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "offset");
        string aDir = Path.Combine(ShapeFixOracleRun.Metadata("OcctSourceDir"), "data", "occ");
        var aWatch = Stopwatch.StartNew();
        foreach (string aFile in Tape.Budget(Directory.GetFiles(aDir, "*.brep").Order(StringComparer.Ordinal), isSpread: true))
        {
            if (new FileInfo(aFile).Length > 300_000)
            {
                continue;
            }
            Assert.True(BrepTools.Read(out Shape aShape, aFile));
            if (aShape.IsNull || !new Explorer(aShape, ShapeType.Face).More())
            {
                continue;
            }
            string aName = Path.GetFileNameWithoutExtension(aFile);
            double aD = Diagonal(aShape) * 0.02;
            bool isSolid = new Explorer(aShape, ShapeType.Solid).More();
            foreach (double aSign in new[] { 1.0, -1.0 })
            {
                double d = aSign * aD;
                foreach (JoinKind aJoin in new[] { Arc, Intersection })
                {
                    Compare(aRun, aTally, new Run($"{aName} offset {aJoin} {d:g4}", Kind.OffsetShape, d, 1.0e-7, aJoin), aShape);
                }
                if (isSolid)
                {
                    Compare(aRun, aTally, new Run($"{aName} hollow arc {d:g4}", Kind.ThickSolid, d, 1.0e-4, Arc, true, Closing: [1]), aShape);
                    Compare(aRun, aTally, new Run($"{aName} hollow inter {d:g4}", Kind.ThickSolid, d, 1.0e-4, Intersection, true, Closing: [1]), aShape);
                    Compare(aRun, aTally, new Run($"{aName} offset3d inter {d:g4}", Kind.ThickSolid, d, 1.0e-4, Intersection, true, RemoveIntEdges: true, Closing: []), aShape);
                }
                else
                {
                    Compare(aRun, aTally, new Run($"{aName} thicken {d:g4}", Kind.MakeOffset, d, 1.0e-5, Intersection, true, true, true), aShape);
                }
            }
        }
        output.WriteLine($"time {aWatch.Elapsed.TotalSeconds:F0} s");
        Report(aTally, "data/occ shapes up to 300 kB");
    }

    internal static IEnumerable<Run> SolidRuns(string theName, double theD)
    {
        foreach (double aSign in new[] { 1.0, -1.0 })
        {
            double d = aSign * theD;
            foreach (JoinKind aJoin in new[] { Arc, Intersection })
            {
                string j = aJoin == Arc ? "arc" : "inter";
                yield return new Run($"{theName} offset {j} {d:g4}", Kind.OffsetShape, d, 1.0e-7, aJoin);
                yield return new Run($"{theName} offset {j} complete {d:g4}", Kind.OffsetShape, d, 1.0e-7, aJoin, true, RemoveIntEdges: true);
                // build123d.NET's Hollow (one opening) and Offset3D (no opening, remove the internal edges)
                yield return new Run($"{theName} hollow {j} {d:g4}", Kind.ThickSolid, d, 1.0e-4, aJoin, true, Closing: [1]);
                yield return new Run($"{theName} hollow2 {j} {d:g4}", Kind.ThickSolid, d, 1.0e-4, aJoin, true, Closing: [1, 2]);
                yield return new Run($"{theName} offset3d {j} {d:g4}", Kind.ThickSolid, d, 1.0e-4, aJoin, true, RemoveIntEdges: true, Closing: []);
                yield return new Run($"{theName} thick {j} no inter {d:g4}", Kind.ThickSolid, d, 1.0e-7, aJoin, false, Closing: [2]);
            }
        }
    }

    private static double Diagonal(in Shape theShape)
    {
        Box aBox = new();
        Bounds.Add(theShape, ref aBox);
        return Math.Sqrt(aBox.SquareExtent());
    }

    // ---------------------------------------------------------------------------------------------------------------
    // One comparison

    internal sealed record Outcome(string? Error, string Status, string? Text, string History, Shape? PortResult);

    internal void Compare(ShapeFixOracleRun theRun, Tally theTally, Run theCase, Shape theInput)
    {
        string aWhat = $"{theCase.Name} [{theCase.Kind} {theCase.Join} inter={theCase.Inter} thick={theCase.Thickening}]";
        if (theRun.Read(Native(theRun, theInput)) is not (N.TopoDS_Shape aNIn, Shape aPIn))
        {
            return;
        }
        theTally.Runs++;
        string aPInText = theRun.PortText(aPIn);

        string aNInText = theRun.NativeText(aNIn);
        Outcome aN = RunNative(theRun, theCase, aNIn);
        if (theRun.NativeText(aNIn) != aNInText)
        {
            theTally.NativeInputsChanged++;
        }
        Outcome aP = RunPort(theRun, theCase, aPIn);

        if (theRun.PortText(aPIn) != aPInText)
        {
            theTally.InputsChanged++;
            theTally.Differences.Add($"{aWhat}: the port's input changed");
        }
        if (aP.PortResult is Shape aPR && !aPR.IsNull && !aPR.TShape()!.IsFrozen)
        {
            theTally.NotFrozen++;
            theTally.Differences.Add($"{aWhat}: the port's result isn't frozen");
        }
        if (aN.Error is not null || aP.Error is not null)
        {
            if (aN.Error == aP.Error || (aN.Error, aP.Error) is ("OSD_Exception_ACCESS_VIOLATION", "NullReferenceException"))
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
            theTally.Differences.Add($"{aWhat}: native {aN.Status}, port {aP.Status}");
            return;
        }
        if (aN.Text is null || aP.Text is null)
        {
            theTally.NotDone++;
            if ((aN.Text is null) != (aP.Text is null))
            {
                theTally.Differences.Add($"{aWhat}: native result {(aN.Text is null ? "none" : "present")}, port {(aP.Text is null ? "none" : "present")}");
            }
            return;
        }
        theTally.Done++;
        if (aN.Text.Length == 0)
        {
            theTally.NullResults++;
        }
        BRepTextDiff aDiff = BRepTextComparison.Compare(aN.Text, aP.Text);
        bool isSameMap = true;
        if (aDiff.IsIdentical)
        {
            theTally.Identical++;
        }
        else if (aDiff.IsIdenticalMaskingFree)
        {
            theTally.FreeMasked++;
        }
        else
        {
            isSameMap = false;
            Shape aNResP = BooleanOracleTests.ReadPort(aN.Text), aPResP = BooleanOracleTests.ReadPort(aP.Text);
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNResP, aPResP, new CanonicalOptions { CheckValidity = false });
            if (aCanon.IsIdentical)
            {
                theTally.Canonical++;
            }
            else
            {
                // native's other runs: where native varies (its maps keyed by address), the port's result is compared
                // with each variant, then geometrically
                bool isNativeVarying = false, isVariant = false, isGeometricVariant = false;
                var aVolumes = new SortedSet<string>(StringComparer.Ordinal);
                for (int k = 0; k < 8 && !isVariant; k++)
                {
                    Outcome aN2 = RunNative(theRun, theCase, theRun.Read(Native(theRun, theInput))!.Value.N);
                    if (aN2.Text == aN.Text)
                    {
                        continue;
                    }
                    isNativeVarying = true;
                    if (aN2.Text is not null && aN2.Status == aP.Status)
                    {
                        BRepTextDiff aDiff2 = BRepTextComparison.Compare(aN2.Text, aP.Text);
                        Shape aN2ResP = BooleanOracleTests.ReadPort(aN2.Text);
                        CanonicalComparison aCanon2 = CanonicalShapeComparer.Compare(aN2ResP, aPResP, new CanonicalOptions { CheckValidity = false });
                        isVariant = aDiff2.IsIdenticalMaskingFree || aCanon2.IsIdentical;
                        isGeometricVariant |= Geometric(aN2ResP, aPResP, aCanon2) is null;
                        aVolumes.Add(Geometric(aN2ResP, aPResP, aCanon2) ?? "same");
                    }
                }
                if (isNativeVarying)
                {
                    theTally.NativeVaries++;
                }
                if (isVariant)
                {
                    theTally.NativeVariant++;
                    return;
                }
                if (isNativeVarying && (isGeometricVariant || Geometric(aNResP, aPResP, aCanon) is null))
                {
                    theTally.NativeVariesGeometric++;
                    theTally.Attributed.Add($"{aWhat}: native varies; the port's result is geometrically native's");
                    return;
                }
                theTally.Differences.Add($"{aWhat}: {aDiff}; canonically {aCanon}{(isNativeVarying ? " (native varies; geometrically: " + Geometric(aNResP, aPResP, aCanon) + "; native's variants: " + string.Join(" | ", aVolumes) + ")" : "")}");
                return;
            }
        }
        // the histories
        theTally.Histories++;
        if (isSameMap)
        {
            if (aN.History == aP.History)
            {
                theTally.HistoriesSame++;
            }
            else
            {
                theTally.HistoryDifferences.Add($"{aWhat}: histories differ: {DeterminismHarness.FirstDifference(aN.History, aP.History)}");
            }
            return;
        }
        string aNKeyed = Keyed(aN.History, BooleanOracleTests.ReadPort(aN.Text));
        string aPKeyed = Keyed(aP.History, BooleanOracleTests.ReadPort(aP.Text));
        if (aNKeyed == aPKeyed)
        {
            theTally.HistoriesCanonical++;
        }
        else
        {
            theTally.HistoryDifferences.Add($"{aWhat}: histories differ canonically: {DeterminismHarness.FirstDifference(aNKeyed, aPKeyed)}");
        }
    }

    // Null if the two results agree geometrically: the same number of each type of sub-shape, the same volume, area and
    // centroid (relative 1e-7), the same validity; else what differs.
    internal static string? Geometric(in Shape theN, in Shape theP, CanonicalComparison theCanon)
    {
        if (!theCanon.CountsEqual)
        {
            return "counts " + string.Join(' ', theCanon.Counts.Select(c => $"{c.Key}:{c.Value.Native}/{c.Value.Port}"));
        }
        Properties aNV = new(), aPV = new(), aNS = new(), aPS = new();
        MassPropertiesBuilder.VolumeProperties(theN, aNV);
        MassPropertiesBuilder.VolumeProperties(theP, aPV);
        MassPropertiesBuilder.SurfaceProperties(theN, aNS);
        MassPropertiesBuilder.SurfaceProperties(theP, aPS);
        double aScale = Math.Max(1.0, Math.Abs(aNS.Mass()));
        bool Near(double a, double b, double s) => Math.Abs(a - b) <= 1.0e-7 * Math.Max(s, Math.Max(Math.Abs(a), Math.Abs(b)));
        bool aNValid0 = new Analyzer(theN).IsValid(), aPValid0 = new Analyzer(theP).IsValid();
        if (!aNValid0 && !aPValid0 && Near(aNS.Mass(), aPS.Mass(), 1.0))
        {
            // both invalid: the volume of an invalid result depends on how its shells happen to be oriented (native's own
            // volume varies from run to run there); the counts and the area agree
            return null;
        }
        if (!Near(aNV.Mass(), aPV.Mass(), 1.0) || !Near(aNS.Mass(), aPS.Mass(), 1.0)
            || aNS.CentreOfMass().DistanceTo(aPS.CentreOfMass()) > 1.0e-7 * Math.Sqrt(aScale))
        {
            return $"volume {aNV.Mass():R}/{aPV.Mass():R} area {aNS.Mass():R}/{aPS.Mass():R}";
        }
        bool aNValid = new Analyzer(theN).IsValid(), aPValid = new Analyzer(theP).IsValid();
        return aNValid == aPValid ? null : $"valid {aNValid}/{aPValid}";
    }

    // The history with each result index replaced by the canonical key of that sub-shape of the result read back,
    // each list sorted.
    private static string Keyed(string theHistory, in Shape theResult)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theResult, aMap);
        CanonicalKeys aKeys = CanonicalKeys.Build(theResult);
        return System.Text.RegularExpressions.Regex.Replace(theHistory, @"([GM])\[([^\]]*)\]", m =>
            m.Groups[1].Value + "[" + string.Join(' ', m.Groups[2].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                                                         .Select(s => (I: int.Parse(s.TrimEnd('r'), CultureInfo.InvariantCulture), R: s.EndsWith('r')))
                                                         .Select(t => (t.I > 0 ? aKeys.Key(aMap[t.I - 1]) : "out") + (t.R ? "r" : ""))
                                                         .Order(StringComparer.Ordinal)) + "]");
    }

    internal static N.TopoDS_Shape Native(ShapeFixOracleRun theRun, in Shape theShape)
    {
        string aText = theRun.PortText(theShape);
        string aFile = Path.Combine(Path.GetTempPath(), $"offset-{Guid.NewGuid():N}.brep");
        File.WriteAllText(aFile, aText);
        try
        {
            return ShapeFixOracleRun.NativeRead(aFile)!;
        }
        finally
        {
            File.Delete(aFile);
        }
    }

    private static string NativeErrorOf(Exception e) => ShapeFixOracleRun.ErrorOf(e) ?? global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType());

    internal Outcome RunNative(ShapeFixOracleRun theRun, Run theCase, N.TopoDS_Shape theInput)
    {
        using var aFaces = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theInput, N.TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
        // the input's sub-shapes before the run: native writes into its input (new vertices on its edges)
        using var anInMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theInput, anInMap);
        var aJoin = (N.GeomAbs_JoinType)(int)theCase.Join;
        try
        {
            N.TopoDS_Shape aRes;
            string aStatus;
            Func<N.TopoDS_Shape, N.TopTools_ListOfShape> aGen, aMod;
            Func<N.TopoDS_Shape, bool> aDel;
            IDisposable anAlgo;
            switch (theCase.Kind)
            {
                case Kind.MakeOffset:
                {
                    var aMO = new N.BRepOffset_MakeOffset();
                    anAlgo = aMO;
                    if (theCase.ThreeArgs)
                    {
                        aMO.Initialize(theInput, theCase.Offset, theCase.Tol);
                    }
                    else
                    {
                        aMO.Initialize(theInput, theCase.Offset, theCase.Tol, N.BRepOffset_Mode.BRepOffset_Skin, theCase.Inter, false, aJoin,
                                       theCase.Thickening, theCase.RemoveIntEdges);
                    }
                    aMO.MakeOffsetShape();
                    aStatus = $"done={aMO.IsDone()} error={(int)aMO.Error()}";
                    aRes = aMO.Shape();
                    aGen = aMO.Generated;
                    aMod = aMO.Modified;
                    aDel = aMO.IsDeleted;
                    break;
                }
                case Kind.OffsetShape:
                {
                    var aMO = new N.BRepOffsetAPI_MakeOffsetShape();
                    anAlgo = aMO;
                    aMO.PerformByJoin(theInput, theCase.Offset, theCase.Tol, N.BRepOffset_Mode.BRepOffset_Skin, theCase.Inter, false, aJoin,
                                      theCase.RemoveIntEdges);
                    aStatus = $"done={aMO.IsDone()} error={(int)aMO.MakeOffset().Error()}";
                    aRes = aMO.IsDone() ? aMO.Shape() : new N.TopoDS_Shape();
                    aGen = aMO.Generated;
                    aMod = aMO.Modified;
                    aDel = aMO.IsDeleted;
                    break;
                }
                default:
                {
                    var aMT = new N.BRepOffsetAPI_MakeThickSolid();
                    anAlgo = aMT;
                    using var aClosing = new N.TopTools_ListOfShape();
                    foreach (int i in theCase.Closing ?? [])
                    {
                        if (i <= aFaces.Extent())
                        {
                            aClosing.Append(aFaces.FindKey(i));
                        }
                    }
                    aMT.MakeThickSolidByJoin(theInput, aClosing, theCase.Offset, theCase.Tol, N.BRepOffset_Mode.BRepOffset_Skin, theCase.Inter,
                                             false, aJoin, theCase.RemoveIntEdges);
                    aMT.Build();
                    aStatus = $"done={aMT.IsDone()} error={(int)aMT.MakeOffset().Error()}";
                    aRes = aMT.IsDone() ? aMT.Shape() : new N.TopoDS_Shape();
                    aGen = aMT.Generated;
                    aMod = aMT.Modified;
                    aDel = aMT.IsDeleted;
                    break;
                }
            }
            using (anAlgo)
            {
                if (aRes.IsNull())
                {
                    return new Outcome(null, aStatus, aStatus.StartsWith("done=True", StringComparison.Ordinal) ? "" : null, "", null);
                }
                string aText = theRun.NativeText(aRes);
                using var aResMap = new N.TopTools_IndexedMapOfShape();
                N.TopExp.MapShapes(aRes, aResMap);
                var aSb = new StringBuilder();
                for (int i = 1; i <= anInMap.Extent(); i++)
                {
                    using N.TopoDS_Shape aS = anInMap.FindKey(i);
                    if (aS.ShapeType() is not (N.TopAbs_ShapeEnum.TopAbs_VERTEX or N.TopAbs_ShapeEnum.TopAbs_EDGE or N.TopAbs_ShapeEnum.TopAbs_FACE))
                    {
                        continue;
                    }
                    aSb.Append(i).Append(':');
                    foreach ((string aTag, Func<N.TopoDS_Shape, N.TopTools_ListOfShape> aF) in new[] { ("G", aGen), ("M", aMod) })
                    {
                        using N.TopTools_ListOfShape aL = aF(aS);
                        aSb.Append(aTag).Append('[').Append(string.Join(' ', aL.Select(x => IndexOf(aResMap, x)))).Append(']');
                    }
                    aSb.Append(aDel(aS) ? "D" : "").Append('\n');
                }
                return new Outcome(null, aStatus, aText, aSb.ToString(), null);
            }
        }
        catch (N.OcctException e)
        {
            return new Outcome(NativeErrorOf(e), "", null, "", null);
        }
    }

    private static string IndexOf(N.TopTools_IndexedMapOfShape theMap, N.TopoDS_Shape theS)
    {
        int anI = theMap.FindIndex(theS);
        return anI.ToString(CultureInfo.InvariantCulture) + (anI > 0 && theS.Orientation() == N.TopAbs_Orientation.TopAbs_REVERSED ? "r" : "");
    }

    private static string IndexOf(IndexedSet<Shape> theMap, in Shape theS)
    {
        int anI = theMap.IndexOf(theS) + 1;
        return anI.ToString(CultureInfo.InvariantCulture) + (anI > 0 && theS.Orientation == Orientation.Reversed ? "r" : "");
    }

    internal static Outcome RunPort(ShapeFixOracleRun theRun, Run theCase, Shape theInput)
    {
        var aFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theInput, ShapeType.Face, aFaces);
        try
        {
            Shape aRes;
            string aStatus;
            Func<Shape, IReadOnlyList<Shape>> aGen, aMod;
            Func<Shape, bool> aDel;
            switch (theCase.Kind)
            {
                case Kind.MakeOffset:
                {
                    var aMO = new MakeOffset();
                    if (theCase.ThreeArgs)
                    {
                        aMO.Initialize(theInput, theCase.Offset, theCase.Tol);
                    }
                    else
                    {
                        aMO.Initialize(theInput, theCase.Offset, theCase.Tol, global::Kerf.Offsets.OffsetMode.Skin, theCase.Inter, false, theCase.Join,
                                       theCase.Thickening, theCase.RemoveIntEdges);
                    }
                    aMO.MakeOffsetShape();
                    aStatus = $"done={aMO.IsDone} error={(int)aMO.Error()}";
                    aRes = aMO.Shape();
                    aGen = s => aMO.Generated(s);
                    aMod = s => aMO.Modified(s);
                    aDel = s => aMO.IsDeleted(s);
                    break;
                }
                case Kind.OffsetShape:
                {
                    var aMO = new MakeOffsetShape();
                    aMO.PerformByJoin(theInput, theCase.Offset, theCase.Tol, global::Kerf.Offsets.OffsetMode.Skin, theCase.Inter, false, theCase.Join,
                                      theCase.RemoveIntEdges);
                    aStatus = $"done={aMO.IsDone} error={(int)aMO.MakeOffset().Error()}";
                    aRes = aMO.IsDone ? aMO.Shape() : new Shape();
                    aGen = s => aMO.Generated(s);
                    aMod = s => aMO.Modified(s);
                    aDel = s => aMO.IsDeleted(s);
                    break;
                }
                default:
                {
                    var aMT = new MakeThickSolid();
                    var aClosing = new NCollection_List<Shape>();
                    foreach (int i in theCase.Closing ?? [])
                    {
                        if (i <= aFaces.Count)
                        {
                            aClosing.Append(aFaces[i - 1]);
                        }
                    }
                    aMT.MakeThickSolidByJoin(theInput, aClosing, theCase.Offset, theCase.Tol, global::Kerf.Offsets.OffsetMode.Skin, theCase.Inter, false,
                                             theCase.Join, theCase.RemoveIntEdges);
                    aMT.Build();
                    aStatus = $"done={aMT.IsDone} error={(int)aMT.MakeOffset().Error()}";
                    aRes = aMT.IsDone ? aMT.Shape() : new Shape();
                    aGen = s => aMT.Generated(s);
                    aMod = s => aMT.Modified(s);
                    aDel = s => aMT.IsDeleted(s);
                    break;
                }
            }
            if (aRes.IsNull)
            {
                return new Outcome(null, aStatus, aStatus.StartsWith("done=True", StringComparison.Ordinal) ? "" : null, "", aRes);
            }
            string aText = theRun.PortText(aRes);
            var aResMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(aRes, aResMap);
            var anInMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(theInput, anInMap);
            var aSb = new StringBuilder();
            for (int i = 1; i <= anInMap.Count; i++)
            {
                Shape aS = anInMap[i - 1];
                if (aS.Type is not (ShapeType.Vertex or ShapeType.Edge or ShapeType.Face))
                {
                    continue;
                }
                aSb.Append(i).Append(':');
                aSb.Append("G[").Append(string.Join(' ', aGen(aS).Select(x => IndexOf(aResMap, x)))).Append(']');
                aSb.Append("M[").Append(string.Join(' ', aMod(aS).Select(x => IndexOf(aResMap, x)))).Append(']');
                aSb.Append(aDel(aS) ? "D" : "").Append('\n');
            }
            return new Outcome(null, aStatus, aText, aSb.ToString(), aRes);
        }
        catch (Exception e) when ((Refusal.IsFailure(e) || e is NullReferenceException))
        {
            return new Outcome(global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType()), "", null, "", null);
        }
    }

    internal void Report(Tally theTally, string theWhat)
    {
        output.WriteLine($"{theWhat}: {theTally.Runs} runs: {theTally.Done} with a result ({theTally.NullResults} null after a reported "
                         + $"success), {theTally.NotDone} not done, {theTally.SameError} the same exception on both sides");
        output.WriteLine($"  results: {theTally.Identical} byte-identical, {theTally.FreeMasked} Free-masked, {theTally.Canonical} canonical only; "
                         + $"native varies in {theTally.NativeVaries}; {theTally.Differences.Count} differences");
        output.WriteLine($"  native varies: the port's result is one of native's variants in {theTally.NativeVariant}, "
                         + $"geometrically native's in {theTally.NativeVariesGeometric} (attributed)");
        output.WriteLine($"  histories: {theTally.HistoriesSame} identical, {theTally.HistoriesCanonical} canonically equal, of "
                         + $"{theTally.Histories}; {theTally.HistoryDifferences.Count} differ");
        output.WriteLine($"  exceptions: {string.Join(", ", theTally.Errors.Select(p => $"{p.Key} {p.Value}"))}");
        output.WriteLine($"  port: inputs changed {theTally.InputsChanged}, results not frozen {theTally.NotFrozen}; "
                         + $"native wrote into its input in {theTally.NativeInputsChanged}");
        foreach (string aDifference in theTally.Differences.Take(80))
        {
            output.WriteLine("  " + aDifference);
        }
        foreach (string aDifference in theTally.HistoryDifferences.Take(40))
        {
            output.WriteLine("  " + aDifference);
        }
        Assert.Empty(theTally.Differences);
        Assert.Empty(theTally.HistoryDifferences);
    }
}
