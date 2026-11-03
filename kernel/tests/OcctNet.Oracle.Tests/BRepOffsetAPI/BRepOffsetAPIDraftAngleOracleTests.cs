// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The draft angle (unit 7G) against native OCCT: BRepOffsetAPI_DraftAngle on plan section 3's cases: primitives and
/// Boolean results like the draft group's inputs (planes, cylinders, cones), each eligible face alone with both signs of
/// the angle and neutral planes at the bottom, in the middle and outside the shape; every side face at once (build123d.NET's
/// <c>Solid.Draft</c>); tapered prisms (build123d.NET's <c>ExtrudeTaper</c>, the neutral plane through the profile); and
/// data/occ's shapes.
/// </summary>
/// <remarks>
/// Both sides read the input from the same BRep text (the port's frozen, as a caller's). Compared: the exception;
/// AddDone, Status and ProblematicShape (as an index of the input's sub-shapes); IsDone; the result as BRep text (bytes,
/// then the Free column masked, then canonically); Generated and Modified of every face of the input as indices of the
/// result's sub-shapes (or canonical keys). The port's input is checked unchanged and its result frozen.
/// </remarks>
public sealed class BRepOffsetAPIDraftAngleOracleTests(ITestOutputHelper output)
{
    internal sealed record Run(string Name, int[] Faces, double[] Angles, Direction3D Dir, Plane3D Plane, bool Flag = true);

    internal sealed class Tally
    {
        public int Runs, Done, NotDone, NotAdded, SameError, Identical, FreeMasked, Canonical, Histories, HistoriesSame,
                   HistoriesCanonical, InputsChanged, NotFrozen, NativeInputsChanged;
        public readonly SortedDictionary<string, int> Errors = new(StringComparer.Ordinal);
        public readonly SortedDictionary<string, int> Statuses = new(StringComparer.Ordinal);
        public readonly List<string> Differences = [];
        public readonly List<string> HistoryDifferences = [];
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Inputs

    private static Shape Box(double x, double y, double z) => new MakeBox(x, y, z).Shape();

    private static Frame3D Ax(double x, double y, double z) => new(new Point3D(x, y, z), new Direction3D(0, 0, 1));

    private static Shape Moved(in Shape theS, double x, double y, double z)
    {
        Transform3D aT = new();
        aT.SetTranslation(new Vector3D(x, y, z));
        return new Transform(theS, aT, true).Shape();
    }

    private static Face Polygon(params (double X, double Y)[] thePoints)
    {
        var aP = new MakePolygon();
        foreach ((double x, double y) in thePoints)
        {
            aP.Add(new Point3D(x, y, 0));
        }
        aP.Close();
        return new MakeFace(aP.Wire()).Face();
    }

    private static Shape Prism(in Face theFace, double h) => new ApiMakePrism(theFace, new Vector3D(0, 0, h)).Shape();

    /// <summary>Solids: primitives and Boolean results like the draft group's.</summary>
    internal static List<(string Name, Shape Shape)> Solids() =>
    [
        ("box 10x20x30", Box(10, 20, 30)),
        ("cube 20 centred", Moved(Box(20, 20, 20), -10, -10, 0)),
        ("cylinder r5 h12", new ApiMakeCylinder(5, 12).Shape()),
        ("cone r6 r2 h10", new ApiMakeCone(6, 2, 10).Shape()),
        ("cone r2 r6 h10", new ApiMakeCone(2, 6, 10).Shape()),
        ("L prism", Prism(Polygon((0, 0), (20, 0), (20, 6), (6, 6), (6, 15), (0, 15)), 8)),
        ("hexagon prism", Prism(Polygon((4, 0), (2, 3.4641016151377544), (-2, 3.4641016151377544), (-4, 0),
                                        (-2, -3.4641016151377544), (2, -3.4641016151377544)), 5)),
        ("box with hole", new Cut(Box(20, 20, 8), new ApiMakeCylinder(Ax(10, 10, -1), 4, 10).Shape()).Shape()),
        ("box with pocket", new Cut(Box(20, 20, 10), Moved(Box(10, 10, 10), 5, 5, 4)).Shape()),
        ("box and cylinder fuse", new Fuse(Box(20, 20, 5), new ApiMakeCylinder(Ax(10, 10, 5), 4, 8).Shape()).Shape()),
        ("stepped fuse", new Fuse(Box(20, 20, 5), Moved(Box(10, 20, 5), 0, 0, 5)).Shape()),
        ("cylinder on box", new Fuse(Moved(Box(20, 20, 10), -10, -10, 0), new ApiMakeCylinder(Ax(0, 0, 10), 5, 10).Shape()).Shape()),
        ("cone on box", new Fuse(Moved(Box(20, 20, 10), -10, -10, 0), new ApiMakeCone(Ax(0, 0, 10), 6, 3, 10).Shape()).Shape()),
        ("half sphere box", new Fuse(new MakeSphere(20, 0, Math.PI / 2).Shape(), Moved(Box(20, 20, 20), -10, -10, 10)).Shape()),
    ];

    /// <summary>build123d.NET's ExtrudeTaper profiles: the prism of each, drafted on its sides about the profile's plane.</summary>
    internal static List<(string Name, Face Profile, double Height)> Profiles()
    {
        Face aCircle = new MakeFace(
            new MakeWire(new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5)).Edge()).Wire()).Face();
        Face aRect = Polygon((-5, -3), (5, -3), (5, 3), (-5, 3));
        Face aRectHole = new Cut(aRect, new MakeFace(
            new MakeWire(new MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.5)).Edge()).Wire()).Face()).Shape() is var aCut
            && new Explorer(aCut, ShapeType.Face) is { } anExp && anExp.More() ? ShapeCasts.Face(anExp.Current()) : aRect;
        return
        [
            ("rectangle", aRect, 4),
            ("circle", aCircle, 6),
            ("L", Polygon((0, 0), (8, 0), (8, 3), (3, 3), (3, 8), (0, 8)), 5),
            ("rectangle with hole", aRectHole, 3),
            ("triangle", Polygon((0, 0), (10, 0), (5, 7)), 2),
        ];
    }

    private static NCollection_IndexedMap<Shape> FacesOf(in Shape theShape)
    {
        var aFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, ShapeType.Face, aFaces);
        return aFaces;
    }

    private static bool IsDraftable(in Shape theFace) => Tool.Surface(ShapeCasts.Face(theFace)) switch
    {
        PlaneSurface or CylindricalSurface or ConicalSurface => true,
        TrimmedSurface t => t.BasisSurface() is PlaneSurface or CylindricalSurface or ConicalSurface,
        _ => false,
    };

    // A side face for the pull direction Z: a plane whose normal is perpendicular to Z, or a cylinder or cone about Z.
    private static bool IsSide(in Shape theFace)
    {
        Surface? aS = Tool.Surface(ShapeCasts.Face(theFace));
        if (aS is TrimmedSurface t)
        {
            aS = t.BasisSurface();
        }
        return aS switch
        {
            PlaneSurface p => Math.Abs(p.Pln().Axis().Direction().Z()) < 1.0e-9,
            ElementarySurface e and (CylindricalSurface or ConicalSurface) => e.Axis().Direction().IsParallel(new Direction3D(0, 0, 1), 1.0e-9),
            _ => false,
        };
    }

    private static (double ZMin, double ZMax) ZRange(in Shape theShape)
    {
        Box aBox = new();
        Bounds.Add(theShape, ref aBox);
        aBox.Get(out _, out _, out double z0, out _, out _, out double z1);
        return (z0, z1);
    }

    private static Plane3D PlaneZ(double z) => new(new Point3D(0, 0, z), new Direction3D(0, 0, 1));

    // ---------------------------------------------------------------------------------------------------------------
    // Tests

    [Fact]
    public void Single_faces_of_primitives_and_Boolean_results_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "draft");
        foreach ((string aName, Shape aShape) in Tape.Budget(Solids()))
        {
            NCollection_IndexedMap<Shape> aFaces = FacesOf(aShape);
            (double z0, double z1) = ZRange(aShape);
            var aPlanes = new[] { ("bottom", z0), ("middle", 0.5 * (z0 + z1)), ("outside", z0 - 0.5 * (z1 - z0)) };
            int aCount = 0;
            for (int f = 1; f <= aFaces.Extent() && aCount < 8; f++)
            {
                if (!IsDraftable(aFaces[f]))
                {
                    continue;
                }
                aCount++;
                foreach (double anAngle in new[] { 5.0, -5.0 })
                {
                    foreach ((string aWhere, double z) in aPlanes)
                    {
                        Compare(aRun, aTally, new Run($"{aName} face {f} {anAngle} {aWhere}", [f], [anAngle], new Direction3D(0, 0, 1), PlaneZ(z)), aShape);
                    }
                }
                // pulled along X about a plane x = 0
                Compare(aRun, aTally, new Run($"{aName} face {f} 3 along X", [f], [3.0], new Direction3D(1, 0, 0),
                                              new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))), aShape);
            }
        }
        Report(aTally, "single faces");
    }

    [Fact]
    public void All_side_faces_and_tapered_prisms_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "draft");
        // build123d.NET's Solid.Draft: every side face at once, one angle, one neutral plane
        foreach ((string aName, Shape aShape) in Tape.Budget(Solids()))
        {
            NCollection_IndexedMap<Shape> aFaces = FacesOf(aShape);
            int[] aSides = Enumerable.Range(1, aFaces.Extent()).Where(i => IsSide(aFaces[i])).ToArray();
            if (aSides.Length == 0)
            {
                continue;
            }
            (double z0, double z1) = ZRange(aShape);
            foreach (double anAngle in new[] { 4.0, -4.0, 10.0 })
            {
                foreach ((string aWhere, double z) in new[] { ("bottom", z0), ("middle", 0.5 * (z0 + z1)) })
                {
                    foreach (bool aFlag in new[] { true, false })
                    {
                        Compare(aRun, aTally, new Run($"{aName} sides {anAngle} {aWhere} flag={aFlag}", aSides,
                                                      aSides.Select(_ => anAngle).ToArray(), new Direction3D(0, 0, 1), PlaneZ(z), aFlag), aShape);
                    }
                }
            }
        }
        // build123d.NET's ExtrudeTaper: the prism's sides drafted about the profile's plane, inner walls the other way
        foreach ((string aName, Face aProfile, double aHeight) in Tape.Budget(Profiles()))
        {
            ApiMakePrism aMaker = new(aProfile, new Vector3D(0, 0, aHeight));
            Shape aPrism = aMaker.Shape();
            NCollection_IndexedMap<Shape> aFaces = FacesOf(aPrism);
            Wire anOuter = BrepTools.OuterWire(aProfile);
            var aSides = new List<int>();
            var anInner = new HashSet<int>();
            for (Explorer anExp = new(aProfile, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                bool isOuter = false;
                for (Explorer anOut = new(anOuter, ShapeType.Edge); anOut.More(); anOut.Next())
                {
                    isOuter |= anOut.Current().IsSame(anExp.Current());
                }
                foreach (Shape aSide in aMaker.Generated(anExp.Current()))
                {
                    int i = aFaces.FindIndex(aSide);
                    if (i > 0 && !aSides.Contains(i))
                    {
                        aSides.Add(i);
                        if (!isOuter)
                        {
                            anInner.Add(i);
                        }
                    }
                }
            }
            foreach (double aTaper in new[] { 5.0, -5.0, 15.0 })
            {
                Compare(aRun, aTally, new Run($"taper {aName} {aTaper}", aSides.ToArray(),
                                              aSides.Select(i => anInner.Contains(i) ? -aTaper : aTaper).ToArray(), new Direction3D(0, 0, 1),
                                              PlaneZ(0.0)), aPrism);
            }
        }
        Report(aTally, "side faces and tapered prisms");
    }

    [Fact]
    public void Data_occ_shapes_as_native()
    {
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "draft");
        string aDir = Path.Combine(ShapeFixOracleRun.Metadata("OcctSourceDir"), "data", "occ");
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
            NCollection_IndexedMap<Shape> aFaces = FacesOf(aShape);
            int[] aSides = Enumerable.Range(1, aFaces.Extent()).Where(i => IsSide(aFaces[i])).Take(6).ToArray();
            (double z0, double z1) = ZRange(aShape);
            foreach (int f in aSides.Take(3))
            {
                Compare(aRun, aTally, new Run($"{aName} face {f}", [f], [3.0], new Direction3D(0, 0, 1), PlaneZ(0.5 * (z0 + z1))), aShape);
            }
            if (aSides.Length > 1)
            {
                Compare(aRun, aTally, new Run($"{aName} sides", aSides, aSides.Select(_ => 2.0).ToArray(), new Direction3D(0, 0, 1), PlaneZ(z0)), aShape);
            }
        }
        Report(aTally, "data/occ shapes up to 300 kB");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // One comparison

    internal sealed record Outcome(string? Error, string Status, string? Text, string History, Shape? PortResult);

    internal void Compare(ShapeFixOracleRun theRun, Tally theTally, Run theCase, Shape theInput)
    {
        string aWhat = theCase.Name;
        if (theRun.Read(BRepOffsetAPIMakeThickSolidOracleTests.Native(theRun, theInput)) is not (N.TopoDS_Shape aNIn, Shape aPIn))
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
        theTally.Statuses[aN.Status] = theTally.Statuses.GetValueOrDefault(aN.Status) + 1;

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
        if (aN.Text is null)
        {
            if (aN.Status.Contains("add=False", StringComparison.Ordinal))
            {
                theTally.NotAdded++;
            }
            else
            {
                theTally.NotDone++;
            }
            return;
        }
        theTally.Done++;
        BRepTextDiff aDiff = BRepTextComparison.Compare(aN.Text, aP.Text!);
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
            Shape aNResP = BooleanOracleTests.ReadPort(aN.Text), aPResP = BooleanOracleTests.ReadPort(aP.Text!);
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNResP, aPResP, new CanonicalOptions { CheckValidity = false });
            if (aCanon.IsIdentical)
            {
                theTally.Canonical++;
            }
            else
            {
                theTally.Differences.Add($"{aWhat}: {aDiff}; canonically {aCanon}");
                return;
            }
        }
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
        string aPKeyed = Keyed(aP.History, BooleanOracleTests.ReadPort(aP.Text!));
        if (aNKeyed == aPKeyed)
        {
            theTally.HistoriesCanonical++;
        }
        else
        {
            theTally.HistoryDifferences.Add($"{aWhat}: histories differ canonically: {DeterminismHarness.FirstDifference(aNKeyed, aPKeyed)}");
        }
    }

    private static string Keyed(string theHistory, in Shape theResult)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theResult, aMap);
        CanonicalKeys aKeys = CanonicalKeys.Build(theResult);
        return System.Text.RegularExpressions.Regex.Replace(theHistory, @"([GM])\[([^\]]*)\]", m =>
            m.Groups[1].Value + "[" + string.Join(' ', m.Groups[2].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                                                         .Select(s => (I: int.Parse(s.TrimEnd('r'), CultureInfo.InvariantCulture), R: s.EndsWith('r')))
                                                         .Select(t => (t.I > 0 ? aKeys.Key(aMap[t.I]) : "out") + (t.R ? "r" : ""))
                                                         .Order(StringComparer.Ordinal)) + "]");
    }

    private static string NativeErrorOf(Exception e) => ShapeFixOracleRun.ErrorOf(e) ?? global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType());

    private static string IndexOf(N.TopTools_IndexedMapOfShape theMap, N.TopoDS_Shape theS)
    {
        if (theS.IsNull())
        {
            return "null";
        }
        int anI = theMap.FindIndex(theS);
        return anI.ToString(CultureInfo.InvariantCulture) + (anI > 0 && theS.Orientation() == N.TopAbs_Orientation.TopAbs_REVERSED ? "r" : "");
    }

    private static string IndexOf(NCollection_IndexedMap<Shape> theMap, in Shape theS)
    {
        if (theS.IsNull)
        {
            return "null";
        }
        int anI = theMap.FindIndex(theS);
        return anI.ToString(CultureInfo.InvariantCulture) + (anI > 0 && theS.Orientation() == Orientation.Reversed ? "r" : "");
    }

    internal Outcome RunNative(ShapeFixOracleRun theRun, Run theCase, N.TopoDS_Shape theInput)
    {
        using var aFaces = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theInput, N.TopAbs_ShapeEnum.TopAbs_FACE, aFaces);
        using var anInMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theInput, anInMap);
        try
        {
            using var aDraft = new N.BRepOffsetAPI_DraftAngle(theInput);
            using var aDir = new N.gp_Dir(theCase.Dir.X(), theCase.Dir.Y(), theCase.Dir.Z());
            Point3D aLoc = theCase.Plane.Location();
            Direction3D aNorm = theCase.Plane.Axis().Direction();
            using var aPLoc = new N.gp_Pnt(aLoc.X(), aLoc.Y(), aLoc.Z());
            using var aPDir = new N.gp_Dir(aNorm.X(), aNorm.Y(), aNorm.Z());
            using var aPln = new N.gp_Pln(aPLoc, aPDir);
            for (int k = 0; k < theCase.Faces.Length; k++)
            {
                using N.TopoDS_Shape aF = aFaces.FindKey(theCase.Faces[k]);
                using N.TopoDS_Face aFace = N.TopoDS.Face(aF);
                aDraft.Add(aFace, aDir, theCase.Angles[k] * Math.PI / 180.0, aPln, theCase.Flag);
                if (!aDraft.AddDone())
                {
                    break;
                }
            }
            if (!aDraft.AddDone())
            {
                using N.TopoDS_Shape aBad = aDraft.ProblematicShape();
                return new Outcome(null, $"add=False status={(int)aDraft.Status()} bad={IndexOf(anInMap, aBad)}", null, "", null);
            }
            aDraft.Build();
            string aStatus = $"add=True done={aDraft.IsDone()} status={(int)aDraft.Status()}";
            if (!aDraft.IsDone())
            {
                using N.TopoDS_Shape aBad = aDraft.ProblematicShape();
                return new Outcome(null, aStatus + $" bad={IndexOf(anInMap, aBad)}", null, "", null);
            }
            using N.TopoDS_Shape aRes = aDraft.Shape();
            string aText = theRun.NativeText(aRes);
            using var aResMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(aRes, aResMap);
            var aSb = new StringBuilder();
            for (int i = 1; i <= anInMap.Extent(); i++)
            {
                using N.TopoDS_Shape aS = anInMap.FindKey(i);
                if (aS.ShapeType() != N.TopAbs_ShapeEnum.TopAbs_FACE)
                {
                    continue;
                }
                aSb.Append(i).Append(':');
                using (N.TopTools_ListOfShape aL = aDraft.Generated(aS))
                {
                    aSb.Append("G[").Append(string.Join(' ', aL.Select(x => IndexOf(aResMap, x)))).Append(']');
                }
                using (N.TopTools_ListOfShape aL = aDraft.Modified(aS))
                {
                    aSb.Append("M[").Append(string.Join(' ', aL.Select(x => IndexOf(aResMap, x)))).Append(']');
                }
                aSb.Append('\n');
            }
            return new Outcome(null, aStatus, aText, aSb.ToString(), null);
        }
        catch (N.OcctException e)
        {
            return new Outcome(NativeErrorOf(e), "", null, "", null);
        }
    }

    internal static Outcome RunPort(ShapeFixOracleRun theRun, Run theCase, Shape theInput)
    {
        NCollection_IndexedMap<Shape> aFaces = FacesOf(theInput);
        var anInMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theInput, anInMap);
        try
        {
            var aDraft = new DraftAngle(theInput);
            for (int k = 0; k < theCase.Faces.Length; k++)
            {
                aDraft.Add(ShapeCasts.Face(aFaces[theCase.Faces[k]]), theCase.Dir, theCase.Angles[k] * Math.PI / 180.0, theCase.Plane, theCase.Flag);
                if (!aDraft.AddDone())
                {
                    break;
                }
            }
            if (!aDraft.AddDone())
            {
                return new Outcome(null, $"add=False status={(int)aDraft.Status()} bad={IndexOf(anInMap, aDraft.ProblematicShape())}", null, "", null);
            }
            aDraft.Build();
            string aStatus = $"add=True done={aDraft.IsDone} status={(int)aDraft.Status()}";
            if (!aDraft.IsDone)
            {
                return new Outcome(null, aStatus + $" bad={IndexOf(anInMap, aDraft.ProblematicShape())}", null, "", null);
            }
            Shape aRes = aDraft.Shape();
            string aText = theRun.PortText(aRes);
            var aResMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(aRes, aResMap);
            var aSb = new StringBuilder();
            for (int i = 1; i <= anInMap.Extent(); i++)
            {
                Shape aS = anInMap[i];
                if (aS.ShapeType() != ShapeType.Face)
                {
                    continue;
                }
                aSb.Append(i).Append(':');
                aSb.Append("G[").Append(string.Join(' ', aDraft.Generated(aS).Select(x => IndexOf(aResMap, x)))).Append(']');
                aSb.Append("M[").Append(string.Join(' ', aDraft.Modified(aS).Select(x => IndexOf(aResMap, x)))).Append(']');
                aSb.Append('\n');
            }
            return new Outcome(null, aStatus, aText, aSb.ToString(), aRes);
        }
        catch (Exception e) when (e is FailureException or NullReferenceException)
        {
            return new Outcome(global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()), "", null, "", null);
        }
    }

    internal void Report(Tally theTally, string theWhat)
    {
        output.WriteLine($"{theWhat}: {theTally.Runs} runs: {theTally.Done} with a result, {theTally.NotAdded} refused by Add, "
                         + $"{theTally.NotDone} not built, {theTally.SameError} the same exception on both sides");
        output.WriteLine($"  results: {theTally.Identical} byte-identical, {theTally.FreeMasked} Free-masked, {theTally.Canonical} canonical only; "
                         + $"{theTally.Differences.Count} differences");
        output.WriteLine($"  histories: {theTally.HistoriesSame} identical, {theTally.HistoriesCanonical} canonically equal, of "
                         + $"{theTally.Histories}; {theTally.HistoryDifferences.Count} differ");
        output.WriteLine($"  statuses: {string.Join(", ", theTally.Statuses.Select(p => $"[{p.Key}] {p.Value}"))}");
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
