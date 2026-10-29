#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Reflection;
using System.Text;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopTools_FormatVersion;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepCheck_Analyzer against native OCCT. Native OCCT exposes only <c>IsValid()</c> and <c>IsValid(sub-shape)</c>, so
/// the comparison is verdict by verdict: every distinct sub-shape of the shape (TopExp::MapShapes, in the same order on
/// both sides), with and without the geometric controls. Both sides analyse the same file: native OCCT writes the shape
/// (current format, with triangulations), then native OCCT and the port each read that file. The port's parallel mode
/// must give its serial mode's statuses exactly (every status of every sub-shape, on its own and in every context);
/// native OCCT's parallel verdicts must equal its serial ones.
/// </summary>
public class BRepCheckAnalyzerOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepcheck-").FullName;
    private int myFileNumber;

    // per category: sub-shape verdicts compared, of which agreeing; whole-shape verdicts; invalid shapes (native)
    private readonly SortedDictionary<string, (int Compared, int Agreed, int Shapes, int Invalid)> myStats =
        new(StringComparer.Ordinal);

    private readonly List<string> myDisagreements = [];

    // Native parallel runs that differ from native serial: OCCT's parallel mode isn't deterministic (it shares each
    // edge's reference adaptor between threads, occt-issues.md); under load, 1 run in 40 of
    // bug29502_split_faces.brep reports a valid face invalid. Reported, not failed: the port is compared with native
    // serial, and its own parallel run with its serial run.
    private readonly List<string> myNativeParallelNotes = [];

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Files

    private string NativeWrite(N.TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(theShape, aFile, true, false, (N.TopTools_FormatVersion)(int)TopTools_FormatVersion_CURRENT));
        return aFile;
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    private static TopoDS_Shape PortRead(string theFile)
    {
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, theFile), theFile);
        return aShape;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Verdicts

    /// <summary>Native IsValid of every sub-shape (index 0: the shape), or the exception's type name.</summary>
    private static (bool[]? Verdicts, string? Error) NativeVerdicts(N.TopoDS_Shape theShape, bool theGeom, bool theParallel)
    {
        try
        {
            using var anAnalyzer = new N.BRepCheck_Analyzer(theShape, theGeom, theParallel);
            using var aMap = new N.TopTools_IndexedMapOfShape();
            N.TopExp.MapShapes(theShape, aMap);
            var aVerdicts = new bool[aMap.Extent()];
            for (int i = 1; i <= aMap.Extent(); i++)
            {
                using N.TopoDS_Shape aSub = aMap.FindKey(i);
                aVerdicts[i - 1] = anAnalyzer.IsValid(aSub);
            }

            return (aVerdicts, null);
        }
        catch (Exception anException)
        {
            return (null, anException.GetType().Name);
        }
    }

    private static NCollection_IndexedMap<TopoDS_Shape> PortMap(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        return aMap;
    }

    /// <summary>
    /// The port's IsValid of every sub-shape, and a dump of every status (on the sub-shape and in each context, the
    /// contexts by their index in the map), or the exception's type name.
    /// </summary>
    private static (bool[]? Verdicts, string Dump, string? Error) PortVerdicts(in TopoDS_Shape theShape, bool theGeom, bool theParallel)
    {
        try
        {
            var anAnalyzer = new BRepCheck_Analyzer(theShape, theGeom, theParallel);
            NCollection_IndexedMap<TopoDS_Shape> aMap = PortMap(theShape);
            var aVerdicts = new bool[aMap.Extent()];
            for (int i = 1; i <= aMap.Extent(); i++)
            {
                aVerdicts[i - 1] = anAnalyzer.IsValid(aMap.FindKey(i));
            }

            return (aVerdicts, Dump(anAnalyzer, aMap, false), null);
        }
        catch (Exception anException)
        {
            return (null, "", anException.GetType().Name + ": " + anException.Message + "\n" + anException.StackTrace);
        }
    }

    private static string Statuses(NCollection_List<BRepCheck_Status> theList)
        => string.Join(",", theList.Select(s => s.ToString().Replace("BRepCheck_", "", StringComparison.Ordinal)));

    /// <summary>Every status of every result; with <paramref name="theFaultsOnly"/>, only of the results with a fault.</summary>
    private static string Dump(BRepCheck_Analyzer theAnalyzer, NCollection_IndexedMap<TopoDS_Shape> theMap, bool theFaultsOnly)
    {
        var aText = new StringBuilder();
        for (int i = 1; i <= theMap.Extent(); i++)
        {
            TopoDS_Shape aSub = theMap.FindKey(i);
            BRepCheck_Result? aResult = theAnalyzer.Result(aSub);
            if (aResult is null)
            {
                continue;
            }

            var aContexts = new List<(int, string)>();
            for (aResult.InitContextIterator(); aResult.MoreShapeInContext(); aResult.NextShapeInContext())
            {
                aContexts.Add((theMap.FindIndex(aResult.ContextualShape()), Statuses(aResult.StatusOnShape())));
            }

            aContexts.Sort();
            string anOwn = Statuses(aResult.Status());
            string aLine = $"{i} {aSub.ShapeType()}: {anOwn}; " + string.Join("; ", aContexts.Select(c => $"in {c.Item1}: {c.Item2}"));
            bool isFaulty = anOwn != "NoError" || aContexts.Any(c => c.Item2 != "NoError");
            if (!theFaultsOnly || isFaulty)
            {
                aText.Append(aLine).Append('\n');
            }
        }

        return aText.ToString();
    }

    private static string FaultDump(in TopoDS_Shape theShape, bool theGeom)
    {
        try
        {
            return Dump(new BRepCheck_Analyzer(theShape, theGeom), PortMap(theShape), true);
        }
        catch (Exception anException)
        {
            return anException.GetType().Name;
        }
    }

    /// <summary>
    /// Compares the analyzers on the shape native OCCT writes: native and port serial verdicts per sub-shape (geometric
    /// controls on and off), native parallel against native serial, and the port's parallel statuses against its
    /// serial ones. Returns the native whole-shape verdict (null if native OCCT threw).
    /// </summary>
    private bool? Check(string theCategory, N.TopoDS_Shape theNative, string theWhat, bool theParallel = true)
        => CheckFile(theCategory, NativeWrite(theNative), theWhat, theParallel, true);

    private bool? CheckFile(string theCategory, string theFile, string theWhat, bool theParallel, bool theSubShapes)
    {
        using N.TopoDS_Shape aNative = NativeRead(theFile);
        TopoDS_Shape aPort = PortRead(theFile);
        Assert.Equal(PortMap(aPort).Extent(), CountNative(aNative));
        bool? aWhole = null;
        foreach (bool aGeom in (bool[])[true, false])
        {
            (bool[]? aN, string? aNError) = NativeVerdicts(aNative, aGeom, false);
            (bool[]? aP, string aPDump, string? aPError) = PortVerdicts(aPort, aGeom, false);
            (int Compared, int Agreed, int Shapes, int Invalid) aStat = myStats.GetValueOrDefault(theCategory);
            aStat.Shapes++;
            if (aN is null || aP is null)
            {
                if ((aN is null) != (aP is null))
                {
                    myDisagreements.Add($"{theCategory} {theWhat} geom={aGeom}: native {aNError ?? "ok"}, port {aPError ?? "ok"}");
                }

                aStat.Compared++;
                aStat.Agreed += (aN is null) == (aP is null) ? 1 : 0;
                myStats[theCategory] = aStat;
                continue;
            }

            if (aGeom)
            {
                aWhole = aN[0];
            }

            aStat.Invalid += aN[0] ? 0 : 1;
            int aNb = theSubShapes ? aN.Length : 1;
            var aBad = new List<int>();
            for (int i = 0; i < aNb; i++)
            {
                aStat.Compared++;
                if (aN[i] == aP[i])
                {
                    aStat.Agreed++;
                }
                else
                {
                    aBad.Add(i + 1);
                }
            }

            myStats[theCategory] = aStat;
            if (aBad.Count > 0)
            {
                myDisagreements.Add($"{theCategory} {theWhat} geom={aGeom}: sub-shapes {string.Join(",", aBad.Take(10))} "
                                    + $"(native whole {aN[0]}, port whole {aP[0]})\n{FaultDump(aPort, aGeom)}");
            }

            if (theParallel)
            {
                (bool[]? aNPar, _) = NativeVerdicts(aNative, aGeom, true);
                if (aNPar is null || !aNPar.SequenceEqual(aN))
                {
                    myNativeParallelNotes.Add($"{theCategory} {theWhat} geom={aGeom}: native parallel differs from native serial");
                }

                (bool[]? aPPar, string aPParDump, _) = PortVerdicts(aPort, aGeom, true);
                if (aPPar is null || aPParDump != aPDump)
                {
                    myDisagreements.Add($"{theCategory} {theWhat} geom={aGeom}: port parallel differs from port serial");
                }
            }
        }

        return aWhole;
    }

    private static int CountNative(N.TopoDS_Shape theShape)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, aMap);
        return aMap.Extent();
    }

    private void Report(string theWhat)
    {
        foreach ((string aCategory, (int aCompared, int anAgreed, int aShapes, int anInvalid)) in myStats)
        {
            output.WriteLine($"{aCategory}: {aShapes / 2} shapes ({anInvalid} analyses invalid natively), "
                             + $"{anAgreed}/{aCompared} verdicts agree");
        }

        int aTotal = myStats.Values.Sum(s => s.Compared), anAgreedTotal = myStats.Values.Sum(s => s.Agreed);
        output.WriteLine($"{theWhat}: {anAgreedTotal}/{aTotal} verdicts agree, {myDisagreements.Count} disagreements");
        foreach (string aNote in myNativeParallelNotes)
        {
            output.WriteLine($"note (OCCT's parallel mode): {aNote}");
        }

        foreach (string aDisagreement in myDisagreements)
        {
            output.WriteLine(aDisagreement);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Native shapes and breaks

    private static N.TopoDS_Shape[] Subs(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, theType, aMap);
        return aMap.ToArray();
    }

    private static N.TopoDS_Shape[] SubsOf(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType)
    {
        var aList = new List<N.TopoDS_Shape>();
        for (var anExp = new N.TopExp_Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }

        return [.. aList];
    }

    private static N.TopoDS_Shape Pick(Oracle theOracle, N.TopoDS_Shape[] theShapes) => theShapes[theOracle.Int(0, theShapes.Length - 1)];

    /// <summary>A random edge of a random face that isn't a seam or degenerated, with that face; null if none.</summary>
    private static (N.TopoDS_Edge, N.TopoDS_Face)? FaceEdge(Oracle theOracle, N.TopoDS_Shape theShape)
    {
        N.TopoDS_Shape[] aFaces = Subs(theShape, N.TopAbs_ShapeEnum.TopAbs_FACE);
        for (int aTry = 0; aTry < 20 && aFaces.Length > 0; aTry++)
        {
            N.TopoDS_Face aF = N.TopoDS.Face(Pick(theOracle, aFaces));
            N.TopoDS_Shape[] anEdges = Subs(aF, N.TopAbs_ShapeEnum.TopAbs_EDGE);
            if (anEdges.Length == 0)
            {
                continue;
            }

            N.TopoDS_Edge anE = N.TopoDS.Edge(Pick(theOracle, anEdges));
            if (!N.BRep_Tool.Degenerated(anE) && !N.BRep_Tool.IsClosed(anE, aF))
            {
                return (anE, aF);
            }
        }

        return null;
    }

    private static N.TopoDS_Edge? RegularEdge(Oracle theOracle, N.TopoDS_Shape theShape)
    {
        N.TopoDS_Shape[] anEdges = Subs(theShape, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        for (int aTry = 0; aTry < 20 && anEdges.Length > 0; aTry++)
        {
            N.TopoDS_Edge anE = N.TopoDS.Edge(Pick(theOracle, anEdges));
            if (!N.BRep_Tool.Degenerated(anE))
            {
                return anE;
            }
        }

        return null;
    }

    private static readonly string[] theBreaks =
    [
        "vertex moved", "3D curve shifted", "pcurve shifted", "pcurve removed", "SameParameter cleared", "SameRange cleared",
        "Degenerated set", "range reversed", "edge tolerance raised",
    ];

    /// <summary>Breaks the shape in place with native BRep_Builder; false if the break doesn't apply.</summary>
    private static bool Break(Oracle theOracle, N.TopoDS_Shape theShape, string theBreak)
    {
        using var aB = new N.BRep_Builder();
        switch (theBreak)
        {
            case "vertex moved":
            {
                N.TopoDS_Shape[] aVertices = Subs(theShape, N.TopAbs_ShapeEnum.TopAbs_VERTEX);
                if (aVertices.Length == 0)
                {
                    return false;
                }

                N.TopoDS_Vertex aV = N.TopoDS.Vertex(Pick(theOracle, aVertices));
                using N.gp_Pnt aP = N.BRep_Tool.Pnt(aV);
                double d = theOracle.Real(0.01, 0.5);
                using var aQ = new N.gp_Pnt(aP.X() + d, aP.Y() - d, aP.Z() + d);
                aB.UpdateVertex(aV, aQ, N.BRep_Tool.Tolerance(aV));
                return true;
            }
            case "3D curve shifted":
            {
                N.TopoDS_Edge? anE = RegularEdge(theOracle, theShape);
                if (anE is null)
                {
                    return false;
                }

                double f = 0, l = 0;
                N.Geom_Curve? aC = N.BRep_Tool.Curve(anE, ref f, ref l);
                if (aC is null)
                {
                    return false;
                }

                double d = theOracle.Real(0.001, 0.2);
                using var aV = new N.gp_Vec(d, d, -d);
                aB.UpdateEdge(anE, N.Geom_Curve.DownCast(aC.Translated(aV)), N.BRep_Tool.Tolerance(anE));
                return true;
            }
            case "pcurve shifted":
            case "pcurve removed":
            {
                if (FaceEdge(theOracle, theShape) is not var (anE, aF))
                {
                    return false;
                }

                double f = 0, l = 0;
                N.Geom2d_Curve? aPC = N.BRep_Tool.CurveOnSurface(anE, aF, ref f, ref l);
                if (aPC is null)
                {
                    return false;
                }

                if (theBreak == "pcurve shifted")
                {
                    double d = theOracle.Real(0.001, 0.1);
                    using var aV = new N.gp_Vec2d(d, -d);
                    aB.UpdateEdge(anE, N.Geom2d_Curve.DownCast(aPC.Translated(aV)), aF, N.BRep_Tool.Tolerance(anE));
                }
                else
                {
                    aB.UpdateEdge(anE, (N.Geom2d_Curve?)null, aF, N.BRep_Tool.Tolerance(anE));
                }

                return true;
            }
            case "SameParameter cleared":
            case "SameRange cleared":
            case "Degenerated set":
            case "range reversed":
            case "edge tolerance raised":
            {
                N.TopoDS_Edge? anE = RegularEdge(theOracle, theShape);
                if (anE is null)
                {
                    return false;
                }

                switch (theBreak)
                {
                    case "SameParameter cleared":
                        aB.SameParameter(anE, false);
                        break;
                    case "SameRange cleared":
                        aB.SameRange(anE, false);
                        break;
                    case "Degenerated set":
                        aB.Degenerated(anE, true);
                        break;
                    case "range reversed":
                    {
                        double f = 0, l = 0;
                        N.BRep_Tool.Range(anE, ref f, ref l);
                        aB.Range(anE, l, f);
                        break;
                    }
                    default:
                        // a larger tolerance: still valid, the vertices' tolerances now smaller than the edge's
                        aB.UpdateEdge(anE, theOracle.Real(1e-4, 1e-2));
                        break;
                }

                return true;
            }
            default:
                throw new ArgumentException(theBreak);
        }
    }

    private static N.TopoDS_Wire PolygonWire(bool theClose, double theZ, params (double X, double Y)[] thePoints)
    {
        using var aPolygon = new N.BRepBuilderAPI_MakePolygon();
        foreach ((double x, double y) in thePoints)
        {
            using var aP = new N.gp_Pnt(x, y, theZ);
            aPolygon.Add(aP);
        }

        if (theClose)
        {
            aPolygon.Close();
        }

        return aPolygon.Wire();
    }

    private static N.gp_Pln Plane(double theZ)
    {
        using var aP = new N.gp_Pnt(0, 0, theZ);
        using var aD = new N.gp_Dir(0, 0, 1);
        return new N.gp_Pln(aP, aD);
    }

    private static (double X, double Y)[] Square(double x, double y, double a, bool theCcw)
        => theCcw ? [(x, y), (x + a, y), (x + a, y + a), (x, y + a)] : [(x, y), (x, y + a), (x + a, y + a), (x + a, y)];

    private static N.TopoDS_Face AddWire(N.TopoDS_Face theFace, N.TopoDS_Wire theWire)
    {
        using var aMaker = new N.BRepBuilderAPI_MakeFace(theFace, theWire);
        return aMaker.Face();
    }

    private static readonly string[] theFaceBreaks =
    [
        "valid square", "valid hole", "open wire", "hole same orientation", "figure-8", "intersecting wires",
        "two outer wires", "hole outside", "nested holes",
    ];

    /// <summary>A planar face of z = theZ built with native makers, valid or with the given defect.</summary>
    private static N.TopoDS_Shape FaceScene(Oracle theOracle, string theKind)
    {
        double z = theOracle.Real(-2, 2), x = theOracle.Real(-3, 3), y = theOracle.Real(-3, 3), a = theOracle.Real(2, 6);
        using N.gp_Pln aPln = Plane(z);
        N.TopoDS_Face Outer()
        {
            using var aMaker = new N.BRepBuilderAPI_MakeFace(aPln, PolygonWire(true, z, Square(x, y, a, true)), true);
            return aMaker.Face();
        }

        switch (theKind)
        {
            case "valid square":
                return Outer();
            case "valid hole":
                return AddWire(Outer(), PolygonWire(true, z, Square(x + a / 4, y + a / 4, a / 2, false)));
            case "open wire":
            {
                using var aMaker = new N.BRepBuilderAPI_MakeFace(aPln, PolygonWire(false, z, Square(x, y, a, true)), true);
                return aMaker.Face();
            }
            case "hole same orientation":
                return AddWire(Outer(), PolygonWire(true, z, Square(x + a / 4, y + a / 4, a / 2, true)));
            case "figure-8":
            {
                using var aMaker = new N.BRepBuilderAPI_MakeFace(aPln, PolygonWire(true, z, (x, y), (x + a, y + a), (x + a, y), (x, y + a)), true);
                return aMaker.Face();
            }
            case "intersecting wires":
                return AddWire(Outer(), PolygonWire(true, z, Square(x + a / 2, y + a / 2, a, false)));
            case "two outer wires":
                return AddWire(Outer(), PolygonWire(true, z, Square(x + 2 * a, y, a / 2, true)));
            case "hole outside":
                return AddWire(Outer(), PolygonWire(true, z, Square(x + 2 * a, y, a / 2, false)));
            default: // nested holes
                return AddWire(AddWire(Outer(), PolygonWire(true, z, Square(x + a / 8, y + a / 8, 3 * a / 4, false))),
                               PolygonWire(true, z, Square(x + a / 4, y + a / 4, a / 2, false)));
        }
    }

    private static N.TopoDS_Shape Box(Oracle theOracle, double theX, double theY, double theZ, double theSize)
    {
        using var aP = new N.gp_Pnt(theX, theY, theZ);
        using var aMaker = new N.BRepPrimAPI_MakeBox(aP, theSize * theOracle.Real(0.5, 1), theSize * theOracle.Real(0.5, 1), theSize);
        return aMaker.Shape();
    }

    private static N.TopoDS_Shell FirstShell(N.TopoDS_Shape theSolid) => N.TopoDS.Shell(Subs(theSolid, N.TopAbs_ShapeEnum.TopAbs_SHELL)[0]);

    private static readonly string[] theSolidBreaks =
    [
        "valid solid", "valid void", "face reversed", "face missing", "face repeated", "shell reversed", "two disjoint shells",
        "nested shells", "vertex in solid", "internal vertex in solid", "edge in solid", "only internal shell",
    ];

    /// <summary>A solid assembled with native BRep_Builder from boxes, valid or with the given defect.</summary>
    private static N.TopoDS_Shape SolidScene(Oracle theOracle, string theKind)
    {
        using var aB = new N.BRep_Builder();
        double x = theOracle.Real(-3, 3), y = theOracle.Real(-3, 3), z = theOracle.Real(-3, 3), a = theOracle.Real(2, 6);
        N.TopoDS_Shape aBig = Box(theOracle, x, y, z, a);
        N.TopoDS_Shell aShell = FirstShell(aBig);
        var aSolid = new N.TopoDS_Solid();
        aB.MakeSolid(aSolid);
        switch (theKind)
        {
            case "valid solid":
                aB.Add(aSolid, aShell);
                break;
            case "valid void":
            case "nested shells":
            {
                aB.Add(aSolid, aShell);
                N.TopoDS_Shell anInner = FirstShell(Box(theOracle, x + a / 4, y + a / 8, z + a / 4, a / 4));
                aB.Add(aSolid, theKind == "valid void" ? anInner.Reversed() : anInner);
                break;
            }
            case "two disjoint shells":
                aB.Add(aSolid, aShell);
                aB.Add(aSolid, FirstShell(Box(theOracle, x + 2 * a, y, z, a / 2)));
                break;
            case "shell reversed":
                aB.Add(aSolid, aShell.Reversed());
                break;
            case "vertex in solid":
            case "internal vertex in solid":
            case "edge in solid":
            {
                aB.Add(aSolid, aShell);
                N.TopoDS_Shape aSub = theKind == "edge in solid"
                    ? Subs(Box(theOracle, x, y, z, a / 2), N.TopAbs_ShapeEnum.TopAbs_EDGE)[0]
                    : Subs(Box(theOracle, x, y, z, a / 2), N.TopAbs_ShapeEnum.TopAbs_VERTEX)[0];
                aB.Add(aSolid, theKind == "internal vertex in solid" ? aSub.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL) : aSub);
                break;
            }
            case "only internal shell":
            {
                var aNewShell = new N.TopoDS_Shell();
                aB.MakeShell(aNewShell);
                foreach (N.TopoDS_Shape aF in SubsOf(aShell, N.TopAbs_ShapeEnum.TopAbs_FACE))
                {
                    aB.Add(aNewShell, aF.Oriented(N.TopAbs_Orientation.TopAbs_INTERNAL));
                }

                aB.Add(aSolid, aNewShell);
                break;
            }
            default:
            {
                N.TopoDS_Shape[] aFaces = SubsOf(aShell, N.TopAbs_ShapeEnum.TopAbs_FACE);
                int k = theOracle.Int(0, aFaces.Length - 1);
                var aNewShell = new N.TopoDS_Shell();
                aB.MakeShell(aNewShell);
                for (int i = 0; i < aFaces.Length; i++)
                {
                    if (i == k && theKind == "face missing")
                    {
                        continue;
                    }

                    aB.Add(aNewShell, i == k && theKind == "face reversed" ? aFaces[i].Reversed() : aFaces[i]);
                }

                if (theKind == "face repeated")
                {
                    aB.Add(aNewShell, aFaces[k]);
                }

                aB.Add(aSolid, aNewShell);
                break;
            }
        }

        return aSolid;
    }

    private static N.TopoDS_Shape Placed(Oracle theOracle, N.TopoDS_Shape theShape)
    {
        if (theOracle.Bool())
        {
            return theShape;
        }

        (N.TopLoc_Location aNL, _) = TopologyOracleTests.RandomLocation(theOracle);
        return theShape.Moved(aNL);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Tests

    [Fact]
    public void Native_shapes_are_checked_as_native_OCCT_checks_them()
    {
        var anOracle = new Oracle(output, 33101);
        int aValid = 0;
        for (int i = 0; i < 120; i++)
        {
            N.TopoDS_Shape aShape = BRepToolsIOOracleTests.NativeShape(anOracle, i, out string aWhat);
            if (Check("valid: " + aWhat.Split(',')[0], aShape, $"#{i} {aWhat}") == true)
            {
                aValid++;
            }
        }

        output.WriteLine($"120 shapes, {aValid} valid natively");
        Report("native shapes");
        Assert.Empty(myDisagreements);
    }

    [Fact]
    public void Broken_edges_and_vertices_are_found_as_native_OCCT_finds_them()
    {
        var anOracle = new Oracle(output, 33102);
        for (int i = 0; i < 36 * theBreaks.Length; i++)
        {
            string aBreak = theBreaks[i % theBreaks.Length];
            N.TopoDS_Shape aShape = BRepToolsIOOracleTests.NativeShape(anOracle, i / theBreaks.Length, out string aWhat);
            // a private, unlocked copy to break
            using N.TopoDS_Shape aCopy = NativeRead(NativeWrite(aShape));
            if (!Break(anOracle, aCopy, aBreak))
            {
                continue;
            }

            Check(aBreak, aCopy, $"#{i} {aWhat}");
        }

        Report("broken edges and vertices");
        Assert.Empty(myDisagreements);
    }

    [Fact]
    public void Broken_faces_and_solids_are_found_as_native_OCCT_finds_them()
    {
        var anOracle = new Oracle(output, 33103);
        for (int i = 0; i < 16 * theFaceBreaks.Length; i++)
        {
            string aKind = theFaceBreaks[i % theFaceBreaks.Length];
            Check(aKind, Placed(anOracle, FaceScene(anOracle, aKind)), $"#{i}");
        }

        for (int i = 0; i < 16 * theSolidBreaks.Length; i++)
        {
            string aKind = theSolidBreaks[i % theSolidBreaks.Length];
            N.TopoDS_Shape aShape = Placed(anOracle, SolidScene(anOracle, aKind));
            Check(aKind, anOracle.Bool() ? aShape : aShape.Reversed(), $"#{i}");
        }

        Report("broken faces and solids");
        Assert.Empty(myDisagreements);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The corpus

    private static string Metadata(string theKey)
        => typeof(BRepCheckAnalyzerOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    /// <summary>Native OCCT reads the file and writes F1 (the current format); both analyse F1. Null if unreadable.</summary>
    private string? NativeF1(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        if (!N.BRepTools.Read(aShape, theFile, aBuilder) || aShape.IsNull())
        {
            return null;
        }

        string aF1 = NativeWrite(aShape);
        aShape.Dispose();
        return aF1;
    }

    [Fact]
    public void The_brep_files_of_data_occ_are_checked_as_native_OCCT_checks_them()
    {
        string[] aFiles = Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep")
                                   .Order(StringComparer.Ordinal).ToArray();
        Assert.Equal(37, aFiles.Length);
        var aWatch = Stopwatch.StartNew();
        int aValid = 0;
        foreach (string aFile in aFiles)
        {
            string aF1 = NativeF1(aFile)!;
            if (CheckFile("data/occ", aF1, Path.GetFileName(aFile), true, true) == true)
            {
                aValid++;
            }

            File.Delete(aF1);
        }

        output.WriteLine($"data/occ: {aFiles.Length} files, {aValid} valid natively, {aWatch.Elapsed.TotalSeconds:F0} s");
        Report("data/occ");
        Assert.Empty(myDisagreements);
    }

    private const int MaxOccurrences = 200_000;

    /// <summary>The number of sub-shape occurrences (TopoDS_Iterator, nested), counted up to just past the limit.</summary>
    private static int Occurrences(in TopoDS_Shape theShape, int theLimit)
    {
        int aCount = 0;
        var aStack = new Stack<TopoDS_Shape>();
        aStack.Push(theShape);
        while (aStack.Count > 0 && aCount <= theLimit)
        {
            TopoDS_Shape aShape = aStack.Pop();
            aCount++;
            for (var anIt = new TopoDS_Iterator(aShape); anIt.More(); anIt.Next())
            {
                aStack.Push(anIt.Value());
            }
        }

        return aCount;
    }

    /// <summary>
    /// OCCT's test dataset: every .brep and .rle file native OCCT reads as a shape, checked as the data/occ files are
    /// (every sub-shape's verdict, geometric controls on and off, serial and parallel). Files of more than 2 MB, and assemblies of more than
    /// <see cref="MaxOccurrences"/> sub-shape occurrences (one has 3.2 million: native OCCT takes minutes), are left
    /// out.
    /// </summary>
    [Fact]
    public void The_test_dataset_is_checked_as_native_OCCT_checks_it()
    {
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to check");
            return;
        }

        string[] aFiles = Directory.EnumerateFiles(aRoot, "*.*", SearchOption.AllDirectories)
                                   .Where(f => f.EndsWith(".brep", StringComparison.OrdinalIgnoreCase)
                                               || f.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                                   .Order(StringComparer.Ordinal).ToArray();
        var aWatch = Stopwatch.StartNew();
        int aChecked = 0, anUnreadable = 0, aLarge = 0, aHuge = 0, aValid = 0;
        var aSlowest = new List<(double, string)>();
        foreach (string aFile in aFiles)
        {
            if (new FileInfo(aFile).Length > 2_000_000)
            {
                aLarge++;
                continue;
            }

            string? aF1 = NativeF1(aFile);
            if (aF1 is null)
            {
                anUnreadable++;
                continue;
            }

            if (Occurrences(PortRead(aF1), MaxOccurrences) > MaxOccurrences)
            {
                aHuge++;
                File.Delete(aF1);
                continue;
            }

            var aFileWatch = Stopwatch.StartNew();
            if (CheckFile("dataset", aF1, Path.GetRelativePath(aRoot, aFile), true, true) == true)
            {
                aValid++;
            }

            aSlowest.Add((aFileWatch.Elapsed.TotalSeconds, Path.GetRelativePath(aRoot, aFile)));
            aChecked++;
            File.Delete(aF1);
        }

        output.WriteLine($"dataset: {aFiles.Length} files, {aChecked} checked ({aValid} valid natively), {anUnreadable} not shapes, "
                         + $"{aLarge} over 2 MB and {aHuge} of more than {MaxOccurrences} sub-shape occurrences left out, "
                         + $"{aWatch.Elapsed.TotalSeconds:F0} s; slowest: "
                         + string.Join(", ", aSlowest.OrderByDescending(t => t.Item1).Take(5).Select(t => $"{t.Item2} {t.Item1:F1} s")));
        Report("dataset");
        Assert.Empty(myDisagreements);
    }
}
#endif
