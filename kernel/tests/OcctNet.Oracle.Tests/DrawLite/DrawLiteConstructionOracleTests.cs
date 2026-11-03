// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using OcctNet.DrawLite;
using Xunit.Abstractions;
using static Occt.TopTools_FormatVersion;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Draw-lite's construction, distance and modification commands (unit 3L2b) against native OCCT: distmini's value
/// and solution shapes against BRepExtrema_DistShapeShape; vertex, edge, polyline, wire, mkedge, mkface, mkshell,
/// mkplane and profile, run with random arguments, against the native makers called as the commands' C++ calls them;
/// scalexyz, deform, sameparameter, fsameparameter, updatetolerance, encoderegularity and solidorientation on
/// data/occ's files against the native algorithms. Shapes are compared through BRep text (BRepTools::Write).
/// </summary>
/// <remarks>
/// Arguments are written with "R", which Draw::Atof reads back exactly. Shapes made from frozen inputs keep their
/// Free flag where native OCCT clears it (design 4.4), so texts that differ are compared again with the Free column
/// masked, as BRepBuilderAPIMakersOracleTests does. Both sides read data/occ's files from native OCCT's copy in the
/// current format.
/// </remarks>
public sealed partial class DrawLiteConstructionOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-drawconstruction-").FullName;
    private int myFileNumber;

    public void Dispose() => Directory.Delete(myDirectory, true);

    private static string Metadata(string theKey)
        => typeof(DrawLiteConstructionOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string ResourcesDirectory => Path.Combine(Metadata("OcctSourceDir"), "resources", "DrawResources");

    private static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static string TclPath(string thePath) => thePath.Replace('\\', '/');

    private static string R(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);

    private static DrawInterpreter Interpreter()
    {
        var aDI = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        aDI.LoadDrawResources(ResourcesDirectory);
        return aDI;
    }

    // Eval, naming the script when it fails
    private static string Run(DrawInterpreter theDI, string theScript)
    {
        try
        {
            return theDI.Eval(theScript);
        }
        catch (DrawException anError)
        {
            throw new InvalidOperationException($"{theScript}: {anError.Reason}", anError);
        }
    }

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    [GeneratedRegex("(?m)^[01]([01]{6})$")]
    private static partial Regex FlagLine();

    [GeneratedRegex("(?m)^Ve\r?\n.*$")]
    private static partial Regex VertexTolerance();

    // a vertex TShape reached at two locations or more
    private static bool HasVertexAtSeveralLocations(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_ShapeEnum.TopAbs_VERTEX, aMap);
        var aTShapes = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aTShapes.Add(aMap[i].TShape()!);
        }

        return aTShapes.Count < aMap.Extent();
    }

    // The Free flag is the first of the seven flag digits on their own line.
    private static string MaskFree(string theText) => FlagLine().Replace(theText, "x$1");

    private sealed class Tally(string theWhat)
    {
        public int Shapes;
        public int Identical;
        public int MaskedEqual;
        public int BothFailed;
        public int OrderDependent;
        public int Values;
        public readonly List<string> Differences = [];

        public string Report()
            => $"{theWhat}: {Shapes} shapes ({Identical} byte-identical, {MaskedEqual} equal with the Free column masked), "
               + $"{Values} values bit-identical, {BothFailed} failed on both sides, {OrderDependent} equal but for the vertex tolerances OCCT sets in address order";
    }

    private void Compare(Tally theTally, string theWhat, N.TopoDS_Shape theNative, in TopoDS_Shape thePort)
    {
        string aN = NativeText(theNative);
        string aP = PortText(thePort);
        theTally.Shapes++;
        if (aN == aP)
        {
            theTally.Identical++;
        }
        else if (MaskFree(aN) == MaskFree(aP))
        {
            theTally.MaskedEqual++;
        }
        else if (theTally.Differences.Count < 5)
        {
            theTally.Differences.Add($"{theWhat}:\n--- native\n{aN}\n--- port\n{aP}");
        }
        else
        {
            theTally.Differences.Add(theWhat);
        }
    }

    private static void Same(Tally theTally, double theNative, double thePort, string theWhat)
    {
        theTally.Values++;
        if (BitConverter.DoubleToInt64Bits(theNative) != BitConverter.DoubleToInt64Bits(thePort))
        {
            theTally.Differences.Add(string.Create(CultureInfo.InvariantCulture, $"{theWhat}: native {theNative:R}, port {thePort:R}"));
        }
    }

    private static void Same(Tally theTally, N.gp_Pnt theNative, in gp_Pnt thePort, string theWhat)
    {
        Same(theTally, theNative.X(), thePort.X(), theWhat + ".X");
        Same(theTally, theNative.Y(), thePort.Y(), theWhat + ".Y");
        Same(theTally, theNative.Z(), thePort.Z(), theWhat + ".Z");
    }

    private void Finish(Tally theTally)
    {
        output.WriteLine(theTally.Report());
        foreach (string aD in theTally.Differences.Take(10))
        {
            output.WriteLine(aD);
        }

        Assert.Empty(theTally.Differences);
    }

    /// <summary>Native OCCT reads the file and writes it in the current format, which both sides then read.</summary>
    private (string File, N.TopoDS_Shape Shape) NativeRead(string theFile)
    {
        using var aBuilder = new N.BRep_Builder();
        var aShape = new N.TopoDS_Shape();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        string aF1 = NewFile();
        Assert.True(N.BRepTools.Write(aShape, aF1, true, false, (N.TopTools_FormatVersion)(int)TopTools_FormatVersion_CURRENT));
        var aF1Shape = new N.TopoDS_Shape();
        Assert.True(N.BRepTools.Read(aF1Shape, aF1, aBuilder), aF1);
        return (aF1, aF1Shape);
    }

    // data/occ's files small enough for many distance computations
    private static List<string> SmallFiles()
        => Directory.GetFiles(DataOcc, "*.brep").Where(f => new FileInfo(f).Length < 60_000).Order(StringComparer.Ordinal).ToList();

    // -----------------------------------------------------------------------------------------------------------------
    // distmini

    private void DistMini(Tally theTally, DrawInterpreter theDI, N.TopoDS_Shape theN1, N.TopoDS_Shape theN2, string theArgs, double theDeflection,
                          bool theParallel, string theWhat)
    {
        string aResult;
        try
        {
            aResult = theDI.Eval($"distmini d a b {theArgs}");
        }
        catch (DrawException anError)
        {
            theTally.Differences.Add($"{theWhat}: port error {anError.Reason}");
            return;
        }

        using var aDist = new N.BRepExtrema_DistShapeShape();
        aDist.LoadS1(theN1);
        aDist.LoadS2(theN2);
        aDist.SetDeflection(theDeflection);
        aDist.SetMultiThread(theParallel);
        aDist.Perform();
        if (!aDist.IsDone())
        {
            if (aResult != "problem: no distance is found\n")
            {
                theTally.Differences.Add($"{theWhat}: native not done, port {aResult}");
            }

            theTally.BothFailed++;
            return;
        }

        Assert.True(theDI.TryGetNumber("d_val", out double aValue), theWhat);
        Same(theTally, aDist.Value(), aValue, theWhat + " value");

        // the multi-threaded inner solutions depend on the tasks' timing (decision 447): the value only
        if (theParallel)
        {
            return;
        }

        string[] aNames = aResult.Split('\n')[1].Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (aNames.Length - 1 != aDist.NbSolution())
        {
            theTally.Differences.Add($"{theWhat}: native {aDist.NbSolution()} solutions, port {aResult}");
            return;
        }

        for (int i = 1; i <= aDist.NbSolution(); i++)
        {
            N.gp_Pnt aP1 = aDist.PointOnShape1(i), aP2 = aDist.PointOnShape2(i);
            TopoDS_Shape aPort = theDI.GetShape(aNames[i]);
            N.TopoDS_Shape aNative = aDist.Value() <= 1.0e-9
                                   ? new N.BRepBuilderAPI_MakeVertex(aP1).Vertex()
                                   : new N.BRepBuilderAPI_MakeEdge(aP1, aP2).Edge();
            Compare(theTally, $"{theWhat} solution {i}", aNative, aPort);
            if (aPort.ShapeType() == TopAbs_ShapeEnum.TopAbs_VERTEX)
            {
                Same(theTally, aP1, BRep_Tool.Pnt(TopoDS.Vertex(aPort)), $"{theWhat} {aNames[i]}");
            }
            else
            {
                TopExp.Vertices(TopoDS.Edge(aPort), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                Same(theTally, aP1, BRep_Tool.Pnt(aV1), $"{theWhat} {aNames[i]} P1");
                Same(theTally, aP2, BRep_Tool.Pnt(aV2), $"{theWhat} {aNames[i]} P2");
            }
        }
    }

    [Fact]
    public void Distmini_matches_native_values_and_solutions()
    {
        var aTally = new Tally("distmini");
        var aRandom = new Random(3201);
        List<string> aFiles = SmallFiles();
        var aShapes = aFiles.Select(NativeRead).ToList();
        for (int k = 0; k < aShapes.Count; k++)
        {
            (string aFile, N.TopoDS_Shape aNative) = aShapes[k];
            DrawInterpreter aDI = Interpreter();
            Run(aDI, $"restore {TclPath(aFile)} a");

            // a point around the shape's box, at several deflections and both threading modes
            using var aBox = new N.Bnd_Box();
            N.BRepBndLib.Add(aNative, aBox);
            double x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;
            aBox.Get(ref x1, ref y1, ref z1, ref x2, ref y2, ref z2);
            for (int i = 0; i < 6; i++)
            {
                double x = x1 + (x2 - x1) * (1.4 * aRandom.NextDouble() - 0.2);
                double y = y1 + (y2 - y1) * (1.4 * aRandom.NextDouble() - 0.2);
                double z = z1 + (z2 - z1) * (1.4 * aRandom.NextDouble() - 0.2);
                Run(aDI, $"vertex b {R(x)} {R(y)} {R(z)}");
                using var aPoint = new N.BRepBuilderAPI_MakeVertex(new N.gp_Pnt(x, y, z));
                N.TopoDS_Shape aVertex = aPoint.Vertex();
                double aDeflection = i % 3 == 0 ? 1.0e-7 : i % 3 == 1 ? 1.0e-3 : 0.1;
                bool aParallel = i == 5;
                string anArgs = (i % 3 == 0 ? "" : R(aDeflection)) + (aParallel ? " -parallel" : "");
                DistMini(aTally, aDI, aNative, aVertex, anArgs, aDeflection, aParallel, $"{Path.GetFileName(aFiles[k])} to point {i}");
            }

            // the next file's shape, and its first face
            (string aFile2, N.TopoDS_Shape aNative2) = aShapes[(k + 1) % aShapes.Count];
            Run(aDI, $"restore {TclPath(aFile2)} b");
            DistMini(aTally, aDI, aNative, aNative2, "", 1.0e-7, false, $"{Path.GetFileName(aFiles[k])} to the next file");
            using var anExp = new N.TopExp_Explorer(aNative2, N.TopAbs_ShapeEnum.TopAbs_FACE);
            if (anExp.More())
            {
                Run(aDI, "explode b f; copy b_1 b");
                DistMini(aTally, aDI, aNative, anExp.Current(), "", 1.0e-7, false, $"{Path.GetFileName(aFiles[k])} to a face of the next file");
            }
        }

        Finish(aTally);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // constructions

    private static N.gp_Pnt NP(double x, double y, double z) => new(x, y, z);

    [Fact]
    public void Constructions_match_the_native_makers()
    {
        var aTally = new Tally("constructions");
        var aRandom = new Random(3202);
        double Real(double a = -100, double b = 100) => a + (b - a) * aRandom.NextDouble();

        foreach (int c in Tape.Budget(60))
        {
            DrawInterpreter aDI = Interpreter();

            // vertices and the edge between them
            double[] p = Enumerable.Range(0, 6).Select(_ => Real()).ToArray();
            Run(aDI, $"vertex v1 {R(p[0])} {R(p[1])} {R(p[2])}; vertex v2 {R(p[3])} {R(p[4])} {R(p[5])}; edge e v1 v2");
            using var aV1 = new N.BRepBuilderAPI_MakeVertex(NP(p[0], p[1], p[2]));
            using var aV2 = new N.BRepBuilderAPI_MakeVertex(NP(p[3], p[4], p[5]));
            Compare(aTally, $"vertex {c}", aV1.Vertex(), aDI.GetShape("v1"));
            using var anE = new N.BRepBuilderAPI_MakeEdge(aV1.Vertex(), aV2.Vertex());
            Compare(aTally, $"edge {c}", anE.Edge(), aDI.GetShape("e"));

            // a polyline of 3 to 6 points, closed every other time
            int aNb = aRandom.Next(3, 7);
            var aPoints = Enumerable.Range(0, aNb).Select(_ => (Real(), Real(), Real())).ToList();
            if (c % 2 == 0)
            {
                aPoints.Add(aPoints[0]);
            }

            Run(aDI, "polyline p " + string.Join(" ", aPoints.Select(q => $"{R(q.Item1)} {R(q.Item2)} {R(q.Item3)}")));
            using var aPolygon = new N.BRepBuilderAPI_MakePolygon();
            foreach ((double x, double y, double z) in aPoints)
            {
                aPolygon.Add(NP(x, y, z));
            }

            Compare(aTally, $"polyline {c}", aPolygon.Wire(), aDI.GetShape("p"));

            // a wire of chained edges, given out of order
            Run(aDI, $"vertex w1 {R(p[3])} {R(p[1])} {R(p[5])}; vertex w2 {R(p[0])} {R(p[4])} {R(p[2])}; edge e2 v2 w1; edge e3 w1 w2; wire w e3 e e2");
            using var aW1 = new N.BRepBuilderAPI_MakeVertex(NP(p[3], p[1], p[5]));
            using var aW2 = new N.BRepBuilderAPI_MakeVertex(NP(p[0], p[4], p[2]));
            using var anE2 = new N.BRepBuilderAPI_MakeEdge(aV2.Vertex(), aW1.Vertex());
            using var anE3 = new N.BRepBuilderAPI_MakeEdge(aW1.Vertex(), aW2.Vertex());
            using var aWire = new N.BRepBuilderAPI_MakeWire();
            aWire.Add(anE3.Edge());
            aWire.Add(anE.Edge());
            aWire.Add(anE2.Edge());
            if (aWire.IsDone())
            {
                Compare(aTally, $"wire {c}", aWire.Wire(), aDI.GetShape("w"));
            }
            else
            {
                Assert.Equal("0", Run(aDI, "isdraw w"));
                aTally.BothFailed++;
            }

            // a circle's edge between two parameters, the circle made as GeomliteTest's anacurve makes it
            double u1 = Real(-3, 3), u2 = u1 + Real(0.1, 6), aRadius = Real(0.1, 50);
            Run(aDI, $"circle c {R(p[0])} {R(p[1])} {R(p[2])} {R(p[3])} {R(p[4])} {R(p[5])} {R(aRadius)}; mkedge ec c {R(u1)} {R(u2)}");
            using var aNCircle = new N.Geom_Circle(new N.gp_Ax2(NP(p[0], p[1], p[2]), new N.gp_Dir(p[3], p[4], p[5])), aRadius);
            using var anEC = new N.BRepBuilderAPI_MakeEdge(aNCircle, u1, u2);
            Compare(aTally, $"mkedge {c}", anEC.Edge(), aDI.GetShape("ec"));

            // faces and shells on a cylinder
            double v1 = Real(-10, 10), v2 = v1 + Real(0.1, 10);
            aRadius = Real(0.1, 50);
            Run(aDI, $"cylinder cy {R(p[0])} {R(p[1])} {R(p[2])} {R(p[3])} {R(p[4])} {R(p[5])} {R(aRadius)}; "
                     + $"mkface fc cy {R(u1)} {R(u2)} {R(v1)} {R(v2)}; mkshell sc cy {R(u1)} {R(u2)} {R(v1)} {R(v2)}");
            using var aNCyl = new N.Geom_CylindricalSurface(new N.gp_Ax3(NP(p[0], p[1], p[2]), new N.gp_Dir(p[3], p[4], p[5])), aRadius);
            using var aFC = new N.BRepBuilderAPI_MakeFace(aNCyl, u1, u2, v1, v2, 1.0e-7);
            Compare(aTally, $"mkface {c}", aFC.Face(), aDI.GetShape("fc"));
            using var aSC = new N.BRepBuilderAPI_MakeShell(aNCyl, u1, u2, v1, v2, false);
            Compare(aTally, $"mkshell {c}", aSC.Shell(), aDI.GetShape("sc"));

            // mkplane on a convex quadrilateral at a random height
            double h = Real();
            double[] r = Enumerable.Range(0, 4).Select(_ => Real(1, 50)).ToArray();
            (double, double, double)[] aQuad = [(r[0], 0, h), (0, r[1], h), (-r[2], 0, h), (0, -r[3], h), (r[0], 0, h)];
            Run(aDI, "polyline q " + string.Join(" ", aQuad.Select(q => $"{R(q.Item1)} {R(q.Item2)} {R(q.Item3)}")) + "; mkplane f q");
            using var aQ = new N.BRepBuilderAPI_MakePolygon();
            foreach ((double x, double y, double z) in aQuad)
            {
                aQ.Add(NP(x, y, z));
            }

            using var aF = new N.BRepBuilderAPI_MakeFace(aQ.Wire(), false);
            Compare(aTally, $"mkplane {c}", aF.Face(), aDI.GetShape("f"));
        }

        Finish(aTally);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // profile: the moving point of BRepTest_CurveCommands.cxx with native makers

    private static N.TopoDS_Shape NativeProfile(IReadOnlyList<string> a)
    {
        int n = a.Count;
        int i = 2;
        double x0 = 0, y0 = 0, x = 0, y = 0, dx = 1, dy = 0;
        using var MW = new N.BRepBuilderAPI_MakeWire();
        var P = new N.gp_Pln(new N.gp_Ax3(N.gp.XOY()));
        bool face = true, close = true;
        double V(int k) => double.Parse(a[k], CultureInfo.InvariantCulture);
        char Second(string s) => s.Length > 1 ? s[1] : '\0';
        while (i < n)
        {
            double length = 0, radius = 0, angle = 0;
            int move = 0; // 0 none, 1 line, 2 circle
            switch (char.ToUpperInvariant(a[i][0]))
            {
                case 'O':
                    i += 3;
                    P.SetLocation(NP(V(i - 2), V(i - 1), V(i)));
                    break;
                case 'P':
                {
                    i += 6;
                    using var vn = new N.gp_Vec(V(i - 5), V(i - 4), V(i - 3));
                    using var vx = new N.gp_Vec(V(i - 2), V(i - 1), V(i));
                    using var ax = new N.gp_Ax2(P.Location(), new N.gp_Dir(vn), new N.gp_Dir(vx));
                    P.SetPosition(new N.gp_Ax3(ax));
                    break;
                }

                case 'F':
                    i += 2;
                    x0 = x = V(i - 1);
                    y0 = y = V(i);
                    break;
                case 'X':
                    i++;
                    length = V(i);
                    if (char.ToUpperInvariant(Second(a[i - 1])) == 'X')
                    {
                        length -= x;
                    }

                    dx = 1;
                    dy = 0;
                    move = 1;
                    break;
                case 'Y':
                    i++;
                    length = V(i);
                    if (char.ToUpperInvariant(Second(a[i - 1])) == 'Y')
                    {
                        length -= y;
                    }

                    dx = 0;
                    dy = 1;
                    move = 1;
                    break;
                case 'L':
                    i++;
                    length = V(i);
                    move = 1;
                    break;
                case 'T':
                {
                    i += 2;
                    double vx = V(i - 1), vy = V(i);
                    if (char.ToUpperInvariant(Second(a[i - 2])) == 'T')
                    {
                        vx -= x;
                        vy -= y;
                    }

                    length = Math.Sqrt(vx * vx + vy * vy);
                    if (length > 1.0e-7)
                    {
                        move = 1;
                        dx = vx / length;
                        dy = vy / length;
                    }

                    break;
                }

                case 'R':
                    i++;
                    angle = V(i) * (Math.PI / 180.0);
                    if (char.ToUpperInvariant(Second(a[i - 1])) == 'R')
                    {
                        dx = Math.Cos(angle);
                        dy = Math.Sin(angle);
                    }
                    else
                    {
                        double cs = Math.Cos(angle), sn = Math.Sin(angle);
                        double t = cs * dx - sn * dy;
                        dy = sn * dx + cs * dy;
                        dx = t;
                    }

                    break;
                case 'D':
                {
                    i += 2;
                    double vx = V(i - 1), vy = V(i);
                    length = Math.Sqrt(vx * vx + vy * vy);
                    if (length > 1.0e-7)
                    {
                        dx = vx / length;
                        dy = vy / length;
                    }

                    break;
                }

                case 'C':
                    i += 2;
                    radius = V(i - 1);
                    if (Math.Abs(radius) > 1.0e-7)
                    {
                        angle = V(i) * (Math.PI / 180.0);
                        move = 2;
                    }

                    break;
                case 'I':
                    i++;
                    length = V(i);
                    if (char.ToUpperInvariant(Second(a[i - 1])) == 'X')
                    {
                        length = Math.Abs(dx) < 1.0e-7 ? throw new InvalidOperationException("cannot intersect") : (length - x) / dx;
                    }
                    else
                    {
                        length = Math.Abs(dy) < 1.0e-7 ? throw new InvalidOperationException("cannot intersect") : (length - y) / dy;
                    }

                    move = 1;
                    break;
                case 'W':
                    face = false;
                    close = char.ToUpperInvariant(Second(a[i])) != 'W';
                    i = n - 1;
                    break;
            }

            while (true)
            {
                if (move == 1)
                {
                    if (length < 0)
                    {
                        length = -length;
                        dx = -dx;
                        dy = -dy;
                    }

                    using var l = new N.Geom2d_Line(new N.gp_Pnt2d(x, y), new N.gp_Dir2d(dx, dy));
                    using var anEdge = new N.BRepBuilderAPI_MakeEdge(N.GeomAPI.To3d(l, P), 0, length);
                    MW.Add(anEdge.Edge());
                    x += length * dx;
                    y += length * dy;
                }
                else if (move == 2)
                {
                    bool sense = true;
                    if (radius < 0)
                    {
                        radius = -radius;
                        sense = !sense;
                        dx = -dx;
                        dy = -dy;
                    }

                    using var ax = new N.gp_Ax2d(new N.gp_Pnt2d(x - radius * dy, y + radius * dx), new N.gp_Dir2d(dy, -dx));
                    if (angle < 0)
                    {
                        angle = -angle;
                        sense = !sense;
                    }

                    using var cc = new N.Geom2d_Circle(ax, radius, sense);
                    using var anEdge = new N.BRepBuilderAPI_MakeEdge(N.GeomAPI.To3d(cc, P), 0, angle);
                    MW.Add(anEdge.Edge());
                    var pp = new N.gp_Pnt2d();
                    var vv = new N.gp_Vec2d();
                    cc.D1(angle, pp, vv);
                    x = pp.X();
                    y = pp.Y();
                    dx = vv.X() / radius;
                    dy = vv.Y() / radius;
                }

                i++;
                move = 0;
                if (i == n && close)
                {
                    dx = x0 - x;
                    dy = y0 - y;
                    length = Math.Sqrt(dx * dx + dy * dy);
                    if (length > 1.0e-7)
                    {
                        move = 1;
                        dx /= length;
                        dy /= length;
                        continue;
                    }
                }

                break;
            }
        }

        return face ? new N.BRepBuilderAPI_MakeFace(P, MW.Wire(), true).Face() : MW.Wire();
    }

    [Fact]
    public void Profile_matches_the_native_makers()
    {
        var aTally = new Tally("profile");
        var aRandom = new Random(3203);
        string Real(double a, double b) => R(Math.Round(a + (b - a) * aRandom.NextDouble(), 3));
        string[] aMoves = ["X", "Y", "L", "XX", "YY", "T", "TT", "R", "RR", "D", "C", "IX", "IY"];
        foreach (int c in Tape.Budget(200))
        {
            var aWords = new List<string> { "profile", "p" };
            if (aRandom.Next(3) == 0)
            {
                aWords.AddRange(["O", Real(-50, 50), Real(-50, 50), Real(-50, 50)]);
            }

            if (aRandom.Next(3) == 0)
            {
                aWords.AddRange(["P", Real(-1, 1), Real(-1, 1), "1", "1", Real(-1, 1), Real(-1, 1)]);
            }

            if (aRandom.Next(2) == 0)
            {
                aWords.AddRange(["F", Real(-10, 10), Real(-10, 10)]);
            }

            int aNb = aRandom.Next(2, 7);
            for (int k = 0; k < aNb; k++)
            {
                string aMove = aMoves[aRandom.Next(aMoves.Length)];
                aWords.Add(aMove);
                switch (aMove)
                {
                    case "X" or "Y" or "L" or "XX" or "YY" or "IX" or "IY":
                        aWords.Add(Real(-20, 20));
                        break;
                    case "R" or "RR":
                        aWords.Add(Real(-180, 180));
                        break;
                    case "C":
                        aWords.AddRange([Real(-10, 10), Real(-270, 270)]);
                        break;
                    default:
                        aWords.AddRange([Real(-20, 20), Real(-20, 20)]);
                        break;
                }
            }

            int anEnd = aRandom.Next(3);
            if (anEnd > 0)
            {
                aWords.Add(anEnd == 1 ? "W" : "WW");
            }

            string aScript = string.Join(" ", aWords);
            DrawInterpreter aDI = Interpreter();
            string? aPortError = null;
            try
            {
                aDI.Eval(aScript);
            }
            catch (DrawException anError)
            {
                aPortError = anError.Reason;
            }

            N.TopoDS_Shape? aNative = null;
            try
            {
                aNative = NativeProfile(aWords);
            }
            catch (Exception) when (aPortError is not null)
            {
            }

            if (aPortError is not null || aNative is null)
            {
                if (aPortError is null || aNative is not null)
                {
                    aTally.Differences.Add($"{aScript}: port {aPortError ?? "done"}, native {(aNative is null ? "failed" : "done")}");
                }
                else
                {
                    aTally.BothFailed++;
                }

                continue;
            }

            Compare(aTally, aScript, aNative, aDI.GetShape("p"));
        }

        Finish(aTally);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // modifications on data/occ's files

    [Fact]
    public void Modifications_match_the_native_algorithms()
    {
        var aTally = new Tally("modifications");
        var aRandom = new Random(3204);
        foreach (string aSource in Tape.Budget(SmallFiles(), isSpread: true))
        {
            string aName = Path.GetFileName(aSource);
            (string aFile, N.TopoDS_Shape aShape) = NativeRead(aSource);
            DrawInterpreter aDI = Interpreter();
            Run(aDI, $"restore {TclPath(aFile)} s");

            // scalexyz and deform: BRepBuilderAPI_GTransform
            double fx = Math.Round(0.5 + 2 * aRandom.NextDouble(), 4), fy = Math.Round(0.5 + 2 * aRandom.NextDouble(), 4), fz = Math.Round(0.5 + 2 * aRandom.NextDouble(), 4);
            Run(aDI, $"scalexyz sx s {R(fx)} {R(fy)} {R(fz)}; deform df s {R(fz)} {R(fx)} {R(fy)}");
            using (var aGT = new N.gp_GTrsf())
            {
                aGT.SetVectorialPart(new N.gp_Mat(fx, 0, 0, 0, fy, 0, 0, 0, fz));
                using var aMaker = new N.BRepBuilderAPI_GTransform(aShape, aGT, false);
                Compare(aTally, $"{aName} scalexyz", aMaker.Shape(), aDI.GetShape("sx"));
            }

            using (var aGT = new N.gp_GTrsf(new N.gp_Trsf()))
            {
                aGT.SetVectorialPart(new N.gp_Mat(fz, 0, 0, 0, fx, 0, 0, 0, fy));
                using var aMaker = new N.BRepBuilderAPI_GTransform(aGT);
                aMaker.Perform(aShape);
                Compare(aTally, $"{aName} deform", aMaker.Shape(), aDI.GetShape("df"));
            }

            // sameparameter and updatetolerance with a result: through a reshaper on both sides
            Run(aDI, "sameparameter r1 s 1e-5; fsameparameter r2 s; updatetolerance r3 s; updatetolerance r4 s 1");
            using (var aResh = new N.BRepTools_ReShape())
            {
                N.BRepLib.SameParameter(aShape, aResh, 1.0e-5, false);
                Compare(aTally, $"{aName} sameparameter r s", aResh.Apply(aShape), aDI.GetShape("r1"));
            }

            using (var aResh = new N.BRepTools_ReShape())
            {
                N.BRepLib.SameParameter(aShape, aResh, 1.0e-7, true);
                Compare(aTally, $"{aName} fsameparameter r s", aResh.Apply(aShape), aDI.GetShape("r2"));
            }

            using (var aResh = new N.BRepTools_ReShape())
            {
                N.BRepLib.UpdateTolerances(aShape, aResh, false);
                Compare(aTally, $"{aName} updatetolerance r s", aResh.Apply(aShape), aDI.GetShape("r3"));
            }

            using (var aResh = new N.BRepTools_ReShape())
            {
                N.BRepLib.UpdateTolerances(aShape, aResh, true);
                Compare(aTally, $"{aName} updatetolerance r s 1", aResh.Apply(aShape), aDI.GetShape("r4"));
            }

            // in place: native changes its own copy of the file's shape, the port binds a changed copy
            (string Script, Action<N.TopoDS_Shape> Native)[] anInPlace =
            [
                ("sameparameter s", s => N.BRepLib.SameParameter(s, 1.0e-7, false)),
                ("fsameparameter s 1e-4", s => N.BRepLib.SameParameter(s, 1.0e-4, true)),
                ("updatetolerance s", s => N.BRepLib.UpdateTolerances(s, false)),
                ("updatetolerance s 1", s => N.BRepLib.UpdateTolerances(s, true)),
                ("encoderegularity s", s => N.BRepLib.EncodeRegularity(s, 1.0e-10)),
                ("encoderegularity s 5", s => N.BRepLib.EncodeRegularity(s, 5 * (Math.PI / 180.0))),
            ];
            foreach ((string aScript, Action<N.TopoDS_Shape> aNative) in anInPlace)
            {
                DrawInterpreter aDI2 = Interpreter();
                Run(aDI2, $"restore {TclPath(aFile)} s; {aScript}");
                // native reads the same file, as the port's restore does
                var aCopy = new N.TopoDS_Shape();
                using (var aBuilder = new N.BRep_Builder())
                {
                    Assert.True(N.BRepTools.Read(aCopy, aFile, aBuilder));
                }

                aNative(aCopy);
                TopoDS_Shape aPort = aDI2.GetShape("s");

                // OCCT's in-place UpdateTolerances sets a vertex shared at several locations once per location, in
                // the order of a map hashed on addresses (occt-issues.md): the last value written can't be
                // reproduced, so such shapes are compared with the vertex tolerances masked
                if (aScript.StartsWith("updatetolerance", StringComparison.Ordinal) && HasVertexAtSeveralLocations(aPort))
                {
                    string aN = NativeText(aCopy), aP = PortText(aPort);
                    if (aN != aP)
                    {
                        aTally.OrderDependent++;
                        if (VertexTolerance().Replace(aN, "Ve\nx") != VertexTolerance().Replace(aP, "Ve\nx"))
                        {
                            aTally.Differences.Add($"{aName} {aScript}: differs beyond the vertex tolerances");
                        }

                        continue;
                    }
                }

                Compare(aTally, $"{aName} {aScript}", aCopy, aPort);
            }

            // solidorientation on each solid
            using var anExp = new N.TopExp_Explorer(aShape, N.TopAbs_ShapeEnum.TopAbs_SOLID);
            if (anExp.More())
            {
                Run(aDI, aDI.GetShape("s").ShapeType() == TopAbs_ShapeEnum.TopAbs_SOLID ? "copy s so" : "explode s so; copy s_1 so");
                N.TopoDS_Solid aSolid = N.TopoDS.Solid(anExp.Current());
                N.BRepLib.OrientClosedSolid(aSolid);
                Run(aDI, "solidorientation so");
                Compare(aTally, $"{aName} solidorientation", aSolid, aDI.GetShape("so"));
                aTally.Values++;
                Assert.Equal((int)aSolid.Orientation(), (int)aDI.GetShape("so").Orientation());
            }
        }

        Finish(aTally);
    }
}
