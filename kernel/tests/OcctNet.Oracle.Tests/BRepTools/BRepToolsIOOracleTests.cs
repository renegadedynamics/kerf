// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Reflection;
using System.Text;
using Xunit.Abstractions;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepTools' text format against native OCCT's <c>BRepTools::Write</c> and <c>Read</c>, byte for byte.
/// </summary>
/// <remarks>
/// <para>
/// Reading normalises directions and axes again, so a file written after a read can differ from the first in the last
/// digits. The comparisons therefore follow decision 233's scheme: native OCCT writes a shape (F1), reads F1 and writes
/// it again (F2); the port reads F1 and must write F2's bytes. The share of files where F1 already equals F2 is
/// reported. Shapes the port builds call for call as native OCCT does (<see cref="TwinShapes"/>) must be written
/// exactly as native OCCT writes its twin.
/// </para>
/// <para>
/// All files are written in every format version (1 to 3), with and without triangulations and normals.
/// </para>
/// </remarks>
public class BRepToolsIOOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepio-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static readonly (FormatVersion Version, bool Triangles, bool Normals)[] theFormats =
    [
        (FormatVersion_VERSION_1, true, false),
        (FormatVersion_VERSION_1, false, false),
        (FormatVersion_VERSION_2, true, false),
        (FormatVersion_VERSION_2, false, false),
        (FormatVersion_VERSION_3, true, false),
        (FormatVersion_VERSION_3, true, true),
        (FormatVersion_VERSION_3, false, false),
        (FormatVersion_VERSION_3, false, true),
    ];

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private static string Text(string theFile) => File.ReadAllText(theFile, Encoding.Latin1);

    /// <summary>Native BRepTools::Write of the shape to a file; the file's path.</summary>
    private string NativeWrite(N.TopoDS_Shape theShape, (FormatVersion Version, bool Triangles, bool Normals) theFormat)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, theFormat.Triangles, theFormat.Normals,
                                      (N.TopTools_FormatVersion)(int)theFormat.Version));
        return aFile;
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    private string PortWrite(in Shape theShape, (FormatVersion Version, bool Triangles, bool Normals) theFormat)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, theFormat.Triangles, theFormat.Normals, theFormat.Version));
        return aFile;
    }

    private static Shape PortRead(string theFile)
    {
        Assert.True(BrepTools.Read(out Shape aShape, theFile), theFile);
        Assert.True(aShape.TShape()!.IsFrozen);
        return aShape;
    }

    private sealed class Counts
    {
        public int Files;
        public int SameAsF1;
        public long Bytes;

        public string Report(string theWhat) => $"{theWhat}: {Files} files, {Bytes} bytes, all byte-identical; F1 = F2 in {SameAsF1}";
    }

    /// <summary>
    /// F1: native writes the shape; F2: native reads F1 and writes it; the port reads F1 and writes F2. Returns the
    /// port's shape.
    /// </summary>
    private Shape CheckF1F2(N.TopoDS_Shape theNative, (FormatVersion, bool, bool) theFormat, Counts theCounts, string theWhat)
    {
        string aF1 = NativeWrite(theNative, theFormat);
        string aF2 = NativeWrite(NativeRead(aF1), theFormat);
        Shape aShape = PortRead(aF1);
        string aP = PortWrite(aShape, theFormat);
        string aF2Text = Text(aF2);
        Assert.True(aF2Text == Text(aP), $"{theWhat} {theFormat}: the port's file differs from native OCCT's ({aP}, {aF2})");
        theCounts.Files++;
        theCounts.Bytes += aF2Text.Length;
        if (Text(aF1) == aF2Text)
        {
            theCounts.SameAsF1++;
        }

        return aShape;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Native shapes

    private static N.gp_Vec NVec(Oracle theOracle, double theMin, double theMax)
        => new(theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax), theOracle.Real(theMin, theMax));

    private static N.TopoDS_Face NativePolygonFace(Oracle theOracle)
    {
        // a convex polygon in the XY plane, at a random height
        double z = theOracle.Real(-2, 2);
        int aNb = theOracle.Int(3, 6);
        using var aPolygon = new N.BRepBuilderAPI_MakePolygon();
        double aPhase = theOracle.Angle();
        for (int i = 0; i < aNb; i++)
        {
            double anAngle = aPhase + 2 * Math.PI * i / aNb;
            double aR = theOracle.Real(1, 3);
            using var aP = new N.gp_Pnt(aR * Math.Cos(anAngle), aR * Math.Sin(anAngle), z);
            aPolygon.Add(aP);
        }

        aPolygon.Close();
        using var aFace = new N.BRepBuilderAPI_MakeFace(aPolygon.Wire());
        return aFace.Face();
    }

    /// <summary>A random native shape: a primitive, a sweep, a Boolean result or a fillet, placed, meshed or not.</summary>
    internal static N.TopoDS_Shape NativeShape(Oracle theOracle, int theIndex, out string theWhat)
    {
        N.TopoDS_Shape aShape;
        (N.gp_Ax2 anAx2, _) = GeomOracleInputs.Ax2s(theOracle);
        using (anAx2)
        {
            switch (theIndex % 12)
            {
                case 0:
                    theWhat = "box";
                    aShape = new N.BRepPrimAPI_MakeBox(anAx2, theOracle.Real(0.5, 10), theOracle.Real(0.5, 10), theOracle.Real(0.5, 10)).Shape();
                    break;
                case 1:
                    theWhat = "cylinder";
                    aShape = theOracle.Bool()
                        ? new N.BRepPrimAPI_MakeCylinder(anAx2, theOracle.Real(0.5, 5), theOracle.Real(0.5, 5)).Shape()
                        : new N.BRepPrimAPI_MakeCylinder(anAx2, theOracle.Real(0.5, 5), theOracle.Real(0.5, 5), theOracle.Real(0.5, 6)).Shape();
                    break;
                case 2:
                    theWhat = "sphere";
                    aShape = theOracle.Bool()
                        ? new N.BRepPrimAPI_MakeSphere(theOracle.Real(0.5, 5)).Shape()
                        : new N.BRepPrimAPI_MakeSphere(theOracle.Real(0.5, 5), theOracle.Real(-1.5, -0.1), theOracle.Real(0.1, 1.5),
                                                       theOracle.Real(1, 6)).Shape();
                    break;
                case 3:
                    theWhat = "cone";
                    aShape = new N.BRepPrimAPI_MakeCone(anAx2, theOracle.Real(0.5, 3), theOracle.Bool() ? 0 : theOracle.Real(0.1, 2),
                                                        theOracle.Real(0.5, 5)).Shape();
                    break;
                case 4:
                    theWhat = "torus";
                    aShape = new N.BRepPrimAPI_MakeTorus(anAx2, theOracle.Real(3, 6), theOracle.Real(0.5, 2)).Shape();
                    break;
                case 5:
                    theWhat = "wedge";
                    aShape = new N.BRepPrimAPI_MakeWedge(anAx2, theOracle.Real(1, 5), theOracle.Real(1, 5), theOracle.Real(1, 5),
                                                         theOracle.Real(0, 0.9)).Shape();
                    break;
                case 6:
                {
                    theWhat = "prism";
                    using N.TopoDS_Face aFace = NativePolygonFace(theOracle);
                    using N.gp_Vec aV = NVec(theOracle, 0.5, 4);
                    aShape = new N.BRepPrimAPI_MakePrism(aFace, aV).Shape();
                }
                    break;
                case 7:
                {
                    theWhat = "revolution";
                    using N.TopoDS_Face aFace = NativePolygonFace(theOracle);
                    using var aP = new N.gp_Pnt(5, 0, 0);
                    using var aD = new N.gp_Dir(0, 1, 0);
                    using var anAxis = new N.gp_Ax1(aP, aD);
                    aShape = theOracle.Bool()
                        ? new N.BRepPrimAPI_MakeRevol(aFace, anAxis).Shape()
                        : new N.BRepPrimAPI_MakeRevol(aFace, anAxis, theOracle.Real(0.5, 6)).Shape();
                }
                    break;
                case 8:
                case 9:
                {
                    using N.TopoDS_Shape aBox = new N.BRepPrimAPI_MakeBox(theOracle.Real(2, 5), theOracle.Real(2, 5), theOracle.Real(2, 5)).Shape();
                    using N.TopoDS_Shape aTool = theOracle.Bool()
                        ? new N.BRepPrimAPI_MakeCylinder(anAx2, theOracle.Real(0.5, 2), theOracle.Real(1, 6)).Shape()
                        : new N.BRepPrimAPI_MakeSphere(theOracle.Real(1, 3)).Shape();
                    switch (theOracle.Int(0, 2))
                    {
                        case 0:
                            theWhat = "fuse";
                            aShape = new N.BRepAlgoAPI_Fuse(aBox, aTool).Shape();
                            break;
                        case 1:
                            theWhat = "cut";
                            aShape = new N.BRepAlgoAPI_Cut(aBox, aTool).Shape();
                            break;
                        default:
                            theWhat = "common";
                            aShape = new N.BRepAlgoAPI_Common(aBox, aTool).Shape();
                            break;
                    }
                }
                    break;
                case 10:
                {
                    theWhat = "fillet";
                    using N.TopoDS_Shape aBox = new N.BRepPrimAPI_MakeBox(anAx2, theOracle.Real(3, 6), theOracle.Real(3, 6), theOracle.Real(3, 6)).Shape();
                    using var aFillet = new N.BRepFilletAPI_MakeFillet(aBox);
                    int anEdge = 0;
                    for (var anExp = new N.TopExp_Explorer(aBox, N.TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
                    {
                        if (anEdge++ % 3 == 0)
                        {
                            aFillet.Add(theOracle.Real(0.2, 0.8), N.TopoDS.Edge(anExp.Current()));
                        }
                    }

                    aShape = aFillet.Shape();
                }
                    break;
                default:
                {
                    theWhat = "loft";
                    using var aLoft = new N.BRepOffsetAPI_ThruSections(theOracle.Bool(), theOracle.Bool());
                    using N.TopoDS_Face aBottom = NativePolygonFace(theOracle);
                    using var aTop = new N.BRepBuilderAPI_MakePolygon();
                    int aNbTop = 0;
                    for (var anExp = new N.TopExp_Explorer(aBottom, N.TopAbs_ShapeEnum.TopAbs_VERTEX); anExp.More(); anExp.Next())
                    {
                        aNbTop++;
                    }

                    // the same number of vertices on top (each vertex is met twice by the explorer)
                    double z = theOracle.Real(3, 6);
                    for (int i = 0; i < aNbTop / 2; i++)
                    {
                        double anAngle = 2 * Math.PI * i / (aNbTop / 2);
                        using var aP = new N.gp_Pnt(Math.Cos(anAngle), Math.Sin(anAngle), z);
                        aTop.Add(aP);
                    }

                    aTop.Close();
                    using N.TopoDS_Wire aBottomWire = OuterWire(aBottom);
                    aLoft.AddWire(aBottomWire);
                    aLoft.AddWire(aTop.Wire());
                    aShape = aLoft.Shape();
                }
                    break;
            }
        }

        if (theOracle.Bool())
        {
            (N.TopLoc_Location aNL, _) = TopologyOracleTests.RandomLocation(theOracle);
            aShape = aShape.Moved(aNL);
            theWhat += ", placed";
        }

        if (theOracle.Int(0, 2) == 0)
        {
            using var aMesh = new N.BRepMesh_IncrementalMesh(aShape, theOracle.Real(0.05, 0.5));
            theWhat += ", meshed";
        }

        return aShape;
    }

    private static N.TopoDS_Wire OuterWire(N.TopoDS_Face theFace) => N.BRepTools.OuterWire(theFace);

    [Fact]
    public void Native_shapes_read_by_the_port_are_written_as_native_OCCT_writes_them()
    {
        var anOracle = new Oracle(output, 33001);
        var aCounts = new Counts();
        var aKinds = new SortedDictionary<string, int>(StringComparer.Ordinal);
        int aSkipped = 0;
        foreach (int i in Tape.Budget(96))
        {
            N.TopoDS_Shape aNative;
            string aWhat;
            try
            {
                aNative = NativeShape(anOracle, i, out aWhat);
            }
            catch (Exception anException) when (anException is not Xunit.Sdk.XunitException)
            {
                // a native algorithm that didn't finish
                aSkipped++;
                continue;
            }

            aKinds[aWhat] = aKinds.GetValueOrDefault(aWhat) + 1;
            foreach ((FormatVersion, bool, bool) aFormat in theFormats)
            {
                CheckF1F2(aNative, aFormat, aCounts, aWhat);
            }
        }

        output.WriteLine(aCounts.Report("native shapes") + $"; {aSkipped} skipped");
        output.WriteLine(string.Join(", ", aKinds.Select(k => $"{k.Key} {k.Value}")));
        Assert.True(aCounts.Files > 0); // as recorded, a budgeted sample (decision 1337)
    }

    [Fact]
    public void Twin_shapes_are_written_exactly_as_native_OCCT_writes_them()
    {
        var anOracle = new Oracle(output, 33002);
        int aFiles = 0;
        long aBytes = 0;
        for (int aScene = 0; aScene < 60; aScene++)
        {
            Twin<N.TopoDS_Shape, Shape> aShape = TopologyOracleTests.RandomScene(anOracle, aScene);
            if (aScene % 3 == 0)
            {
                // a triangulation on every face
                var aTwins = new TwinShapes();
                var aNExp = new N.TopExp_Explorer(aShape.N, N.TopAbs_ShapeEnum.TopAbs_FACE);
                var anExp = new Explorer(aShape.P, ShapeType.Face);
                for (; anExp.More(); anExp.Next(), aNExp.Next())
                {
                    aTwins.AttachTriangulation(new(N.TopoDS.Face(aNExp.Current()), ShapeCasts.Face(anExp.Current())), anOracle.Real(0.01, 1));
                }
            }

            foreach ((FormatVersion, bool, bool) aFormat in theFormats)
            {
                string aNative = Text(NativeWrite(aShape.N, aFormat));
                string aPort = Text(PortWrite(aShape.P, aFormat));
                Assert.True(aNative == aPort, $"scene {aScene} {aFormat}");
                aFiles++;
                aBytes += aPort.Length;
            }
        }

        output.WriteLine($"twin shapes: {aFiles} files, {aBytes} bytes, all byte-identical");
    }

    [Fact]
    public void Files_the_port_writes_are_read_by_native_OCCT_as_the_port_reads_them()
    {
        // the port's files (of shapes it read, and of shapes it built): native OCCT reads one and writes it (N2), the
        // port reads it and writes it (P2): N2 = P2
        var anOracle = new Oracle(output, 33003);
        int aFiles = 0, aSameAsWritten = 0;
        long aBytes = 0;
        var aShapes = new List<Shape>();
        for (int i = 0; i < 48; i++)
        {
            try
            {
                N.TopoDS_Shape aNative = NativeShape(anOracle, i, out _);
                aShapes.Add(PortRead(NativeWrite(aNative, (FormatVersion_CURRENT, true, true))));
            }
            catch (Exception anException) when (anException is not Xunit.Sdk.XunitException)
            {
                // a native algorithm that didn't finish
            }
        }

        for (int aScene = 0; aScene < 24; aScene++)
        {
            aShapes.Add(TopologyOracleTests.RandomScene(anOracle, aScene).P);
        }

        foreach (Shape aShape in aShapes)
        {
            foreach ((FormatVersion, bool, bool) aFormat in theFormats)
            {
                string aP1 = PortWrite(aShape, aFormat);
                string aN2 = Text(NativeWrite(NativeRead(aP1), aFormat));
                string aP2 = Text(PortWrite(PortRead(aP1), aFormat));
                Assert.True(aN2 == aP2, $"{aP1}: native OCCT and the port read it differently");
                aFiles++;
                aBytes += aP2.Length;
                if (aN2 == Text(aP1))
                {
                    aSameAsWritten++;
                }
            }
        }

        output.WriteLine($"port files: {aFiles} files, {aBytes} bytes, all byte-identical; the second write equals the first in {aSameAsWritten}");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The corpus

    private static string OcctSourceDirectory
        => typeof(BRepToolsIOOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctSourceDir").Value!;

    private static string OcctTestDataDirectory
        => typeof(BRepToolsIOOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!;

    /// <summary>
    /// The F1/F2 scheme on files: native OCCT reads the file and writes F1 (the current format, with triangulations),
    /// reads F1 and writes F2; the port reads F1 and writes F2's bytes. Files native OCCT can't read are counted apart.
    /// </summary>
    private (int Files, int Unreadable, long Bytes, int SameAsF1) Corpus(IEnumerable<string> theFiles)
    {
        int aFiles = 0, anUnreadable = 0, aSameAsF1 = 0;
        long aBytes = 0;
        var aFormat = (FormatVersion_CURRENT, true, false);
        foreach (string aFile in Tape.Budget(theFiles, isSpread: true))
        {
            var aShape = new N.TopoDS_Shape();
            using (var aBuilder = new N.BRep_Builder())
            {
                if (!N.BRepTools.Read(aShape, aFile, aBuilder) || aShape.IsNull())
                {
                    // Draw's geometry files (DrawTrSurf_...) and the like: the port doesn't read a shape either
                    Assert.False(BrepTools.Read(out Shape aNone, aFile) && !aNone.IsNull, aFile);
                    anUnreadable++;
                    continue;
                }
            }

            string aF1 = NativeWrite(aShape, aFormat);
            aShape.Dispose();
            string aF2 = NativeWrite(NativeRead(aF1), aFormat);
            string aP = PortWrite(PortRead(aF1), aFormat);
            string aF2Text = Text(aF2);
            Assert.True(aF2Text == Text(aP), $"{aFile}: the port's file differs from native OCCT's ({aP}, {aF2})");
            aFiles++;
            aBytes += aF2Text.Length;
            if (aF2Text == Text(aF1))
            {
                aSameAsF1++;
            }

            File.Delete(aF1);
            File.Delete(aF2);
            File.Delete(aP);
        }

        return (aFiles, anUnreadable, aBytes, aSameAsF1);
    }

    [Fact]
    public void The_brep_files_of_data_occ_round_trip_as_native_OCCT()
    {
        string aDirectory = Path.Combine(OcctSourceDirectory, "data", "occ");
        string[] aFiles = Directory.GetFiles(aDirectory, "*.brep").Order(StringComparer.Ordinal).ToArray();
        Assert.Equal(37, aFiles.Length);
        (int aRead, int anUnreadable, long aBytes, int aSame) = Corpus(aFiles);
        output.WriteLine($"data/occ: {aRead} of {aFiles.Length} .brep files byte-identical to native F2 ({aBytes} bytes), "
                         + $"{anUnreadable} unreadable by native OCCT; F1 = F2 in {aSame}");
        Assert.True(aRead > 0);

        string[] aRle = Directory.GetFiles(aDirectory, "*.rle").Order(StringComparer.Ordinal).ToArray();
        (aRead, anUnreadable, aBytes, aSame) = Corpus(aRle);
        output.WriteLine($"data/occ: {aRead} of {aRle.Length} .rle files byte-identical ({aBytes} bytes), {anUnreadable} unreadable; F1 = F2 in {aSame}");
    }

    [Fact]
    public void The_brep_and_rle_files_of_OCCTs_test_data_round_trip_as_native_OCCT()
    {
        Assert.True(Directory.Exists(OcctTestDataDirectory), $"No test data at {OcctTestDataDirectory}: run tools/get-occt-testdata.ps1");
        string[] aFiles = Directory.EnumerateFiles(OcctTestDataDirectory, "*", SearchOption.AllDirectories)
                                   .Where(f => f.EndsWith(".brep", StringComparison.OrdinalIgnoreCase)
                                               || f.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                                   .Order(StringComparer.Ordinal)
                                   .ToArray();
        (int aRead, int anUnreadable, long aBytes, int aSame) = Corpus(aFiles);
        output.WriteLine($"test data: {aRead} of {aFiles.Length} .brep/.rle files byte-identical to native F2 ({aBytes} bytes), "
                         + $"{anUnreadable} not readable as text by native OCCT; F1 = F2 in {aSame}");
        Assert.True(aRead > 0); // as recorded, a sample (decision 1337)
    }
}
