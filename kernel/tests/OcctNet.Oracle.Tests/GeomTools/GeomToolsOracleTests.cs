// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Xunit.Abstractions;
using static OcctNet.Oracle.Tests.AdaptorOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomTools against native OCCT's BRepTools::Write and Read, whose geometry sections ("Curve2ds", "Curves",
/// "Surfaces") are GeomTools output: the port must write the same text, byte for byte, for the same geometry, and
/// read native OCCT's files into geometry that it writes back as native OCCT does.
/// </summary>
public class GeomToolsOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-geomtools-").FullName;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private int myFileNumber;

    /// <summary>Native BRepTools::Write of the shape, read back as text with LF line ends.</summary>
    private string NativeWrite(N.TopoDS_Shape theShape)
    {
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile).Replace("\r\n", "\n", StringComparison.Ordinal);
    }

    /// <summary>The text of a section, from its header line to the next section (exclusive).</summary>
    private static string Section(string theText, string theHeader, string theNext)
    {
        int aStart = theText.IndexOf("\n" + theHeader + " ", StringComparison.Ordinal) + 1;
        Assert.True(aStart > 0, $"no {theHeader} section");
        int anEnd = theText.IndexOf("\n" + theNext + " ", aStart, StringComparison.Ordinal);
        return theText.Substring(aStart, anEnd + 1 - aStart);
    }

    private static string CurvesSection(string theText) => Section(theText, "Curves", "Polygon3D");

    private static string Curve2dsSection(string theText) => Section(theText, "Curve2ds", "Curves");

    private static string SurfacesSection(string theText) => Section(theText, "Surfaces", "Triangulations");

    private static string PortWrite(Geom_Curve theCurve)
    {
        var aSet = new GeomTools_CurveSet();
        aSet.Add(theCurve);
        var aWriter = new StringWriter();
        aSet.Write(aWriter);
        return aWriter.ToString();
    }

    private static string PortWrite(Geom2d_Curve theCurve)
    {
        var aSet = new GeomTools_Curve2dSet();
        aSet.Add(theCurve);
        var aWriter = new StringWriter();
        aSet.Write(aWriter);
        return aWriter.ToString();
    }

    private static string PortWrite(Geom_Surface theSurface)
    {
        var aSet = new GeomTools_SurfaceSet();
        aSet.Add(theSurface);
        var aWriter = new StringWriter();
        aSet.Write(aWriter);
        return aWriter.ToString();
    }

    private sealed class Counter(ITestOutputHelper theOutput, string theWhat)
    {
        private readonly SortedDictionary<string, (int Count, long Bytes)> myKinds = new(StringComparer.Ordinal);

        public void Add(string theKind, string theText)
        {
            myKinds.TryGetValue(theKind, out (int Count, long Bytes) aValue);
            myKinds[theKind] = (aValue.Count + 1, aValue.Bytes + theText.Length);
        }

        public void Report()
        {
            int aCount = myKinds.Values.Sum(v => v.Count);
            long aBytes = myKinds.Values.Sum(v => v.Bytes);
            theOutput.WriteLine($"{theWhat}: {aCount} objects, {aBytes} bytes, all byte-identical ("
                                + string.Join(", ", myKinds.Select(k => $"{k.Key} {k.Value.Count}")) + ")");
        }
    }

    [Fact]
    public void Curves_are_written_as_native_OCCT_writes_them()
    {
        var anOracle = new Oracle(output);
        var aCounter = new Counter(output, "3D curves");
        using var aBuilder = new N.BRep_Builder();
        foreach (int i in Tape.Budget(1000))
        {
            int aKind = i % CurveKinds.Length;
            (N.Geom_Curve aNative, Geom_Curve aPort, _) = RandomCurve(anOracle, aKind);
            using (aNative)
            {
                using var anEdge = new N.TopoDS_Edge();
                aBuilder.MakeEdge(anEdge, aNative, 1e-7);
                string anExpected = CurvesSection(NativeWrite(anEdge));
                string anActual = PortWrite(aPort);
                Assert.Equal(anExpected, anActual);
                aCounter.Add(CurveKinds[aKind], anActual);

                // and the port reads that text as native OCCT reads it: native OCCT reads the file and writes it again
                // (reading normalises directions again, so the text can change in the last digits), and the port
                // writes what it reads the same way
                var aSet = new GeomTools_CurveSet();
                aSet.Read(new StringReader(anExpected));
                Assert.Equal(CurvesSection(NativeRewrite()), PortWriteSet(aSet));
            }
        }

        aCounter.Report();
    }

    private static string PortWriteSet(GeomTools_CurveSet theSet)
    {
        var aWriter = new StringWriter();
        theSet.Write(aWriter);
        return aWriter.ToString();
    }

    [Fact]
    public void Surfaces_are_written_as_native_OCCT_writes_them()
    {
        var anOracle = new Oracle(output);
        var aCounter = new Counter(output, "surfaces");
        using var aBuilder = new N.BRep_Builder();
        foreach (int i in Tape.Budget(1100))
        {
            int aKind = i % SurfaceKinds.Length;
            (N.Geom_Surface aNative, Geom_Surface aPort) = RandomSurface(anOracle, aKind);
            using (aNative)
            {
                using var aFace = new N.TopoDS_Face();
                aBuilder.MakeFace(aFace, aNative, 1e-7);
                string anExpected = SurfacesSection(NativeWrite(aFace));
                string anActual = PortWrite(aPort);
                Assert.Equal(anExpected, anActual);
                aCounter.Add(SurfaceKinds[aKind], anActual);

                var aSet = new GeomTools_SurfaceSet();
                aSet.Read(new StringReader(anExpected));
                var aWriter = new StringWriter();
                aSet.Write(aWriter);
                Assert.Equal(SurfacesSection(NativeRewrite()), aWriter.ToString());
            }
        }

        aCounter.Report();
    }

    [Fact]
    public void Offset_surfaces_are_written_as_native_OCCT_writes_them()
    {
        var anOracle = new Oracle(output);
        var aCounter = new Counter(output, "offset surfaces");
        using var aBuilder = new N.BRep_Builder();
        foreach (int i in Tape.Budget(300))
        {
            (N.Geom_Surface aNative, Geom_Surface aPort) = RandomOffsetSurface(anOracle);
            using (aNative)
            {
                using var aFace = new N.TopoDS_Face();
                aBuilder.MakeFace(aFace, aNative, 1e-7);
                string anExpected = SurfacesSection(NativeWrite(aFace));
                string anActual = PortWrite(aPort);
                Assert.Equal(anExpected, anActual);
                aCounter.Add("offset", anActual);
            }
        }

        aCounter.Report();
    }

    [Fact]
    public void Curve2ds_are_written_as_native_OCCT_writes_them()
    {
        var anOracle = new Oracle(output);
        var aCounter = new Counter(output, "2D curves");
        using var aBuilder = new N.BRep_Builder();
        using var aNOrigin = new N.gp_Pnt(0, 0, 0);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aPlane = new N.Geom_Plane(aNOrigin, aNZ);
        using var aLocation = new N.TopLoc_Location();
        foreach (int i in Tape.Budget(1000))
        {
            (N.Geom2d_Curve aNative, Geom2d_Curve aPort, string aKind) = TestAdaptorOracleTests.RandomCurve2d(anOracle);
            if (anOracle.Int(0, 2) == 0)
            {
                // a trimmed curve of it
                double aF = aPort.FirstParameter(), aL = aPort.LastParameter();
                if (Precision.IsInfinite(aF) || Precision.IsInfinite(aL) || aPort.IsPeriodic())
                {
                    aF = anOracle.Real(-7, 7);
                    aL = aF + anOracle.Real(0.5, 6);
                }

                double aU1 = anOracle.Real(aF, aF + 0.45 * (aL - aF)), aU2 = anOracle.Real(aF + 0.55 * (aL - aF), aL);
                N.Geom2d_Curve aNTrimmed = new N.Geom2d_TrimmedCurve(aNative, aU1, aU2);
                aNative.Dispose();
                aNative = aNTrimmed;
                aPort = new Geom2d_TrimmedCurve(aPort, aU1, aU2);
                aKind = "trimmed " + aKind;
            }

            using (aNative)
            {
                using var anEdge = new N.TopoDS_Edge();
                aBuilder.MakeEdge(anEdge);
                aBuilder.UpdateEdge(anEdge, aNative, aPlane, aLocation, 1e-7);
                string anExpected = Curve2dsSection(NativeWrite(anEdge));
                string anActual = PortWrite(aPort);
                Assert.Equal(anExpected, anActual);
                aCounter.Add(aKind.Split(' ')[0] == "trimmed" ? "trimmed" : aKind.Split(' ')[0], anActual);

                var aSet = new GeomTools_Curve2dSet();
                aSet.Read(new StringReader(anExpected));
                var aWriter = new StringWriter();
                aSet.Write(aWriter);
                Assert.Equal(Curve2dsSection(NativeRewrite()), aWriter.ToString());
            }
        }

        aCounter.Report();
    }

    private static string CorpusDirectory
    {
        get
        {
            string aSource = typeof(GeomToolsOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                          .Single(a => a.Key == "OcctSourceDir").Value!;
            string aDirectory = Path.Combine(aSource, "data", "occ");
            Assert.True(Directory.Exists(aDirectory), $"No OCCT source at {aSource}: run tools/get-occt.ps1");
            return aDirectory;
        }
    }

    /// <summary>
    /// For each .brep file of OCCT's test data: native OCCT reads it and writes it (F1), then reads F1 and writes it
    /// again (F2). The port reads F1's geometry and writes it: the text must be F2's, byte for byte.
    /// </summary>
    [Fact]
    public void Corpus_files_read_and_written_as_native_OCCT_does()
    {
        int aFiles = 0, anObjects = 0;
        long aBytes = 0;
        foreach (string aFile in Tape.Budget(Directory.EnumerateFiles(CorpusDirectory, "*.brep").Order(StringComparer.Ordinal), isSpread: true, isRollingBack: true))
        {
            string aF1 = NativeReadWrite(aFile);
            string aF2 = NativeReadWrite(Path.Combine(myDirectory, $"s{myFileNumber - 1}.brep"));

            var aCurves2d = new GeomTools_Curve2dSet();
            aCurves2d.Read(new StringReader(Curve2dsSection(aF1)));
            var aWriter = new StringWriter();
            aCurves2d.Write(aWriter);
            Assert.Equal(Curve2dsSection(aF2), aWriter.ToString());
            aBytes += aWriter.ToString().Length;

            var aCurves = new GeomTools_CurveSet();
            aCurves.Read(new StringReader(CurvesSection(aF1)));
            aWriter = new StringWriter();
            aCurves.Write(aWriter);
            Assert.Equal(CurvesSection(aF2), aWriter.ToString());
            aBytes += aWriter.ToString().Length;

            var aSurfaces = new GeomTools_SurfaceSet();
            aSurfaces.Read(new StringReader(SurfacesSection(aF1)));
            aWriter = new StringWriter();
            aSurfaces.Write(aWriter);
            Assert.Equal(SurfacesSection(aF2), aWriter.ToString());
            aBytes += aWriter.ToString().Length;

            aFiles++;
            anObjects += Count(aF2, "Curve2ds") + Count(aF2, "Curves") + Count(aF2, "Surfaces");
        }

        output.WriteLine($"{aFiles} files, {anObjects} curves and surfaces, {aBytes} bytes, all byte-identical");
    }

    private static int Count(string theText, string theHeader)
    {
        string aSection = Section(theText, theHeader, theHeader == "Curve2ds" ? "Curves" : theHeader == "Curves" ? "Polygon3D" : "Triangulations");
        return int.Parse(aSection[(theHeader.Length + 1)..aSection.IndexOf('\n', StringComparison.Ordinal)],
                         System.Globalization.CultureInfo.InvariantCulture);
    }

    /// <summary>Native OCCT reads the last file written and writes it again.</summary>
    private string NativeRewrite() => NativeReadWrite(Path.Combine(myDirectory, $"s{myFileNumber - 1}.brep"));

    private string NativeReadWrite(string theFile)
    {
        using var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return NativeWrite(aShape);
    }
}
