using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Exchange;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>Import and export: BREP, STEP, STL, glTF and 3MF.</summary>
public partial class ExchangeTests
{
    private static Part Block() => (Part)(new Box(10, 10, 10) - new Cylinder(3, 14));

    // The extent of a mesh read from STL or glTF, from its triangles
    private static (Vector Min, Vector Max) Extent(Shape mesh)
    {
        var vertices = mesh.Tessellate(1).Vertices;
        return (new Vector(vertices.Min(v => v.X), vertices.Min(v => v.Y), vertices.Min(v => v.Z)),
                new Vector(vertices.Max(v => v.X), vertices.Max(v => v.Y), vertices.Max(v => v.Z)));
    }

    // ---- BREP ----

    [Fact]
    public void BrepRoundTripsThroughStreamsAndFiles()
    {
        var block = Block();
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportBrep(block, stream).IsSuccess);
        stream.Position = 0;
        var read = Importers.ImportBrep(stream).Value;
        Check.Is<Compound>(read);   // a Part's shape is a compound
        Check.AlmostEqual(block.Volume, read.Volume, 9);

        var path = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}.brep");
        try
        {
            Assert.True(Exporters3D.ExportBrep(block, path).IsSuccess);
            Check.AlmostEqual(block.Volume, Importers.ImportBrep(path).Value.Volume, 9);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void NothingToExportSaysSo()
    {
        var error = Exporters3D.ExportStep(default(Compound), new MemoryStream()).Error!;
        Assert.Equal("exchange.nothing-done", error.Code.Value);
        Assert.Equal("There is nothing to export: the Compound is empty", error.Message);
        Assert.Equal("exchange.nothing-done", Exporters3D.ExportBrep(default(Compound), new MemoryStream()).Error!.Code.Value);
    }

    [Fact]
    public void AFileThatCantBeWrittenIsAFailedResult()
    {
        var folder = Path.GetTempPath();   // a folder can't be opened as a file
        Assert.Equal("exchange.write-failed", Exporters3D.ExportStep(Block(), folder.TrimEnd(Path.DirectorySeparatorChar)).Error!.Code.Value);
    }

    [Fact]
    public void MissingFilesAndFoldersSaySo()
    {
        var missing = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}");
        Assert.Throws<FileNotFoundException>(() => Importers.ImportStep(missing + ".step"));
        Assert.Throws<FileNotFoundException>(() => Importers.ImportBrep(missing + ".brep"));
        Assert.Throws<DirectoryNotFoundException>(() => Exporters3D.ExportStep(Block(), Path.Combine(missing, "a.step")));
    }

    // ---- STEP ----

    [Fact]
    public void StepRoundTripsThroughStreamsAndFiles()
    {
        var block = Block();
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStep(block, stream).IsSuccess);
        stream.Position = 0;
        var read = Importers.ImportStep(stream).Value;
        Check.Is<Solid>(read);
        Check.AlmostEqual(block.Volume, read.Volume, 6);
        Assert.Equal(block.Faces().Count, read.SelectFaces().Count);

        var path = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}.step");
        try
        {
            Assert.True(Exporters3D.ExportStep(block, path).IsSuccess);
            Check.AlmostEqual(block.Volume, Importers.ImportStep(path).Value.Volume, 6);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void StepOfSeveralShapesIsACompound()
    {
        var two = Compound.Make([new Box(1, 1, 1), new Box(1, 1, 1).Moved(new Location(new Vector(5.0, 0.0, 0.0)))]);
        var stream = new MemoryStream();
        Exporters3D.ExportStep(two, stream);
        stream.Position = 0;
        var read = Importers.ImportStep(stream).Value;
        Check.Is<Compound>(read);
        Assert.Equal(2, read.SelectSolids().Count);
        Check.AlmostEqual(2.0, read.SelectSolids().Sum(s => s.Volume), 9);
    }

    [Fact]
    public void StepHeaderNamesBuild123d()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block().WithLabel("bracket 'A' – v2"), stream, new StepExportOptions { Timestamp = "2026-11-02T12:00:00" });
        var text = Encoding.Latin1.GetString(stream.ToArray());
        var fileName = Regex.Match(text, @"FILE_NAME\((.*?)\);", RegexOptions.Singleline).Groups[1].Value;
        Assert.StartsWith(@"'bracket ''A'' \X2\2013\X0\ v2','2026-11-02T12:00:00',", fileName);
        Assert.Contains("'build123d'", fileName);
        // what OCCT wrote of the other arguments stays
        Assert.Contains("'Open CASCADE STEP processor", fileName);
    }

    [Fact]
    public void StepHeaderEncodesAsPart21()
    {
        Assert.Equal("'it''s'", StepHeader.Encode("it's"));
        Assert.Equal(@"'a\\b'", StepHeader.Encode(@"a\b"));
        Assert.Equal(@"'\X2\00E9\X0\'", StepHeader.Encode("é"));
        Assert.Equal(@"'\X4\0001F600\X0\'", StepHeader.Encode("😀"));
    }

    [Fact]
    public void BrokenStepDataIsRefused()
    {
        var read = Importers.ImportStep(new MemoryStream("ISO-10303-21;\nHEADER;\ngarbage"u8.ToArray()));
        Assert.StartsWith("Could not import STEP data", read.Error!.Message);
        Assert.Equal("exchange.failed", read.Error.Code.Value);
        Assert.Equal("exchange.failed", Importers.ImportStep(new MemoryStream([])).Error!.Code.Value);
    }

    [Fact]
    public void StepPastTheByteLimitIsRefused()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block(), stream);
        stream.Position = 0;
        Assert.Equal("exchange.failed", Importers.ImportStep(stream, new ImportLimits(MaxBytes: 1000)).Error!.Code.Value);
    }

    [Fact]
    public void StepPastTheRecordLimitIsRefused()
    {
        // OCCT.NET's parser counts records; native OCCT reads from a temporary file, with no such limit
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block(), stream);
        stream.Position = 0;
        Assert.Equal("exchange.failed", Importers.ImportStep(stream, new ImportLimits(MaxEntities: 50)).Error!.Code.Value);
    }

    [Fact]
    public void StepWritesOtherUnitsAndModes()
    {
        var box = new Box(10, 20, 30);
        var inches = new MemoryStream();
        Exporters3D.ExportStep(box, inches, new StepExportOptions { Unit = Unit.Inch, WritePcurves = false, PrecisionMode = PrecisionMode.Greatest });
        var text = Encoding.Latin1.GetString(inches.ToArray());
        Assert.Contains("'INCH'", text);
        Assert.DoesNotContain("PCURVE", text);
        inches.Position = 0;
        // The shapes were in inches and are written as such: read back in millimetres, they are 25.4 times larger
        Check.AlmostEqual(10 * 20 * 30 * Math.Pow(25.4, 3), Importers.ImportStep(inches).Value.Volume, 3);
    }

    [Fact]
    public void CancelledImportsAndExportsStop()
    {
        var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block(), stream);
        stream.Position = 0;
        Assert.ThrowsAny<OperationCanceledException>(() => Importers.ImportStep(stream, cancellationToken: cancelled.Token));
        Assert.ThrowsAny<OperationCanceledException>(() => Exporters3D.ExportStep(Block(), new MemoryStream(), cancellationToken: cancelled.Token));
    }

    // ---- STL, glTF and 3MF ----

    [Fact]
    public void StepKeepsLabelsColoursAndAssembliesOnOcctNetOnly()
    {
        var pair = Compound.Assembly([new Box(1, 1, 1).WithLabel("a").WithColor(new Color("red")),
                                      new Sphere(1).Moved(new Location(new Vector(3.0, 0.0, 0.0))).WithLabel("b")], "pair");
        var stream = new MemoryStream();
        Exporters3D.ExportStep(pair, stream);
        stream.Position = 0;
        var read = Importers.ImportStep(stream).Value;
        Check.AlmostEqual(pair.Volume, read.Volume, 6);
        Assert.Equal("pair", read.Label);
        Assert.Equal(["a", "b"], ((Compound)read).AssemblyChildren.Select(c => c.Label));
        Assert.Equal(new Color("red"), ((Compound)read).AssemblyChildren[0].Color);
    }

    [Fact]
    public void StlRoundTripsBinaryAndAscii()
    {
        var box = new Box(10, 20, 30);
        foreach (var ascii in new[] { false, true })
        {
            var stream = new MemoryStream();
            Assert.True(Exporters3D.ExportStl(box, stream, new StlExportOptions { Ascii = ascii }).IsSuccess);
            if (ascii)
                Assert.StartsWith("solid", Encoding.ASCII.GetString(stream.ToArray()));
            else
                Assert.Equal(84 + 12 * 50, stream.Length);
            stream.Position = 0;
            var face = Importers.ImportStl(stream).Value;
            var (vertices, triangles) = face.Tessellate(1);
            Assert.Equal(12, triangles.Count);
            Assert.Equal(8, vertices.Count);   // nodes merged
            var (min, max) = Extent(face);
            Check.AlmostEqual(10, max.X - min.X, 6);
            Check.AlmostEqual(30, max.Z - min.Z, 6);
        }
    }

    [Fact]
    public void AMeshHasABoundingBox()
    {
        // Faces holding only triangles have no surface to bound: the box is the triangles'
        var stream = new MemoryStream();
        Exporters3D.ExportStl(new Box(10, 20, 30), stream);
        stream.Position = 0;
        var box = Importers.ImportStl(stream).Value.BoundingBox();
        Assert.False(box.IsEmpty);
        Check.AlmostEqual(20, box.Size.Y, 6);
        Check.AlmostEqual(-15, box.Min.Z, 6);
    }

    [Fact]
    public void StlScalesItsUnit()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStl(new Box(1, 2, 3), stream);
        stream.Position = 0;
        var face = Importers.ImportStl(stream, Unit.Meter).Value;
        var (min, max) = Extent(face);
        Check.AlmostEqual(3000, max.Z - min.Z, 3);
        Assert.Throws<ArgumentException>(() => Importers.ImportStl(new MemoryStream(), Unit.Gram));
    }

    [Fact]
    public void StlLimitsRefuseHostileData()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStl(new Sphere(5), stream);
        stream.Position = 0;
        Assert.Equal("exchange.malformed", Importers.ImportStl(stream, limits: new ImportLimits(MaxTriangles: 10)).Error!.Code.Value);
        Assert.Equal("exchange.malformed", Importers.ImportStl(new MemoryStream("solid x\nfacet garbage"u8.ToArray())).Error!.Code.Value);
    }

    [Fact]
    public void GltfRoundTripsAnAssemblyWithLabelsAndColours()
    {
        var assembly = Compound.Assembly(
        [
            new Box(10, 10, 10).WithLabel("box").WithColor(new Color("red")),
            new Sphere(4).Moved(new Location(new Vector(20.0, 0.0, 0.0))).WithLabel("ball").WithColor(new Color(0.0, 0.5, 1.0)),
        ], "pair");
        foreach (var binary in new[] { true, false })
        {
            var stream = new MemoryStream();
            Assert.True(Exporters3D.ExportGltf(assembly, stream, new GltfExportOptions { Binary = binary }).IsSuccess);
            if (binary)
                Assert.Equal("glTF"u8.ToArray(), stream.ToArray()[..4]);
            else
                Assert.Contains("data:application/octet-stream;base64,", Encoding.UTF8.GetString(stream.ToArray()));
            stream.Position = 0;
            var read = Check.Is<Compound>(Importers.ImportGltf(stream).Value);
            Assert.Equal("pair", read.Label);
            Assert.Equal(["box", "ball"], read.AssemblyChildren.Select(c => c.Label));
            Assert.Equal(new Color("red"), read.AssemblyChildren[0].Color);
            var (min, max) = Extent(read);
            Check.AlmostEqual(-5, min.X, 4);
            Check.AlmostEqual(24, max.X, 2);
            Check.AlmostEqual(5, max.Z, 4);   // Z-up again
        }
    }

    [Fact]
    public void GltfFilesKeepTheirBufferBeside()
    {
        var folder = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}");
        Directory.CreateDirectory(folder);
        try
        {
            var path = Path.Combine(folder, "part.gltf");
            Assert.True(Exporters3D.ExportGltf(Block(), path).IsSuccess);
            Assert.True(File.Exists(Path.Combine(folder, "part.bin")));
            Assert.Contains("\"part.bin\"", File.ReadAllText(path));
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }

    [Fact]
    public void ThreeMfHoldsAModel()
    {
        var stream = new MemoryStream();
        Assert.True(Exporters3D.Export3mf(Compound.Assembly([Block(), new Sphere(3).WithColor(new Color("blue"))]), stream).IsSuccess);
        stream.Position = 0;
        var zip = new ZipArchive(stream, ZipArchiveMode.Read);
        using var model = new StreamReader(zip.GetEntry("3D/3dmodel.model")!.Open());
        var xml = model.ReadToEnd();
        Assert.Contains("unit=\"millimeter\"", xml);
        Assert.Contains("<basematerials", xml);
        Assert.Equal(2, Regex.Matches(xml, "<mesh>").Count);
    }
}
