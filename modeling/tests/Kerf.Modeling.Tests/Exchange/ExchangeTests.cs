using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;
using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>Import and export: BREP, STEP, STL, glTF and 3MF.</summary>
public partial class ExchangeTests
{
    private static Part Block() => (Part)(new Box(10, 10, 10) - new Cylinder(3, 14));

    // The extent of a mesh read from STL or glTF, from its triangles
    private static (Point3D Min, Point3D Max) Extent(Shape mesh)
    {
        var vertices = mesh.Tessellate(1).Vertices;
        return (new Point3D(vertices.Min(v => v.X), vertices.Min(v => v.Y), vertices.Min(v => v.Z)),
                new Point3D(vertices.Max(v => v.X), vertices.Max(v => v.Y), vertices.Max(v => v.Z)));
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
        Assert.Equal(block.Faces().Count, read.Faces().Count);

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
        var two = Compound.Make([new Box(1, 1, 1), new Box(1, 1, 1).Moved(new Location(new Point3D(5.0, 0.0, 0.0)))]);
        var stream = new MemoryStream();
        Exporters3D.ExportStep(two, stream);
        stream.Position = 0;
        var read = Importers.ImportStep(stream).Value;
        Check.Is<Compound>(read);
        Assert.Equal(2, read.Solids().Count);
        Check.AlmostEqual(2.0, read.Solids().Sum(s => s.Volume), 9);
    }

    [Fact]
    public void StepHeaderNamesKerf()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block().WithLabel("bracket 'A' – v2"), stream, new StepExportOptions { Timestamp = "2026-11-02T12:00:00" });
        var text = Encoding.Latin1.GetString(stream.ToArray());
        var fileName = Regex.Match(text, @"FILE_NAME\((.*?)\);", RegexOptions.Singleline).Groups[1].Value;
        Assert.StartsWith(@"'bracket ''A'' \X2\2013\X0\ v2','2026-11-02T12:00:00',", fileName);
        Assert.Contains("'Kerf.Modeling'", fileName);
        Assert.Matches(@"'Kerf STEP processor \d+\.\d+\.\d+'", fileName);
        Assert.Contains("'Kerf is derived from Open CASCADE Technology 8.0'", text);
        Assert.DoesNotContain("Open CASCADE STEP", text);
        Assert.DoesNotContain("'Open CASCADE'", text);
    }

    // FILE_NAME rewritten as a string, as before unit 14A: the reference for the rewrite that writes the rest as it is
    private static byte[] RewrittenAsText(byte[] step, string? name, string? timestamp, string? system)
    {
        var text = Encoding.Latin1.GetString(step);
        int start = text.IndexOf("FILE_NAME(", StringComparison.Ordinal), data = text.IndexOf("DATA;", StringComparison.Ordinal);
        if (start < 0 || (data >= 0 && start > data))
            return step;
        var spans = new List<(int From, int To)>();
        int depth = 0, argument = start + "FILE_NAME(".Length;
        bool quoted = false, closed = false;
        for (int i = argument; i < text.Length && !closed; i++)
        {
            char c = text[i];
            if (quoted)
            {
                if (c == '\'')
                {
                    if (i + 1 < text.Length && text[i + 1] == '\'')
                        i++;
                    else
                        quoted = false;
                }
                continue;
            }
            if (c == '\'')
                quoted = true;
            else if (c == '(')
                depth++;
            else if (c == ')' && depth > 0)
                depth--;
            else if (c is ',' or ')' && depth == 0)
            {
                int from = argument, to = i;
                while (from < to && char.IsWhiteSpace(text[from]))
                    from++;
                while (to > from && char.IsWhiteSpace(text[to - 1]))
                    to--;
                spans.Add((from, to));
                closed = c == ')';
                argument = i + 1;
            }
        }
        if (!closed || spans.Count != 7)
            return step;
        foreach (var (index, value) in new (int, string?)[] { (5, system), (1, timestamp), (0, name) })
        {
            if (value is not null)
                text = string.Concat(text.AsSpan(0, spans[index].From), StepHeader.Encode(value), text.AsSpan(spans[index].To));
        }
        return Encoding.Latin1.GetBytes(text);
    }

    [Theory]
    [InlineData("ISO-10303-21;\nHEADER;\nFILE_NAME('a','b',('c'),('d'),'e','f','g');\nENDSEC;\nDATA;\n#1=X('é');\nENDSEC;\n", "part 'one'", "2026", "Kerf")]
    [InlineData("ISO-10303-21;\nHEADER;\nFILE_NAME( 'a' ,\n 'b''s' , ( 'c' , 'd' ),('d'),'e(' , 'f'  ,'g' ) ;\nDATA;\n", "x", null, "y ")]
    [InlineData("ISO-10303-21;\nHEADER;\nFILE_NAME('a','b');\nDATA;\n", "x", "t", "s")]
    [InlineData("ISO-10303-21;\nDATA;\nFILE_NAME('a','b',('c'),('d'),'e','f','g');\n", "x", "t", "s")]
    [InlineData("ISO-10303-21;\nHEADER;\nENDSEC;\n", "x", "t", "s")]
    [InlineData("ISO-10303-21;\nHEADER;\nFILE_NAME(\u0085'a' ,'b',('c'),('d'),'e','f','g'\u0085);\nDATA;\n", null, "t", null)]
    public void StepHeaderRewritesOnlyItsRecord(string step, string? name, string? timestamp, string? system)
    {
        var bytes = Encoding.Latin1.GetBytes(step);
        Assert.Equal(RewrittenAsText(bytes, name, timestamp, system), StepHeader.WithFileName(bytes, name, timestamp, system));
        var stream = new MemoryStream();
        StepHeader.Write(stream, bytes, name, timestamp, system);
        Assert.Equal(RewrittenAsText(bytes, name, timestamp, system), stream.ToArray());
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
    public void StepThatRefersToAnEntityItDoesntHaveImportsWithAWarning()
    {
        // A box's STEP data with one vertex's point changed to an identifier the data doesn't have (the kernel's
        // decision 2308): the import said nothing, and gave whatever the kernel made of it
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStep(new Box(10, 10, 10), stream).IsSuccess);
        string step = Encoding.ASCII.GetString(stream.ToArray());
        Match vertex = Regex.Match(step, @"VERTEX_POINT\('',#(\d+)\)");
        Assert.True(vertex.Success);
        string broken = step.Replace(vertex.Value, "VERTEX_POINT('',#99999999)");

        var read = Importers.ImportStep(new MemoryStream(Encoding.ASCII.GetBytes(broken)));

        Assert.True(read.IsSuccess);
        var skipped = Assert.Single(read.Diagnostics);
        Assert.Equal("exchange.entity-skipped", skipped.Code.Value);
        // the solid holding the vertex is left out with it, and with it all the box: the warning says the model is empty
        // (the kernel's decision 2315)
        Assert.EndsWith("Nothing else was read: the model is empty.", skipped.Message);
        Assert.Empty(Importers.ImportStep(new MemoryStream(Encoding.ASCII.GetBytes(step))).Diagnostics);
    }

    [Fact]
    public void StepWhoseFacesReadInvalidImportsWithAWarning()
    {
        // OCCT's issue 349's strip, two lines between the same two vertices, read as the kernel read it before its
        // decision 2306 (a face of no area): the import passes on the read's warning (the kernel's decision 2314)
        string step = string.Join('\n',
            "ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION((''),'2;1');", "FILE_NAME('','',(''),(''),'','','');",
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 1 }'));", "ENDSEC;", "DATA;",
            "#1=CARTESIAN_POINT('',(0.,0.,0.));", "#2=CARTESIAN_POINT('',(1.,0.,0.));",
            "#3=CARTESIAN_POINT('',(0.,-0.0005,0.));", "#4=CARTESIAN_POINT('',(0.,0.0005,0.));",
            "#11=VERTEX_POINT('',#1);", "#12=VERTEX_POINT('',#2);",
            "#20=DIRECTION('',(0.,0.,1.));", "#21=DIRECTION('',(1.,0.,0.));", "#25=AXIS2_PLACEMENT_3D('',#1,#20,#21);",
            "#33=VECTOR('',#21,1.);", "#34=LINE('',#3,#33);", "#35=LINE('',#4,#33);", "#36=PLANE('',#25);",
            "#40=EDGE_CURVE('',#11,#12,#34,.T.);", "#41=EDGE_CURVE('',#11,#12,#35,.T.);",
            "#50=ORIENTED_EDGE('',*,*,#40,.T.);", "#51=ORIENTED_EDGE('',*,*,#41,.F.);",
            "#60=EDGE_LOOP('',(#50,#51));", "#62=FACE_OUTER_BOUND('',#60,.T.);",
            "#64=ADVANCED_FACE('',(#62),#36,.T.);", "#65=OPEN_SHELL('',(#64));", "#66=SHELL_BASED_SURFACE_MODEL('',(#65));",
            "#70=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));", "#71=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));",
            "#72=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());",
            "#73=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#70,'distance_accuracy_value','');",
            "#74=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#73))GLOBAL_UNIT_ASSIGNED_CONTEXT((#70,#71,#72))REPRESENTATION_CONTEXT('',''));",
            "#75=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#66),#74);",
            "#80=APPLICATION_CONTEXT('automotive design');", "#81=PRODUCT_CONTEXT('',#80,'mechanical');", "#82=PRODUCT('strip','strip','',(#81));",
            "#83=PRODUCT_DEFINITION_FORMATION('','',#82);", "#84=PRODUCT_DEFINITION_CONTEXT('part definition',#80,'design');",
            "#85=PRODUCT_DEFINITION('','',#83,#84);", "#86=PRODUCT_DEFINITION_SHAPE('','',#85);", "#87=SHAPE_DEFINITION_REPRESENTATION(#86,#75);",
            "ENDSEC;", "END-ISO-10303-21;", "");

        Kerf.Result<ModelShape> read;
        using (Kerf.Engine.Healing.Fixes.UndoneFixes.Scope())
        {
            read = Importers.ImportStep(new MemoryStream(Encoding.ASCII.GetBytes(step)));
        }

        Assert.True(read.IsSuccess);
        var invalid = Assert.Single(read.Diagnostics);
        Assert.Equal(Kerf.Exchange.Step.InvalidFace, invalid.Code);
        Assert.Contains("#64", invalid.Message);
        Assert.Empty(Importers.ImportStep(new MemoryStream(Encoding.ASCII.GetBytes(step))).Diagnostics);
    }

    [Fact]
    public void StepPastTheByteLimitIsRefused()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block(), stream);
        stream.Position = 0;
        Assert.Equal("exchange.limit-exceeded", Importers.ImportStep(stream, new ImportLimits(MaxBytes: 1000)).Error!.Code.Value);
    }

    [Fact]
    public void StepPastTheRecordLimitIsRefused()
    {
        // Kerf's parser counts records; native OCCT reads from a temporary file, with no such limit (decision 1965: reported as the limit)
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Block(), stream);
        stream.Position = 0;
        Assert.Equal("exchange.limit-exceeded", Importers.ImportStep(stream, new ImportLimits(MaxEntities: 50)).Error!.Code.Value);
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
        var pair = new Assembly([new Box(1, 1, 1).WithLabel("a").WithColor(Color.Parse("red")),
                                      new Sphere(1).Moved(new Location(new Point3D(3.0, 0.0, 0.0))).WithLabel("b")], "pair");
        var stream = new MemoryStream();
        Exporters3D.ExportStep(pair, stream);
        stream.Position = 0;
        var read = Importers.ImportStep(stream).Value;
        Check.AlmostEqual(pair.Volume, read.Volume, 6);
        Assert.Equal("pair", read.Label);
        Assert.Equal(["a", "b"], ((Assembly)read).Children.Select(c => c.Label));
        Assert.Equal(Color.Parse("red"), ((Assembly)read).Children[0].Color);
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
        var box = Importers.ImportStl(stream).Value.Bounds();
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
        var assembly = new Assembly(
        [
            new Box(10, 10, 10).WithLabel("box").WithColor(Color.Parse("red")),
            new Sphere(4).Moved(new Location(new Point3D(20.0, 0.0, 0.0))).WithLabel("ball").WithColor(Color.Rgb(0.0, 0.5, 1.0)),
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
            var read = Assert.IsType<Assembly>(Importers.ImportGltf(stream).Value);
            Assert.Equal("pair", read.Label);
            Assert.Equal(["box", "ball"], read.Children.Select(c => c.Label));
            Assert.Equal(Color.Parse("red"), read.Children[0].Color);
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
        Assert.True(Exporters3D.Export3mf(new Assembly([Block(), new Sphere(3).WithColor(Color.Parse("blue"))]), stream).IsSuccess);
        stream.Position = 0;
        var zip = new ZipArchive(stream, ZipArchiveMode.Read);
        using var model = new StreamReader(zip.GetEntry("3D/3dmodel.model")!.Open());
        var xml = model.ReadToEnd();
        Assert.Contains("unit=\"millimeter\"", xml);
        Assert.Contains("<basematerials", xml);
        Assert.Equal(2, Regex.Matches(xml, "<mesh>").Count);
    }
}
