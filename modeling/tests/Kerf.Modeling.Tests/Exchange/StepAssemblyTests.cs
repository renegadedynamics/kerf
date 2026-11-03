using System.Text;
using System.Text.RegularExpressions;
using O = global::Occt;

namespace Build123d.Tests.Exchange;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// STEP's names, colours and assemblies on the OCCT.NET backend (OCCT.NET's STEPCAFControl over its XdeDoc tree).
/// What <see cref="Exporters3D.ExportStep(Shape, Stream, Unit, bool, PrecisionMode, string?, CancellationToken)"/>
/// writes is compared with OCCT.NET's STEPCAFControl_Writer (compared with native XCAF by OCCT.NET's unit 8G2) over a
/// document built with XCAF's calls, as build123d's <c>_create_xde</c> builds its XCAF document, but for one thing: a
/// node without assembly children is a part even when its shape is a compound (build123d's Part is one), where
/// <c>AddShape(shape, makeAssembly: true)</c> of the whole would make it an assembly of its solid. Reading back checks
/// the round trip.
/// </summary>
public class StepAssemblyTests
{
    private const string Timestamp = "2026-11-02T12:00:00";

    private static readonly string Folder = Path.Combine(AppContext.BaseDirectory, "Reference", "Exchange");

    // As StepNativeTests masks them, and the sign of a zero: the reference's shapes come through BRep text, which
    // writes a placement's -0 as 0
    private static string Masked(byte[] step)
    {
        var text = Regex.Replace(Encoding.Latin1.GetString(step), @"(Open CASCADE STEP translator [0-9.]+) \d+", "$1 #");
        text = Regex.Replace(text, @"-0\.(?=[,)])", "0.");
        return Regex.Replace(text, @"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('\d+'", "NEXT_ASSEMBLY_USAGE_OCCURRENCE('#'");
    }

    private static byte[] Export(Shape shape)
    {
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStep(shape, stream, timestamp: Timestamp));
        return stream.ToArray();
    }

    // The shape as OCCT.NET holds it, from the same BRep text (shared sub-shapes stay shared)
    private static global::Kerf.Topology.Shape Occt(Shape shape)
    {
        var text = Encoding.ASCII.GetString(Extras.WriteBRep(shape.Wrapped));
        var reader = new StringReader(text);
        global::Kerf.Engine.Foundation.StreamUtilities.SetInputLength(reader, text.Length);
        global::Kerf.Engine.Topology.Tools.BrepTools.Read(out var read, reader);
        return read;
    }

    private static global::Kerf.Engine.Foundation.ColorWithAlpha Linear(Color color)
    {
        var rgba = color.ToQuantityColorRgba();
        var rgb = rgba.GetRGB();
        return new global::Kerf.Engine.Foundation.ColorWithAlpha((float)rgb.Red(), (float)rgb.Green(), (float)rgb.Blue(), (float)rgba.Alpha());
    }

    /// <summary>
    /// What OCCT.NET's STEPCAFControl writes of a document built with XCAF's calls: a part added with <c>AddShape</c>
    /// (not as an assembly, so parts of one shape share their label) and placed with <c>AddComponent</c>, an assembly
    /// made with <c>NewShape</c>; each node's label and colour set on its shape's label.
    /// </summary>
    private static byte[] AsXcafWrites(Shape shape)
    {
        var doc = new global::Kerf.Engine.Exchange.Documents.ExchangeDocument();   // AutoNaming off, as build123d sets it
        doc.SetLengthUnit(new global::Kerf.Engine.Exchange.Documents.DocumentLengthUnit("MM", 0.001));
        void Attributes(Shape node, global::Kerf.Engine.Exchange.Documents.Label label)
        {
            if (node.Label.Length > 0)
                doc.SetName(label, node.Label);
            if (node.Color is { } c)
                doc.SetColor(label, Linear(c), node.Faces().Count > 0 ? global::Kerf.Engine.Exchange.Documents.ColorType.ColorSurf : global::Kerf.Engine.Exchange.Documents.ColorType.ColorCurv);
        }
        global::Kerf.Engine.Exchange.Documents.Label Assembly(Compound assembly, global::Kerf.Topology.Shape s)
        {
            var label = doc.NewShape();
            Attributes(assembly, label);
            var it = new global::Kerf.Engine.Topology.Iterator(s);
            foreach (var child in assembly.AssemblyChildren)
            {
                var c = it.Value();
                if (child is Compound { AssemblyChildren.Count: > 0 } sub)
                    doc.AddComponent(label, Assembly(sub, c.Located(new global::Kerf.Topology.Placement())), c.Placement);
                else
                {
                    doc.AddComponent(label, c, false);
                    Attributes(child, doc.FindShape(c, false));
                }
                it.Next();
            }
            return label;
        }
        var occt = Occt(shape);
        if (shape is Compound { AssemblyChildren.Count: > 0 } root)
            Assembly(root, occt);
        else
        {
            doc.AddShape(occt, false);
            Attributes(shape, doc.FindShape(occt, false));
        }
        doc.UpdateAssemblies();
        var writer = new global::Kerf.Engine.Exchange.Step.Caf.CafWriter();
        Assert.True(writer.Transfer(doc));
        var text = new MemoryStream();
        Assert.Equal(global::Kerf.Engine.Exchange.Framework.Selection.ReturnStatus.RetDone, writer.WriteStream(text));
        return StepHeader.WithFileName(text.ToArray(), shape.Label.Length > 0 ? shape.Label : null, Timestamp, "build123d");
    }

    /// <summary>A plate and a sub-assembly of the same bolt placed twice, the sub-assembly itself placed.</summary>
    private static Compound Fixture()
    {
        var bolt = new Cylinder(1, 10).WithLabel("bolt").WithColor(new Color("silver"));
        var plate = new Box(20, 20, 2).WithLabel("plate").WithColor(new Color("steelblue", 0.5));
        var bolts = Compound.Assembly([bolt.Moved(new Location((5.0, 5.0, 0.0))), bolt.Moved(new Location((-5.0, 5.0, 0.0)))], "bolts");
        return Compound.Assembly([plate, bolts.Moved(new Location((0.0, 0.0, 6.0), (0.0, 0.0, 90.0)))], "fixture");
    }

    public static TheoryData<string> Names() => StepNativeTests.Names();

    [Theory]
    [MemberData(nameof(Names))]
    public void ShapesAreWrittenAsStepCafControlWritesThem(string name)
    {
        var shape = Importers.ImportBrep(Path.Combine(Folder, name + ".brep"));
        Assert.Equal(Masked(AsXcafWrites(shape)), Masked(Export(shape)));
    }

    [Fact]
    public void AnAssemblyIsWrittenAsXcafWritesIt()
    {
        var fixture = Fixture();
        var written = Export(fixture);
        Assert.Equal(Masked(AsXcafWrites(fixture)), Masked(written));
        var text = Encoding.Latin1.GetString(written);
        // The bolt is one product placed twice; four components in all
        Assert.Single(Regex.Matches(text, @"PRODUCT\('bolt'"));
        Assert.Equal(4, Regex.Matches(text, "NEXT_ASSEMBLY_USAGE_OCCURRENCE").Count);
        Assert.Contains("PRODUCT('fixture'", text);
        Assert.Contains("PRODUCT('bolts'", text);
    }

    [Fact]
    public void AnAssemblyRoundTripsWithItsLabelsColoursAndPlaces()
    {
        var fixture = Fixture();
        var read = Assert.IsType<Compound>(Importers.ImportStep(new MemoryStream(Export(fixture))));
        Assert.Equal("fixture", read.Label);
        Assert.Equal(["plate", "bolts"], read.AssemblyChildren.Select(c => c.Label));
        var plate = read.AssemblyChildren[0];
        Assert.Equal(new Color("steelblue", 0.5), plate.Color);
        var bolts = Assert.IsType<Compound>(read.AssemblyChildren[1]);
        Assert.Equal(["bolt", "bolt"], bolts.AssemblyChildren.Select(c => c.Label));
        Assert.All(bolts.AssemblyChildren, b => Assert.Equal(new Color("silver"), b.Color));
        // Each part where it was, and the bolts still one shape placed twice
        var expected = fixture.Parts();
        var parts = read.Parts();
        Assert.Equal(expected.Count, parts.Count);
        for (int i = 0; i < parts.Count; i++)
        {
            Check.AlmostEqual(expected[i].Placed.Volume, parts[i].Placed.Volume, 6);
            var (a, b) = (expected[i].Placed.Center(), parts[i].Placed.Center());
            Check.AlmostEqual(a.X, b.X, 6);
            Check.AlmostEqual(a.Y, b.Y, 6);
            Check.AlmostEqual(a.Z, b.Z, 6);
        }
        Assert.True(bolts.AssemblyChildren[0].Wrapped.IsPartner(bolts.AssemblyChildren[1].Wrapped));
        Assert.True(read.IsValid);
    }

    [Fact]
    public void OneNamedShapeComesBackAsItself()
    {
        var read = Importers.ImportStep(new MemoryStream(Export(new Box(1, 2, 3).WithLabel("brick").WithColor(new Color("red")))));
        var solid = Assert.IsType<Solid>(read);
        Assert.Equal("brick", solid.Label);
        Assert.Equal(new Color("red"), solid.Color);
        Check.AlmostEqual(6, solid.Volume, 6);
    }

    [Fact]
    public void OneShapeUnderTwoNamesIsTwoProducts()
    {
        // STEP's writer maps shapes to products, so the second name needs a shape of its own
        var ball = new Sphere(2);
        var pair = Compound.Assembly([ball.WithLabel("left"), ball.Moved(new Location((5.0, 0.0, 0.0))).WithLabel("right").WithColor(new Color("blue"))]);
        var read = Assert.IsType<Compound>(Importers.ImportStep(new MemoryStream(Export(pair))));
        Assert.Equal(["left", "right"], read.AssemblyChildren.Select(c => c.Label));
        Assert.Null(read.AssemblyChildren[0].Color);
        Assert.Equal(new Color("blue"), read.AssemblyChildren[1].Color);
        Check.AlmostEqual(5, read.AssemblyChildren[1].Center().X, 6);
    }

    [Fact]
    public void ACurveIsColouredAsACurve()
    {
        var line = new Polyline((0.0, 0.0), (10.0, 0.0), (10.0, 5.0)).WithLabel("path").WithColor(new Color("green"));
        var read = Importers.ImportStep(new MemoryStream(Export(line)));
        Assert.Equal("path", read.Label);
        Assert.Equal(new Color("green"), read.Color);
    }

    [Fact]
    public void AnAssemblyIsWrittenInOtherUnits()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Fixture(), stream, Unit.Inch);
        Assert.Contains("INCH", Encoding.Latin1.GetString(stream.ToArray()));
        stream.Position = 0;
        var read = Assert.IsType<Compound>(Importers.ImportStep(stream));
        Check.AlmostEqual(Fixture().Parts()[0].Placed.Volume * Math.Pow(25.4, 3), read.Parts()[0].Placed.Volume, 1);
    }

    [Fact]
    public void ExternalFilesAreReadBesideAFileButNeverFromAStream()
    {
        var folder = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}");
        Directory.CreateDirectory(folder);
        try
        {
            // A two-part assembly written as three files: the assembly, referring to a file per part
            var doc = global::Kerf.Engine.Exchange.Documents.ExchangeDocument.FromTree(new global::Kerf.Engine.Exchange.Documents.Tree(global::Kerf.Engine.Exchange.Documents.DocumentProduct.Assembly("pair",
                global::Kerf.Engine.Exchange.Documents.DocumentProduct.Part(Occt(new Box(1, 1, 1)), "a").Placed(new global::Kerf.Topology.Placement()),
                global::Kerf.Engine.Exchange.Documents.DocumentProduct.Part(Occt(new Sphere(1)), "b").Placed(new global::Kerf.Topology.Placement()))
                .Placed(new global::Kerf.Topology.Placement())));
            var writer = new global::Kerf.Engine.Exchange.Step.Caf.CafWriter();
            Assert.True(writer.Transfer(doc, global::Kerf.Engine.Exchange.Step.ModelType.AsIs, ""));
            var main = Path.Combine(folder, "pair.step");
            Assert.Equal(global::Kerf.Engine.Exchange.Framework.Selection.ReturnStatus.RetDone, writer.Write(main));
            Assert.True(Directory.GetFiles(folder).Length > 1);

            var read = Assert.IsType<Compound>(Importers.ImportStep(main));
            Assert.Equal(2, read.Parts().Count);
            using var stream = File.OpenRead(main);
            var e = Assert.Throws<InvalidDataException>(() => Importers.ImportStep(stream));
            Assert.Contains("other STEP file", e.Message);
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }
}
