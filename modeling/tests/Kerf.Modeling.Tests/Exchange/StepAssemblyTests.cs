using System.Text;
using System.Text.RegularExpressions;
using O = global::Kerf.Engine;

namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>
/// STEP's names, colours and assemblies on the Kerf backend (Kerf's STEPCAFControl over its XdeDoc tree).
/// What <see cref="Exporters3D.ExportStep(Shape, Stream, StepExportOptions?, CancellationToken)"/>
/// writes is compared with Kerf's STEPCAFControl_Writer (compared with native XCAF by Kerf's unit 8G2) over a
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

    private static byte[] Export(IShape shape)
    {
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStep(shape, stream, new StepExportOptions { Timestamp = Timestamp }).IsSuccess);
        return stream.ToArray();
    }

    // The shape as Kerf holds it, from the same BRep text (shared sub-shapes stay shared)
    private static Shape Occt(Shape shape)
    {
        var text = Encoding.ASCII.GetString(Extras.WriteBRep(shape));
        var reader = new StringReader(text);
        global::Kerf.Engine.Foundation.StreamUtilities.SetInputLength(reader, text.Length);
        global::Kerf.Engine.Topology.Tools.BrepTools.Read(out var read, reader);
        return read;
    }

    private static global::Kerf.Engine.Foundation.ColorWithAlpha Linear(Color color)
    {
        var (r, g, b, a) = color.ToLinear();
        return new global::Kerf.Engine.Foundation.ColorWithAlpha((float)r, (float)g, (float)b, (float)a);
    }

    /// <summary>
    /// What Kerf's STEPCAFControl writes of a document built with XCAF's calls: a part added with <c>AddShape</c>
    /// (not as an assembly, so parts of one shape share their label) and placed with <c>AddComponent</c>, an assembly
    /// made with <c>NewShape</c>; each node's label and colour set on its shape's label.
    /// </summary>
    private static byte[] AsXcafWrites(ModelShape shape)
    {
        var doc = new global::Kerf.Engine.Exchange.Documents.ExchangeDocument();   // AutoNaming off, as build123d sets it
        doc.SetLengthUnit(new global::Kerf.Engine.Exchange.Documents.DocumentLengthUnit("MM", 0.001));
        void Attributes(ModelShape node, global::Kerf.Engine.Exchange.Documents.Label label)
        {
            if (node.Label.Length > 0)
                doc.SetName(label, node.Label);
            if (node.Color is { } c)
                doc.SetColor(label, Linear(c), node.Faces().Count > 0 ? global::Kerf.Engine.Exchange.Documents.ColorType.ColorSurf : global::Kerf.Engine.Exchange.Documents.ColorType.ColorCurv);
        }
        global::Kerf.Engine.Exchange.Documents.Label AssemblyLabel(Assembly assembly, Shape s)
        {
            var label = doc.NewShape();
            Attributes(assembly, label);
            var it = new global::Kerf.Engine.Topology.Iterator(s);
            foreach (var child in assembly.Children)
            {
                var c = it.Value();
                if (child is Assembly { Children.Count: > 0 } sub)
                    doc.AddComponent(label, AssemblyLabel(sub, c.Located(new Placement())), c.Placement);
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
        if (shape is Assembly { Children.Count: > 0 } root)
            AssemblyLabel(root, occt);
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
    private static Assembly Fixture()
    {
        var bolt = new Cylinder(1, 10).WithLabel("bolt").WithColor(Color.Parse("silver"));
        var plate = new Box(20, 20, 2).WithLabel("plate").WithColor(Color.Parse("steelblue", 0.5));
        var bolts = new Assembly([bolt.Moved(new Location(new Point3D(5.0, 5.0, 0.0))), bolt.Moved(new Location(new Point3D(-5.0, 5.0, 0.0)))], "bolts");
        return new Assembly([plate, bolts.Moved(new Location(new Point3D(0.0, 0.0, 6.0), new Vector3D(0.0, 0.0, 90.0)))], "fixture");
    }

    public static TheoryData<string> Names() => StepNativeTests.Names();

    [Theory]
    [MemberData(nameof(Names))]
    public void ShapesAreWrittenAsStepCafControlWritesThem(string name)
    {
        var shape = Importers.ImportBrep(Path.Combine(Folder, name + ".brep")).Value;
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
        var read = Assert.IsType<Assembly>(Importers.ImportStep(new MemoryStream(Export(fixture))).Value);
        Assert.Equal("fixture", read.Label);
        Assert.Equal(["plate", "bolts"], read.Children.Select(c => c.Label));
        var plate = read.Children[0];
        Assert.Equal(Color.Parse("steelblue", 0.5), plate.Color);
        var bolts = Assert.IsType<Assembly>(read.Children[1]);
        Assert.Equal(["bolt", "bolt"], bolts.Children.Select(c => c.Label));
        Assert.All(bolts.Children, b => Assert.Equal(Color.Parse("silver"), b.Color));
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
        Assert.True(bolts.Children[0].Shape.IsPartner(bolts.Children[1].Shape));
        Assert.True(read.IsValid);
    }

    [Fact]
    public void OneNamedShapeComesBackAsItself()
    {
        var read = Importers.ImportStep(new MemoryStream(Export(new Box(1, 2, 3).WithLabel("brick").WithColor(Color.Parse("red"))))).Value;
        var solid = read;
        Check.Is<Solid>(read);
        Assert.Equal("brick", solid.Label);
        Assert.Equal(Color.Parse("red"), solid.Color);
        Check.AlmostEqual(6, solid.Volume, 6);
    }

    [Fact]
    public void OneShapeUnderTwoNamesIsTwoProducts()
    {
        // STEP's writer maps shapes to products, so the second name needs a shape of its own
        var ball = new Sphere(2);
        var pair = new Assembly([ball.WithLabel("left"), ball.Moved(new Location(new Point3D(5.0, 0.0, 0.0))).WithLabel("right").WithColor(Color.Parse("blue"))]);
        var read = Assert.IsType<Assembly>(Importers.ImportStep(new MemoryStream(Export(pair))).Value);
        Assert.Equal(["left", "right"], read.Children.Select(c => c.Label));
        Assert.Null(read.Children[0].Color);
        Assert.Equal(Color.Parse("blue"), read.Children[1].Color);
        Check.AlmostEqual(5, read.Children[1].Center().X, 6);
    }

    [Fact]
    public void ACurveIsColouredAsACurve()
    {
        var line = new Polyline(new Point3D(0.0, 0.0, 0), new Point3D(10.0, 0.0, 0), new Point3D(10.0, 5.0, 0)).WithLabel("path").WithColor(Color.Parse("green"));
        var read = Importers.ImportStep(new MemoryStream(Export(line))).Value;
        Assert.Equal("path", read.Label);
        Assert.Equal(Color.Parse("green"), read.Color);
    }

    [Fact]
    public void AnAssemblyIsWrittenInOtherUnits()
    {
        var stream = new MemoryStream();
        Exporters3D.ExportStep(Fixture(), stream, new StepExportOptions { Unit = Unit.Inch });
        Assert.Contains("INCH", Encoding.Latin1.GetString(stream.ToArray()));
        stream.Position = 0;
        var read = Assert.IsType<Assembly>(Importers.ImportStep(stream).Value);
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
                global::Kerf.Engine.Exchange.Documents.DocumentProduct.Part(Occt(new Box(1, 1, 1)), "a").Placed(new Placement()),
                global::Kerf.Engine.Exchange.Documents.DocumentProduct.Part(Occt(new Sphere(1)), "b").Placed(new Placement()))
                .Placed(new Placement())));
            var writer = new global::Kerf.Engine.Exchange.Step.Caf.CafWriter();
            Assert.True(writer.Transfer(doc, global::Kerf.Engine.Exchange.Step.ModelType.AsIs, ""));
            var main = Path.Combine(folder, "pair.step");
            Assert.Equal(global::Kerf.Engine.Exchange.Framework.Selection.ReturnStatus.RetDone, writer.Write(main));
            Assert.True(Directory.GetFiles(folder).Length > 1);

            var read = Assert.IsType<Assembly>(Importers.ImportStep(main).Value);
            Assert.Equal(2, read.Parts().Count);
            using var stream = File.OpenRead(main);
            var refused = Importers.ImportStep(stream);
            Assert.Equal("exchange.external-references", refused.Error!.Code.Value);
            Assert.Contains("other STEP file", refused.Error.Message);
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }
}
