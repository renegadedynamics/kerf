using System.Text;
using System.Text.RegularExpressions;
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Exchange;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// STEP as native OCCT writes and reads it (Reference/Exchange): each case is a shape built natively and kept as BRep
/// text (<c>name.brep</c>), the STEP native OCCT writes of it (<c>name.step</c>, with a fixed time stamp) and the BRep
/// text native OCCT reads back from that (<c>name.read.brep</c>). Both backends start from the same BRep, so what's
/// compared is the exchange alone. The native backend writes the files when <c>B3D_WRITE_EXCHANGE_REFERENCE</c> names
/// the folder (tests/Build123d.Tests/Reference/Exchange). Natively, STEP is written by STEPControl_Writer; on Kerf by
/// STEPCAFControl_Writer, which writes these shapes' text as STEPControl does but for the compound's.
/// </summary>
public partial class StepNativeTests
{
    private const string Timestamp = "2026-11-02T12:00:00";

    private static readonly string Folder = Path.Combine(AppContext.BaseDirectory, "Reference", "Exchange");

    /// <summary>The cases: solids of each surface type, a face, a wire and a compound.</summary>
    private static IEnumerable<(string Name, Func<Shape> Make)> Shapes()
    {
        yield return ("box", () => new Box(10, 20, 30));
        yield return ("block", () => new Box(10, 10, 10) - new Cylinder(3, 14));
        yield return ("sphere", () => new Sphere(5));
        yield return ("torus", () => new Torus(10, 2));
        yield return ("cone", () => new Cone(5, 2, 8));
        yield return ("fillet", () => Ops.Fillet(new Box(10, 10, 10).Edges(), 1).Value);
        yield return ("loft", () => Ops.Loft([new Circle(5), new Rectangle(6, 4).Moved(new Location(new Point3D(0.0, 0.0, 10.0)))]).Value);
        yield return ("face", () => Face.MakeRect(10, 6));
        yield return ("wire", () => new Polyline(new Point3D(0.0, 0.0, 0), new Point3D(10.0, 0.0, 0), new Point3D(10.0, 5.0, 0), new Point3D(3.0, 8.0, 0)));
        yield return ("compound", () => Compound.Make([new Box(1, 1, 1), new Sphere(1).Moved(new Location(new Point3D(4.0, 0.0, 0.0)))]));
    }

    public static TheoryData<string> Names()
    {
        var data = new TheoryData<string>();
        foreach (var (name, _) in Shapes())
            data.Add(name);
        return data;
    }

    // The product name's index and the assembly occurrences' identifiers count what a process has written natively
    // (OCCT's controller is process-wide, decisions 1264 and 1266 of Kerf); a fresh Kerf writer counts from 1
    private static string Masked(byte[] step)
    {
        var text = Regex.Replace(Encoding.Latin1.GetString(step), @"(Open CASCADE STEP translator [0-9.]+) \d+", "$1 #");
        return Regex.Replace(text, @"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('\d+'", "NEXT_ASSEMBLY_USAGE_OCCURRENCE('#'");
    }

    private static byte[] Write(Shape shape)
    {
        var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportStep(shape, stream, new StepExportOptions { Timestamp = Timestamp }).IsSuccess);
        return stream.ToArray();
    }

    [Theory]
    [MemberData(nameof(Names))]
    public void WritesAsNative(string name)
    {
        // The Kerf backend writes STEP through STEPCAFControl, as build123d does, and native's reference is
        // STEPControl's: they write the same text but for a compound, whose representations STEPCAFControl orders
        // differently (StepAssemblyTests compares it with Kerf's STEPCAFControl_Writer)
        if (name == "compound")
            return;
        var shape = Importers.ImportBrep(Path.Combine(Folder, name + ".brep")).Value;
        Assert.Equal(Masked(File.ReadAllBytes(Path.Combine(Folder, name + ".step"))), Masked(Write(shape)));
    }

    [Theory]
    [MemberData(nameof(Names))]
    public void ReadsAsNative(string name)
    {
        var read = Importers.ImportStep(Path.Combine(Folder, name + ".step")).Value;
        var expected = File.ReadAllText(Path.Combine(Folder, name + ".read.brep"));
        Assert.Equal(expected, Encoding.ASCII.GetString(Extras.WriteBRep(read)));
    }

}
