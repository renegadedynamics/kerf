using System.Collections.Concurrent;
using static Kerf.Modeling.Operations;

namespace Build123d.Tests.Immutability;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// Parts built on many threads at once are exactly the parts built one at a time (unit 10I): the BRep text of each,
/// and of its mesh, is the serial run's. The parts are Bento's guide's examples, as the kernel's Bento benchmark builds
/// them.
/// </summary>
public class ConcurrentPartsTests
{
    private const double Length = 80.0, Width = 60.0, Thickness = 10.0;

    private static readonly (string Name, Func<Shape> Build)[] Parts =
    [
        ("plate with hole", () => new Box(Length, Width, Thickness) - new Cylinder(22.0 / 2, Thickness)),
        ("prismatic solid", () => Extrude(new Circle(Width) - new Rectangle(Length / 2, Width / 2), 2 * Thickness).Value),
        ("working point", () => Extrude(new Circle(90) - new Pos(45, 0.0) * new Rectangle(15, 15) - new Pos(0.0, 45) * new Circle(7.5), 15).Value),
        ("chamfer, fillet", () =>
        {
            Shape a1 = new Box(Length, Width, Thickness);
            Shape a4 = Chamfer(a1.SelectEdges().GroupBy(Axis.Z)[^1], length: 4).Value;
            return Fillet(a4.SelectEdges().ParallelTo(Axis.Z), radius: 5).Value;
        }),
        ("revolution", () => Revolve(new Pos(0, 50) * new Circle(6), Axis.X, revolutionArc: 180).Value),
    ];

    private static string Text(Shape shape)
    {
        using var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportBrep(shape, stream).IsSuccess);
        ShapeMesh mesh = shape.Triangulate();
        return System.Text.Encoding.UTF8.GetString(stream.ToArray()) + $"\n{mesh.TriangleCount} triangles, volume {shape.Volume:R}";
    }

    [Fact]
    public void PartsBuiltInParallelAreTheSerialParts()
    {
        string[] serial = [.. Parts.Select(part => Text(part.Build()))];
        var differences = new ConcurrentBag<string>();
        int runs = Math.Max(4, Environment.ProcessorCount / 2) * Parts.Length;
        Parallel.For(0, runs, k =>
        {
            int i = k % Parts.Length;
            if (Text(Parts[i].Build()) != serial[i])
            {
                differences.Add(Parts[i].Name);
            }
        });
        Assert.True(differences.IsEmpty, string.Join(", ", differences.Distinct()));
    }
}
