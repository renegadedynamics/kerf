using System.Text;
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Errors;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// Broken data and degenerate geometry give failures or exceptions, never a crash or a hang: a test host that survives these
/// tests is the assertion that matters.
/// </summary>
public class MalformedInputTests
{
    private static byte[] Brep(Shape shape, bool withTriangles = false) => Extras.WriteBRep(shape, withTriangles);

    [Fact]
    public void ValidBRepStillReads()
    {
        var part = Ops.Fillet(new Box(4, 4, 4).Edges(), 1).Value - new Cylinder(1, 10);
        foreach (var withTriangles in new[] { false, true })
        {
            var read = (Extras.ReadBRep(Brep(part, withTriangles)).Value);
            Assert.Equal(part.Volume, read.Volume, 6);
            Assert.Equal(part.Faces().Count, read.SelectFaces().Count);
        }
    }

    [Fact]
    public void TruncatedBRepIsAnError()
    {
        // The reader loops for ever at the end of truncated data unless stopped
        var data = Brep(new Box(1, 2, 3), withTriangles: true);
        for (int length = 0; length < data.Length - 1; length += 13)
        {
            // Most often the end is found as such; cut in the right place, a record is malformed instead
            var read = Extras.ReadBRep(data.AsSpan(0, length));
            Assert.Equal("exchange.malformed", read.Error!.Code.Value);
        }
    }

    [Theory]
    [InlineData("")]
    [InlineData("hello")]
    [InlineData("CASCADE Topology V3, (c) Open Cascade\nLocations 99999999\n")]
    public void NonBRepIsAnError(string text) =>
        Assert.Equal("exchange.malformed", Extras.ReadBRep(Encoding.ASCII.GetBytes(text)).Error!.Code.Value);

    [Fact]
    public void AnOverlongWordIsRefusedBeforeOcctReadsIt()
    {
        // OCCT reads words into fixed buffers without a limit
        var data = Encoding.ASCII.GetBytes("CASCADE Topology V3, (c) Open Cascade\nLocations " + new string('9', 100_000));
        var read = Extras.ReadBRep(data);
        Assert.Contains("too long", read.Error!.Message);
        Assert.Equal("exchange.malformed", Extras.ReadBRep("Locations\01"u8).Error!.Code.Value);
    }

    [Fact]
    public void MutatedBRepReadsOrFailsCleanly()
    {
        var sources = new Shape[] { new Box(1, 2, 3), Ops.Fillet(new Box(4, 4, 4).Edges(), 1).Value - new Cylinder(1, 10), new Sphere(2) };
        var texts = sources.SelectMany(s => new[] { Brep(s), Brep(s, withTriangles: true) })
                           .Select(Encoding.ASCII.GetString).ToList();
        int read = 0, refused = 0;
        for (int seed = 0; seed < 300; seed++)
        {
            var data = Mutate(texts[seed % texts.Count], new Random(seed));
            var result = Extras.ReadBRep(data);
            if (result.IsFailure)
            {
                refused++;
                continue;
            }
            var shape = (result.Value);
            read++;
            // A shape read from broken data may itself be broken: asking about it fails cleanly or not at all
            try
            {
                _ = shape.IsValid;
                _ = shape.BoundBox();
                _ = shape.Volume;
                _ = shape.SelectFaces().Sum(f => f.Area);
            }
            catch (Exception e) when (Failures.IsKernelFailure(e) || e is ArgumentException or InvalidOperationException)
            {
            }
        }
        Assert.True(read > 30 && refused > 30, $"{read} read, {refused} refused");
    }

    /// <summary>A few random edits: digits changed, characters replaced, runs deleted, numbers inserted.</summary>
    private static byte[] Mutate(string text, Random rng)
    {
        var sb = new StringBuilder(text);
        for (int m = 0; m < 1 + rng.Next(4); m++)
        {
            int at = rng.Next(sb.Length);
            switch (rng.Next(4))
            {
                case 0: if (char.IsDigit(sb[at])) sb[at] = (char)('0' + rng.Next(10)); break;
                case 1: sb[at] = "0123456789-.e +"[rng.Next(15)]; break;
                case 2: sb.Remove(at, Math.Min(rng.Next(1, 20), sb.Length - at)); break;
                case 3: sb.Insert(at, rng.Next(1000).ToString()); break;
            }
        }
        return Encoding.ASCII.GetBytes(sb.ToString());
    }

    public static TheoryData<string> DegenerateCases => [.. Degenerate.Keys];

    private static readonly Dictionary<string, Func<object>> Degenerate = new()
    {
        ["box of zero size"] = () => new Box(0, 1, 1),
        ["box of negative size"] = () => new Box(-1, 1, 1),
        ["cylinder of zero radius"] = () => new Cylinder(0, 1).Volume,
        ["sphere of zero radius"] = () => new Sphere(0).Volume,
        ["line between one point"] = () => Edge.MakeLine(new Vector(0, 0, 0), new Vector(0, 0, 0)),
        ["rectangle of zero size"] = () => Face.MakeRect(0, 0).Area,
        ["extrude by nothing"] = () => Ops.Extrude(Face.MakeRect(1, 1), 0),
        ["loft of two equal sections"] = () => Solid.MakeLoft([Wire.MakeRect(1, 1), Wire.MakeRect(1, 1)]),
        ["loft of one section"] = () => Solid.MakeLoft([Wire.MakeRect(1, 1)]),
        ["revolve about an axis through the face"] = () => Solid.Revolve(Face.MakeRect(1, 1), 360, Axis.X),
        ["revolve by nothing"] = () => Solid.Revolve(Face.MakeRect(1, 1).Moved(new Location(new Vector(5.0, 0.0, 0.0))), 0, Axis.Y),
        ["fillet too large"] = () => Ops.Fillet(new Box(1, 1, 1).Edges(), 5),
        ["chamfer too large"] = () => Ops.Chamfer(new Box(1, 1, 1).Edges(), 5),
        ["offset through itself"] = () => Ops.Offset(new Box(1, 1, 1), -2),
        ["thicken by nothing"] = () => Solid.Thicken(Face.MakeRect(1, 1), 0),
        ["sweep along a point"] = () => Solid.Sweep(Wire.MakeCircle(1), Edge.MakeLine(new Vector(0, 0, 0), new Vector(0, 0, 1e-9))),
        ["cut by itself"] = () => (new Box(1, 1, 1) - new Box(1, 1, 1)).Volume,
        ["fuse with a huge fuzzy value"] = () => new Box(1, 1, 1).Fuse([new Box(1, 1, 1).Moved(new Location(new Vector(0.5, 0.0, 0.0)))], tolerance: 10),
        ["triangulate a point"] = () => Vertex.Make(1, 2, 3).Triangulate(),
        ["triangulate with zero deflection"] = () => new Box(1, 1, 1).Triangulate(0),
    };

    [Theory]
    [MemberData(nameof(DegenerateCases))]
    public void DegenerateGeometryFailsCleanly(string name)
    {
        try
        {
            _ = Degenerate[name]();
        }
        catch (Exception e) when (Failures.IsKernelFailure(e) || e is ArgumentException or InvalidOperationException)
        {
            Assert.False(string.IsNullOrWhiteSpace(e.Message), $"{name}: an exception without a message");
        }
    }
}
