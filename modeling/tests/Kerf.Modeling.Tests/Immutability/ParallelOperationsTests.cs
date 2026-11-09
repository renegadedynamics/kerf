using System.Collections.Concurrent;
using System.Security.Cryptography;
using System.Text;
using Kerf.Drawing;
using Kerf.Meshing;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Immutability;

using Kerf.Modeling;

/// <summary>
/// The guide's "Running operations in parallel" (unit 14G): the common operations it teaches, run on many threads at
/// once, give exactly what they give one at a time: each part's BREP text, volume, mesh, exported files (STEP, STL,
/// glTF, 3MF, DXF, SVG, PNG) and the model read back, as a hash. Independent parts each on a thread, and one part shared
/// by every thread, which none of them may change. <c>KERF_STRESS_THREADS</c> sets the threads (by default the
/// processors, up to 8).
/// </summary>
public class ParallelOperationsTests
{
    private static int Threads =>
        int.TryParse(Environment.GetEnvironmentVariable("KERF_STRESS_THREADS"), out int n) && n > 0 ? n : Math.Clamp(Environment.ProcessorCount, 2, 8);

    private static readonly StepExportOptions Step = new() { Timestamp = "2026-11-09T12:00:00" };

    // A part built with the guide's operations, its size set by k
    private static Part Build(int k)
    {
        double size = 40 + 3 * k;
        Part block = new Box(size, 30, 8);
        Part rounded = Fillet(block, block.Edges().GroupBy(Axis.Z).Last(), 1.5).Value;
        Part chamfered = Chamfer(rounded, rounded.Edges().GroupBy(Axis.Z).First(), 1).Value;
        Part plate = chamfered - new GridLocations(size / 2, 15, 2, 2) * new Cylinder(2 + 0.1 * k, 8);
        Face bottom = plate.Faces().OrderBy(Axis.Z).First();
        Part shelled = Offset(plate, -1, new OffsetOptions { Openings = [bottom] }).Value;

        Sketch outline = new Rectangle(20, 10) - new Circle(2 + 0.05 * k);
        Part bar = new Pos(0, 0, 8) * Extrude(outline, 5).Value;
        Sketch profile = new Location(Plane.XZ) * new Rectangle(3, 4, align: new Alignment(Align.Min, Align.Min));
        Part ring = new Pos(size, 0, 0) * Revolve(profile.Moved(new Location(new Vector3D(6, 0, 0))), Axis.Z).Value;
        Part loft = new Pos(-size, 0, 0) * Loft([new Rectangle(8, 8), new Location(new Vector3D(0, 0, 12 + k)) * new Circle(3)]).Value;
        Curve path = new Spline([new Point3D(0, 0, 0), new Point3D(0, 3, 10), new Point3D(0, 0, 20 + k)], [new Vector3D(0, 0, 1), new Vector3D(0, 0, 1)]);
        Part swept = new Pos(0, -size, 0) * Sweep(new Circle(1.5), path).Value;
        Part hull = new Pos(0, size, 0) * new ConvexPolyhedron([new(0, 0, 0), new(6 + k, 0, 0), new(0, 6, 0), new(0, 0, 6), new(4, 4, 4)]);
        Sketch label = new Text($"K{k}", 5, font: "Liberation Sans");
        Part embossed = Extrude(new Pos(0, 0, 13) * label, 1).Value;
        return shelled + bar + ring + loft + swept + hull + embossed;
    }

    private static string Hash(byte[] bytes) => Convert.ToHexString(SHA256.HashData(bytes))[..16];

    private static string Hash(string text) => Hash(Encoding.UTF8.GetBytes(text));

    private static byte[] Written(Func<Stream, Result> write)
    {
        var stream = new MemoryStream();
        var written = write(stream);
        Assert.True(written.IsSuccess, written.Error?.Message);
        return stream.ToArray();
    }

    // What a part gives: its BREP and volume, its mesh, its files and the model read back from its STEP
    private static string Fingerprint(Part part)
    {
        var text = new StringBuilder();
        byte[] brep = Written(s => Exporters3D.ExportBrep(part, s));
        text.Append("brep ").Append(Hash(brep)).Append(" volume ").Append(part.Volume.ToString("R")).Append('\n');
        var mesh = part.Shape.Mesh(new MeshOptions { LinearDeflection = 0.05, AngularDeflection = 0.3 }).Value;
        text.Append("mesh ").Append(Hash(MemoryMarshalBytes(mesh.Positions.ToArray()))).Append(' ').Append(mesh.TriangleCount).Append('\n');
        byte[] step = Written(s => Exporters3D.ExportStep(part, s, Step));
        text.Append("step ").Append(Hash(step)).Append('\n');
        text.Append("stl ").Append(Hash(Written(s => Exporters3D.ExportStl(part, s)))).Append('\n');
        text.Append("glb ").Append(Hash(Written(s => Exporters3D.ExportGltf(part, s, new GltfExportOptions { Binary = true })))).Append('\n');
        text.Append("3mf ").Append(Hash(Written(s => Exporters3D.Export3mf(part, s)))).Append('\n');
        LineDrawing drawing = part.Shape.ProjectLines(Camera.Isometric, new DrawingOptions { Method = HiddenLineMethod.Polygonal }).Value;
        text.Append("svg ").Append(Hash(Svg.WriteToString(drawing))).Append('\n');
        text.Append("dxf ").Append(Hash(new ExportDxf().AddDrawing(drawing).WriteToString())).Append('\n');
        RgbaImage image = Renderer.Render(new Scene(new SceneItem(mesh)), Camera.Isometric, new RenderOptions { Width = 96, Height = 64 }).Value;
        text.Append("png ").Append(Hash(Png.Encode(image, new PngOptions { Compression = PngCompression.Deterministic }))).Append('\n');
        var read = Importers.ImportStep(new MemoryStream(step)).Value;
        text.Append("read ").Append(Hash(Written(s => Exporters3D.ExportBrep(read, s)))).Append(' ').Append(read.Volume.ToString("R")).Append('\n');
        return text.ToString();
    }

    private static byte[] MemoryMarshalBytes(float[] values)
    {
        var bytes = new byte[values.Length * sizeof(float)];
        Buffer.BlockCopy(values, 0, bytes, 0, bytes.Length);
        return bytes;
    }

    [Fact]
    public void IndependentPartsBuiltOnManyThreadsAreThePartsBuiltOneAtATime()
    {
        const int variants = 6;
        string[] serial = [.. Enumerable.Range(0, variants).Select(k => Fingerprint(Build(k)))];
        var differences = new ConcurrentBag<string>();
        Parallel.For(0, 3 * variants, new ParallelOptions { MaxDegreeOfParallelism = Threads }, j =>
        {
            int k = j % variants;
            string parallel = Fingerprint(Build(k));
            if (parallel != serial[k])
                differences.Add($"part {k}:\n{serial[k]}---\n{parallel}");
        });
        Assert.True(differences.IsEmpty, string.Join("\n", differences));
    }

    [Fact]
    public void OnePartSharedByManyThreadsIsUnchangedAndGivesTheSameEverywhere()
    {
        Part shared = Build(2);
        byte[] before = Written(s => Exporters3D.ExportBrep(shared, s));
        // Operations on the shared part, each thread's result as it is serially
        Func<Part, string>[] uses =
        [
            p => Fingerprint(p),
            p => Hash(Written(s => Exporters3D.ExportBrep(Fillet(p, p.Edges().GroupBy(Axis.Z).Last(), 0.3).Value, s))),
            p => Hash(Written(s => Exporters3D.ExportBrep(p - new Pos(5, 5, 0) * new Cylinder(1, 40), s))),
            p => Hash(Written(s => Exporters3D.ExportBrep(p.Moved(new Location(new Vector3D(1, 2, 3))), s))),
            p => p.Faces().Count + " " + p.Edges().Count + " " + p.Area.ToString("R") + " " + p.Center().ToString(),
        ];
        string[] serial = [.. uses.Select(use => use(shared))];
        var differences = new ConcurrentBag<string>();
        Parallel.For(0, 3 * uses.Length, new ParallelOptions { MaxDegreeOfParallelism = Threads }, j =>
        {
            int u = j % uses.Length;
            if (uses[u](shared) != serial[u])
                differences.Add($"use {u}");
        });
        Assert.True(differences.IsEmpty, string.Join(", ", differences));
        Assert.Equal(before, Written(s => Exporters3D.ExportBrep(shared, s)));
    }

    [Fact]
    public async Task TheGuidesExampleRuns()
    {
        string folder = Path.Combine(Path.GetTempPath(), $"kerf-14g-{Guid.NewGuid():N}");
        Directory.CreateDirectory(folder);
        try
        {
            int[] sizes = [40, 50, 60, 70];
            Parallel.ForEach(sizes, new ParallelOptions { MaxDegreeOfParallelism = Threads }, size =>
            {
                Part plate = new Box(size, 30, 8) - new GridLocations(size / 2.0, 15, 2, 2) * new Cylinder(3, 8);
                Part part = Fillet(plate, plate.Edges().GroupBy(Axis.Z).Last(), 1.5).Value;
                Exporters3D.ExportStep(part, Path.Combine(folder, $"plate-{size}.step"));
            });
            Result[] written = await Task.WhenAll(sizes.Select(async size =>
            {
                Part part = await Task.Run(() => (Part)(new Box(size, 30, 8) - new Cylinder(3, 8)));
                return await Exporters3D.ExportStepAsync(part, Path.Combine(folder, $"block-{size}.step"));
            }));
            Assert.All(written, r => Assert.True(r.IsSuccess));
            foreach (int size in sizes)
            {
                Check.AlmostEqual(size * 30 * 8 - Math.PI * 9 * 8, Importers.ImportStep(Path.Combine(folder, $"block-{size}.step")).Value.Volume, 6);
                Assert.True(Importers.ImportStep(Path.Combine(folder, $"plate-{size}.step")).IsSuccess);
            }
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }

    [Fact]
    public async Task PartsBuiltAndWrittenAsTasksAreThePartsBuiltOneAtATime()
    {
        // The guide's Task.WhenAll example: each part built on the thread pool, then written asynchronously
        const int variants = 4;
        byte[][] serial = [.. Enumerable.Range(0, variants).Select(k => Written(s => Exporters3D.ExportStep(Build(k), s, Step)))];
        using var throttle = new SemaphoreSlim(Threads);
        var streams = new MemoryStream[variants];
        var results = await Task.WhenAll(Enumerable.Range(0, variants).Select(async k =>
        {
            await throttle.WaitAsync();
            try
            {
                Part part = await Task.Run(() => Build(k));
                streams[k] = new MemoryStream();
                return await Exporters3D.ExportStepAsync(part, streams[k], Step);
            }
            finally
            {
                throttle.Release();
            }
        }));
        Assert.All(results, r => Assert.True(r.IsSuccess, r.Error?.Message));
        for (int k = 0; k < variants; k++)
            Assert.Equal(serial[k], streams[k].ToArray());
    }
}
