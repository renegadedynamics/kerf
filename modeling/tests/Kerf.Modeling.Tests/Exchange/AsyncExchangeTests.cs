namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>
/// The importers' and exporters' async forms (unit 14G): the same results and bytes as the synchronous forms, through
/// streams that refuse synchronous I/O and through files; a missing file or folder throws before the task.
/// </summary>
public class AsyncExchangeTests
{
    private static Part Block() => (Part)(new Box(10, 10, 10) - new Cylinder(3, 14));

    private static Assembly TwoParts() =>
        new([Block().WithLabel("block").WithColor(new Color(1, 0, 0)), new Pos(20, 0, 0) * new Sphere(4).WithLabel("ball")], "pair");

    private static byte[] Written(Func<Stream, Result> write)
    {
        var stream = new MemoryStream();
        var written = write(stream);
        Assert.True(written.IsSuccess, written.Error?.Message);
        return stream.ToArray();
    }

    private static byte[] BrepOf(Shape shape) => Written(s => Exporters3D.ExportBrep(shape, s));

    private static string TempPath(string extension) => Path.Combine(Path.GetTempPath(), $"kerf-14g-{Guid.NewGuid():N}{extension}");

    [Fact]
    public async Task EachExporterWritesTheSameBytesToAStreamAndAFile()
    {
        var model = TwoParts();
        var step = new StepExportOptions { Timestamp = "2026-11-09T12:00:00" };
        var cases = new (string Extension, Func<Stream, Result> Sync, Func<Stream, Task<Result>> Async, Func<string, Result> SyncFile, Func<string, Task<Result>> AsyncFile)[]
        {
            (".brep", s => Exporters3D.ExportBrep(model, s), s => Exporters3D.ExportBrepAsync(model, s), p => Exporters3D.ExportBrep(model, p), p => Exporters3D.ExportBrepAsync(model, p)),
            (".step", s => Exporters3D.ExportStep(model, s, step), s => Exporters3D.ExportStepAsync(model, s, step), p => Exporters3D.ExportStep(model, p, step), p => Exporters3D.ExportStepAsync(model, p, step)),
            (".stl", s => Exporters3D.ExportStl(model, s), s => Exporters3D.ExportStlAsync(model, s), p => Exporters3D.ExportStl(model, p), p => Exporters3D.ExportStlAsync(model, p)),
            (".glb", s => Exporters3D.ExportGltf(model, s, new GltfExportOptions { Binary = true }), s => Exporters3D.ExportGltfAsync(model, s, new GltfExportOptions { Binary = true }),
                     p => Exporters3D.ExportGltf(model, p, new GltfExportOptions { Binary = true }), p => Exporters3D.ExportGltfAsync(model, p, new GltfExportOptions { Binary = true })),
            (".3mf", s => Exporters3D.Export3mf(model, s), s => Exporters3D.Export3mfAsync(model, s), p => Exporters3D.Export3mf(model, p), p => Exporters3D.Export3mfAsync(model, p)),
        };
        foreach (var (extension, sync, async, syncFile, asyncFile) in cases)
        {
            byte[] expected = Written(sync);
            var sink = new AsyncOnlyStream();
            var written = await async(sink);
            Assert.True(written.IsSuccess, $"{extension}: {written.Error?.Message}");
            Assert.Equal(expected, sink.Written);
            Assert.True(sink.Flushed);

            string syncPath = TempPath(extension), asyncPath = TempPath(extension);
            try
            {
                Assert.True(syncFile(syncPath).IsSuccess);
                Assert.True((await asyncFile(asyncPath)).IsSuccess);
                Assert.Equal(File.ReadAllBytes(syncPath), File.ReadAllBytes(asyncPath));
            }
            finally
            {
                File.Delete(syncPath);
                File.Delete(asyncPath);
            }
        }
    }

    [Fact]
    public async Task GltfJsonFilesKeepTheirBufferBesideAsSynchronously()
    {
        var model = TwoParts();
        string folder = Path.Combine(Path.GetTempPath(), $"kerf-14g-{Guid.NewGuid():N}");
        Directory.CreateDirectory(folder);
        try
        {
            Assert.True(Exporters3D.ExportGltf(model, Path.Combine(folder, "sync.gltf")).IsSuccess);
            Assert.True((await Exporters3D.ExportGltfAsync(model, Path.Combine(folder, "async.gltf"))).IsSuccess);
            Assert.Equal(File.ReadAllText(Path.Combine(folder, "sync.gltf")).Replace("sync.bin", "x.bin"),
                         File.ReadAllText(Path.Combine(folder, "async.gltf")).Replace("async.bin", "x.bin"));
            Assert.Equal(File.ReadAllBytes(Path.Combine(folder, "sync.bin")), File.ReadAllBytes(Path.Combine(folder, "async.bin")));
        }
        finally
        {
            Directory.Delete(folder, true);
        }
    }

    [Fact]
    public async Task EachImporterReadsTheSameFromAStreamAndAFile()
    {
        var model = TwoParts();
        byte[] brep = Written(s => Exporters3D.ExportBrep(model, s));
        byte[] step = Written(s => Exporters3D.ExportStep(model, s));
        byte[] stl = Written(s => Exporters3D.ExportStl(model, s));
        byte[] glb = Written(s => Exporters3D.ExportGltf(model, s, new GltfExportOptions { Binary = true }));

        Assert.Equal(BrepOf(Importers.ImportBrep(new MemoryStream(brep)).Value), BrepOf((await Importers.ImportBrepAsync(new AsyncOnlyStream(brep))).Value));
        AssertSameModel(Importers.ImportStep(new MemoryStream(step)).Value, (await Importers.ImportStepAsync(new AsyncOnlyStream(step))).Value);
        Assert.Equal(BrepOf(Importers.ImportStl(new MemoryStream(stl), Unit.Centimeter).Value),
                     BrepOf((await Importers.ImportStlAsync(new AsyncOnlyStream(stl), Unit.Centimeter)).Value));
        AssertSameModel(Importers.ImportGltf(new MemoryStream(glb)).Value, (await Importers.ImportGltfAsync(new AsyncOnlyStream(glb))).Value);

        foreach (var (extension, bytes) in new[] { (".brep", brep), (".step", step), (".stl", stl), (".glb", glb) })
        {
            string path = TempPath(extension);
            try
            {
                File.WriteAllBytes(path, bytes);
                switch (extension)
                {
                    case ".brep":
                        Assert.Equal(BrepOf(Importers.ImportBrep(path).Value), BrepOf((await Importers.ImportBrepAsync(path)).Value));
                        break;
                    case ".step":
                        AssertSameModel(Importers.ImportStep(path).Value, (await Importers.ImportStepAsync(path)).Value);
                        break;
                    case ".stl":
                        Assert.Equal(BrepOf(Importers.ImportStl(path).Value), BrepOf((await Importers.ImportStlAsync(path)).Value));
                        break;
                    default:
                        AssertSameModel(Importers.ImportGltf(path).Value, (await Importers.ImportGltfAsync(path)).Value);
                        break;
                }
            }
            finally
            {
                File.Delete(path);
            }
        }
    }

    [Fact]
    public async Task FailuresAreTheSynchronousFormsAndWriteNothing()
    {
        // Nothing to export
        var sink = new AsyncOnlyStream();
        var empty = await Exporters3D.ExportStepAsync(default(Compound), sink);
        Assert.Equal(Exporters3D.ExportStep(default(Compound), new MemoryStream()).Error!.Message, empty.Error!.Message);
        Assert.Empty(sink.Written);
        string path = TempPath(".step");
        Assert.True((await Exporters3D.ExportStepAsync(default(Compound), path)).IsFailure);
        Assert.False(File.Exists(path));

        // Past a limit, and not the format
        byte[] step = Written(s => Exporters3D.ExportStep(Block(), s));
        var small = new ImportLimits { MaxBytes = 100 };
        Assert.Equal(Importers.ImportStep(new MemoryStream(step), small).Error!.Message, (await Importers.ImportStepAsync(new AsyncOnlyStream(step), small)).Error!.Message);
        byte[] brep = Written(s => Exporters3D.ExportBrep(Block(), s));
        Assert.Equal(Importers.ImportBrep(new MemoryStream(brep), small).Error!.Message, (await Importers.ImportBrepAsync(new AsyncOnlyStream(brep), small)).Error!.Message);
        byte[] garbage = "not a shape"u8.ToArray();
        Assert.Equal(Importers.ImportStep(new MemoryStream(garbage)).Error!.Message, (await Importers.ImportStepAsync(new AsyncOnlyStream(garbage))).Error!.Message);

        // A folder can't be written as a file
        var folder = Path.GetTempPath();
        Assert.Equal(Exporters3D.ExportStl(Block(), folder).Error!.Code, (await Exporters3D.ExportStlAsync(Block(), folder)).Error!.Code);

        // Missing files and folders throw before the task, as the synchronous forms throw
        string missing = Path.Combine(Path.GetTempPath(), "kerf-14g-none.step");
        Assert.Throws<FileNotFoundException>(() => { _ = Importers.ImportStepAsync(missing); });
        Assert.Throws<FileNotFoundException>(() => { _ = Importers.ImportBrepAsync(missing); });
        Assert.Throws<FileNotFoundException>(() => { _ = Importers.ImportStlAsync(missing); });
        Assert.Throws<FileNotFoundException>(() => { _ = Importers.ImportGltfAsync(missing); });
        string noFolder = Path.Combine(Path.GetTempPath(), $"kerf-14g-{Guid.NewGuid():N}", "part.step");
        Assert.Throws<DirectoryNotFoundException>(() => { _ = Exporters3D.ExportStepAsync(Block(), noFolder); });
        Assert.Throws<DirectoryNotFoundException>(() => { _ = new ExportDxf().AddShape(new Rectangle(5, 5)).WriteAsync(noFolder); });
        Assert.Throws<ArgumentException>(() => { _ = Importers.ImportStlAsync(new MemoryStream(), Unit.Gram); });

        // Cancelled
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Exporters3D.ExportStlAsync(Block(), sink, cancellationToken: cancelled.Token));
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Importers.ImportStepAsync(new AsyncOnlyStream(step), cancellationToken: cancelled.Token));
        Assert.Empty(sink.Written);
    }

    [Fact]
    public async Task DxfIsWrittenTheSameWithItsWarnings()
    {
        var dxf = new ExportDxf().AddLayer("cut").AddShape(new Rectangle(50, 20) - new Circle(4), "cut")
                                 .AddShape(new Pos(0, 0, 5) * new Circle(2));   // off the XY plane: a warning
        var bytes = new MemoryStream();
        var expected = dxf.Write(bytes);
        var sink = new AsyncOnlyStream();
        var written = await dxf.WriteAsync(sink);
        Assert.True(written.IsSuccess);
        Assert.Equal(bytes.ToArray(), sink.Written);
        Assert.Equal(expected.Diagnostics.Select(d => d.Message), written.Diagnostics.Select(d => d.Message));
        Assert.NotEmpty(written.Diagnostics);

        string syncPath = TempPath(".dxf"), asyncPath = TempPath(".dxf");
        try
        {
            Assert.True(dxf.Write(syncPath).IsSuccess);
            Assert.True((await dxf.WriteAsync(asyncPath)).IsSuccess);
            Assert.Equal(File.ReadAllBytes(syncPath), File.ReadAllBytes(asyncPath));
        }
        finally
        {
            File.Delete(syncPath);
            File.Delete(asyncPath);
        }
    }

    // The same tree: labels, colours, locations and each leaf's shape as BREP
    private static void AssertSameModel(ModelShape a, ModelShape b)
    {
        Assert.Equal(a.GetType(), b.GetType());
        Assert.Equal(a.Label, b.Label);
        Assert.Equal(a.Color, b.Color);
        if (a is Assembly { Children.Count: > 0 } assembly)
        {
            var other = (Assembly)b;
            Assert.Equal(assembly.Children.Count, other.Children.Count);
            for (int i = 0; i < assembly.Children.Count; i++)
                AssertSameModel(assembly.Children[i], other.Children[i]);
        }
        Assert.Equal(BrepOf(a.Shape), BrepOf(b.Shape));
    }

    /// <summary>A stream that can be read and written only asynchronously, a few bytes at a time, yielding each time.</summary>
    private sealed class AsyncOnlyStream(byte[]? data = null) : Stream
    {
        private readonly byte[] _data = data ?? [];
        private readonly MemoryStream _written = new();
        private int _position;

        public byte[] Written => _written.ToArray();

        public bool Flushed { get; private set; }

        public override bool CanRead => true;

        public override bool CanSeek => false;

        public override bool CanWrite => true;

        public override long Length => throw new NotSupportedException();

        public override long Position { get => throw new NotSupportedException(); set => throw new NotSupportedException(); }

        public override int Read(byte[] buffer, int offset, int count) => throw new InvalidOperationException("A synchronous read");

        public override void Write(byte[] buffer, int offset, int count) => throw new InvalidOperationException("A synchronous write");

        public override void Flush() => throw new InvalidOperationException("A synchronous flush");

        public override async ValueTask<int> ReadAsync(Memory<byte> buffer, CancellationToken cancellationToken = default)
        {
            await Task.Yield();
            cancellationToken.ThrowIfCancellationRequested();
            int count = Math.Min(Math.Min(buffer.Length, 4093), _data.Length - _position);
            _data.AsSpan(_position, count).CopyTo(buffer.Span);
            _position += count;
            return count;
        }

        public override Task<int> ReadAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken)
            => ReadAsync(buffer.AsMemory(offset, count), cancellationToken).AsTask();

        public override async ValueTask WriteAsync(ReadOnlyMemory<byte> buffer, CancellationToken cancellationToken = default)
        {
            await Task.Yield();
            cancellationToken.ThrowIfCancellationRequested();
            _written.Write(buffer.Span);
        }

        public override Task WriteAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken)
            => WriteAsync(buffer.AsMemory(offset, count), cancellationToken).AsTask();

        public override async Task FlushAsync(CancellationToken cancellationToken)
        {
            await Task.Yield();
            Flushed = true;
        }

        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();

        public override void SetLength(long value) => throw new NotSupportedException();
    }
}
