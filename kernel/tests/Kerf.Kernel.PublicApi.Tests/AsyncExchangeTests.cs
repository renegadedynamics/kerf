// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using System.Text;
using System.Text.RegularExpressions;
using Kerf.Drawing;
using Kerf.Exchange;
using Kerf.Fonts;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The readers' and writers' async forms (unit 14G): the same results and bytes as the synchronous forms, with the I/O
/// done asynchronously only (the streams here refuse synchronous reads and writes), limits, failures and cancellation
/// as the synchronous forms have them.
/// </summary>
public class AsyncExchangeTests
{
    // ------------------------------------------------------------------------------------------------- the readers

    [Fact]
    public async Task BREP_reads_the_same_shape_and_writes_the_same_bytes()
    {
        Solid box = Fixtures.Box();
        foreach (bool binary in new[] { false, true })
        {
            var options = new BrepWriteOptions { Binary = binary };
            byte[] sync = Written(s => Brep.Write(box, s, options));
            var sink = new AsyncOnlyStream();
            Assert.True((await Brep.WriteAsync(box, sink, options)).IsSuccess);
            Assert.Equal(sync, sink.Written);
            Assert.True(sink.Flushed);

            Shape read = Brep.Read(new MemoryStream(sync)).Value;
            Result<Shape> readAsync = await Brep.ReadAsync(new AsyncOnlyStream(sync));
            Assert.True(readAsync.IsSuccess, readAsync.Error?.Message);
            Assert.Equal(BrepText(read), BrepText(readAsync.Value));
        }
    }

    [Fact]
    public async Task STEP_reads_the_same_model_and_writes_the_same_text()
    {
        var orange = new Color(1, 0.5, 0);
        ModelNode model = ModelNode.Assembly([ModelNode.Part(Fixtures.Box(), "Body", orange), ModelNode.Part(Fixtures.Cylinder(), "Lid")], "Box");
        string sync = WithoutTime(Encoding.Latin1.GetString(Written(s => Step.Write(model, s))));
        var sink = new AsyncOnlyStream();
        Assert.True((await Step.WriteAsync(model, sink)).IsSuccess);
        Assert.Equal(sync, WithoutTime(Encoding.Latin1.GetString(sink.Written)));
        var shapeSink = new AsyncOnlyStream();
        Assert.True((await Step.WriteAsync(Fixtures.Box(), shapeSink)).IsSuccess);
        Assert.Equal(WithoutTime(Encoding.Latin1.GetString(Written(s => Step.Write(Fixtures.Box(), s)))), WithoutTime(Encoding.Latin1.GetString(shapeSink.Written)));

        byte[] bytes = sink.Written;
        ModelNode read = Step.Read(new MemoryStream(bytes)).Value;
        Result<ModelNode> readAsync = await Step.ReadAsync(new AsyncOnlyStream(bytes));
        Assert.True(readAsync.IsSuccess, readAsync.Error?.Message);
        Assert.Equal(Describe(read), Describe(readAsync.Value));

        // The file form, which may read the files it refers to
        string path = Path.Combine(Path.GetTempPath(), $"kerf-14g-{Guid.NewGuid():N}.step");
        try
        {
            File.WriteAllBytes(path, bytes);
            Result<ModelNode> fromFile = await Step.ReadAsync(path, new StepReadOptions { ExternalReferences = true });
            Assert.Equal(Describe(Step.Read(path, new StepReadOptions { ExternalReferences = true }).Value), Describe(fromFile.Value));
        }
        finally
        {
            File.Delete(path);
        }

        Assert.Throws<FileNotFoundException>(() => { _ = Step.ReadAsync(Path.Combine(Path.GetTempPath(), "kerf-14g-none.step")); });
    }

    [Fact]
    public async Task STL_reads_the_same_mesh_and_writes_the_same_bytes()
    {
        ShapeMesh mesh = Fixtures.Cylinder().Mesh().Value;
        foreach (bool ascii in new[] { false, true })
        {
            var options = new StlWriteOptions { Ascii = ascii };
            byte[] sync = Written(s => Stl.Write(mesh, s, options));
            var sink = new AsyncOnlyStream();
            Assert.True((await Stl.WriteAsync(mesh, sink, options)).IsSuccess);
            Assert.Equal(sync, sink.Written);

            var readOptions = new StlReadOptions { Unit = LengthUnit.Centimetre };
            ShapeMesh read = Stl.Read(new MemoryStream(sync), readOptions).Value;
            ShapeMesh readAsync = (await Stl.ReadAsync(new AsyncOnlyStream(sync), readOptions)).Value;
            AssertSameMesh(read, readAsync);
        }
    }

    [Fact]
    public async Task GlTF_reads_the_same_model_and_writes_the_same_bytes()
    {
        ModelNode model = ModelNode.Assembly([ModelNode.Part(Fixtures.Box(), "Body", new Color(0, 0, 1)), ModelNode.Part(Fixtures.Cylinder(), "Lid")]);
        foreach (bool binary in new[] { true, false })
        {
            var options = new GltfWriteOptions { Binary = binary };
            byte[] sync = Written(s => Gltf.Write(model, s, options));
            var sink = new AsyncOnlyStream();
            Assert.True((await Gltf.WriteAsync(model, sink, options)).IsSuccess);
            Assert.Equal(sync, sink.Written);

            ModelNode read = Gltf.Read(new MemoryStream(sync)).Value;
            ModelNode readAsync = (await Gltf.ReadAsync(new AsyncOnlyStream(sync))).Value;
            Assert.Equal(Describe(read), Describe(readAsync));
        }

        // JSON and its buffer in two streams
        var json = new MemoryStream();
        var bin = new MemoryStream();
        Assert.True(Gltf.Write(model, json, bin, "model.bin").IsSuccess);
        var jsonSink = new AsyncOnlyStream();
        var binSink = new AsyncOnlyStream();
        Assert.True((await Gltf.WriteAsync(model, jsonSink, binSink, "model.bin")).IsSuccess);
        Assert.Equal(json.ToArray(), jsonSink.Written);
        Assert.Equal(bin.ToArray(), binSink.Written);
    }

    [Fact]
    public async Task ThreeMF_and_OBJ_write_the_same_bytes()
    {
        ModelNode model = ModelNode.Part(Fixtures.Box(), "Body", new Color(0.2, 0.4, 0.6));
        byte[] package = Written(s => ThreeMf.Write(model, s));
        var sink = new AsyncOnlyStream();
        Assert.True((await ThreeMf.WriteAsync(model, sink)).IsSuccess);
        Assert.Equal(package, sink.Written);

        var obj = new MemoryStream();
        var mtl = new MemoryStream();
        Assert.True(Obj.Write(model, obj, mtl, "body.mtl").IsSuccess);
        var objSink = new AsyncOnlyStream();
        var mtlSink = new AsyncOnlyStream();
        Assert.True((await Obj.WriteAsync(model, objSink, mtlSink, "body.mtl")).IsSuccess);
        Assert.Equal(obj.ToArray(), objSink.Written);
        Assert.Equal(mtl.ToArray(), mtlSink.Written);
        var objAlone = new AsyncOnlyStream();
        Assert.True((await Obj.WriteAsync(model, objAlone)).IsSuccess);
        Assert.Equal(Written(s => Obj.Write(model, s)), objAlone.Written);
    }

    [Fact]
    public async Task SVG_and_PNG_write_the_same_bytes()
    {
        LineDrawing drawing = Fixtures.Box().ProjectLines(Camera.Isometric).Value;
        var svg = new AsyncOnlyStream();
        Assert.True((await Svg.WriteAsync(drawing, svg)).IsSuccess);
        Assert.Equal(Written(s => Svg.Write(drawing, s)), svg.Written);
        Assert.True(svg.Flushed);

        RgbaImage image = Fixtures.Box().Render(Camera.Isometric, new RenderOptions { Width = 64, Height = 48 }).Value;
        var png = new AsyncOnlyStream();
        Assert.True((await Png.WriteAsync(image, png)).IsSuccess);
        Assert.Equal(Written(s => Png.Write(image, s)), png.Written);

        // A stream that can't be written: the same failure as the synchronous form's
        Result svgFailed = Svg.Write(drawing, new BrokenStream());
        Result svgFailedAsync = await Svg.WriteAsync(drawing, new BrokenStream());
        Assert.Equal(svgFailed.Error!.Code, svgFailedAsync.Error!.Code);
        Assert.Equal(svgFailed.Error.Message, svgFailedAsync.Error.Message);
        Result pngFailed = Png.Write(image, new BrokenStream());
        Result pngFailedAsync = await Png.WriteAsync(image, new BrokenStream());
        Assert.Equal(pngFailed.Error!.Message, pngFailedAsync.Error!.Message);
    }

    [Fact]
    public async Task A_font_is_read_the_same()
    {
        byte[] bytes = LiberationSans();
        Font font = Font.Read(new MemoryStream(bytes)).Value;
        Font fontAsync = (await Font.ReadAsync(new AsyncOnlyStream(bytes))).Value;
        Assert.Equal(font.PostScriptName, fontAsync.PostScriptName);
        Assert.Equal(font.GlyphCount, fontAsync.GlyphCount);
        Assert.Equal(font.Outline(36).Value.Contours.Select(c => c.ToArray()), fontAsync.Outline(36).Value.Contours.Select(c => c.ToArray()));
        IReadOnlyList<Font> collection = (await Font.ReadCollectionAsync(new AsyncOnlyStream(bytes))).Value;
        Assert.Equal(font.PostScriptName, Assert.Single(collection).PostScriptName);

        var limits = new FontReadLimits { MaxBytes = 1000 };
        Result<Font> large = Font.Read(new MemoryStream(bytes), limits: limits);
        Result<Font> largeAsync = await Font.ReadAsync(new AsyncOnlyStream(bytes), limits: limits);
        Assert.Equal(large.Error!.Message, largeAsync.Error!.Message);
        Assert.Equal(Font.LimitExceeded, (await Font.ReadCollectionAsync(new AsyncOnlyStream(bytes), limits)).Error!.Code);
    }

    // --------------------------------------------------------------------------------------- limits and failures

    [Fact]
    public async Task Data_past_a_limit_or_malformed_fails_as_the_synchronous_form_fails()
    {
        byte[] box = Fixtures.Bytes("box");
        var small = new ReadLimits { MaxBytes = 100 };
        AssertSameFailure(Brep.Read(new MemoryStream(box), small), await Brep.ReadAsync(new AsyncOnlyStream(box), small));
        AssertSameFailure(Brep.Read(new MemoryStream("not a shape"u8.ToArray())), await Brep.ReadAsync(new AsyncOnlyStream("not a shape"u8.ToArray())));

        byte[] step = Written(s => Step.Write(Fixtures.Box(), s));
        var stepSmall = new StepReadOptions { Limits = small };
        AssertSameFailure(Step.Read(new MemoryStream(step), stepSmall), await Step.ReadAsync(new AsyncOnlyStream(step), stepSmall));
        AssertSameFailure(Step.Read(new MemoryStream("ISO-10303-21; garbage"u8.ToArray())), await Step.ReadAsync(new AsyncOnlyStream("ISO-10303-21; garbage"u8.ToArray())));
        // A limit exactly the data's size passes, as it does synchronously
        var exact = new StepReadOptions { Limits = new ReadLimits { MaxBytes = step.Length } };
        Assert.True((await Step.ReadAsync(new AsyncOnlyStream(step), exact)).IsSuccess);

        byte[] stl = Written(s => Stl.Write(Fixtures.Box().Mesh().Value, s));
        var stlSmall = new StlReadOptions { Limits = small };
        AssertSameFailure(Stl.Read(new MemoryStream(stl), stlSmall), await Stl.ReadAsync(new AsyncOnlyStream(stl), stlSmall));

        byte[] glb = Written(s => Gltf.Write(ModelNode.Part(Fixtures.Box()), s));
        var gltfSmall = new GltfReadOptions { Limits = small };
        AssertSameFailure(Gltf.Read(new MemoryStream(glb), gltfSmall), await Gltf.ReadAsync(new AsyncOnlyStream(glb), gltfSmall));
        AssertSameFailure(Gltf.Read(new MemoryStream("{ not json"u8.ToArray())), await Gltf.ReadAsync(new AsyncOnlyStream("{ not json"u8.ToArray())));
    }

    [Fact]
    public async Task Arguments_are_checked_before_the_task_and_cancellation_cancels_it()
    {
        Assert.ThrowsAny<ArgumentNullException>(() => { _ = Brep.ReadAsync(null!); });
        Assert.ThrowsAny<ArgumentNullException>(() => { _ = Step.WriteAsync(Fixtures.Box(), null!); });
        Assert.ThrowsAny<ArgumentNullException>(() => { _ = Png.WriteAsync(null!, new MemoryStream()); });

        byte[] step = Written(s => Step.Write(Fixtures.Box(), s));
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Step.ReadAsync(new AsyncOnlyStream(step), cancellationToken: cancelled.Token));
        var sink = new AsyncOnlyStream();
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Step.WriteAsync(Fixtures.Box(), sink, cancellationToken: cancelled.Token));
        Assert.Empty(sink.Written);
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Svg.WriteAsync(Fixtures.Box().ProjectLines(Camera.Top).Value, sink, cancellationToken: cancelled.Token));

        // The token in force (Cancellation.Use) is obeyed as by the synchronous forms, across the awaits
        using (Cancellation.Use(cancelled.Token))
        {
            await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Brep.ReadAsync(new AsyncOnlyStream(Fixtures.Bytes("box"))));
            await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Stl.WriteAsync(Fixtures.Box().Mesh().Value, sink));
        }

        // Cancelled while reading: the stream's read is cancelled
        using var later = new CancellationTokenSource();
        var slow = new AsyncOnlyStream(step, onRead: () => later.Cancel());
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => Step.ReadAsync(slow, cancellationToken: later.Token));
    }

    // ----------------------------------------------------------------------------------------------------- helpers

    private static void AssertSameFailure<T>(Result<T> sync, Result<T> async)
    {
        Assert.True(sync.IsFailure);
        Assert.True(async.IsFailure);
        Assert.Equal(sync.Error!.Code, async.Error!.Code);
        Assert.Equal(sync.Error.Message, async.Error.Message);
    }

    private static byte[] Written(Func<Stream, Result> write)
    {
        var stream = new MemoryStream();
        Assert.True(write(stream).IsSuccess);
        return stream.ToArray();
    }

    private static string BrepText(Shape shape) => Encoding.Latin1.GetString(Written(s => Brep.Write(shape, s)));

    // FILE_NAME's time of writing, which differs between two writes
    private static string WithoutTime(string step) => Regex.Replace(step, @"'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d'", "'time'");

    // A model node and its descendants, each with its shape as BREP text or its mesh's arrays
    private static string Describe(ModelNode node)
    {
        var text = new StringBuilder();
        Describe(node, 0, text);
        return text.ToString();
    }

    private static void Describe(ModelNode node, int depth, StringBuilder text)
    {
        text.Append(' ', depth).Append(node.Name).Append(' ').Append(node.Color).Append(' ').Append(node.Placement).Append('\n');
        if (node.Shape is { } shape)
        {
            text.Append(BrepText(shape)).Append('\n');
        }

        if (node.Mesh is { } mesh)
        {
            text.AppendJoin(',', mesh.Positions.ToArray()).Append('\n').AppendJoin(',', mesh.Triangles.ToArray()).Append('\n');
        }

        foreach (ModelNode child in node.Children)
        {
            Describe(child, depth + 1, text);
        }
    }

    private static void AssertSameMesh(ShapeMesh a, ShapeMesh b)
    {
        Assert.Equal(a.Positions.ToArray(), b.Positions.ToArray());
        Assert.Equal(a.Normals.ToArray(), b.Normals.ToArray());
        Assert.Equal(a.Triangles.ToArray(), b.Triangles.ToArray());
    }

    private static byte[] LiberationSans()
    {
        using Stream compressed = typeof(Font).Assembly.GetManifestResourceStream("Kerf.Fonts.LiberationSans-Regular.ttf.gz")
                                  ?? throw new InvalidOperationException("No Liberation Sans");
        using var font = new GZipStream(compressed, CompressionMode.Decompress);
        using var memory = new MemoryStream();
        font.CopyTo(memory);
        return memory.ToArray();
    }

    /// <summary>
    /// A stream that can be read and written only asynchronously, a few bytes at a time and yielding each time, so a
    /// reader or writer that did synchronous I/O, or assumed one read fills its buffer, fails.
    /// </summary>
    private sealed class AsyncOnlyStream(byte[]? data = null, Action? onRead = null) : Stream
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
            onRead?.Invoke();
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

    /// <summary>A stream whose every write fails, as a full disk's.</summary>
    private sealed class BrokenStream : Stream
    {
        public override bool CanRead => false;

        public override bool CanSeek => false;

        public override bool CanWrite => true;

        public override long Length => throw new NotSupportedException();

        public override long Position { get => throw new NotSupportedException(); set => throw new NotSupportedException(); }

        public override int Read(byte[] buffer, int offset, int count) => throw new NotSupportedException();

        public override void Write(byte[] buffer, int offset, int count) => throw new IOException("The disk is full.");

        public override ValueTask WriteAsync(ReadOnlyMemory<byte> buffer, CancellationToken cancellationToken = default)
            => ValueTask.FromException(new IOException("The disk is full."));

        public override void Flush()
        {
        }

        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();

        public override void SetLength(long value) => throw new NotSupportedException();
    }
}
