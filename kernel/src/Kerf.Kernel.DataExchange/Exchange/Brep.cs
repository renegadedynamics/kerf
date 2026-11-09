// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Exchange.Brep.Binary;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Kerf.Exchange;

/// <summary>BREP, OCCT's own format of shapes: text or binary (OCCT's BRepTools and BinTools).</summary>
public static class Brep
{
    // The binary format's first line after a line end (BinTools_ShapeSet), which text's never is
    private static ReadOnlySpan<byte> BinaryHeader => "Open CASCADE Topology V"u8;

    /// <summary>
    /// The shape of BREP data, text or binary (recognised by its first line), within <see cref="ReadLimits.MaxBytes"/>.
    /// The shape keeps any meshes stored with it; the public API reads them only through <see cref="Brep.Write"/>.
    /// </summary>
    /// <returns>The shape, or a failure: <c>exchange.malformed</c>, <c>exchange.limit-exceeded</c>.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<Shape> Read(Stream stream, ReadLimits? limits = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        limits ??= ReadLimits.Default;
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
        cancellationToken.ThrowIfCancellationRequested();
        Shape shape;
        try
        {
            using MemoryStream data = ExchangeTree.ReadAll(stream, limits.MaxBytes, cancellationToken);
            ReadOnlySpan<byte> bytes = data.GetBuffer().AsSpan(0, (int)data.Length);
            var range = new ProgressIndicator(cancellationToken).Start();
            if (bytes.TrimStart("\r\n"u8).StartsWith(BinaryHeader))
            {
                BinTools.Read(out shape, data, range);
            }
            else
            {
                string text = Encoding.Latin1.GetString(bytes);
                var reader = new StringReader(text);
                StreamUtilities.SetInputLength(reader, text.Length);
                BrepTools.Read(out shape, reader, range);
            }
        }
        catch (Exception e) when (e is not OperationCanceledException && ExchangeTree.IsBadData(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<Shape>.Failure(ExchangeTree.BadData(e, "BREP"));
        }

        cancellationToken.ThrowIfCancellationRequested();
        return shape.IsNull
            ? Result<Shape>.Failure(Diagnostic.Error(ExchangeTree.Malformed, "The BREP data holds no shape."))
            : shape;
    }

    /// <summary>
    /// <see cref="Read(Stream, ReadLimits?, CancellationToken)"/> with its input read asynchronously: the stream's bytes
    /// are read into memory without blocking a thread (up to <see cref="ReadLimits.MaxBytes"/> and one more, so data past
    /// the limit is refused as the synchronous form refuses it), then parsed synchronously, on the thread the read
    /// finished on. The same result as the synchronous form.
    /// </summary>
    /// <returns>The shape, or a failure: <c>exchange.malformed</c>, <c>exchange.limit-exceeded</c>.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result<Shape>> ReadAsync(Stream stream, ReadLimits? limits = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        limits ??= ReadLimits.Default;
        return MemorySpool.ReadThenAsync(stream, limits.MaxBytes, data => Read(data, limits, cancellationToken), cancellationToken);
    }

    /// <summary>
    /// <see cref="Write"/> with its output written asynchronously: the BREP is built in memory, synchronously, on the
    /// caller's thread, then written to the stream without blocking a thread, and the stream flushed. The same bytes as
    /// the synchronous form; nothing is written on a failure.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result> WriteAsync(Shape shape, Stream stream, BrepWriteOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        if (shape.IsNull)
        {
            throw new KernelArgumentException("The shape is null.", nameof(shape));
        }

        return MemorySpool.WriteThenAsync(stream, data => Write(shape, data, options), cancellationToken);
    }

    /// <summary>
    /// Writes a shape as BREP: OCCT's text (the current version, LF line ends, no Draw header line) or its binary
    /// format, with the meshes stored on its faces unless <see cref="BrepWriteOptions.WithTriangles"/> is false.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static Result Write(Shape shape, Stream stream, BrepWriteOptions? options = null)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        if (shape.IsNull)
        {
            throw new KernelArgumentException("The shape is null.", nameof(shape));
        }

        options ??= BrepWriteOptions.Default;
        if (options.Binary)
        {
            BinTools.Write(shape, stream, options.WithTriangles, options.WithNormals, BinaryFormatVersion.FormatVersion_CURRENT);
            return Result.Success();
        }

        var writer = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(shape, writer, options.WithTriangles, options.WithNormals, FormatVersion.FormatVersion_CURRENT);
        stream.Write(Encoding.Latin1.GetBytes(writer.ToString()));
        return Result.Success();
    }
}
