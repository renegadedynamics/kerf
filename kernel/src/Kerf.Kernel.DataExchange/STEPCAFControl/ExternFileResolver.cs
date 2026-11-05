// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Caf;

using Kerf.Engine.Exchange.Step.Parser;

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Documents;

/// <summary>
/// Opens an external file a STEP file refers to, for <see cref="CafReader"/> (Kerf, unit 8Z: OCCT
/// reads external files from the disk, beside the main file). Returns a readable stream, which the reader disposes, or
/// null to leave the reference unresolved, as OCCT leaves a reference to a missing file.
/// </summary>
/// <param name="fileName">The file's name as the reference gives it (relative, absolute, a UNC path...).</param>
/// <param name="referencingName">The name of the file holding the reference: the main file's name as given to
/// ReadFile or ReadStream, or, for a file an external file refers to, the name the reader gave that file (see
/// <see cref="CafReader.SetExternFileResolver"/>).</param>
internal delegate Stream? ExternFileResolver(string fileName, string referencingName);

/// <summary>
/// Opens the stream an external file is written to, for <see cref="CafWriter.WriteStream(Stream,
/// ExternFileWriter)"/> (Kerf, unit 8Z). Returns a writable stream, which the writer disposes, or
/// null to refuse the file (its write status is then <c>IFSelect_RetError</c>).
/// </summary>
/// <param name="fileName">The external file's name as the writer made it (the prefix and the label's name, as
/// OCCT): it comes from the document and may hold separators, <c>..</c> or a drive.</param>
internal delegate Stream? ExternFileWriter(string fileName);

/// <summary>
/// The budget of a <see cref="CafReader"/>'s external reads, across all the files read for one main file
/// (Kerf, unit 8Z). Each external file is read with the caller's <see cref="ParserReadLimits"/> too, its
/// <see cref="ParserReadLimits.MaxFileSize"/> lowered to what is left of <see cref="MaxTotalSize"/>.
/// </summary>
internal sealed record ExternFileLimits
{
    /// <summary>The defaults: 1,000 files, 16 GiB in all, nesting 100 deep.</summary>
    public static ExternFileLimits Default { get; } = new();

    /// <summary>The most external files opened.</summary>
    public int MaxFiles { get; init; } = 1_000;

    /// <summary>The most bytes read from all the external files together (the main file is not counted).</summary>
    public long MaxTotalSize { get; init; } = 16L << 30;

    /// <summary>The deepest chain of external files (the main file refers to one at depth 1).</summary>
    public int MaxDepth { get; init; } = 100;
}

/// <summary>
/// Counts the bytes read through a stream (the external files' budget), and reads no more than one byte past the
/// limit (enough for the parser to see the limit passed).
/// </summary>
internal sealed class CountingStream(Stream theInner, long theMaxSize) : Stream
{
    private int Allowed(int count)
    {
        long left = theMaxSize == long.MaxValue ? long.MaxValue : theMaxSize + 1 - BytesRead;
        return (int)Math.Max(0, Math.Min(count, left));
    }

    public long BytesRead { get; private set; }

    public override bool CanRead => theInner.CanRead;
    public override bool CanSeek => false;
    public override bool CanWrite => false;
    public override long Length => throw new NotSupportedException();
    public override long Position { get => BytesRead; set => throw new NotSupportedException(); }
    public override void Flush() { }
    public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
    public override void SetLength(long value) => throw new NotSupportedException();
    public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();

    public override int Read(byte[] buffer, int offset, int count)
    {
        int aCount = Allowed(count);
        if (aCount == 0)
        {
            return 0;
        }
        int read = theInner.Read(buffer, offset, aCount);
        BytesRead += read;
        return read;
    }

    public override int Read(Span<byte> buffer)
    {
        int count = Allowed(buffer.Length);
        if (count == 0)
        {
            return 0;
        }
        int read = theInner.Read(buffer.Slice(0, count));
        BytesRead += read;
        return read;
    }
}
