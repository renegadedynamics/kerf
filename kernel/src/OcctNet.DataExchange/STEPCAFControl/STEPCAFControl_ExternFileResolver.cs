// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Opens an external file a STEP file refers to, for <see cref="STEPCAFControl_Reader"/> (OCCT.NET, unit 8Z: OCCT
/// reads external files from the disk, beside the main file). Returns a readable stream, which the reader disposes, or
/// null to leave the reference unresolved, as OCCT leaves a reference to a missing file.
/// </summary>
/// <param name="theFileName">The file's name as the reference gives it (relative, absolute, a UNC path...).</param>
/// <param name="theReferencingName">The name of the file holding the reference: the main file's name as given to
/// ReadFile or ReadStream, or, for a file an external file refers to, the name the reader gave that file (see
/// <see cref="STEPCAFControl_Reader.SetExternFileResolver"/>).</param>
public delegate Stream? STEPCAFControl_ExternFileResolver(string theFileName, string theReferencingName);

/// <summary>
/// Opens the stream an external file is written to, for <see cref="STEPCAFControl_Writer.WriteStream(Stream,
/// STEPCAFControl_ExternFileWriter)"/> (OCCT.NET, unit 8Z). Returns a writable stream, which the writer disposes, or
/// null to refuse the file (its write status is then <c>IFSelect_RetError</c>).
/// </summary>
/// <param name="theFileName">The external file's name as the writer made it (the prefix and the label's name, as
/// OCCT): it comes from the document and may hold separators, <c>..</c> or a drive.</param>
public delegate Stream? STEPCAFControl_ExternFileWriter(string theFileName);

/// <summary>
/// The budget of a <see cref="STEPCAFControl_Reader"/>'s external reads, across all the files read for one main file
/// (OCCT.NET, unit 8Z). Each external file is read with the caller's <see cref="StepFile_ReadLimits"/> too, its
/// <see cref="StepFile_ReadLimits.MaxFileSize"/> lowered to what is left of <see cref="MaxTotalSize"/>.
/// </summary>
public sealed record STEPCAFControl_ExternFileLimits
{
    /// <summary>The defaults: 1,000 files, 16 GiB in all, nesting 100 deep.</summary>
    public static STEPCAFControl_ExternFileLimits Default { get; } = new();

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
internal sealed class STEPCAFControl_CountingStream(Stream theInner, long theMaxSize) : Stream
{
    private int allowed(int theCount)
    {
        long aLeft = theMaxSize == long.MaxValue ? long.MaxValue : theMaxSize + 1 - BytesRead;
        return (int)Math.Max(0, Math.Min(theCount, aLeft));
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
        int aCount = allowed(count);
        if (aCount == 0)
        {
            return 0;
        }
        int aRead = theInner.Read(buffer, offset, aCount);
        BytesRead += aRead;
        return aRead;
    }

    public override int Read(Span<byte> buffer)
    {
        int aCount = allowed(buffer.Length);
        if (aCount == 0)
        {
            return 0;
        }
        int aRead = theInner.Read(buffer.Slice(0, aCount));
        BytesRead += aRead;
        return aRead;
    }
}
