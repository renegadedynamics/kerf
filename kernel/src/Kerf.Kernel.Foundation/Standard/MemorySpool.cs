// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Foundation;

/// <summary>
/// Bytes held in memory in chunks, as a seekable stream, for the async forms of the readers and writers (unit 14G): a
/// reader's input read asynchronously and then parsed from here, a writer's output built here and then written
/// asynchronously. Chunks rather than one array, so that it grows without copying what it holds (a
/// <see cref="MemoryStream"/> doubles its array, holding both while it copies) and isn't limited to 2 GB.
/// </summary>
/// <remarks>Not thread-safe, as a <see cref="MemoryStream"/> isn't: one reader or writer at a time.</remarks>
internal sealed class MemorySpool : Stream
{
    // The first chunk is small, for the many small files; each one after is twice the last, up to a megabyte, so the
    // space unused at the end is at most a megabyte, and a gigabyte is about a thousand chunks
    private const int FirstChunk = 16 << 10;
    private const int LargestChunk = 1 << 20;

    private readonly List<byte[]> _chunks = [];
    private long _length;
    private long _capacity;
    private long _position;

    /// <inheritdoc/>
    public override bool CanRead => true;

    /// <inheritdoc/>
    public override bool CanSeek => true;

    /// <inheritdoc/>
    public override bool CanWrite => true;

    /// <inheritdoc/>
    public override long Length => _length;

    /// <inheritdoc/>
    public override long Position
    {
        get => _position;
        set
        {
            KernelArgumentOutOfRangeException.ThrowIfNegative(value);
            _position = value;
        }
    }

    /// <summary>
    /// The bytes of <paramref name="source"/> from its position, read asynchronously: all of them, or the first
    /// <paramref name="maxBytes"/> + 1 if there are more, so that the reader given them refuses them for their size as
    /// it would have refused the stream. Positioned at the start.
    /// </summary>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static async Task<MemorySpool> ReadFromAsync(Stream source, long maxBytes, CancellationToken cancellationToken)
    {
        long most = maxBytes >= long.MaxValue - 1 ? long.MaxValue : Math.Max(0, maxBytes) + 1;
        var spool = new MemorySpool();
        try
        {
            while (spool._length < most)
            {
                cancellationToken.ThrowIfCancellationRequested();
                Memory<byte> free = spool.FreeSpace(most - spool._length);
                int read = await source.ReadAsync(free, cancellationToken).ConfigureAwait(false);
                if (read <= 0)
                {
                    break;
                }

                spool._length += read;
            }
        }
        catch
        {
            spool.Dispose();
            throw;
        }

        spool._position = 0;
        return spool;
    }

    /// <summary>Writes every byte held to <paramref name="destination"/> asynchronously, and flushes it.</summary>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public async Task WriteToAsync(Stream destination, CancellationToken cancellationToken)
    {
        long left = _length;
        foreach (byte[] chunk in _chunks)
        {
            if (left == 0)
            {
                break;
            }

            int count = (int)Math.Min(chunk.Length, left);
            await destination.WriteAsync(chunk.AsMemory(0, count), cancellationToken).ConfigureAwait(false);
            left -= count;
        }

        await destination.FlushAsync(cancellationToken).ConfigureAwait(false);
    }

    /// <summary>
    /// What <paramref name="read"/> makes of the bytes of <paramref name="source"/>, read asynchronously into memory
    /// first (<see cref="ReadFromAsync"/>): the async form of a reader, its parsing still synchronous.
    /// </summary>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static async Task<T> ReadThenAsync<T>(Stream source, long maxBytes, Func<Stream, T> read, CancellationToken cancellationToken)
    {
        using MemorySpool data = await ReadFromAsync(source, maxBytes, Cancellation.Effective(cancellationToken)).ConfigureAwait(false);
        return read(data);
    }

    /// <summary>
    /// <paramref name="write"/> run into memory, and its output then written to <paramref name="destination"/>
    /// asynchronously, and flushed, if it succeeded: the async form of a writer, its building still synchronous (on
    /// the caller's thread, before the task is returned). Nothing is written on a failure.
    /// </summary>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static async Task<Result> WriteThenAsync(Stream destination, Func<Stream, Result> write, CancellationToken cancellationToken)
    {
        CancellationToken effective = Cancellation.Effective(cancellationToken);
        effective.ThrowIfCancellationRequested();
        using var data = new MemorySpool();
        Result written = write(data);
        if (written.IsFailure)
        {
            return written;
        }

        await data.WriteToAsync(destination, effective).ConfigureAwait(false);
        return written;
    }

    /// <inheritdoc/>
    public override int Read(byte[] buffer, int offset, int count) => Read(buffer.AsSpan(offset, count));

    /// <inheritdoc/>
    public override int Read(Span<byte> buffer)
    {
        int done = 0;
        while (done < buffer.Length && _position < _length)
        {
            (int index, int at) = Locate(_position);
            byte[] chunk = _chunks[index];
            int count = (int)Math.Min(Math.Min(chunk.Length - at, buffer.Length - done), _length - _position);
            chunk.AsSpan(at, count).CopyTo(buffer[done..]);
            done += count;
            _position += count;
        }

        return done;
    }

    /// <inheritdoc/>
    public override int ReadByte()
    {
        Span<byte> one = stackalloc byte[1];
        return Read(one) == 1 ? one[0] : -1;
    }

    /// <inheritdoc/>
    public override void Write(byte[] buffer, int offset, int count) => Write(buffer.AsSpan(offset, count));

    /// <inheritdoc/>
    public override void Write(ReadOnlySpan<byte> buffer)
    {
        long end = _position + buffer.Length;
        if (_position > _length)
        {
            // Past the end: the gap reads as zeros, as a MemoryStream's
            Grow(_position);
            ClearRange(_length, _position);
        }

        Grow(end);
        while (!buffer.IsEmpty)
        {
            (int index, int at) = Locate(_position);
            byte[] chunk = _chunks[index];
            int count = Math.Min(chunk.Length - at, buffer.Length);
            buffer[..count].CopyTo(chunk.AsSpan(at));
            buffer = buffer[count..];
            _position += count;
        }

        _length = Math.Max(_length, end);
    }

    /// <inheritdoc/>
    public override void WriteByte(byte value) => Write([value]);

    /// <inheritdoc/>
    public override long Seek(long offset, SeekOrigin origin)
    {
        long position = origin switch
        {
            SeekOrigin.Begin => offset,
            SeekOrigin.Current => _position + offset,
            SeekOrigin.End => _length + offset,
            _ => throw new KernelArgumentOutOfRangeException(nameof(origin)),
        };
        if (position < 0)
        {
            throw new IOException("An attempt was made to move the position before the beginning of the stream.");
        }

        _position = position;
        return position;
    }

    /// <inheritdoc/>
    public override void SetLength(long value)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(value);
        if (value > _length)
        {
            Grow(value);
            ClearRange(_length, value);
        }

        _length = value;
    }

    /// <inheritdoc/>
    public override void Flush()
    {
    }

    /// <inheritdoc/>
    protected override void Dispose(bool disposing)
    {
        _chunks.Clear();
        _length = _capacity = _position = 0;
        base.Dispose(disposing);
    }

    // The room after the last byte held, in the chunk it falls in, a new chunk if that is full; at most `most` bytes
    private Memory<byte> FreeSpace(long most)
    {
        if (_length == _capacity)
        {
            AddChunk();
        }

        (int index, int at) = Locate(_length);
        byte[] chunk = _chunks[index];
        return chunk.AsMemory(at, (int)Math.Min(chunk.Length - at, most));
    }

    private void Grow(long capacity)
    {
        while (_capacity < capacity)
        {
            AddChunk();
        }
    }

    private void AddChunk()
    {
        int size = _chunks.Count == 0 ? FirstChunk : Math.Min(_chunks[^1].Length * 2, LargestChunk);
        _chunks.Add(new byte[size]);
        _capacity += size;
    }

    // The chunk holding a position, and where in it: the chunks before the largest size are 16, 32 … 512 KB, a
    // megabyte less 16 KB in all
    private (int Index, int At) Locate(long position)
    {
        long start = 0;
        int index = 0;
        int size = FirstChunk;
        while (size < LargestChunk)
        {
            if (position < start + size)
            {
                return (index, (int)(position - start));
            }

            start += size;
            index++;
            size *= 2;
        }

        long past = position - start;
        return (index + (int)(past / LargestChunk), (int)(past % LargestChunk));
    }

    // Zeros over a range of chunks a write past the end skipped, which may hold bytes from before a SetLength
    private void ClearRange(long from, long to)
    {
        while (from < to)
        {
            (int index, int at) = Locate(from);
            byte[] chunk = _chunks[index];
            int count = (int)Math.Min(chunk.Length - at, to - from);
            Array.Clear(chunk, at, count);
            from += count;
        }
    }
}
