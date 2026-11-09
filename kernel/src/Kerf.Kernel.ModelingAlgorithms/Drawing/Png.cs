// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Buffers.Binary;
using System.IO.Compression;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Text;

namespace Kerf.Drawing;

/// <summary>How <see cref="Png"/> compresses.</summary>
public enum PngCompression
{
    /// <summary>Kerf's own deflate (fixed Huffman codes, LZ77 matching): the same bytes on every machine and .NET
    /// version.</summary>
    Deterministic,

    /// <summary>.NET's zlib at its smallest size: smaller files, whose bytes may differ between platforms and .NET
    /// versions (the pixels don't).</summary>
    Smallest,
}

/// <summary>How <see cref="Png"/> writes.</summary>
public sealed record PngOptions
{
    /// <summary>Deterministic compression.</summary>
    public static PngOptions Default { get; } = new();

    /// <summary>How the pixels are compressed, deterministically by default.</summary>
    public PngCompression Compression { get; init; }
}

/// <summary>
/// The PNG writer (docs/api/drawing.md), in managed code: 8-bit RGBA, each row filtered with the filter that suits it,
/// zlib-compressed, with its CRC-32s. With <see cref="PngCompression.Deterministic"/> the same picture gives the same
/// bytes on every machine.
/// </summary>
public static class Png
{
    /// <summary>The code of a picture that couldn't be written: <c>png.write-failed</c>.</summary>
    public static DiagnosticCode WriteFailed { get; } = new("png.write-failed");

    /// <summary>Writes the picture as PNG to the stream.</summary>
    /// <returns>Success, or a failure (<see cref="WriteFailed"/>) if the stream couldn't be written.</returns>
    /// <exception cref="ArgumentNullException">The picture or the stream is null.</exception>
    public static Result Write(RgbaImage image, Stream stream, PngOptions? options = null)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        byte[] bytes = Encode(image, options);
        try
        {
            stream.Write(bytes, 0, bytes.Length);
            stream.Flush();
            return Result.Success();
        }
        catch (Exception e) when (e is IOException or NotSupportedException or ObjectDisposedException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(WriteFailed, $"The PNG couldn't be written: {FailureDiagnostics.PublicText(e)}"));
        }
    }

    /// <summary>
    /// <see cref="Write"/> with its output written asynchronously: the PNG is encoded in memory, synchronously, on the
    /// caller's thread, then written to the stream without blocking a thread, and the stream flushed. The same bytes and
    /// result as the synchronous form.
    /// </summary>
    /// <returns>Success, or a failure (<see cref="WriteFailed"/>) if the stream couldn't be written.</returns>
    /// <exception cref="ArgumentNullException">The picture or the stream is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result> WriteAsync(RgbaImage image, Stream stream, PngOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        KernelArgumentNullException.ThrowIfNull(image);
        return WriteBytes();

        async Task<Result> WriteBytes()
        {
            CancellationToken effective = Cancellation.Effective(cancellationToken);
            effective.ThrowIfCancellationRequested();
            byte[] bytes = Encode(image, options);
            try
            {
                await stream.WriteAsync(bytes, effective).ConfigureAwait(false);
                await stream.FlushAsync(effective).ConfigureAwait(false);
                return Result.Success();
            }
            catch (Exception e) when (e is IOException or NotSupportedException or ObjectDisposedException or UnauthorizedAccessException)
            {
                return Result.Failure(Diagnostic.Error(WriteFailed, $"The PNG couldn't be written: {FailureDiagnostics.PublicText(e)}"));
            }
        }
    }

    /// <summary>The picture as PNG bytes.</summary>
    /// <exception cref="ArgumentNullException">The picture is null.</exception>
    public static byte[] Encode(RgbaImage image, PngOptions? options = null)
    {
        KernelArgumentNullException.ThrowIfNull(image);
        options ??= PngOptions.Default;
        byte[] filtered = Filter(image);
        byte[] compressed = options.Compression == PngCompression.Smallest ? ZLib(filtered) : Deflater.ZLib(filtered);

        using var png = new MemoryStream(compressed.Length + 64);
        png.Write([0x89, (byte)'P', (byte)'N', (byte)'G', 0x0D, 0x0A, 0x1A, 0x0A]);
        Span<byte> header = stackalloc byte[13];
        BinaryPrimitives.WriteInt32BigEndian(header, image.Width);
        BinaryPrimitives.WriteInt32BigEndian(header[4..], image.Height);
        header[8] = 8; // bit depth
        header[9] = 6; // RGBA
        header[10] = header[11] = header[12] = 0; // deflate, adaptive filtering, no interlace
        Chunk(png, "IHDR", header);
        Chunk(png, "IDAT", compressed);
        Chunk(png, "IEND", []);
        return png.ToArray();
    }

    private static byte[] ZLib(byte[] data)
    {
        using var output = new MemoryStream();
        using (var zlib = new ZLibStream(output, CompressionLevel.SmallestSize, leaveOpen: true))
        {
            zlib.Write(data);
        }

        return output.ToArray();
    }

    private static void Chunk(Stream png, string type, ReadOnlySpan<byte> data)
    {
        Span<byte> word = stackalloc byte[4];
        BinaryPrimitives.WriteInt32BigEndian(word, data.Length);
        png.Write(word);
        byte[] typeBytes = Encoding.ASCII.GetBytes(type);
        png.Write(typeBytes);
        png.Write(data);
        uint crc = Crc32.Update(Crc32.Update(0xFFFFFFFFu, typeBytes), data) ^ 0xFFFFFFFFu;
        BinaryPrimitives.WriteUInt32BigEndian(word, crc);
        png.Write(word);
    }

    // Each row prefixed by the filter (none, sub, up, average, Paeth) whose output has the smallest sum of magnitudes
    private static byte[] Filter(RgbaImage image)
    {
        int stride = 4 * image.Width;
        ReadOnlySpan<byte> pixels = image.Pixels.Span;
        byte[] output = new byte[(stride + 1) * image.Height];
        byte[] zero = new byte[stride];
        for (int y = 0; y < image.Height; y++)
        {
            ReadOnlySpan<byte> row = pixels.Slice(y * stride, stride), up = y > 0 ? pixels.Slice((y - 1) * stride, stride) : zero;
            // The first filter of the lowest cost; a cost only grows along the row, so a filter stops once it can't win
            int chosen = 0;
            long best = CostOfNone(row);
            for (int filter = 1; filter < 5 && best > 0; filter++)
            {
                long cost = filter switch
                {
                    1 => CostOfSub(row, best),
                    2 => CostOfUp(row, up, best),
                    3 => CostOfAverage(row, up, best),
                    _ => CostOfPaeth(row, up, best),
                };
                if (cost < best)
                {
                    best = cost;
                    chosen = filter;
                }
            }

            output[y * (stride + 1)] = (byte)chosen;
            Apply(chosen, row, up, output.AsSpan(y * (stride + 1) + 1, stride));
        }

        return output;
    }

    // What a filtered byte counts for: its distance from zero, modulo 256
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int Cost(int value)
    {
        value &= 0xFF;
        return value < 128 ? value : 256 - value;
    }

    // The same, summed over sixteen filtered bytes: a byte's distance from zero is the lesser of it and its negation
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static uint Cost(Vector128<byte> filtered)
    {
        (Vector128<ushort> low, Vector128<ushort> high) = Vector128.Widen(Vector128.Min(filtered, Vector128<byte>.Zero - filtered));
        return Vector128.Sum(low + high);
    }

    // The rows are filtered sixteen bytes at a time (unit 14A: filtering and compressing were a fifth of the time the blade
    // bumper's renders took). Each predictor is the one byte by byte below, and a cost is checked against its limit every
    // sixteen bytes: a filter's cost is the whole row's when that's under the limit, and at least the limit when it isn't,
    // as before, so the same filter wins every row
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector128<byte> At(ReadOnlySpan<byte> bytes, int i) => Vector128.Create(bytes.Slice(i, 16));

    // The average of the left (a) and up (b) bytes, rounded down
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector128<byte> Average(Vector128<byte> a, Vector128<byte> b) => (a & b) + Vector128.ShiftRightLogical(a ^ b, 1);

    // Paeth's predictor of the left (a), up (b) and upper left (c) bytes
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector128<byte> Paeth(Vector128<byte> a, Vector128<byte> b, Vector128<byte> c)
    {
        (Vector128<ushort> aLow, Vector128<ushort> aHigh) = Vector128.Widen(a);
        (Vector128<ushort> bLow, Vector128<ushort> bHigh) = Vector128.Widen(b);
        (Vector128<ushort> cLow, Vector128<ushort> cHigh) = Vector128.Widen(c);
        return Vector128.Narrow(Paeth(aLow.AsInt16(), bLow.AsInt16(), cLow.AsInt16()), Paeth(aHigh.AsInt16(), bHigh.AsInt16(), cHigh.AsInt16()));
    }

    // With p = a + b - c: |p - a| = |b - c|, |p - b| = |a - c| and |p - c| = |a + b - 2c|
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector128<ushort> Paeth(Vector128<short> a, Vector128<short> b, Vector128<short> c)
    {
        Vector128<short> pa = Vector128.Abs(b - c), pb = Vector128.Abs(a - c), pc = Vector128.Abs(a + b - c - c);
        Vector128<short> takeA = Vector128.LessThanOrEqual(pa, pb) & Vector128.LessThanOrEqual(pa, pc);
        Vector128<short> takeB = Vector128.LessThanOrEqual(pb, pc);
        return Vector128.ConditionalSelect(takeA, a, Vector128.ConditionalSelect(takeB, b, c)).AsUInt16();
    }

    private static long CostOfNone(ReadOnlySpan<byte> row)
    {
        long cost = 0;
        int i = 0;
        for (; i + 16 <= row.Length; i += 16)
        {
            cost += Cost(At(row, i));
        }

        for (; i < row.Length; i++)
        {
            cost += Cost(row[i]);
        }

        return cost;
    }

    private static long CostOfSub(ReadOnlySpan<byte> row, long limit)
    {
        long cost = 0;
        int i = 0;
        for (; i < 4 && i < row.Length; i++)
        {
            cost += Cost(row[i]);
        }

        for (; i + 16 <= row.Length && cost < limit; i += 16)
        {
            cost += Cost(At(row, i) - At(row, i - 4));
        }

        for (; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - row[i - 4]);
        }

        return cost;
    }

    private static long CostOfUp(ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, long limit)
    {
        long cost = 0;
        int i = 0;
        for (; i + 16 <= row.Length && cost < limit; i += 16)
        {
            cost += Cost(At(row, i) - At(up, i));
        }

        for (; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - up[i]);
        }

        return cost;
    }

    private static long CostOfAverage(ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, long limit)
    {
        long cost = 0;
        int i = 0;
        for (; i < 4 && i < row.Length; i++)
        {
            cost += Cost(row[i] - (up[i] >> 1));
        }

        for (; i + 16 <= row.Length && cost < limit; i += 16)
        {
            cost += Cost(At(row, i) - Average(At(row, i - 4), At(up, i)));
        }

        for (; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - ((row[i - 4] + up[i]) >> 1));
        }

        return cost;
    }

    private static long CostOfPaeth(ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, long limit)
    {
        long cost = 0;
        int i = 0;
        for (; i < 4 && i < row.Length; i++)
        {
            cost += Cost(row[i] - Paeth(0, up[i], 0));
        }

        for (; i + 16 <= row.Length && cost < limit; i += 16)
        {
            cost += Cost(At(row, i) - Paeth(At(row, i - 4), At(up, i), At(up, i - 4)));
        }

        for (; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - Paeth(row[i - 4], up[i], up[i - 4]));
        }

        return cost;
    }

    private static void Apply(int filter, ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, Span<byte> target)
    {
        // The first pixel has no left neighbour: byte by byte, then sixteen at a time
        int i = 0;
        for (; i < 4 && i < row.Length; i++)
        {
            target[i] = (byte)(row[i] - Predictor(filter, row, up, i));
        }

        for (; i + 16 <= row.Length; i += 16)
        {
            Vector128<byte> predictor = filter switch
            {
                0 => Vector128<byte>.Zero,
                1 => At(row, i - 4),
                2 => At(up, i),
                3 => Average(At(row, i - 4), At(up, i)),
                _ => Paeth(At(row, i - 4), At(up, i), At(up, i - 4)),
            };
            (At(row, i) - predictor).CopyTo(target.Slice(i, 16));
        }

        for (; i < row.Length; i++)
        {
            target[i] = (byte)(row[i] - Predictor(filter, row, up, i));
        }
    }

    private static int Predictor(int filter, ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, int i)
    {
        int a = i >= 4 ? row[i - 4] : 0, b = up[i], c = i >= 4 ? up[i - 4] : 0;
        return filter switch
        {
            0 => 0,
            1 => a,
            2 => b,
            3 => (a + b) >> 1,
            _ => Paeth(a, b, c),
        };
    }

    private static int Paeth(int a, int b, int c)
    {
        int p = a + b - c, pa = Math.Abs(p - a), pb = Math.Abs(p - b), pc = Math.Abs(p - c);
        return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
    }

    /// <summary>CRC-32 (ISO-HDLC, PNG's).</summary>
    internal static class Crc32
    {
        private static readonly uint[] Table = Make();

        private static uint[] Make()
        {
            var table = new uint[256];
            for (uint n = 0; n < 256; n++)
            {
                uint c = n;
                for (int k = 0; k < 8; k++)
                {
                    c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                }

                table[n] = c;
            }

            return table;
        }

        public static uint Update(uint crc, ReadOnlySpan<byte> data)
        {
            foreach (byte b in data)
            {
                crc = Table[(crc ^ b) & 0xFF] ^ (crc >> 8);
            }

            return crc;
        }
    }

    /// <summary>
    /// A small deflate (RFC 1951) in a zlib stream (RFC 1950): one block of fixed Huffman codes, greedy LZ77 matching on a
    /// hash of three bytes with a bounded chain. Plain integer arithmetic, so the same input always gives the same bytes.
    /// </summary>
    internal static class Deflater
    {
        private const int WindowSize = 32768, HashBits = 15, MaxChain = 32, MinMatch = 3, MaxMatch = 258;

        private static readonly int[] LengthBase = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
        private static readonly int[] LengthExtra = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
        private static readonly int[] DistanceBase = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];
        private static readonly int[] DistanceExtra = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];

        public static byte[] ZLib(byte[] data)
        {
            var bits = new BitWriter(data.Length / 4 + 64);
            bits.WriteByte(0x78);
            bits.WriteByte(0x01);
            bits.Write(1, 1); // the final block
            bits.Write(1, 2); // fixed Huffman codes

            int[] head = new int[1 << HashBits];
            Array.Fill(head, -1);
            int[] previous = new int[WindowSize];
            int pos = 0;
            while (pos < data.Length)
            {
                int bestLength = 0, bestDistance = 0;
                if (pos + MinMatch <= data.Length)
                {
                    int hash = Hash(data, pos);
                    int candidate = head[hash];
                    int maxLength = Math.Min(MaxMatch, data.Length - pos);
                    for (int chain = 0; candidate >= 0 && pos - candidate <= WindowSize && chain < MaxChain; chain++)
                    {
                        if (data[candidate + bestLength] == data[pos + bestLength])
                        {
                            // How far the bytes agree, compared many at a time; a candidate is before pos, so its bytes are there
                            int length = data.AsSpan(candidate, maxLength).CommonPrefixLength(data.AsSpan(pos, maxLength));
                            if (length > bestLength)
                            {
                                bestLength = length;
                                bestDistance = pos - candidate;
                                if (length == maxLength)
                                {
                                    break;
                                }
                            }
                        }

                        int next = previous[candidate % WindowSize];
                        if (next >= candidate)
                        {
                            break;
                        }

                        candidate = next;
                    }
                }

                int advance = bestLength >= MinMatch ? bestLength : 1;
                if (bestLength >= MinMatch)
                {
                    WriteLength(bits, bestLength);
                    WriteDistance(bits, bestDistance);
                }
                else
                {
                    WriteLiteral(bits, data[pos]);
                }

                for (int i = 0; i < advance; i++, pos++)
                {
                    if (pos + MinMatch <= data.Length)
                    {
                        int hash = Hash(data, pos);
                        previous[pos % WindowSize] = head[hash];
                        head[hash] = pos;
                    }
                }
            }

            WriteCode(bits, 256); // the end of the block
            bits.Align();
            uint adler = Adler32(data);
            bits.WriteByte((byte)(adler >> 24));
            bits.WriteByte((byte)(adler >> 16));
            bits.WriteByte((byte)(adler >> 8));
            bits.WriteByte((byte)adler);
            return bits.ToArray();
        }

        private static int Hash(byte[] data, int pos)
            => ((data[pos] << 10) ^ (data[pos + 1] << 5) ^ data[pos + 2]) & ((1 << HashBits) - 1);

        private static void WriteLiteral(BitWriter bits, int literal) => WriteCode(bits, literal);

        // The fixed Huffman table's codes, their bits reversed as they're written (least significant first), and their lengths;
        // and each length's and each distance's code, found once rather than by a search for each match
        private static readonly (int Code, int Bits)[] Codes = MakeCodes();
        private static readonly byte[] LengthCodes = MakeCodes(LengthBase, MaxMatch);
        private static readonly byte[] DistanceCodes = MakeCodes(DistanceBase, WindowSize);
        private static readonly int[] ReversedDistanceCodes = [.. Enumerable.Range(0, DistanceBase.Length).Select(code => Reversed(code, 5))];

        private static (int, int)[] MakeCodes()
        {
            var codes = new (int, int)[288];
            for (int symbol = 0; symbol < codes.Length; symbol++)
            {
                (int code, int bits) = symbol < 144 ? (0x30 + symbol, 8) : symbol < 256 ? (0x190 + symbol - 144, 9)
                    : symbol < 280 ? (symbol - 256, 7) : (0xC0 + symbol - 280, 8);
                codes[symbol] = (Reversed(code, bits), bits);
            }

            return codes;
        }

        // For each value up to the largest, the index of the last base no larger than it
        private static byte[] MakeCodes(int[] bases, int largest)
        {
            var codes = new byte[largest + 1];
            for (int value = bases[0]; value <= largest; value++)
            {
                int code = bases.Length - 1;
                while (bases[code] > value)
                {
                    code--;
                }

                codes[value] = (byte)code;
            }

            return codes;
        }

        private static int Reversed(int code, int count)
        {
            int reversed = 0;
            for (int i = 0; i < count; i++)
            {
                reversed = (reversed << 1) | ((code >> i) & 1);
            }

            return reversed;
        }

        // A literal or length code of the fixed Huffman table, most significant bit first
        private static void WriteCode(BitWriter bits, int symbol)
        {
            (int code, int count) = Codes[symbol];
            bits.Write(code, count);
        }

        private static void WriteLength(BitWriter bits, int length)
        {
            int code = LengthCodes[length];
            WriteCode(bits, 257 + code);
            bits.Write(length - LengthBase[code], LengthExtra[code]);
        }

        private static void WriteDistance(BitWriter bits, int distance)
        {
            int code = DistanceCodes[distance];
            bits.Write(ReversedDistanceCodes[code], 5);
            bits.Write(distance - DistanceBase[code], DistanceExtra[code]);
        }

        // Sixteen bytes at a time: over sixteen bytes x0 to x15, a grows by their sum and b by 16a + 16 x0 + 15 x1 + ... + x15,
        // which is what adding them one by one gives; 5552 is a multiple of 16, so the sums are reduced where they were
        private static readonly Vector128<ushort> FirstWeights = Vector128.Create((ushort)16, 15, 14, 13, 12, 11, 10, 9);
        private static readonly Vector128<ushort> LastWeights = Vector128.Create((ushort)8, 7, 6, 5, 4, 3, 2, 1);

        private static uint Adler32(byte[] data)
        {
            uint a = 1, b = 0;
            int i = 0;
            while (i < data.Length)
            {
                int end = Math.Min(data.Length, i + 5552);
                for (; i + 16 <= end; i += 16)
                {
                    (Vector128<ushort> first, Vector128<ushort> last) = Vector128.Widen(Vector128.Create(data.AsSpan(i, 16)));
                    b += 16 * a + Vector128.Sum(first * FirstWeights + last * LastWeights);
                    a += Vector128.Sum(first + last);
                }

                for (; i < end; i++)
                {
                    a += data[i];
                    b += a;
                }

                a %= 65521;
                b %= 65521;
            }

            return (b << 16) | a;
        }

        private sealed class BitWriter(int capacity)
        {
            private readonly MemoryStream _bytes = new(capacity);
            private ulong _buffer;
            private int _count;

            public void Write(int value, int count)
            {
                if (count == 0)
                {
                    return;
                }

                _buffer |= (ulong)(uint)value << _count;
                _count += count;
                while (_count >= 8)
                {
                    _bytes.WriteByte((byte)_buffer);
                    _buffer >>= 8;
                    _count -= 8;
                }
            }

            public void Align()
            {
                if (_count > 0)
                {
                    Write(0, 8 - _count);
                }
            }

            public void WriteByte(byte value) => Write(value, 8);

            public byte[] ToArray() => _bytes.ToArray();
        }
    }
}
