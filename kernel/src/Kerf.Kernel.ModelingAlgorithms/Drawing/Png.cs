// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Buffers.Binary;
using System.IO.Compression;
using System.Runtime.CompilerServices;
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

    /// <summary>The compression.</summary>
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
        ArgumentNullException.ThrowIfNull(stream);
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

    /// <summary>The picture as PNG bytes.</summary>
    /// <exception cref="ArgumentNullException">The picture is null.</exception>
    public static byte[] Encode(RgbaImage image, PngOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(image);
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

    private static long CostOfNone(ReadOnlySpan<byte> row)
    {
        long cost = 0;
        foreach (byte value in row)
        {
            cost += Cost(value);
        }

        return cost;
    }

    private static long CostOfSub(ReadOnlySpan<byte> row, long limit)
    {
        long cost = 0;
        for (int i = 0; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - (i >= 4 ? row[i - 4] : 0));
        }

        return cost;
    }

    private static long CostOfUp(ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, long limit)
    {
        long cost = 0;
        for (int i = 0; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - up[i]);
        }

        return cost;
    }

    private static long CostOfAverage(ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, long limit)
    {
        long cost = 0;
        for (int i = 0; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - (((i >= 4 ? row[i - 4] : 0) + up[i]) >> 1));
        }

        return cost;
    }

    private static long CostOfPaeth(ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, long limit)
    {
        long cost = 0;
        for (int i = 0; i < row.Length && cost < limit; i++)
        {
            cost += Cost(row[i] - (i >= 4 ? Paeth(row[i - 4], up[i], up[i - 4]) : Paeth(0, up[i], 0)));
        }

        return cost;
    }

    private static void Apply(int filter, ReadOnlySpan<byte> row, ReadOnlySpan<byte> up, Span<byte> target)
    {
        for (int i = 0; i < row.Length; i++)
        {
            int a = i >= 4 ? row[i - 4] : 0, b = up[i], c = i >= 4 ? up[i - 4] : 0;
            int predictor = filter switch
            {
                0 => 0,
                1 => a,
                2 => b,
                3 => (a + b) >> 1,
                _ => Paeth(a, b, c),
            };
            target[i] = (byte)(row[i] - predictor);
        }
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
                            int length = 0;
                            while (length < maxLength && data[candidate + length] == data[pos + length])
                            {
                                length++;
                            }

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

        // A literal or length code of the fixed Huffman table, most significant bit first
        private static void WriteCode(BitWriter bits, int symbol)
        {
            if (symbol < 144)
            {
                bits.WriteReversed(0x30 + symbol, 8);
            }
            else if (symbol < 256)
            {
                bits.WriteReversed(0x190 + symbol - 144, 9);
            }
            else if (symbol < 280)
            {
                bits.WriteReversed(symbol - 256, 7);
            }
            else
            {
                bits.WriteReversed(0xC0 + symbol - 280, 8);
            }
        }

        private static void WriteLength(BitWriter bits, int length)
        {
            int code = LengthBase.Length - 1;
            while (LengthBase[code] > length)
            {
                code--;
            }

            WriteCode(bits, 257 + code);
            bits.Write(length - LengthBase[code], LengthExtra[code]);
        }

        private static void WriteDistance(BitWriter bits, int distance)
        {
            int code = DistanceBase.Length - 1;
            while (DistanceBase[code] > distance)
            {
                code--;
            }

            bits.WriteReversed(code, 5);
            bits.Write(distance - DistanceBase[code], DistanceExtra[code]);
        }

        private static uint Adler32(byte[] data)
        {
            uint a = 1, b = 0;
            int i = 0;
            while (i < data.Length)
            {
                int end = Math.Min(data.Length, i + 5552);
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

            public void WriteReversed(int code, int count)
            {
                int reversed = 0;
                for (int i = 0; i < count; i++)
                {
                    reversed = (reversed << 1) | ((code >> i) & 1);
                }

                Write(reversed, count);
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
