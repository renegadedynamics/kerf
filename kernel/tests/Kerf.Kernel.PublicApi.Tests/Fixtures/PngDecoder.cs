// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers.Binary;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// A small PNG reader for the tests, independent of the writer: the signature, the chunks and their CRC-32s, IHDR,
/// IDAT inflated by .NET's zlib, and the five filters undone. It reads 8-bit RGBA without interlacing only, as Kerf
/// writes, and fails on anything else.
/// </summary>
internal static class PngDecoder
{
    public sealed record Decoded(int Width, int Height, byte[] Rgba, string[] Chunks)
    {
        public (byte R, byte G, byte B, byte A) Pixel(int x, int y)
        {
            int i = 4 * (y * Width + x);
            return (Rgba[i], Rgba[i + 1], Rgba[i + 2], Rgba[i + 3]);
        }
    }

    public static Decoded Decode(byte[] png)
    {
        byte[] signature = [0x89, (byte)'P', (byte)'N', (byte)'G', 0x0D, 0x0A, 0x1A, 0x0A];
        if (!png.AsSpan(0, 8).SequenceEqual(signature))
        {
            throw new InvalidDataException("Not a PNG signature.");
        }

        int pos = 8, width = 0, height = 0;
        var chunks = new List<string>();
        using var idat = new MemoryStream();
        while (pos < png.Length)
        {
            int length = BinaryPrimitives.ReadInt32BigEndian(png.AsSpan(pos));
            string type = Encoding.ASCII.GetString(png, pos + 4, 4);
            ReadOnlySpan<byte> data = png.AsSpan(pos + 8, length);
            uint crc = BinaryPrimitives.ReadUInt32BigEndian(png.AsSpan(pos + 8 + length));
            if (Crc(png.AsSpan(pos + 4, 4 + length)) != crc)
            {
                throw new InvalidDataException($"Bad CRC in {type}.");
            }

            chunks.Add(type);
            switch (type)
            {
                case "IHDR":
                    width = BinaryPrimitives.ReadInt32BigEndian(data);
                    height = BinaryPrimitives.ReadInt32BigEndian(data[4..]);
                    if (data[8] != 8 || data[9] != 6 || data[10] != 0 || data[11] != 0 || data[12] != 0)
                    {
                        throw new InvalidDataException("Not 8-bit RGBA without interlacing.");
                    }

                    break;
                case "IDAT":
                    idat.Write(data);
                    break;
            }

            pos += 12 + length;
            if (type == "IEND")
            {
                break;
            }
        }

        idat.Position = 0;
        using var inflated = new MemoryStream();
        using (var zlib = new ZLibStream(idat, CompressionMode.Decompress))
        {
            zlib.CopyTo(inflated);
        }

        byte[] raw = inflated.ToArray();
        int stride = 4 * width;
        if (raw.Length != (stride + 1) * height)
        {
            throw new InvalidDataException("The image data has the wrong size.");
        }

        byte[] rgba = new byte[stride * height];
        for (int y = 0; y < height; y++)
        {
            int filter = raw[y * (stride + 1)];
            for (int i = 0; i < stride; i++)
            {
                int a = i >= 4 ? rgba[y * stride + i - 4] : 0, b = y > 0 ? rgba[(y - 1) * stride + i] : 0;
                int c = i >= 4 && y > 0 ? rgba[(y - 1) * stride + i - 4] : 0;
                int predictor = filter switch
                {
                    0 => 0,
                    1 => a,
                    2 => b,
                    3 => (a + b) / 2,
                    4 => Paeth(a, b, c),
                    _ => throw new InvalidDataException("Unknown filter."),
                };
                rgba[y * stride + i] = (byte)(raw[y * (stride + 1) + 1 + i] + predictor);
            }
        }

        return new Decoded(width, height, rgba, [.. chunks]);
    }

    private static int Paeth(int a, int b, int c)
    {
        int p = a + b - c, pa = Math.Abs(p - a), pb = Math.Abs(p - b), pc = Math.Abs(p - c);
        return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
    }

    private static uint Crc(ReadOnlySpan<byte> data)
    {
        uint crc = 0xFFFFFFFF;
        foreach (byte d in data)
        {
            crc ^= d;
            for (int k = 0; k < 8; k++)
            {
                crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
            }
        }

        return crc ^ 0xFFFFFFFF;
    }
}
