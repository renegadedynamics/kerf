// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Drawing;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Kerf.Drawing's PNG writer, read back by the tests' own decoder (CRCs checked, .NET's zlib inflating).</summary>
public class PngTests
{
    private static RgbaImage Noise(int width, int height, int seed)
    {
        var random = new Random(seed);
        byte[] pixels = new byte[4 * width * height];
        random.NextBytes(pixels);
        // Runs and repeats too, for the matcher's long and distant matches
        for (int i = 0; i < pixels.Length / 3; i++)
        {
            pixels[i] = (byte)(i % 7);
        }

        return new RgbaImage(width, height, pixels);
    }

    [Theory]
    [InlineData(1, 1, PngCompression.Deterministic)]
    [InlineData(37, 23, PngCompression.Deterministic)]
    [InlineData(300, 200, PngCompression.Deterministic)]
    [InlineData(300, 200, PngCompression.Smallest)]
    public void A_picture_reads_back_the_same(int width, int height, PngCompression compression)
    {
        RgbaImage image = Noise(width, height, width + height);
        byte[] png = Png.Encode(image, new PngOptions { Compression = compression });
        PngDecoder.Decoded decoded = PngDecoder.Decode(png);
        Assert.Equal(["IHDR", "IDAT", "IEND"], decoded.Chunks);
        Assert.Equal((width, height), (decoded.Width, decoded.Height));
        Assert.Equal(image.Pixels.ToArray(), decoded.Rgba);
    }

    [Fact]
    public void A_render_reads_back_the_same_and_compresses()
    {
        RgbaImage image = Fixtures.Cylinder().Render(Drawing.Camera.Isometric).Value;
        byte[] png = Png.Encode(image);
        Assert.Equal(image.Pixels.ToArray(), PngDecoder.Decode(png).Rgba);
        Assert.True(png.Length < image.Pixels.Length / 20, $"{png.Length} bytes");
        var stream = new MemoryStream();
        Assert.True(Png.Write(image, stream).IsSuccess);
        Assert.Equal(png, stream.ToArray());
        Assert.Equal(Png.WriteFailed, Png.Write(image, new MemoryStream([], writable: false)).Error!.Code);
    }

    // Pictures that use each filter: noise, flat colour, gradients across and down, stripes, and a render
    public static TheoryData<string, int, int> Pictures() => new()
    {
        { "noise", 1, 1 }, { "noise", 3, 2 }, { "noise", 4, 4 }, { "noise", 5, 7 }, { "noise", 37, 23 }, { "noise", 300, 200 },
        { "flat", 1, 3 }, { "flat", 64, 64 }, { "across", 61, 17 }, { "down", 33, 70 }, { "diagonal", 129, 65 },
        { "stripes", 100, 50 }, { "render", 0, 0 },
    };

    private static RgbaImage Picture(string kind, int width, int height)
    {
        if (kind == "noise")
        {
            return Noise(width, height, width * 31 + height);
        }

        if (kind == "render")
        {
            return Fixtures.Cylinder().Render(Drawing.Camera.Isometric).Value;
        }

        byte[] pixels = new byte[4 * width * height];
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int p = 4 * (y * width + x);
                for (int k = 0; k < 4; k++)
                {
                    pixels[p + k] = kind switch
                    {
                        "flat" => (byte)(40 + 50 * k),
                        "across" => (byte)(x * (k + 3) + k),
                        "down" => (byte)(y * (5 - k)),
                        "diagonal" => (byte)((x + 2 * y) * (k + 1)),
                        _ => (byte)((x / 3 + y / 5) % 2 == 0 ? 255 - 60 * k : 7 * k),
                    };
                }
            }
        }

        return new RgbaImage(width, height, pixels);
    }

    [Theory]
    [MemberData(nameof(Pictures))]
    public void Deterministic_compression_gives_the_bytes_it_did_byte_by_byte(string kind, int width, int height)
    {
        // Unit 14A: the filters and the matcher work sixteen bytes at a time, and must choose and write exactly as before
        RgbaImage image = Picture(kind, width, height);
        Assert.Equal(ReferencePng.Encode(image), Png.Encode(image));
    }

    [Fact]
    public void Deterministic_compression_gives_known_bytes()
    {
        // A 2 × 1 picture, red and transparent: the bytes as written (checked once with Python's zlib: CRCs, and the data
        // inflating to filter 0 and the two pixels)
        byte[] png = Png.Encode(new RgbaImage(2, 1, [255, 0, 0, 255, 0, 0, 0, 0]));
        Assert.Equal("89504E470D0A1A0A0000000D4948445200000002000000010806000000F4227F8A0000000E49444154780163F8CF00420C0C000CFC01FF773B9E950000000049454E44AE426082",
                     Convert.ToHexString(png));
        KernelAssert.Throws<ArgumentException>(() => new RgbaImage(2, 2, new byte[15]));
    }
}
