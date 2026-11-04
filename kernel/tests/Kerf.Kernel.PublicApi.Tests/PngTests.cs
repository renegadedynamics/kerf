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

    [Fact]
    public void Deterministic_compression_gives_known_bytes()
    {
        // A 2 × 1 picture, red and transparent: the bytes as written (checked once with Python's zlib: CRCs, and the data
        // inflating to filter 0 and the two pixels)
        byte[] png = Png.Encode(new RgbaImage(2, 1, [255, 0, 0, 255, 0, 0, 0, 0]));
        Assert.Equal("89504E470D0A1A0A0000000D4948445200000002000000010806000000F4227F8A0000000E49444154780163F8CF00420C0C000CFC01FF773B9E950000000049454E44AE426082",
                     Convert.ToHexString(png));
        Assert.Throws<ArgumentException>(() => new RgbaImage(2, 2, new byte[15]));
    }
}
