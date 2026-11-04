// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Drawing;

/// <summary>
/// A picture: 8-bit red, green, blue and alpha per pixel (sRGB, alpha not premultiplied), row by row from the top left,
/// as <see cref="Renderer"/> makes it and <see cref="Png"/> writes it.
/// </summary>
public sealed class RgbaImage
{
    private readonly byte[] _pixels;

    /// <summary>A picture of the given size from its pixels, four bytes each, which are copied.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A size isn't positive.</exception>
    /// <exception cref="ArgumentException">There aren't 4 × width × height bytes.</exception>
    public RgbaImage(int width, int height, ReadOnlySpan<byte> pixels)
        : this(width, height, Check(width, height, pixels).ToArray())
    {
    }

    internal RgbaImage(int width, int height, byte[] pixels)
    {
        Width = width;
        Height = height;
        _pixels = pixels;
    }

    /// <summary>The width in pixels.</summary>
    public int Width { get; }

    /// <summary>The height in pixels.</summary>
    public int Height { get; }

    /// <summary>The pixels: red, green, blue and alpha, row by row from the top left.</summary>
    public ReadOnlyMemory<byte> Pixels => _pixels;

    /// <summary>The pixel at column <paramref name="x"/> and row <paramref name="y"/> (from the top).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The pixel is outside the picture.</exception>
    public (byte R, byte G, byte B, byte A) GetPixel(int x, int y)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(x);
        ArgumentOutOfRangeException.ThrowIfNegative(y);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(x, Width);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(y, Height);
        int i = 4 * (y * Width + x);
        return (_pixels[i], _pixels[i + 1], _pixels[i + 2], _pixels[i + 3]);
    }

    private static ReadOnlySpan<byte> Check(int width, int height, ReadOnlySpan<byte> pixels)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(height);
        if ((long)width * height * 4 != pixels.Length)
        {
            throw new ArgumentException("There must be four bytes a pixel.", nameof(pixels));
        }

        return pixels;
    }
}
