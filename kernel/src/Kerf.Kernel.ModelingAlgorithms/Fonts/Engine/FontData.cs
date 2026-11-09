// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers.Binary;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Fonts;

/// <summary>A font's data that isn't what its format says: an offset or a count past its table, a loop (unit 13D).</summary>
internal sealed class FontMalformedException(string message) : Exception(message);

/// <summary>A font's data past a <see cref="Kerf.Fonts.FontReadLimits"/> limit (unit 13D).</summary>
internal sealed class FontLimitException(string message) : Exception(message);

/// <summary>A font in a form the reader doesn't read (CFF2, bitmap-only, a Type 1 font): <c>font.unsupported</c>.</summary>
internal sealed class FontUnsupportedException(string message) : Exception(message);

/// <summary>
/// Big-endian reads from a font's bytes, every one checked against the end of the range it may read: an offset or count
/// past it is <see cref="FontMalformedException"/>, never an index out of range or a read of another table.
/// </summary>
internal readonly struct FontData
{
    private readonly ReadOnlyMemory<byte> _bytes;

    public FontData(ReadOnlyMemory<byte> bytes) => _bytes = bytes;

    public int Length => _bytes.Length;

    public ReadOnlySpan<byte> Span => _bytes.Span;

    public ReadOnlyMemory<byte> Memory => _bytes;

    /// <summary>The <paramref name="length"/> bytes at <paramref name="offset"/>, which must be inside these.</summary>
    public FontData Slice(long offset, long length, string what)
    {
        if (offset < 0 || length < 0 || offset > _bytes.Length || length > _bytes.Length - offset)
        {
            throw Malformed($"{what} (at {offset}, {length} bytes) runs past the end of its data ({_bytes.Length} bytes)");
        }

        return new FontData(_bytes.Slice((int)offset, (int)length));
    }

    /// <summary>The bytes from <paramref name="offset"/> to the end.</summary>
    public FontData From(long offset, string what) => Slice(offset, offset <= _bytes.Length ? _bytes.Length - offset : 0, what);

    public byte U8(int at)
    {
        Check(at, 1);
        return _bytes.Span[at];
    }

    public sbyte I8(int at) => (sbyte)U8(at);

    public ushort U16(int at)
    {
        Check(at, 2);
        return BinaryPrimitives.ReadUInt16BigEndian(_bytes.Span.Slice(at, 2));
    }

    public short I16(int at)
    {
        Check(at, 2);
        return BinaryPrimitives.ReadInt16BigEndian(_bytes.Span.Slice(at, 2));
    }

    public int U24(int at)
    {
        Check(at, 3);
        ReadOnlySpan<byte> s = _bytes.Span.Slice(at, 3);
        return (s[0] << 16) | (s[1] << 8) | s[2];
    }

    public uint U32(int at)
    {
        Check(at, 4);
        return BinaryPrimitives.ReadUInt32BigEndian(_bytes.Span.Slice(at, 4));
    }

    public int I32(int at)
    {
        Check(at, 4);
        return BinaryPrimitives.ReadInt32BigEndian(_bytes.Span.Slice(at, 4));
    }

    /// <summary>A 32-bit offset, which must fit an <see cref="int"/>.</summary>
    public int Offset32(int at, string what)
    {
        uint value = U32(at);
        if (value > int.MaxValue)
        {
            throw Malformed($"{what} has an offset of {value}");
        }

        return (int)value;
    }

    /// <summary>A 2.14 fixed-point number.</summary>
    public double F2Dot14(int at) => I16(at) / 16384.0;

    /// <summary>A 16.16 fixed-point number.</summary>
    public double Fixed(int at) => I32(at) / 65536.0;

    /// <summary>Checks that <paramref name="count"/> records of <paramref name="size"/> bytes from <paramref name="at"/> fit.</summary>
    public void CheckArray(int at, long count, int size, string what)
    {
        if (count < 0 || at < 0 || at > _bytes.Length || count * size > _bytes.Length - at)
        {
            throw Malformed($"{what} ({count} of {size} bytes at {at}) runs past the end of its data ({_bytes.Length} bytes)");
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Check(int at, int size)
    {
        if ((uint)at > (uint)(_bytes.Length - size) || _bytes.Length < size)
        {
            ThrowPast(at, size);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private void ThrowPast(int at, int size) =>
        throw Malformed($"a read of {size} bytes at {at} runs past the end of its data ({_bytes.Length} bytes)");

    public static FontMalformedException Malformed(string message) => new(message);
}
