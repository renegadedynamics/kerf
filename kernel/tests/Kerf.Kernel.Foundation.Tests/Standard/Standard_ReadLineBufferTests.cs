// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>Standard_ReadLineBuffer and Standard_IStream (unit 8H1), and Standard_Stream's FormatScientific.</summary>
public class Standard_ReadLineBufferTests
{
    private static List<string> Lines(string theText, int theBufferSize, bool theMultiline = false)
    {
        var aStream = new InputStream(new MemoryStream(Encoding.Latin1.GetBytes(theText)));
        var aBuffer = new ReadLineBuffer(theBufferSize);
        aBuffer.SetMultilineMode(theMultiline);
        var aLines = new List<string>();
        while (aBuffer.ReadLine(aStream, out Span<byte> aLine))
        {
            aLines.Add(Encoding.Latin1.GetString(aLine));
        }
        return aLines;
    }

    [Theory]
    [InlineData(1024)]
    [InlineData(7)]
    [InlineData(2)]
    public void Lines_split_on_LF_and_CRLF_across_blocks(int theBufferSize)
    {
        Assert.Equal(["solid a", "", "  facet normal 0 0 1", "endsolid", "last"],
                     Lines("solid a\r\n\n  facet normal 0 0 1\nendsolid\r\nlast", theBufferSize));
        Assert.Equal(["x"], Lines("x\n", theBufferSize));
        Assert.Empty(Lines("", theBufferSize));
    }

    [Fact]
    public void The_buffer_reads_ahead_in_blocks()
    {
        var aStream = new InputStream(new MemoryStream(Encoding.ASCII.GetBytes("ab\ncd\nef\n")));
        var aBuffer = new ReadLineBuffer(4);
        Assert.True(aBuffer.ReadLine(aStream, out Span<byte> aLine, out long aRead));
        Assert.Equal("ab", Encoding.ASCII.GetString(aLine));
        Assert.Equal(4, aRead);
        Assert.Equal(4, aStream.TellG());
        Assert.True(aBuffer.ReadLine(aStream, out aLine, out aRead));
        Assert.Equal("cd", Encoding.ASCII.GetString(aLine));
        Assert.Equal(4, aRead);
        Assert.True(aStream.Good());
        Assert.True(aBuffer.ReadLine(aStream, out aLine, out aRead));
        Assert.Equal("ef", Encoding.ASCII.GetString(aLine));
        Assert.Equal(1, aRead);
        // the last block is short: the stream is at its end, failed
        Assert.True(aStream.Eof());
        Assert.True(aStream.Fail());
        Assert.False(aStream.Bad());
    }

    [Fact]
    public void Multiline_mode_joins_lines_ending_with_a_backslash()
    {
        Assert.Equal(["1/2/3  4/5/6", "7"], Lines("1/2/3 \\\n4/5/6\n7\n", 1024, true));
        Assert.Equal(["1/2/3 \\", "4/5/6"], Lines("1/2/3 \\\n4/5/6\n", 1024, false));
    }

    [Fact]
    public void The_IStream_keeps_CPlusPlus_flags()
    {
        var aStream = new InputStream(new MemoryStream([1, 2, 3, (byte)' ', (byte)'\n', (byte)'x']));
        Span<byte> aBuffer = stackalloc byte[2];
        Assert.Equal(2, aStream.Read(aBuffer));
        Assert.True(aStream.Good());
        Assert.Equal(2, aStream.TellG());
        aStream.Unget();
        Assert.Equal(1, aStream.TellG());
        aStream.SeekG(3);
        aStream.SkipWs();
        Assert.True(aStream.Good());
        Assert.Equal(5, aStream.TellG());
        // a short read: eof and fail; tellg -1; seekg clears eof only and then does nothing
        Assert.Equal(1, aStream.Read(aBuffer));
        Assert.True(aStream.Eof() && aStream.Fail());
        Assert.Equal(-1, aStream.TellG());
        aStream.SeekG(0);
        Assert.False(aStream.Eof());
        Assert.True(aStream.Fail());
        Assert.Equal(0, aStream.Read(aBuffer));
        aStream.Clear();
        aStream.SeekG(0);
        Assert.Equal(0, aStream.TellG());
        // >> std::ws at the end: eof without fail
        aStream.SeekGEnd();
        aStream.SkipWs();
        Assert.True(aStream.Eof());
        Assert.False(aStream.Fail());
    }

    [Fact]
    public void The_IStream_counts_from_the_streams_position()
    {
        var aMemory = new MemoryStream([0, 0, 7, 8]) { Position = 2 };
        var aStream = new InputStream(aMemory);
        Assert.Equal(0, aStream.TellG());
        aStream.SeekGEnd();
        Assert.Equal(2, aStream.TellG());
        aStream.SeekG(0);
        Span<byte> aBuffer = stackalloc byte[1];
        aStream.Read(aBuffer);
        Assert.Equal(7, aBuffer[0]);
        Assert.Throws<ArgumentException>(() => new InputStream(new NonSeekableStream()));
    }

    private sealed class NonSeekableStream : MemoryStream
    {
        public override bool CanSeek => false;
    }

    [Theory]
    [InlineData(1.0, "1.000000000000e+00")]
    [InlineData(-0.0, "-0.000000000000e+00")]
    [InlineData(150.0, "1.500000000000e+02")]
    [InlineData(1e300, "1.000000000000e+300")]
    [InlineData(1e-310, "1.000000000000e-310")]
    [InlineData(5e-324, "4.940656458412e-324")]
    [InlineData(0.1, "1.000000000000e-01")]
    [InlineData(123456.789, "1.234567890000e+05")]
    [InlineData(double.PositiveInfinity, "inf")]
    public void FormatScientific_writes_as_printf_e(double theValue, string theExpected)
        => Assert.Equal(theExpected, StreamUtilities.FormatScientific(theValue, 12));

    [Fact]
    public void Strtod_reads_spans()
    {
        Assert.Equal(1.5, StreamUtilities.Strtod("  1.5e0x".AsSpan(), out int anEnd));
        Assert.Equal(7, anEnd);
        Assert.Equal(0.0, StreamUtilities.Strtod("abc".AsSpan(), out anEnd));
        Assert.Equal(0, anEnd);
        Assert.Equal(double.PositiveInfinity, StreamUtilities.Strtod("3e400".AsSpan(), out _));
    }
}
