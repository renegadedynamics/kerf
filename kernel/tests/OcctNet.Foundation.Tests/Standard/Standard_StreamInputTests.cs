// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Runtime.InteropServices;
using System.Text;

namespace OcctNet.Foundation.Tests;

/// <summary>
/// Standard_Stream's C strings and lines (C++'s <c>IS &gt;&gt; char[N]</c> and <c>getline</c>, C's <c>atoi</c>) and its
/// guards for untrusted input (the reader's length, count checks).
/// </summary>
public class Standard_StreamInputTests
{
    [DllImport("ucrtbase.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi, BestFitMapping = false)]
    private static extern int atoi([MarshalAs(UnmanagedType.LPStr)] string str);

    [Fact]
    public void Atoi_is_the_C_runtimes()
    {
        string[] aTexts =
        [
            "", "0", "12", "-12", "+7", "  42abc", "\t\n-3", "abc", "-", "+", "2147483647", "2147483648", "-2147483648",
            "-2147483649", "99999999999999999999", "-99999999999999999999", "0012", "1e5", "3.7", " - 4",
        ];
        foreach (string aText in aTexts)
        {
            Assert.True(atoi(aText) == StreamUtilities.Atoi(aText), aText);
        }

        var aRand = new Random(77);
        for (int i = 0; i < 2000; i++)
        {
            string aDigits = aRand.NextInt64(0, long.MaxValue).ToString(System.Globalization.CultureInfo.InvariantCulture);
            string aText = (aRand.Next(3) switch { 0 => "-", 1 => "+", _ => "" }) + aDigits[..Math.Min(aDigits.Length, aRand.Next(1, 19))];
            Assert.True(atoi(aText) == StreamUtilities.Atoi(aText), aText);
        }

        // the C string ends at its null
        Assert.Equal(12, StreamUtilities.Atoi("12\034".AsSpan()));
        Assert.Equal("ab", StreamUtilities.CString("ab\0cd".AsSpan()));
        Assert.Equal(2, StreamUtilities.Strlen("ab\0cd".AsSpan()));
        Assert.Equal(3, StreamUtilities.Strlen("abc".AsSpan()));
    }

    [Fact]
    public void A_word_goes_into_a_C_buffer()
    {
        char[] aBuffer = "xxxxxxxxxx".ToCharArray();
        var aReader = new StringReader("  ab \n cde");
        Assert.True(StreamUtilities.ReadWord(aReader, aBuffer));
        Assert.Equal("ab\0xxxxxxx", new string(aBuffer));
        Assert.True(StreamUtilities.ReadWord(aReader, aBuffer));
        Assert.Equal("cde\0xxxxxx", new string(aBuffer));
        Assert.True(StreamUtilities.Eof(aReader));
        // no word: the buffer is left as it was
        Assert.False(StreamUtilities.ReadWord(aReader, aBuffer));
        Assert.Equal("cde\0xxxxxx", new string(aBuffer));
        Assert.True(StreamUtilities.Fail(aReader));

        // nine characters fit a buffer of ten; ten don't (OCCT writes past its buffer)
        Assert.True(StreamUtilities.ReadWord(new StringReader("123456789"), aBuffer));
        Assert.Throws<StreamFormatErrorException>(() => StreamUtilities.ReadWord(new StringReader("1234567890"), aBuffer));
    }

    [Fact]
    public void Getline_follows_Cpp()
    {
        char[] aBuffer = new char[8];
        var aReader = new StringReader("ab\n\ncdefghijk\nlast");
        Assert.True(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("ab", StreamUtilities.CString(aBuffer));
        // an empty line extracts the delimiter: not a failure
        Assert.True(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("", StreamUtilities.CString(aBuffer));
        // a line longer than 7 characters: 7 stored, the fail flag set, the rest left in the stream
        Assert.False(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("cdefghi", StreamUtilities.CString(aBuffer));
        Assert.True(StreamUtilities.Fail(aReader));
        Assert.Equal('j', (char)aReader.Peek());
        // a failed stream stores the null only
        Assert.False(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("", StreamUtilities.CString(aBuffer));
        StreamUtilities.Clear(aReader);
        Assert.True(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("jk", StreamUtilities.CString(aBuffer));
        // the last line ends with the text: end of file, not a failure
        Assert.True(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("last", StreamUtilities.CString(aBuffer));
        Assert.True(StreamUtilities.Eof(aReader));
        // nothing left: end of file and failure
        Assert.False(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));

        // exactly theCount - 1 characters then the delimiter: no failure
        aReader = new StringReader("1234567\nx");
        Assert.True(StreamUtilities.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("1234567", StreamUtilities.CString(aBuffer));
        Assert.Equal('x', (char)aReader.Peek());
        Assert.Throws<ArgumentOutOfRangeException>(() => StreamUtilities.GetLine(aReader, aBuffer, 9, '\n'));
    }

    [Fact]
    public void Counts_are_checked_against_the_input_length()
    {
        var aReader = new StringReader("0123456789");
        Assert.Equal(StreamUtilities.UnknownInputLength, StreamUtilities.InputLength(aReader));
        StreamUtilities.SetInputLength(aReader, 10);
        Assert.Equal(10, StreamUtilities.InputLength(aReader));
        StreamUtilities.CheckCount(aReader, 5, 2, "items");
        Assert.Throws<StreamFormatErrorException>(() => StreamUtilities.CheckCount(aReader, 6, 2, "items"));
        Assert.Throws<StreamFormatErrorException>(() => StreamUtilities.CheckCount(aReader, -1, 2, "items"));
        Assert.Throws<ArgumentOutOfRangeException>(() => StreamUtilities.SetInputLength(aReader, -1));

        var aStream = new StreamReader(new MemoryStream(Encoding.ASCII.GetBytes("12345")));
        Assert.Equal(5, StreamUtilities.InputLength(aStream));
        aStream.Dispose();
        Assert.Equal(StreamUtilities.UnknownInputLength, StreamUtilities.InputLength(aStream));

        int aValue = 0;
        StreamUtilities.CheckNotFailed(aReader, "here");
        StreamUtilities.Read(new StringReader("x"), ref aValue);
        var aFailed = new StringReader("x");
        StreamUtilities.Read(aFailed, ref aValue);
        Assert.Throws<StreamFormatErrorException>(() => StreamUtilities.CheckNotFailed(aFailed, "here"));
    }
}
