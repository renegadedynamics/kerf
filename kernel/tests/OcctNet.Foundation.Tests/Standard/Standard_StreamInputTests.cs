// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
            Assert.True(atoi(aText) == Standard_Stream.Atoi(aText), aText);
        }

        var aRand = new Random(77);
        for (int i = 0; i < 2000; i++)
        {
            string aDigits = aRand.NextInt64(0, long.MaxValue).ToString(System.Globalization.CultureInfo.InvariantCulture);
            string aText = (aRand.Next(3) switch { 0 => "-", 1 => "+", _ => "" }) + aDigits[..Math.Min(aDigits.Length, aRand.Next(1, 19))];
            Assert.True(atoi(aText) == Standard_Stream.Atoi(aText), aText);
        }

        // the C string ends at its null
        Assert.Equal(12, Standard_Stream.Atoi("12\034".AsSpan()));
        Assert.Equal("ab", Standard_Stream.CString("ab\0cd".AsSpan()));
        Assert.Equal(2, Standard_Stream.Strlen("ab\0cd".AsSpan()));
        Assert.Equal(3, Standard_Stream.Strlen("abc".AsSpan()));
    }

    [Fact]
    public void A_word_goes_into_a_C_buffer()
    {
        char[] aBuffer = "xxxxxxxxxx".ToCharArray();
        var aReader = new StringReader("  ab \n cde");
        Assert.True(Standard_Stream.ReadWord(aReader, aBuffer));
        Assert.Equal("ab\0xxxxxxx", new string(aBuffer));
        Assert.True(Standard_Stream.ReadWord(aReader, aBuffer));
        Assert.Equal("cde\0xxxxxx", new string(aBuffer));
        Assert.True(Standard_Stream.Eof(aReader));
        // no word: the buffer is left as it was
        Assert.False(Standard_Stream.ReadWord(aReader, aBuffer));
        Assert.Equal("cde\0xxxxxx", new string(aBuffer));
        Assert.True(Standard_Stream.Fail(aReader));

        // nine characters fit a buffer of ten; ten don't (OCCT writes past its buffer)
        Assert.True(Standard_Stream.ReadWord(new StringReader("123456789"), aBuffer));
        Assert.Throws<Storage_StreamFormatError>(() => Standard_Stream.ReadWord(new StringReader("1234567890"), aBuffer));
    }

    [Fact]
    public void Getline_follows_Cpp()
    {
        char[] aBuffer = new char[8];
        var aReader = new StringReader("ab\n\ncdefghijk\nlast");
        Assert.True(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("ab", Standard_Stream.CString(aBuffer));
        // an empty line extracts the delimiter: not a failure
        Assert.True(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("", Standard_Stream.CString(aBuffer));
        // a line longer than 7 characters: 7 stored, the fail flag set, the rest left in the stream
        Assert.False(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("cdefghi", Standard_Stream.CString(aBuffer));
        Assert.True(Standard_Stream.Fail(aReader));
        Assert.Equal('j', (char)aReader.Peek());
        // a failed stream stores the null only
        Assert.False(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("", Standard_Stream.CString(aBuffer));
        Standard_Stream.Clear(aReader);
        Assert.True(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("jk", Standard_Stream.CString(aBuffer));
        // the last line ends with the text: end of file, not a failure
        Assert.True(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("last", Standard_Stream.CString(aBuffer));
        Assert.True(Standard_Stream.Eof(aReader));
        // nothing left: end of file and failure
        Assert.False(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));

        // exactly theCount - 1 characters then the delimiter: no failure
        aReader = new StringReader("1234567\nx");
        Assert.True(Standard_Stream.GetLine(aReader, aBuffer, 8, '\n'));
        Assert.Equal("1234567", Standard_Stream.CString(aBuffer));
        Assert.Equal('x', (char)aReader.Peek());
        Assert.Throws<ArgumentOutOfRangeException>(() => Standard_Stream.GetLine(aReader, aBuffer, 9, '\n'));
    }

    [Fact]
    public void Counts_are_checked_against_the_input_length()
    {
        var aReader = new StringReader("0123456789");
        Assert.Equal(Standard_Stream.UnknownInputLength, Standard_Stream.InputLength(aReader));
        Standard_Stream.SetInputLength(aReader, 10);
        Assert.Equal(10, Standard_Stream.InputLength(aReader));
        Standard_Stream.CheckCount(aReader, 5, 2, "items");
        Assert.Throws<Storage_StreamFormatError>(() => Standard_Stream.CheckCount(aReader, 6, 2, "items"));
        Assert.Throws<Storage_StreamFormatError>(() => Standard_Stream.CheckCount(aReader, -1, 2, "items"));
        Assert.Throws<ArgumentOutOfRangeException>(() => Standard_Stream.SetInputLength(aReader, -1));

        var aStream = new StreamReader(new MemoryStream(Encoding.ASCII.GetBytes("12345")));
        Assert.Equal(5, Standard_Stream.InputLength(aStream));
        aStream.Dispose();
        Assert.Equal(Standard_Stream.UnknownInputLength, Standard_Stream.InputLength(aStream));

        int aValue = 0;
        Standard_Stream.CheckNotFailed(aReader, "here");
        Standard_Stream.Read(new StringReader("x"), ref aValue);
        var aFailed = new StringReader("x");
        Standard_Stream.Read(aFailed, ref aValue);
        Assert.Throws<Storage_StreamFormatError>(() => Standard_Stream.CheckNotFailed(aFailed, "here"));
    }
}
