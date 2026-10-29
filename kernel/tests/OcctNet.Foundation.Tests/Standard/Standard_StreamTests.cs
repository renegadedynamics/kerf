// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;
using System.Text;

namespace OcctNet.Foundation.Tests;

/// <summary>
/// Standard_Stream against the C runtime that OCCT's Windows build uses (ucrtbase.dll): the "%.*g" formatting of
/// C++'s <c>OS &lt;&lt; double</c>, and strtod.
/// </summary>
public class Standard_StreamTests
{
    // int __stdio_common_vsprintf(unsigned __int64 options, char* buffer, size_t count, const char* format,
    //                             _locale_t locale, va_list arglist); a va_list on x64 Windows points to 8-byte slots
    [DllImport("ucrtbase.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern int __stdio_common_vsprintf(ulong options, IntPtr buffer, nuint count, IntPtr format,
                                                      IntPtr locale, IntPtr arglist);

    [DllImport("ucrtbase.dll", CallingConvention = CallingConvention.Cdecl)]
    private static extern double strtod(IntPtr str, out IntPtr endptr);

    private static string CFormat(double theValue, int thePrecision)
    {
        IntPtr aFormat = Marshal.StringToHGlobalAnsi("%.*g");
        IntPtr anArgs = Marshal.AllocHGlobal(16);
        IntPtr aBuffer = Marshal.AllocHGlobal(512);
        try
        {
            Marshal.WriteInt64(anArgs, thePrecision);
            Marshal.WriteInt64(anArgs, 8, BitConverter.DoubleToInt64Bits(theValue));
            // _CRT_INTERNAL_PRINTF_STANDARD_ROUNDING: ties to even, as in programs built with a recent SDK (native
            // OCCT's output rounds so); without it the C runtime rounds ties away from zero, as older programs did
            int aLength = __stdio_common_vsprintf(1UL << 5, aBuffer, 512, aFormat, IntPtr.Zero, anArgs);
            Assert.True(aLength > 0);
            return Marshal.PtrToStringAnsi(aBuffer, aLength);
        }
        finally
        {
            Marshal.FreeHGlobal(aFormat);
            Marshal.FreeHGlobal(anArgs);
            Marshal.FreeHGlobal(aBuffer);
        }
    }

    private static (double Value, int End) CStrtod(string theText)
    {
        IntPtr aText = Marshal.StringToHGlobalAnsi(theText);
        try
        {
            double aValue = strtod(aText, out IntPtr anEnd);
            return (aValue, (int)(anEnd - aText));
        }
        finally
        {
            Marshal.FreeHGlobal(aText);
        }
    }

    private static IEnumerable<double> Values(int theCount)
    {
        var aRandom = new Random(20261029);
        double[] aSpecial =
        [
            0.0, -0.0, 1.0, -1.0, 0.1, 0.5, 1e-5, 1e-4, 9.9999949999999e-5, 99999.95, 999999.5, 1e15, 1e16, 1e17,
            123456789012345678.0, 1e100, 1e-100, 1e300, double.Epsilon, double.MaxValue, double.MinValue,
            2.2250738585072014e-308, 0.95782628522115132, -0.28734788556634538, Math.PI, Math.E, 1.0 / 3.0,
            double.PositiveInfinity, double.NegativeInfinity, double.NaN,
        ];
        foreach (double aValue in aSpecial)
        {
            yield return aValue;
        }

        // exact ties at some precision: an integer and a binary fraction
        for (int i = 0; i < 2000; i++)
        {
            double aTie = aRandom.NextInt64(1, 1L << 50) + aRandom.Next(1, 8) / 8.0;
            yield return i % 2 == 0 ? aTie : -aTie;
            yield return Math.ScaleB(aTie, aRandom.Next(-60, 60));
        }

        for (int i = 0; i < theCount; i++)
        {
            switch (i % 4)
            {
                case 0:
                    yield return aRandom.NextDouble() * 200 - 100;
                    break;
                case 1:
                    yield return (aRandom.NextDouble() * 2 - 1) * Math.Pow(10, aRandom.Next(-30, 30));
                    break;
                case 2:
                    yield return BitConverter.Int64BitsToDouble(aRandom.NextInt64());
                    break;
                default:
                    // short decimals, as CAD data often has
                    yield return Math.Round(aRandom.NextDouble() * 1000, aRandom.Next(0, 6));
                    break;
            }
        }
    }

    [Fact]
    public void FormatReal_writes_what_the_C_runtime_writes()
    {
        if (!OperatingSystem.IsWindows())
        {
            return; // compares with the Windows C runtime
        }

        int aCount = 0;
        foreach (double aValue in Values(20000))
        {
            foreach (int aPrecision in new[] { 1, 6, 15, 16, 17, 20 })
            {
                Assert.Equal(CFormat(aValue, aPrecision), Standard_Stream.FormatReal(aValue, aPrecision));
                aCount++;
            }
        }

        Assert.True(aCount > 100000);
    }

    [Fact]
    public void Strtod_reads_what_the_C_runtime_reads()
    {
        if (!OperatingSystem.IsWindows())
        {
            return; // compares with the Windows C runtime
        }

        var aTexts = new List<string>
        {
            "", " ", "abc", "1", "-1", "+2.5", ".5", "5.", ".", "-.", "1e5", "1e", "1e+", "1e-x", "1E-3", "  42abc",
            "inf", "-INF", "infinity", "Infinity!", "infx", "nan", "-nan", "nan(ind)", "nan(123)", "nanx", "1.5.5",
            "0.95782628522115132", "1e400", "-1e400", "1e-400", "4.9406564584124654e-324", "2.4703282292062328e-324",
            "123456789012345678901234567890", "0.000000000000000000000000000001234", "1,5", "--1", "+-1",
        };
        foreach (double aValue in Values(5000))
        {
            aTexts.Add(Standard_Stream.FormatReal(aValue, 17));
            aTexts.Add(Standard_Stream.FormatReal(aValue, 15));
        }

        foreach (string aText in aTexts)
        {
            (double anExpected, int anExpectedEnd) = CStrtod(aText);
            double anActual = Standard_Stream.Strtod(aText, out int anEnd);
            Assert.True(BitConverter.DoubleToInt64Bits(anExpected) == BitConverter.DoubleToInt64Bits(anActual)
                        || (double.IsNaN(anExpected) && double.IsNaN(anActual)),
                        $"\"{aText}\": C {anExpected:R}, port {anActual:R}");
            Assert.Equal(anExpectedEnd, anEnd);
        }
    }

    [Fact]
    public void Precision_is_kept_per_writer()
    {
        var aWriter = new StringWriter();
        var anOther = new StringWriter();
        Assert.Equal(6, Standard_Stream.Precision(aWriter));
        Assert.Equal(6, Standard_Stream.Precision(aWriter, 17));
        Assert.Equal(17, Standard_Stream.Precision(aWriter));
        Assert.Equal(6, Standard_Stream.Precision(anOther));

        Standard_Stream.Write(aWriter, 1.0 / 3.0);
        Standard_Stream.Write(anOther, 1.0 / 3.0);
        Assert.Equal("0.33333333333333331", aWriter.ToString());
        Assert.Equal("0.333333", anOther.ToString());
        Assert.Equal(17, Standard_Stream.Precision(aWriter, 6));

        // an exact tie rounds to even, as native OCCT writes it (229.12124633789062 in a corpus file)
        Assert.Equal("229.12124633789062", Standard_Stream.FormatReal(229.121246337890625, 17));
        Assert.Equal("894716758318707.2", Standard_Stream.FormatReal(894716758318707.25, 16));
    }

    [Fact]
    public void Extraction_follows_Cpp_streams()
    {
        var aReader = new StringReader("  12 -7 +3 1.5 0 1 2 word  rest");
        int anInt = -1;
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(12, anInt);
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(-7, anInt);
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(3, anInt);

        // an integer stops at the decimal point, which the next word read starts with
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(1, anInt);
        Assert.Equal(".5", Standard_Stream.ReadWord(aReader));

        bool aBool = true;
        Standard_Stream.Read(aReader, ref aBool);
        Assert.False(aBool);
        Standard_Stream.Read(aReader, ref aBool);
        Assert.True(aBool);
        Assert.True(Standard_Stream.Good(aReader));

        // 2 is not a boolean: true, and the stream fails
        aBool = false;
        Standard_Stream.Read(aReader, ref aBool);
        Assert.True(aBool);
        Assert.True(Standard_Stream.Fail(aReader));

        // a failed stream extracts nothing and leaves the value as it was
        anInt = 99;
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(99, anInt);
        Assert.Null(Standard_Stream.ReadWord(aReader));

        Standard_Stream.Clear(aReader);
        Assert.Equal("word", Standard_Stream.ReadWord(aReader));

        // a word that isn't a number: 0, and the stream fails
        anInt = 99;
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(0, anInt);
        Assert.True(Standard_Stream.Fail(aReader));
    }

    [Fact]
    public void Extraction_at_the_end_sets_eof()
    {
        var aReader = new StringReader("abc 5");
        Assert.Equal("ab", Standard_Stream.ReadWord(aReader, 3));
        Assert.Equal("c", Standard_Stream.ReadWord(aReader));
        int anInt = 0;
        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(5, anInt);
        Assert.True(Standard_Stream.Eof(aReader));
        Assert.False(Standard_Stream.Fail(aReader));

        Standard_Stream.Read(aReader, ref anInt);
        Assert.Equal(5, anInt);
        Assert.True(Standard_Stream.Fail(aReader));

        var anOther = new StringReader("2147483648 -2147483649");
        Standard_Stream.Read(anOther, ref anInt);
        Assert.Equal(int.MaxValue, anInt);
        Assert.True(Standard_Stream.Fail(anOther));
    }

    [Fact]
    public void Setw_pads_on_the_left()
    {
        Assert.Equal("   7", Standard_Stream.Setw(4, "7"));
        Assert.Equal("12345", Standard_Stream.Setw(4, "12345"));
        Assert.Equal(Encoding.ASCII.GetBytes(" 3"), Encoding.ASCII.GetBytes(Standard_Stream.Setw(2, "3")));
    }
}
