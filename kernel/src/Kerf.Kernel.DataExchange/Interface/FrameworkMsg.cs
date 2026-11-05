// Port of: src/DataExchange/TKXSBase/Interface/Interface_MSG.hxx (the static utilities: Intervalled, TDate, NDate,
//          CDate, Blanks, Print)
// Port of: src/DataExchange/TKXSBase/Interface/Interface_MSG.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Utilities for messages: rounding of reals to "round" values, dates as text, and blanks for aligned printing.
/// </summary>
/// <remarks>
/// OCCT's class is also a message object translated through a process-wide dictionary of messages (its constructors,
/// <c>Value</c>, <c>Read</c>, <c>Write</c>, <c>IsKey</c>, <c>Translated</c>, <c>Record</c>, <c>SetTrace</c>,
/// <c>SetMode</c>, <c>PrintTrace</c>), which nothing in STEP, Transfer, XSControl or IFSelect uses: that part is left
/// out with its process-wide state (decision 1183). The static utilities, which they use, are ported.
/// </remarks>
internal static class FrameworkMsg
{
    private const string BlankText =
        "                                                                            ";

    private static readonly int Maxblank = BlankText.Length;

    //  ###########    FLOATING POINT ROUNDING    ############

    /// <summary>
    /// Returns an "interval" value: the value rounded to a "round" number of its decade, upwards (default) or
    /// downwards, with <paramref name="order"/> values per decade (1: 1; 2: 1, 3; 3: 1, 2, 5; 4: 1, 2, 3, 6; 5 and 6:
    /// 1, 1.5, 2, 3, 5, 7; more: 1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8).
    /// </summary>
    public static double Intervalled(double val, int order = 3, bool upper = false)
    {
        double vl = (val > 0.0 ? val : -val);
        double bl = 1.0, bu = 1.0;
        if (vl >= 1.0)
        {
            bu = 10.0;
            for (int i = 0; i < 200; i++)
            {
                if (vl < bu)
                {
                    break;
                }
                bl = bu;
                bu *= 10.0;
            }
        }
        else
        {
            bl = 0.1;
            for (int i = 0; i < 200; i++)
            {
                if (vl >= bl)
                {
                    break;
                }
                bu = bl;
                bl /= 10.0;
            }
            if (vl == 0.0)
            {
                return 0.0;
            }
        }

        double rst = vl / bl;
        if (order <= 1)
        {
            rst = (upper ? 10.0 : 1.0);
        }
        else if (order == 2)
        {
            if (rst <= 3.0)
            {
                rst = (upper ? 3.0 : 1.0);
            }
            else
            {
                rst = (upper ? 10.0 : 3.0);
            }
        }
        else if (order == 3)
        {
            if (rst <= 2.0)
            {
                rst = (upper ? 2.0 : 1.0);
            }
            else if (rst <= 5.0)
            {
                rst = (upper ? 5.0 : 2.0);
            }
            else
            {
                rst = (upper ? 10.0 : 5.0);
            }
        }
        else if (order == 4)
        {
            if (rst <= 2.0)
            {
                rst = (upper ? 2.0 : 1.0);
            }
            else if (rst <= 3.0)
            {
                rst = (upper ? 3.0 : 2.0);
            }
            else if (rst <= 6.0)
            {
                rst = (upper ? 6.0 : 3.0);
            }
            else
            {
                rst = (upper ? 10.0 : 6.0);
            }
        }
        else if (order <= 6)
        {
            if (rst <= 1.5)
            {
                rst = (upper ? 1.5 : 1.0);
            }
            else if (rst <= 2.0)
            {
                rst = (upper ? 2.0 : 1.5);
            }
            else if (rst <= 3.0)
            {
                rst = (upper ? 3.0 : 2.0);
            }
            else if (rst <= 5.0)
            {
                rst = (upper ? 5.0 : 3.0);
            }
            else if (rst <= 7.0)
            {
                rst = (upper ? 7.0 : 5.0);
            }
            else
            {
                rst = (upper ? 10.0 : 7.0);
            }
        }
        else
        { // only makes sense up to 10 ...
            if (rst <= 1.2)
            {
                rst = (upper ? 1.2 : 1.0);
            }
            else if (rst <= 1.5)
            {
                rst = (upper ? 1.5 : 1.2);
            }
            else if (rst <= 2.0)
            {
                rst = (upper ? 2.0 : 1.5);
            }
            else if (rst <= 2.5)
            {
                rst = (upper ? 2.5 : 2.0);
            }
            else if (rst <= 3.0)
            {
                rst = (upper ? 3.0 : 2.5);
            }
            else if (rst <= 4.0)
            {
                rst = (upper ? 4.0 : 3.0);
            }
            else if (rst <= 5.0)
            {
                rst = (upper ? 5.0 : 4.0);
            }
            else if (rst <= 6.0)
            {
                rst = (upper ? 6.0 : 5.0);
            }
            else if (rst <= 8.0)
            {
                rst = (upper ? 8.0 : 6.0);
            }
            else
            {
                rst = (upper ? 10.0 : 8.0);
            }
        }
        return ((val < 0.0) ? -(bl * rst) : (bl * rst));
    }

    //  ###########    DATES    ############

    /// <summary>
    /// Writes a date as text: by default "YYYY-MM-DD:HH-MN-SS"; a <paramref name="format"/> "C:..." gives a C format
    /// for the six values. Null values at the beginning, with at least the last one not null, are completed from the
    /// current (local) date.
    /// </summary>
    public static void TDate(out string text, int yy, int mm, int dd, int hh, int mn, int ss, string? format = "")
    {
        //  null values : at the beginning (with at least one non-null, the last one)
        //  -> completed with current values (system date)
        //  all null we leave

        // svv #2 int y1 , m1 , d1 , h1 , n1 , s1;
        int y2 = yy, m2 = mm, d2 = dd, h2 = hh, n2 = mn, s2 = ss;
        if (yy == 0 && ss != 0)
        {
            //  completion
            DateTime ladate = DateTime.Now; // OCCT: OSD_Process::SystemDate(), the local time
            if (yy == 0)
            {
                y2 = ladate.Year;
                if (mm == 0)
                {
                    m2 = ladate.Month;
                    if (dd == 0)
                    {
                        d2 = ladate.Day;
                        if (hh == 0)
                        {
                            h2 = ladate.Hour;
                            if (mn == 0)
                            {
                                n2 = ladate.Minute;
                                s2 = ladate.Second;
                            }
                        }
                    }
                }
            }
        }
        text = "";
        if (format is null || format.Length == 0)
        {
            text = Sprintf("%4.4d-%2.2d-%2.2d:%2.2d-%2.2d-%2.2d", y2, m2, d2, h2, n2, s2);
        }
        else if ((format[0] == 'c' || format[0] == 'C') && CStringChar(format, 1) == ':')
        {
            text = Sprintf(format.Substring(2), y2, m2, d2, h2, n2, s2);
        }
        // OCCT: any other format leaves the caller's buffer as it was (an empty text here)
    }

    /// <summary>
    /// Decodes a date written as text: the successive groups of digits are the year, month, day, hour, minute and
    /// second, added (as digits) to the given values. Returns true.
    /// </summary>
    public static bool NDate(string text, ref int yy, ref int mm, ref int dd, ref int hh, ref int mn, ref int ss)
    {
        int i, num = 1;
        for (i = 0; i < text.Length; i++)
        {
            char val = text[i];
            if (val >= 48 && val <= 57)
            {
                if ((num & 1) == 0)
                {
                    num++;
                }
                if (num == 1)
                {
                    yy = yy * 10 + (val - 48);
                }
                if (num == 3)
                {
                    mm = mm * 10 + (val - 48);
                }
                if (num == 5)
                {
                    dd = dd * 10 + (val - 48);
                }
                if (num == 7)
                {
                    hh = hh * 10 + (val - 48);
                }
                if (num == 9)
                {
                    mn = mn * 10 + (val - 48);
                }
                if (num == 11)
                {
                    ss = ss * 10 + (val - 48);
                }
            }
            else if ((num & 1) != 0)
            {
                num++;
            }
        }
        return (num > 0);
    }

    /// <summary>Compares two dates written as text: -1 if the first is earlier, 1 if later, 0 if equal.</summary>
    public static int CDate(string text1, string text2)
    {
        int i1 = 0, i2 = 0, i3 = 0, i4 = 0, i5 = 0, i6 = 0, j1 = 0, j2 = 0, j3 = 0, j4 = 0, j5 = 0, j6 = 0;
        if (!NDate(text1, ref i1, ref i2, ref i3, ref i4, ref i5, ref i6))
        {
            return 0;
        }
        if (!NDate(text2, ref j1, ref j2, ref j3, ref j4, ref j5, ref j6))
        {
            return 0;
        }
        if (i1 < j1)
        {
            return -1;
        }
        if (i1 > j1)
        {
            return 1;
        }
        if (i2 < j2)
        {
            return -1;
        }
        if (i2 > j2)
        {
            return 1;
        }
        if (i3 < j3)
        {
            return -1;
        }
        if (i3 > j3)
        {
            return 1;
        }
        if (i4 < j4)
        {
            return -1;
        }
        if (i4 > j4)
        {
            return 1;
        }
        if (i5 < j5)
        {
            return -1;
        }
        if (i5 > j5)
        {
            return 1;
        }
        if (i6 < j6)
        {
            return -1;
        }
        if (i6 > j6)
        {
            return 1;
        }
        return 0;
    }

    /// <summary>
    /// Returns blanks to complete an integer printed in a field of <paramref name="max"/> characters (right
    /// justified): max minus its count of digits (a negative value counts its sign).
    /// </summary>
    public static string Blanks(int val, int max)
    {
        int count;
        if (val < 0)
        {
            return FrameworkMsg.Blanks(-val, max - 1);
        }
        if (val < 10)
        {
            count = 9;
        }
        else if (val < 100)
        {
            count = 8;
        }
        else if (val < 1000)
        {
            count = 7;
        }
        else if (val < 10000)
        {
            count = 6;
        }
        else if (val < 100000)
        {
            count = 5;
        }
        else if (val < 1000000)
        {
            count = 4;
        }
        else if (val < 10000000)
        {
            count = 3;
        }
        else if (val < 100000000)
        {
            count = 2;
        }
        else if (val < 1000000000)
        {
            count = 1;
        }
        else
        {
            count = 0;
        }
        count = count + max - 10;
        if (count < 0)
        {
            count = 0;
        }
        return Blank(count); // OCCT: reads before its buffer of blanks beyond 76
    }

    /// <summary>Returns blanks to complete a text printed in a field of <paramref name="max"/> characters.</summary>
    public static string Blanks(string val, int max)
    {
        int lng = val.Length;
        if (lng > Maxblank || lng > max)
        {
            return "";
        }
        return Blank(max - lng); // OCCT: reads before its buffer of blanks beyond 76
    }

    /// <summary>Returns a count of blanks (at most 76).</summary>
    public static string Blanks(int count)
    {
        if (count <= 0)
        {
            return "";
        }
        if (count >= Maxblank)
        {
            return BlankText;
        }
        return Blank(count);
    }

    /// <summary>
    /// Prints a text in a field of <paramref name="max"/> characters (at most 76), justified left
    /// (<paramref name="just"/> &lt; 0), centred (0) or right (&gt; 0); a longer text is printed as it is.
    /// </summary>
    public static void Print(TextWriter s, string val, int max, int just = -1)
    {
        if (max > Maxblank)
        {
            Print(s, val, Maxblank, just);
            return;
        }
        int lng = val.Length;
        if (lng > max)
        {
            s.Write(val);
            return;
        }
        int m1 = (max - lng) / 2;
        int m2 = max - lng - m1;
        if (just < 0)
        {
            s.Write(val);
            s.Write(Blank(m1 + m2));
        }
        else if (just == 0)
        {
            s.Write(Blank(m1));
            s.Write(val);
            s.Write(Blank(m2));
        }
        else
        {
            s.Write(Blank(m1 + m2));
            s.Write(val);
        }
    }

    // &blank[maxblank - count]: the last <count> blanks of the buffer
    private static string Blank(int count) => count <= 0 ? "" : new string(' ', count);
}
