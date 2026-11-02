// Port of: src/DataExchange/TKXSBase/Interface/Interface_FloatWriter.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_FloatWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a floating number (real) to a string, when the standard C/C++ output functions are not convenient: to
/// suppress trailing '0' and "E+00" (if desired), and to control exponent and floating point output. Formats are
/// given in the form used by printf (and written as Microsoft's C runtime writes them).
/// </summary>
/// <remarks>
/// STEP writes its reals through this class, so it reproduces OCCT's character buffer work exactly, including its
/// limits (the zero suppression looks at the first 16 characters only).
/// </remarks>
public class Interface_FloatWriter
{
    /// <summary>
    /// Creates a FloatWriter ready to work, with default options: zero suppress option set, main format "%E",
    /// secondary format "%f" for values between 0.1 and 1000. in absolute values. If <paramref name="chars"/> is given
    /// (and positive), produces options for this count of characters: "%(chars+2).(chars)f" and "...E".
    /// </summary>
    public Interface_FloatWriter(int chars = 0)
    {
        SetDefaults(chars);
    }

    /// <summary>
    /// Sets a specific format for sending reals (main format), "%E" from creation. If <paramref name="reset"/> is true
    /// (default), clears the effects of former calls to SetFormatForRange and SetZeroSuppress.
    /// </summary>
    public void SetFormat(string form, bool reset = true)
    {
        themainform = CFormat(form);
        if (!reset)
        {
            return;
        }
        therange1 = therange2 = 0.0; // second form : inhibee
        thezerosup = false;
    }

    /// <summary>
    /// Sets a secondary format for reals, applied between <paramref name="R1"/> and <paramref name="R2"/> (in absolute
    /// values); "%f" between 0.1 and 1000. from creation. If the condition 0. &lt;= R1 &lt; R2 is not fulfilled, this
    /// secondary form is cancelled.
    /// </summary>
    public void SetFormatForRange(string form, double R1, double R2)
    {
        therangeform = CFormat(form);
        therange1 = R1;
        therange2 = R2;
    }

    /// <summary>Sets (or resets) the suppression of trailing zeros and of a null exponent ("E+00").</summary>
    public void SetZeroSuppress(bool mode)
    {
        thezerosup = mode;
    }

    /// <summary>Sets again the options to the defaults given at creation.</summary>
    public void SetDefaults(int chars = 0)
    {
        if (chars <= 0)
        {
            themainform = "%E";
            therangeform = "%f";
        }
        else
        {
            char pourcent = '%';
            char point = '.';
            themainform = CFormat(Sprintf("%c%d%c%dE", pourcent, chars + 2, point, chars));
            therangeform = CFormat(Sprintf("%c%d%c%df", pourcent, chars + 2, point, chars));
        }
        therange1 = 0.1;
        therange2 = 1000.0;
        thezerosup = true;
    }

    /// <summary>
    /// Returns the active options: <paramref name="zerosup"/> the zero suppression, <paramref name="range"/> true if a
    /// range is set (<paramref name="R1"/>, <paramref name="R2"/>).
    /// </summary>
    public void Options(out bool zerosup, out bool range, out double R1, out double R2)
    {
        zerosup = thezerosup;
        range = (therange2 >= therange1 && therange1 >= 0.0);
        R1 = therange1;
        R2 = therange2;
    }

    /// <summary>Returns the main format.</summary>
    public string MainFormat() => themainform;

    /// <summary>Returns the format for range (meaningful only if a range is set).</summary>
    public string FormatForRange() => therangeform;

    /// <summary>
    /// Writes a real to a string by using the options; returns the useful length of the string. Calls
    /// <see cref="Convert"/>.
    /// </summary>
    public int Write(double val, out string text) =>
        Convert(val, out text, thezerosup, therange1, therange2, themainform, therangeform);

    /// <summary>
    /// Converts a real to a string, given options as arguments: the range format between <paramref name="R1"/> and
    /// <paramref name="R2"/> in absolute values, the main format else, then (if <paramref name="zsup"/>) trailing
    /// zeros and a null exponent removed. Returns the length of the string.
    /// </summary>
    public static int Convert(double val, out string text, bool zsup, double R1, double R2, string mainform,
                              string rangeform)
    {
        //    Float value, purged of trailing "0000" and "E+00"
        const int anMasSize = 5; // change 6 to 5: index 5 is not used below
        Span<char> lxp = stackalloc char[anMasSize];
        int i0 = 0, j0 = 0;

        for (int i = 0; i < anMasSize; ++i)
        {
            lxp[i] = '\0';
        }

        // OCCT writes into the caller's buffer (20 to 30 characters); a buffer of the formatted length plus the
        // zero suppression's reach stands for it here
        string aFormatted;
        if ((val >= R1 && val < R2) || (val <= -R1 && val > -R2))
        {
            aFormatted = Sprintf(rangeform, val);
        }
        else
        {
            aFormatted = Sprintf(mainform, val);
        }
        char[] pText = new char[Math.Max(aFormatted.Length, 16) + 8];
        aFormatted.CopyTo(0, pText, 0, aFormatted.Length);

        if (zsup)
        {
            for (int i = 0; i < 16; i++)
            {
                i0 = i;
                if (pText[i] == 'e' || pText[i] == 'E')
                {
                    lxp[0] = 'E';
                    lxp[1] = pText[i + 1];
                    lxp[2] = pText[i + 2];
                    lxp[3] = pText[i + 3];
                    lxp[4] = pText[i + 4];

                    if (lxp[1] == '+' && lxp[2] == '0' && lxp[3] == '0' && lxp[4] == '\0')
                    {
                        lxp[0] = '\0';
                    }

                    pText[i] = '\0';
                }
                if (pText[i] == '\0')
                {
                    break;
                }
            }
            // #52 rln 23.12.98 converting 1e-07 throws exception
            for (int j = i0 - 1; j >= 0; j--)
            {
                j0 = j;
                if (pText[j] != '0')
                {
                    break;
                }
                pText[j] = '\0';
            }
            pText[j0 + 1] = lxp[0];
            pText[j0 + 2] = lxp[1];
            pText[j0 + 3] = lxp[2];
            pText[j0 + 4] = lxp[3];
            pText[j0 + 5] = lxp[4];
            pText[j0 + 6] = '\0';
        }
        int aLength = Array.IndexOf(pText, '\0');
        text = new string(pText, 0, aLength < 0 ? pText.Length : aLength);
        return text.Length; // strlen(text)
    }

    // OCCT keeps the formats in char[12] buffers, copied with strcpy (no check of the length)
    private static string CFormat(string theForm) => theForm;

    private string themainform = "";
    private double therange1;
    private double therange2;
    private string therangeform = "";
    private bool thezerosup;
}
