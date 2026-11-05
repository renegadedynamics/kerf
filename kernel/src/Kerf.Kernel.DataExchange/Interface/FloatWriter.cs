// Port of: src/DataExchange/TKXSBase/Interface/Interface_FloatWriter.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_FloatWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Converts a floating number (real) to a string, when the standard C/C++ output functions are not convenient: to
/// suppress trailing '0' and "E+00" (if desired), and to control exponent and floating point output. Formats are
/// given in the form used by printf (and written as Microsoft's C runtime writes them).
/// </summary>
/// <remarks>
/// STEP writes its reals through this class, so it reproduces OCCT's character buffer work exactly, including its
/// limits (the zero suppression looks at the first 16 characters only).
/// </remarks>
internal class FloatWriter
{
    /// <summary>
    /// Creates a FloatWriter ready to work, with default options: zero suppress option set, main format "%E",
    /// secondary format "%f" for values between 0.1 and 1000. in absolute values. If <paramref name="chars"/> is given
    /// (and positive), produces options for this count of characters: "%(chars+2).(chars)f" and "...E".
    /// </summary>
    public FloatWriter(int chars = 0)
    {
        SetDefaults(chars);
    }

    /// <summary>
    /// Sets a specific format for sending reals (main format), "%E" from creation. If <paramref name="reset"/> is true
    /// (default), clears the effects of former calls to SetFormatForRange and SetZeroSuppress.
    /// </summary>
    public void SetFormat(string form, bool reset = true)
    {
        _themainform = CFormat(form);
        if (!reset)
        {
            return;
        }
        _therange1 = _therange2 = 0.0; // second form : inhibee
        _thezerosup = false;
    }

    /// <summary>
    /// Sets a secondary format for reals, applied between <paramref name="r1"/> and <paramref name="r2"/> (in absolute
    /// values); "%f" between 0.1 and 1000. from creation. If the condition 0. &lt;= R1 &lt; R2 is not fulfilled, this
    /// secondary form is cancelled.
    /// </summary>
    public void SetFormatForRange(string form, double r1, double r2)
    {
        _therangeform = CFormat(form);
        _therange1 = r1;
        _therange2 = r2;
    }

    /// <summary>Sets (or resets) the suppression of trailing zeros and of a null exponent ("E+00").</summary>
    public void SetZeroSuppress(bool mode)
    {
        _thezerosup = mode;
    }

    /// <summary>Sets again the options to the defaults given at creation.</summary>
    public void SetDefaults(int chars = 0)
    {
        if (chars <= 0)
        {
            _themainform = "%E";
            _therangeform = "%f";
        }
        else
        {
            char pourcent = '%';
            char point = '.';
            _themainform = CFormat(Sprintf("%c%d%c%dE", pourcent, chars + 2, point, chars));
            _therangeform = CFormat(Sprintf("%c%d%c%df", pourcent, chars + 2, point, chars));
        }
        _therange1 = 0.1;
        _therange2 = 1000.0;
        _thezerosup = true;
    }

    /// <summary>
    /// Returns the active options: <paramref name="zerosup"/> the zero suppression, <paramref name="range"/> true if a
    /// range is set (<paramref name="r1"/>, <paramref name="r2"/>).
    /// </summary>
    public void Options(out bool zerosup, out bool range, out double r1, out double r2)
    {
        zerosup = _thezerosup;
        range = (_therange2 >= _therange1 && _therange1 >= 0.0);
        r1 = _therange1;
        r2 = _therange2;
    }

    /// <summary>Returns the main format.</summary>
    public string MainFormat() => _themainform;

    /// <summary>Returns the format for range (meaningful only if a range is set).</summary>
    public string FormatForRange() => _therangeform;

    /// <summary>
    /// Writes a real to a string by using the options; returns the useful length of the string. Calls
    /// <see cref="Convert"/>.
    /// </summary>
    public int Write(double val, out string text) =>
        Convert(val, out text, _thezerosup, _therange1, _therange2, _themainform, _therangeform);

    /// <summary>
    /// Converts a real to a string, given options as arguments: the range format between <paramref name="r1"/> and
    /// <paramref name="r2"/> in absolute values, the main format else, then (if <paramref name="zsup"/>) trailing
    /// zeros and a null exponent removed. Returns the length of the string.
    /// </summary>
    public static int Convert(double val, out string text, bool zsup, double r1, double r2, string mainform,
                              string rangeform)
    {
        //    Float value, purged of trailing "0000" and "E+00"
        const int masSize = 5; // change 6 to 5: index 5 is not used below
        Span<char> lxp = stackalloc char[masSize];
        int i0 = 0, j0 = 0;

        for (int i = 0; i < masSize; ++i)
        {
            lxp[i] = '\0';
        }

        // OCCT writes into the caller's buffer (20 to 30 characters); a buffer of the formatted length plus the
        // zero suppression's reach stands for it here
        string formatted;
        if ((val >= r1 && val < r2) || (val <= -r1 && val > -r2))
        {
            formatted = Sprintf(rangeform, val);
        }
        else
        {
            formatted = Sprintf(mainform, val);
        }
        char[] pText = new char[Math.Max(formatted.Length, 16) + 8];
        formatted.CopyTo(0, pText, 0, formatted.Length);

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
        int length = Array.IndexOf(pText, '\0');
        text = new string(pText, 0, length < 0 ? pText.Length : length);
        return text.Length; // strlen(text)
    }

    // OCCT keeps the formats in char[12] buffers, copied with strcpy (no check of the length)
    private static string CFormat(string form) => form;

    private string _themainform = "";
    private double _therange1;
    private double _therange2;
    private string _therangeform = "";
    private bool _thezerosup;
}
