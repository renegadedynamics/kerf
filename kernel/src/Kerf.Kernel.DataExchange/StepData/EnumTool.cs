// Port of: src/DataExchange/TKDESTEP/StepData/StepData_EnumTool.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_EnumTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// This class gives a way of conversion between the value of an enumeration and its representation in STEP. An
/// enumeration corresponds in STEP to a list of texts. It is defined by a list of texts, the value 0 for the first,
/// 1 for the second... Each text is given with its dots (".T." for instance); an empty text is ignored; a text
/// "$" means the null value (optional); several texts given in one string, separated by blanks, are separate values.
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). OCCT's constructor takes up to 40 texts as arguments with
/// defaults; the port takes them as a <c>params</c> array (the same calls).
/// </remarks>
internal sealed class EnumTool
{
    private readonly NCollection_Sequence<string> _thetexts = new();
    private readonly int _theinit;
    private bool _theopt;

    /// <summary>Creates an EnumTool with definitions given by the texts (each one as by AddDefinition).</summary>
    public EnumTool(params string[] e)
    {
        foreach (string term in e)
        {
            AddDefinition(term);
        }
        _theinit = _thetexts.Length();
        _theopt = true;
    }

    /// <summary>
    /// Processes a definition, splits it according blanks if any. Empty definitions are ignored. A null definition
    /// can be input by given "$": the corresponding position is attached to "null/undefined" value (as one particular
    /// item of the enumeration list). The dots are added if not given.
    /// </summary>
    public void AddDefinition(string term)
    {
        char[] text = new char[80];
        if (term is null)
        {
            return;
        }
        if (term.Length == 0 || term[0] == '\0')
        {
            return;
        }
        int n0 = 0, n1 = 0;
        for (; n0 < term.Length && term[n0] != '\0'; n0++)
        {
            if (term[n0] <= 32)
            {
                if (n1 == 0)
                {
                    continue;
                }
                if (n1 > 1 || text[0] != '$')
                {
                    if (text[n1 - 1] != '.')
                    {
                        text[n1] = '.';
                        n1++;
                    }
                    text[n1] = '\0';
                }
                _thetexts.Append(CString(text)); // OCCT: no continue, so the next term starts with this blank
                n1 = 0;
            }
            if (n1 == 0 && term[n0] != '.' && term[n0] != '$')
            {
                text[0] = '.';
                n1++;
            }
            text[n1] = term[n0];
            n1++;
        }
        // OCCT: a definition ending with a blank leaves n1 at 0, and reads text[-1] below; the port skips the dots then
        if (n1 > 0 || text[0] != '$')
        {
            if (n1 > 0 && text[n1 - 1] != '.')
            {
                text[n1] = '.';
                n1++;
            }
            text[n1] = '\0';
        }
        if (n1 > 0 && text[n1 - 1] != '.')
        {
            text[n1] = '.';
            n1++;
        }
        text[n1] = '\0';
        _thetexts.Append(CString(text));
    }

    // The C string in a char buffer: up to its first '\0'
    private static string CString(char[] text)
    {
        int len = Array.IndexOf(text, '\0');
        return new string(text, 0, len < 0 ? text.Length : len);
    }

    /// <summary>Returns True if at least one definition has been entered after creation time (i.e. by AddDefinition only).</summary>
    public bool IsSet
    {
        get
        {
            return (_thetexts.Length() > _theinit);
        }
    }

    /// <summary>Returns the maximum integer for a suitable value. Remark: while values begin at zero, MaxValue is the count of recorded values minus one.</summary>
    public int MaxValue()
    {
        return _thetexts.Length() - 1;
    }

    /// <summary>Sets or unsets the EnumTool to accept undefined value (for optional field). Ignored if no null value is defined (by "$"). Can be changed during execution (to read each field). By default, EnumTool does not accept undefined value.</summary>
    public void Optional(bool mode)
    {
        _theopt = mode;
    }

    /// <summary>Returns the value attached to "null/undefined value". If none is specified or if Optional has been set to False, returns -1. Null Value has been specified by definition "$".</summary>
    public int NullValue()
    {
        return (_theopt ? Value("$") : 0); // OCCT returns false (0) where it documents -1
    }

    /// <summary>Returns the text which corresponds to a given numeric value. It is limited by dots. If num is out of range, returns an empty string.</summary>
    public string Text(int num)
    {
        if (num < 0 || num >= _thetexts.Length())
        {
            return "";
        }
        return _thetexts.Value(num + 1);
    }

    /// <summary>Returns the numeric value found for a text. The text must be in capitals and limited by dots. A non-suitable text gives a negative value to be returned.</summary>
    public int Value(string txt)
    {
        int nb = _thetexts.Length();
        for (int i = 1; i <= nb; i++)
        {
            if (string.Equals(_thetexts.Value(i), txt, StringComparison.Ordinal))
            {
                return i - 1;
            }
        }
        return (-1);
    }
}
