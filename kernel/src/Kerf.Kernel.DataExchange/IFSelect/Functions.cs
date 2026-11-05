// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Functions.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Functions.cxx (its complementary functions)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// Functions gives access to the actions which can be commanded with the resources provided by IFSelect:
/// especially WorkSession and various types of Selections and Dispatches. It works by adding functions by
/// method Init.
/// </summary>
/// <remarks>
/// Kerf ports the complementary functions the commands use (<see cref="GiveEntity"/>,
/// <see cref="GiveEntityNumber"/>, <see cref="GiveList"/>, <see cref="GiveDispatch"/>). The command table
/// (<c>Init</c>, which registers some hundred functions in IFSelect_Act's process-wide dictionary, and their bodies)
/// is left out with IFSelect_Act and IFSelect_SessionPilot: Draw-lite ports the commands it needs directly
/// (decision 1221).
/// </remarks>
internal static class Functions
{
    /// <summary>
    /// Takes the name of an entity, either as argument, or (if &lt;name&gt; is empty) on keyboard, and returns the
    /// entity name can be a label or a number (in alphanumeric), it is searched by NumberFromLabel from
    /// WorkSession. If &lt;name&gt; doesn't match en entity, a Null Handle is returned
    /// </summary>
    public static Transient? GiveEntity(WorkSession ws, string name = "")
    {
        Transient ent = null; // starts at Null
        int num = GiveEntityNumber(ws, name);
        if (num > 0)
        {
            ent = ws.StartingEntity(num);
        }
        return ent;
    }

    /// <summary>
    /// Same as GiveEntity, but a returns the Entity Number (in the Model of the WorkSession) or zero
    /// </summary>
    /// <remarks>
    /// OCCT reads the name from the standard input when it is empty: the port has no keyboard and returns 0
    /// (OCCT's answer to an empty line).
    /// </remarks>
    public static int GiveEntityNumber(WorkSession ws, string name = "")
    {
        int num = 0;
        if (name is null || CStringChar(name, 0) == '\0')
        {
            return 0;
        }
        num = ws.NumberFromLabel(name);
        return num;
    }

    /// <summary>
    /// Computes a List of entities from a WorkSession and two idents, first and second, as follows : if
    /// &lt;first&gt; is a Number or Label of an entity : this entity if &lt;first&gt; is the name of a Selection
    /// in &lt;WS&gt;, and &lt;second&gt; not defined, the standard result of this Selection if &lt;first&gt; is
    /// for a Selection and &lt;second&gt; is defined, the standard result of this selection from the list
    /// computed with &lt;second&gt; (an entity or a selection) If &lt;second&gt; is erroneous, it is ignored
    /// </summary>
    public static NCollection_HSequence<Transient>? GiveList(WorkSession ws, string first = "", string second = "")
    {
        return ws.GiveList(first, second);
    }

    /// <summary>
    /// Evaluates and returns a Dispatch, from data of a WorkSession if &lt;mode&gt; is False, searches for exact
    /// name of Dispatch in WS Else (D), allows a parameter between brackets : ex.: dispatch_name(parameter) The
    /// parameter can be: an integer for DispPerCount or DispPerFiles or the name of a Signature for
    /// DispPerSignature Returns Null Handle if not found not well evaluated
    /// </summary>
    public static Dispatch? GiveDispatch(WorkSession ws, string name, bool mode = true)
    {
        Dispatch disp = ws.NamedItem(name) as Dispatch;
        if (disp is not null)
        {
            return disp; // OK as it is given
        }

        //   Else, let s try special cases
        // OCCT puts '\0' at the positions of '(' and ')' (TCollection_AsciiString::SetValue, 1-based): the name is
        // the text before '(', the parameter the text from '(' + 1 up to ')'
        string nam = name ?? "";
        int paro = nam.IndexOf('(') + 1;
        int parf = nam.IndexOf(')') + 1;
        if (paro <= 0 || parf <= 0)
        {
            // OCCT: SetValue(0, ...) throws before the test "no parenthesis: not found" is reached, so an unknown
            // name without parentheses raises Standard_OutOfRange (occt-issues.md, 8A3)
            throw new OutOfRangeException("TCollection_AsciiString::SetValue(): out of range location");
        }
        char[] buf = nam.ToCharArray();
        buf[paro - 1] = '\0';
        buf[parf - 1] = '\0';
        string disname = CString(buf, 0);
        string param = CString(buf, paro);
        disp = ws.NamedItem(disname) as Dispatch;
        if (disp is null)
        {
            return disp; // KO anyway
        }

        //  According to the type of dispatch :
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        if (disp is DispPerCount dc)
        {
            int nb = Atoi(param);
            if (nb <= 0)
            {
                sout.Append(" DispPerCount, count is not positive").Append('\n');
                disp = null;
                return disp;
            }
            if (mode)
            {
                IntParam val = new IntParam();
                val.SetValue(nb);
                dc.SetCount(val);
            }
            return dc;
        }
        if (disp is DispPerFiles dp)
        {
            int nb = Atoi(param);
            if (nb <= 0)
            {
                sout.Append(" DispPerFiles, count is not positive").Append('\n');
                disp = null;
                return disp;
            }
            if (mode)
            {
                IntParam val = new IntParam();
                val.SetValue(nb);
                dp.SetCount(val);
            }
            return dp;
        }
        if (disp is DispPerSignature ds)
        {
            Signature sg = ws.NamedItem(param) as Signature;
            if (sg is null)
            {
                sout.Append("DispPerSignature ").Append(disname).Append(" , Signature not valid : ").Append(param)
                    .Append('\n');
                disp = null;
                return disp;
            }
            if (mode)
            {
                ds.SetSignCounter(new SignCounter(sg));
            }
            return ds;
        }
        sout.Append("Dispatch : ").Append(name).Append(" , Parameter : ").Append(param).Append('\n');
        return disp;
    }

    // A C string held in a char array from an index: up to the first '\0' (Kerf).
    private static string CString(char[] buffer, int theFrom)
    {
        int end = theFrom;
        while (end < buffer.Length && buffer[end] != '\0')
        {
            end++;
        }
        return new string(buffer, theFrom, end - theFrom);
    }
}
