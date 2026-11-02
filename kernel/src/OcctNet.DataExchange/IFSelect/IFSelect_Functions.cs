// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Functions.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Functions.cxx (its complementary functions)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Functions gives access to the actions which can be commanded with the resources provided by IFSelect:
/// especially WorkSession and various types of Selections and Dispatches. It works by adding functions by
/// method Init.
/// </summary>
/// <remarks>
/// OCCT.NET ports the complementary functions the commands use (<see cref="GiveEntity"/>,
/// <see cref="GiveEntityNumber"/>, <see cref="GiveList"/>, <see cref="GiveDispatch"/>). The command table
/// (<c>Init</c>, which registers some hundred functions in IFSelect_Act's process-wide dictionary, and their bodies)
/// is left out with IFSelect_Act and IFSelect_SessionPilot: Draw-lite ports the commands it needs directly
/// (decision 12702).
/// </remarks>
public static class IFSelect_Functions
{
    /// <summary>
    /// Takes the name of an entity, either as argument, or (if &lt;name&gt; is empty) on keyboard, and returns the
    /// entity name can be a label or a number (in alphanumeric), it is searched by NumberFromLabel from
    /// WorkSession. If &lt;name&gt; doesn't match en entity, a Null Handle is returned
    /// </summary>
    public static Standard_Transient? GiveEntity(IFSelect_WorkSession WS, string name = "")
    {
        Standard_Transient ent = null; // starts at Null
        int num = GiveEntityNumber(WS, name);
        if (num > 0)
        {
            ent = WS.StartingEntity(num);
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
    public static int GiveEntityNumber(IFSelect_WorkSession WS, string name = "")
    {
        int num = 0;
        if (name is null || CStringChar(name, 0) == '\0')
        {
            return 0;
        }
        num = WS.NumberFromLabel(name);
        return num;
    }

    /// <summary>
    /// Computes a List of entities from a WorkSession and two idents, first and second, as follows : if
    /// &lt;first&gt; is a Number or Label of an entity : this entity if &lt;first&gt; is the name of a Selection
    /// in &lt;WS&gt;, and &lt;second&gt; not defined, the standard result of this Selection if &lt;first&gt; is
    /// for a Selection and &lt;second&gt; is defined, the standard result of this selection from the list
    /// computed with &lt;second&gt; (an entity or a selection) If &lt;second&gt; is erroneous, it is ignored
    /// </summary>
    public static NCollection_HSequence<Standard_Transient>? GiveList(IFSelect_WorkSession WS, string first = "", string second = "")
    {
        return WS.GiveList(first, second);
    }

    /// <summary>
    /// Evaluates and returns a Dispatch, from data of a WorkSession if &lt;mode&gt; is False, searches for exact
    /// name of Dispatch in WS Else (D), allows a parameter between brackets : ex.: dispatch_name(parameter) The
    /// parameter can be: an integer for DispPerCount or DispPerFiles or the name of a Signature for
    /// DispPerSignature Returns Null Handle if not found not well evaluated
    /// </summary>
    public static IFSelect_Dispatch? GiveDispatch(IFSelect_WorkSession WS, string name, bool mode = true)
    {
        IFSelect_Dispatch disp = WS.NamedItem(name) as IFSelect_Dispatch;
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
            throw new Standard_OutOfRange("TCollection_AsciiString::SetValue(): out of range location");
        }
        char[] buf = nam.ToCharArray();
        buf[paro - 1] = '\0';
        buf[parf - 1] = '\0';
        string disname = CString(buf, 0);
        string param = CString(buf, paro);
        disp = WS.NamedItem(disname) as IFSelect_Dispatch;
        if (disp is null)
        {
            return disp; // KO anyway
        }

        //  According to the type of dispatch :
        using Message_Messenger.StreamBuffer sout = Message.SendInfo();
        if (disp is IFSelect_DispPerCount dc)
        {
            int nb = atoi(param);
            if (nb <= 0)
            {
                sout.Append(" DispPerCount, count is not positive").Append('\n');
                disp = null;
                return disp;
            }
            if (mode)
            {
                IFSelect_IntParam val = new IFSelect_IntParam();
                val.SetValue(nb);
                dc.SetCount(val);
            }
            return dc;
        }
        if (disp is IFSelect_DispPerFiles dp)
        {
            int nb = atoi(param);
            if (nb <= 0)
            {
                sout.Append(" DispPerFiles, count is not positive").Append('\n');
                disp = null;
                return disp;
            }
            if (mode)
            {
                IFSelect_IntParam val = new IFSelect_IntParam();
                val.SetValue(nb);
                dp.SetCount(val);
            }
            return dp;
        }
        if (disp is IFSelect_DispPerSignature ds)
        {
            IFSelect_Signature sg = WS.NamedItem(param) as IFSelect_Signature;
            if (sg is null)
            {
                sout.Append("DispPerSignature ").Append(disname).Append(" , Signature not valid : ").Append(param)
                    .Append('\n');
                disp = null;
                return disp;
            }
            if (mode)
            {
                ds.SetSignCounter(new IFSelect_SignCounter(sg));
            }
            return ds;
        }
        sout.Append("Dispatch : ").Append(name).Append(" , Parameter : ").Append(param).Append('\n');
        return disp;
    }

    // A C string held in a char array from an index: up to the first '\0' (OCCT.NET).
    private static string CString(char[] theBuffer, int theFrom)
    {
        int anEnd = theFrom;
        while (anEnd < theBuffer.Length && theBuffer[anEnd] != '\0')
        {
            anEnd++;
        }
        return new string(theBuffer, theFrom, anEnd - theFrom);
    }
}
