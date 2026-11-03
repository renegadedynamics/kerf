// Port of: src/DataExchange/TKXSBase/Interface/Interface_Static.cxx (the dictionary of statics: Init, Static,
//          IsPresent, CDef, IDef, IsSet, CVal, IVal, RVal, SetCVal, SetIVal, SetRVal, Update, IsUpdated, Items, FillMap)
// Port of: src/DataExchange/TKXSBase/Interface/Interface_StaticStandards.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Messages;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A set of data exchange's parameters (<see cref="Static"/>s, named "read.precision.mode",
/// "write.step.schema"...): the parameter object that replaces OCCT's process-wide dictionary of statics, passed to
/// each read and write (plan 2.7, decision 1181).
/// </summary>
/// <remarks>
/// <para>
/// Its instance methods are OCCT's static methods of <c>Interface_Static</c>, with their names and behaviour, so
/// ported call sites stay recognisable: <c>Interface_Static::IVal("read.precision.mode")</c> becomes
/// <c>theStatics.IVal("read.precision.mode")</c>, and <c>Interface_Static::Init("XSTEP", "read.precision.mode", 'e', "")</c>
/// becomes <c>theStatics.Init("XSTEP", "read.precision.mode", 'e', "")</c>.
/// </para>
/// <para>
/// Each layer that declares parameters fills a set in its own initialisation, as OCCT's do (<see cref="Standards"/>
/// here; <c>XSAlgo</c>'s, <c>STEPControl_Controller</c>'s and <c>STEPCAFControl_Controller</c>'s in later units),
/// once per set: a set remembers which layers it holds. Copies are deep (<see cref="StaticSet(StaticSet)"/>),
/// so a reader can be given a copy that nobody else changes, and Draw-lite keeps one set per interpreter.
/// </para>
/// <para>
/// A set is a working object: it is not thread-safe, so a set that is being changed must not be read by another
/// thread at the same time. Lists (<see cref="Items"/>, <see cref="FillMap"/>) follow the order in which the
/// parameters were declared, where OCCT's follow its map's hash order.
/// </para>
/// </remarks>
public sealed class StaticSet
{
    // OCCT's MoniTool_TypedValue::Stats(): the statics by name, in declaration order (decision 1181)
    private readonly NCollection_DataMap<string, Static> _mystats = new();

    // The layers whose initialisation has filled this set (OCCT's static "deja" flags, one per initialisation)
    private readonly HashSet<string> _myinits = new(StringComparer.Ordinal);

    /// <summary>An empty set (no parameter declared). <see cref="Standards"/> declares data exchange's general ones.</summary>
    public StaticSet()
    {
    }

    /// <summary>
    /// A deep copy of another set: each parameter is copied with its definition, value, family and status, its
    /// wild-card mapped to the copy's own; later changes to either set don't reach the other.
    /// </summary>
    public StaticSet(StaticSet other)
    {
        foreach (KeyValuePair<string, Static> item in other._mystats)
        {
            _mystats.Bind(item.Key, new Static(item.Value, true));
        }
        foreach (KeyValuePair<string, Static> item in other._mystats)
        {
            Static? wild = item.Value.Wild();
            if (wild is not null)
            {
                Static? copy = null;
                foreach (KeyValuePair<string, Static> candidate in other._mystats)
                {
                    if (candidate.Value == wild)
                    {
                        copy = _mystats.Find(candidate.Key);
                        break;
                    }
                }
                _mystats.Find(item.Key).SetWild(copy ?? wild);
            }
        }
        _myinits.UnionWith(other._myinits);
    }

    /// <summary>
    /// Marks the initialisation named <paramref name="layer"/> as done for this set, and tells whether it was the
    /// first time: OCCT's static "already done" flags (such as <c>Standards</c>' <c>THE_Interface_Static_deja</c>), one
    /// per set here. Later units' initialisations (<c>XSAlgo::Init</c>, <c>STEPControl_Controller</c>...) use it.
    /// </summary>
    public bool StartInit(string layer) => _myinits.Add(layer);

    /// <summary>
    /// Declares a new static (by calling its constructor). If this name is already taken, does nothing and returns
    /// false; else creates it and returns true. For additional definitions, get the static then edit it.
    /// <see cref="ParamMisc"/> as type makes a static with the same definition as the static named by
    /// <paramref name="init"/>.
    /// </summary>
    public bool Init(string family, string name, ParamType type, string init = "")
    {
        if (name.Length == 0)
        {
            return false;
        }

        if (_mystats.IsBound(name))
        {
            return false;
        }
        Static item;
        if (type == ParamType.ParamMisc)
        {
            Static? other = Static(init);
            if (other is null)
            {
                return false;
            }
            item = new Static(family, name, other);
        }
        else
        {
            item = new Static(family, name, type, init);
        }

        _mystats.Bind(name, item);
        return true;
    }

    /// <summary>
    /// As Init with a ParamType, but the type is given as a character: 'i' integer, 'r' real, 't' text, 'e' enum,
    /// 'o' object, 'p' path (a text), '=' for the same definition as the static named by <paramref name="init"/>, and
    /// '&amp;' to complete the definition of an existing static by the edition text <paramref name="init"/>
    /// ("imin ival", "imax ival", "rmin rval", "rmax rval", "unit def", "enum from", "ematch from", "eval cval").
    /// Returns false if <paramref name="type"/> does not match this list.
    /// </summary>
    public bool Init(string family, string name, char type, string init = "")
    {
        ParamType epyt;
        switch (type)
        {
            case 'e':
                epyt = ParamType.ParamEnum;
                break;
            case 'i':
                epyt = ParamType.ParamInteger;
                break;
            case 'o':
                epyt = ParamType.ParamIdent;
                break;
            case 'p':
                epyt = ParamType.ParamText;
                break;
            case 'r':
                epyt = ParamType.ParamReal;
                break;
            case 't':
                epyt = ParamType.ParamText;
                break;
            case '=':
                epyt = ParamType.ParamMisc;
                break;
            case '&':
            {
                Static? unstat = Static(name);
                if (unstat is null)
                {
                    return false;
                }
                //    Editions : init gives a small edition text, in 2 terms "cmd var" :
                //  imin <ival>  imax <ival>  rmin <rval>  rmax <rval>  unit <def>
                //  enum <from>  ematch <from>  eval <cval>
                int i, iblc = 0;
                for (i = 0; i < init.Length; i++)
                {
                    if (init[i] == ' ')
                    {
                        iblc = i + 1;
                    }
                }
                //  Recognition of the sub-case and routing
                if (CStringChar(init, 0) == 'i' && CStringChar(init, 2) == 'i')
                {
                    unstat.SetIntegerLimit(false, Atoi(init.Substring(iblc)));
                }
                else if (CStringChar(init, 0) == 'i' && CStringChar(init, 2) == 'a')
                {
                    unstat.SetIntegerLimit(true, Atoi(init.Substring(iblc)));
                }
                else if (CStringChar(init, 0) == 'r' && CStringChar(init, 2) == 'i')
                {
                    unstat.SetRealLimit(false, Atof(init.Substring(iblc)));
                }
                else if (CStringChar(init, 0) == 'r' && CStringChar(init, 2) == 'a')
                {
                    unstat.SetRealLimit(true, Atof(init.Substring(iblc)));
                }
                else if (CStringChar(init, 0) == 'u')
                {
                    unstat.SetUnitDef(init.Substring(iblc));
                }
                else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'm')
                {
                    unstat.StartEnum(Atoi(init.Substring(iblc)), true);
                }
                else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'n')
                {
                    unstat.StartEnum(Atoi(init.Substring(iblc)), false);
                }
                else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'v')
                {
                    unstat.AddEnum(init.Substring(iblc));
                }
                else
                {
                    return false;
                }
                return true;
            }
            default:
                return false;
        }

        if (!Init(family, name, epyt, init))
        {
            return false;
        }
        if (type != 'p')
        {
            return true;
        }
        Static stat = Static(name)!;
        // NT  stat->SetSatisfies (StaticPath,"Path");
        if (!stat.Satisfies(stat.HStringValue()))
        {
            stat.SetCStringValue("");
        }
        return true;
    }

    /// <summary>Returns a static from its name, null if not present.</summary>
    public Static? Static(string name)
    {
        _mystats.Find(name, out Static? result);
        return result;
    }

    /// <summary>Returns true if a static named <paramref name="name"/> is present.</summary>
    public bool IsPresent(string name) => _mystats.IsBound(name);

    /// <summary>
    /// Returns a part of the definition of a static, as a string; empty if <paramref name="name"/> is not present or
    /// <paramref name="part"/> is not defined for it. Parts: family, type ("integer", "real", "text", "enum"), label,
    /// satis (the satisfies function's name), rmin, rmax, imin, imax, "enum nn" (the enum value for nn), unit.
    /// </summary>
    public string CDef(string name, string? part)
    {
        if (part is null || part.Length == 0)
        {
            return "";
        }
        Static? stat = Static(name);
        if (stat is null)
        {
            return "";
        }
        if (CStringChar(part, 0) == 'f' && CStringChar(part, 1) == 'a')
        {
            return stat.Family();
        }
        if (CStringChar(part, 0) == 'l' && CStringChar(part, 1) == 'a')
        {
            return stat.Label();
        }
        if (CStringChar(part, 0) == 's' && CStringChar(part, 1) == 'a')
        {
            return stat.SatisfiesName();
        }
        if (CStringChar(part, 0) == 't' && CStringChar(part, 1) == 'y')
        {
            ParamType typ = stat.Type();
            if (typ == ParamType.ParamInteger)
            {
                return "integer";
            }
            if (typ == ParamType.ParamReal)
            {
                return "real";
            }
            if (typ == ParamType.ParamText)
            {
                return "text";
            }
            if (typ == ParamType.ParamEnum)
            {
                return "enum";
            }
            return "?type?";
        }
        if (CStringChar(part, 0) == 'e')
        {
            // sscanf(part, "%30s %d", defmess, &nume)
            int nume = SscanfSecondInteger(part);
            return stat.EnumVal(nume);
        }
        if (CStringChar(part, 0) == 'i')
        {
            int ilim;
            if (!stat.IntegerLimit((CStringChar(part, 2) == 'a'), out ilim))
            {
                return "";
            }
            return Sprintf("%d", ilim);
        }
        if (CStringChar(part, 0) == 'r')
        {
            double rlim;
            if (!stat.RealLimit((CStringChar(part, 2) == 'a'), out rlim))
            {
                return "";
            }
            return Sprintf("%f", rlim);
        }
        if (CStringChar(part, 0) == 'u')
        {
            return stat.UnitDef();
        }
        return "";
    }

    /// <summary>
    /// Returns a part of the definition of a static, as an integer; zero if <paramref name="name"/> is not present
    /// or <paramref name="part"/> is not defined for it. Parts: imin, imax, estart (the first enum number), ecount
    /// (the count of enum values), ematch (the exact match status), "eval val" (the case of a string).
    /// </summary>
    public int IDef(string name, string? part)
    {
        if (part is null || part.Length == 0)
        {
            return 0;
        }
        Static? stat = Static(name);
        if (stat is null)
        {
            return 0;
        }
        if (CStringChar(part, 0) == 'i')
        {
            int ilim;
            if (!stat.IntegerLimit((CStringChar(part, 2) == 'a'), out ilim))
            {
                return 0;
            }
            return ilim;
        }
        if (CStringChar(part, 0) == 'e')
        {
            int startcase, endcase;
            bool match;
            stat.EnumDef(out startcase, out endcase, out match);
            if (CStringChar(part, 1) == 's')
            {
                return startcase;
            }
            if (CStringChar(part, 1) == 'c')
            {
                return (endcase - startcase + 1);
            }
            if (CStringChar(part, 1) == 'm')
            {
                return (match ? 1 : 0);
            }
            if (CStringChar(part, 1) == 'v')
            {
                // sscanf(part, "%30s %50s", defmess, vale)
                string vale = SscanfSecondWord(part, 50);
                return stat.EnumCase(vale);
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns true if <paramref name="name"/> is present and set. <paramref name="proper"/> true: considers this
    /// item only; false: if not set and attached to a wild-card, considers this wild-card.
    /// </summary>
    public bool IsSet(string name, bool proper = true)
    {
        Static? item = Static(name);
        if (item is null)
        {
            return false;
        }
        if (item.IsSetValue())
        {
            return true;
        }
        if (proper)
        {
            return false;
        }
        item = item.Wild();
        return item!.IsSetValue(); // OCCT: no check that a wild-card is attached
    }

    /// <summary>
    /// Returns the value of the parameter named <paramref name="name"/>, an empty string if it does not exist.
    /// Example: <c>CVal("write.step.schema")</c>, which could return "AP214".
    /// </summary>
    public string CVal(string name)
    {
        Static? item = Static(name);
        if (item is null)
        {
            // OCCT_DEBUG warning left out
            return "";
        }
        return item.CStringValue();
    }

    /// <summary>
    /// Returns the integer value of the parameter named <paramref name="name"/>, 0 if it does not exist. Example:
    /// <c>IVal("write.step.schema")</c>, which could return 3.
    /// </summary>
    public int IVal(string name)
    {
        Static? item = Static(name);
        if (item is null)
        {
            // OCCT_DEBUG warning left out
            return 0;
        }
        return item.IntegerValue();
    }

    /// <summary>Returns the real value of the parameter named <paramref name="name"/>, 0.0 if it does not exist.</summary>
    public double RVal(string name)
    {
        Static? item = Static(name);
        if (item is null)
        {
            // OCCT_DEBUG warning left out
            return 0.0;
        }
        return item.RealValue();
    }

    /// <summary>
    /// Modifies the value of the parameter named <paramref name="name"/>, given as a string; false if the parameter
    /// does not exist or the value doesn't satisfy its definition. Example: <c>SetCVal("write.step.schema", "AP203")</c>.
    /// </summary>
    public bool SetCVal(string name, string val)
    {
        Static? item = Static(name);
        if (item is null)
        {
            return false;
        }
        return item.SetCStringValue(val);
    }

    /// <summary>
    /// Modifies the value of the parameter named <paramref name="name"/>, given as an integer; false if the parameter
    /// does not exist or the value doesn't satisfy its definition. Example: <c>SetIVal("write.step.schema", 3)</c>.
    /// </summary>
    public bool SetIVal(string name, int val)
    {
        Static? item = Static(name);
        if (item is null)
        {
            return false;
        }
        if (!item.SetIntegerValue(val))
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Modifies the value of the parameter named <paramref name="name"/>, given as a real; false if the parameter
    /// does not exist or the value doesn't satisfy its definition.
    /// </summary>
    public bool SetRVal(string name, double val)
    {
        Static? item = Static(name);
        if (item is null)
        {
            return false;
        }
        return item.SetRealValue(val);
    }

    /// <summary>Sets a static to be "uptodate"; false if <paramref name="name"/> is not present.</summary>
    public bool Update(string name)
    {
        Static? item = Static(name);
        if (item is null)
        {
            return false;
        }
        item.SetUptodate();
        return true;
    }

    /// <summary>Returns the status "uptodate" of a static; false if <paramref name="name"/> is not present.</summary>
    public bool IsUpdated(string name)
    {
        Static? item = Static(name);
        if (item is null)
        {
            return false;
        }
        return item.UpdatedStatus();
    }

    /// <summary>
    /// Returns the names of statics: those of the family <paramref name="criter"/>, or all (but the families
    /// beginning with '$', which are listed only by criter "$") if it is empty. <paramref name="mode"/> + 100 takes
    /// only non-updated items, + 200 only updated items. In declaration order (OCCT: its map's order).
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString> Items(int mode = 0, string criter = "")
    {
        int modup = (mode / 100); // 0 any, 1 non-update, 2 update
        NCollection_HSequence<TCollection_HAsciiString> list = new NCollection_HSequence<TCollection_HAsciiString>();
        foreach (KeyValuePair<string, Static> iter in _mystats)
        {
            Static item = iter.Value;
            bool ok = true;
            if (CStringChar(criter, 0) == '$' && CStringChar(criter, 1) == '\0')
            {
                if (CStringChar(item.Family(), 0) != '$')
                {
                    ok = false;
                }
            }
            else if (criter.Length > 0)
            {
                if (!string.Equals(criter, item.Family(), StringComparison.Ordinal))
                {
                    continue;
                }
                ok = true;
            }
            else
            { // all ... except family with $
                if (CStringChar(item.Family(), 0) == '$')
                {
                    continue;
                }
            }
            if (ok && (modup == 1))
            {
                ok = !item.UpdatedStatus();
            }
            if (ok && (modup == 2))
            {
                ok = item.UpdatedStatus();
            }

            if (ok)
            {
                list.Append(new TCollection_HAsciiString(iter.Key));
            }
        }
        return list;
    }

    /// <summary>Fills the given string-to-string map with all the statics' values (the map is cleared first).</summary>
    public void FillMap(NCollection_DataMap<string, string> map)
    {
        map.Clear();
        foreach (KeyValuePair<string, Static> it in _mystats)
        {
            Static value = it.Value;
            if (value.HStringValue() is null)
            {
                continue;
            }
            map.Bind(it.Key, value.HStringValue()!.String());
        }
    }

    /// <summary>
    /// Declares the standard parameters of data exchange (family "XSTEP": read and write precision, surface curves,
    /// encode regularity), once per set, and loads data exchange's messages into <see cref="MsgFile"/>.
    /// </summary>
    /// <exception cref="ProgramErrorException">The messages can't be loaded (never with the compiled-in text).</exception>
    public void Standards()
    {
        if (!StartInit("Interface_Static::Standards"))
        {
            return;
        }

        //   read precision
        // #74 rln 10.03.99 S4135: new values and default value
        Init("XSTEP", "read.precision.mode", 'e', "");
        Init("XSTEP", "read.precision.mode", '&', "ematch 0");
        Init("XSTEP", "read.precision.mode", '&', "eval File");
        Init("XSTEP", "read.precision.mode", '&', "eval User");
        SetIVal("read.precision.mode", 0);

        Init("XSTEP", "read.precision.val", 'r', "1.e-03");

        Init("XSTEP", "read.maxprecision.mode", 'e', "");
        Init("XSTEP", "read.maxprecision.mode", '&', "ematch 0");
        Init("XSTEP", "read.maxprecision.mode", '&', "eval Preferred");
        Init("XSTEP", "read.maxprecision.mode", '&', "eval Forced");
        SetIVal("read.maxprecision.mode", 0);

        Init("XSTEP", "read.maxprecision.val", 'r', "1.");

        //   encode regularity
        //  negative or null : do nothing. positive : let's go
        Init("XSTEP", "read.encoderegularity.angle", 'r', "0.01");

        //   compute surface curves
        //  0 : by default. 2 : keep only 2D. 3 : keep only 3D
        // gka S4054
        Init("XSTEP", "read.surfacecurve.mode", 'e', "");
        Init("XSTEP", "read.surfacecurve.mode", '&', "ematch -3");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval 3DUse_Forced");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval 2DUse_Forced");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval ?");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval Default");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval ?");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval 2DUse_Preferred");
        Init("XSTEP", "read.surfacecurve.mode", '&', "eval 3DUse_Preferred");
        SetIVal("read.surfacecurve.mode", 0);

        //   write precision
        Init("XSTEP", "write.precision.mode", 'e', "");
        Init("XSTEP", "write.precision.mode", '&', "ematch -1");
        Init("XSTEP", "write.precision.mode", '&', "eval Min");
        Init("XSTEP", "write.precision.mode", '&', "eval Average");
        Init("XSTEP", "write.precision.mode", '&', "eval Max");
        Init("XSTEP", "write.precision.mode", '&', "eval User");
        SetIVal("write.precision.mode", 0);

        Init("XSTEP", "write.precision.val", 'r', "1.e-03");

        // Write surface curves
        // 0: write (defaut), 1: do not write, 2: write except for analytical surfaces
        Init("XSTEP", "write.surfacecurve.mode", 'e', "");
        Init("XSTEP", "write.surfacecurve.mode", '&', "ematch 0");
        Init("XSTEP", "write.surfacecurve.mode", '&', "eval Off");
        Init("XSTEP", "write.surfacecurve.mode", '&', "eval On");
        //  Interface_Static::Init("XSTEP"  ,"write.surfacecurve.mode", '&',"eval NoAnalytic");
        SetIVal("write.surfacecurve.mode", 1);

        //  lastpreci : to recover the last encoded value (cf XSControl)
        //    (0 to say : not encoded)
        //: S4136  Interface_Static::Init("std"    ,"lastpreci", 'r',"0.");

        // load messages if needed
        LoadMessages();
    }

    /// <summary>
    /// Loads data exchange's messages (XSMessage's texts) into the defaults of <see cref="MsgFile"/>, once per
    /// process: the end of OCCT's <c>Interface_Static::Standards</c>.
    /// </summary>
    /// <remarks>
    /// OCCT first tries the message file the environment variable CSF_XSMessage names; the port loads only the text
    /// compiled in, OCCT's XSTEP.us (as decision 975 for Shape Healing), once (a one-time initialisation, unit 10I).
    /// </remarks>
    /// <exception cref="ProgramErrorException">The messages can't be loaded (never with the compiled-in text).</exception>
    public static void LoadMessages() => _ = MessagesLoaded.Value;

    private static readonly Lazy<bool> MessagesLoaded = new(() =>
    {
        // OCCT.NET: OCCT's Message_MsgFile::LoadFromEnv("CSF_XSMessage", "XSTEP") left out (decision 1181)
        MsgFile.AddDefaults(XstepMessages.Text);
        if (!MsgFile.HasMsg("XSTEP_1"))
        {
            throw new ProgramErrorException("Critical Error - message resources for Interface_Static are invalid or undefined!");
        }
        return true;
    });

    // sscanf(part, "%30s %d", ...): the integer after the first word (0 if none, as OCCT's initialised nume)
    private static int SscanfSecondInteger(string part)
    {
        string rest = AfterFirstWord(part, 30);
        int value = TCollection_AsciiString.Strtol(rest, out int end);
        return end > 0 ? value : 0;
    }

    // sscanf(part, "%30s %Ns", ...): the second word, at most theMax characters (empty if none: OCCT reads an
    // uninitialised buffer there)
    private static string SscanfSecondWord(string part, int max)
    {
        string rest = AfterFirstWord(part, 30).TrimStart(' ', '\t', '\n', '\v', '\f', '\r');
        int end = 0;
        while (end < rest.Length && end < max && !char.IsWhiteSpace(rest[end]))
        {
            end++;
        }
        if (end == 0)
        {
            // OCCT: sscanf reads nothing and leaves the buffer uninitialised; its garbage matches no value, so stand
            // for it by a text that matches none (EnumCase gives the start minus 1)
            return "\u0001";
        }
        return rest.Substring(0, end);
    }

    // The text after the first word of at most theMax characters, as sscanf's "%Ns" leaves it
    private static string AfterFirstWord(string text, int max)
    {
        int i = 0;
        while (i < text.Length && char.IsWhiteSpace(text[i]))
        {
            i++;
        }
        int count = 0;
        while (i < text.Length && count < max && !char.IsWhiteSpace(text[i]))
        {
            i++;
            count++;
        }
        return text.Substring(i);
    }
}
