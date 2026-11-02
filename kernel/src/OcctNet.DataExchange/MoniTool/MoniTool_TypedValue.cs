// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_TypedValue.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_TypedValue.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.MoniTool_ValueType;

namespace Occt;

/// <summary>
/// This class allows to dynamically manage .. typed values, i.e. values which have an alphanumeric
/// expression, but with controls. Such as "must be an Integer" or "Enumerative Text" etc Hence, a TypedValue
/// brings a specification (type + constraints if any) and a value. Its basic form is a string, it can be
/// specified as integer or real or enumerative string, then queried as such. Its string content, which is a
/// occ::handle&lt;HAsciiString&gt; can be shared by other data structures, hence gives a direct on line
/// access to its value.
/// </summary>
public class MoniTool_TypedValue : Standard_Transient
{
    // The library of typed values (OCCT's static thelibtv): the standard types, built once (decision 1181). OCCT's
    // AddLib, which adds to it, and the static dictionary of "static" values (OCCT's astats, Stats(), StaticValue)
    // are left out: the static values are a parameter set's (Interface_StaticSet), passed explicitly.
    private static readonly Lazy<NCollection_DataMap<string, Standard_Transient>> thelibtv = new(MakeLib);

    private static NCollection_DataMap<string, Standard_Transient> MakeLib()
    {
        var aLib = new NCollection_DataMap<string, Standard_Transient>();
        MoniTool_TypedValue tv;
        tv = new MoniTool_TypedValue("Integer", MoniTool_ValueInteger);
        aLib.Bind("Integer", tv);
        tv = new MoniTool_TypedValue("Real", MoniTool_ValueReal);
        aLib.Bind("Real", tv);
        tv = new MoniTool_TypedValue("Text", MoniTool_ValueText);
        aLib.Bind("Text", tv);
        tv = new MoniTool_TypedValue("Transient", MoniTool_ValueIdent);
        aLib.Bind("Transient", tv);
        tv = new MoniTool_TypedValue("Boolean", MoniTool_ValueEnum);
        tv.AddDef("enum 0"); //    = 0 False  ,  > 0 True
        tv.AddDef("eval False");
        tv.AddDef("eval True");
        aLib.Bind("Boolean", tv);
        tv = new MoniTool_TypedValue("Logical", MoniTool_ValueEnum);
        tv.AddDef("enum -1"); //    < 0 False  ,  = 0 Unk  ,  > 0 True
        tv.AddDef("eval False");
        tv.AddDef("eval Unknown");
        tv.AddDef("eval True");
        aLib.Bind("Logical", tv);
        return aLib;
    }

    private static NCollection_DataMap<string, Standard_Transient> libtv() => thelibtv.Value;

    /// <summary>
    /// Creates a TypedValue, with a name type gives the type of the parameter, default is free text Also
    /// available : Integer, Real, Enum, Entity (i.e. Object) More precise specifications, titles, can be given
    /// to the TypedValue once created init gives an initial value. If it is not given, the TypedValue begins as
    /// "not set", its value is empty
    /// </summary>
    public MoniTool_TypedValue(string name, MoniTool_ValueType type = MoniTool_ValueText, string init = "")
    {
        thename = name;
        thetype = type;
        thelims = 0;
        themaxlen = 0;
        theintlow = 0;
        theintup = -1;
        therealow = 0.0;
        therealup = 0.0;
        theinterp = null;
        thesatisf = null;
        theival = 0;
        thehval = new TCollection_HAsciiString("");
        if (type != MoniTool_ValueInteger && type != MoniTool_ValueReal && type != MoniTool_ValueEnum
                && type != MoniTool_ValueText && type != MoniTool_ValueIdent)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : Type not supported");
        }
        if (init.Length > 0)
        {
            if (Satisfies(new TCollection_HAsciiString(init)))
            {
                SetCStringValue(init);
            }
        }
    }

    /// <summary>
    /// Creates a TypedValue from another one, by duplication
    /// </summary>
    public MoniTool_TypedValue(MoniTool_TypedValue? other)
    {
        thename = other.Name();
        thedef = other.Definition();
        thelabel = other.Label();
        thetype = other.ValueType();
        theotyp = other.ObjectType();
        thelims = 0;
        themaxlen = other.MaxLength();
        theintlow = 0;
        theintup = 0;
        therealow = 0;
        therealup = 0;
        theunidef = other.UnitDef();
        theival = other.IntegerValue();
        thehval = other.HStringValue();
        theoval = other.ObjectValue();
        other.Internals(out theinterp, out thesatisf, out string satisname, out NCollection_DataMap<string, int> eadds);
        thesatisn += satisname;

        if (other.IntegerLimit(false, out theintlow))
        {
            thelims |= 1;
        }
        if (other.IntegerLimit(true, out theintup))
        {
            thelims |= 2;
        }
        if (other.RealLimit(false, out therealow))
        {
            thelims |= 1;
        }
        if (other.RealLimit(true, out therealup))
        {
            thelims |= 2;
        }

        int startcase, endcase;
        bool match;
        if (other.EnumDef(out startcase, out endcase, out match))
        {
            theintlow = startcase;
            theintup = endcase;
            if (match)
            {
                thelims |= 4;
            }
            if (theintup >= theintlow)
            {
                theenums = new NCollection_HArray1<string>(theintlow, theintup);
                theenums.Init("");
            }
            for (startcase = theintlow; startcase <= theintup; startcase++)
            {
                theenums.SetValue(startcase, other.EnumVal(startcase));
            }
        }
        //  dupliquer theeadds
        if (!eadds.IsEmpty())
        {
            foreach (KeyValuePair<string, int> itad in eadds)
            {
                theeadds.Bind(itad.Key, itad.Value);
            }
        }

        //  we duplicate the string
        if (thehval is not null)
        {
            thehval = new TCollection_HAsciiString(other.CStringValue());
        }
    }

    /// <summary>
    /// An exact copy of another typed value (OCCT.NET): every field as it is, the enforced definition included (where
    /// OCCT's copy constructor enforces the computed definition), the value in a new string. For
    /// <see cref="Interface_StaticSet"/>'s copies, which must behave as the original under later definitions.
    /// </summary>
    internal MoniTool_TypedValue(MoniTool_TypedValue other, bool theExact)
    {
        _ = theExact;
        thename = other.thename;
        thedef = other.thedef;
        thelabel = other.thelabel;
        thetype = other.thetype;
        theotyp = other.theotyp;
        thelims = other.thelims;
        themaxlen = other.themaxlen;
        theintlow = other.theintlow;
        theintup = other.theintup;
        therealow = other.therealow;
        therealup = other.therealup;
        theunidef = other.theunidef;
        theenums = other.theenums is null ? null : new NCollection_HArray1<string>(other.theenums);
        theeadds = new NCollection_DataMap<string, int>(other.theeadds);
        theinterp = other.theinterp;
        thesatisf = other.thesatisf;
        thesatisn = other.thesatisn;
        theival = other.theival;
        thehval = other.thehval is null ? null : new TCollection_HAsciiString(other.thehval);
        theoval = other.theoval;
    }

    /// <summary>
    /// Access to internal data which have no other access
    /// </summary>
    public void Internals(out MoniTool_ValueInterpret interp, out MoniTool_ValueSatisfies satisf, out string satisname, out NCollection_DataMap<string, int> enums)
    {
        interp = theinterp;
        satisf = thesatisf;
        satisname = thesatisn;
        enums = new NCollection_DataMap<string, int>(theeadds); // a copy, as C++'s assignment
    }

    /// <summary>
    /// Returns the name
    /// </summary>
    public string Name()
    {
        return thename;
    }

    /// <summary>
    /// Returns the type of the value
    /// </summary>
    public MoniTool_ValueType ValueType()
    {
        return thetype;
    }

    /// <summary>
    /// Returns the Definition By priority, the enforced one, else an automatic one, computed from the
    /// specification
    /// </summary>
    public string Definition()
    {
        if (thedef.Length > 0)
        {
            return thedef;
        }
        string def = "";
        string mess;
        switch (thetype)
        {
            case MoniTool_ValueInteger: {
                def += "Integer";
                int ilim;
                if (IntegerLimit(false, out ilim))
                {
                    mess = Sprintf("  >= %d", ilim);
                    def += mess;
                }
                if (IntegerLimit(true, out ilim))
                {
                    mess = Sprintf("  <= %d", ilim);
                    def += mess;
                }
            }
            break;
            case MoniTool_ValueReal: {
                def += "Real";
                double rlim;
                if (RealLimit(false, out rlim))
                {
                    mess = Sprintf("  >= %f", rlim);
                    def += mess;
                }
                if (RealLimit(true, out rlim))
                {
                    mess = Sprintf("  <= %f", rlim);
                    def += mess;
                }
                if (theunidef.Length > 0)
                {
                    def += "  Unit:";
                    def += UnitDef();
                }
            }
            break;
            case MoniTool_ValueEnum: {
                def += "Enum";
                int startcase = 0, endcase = 0;
                bool match = false;
                EnumDef(out startcase, out endcase, out match);
                mess = Sprintf(" [%s%d-%d]", (match ? "in " : ""), startcase, endcase);
                def += mess;
                for (int i = startcase; i <= endcase; i++)
                {
                    string enva = EnumVal(i);
                    if (enva.Length == 0 || enva[0] == '?')
                    {
                        continue;
                    }
                    mess = Sprintf(" %d:%s", i, enva);
                    def += mess;
                }
                if (!theeadds.IsEmpty())
                {
                    def += " , alpha: ";
                    // OCCT.NET: the map iterates in insertion order where OCCT's iterates in hash order (decision 1181)
                    foreach (KeyValuePair<string, int> listadd in theeadds)
                    {
                        string aName = listadd.Key;
                        string enva = aName;
                        if (enva.Length > 0 && enva[0] == '?')
                        {
                            continue;
                        }
                        mess = Sprintf(":%d ", listadd.Value);
                        def += enva;
                        def += mess;
                    }
                }
            }
            break;
            case MoniTool_ValueIdent: {
                def += "Object(Entity)";
                if (theotyp is not null)
                {
                    def += ":";
                    def += theotyp.Name;
                }
            }
            break;
            case MoniTool_ValueText: {
                def += "Text";
                if (themaxlen > 0)
                {
                    mess = Sprintf(" <= %d C.", themaxlen);
                    def += mess;
                }
                break;
            }
            default:
                def += "(undefined)";
                break;
        }
        return def;
    }

    /// <summary>
    /// Enforces a Definition
    /// </summary>
    public void SetDefinition(string deftext)
    {
        thedef = "";
        thedef += deftext;
    }

    /// <summary>
    /// Prints definition, specification, and actual status and value
    /// </summary>
    public virtual void Print(TextWriter S)
    {
        S.Write("--- Typed Value : ");
        S.Write(Name());
        if (thelabel.Length > 0)
        {
            S.Write("  Label : ");
            S.Write(Label());
        }
        S.Write('\n');
        S.Write("--- Type : ");
        S.Write(Definition());
        S.Write('\n');
        S.Write("--- Value : ");

        PrintValue(S);
        S.Write('\n');

        if (thesatisf is not null)
        {
            S.Write(" -- Specific Function for Satisfies : ");
            S.Write(thesatisn);
            S.Write('\n');
        }
    }

    /// <summary>
    /// Prints only the Value
    /// </summary>
    public void PrintValue(TextWriter S)
    {
        if (IsSetValue())
        {
            if (thetype == MoniTool_ValueIdent)
            {
                S.Write(" (type) ");
                S.Write(theoval.DynamicType().Name);
            }
            if (thehval is not null)
            {
                S.Write((thetype == MoniTool_ValueIdent ? " : " : ""));
                S.Write(thehval.ToCString());
            }

            if (HasInterpret())
            {
                S.Write("  (");
                TCollection_HAsciiString str = Interpret(thehval, true);
                if (str is not null && str != thehval)
                {
                    S.Write("Native:");
                    S.Write(str.ToCString());
                }
                str = Interpret(thehval, false);
                if (str is not null && str != thehval)
                {
                    S.Write("  Coded:");
                    S.Write(str.ToCString());
                }
                S.Write(")");
            }
        }
        else
        {
            S.Write("(not set)");
        }
    }

    /// <summary>
    /// Completes the definition of a TypedValue by command &lt;initext&gt;, once created with its type Returns
    /// True if done, False if could not be interpreted &lt;initext&gt; may be : imin ival : minimum value for an
    /// integer imax ival : maximum value for an integer rmin rval : minimum value for a real rmax rval : maximum
    /// value for a real unit name : name of unit ematch i : enum from integer value i, match required enum i :
    /// enum from integer value i, match not required eval text : add an enumerative value (increments max by 1)
    /// eval ?? : add a non-authorised enum value (to be skipped) tmax l : maximum length for a text
    /// </summary>
    public bool AddDef(string init)
    {
        //    Editions : init gives a small edition text, in 2 terms "cmd var" :
        int i, iblc = 0;
        for (i = 0; i < init.Length; i++)
        {
            if (init[i] == ' ')
            {
                iblc = i + 1;
            }
        }
        if (iblc == 0)
        {
            return false;
        }
        //  Recognition of sub-case and routing
        if (CStringChar(init, 0) == 'i' && CStringChar(init, 2) == 'i')
        { // imin ival
            SetIntegerLimit(false, atoi(init.Substring(iblc)));
        }
        else if (CStringChar(init, 0) == 'i' && CStringChar(init, 2) == 'a')
        { // imax ival
            SetIntegerLimit(true, atoi(init.Substring(iblc)));
        }
        else if (CStringChar(init, 0) == 'r' && CStringChar(init, 2) == 'i')
        { // rmin rval
            SetRealLimit(false, Atof(init.Substring(iblc)));
        }
        else if (CStringChar(init, 0) == 'r' && CStringChar(init, 2) == 'a')
        { // rmax rval
            SetRealLimit(true, Atof(init.Substring(iblc)));
        }
        else if (CStringChar(init, 0) == 'u')
        { // unit name
            SetUnitDef(init.Substring(iblc));
        }
        else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'm')
        { // ematch istart
            StartEnum(atoi(init.Substring(iblc)), true);
        }
        else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'n')
        { // enum istart
            StartEnum(atoi(init.Substring(iblc)), false);
        }
        else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'v')
        { // eval text
            AddEnum(init.Substring(iblc));
        }
        else if (CStringChar(init, 0) == 't' && CStringChar(init, 1) == 'm')
        { // tmax length
            SetMaxLength(atoi(init.Substring(iblc)));
        }
        else
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// Sets a label, which can then be displayed
    /// </summary>
    public void SetLabel(string label)
    {
        thelabel = "";
        thelabel += label;
    }

    /// <summary>
    /// Returns the label, if set; else returns an empty string
    /// </summary>
    public string Label()
    {
        return thelabel;
    }

    /// <summary>
    /// Sets a maximum length for a text (active only for a free text)
    /// </summary>
    public void SetMaxLength(int max)
    {
        themaxlen = max;
        if (max < 0)
        {
            themaxlen = 0;
        }
    }

    /// <summary>
    /// Returns the maximum length, 0 if not set
    /// </summary>
    public int MaxLength()
    {
        return themaxlen;
    }

    /// <summary>
    /// Sets an Integer limit (included) to &lt;val&gt;, the upper limit if &lt;max&gt; is True, the lower limit
    /// if &lt;max&gt; is False
    /// </summary>
    public void SetIntegerLimit(bool max, int val)
    {
        if (thetype != MoniTool_ValueInteger)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : SetIntegerLimit, not an Integer");
        }

        if (max)
        {
            thelims |= 2;
            theintup = val;
        }
        else
        {
            thelims |= 1;
            theintlow = val;
        }
    }

    /// <summary>
    /// Gives an Integer Limit (upper if &lt;max&gt; True, lower if &lt;max&gt; False). Returns True if this
    /// limit is defined, False else (in that case, gives the natural limit for Integer)
    /// </summary>
    public bool IntegerLimit(bool max, out int val)
    {
        bool res = false;
        if (max)
        {
            res = (thelims & 2) != 0;
            val = (res ? theintup : IntegerLast());
        }
        else
        {
            res = (thelims & 1) != 0;
            val = (res ? theintlow : IntegerFirst());
        }
        return res;
    }

    /// <summary>
    /// Sets a Real limit (included) to &lt;val&gt;, the upper limit if &lt;max&gt; is True, the lower limit if
    /// &lt;max&gt; is False
    /// </summary>
    public void SetRealLimit(bool max, double val)
    {
        if (thetype != MoniTool_ValueReal)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : SetRealLimit, not a Real");
        }

        if (max)
        {
            thelims |= 2;
            therealup = val;
        }
        else
        {
            thelims |= 1;
            therealow = val;
        }
    }

    /// <summary>
    /// Gives an Real Limit (upper if &lt;max&gt; True, lower if &lt;max&gt; False). Returns True if this limit
    /// is defined, False else (in that case, gives the natural limit for Real)
    /// </summary>
    public bool RealLimit(bool max, out double val)
    {
        bool res = false;
        if (max)
        {
            res = (thelims & 2) != 0;
            val = (res ? therealup : RealLast());
        }
        else
        {
            res = (thelims & 1) != 0;
            val = (res ? therealow : RealFirst());
        }
        return res;
    }

    /// <summary>
    /// Sets (Clears if &lt;def&gt; empty) a unit definition, as an equation of dimensions. TypedValue just
    /// records this definition, does not exploit it, to be done as required by user applications
    /// </summary>
    public void SetUnitDef(string def)
    {
        theunidef = "";
        theunidef += def;
    }

    /// <summary>
    /// Returns the recorded unit definition, empty if not set
    /// </summary>
    public string UnitDef()
    {
        return theunidef;
    }

    /// <summary>
    /// For an enumeration, precises the starting value (default 0) and the match condition : if True (D), the
    /// string value must match the definition, else it may take another value : in that case, the Integer Value
    /// will be Start - 1. (empty value remains allowed)
    /// </summary>
    public void StartEnum(int start = 0, bool match = true)
    {
        if (thetype != MoniTool_ValueEnum)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : StartEnum, Not an Enum");
        }

        thelims |= 4;
        if (!match)
        {
            thelims -= 4;
        }
        theintlow = start;
        theintup = start - 1;
    }

    /// <summary>
    /// Adds enumerative definitions. For more than 10, several calls
    /// </summary>
    public void AddEnum(string v1 = "", string v2 = "", string v3 = "", string v4 = "", string v5 = "", string v6 = "", string v7 = "", string v8 = "", string v9 = "", string v10 = "")
    {
        if (thetype != MoniTool_ValueEnum)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : AddEnum, Not an Enum");
        }
        if (theenums is null)
        {
            theenums = new NCollection_HArray1<string>(theintlow, theintlow + 10);
            theenums.Init("");
        }
        else if (theenums.Upper() < theintup + 10)
        {
            NCollection_HArray1<string> enums =
                new NCollection_HArray1<string>(theintlow, theintup + 10);
            enums.Init("");
            for (int i = theintlow; i <= theintup; i++)
            {
                enums.SetValue(i, theenums.Value(i));
            }
            theenums = enums;
        }

        if (v1.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v1));
            theeadds.Bind(v1, theintup);
        }
        if (v2.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v2));
            theeadds.Bind(v2, theintup);
        }
        if (v3.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v3));
            theeadds.Bind(v3, theintup);
        }
        if (v4.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v4));
            theeadds.Bind(v4, theintup);
        }
        if (v5.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v5));
            theeadds.Bind(v5, theintup);
        }
        if (v6.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v6));
            theeadds.Bind(v6, theintup);
        }
        if (v7.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v7));
            theeadds.Bind(v7, theintup);
        }
        if (v8.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v8));
            theeadds.Bind(v8, theintup);
        }
        if (v9.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v9));
            theeadds.Bind(v9, theintup);
        }
        if (v10.Length > 0)
        {
            theintup++;
            theenums.SetValue(theintup, (v10));
            theeadds.Bind(v10, theintup);
        }
    }

    /// <summary>
    /// Adds an enumeration definition, by its string and numeric values. If it is the first setting for this
    /// value, it is recorded as main value. Else, it is recognized as alternate string for this numeric value
    /// </summary>
    public void AddEnumValue(string val, int num)
    {
        if (thetype != MoniTool_ValueEnum)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : AddEnum, Not an Enum");
        }
        if (num < theintlow)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : AddEnum, out of range");
        }
        if (val.Length == 0)
        {
            return;
        }
        if (theenums is null)
        {
            theenums = new NCollection_HArray1<string>(theintlow, num + 1);
            theenums.Init("");
        }
        else if (theenums.Upper() < num)
        {
            NCollection_HArray1<string> enums =
                new NCollection_HArray1<string>(theintlow, num + 1);
            enums.Init("");
            for (int i = theintlow; i <= theintup; i++)
            {
                enums.SetValue(i, theenums.Value(i));
            }
            theenums = enums;
        }

        if (theintup < num)
        {
            theintup = num;
        }
        if (string.IsNullOrEmpty(theenums.Value(num)))
        {
            theenums.SetValue(num, (val));
        }
        //    We ALSO put in the dictionary
        //  else {
        theeadds.Bind(val, num);
        //  }
    }

    /// <summary>
    /// Gives the Enum definitions : start value, end value, match status. Returns True for an Enum, False else.
    /// </summary>
    public bool EnumDef(out int startcase, out int endcase, out bool match)
    {
        if (thetype != MoniTool_ValueEnum)
        {
            startcase = endcase = 0; // OCCT leaves them as they were
            match = false;
            return false;
        }
        startcase = theintlow;
        endcase = theintup;
        match = ((thelims & 4) != 0);
        return true;
    }

    /// <summary>
    /// Returns the value of an enumerative definition, from its rank Empty string if out of range or not an Enum
    /// </summary>
    public string EnumVal(int num)
    {
        if (thetype != MoniTool_ValueEnum)
        {
            return "";
        }
        if (num < theintlow || num > theintup)
        {
            return "";
        }
        return theenums.Value(num);
    }

    /// <summary>
    /// Returns the case number which corresponds to a string value Works with main and additional values Returns
    /// (StartEnum - 1) if not OK, -1 if not an Enum
    /// </summary>
    public int EnumCase(string val)
    {
        if (thetype != MoniTool_ValueEnum)
        {
            return (theintlow - 1);
        }
        int i; // svv Jan 10 2000 : porting on DEC
        for (i = theintlow; i <= theintup; i++)
        {
            if (theenums.Value(i) == val)
            {
                return i;
            }
        }
        //  cas additionnel ?
        if (!theeadds.IsEmpty())
        {
            if (theeadds.Find(val, out i))
            {
                return i;
            }
        }
        //  entier possible
        // gka S4054
        for (i = 0; i < val.Length; i++)
        {
            if (val[i] != ' ' && val[i] != '-' && (val[i] < '0' || val[i] > '9'))
            {
                return (theintlow - 1);
            }
        }
        return atoi(val);
    }

    /// <summary>
    /// Sets type of which an Object TypedValue must be kind of Error for a TypedValue not an Object (Entity)
    /// </summary>
    public void SetObjectType(Type? typ)
    {
        if (thetype != MoniTool_ValueIdent)
        {
            throw new Standard_ConstructionError("MoniTool_TypedValue : AddEnum, Not an Entity/Object");
        }
        theotyp = typ;
    }

    /// <summary>
    /// Returns the type of which an Object TypedValue must be kind of Default is Standard_Transient Null for a
    /// TypedValue not an Object
    /// </summary>
    public Type? ObjectType()
    {
        if (theotyp is not null)
        {
            return theotyp;
        }
        return typeof(Standard_Transient);
    }

    /// <summary>
    /// Sets a specific Interpret function
    /// </summary>
    public void SetInterpret(MoniTool_ValueInterpret func)
    {
        theinterp = func;
    }

    /// <summary>
    /// Tells if a TypedValue has an Interpret
    /// </summary>
    public virtual bool HasInterpret()
    {
        if (theinterp is not null)
        {
            return true;
        }
        if (thetype == MoniTool_ValueEnum)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Sets a specific Satisfies function : it is added to the already defined criteria It must match the form :
    /// satisfies (val : HAsciiString) returns Boolean
    /// </summary>
    public void SetSatisfies(MoniTool_ValueSatisfies func, string name)
    {
        thesatisn = "";
        thesatisf = func;
        if (thesatisf is not null)
        {
            thesatisn += name;
        }
    }

    /// <summary>
    /// Returns name of specific satisfy, empty string if none
    /// </summary>
    public string SatisfiesName()
    {
        return thesatisn;
    }

    /// <summary>
    /// Returns True if the value is set (not empty/not null object)
    /// </summary>
    public bool IsSetValue()
    {
        if (thetype == MoniTool_ValueIdent)
        {
            return (theoval is not null);
        }
        if (thehval.Length() > 0)
        {
            return true;
        }
        if (theoval is not null)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Returns the value, as a cstring. Empty if not set.
    /// </summary>
    public string CStringValue()
    {
        if (thehval is null)
        {
            return "";
        }
        return thehval.ToCString();
    }

    /// <summary>
    /// Returns the value, as a Handle (can then be shared) Null if not defined
    /// </summary>
    public TCollection_HAsciiString? HStringValue()
    {
        return thehval;
    }

    /// <summary>
    /// Interprets a value. &lt;native&gt; True : returns a native value &lt;native&gt; False : returns a coded
    /// value If the Interpret function is set, calls it Else, for an Enum, Native returns the Text, Coded
    /// returns the number STANDARD RETURNS : = hval means no specific interpretation Null means senseless Can
    /// also be redefined
    /// </summary>
    public virtual TCollection_HAsciiString? Interpret(TCollection_HAsciiString? hval, bool native)
    {
        TCollection_HAsciiString inter = hval;
        if (hval is null)
        {
            return hval;
        }
        if (theinterp is not null)
        {
            return theinterp(this, hval, native);
        }
        if (thetype == MoniTool_ValueEnum)
        {
            //  We accept both forms : Enum preferably, otherwise Integer
            int startcase, endcase;
            bool match;
            EnumDef(out startcase, out endcase, out match);
            int encas = EnumCase(hval.ToCString());
            if (encas < startcase)
            {
                return hval; // loupe
            }
            if (native)
            {
                inter = new TCollection_HAsciiString(EnumVal(encas));
            }
            else
            {
                inter = new TCollection_HAsciiString(encas);
            }
        }
        return inter;
    }

    /// <summary>
    /// Returns True if a value statifies the specification (remark : does not apply to Entity : see ObjectType,
    /// for this type, the string is just a comment)
    /// </summary>
    public virtual bool Satisfies(TCollection_HAsciiString? val)
    {
        if (val is null)
        {
            return false;
        }
        if (thesatisf is not null)
        {
            if (!thesatisf(val))
            {
                return false;
            }
        }
        if (val.Length() == 0)
        {
            return true;
        }
        switch (thetype)
        {
            case MoniTool_ValueInteger: {
                if (!val.IsIntegerValue())
                {
                    return false;
                }
                int ival, ilim;
                ival = atoi(val.ToCString());
                if (IntegerLimit(false, out ilim))
                {
                    if (ilim > ival)
                    {
                        return false;
                    }
                }
                if (IntegerLimit(true, out ilim))
                {
                    if (ilim < ival)
                    {
                        return false;
                    }
                }
                return true;
            }
            case MoniTool_ValueReal: {
                if (!val.IsRealValue())
                {
                    return false;
                }
                double rval, rlim;
                rval = val.RealValue();
                if (RealLimit(false, out rlim))
                {
                    if (rlim > rval)
                    {
                        return false;
                    }
                }
                if (RealLimit(true, out rlim))
                {
                    if (rlim < rval)
                    {
                        return false;
                    }
                }
                return true;
            }
            case MoniTool_ValueEnum: {
                //  We accept both forms : Enum preferably, otherwise Integer
                int startcase, endcase; // unused ival;
                bool match;
                EnumDef(out startcase, out endcase, out match);
                if (!match)
                {
                    return true;
                }
                if (EnumCase(val.ToCString()) >= startcase)
                {
                    return true;
                }
                //  Here, we accept an integer in the range
                ////      if (val.IsIntegerValue()) ival = atoi (val);

                // PTV 16.09.2000 The if is comment, cause this check is never been done (You can see the
                // logic)
                //      if (ival >= startcase && ival <= endcase) return true;
                return false;
            }
            case MoniTool_ValueText: {
                if (themaxlen > 0 && val.Length() > themaxlen)
                {
                    return false;
                }
                break;
            }
            default:
                break;
        }
        return true;
    }

    /// <summary>
    /// Clears the recorded Value : it is now unset
    /// </summary>
    public void ClearValue()
    {
        thehval = null; // OCCT: IsSetValue, SetCStringValue and RealValue dereference it afterwards
        theoval = null;
        theival = 0;
    }

    /// <summary>
    /// Changes the value. The new one must satisfy the specification Returns False (and did not set) if the new
    /// value does not satisfy the specification Can be redefined to be managed (in a subclass)
    /// </summary>
    public virtual bool SetCStringValue(string val)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(val);
        if (hval.IsSameString(thehval))
        {
            return true;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        if (thetype == MoniTool_ValueInteger)
        {
            thehval.Clear();
            theival = atoi(val);
            thehval.AssignCat(val);
        }
        else if (thetype == MoniTool_ValueEnum)
        {
            int ival = EnumCase(val);
            string cval = EnumVal(ival);
            if (string.IsNullOrEmpty(cval))
            {
                return false;
            }
            theival = ival;
            thehval.Clear();
            thehval.AssignCat(cval);
        }
        else
        {
            thehval.Clear();
            thehval.AssignCat(val);
            return true;
        }
        return true;
    }

    /// <summary>
    /// Forces a new Handle for the Value It can be empty, else (if Type is not free Text), it must satisfy the
    /// specification. Not only the value is changed, but also the way it is shared Remark : for Type=Object,
    /// this value is not controlled, it can be set as a comment Returns False (and did not set) if the new value
    /// does not satisfy the specification Can be redefined to be managed (in a subclass)
    /// </summary>
    public virtual bool SetHStringValue(TCollection_HAsciiString? hval)
    {
        if (hval is null)
        {
            return false;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        thehval = hval;
        if (thetype == MoniTool_ValueInteger)
        {
            theival = atoi(hval.ToCString());
        }
        else if (thetype == MoniTool_ValueEnum)
        {
            theival = EnumCase(hval.ToCString());
        }
        //  else return true;
        return true;
    }

    /// <summary>
    /// Returns the value as integer, i.e. : For type = Integer, the integer itself; 0 if not set For type =
    /// Enum, the designated rank (see Enum definition) StartEnum - 1 if not set or not in the definition Else,
    /// returns 0
    /// </summary>
    public int IntegerValue()
    {
        return theival;
    }

    /// <summary>
    /// Changes the value as an integer, only for Integer or Enum
    /// </summary>
    public virtual bool SetIntegerValue(int ival)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(ival);
        if (hval.IsSameString(thehval))
        {
            return true;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        thehval.Clear();
        if (thetype == MoniTool_ValueEnum)
        {
            thehval.AssignCat(EnumVal(ival));
        }
        else
        {
            thehval.AssignCat(hval);
        }
        theival = ival;
        return true;
    }

    /// <summary>
    /// Returns the value as real, for a Real type TypedValue Else, returns 0.
    /// </summary>
    public double RealValue()
    {
        if (thehval.Length() == 0)
        {
            return 0.0;
        }
        if (!thehval.IsRealValue())
        {
            return 0.0;
        }
        return thehval.RealValue();
    }

    /// <summary>
    /// Changes the value as a real, only for Real
    /// </summary>
    public virtual bool SetRealValue(double rval)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(rval);
        if (hval.IsSameString(thehval))
        {
            return true;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        thehval.Clear();
        thehval.AssignCat(hval);
        return true;
    }

    /// <summary>
    /// Returns the value as Transient Object, only for Object/Entity Remark that the "HString value" is IGNORED
    /// here Null if not set; remains to be casted
    /// </summary>
    public Standard_Transient? ObjectValue()
    {
        return theoval;
    }

    /// <summary>
    /// Same as ObjectValue, but avoids DownCast : the receiving variable is directly loaded. It is assumed that
    /// it complies with the definition of ObjectType ! Otherwise, big trouble
    /// </summary>
    public void GetObjectValue(out Standard_Transient? val)
    {
        val = theoval;
    }

    /// <summary>
    /// Changes the value as Transient Object, only for Object/Entity Returns False if DynamicType does not
    /// satisfy ObjectType Can be redefined to be managed (in a subclass)
    /// </summary>
    public virtual bool SetObjectValue(Standard_Transient? obj)
    {
        if (thetype != MoniTool_ValueIdent)
        {
            return false;
        }
        if (obj is null)
        {
            theoval = null;
            return true;
        }
        if (theotyp is not null)
        {
            if (!obj.IsKind(theotyp))
            {
                return false;
            }
        }
        theoval = obj;
        return true;
    }

    /// <summary>
    /// Returns the type name of the ObjectValue, or an empty string if not set
    /// </summary>
    public string ObjectTypeName()
    {
        if (theoval is null)
        {
            return "";
        }
        MoniTool_Element elm = (theoval as MoniTool_Element);
        if (elm is not null)
        {
            return elm.ValueTypeName();
        }
        return theoval.DynamicType().Name;
    }

    // OCCT.NET: AddLib (which adds a typed value to the process-wide library) is left out (decision 1181).

    /// <summary>
    /// Returns the TypedValue bound with a given Name Null Handle if none recorded Warning: it is the original,
    /// not duplicated
    /// </summary>
    public static MoniTool_TypedValue? Lib(string defin)
    {
        MoniTool_TypedValue val;
        if (libtv().Find(defin, out Standard_Transient aTVal))
        {
            val = (aTVal as MoniTool_TypedValue);
        }
        else
        {
            val = null;
        }
        return val;
    }

    /// <summary>
    /// Returns a COPY of the TypedValue bound with a given Name Null Handle if none recorded
    /// </summary>
    public static MoniTool_TypedValue? FromLib(string defin)
    {
        MoniTool_TypedValue val = MoniTool_TypedValue.Lib(defin);
        if (val is not null)
        {
            val = new MoniTool_TypedValue(val);
        }
        return val;
    }

    /// <summary>
    /// Returns the list of names of items of the Library of Types Library of TypedValue as Valued Parameters,
    /// accessed by parameter name for use by management of Static Parameters
    /// </summary>
    public static NCollection_HSequence<string> LibList()
    {
        NCollection_HSequence<string> list =
            new NCollection_HSequence<string>();
        if (libtv().IsEmpty())
        {
            return list;
        }
        foreach (KeyValuePair<string, Standard_Transient> it in libtv())
        {
            list.Append(it.Key);
        }
        return list;
    }

    // OCCT.NET: StaticValue and Stats (the process-wide dictionary of static values) are left out: static values are
    // an Interface_StaticSet's, passed explicitly (decision 1181).

    private string thename;
    private string thedef = "";
    private string thelabel = "";
    private MoniTool_ValueType thetype;
    private Type? theotyp;
    private int thelims;
    private int themaxlen;
    private int theintlow;
    private int theintup;
    private double therealow;
    private double therealup;
    private string theunidef = "";
    private NCollection_HArray1<string>? theenums;
    private NCollection_DataMap<string, int> theeadds = new();
    private MoniTool_ValueInterpret? theinterp;
    private MoniTool_ValueSatisfies? thesatisf;
    private string thesatisn = "";
    private int theival;
    private TCollection_HAsciiString? thehval;
    private Standard_Transient? theoval;
}
