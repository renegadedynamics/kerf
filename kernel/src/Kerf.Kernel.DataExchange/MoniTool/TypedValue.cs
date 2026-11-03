// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_TypedValue.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_TypedValue.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using System.Collections.Frozen;
using System.Collections.Immutable;
using static Kerf.Engine.Exchange.Framework.Monitoring.MonitorValueType;

namespace Kerf.Engine.Exchange.Framework.Monitoring;

/// <summary>
/// This class allows to dynamically manage .. typed values, i.e. values which have an alphanumeric
/// expression, but with controls. Such as "must be an Integer" or "Enumerative Text" etc Hence, a TypedValue
/// brings a specification (type + constraints if any) and a value. Its basic form is a string, it can be
/// specified as integer or real or enumerative string, then queried as such. Its string content, which is a
/// occ::handle&lt;HAsciiString&gt; can be shared by other data structures, hence gives a direct on line
/// access to its value.
/// </summary>
public class TypedValue : Transient
{
    // The library of typed values (OCCT's static thelibtv): the standard types, built once (decision 1181). OCCT's
    // AddLib, which adds to it, and the static dictionary of "static" values (OCCT's astats, Stats(), StaticValue)
    // are left out: the static values are a parameter set's (Interface_StaticSet), passed explicitly.
    // The library of typed values, built once and frozen (unit 10J), with its names in the order OCCT binds them
    private static readonly Lazy<(FrozenDictionary<string, TypedValue> Map, ImmutableArray<string> Names)> Thelibtv =
        new(MakeLib);

    private static (FrozenDictionary<string, TypedValue> Map, ImmutableArray<string> Names) MakeLib()
    {
        var lib = new Dictionary<string, TypedValue>();
        TypedValue tv;
        tv = new TypedValue("Integer", MonitorValueType.ValueInteger);
        lib["Integer"] = tv;
        tv = new TypedValue("Real", MonitorValueType.ValueReal);
        lib["Real"] = tv;
        tv = new TypedValue("Text", MonitorValueType.ValueText);
        lib["Text"] = tv;
        tv = new TypedValue("Transient", MonitorValueType.ValueIdent);
        lib["Transient"] = tv;
        tv = new TypedValue("Boolean", MonitorValueType.ValueEnum);
        tv.AddDef("enum 0"); //    = 0 False  ,  > 0 True
        tv.AddDef("eval False");
        tv.AddDef("eval True");
        lib["Boolean"] = tv;
        tv = new TypedValue("Logical", MonitorValueType.ValueEnum);
        tv.AddDef("enum -1"); //    < 0 False  ,  = 0 Unk  ,  > 0 True
        tv.AddDef("eval False");
        tv.AddDef("eval Unknown");
        tv.AddDef("eval True");
        lib["Logical"] = tv;
        return (lib.ToFrozenDictionary(StringComparer.Ordinal), [.. lib.Keys]);
    }

    /// <summary>
    /// Creates a TypedValue, with a name type gives the type of the parameter, default is free text Also
    /// available : Integer, Real, Enum, Entity (i.e. Object) More precise specifications, titles, can be given
    /// to the TypedValue once created init gives an initial value. If it is not given, the TypedValue begins as
    /// "not set", its value is empty
    /// </summary>
    public TypedValue(string name, MonitorValueType type = MonitorValueType.ValueText, string init = "")
    {
        _thename = name;
        _thetype = type;
        _thelims = 0;
        _themaxlen = 0;
        _theintlow = 0;
        _theintup = -1;
        _therealow = 0.0;
        _therealup = 0.0;
        _theinterp = null;
        _thesatisf = null;
        _theival = 0;
        _thehval = new TCollection_HAsciiString("");
        if (type != MonitorValueType.ValueInteger && type != MonitorValueType.ValueReal && type != MonitorValueType.ValueEnum
                && type != MonitorValueType.ValueText && type != MonitorValueType.ValueIdent)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : Type not supported");
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
    public TypedValue(TypedValue? other)
    {
        _thename = other.Name();
        _thedef = other.Definition();
        _thelabel = other.Label();
        _thetype = other.ValueType();
        _theotyp = other.ObjectType();
        _thelims = 0;
        _themaxlen = other.MaxLength();
        _theintlow = 0;
        _theintup = 0;
        _therealow = 0;
        _therealup = 0;
        _theunidef = other.UnitDef();
        _theival = other.IntegerValue();
        _thehval = other.HStringValue();
        _theoval = other.ObjectValue();
        other.Internals(out _theinterp, out _thesatisf, out string satisname, out Dictionary<string, int> eadds);
        _thesatisn += satisname;

        if (other.IntegerLimit(false, out _theintlow))
        {
            _thelims |= 1;
        }
        if (other.IntegerLimit(true, out _theintup))
        {
            _thelims |= 2;
        }
        if (other.RealLimit(false, out _therealow))
        {
            _thelims |= 1;
        }
        if (other.RealLimit(true, out _therealup))
        {
            _thelims |= 2;
        }

        int startcase, endcase;
        bool match;
        if (other.EnumDef(out startcase, out endcase, out match))
        {
            _theintlow = startcase;
            _theintup = endcase;
            if (match)
            {
                _thelims |= 4;
            }
            if (_theintup >= _theintlow)
            {
                _theenums = new NCollection_HArray1<string>(_theintlow, _theintup);
                _theenums.Init("");
            }
            for (startcase = _theintlow; startcase <= _theintup; startcase++)
            {
                _theenums.SetValue(startcase, other.EnumVal(startcase));
            }
        }
        //  dupliquer theeadds
        if (eadds.Count != 0)
        {
            foreach (KeyValuePair<string, int> itad in eadds)
            {
                _theeadds[itad.Key] = itad.Value;
            }
        }

        //  we duplicate the string
        if (_thehval is not null)
        {
            _thehval = new TCollection_HAsciiString(other.CStringValue());
        }
    }

    /// <summary>
    /// An exact copy of another typed value (OCCT.NET): every field as it is, the enforced definition included (where
    /// OCCT's copy constructor enforces the computed definition), the value in a new string. For
    /// <see cref="StaticSet"/>'s copies, which must behave as the original under later definitions.
    /// </summary>
    internal TypedValue(TypedValue other, bool exact)
    {
        _ = exact;
        _thename = other._thename;
        _thedef = other._thedef;
        _thelabel = other._thelabel;
        _thetype = other._thetype;
        _theotyp = other._theotyp;
        _thelims = other._thelims;
        _themaxlen = other._themaxlen;
        _theintlow = other._theintlow;
        _theintup = other._theintup;
        _therealow = other._therealow;
        _therealup = other._therealup;
        _theunidef = other._theunidef;
        _theenums = other._theenums is null ? null : new NCollection_HArray1<string>(other._theenums);
        _theeadds = new Dictionary<string, int>(other._theeadds,other._theeadds.Comparer);
        _theinterp = other._theinterp;
        _thesatisf = other._thesatisf;
        _thesatisn = other._thesatisn;
        _theival = other._theival;
        _thehval = other._thehval is null ? null : new TCollection_HAsciiString(other._thehval);
        _theoval = other._theoval;
    }

    /// <summary>
    /// Access to internal data which have no other access
    /// </summary>
    public void Internals(out MonitorValueInterpret interp, out MonitorValueSatisfies satisf, out string satisname, out Dictionary<string, int> enums)
    {
        interp = _theinterp;
        satisf = _thesatisf;
        satisname = _thesatisn;
        enums = new Dictionary<string, int>(_theeadds,_theeadds.Comparer); // a copy, as C++'s assignment
    }

    /// <summary>
    /// Returns the name
    /// </summary>
    public string Name()
    {
        return _thename;
    }

    /// <summary>
    /// Returns the type of the value
    /// </summary>
    public MonitorValueType ValueType()
    {
        return _thetype;
    }

    /// <summary>
    /// Returns the Definition By priority, the enforced one, else an automatic one, computed from the
    /// specification
    /// </summary>
    public string Definition()
    {
        if (_thedef.Length > 0)
        {
            return _thedef;
        }
        string def = "";
        string mess;
        switch (_thetype)
        {
            case MonitorValueType.ValueInteger: {
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
            case MonitorValueType.ValueReal: {
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
                if (_theunidef.Length > 0)
                {
                    def += "  Unit:";
                    def += UnitDef();
                }
            }
            break;
            case MonitorValueType.ValueEnum: {
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
                if (_theeadds.Count != 0)
                {
                    def += " , alpha: ";
                    // OCCT.NET: the map iterates in insertion order where OCCT's iterates in hash order (decision 1181)
                    foreach (KeyValuePair<string, int> listadd in _theeadds)
                    {
                        string name = listadd.Key;
                        string enva = name;
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
            case MonitorValueType.ValueIdent: {
                def += "Object(Entity)";
                if (_theotyp is not null)
                {
                    def += ":";
                    def += _theotyp.Name;
                }
            }
            break;
            case MonitorValueType.ValueText: {
                def += "Text";
                if (_themaxlen > 0)
                {
                    mess = Sprintf(" <= %d C.", _themaxlen);
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
        _thedef = "";
        _thedef += deftext;
    }

    /// <summary>
    /// Prints definition, specification, and actual status and value
    /// </summary>
    public virtual void Print(TextWriter s)
    {
        s.Write("--- Typed Value : ");
        s.Write(Name());
        if (_thelabel.Length > 0)
        {
            s.Write("  Label : ");
            s.Write(Label());
        }
        s.Write('\n');
        s.Write("--- Type : ");
        s.Write(Definition());
        s.Write('\n');
        s.Write("--- Value : ");

        PrintValue(s);
        s.Write('\n');

        if (_thesatisf is not null)
        {
            s.Write(" -- Specific Function for Satisfies : ");
            s.Write(_thesatisn);
            s.Write('\n');
        }
    }

    /// <summary>
    /// Prints only the Value
    /// </summary>
    public void PrintValue(TextWriter s)
    {
        if (IsSetValue())
        {
            if (_thetype == MonitorValueType.ValueIdent)
            {
                s.Write(" (type) ");
                s.Write(_theoval.DynamicTypeName());
            }
            if (_thehval is not null)
            {
                s.Write((_thetype == MonitorValueType.ValueIdent ? " : " : ""));
                s.Write(_thehval.ToCString());
            }

            if (HasInterpret())
            {
                s.Write("  (");
                TCollection_HAsciiString str = Interpret(_thehval, true);
                if (str is not null && str != _thehval)
                {
                    s.Write("Native:");
                    s.Write(str.ToCString());
                }
                str = Interpret(_thehval, false);
                if (str is not null && str != _thehval)
                {
                    s.Write("  Coded:");
                    s.Write(str.ToCString());
                }
                s.Write(")");
            }
        }
        else
        {
            s.Write("(not set)");
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
            SetIntegerLimit(false, Atoi(init.Substring(iblc)));
        }
        else if (CStringChar(init, 0) == 'i' && CStringChar(init, 2) == 'a')
        { // imax ival
            SetIntegerLimit(true, Atoi(init.Substring(iblc)));
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
            StartEnum(Atoi(init.Substring(iblc)), true);
        }
        else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'n')
        { // enum istart
            StartEnum(Atoi(init.Substring(iblc)), false);
        }
        else if (CStringChar(init, 0) == 'e' && CStringChar(init, 1) == 'v')
        { // eval text
            AddEnum(init.Substring(iblc));
        }
        else if (CStringChar(init, 0) == 't' && CStringChar(init, 1) == 'm')
        { // tmax length
            SetMaxLength(Atoi(init.Substring(iblc)));
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
        _thelabel = "";
        _thelabel += label;
    }

    /// <summary>
    /// Returns the label, if set; else returns an empty string
    /// </summary>
    public string Label()
    {
        return _thelabel;
    }

    /// <summary>
    /// Sets a maximum length for a text (active only for a free text)
    /// </summary>
    public void SetMaxLength(int max)
    {
        _themaxlen = max;
        if (max < 0)
        {
            _themaxlen = 0;
        }
    }

    /// <summary>
    /// Returns the maximum length, 0 if not set
    /// </summary>
    public int MaxLength()
    {
        return _themaxlen;
    }

    /// <summary>
    /// Sets an Integer limit (included) to &lt;val&gt;, the upper limit if &lt;max&gt; is True, the lower limit
    /// if &lt;max&gt; is False
    /// </summary>
    public void SetIntegerLimit(bool max, int val)
    {
        if (_thetype != MonitorValueType.ValueInteger)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : SetIntegerLimit, not an Integer");
        }

        if (max)
        {
            _thelims |= 2;
            _theintup = val;
        }
        else
        {
            _thelims |= 1;
            _theintlow = val;
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
            res = (_thelims & 2) != 0;
            val = (res ? _theintup : IntegerLast());
        }
        else
        {
            res = (_thelims & 1) != 0;
            val = (res ? _theintlow : IntegerFirst());
        }
        return res;
    }

    /// <summary>
    /// Sets a Real limit (included) to &lt;val&gt;, the upper limit if &lt;max&gt; is True, the lower limit if
    /// &lt;max&gt; is False
    /// </summary>
    public void SetRealLimit(bool max, double val)
    {
        if (_thetype != MonitorValueType.ValueReal)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : SetRealLimit, not a Real");
        }

        if (max)
        {
            _thelims |= 2;
            _therealup = val;
        }
        else
        {
            _thelims |= 1;
            _therealow = val;
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
            res = (_thelims & 2) != 0;
            val = (res ? _therealup : RealLast());
        }
        else
        {
            res = (_thelims & 1) != 0;
            val = (res ? _therealow : RealFirst());
        }
        return res;
    }

    /// <summary>
    /// Sets (Clears if &lt;def&gt; empty) a unit definition, as an equation of dimensions. TypedValue just
    /// records this definition, does not exploit it, to be done as required by user applications
    /// </summary>
    public void SetUnitDef(string def)
    {
        _theunidef = "";
        _theunidef += def;
    }

    /// <summary>
    /// Returns the recorded unit definition, empty if not set
    /// </summary>
    public string UnitDef()
    {
        return _theunidef;
    }

    /// <summary>
    /// For an enumeration, precises the starting value (default 0) and the match condition : if True (D), the
    /// string value must match the definition, else it may take another value : in that case, the Integer Value
    /// will be Start - 1. (empty value remains allowed)
    /// </summary>
    public void StartEnum(int start = 0, bool match = true)
    {
        if (_thetype != MonitorValueType.ValueEnum)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : StartEnum, Not an Enum");
        }

        _thelims |= 4;
        if (!match)
        {
            _thelims -= 4;
        }
        _theintlow = start;
        _theintup = start - 1;
    }

    /// <summary>
    /// Adds enumerative definitions. For more than 10, several calls
    /// </summary>
    public void AddEnum(string v1 = "", string v2 = "", string v3 = "", string v4 = "", string v5 = "", string v6 = "", string v7 = "", string v8 = "", string v9 = "", string v10 = "")
    {
        if (_thetype != MonitorValueType.ValueEnum)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : AddEnum, Not an Enum");
        }
        if (_theenums is null)
        {
            _theenums = new NCollection_HArray1<string>(_theintlow, _theintlow + 10);
            _theenums.Init("");
        }
        else if (_theenums.Upper() < _theintup + 10)
        {
            NCollection_HArray1<string> enums =
                new NCollection_HArray1<string>(_theintlow, _theintup + 10);
            enums.Init("");
            for (int i = _theintlow; i <= _theintup; i++)
            {
                enums.SetValue(i, _theenums.Value(i));
            }
            _theenums = enums;
        }

        if (v1.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v1));
            _theeadds[v1] = _theintup;
        }
        if (v2.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v2));
            _theeadds[v2] = _theintup;
        }
        if (v3.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v3));
            _theeadds[v3] = _theintup;
        }
        if (v4.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v4));
            _theeadds[v4] = _theintup;
        }
        if (v5.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v5));
            _theeadds[v5] = _theintup;
        }
        if (v6.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v6));
            _theeadds[v6] = _theintup;
        }
        if (v7.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v7));
            _theeadds[v7] = _theintup;
        }
        if (v8.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v8));
            _theeadds[v8] = _theintup;
        }
        if (v9.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v9));
            _theeadds[v9] = _theintup;
        }
        if (v10.Length > 0)
        {
            _theintup++;
            _theenums.SetValue(_theintup, (v10));
            _theeadds[v10] = _theintup;
        }
    }

    /// <summary>
    /// Adds an enumeration definition, by its string and numeric values. If it is the first setting for this
    /// value, it is recorded as main value. Else, it is recognized as alternate string for this numeric value
    /// </summary>
    public void AddEnumValue(string val, int num)
    {
        if (_thetype != MonitorValueType.ValueEnum)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : AddEnum, Not an Enum");
        }
        if (num < _theintlow)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : AddEnum, out of range");
        }
        if (val.Length == 0)
        {
            return;
        }
        if (_theenums is null)
        {
            _theenums = new NCollection_HArray1<string>(_theintlow, num + 1);
            _theenums.Init("");
        }
        else if (_theenums.Upper() < num)
        {
            NCollection_HArray1<string> enums =
                new NCollection_HArray1<string>(_theintlow, num + 1);
            enums.Init("");
            for (int i = _theintlow; i <= _theintup; i++)
            {
                enums.SetValue(i, _theenums.Value(i));
            }
            _theenums = enums;
        }

        if (_theintup < num)
        {
            _theintup = num;
        }
        if (string.IsNullOrEmpty(_theenums.Value(num)))
        {
            _theenums.SetValue(num, (val));
        }
        //    We ALSO put in the dictionary
        //  else {
        _theeadds[val] = num;
        //  }
    }

    /// <summary>
    /// Gives the Enum definitions : start value, end value, match status. Returns True for an Enum, False else.
    /// </summary>
    public bool EnumDef(out int startcase, out int endcase, out bool match)
    {
        if (_thetype != MonitorValueType.ValueEnum)
        {
            startcase = endcase = 0; // OCCT leaves them as they were
            match = false;
            return false;
        }
        startcase = _theintlow;
        endcase = _theintup;
        match = ((_thelims & 4) != 0);
        return true;
    }

    /// <summary>
    /// Returns the value of an enumerative definition, from its rank Empty string if out of range or not an Enum
    /// </summary>
    public string EnumVal(int num)
    {
        if (_thetype != MonitorValueType.ValueEnum)
        {
            return "";
        }
        if (num < _theintlow || num > _theintup)
        {
            return "";
        }
        return _theenums.Value(num);
    }

    /// <summary>
    /// Returns the case number which corresponds to a string value Works with main and additional values Returns
    /// (StartEnum - 1) if not OK, -1 if not an Enum
    /// </summary>
    public int EnumCase(string val)
    {
        if (_thetype != MonitorValueType.ValueEnum)
        {
            return (_theintlow - 1);
        }
        int i; // svv Jan 10 2000 : porting on DEC
        for (i = _theintlow; i <= _theintup; i++)
        {
            if (_theenums.Value(i) == val)
            {
                return i;
            }
        }
        //  cas additionnel ?
        if (_theeadds.Count != 0)
        {
            if (_theeadds.TryGetValue(val, out i))
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
                return (_theintlow - 1);
            }
        }
        return Atoi(val);
    }

    /// <summary>
    /// Sets type of which an Object TypedValue must be kind of Error for a TypedValue not an Object (Entity)
    /// </summary>
    public void SetObjectType(Type? typ)
    {
        if (_thetype != MonitorValueType.ValueIdent)
        {
            throw new ConstructionErrorException("MoniTool_TypedValue : AddEnum, Not an Entity/Object");
        }
        _theotyp = typ;
    }

    /// <summary>
    /// Returns the type of which an Object TypedValue must be kind of Default is Standard_Transient Null for a
    /// TypedValue not an Object
    /// </summary>
    public Type? ObjectType()
    {
        if (_theotyp is not null)
        {
            return _theotyp;
        }
        return typeof(Transient);
    }

    /// <summary>
    /// Sets a specific Interpret function
    /// </summary>
    public void SetInterpret(MonitorValueInterpret func)
    {
        _theinterp = func;
    }

    /// <summary>
    /// Tells if a TypedValue has an Interpret
    /// </summary>
    public virtual bool HasInterpret()
    {
        if (_theinterp is not null)
        {
            return true;
        }
        if (_thetype == MonitorValueType.ValueEnum)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Sets a specific Satisfies function : it is added to the already defined criteria It must match the form :
    /// satisfies (val : HAsciiString) returns Boolean
    /// </summary>
    public void SetSatisfies(MonitorValueSatisfies func, string name)
    {
        _thesatisn = "";
        _thesatisf = func;
        if (_thesatisf is not null)
        {
            _thesatisn += name;
        }
    }

    /// <summary>
    /// Returns name of specific satisfy, empty string if none
    /// </summary>
    public string SatisfiesName()
    {
        return _thesatisn;
    }

    /// <summary>
    /// Returns True if the value is set (not empty/not null object)
    /// </summary>
    public bool IsSetValue()
    {
        if (_thetype == MonitorValueType.ValueIdent)
        {
            return (_theoval is not null);
        }
        if (_thehval.Length() > 0)
        {
            return true;
        }
        if (_theoval is not null)
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
        if (_thehval is null)
        {
            return "";
        }
        return _thehval.ToCString();
    }

    /// <summary>
    /// Returns the value, as a Handle (can then be shared) Null if not defined
    /// </summary>
    public TCollection_HAsciiString? HStringValue()
    {
        return _thehval;
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
        if (_theinterp is not null)
        {
            return _theinterp(this, hval, native);
        }
        if (_thetype == MonitorValueType.ValueEnum)
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
        if (_thesatisf is not null)
        {
            if (!_thesatisf(val))
            {
                return false;
            }
        }
        if (val.Length() == 0)
        {
            return true;
        }
        switch (_thetype)
        {
            case MonitorValueType.ValueInteger: {
                if (!val.IsIntegerValue())
                {
                    return false;
                }
                int ival, ilim;
                ival = Atoi(val.ToCString());
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
            case MonitorValueType.ValueReal: {
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
            case MonitorValueType.ValueEnum: {
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
            case MonitorValueType.ValueText: {
                if (_themaxlen > 0 && val.Length() > _themaxlen)
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
        _thehval = null; // OCCT: IsSetValue, SetCStringValue and RealValue dereference it afterwards
        _theoval = null;
        _theival = 0;
    }

    /// <summary>
    /// Changes the value. The new one must satisfy the specification Returns False (and did not set) if the new
    /// value does not satisfy the specification Can be redefined to be managed (in a subclass)
    /// </summary>
    public virtual bool SetCStringValue(string val)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(val);
        if (hval.IsSameString(_thehval))
        {
            return true;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        if (_thetype == MonitorValueType.ValueInteger)
        {
            _thehval.Clear();
            _theival = Atoi(val);
            _thehval.AssignCat(val);
        }
        else if (_thetype == MonitorValueType.ValueEnum)
        {
            int ival = EnumCase(val);
            string cval = EnumVal(ival);
            if (string.IsNullOrEmpty(cval))
            {
                return false;
            }
            _theival = ival;
            _thehval.Clear();
            _thehval.AssignCat(cval);
        }
        else
        {
            _thehval.Clear();
            _thehval.AssignCat(val);
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
        _thehval = hval;
        if (_thetype == MonitorValueType.ValueInteger)
        {
            _theival = Atoi(hval.ToCString());
        }
        else if (_thetype == MonitorValueType.ValueEnum)
        {
            _theival = EnumCase(hval.ToCString());
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
        return _theival;
    }

    /// <summary>
    /// Changes the value as an integer, only for Integer or Enum
    /// </summary>
    public virtual bool SetIntegerValue(int ival)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(ival);
        if (hval.IsSameString(_thehval))
        {
            return true;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        _thehval.Clear();
        if (_thetype == MonitorValueType.ValueEnum)
        {
            _thehval.AssignCat(EnumVal(ival));
        }
        else
        {
            _thehval.AssignCat(hval);
        }
        _theival = ival;
        return true;
    }

    /// <summary>
    /// Returns the value as real, for a Real type TypedValue Else, returns 0.
    /// </summary>
    public double RealValue()
    {
        if (_thehval.Length() == 0)
        {
            return 0.0;
        }
        if (!_thehval.IsRealValue())
        {
            return 0.0;
        }
        return _thehval.RealValue();
    }

    /// <summary>
    /// Changes the value as a real, only for Real
    /// </summary>
    public virtual bool SetRealValue(double rval)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(rval);
        if (hval.IsSameString(_thehval))
        {
            return true;
        }
        if (!Satisfies(hval))
        {
            return false;
        }
        _thehval.Clear();
        _thehval.AssignCat(hval);
        return true;
    }

    /// <summary>
    /// Returns the value as Transient Object, only for Object/Entity Remark that the "HString value" is IGNORED
    /// here Null if not set; remains to be casted
    /// </summary>
    public Transient? ObjectValue()
    {
        return _theoval;
    }

    /// <summary>
    /// Same as ObjectValue, but avoids DownCast : the receiving variable is directly loaded. It is assumed that
    /// it complies with the definition of ObjectType ! Otherwise, big trouble
    /// </summary>
    public void GetObjectValue(out Transient? val)
    {
        val = _theoval;
    }

    /// <summary>
    /// Changes the value as Transient Object, only for Object/Entity Returns False if DynamicType does not
    /// satisfy ObjectType Can be redefined to be managed (in a subclass)
    /// </summary>
    public virtual bool SetObjectValue(Transient? obj)
    {
        if (_thetype != MonitorValueType.ValueIdent)
        {
            return false;
        }
        if (obj is null)
        {
            _theoval = null;
            return true;
        }
        if (_theotyp is not null)
        {
            if (!obj.IsKind(_theotyp))
            {
                return false;
            }
        }
        _theoval = obj;
        return true;
    }

    /// <summary>
    /// Returns the type name of the ObjectValue, or an empty string if not set
    /// </summary>
    public string ObjectTypeName()
    {
        if (_theoval is null)
        {
            return "";
        }
        Element elm = (_theoval as Element);
        if (elm is not null)
        {
            return elm.ValueTypeName();
        }
        return _theoval.DynamicTypeName();
    }

    // OCCT.NET: AddLib (which adds a typed value to the process-wide library) is left out (decision 1181).

    /// <summary>
    /// Returns the TypedValue bound with a given Name Null Handle if none recorded Warning: it is the original,
    /// not duplicated
    /// </summary>
    public static TypedValue? Lib(string defin)
    {
        return Thelibtv.Value.Map.GetValueOrDefault(defin);
    }

    /// <summary>
    /// Returns a COPY of the TypedValue bound with a given Name Null Handle if none recorded
    /// </summary>
    public static TypedValue? FromLib(string defin)
    {
        TypedValue val = TypedValue.Lib(defin);
        if (val is not null)
        {
            val = new TypedValue(val);
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
        foreach (string name in Thelibtv.Value.Names)
        {
            list.Append(name);
        }
        return list;
    }

    // OCCT.NET: StaticValue and Stats (the process-wide dictionary of static values) are left out: static values are
    // an Interface_StaticSet's, passed explicitly (decision 1181).

    private string _thename;
    private string _thedef = "";
    private string _thelabel = "";
    private MonitorValueType _thetype;
    private Type? _theotyp;
    private int _thelims;
    private int _themaxlen;
    private int _theintlow;
    private int _theintup;
    private double _therealow;
    private double _therealup;
    private string _theunidef = "";
    private NCollection_HArray1<string>? _theenums;
    private Dictionary<string, int> _theeadds = new();
    private MonitorValueInterpret? _theinterp;
    private MonitorValueSatisfies? _thesatisf;
    private string _thesatisn = "";
    private int _theival;
    private TCollection_HAsciiString? _thehval;
    private Transient? _theoval;
}
