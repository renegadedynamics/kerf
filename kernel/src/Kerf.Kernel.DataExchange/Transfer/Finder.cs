// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_Finder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_Finder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// A Finder allows to map any kind of object as a Key for a Map. This works by defining, for a Hash Code, that of the
/// real Key, not of the Finder which acts only as an intermediate. When a Map asks for the HashCode of a Finder, this
/// one returns the code it has determined at creation time.
/// </summary>
/// <remarks>
/// OCCT's <c>GetHashCode()</c> (a <c>size_t</c>) is .NET's <see cref="GetHashCode"/> here, overridden to return the
/// stored code; equality stays by reference, and <see cref="FindHasher"/> compares finders with
/// <see cref="Equates"/>, as OCCT's maps of finders do (decision 1200).
/// </remarks>
internal abstract class Finder : Transient
{
    /// <summary>Returns the HashCode which has been stored by SetHashCode.</summary>
    public override int GetHashCode()
    {
        return _thecode;
    }

    /// <summary>
    /// Specific test of equality : to be defined by each sub-class, must be False if Finders have not the same true
    /// Type, else their contents must be compared.
    /// </summary>
    public abstract bool Equates(Finder? other);

    /// <summary>Returns the Type of the Value. By default, returns the DynamicType of the finder, but can be redefined.</summary>
    public virtual Type? ValueType()
    {
        return DynamicType();
    }

    /// <summary>Returns the name of the Type of the Value. Default is name of ValueType, unless it is for a non-handled object.</summary>
    public virtual string ValueTypeName()
    {
        return "(finder)";
    }

    //  ####    ATTRIBUTES    ####
    // Integer -> IntVal, Real -> Geom2d_CartesianPoint, CString -> HAsciiString

    /// <summary>Adds an attribute with a given name (replaces the former one with the same name if already exists).</summary>
    public void SetAttribute(string name, Transient? val)
    {
        _theattrib[name] = val;
    }

    /// <summary>Removes an attribute. Returns True when done, False if this attribute did not exist.</summary>
    public bool RemoveAttribute(string name)
    {
        if (_theattrib.Count == 0)
        {
            return false;
        }
        return _theattrib.Remove(name);
    }

    /// <summary>
    /// Returns an attribute from its name, filtered by a type. If no attribute has this name, or if it is not kind of
    /// this type, <paramref name="val"/> is Null and returned value is False. Else, it is True.
    /// </summary>
    public bool GetAttribute(string name, Type? type, out Transient? val)
    {
        if (_theattrib.Count == 0)
        {
            val = null;
            return false;
        }
        if (!_theattrib.TryGetValue(name, out val))
        {
            val = null;
            return false;
        }
        if (type is null || !val.IsKind(type)) // Standard_Type::SubType is false for a null type
        {
            val = null;
            return false;
        }
        return true;
    }

    /// <summary>Returns an attribute from its name. Null Handle if not recorded (whatever Transient, Integer, Real ...).</summary>
    public Transient? Attribute(string name)
    {
        Transient? atr = null;
        if (_theattrib.Count == 0)
        {
            return atr;
        }
        if (!_theattrib.TryGetValue(name, out atr))
        {
            atr = null;
        }
        return atr;
    }

    /// <summary>
    /// Returns the type of an attribute : ParamInt , ParamReal , ParamText (String) , ParamIdent (any) or ParamVoid
    /// (not recorded).
    /// </summary>
    public ParamType AttributeType(string name)
    {
        Transient? atr = Attribute(name);
        if (atr is null)
        {
            return ParamType.ParamVoid;
        }
        if (atr.DynamicType() == typeof(IntVal))
        {
            return ParamType.ParamInteger;
        }
        if (atr.DynamicType() == typeof(CartesianPoint2))
        {
            return ParamType.ParamReal;
        }
        if (atr.DynamicType() == typeof(TCollection_HAsciiString))
        {
            return ParamType.ParamText;
        }
        return ParamType.ParamIdent;
    }

    /// <summary>Adds an integer value for an attribute.</summary>
    public void SetIntegerAttribute(string name, int val)
    {
        IntVal ival = new IntVal();
        ival.CValue() = val;
        SetAttribute(name, ival);
    }

    /// <summary>
    /// Returns an attribute from its name, as integer. If no attribute has this name, or not an integer,
    /// <paramref name="val"/> is 0 and returned value is False. Else, it is True.
    /// </summary>
    public bool GetIntegerAttribute(string name, out int val)
    {
        IntVal? ival = Attribute(name) as IntVal;
        if (ival is null)
        {
            val = 0;
            return false;
        }
        val = ival.Value();
        return true;
    }

    /// <summary>Returns an integer attribute from its name. 0 if not recorded.</summary>
    public int IntegerAttribute(string name)
    {
        IntVal? ival = Attribute(name) as IntVal;
        if (ival is null)
        {
            return 0;
        }
        return ival.Value();
    }

    /// <summary>Adds a real value for an attribute.</summary>
    public void SetRealAttribute(string name, double val)
    {
        CartesianPoint2 rval = new CartesianPoint2(val, 0);
        SetAttribute(name, rval);
    }

    /// <summary>
    /// Returns an attribute from its name, as real. If no attribute has this name, or not a real,
    /// <paramref name="val"/> is 0.0 and returned value is False. Else, it is True.
    /// </summary>
    public bool GetRealAttribute(string name, out double val)
    {
        CartesianPoint2? rval = Attribute(name) as CartesianPoint2;
        if (rval is null)
        {
            val = 0.0;
            return false;
        }
        val = rval.X();
        return true;
    }

    /// <summary>Returns a real attribute from its name. 0.0 if not recorded.</summary>
    public double RealAttribute(string name)
    {
        CartesianPoint2? rval = Attribute(name) as CartesianPoint2;
        if (rval is null)
        {
            return 0;
        }
        return rval.X();
    }

    /// <summary>Adds a String value for an attribute.</summary>
    public void SetStringAttribute(string name, string val)
    {
        TCollection_HAsciiString hval = new TCollection_HAsciiString(val);
        SetAttribute(name, hval);
    }

    /// <summary>
    /// Returns an attribute from its name, as String. If no attribute has this name, or not a String,
    /// <paramref name="val"/> is "" and returned value is False. Else, it is True.
    /// </summary>
    public bool GetStringAttribute(string name, out string val)
    {
        TCollection_HAsciiString? hval = Attribute(name) as TCollection_HAsciiString;
        if (hval is null)
        {
            val = "";
            return false;
        }
        val = hval.ToCString();
        return true;
    }

    /// <summary>Returns a String attribute from its name. "" if not recorded.</summary>
    public string StringAttribute(string name)
    {
        TCollection_HAsciiString? hval = Attribute(name) as TCollection_HAsciiString;
        if (hval is null)
        {
            return "";
        }
        return hval.ToCString();
    }

    /// <summary>Returns the exhaustive list of attributes (the finder's own map, which callers may edit).</summary>
    public Dictionary<string, Transient?> AttrList()
    {
        return _theattrib;
    }

    /// <summary>
    /// Gets the list of attributes from <paramref name="other"/>, as such, i.e. not copied : attributes are shared,
    /// any attribute edited, added, or removed in <paramref name="other"/> is also in this finder and vice versa. The
    /// former list of attributes is dropped.
    /// </summary>
    /// <remarks>
    /// OCCT assigns the map (<c>theattrib = other->AttrList()</c>), a copy in C++, so the lists are not shared there
    /// in fact (occt-issues.md): the port copies too.
    /// </remarks>
    public void SameAttributes(Finder? other)
    {
        if (other is not null)
        {
            _theattrib = new Dictionary<string, Transient?>(other.AttrList()); // OCCT: a copy, despite the documentation
        }
    }

    /// <summary>
    /// Gets the list of attributes from <paramref name="other"/>, by copying it. By default, considers all the
    /// attributes from <paramref name="other"/>. If <paramref name="fromname"/> is given, considers only the attributes
    /// with name beginning by <paramref name="fromname"/>. For each attribute, if <paramref name="copied"/> is True
    /// (D), its value is also copied if it is a basic type (Integer,Real,String), else it remains shared. These new
    /// attributes are added to the existing ones, in case of same name, they replace the existing ones.
    /// </summary>
    public void GetAttributes(Finder? other, string fromname = "", bool copied = true)
    {
        if (other is null)
        {
            return;
        }
        Dictionary<string, Transient?> list = other.AttrList();
        if (list.Count == 0)
        {
            return;
        }
        foreach (KeyValuePair<string, Transient?> iter in list)
        {
            string name = iter.Key;
            if (!name.StartsWith(fromname, StringComparison.Ordinal))
            {
                continue;
            }
            Transient? atr = iter.Value;
            Transient? newatr = atr;
            //    Copy ? according type
            if (copied)
            {
                IntVal? ival = atr as IntVal;
                if (ival is not null)
                {
                    int intval = ival.Value();
                    ival = new IntVal();
                    ival.CValue() = intval;
                    newatr = ival;
                }
                CartesianPoint2? rval = atr as CartesianPoint2;
                if (rval is not null)
                {
                    double realval = rval.X();
                    rval = new CartesianPoint2(realval, 0);
                    newatr = rval;
                }
                TCollection_HAsciiString? hval = atr as TCollection_HAsciiString;
                if (hval is not null)
                {
                    TCollection_HAsciiString strval = new TCollection_HAsciiString(hval.ToCString());
                    newatr = strval;
                }
            }
            _theattrib[name] = newatr;
        }
    }

    /// <summary>Stores the HashCode which corresponds to the Value given to create the Mapper.</summary>
    protected void SetHashCode(int code)
    {
        _thecode = code;
    }

    private int _thecode;
    private Dictionary<string, Transient?> _theattrib = new(StringComparer.Ordinal);
}
